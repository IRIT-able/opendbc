from opendbc.can import CANPacker
from opendbc.car import Bus
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.lateral import apply_steer_angle_limits_vm
from opendbc.car.tesla.preap.carcontroller import PreAPLongController, init_preap_can
from opendbc.car.tesla.preap.constants import get_hands_on_disengage_level
from opendbc.car.tesla.preap.nap_conf import nap_conf
from opendbc.car.tesla.preap.stock_cc_spoofer import StockCCSpoofer
from opendbc.car.tesla.values import CANBUS, CarControllerParams
from opendbc.car.vehicle_model import VehicleModel

try:
  from cereal import messaging
except ImportError:
  messaging = None

class PreAPCarController(CarControllerBase):
  def __init__(self, dbc_names, CP, CP_SP):
    super().__init__(dbc_names, CP, CP_SP)
    self.apply_angle_last = 0
    safety_param = 0
    if getattr(self.CP, "safetyConfigs", None):
      safety_param = int(getattr(self.CP.safetyConfigs[0], "safetyParam", 0) or 0)
    self._hands_on_disengage_level = get_hands_on_disengage_level(safety_param)

    CANBUS.powertrain = CANBUS.party
    CANBUS.autopilot_powertrain = CANBUS.autopilot_party
    self.packers = {
      CANBUS.party: CANPacker(dbc_names[Bus.party]),
      CANBUS.powertrain: CANPacker(dbc_names[Bus.pt]),
    }
    self.preap_long = PreAPLongController()
    self.stock_cc = StockCCSpoofer()
    self.tesla_can = init_preap_can(dbc_names, self.packers)
    self.radar_vin_idx = 0
    
    self.sm = messaging.SubMaster(['wideRoadCameraState']) if messaging else None
    self.high_beam_state = False
    self.auto_brights_enabled = False

    from opendbc.car.tesla.interface import CarInterface
    # Same CarSpecs as NAP's HW3 VehicleModel; PREAP is the wired candidate.
    self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_PREAP"))

  def update(self, CC, CC_SP, CS, now_nanos):
    del CC_SP
    actuators = CC.actuators
    can_sends = []

    if self.sm is not None:
      self.sm.update(0)
      
    # MADS drives CC.latActive on sunnypilot (controlsd_ext.get_lat_active).
    # Do not consult CS.cruiseEnabled for steer TX.
    lat_active = CC.latActive and CS.hands_on_level < self._hands_on_disengage_level

    if self.frame % 2 == 0:
      self.apply_angle_last = apply_steer_angle_limits_vm(
        actuators.steeringAngleDeg, self.apply_angle_last, CS.out.vEgoRaw, CS.out.steeringAngleDeg,
        lat_active, CarControllerParams, self.VM)
      cntr = (self.frame // 2) % 16
      can_sends.append(self.tesla_can.create_steering_control(cntr, self.apply_angle_last, lat_active))
      can_sends.append(self.tesla_can.create_epas_control(cntr, 1))

    # Reset pccEvent each tick so it expresses one-frame edge events. Without
    # this, the previous frame's value sticks (preap_long resets it, but only
    # runs in pedal mode), and the teslaCC{Engaged,Disengaged} alert
    # re-triggers indefinitely instead of fading after its 0.8s duration.
    CS.pccEvent = None

    # Pedal-mode longitudinal control. Runs only when op-long is on
    # (i.e. Comma Pedal present). May write CS.preap_cc_cancel_needed when
    # pedal mode wants to drop a running stock CC — consumed by stock_cc below.
    if self.CP.openpilotLongitudinalControl:
      can_sends.extend(self.preap_long.update(CC, CS, self.frame, self.tesla_can, CANBUS.party, now_nanos))

    # Stock-CC stalk spoofs (CANCEL / SET_ACCEL). Independent of op-long —
    # the engagement FSM publishes its intent through CarState flags and the
    # spoofer is the only TX path for 0x45 STW_ACTN_RQ frames.
    can_sends.extend(self.stock_cc.update(CS, self.frame, self.tesla_can, CANBUS.party))
    if self.stock_cc.pcc_event:
      CS.pccEvent = self.stock_cc.pcc_event

    # Tinkla 0.6.6 donor contract: stream VIN/position/EPAS on 0x560
    # when radar is on. Empty VIN is 17 spaces (this-car passthrough);
    # position and EPAS still apply. Panda stays silent until all three
    # fragments arrive, so 10 Hz keeps that pause around 300 ms.
    if nap_conf.radar_enabled and self.frame % 10 == 0:
      can_sends.append(self.tesla_can.create_radar_vin_msg(
        self.radar_vin_idx, nap_conf.radar_donor_vin, True,
        nap_conf.radar_position, nap_conf.radar_epas_type,
      ))
      self.radar_vin_idx = (self.radar_vin_idx + 1) % 3

    # Turn-signal drive: keep the indicator flashing during the lane-change
    # arming window and maneuver. controlsd sets CC.leftBlinker/rightBlinker
    # whenever laneChangeState != off, and clears them when it returns to off
    # (so the blinker stops automatically when the maneuver completes).
    # turn: 0=none, 1=left, 2=right. Pre-AP has no AP ECU, so openpilot is the
    # sole source of DAS_bodyControls.
    
    # Auto Brights logic
    stalk = getattr(CS.out, "napHighBeamStalk", 0)
    
    self.auto_brights_enabled = False
    self.high_beam_state = False

    if not hasattr(self, "monotonic_offset"):
      self.monotonic_offset = None

    if stalk == 1 and nap_conf.auto_brights:
      # Pushed forward: Armed mode!
      self.auto_brights_enabled = True
      
      stw_msg = getattr(CS, "msg_stw_actn_req", None)
      if stw_msg is not None:
        jam_msg = dict(stw_msg)
        jam_msg["HiBmLvr_Stat"] = 0
        real_counter = int(jam_msg.get("MC_STW_ACTN_RQ", 0))
        
        # We want to permanently stay exactly +3 ahead of the Gateway.
        # If we just add +3 to real_counter, USB jitter causes stalls and the BCM resets.
        # If we use a purely free-running clock, Linux drift causes us to cross the Gateway and strobe.
        # Solution: Ultra-Low-Pass Monotonic Counter stream!
        target_counter = real_counter + 3.0
        
        if self.monotonic_offset is None:
          # Initialize offset so that (frame/10) + offset == target_counter
          self.monotonic_offset = target_counter - (self.frame / 10.0)
          
        # Calculate what our smooth counter is right now
        current_smooth = (self.frame / 10.0) + self.monotonic_offset
        
        # Calculate phase error between our smooth counter and the physical Gateway (+3)
        diff = target_counter - current_smooth
        # Handle 16-counter wrap-around math
        if diff > 8: diff -= 16
        elif diff < -8: diff += 16
        
        # SLOWLY pull our offset towards the Gateway to correct for long-term Linux clock drift
        # A tiny factor of 0.01 completely absorbs all short-term USB jitter stalls!
        self.monotonic_offset += diff * 0.01
        
        # Re-calculate our perfectly stable counter
        current_smooth = (self.frame / 10.0) + self.monotonic_offset
        preempt_counter = int(current_smooth) % 16
        
        # Send 1 message per frame continuously. 
        # Because the stream never stalls, the BCM never times out.
        # Because we are +3 ahead, the BCM permanently drops the Gateway as a duplicate.
        can_sends.append(self.tesla_can.create_action_request(
          button_to_press=jam_msg.get("SpdCtrlLvr_Stat", 0),
          bus=CANBUS.party,
          counter=preempt_counter,
          msg_stw=jam_msg
        ))
    elif stalk == 1 or stalk == 2:
      self.high_beam_state = True
    else:
      self.high_beam_state = False
      
    if self.auto_brights_enabled and self.sm is not None:
      exposure = self.sm['wideRoadCameraState'].exposureValPercent
      # High exposure means the camera is compensating for darkness
      is_dark = exposure > 50.0
      no_lead = not CC.hudControl.leadVisible
      moving_fast = True # CS.out.vEgo > 10.0 # 22 mph (TEMPORARY BYPASS)
      
      if is_dark and no_lead and moving_fast:
        self.high_beam_state = True
      elif CS.out.vEgo < 5.0 or not is_dark or not no_lead:
        self.high_beam_state = False
        
    if self.frame % 10 == 0:
      turn = int(CC.rightBlinker) * 2 + int(CC.leftBlinker)
      cntr = (self.frame // 10) % 16
      
      # If stalk is 2, and we are not forcing it, or we are forcing it
      # Or if auto brights is disabled, we just send False or True.
      # Wait, if we send None, create_body_controls_message will send 0.
      send_hb = self.high_beam_state
      # If stalk == 0, we could send None to let car decide, but car doesn't have auto brights.
      # We just send send_hb
      
      can_sends.append(self.tesla_can.create_body_controls_message(turn, 0, send_hb, CANBUS.party, cntr))

    new_actuators = actuators.as_builder()
    new_actuators.steeringAngleDeg = self.apply_angle_last

    self.frame += 1
    return new_actuators, can_sends
