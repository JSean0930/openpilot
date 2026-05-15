import math
import numpy as np
from opendbc.car import Bus, make_tester_present_msg, rate_limit, structs, ACCELERATION_DUE_TO_GRAVITY, DT_CTRL
from opendbc.car.lateral import apply_meas_steer_torque_limits, apply_std_steer_angle_limits, common_fault_avoidance
from opendbc.car.can_definitions import CanData
from opendbc.car.carlog import carlog
from opendbc.car.common.filter_simple import FirstOrderFilter, HighPassFilter
from opendbc.car.common.pid import PIDController
from opendbc.car.secoc import add_mac, build_sync_mac
from opendbc.car.interfaces import CarControllerBase
from opendbc.car.toyota import toyotacan
from opendbc.car.toyota.values import CAR, NO_STOP_TIMER_CAR, TSS2_CAR, \
                                        CarControllerParams, ToyotaFlags, \
                                        UNSUPPORTED_DSU_CAR
from opendbc.can import CANPacker

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# 🌟 核心優化 1：放寬底層變化率限制 (Slew Rate Limit)
# 既然 Planner 已控管平順度，底層應作為「直通通道」，從 4.0 提高到 7.0 以減少加減速訊號延遲
ACCEL_WINDUP_LIMIT = 7.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -7.0 * DT_CTRL * 3  # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3  # m/s^2 / frame

MAX_PITCH_COMPENSATION = 1.5  # m/s^2

# LKA limits
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut
MAX_USER_TORQUE = 500

from opendbc.car.common.conversions import Conversions as CV
LOCK_SPEED = 20 * CV.KPH_TO_MS

LOCK_UNLOCK_CAN_ID = 0x750
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'

from cereal import car
PARK = car.CarState.GearShifter.park
DRIVE = car.CarState.GearShifter.drive

def get_long_tune(CP, params):
  if CP.carFingerprint in TSS2_CAR:
    # 🌟 核心優化 2：針對 TSS2 低速反應進行 PID 重調
    # 提高低速時的 kiV (積分增益)，用來抵銷 Toyota PCM 原廠 0.5s 的起步油門反應延遲
    kiBP = [0., 2.5, 5.]
    kiV = [1.5, 0.8, 0.25]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(0.0, (kiBP, kiV), k_f=1.0,
                       pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
                       rate=1 / (DT_CTRL * 3))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.params = CarControllerParams(self.CP)
    self.last_torque = 0
    self.last_angle = 0
    self.alert_active = False
    self.last_standstill = False
    self.standstill_req = False
    self.permit_braking = True
    self.steer_rate_counter = 0
    self.distance_button = 0

    # *** start long control state ***
    self.long_pid = get_long_tune(self.CP, self.params)
    self.aego = FirstOrderFilter(0.0, 0.25, DT_CTRL * 3)
    self.pitch = FirstOrderFilter(0, 0.5, DT_CTRL)
    self.pitch_hp = HighPassFilter(0.0, 0.25, 1.5, DT_CTRL)

    self.accel = 0
    self.prev_accel = 0
    # *** end long control state ***

    self.packer = CANPacker(dbc_names[Bus.pt])

    self.secoc_lka_message_counter = 0
    self.secoc_lta_message_counter = 0
    self.secoc_acc_message_counter = 0
    self.secoc_prev_reset_counter = 0

    self.doors_locked = False

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
      self.pitch_hp.update(CC.orientationNED[1])

    can_sends = []

    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_acc_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key, int(CS.secoc_synchronization['TRIP_CNT']), int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    new_torque = int(round(actuators.torque * self.params.STEER_MAX))
    apply_torque = apply_meas_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorqueEps, self.params)

    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(abs(CS.out.steeringRateDeg) >= MAX_STEER_RATE, lat_active,
                                                                      self.steer_rate_counter, MAX_STEER_RATE_FRAMES)

    if not lat_active:
      apply_torque = 0

    if self.CP.steerControlType == SteerControlType.angle:
      apply_torque = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
        self.last_angle = apply_std_steer_angle_limits(apply_angle, self.last_angle, CS.out.vEgoRaw,
                                                       CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
                                                       CC.latActive, self.params.ANGLE_LIMITS)

    self.last_torque = apply_torque

    steer_command = toyotacan.create_steer_command(self.packer, apply_torque, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    if self.frame % 2 == 0 and self.CP.carFingerprint in TSS2_CAR:
      lta_active = lat_active and self.CP.steerControlType == SteerControlType.angle
      full_torque_condition = (abs(CS.out.steeringTorqueEps) < self.params.STEER_MAX and
                               abs(CS.out.steeringTorque) < self.params.MAX_LTA_DRIVER_TORQUE_ALLOWANCE)

      torque_wind_down = 100 if lta_active and full_torque_condition else 0
      can_sends.append(toyotacan.create_lta_steer_command(self.packer, self.CP.steerControlType, self.last_angle,
                                                          lta_active, self.frame // 2, torque_wind_down))

      if self.CP.flags & ToyotaFlags.SECOC.value:
        lta_steer_2 = toyotacan.create_lta_steer_command_2(self.packer, self.frame // 2)
        lta_steer_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lta_message_counter,
                              lta_steer_2)
        self.secoc_lta_message_counter += 1
        can_sends.append(lta_steer_2)

    if self.CP.carFingerprint not in NO_STOP_TIMER_CAR:
      if CS.out.standstill and not self.last_standstill:
        self.standstill_req = True
      if CS.pcm_acc_status != 8:
        self.standstill_req = False
    else:
      should_resume = actuators.accel > 0
      if should_resume:
        self.standstill_req = False
      if not should_resume and CS.out.cruiseState.standstill:
        self.standstill_req = True

    if (self.CP.flags & ToyotaFlags.TSS1_SNG.value) and CS.out.standstill and not self.last_standstill:
      self.standstill_req = False

    self.last_standstill = CS.out.standstill

    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, ACCEL_WINDUP_LIMIT)
        self.prev_accel = pcm_accel_cmd

        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)

        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))
          error_future = pcm_accel_cmd - a_ego_future

          if not stopping:
            pitch_compensation = float(np.clip(math.sin(self.pitch_hp.x) * ACCELERATION_DUE_TO_GRAVITY,
                                               -MAX_PITCH_COMPENSATION, MAX_PITCH_COMPENSATION))
            pcm_accel_cmd += pitch_compensation

          pcm_accel_cmd = self.long_pid.update(error_future,
                                               speed=CS.out.vEgo,
                                               feedforward=pcm_accel_cmd,
                                               freeze_integrator=actuators.longControlState != LongCtrlState.pid)
        else:
          self.long_pid.reset()

        # 🌟 核心優化 3：提早解鎖煞車油壓 (Permit Braking)
        # 只要目標加速度 >= 0.15，就強迫 PCM 徹底鬆開煞車準備，讓起步訊號 100% 傳遞到輪胎
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min <= 0.05 or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min >= 0.15:
          self.permit_braking = False

        if not self.CP.carFingerprint in TSS2_CAR:
          pcm_accel_cmd = actuators.accel
          
        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        main_accel_cmd = 0. if self.CP.flags & ToyotaFlags.SECOC.value else pcm_accel_cmd
        can_sends.append(toyotacan.create_accel_command(self.packer, main_accel_cmd, pcm_cancel_cmd, self.permit_braking, self.standstill_req, lead,
                                                        CS.acc_type, fcw_alert, self.distance_button))
        if self.CP.flags & ToyotaFlags.SECOC.value:
          acc_cmd_2 = toyotacan.create_accel_command_2(self.packer, pcm_accel_cmd)
          acc_cmd_2 = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_acc_message_counter,
                              acc_cmd_2)
          self.secoc_acc_message_counter += 1
          can_sends.append(acc_cmd_2)

        self.accel = pcm_accel_cmd

    else:
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead, CS.acc_type, False, self.distance_button))

    if self.CP.carFingerprint != CAR.TOYOTA_PRIUS_V:
      send_ui = False
      if ((fcw_alert or steer_alert) and not self.alert_active) or \
         (not (fcw_alert or steer_alert) and self.alert_active):
        send_ui = True
        self.alert_active = not self.alert_active
      elif pcm_cancel_cmd:
        send_ui = True

      if self.frame % 20 == 0 or send_ui:
        can_sends.append(toyotacan.create_ui_command(self.packer, steer_alert, pcm_cancel_cmd, hud_control.leftLaneVisible,
                                                     hud_control.rightLaneVisible, hud_control.leftLaneDepart,
                                                     hud_control.rightLaneDepart, lat_active, CS.lkas_hud))

      if (self.frame % 100 == 0 or send_ui) and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value and not self.CP.flags & ToyotaFlags.RADAR_FILTER.value:
        can_sends.append(toyotacan.create_fcw_command(self.packer, fcw_alert))

    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value and not self.CP.flags & ToyotaFlags.RADAR_FILTER.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    if self.CP.flags & ToyotaFlags.LOCK_CTRL.value:
      if not self.doors_locked and CS.out.gearShifter == DRIVE and CS.out.vEgo >= LOCK_SPEED:
        can_sends.append(CanData(LOCK_UNLOCK_CAN_ID, LOCK_CMD, 0))
        self.doors_locked = True
      elif self.doors_locked and CS.out.gearShifter == PARK:
        can_sends.append(CanData(LOCK_UNLOCK_CAN_ID, UNLOCK_CMD, 0))
        self.doors_locked = False

    self.frame += 1
    return new_actuators, can_sends

