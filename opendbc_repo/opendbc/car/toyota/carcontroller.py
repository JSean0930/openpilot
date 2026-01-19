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
from opendbc.car.toyota.values import CAR, NO_STOP_TIMER_CAR, TSS2_CAR, CarControllerParams, ToyotaFlags, UNSUPPORTED_DSU_CAR
from opendbc.can import CANPacker

from opendbc.car.common.conversions import Conversions as CV
from cereal import car


# ==============================================================================
# 可調參數集中區（TSS2 扭力轉向顆粒感改善 + 起步更快）
# 調整建議（先照建議值跑，再依體感微調）：
# - 橫向顆粒感（扭力像一格一格）：
#   1) TORQUE_SMOOTH_TC ↑ => 更滑但反應慢；↓ => 更跟手但可能顆粒感回來
#      建議：0.10 ~ 0.16 s
#   2) STEER_RATE_FILTER_TC ↑ => 門檻抖動更少；太大會讓保護反應變慢
#      建議：0.06 ~ 0.10 s
#   3) STEER_RATE_EXIT 比例越低 => 解除保護更晚（更保守但更不抖）
#      建議：0.85 ~ 0.92 * MAX_STEER_RATE
#
# - 縱向起步不猶豫：
#   1) ACCEL_WINDUP_LIMIT_LAUNCH ↑ => 起步/低速加速度爬升更快（更衝）
#      建議：5.0~7.0 * DT_CTRL * 3（原本是 4.0 * DT_CTRL * 3）
#   2) LAUNCH_BOOST_ACCEL ↑ => 起步更乾脆；太大可能有「一口氣」的感覺
#      建議：0.20 ~ 0.35 m/s^2
#   3) LAUNCH_BOOST_TIME ↑ => boost 撐更久；太久可能讓低速太積極
#      建議：0.45 ~ 0.75 s
#   4) PB_RELEASE_TH ↓ => permit_braking 更早放開（更敢走）；太低可能早放煞車權
#      建議：0.20 ~ 0.24（PB_HOLD_TH 建議比它低 0.03~0.05）
# ==============================================================================

# --- 橫向：扭力平滑（降低顆粒感） ---
TORQUE_SMOOTH_ENABLE = True
TORQUE_SMOOTH_TC = 0.20  # s 0.12

# --- 橫向：方向盤角速度門檻防抖（避免 apply_steer_req 反覆切換） ---
STEER_RATE_FILTER_ENABLE = True
STEER_RATE_FILTER_TC = 0.08  # s
# hysteresis：進入/解除門檻（ENTER=原門檻；EXIT=較低門檻避免抖動）
# EXIT 建議用 0.90 左右（0.85~0.92）
STEER_RATE_EXIT_RATIO = 0.90

# --- 縱向：起步更快 ---
LAUNCH_BOOST_ENABLE = True
LAUNCH_V_MAX = 3.0          # m/s（低於此速才啟用起步優化）
LAUNCH_BOOST_TIME = 0.60    # s
LAUNCH_BOOST_ACCEL = 0.25   # m/s^2
ACCEL_WINDUP_LIMIT_LAUNCH = 6.0 * DT_CTRL * 3  # m/s^2/frame（低速上升更快）

# permit_braking 解除門檻（更果斷動起來）
PB_RELEASE_TH = 0.22  # > 此值 => permit_braking=False（更早放開煞車權）
PB_HOLD_TH = 0.18     # < 此值 => permit_braking=True

# ==============================================================================
# 原始常數/參數（保留）
# ==============================================================================

Ecu = structs.CarParams.Ecu
LongCtrlState = structs.CarControl.Actuators.LongControlState
SteerControlType = structs.CarParams.SteerControlType
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# The up limit allows the brakes/gas to unwind quickly leaving a stop,
# the down limit roughly matches the rate of ACCEL_NET, reducing PCM compensation windup
ACCEL_WINDUP_LIMIT = 4.0 * DT_CTRL * 3      # m/s^2 / frame
ACCEL_WINDDOWN_LIMIT = -4.0 * DT_CTRL * 3   # m/s^2 / frame
ACCEL_PID_UNWIND = 0.03 * DT_CTRL * 3       # m/s^2 / frame

MAX_PITCH_COMPENSATION = 1.5  # m/s^2

# LKA limits
# EPS faults if you apply torque while the steering rate is above 100 deg/s for too long
MAX_STEER_RATE = 100  # deg/s
MAX_STEER_RATE_FRAMES = 18  # tx control frames needed before torque can be cut

# EPS allows user torque above threshold for 50 frames before permanently faulting
MAX_USER_TORQUE = 500

# Lock / unlock door commands
LOCK_SPEED = 20 * CV.KPH_TO_MS

LOCK_UNLOCK_CAN_ID = 0x750
UNLOCK_CMD = b'\x40\x05\x30\x11\x00\x40\x00\x00'
LOCK_CMD = b'\x40\x05\x30\x11\x00\x80\x00\x00'

PARK = car.CarState.GearShifter.park
DRIVE = car.CarState.GearShifter.drive


def get_long_tune(CP, params):
  # 縱向 PID：主要用 I 控制（P=0）
  if CP.carFingerprint in TSS2_CAR:
    kiBP = [2., 5.]
    kiV = [0.5, 0.25]
  else:
    kiBP = [0., 5., 35.]
    kiV = [3.6, 2.4, 1.5]

  return PIDController(
    0.0, (kiBP, kiV), k_f=1.0,
    pos_limit=params.ACCEL_MAX, neg_limit=params.ACCEL_MIN,
    rate=1 / (DT_CTRL * 3)
  )


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

    # ==========================================================================
    # 橫向顆粒感改善：扭力輸入平滑 + steeringRate 防抖（門檻 hysteresis）
    # ==========================================================================
    self.torque_des_f = FirstOrderFilter(0.0, TORQUE_SMOOTH_TC, DT_CTRL)
    self.steer_rate_f = FirstOrderFilter(0.0, STEER_RATE_FILTER_TC, DT_CTRL)
    self.high_steer_rate = False

    # ==========================================================================
    # 縱向起步更快：起步 boost 計時（從 standstill -> moving 觸發）
    # ==========================================================================
    self.launch_start_t = None

  def update(self, CC, CS, now_nanos):
    actuators = CC.actuators
    stopping = actuators.longControlState == LongCtrlState.stopping
    hud_control = CC.hudControl
    pcm_cancel_cmd = CC.cruiseControl.cancel

    # lat_active：橫向允許輸出（同時限制駕駛扭力過大時停用，避免 EPS 風險）
    lat_active = CC.latActive and abs(CS.out.steeringTorque) < MAX_USER_TORQUE

    # 供起步 boost 使用的時間（秒）
    t_now = now_nanos * 1e-9

    if len(CC.orientationNED) == 3:
      self.pitch.update(CC.orientationNED[1])
      self.pitch_hp.update(CC.orientationNED[1])

    can_sends = []

    # *** handle secoc reset counter increase ***
    if self.CP.flags & ToyotaFlags.SECOC.value:
      if CS.secoc_synchronization['RESET_CNT'] != self.secoc_prev_reset_counter:
        self.secoc_lka_message_counter = 0
        self.secoc_lta_message_counter = 0
        self.secoc_acc_message_counter = 0
        self.secoc_prev_reset_counter = CS.secoc_synchronization['RESET_CNT']

        expected_mac = build_sync_mac(self.secoc_key,
                                      int(CS.secoc_synchronization['TRIP_CNT']),
                                      int(CS.secoc_synchronization['RESET_CNT']))
        if int(CS.secoc_synchronization['AUTHENTICATOR']) != expected_mac:
          carlog.error("SecOC synchronization MAC mismatch, wrong key?")

    # ==========================================================================
    # *** steer torque（扭力轉向 / LKA）***
    # - 顆粒感改善：先對欲輸出扭力做平滑，再進原本 Toyota 的扭力限幅/保護
    # ==========================================================================
    desired_torque = float(actuators.torque) * self.params.STEER_MAX
    if TORQUE_SMOOTH_ENABLE:
      # 先濾波再 round 成 int，可明顯降低「一格一格」感
      self.torque_des_f.update(desired_torque)
      desired_torque = float(self.torque_des_f.x)

    new_torque = int(round(desired_torque))
    apply_torque = apply_meas_steer_torque_limits(new_torque, self.last_torque, CS.out.steeringTorqueEps, self.params)

    # >100 deg/s steering fault prevention（加入濾波 + hysteresis 防抖）
    steer_rate = abs(CS.out.steeringRateDeg)
    if STEER_RATE_FILTER_ENABLE:
      self.steer_rate_f.update(steer_rate)
      steer_rate = float(self.steer_rate_f.x)

    steer_rate_enter = MAX_STEER_RATE
    steer_rate_exit = MAX_STEER_RATE * float(STEER_RATE_EXIT_RATIO)

    # hysteresis：避免在門檻附近反覆切換 apply_steer_req，造成顆粒/頓挫
    if not self.high_steer_rate:
      self.high_steer_rate = steer_rate >= steer_rate_enter
    else:
      self.high_steer_rate = steer_rate >= steer_rate_exit

    self.steer_rate_counter, apply_steer_req = common_fault_avoidance(
      self.high_steer_rate, lat_active, self.steer_rate_counter, MAX_STEER_RATE_FRAMES
    )

    if not lat_active:
      apply_torque = 0

    # *** steer angle（角度轉向 / LTA）***（保留原邏輯，TSS2 扭力車通常不走這段）
    if self.CP.steerControlType == SteerControlType.angle:
      apply_torque = 0
      apply_steer_req = False
      if self.frame % 2 == 0:
        apply_angle = actuators.steeringAngleDeg + CS.out.steeringAngleOffsetDeg
        self.last_angle = apply_std_steer_angle_limits(
          apply_angle, self.last_angle, CS.out.vEgoRaw,
          CS.out.steeringAngleDeg + CS.out.steeringAngleOffsetDeg,
          CC.latActive, self.params.ANGLE_LIMITS
        )

    self.last_torque = apply_torque

    # LKA command
    steer_command = toyotacan.create_steer_command(self.packer, apply_torque, apply_steer_req)
    if self.CP.flags & ToyotaFlags.SECOC.value:
      steer_command = add_mac(self.secoc_key,
                              int(CS.secoc_synchronization['TRIP_CNT']),
                              int(CS.secoc_synchronization['RESET_CNT']),
                              self.secoc_lka_message_counter,
                              steer_command)
      self.secoc_lka_message_counter += 1
    can_sends.append(steer_command)

    # LTA command (TSS2)
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

    # ==========================================================================
    # *** gas and brake（縱向）***
    # ==========================================================================
    # standstill 維持請求（Toyota 特化）
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

    # 起步 boost：從 standstill -> moving 的瞬間開始計時
    if self.last_standstill and not CS.out.standstill:
      self.launch_start_t = t_now
    # 若又回到 standstill，清掉起步計時
    if CS.out.standstill:
      self.launch_start_t = None

    self.last_standstill = CS.out.standstill

    # HUD / lead 狀態
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    lead = hud_control.leadVisible or CS.out.vEgo < 12.

    if self.CP.openpilotLongitudinalControl:
      if self.frame % 3 == 0:
        # distance button：把 PCM 車距 bars 調到 hud_control.leadDistanceBars
        if self.frame % 6 == 0 and self.CP.openpilotLongitudinalControl:
          desired_distance = 4 - hud_control.leadDistanceBars
          if CS.out.cruiseState.enabled and CS.pcm_follow_distance != desired_distance:
            self.distance_button = not self.distance_button
          else:
            self.distance_button = 0

        # 低速起步：放寬 windup，上升更快、更不猶豫
        pcm_accel_cmd = actuators.accel
        if CC.longActive:
          up_lim = ACCEL_WINDUP_LIMIT
          if LAUNCH_BOOST_ENABLE and CS.out.vEgo < LAUNCH_V_MAX:
            up_lim = ACCEL_WINDUP_LIMIT_LAUNCH
          pcm_accel_cmd = rate_limit(pcm_accel_cmd, self.prev_accel, ACCEL_WINDDOWN_LIMIT, up_lim)
        self.prev_accel = pcm_accel_cmd

        # 下坡補償（只取 downhill，避免上坡誤判 permit_braking）
        accel_due_to_pitch = math.sin(min(self.pitch.x, 0.0)) * ACCELERATION_DUE_TO_GRAVITY
        net_acceleration_request = pcm_accel_cmd + accel_due_to_pitch

        # GVC blending（非 SecOC）
        if not self.CP.flags & ToyotaFlags.SECOC.value:
          a_ego_blended = float(np.interp(CS.out.vEgo, [1.0, 2.0], [CS.gvc, CS.out.aEgo]))
        else:
          a_ego_blended = CS.out.aEgo

        # jerk + 未來 a_ego 預估（降低 overshoot）
        prev_aego = self.aego.x
        self.aego.update(a_ego_blended)
        j_ego = (self.aego.x - prev_aego) / (DT_CTRL * 3)
        future_t = float(np.interp(CS.out.vEgo, [2., 5.], [0.25, 0.5]))
        a_ego_future = a_ego_blended + j_ego * future_t

        if CC.longActive:
          # 積分慢慢回退，避免暫時誤差殘留
          self.long_pid.i -= ACCEL_PID_UNWIND * float(np.sign(self.long_pid.i))
          error_future = pcm_accel_cmd - a_ego_future

          if not stopping:
            pitch_compensation = float(np.clip(
              math.sin(self.pitch_hp.x) * ACCELERATION_DUE_TO_GRAVITY,
              -MAX_PITCH_COMPENSATION, MAX_PITCH_COMPENSATION
            ))
            pcm_accel_cmd += pitch_compensation

          pcm_accel_cmd = self.long_pid.update(
            error_future,
            speed=CS.out.vEgo,
            feedforward=pcm_accel_cmd,
            freeze_integrator=actuators.longControlState != LongCtrlState.pid
          )
        else:
          self.long_pid.reset()

        # 起步 boost：起步初期額外給一點正加速度（會隨時間淡出）
        if (LAUNCH_BOOST_ENABLE and CC.longActive and self.launch_start_t is not None and
            CS.out.vEgo < LAUNCH_V_MAX and pcm_accel_cmd > 0.0):
          dt_launch = t_now - self.launch_start_t
          if 0.0 <= dt_launch <= LAUNCH_BOOST_TIME:
            w = 1.0 - (dt_launch / LAUNCH_BOOST_TIME)
            pcm_accel_cmd += LAUNCH_BOOST_ACCEL * w

        # permit_braking：更果斷解除（起步更乾脆）
        net_acceleration_request_min = min(actuators.accel + accel_due_to_pitch, net_acceleration_request)
        if net_acceleration_request_min < PB_HOLD_TH or stopping or not CC.longActive:
          self.permit_braking = True
        elif net_acceleration_request_min > PB_RELEASE_TH:
          self.permit_braking = False

        # 非 TSS2：不做 delay compensation（保留原行為）
        if not self.CP.carFingerprint in TSS2_CAR:
          pcm_accel_cmd = actuators.accel

        pcm_accel_cmd = float(np.clip(pcm_accel_cmd, self.params.ACCEL_MIN, self.params.ACCEL_MAX))

        main_accel_cmd = 0. if self.CP.flags & ToyotaFlags.SECOC.value else pcm_accel_cmd
        can_sends.append(toyotacan.create_accel_command(self.packer, main_accel_cmd, pcm_cancel_cmd, self.permit_braking,
                                                        self.standstill_req, lead, CS.acc_type, fcw_alert, self.distance_button))
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
      # lat-only 仍可 cancel 原車系統
      if pcm_cancel_cmd:
        if self.CP.carFingerprint in UNSUPPORTED_DSU_CAR:
          can_sends.append(toyotacan.create_acc_cancel_command(self.packer))
        else:
          can_sends.append(toyotacan.create_accel_command(self.packer, 0, pcm_cancel_cmd, True, False, lead,
                                                          CS.acc_type, False, self.distance_button))

    # ==========================================================================
    # *** HUD UI ***
    # ==========================================================================
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

    # keep radar disabled
    if self.frame % 20 == 0 and self.CP.flags & ToyotaFlags.DISABLE_RADAR.value and not self.CP.flags & ToyotaFlags.RADAR_FILTER.value:
      can_sends.append(make_tester_present_msg(0x750, 0, 0xF))

    # 回填實際輸出（讓上層顯示/記錄一致）
    new_actuators = actuators.as_builder()
    new_actuators.torque = apply_torque / self.params.STEER_MAX
    new_actuators.torqueOutputCan = apply_torque
    new_actuators.steeringAngleDeg = self.last_angle
    new_actuators.accel = self.accel

    # 門鎖控制（選配）
    if self.CP.flags & ToyotaFlags.LOCK_CTRL.value:
      if not self.doors_locked and CS.out.gearShifter == DRIVE and CS.out.vEgo >= LOCK_SPEED:
        can_sends.append(CanData(LOCK_UNLOCK_CAN_ID, LOCK_CMD, 0))
        self.doors_locked = True
      elif self.doors_locked and CS.out.gearShifter == PARK:
        can_sends.append(CanData(LOCK_UNLOCK_CAN_ID, UNLOCK_CMD, 0))
        self.doors_locked = False

    self.frame += 1
    return new_actuators, can_sends
