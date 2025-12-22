#!/usr/bin/env python3
import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# ============================================================
# Corolla EPS 取向優化版（以你提供的延遲/jerk buffer 架構為基底）
# 目標：低速扭力更願意推 + 反應更快 + 仍保持柔順
# ============================================================

# --- 基本 PID 參數（Kp 走插值表） --------------------------------
KI = 0.26
KD = 0.0

# Kp 插值：速度越低 Kp 越高（更快、更能推），高速回到穩定基準
# 單位：m/s
INTERP_SPEEDS = [1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 30.0]
# Corolla 取向：低速比你原表更積極，但不追求極端（避免停車場抖）
KP_INTERP =     [320, 170,  95,  42,  16,   7.5,  4.5,  2.6,  1.0]

# --- 測量速度率濾波（影響 D-like 阻尼感與噪聲） -------------------
# 原本 1.2Hz 偏“滑但慢”，Corolla 取向稍微提高反應但仍保守
LP_FILTER_CUTOFF_HZ = 1.8

# --- 延遲 buffer --------------------------------------------------
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# --- jerk setpoint gain：低速略超前，提升跟手 ----------------------
# setpoint = expected + (lat_delay * jerk_gain) * jerk
JERK_GAIN_V_BP = [0.0, 3.0, 10.0, 30.0]          # m/s
JERK_GAIN_VALS = [1.25, 1.18, 1.05, 1.00]        # 低速更超前

# --- 摩擦補償縮放：低速更願意推，高速收斂避免黏 -------------------
FRIC_SCALE_V_BP = [0.0, 5.0, 15.0, 30.0]         # m/s
FRIC_SCALE_VALS = [1.20, 1.00, 0.80, 0.65]

# --- 轉角死區縮放：低速縮小以加快起手，但 0~極低速保留抖抑制 -----
# deadzone_eff = base_deadzone_deg * scale(v)
DEADZONE_SCALE_V_BP = [0.0, 1.5, 6.0, 15.0, 30.0]
DEADZONE_SCALE_VALS = [1.15, 0.92, 0.85, 0.95, 1.00]

# --- 積分凍結：低速要更能修正，可略降低凍結速度 -------------------
FREEZE_V_THRESH_MPS = 3.5   # 原本 5.0；越低越能在低速用 I 修正，但太低可能抖

# --- 扭力斜率限制（柔順度關鍵）：低速允許快，高速更平滑 ----------
# max_delta = steer_max * rate_ratio(v) * dt
TORQUE_SLEW_RATE_V_BP    = [0.0, 5.0, 15.0, 30.0]  # m/s
TORQUE_SLEW_RATE_RATIOS  = [7.0, 5.0, 3.0, 2.0]    # 倍/秒（越大越快，但更可能有顆粒感）

# --- 安全保護：lat_delay 下限，避免除 0 / 太小導致 jerk 爆掉 -------
LAT_DELAY_MIN = 0.05   # seconds
# --- 回正（Return-to-Center, RTC）輔助 ---------------------------------
# 目的：
# - 彎道結束後，因 latency buffer 仍殘留 setpoint，會讓扭力「留太久」→ 回正慢
# - 透過「只在接近直行」時的微量回正扭力 + 放寬回正 slew，讓方向盤更快回到 0
#
# 注意：
# - 這不會讓 openpilot 做「90 度市區轉彎」(那是規劃/路徑問題)，只改善「彎已結束但還不回正」
# - 數值保守起手，避免低速抖動/蛇行

RTC_ENABLE = True

# 只在「接近直行」才加 RTC（用 future_desired_lateral_accel 判斷）
RTC_A_LAT_THR = 0.35          # m/s^2，越大越早介入（太大可能干擾彎中）
RTC_ANGLE_THR_DEG = 6.0       # deg，方向盤偏角超過才補回正

# RTC 扭力 = -Kp * steeringAngleDeg（再乘速度縮放）
RTC_KP = 0.0022               # 每 1 度給多少 normalized torque（0.0022→50 度約 0.11）
RTC_MAX_TORQUE = 0.18         # normalized，上限避免猛拉

# 速度越低越需要回正推力（低速摩擦大）
RTC_V_BP = [0.0, 5.0, 15.0, 30.0]     # m/s
RTC_V_SCALE = [1.25, 1.10, 0.90, 0.75]

# 回正時放寬 slew（讓扭力可以更快變化）
RTC_SLEW_MULT = 1.6



class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()

    # PID 在「側向加速度」空間運作，Kp 依速插值
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt)

    self.update_limits()

    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len,
                                          maxlen=self.lat_accel_request_buffer_len)

    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)

    # 狀態：斜率限制與啟閉重置
    self._last_output_torque = 0.0
    self._was_active = False

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    # PID 輸出限制用「可達側向加速度」來設，但若 torque tune 的 latAccelFactor 偏大，
    # 直接用 inverse 會讓可達 a_lat 變得很小 → 方向盤角度被『軟性封頂』。
    # 這裡加一個保底，讓控制器至少能推到一定的 a_lat（最後仍會被扭力飽和/安全限幅夾住）。
    a_lat_pos = float(self.lateral_accel_from_torque(self.steer_max, self.torque_params))
    a_lat_neg = float(self.lateral_accel_from_torque(-self.steer_max, self.torque_params))
    a_lat_max = max(abs(a_lat_pos), abs(a_lat_neg), 3.5)  # m/s^2 保底（可依體感微調）
    self.pid.set_limits(a_lat_max, -a_lat_max)
  def _apply_torque_slew_limit(self, output_torque, v_ego, unwind_boost=False):
    rate = float(np.interp(v_ego, TORQUE_SLEW_RATE_V_BP, TORQUE_SLEW_RATE_RATIOS))  # 1/sec (ratio of steer_max per sec)
    if unwind_boost and RTC_ENABLE:
      rate *= float(RTC_SLEW_MULT)
    max_delta = rate * float(self.steer_max) * float(self.dt)
    delta = float(np.clip(output_torque - self._last_output_torque, -max_delta, max_delta))
    out = self._last_output_torque + delta
    self._last_output_torque = out
    return out
  def _reset_states(self):
    self.pid.reset()
    self.lat_accel_request_buffer.clear()
    self.lat_accel_request_buffer.extend([0.] * self.lat_accel_request_buffer_len)
    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self._last_output_torque = 0.0

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    if not active:
      # 只在 active->inactive transition 時 reset，避免每帧歸零造成重新啟用時跳變
      if self._was_active:
        self._reset_states()
      self._was_active = False

      output_torque = 0.0
      pid_log.active = False
      return -output_torque, 0.0, pid_log

    # 第一次啟用：初始化避免 kick
    if not self._was_active:
      self._reset_states()
      self._was_active = True

    # --- lat_delay 安全處理 ---
    lat_delay_safe = float(np.clip(lat_delay, LAT_DELAY_MIN, LAT_ACCEL_REQUEST_BUFFER_SECONDS))

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
                                           CS.vEgo, params.roll)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY

    # --- 自適應死區（加快起手但防極低速抖） ---
    dz_scale = float(np.interp(CS.vEgo, DEADZONE_SCALE_V_BP, DEADZONE_SCALE_VALS))
    steering_angle_deadzone_deg_eff = float(self.steering_angle_deadzone_deg) * dz_scale
    curvature_deadzone = abs(VM.calc_curvature(math.radians(steering_angle_deadzone_deg_eff), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    # --- 延遲 buffer 取 expected ---
    delay_frames = int(np.clip(lat_delay_safe / self.dt, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = float(self.lat_accel_request_buffer[-delay_frames])

    # --- 期望 a_lat（未來）與 jerk（用 safe delay）---
    future_desired_lateral_accel = float(desired_curvature * CS.vEgo ** 2)
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation
    desired_lateral_jerk = float((future_desired_lateral_accel - expected_lateral_accel) / lat_delay_safe)

    # --- measurement 與 measurement_rate ---
    measurement = float(measured_curvature * CS.vEgo ** 2)
    measurement_rate = float(self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt))
    self.previous_measurement = measurement

    # --- jerk gain：低速更超前 ---
    jerk_gain = float(np.interp(CS.vEgo, JERK_GAIN_V_BP, JERK_GAIN_VALS))
    setpoint = float((lat_delay_safe * jerk_gain) * desired_lateral_jerk + expected_lateral_accel)
    error = float(setpoint - measurement)

    pid_log.error = error

    # --- FF（lat accel 空間）---
    ff = float(gravity_adjusted_future_lateral_accel)
    # latAccelOffset 校正 roll bias（裝置/車身對齊誤差）
    ff -= float(self.torque_params.latAccelOffset)

    # --- 摩擦補償：低速更願意推、高速收斂 ---
    fric_scale = float(np.interp(CS.vEgo, FRIC_SCALE_V_BP, FRIC_SCALE_VALS))
    ff += fric_scale * get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    # --- integrator freeze：低速允許更早開 I（但仍避開極低速）---
    freeze_integrator = bool(steer_limited_by_safety or CS.steeringPressed or CS.vEgo < FREEZE_V_THRESH_MPS)

    # PID update（在 lat accel 空間），error_rate 用 -measurement_rate 提供阻尼
    output_lataccel = self.pid.update(error,
                                      -measurement_rate,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

    # 轉成 torque 命令（處理非線性 torque response）
    output_torque = float(self.torque_from_lateral_accel(output_lataccel, self.torque_params))

    # --- 回正（RTC）輔助：只在接近直行時，提供一點『回 0』的扭力 ---
    end_of_turn = (RTC_ENABLE and (abs(future_desired_lateral_accel) < RTC_A_LAT_THR) and
                   (abs(CS.steeringAngleDeg) > RTC_ANGLE_THR_DEG))
    if end_of_turn:
      rtc_scale = float(np.interp(CS.vEgo, RTC_V_BP, RTC_V_SCALE))
      rtc_torque = float(np.clip(-RTC_KP * float(CS.steeringAngleDeg), -RTC_MAX_TORQUE, RTC_MAX_TORQUE))
      output_torque += rtc_scale * rtc_torque

    # 最終仍以 steer_max 夾住（避免任何額外項超出）
    output_torque = float(np.clip(output_torque, -self.steer_max, self.steer_max))

    # --- 扭力斜率限制（柔順度核心）：回正時放寬以加快回到 0 ---
    output_torque = float(self._apply_torque_slew_limit(output_torque, CS.vEgo, unwind_boost=end_of_turn))

    # log
    pid_log.active = True
    pid_log.p = float(self.pid.p)
    pid_log.i = float(self.pid.i)
    pid_log.d = float(self.pid.d)
    pid_log.f = float(self.pid.f)
    pid_log.output = float(-output_torque)  # torque sign convention
    pid_log.actualLateralAccel = float(measurement)
    pid_log.desiredLateralAccel = float(setpoint)
    pid_log.desiredLateralJerk = float(desired_lateral_jerk)
    pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3,
                                                    CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log