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
# Corolla EPS 取向優化版（延遲/jerk buffer 架構）
# 新增：Return-to-center 模式，加速方向盤回正
# ============================================================

KI = 0.26
KD = 0.0

INTERP_SPEEDS = [1.0, 1.5, 2.0, 3.0, 5.0, 7.5, 10.0, 15.0, 30.0]
KP_INTERP     = [320, 170,  95,  42,  16,   7.5,  4.5,  2.6,  1.0]

# 稍微提高 cutoff，回正時的 measurement_rate 阻尼更即時（但不要太高避免噪聲）
LP_FILTER_CUTOFF_HZ = 2.2

LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 2

JERK_GAIN_V_BP = [0.0, 3.0, 10.0, 30.0]          # m/s
JERK_GAIN_VALS = [1.25, 1.18, 1.05, 1.00]

FRIC_SCALE_V_BP = [0.0, 5.0, 15.0, 30.0]         # m/s
FRIC_SCALE_VALS = [1.20, 1.00, 0.80, 0.65]

DEADZONE_SCALE_V_BP = [0.0, 1.5, 6.0, 15.0, 30.0]
DEADZONE_SCALE_VALS = [1.15, 0.92, 0.85, 0.95, 1.00]

FREEZE_V_THRESH_MPS = 3.5

TORQUE_SLEW_RATE_V_BP    = [0.0, 5.0, 15.0, 30.0]  # m/s
TORQUE_SLEW_RATE_RATIOS  = [7.0, 5.0, 3.0, 2.0]    # 倍/秒

LAT_DELAY_MIN = 0.05   # seconds

# =========================
# 回正加速參數（核心）
# =========================
# 以 lateral accel 判斷「我其實想直行」：小於此值視為回正/直行需求
RTC_DES_LATACCEL_THR = 0.12   # m/s^2  (越大越常進入回正模式)
# 但實際 lateral accel 還大於此值 → 真的在彎/回正中
RTC_MEAS_LATACCEL_THR = 0.18  # m/s^2

# 回正模式下：把 expected_lateral_accel 快速衰減，避免黏 buffer
# 每個控制迴圈 expected *= RTC_EXPECTED_DECAY（越小衰減越快）
RTC_EXPECTED_DECAY = 0.88

# 回正模式下：放寬扭力 slew（回正更快）
RTC_SLEW_MULT_V_BP = [0.0, 5.0, 15.0, 30.0]
RTC_SLEW_MULT_VALS = [1.60, 1.45, 1.25, 1.10]

# 回正模式下：jerk_gain 增益（把 setpoint 更快拉回）
RTC_JERK_MULT = 1.15

# 回正模式下：摩擦補償略增，幫助跨越靜摩擦（避免卡住不回）
RTC_FRIC_MULT = 1.08

# 回正模式下：積分釋放（避免 I 殘留讓方向盤“黏”）
# 若覺得回正還慢，可把 0.94 → 0.90；若覺得回正時有顆粒感，把它提高到 0.97
RTC_I_BLEED = 0.94


class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()

    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, KD, rate=1/self.dt)
    self.update_limits()

    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg

    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len,
                                          maxlen=self.lat_accel_request_buffer_len)

    self.previous_measurement = 0.0
    self.measurement_rate_filter = FirstOrderFilter(
      0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt
    )

    self._last_output_torque = 0.0
    self._was_active = False

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def _apply_torque_slew_limit(self, output_torque, v_ego, extra_mult=1.0):
    base_rate = float(np.interp(v_ego, TORQUE_SLEW_RATE_V_BP, TORQUE_SLEW_RATE_RATIOS))
    rate = base_rate * float(extra_mult)
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
    self.measurement_rate_filter = FirstOrderFilter(
      0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt
    )
    self._last_output_torque = 0.0

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION

    if not active:
      if self._was_active:
        self._reset_states()
      self._was_active = False
      output_torque = 0.0
      pid_log.active = False
      return -output_torque, 0.0, pid_log

    if not self._was_active:
      self._reset_states()
      self._was_active = True

    lat_delay_safe = float(np.clip(lat_delay, LAT_DELAY_MIN, LAT_ACCEL_REQUEST_BUFFER_SECONDS))

    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg),
                                           CS.vEgo, params.roll)
    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY

    dz_scale = float(np.interp(CS.vEgo, DEADZONE_SCALE_V_BP, DEADZONE_SCALE_VALS))
    steering_angle_deadzone_deg_eff = float(self.steering_angle_deadzone_deg) * dz_scale
    curvature_deadzone = abs(VM.calc_curvature(math.radians(steering_angle_deadzone_deg_eff), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    # --- 延遲 buffer 取 expected ---
    delay_frames = int(np.clip(lat_delay_safe / self.dt, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = float(self.lat_accel_request_buffer[-delay_frames])

    # --- 期望 a_lat（未來）與 jerk ---
    future_desired_lateral_accel = float(desired_curvature * CS.vEgo ** 2)
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation

    desired_lateral_jerk = float((future_desired_lateral_accel - expected_lateral_accel) / lat_delay_safe)

    # --- measurement 與 measurement_rate ---
    measurement = float(measured_curvature * CS.vEgo ** 2)
    measurement_rate = float(self.measurement_rate_filter.update((measurement - self.previous_measurement) / self.dt))
    self.previous_measurement = measurement

    # ============================================================
    # Return-to-center 模式判斷
    # - 想直行（desired 很小）但實際還在偏（meas 還不小） → 加速回正
    # ============================================================
    rtc = (abs(future_desired_lateral_accel) < RTC_DES_LATACCEL_THR) and (abs(measurement) > RTC_MEAS_LATACCEL_THR)

    # 回正時讓 expected 快速衰減，避免 setpoint “黏住”舊彎道狀態
    if rtc:
      expected_lateral_accel *= RTC_EXPECTED_DECAY
      desired_lateral_jerk = float((future_desired_lateral_accel - expected_lateral_accel) / lat_delay_safe)

    jerk_gain = float(np.interp(CS.vEgo, JERK_GAIN_V_BP, JERK_GAIN_VALS))
    if rtc:
      jerk_gain *= RTC_JERK_MULT

    setpoint = float((lat_delay_safe * jerk_gain) * desired_lateral_jerk + expected_lateral_accel)
    error = float(setpoint - measurement)
    pid_log.error = error

    # --- FF（lat accel 空間）---
    ff = float(gravity_adjusted_future_lateral_accel)
    ff -= float(self.torque_params.latAccelOffset)

    fric_scale = float(np.interp(CS.vEgo, FRIC_SCALE_V_BP, FRIC_SCALE_VALS))
    if rtc:
      fric_scale *= RTC_FRIC_MULT
    ff += fric_scale * get_friction(error, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    freeze_integrator = bool(steer_limited_by_safety or CS.steeringPressed or CS.vEgo < FREEZE_V_THRESH_MPS)

    output_lataccel = self.pid.update(error,
                                      -measurement_rate,
                                      feedforward=ff,
                                      speed=CS.vEgo,
                                      freeze_integrator=freeze_integrator)

    # 回正時，釋放積分殘留（避免回正“黏”）
    if rtc and (not freeze_integrator):
      self.pid.i *= RTC_I_BLEED

    output_torque = float(self.torque_from_lateral_accel(output_lataccel, self.torque_params))

    # 回正時放寬 slew：退扭更快、回正更利落
    slew_mult = 1.0
    if rtc:
      slew_mult = float(np.interp(CS.vEgo, RTC_SLEW_MULT_V_BP, RTC_SLEW_MULT_VALS))
    output_torque = float(self._apply_torque_slew_limit(output_torque, CS.vEgo, extra_mult=slew_mult))

    pid_log.active = True
    pid_log.p = float(self.pid.p)
    pid_log.i = float(self.pid.i)
    pid_log.d = float(self.pid.d)
    pid_log.f = float(self.pid.f)
    pid_log.output = float(-output_torque)
    pid_log.actualLateralAccel = float(measurement)
    pid_log.desiredLateralAccel = float(setpoint)
    pid_log.desiredLateralJerk = float(desired_lateral_jerk)
    pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3,
                                                    CS, steer_limited_by_safety, curvature_limited))

    return -output_torque, 0.0, pid_log
