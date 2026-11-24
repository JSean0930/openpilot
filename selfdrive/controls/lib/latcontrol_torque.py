import math
import numpy as np
from collections import deque

from cereal import log
from opendbc.car.lateral import FRICTION_THRESHOLD, get_friction
from openpilot.common.constants import ACCELERATION_DUE_TO_GRAVITY
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.selfdrive.controls.lib.latcontrol import LatControl
from openpilot.common.pid import PIDController

# At higher speeds (25+mph) we can assume:
# Lateral acceleration achieved by a specific car correlates to
# torque applied to the steering rack. It does not correlate to
# wheel slip, or to speed.

# This controller applies torque to achieve desired lateral
# accelerations. To compensate for the low speed effects the
# proportional gain is increased at low speeds by the PID controller.
# Additionally, there is friction in the steering wheel that needs
# to be overcome to move it at all, this is compensated for too.

KP = 0.8
KI = 0.15

INTERP_SPEEDS = [1, 1.5, 2.0, 3.0, 5, 7.5, 10, 15, 30]
KP_INTERP = [250, 120, 65, 30, 11.5, 5.5, 3.5, 2.0, KP]

LP_FILTER_CUTOFF_HZ = 1.2
JERK_LOOKAHEAD_SECONDS = 0.19
JERK_GAIN = 0.3
LAT_ACCEL_REQUEST_BUFFER_SECONDS = 1.0
VERSION = 1

# --- humanlike lateral (dp fork) ---
# 1) Straight-line ping-pong suppression.
#    On near-straight roads only, soften the P term (and the friction input) for *small* errors.
#    The integrator still sees the full error (no steady-state offset) and the feedforward is untouched,
#    so curve entry (driven by FF + jerk lookahead) is not delayed. Set STRAIGHT_SHAPING_ENABLED = False to disable.
STRAIGHT_SHAPING_ENABLED = True
STRAIGHT_MIN_SPEED_BP = [12., 16.]       # m/s: no shaping below 12 m/s (KP_INTERP region), full above 16 m/s
STRAIGHT_LAT_ACCEL_BP = [0.3, 0.6]       # |desired lat accel| m/s^2: full shaping below 0.3, none above 0.6
STRAIGHT_LAT_JERK_BP = [0.3, 0.8]        # |desired lat jerk| m/s^3: fades out as soon as a curve is being requested
SMALL_ERROR_BP = [0.0, 0.1, 0.25]        # |error| m/s^2
SMALL_ERROR_P_SCALE = [0.5, 0.7, 1.0]    # e * scale(|e|) stays monotonic -> no limit cycle from a hard deadzone

# 2) Driver-override softening ("not fighting the driver").
#    When the driver applies torque *against* our command (steeringPressed, but below the carcontroller cut
#    threshold, e.g. Toyota 100..500), ramp our torque down; ramp back up slowly on release.
DRIVER_OVERRIDE_SOFTEN_ENABLED = True
DRIVER_OVERRIDE_SCALE = 0.6
DRIVER_OVERRIDE_RAMP_DOWN_S = 0.2        # time to go 1.0 -> DRIVER_OVERRIDE_SCALE
DRIVER_OVERRIDE_RAMP_UP_S = 0.8          # time to go DRIVER_OVERRIDE_SCALE -> 1.0

class LatControlTorque(LatControl):
  def __init__(self, CP, CI, dt):
    super().__init__(CP, CI, dt)
    self.torque_params = CP.lateralTuning.torque.as_builder()
    self.torque_from_lateral_accel = CI.torque_from_lateral_accel()
    self.lateral_accel_from_torque = CI.lateral_accel_from_torque()
    self.pid = PIDController([INTERP_SPEEDS, KP_INTERP], KI, rate=1/self.dt)
    self.update_limits()
    self.steering_angle_deadzone_deg = self.torque_params.steeringAngleDeadzoneDeg
    self.lat_accel_request_buffer_len = int(LAT_ACCEL_REQUEST_BUFFER_SECONDS / self.dt)
    self.lat_accel_request_buffer = deque([0.] * self.lat_accel_request_buffer_len , maxlen=self.lat_accel_request_buffer_len)
    self.lookahead_frames = int(JERK_LOOKAHEAD_SECONDS / self.dt)
    self.jerk_filter = FirstOrderFilter(0.0, 1 / (2 * np.pi * LP_FILTER_CUTOFF_HZ), self.dt)
    self.override_scale = 1.0

  def reset(self):
    super().reset()
    self.override_scale = 1.0

  def _straight_factor(self, v_ego, desired_lateral_accel, desired_lateral_jerk):
    # 1.0 = clearly driving straight at speed, 0.0 = curve / low speed. Product of smooth ramps, no switching.
    if not STRAIGHT_SHAPING_ENABLED:
      return 0.0
    speed_w = np.interp(v_ego, STRAIGHT_MIN_SPEED_BP, [0.0, 1.0])
    accel_w = np.interp(abs(desired_lateral_accel), STRAIGHT_LAT_ACCEL_BP, [1.0, 0.0])
    jerk_w = np.interp(abs(desired_lateral_jerk), STRAIGHT_LAT_JERK_BP, [1.0, 0.0])
    return float(speed_w * accel_w * jerk_w)

  def _update_override_scale(self, CS, commanded_torque):
    # steeringTorque and the returned actuator torque (-output_torque) are both left-positive
    opposing = CS.steeringPressed and (CS.steeringTorque * -commanded_torque) < 0
    target = DRIVER_OVERRIDE_SCALE if (DRIVER_OVERRIDE_SOFTEN_ENABLED and opposing) else 1.0
    span = 1.0 - DRIVER_OVERRIDE_SCALE
    if target < self.override_scale:
      self.override_scale = max(target, self.override_scale - span * self.dt / DRIVER_OVERRIDE_RAMP_DOWN_S)
    else:
      self.override_scale = min(target, self.override_scale + span * self.dt / DRIVER_OVERRIDE_RAMP_UP_S)
    return self.override_scale

  def update_live_torque_params(self, latAccelFactor, latAccelOffset, friction):
    self.torque_params.latAccelFactor = latAccelFactor
    self.torque_params.latAccelOffset = latAccelOffset
    self.torque_params.friction = friction
    self.update_limits()

  def update_limits(self):
    self.pid.set_limits(self.lateral_accel_from_torque(self.steer_max, self.torque_params),
                        self.lateral_accel_from_torque(-self.steer_max, self.torque_params))

  def update(self, active, CS, VM, params, steer_limited_by_safety, desired_curvature, curvature_limited, lat_delay):
    pid_log = log.ControlsState.LateralTorqueState.new_message()
    pid_log.version = VERSION
    measured_curvature = -VM.calc_curvature(math.radians(CS.steeringAngleDeg - params.angleOffsetDeg), CS.vEgo, params.roll)
    measurement = measured_curvature * CS.vEgo ** 2
    future_desired_lateral_accel = desired_curvature * CS.vEgo ** 2
    self.lat_accel_request_buffer.append(future_desired_lateral_accel)

    roll_compensation = params.roll * ACCELERATION_DUE_TO_GRAVITY
    curvature_deadzone = abs(VM.calc_curvature(math.radians(self.steering_angle_deadzone_deg), CS.vEgo, 0.0))
    lateral_accel_deadzone = curvature_deadzone * CS.vEgo ** 2

    delay_frames = int(np.clip(lat_delay / self.dt + 1, 1, self.lat_accel_request_buffer_len))
    expected_lateral_accel = self.lat_accel_request_buffer[-delay_frames]
    setpoint = expected_lateral_accel
    error = setpoint - measurement

    lookahead_idx = int(np.clip(-delay_frames + self.lookahead_frames, -self.lat_accel_request_buffer_len+1, -2))
    raw_lateral_jerk = (self.lat_accel_request_buffer[lookahead_idx+1] - self.lat_accel_request_buffer[lookahead_idx-1]) / (2 * self.dt)
    desired_lateral_jerk = self.jerk_filter.update(raw_lateral_jerk)
    gravity_adjusted_future_lateral_accel = future_desired_lateral_accel - roll_compensation

    # straight-line small-error shaping (P and friction input only)
    straight_w = self._straight_factor(CS.vEgo, setpoint, desired_lateral_jerk)
    small_err_scale = float(np.interp(abs(error), SMALL_ERROR_BP, SMALL_ERROR_P_SCALE))
    p_scale = 1.0 - straight_w * (1.0 - small_err_scale)

    ff = gravity_adjusted_future_lateral_accel
    # latAccelOffset corrects roll compensation bias from device roll misalignment relative to car roll
    ff -= self.torque_params.latAccelOffset
    ff += get_friction(p_scale * error + JERK_GAIN * desired_lateral_jerk, lateral_accel_deadzone, FRICTION_THRESHOLD, self.torque_params)

    if not active:
      output_torque = 0.0
      self.override_scale = 1.0
      pid_log.active = False
    else:
      # do error correction in lateral acceleration space, convert at end to handle non-linear torque responses correctly
      pid_log.error = float(error)

      freeze_integrator = steer_limited_by_safety or CS.steeringPressed or CS.vEgo < 5
      output_lataccel = self.pid.update(pid_log.error, speed=CS.vEgo, feedforward=ff, freeze_integrator=freeze_integrator)
      if p_scale < 1.0:
        # remove the softened part of P; integrator already updated with the full error
        output_lataccel = float(np.clip(output_lataccel - (1.0 - p_scale) * self.pid.p, self.pid.neg_limit, self.pid.pos_limit))
        self.pid.p *= p_scale
      output_torque = self.torque_from_lateral_accel(output_lataccel, self.torque_params)
      output_torque *= self._update_override_scale(CS, output_torque)

      pid_log.active = True
      pid_log.p = float(self.pid.p)
      pid_log.i = float(self.pid.i)
      pid_log.d = float(self.pid.d)
      pid_log.f = float(self.pid.f)
      pid_log.output = float(-output_torque) # TODO: log lat accel?
      pid_log.actualLateralAccel = float(measurement)
      pid_log.desiredLateralAccel = float(setpoint)
      pid_log.desiredLateralJerk = float(desired_lateral_jerk)
      pid_log.saturated = bool(self._check_saturation(self.steer_max - abs(output_torque) < 1e-3, CS, steer_limited_by_safety, curvature_limited))

    # TODO left is positive in this convention
    return -output_torque, 0.0, pid_log
