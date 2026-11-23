import numpy as np
from cereal import car
from openpilot.common.realtime import DT_CTRL
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N
from openpilot.common.pid import PIDController
from openpilot.selfdrive.modeld.constants import ModelConstants

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

LongCtrlState = car.CarControl.Actuators.LongControlState

# 起步靜摩擦補償參數
# 只在「openpilot 端 PID 有閉迴路增益」的車種啟用（此時補償只進前饋，PID 會把實際加速度拉回 a_target）。
# Toyota / VW 等 kpV=kiV=0 的車種，actuators.accel 會被車端 PID 當成目標加速度追蹤，
# 補償等於真的多要加速度且被車端積分放大 → 自動停用，起步補償交給 CarController。
STICTION_BOOST_FORCE = False  # True：不論車種都啟用（不建議用於 Toyota）
STICTION_BOOST_MAX = 0.4      # m/s^2，靜止時最大補償（與原本 v=0 時的 0.4 相同）
STICTION_FADE_SPEED = 2.78    # m/s (10 km/h)，在此速度完全淡出
STICTION_A_TARGET_FULL = 0.3  # m/s^2，a_target 達此值才給滿補償


def get_stiction_boost(v_ego, a_target):
  """連續、非負的起步補償；v_ego >= STICTION_FADE_SPEED 或 a_target <= 0 時為 0"""
  if a_target <= 0.0:
    return 0.0
  x = float(np.clip(1.0 - v_ego / STICTION_FADE_SPEED, 0.0, 1.0))
  speed_scale = x * x * (3.0 - 2.0 * x)  # smoothstep：兩端斜率為 0，淡出處無轉折
  accel_scale = float(np.clip(a_target / STICTION_A_TARGET_FULL, 0.0, 1.0))
  return STICTION_BOOST_MAX * speed_scale * accel_scale


def has_closed_loop_long_pid(CP):
  return any(k != 0.0 for k in list(CP.longitudinalTuning.kpV) + list(CP.longitudinalTuning.kiV))


def long_control_state_trans(CP, active, long_control_state, v_ego,
                             should_stop, brake_pressed, cruise_standstill):
  stopping_condition = should_stop
  starting_condition = (not should_stop and
                        not cruise_standstill and
                        not brake_pressed)
  started_condition = v_ego > CP.vEgoStarting

  if not active:
    long_control_state = LongCtrlState.off

  else:
    if long_control_state == LongCtrlState.off:
      if not starting_condition:
        long_control_state = LongCtrlState.stopping
      else:
        if starting_condition and CP.startingState:
          long_control_state = LongCtrlState.starting
        else:
          long_control_state = LongCtrlState.pid

    elif long_control_state == LongCtrlState.stopping:
      if starting_condition and CP.startingState:
        long_control_state = LongCtrlState.starting
      elif starting_condition:
        long_control_state = LongCtrlState.pid

    elif long_control_state in [LongCtrlState.starting, LongCtrlState.pid]:
      if stopping_condition:
        long_control_state = LongCtrlState.stopping
      elif started_condition:
        long_control_state = LongCtrlState.pid
  return long_control_state

class LongControl:
  def __init__(self, CP):
    self.CP = CP
    self.long_control_state = LongCtrlState.off
    self.pid = PIDController((CP.longitudinalTuning.kpBP, CP.longitudinalTuning.kpV),
                             (CP.longitudinalTuning.kiBP, CP.longitudinalTuning.kiV),
                             rate=1 / DT_CTRL)
    self.last_output_accel = 0.0
    self.stiction_boost_enabled = STICTION_BOOST_FORCE or has_closed_loop_long_pid(CP)

  def reset(self):
    self.pid.reset()

  def update(self, active, CS, a_target, should_stop, accel_limits):
    """Update longitudinal control. This updates the state machine and runs a PID loop"""
    # 確保進入運算前，先賦予最新的安全極限 (防止積分飽和)
    self.pid.neg_limit = accel_limits[0]
    self.pid.pos_limit = accel_limits[1]

    self.long_control_state = long_control_state_trans(self.CP, active, self.long_control_state, CS.vEgo,
                                                       should_stop, CS.brakePressed,
                                                       CS.cruiseState.standstill)
    if self.long_control_state == LongCtrlState.off:
      self.reset()
      output_accel = 0.

    elif self.long_control_state == LongCtrlState.stopping:
      output_accel = self.last_output_accel
      if output_accel > self.CP.stopAccel:
        output_accel = min(output_accel, 0.0)
        output_accel -= self.CP.stoppingDecelRate * DT_CTRL
      self.reset()

    elif self.long_control_state == LongCtrlState.starting:
      output_accel = self.CP.startAccel
      self.reset()

    else:  # LongCtrlState.pid
      # ============================================================
      # 起步靜摩擦補償 (Stiction Boost)
      #  - 只加在前饋 (feedforward)，不改 PID 追蹤目標：
      #    目的是「推得動」，不是讓車實際加速度超過 planner 要的 a_target
      #  - 對車速：連續、非負、以 smoothstep 在 STICTION_FADE_SPEED 淡出為 0（無跳階）
      #  - 對 a_target：a_target 越接近 0 補償越小，避免微小加速需求被放大成「竄」
      # ============================================================
      stiction_boost = get_stiction_boost(CS.vEgo, a_target) if self.stiction_boost_enabled else 0.0
      error = a_target - CS.aEgo

      # 保留 standstill 時凍結積分，避免停等時積分累積造成起步暴衝
      output_accel = self.pid.update(error, speed=CS.vEgo,
                                     feedforward=a_target + stiction_boost,
                                     freeze_integrator=CS.cruiseState.standstill)

    # 最終輸出安全限制
    self.last_output_accel = np.clip(output_accel, accel_limits[0], accel_limits[1])
    return self.last_output_accel
