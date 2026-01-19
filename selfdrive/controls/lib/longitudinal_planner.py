#!/usr/bin/env python3
import math
import numpy as np

import cereal.messaging as messaging
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.constants import CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import CONTROL_N, get_accel_from_plan
from openpilot.selfdrive.car.cruise import V_CRUISE_MAX, V_CRUISE_UNSET
from openpilot.common.swaglog import cloudlog
from dragonpilot.selfdrive.controls.lib.acm import ACM
from dragonpilot.selfdrive.controls.lib.aem import AEM
from dragonpilot.selfdrive.controls.lib.dtsc import DTSC


# ====================================================================
# ====================== 可調參數區（TUNING PARAMS） ======================
# ====================================================================

# ---------------------- 低速 E2E / MPC 混合（分兩段） ----------------------
# 你要求：
# (A) 0 <= v_kph < ENTER：也做 MPC+E2E 混合，但「MPC 比例隨車速上升而遞減」
# (B) ENTER < v_kph < EXIT：MPC 比例隨車速上升而遞增（原本設計）
#
# 因此本版 MPC 權重 w_mpc 分成兩段：
# 1) 低速段 [0, ENTER)：w_mpc 由 W_MPC_AT_ZERO 線性降到 W_MPC_AT_ENTER
# 2) 混合段 [ENTER, EXIT]：w_mpc 由 W_MPC_AT_ENTER 線性升到 1.0
#
# 重要：W_MPC_AT_ENTER 會同時作為兩段的銜接點，避免 ENTER 處不連續。
E2E_MPC_BLEND_ENABLE = True
E2E_ONLY_ENTER_KPH = 10.0
E2E_ONLY_EXIT_KPH  = 30.0

# 低速段端點權重（可調）
W_MPC_AT_ZERO  = 0.85   # v=0 時 MPC 權重（越大越偏 MPC；建議 0.6~1.0）
W_MPC_AT_ENTER = 0.20   # v=ENTER 時 MPC 權重（銜接點；建議 0.0~0.4）

# 混合曲線（t 的 gamma shaping）
# - 1.0：線性
# - >1：更偏兩端（中間變化更慢/更集中在尾端）
# - <1：更早進入下一端特性
E2E_MPC_BLEND_GAMMA_LOW = 1.0   # 低速段 [0,ENTER)
E2E_MPC_BLEND_GAMMA_MID = 1.0   # 中段   [ENTER,EXIT]

# 是否把「軌跡（speeds/accels/jerks）」也一起做混合（建議 True，debug 更一致）
BLEND_TRAJECTORY_ENABLE = True

# ---------------------- 低速「純感」：阻擋其他模組改輸出（可選） ----------------------
# 注意：本版是混合控制，不是純 E2E；這些是避免其他模組干擾你評估混合效果
E2E_ONLY_BLOCK_ACM  = True
E2E_ONLY_BLOCK_DTSC = False
E2E_ONLY_BLOCK_AEM  = False

# ---------------------- 硬安全規則（不可違反） ----------------------
LEAD_GUARD_ENABLE = True
MIN_LEAD_DIST_M = 5.0

TTC_CLOSING_MIN_MPS = 0.3
TTC_HARD_S = 1.2
HARD_BRAKE_DECEL = -4.0

TTC_SOFT_ENABLE = True
TTC_SOFT_START_S = 2.6

FCW_ENABLE = True
FCW_TTC_S = 1.0
FCW_COUNT_TRIG = 3
FCW_DECAY = 1

# ---------------------- v_desired_filter 反應加速（減少體感慢半拍） ----------------------
FAST_V_DESIRED_ENABLE = True
FAST_V_DESIRED_LOW_SPEED_KPH = 35.0
FAST_V_DESIRED_LEAD_DECEL_THRESH = -0.6
FAST_V_DESIRED_BLEND = 0.65

# ---------------------- accel_clip slew rate（上限變化不要太慢） ----------------------
# 重點：下限（ACCEL_MIN）不做 slew，避免緊急煞車被“slew 卡住”
ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S = 1.0
ACCEL_CLIP_SLEW_FAST_MPS2_PER_S = 2.5
ACCEL_CLIP_FAST_LOW_SPEED_KPH = 35.0
ACCEL_CLIP_FAST_LEAD_DECEL_THRESH = -0.8

# ---------------------- Throttle gating（保留你原本設定） ----------------------
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# ---------------------- A_CRUISE_MAX：速度 -> 最大允許加速度（m/s^2） ----------------------
A_CRUISE_MAX_VALS = [1.00, 1.085, 0.805, 0.644, 0.441, 0.245, 0.198]
A_CRUISE_MAX_BP   = [0.0,  8.33,  15.0,  20.0,  25.0,  30.0,  36.11]

# ---------------------------------------------------------
LON_MPC_STEP = 0.2  # first step is 0.2s
CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# Lookup table for turns（原版保留）
_A_TOTAL_MAX_V = [1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


class DPFlags:
  ACM = 1
  AEM = 2
  DTSC = 2 ** 2
  pass


def get_max_accel(v_ego):
  return float(np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS))


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  return [a_target[0], min(a_target[1], a_x_allowed)]


def _get_lead_decel_a(sm) -> float:
  try:
    if sm['radarState'].leadOne.status:
      return float(sm['radarState'].leadOne.aLeadK)
  except Exception:
    pass
  return 0.0


def _should_fast_response(v_ego: float, lead_a: float) -> bool:
  v_kph = v_ego * CV.MS_TO_KPH
  low_speed = v_kph < FAST_V_DESIRED_LOW_SPEED_KPH
  lead_braking = lead_a < FAST_V_DESIRED_LEAD_DECEL_THRESH
  return bool(low_speed or lead_braking)


def _accel_clip_slew_step(dt: float, v_ego: float, lead_a: float) -> float:
  v_kph = v_ego * CV.MS_TO_KPH
  fast = (v_kph < ACCEL_CLIP_FAST_LOW_SPEED_KPH) or (lead_a < ACCEL_CLIP_FAST_LEAD_DECEL_THRESH)
  slew_per_s = ACCEL_CLIP_SLEW_FAST_MPS2_PER_S if fast else ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S
  return float(max(0.01, slew_per_s * dt))


def _interp_to_control_n(arr_mpc):
  return np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, arr_mpc)


def _safe_model_action(sm):
  try:
    a_e2e = float(sm['modelV2'].action.desiredAcceleration)
    should_stop = bool(sm['modelV2'].action.shouldStop)
    if not np.isfinite(a_e2e):
      a_e2e = 0.0
      should_stop = False
    return a_e2e, should_stop
  except Exception:
    return 0.0, False


def _pick_closest_lead(radarstate):
  best = None
  best_d = 1e9
  for lead in (radarstate.leadOne, radarstate.leadTwo):
    try:
      if lead is not None and lead.status and np.isfinite(lead.dRel):
        if float(lead.dRel) < best_d:
          best_d = float(lead.dRel)
          best = lead
    except Exception:
      pass
  return best


def _compute_ttc(v_ego, lead):
  if lead is None:
    return float('inf'), 0.0, float('inf')

  d = float(getattr(lead, 'dRel', 1e9))
  v_lead = float(getattr(lead, 'vLead', v_ego))
  closing = float(v_ego - v_lead)

  if not np.isfinite(d) or d <= 0.0:
    return 0.0, closing, d

  if closing <= TTC_CLOSING_MIN_MPS:
    return float('inf'), closing, d

  ttc = d / max(closing, 1e-3)
  return float(ttc), closing, d


def _gamma_shape(t: float, gamma: float) -> float:
  t = float(np.clip(t, 0.0, 1.0))
  g = float(max(0.1, gamma))
  return float(np.clip(t ** g, 0.0, 1.0))


def _blend_weight_mpc(v_kph: float) -> float:
  """
  回傳 MPC 權重 w_mpc（0~1）

  你要求的分段：
  1) 0 <= v < ENTER：
     w_mpc 從 W_MPC_AT_ZERO 隨車速上升而「遞減」到 W_MPC_AT_ENTER
  2) ENTER <= v < EXIT：
     w_mpc 從 W_MPC_AT_ENTER 隨車速上升而「遞增」到 1.0
  3) v >= EXIT：
     w_mpc = 1.0

  備註：
  - 兩段在 ENTER 點用同一個 W_MPC_AT_ENTER，避免不連續。
  """
  if not E2E_MPC_BLEND_ENABLE:
    return 1.0

  enter = float(max(0.0, E2E_ONLY_ENTER_KPH))
  exitv = float(max(enter + 0.1, E2E_ONLY_EXIT_KPH))

  w0 = float(np.clip(W_MPC_AT_ZERO, 0.0, 1.0))
  we = float(np.clip(W_MPC_AT_ENTER, 0.0, 1.0))

  # 低速段：0 ~ ENTER（遞減）
  if v_kph < enter:
    if enter <= 0.1:
      return we
    t = float(np.clip(v_kph / enter, 0.0, 1.0))          # v=0 -> 0, v=enter -> 1
    t = _gamma_shape(t, E2E_MPC_BLEND_GAMMA_LOW)
    w = (1.0 - t) * w0 + t * we                          # 線性下降到 we
    return float(np.clip(w, 0.0, 1.0))

  # 中段：ENTER ~ EXIT（遞增）
  if v_kph < exitv:
    t = float(np.clip((v_kph - enter) / (exitv - enter), 0.0, 1.0))  # enter->0, exit->1
    t = _gamma_shape(t, E2E_MPC_BLEND_GAMMA_MID)
    w = (1.0 - t) * we + t * 1.0
    return float(np.clip(w, 0.0, 1.0))

  # 高速：全 MPC
  return 1.0


def _apply_lead_guard(v_ego, radarstate, a_target, should_stop, fcw_cnt):
  """
  硬安全規則（不可違反）：
  1) dRel < MIN_LEAD_DIST_M  -> 強制至少 hard brake + shouldStop=True
  2) TTC < TTC_HARD_S        -> 強制至少 hard brake（不可違反）
  3) TTC 軟介入（可選）      -> TTC < TTC_SOFT_START_S 開始逐步壓制到 hard brake
  4) FCW 用 TTC 計數
  """
  if not LEAD_GUARD_ENABLE:
    return float(a_target), bool(should_stop), False, int(fcw_cnt)

  lead = _pick_closest_lead(radarstate)
  ttc, closing, d = _compute_ttc(v_ego, lead)

  hard_brake = float(max(HARD_BRAKE_DECEL, ACCEL_MIN))

  a_new = float(a_target)
  stop_new = bool(should_stop)
  hard_violation = False

  if lead is not None and np.isfinite(d) and d < MIN_LEAD_DIST_M:
    a_new = min(a_new, hard_brake)
    stop_new = True
    hard_violation = True

  if np.isfinite(ttc) and ttc < TTC_HARD_S:
    a_new = min(a_new, hard_brake)
    hard_violation = True

  if TTC_SOFT_ENABLE and np.isfinite(ttc) and (TTC_HARD_S < ttc < TTC_SOFT_START_S):
    w = (TTC_SOFT_START_S - ttc) / max(TTC_SOFT_START_S - TTC_HARD_S, 1e-3)
    w = float(np.clip(w, 0.0, 1.0))
    a_soft = (1.0 - w) * a_new + w * hard_brake
    a_new = min(a_new, float(a_soft))

  fcw = False
  cnt = int(fcw_cnt)
  if FCW_ENABLE and np.isfinite(ttc) and (ttc < FCW_TTC_S):
    cnt = min(cnt + 1, FCW_COUNT_TRIG + 5)
  else:
    cnt = max(cnt - FCW_DECAY, 0)
  fcw = cnt >= FCW_COUNT_TRIG

  return float(a_new), bool(stop_new), bool(fcw), int(cnt)


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    self.mpc.mode = 'acc'
    self.dt = dt

    self.fcw = False
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)

    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)

    self.acm = ACM()
    self.aem = AEM()
    self.dtsc = DTSC(aggressiveness=0.8)

    self.ttc_fcw_cnt = 0

  @staticmethod
  def parse_model(model_msg):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
      len(model_msg.velocity.x) == ModelConstants.IDX_N and
      len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))

    if len(model_msg.meta.disengagePredictions.gasPressProbs) > 1:
      throttle_prob = model_msg.meta.disengagePredictions.gasPressProbs[1]
    else:
      throttle_prob = 1.0
    return x, v, a, j, throttle_prob

  def update(self, sm, dp_flags=0):
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_kph = v_ego * CV.MS_TO_KPH

    if (dp_flags & DPFlags.AEM) and not (v_kph < E2E_ONLY_EXIT_KPH and E2E_ONLY_BLOCK_AEM):
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=v_ego)
      mode = self.aem.get_mode(mode)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    lead_a = _get_lead_decel_a(sm)
    fast_response = FAST_V_DESIRED_ENABLE and _should_fast_response(v_ego, lead_a)

    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or not v_cruise_initialized
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # accel_clip：ACC + blended 都套用 A_CRUISE_MAX
    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    if mode == 'acc':
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    if fast_response:
      self.v_desired_filter.x = (1.0 - FAST_V_DESIRED_BLEND) * self.v_desired_filter.x + FAST_V_DESIRED_BLEND * v_ego

    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])

    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED
    if not self.allow_throttle:
      clipped_accel_coast = max(accel_coast, accel_clip[0])
      clipped_accel_coast_interp = np.interp(
        v_ego,
        [MIN_ALLOW_THROTTLE_SPEED, MIN_ALLOW_THROTTLE_SPEED * 2],
        [accel_clip[1], clipped_accel_coast]
      )
      accel_clip[1] = min(accel_clip[1], clipped_accel_coast_interp)

    if force_slow_decel:
      v_cruise = 0.0

    # MPC：一律計算（方法B：假設 long_mpc.update 支援 a_min/a_max）
    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    a_min_mpc = float(accel_clip[0])
    a_max_mpc = float(accel_clip[1])

    use_dtsc = (dp_flags & DPFlags.DTSC) and not (v_kph < E2E_ONLY_EXIT_KPH and E2E_ONLY_BLOCK_DTSC)
    if use_dtsc:
      a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(sm['modelV2'], v_ego, accel_clip[0], accel_clip[1])
      a_min_mpc = np.maximum(a_min_mpc, np.asarray(a_min_dtsc, dtype=float))
      a_max_mpc = np.minimum(a_max_mpc, np.asarray(a_max_dtsc, dtype=float))

    self.mpc.update(
      sm['radarState'], v_cruise, x, v, a, j,
      personality=sm['selfdriveState'].personality,
      a_min=a_min_mpc, a_max=a_max_mpc
    )

    # MPC trajectory
    v_traj_mpc = _interp_to_control_n(self.mpc.v_solution)
    a_traj_mpc = _interp_to_control_n(self.mpc.a_solution)
    j_traj_mpc = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # E2E trajectory
    v_traj_e2e = _interp_to_control_n(v)
    a_traj_e2e = _interp_to_control_n(a)
    j_traj_e2e = np.zeros_like(v_traj_e2e)
    if len(j_traj_e2e) > 1:
      j_traj_e2e[1:] = np.diff(a_traj_e2e) / max(self.dt, 1e-3)
      j_traj_e2e[0] = j_traj_e2e[1]

    # actions：E2E / MPC
    a_e2e_action, should_stop_e2e = _safe_model_action(sm)
    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    a_mpc_action, should_stop_mpc = get_accel_from_plan(
      v_traj_mpc, a_traj_mpc, CONTROL_N_T_IDX,
      action_t=action_t, vEgoStopping=self.CP.vEgoStopping
    )

    # 依車速混合（含你新增的 0~ENTER 反向遞減段）
    w_mpc = _blend_weight_mpc(v_kph)
    w_e2e = 1.0 - w_mpc

    a_blend = float(w_mpc * a_mpc_action + w_e2e * a_e2e_action)
    should_stop_blend = bool(should_stop_mpc or should_stop_e2e)

    if BLEND_TRAJECTORY_ENABLE and E2E_MPC_BLEND_ENABLE:
      self.v_desired_trajectory = w_mpc * v_traj_mpc + w_e2e * v_traj_e2e
      self.a_desired_trajectory = w_mpc * a_traj_mpc + w_e2e * a_traj_e2e
      self.j_desired_trajectory = w_mpc * j_traj_mpc + w_e2e * j_traj_e2e
    else:
      self.v_desired_trajectory = v_traj_mpc
      self.a_desired_trajectory = a_traj_mpc
      self.j_desired_trajectory = j_traj_mpc

    # ACM：低速區可阻擋（< EXIT）
    acm_enabled = bool(dp_flags & DPFlags.ACM) and not (v_kph < E2E_ONLY_EXIT_KPH and E2E_ONLY_BLOCK_ACM)
    if acm_enabled:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)

    # 硬安全規則（不可違反）
    a_guarded, stop_guarded, fcw_ttc, self.ttc_fcw_cnt = _apply_lead_guard(
      v_ego, sm['radarState'], a_blend, should_stop_blend, self.ttc_fcw_cnt
    )

    self.output_should_stop = bool(stop_guarded)

    fcw_mpc = (self.mpc.crash_cnt > 2 and not sm['carState'].standstill)
    self.fcw = bool(fcw_mpc or fcw_ttc)
    if self.fcw:
      cloudlog.info("FCW triggered")

    # 狀態推進（用 trajectory）
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    # accel_clip slew：只對上限做（下限固定 ACCEL_MIN）
    max_delta = _accel_clip_slew_step(self.dt, v_ego, lead_a)
    accel_clip[1] = float(np.clip(
      accel_clip[1],
      self.prev_accel_clip[1] - max_delta,
      self.prev_accel_clip[1] + max_delta
    ))
    self.prev_accel_clip = [ACCEL_MIN, accel_clip[1]]

    # 最終輸出 clip
    self.output_a_target = float(np.clip(a_guarded, accel_clip[0], accel_clip[1]))

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')
    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState', 'selfdriveState', 'radarState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']
    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
