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
# 調參提示（僅註解）
# - 低速跟停還想更敏感：FAST_V_DESIRED_BLEND 往上加到 0.75~0.8
# - 若覺得偶爾有點「急」：ACCEL_CLIP_SLEW_FAST_MPS2_PER_S 往下調到 2.0
# - 若你主要想改善「前車一煞立刻反應」：FAST_V_DESIRED_LEAD_DECEL_THRESH 從 -0.6 改 -0.5
# ====================================================================

# ====================== 可調參數區（TUNING PARAMS） ======================

# --- 方案 5A：低速純 E2E（e2e_only_active 時不跑 mpc.update） ---
E2E_ONLY_ENABLE = True
E2E_ONLY_ENTER_KPH = 58.0   # 進入純E2E（建議 55~60）
E2E_ONLY_EXIT_KPH  = 62.0   # 離開純E2E（> enter，做 hysteresis 防抖）

# --- 低速純E2E時，阻擋其他模組「改動輸出」的開關 ---
# 目的：避免 ACM/DTSC/AEM 等介入，導致體感「不是純E2E」
E2E_ONLY_BLOCK_ACM  = True  # True: 低速純E2E時，不套用 ACM 修飾
E2E_ONLY_BLOCK_DTSC = True  # True: 低速純E2E時，不套用 DTSC 產生的 per-stage a_min/a_max
E2E_ONLY_BLOCK_AEM  = True  # True: 低速純E2E時，不讓 AEM 改 mode（避免模式被切走）

# --- [新增] 低速純E2E的安全補丁：TTC + 最小距離 ---
# 你目前問題：前車減速到停，有時 E2E 來不及煞車 -> 在 planner 端加「底線」保護
E2E_TTC_GUARD_ENABLE = True

E2E_MIN_LEAD_DIST_M = 5.0          # 距離前車不可低於 5m（硬底線）
E2E_MIN_LEAD_DIST_HARD_BRAKE = -4.0  # 低於底線時，至少給這個減速度（m/s^2，越負越兇）

E2E_TTC_CLOSING_MIN_MPS = 0.3      # 只有 closing speed > 這個才算「真的在追近」（避免抖動）
E2E_TTC_BRAKE_START_S = 2.6        # TTC < 這個開始「介入煞車」
E2E_TTC_BRAKE_FULL_S  = 1.4        # TTC < 這個介入到「最強煞車」
E2E_TTC_BRAKE_MAX_DECEL = -3.2     # TTC 介入時最多給到的減速度（m/s^2，可調更兇/更柔）

# --- [新增] 低速純E2E的簡易 FCW（因為 5A 不跑 mpc.update，crash_cnt 不可用） ---
E2E_FCW_ENABLE = True
E2E_FCW_TTC_S = 1.0                # TTC 小於此值就累積 FCW 計數（可調 0.8~1.2）
E2E_FCW_COUNT_TRIG = 3             # 累積幾次觸發 FCW（類似你原本 crash_cnt>2）
E2E_FCW_DECAY = 1                  # 未達條件時每回合衰退多少（越大越不容易黏住）

# --- v_desired_filter 反應加速（減少體感慢半拍） ---
FAST_V_DESIRED_ENABLE = True
FAST_V_DESIRED_LOW_SPEED_KPH = 35.0            # 低於此速就更敏感（km/h）
FAST_V_DESIRED_LEAD_DECEL_THRESH = -0.6        # leadOne.aLeadK 小於此值視為前車在明顯減速（m/s^2）
FAST_V_DESIRED_BLEND = 0.65                    # 0~1，越大越快貼近 v_ego（建議 0.5~0.8）

# --- accel_clip slew rate（原本固定 0.05/step 太慢） ---
ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S = 1.0        # 原本 0.05@20Hz ≈ 1.0 m/s^2 per sec
ACCEL_CLIP_SLEW_FAST_MPS2_PER_S = 2.5          # 低速/前車強減速時放寬（建議 2.0~3.5）
ACCEL_CLIP_FAST_LOW_SPEED_KPH = 35.0           # 低於此速可放寬（km/h）
ACCEL_CLIP_FAST_LEAD_DECEL_THRESH = -0.8       # 前車減速強於此值可放寬（m/s^2）

# --- Throttle gating（保留你原本設定） ---
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# =======================================================================

LON_MPC_STEP = 0.2  # first step is 0.2s

# A_CRUISE_MAX：速度 -> 最大允許加速度（m/s^2）
# 注意：本版維持「ACC + blended 都套用」（方法B配套）
A_CRUISE_MAX_VALS = [1.00, 1.085, 0.805, 0.644, 0.441, 0.245, 0.198]
A_CRUISE_MAX_BP   = [0.0,  8.33,  15.0,  20.0,  25.0,  30.0,  36.11]

CONTROL_N_T_IDX = ModelConstants.T_IDXS[:CONTROL_N]

# Lookup table for turns
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
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  returns limited longitudinal accel allowed, depending on lateral accel
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))
  return [a_target[0], min(a_target[1], a_x_allowed)]


def _get_lead_decel_a(sm) -> float:
  """取得 leadOne 的 aLeadK（若不可用則回傳 0.0）"""
  try:
    if sm['radarState'].leadOne.status:
      return float(sm['radarState'].leadOne.aLeadK)
  except Exception:
    pass
  return 0.0


def _should_fast_response(v_ego: float, lead_a: float) -> bool:
  """
  決定是否啟用「更快反應」：
  - 低速更敏感（壅塞/跟停）
  - 前車明顯減速更敏感
  """
  v_kph = v_ego * CV.MS_TO_KPH
  low_speed = v_kph < FAST_V_DESIRED_LOW_SPEED_KPH
  lead_braking = lead_a < FAST_V_DESIRED_LEAD_DECEL_THRESH
  return bool(low_speed or lead_braking)


def _accel_clip_slew_step(dt: float, v_ego: float, lead_a: float) -> float:
  """回傳 accel_clip 每 step 的最大允許變化量（m/s^2）"""
  v_kph = v_ego * CV.MS_TO_KPH
  fast = (v_kph < ACCEL_CLIP_FAST_LOW_SPEED_KPH) or (lead_a < ACCEL_CLIP_FAST_LEAD_DECEL_THRESH)
  slew_per_s = ACCEL_CLIP_SLEW_FAST_MPS2_PER_S if fast else ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S
  return float(max(0.01, slew_per_s * dt))  # 最低給 0.01 避免完全卡死


def _interp_to_control_n(arr_mpc):
  """把 (len(T_IDXS_MPC)) 插值到 CONTROL_N 長度（給 publish 用）"""
  return np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, arr_mpc)


def _safe_model_action(sm):
  """
  讀取 modelV2.action，並做簡單防呆：
  - 若欄位不存在或不合理，回傳 (0.0, False)
  """
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
  """
  選擇「最需要注意」的 lead（取 dRel 最小者）
  回傳 None 或 lead 物件（具備 dRel/vLead/status）
  """
  best = None
  best_d = 1e9
  for lead in (radarstate.leadOne, radarstate.leadTwo):
    try:
      if lead is not None and lead.status and np.isfinite(lead.dRel):
        if lead.dRel < best_d:
          best_d = float(lead.dRel)
          best = lead
    except Exception:
      pass
  return best


def _compute_ttc(v_ego, lead):
  """
  簡易 TTC：
  - closing_speed = v_ego - v_lead
  - 只有 closing_speed > E2E_TTC_CLOSING_MIN_MPS 才計算 TTC（避免抖動）
  """
  if lead is None:
    return float('inf'), 0.0, float('inf')

  d = float(getattr(lead, 'dRel', 1e9))
  v_lead = float(getattr(lead, 'vLead', v_ego))
  closing = float(v_ego - v_lead)

  if not np.isfinite(d) or d <= 0.0:
    return 0.0, closing, d

  if closing <= E2E_TTC_CLOSING_MIN_MPS:
    return float('inf'), closing, d

  ttc = d / max(closing, 1e-3)
  return float(ttc), closing, d


def _ttc_brake_override(v_ego, radarstate, a_target):
  """
  低速純E2E用的安全補丁：
  1) 距離底線：dRel < E2E_MIN_LEAD_DIST_M -> 強制至少給 hard brake
  2) TTC 介入：TTC < start -> 開始壓 a_target 往負；TTC < full -> 到最強

  回傳：
  - a_target_new
  - should_stop_override（底線距離時可選擇 True）
  - ttc, d, closing（便於 debug/log）
  """
  lead = _pick_closest_lead(radarstate)
  ttc, closing, d = _compute_ttc(v_ego, lead)

  a_new = float(a_target)
  should_stop = False

  if lead is None:
    return a_new, should_stop, ttc, d, closing

  # 1) 距離硬底線（你要求：不可低於 5m）
  if np.isfinite(d) and d < E2E_MIN_LEAD_DIST_M:
    a_new = min(a_new, float(E2E_MIN_LEAD_DIST_HARD_BRAKE))
    # 低於 5m 通常已經非常危險，直接允許 shouldStop 幫你更保守
    should_stop = True

  # 2) TTC 介入（用線性插值把 a_target 壓到負向）
  if np.isfinite(ttc) and ttc < E2E_TTC_BRAKE_START_S:
    # 介入程度：ttc=START -> 0；ttc=FULL -> 1
    if E2E_TTC_BRAKE_START_S > E2E_TTC_BRAKE_FULL_S:
      w = (E2E_TTC_BRAKE_START_S - ttc) / max(E2E_TTC_BRAKE_START_S - E2E_TTC_BRAKE_FULL_S, 1e-3)
    else:
      w = 1.0
    w = float(np.clip(w, 0.0, 1.0))
    # 目標煞車值（越危險越接近 max_decel）
    a_brake = float(E2E_TTC_BRAKE_MAX_DECEL)
    # 只做「往更負」方向的介入（不會把你煞車變小）
    a_new = min(a_new, (1.0 - w) * a_new + w * a_brake)

  return a_new, should_stop, ttc, d, closing


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
    # TODO remove mpc modes when TR released
    self.mpc.mode = 'acc'
    self.fcw = False
    self.dt = dt
    self.allow_throttle = True

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.prev_accel_clip = [ACCEL_MIN, ACCEL_MAX]
    self.output_a_target = 0.0
    self.output_should_stop = False

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.acm = ACM()
    self.aem = AEM()
    self.dtsc = DTSC(aggressiveness=0.8)

    # 低速純E2E狀態（hysteresis 防抖）
    self.e2e_only_active = False

    # 方案5A：低速 FCW 用 TTC 計數
    self.ttc_fcw_cnt = 0

  @staticmethod
  def parse_model(model_msg):
    # 這裡的 x/v/a 是「縱向軌跡」（不是橫向）
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

  def _update_e2e_only_state(self, v_ego: float):
    """低速純E2E狀態機（hysteresis）"""
    if not E2E_ONLY_ENABLE:
      self.e2e_only_active = False
      return

    v_kph = v_ego * CV.MS_TO_KPH
    if self.e2e_only_active:
      if v_kph > E2E_ONLY_EXIT_KPH:
        self.e2e_only_active = False
    else:
      if v_kph < E2E_ONLY_ENTER_KPH:
        self.e2e_only_active = True

  def update(self, sm, dp_flags=0):
    # ------------------------------------------------------------
    # mode 的來源：
    # - experimentalMode => blended
    # - 否則 acc
    # - AEM 可能改 mode
    # ------------------------------------------------------------
    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    self._update_e2e_only_state(v_ego)

    # 低速純E2E時，避免 AEM 改 mode（你要求更「純」）
    if (dp_flags & DPFlags.AEM) and not (self.e2e_only_active and E2E_ONLY_BLOCK_AEM):
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=v_ego)
      mode = self.aem.get_mode(mode)

    # coast accel（用於 throttle gating）
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

    # ============================================================
    # ACC + blended 都套用 A_CRUISE_MAX
    # - accel_clip[1]：速度上限加速度（含你調的表）
    # - 轉彎加速限制：維持只在 ACC 套用
    # ============================================================
    accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
    if mode == 'acc':
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # 快速反應：縮短體感延遲
    if fast_response:
      self.v_desired_filter.x = (1.0 - FAST_V_DESIRED_BLEND) * self.v_desired_filter.x + FAST_V_DESIRED_BLEND * v_ego

    # 解析 model 縱向軌跡
    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])

    # Don't clip at low speeds since throttle_prob doesn't account for creep
    self.allow_throttle = throttle_prob > ALLOW_THROTTLE_THRESHOLD or v_ego <= MIN_ALLOW_THROTTLE_SPEED

    # 若不允許油門：把 accel_clip[1] 壓到 coast/限制更保守
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

    # ============================================================
    # 方案 5A 核心差異：
    # - e2e_only_active 時「不跑 mpc.update」 -> 更純、更省算力
    # - 因此：低速 FCW 改用「簡易 TTC」計數（你要求）
    # ============================================================

    # ====== 先準備 E2E trajectory ======
    v_traj_e2e = _interp_to_control_n(v)
    a_traj_e2e = _interp_to_control_n(a)
    j_traj_e2e = np.zeros_like(v_traj_e2e)
    if len(j_traj_e2e) > 1:
      j_traj_e2e[1:] = np.diff(a_traj_e2e) / max(self.dt, 1e-3)
      j_traj_e2e[0] = j_traj_e2e[1]

    a_e2e_action, should_stop_e2e = _safe_model_action(sm)

    # ====== ACM（可選阻擋：低速純E2E時不介入）=====
    acm_enabled = bool(dp_flags & DPFlags.ACM) and not (self.e2e_only_active and E2E_ONLY_BLOCK_ACM)
    if acm_enabled:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)

    if self.e2e_only_active:
      # ==========================================================
      # 低速純E2E（方案5A）
      # - 不跑 mpc.update
      # - 輸出/軌跡都用 E2E
      # - FCW 用 TTC
      # - 加上：TTC + 最小距離 5m 的「底線煞車補丁」
      # ==========================================================
      self.v_desired_trajectory = v_traj_e2e
      self.a_desired_trajectory = a_traj_e2e
      self.j_desired_trajectory = j_traj_e2e

      # 先用 E2E action 當輸出
      output_a_target = float(a_e2e_action)
      self.output_should_stop = bool(should_stop_e2e)

      # TTC/距離安全補丁（只在低速純E2E啟用，避免干擾高速 MPC 體感）
      if E2E_TTC_GUARD_ENABLE:
        a_guard, should_stop_guard, ttc, d, closing = _ttc_brake_override(
          v_ego, sm['radarState'], output_a_target
        )
        output_a_target = float(a_guard)
        self.output_should_stop = bool(self.output_should_stop or should_stop_guard)

        # 低速 FCW（TTC 計數）
        if E2E_FCW_ENABLE and np.isfinite(ttc) and (ttc < E2E_FCW_TTC_S):
          self.ttc_fcw_cnt = min(self.ttc_fcw_cnt + 1, E2E_FCW_COUNT_TRIG + 5)
        else:
          self.ttc_fcw_cnt = max(self.ttc_fcw_cnt - E2E_FCW_DECAY, 0)

        self.fcw = self.ttc_fcw_cnt >= E2E_FCW_COUNT_TRIG
        if self.fcw:
          cloudlog.info("FCW triggered (E2E TTC)")
      else:
        self.fcw = False
        self.ttc_fcw_cnt = 0

      # 低速純E2E時，標記來源（避免外部誤判）
      try:
        self.mpc.source = 'e2e'
        self.mpc.solve_time = 0.0
      except Exception:
        pass

    else:
      # ==========================================================
      # 非低速純E2E：維持你原本（方法B + blended）行為，照跑 MPC
      # ==========================================================
      self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
      self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

      # 方法B：把 a_min/a_max 明確送進 MPC（ACC + blended 都會生效）
      a_min_mpc = float(accel_clip[0])
      a_max_mpc = float(accel_clip[1])

      use_dtsc = (dp_flags & DPFlags.DTSC) and not (self.e2e_only_active and E2E_ONLY_BLOCK_DTSC)
      if use_dtsc:
        a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(
          sm['modelV2'], v_ego, accel_clip[0], accel_clip[1]
        )
        a_min_mpc = np.maximum(a_min_mpc, np.asarray(a_min_dtsc, dtype=float))
        a_max_mpc = np.minimum(a_max_mpc, np.asarray(a_max_dtsc, dtype=float))

      # 注意：這裡假設你的 long_mpc.update 支援 a_min/a_max 參數（你前面方法B的配套）
      self.mpc.update(
        sm['radarState'], v_cruise, x, v, a, j,
        personality=sm['selfdriveState'].personality,
        a_min=a_min_mpc, a_max=a_max_mpc
      )

      v_traj_mpc = _interp_to_control_n(self.mpc.v_solution)
      a_traj_mpc = _interp_to_control_n(self.mpc.a_solution)
      j_traj_mpc = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

      # FCW：沿用 MPC crash_cnt
      self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
      if self.fcw:
        cloudlog.info("FCW triggered")

      self.v_desired_trajectory = v_traj_mpc
      self.a_desired_trajectory = a_traj_mpc
      self.j_desired_trajectory = j_traj_mpc

      # MPC action
      action_t = self.CP.longitudinalActuatorDelay + DT_MDL
      output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
        self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
        action_t=action_t, vEgoStopping=self.CP.vEgoStopping
      )

      if mode == 'acc':
        output_a_target = output_a_target_mpc
        self.output_should_stop = output_should_stop_mpc
      else:
        # blended：維持你原本 min(mpc, e2e_action)
        output_a_target = min(output_a_target_mpc, a_e2e_action)
        self.output_should_stop = bool(should_stop_e2e) or output_should_stop_mpc

      # 非低速純E2E：TTC FCW counter 清掉，避免狀態殘留
      self.ttc_fcw_cnt = 0

    # ACM 套用（對 trajectory 做修飾）
    if acm_enabled:
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)

    # ====== 狀態推進（用 trajectory 的加速度）=====
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    # === accel_clip slew rate：動態放寬（低速/前車強減速更快）===
    max_delta = _accel_clip_slew_step(self.dt, v_ego, lead_a)
    for idx in range(2):
      accel_clip[idx] = np.clip(
        accel_clip[idx],
        self.prev_accel_clip[idx] - max_delta,
        self.prev_accel_clip[idx] + max_delta
      )

    # 最終輸出保護：clip + slew（就算 TTC 補丁介入，也會被 A_CRUISE_MAX 壓住）
    self.output_a_target = float(np.clip(output_a_target, accel_clip[0], accel_clip[1]))
    self.prev_accel_clip = accel_clip

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
