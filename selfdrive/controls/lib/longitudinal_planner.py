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

# --------------------------------------------------------------------
# ✅ 你要的「2~3 個旋鈕」(更早介入版)
# --------------------------------------------------------------------
# EARLYNESS：越大越早介入（同時影響 fast_response + pre-brake 觸發門檻）
# STRENGTH ：最大預煞強度（越負越兇）
# SENS     ：觸發敏感度（越大越容易觸發、距離窗更大、a_req 門檻更寬鬆）
EARLYNESS = 1.45   # 建議 0.8 ~ 1.6（你要更早介入 => 往上加）,1.2
STRENGTH  = 1.15   # 建議 0.8 ~ 1.6（對應最大預煞強度）,1.20
SENS      = 1.45   # 建議 0.8 ~ 1.6（越大越敏感）,1.40

# --- [新增] 只在指定區間做「暴力平均」混合： (MPC + E2E) / 2 ---
# 注意：這裡完全不使用權重，也不改 long_mpc，只在 planner 最終輸出做平均。
MIX_AVG_ENABLE = True
MIX_AVG_MIN_KPH = 5.0
MIX_AVG_MAX_KPH = 60.0 #20.0

# --- v_desired_filter 反應加速（減少體感慢半拍） ---
# 低速 or 前車明顯減速時，將 v_desired_filter.x 更快貼近 v_ego
# ✅ 重要：本版已把 fast_response 的觸發「統一」成同一套 approach_trigger（相對速度/距離/TTC）
FAST_V_DESIRED_ENABLE = True
FAST_V_DESIRED_LOW_SPEED_KPH = 30.0            # 低於此速就更敏感（km/h）, 35.0
FAST_V_DESIRED_LEAD_DECEL_THRESH = -0.20        # leadOne.aLeadK 小於此值視為前車在明顯減速（m/s^2）/ -0.25
FAST_V_DESIRED_BLEND = 0.85                    # 0~1，越大越快貼近 v_ego（建議 0.5~0.8）/ 0.65

# --- accel_clip slew rate（原本固定 0.05/step 太慢） ---
# 改成「每秒可變化多少 m/s^2」，再乘上 dt → 每 step 的允許變化量
#
# [B方案] 重要：這裡會搭配「上下不對稱 slew」
# - 往更負（煞車方向）允許更快變化 -> 該煞就即時煞
# - 往更正（油門方向）維持較保守 -> 避免油門抖、暴衝
ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S = 1.4        # 原本 0.05@20Hz ≈ 1.0 m/s^2 per sec 1.0
ACCEL_CLIP_SLEW_FAST_MPS2_PER_S = 5.0          # 低速/前車強減速時放寬（建議 2.0~3.5）2.5 / 4.0
ACCEL_CLIP_FAST_LOW_SPEED_KPH = 40.0           # 低於此速可放寬（km/h）35
ACCEL_CLIP_FAST_LEAD_DECEL_THRESH = -0.25       # 前車減速強於此值可放寬（m/s^2）-0.20

# --- Throttle gating（保留你原本設定） ---
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# --------------------------------------------------------------------
# ✅ 新增：TTC / 相對速度 / 相對距離 的「人性化判斷」(由旋鈕推導)
# --------------------------------------------------------------------
# 目標：你提到的「高速遇到停等車，會慢煞 + 最後急煞」=> 提前預煞
# 核心依據：closing(相對速度) + dRel(距離) => TTC + 需求減速度 a_req
# 注意：這些都是底層參數，通常只要動上面 3 個旋鈕就好

# 最小距離硬底線（不可違反）
HARD_MIN_LEAD_DIST_M = 3.0 #6.0

# 只有 closing_speed > 這個才算真的在追近（避免抖動/誤觸）
CLOSING_MIN_MPS = 0.25

# 用於估算 a_req 的距離緩衝（避免 dRel 很小時 a_req 爆炸）
A_REQ_DIST_BUFFER_M = 2.0

# 「預煞」作用距離窗（只在這範圍內才介入，避免遠距離一直點煞）
# 距離窗會被 SENS 放大，也會被 EARLYNESS 稍微放大（更早介入）
PREBRAKE_DIST_MIN_M = 1.5 #6.0
PREBRAKE_DIST_MAX_M_BASE = 55.0  # base，實際會乘上旋鈕推導

# 只對「停等/慢車」更積極（避免你追近一台正常車速的車也被預煞）
LEAD_SLOW_MPS_BASE = 9.0  # 約 32km/h，實際會被 EARLYNESS/SENS 推導

# TTC 門檻（越早介入 => TTC_START 更大）
TTC_START_BASE = 2.2
TTC_FULL_BASE  = 1.2

# a_req 門檻（越早介入 => 更不需要那麼大的 a_req 就觸發）
A_REQ_START_BASE = -1.2
A_REQ_FULL_BASE  = -2.6

# 預煞最大減速度（由 STRENGTH 推導，越大越兇）
PREBRAKE_MAX_DECEL_BASE = -1.6  # base（會乘 STRENGTH）


# =======================================================================

LON_MPC_STEP = 0.2  # first step is 0.2s

# A_CRUISE_MAX：速度 -> 最大允許加速度（m/s^2）
# 注意：本版已修改為「ACC + blended 都套用」(方法B配套)
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


def _lead_metrics(v_ego: float, radarstate):
  """
  由 lead 計算「相對距離/相對速度/TTC/需求減速度 a_req」
  - closing = v_ego - v_lead
  - TTC = d_rel / closing（只在 closing > CLOSING_MIN_MPS 時計算）
  - a_req：用能量法估算「要在剩餘距離內把 closing 消掉」的需求減速度（負值）
          a_req ≈ -(closing^2) / (2*(d_rel - buffer))
  """
  lead = _pick_closest_lead(radarstate)
  if lead is None:
    return None, float('inf'), v_ego, 0.0, float('inf'), 0.0

  d_rel = float(getattr(lead, 'dRel', 1e9))
  v_lead = float(getattr(lead, 'vLead', v_ego))
  closing = float(v_ego - v_lead)

  # TTC
  if (not np.isfinite(d_rel)) or d_rel <= 0.0:
    ttc = 0.0
  elif closing <= CLOSING_MIN_MPS:
    ttc = float('inf')
  else:
    ttc = d_rel / max(closing, 1e-3)

  # a_req（需求減速度，負值才需要煞車）
  d_eff = max(d_rel - A_REQ_DIST_BUFFER_M, 0.5)
  if closing <= CLOSING_MIN_MPS:
    a_req = 0.0
  else:
    a_req = - (closing * closing) / (2.0 * d_eff)

  return lead, d_rel, v_lead, closing, float(ttc), float(a_req)


def _derived_thresholds():
  """
  把 3 個旋鈕推導成底層門檻（集中管理）
  你要「更早介入」=> EARLYNESS 往上加
  """
  # TTC 門檻：EARLYNESS 越大 => START/FULL 越大（更早開始、滿介入也更早）
  ttc_start = TTC_START_BASE + 0.9 * (EARLYNESS - 1.0)
  ttc_full  = TTC_FULL_BASE  + 0.5 * (EARLYNESS - 1.0)
  ttc_start = float(np.clip(ttc_start, 1.6, 4.0))
  ttc_full  = float(np.clip(ttc_full, 0.8, ttc_start - 0.2))

  # a_req 門檻：EARLYNESS/SENS 越大 => 門檻更接近 0（更早觸發）
  # 例：-1.2 -> -0.8（更早）
  a_req_start = A_REQ_START_BASE + 0.7 * (EARLYNESS - 1.0) + 0.5 * (SENS - 1.0)
  a_req_full  = A_REQ_FULL_BASE  + 0.5 * (EARLYNESS - 1.0) + 0.3 * (SENS - 1.0)
  a_req_start = float(np.clip(a_req_start, -2.0, -0.3))
  a_req_full  = float(np.clip(a_req_full,  -4.5, a_req_start - 0.3))

  # 距離窗：SENS/EARLYNESS 越大 => 視為更早需要預煞的距離更遠
  dist_max = PREBRAKE_DIST_MAX_M_BASE * (0.9 + 0.35 * (SENS - 1.0)) * (0.95 + 0.25 * (EARLYNESS - 1.0))
  dist_max = float(np.clip(dist_max, 35.0, 110.0))

  # 慢車門檻：EARLYNESS/SENS 越大 => 更常把「前方慢/停等」當作預煞目標
  lead_slow_mps = LEAD_SLOW_MPS_BASE * (0.95 + 0.25 * (SENS - 1.0)) * (0.95 + 0.15 * (EARLYNESS - 1.0))
  lead_slow_mps = float(np.clip(lead_slow_mps, 6.0, 14.0))

  # 最大預煞強度：STRENGTH 越大 => 更負（更兇）
  prebrake_max_decel = PREBRAKE_MAX_DECEL_BASE * STRENGTH
  prebrake_max_decel = float(np.clip(prebrake_max_decel, -3.5, -0.8))

  return ttc_start, ttc_full, a_req_start, a_req_full, dist_max, lead_slow_mps, prebrake_max_decel


def _approach_trigger(v_ego: float, radarstate):
  """
  ✅ 統一觸發：fast_response + pre-brake 共用同一組條件（相對速度/距離）
  回傳：
  - trigger: bool
  - metrics: (lead, d_rel, v_lead, closing, ttc, a_req)
  """
  ttc_start, ttc_full, a_req_start, a_req_full, dist_max, lead_slow_mps, prebrake_max_decel = _derived_thresholds()

  lead, d_rel, v_lead, closing, ttc, a_req = _lead_metrics(v_ego, radarstate)
  if lead is None:
    return False, (lead, d_rel, v_lead, closing, ttc, a_req)

  # 只在距離窗內才考慮（避免太遠就開始抖）
  in_dist_window = (np.isfinite(d_rel) and (PREBRAKE_DIST_MIN_M <= d_rel <= dist_max))

  # 必須真的在追近
  is_closing = closing > CLOSING_MIN_MPS

  # 偏向停等/慢車：v_lead 很慢 或 d_rel 很近
  slow_or_close = (v_lead <= lead_slow_mps) or (d_rel <= 18.0)

  # TTC 或 a_req 觸發任一成立 => 觸發
  ttc_trig = np.isfinite(ttc) and (ttc < ttc_start)
  req_trig = (a_req < a_req_start)

  trigger = bool(in_dist_window and is_closing and slow_or_close and (ttc_trig or req_trig))
  return trigger, (lead, d_rel, v_lead, closing, ttc, a_req)


def _prebrake_override(a_target: float, metrics):
  """
  依 approach_trigger 的同一套 metrics 來做「預煞」：提前把減速度往負推
  - 只做「往更負」方向（不會把你煞車變小）
  - TTC 越小 / a_req 越負 => 介入越大
  """
  ttc_start, ttc_full, a_req_start, a_req_full, dist_max, lead_slow_mps, prebrake_max_decel = _derived_thresholds()
  lead, d_rel, v_lead, closing, ttc, a_req = metrics

  a_new = float(a_target)

  # 硬底線：距離不可低於 5m（不可違反）
  if np.isfinite(d_rel) and d_rel < HARD_MIN_LEAD_DIST_M:
    return float(min(a_new, prebrake_max_decel)), True

  # 沒在追近就不介入
  if closing <= CLOSING_MIN_MPS:
    return a_new, False

  # 介入權重：TTC
  w_ttc = 0.0
  if np.isfinite(ttc) and ttc < ttc_start:
    w_ttc = (ttc_start - ttc) / max(ttc_start - ttc_full, 1e-3)
    w_ttc = float(np.clip(w_ttc, 0.0, 1.0))

  # 介入權重：a_req（需求減速度）
  w_req = 0.0
  if a_req < a_req_start:
    w_req = (a_req_start - a_req) / max(a_req_start - a_req_full, 1e-3)
    w_req = float(np.clip(w_req, 0.0, 1.0))

  w = float(max(w_ttc, w_req))
  if w <= 0.0:
    return a_new, False

  # 目標預煞減速度（越危險越接近 max_decel）
  a_brake = float(prebrake_max_decel)

  # 只往更負方向推
  a_cmd = (1.0 - w) * a_new + w * a_brake
  a_new = float(min(a_new, a_cmd))
  return a_new, False


def _accel_clip_slew_step(dt: float, v_ego: float, lead_a: float, trigger_approach: bool, ttc: float, a_req: float) -> tuple[float, float]:
  """
  [B方案 1)]
  回傳 (delta_down, delta_up)

  - delta_down：允許 accel_clip 更快往「更負」方向變化（煞車更即時）
  - delta_up  ：允許 accel_clip 往「更正」方向較慢變化（油門較穩、避免抖）

  fast 條件：維持你原本邏輯 + ✅加入相對指標
  - 低於 ACCEL_CLIP_FAST_LOW_SPEED_KPH -> fast
  - lead_a < ACCEL_CLIP_FAST_LEAD_DECEL_THRESH -> fast
  - approach_trigger / TTC 很小 / a_req 很負 -> fast
  """
  v_kph = v_ego * CV.MS_TO_KPH

  # 由旋鈕推導一個「fast」的 TTC/a_req 門檻（比 start 再更早一點）
  ttc_start, ttc_full, a_req_start, a_req_full, dist_max, lead_slow_mps, prebrake_max_decel = _derived_thresholds()
  ttc_fast = min(ttc_start + 0.4, 4.0)
  a_req_fast = min(a_req_start + 0.3, -0.2)

  fast_rel = bool(trigger_approach or (np.isfinite(ttc) and ttc < ttc_fast) or (a_req < a_req_fast))
  fast = (v_kph < ACCEL_CLIP_FAST_LOW_SPEED_KPH) or (lead_a < ACCEL_CLIP_FAST_LEAD_DECEL_THRESH) or fast_rel

  # 上行（加速方向）保持較保守：用 NORMAL
  slew_up_per_s = ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S

  # 下行（煞車方向）可更激進：fast 時用 FAST，否則 NORMAL
  slew_down_per_s = ACCEL_CLIP_SLEW_FAST_MPS2_PER_S if fast else ACCEL_CLIP_SLEW_NORMAL_MPS2_PER_S

  delta_up = float(max(0.01, slew_up_per_s * dt))      # 最低給 0.01 避免完全卡死
  delta_down = float(max(0.01, slew_down_per_s * dt))  # 最低給 0.01 避免完全卡死
  return delta_down, delta_up


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

    if dp_flags & DPFlags.AEM:
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo)
      mode = self.aem.get_mode(mode)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    lead_a = _get_lead_decel_a(sm)

    # ============================================================
    # ✅ 統一觸發：fast_response + pre-brake 共用同一組條件（相對速度/距離/TTC）
    # ============================================================
    trigger_approach, metrics = _approach_trigger(v_ego, sm['radarState'])
    _lead, _d_rel, _v_lead, _closing, _ttc, _a_req = metrics

    # 保留你原本 low_speed / lead_a 的 fast_response 結構，但現在主要由 trigger_approach 驅動
    fast_response = bool(FAST_V_DESIRED_ENABLE and (trigger_approach or (v_ego * CV.MS_TO_KPH < FAST_V_DESIRED_LOW_SPEED_KPH) or (lead_a < FAST_V_DESIRED_LEAD_DECEL_THRESH)))

    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    # ============================================================
    # 核心修改(1)：ACC + blended 都套用 A_CRUISE_MAX（你要求的 blended 套用）
    # - 原本 blended 用 ACCEL_MAX，現在改成 get_max_accel(v_ego)
    # - 轉彎加速限制：維持原本只在 ACC 套用（需要的話也可改成兩種模式都套用）
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

    # === 快速反應：縮短體感延遲（不改 FirstOrderFilter 結構，只做額外貼近）===
    # ✅ 現在 fast_response 會隨「追近停等車」更早觸發，避免慢煞 + 最後急煞
    if fast_response:
      self.v_desired_filter.x = (1.0 - FAST_V_DESIRED_BLEND) * self.v_desired_filter.x + FAST_V_DESIRED_BLEND * v_ego

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

    # MPC 設定
    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # ============================================================
    # 核心修改(2)：方法B（明確把 a_min/a_max 傳進 long_mpc）
    #
    # - 不再寫 self.mpc.params[...]（因為 long_mpc.update() 內會自己設定 params）
    # - a_max 預設使用 accel_clip[1]（已包含 A_CRUISE_MAX，且 throttle gating 也會反映）
    # - 若啟用 DTSC：用 per-stage 的曲率限制再疊加 accel_clip，並傳入 array (N+1)
    # ============================================================
    a_min_mpc = float(accel_clip[0])
    a_max_mpc = float(accel_clip[1])

    if dp_flags & DPFlags.DTSC:
      a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(
        sm['modelV2'], v_ego, accel_clip[0], accel_clip[1]
      )
      # 轉 numpy array，與 accel_clip 疊加
      a_min_mpc = np.maximum(a_min_mpc, np.asarray(a_min_dtsc, dtype=float))
      a_max_mpc = np.minimum(a_max_mpc, np.asarray(a_max_dtsc, dtype=float))

    # 方法B：把 a_min/a_max 明確送進 MPC（ACC + blended 都會生效）
    self.mpc.update(
      sm['radarState'], v_cruise, x, v, a, j,
      personality=sm['selfdriveState'].personality,
      a_min=a_min_mpc, a_max=a_max_mpc
    )

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)

    # ACM - Adaptive Coasting Module
    if dp_flags & DPFlags.ACM:
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)

    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate dt seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory, self.a_desired_trajectory, CONTROL_N_T_IDX,
      action_t=action_t, vEgoStopping=self.CP.vEgoStopping
    )

    # 讀 e2e action（保留你原本寫法）
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    # 原本模式輸出
    if mode == 'acc':
      output_a_target = float(output_a_target_mpc)
      self.output_should_stop = bool(output_should_stop_mpc)
    else:
      output_a_target = float(min(output_a_target_mpc, output_a_target_e2e))
      self.output_should_stop = bool(output_should_stop_e2e) or bool(output_should_stop_mpc)

    # ============================================================
    # [你要的修改] 只在  MIX_AVG_MIN_KPH <= v_kph < MIX_AVG_MAX_KPH 區間做「(MPC + E2E) / 2」平均混合
    # - 不使用權重
    # - 不改 long_mpc
    # - 不寫入 self.mpc.source（避免 source enum/capnp 崩潰造成 process not running）
    # - shouldStop：更保守（任一要求停就停）
    # ============================================================
    if MIX_AVG_ENABLE:
      v_kph = float(v_ego) * CV.MS_TO_KPH
      if MIX_AVG_MIN_KPH <= v_kph < MIX_AVG_MAX_KPH:
        output_a_target = 0.5 * (float(output_a_target_mpc) + float(output_a_target_e2e))
        self.output_should_stop = bool(output_should_stop_e2e) or bool(output_should_stop_mpc)

    # ============================================================
    # ✅ 新增：pre-brake（提前預煞）— 與 fast_response 共用同一套 approach_trigger
    # 目的：高速遇上停等車，不要慢煞到最後才急煞
    # ============================================================
    if trigger_approach:
      output_a_target, hard_stop = _prebrake_override(output_a_target, metrics)
      # 硬底線/極端危險 => shouldStop 更保守
      if hard_stop:
        self.output_should_stop = True

    # ============================================================
    # [B方案 2)]
    # === accel_clip slew rate：上下不對稱放寬 ===
    # - 往更負（煞車方向）允許更快變化 -> 更即時
    # - 往更正（油門方向）維持較保守 -> 更穩
    # ============================================================
    delta_down, delta_up = _accel_clip_slew_step(self.dt, v_ego, lead_a, trigger_approach, _ttc, _a_req)
    for idx in range(2):
      accel_clip[idx] = np.clip(
        accel_clip[idx],
        self.prev_accel_clip[idx] - delta_down,  # 往更負（更小）放寬
        self.prev_accel_clip[idx] + delta_up     # 往更正（更大）保守
      )

    # 最終輸出仍保留 accel_clip（雙保險：MPC內也有限制、輸出端也不會超）
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

    # ============================================================
    # [防爆修正] 你先前的 process not running，多半是這行炸掉：
    # longitudinalPlan.longitudinalPlanSource = self.mpc.source
    #
    # 原因：某些 fork 的 schema 把 longitudinalPlanSource 定義成 enum，
    #      若 self.mpc.source 不在 enum 內，capnp 會直接丟例外 -> process crash
    #
    # 解法：保留原本行為，但加 try/except，避免任何意外值讓進程掛掉。
    # ============================================================
    try:
      longitudinalPlan.longitudinalPlanSource = self.mpc.source
    except Exception:
      # fallback：用最常見且通常存在的來源字串
      # 若你的 schema 是 enum，請確認 'cruise' 在你的 enum 裡（大多數 fork 都有）
      longitudinalPlan.longitudinalPlanSource = 'cruise'

    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.aTarget = float(self.output_a_target)
    longitudinalPlan.shouldStop = bool(self.output_should_stop)
    longitudinalPlan.allowBrake = True
    longitudinalPlan.allowThrottle = bool(self.allow_throttle)

    pm.send('longitudinalPlan', plan_send)
