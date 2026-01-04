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


# ====================== 功能開關（由此檔案控制） ======================
# 只要改這裡即可（True/False）
ACM_FUNCTION  = True   # Adaptive Coasting Module
AEM_FUNCTION  = True   # AEM mode switch / state machine
DTSC_FUNCTION = True   # Dynamic Turn Speed Control (curve constraint)
# ====================================================================

# 提示（調參方向）：
# - 低速跟停還想更敏感：可把 FAST_V_DESIRED_BLEND 往上加到 0.70~0.80（但要小心 yo-yo）
# - 若覺得偶爾有點「急」：先把 ACCEL_CLIP_SLEW_UP_FAST_MPS2_PER_S 往下調（例如 1.0）
# - 若你主要想改善「前車一煞立刻反應」：把 FAST_V_DESIRED_LEAD_DECEL_THRESH 從 -0.6 改 -0.5
"""
  • 仍會起步衝一下：SNG_A_POS_CAP 先降到 0.25~0.30、或 SNG_GUARD_TIME 加到 1.2
  • 覺得太龜、跟不緊：SNG_A_POS_CAP 提到 0.40~0.45，或 SNG_GUARD_TIME 降到 0.8
  • 前車一煞仍慢半拍：把 ACCEL_CLIP_SLEW_DN_FAST_MPS2_PER_S 往上加（例如 5.0），或把 SNG_LEAD_BRAKE_A 從 -0.4 → -0.3
"""

# ====================== 可調參數區（TUNING PARAMS） ======================

# --- v_desired_filter 反應加速（減少體感慢半拍） ---
# 注意：yo-yo 場景中，「低速就一直加速反應」反而可能讓起步太衝
# 這裡建議把觸發條件收斂到「前車明顯減速」或「你正在追近」時才啟用快反應
FAST_V_DESIRED_ENABLE = True
FAST_V_DESIRED_LOW_SPEED_KPH = 25.0            # 低於此速「具備啟用資格」（km/h）
FAST_V_DESIRED_LEAD_DECEL_THRESH = -0.6        # leadOne.aLeadK 小於此值視為前車在明顯減速（m/s^2）
FAST_V_DESIRED_CLOSING_VREL = -0.5             # leadOne.vRel 小於此值（開始追近）可啟用（m/s）
FAST_V_DESIRED_NEAR_DREL = 18.0                # leadOne.dRel 近於此距離才啟用（m）
FAST_V_DESIRED_BLEND = 0.65                    # 0~1，越大越快貼近 v_ego（建議 0.55~0.80）

# --- accel_clip slew rate（原本固定 0.05/step 太慢） ---
# 改成「每秒可變化多少 m/s^2」，再乘上 dt → 每 step 的允許變化量
# 重點：非對稱 slew（煞車放寬更快、加速放寬更慢）能顯著改善 SNG yo-yo
ACCEL_CLIP_FAST_LOW_SPEED_KPH = 35.0           # 低於此速可進入 fast 判斷（km/h）
ACCEL_CLIP_FAST_LEAD_DECEL_THRESH = -0.8       # 前車減速強於此值可進入 fast 判斷（m/s^2）

# 一般情況：加速放寬慢、煞車放寬快（避免起步衝）
ACCEL_CLIP_SLEW_UP_NORMAL_MPS2_PER_S = 0.8     # a_max 變大（更敢加速）的速度（m/s^2 per s）
ACCEL_CLIP_SLEW_DN_NORMAL_MPS2_PER_S = 2.5     # a_min 變小（更敢煞）或 a_max 變小（更保守）的速度

# fast 情況（低速或前車強減速）：讓煞車放寬更快；加速仍保守但略放寬
ACCEL_CLIP_SLEW_UP_FAST_MPS2_PER_S = 1.2
ACCEL_CLIP_SLEW_DN_FAST_MPS2_PER_S = 4.0

# --- Throttle gating（保留你原本設定） ---
ALLOW_THROTTLE_THRESHOLD = 0.4
MIN_ALLOW_THROTTLE_SPEED = 2.5

# ====================== SNG yo-yo 防抖（核心推薦） ======================
# 目標：塞車走走停停時，前車「剛起步又馬上再煞停」避免你先衝出去再急煞
SNG_GUARD_ENABLE = True

# 僅在低速/近距離啟用，避免影響一般跟車與高速行駛
SNG_GUARD_VEGO_MAX = 8.0            # m/s ≈ 29 km/h
SNG_GUARD_DREL_MAX = 18.0           # m

# 前車剛開始動（或你剛從 standstill 起步）的觀察窗（短暫保守）
SNG_GUARD_TIME = 1.0                # s
SNG_LEAD_START_V = 1.0              # m/s（判斷前車開始動）

# 觀察窗內：正向加速度上限（避免「起步衝一下」）
SNG_A_POS_CAP = 0.35                # m/s^2（想更保守→0.25；想更跟得上→0.45）

# 觀察窗或低速近距離下，如果偵測到前車又在煞/或你正在追近，直接禁止正加速
SNG_LEAD_BRAKE_A = -0.4             # m/s^2（前車開始煞的門檻）
SNG_VREL_CLOSING = -0.5             # m/s（vRel 變負＝你在追近）

# ====================== MPC accel limit 寫入（配合 long_mpc USE_CALLER_ACCEL_LIMITS） ======================
# 目的：每回合把本回合 accel_clip（含 SNG guard / throttle gating / turn limit）寫入 MPC params
# 好處：MPC 的可行域會一致反映 planner 的上/下限，SNG guard 的上限也會「真的」影響 MPC 解
# 注意：不同 fork 的 long_mpc 可能沒有 params 或型別不同，因此這裡做相容寫法避免 process crash
WRITE_ACCEL_LIMITS_TO_MPC = True
MPC_PARAMS_WARN_INTERVAL_S = 5.0     # 若寫入失敗，警告節流（秒）
# =======================================================================

LON_MPC_STEP = 0.2  # first step is 0.2s

A_CRUISE_MAX_VALS = [1.0, 1.085, 1.176, 0.805, 0.644, 0.441, 0.245, 0.198]
A_CRUISE_MAX_BP   = [0.0,  8.33,  11.0,  15.0,  20.0,  25.0,  30.0,  36.11]

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
  return np.interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)


def get_coast_accel(pitch):
  return np.sin(pitch) * -5.65 - 0.3  # fitted from data using xx/projects/allow_throttle/compute_coast_accel.py


def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """
  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = np.interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


def _get_lead_info(sm):
  """
  取得 leadOne 的資訊（若不可用則回傳預設值）
  用在：
  - yo-yo 防抖（SNG guard）
  - fast_response（只在真正需要時加速反應）
  - accel_clip slew 的 fast 判斷
  """
  try:
    l = sm['radarState'].leadOne
    if l.status:
      return True, float(l.dRel), float(l.vLead), float(l.vRel), float(l.aLeadK)
  except Exception:
    pass
  return False, 0.0, 0.0, 0.0, 0.0


def _should_fast_response(v_ego: float, lead_ok: bool, dRel: float, vRel: float, aLead: float) -> bool:
  """
  決定是否啟用「更快反應」（只在“真的需要”時）
  - yo-yo 場景：前車突然煞、或你正在追近（vRel<0）才需要快
  - 避免低速全部情境都快反應，導致起步太衝
  """
  if not FAST_V_DESIRED_ENABLE:
    return False

  v_kph = v_ego * CV.MS_TO_KPH
  if v_kph >= FAST_V_DESIRED_LOW_SPEED_KPH:
    # 高於低速門檻：僅在明顯煞車才加速反應（避免中速也過敏）
    return bool(lead_ok and (aLead < FAST_V_DESIRED_LEAD_DECEL_THRESH))

  # 低速：要更精準觸發
  if not lead_ok:
    return False

  near = dRel <= FAST_V_DESIRED_NEAR_DREL
  lead_braking = aLead < FAST_V_DESIRED_LEAD_DECEL_THRESH
  closing = vRel < FAST_V_DESIRED_CLOSING_VREL
  return bool(near and (lead_braking or closing))


def _accel_clip_slew_steps(dt: float, v_ego: float, lead_a: float):
  """
  回傳 accel_clip 每 step 的最大允許變化量（m/s^2）
  非對稱：
    - max_up：放寬加速（a_max 變大）速度較慢（避免起步衝）
    - max_dn：放寬煞車/收斂更保守（a_min 變小 或 a_max 變小）速度較快（避免慢半拍）
  """
  v_kph = v_ego * CV.MS_TO_KPH
  fast = (v_kph < ACCEL_CLIP_FAST_LOW_SPEED_KPH) or (lead_a < ACCEL_CLIP_FAST_LEAD_DECEL_THRESH)

  up_per_s = ACCEL_CLIP_SLEW_UP_FAST_MPS2_PER_S if fast else ACCEL_CLIP_SLEW_UP_NORMAL_MPS2_PER_S
  dn_per_s = ACCEL_CLIP_SLEW_DN_FAST_MPS2_PER_S if fast else ACCEL_CLIP_SLEW_DN_NORMAL_MPS2_PER_S

  max_up = float(max(0.01, up_per_s * dt))
  max_dn = float(max(0.01, dn_per_s * dt))
  return max_up, max_dn


def _try_write_mpc_accel_limits(mpc: LongitudinalMpc, accel_clip, warn_cb=None) -> bool:
  """
  相容寫法：把 accel_clip[0/1] 寫進 mpc.params 的 a_min/a_max
  - 支援 numpy array：mpc.params[:,0/1]
  - 支援 list-of-array：for i: mpc.params[i][0/1]
  - 若 mpc 沒有 params 或不可寫，回傳 False（不丟例外，避免 process crash）
  """
  if not hasattr(mpc, "params"):
    if warn_cb is not None:
      warn_cb("LongitudinalPlanner: mpc has no params; skip writing accel limits")
    return False

  try:
    # numpy array 版本
    mpc.params[:, 0] = float(accel_clip[0])  # a_min
    mpc.params[:, 1] = float(accel_clip[1])  # a_max
    return True
  except Exception:
    pass

  try:
    # list-of-array 版本
    for i in range(len(mpc.params)):
      mpc.params[i][0] = float(accel_clip[0])
      mpc.params[i][1] = float(accel_clip[1])
    return True
  except Exception:
    if warn_cb is not None:
      warn_cb("LongitudinalPlanner: mpc.params not writable; skip writing accel limits")
    return False


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(dt=dt)
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

    # SNG yo-yo 防抖狀態
    self.sng_guard_t = 999.0
    self.prev_standstill = False
    self.prev_lead_v = 0.0

    # mpc.params 寫入失敗警告節流
    self._last_mpc_params_warn_t = -1e9

    # 依照本檔案開關決定是否建立模組（避免多餘運算/依賴外部 flags）
    self.acm = ACM() if ACM_FUNCTION else None
    self.aem = AEM() if AEM_FUNCTION else None
    self.dtsc = DTSC(aggressiveness=0.8) if DTSC_FUNCTION else None

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
    # ====== 外部 dp_flags 不再控制，改由本檔案最上方布林值決定 ======
    dp_flags = 0
    if ACM_FUNCTION:
      dp_flags |= DPFlags.ACM
    if AEM_FUNCTION:
      dp_flags |= DPFlags.AEM
    if DTSC_FUNCTION:
      dp_flags |= DPFlags.DTSC
    # ============================================================

    mode = 'blended' if sm['selfdriveState'].experimentalMode else 'acc'

    if (dp_flags & DPFlags.AEM) and (self.aem is not None):
      self.aem.update_states(model_msg=sm['modelV2'], radar_msg=sm['radarState'], v_ego=sm['carState'].vEgo)
      mode = self.aem.get_mode(mode)

    if len(sm['carControl'].orientationNED) == 3:
      accel_coast = get_coast_accel(sm['carControl'].orientationNED[1])
    else:
      accel_coast = ACCEL_MAX

    v_ego = sm['carState'].vEgo
    standstill = bool(sm['carState'].standstill)

    # 讀 leadOne（yo-yo、防抖、slew、fast_response 都會用到）
    lead_ok, lead_d, lead_v, lead_vrel, lead_a = _get_lead_info(sm)

    # ===== SNG 起步防抖（核心）=====
    # 觸發條件：你剛從 standstill 起步 OR 前車剛開始動
    lead_started = lead_ok and (self.prev_lead_v < SNG_LEAD_START_V) and (lead_v >= SNG_LEAD_START_V)
    ego_started = (self.prev_standstill and (not standstill))

    if (SNG_GUARD_ENABLE and (ego_started or lead_started) and
        (v_ego <= SNG_GUARD_VEGO_MAX) and (lead_ok and lead_d <= SNG_GUARD_DREL_MAX)):
      self.sng_guard_t = 0.0
    else:
      self.sng_guard_t = min(self.sng_guard_t + self.dt, 999.0)

    in_sng_guard = (SNG_GUARD_ENABLE and (self.sng_guard_t < SNG_GUARD_TIME) and
                    (v_ego <= SNG_GUARD_VEGO_MAX) and (lead_ok and lead_d <= SNG_GUARD_DREL_MAX))

    # ===== 快速反應（只在需要時）=====
    fast_response = _should_fast_response(v_ego, lead_ok, lead_d, lead_vrel, lead_a)

    v_cruise_kph = min(sm['carState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS
    v_cruise_initialized = sm['carState'].vCruise != V_CRUISE_UNSET

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
    reset_state = reset_state or not v_cruise_initialized

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or standstill)

    # 依 mode 決定 accel_clip（本回合的 a_min/a_max）
    if mode == 'acc':
      accel_clip = [ACCEL_MIN, get_max_accel(v_ego)]
      steer_angle_without_offset = sm['carState'].steeringAngleDeg - sm['liveParameters'].angleOffsetDeg
      accel_clip = limit_accel_in_turns(v_ego, steer_angle_without_offset, accel_clip, self.CP)
    else:
      accel_clip = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      self.a_desired = np.clip(sm['carState'].aEgo, accel_clip[0], accel_clip[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))

    # === 快速反應：縮短體感延遲（僅在「前車煞/追近」時更快貼近）===
    if fast_response:
      self.v_desired_filter.x = (1.0 - FAST_V_DESIRED_BLEND) * self.v_desired_filter.x + FAST_V_DESIRED_BLEND * v_ego

    x, v, a, j, throttle_prob = self.parse_model(sm['modelV2'])

    # Don't clip at low speeds since throttle_prob doesn't account for creep
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

    # ====================== yo-yo 防抖：提前收緊 accel_clip（讓 MPC 也吃到這個上限） ======================
    # 1) 觀察窗內：限制正向加速度上限（避免起步衝一下）
    # 2) 若前車又在煞/你正在追近：直接禁止正加速（避免先衝再急煞）
    if in_sng_guard:
      accel_clip[1] = min(accel_clip[1], SNG_A_POS_CAP)

    if (lead_ok and (v_ego <= SNG_GUARD_VEGO_MAX) and (lead_d <= SNG_GUARD_DREL_MAX) and
        ((lead_a < SNG_LEAD_BRAKE_A) or (lead_vrel < SNG_VREL_CLOSING))):
      accel_clip[1] = min(accel_clip[1], 0.0)
    # =================================================================================================

    self.mpc.set_weights(prev_accel_constraint, personality=sm['selfdriveState'].personality)
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)

    # ====================== 關鍵修改：每回合把 accel_clip 寫進 MPC params（整段 horizon） ======================
    # 你之前版本只有 DTSC 時才寫；這裡改成「每回合都寫」
    # - 讓 long_mpc 的 USE_CALLER_ACCEL_LIMITS=True 能完整生效
    # - 讓 SNG guard / throttle gating / turn limit 的上限下限一致地影響 MPC 可行域
    if WRITE_ACCEL_LIMITS_TO_MPC:
      now_t = 1e-9 * sm.logMonoTime['modelV2']
      def _warn_once(msg: str):
        if (now_t - self._last_mpc_params_warn_t) > MPC_PARAMS_WARN_INTERVAL_S:
          self._last_mpc_params_warn_t = now_t
          cloudlog.warning(msg)

      _try_write_mpc_accel_limits(self.mpc, accel_clip, warn_cb=_warn_once)
    # ===================================================================================================

    # Apply DTSC curve speed constraints if enabled
    # 注意：DTSC 是逐點（horizon）限制，會在這裡覆蓋/收緊剛剛寫入的 accel_clip
    if (dp_flags & DPFlags.DTSC) and (self.dtsc is not None):
      a_min_dtsc, a_max_dtsc = self.dtsc.get_mpc_constraints(sm['modelV2'], v_ego, accel_clip[0], accel_clip[1])
      # 優先使用 accel_clip（含 SNG guard），再套 DTSC 逐點收緊
      try:
        for i in range(len(a_min_dtsc)):
          self.mpc.params[i, 0] = max(float(accel_clip[0]), float(a_min_dtsc[i]))   # a_min
          self.mpc.params[i, 1] = min(float(accel_clip[1]), float(a_max_dtsc[i]))   # a_max
      except Exception:
        # 若 mpc.params 不是 ndarray，回退到 list-of-array 寫法（避免 crash）
        try:
          for i in range(len(a_min_dtsc)):
            self.mpc.params[i][0] = max(float(accel_clip[0]), float(a_min_dtsc[i]))
            self.mpc.params[i][1] = min(float(accel_clip[1]), float(a_max_dtsc[i]))
        except Exception:
          # 再失敗就算了，不要把 process 弄掛
          pass

    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, personality=sm['selfdriveState'].personality)

    self.v_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC, self.mpc.a_solution)
    self.j_desired_trajectory = np.interp(CONTROL_N_T_IDX, T_IDXS_MPC[:-1], self.mpc.j_solution)

    # ACM - Adaptive Coasting Module
    if (dp_flags & DPFlags.ACM) and (self.acm is not None):
      user_control = long_control_off if self.CP.openpilotLongitudinalControl else not sm['selfdriveState'].enabled
      self.acm.update_states(sm['carControl'], sm['radarState'], user_control, v_ego, v_cruise)
      self.a_desired_trajectory = self.acm.update_a_desired_trajectory(self.a_desired_trajectory)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(np.interp(self.dt, CONTROL_N_T_IDX, self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

    action_t = self.CP.longitudinalActuatorDelay + DT_MDL
    output_a_target_mpc, output_should_stop_mpc = get_accel_from_plan(
      self.v_desired_trajectory,
      self.a_desired_trajectory,
      CONTROL_N_T_IDX,
      action_t=action_t,
      vEgoStopping=self.CP.vEgoStopping
    )
    output_a_target_e2e = sm['modelV2'].action.desiredAcceleration
    output_should_stop_e2e = sm['modelV2'].action.shouldStop

    if mode == 'acc':
      output_a_target = output_a_target_mpc
      self.output_should_stop = output_should_stop_mpc
    else:
      # blended：取更保守（較小）的加速度要求，並保留 stop 判斷
      output_a_target = min(output_a_target_mpc, output_a_target_e2e)
      self.output_should_stop = output_should_stop_e2e or output_should_stop_mpc

    # ====================== yo-yo 防抖：輸出端限制（最關鍵） ======================
    # 1) 觀察窗內：限制正向加速度上限（避免起步衝一下）
    # 2) 如果偵測前車又在煞/你正在追近：直接禁止正加速（避免慢半拍又補救）
    if in_sng_guard:
      output_a_target = min(output_a_target, SNG_A_POS_CAP)

    if (lead_ok and (v_ego <= SNG_GUARD_VEGO_MAX) and (lead_d <= SNG_GUARD_DREL_MAX) and
        ((lead_a < SNG_LEAD_BRAKE_A) or (lead_vrel < SNG_VREL_CLOSING))):
      output_a_target = min(output_a_target, 0.0)

    # ====================== accel_clip slew rate（非對稱） ======================
    # 讓「煞車放寬 / 收緊上限」更快，避免慢半拍
    # 讓「加速放寬」更慢，避免起步衝
    max_up, max_dn = _accel_clip_slew_steps(self.dt, v_ego, lead_a)
    for idx in range(2):
      accel_clip[idx] = np.clip(
        accel_clip[idx],
        self.prev_accel_clip[idx] - max_dn,
        self.prev_accel_clip[idx] + max_up
      )

    self.output_a_target = np.clip(output_a_target, accel_clip[0], accel_clip[1])
    self.prev_accel_clip = accel_clip

    # 更新狀態（供下次判斷 lead 起步 / ego 起步）
    self.prev_standstill = standstill
    self.prev_lead_v = lead_v if lead_ok else 0.0

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
