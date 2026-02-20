import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.common.swaglog import cloudlog

# ==============================
# DTSC 可調參數（都在這裡）
# ==============================

COMFORT_LAT_G = 0.27          # 舒適側向加速度上限（g）0.20
SAFETY_FACTOR = 1.00          # 越小越保守（你目前很保守 OK）

AGGR_MIN = 0.5
AGGR_MAX = 1.5

CURV_V_MIN = 5.0              # 曲率計算最小速度（m/s）2.0
MIN_CURVATURE = 0.0015        # 彎道判定（主要用於「確定是彎」）
ENTRY_CURVATURE = 0.0011      # 入彎入口判定（更早、更敏感）建議 < MIN_CURVATURE 0.0009

EXCESS_SPEED_MPS = 0.8        # 超速門檻（m/s）
MIN_CURVE_DISTANCE = 8.0      # 太近不動作
BRAKE_PREBUFFER_M = 18.0      # 提前鋪陳距離（相對 critical）

SAFE_SPEED_BOOST_MPS = 0.50   # 避免 safe_v 太貼造成抖動（越大越不容易煞）0.15
MAX_COMFORT_DECEL = -1.0      # 舒適減速度上限（越負=更強煞）
DECEL_ENABLE_THRESHOLD = -0.12  # req_decel 沒超過這個（不夠負）就不啟動

CURV_LPF_ALPHA = 0.20         # 曲率 horizon 低通濾波 0.35
LOG_EVERY_N = 20              # log 節流

MIN_ENTRY_DECEL = -0.15       # 觸發後最小入彎減速（讓「一開始就有緩煞」）-0.25
RAMP_MIN_S = 0.8              # ramp 最短秒數
RAMP_MAX_S = 1.5              # ramp 最長秒數

# ✅ 需求2：減速幅度最多就是「設定車速的 90%」
DTSC_MIN_TARGET_RATIO = 0.90

# ============================================================
# ✅ 新增：入彎提前距離「速度相依」功能 + 一鍵開關
# ============================================================

ENABLE_SPEED_DEP_ENTRY_LEAD = True  # ✅ 開關：True=速度相依，False=固定距離

# 關閉速度相依時使用的固定提前距離（m）
ENTRY_BRAKE_LEAD_M_CONST = 12.0

# 速度相依插值表（m/s 與 m）
# 直覺：車速越高 -> 提前越多，讓你「轉向前就開始緩減速」，更人性
# 你可依體感調整：高速想更早就把後段 meters 拉高
ENTRY_BRAKE_LEAD_V_BP = [0.0, 8.3, 16.7, 25.0, 33.3]   # 0, 30, 60, 90, 120 km/h
ENTRY_BRAKE_LEAD_M_BP = [6.0, 12.0, 18.0, 26.0, 35.0]  # 對應提前距離（m）[6.0, 10.0, 14.0, 18.0, 22.0]

# 安全夾制（避免表格被改壞）
ENTRY_BRAKE_LEAD_M_MIN = 4.0
ENTRY_BRAKE_LEAD_M_MAX = 40.0 #30.0

# ==============================
# 物理常數
# ==============================
BASE_LAT_ACC = COMFORT_LAT_G * 9.81  # m/s^2


def get_entry_brake_lead_m(v_ego_mps: float) -> float:
  """
  入彎提前距離（m）
  - 若 ENABLE_SPEED_DEP_ENTRY_LEAD=True：依 v_ego 做插值
  - 否則使用固定 ENTRY_BRAKE_LEAD_M_CONST
  """
  if not ENABLE_SPEED_DEP_ENTRY_LEAD:
    return float(ENTRY_BRAKE_LEAD_M_CONST)

  v = float(max(v_ego_mps, 0.0))
  lead = float(np.interp(v, ENTRY_BRAKE_LEAD_V_BP, ENTRY_BRAKE_LEAD_M_BP))
  return float(np.clip(lead, ENTRY_BRAKE_LEAD_M_MIN, ENTRY_BRAKE_LEAD_M_MAX))


class DTSC:
  """
  Dynamic Turn Speed Controller（提前入彎緩減速 + 目標速度下限 + 速度相依提前距離）
  """

  def __init__(self, aggressiveness=1.0):
    self.aggressiveness = float(np.clip(aggressiveness, AGGR_MIN, AGGR_MAX))
    self.active = False
    self.debug_msg = ""
    self._log_cnt = 0
    cloudlog.info(f"DTSC: Initialized with aggressiveness {self.aggressiveness:.2f}")

  def set_aggressiveness(self, value):
    self.aggressiveness = float(np.clip(value, AGGR_MIN, AGGR_MAX))
    cloudlog.info(f"DTSC: Aggressiveness updated to {self.aggressiveness:.2f}")

  # v_set_mps：可選，傳入你「設定的車速」（例如 v_cruise），才能更精準套 90% 規則
  def get_mpc_constraints(self, model_msg, v_ego, base_a_min, base_a_max, v_set_mps=None):
    a_min = np.ones(len(T_IDXS_MPC)) * base_a_min
    a_max = np.ones(len(T_IDXS_MPC)) * base_a_max

    if not self._is_model_data_valid(model_msg):
      self._deactivate()
      return a_min, a_max

    # --- 對齊 MPC horizon 的預測 ---
    v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
    yaw_rate = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)
    pos_x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)

    # --- 曲率 k = |yaw_rate| / v ---
    v_for_curv = np.clip(v_pred, CURV_V_MIN, 100.0)
    curv = np.abs(yaw_rate) / v_for_curv

    # --- 曲率 horizon 低通（抑制尖峰） ---
    curv_s = np.empty_like(curv)
    curv_s[0] = curv[0]
    alpha = float(np.clip(CURV_LPF_ALPHA, 0.05, 0.95))
    for i in range(1, len(curv)):
      curv_s[i] = alpha * curv[i] + (1.0 - alpha) * curv_s[i - 1]

    # --- 安全速度：v_max = sqrt(a_lat/k)*safety ---
    lat_acc_limit = BASE_LAT_ACC * self.aggressiveness
    safe_v = np.sqrt(lat_acc_limit / (curv_s + 1e-6)) * SAFETY_FACTOR
    safe_v = safe_v + SAFE_SPEED_BOOST_MPS

    v0 = float(max(v_ego, 0.0))
    v_set = float(v0 if v_set_mps is None else max(v_set_mps, 0.0))
    min_target_v = v_set * float(np.clip(DTSC_MIN_TARGET_RATIO, 0.5, 1.0))

    # ✅ 入彎入口：更早偵測「開始轉向前」曲率抬升
    entry_candidates = np.where((curv_s > ENTRY_CURVATURE) & (pos_x > MIN_CURVE_DISTANCE))[0]
    if entry_candidates.size == 0:
      self._deactivate()
      return a_min, a_max
    entry_idx = int(entry_candidates[0])
    entry_dist = float(pos_x[entry_idx])

    # ✅ 真正彎內（較嚴格）判斷，避免雜訊
    curve_candidates = np.where((curv_s > MIN_CURVATURE) & (pos_x > MIN_CURVE_DISTANCE))[0]
    if curve_candidates.size == 0:
      self._deactivate()
      return a_min, a_max

    # overspeed：確定有「需要煞」的理由
    excess = v_pred - safe_v
    overspeed_idxs = np.where((excess > EXCESS_SPEED_MPS) & (curv_s > MIN_CURVATURE) & (pos_x > MIN_CURVE_DISTANCE))[0]
    if overspeed_idxs.size == 0:
      self._deactivate()
      return a_min, a_max

    # ✅ critical：優先取彎內最嚴格點（safe_v 最小），避免拖到彎頂才反應
    tight_idx = int(curve_candidates[np.argmin(safe_v[curve_candidates])])
    critical_idx = tight_idx if excess[tight_idx] > EXCESS_SPEED_MPS else int(overspeed_idxs[0])

    critical_dist = float(pos_x[critical_idx])
    target_v = float(safe_v[critical_idx])

    # ✅ 需求2：目標速度不得低於「設定車速的 90%」
    target_v = max(target_v, min_target_v)

    # 抬高後若其實不用煞，就不啟動
    if target_v >= v0 - 0.05:
      self._deactivate()
      return a_min, a_max

    # ✅ 速度相依的入彎提前距離
    entry_lead_m = get_entry_brake_lead_m(v0)

    # ✅ 提前開始減速距離：
    # - 入口前 entry_lead_m 開始鋪陳（確保轉向前有緩煞）
    # - 同時保留 critical 前 BRAKE_PREBUFFER_M 的鋪陳（兩者取更早開始）
    start_dist_entry = max(MIN_CURVE_DISTANCE, entry_dist - entry_lead_m)
    start_dist_crit  = max(MIN_CURVE_DISTANCE, critical_dist - BRAKE_PREBUFFER_M)
    start_dist = min(start_dist_entry, start_dist_crit)

    # --- 計算所需等加速度 decel（讓 v0 在 critical_dist 附近降到 target_v）---
    req_decel = (target_v * target_v - v0 * v0) / (2.0 * max(critical_dist, 5.0))
    req_decel = min(req_decel, 0.0)
    req_decel = max(req_decel, MAX_COMFORT_DECEL)

    if req_decel > DECEL_ENABLE_THRESHOLD:
      self._deactivate()
      return a_min, a_max

    # --- 用二次方程估算到 start / critical 的時間 ---
    def solve_time_to_distance(s, a):
      s = float(max(s, 0.0))
      a = float(a)
      if v0 < 0.1:
        return 0.0
      if abs(a) < 1e-3:
        return s / max(v0, 1e-3)

      A = 0.5 * a
      B = v0
      C = -s
      disc = B * B - 4.0 * A * C
      if disc <= 0.0:
        return s / max(v0, 1e-3)

      sqrt_disc = float(np.sqrt(disc))
      t1 = (-B + sqrt_disc) / (2.0 * A)
      t2 = (-B - sqrt_disc) / (2.0 * A)
      ts = [t for t in (t1, t2) if t > 0.0]
      return float(min(ts)) if len(ts) else (s / max(v0, 1e-3))

    t_start = solve_time_to_distance(start_dist, 0.0)
    t_crit  = solve_time_to_distance(critical_dist, req_decel)

    if t_crit <= t_start + 0.2:
      t_crit = t_start + 0.6

    ramp_time = float(np.clip(t_crit - t_start, RAMP_MIN_S, RAMP_MAX_S))

    # --- 套用 a_max 約束：入彎前先緩煞，並在 t_crit 前拉到 req_decel ---
    for i, t in enumerate(T_IDXS_MPC):
      if t <= t_start:
        continue
      if t > t_crit:
        break

      p = np.clip((t - t_start) / max(ramp_time, 1e-3), 0.0, 1.0)

      # p=0 時就至少有 MIN_ENTRY_DECEL（緩煞），然後逐步拉到 req_decel
      a_cap = (1.0 - p) * MIN_ENTRY_DECEL + p * req_decel
      a_max[i] = min(a_max[i], a_cap)

    self.active = True
    self.debug_msg = (
      f"Entry {entry_dist:.0f}m start {start_dist:.0f}m (lead {entry_lead_m:.1f}m, dep={ENABLE_SPEED_DEP_ENTRY_LEAD}), "
      f"crit {critical_dist:.0f}m, target {target_v*3.6:.0f}km/h (floor {min_target_v*3.6:.0f}), "
      f"req {req_decel:.2f}, t_start {t_start:.2f}, t_crit {t_crit:.2f}"
    )
    self._log_cnt += 1
    if (self._log_cnt % LOG_EVERY_N) == 0:
      cloudlog.info(f"DTSC: {self.debug_msg} (aggr={self.aggressiveness:.2f})")

    return a_min, a_max

  def _is_model_data_valid(self, model_msg):
    return (len(model_msg.position.x) == ModelConstants.IDX_N and
            len(model_msg.velocity.x) == ModelConstants.IDX_N and
            len(model_msg.orientationRate.z) == ModelConstants.IDX_N)

  def _deactivate(self):
    self.active = False
    self.debug_msg = ""
