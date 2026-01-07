import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.common.swaglog import cloudlog

# ==============================
# DTSC 可調參數（都在這裡）
# ==============================

COMFORT_LAT_G = 0.20          # 舒適側向加速度上限（g）
SAFETY_FACTOR = 0.65          # 越小越保守（你目前很保守 OK）0.75

AGGR_MIN = 0.5
AGGR_MAX = 1.5

CURV_V_MIN = 2.0              # 曲率計算最小速度（m/s）
MIN_CURVATURE = 0.0015        # 小於此曲率視為直線（避免雜訊觸發）

EXCESS_SPEED_MPS = 0.4        # 超速門檻（m/s），降低一點更容易觸發煞車 0.6
MIN_CURVE_DISTANCE = 4.0      # 太近不動作 8.0
BRAKE_PREBUFFER_M = 18.0      # 提前鋪陳距離（越大越早開始）

SAFE_SPEED_BOOST_MPS = 0.15   # ✅ 改小：避免把目標速度放太寬導致不煞

MAX_COMFORT_DECEL = -1.6      # ✅ 稍微加強一點（你原本 -1.4 可能太柔）
DECEL_ENABLE_THRESHOLD = -0.12  # ✅ 讓較小的 req_decel 也會啟動（避免不動作）

CURV_LPF_ALPHA = 0.35         # 曲率 horizon 低通濾波
LOG_EVERY_N = 20              # log 節流

# ✅ 新增：觸發後最小入彎減速（避免 ramp 初期完全不煞）
MIN_ENTRY_DECEL = -0.25       # m/s^2（太大會「一觸發就煞」，太小會沒感覺）-0.35

# ✅ 新增：確保 ramp 至少有一段時間，不要過短導致突然變很負
RAMP_MIN_S = 0.8              # ramp 最短秒數
RAMP_MAX_S = 3.0              # ramp 最長秒數（避免拉太久來不及煞）

# ==============================
# 物理常數
# ==============================
BASE_LAT_ACC = COMFORT_LAT_G * 9.81  # m/s^2


class DTSC:
  """
  Dynamic Turn Speed Controller
  修正版重點：
  - 用二次方程估算 t_crit（在 req_decel 下到達 critical_dist 的時間）
  - ramp 會保證在 t_crit 前拉到 full req_decel（避免幾乎不煞）
  - 加 MIN_ENTRY_DECEL，觸發後至少先給一點點煞車感
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

  def get_mpc_constraints(self, model_msg, v_ego, base_a_min, base_a_max):
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

    is_curve = curv_s > MIN_CURVATURE
    excess = v_pred - safe_v

    # 找「最早超速點」
    candidates = np.where((excess > EXCESS_SPEED_MPS) & is_curve & (pos_x > MIN_CURVE_DISTANCE))[0]
    if candidates.size == 0:
      self._deactivate()
      return a_min, a_max

    critical_idx = int(candidates[0])
    critical_dist = float(pos_x[critical_idx])
    target_v = float(safe_v[critical_idx])

    # 提前開始減速距離
    start_dist = max(MIN_CURVE_DISTANCE, critical_dist - BRAKE_PREBUFFER_M)

    # --- 計算所需等加速度 decel ---
    v0 = float(max(v_ego, 0.0))
    req_decel = (target_v * target_v - v0 * v0) / (2.0 * max(critical_dist, 1.0))
    req_decel = min(req_decel, 0.0)                 # 只允許減速
    req_decel = max(req_decel, MAX_COMFORT_DECEL)   # 舒適上限
    if req_decel > DECEL_ENABLE_THRESHOLD:
      self._deactivate()
      return a_min, a_max

    # --- 用二次方程估算到 start / critical 的時間 ---
    # s(t) = v0*t + 0.5*a*t^2
    # 0.5*a*t^2 + v0*t - s = 0
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
      # 取正根且較小的那個（物理上到達距離的時間）
      t1 = (-B + sqrt_disc) / (2.0 * A)
      t2 = (-B - sqrt_disc) / (2.0 * A)
      ts = [t for t in (t1, t2) if t > 0.0]
      return float(min(ts)) if len(ts) else (s / max(v0, 1e-3))

    t_start = solve_time_to_distance(start_dist, 0.0)      # 起始點先用線性估（穩）
    t_crit  = solve_time_to_distance(critical_dist, req_decel)  # ✅ 用 req_decel 估比較準

    # 若時間排序怪怪的，做保護
    if t_crit <= t_start + 0.2:
      t_crit = t_start + 0.6

    # ramp 時間：保證在 t_crit 前拉到 full req_decel
    ramp_time = float(np.clip(t_crit - t_start, RAMP_MIN_S, RAMP_MAX_S))

    # --- 套用 a_max 約束（關鍵修正：確保會煞） ---
    for i, t in enumerate(T_IDXS_MPC):
      if t <= t_start:
        continue
      if t > t_crit:
        break

      # p: 0->1，保證在 t_crit 時達到 1
      p = np.clip((t - t_start) / max(ramp_time, 1e-3), 0.0, 1.0)

      # ✅ 先給最小入彎 decel，再逐步拉到 req_decel
      # p=0 時 a_cap ≈ MIN_ENTRY_DECEL（而不是 0）
      a_cap = (1.0 - p) * MIN_ENTRY_DECEL + p * req_decel

      # 只會限制「最大加速度」到更負（更會煞）
      a_max[i] = min(a_max[i], a_cap)

    # 狀態/訊息
    self.active = True
    self.debug_msg = (f"Curve {critical_dist:.0f}m, target {target_v*3.6:.0f}km/h, "
                      f"req {req_decel:.2f}, t_start {t_start:.2f}, t_crit {t_crit:.2f}")
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
