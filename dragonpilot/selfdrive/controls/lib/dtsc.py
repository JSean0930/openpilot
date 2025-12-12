import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.common.swaglog import cloudlog

# ==============================
# DTSC 可調參數（都在這裡）
# ==============================

# 舒適側向加速度上限（g），越大 = 允許更快過彎
COMFORT_LAT_G = 0.20

# 安全係數（<1 會更保守），越小 = 彎道目標速度越低
SAFETY_FACTOR = 0.92

# aggressiveness 允許範圍（外部 set_aggressiveness 也會夾在這裡）
AGGR_MIN = 0.5
AGGR_MAX = 1.5

# 曲率計算用的最小速度（避免低速除法爆掉）
CURV_V_MIN = 2.0  # m/s

# 忽略曲率雜訊的門檻（太小視為直線）
MIN_CURVATURE = 0.0015  # ~1/666m

# 判定「真的超速」的門檻（避免 0.1m/s 這種抖動一直啟動）
EXCESS_SPEED_MPS = 0.8  # m/s  (~3 km/h)

# 最小反應距離（太近就不管，交給其他控制）
MIN_CURVE_DISTANCE = 8.0  # m

# 額外提前開始減速的緩衝距離（越大=越早開始鋪陳）
BRAKE_PREBUFFER_M = 18.0  # m

# 目標速度「放寬」一些，避免過度保守（越大=越不煞、越人性化）
SAFE_SPEED_BOOST_MPS = 0.6  # m/s (~2 km/h)

# decel 上限（越接近 0 越柔；負值越大絕對值越激烈）
MAX_COMFORT_DECEL = -1.4  # m/s^2

# 若算出來需要的 decel 太小（接近 0），就不要啟動（避免一直小修小補）
DECEL_ENABLE_THRESHOLD = -0.20  # m/s^2

# 對曲率做 horizon 低通濾波：alpha 越小越平滑（越不容易過度煞）
CURV_LPF_ALPHA = 0.35

# decel 約束的漸進 ramp 時間（越大=越慢拉進來、更像人）
RAMP_TIME_S = 2.2

# log 節流（避免每幀刷 log）
LOG_EVERY_N = 20

# ==============================
# 物理常數
# ==============================
BASE_LAT_ACC = COMFORT_LAT_G * 9.81  # m/s^2


class DTSC:
  """
  Dynamic Turn Speed Controller
  - 以模型預測曲率估算「舒適安全速度」
  - 找到「最早超速點」並提前鋪陳減速
  - 用 ramp 漸進施加 a_max 限制，避免突然大煞車
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
    """
    Returns:
      a_min, a_max: arrays aligned with T_IDXS_MPC
    """
    a_min = np.ones(len(T_IDXS_MPC)) * base_a_min
    a_max = np.ones(len(T_IDXS_MPC)) * base_a_max

    if not self._is_model_data_valid(model_msg):
      self._deactivate()
      return a_min, a_max

    # 取 MPC horizon 對齊的預測
    v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
    yaw_rate = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)
    pos_x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)

    # 曲率 k = |yaw_rate| / v
    v_for_curv = np.clip(v_pred, CURV_V_MIN, 100.0)
    curv = np.abs(yaw_rate) / v_for_curv

    # 對曲率做 horizon 低通，抑制尖峰（避免過度煞車）
    curv_s = np.empty_like(curv)
    curv_s[0] = curv[0]
    alpha = float(np.clip(CURV_LPF_ALPHA, 0.05, 0.95))
    for i in range(1, len(curv)):
      curv_s[i] = alpha * curv[i] + (1.0 - alpha) * curv_s[i - 1]

    # 計算安全速度：v_max = sqrt(a_lat / k) * safety
    lat_acc_limit = BASE_LAT_ACC * self.aggressiveness
    safe_v = np.sqrt(lat_acc_limit / (curv_s + 1e-6)) * SAFETY_FACTOR

    # 放寬一點，避免太保守（更人性化）
    safe_v = safe_v + SAFE_SPEED_BOOST_MPS

    # 忽略直線/雜訊：曲率太小就視為不需要 DTSC
    is_curve = curv_s > MIN_CURVATURE

    # 超速量
    excess = v_pred - safe_v

    # 找「最早超速點」：更早開始處理，而不是等到最大超速
    candidates = np.where((excess > EXCESS_SPEED_MPS) & is_curve & (pos_x > MIN_CURVE_DISTANCE))[0]
    if candidates.size == 0:
      self._deactivate()
      return a_min, a_max

    critical_idx = int(candidates[0])
    critical_dist = float(pos_x[critical_idx])
    target_v = float(safe_v[critical_idx])

    # 提前開始減速的距離（鋪陳用）
    start_dist = max(MIN_CURVE_DISTANCE, critical_dist - BRAKE_PREBUFFER_M)

    # 用能量公式估算需要的等加速度 decel
    v0 = float(max(v_ego, 0.0))
    req_decel = (target_v * target_v - v0 * v0) / (2.0 * max(critical_dist, 1.0))

    # 只允許減速（<=0）
    req_decel = min(req_decel, 0.0)

    # 限制成「舒適 decel」範圍（避免太煞）
    req_decel = max(req_decel, MAX_COMFORT_DECEL)

    # 太小就不啟動，避免一直微調
    if req_decel > DECEL_ENABLE_THRESHOLD:
      self._deactivate()
      return a_min, a_max

    # ==========================
    # 漸進式施加 a_max 限制（更像人）
    # - 在 start_dist 之前不施加
    # - 進入後用 ramp_time 逐步把 a_max 拉到 req_decel
    # ==========================
    v_for_time = max(v0, 1.0)
    t_start = start_dist / v_for_time
    ramp_time = max(RAMP_TIME_S, 0.5)

    for i, t in enumerate(T_IDXS_MPC):
      if t <= t_start:
        continue

      # ramp 進度：0 -> 1
      p = np.clip((t - t_start) / ramp_time, 0.0, 1.0)
      a_cap = p * req_decel  # 逐步變得更負（更強的減速上限）

      # 只在「尚未到 critical_dist 的階段」施加限制
      # 用目前速度近似估距（不做二次項，避免因模型誤差導致太保守）
      dist_at_t = v0 * t
      if dist_at_t < critical_dist:
        a_max[i] = min(a_max[i], a_cap)

    # 狀態/訊息
    self.active = True
    self.debug_msg = f"Curve {critical_dist:.0f}m, target {target_v*3.6:.0f}km/h, a_cap {req_decel:.2f}"
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
