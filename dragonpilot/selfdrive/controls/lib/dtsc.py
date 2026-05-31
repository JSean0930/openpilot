import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.common.swaglog import cloudlog

# ==============================
# DTSC 可調參數（都在這裡）
# ==============================

COMFORT_LAT_G = 0.27          # 舒適側向加速度上限（g）
SAFETY_FACTOR = 1.00          # 越小越保守

AGGR_MIN = 0.5
AGGR_MAX = 1.5

CURV_V_MIN = 5.0              # 曲率計算最小速度（m/s）
MIN_CURVATURE = 0.0015        # 彎道判定
ENTRY_CURVATURE = 0.0011      # 入彎入口判定

EXCESS_SPEED_MPS = 0.8        # 超速門檻（m/s）
MIN_CURVE_DISTANCE = 8.0      # 太近不動作
BRAKE_PREBUFFER_M = 18.0      # 提前鋪陳距離

SAFE_SPEED_BOOST_MPS = 0.50   # 避免 safe_v 太貼造成抖動
MAX_COMFORT_DECEL = -1.0      # 舒適減速度上限
DECEL_ENABLE_THRESHOLD = -0.12

CURV_LPF_ALPHA = 0.20         # 曲率 horizon 低通濾波 (入彎時使用)
LOG_EVERY_N = 20              # log 節流

MIN_ENTRY_DECEL = -0.15       # 觸發後最小入彎減速
RAMP_MIN_S = 0.5              # ramp 最短秒數
RAMP_MAX_S = 2.0              # ramp 最長秒數

DTSC_MIN_TARGET_RATIO = 0.90  # 減速幅度最多就是「設定車速的 90%」

# ============================================================
# ✅ 入彎提前距離「速度相依」功能
# ============================================================
ENABLE_SPEED_DEP_ENTRY_LEAD = True  

ENTRY_BRAKE_LEAD_M_CONST = 12.0
ENTRY_BRAKE_LEAD_V_BP = [0.0, 8.3, 16.7, 25.0, 33.3]   # 0, 30, 60, 90, 120 km/h
ENTRY_BRAKE_LEAD_M_BP = [6.0, 12.0, 18.0, 26.0, 35.0]  # 對應提前距離（m）

ENTRY_BRAKE_LEAD_M_MIN = 4.0
ENTRY_BRAKE_LEAD_M_MAX = 40.0

# ============================================================
# ✅ 新增：前車動態複製開關 (Lead Car Proxy / 反射神經前饋)
# ============================================================
ENABLE_LEAD_PROXY = True      # 開啟前車領航代駕模式
LEAD_PROXY_MAX_DIST = 80.0    # 參考前車的最大有效距離(m)
LEAD_PROXY_SPEED_BUFFER = 1.0 # 容許比前車慢多少(m/s)，保留安全緩衝避免貼太死
LEAD_PROXY_BRAKE_RATIO = 0.8  # 複製前車煞車力道的比例（0.8代表八成力，避免過度神經質）

# ==============================
# 物理常數
# ==============================
BASE_LAT_ACC = COMFORT_LAT_G * 9.81  # m/s^2


def get_entry_brake_lead_m(v_ego_mps: float) -> float:
  if not ENABLE_SPEED_DEP_ENTRY_LEAD:
    return float(ENTRY_BRAKE_LEAD_M_CONST)

  v = float(max(v_ego_mps, 0.0))
  lead = float(np.interp(v, ENTRY_BRAKE_LEAD_V_BP, ENTRY_BRAKE_LEAD_M_BP))
  return float(np.clip(lead, ENTRY_BRAKE_LEAD_M_MIN, ENTRY_BRAKE_LEAD_M_MAX))


class DTSC:
  def __init__(self, aggressiveness=1.0):
    self.aggressiveness = float(np.clip(aggressiveness, AGGR_MIN, AGGR_MAX))
    self.active = False
    self.debug_msg = ""
    self._log_cnt = 0
    cloudlog.info(f"DTSC: Initialized with aggressiveness {self.aggressiveness:.2f}")

  def set_aggressiveness(self, value):
    self.aggressiveness = float(np.clip(value, AGGR_MIN, AGGR_MAX))
    cloudlog.info(f"DTSC: Aggressiveness updated to {self.aggressiveness:.2f}")

  # 💡 修改：加入 lead_msg 參數，用來接收前車雷達/視覺動態資訊
  def get_mpc_constraints(self, model_msg, v_ego, base_a_min, base_a_max, v_set_mps=None, lead_msg=None):
    a_min = np.ones(len(T_IDXS_MPC)) * base_a_min
    a_max = np.ones(len(T_IDXS_MPC)) * base_a_max

    if not self._is_model_data_valid(model_msg):
      self._deactivate()
      return a_min, a_max

    # --- 對齊 MPC horizon 的預測 ---
    v_pred = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x)
    yaw_rate = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.orientationRate.z)
    pos_x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x)

    v_for_curv = np.clip(v_pred, CURV_V_MIN, 100.0)
    curv = np.abs(yaw_rate) / v_for_curv

    # --- 非對稱曲率低通濾波 ---
    curv_s = np.empty_like(curv)
    curv_s[0] = curv[0]
    alpha_entry = float(np.clip(CURV_LPF_ALPHA, 0.05, 0.95))
    alpha_exit = 0.65  
    
    for i in range(1, len(curv)):
      if curv[i] < curv_s[i - 1]:
        curv_s[i] = alpha_exit * curv[i] + (1.0 - alpha_exit) * curv_s[i - 1]
      else:
        curv_s[i] = alpha_entry * curv[i] + (1.0 - alpha_entry) * curv_s[i - 1]

    lat_acc_limit = BASE_LAT_ACC * self.aggressiveness
    safe_v = np.sqrt(lat_acc_limit / (curv_s + 1e-6)) * SAFETY_FACTOR
    safe_v = safe_v + SAFE_SPEED_BOOST_MPS

    v0 = float(max(v_ego, 0.0))
    v_set = float(v0 if v_set_mps is None else max(v_set_mps, 0.0))
    min_target_v = v_set * float(np.clip(DTSC_MIN_TARGET_RATIO, 0.5, 1.0))

    entry_candidates = np.where((curv_s > ENTRY_CURVATURE) & (pos_x > MIN_CURVE_DISTANCE))[0]
    if entry_candidates.size == 0:
      self._deactivate()
      return a_min, a_max
    entry_idx = int(entry_candidates[0])
    entry_dist = float(pos_x[entry_idx])

    curve_candidates = np.where((curv_s > MIN_CURVATURE) & (pos_x > MIN_CURVE_DISTANCE))[0]
    if curve_candidates.size == 0:
      self._deactivate()
      return a_min, a_max

    excess = v_pred - safe_v
    overspeed_idxs = np.where((excess > EXCESS_SPEED_MPS) & (curv_s > MIN_CURVATURE) & (pos_x > MIN_CURVE_DISTANCE))[0]
    if overspeed_idxs.size == 0:
      self._deactivate()
      return a_min, a_max

    tight_idx = int(curve_candidates[np.argmin(safe_v[curve_candidates])])
    critical_idx = tight_idx if excess[tight_idx] > EXCESS_SPEED_MPS else int(overspeed_idxs[0])

    critical_dist = float(pos_x[critical_idx])
    target_v = float(safe_v[critical_idx])
    target_v = max(target_v, min_target_v)

    # ==========================================================
    # ✅ 擬人化前車動態複製 (Lead Car Proxy & Reflex Brake)
    # 概念：把前車當作「彎道探路員」，完美融合他的動態來繞過 MPC 延遲
    # ==========================================================
    lead_a_feedforward = 0.0
    
    # 確保前車存在且狀態有效
    if ENABLE_LEAD_PROXY and lead_msg is not None and getattr(lead_msg, 'status', False):
      lead_d = getattr(lead_msg, 'dRel', 100.0)
      
      # 條件：前車在有效的參考視距內 (預設 80m 內)
      if lead_d < LEAD_PROXY_MAX_DIST:
        lead_v = float(max(v0 + getattr(lead_msg, 'vRel', 0.0), 0.0))
        lead_a = float(getattr(lead_msg, 'aLeadK', 0.0))
        
        # 【1. 動態解封限速】：如果前車能用更高速度過彎，我們就相信他是安全的！
        # 解決「前車順順過，我卻被死板的曲率公式拖累」的問題，直接突破 safe_v 天花板。
        target_v = max(target_v, lead_v - LEAD_PROXY_SPEED_BUFFER)
        
        # 【2. 反射神經前饋】：如果前車在彎道前已經開始重踩煞車
        # 將 Planner 裡的反射神經概念引進，不等曲率發作，直接複製他的煞車力道。
        if lead_a < -0.3:
          lead_a_feedforward = lead_a * LEAD_PROXY_BRAKE_RATIO

    # 抬高後若其實不用煞，且沒有觸發前車反射煞車，就不啟動
    if target_v >= v0 - 0.05 and lead_a_feedforward >= -0.1:
      self._deactivate()
      return a_min, a_max

    entry_lead_m = get_entry_brake_lead_m(v0)
    start_dist_entry = max(MIN_CURVE_DISTANCE, entry_dist - entry_lead_m)
    start_dist_crit  = max(MIN_CURVE_DISTANCE, critical_dist - BRAKE_PREBUFFER_M)
    start_dist = min(start_dist_entry, start_dist_crit)

    # --- 計算所需等加速度 decel ---
    req_decel = (target_v * target_v - v0 * v0) / (2.0 * max(critical_dist, 5.0))
    
    # ✅ 將前車的「反射神經煞車」力道融合進來，取兩者最嚴格（最負）的，做到前車煞我秒煞
    req_decel = min(req_decel, lead_a_feedforward)
    
    req_decel = min(req_decel, 0.0)
    req_decel = max(req_decel, MAX_COMFORT_DECEL)

    if req_decel > DECEL_ENABLE_THRESHOLD:
      self._deactivate()
      return a_min, a_max

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
        if a < -1e-3:
          return -v0 / a
        else:
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

    t_total = t_crit - t_start
    ramp_time = float(np.clip(t_total * 0.6, RAMP_MIN_S, RAMP_MAX_S))
    t_peak = t_start + ramp_time
    t_crit = max(t_crit, t_peak + 0.2)
    peak_decel = max(req_decel * 1.25, MAX_COMFORT_DECEL)

    # --- 套用 a_max 約束 ---
    for i, t in enumerate(T_IDXS_MPC):
      if t <= t_start:
        continue
      
      if t > t_crit and curv_s[i] < MIN_CURVATURE:
        break
      if t > t_crit + 0.5:
        break

      if t <= t_peak:
        p = np.clip((t - t_start) / max(ramp_time, 1e-3), 0.0, 1.0)
        p_smooth = p * p * (3.0 - 2.0 * p) 
        a_cap = p_smooth * peak_decel
        
      elif t <= t_crit:
        p = np.clip((t - t_peak) / max(t_crit - t_peak, 1e-3), 0.0, 1.0)
        p_smooth = p * p * (3.0 - 2.0 * p)
        target_trail_decel = max(peak_decel, MIN_ENTRY_DECEL)
        a_cap = peak_decel + p_smooth * (target_trail_decel - peak_decel)
        
      else:
        p = np.clip((t - t_crit) / 0.4, 0.0, 1.0)
        target_trail_decel = max(peak_decel, MIN_ENTRY_DECEL)
        a_cap = target_trail_decel * (1.0 - p) + p * base_a_max

      a_max[i] = min(a_max[i], a_cap)

    self.active = True
    self.debug_msg = (
      f"Entry {entry_dist:.0f}m start {start_dist:.0f}m "
      f"crit {critical_dist:.0f}m, target {target_v*3.6:.0f}km/h "
      f"req {req_decel:.2f} (LeadBrake: {lead_a_feedforward:.2f}), peak {peak_decel:.2f}"
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

