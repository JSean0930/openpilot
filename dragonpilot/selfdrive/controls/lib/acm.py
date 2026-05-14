import time
import numpy as np
from openpilot.common.swaglog import cloudlog

# ==========================================================
# ✅ 可調參數集中區（全速域滑行管理）
# ==========================================================

# ----------------------------------------------------
# [高速宏觀] 軌跡抑制滑行參數 (Macro Coasting)
# ----------------------------------------------------
MIN_ACM_SPEED_KPH = 40.0   
HIGH_SPEED_KPH    = 90.0   

SPEED_RATIO_ON       = 0.985  
SPEED_RATIO_ON_HIGH  = 0.970  
SPEED_RATIO_OFF      = 0.965  

TTC_ON  = 3.2                
TTC_OFF = 3.8                

EMERGENCY_TTC = 2.0               
EMERGENCY_REL_SPEED = 10.0        
EMERGENCY_DECEL_THRESHOLD = -1.5  

LEAD_COOLDOWN_TIME = 0.5  
MIN_ACTIVE_TIME = 1.0     

SPEED_BP = [0., 10., 20., 30.]     
MIN_DIST_V = [15., 20., 25., 30.]  
MIN_DIST_FACTOR = 1.0              

REL_SPEED_ALPHA = 0.2    

SUPPRESS_SPEED_BP = [0., 11.1, 22.2]  
SUPPRESS_A_MIN_V  = [-1.0, -0.6, -0.3]

SUPPRESS_SCALE_AT_A_MIN = 0.30  
SUPPRESS_SCALE_AT_ZERO  = 0.00  
SUPPRESS_CURVE_POWER = 1.6

MAX_COAST_TIME = 10.0      
COAST_REST_TIME = 1.5      

LOW_SPEED_SNG = 8.0        
SNG_A_MIN_CAP = -0.6       

# ----------------------------------------------------
# [低速微觀] 防震盪訊號熨斗參數 (Micro Signal Iron)
# ----------------------------------------------------
IRON_DIST_MAX = 6.0        # 距離 > 6.0m 開始準備啟動熨斗
IRON_DIST_RANGE = 4.0      # 距離緩衝區長度
IRON_CLOSE_MAX = 2.0       # 速差 < 2.0m/s 開始準備啟動熨斗
IRON_CLOSE_RANGE = 1.5     # 速差緩衝區長度
IRON_LEAD_A_REJECT = 0.6   # 前車加速度 > 0.6 m/s² 瞬間退回給 MPC (防追尾)
IRON_ZONE = 0.45           # 熨斗拋物線吸附區間 (相差 +-0.45 內生效)


# ==========================================================
# ACM 控制器 (全速域滑行大師)
# ==========================================================
class ACM:
  def __init__(self):
    self.enabled = False

    # 宏觀滑行變數
    self._has_lead = False
    self._in_cooldown = False
    self._active_prev = False
    self._active_start_time = 0.0
    self._last_lead_time = 0.0
    self.active = False
    self.just_disabled = False
    self._closing_speed_f = 0.0
    self._coast_start_time = 0.0
    self._rest_until_time = 0.0
    self.lead_ttc = float('inf')

    # 微觀熨斗變數
    self.smooth_coast_weight = 0.0 

  def _get_closing_ttc(self, lead, v_ego):
    if not lead or not lead.status:
      return float('inf'), 0.0
    closing_speed = max(v_ego - lead.vLead, 0.0)
    self._closing_speed_f = (1.0 - REL_SPEED_ALPHA) * self._closing_speed_f + REL_SPEED_ALPHA * closing_speed
    ttc = lead.dRel / max(self._closing_speed_f, 0.1)
    return ttc, self._closing_speed_f

  def _check_emergency_conditions(self, lead, v_ego, current_time):
    if not lead or not lead.status:
      return False
    ttc, closing_speed_f = self._get_closing_ttc(lead, v_ego)
    min_dist_for_speed = np.interp(v_ego, SPEED_BP, MIN_DIST_V) * MIN_DIST_FACTOR
    if lead.dRel < min_dist_for_speed and (ttc < EMERGENCY_TTC or closing_speed_f > EMERGENCY_REL_SPEED):
      self._last_lead_time = current_time
      return True
    return False

  def _update_lead_status(self, lead, v_ego, current_time):
    if lead and lead.status:
      ttc, _ = self._get_closing_ttc(lead, v_ego)
      self.lead_ttc = ttc
      if not self._has_lead and ttc < TTC_ON:
        self._has_lead = True
        self._last_lead_time = current_time
      elif self._has_lead and ttc > TTC_OFF:
        self._has_lead = False
    else:
      self._has_lead = False
      self.lead_ttc = float('inf')

  def _check_cooldown(self, current_time):
    return (current_time - self._last_lead_time) < LEAD_COOLDOWN_TIME

  def _should_activate(self, user_ctrl_lon, v_ego, v_cruise, current_time):
    if user_ctrl_lon or self._in_cooldown: return False
    v_kph = v_ego * 3.6
    if v_kph < MIN_ACM_SPEED_KPH: return False
    ratio_on = SPEED_RATIO_ON_HIGH if v_kph > HIGH_SPEED_KPH else SPEED_RATIO_ON
    if v_ego <= (v_cruise * ratio_on): return False
    if self._has_lead or current_time < self._rest_until_time: return False
    return True

  def _should_deactivate(self, v_ego, v_cruise, current_time):
    if self.active and (current_time - self._active_start_time) < MIN_ACTIVE_TIME: return False
    if v_ego < (v_cruise * SPEED_RATIO_OFF) or self._has_lead: return True
    return False

  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise):
    if not self.enabled or len(cc.orientationNED) != 3:
      self.active = False
      return
    current_time = time.monotonic()
    lead = rs.leadOne
    if self._check_emergency_conditions(lead, v_ego, current_time):
      self.active = False
      self._active_prev = self.active
      return
    self._update_lead_status(lead, v_ego, current_time)
    self._in_cooldown = self._check_cooldown(current_time)
    
    if not self.active:
      if self._should_activate(user_ctrl_lon, v_ego, v_cruise, current_time):
        self.active = True
        self._active_start_time = current_time
        self._coast_start_time = current_time
    else:
      if self._should_deactivate(v_ego, v_cruise, current_time):
        self.active = False
    self.just_disabled = self._active_prev and not self.active
    self._active_prev = self.active

  # --- 1. 高速宏觀滑行 (修改整段軌跡) ---
  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=None):
    if not self.active: return a_desired_trajectory
    current_time = time.monotonic()
    min_accel = float(np.min(a_desired_trajectory))
    if min_accel < EMERGENCY_DECEL_THRESHOLD:
      self.active = False
      return a_desired_trajectory
    if (current_time - self._coast_start_time) > MAX_COAST_TIME:
      self._rest_until_time = current_time + COAST_REST_TIME
      self.active = False
      return a_desired_trajectory

    v_ego = float(v_ego) if v_ego is not None else 0.0
    a_min = np.interp(v_ego, SUPPRESS_SPEED_BP, SUPPRESS_A_MIN_V)
    if v_ego < LOW_SPEED_SNG: a_min = max(a_min, SNG_A_MIN_CAP)

    modified = np.copy(a_desired_trajectory)
    for i in range(len(modified)):
      a = modified[i]
      if a_min < a < 0.0:
        t = np.clip((a - a_min) / (0.0 - a_min), 0.0, 1.0)
        s = (1.0 - t**SUPPRESS_CURVE_POWER)
        scale = SUPPRESS_SCALE_AT_ZERO + (SUPPRESS_SCALE_AT_A_MIN - SUPPRESS_SCALE_AT_ZERO) * s
        modified[i] = a * scale
    return modified

  # --- 2. 低速微觀防震盪 (熨平單點目標) ---
  def apply_signal_iron(self, base_a_target, accel_coast, has_lead, d_rel, closing, lead_a):
    """
    訊號熨斗：在跟車狀態下，消除微小的 MPC 震盪 (騎馬感)，無縫吸附至自然滑行。
    """
    if has_lead:
      w_dist = float(np.clip((d_rel - IRON_DIST_MAX) / IRON_DIST_RANGE, 0.0, 1.0))
      w_close = float(np.clip((IRON_CLOSE_MAX - abs(closing)) / IRON_CLOSE_RANGE, 0.0, 1.0))
      raw_coast_weight = min(w_dist, w_close)

      # 否決權：前車急加/減速
      if abs(lead_a) > IRON_LEAD_A_REJECT:
        raw_coast_weight = 0.0

      # EMA 平滑權重
      if raw_coast_weight > self.smooth_coast_weight:
        self.smooth_coast_weight += 0.02 * (raw_coast_weight - self.smooth_coast_weight)
      else:
        self.smooth_coast_weight += 0.20 * (raw_coast_weight - self.smooth_coast_weight)

      # 啟動拋物線引力場
      if self.smooth_coast_weight > 0.01:
        natural_coast = float(np.clip(accel_coast * 0.60 - 0.02, -0.25, 0.1))
        diff = base_a_target - natural_coast
        
        if abs(diff) < IRON_ZONE:
          ratio = (abs(diff) / IRON_ZONE) ** 2
          smoothed_a = natural_coast + diff * ratio
        else:
          smoothed_a = base_a_target

        return (1.0 - self.smooth_coast_weight) * base_a_target + self.smooth_coast_weight * smoothed_a

    else:
      self.smooth_coast_weight *= 0.6

    return base_a_target

