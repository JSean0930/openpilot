import time
import numpy as np
from openpilot.common.swaglog import cloudlog

# ==========================================================
# ✅ 可調參數集中區（調體感就改這裡）
# ==========================================================

# --- 速度區間限制 ---
MIN_ACM_SPEED_KPH = 40.0   # ✅ 新增：時速低於此值禁止 ACM 啟動
HIGH_SPEED_KPH    = 90.0   # ✅ 新增：時速高於此值視為高速區，放寬啟動條件

# --- 啟動/退出 遲滯（Hysteresis） ---
SPEED_RATIO_ON       = 0.985  # 一般情況啟動：v_ego > v_cruise * SPEED_RATIO_ON
SPEED_RATIO_ON_HIGH  = 0.970  # ✅ 高速時啟動使用的比例，數值較小 → 更容易達成條件
SPEED_RATIO_OFF      = 0.965  # 退出：v_ego < v_cruise * SPEED_RATIO_OFF

TTC_ON  = 3.2                # 一般 lead 進入門檻（closing TTC < TTC_ON  => 視為有 lead）
TTC_OFF = 3.8                # 一般 lead 退出門檻（closing TTC > TTC_OFF => 視為無 lead）

# --- 緊急 disable 門檻（更危險才動作） ---
EMERGENCY_TTC = 2.0               # closing TTC < 2s => 立刻停 ACM
EMERGENCY_REL_SPEED = 10.0        # closing speed > 10 m/s => 立刻停 ACM
EMERGENCY_DECEL_THRESHOLD = -1.5  # MPC 要求 a < -1.5 => 立刻停 ACM、且不抑制

# --- Lead 冷卻時間 + 最短維持時間 ---
LEAD_COOLDOWN_TIME = 0.5  # lead 剛消失後冷卻，避免反覆啟停
MIN_ACTIVE_TIME = 1.0     # ACM 一旦啟動，至少維持這麼久才允許退出

# --- 最小安全距離（速度插值） + 係數 ---
SPEED_BP = [0., 10., 20., 30.]     # m/s
MIN_DIST_V = [15., 20., 25., 30.]  # m
MIN_DIST_FACTOR = 1.0              # 全局倍率（想更保守 => 1.1~1.2）

# --- 相對速度濾波（避免 lead 噪聲） ---
REL_SPEED_ALPHA = 0.2    # EMA 濾波係數，越小越平滑（0.1~0.3）

# --- 抑制煞車的速度自適應區間（越快越保守） ---
# 速度（m/s）分段：每段對應可抑制的「最強輕煞門檻」(m/s^2)
SUPPRESS_SPEED_BP = [0., 11.1, 22.2]  # 0, 40, 80 km/h
SUPPRESS_A_MIN_V  = [-1.0, -0.6, -0.3]

# --- 漸進式抑制曲線 ---
# a 在 (A_MIN, 0) 之間時，不是歸零，而是縮放到 a * scale
SUPPRESS_SCALE_AT_A_MIN = 0.30  # 當 a = A_MIN 時，剩下 30%
SUPPRESS_SCALE_AT_ZERO  = 0.00  # 當 a -> 0 時，縮到 0（滑行）
# 曲線指數：>1 代表更「慢慢放掉煞車」，更人性化
SUPPRESS_CURVE_POWER = 1.6

# --- 最大滑行時間（避免長下坡拖太久） ---
MAX_COAST_TIME = 10.0      # active 連續超過此秒數會降級抑制
COAST_REST_TIME = 1.5      # 降級後的「休息時間」，讓 MPC 正常控一下

# --- 低速 stop & go 特化 ---
LOW_SPEED_SNG = 8.0        # m/s (~29 km/h) 以下才允許最強抑制
SNG_A_MIN_CAP = -0.6       # 低速時抑制上限再更集中（避免過頭滑）

# ==========================================================
# ACM 控制器
# ==========================================================
class ACM:
  def __init__(self):
    self.enabled = False

    self._has_lead = False
    self._in_cooldown = False

    self._active_prev = False
    self._active_start_time = 0.0
    self._last_lead_time = 0.0

    self.active = False
    self.just_disabled = False

    # 濾波用相對速度（closing speed）
    self._closing_speed_f = 0.0

    # 長滑行管理
    self._coast_start_time = 0.0
    self._rest_until_time = 0.0

    self.lead_ttc = float('inf')

  # --------- 工具：closing TTC（更貼近風險） ----------
  def _get_closing_ttc(self, lead, v_ego):
    if not lead or not lead.status:
      return float('inf'), 0.0

    # closing speed：我比前車快多少（<=0 表示沒有逼近風險）
    closing_speed = max(v_ego - lead.vLead, 0.0)

    # EMA 濾波，避免 lead.vLead 噪聲
    self._closing_speed_f = (1.0 - REL_SPEED_ALPHA) * self._closing_speed_f + REL_SPEED_ALPHA * closing_speed

    ttc = lead.dRel / max(self._closing_speed_f, 0.1)
    return ttc, self._closing_speed_f

  # --------- 緊急 disable（最高優先） ----------
  def _check_emergency_conditions(self, lead, v_ego, current_time):
    if not lead or not lead.status:
      return False

    ttc, closing_speed_f = self._get_closing_ttc(lead, v_ego)

    # 速度自適應最小距離（含全局倍率）
    min_dist_for_speed = np.interp(v_ego, SPEED_BP, MIN_DIST_V) * MIN_DIST_FACTOR

    # 危險條件：距離過近 +（closing TTC 過短 or closing speed 還在快速逼近）
    if lead.dRel < min_dist_for_speed and (
        ttc < EMERGENCY_TTC or
        closing_speed_f > EMERGENCY_REL_SPEED):

      self._last_lead_time = current_time
      if self.active:
        cloudlog.warning(
          f"ACM emergency disable: dRel={lead.dRel:.1f}m, "
          f"closingTTC={ttc:.2f}s, closingSpeed={closing_speed_f:.1f}m/s"
        )
      return True

    return False

  # --------- 一般 lead 狀態（含 TTC 遲滯） ----------
  def _update_lead_status(self, lead, v_ego, current_time):
    if lead and lead.status:
      ttc, _ = self._get_closing_ttc(lead, v_ego)
      self.lead_ttc = ttc

      # TTC 遲滯：避免 3 秒上下抖動
      if not self._has_lead and ttc < TTC_ON:
        self._has_lead = True
        self._last_lead_time = current_time
      elif self._has_lead and ttc > TTC_OFF:
        self._has_lead = False
      # 介於 TTC_ON/TTC_OFF 之間保持原狀

    else:
      self._has_lead = False
      self.lead_ttc = float('inf')

  def _check_cooldown(self, current_time):
    return (current_time - self._last_lead_time) < LEAD_COOLDOWN_TIME

  # --------- ACM 啟動條件（含速度遲滯＋低速/高速條件） ----------
  def _should_activate(self, user_ctrl_lon, v_ego, v_cruise, current_time):
    if user_ctrl_lon:
      return False

    # 冷卻中不啟動
    if self._in_cooldown:
      return False

    # 轉成 km/h 方便理解條件
    v_kph = v_ego * 3.6

    # ✅ 條件 1：低於 MIN_ACM_SPEED_KPH（例如 40km/h）禁止 ACM 啟動
    if v_kph < MIN_ACM_SPEED_KPH:
      return False

    # 根據速度決定啟動速度比例：
    # - 一般：SPEED_RATIO_ON
    # - 高速（> HIGH_SPEED_KPH）：用 SPEED_RATIO_ON_HIGH（較小，較容易啟動）
    ratio_on = SPEED_RATIO_ON
    if v_kph > HIGH_SPEED_KPH:
      ratio_on = SPEED_RATIO_ON_HIGH

    # 未達啟動速度門檻，不啟動
    if v_ego <= (v_cruise * ratio_on):
      return False

    # 有 lead 威脅，不啟動
    if self._has_lead:
      return False

    # 長滑行 rest（降級休息）中，不啟動
    if current_time < self._rest_until_time:
      return False

    return True

  # --------- ACM 退出條件（含最短維持時間） ----------
  def _should_deactivate(self, v_ego, v_cruise, current_time):
    # 最短維持時間內不退出
    if self.active and (current_time - self._active_start_time) < MIN_ACTIVE_TIME:
      return False

    # 速度掉到退出閾值以下 → 退出
    if v_ego < (v_cruise * SPEED_RATIO_OFF):
      return True

    # 有 lead 威脅 → 退出
    if self._has_lead:
      return True

    return False

  # --------- 每幀更新 ACM 狀態 ----------
  def update_states(self, cc, rs, user_ctrl_lon, v_ego, v_cruise):
    if not self.enabled or len(cc.orientationNED) != 3:
      self.active = False
      return

    current_time = time.monotonic()
    lead = rs.leadOne

    # 緊急狀況優先處理
    if self._check_emergency_conditions(lead, v_ego, current_time):
      self.active = False
      self._active_prev = self.active
      return

    # 更新一般 lead 威脅
    self._update_lead_status(lead, v_ego, current_time)

    # 冷卻期
    self._in_cooldown = self._check_cooldown(current_time)

    # 狀態機
    if not self.active:
      if self._should_activate(user_ctrl_lon, v_ego, v_cruise, current_time):
        self.active = True
        self._active_start_time = current_time
        self._coast_start_time = current_time
        cloudlog.info(
          f"ACM activated: v_ego={v_ego*3.6:.1f} km/h, "
          f"v_cruise={v_cruise*3.6:.1f} km/h"
        )
    else:
      if self._should_deactivate(v_ego, v_cruise, current_time):
        self.active = False
        cloudlog.info("ACM deactivated")

    # 記錄 disable 邊緣事件
    self.just_disabled = self._active_prev and not self.active
    self._active_prev = self.active

  # --------- 抑制軌跡（人性化漸進式滑行） ----------
  def update_a_desired_trajectory(self, a_desired_trajectory, v_ego=None):
    """
    人性化抑制策略：
    1) 若 MPC 要求明顯煞車（min_a < EMERGENCY_DECEL_THRESHOLD） => 立刻退出且不改
    2) 依速度調整可抑制區間 A_MIN（越快越保守）
    3) 在 (A_MIN, 0) 用連續曲線縮放，而非硬歸零
    4) 長時間滑行超時 => 降級抑制 + 進入 rest，避免下坡拖很久
    """
    if not self.active:
      return a_desired_trajectory

    current_time = time.monotonic()

    # 1) 安全檢查：MPC 要求大煞車 => 不干預
    min_accel = float(np.min(a_desired_trajectory))
    if min_accel < EMERGENCY_DECEL_THRESHOLD:
      cloudlog.warning(f"ACM aborting: MPC requested {min_accel:.2f} m/s² braking")
      self.active = False
      return a_desired_trajectory

    # 2) 長滑行超時降級
    if (current_time - self._coast_start_time) > MAX_COAST_TIME:
      # 進入 rest 一段時間，讓 MPC 正常控速
      self._rest_until_time = current_time + COAST_REST_TIME
      self.active = False
      cloudlog.info("ACM coast timeout -> rest")
      return a_desired_trajectory

    # 若外部沒傳 v_ego 就用 0 當作保守
    v_ego = float(v_ego) if v_ego is not None else 0.0

    # 3) 依速度決定 A_MIN（可抑制最強輕煞）
    a_min = np.interp(v_ego, SUPPRESS_SPEED_BP, SUPPRESS_A_MIN_V)

    # stop&go 低速再更集中（避免滑太大）
    if v_ego < LOW_SPEED_SNG:
      a_min = max(a_min, SNG_A_MIN_CAP)

    # 4) 漸進式曲線縮放
    modified = np.copy(a_desired_trajectory)
    for i in range(len(modified)):
      a = modified[i]

      if a_min < a < 0.0:
        # 將 a 映射到 [0,1]（0 對應 a_min，1 對應 0）
        t = (a - a_min) / (0.0 - a_min)
        t = np.clip(t, 0.0, 1.0)

        # 曲線：t^power 讓靠近 0 的地方更平滑
        s = (1.0 - t**SUPPRESS_CURVE_POWER)  # 0->1 的剩餘比例
        scale = SUPPRESS_SCALE_AT_ZERO + (SUPPRESS_SCALE_AT_A_MIN - SUPPRESS_SCALE_AT_ZERO) * s

        modified[i] = a * scale
      # a <= a_min（更強煞車）或 a>=0 都不動

    return modified
