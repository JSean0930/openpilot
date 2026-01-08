import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

# ============================================================
# AEM v2：人性化 Experimental Mode（blended）切換器
#
# 功能摘要：
# 1) 5~30 km/h 低速區間：強制鎖定 experimental（回傳 blended），離開立刻解除
# 2) 非強制區間：用模型預測判斷「前方可能要停」(stopline/紅燈/停止線)
#    - 連續幀確認 + 趨勢收斂判斷避免誤觸
#    - 觸發後先硬覆蓋一小段時間，再用權重淡入淡出
# 3) lead gate：若前方有可信 lead 且情況穩定，避免 stopline 觸發搶控
# ============================================================
"""
A) 低速 5–30 kph 強制鎖定（體感最直接）
	•	FORCE_EXP_KPH_MIN / MAX
	•	想讓更多低速都進 experimental：把 MAX 往上（例如 35）
	•	想只保留 stop&go：把 MIN 調到 8、MAX 調到 25
	•	FORCE_EXP_ENABLE
	•	要快速 A/B 測試就靠它開關

B) 切換更快 / 更不猶豫
	•	HARD_OVERRIDE_HOLD
	•	增大：切進去更乾脆（0.25~0.35），但可能太常 “硬”
	•	減小：更柔和，但可能覺得猶豫
	•	W_MODE_OVERRIDE
	•	降低：更容易保持 blended（更激進）
	•	提高：更不容易覆蓋（更保守）

C) stopline 偵測靈敏度（誤觸 vs 早觸發）
	•	SLOW_DOWN_DIST / SLOW_DOWN_BP
	•	這是「速度越快越早判停」的核心表
	•	TRIGGER_CONFIRM_FRAMES
	•	變小：更敏感但更容易誤觸（2 或 1）
	•	TREND_DX_MAX
	•	變大：更容易認定在收斂（更敏感）
	•	變小：更嚴格（更穩）
	•	STRONG_STOP_MARGIN / STRONG_TREND_DX_MAX
	•	這兩個決定「強信號」多容易成立 → 影響 1 幀快速觸發的頻率

D) 抖動/來回切換
	•	RETRIGGER_BLOCK_TIME
	•	增大：更不抖，但反應可能慢一點
	•	RELEASE_MARGIN
	•	增大：更晚解除（更穩但拖）
	•	MIN_ACTIVE_TIME
	•	增大：至少保持久一點（避免瞬間跳出）

E) lead gate（避免跟車時被 stopline 搶控）
	•	LEAD_GATE_DIST / LEAD_GATE_VREL
	•	如果你常在跟車時誤判停止線，可把 LEAD_GATE_DIST 拉大一點（例如 55）

"""
# ============================================================
# 可調參數集中區（AEM v2 + 快速切換靈敏化）
# ============================================================

# --- 0) 快速切換 / 防抖 ---------------------------------------
FAST_SWITCH_ENABLE = True

# 觸發後先「硬覆蓋」blended 的時間（避免權重還在爬升，體感像猶豫）
HARD_OVERRIDE_HOLD = 0.4   # s，0.15~0.30 常用，0.2

# 觸發後短時間內禁止再次觸發（避免模式來回抖動）
RETRIGGER_BLOCK_TIME = 0.10 # s，0.15~0.40 常用，0.2


# --- 0.1) 5~30kph 強制鎖定 Experimental Mode -------------------
FORCE_EXP_ENABLE = True

# 只要速度落在此區間，就強制回 blended（experimental）
FORCE_EXP_KPH_MIN = 5.0
FORCE_EXP_KPH_MAX = 30.0
# 需求：離開區間要立刻解除 -> 下方會呼叫 _reset_state_immediate()


# --- 1) 權重淡入淡出 + cooldown（stopline 觸發使用） ----------
# 權重淡入淡出基礎時間：越小越快進入/退出
RAMP_UP_TIME_BASE = 0.18
RAMP_DOWN_TIME_BASE = 0.38

# 隨速度調整淡入淡出倍率（高速稍微慢一些比較穩）
RAMP_V_BP = [0.0, 6.0, 14.0, 25.0]         # m/s
RAMP_V_MULT = [0.8, 1.0, 1.35, 1.7]

# stopline 觸發後維持時間（秒），速度越高通常可稍長/或依你策略調整
COOLDOWN_V_BP = [0.0, 6.0, 14.0, 25.0]     # m/s
COOLDOWN_V_VALS = [0.28, 0.50, 0.72, 0.95] # s

# 權重大於這個值，就覆蓋成 blended（越小越容易覆蓋）
W_MODE_OVERRIDE = 0.30 #0.45


# --- 2) 停止線/紅燈判停距離（用模型末端距離判斷） -------------
# v_ego(m/s) -> 判停距離(m)，速度越高通常越早判停（距離更長）
SLOW_DOWN_BP =   [0.0,  2.5,  5.5,  8.5,  11.5,  14.0,  17.0,  20.0,  25.0]
SLOW_DOWN_DIST = [8.0,  18.0, 35.0, 55.0, 70.0,  85.0, 105.0, 130.0, 165.0]


# --- 3) 連續幀確認 / 趨勢判斷 -----------------------------------
# 弱信號：需要連續幾幀都成立才觸發（避免單幀誤判）
TRIGGER_CONFIRM_FRAMES = 3

# 趨勢收斂：看最後 TREND_K 段的 position.x 增量平均，小於門檻表示路徑在收斂（像要停）
TREND_K = 4
TREND_DX_MAX = 0.80          # m/step（越小越嚴格）

# 停止線/紅燈（stopline）觸發距離門檻的倍率。
# 更靈敏、更早切：1.05 ~ 1.20。更穩、更少誤觸：0.90 ~ 1.00
TRIGGER_MARGIN = 1.20 #1.00

# 強信號快速通道：更明顯要停 -> 1 幀觸發
STRONG_STOP_MARGIN = 0.75
STRONG_TREND_DX_MAX = 0.55
STRONG_CONFIRM_FRAMES = 1


# --- 4) 解除 hysteresis（避免抖動） -----------------------------
# 解除裕度：需要回到更安全距離才解除（越大越不易解除）
RELEASE_MARGIN = 1.15

# 最短保持時間：剛觸發後至少維持這麼久，避免瞬間解除造成抖
MIN_ACTIVE_TIME = 1.0 #0.18


# --- 5) lead gate（避免 stopline 搶控） ------------------------
# 有 lead 且距離近，且沒有快速逼近時，就不讓 stopline 觸發搶控
LEAD_GATE_DIST = 45.0
LEAD_GATE_VREL = -3.0   # vRel > -3 表示沒有很快逼近（較穩定）


# --- 6) 舊參數（未使用，可刪） ---------------------------------
LOW_SPEED_LEAD_TRIG_KPH   = 40.0
LOW_SPEED_LEAD_DIST_MAX   = 40.0
LOW_SPEED_LEAD_DECEL_TRIG = -0.01


class AEM:
  def __init__(self):
    # 一般觸發 active（stopline/cooldown）
    self._active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._last_w = 0.0

    # 用 dv/dt 估計自車加速度（目前只保留做擴充用）
    self._last_v_ego = None
    self._last_v_time = None

    # 快速切換機制：硬覆蓋時間 + 防抖時間
    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0

    # 5~30kph 強制鎖定旗標
    self._force_active = False

  def _get_lead_info(self, radar_msg):
    """安全取得 leadOne 資訊（避免欄位不存在）"""
    lead = getattr(radar_msg, "leadOne", None)
    if lead is None:
      return False, 1e9, 0.0
    status = bool(getattr(lead, "status", False))
    d_rel = float(getattr(lead, "dRel", 1e9))
    v_rel = float(getattr(lead, "vRel", 0.0))
    return status, d_rel, v_rel

  def _estimate_ego_accel(self, v_ego):
    """用單純 dv/dt 估計自車加速度（目前主要是保留擴充彈性）"""
    t_now = time.monotonic()
    a_ego = 0.0
    if self._last_v_ego is not None and self._last_v_time is not None:
      dt = t_now - self._last_v_time
      if dt > 1e-3:
        a_ego = (v_ego - self._last_v_ego) / dt
    self._last_v_ego = v_ego
    self._last_v_time = t_now
    return a_ego

  def _reset_state_immediate(self):
    """立刻解除：離開強制區間時直接清空所有狀態（不等 cooldown/淡出）"""
    self._active = False
    self._force_active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0
    self._last_w = 0.0

  def _set_force_active(self):
    """
    進入 5~30kph 強制區間時：
    - force_active = True
    - 同時 active = True（方便外部若需要知道目前 AEM 狀態）
    - 用很遠的 cooldown_end_time 只是保險：實際上 get_mode/update_states 會直接鎖 blended
    """
    t_now = time.monotonic()
    self._force_active = True
    self._active = True
    self._start_time = t_now
    self._cooldown_end_time = t_now + 3600.0
    self._hard_override_end_time = t_now + 3600.0

  def _perform_experimental_mode(self, v_ego):
    """stopline/判停觸發：啟動 cooldown 模式，並加上硬覆蓋與防抖"""
    t_now = time.monotonic()
    if FAST_SWITCH_ENABLE and t_now < self._retrigger_block_end_time:
      return

    cooldown = float(np.interp(v_ego, COOLDOWN_V_BP, COOLDOWN_V_VALS))
    self._active = True
    self._start_time = t_now
    self._cooldown_end_time = t_now + cooldown

    if FAST_SWITCH_ENABLE:
      self._hard_override_end_time = t_now + HARD_OVERRIDE_HOLD
      self._retrigger_block_end_time = t_now + RETRIGGER_BLOCK_TIME

  def get_weight(self, v_ego):
    """
    輸出 AEM 權重（給外部做更細緻混合用）：
    - force_active 時固定 1.0
    - 否則依淡入淡出曲線 w_in*w_out（先上升再下降）
    """
    t_now = time.monotonic()
    if not self._active:
      self._last_w = 0.0
      return 0.0

    if self._force_active:
      self._last_w = 1.0
      return 1.0

    v_mult = float(np.interp(v_ego, RAMP_V_BP, RAMP_V_MULT))
    ramp_up = RAMP_UP_TIME_BASE * v_mult
    ramp_dn = RAMP_DOWN_TIME_BASE * v_mult

    t_start = self._start_time
    t_end = self._cooldown_end_time

    w_in = np.clip((t_now - t_start) / max(ramp_up, 1e-3), 0.0, 1.0)
    w_out = np.clip((t_end - t_now) / max(ramp_dn, 1e-3), 0.0, 1.0)

    w = float(w_in * w_out)
    self._last_w = w
    return w

  def get_mode(self, mode, v_ego=None):
    """
    mode 覆蓋邏輯（回傳 "blended" 或原 mode）：
    1) 若在 5~30kph 強制區間：直接 blended，離開立刻解除
    2) stopline 觸發後：先硬覆蓋 HARD_OVERRIDE_HOLD 秒
    3) 之後用權重 >= W_MODE_OVERRIDE 且 still in cooldown 來覆蓋
    """
    t_now = time.monotonic()

    # 1) 5~30kph 強制鎖定
    if FORCE_EXP_ENABLE and v_ego is not None:
      v_kph = v_ego * 3.6
      in_force = (FORCE_EXP_KPH_MIN <= v_kph <= FORCE_EXP_KPH_MAX)

      if in_force:
        if not self._force_active:
          self._set_force_active()
        return "blended"
      else:
        if self._force_active:
          self._reset_state_immediate()

    # 2) 觸發後硬覆蓋（快速切入，不等權重）
    if FAST_SWITCH_ENABLE and t_now < self._hard_override_end_time:
      return "blended"

    # 3) 權重覆蓋（淡入淡出）
    w = self._last_w if v_ego is None else self.get_weight(v_ego)
    if w >= W_MODE_OVERRIDE and t_now < self._cooldown_end_time:
      return "blended"

    # 4) cooldown 到就解除 active
    if t_now >= self._cooldown_end_time:
      self._active = False
      self._trigger_cnt = 0

    return mode

  def update_states(self, model_msg, radar_msg, v_ego):
    """
    更新觸發狀態：
    - 強制區間：進入就鎖定並 return（最高優先）
    - 非強制區間：檢查 stopline 判停條件，滿足後觸發 experimental
    """
    _ = self._estimate_ego_accel(v_ego)  # 目前主要保留擴充用途
    v_ego_kph = v_ego * 3.6

    # 強制區間：只要在 5~30kph，就直接鎖定
    if FORCE_EXP_ENABLE:
      in_force = (FORCE_EXP_KPH_MIN <= v_ego_kph <= FORCE_EXP_KPH_MAX)
      if in_force:
        if not self._force_active:
          self._set_force_active()
        return
      else:
        # 離開區間立刻解除（即使外部還沒呼叫 get_mode）
        if self._force_active:
          self._reset_state_immediate()
        # 繼續往下走，允許 stopline 正常觸發

    t_now = time.monotonic()

    # lead gate：避免 stopline 搶控
    lead_ok, d_rel, v_rel = self._get_lead_info(radar_msg)
    lead_gate_block = lead_ok and d_rel < LEAD_GATE_DIST and v_rel > LEAD_GATE_VREL

    # 模型輸出可用性檢查
    N = ModelConstants.IDX_N
    if not (len(model_msg.orientation.x) == len(model_msg.position.x) == N):
      self._trigger_cnt = 0
      return

    pos_x = np.asarray(model_msg.position.x)
    x_end = float(pos_x[N - 1])

    # 依速度插值出判停距離門檻
    slow_dist = float(np.interp(v_ego, SLOW_DOWN_BP, SLOW_DOWN_DIST))
    trigger_th = slow_dist * TRIGGER_MARGIN
    release_th = slow_dist * RELEASE_MARGIN

    # 趨勢判斷：末段增量平均
    k = min(TREND_K, N - 1)
    dx_tail = np.diff(pos_x[-(k+1):])
    mean_dx_tail = float(np.mean(dx_tail)) if dx_tail.size else 1e9
    trend_ok = mean_dx_tail < TREND_DX_MAX

    # stopline/判停觸發
    stopline_trigger_now = (x_end < trigger_th) and trend_ok and (not lead_gate_block)

    # 強信號快速通道：更明顯要停 -> 1 幀觸發
    strong_stop = (x_end < slow_dist * STRONG_STOP_MARGIN) and (mean_dx_tail < STRONG_TREND_DX_MAX)
    fast_path = FAST_SWITCH_ENABLE and strong_stop
    confirm_frames = STRONG_CONFIRM_FRAMES if fast_path else TRIGGER_CONFIRM_FRAMES

    if not self._active:
      if stopline_trigger_now:
        self._trigger_cnt += 1
        if self._trigger_cnt >= confirm_frames:
          self._perform_experimental_mode(v_ego)
          self._trigger_cnt = 0
      else:
        self._trigger_cnt = 0

    # stopline 觸發解除：時間到且回到安全距離
    if self._active:
      if (t_now - self._start_time) < MIN_ACTIVE_TIME:
        return
      if t_now >= self._cooldown_end_time and x_end > release_th:
        self._active = False
        self._trigger_cnt = 0