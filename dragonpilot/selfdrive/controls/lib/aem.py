import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

# ============================================================
# AEM v3（全速域預測煞停版）
# 需求：
# - 取消原本 50 km/h 的硬性啟動門檻。
# - 改為全速域監控：當模型預測需要煞車/遇到停止線時（軌跡縮短且收斂），自動切換 blended。
# - 保留極低速防護（< 5 km/h），避免車輛靜止或蠕行時頻繁切換模式。
# ============================================================

# ============================================================
# 可調參數集中區
# ============================================================

# --- 0) 極低速防護門檻 ----------------------------------------
MIN_ENGAGE_SPEED = 5.0        # km/h：低於此速度時已不需要 AEM 介入，避免靜止時亂跳

# --- 1) 快速切換 / 防抖 ---------------------------------------
FAST_SWITCH_ENABLE = True
HARD_OVERRIDE_HOLD = 0.40     # s：觸發後先硬覆蓋 blended 一段時間（避免猶豫）
RETRIGGER_BLOCK_TIME = 0.10   # s：觸發後短時間禁止再次觸發（避免抖動）

# --- 2) 權重淡入淡出 + cooldown（stopline 觸發使用） ----------
RAMP_UP_TIME_BASE = 0.18
RAMP_DOWN_TIME_BASE = 0.38
RAMP_V_BP = [0.0, 6.0, 14.0, 25.0]         # m/s
RAMP_V_MULT = [0.8, 1.0, 1.35, 1.7]
COOLDOWN_V_BP = [0.0, 6.0, 14.0, 25.0]     # m/s
COOLDOWN_V_VALS = [0.28, 0.50, 0.72, 0.95] # s
W_MODE_OVERRIDE = 0.30                     # 越小越容易覆蓋 blended

# --- 3) 停止線/紅燈判停距離（用模型末端距離判斷） -------------
SLOW_DOWN_BP =   [0.0,  2.5,  5.5,  8.5,  11.5,  14.0,  17.0,  20.0,  25.0]
SLOW_DOWN_DIST = [8.0,  18.0, 35.0, 55.0, 70.0,  85.0, 105.0, 130.0, 165.0]

# --- 4) 連續幀確認 / 趨勢判斷（煞車需求判定核心） -------------
TRIGGER_CONFIRM_FRAMES = 3
TREND_K = 4
TREND_DX_MAX = 0.80           # 判斷軌跡末端是否收斂（代表要煞停）
TRIGGER_MARGIN = 1.20

STRONG_STOP_MARGIN = 0.75
STRONG_TREND_DX_MAX = 0.55
STRONG_CONFIRM_FRAMES = 1

# --- 5) 解除 hysteresis（避免抖動） -----------------------------
RELEASE_MARGIN = 1.15
MIN_ACTIVE_TIME = 1.0

# --- 6) lead gate（避免 stopline 搶控） ------------------------
LEAD_GATE_DIST = 45.0
LEAD_GATE_VREL = -3.0


class AEM:
  def __init__(self):
    self._active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._last_w = 0.0

    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0

  def _get_lead_info(self, radar_msg):
    """安全取得 leadOne 資訊（避免欄位不存在）"""
    lead = getattr(radar_msg, "leadOne", None)
    if lead is None:
      return False, 1e9, 0.0
    status = bool(getattr(lead, "status", False))
    d_rel = float(getattr(lead, "dRel", 1e9))
    v_rel = float(getattr(lead, "vRel", 0.0))
    return status, d_rel, v_rel

  def _reset_state_immediate(self):
    """立刻解除：清空所有狀態（不等 cooldown/淡出）"""
    self._active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0
    self._last_w = 0.0

  def _perform_experimental_mode(self, v_ego):
    """預測到煞車需求：啟動 cooldown 模式，並加上硬覆蓋與防抖"""
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
    """輸出 AEM 權重（給外部做更細緻混合用）"""
    t_now = time.monotonic()
    if not self._active:
      self._last_w = 0.0
      return 0.0

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
    決定當下模式：
    - 極低速時禁用 AEM
    - 當判斷有煞車需求時，允許 AEM 覆蓋為 blended
    """
    t_now = time.monotonic()

    # 極低速防護：避免靜止或蠕行時殘留 AEM 狀態
    if v_ego is not None:
      v_kph = v_ego * 3.6
      if v_kph < MIN_ENGAGE_SPEED:
        if self._active:
          self._reset_state_immediate()
        return mode

    # 觸發後硬覆蓋（快速切入，不等權重）
    if FAST_SWITCH_ENABLE and t_now < self._hard_override_end_time:
      return "blended"

    # 權重覆蓋（淡入淡出）
    w = self._last_w if v_ego is None else self.get_weight(v_ego)
    if w >= W_MODE_OVERRIDE and t_now < self._cooldown_end_time:
      return "blended"

    # cooldown 到就解除 active
    if t_now >= self._cooldown_end_time:
      self._active = False
      self._trigger_cnt = 0

    return mode

  def update_states(self, model_msg, radar_msg, v_ego):
    """
    核心狀態更新：
    全速域監控模型軌跡，當滿足「需要煞車/判停」條件時觸發 AEM。
    """
    v_ego_kph = v_ego * 3.6

    # 極低速防護：低於 5 km/h 不累積任何觸發狀態
    if v_ego_kph < MIN_ENGAGE_SPEED:
      if self._active:
        self._reset_state_immediate()
      return

    t_now = time.monotonic()

    # lead gate：前方有移動前車時，避免被 stopline 邏輯搶控
    lead_ok, d_rel, v_rel = self._get_lead_info(radar_msg)
    lead_gate_block = lead_ok and d_rel < LEAD_GATE_DIST and v_rel > LEAD_GATE_VREL

    # 模型輸出可用性檢查
    N = ModelConstants.IDX_N
    if not (len(model_msg.orientation.x) == len(model_msg.position.x) == N):
      self._trigger_cnt = 0
      return

    pos_x = np.asarray(model_msg.position.x)
    x_end = float(pos_x[N - 1])

    # 依速度插值出預期的煞停距離門檻
    slow_dist = float(np.interp(v_ego, SLOW_DOWN_BP, SLOW_DOWN_DIST))
    trigger_th = slow_dist * TRIGGER_MARGIN
    release_th = slow_dist * RELEASE_MARGIN

    # 趨勢判斷：確認軌跡末端是否收斂（代表模型想要煞車）
    k = min(TREND_K, N - 1)
    dx_tail = np.diff(pos_x[-(k+1):])
    mean_dx_tail = float(np.mean(dx_tail)) if dx_tail.size else 1e9
    trend_ok = mean_dx_tail < TREND_DX_MAX

    # 煞車需求觸發：距離縮短 + 軌跡收斂 + 前方無干擾車輛
    stopline_trigger_now = (x_end < trigger_th) and trend_ok and (not lead_gate_block)

    # 強信號快速通道：非常明顯需要急煞 -> 1 幀直接觸發
    strong_stop = (x_end < slow_dist * STRONG_STOP_MARGIN) and (mean_dx_tail < STRONG_TREND_DX_MAX)
    fast_path = FAST_SWITCH_ENABLE and strong_stop
    confirm_frames = STRONG_CONFIRM_FRAMES if fast_path else TRIGGER_CONFIRM_FRAMES

    # 觸發判定（只在非 active 時累積）
    if not self._active:
      if stopline_trigger_now:
        self._trigger_cnt += 1
        if self._trigger_cnt >= confirm_frames:
          self._perform_experimental_mode(v_ego)
          self._trigger_cnt = 0
      else:
        self._trigger_cnt = 0

    # 觸發解除：時間滿足最低要求，且軌跡重新拉長（代表不需要煞車了）
    if self._active:
      if (t_now - self._start_time) < MIN_ACTIVE_TIME:
        return
      if t_now >= self._cooldown_end_time and x_end > release_th:
        self._active = False
        self._trigger_cnt = 0

