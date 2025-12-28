"""
Copyright (c) 2025, Rick Lan
... (略)
"""

import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants


# ============================================================
# 可調參數集中區（AEM v2 + 快速切換靈敏化）
# ============================================================

# --- 0) 快速切換總開關/防抖 -----------------------------------
FAST_SWITCH_ENABLE = True

# 觸發後先硬覆蓋 blended 一小段時間（避免權重還在淡入導致“猶豫”）
HARD_OVERRIDE_HOLD = 0.20   # s，建議 0.15~0.30

# 觸發後短時間內禁止再次觸發（防止來回抖）
RETRIGGER_BLOCK_TIME = 0.20 # s


# --- 1) 觸發與解除時間/權重（平滑淡入淡出） -------------------
RAMP_UP_TIME_BASE = 0.18     # 更快進入（原 0.25）
RAMP_DOWN_TIME_BASE = 0.38   # 稍快退出（原 0.45）

RAMP_V_BP = [0.0, 6.0, 14.0, 25.0]
RAMP_V_MULT = [0.8, 1.0, 1.35, 1.7]

COOLDOWN_V_BP = [0.0, 6.0, 14.0, 25.0]
COOLDOWN_V_VALS = [0.28, 0.50, 0.72, 0.95]  # 稍短一些（原 0.30/0.55/0.80/1.10）

# 更早進入 mode 覆蓋（原 0.60）
W_MODE_OVERRIDE = 0.45


# --- 2) 停止線/紅燈判停距離 -----------------------------------
SLOW_DOWN_BP =   [0.0,  2.5,  5.5,  8.5,  11.5,  14.0,  17.0,  20.0,  25.0]
SLOW_DOWN_DIST = [8.0,  18.0, 35.0, 55.0, 70.0,  85.0, 105.0, 130.0, 165.0]


# --- 3) 連續幀確認 / 趨勢判斷（改成可自適應） -----------------
TRIGGER_CONFIRM_FRAMES = 3   # 基準值（在弱信號情境仍維持）
TREND_K = 4
TREND_DX_MAX = 0.80

TRIGGER_MARGIN = 1.00

# 強信號快速通道：滿足更“明顯要停”的條件時，直接用 1 幀觸發
STRONG_STOP_MARGIN = 0.75     # x_end < slow_dist*0.75 視為強信號（越小越嚴格）
STRONG_TREND_DX_MAX = 0.55    # 趨勢更收斂才算強信號
STRONG_CONFIRM_FRAMES = 1     # 強信號直接 1 幀


# --- 4) 解除 hysteresis ---------------------------------------
RELEASE_MARGIN = 1.15         # 解除更快（原 1.20）
MIN_ACTIVE_TIME = 0.18        # 更快允許解除（原 0.25）

# 若你希望“更不猶豫”，可以再降到 0.15，但抖動風險會升高


# --- 5) lead gate ---------------------------------------------
LEAD_GATE_DIST = 45.0
LEAD_GATE_VREL = -3.0


# --- 6) 低速跟車減速觸發 AEM（快速通道） ----------------------
LOW_SPEED_LEAD_TRIG_KPH   = 40.0
LOW_SPEED_LEAD_DIST_MAX   = 40.0
LOW_SPEED_LEAD_DECEL_TRIG = -0.01   # 你目前設定非常敏感（幾乎只要速度稍降就觸發）


class AEM:
  def __init__(self):
    self._active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._last_w = 0.0

    self._last_v_ego = None
    self._last_v_time = None

    # 新增：硬覆蓋與防抖
    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0

  def _get_lead_info(self, radar_msg):
    lead = getattr(radar_msg, "leadOne", None)
    if lead is None:
      return False, 1e9, 0.0
    status = bool(getattr(lead, "status", False))
    d_rel = float(getattr(lead, "dRel", 1e9))
    v_rel = float(getattr(lead, "vRel", 0.0))
    return status, d_rel, v_rel

  def _estimate_ego_accel(self, v_ego):
    t_now = time.monotonic()
    a_ego = 0.0
    if self._last_v_ego is not None and self._last_v_time is not None:
      dt = t_now - self._last_v_time
      if dt > 1e-3:
        a_ego = (v_ego - self._last_v_ego) / dt
    self._last_v_ego = v_ego
    self._last_v_time = t_now
    return a_ego

  def _perform_experimental_mode(self, v_ego):
    t_now = time.monotonic()

    # 防抖：短時間內不允許重複觸發（避免來回）
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
    強化：觸發後先硬覆蓋一小段時間，做到“快速切換不猶豫”。
    """
    t_now = time.monotonic()

    # 1) 觸發後硬覆蓋（不等權重爬升）
    if FAST_SWITCH_ENABLE and t_now < self._hard_override_end_time:
      return "blended"

    # 2) 原本權重覆蓋
    w = self._last_w if v_ego is None else self.get_weight(v_ego)
    if w >= W_MODE_OVERRIDE and t_now < self._cooldown_end_time:
      return "blended"

    # 3) cooldown 結束才解除 active
    if t_now >= self._cooldown_end_time:
      self._active = False
      self._trigger_cnt = 0

    return mode

  def update_states(self, model_msg, radar_msg, v_ego):
    a_ego = self._estimate_ego_accel(v_ego)
    v_ego_kph = v_ego * 3.6
    t_now = time.monotonic()

    lead_ok, d_rel, v_rel = self._get_lead_info(radar_msg)

    # 停止線觸發用 lead gate
    lead_gate_block = lead_ok and d_rel < LEAD_GATE_DIST and v_rel > LEAD_GATE_VREL

    # 低速跟車減速：快速通道（不受 lead_gate_block 限制）
    low_speed_lead_trigger = (
      lead_ok and
      (v_ego_kph < LOW_SPEED_LEAD_TRIG_KPH) and
      (d_rel < LOW_SPEED_LEAD_DIST_MAX) and
      (a_ego < LOW_SPEED_LEAD_DECEL_TRIG)
    )

    # 模型輸出可用性檢查
    N = ModelConstants.IDX_N
    if not (len(model_msg.orientation.x) == len(model_msg.position.x) == N):
      self._trigger_cnt = 0
      return

    pos_x = np.asarray(model_msg.position.x)
    x_end = float(pos_x[N - 1])

    slow_dist = float(np.interp(v_ego, SLOW_DOWN_BP, SLOW_DOWN_DIST))
    trigger_th = slow_dist * TRIGGER_MARGIN
    release_th = slow_dist * RELEASE_MARGIN

    # 趨勢判斷
    k = min(TREND_K, N - 1)
    dx_tail = np.diff(pos_x[-(k+1):])
    mean_dx_tail = float(np.mean(dx_tail)) if dx_tail.size else 1e9
    trend_ok = mean_dx_tail < TREND_DX_MAX

    # 停止線觸發（原邏輯）
    stopline_trigger_now = (x_end < trigger_th) and trend_ok and (not lead_gate_block)

    # -----------------------
    # ★ 快速切換：自適應確認幀數
    # -----------------------
    # 強信號：更明顯“要停了” -> 1 幀觸發
    strong_stop = (x_end < slow_dist * STRONG_STOP_MARGIN) and (mean_dx_tail < STRONG_TREND_DX_MAX)

    # 低速跟車減速：同樣視為強信號，走 1 幀快速通道
    fast_path = FAST_SWITCH_ENABLE and (low_speed_lead_trigger or strong_stop)

    confirm_frames = STRONG_CONFIRM_FRAMES if fast_path else TRIGGER_CONFIRM_FRAMES

    # 最終觸發
    trigger_now = stopline_trigger_now or low_speed_lead_trigger

    # 觸發判定
    if not self._active:
      if trigger_now:
        self._trigger_cnt += 1
        if self._trigger_cnt >= confirm_frames:
          self._perform_experimental_mode(v_ego)
          self._trigger_cnt = 0
      else:
        self._trigger_cnt = 0

    # 解除邏輯：更快允許解除，但仍保留門檻避免抖動
    if self._active:
      if (t_now - self._start_time) < MIN_ACTIVE_TIME:
        return

      if t_now >= self._cooldown_end_time and x_end > release_th:
        self._active = False
        self._trigger_cnt = 0
