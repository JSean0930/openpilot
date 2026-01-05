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
HARD_OVERRIDE_HOLD = 0.20   # s
RETRIGGER_BLOCK_TIME = 0.20 # s

# --- 0.1) 新增：5~30kph 強制鎖定 Experimental Mode -------------
FORCE_EXP_ENABLE = True
FORCE_EXP_KPH_MIN = 5.0
FORCE_EXP_KPH_MAX = 30.0
# 需求：離開區間要「立刻解除」 -> 不使用淡出


# --- 1) 觸發與解除時間/權重（平滑淡入淡出） -------------------
RAMP_UP_TIME_BASE = 0.18
RAMP_DOWN_TIME_BASE = 0.38

RAMP_V_BP = [0.0, 6.0, 14.0, 25.0]
RAMP_V_MULT = [0.8, 1.0, 1.35, 1.7]

COOLDOWN_V_BP = [0.0, 6.0, 14.0, 25.0]
COOLDOWN_V_VALS = [0.28, 0.50, 0.72, 0.95]

W_MODE_OVERRIDE = 0.45


# --- 2) 停止線/紅燈判停距離 -----------------------------------
SLOW_DOWN_BP =   [0.0,  2.5,  5.5,  8.5,  11.5,  14.0,  17.0,  20.0,  25.0]
SLOW_DOWN_DIST = [8.0,  18.0, 35.0, 55.0, 70.0,  85.0, 105.0, 130.0, 165.0]


# --- 3) 連續幀確認 / 趨勢判斷（改成可自適應） -----------------
TRIGGER_CONFIRM_FRAMES = 3
TREND_K = 4
TREND_DX_MAX = 0.80
TRIGGER_MARGIN = 1.00

STRONG_STOP_MARGIN = 0.75
STRONG_TREND_DX_MAX = 0.55
STRONG_CONFIRM_FRAMES = 1


# --- 4) 解除 hysteresis ---------------------------------------
RELEASE_MARGIN = 1.15
MIN_ACTIVE_TIME = 0.18


# --- 5) lead gate ---------------------------------------------
LEAD_GATE_DIST = 45.0
LEAD_GATE_VREL = -3.0


# --- 6) 舊低速跟車減速觸發（已不再使用，可保留避免外部引用爆掉） ---
LOW_SPEED_LEAD_TRIG_KPH   = 40.0
LOW_SPEED_LEAD_DIST_MAX   = 40.0
LOW_SPEED_LEAD_DECEL_TRIG = -0.01


class AEM:
  def __init__(self):
    self._active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._last_w = 0.0

    self._last_v_ego = None
    self._last_v_time = None

    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0

    # 新增：強制區間鎖定旗標
    self._force_active = False

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

  def _reset_state_immediate(self):
    """立刻解除：清掉所有 active/override/cooldown 狀態"""
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
    進入強制區間時：
    - 鎖定 force_active
    - 同時把 active 設為 True（方便外部若會看 active 狀態）
    - 不靠 cooldown 進出（因為要“保持”）
    """
    t_now = time.monotonic()
    self._force_active = True
    self._active = True
    self._start_time = t_now
    # 讓 cooldown_end_time 足夠遠（實際上 get_mode 會直接鎖 blended）
    self._cooldown_end_time = t_now + 3600.0
    # 也把硬覆蓋延長（同樣只是保險，get_mode 會直接鎖）
    self._hard_override_end_time = t_now + 3600.0

  def _perform_experimental_mode(self, v_ego):
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
    t_now = time.monotonic()
    if not self._active:
      self._last_w = 0.0
      return 0.0

    # 強制鎖定期間：權重直接視為 1（讓外部混合也能吃到強制效果）
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
    需求：5~30kph 期間保持 experimental（blended），離開區間立刻解除。
    """
    t_now = time.monotonic()

    if FORCE_EXP_ENABLE and v_ego is not None:
      v_kph = v_ego * 3.6
      in_force = (FORCE_EXP_KPH_MIN <= v_kph <= FORCE_EXP_KPH_MAX)

      if in_force:
        # 進入/保持強制鎖定
        if not self._force_active:
          self._set_force_active()
        return "blended"
      else:
        # 一離開區間 -> 立刻解除（不等 cooldown）
        if self._force_active:
          self._reset_state_immediate()

    # 1) 觸發後硬覆蓋
    if FAST_SWITCH_ENABLE and t_now < self._hard_override_end_time:
      return "blended"

    # 2) 權重覆蓋
    w = self._last_w if v_ego is None else self.get_weight(v_ego)
    if w >= W_MODE_OVERRIDE and t_now < self._cooldown_end_time:
      return "blended"

    # 3) cooldown 結束才解除 active
    if t_now >= self._cooldown_end_time:
      self._active = False
      self._trigger_cnt = 0

    return mode

  def update_states(self, model_msg, radar_msg, v_ego):
    _ = self._estimate_ego_accel(v_ego)
    v_ego_kph = v_ego * 3.6

    # 強制區間：只要在 5~30kph，就直接鎖定，不走下面觸發邏輯
    if FORCE_EXP_ENABLE:
      in_force = (FORCE_EXP_KPH_MIN <= v_ego_kph <= FORCE_EXP_KPH_MAX)
      if in_force:
        if not self._force_active:
          self._set_force_active()
        return
      else:
        # 離開區間立刻解除（若 get_mode 尚未被呼叫，也能在這邊解除）
        if self._force_active:
          self._reset_state_immediate()
        # 繼續往下走，允許 stopline 正常觸發

    t_now = time.monotonic()
    lead_ok, d_rel, v_rel = self._get_lead_info(radar_msg)
    lead_gate_block = lead_ok and d_rel < LEAD_GATE_DIST and v_rel > LEAD_GATE_VREL

    N = ModelConstants.IDX_N
    if not (len(model_msg.orientation.x) == len(model_msg.position.x) == N):
      self._trigger_cnt = 0
      return

    pos_x = np.asarray(model_msg.position.x)
    x_end = float(pos_x[N - 1])

    slow_dist = float(np.interp(v_ego, SLOW_DOWN_BP, SLOW_DOWN_DIST))
    trigger_th = slow_dist * TRIGGER_MARGIN
    release_th = slow_dist * RELEASE_MARGIN

    k = min(TREND_K, N - 1)
    dx_tail = np.diff(pos_x[-(k+1):])
    mean_dx_tail = float(np.mean(dx_tail)) if dx_tail.size else 1e9
    trend_ok = mean_dx_tail < TREND_DX_MAX

    stopline_trigger_now = (x_end < trigger_th) and trend_ok and (not lead_gate_block)

    strong_stop = (x_end < slow_dist * STRONG_STOP_MARGIN) and (mean_dx_tail < STRONG_TREND_DX_MAX)
    fast_path = FAST_SWITCH_ENABLE and strong_stop
    confirm_frames = STRONG_CONFIRM_FRAMES if fast_path else TRIGGER_CONFIRM_FRAMES

    trigger_now = stopline_trigger_now

    if not self._active:
      if trigger_now:
        self._trigger_cnt += 1
        if self._trigger_cnt >= confirm_frames:
          self._perform_experimental_mode(v_ego)
          self._trigger_cnt = 0
      else:
        self._trigger_cnt = 0

    if self._active:
      if (t_now - self._start_time) < MIN_ACTIVE_TIME:
        return
      if t_now >= self._cooldown_end_time and x_end > release_th:
        self._active = False
        self._trigger_cnt = 0
