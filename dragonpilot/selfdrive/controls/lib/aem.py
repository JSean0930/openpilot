import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants

# ============================================================
# AEM v2：人性化 Experimental Mode（blended）切換器
#
# 功能摘要：
# 1) 0~X km/h 低速區間：強制鎖定 blended（你原本的 FORCE_EXP 機制）
# 2) 非強制區間：用模型預測判斷「前方可能要停」(stopline/紅燈/停止線)
#    - 連續幀確認 + 趨勢收斂判斷避免誤觸
#    - 觸發後先硬覆蓋一小段時間，再用權重淡入淡出
# 3) lead gate：若前方有可信 lead 且情況穩定，避免 stopline 觸發搶控
#
# [新增] Deadlock Breaker（鎖死破除機制）：
# - 低速/起步最容易被「幽靈 lead / ACC 鎖死」卡住不走
# - 透過遲滯 + 最短保持 + 防抖，強制 blended 一段時間「把車帶動起來」
# - Deadlock Breaker 期間跳過其他判斷，避免被 stopline/lead gate 等邏輯拉扯
# ============================================================


# ============================================================
# 可調參數集中區（TUNING PARAMS）
# ============================================================

# ---------------------------------------------------------------------
# 0) Deadlock Breaker：鎖死破除機制（強烈建議開）
# ---------------------------------------------------------------------
DEADLOCK_BREAKER_ENABLE = True

# 進入條件：車速低於此值（m/s）視為「剛起步/近乎靜止」，啟動破除
# - 建議 0.6 ~ 1.2 m/s（約 2.2 ~ 4.3 km/h）
DEADLOCK_ENTER_MPS = 1.0

# 退出條件：車速高於此值（m/s）才允許退出（遲滯）
# - 建議 2.0 ~ 3.5 m/s（約 7.2 ~ 12.6 km/h）
# - 越大：越不會「剛動就被拉回」(更不鎖死)；但 blended 鎖定時間會更長
DEADLOCK_EXIT_MPS = 3.0

# 最短保持時間（秒）：一旦進入 deadlock，至少維持這麼久才允許退出
# - 建議 0.6 ~ 1.2 s
DEADLOCK_MIN_HOLD_S = 0.9

# 防抖重入（秒）：退出後短時間內不要立刻又進入，避免速度抖動造成來回切換
# - 建議 0.2 ~ 0.8 s
DEADLOCK_REARM_S = 0.4


# ---------------------------------------------------------------------
# 1) 快速切換 / 防抖（你原本的 AEM 快速不猶豫）
# ---------------------------------------------------------------------
FAST_SWITCH_ENABLE = True

# 觸發後先「硬覆蓋」blended 的時間（避免權重還在爬升，體感像猶豫）
HARD_OVERRIDE_HOLD = 0.40   # s（你目前 0.4）

# 觸發後短時間內禁止再次觸發（避免模式來回抖動）
RETRIGGER_BLOCK_TIME = 0.10 # s（你目前 0.10）


# ---------------------------------------------------------------------
# 2) 低速區間強制鎖定 blended（你原本的 FORCE_EXP）
# ---------------------------------------------------------------------
FORCE_EXP_ENABLE = True
FORCE_EXP_KPH_MIN = 0.0
FORCE_EXP_KPH_MAX = 25.0
# 離開區間：立刻解除（下方 _reset_state_immediate）


# ---------------------------------------------------------------------
# 3) 權重淡入淡出 + cooldown（stopline 觸發使用）
# ---------------------------------------------------------------------
RAMP_UP_TIME_BASE = 0.18
RAMP_DOWN_TIME_BASE = 0.38

RAMP_V_BP = [0.0, 6.0, 14.0, 25.0]      # m/s
RAMP_V_MULT = [0.8, 1.0, 1.35, 1.7]

COOLDOWN_V_BP = [0.0, 6.0, 14.0, 25.0]  # m/s
COOLDOWN_V_VALS = [0.28, 0.50, 0.72, 0.95]  # s

# 權重大於這個值，就覆蓋成 blended（越小越容易覆蓋）
W_MODE_OVERRIDE = 0.30  # 你目前 0.30


# ---------------------------------------------------------------------
# 4) 停止線/紅燈判停距離（用模型末端距離判斷）
# ---------------------------------------------------------------------
SLOW_DOWN_BP =   [0.0,  2.5,  5.5,  8.5,  11.5,  14.0,  17.0,  20.0,  25.0]
SLOW_DOWN_DIST = [8.0,  18.0, 35.0, 55.0, 70.0,  85.0, 105.0, 130.0, 165.0]


# ---------------------------------------------------------------------
# 5) 連續幀確認 / 趨勢判斷（stopline 偵測）
# ---------------------------------------------------------------------
TRIGGER_CONFIRM_FRAMES = 3
TREND_K = 4
TREND_DX_MAX = 0.80

# stopline 觸發距離倍率：越大越早觸發（也更容易誤觸）
TRIGGER_MARGIN = 1.20

# 強信號快速通道：更明顯要停 -> 1 幀觸發
STRONG_STOP_MARGIN = 0.75
STRONG_TREND_DX_MAX = 0.55
STRONG_CONFIRM_FRAMES = 1


# ---------------------------------------------------------------------
# 6) 解除 hysteresis（stopline 解除）
# ---------------------------------------------------------------------
RELEASE_MARGIN = 1.15
MIN_ACTIVE_TIME = 1.0  # 你目前 1.0（會讓 stopline 觸發後很「黏」）


# ---------------------------------------------------------------------
# 7) lead gate（避免 stopline 搶控）
# ---------------------------------------------------------------------
LEAD_GATE_DIST = 45.0
LEAD_GATE_VREL = -3.0


# ---------------------------------------------------------------------
# 8) 舊參數（未使用，可刪）
# ---------------------------------------------------------------------
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

    # dv/dt 估計（目前主要保留擴充彈性）
    self._last_v_ego = None
    self._last_v_time = None

    # 快速切換機制：硬覆蓋時間 + 防抖時間
    self._hard_override_end_time = 0.0
    self._retrigger_block_end_time = 0.0

    # FORCE_EXP 強制區間旗標（你原本的）
    self._force_active = False

    # -----------------------------
    # [新增] Deadlock Breaker 狀態
    # -----------------------------
    self._deadlock_active = False
    self._deadlock_start_time = 0.0
    self._deadlock_rearm_until = 0.0

  # ============================================================
  # 基礎工具
  # ============================================================
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
    """立刻解除：清空一般 AEM 狀態（不含 deadlock 狀態，由專用函式管理）"""
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
    進入 FORCE_EXP 區間時：
    - force_active = True
    - active = True（方便外部若需要知道目前 AEM 狀態）
    - 用很遠的 cooldown_end_time 只是保險：實際上 get_mode/update_states 會直接鎖 blended
    """
    t_now = time.monotonic()
    self._force_active = True
    self._active = True
    self._start_time = t_now
    self._cooldown_end_time = t_now + 3600.0
    self._hard_override_end_time = t_now + 3600.0

  # ============================================================
  # [新增] Deadlock Breaker（鎖死破除）核心
  # ============================================================
  def _enter_deadlock(self, t_now):
    """
    進入 deadlock 模式：
    - 直接強制 blended，並「跳過」其他邏輯（stopline/lead gate 等）
    - 使用遲滯 + 最短保持，避免剛動就被切回造成鎖死
    """
    self._deadlock_active = True
    self._deadlock_start_time = t_now
    self._deadlock_rearm_until = t_now + DEADLOCK_REARM_S

    # 進入 deadlock 時，清掉一般 AEM 狀態，避免殘留造成拉扯
    self._reset_state_immediate()

    # 立刻讓外部看起來是 active（但不使用 FORCE_EXP 的 force_active）
    self._active = True
    self._hard_override_end_time = t_now + max(HARD_OVERRIDE_HOLD, 0.15)

  def _can_exit_deadlock(self, v_ego, t_now):
    """退出條件：速度 > EXIT 且已持有 MIN_HOLD"""
    if not self._deadlock_active:
      return True
    held = (t_now - self._deadlock_start_time) >= DEADLOCK_MIN_HOLD_S
    return held and (v_ego >= DEADLOCK_EXIT_MPS)

  def _exit_deadlock(self):
    """退出 deadlock：立刻恢復到一般流程（不保留任何覆蓋狀態）"""
    self._deadlock_active = False
    # 退出後不保留 active，避免卡在 blended
    self._reset_state_immediate()

  # ============================================================
  # 一般 AEM 行為
  # ============================================================
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
    - deadlock 期間固定 1.0（強制）
    - force_active 期間固定 1.0（強制）
    - 否則依淡入淡出曲線 w_in*w_out（先上升再下降）
    """
    t_now = time.monotonic()
    if not self._active:
      self._last_w = 0.0
      return 0.0

    if self._deadlock_active or self._force_active:
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
    0) [新增] deadlock 破除期間：強制 blended，直到滿足退出條件才解除
    1) FORCE_EXP 強制區間：直接 blended，離開立刻解除
    2) stopline 觸發後：先硬覆蓋 HARD_OVERRIDE_HOLD 秒
    3) 之後用權重 >= W_MODE_OVERRIDE 且 still in cooldown 來覆蓋
    """
    t_now = time.monotonic()

    # ------------------------------------------------------------
    # 0) Deadlock Breaker：最高優先
    # ------------------------------------------------------------
    if DEADLOCK_BREAKER_ENABLE and self._deadlock_active:
      # 若外部沒給 v_ego，就保守保持 blended（建議外部一定要傳 v_ego）
      if v_ego is None:
        return "blended"

      # 滿足退出條件才解除，否則一路 blended
      if self._can_exit_deadlock(v_ego, t_now):
        self._exit_deadlock()
      else:
        return "blended"

    # ------------------------------------------------------------
    # 1) FORCE_EXP：低速區間強制
    # ------------------------------------------------------------
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

    # ------------------------------------------------------------
    # 2) 觸發後硬覆蓋（快速切入，不等權重）
    # ------------------------------------------------------------
    if FAST_SWITCH_ENABLE and t_now < self._hard_override_end_time:
      return "blended"

    # ------------------------------------------------------------
    # 3) 權重覆蓋（淡入淡出）
    # ------------------------------------------------------------
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
    更新觸發狀態：
    0) [新增] deadlock breaker 判斷（必要時直接鎖 blended 並 return）
    1) FORCE_EXP 區間：進入就鎖定並 return（高優先）
    2) 非強制區間：檢查 stopline 判停條件，滿足後觸發 experimental
    """
    t_now = time.monotonic()
    _ = self._estimate_ego_accel(v_ego)
    v_ego_kph = v_ego * 3.6

    # ------------------------------------------------------------
    # 0) Deadlock Breaker：低速鎖死破除（hysteresis + hold）
    # ------------------------------------------------------------
    if DEADLOCK_BREAKER_ENABLE:
      # 若已在 deadlock，且滿足退出條件就解除；否則維持並跳過其他邏輯
      if self._deadlock_active:
        if self._can_exit_deadlock(v_ego, t_now):
          self._exit_deadlock()
        else:
          # deadlock 期間直接鎖 blended，避免其他邏輯干擾
          self._active = True
          return

      # 尚未在 deadlock：符合進入條件 + 已過 rearm 時間 才能進入
      # 這裡不看 radar lead（因為幽靈 lead 真假難辨），以「車速」作最可靠的保護
      if (v_ego < DEADLOCK_ENTER_MPS) and (t_now >= self._deadlock_rearm_until):
        self._enter_deadlock(t_now)
        return

    # ------------------------------------------------------------
    # 1) FORCE_EXP：低速區間強制鎖定（你原本的）
    # ------------------------------------------------------------
    if FORCE_EXP_ENABLE:
      in_force = (FORCE_EXP_KPH_MIN <= v_ego_kph <= FORCE_EXP_KPH_MAX)
      if in_force:
        if not self._force_active:
          self._set_force_active()
        return
      else:
        if self._force_active:
          self._reset_state_immediate()
        # 繼續往下走，允許 stopline 正常觸發

    # ------------------------------------------------------------
    # 2) stopline 判停（你原本的）
    # ------------------------------------------------------------
    # lead gate：避免 stopline 搶控（保留你原本功能，不新增任何 lead 相關「屏蔽/切ACC」邏輯）
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

    # 趨勢判斷：末段增量平均（收斂代表像要停）
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
