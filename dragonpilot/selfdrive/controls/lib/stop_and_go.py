"""
紅綠燈停走（Stop & Go）— humanlike 2026-11，取代 AEM 的「油門意圖借用」

問題（試車 1128 離線驗證，6.9 h 紀錄）：
- 舊規則用 gasPressProb 決定 acc / blended：市區幾乎一直借用 ACC，等油門意圖掉到 0.25 以下才切回，
  紅燈時晚 1~2 s 才開始減速、最後 −1.5~−2.5 急煞；而且一切回 blended 就照抄 e2e 的加速度，
  OP 時「模型把指令拉低、之後卻沒停」約 65 次/h（莫名煞車）。
- 模型的「速度軌跡」比單一瞬間的訊號可靠：
  - 軌跡會停（最低速 < 1.5 m/s）持續 0.5 s：誤報約 2 次/h，抓到 77% 的無前車停車
  - 軌跡最低速 < 現速 ×0.3 持續 0.5 s：只比你開始收油晚約 1 s，誤報約 5 次/h（誤報只會滑行）
  - 起步：軌跡 2 s 後速度 > 1 m/s 且模型沒說要停，持續 0.3 s：和你踩油門差不多同時
  - 你手動時停在模型預測停止點前約 1.1 m

做法（平常完全走 ACC/MPC，只在下列狀態介入，且永遠取 min(MPC, 本模組)，不會少煞車）：
  CRUISE   不介入
  COAST    軌跡明顯減速 → 只收油滑行（上限 = 自然滑行減速度），不煞車
  STOP     軌跡會停 → 依「模型停止點 − 1 m」算一條等減速的溫和曲線
  STOPPED  靜止、前方沒有近車 → 停住，直到模型說要走
  LAUNCH   起步後的幾秒內，要更強、更久的訊號才會再停（避免走了又停）
緊急路徑：模型要求明顯煞車（e2e < −1.0）且軌跡大幅減速時，不等持續時間，直接採用 e2e。
車速：80 km/h 以上完全不介入；60~80 km/h 只接受「軌跡會停持續 1 s」與緊急路徑。
前方有車且停止點在前車之後：交給 MPC（跟車停）與 LaunchFollow。
"""

import numpy as np

from dragonpilot.selfdrive.controls.lib.acm import coast_accel

KPH = 1.0 / 3.6

# 車速範圍
MIN_V = 10.0 * KPH            # 低於這速度不再判斷 COAST/STOP 進入（停車末段由 STOP 自己收尾）
LIMITED_V = 60.0 * KPH        # 60~80 km/h：只接受強訊號
MAX_V = 80.0 * KPH            # 80 km/h 以上：完全不介入

# 訊號門檻
COAST_VMIN_FRAC = 0.3         # 軌跡最低速 < 現速 × 此值 → 收油
COAST_EXIT_FRAC = 0.5         # 軌跡最低速 > 現速 × 此值 → 解除收油
STOP_VMIN = 1.5               # 軌跡最低速 < 此值 (m/s) → 會停
STOP_EXIT_VMIN = 3.0          # 軌跡最低速 > 此值 → 模型改變主意（例如轉綠燈）
GO_V2 = 1.0                   # 軌跡 2 s 後速度 > 此值 → 要走
URGENT_E2E = -1.0             # 緊急路徑：e2e 要求的減速度

# 持續時間 (s)
T_COAST = 0.5
T_STOP = 0.5
T_STOP_LIMITED = 1.0          # 60~80 km/h
T_STOP_AFTER_LAUNCH = 1.0     # 起步後再停需要更久
T_EXIT = 0.5
T_GO = 0.3
LAUNCH_GUARD_T = 6.0          # 起步後這段時間內用較嚴格的再停門檻
LAUNCH_GUARD_V = 5.0          # 或車速超過此值 (m/s) 後解除

# 停車曲線
STOP_MARGIN = 1.0             # m，停在模型停止點前
STOP_TARGET_TAU = 1.0         # s，停止點估計的平滑時間常數
STOP_FREEZE_V = 2.0           # m/s，低於此速度不再更新停止點（模型末段估計不穩）
MIN_STOP_DIST = 0.3           # m
T_LAG = 0.5                   # s，致動落後：用目前加速度預測這段時間後的位置/速度再算（OP 實測多衝 0.8 m）
A_STOP_MIN = -2.5             # 自己的曲線最多煞到這裡（更重的需求交給 MPC / e2e）
E2E_OVERRIDE_MARGIN = 0.8     # STOP 中 e2e 比曲線重這麼多且 < −1.5 → 採用 e2e（多半是障礙物）
E2E_OVERRIDE_MIN = -1.5
STOPPED_V = 0.3
STOPPING_V = 0.5              # STOP 中低於此速度送 should_stop

# 前車
LEAD_STOP_BUFFER = 5.0        # m：前車距離 < 停止點 + 此值 → 是跟車停，交給 MPC
LEAD_NEAR_STOPPED = 20.0      # m：靜止時前方這距離內有車 → 交給 MPC / LaunchFollow
LEAD_COAST_RANGE = 80.0       # m：收油判斷時，這距離內有較慢前車就交給 ACM
LEAD_FOLLOW_RANGE = 20.0      # m：收油判斷時，這距離內有前車（正在跟車）也交給 ACM
LEAD_GO_V = 1.0               # m/s：停住時前車以此速度離開、且模型沒說要停 → 跟著起步

# 平順化
JERK_DOWN = 2.0               # m/s³，介入時指令往下的速率上限
JERK_UP = 1.5                 # m/s³，解除時指令往上的速率上限
JERK_UP_LAUNCH = 3.0          # m/s³，起步時（與 LaunchFollow 相同）
JERK_DOWN_URGENT = 5.0        # m/s³，緊急路徑

CRUISE, COAST, STOP, STOPPED, LAUNCH = 'cruise', 'coast', 'stop', 'stopped', 'launch'


def model_features(t_idxs, v_traj, x_traj):
  """由模型速度/位置軌跡取出：最低速、2 s 後速度、停止點距離（沒有要停 → None）。"""
  v = np.asarray(v_traj, dtype=float)
  if len(v) < 2:
    return None
  t = np.asarray(t_idxs[:len(v)], dtype=float)
  x = np.asarray(x_traj[:len(v)], dtype=float)
  vmin = float(v.min())
  v2 = float(np.interp(2.0, t, v))
  idx = np.flatnonzero(v < 0.5)
  x_stop = float(x[idx[0]]) if len(idx) else None
  return vmin, v2, x_stop


class _Timer:
  def __init__(self):
    self.t = 0.0

  def update(self, cond, dt):
    self.t = self.t + dt if cond else 0.0
    return self.t

  def reset(self):
    self.t = 0.0


class StopAndGo:
  def __init__(self, dt=0.05):
    self.dt = dt
    self.reset()

  def reset(self):
    self.state = CRUISE
    self.odo = 0.0
    self.x_stop_world = None
    self.launch_t = 0.0
    self.a_last = None
    self.capped = False
    self.t_nogo = _Timer()
    self.t_coast = _Timer()
    self.t_coast_exit = _Timer()
    self.t_stop = _Timer()
    self.t_stop_exit = _Timer()
    self.t_go = _Timer()

  @property
  def holding(self):
    return self.state == STOPPED

  @property
  def active(self):
    return self.state in (COAST, STOP, STOPPED)

  def _lead_owns_stop(self, lead, x_stop):
    if lead is None or not lead.status:
      return False
    if x_stop is None:
      return lead.dRel < LEAD_COAST_RANGE and (lead.vLead < 3.0 or lead.dRel < LEAD_FOLLOW_RANGE)
    return lead.dRel < x_stop + LEAD_STOP_BUFFER

  def _stop_target(self, x_stop, v_ego):
    """世界座標下的停車點（平滑、低速凍結），回傳剩餘距離。"""
    if x_stop is not None and (self.x_stop_world is None or v_ego > STOP_FREEZE_V):
      target = self.odo + x_stop - STOP_MARGIN
      if self.x_stop_world is None:
        self.x_stop_world = target
      else:
        alpha = self.dt / (STOP_TARGET_TAU + self.dt)
        self.x_stop_world += alpha * (target - self.x_stop_world)
    return self.x_stop_world - self.odo

  def update(self, v_ego, a_mpc, should_stop_mpc, e2e_accel, e2e_should_stop, feats, lead, a_ego=0.0):
    """回傳 (a_target, should_stop, used_e2e)。永遠 a_target <= a_mpc。"""
    dt = self.dt
    self.odo += v_ego * dt
    if feats is None:
      self.state = CRUISE
      return self._smooth(a_mpc, a_mpc), should_stop_mpc, False
    vmin, v2, x_stop = feats

    standstill = v_ego < STOPPED_V
    lead_near = lead is not None and lead.status and lead.dRel < LEAD_NEAR_STOPPED
    go = v2 > GO_V2 and not e2e_should_stop
    t_go = self.t_go.update(go, dt)
    t_nogo = self.t_nogo.update(not go, dt)

    # ---- 靜止 ----
    if standstill:
      if lead_near and self.state != STOPPED:
        # 跟車停：交給 MPC / LaunchFollow
        if self.state != LAUNCH:
          self.state = CRUISE
      elif self.state == LAUNCH and t_nogo < T_STOP_AFTER_LAUNCH:
        pass  # 已決定起步、車還沒動（致動延遲）：除非模型持續改口，否則不再停住
      elif self.state == STOPPED or not lead_near:
        if self.state != STOPPED:
          self.state = STOPPED
          self.t_go.reset()
          t_go = 0.0
        lead_leaving = lead_near and lead.vLead > LEAD_GO_V and not e2e_should_stop and v2 > 0.3
        if t_go >= T_GO or lead_leaving:
          self._start_launch()
        else:
          self.a_last = min(a_mpc, 0.0)
          self.capped = self.a_last < a_mpc - 1e-6
          return self.a_last, True, False
    elif self.state == STOPPED:
      self._start_launch()

    if self.state == LAUNCH:
      self.launch_t += dt
      if self.launch_t > LAUNCH_GUARD_T or v_ego > LAUNCH_GUARD_V:
        self.state = CRUISE

    # ---- 車速範圍 ----
    if v_ego > MAX_V:
      self._to_cruise()
      return self._smooth(a_mpc, a_mpc), should_stop_mpc, False
    limited = v_ego > LIMITED_V

    lead_owns = self._lead_owns_stop(lead, x_stop)
    stop_sig = vmin < STOP_VMIN and x_stop is not None and not lead_owns
    t_need = T_STOP_LIMITED if limited else (T_STOP_AFTER_LAUNCH if self.state == LAUNCH else T_STOP)
    t_stop = self.t_stop.update(stop_sig and v_ego > MIN_V, dt)
    coast_sig = vmin < COAST_VMIN_FRAC * v_ego and not lead_owns and not limited
    t_coast = self.t_coast.update(coast_sig and v_ego > MIN_V, dt)
    urgent = (e2e_accel < URGENT_E2E and vmin < COAST_VMIN_FRAC * v_ego and not lead_owns)

    # ---- 狀態轉移 ----
    if self.state in (CRUISE, LAUNCH, COAST):
      if t_stop >= t_need or (urgent and x_stop is not None):
        self.state = STOP
        self.x_stop_world = None
        self.t_stop_exit.reset()
      elif self.state != COAST and self.state != LAUNCH and t_coast >= T_COAST:
        self.state = COAST
        self.t_coast_exit.reset()
    if self.state == COAST:
      if self.t_coast_exit.update(vmin > COAST_EXIT_FRAC * v_ego or lead_owns, dt) >= T_EXIT:
        self._to_cruise()
    elif self.state == STOP:
      if self.t_stop_exit.update(vmin > STOP_EXIT_VMIN or lead_owns, dt) >= T_EXIT:
        self._to_cruise()

    # ---- 輸出 ----
    used_e2e = False
    should_stop = should_stop_mpc
    if self.state == COAST:
      target = min(a_mpc, coast_accel(v_ego))
      if urgent:
        target = min(target, e2e_accel)
        used_e2e = True
      out = self._smooth(target, a_mpc, JERK_DOWN_URGENT if urgent else JERK_DOWN)
    elif self.state == STOP:
      d = self._stop_target(x_stop, v_ego)
      a_now = min(a_ego, 0.0)
      v_p = max(v_ego + a_now * T_LAG, 0.0)
      d_p = max(d - (v_ego * T_LAG + 0.5 * a_now * T_LAG ** 2), MIN_STOP_DIST)
      a_prof = max(-v_p ** 2 / (2.0 * d_p), A_STOP_MIN)
      target = min(a_mpc, a_prof, coast_accel(v_ego))
      if e2e_accel < min(a_prof - E2E_OVERRIDE_MARGIN, E2E_OVERRIDE_MIN):
        target = min(target, e2e_accel)
        used_e2e = True
      out = self._smooth(target, a_mpc, JERK_DOWN_URGENT if (used_e2e or urgent) else JERK_DOWN)
      if v_ego < STOPPING_V:
        should_stop = True
    else:
      if urgent:
        out = self._smooth(min(a_mpc, e2e_accel), a_mpc, JERK_DOWN_URGENT)
        used_e2e = True
      else:
        out = self._smooth(a_mpc, a_mpc)
    return out, should_stop, used_e2e

  def _start_launch(self):
    self.state = LAUNCH
    self.launch_t = 0.0
    self.t_stop.reset()
    self.x_stop_world = None

  def _to_cruise(self):
    self.state = CRUISE
    self.x_stop_world = None

  def _smooth(self, target, a_mpc, jerk_down=JERK_DOWN):
    """介入時限制往下的速率；解除時從目前值以 JERK_UP 慢慢回到 MPC。
    永遠不比 MPC 多油門；MPC 自己更重的煞車直接通過，不受平滑影響。"""
    if self.a_last is None:
      self.a_last = min(target, a_mpc)
      return self.a_last
    jerk_up = JERK_UP_LAUNCH if self.state == LAUNCH else JERK_UP
    if target >= a_mpc:
      # 不介入：只有先前壓低過才需要慢慢放開，否則 MPC 原樣通過
      out = min(a_mpc, self.a_last + jerk_up * self.dt) if self.capped else a_mpc
    else:
      out = float(np.clip(target, self.a_last - jerk_down * self.dt, self.a_last + jerk_up * self.dt))
      out = min(out, a_mpc)
    self.capped = out < a_mpc - 1e-6
    self.a_last = out
    return out
