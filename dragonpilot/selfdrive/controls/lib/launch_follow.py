"""
Launch Follow（起步跟隨）— humanlike 2026-11

問題（試車 53~62，25 次 OP 起步，從前車最後靜止的時刻起算，中位數）：
            決定要走     力道到位(>0.5)   車動
  你        0.25 s 油門  踩下即到位       1.15 s
  OP        0.65 s       1.10 s           1.95 s
「力道到位 → 車動」兩者都約 0.8 s，車端並不慢；落後的是 OP 決定得晚（0.4 s），
以及決定後 MPC 的加速度要 0.45 s 才從 0 爬到 0.5。

你自己起步的力道（動後 1 s 平均加速度 P50）會跟著前車走：
  前車蠕動（3 s 內 < 7 km/h）0.56、前車慢起 0.84、正常起步 1.42
所以力道不設下限，完全依前車給；上升與下降都限制 jerk，避免塞車走走停停時「衝一下又縮」的點頭。
（v1 用固定下限 0.6，重播蠕動情境指令衝到 1.08、結束 0.25 s 內又掉 0.57 → 會點頭，已捨棄）

做法（只在 MPC 也同意要走時才介入，不靠雷達單獨判斷，避免前車蠕動時誤起步）：
- 觸發：本車靜止、前車確實在離開（雷達速度 > 0.35 m/s、距離比停住時多 0.1 m），
  且 MPC 已放開 shouldStop 或目標加速度已轉正（> 0.02）
- 力道 = K_A × 前車加速度 + K_V × (前車速度 − 本車速度)，夾在 [0, 1.3]
- 力道上升限制 3 m/s³、收回限制 1.5 m/s³，從 MPC 當下的值開始爬，不會跳階
- 輸出永遠 >= MPC（不會比原本少加速）

交還 MPC：
- 立刻：前車消失、距離 < 1.5 m、前車急煞（< -1.0）、e2e 要停（blended 模式下紅燈等）
- 漸進（力道以 1.5 m/s³ 降回 MPC）：本車追上前車速度、前車輕煞、超過 11 km/h、介入超過 4 s
"""

import numpy as np

TRIGGER_V_LEAD = 0.35     # m/s（0.3 時重播有 3 次前車只蠕動就起步，0.35 為 2 次）
TRIGGER_D_GROW = 0.1      # m，距離比停住時多這麼多
MPC_AGREE_ACCEL = 0.02    # m/s²，MPC 目標已轉正視為同意
STOPPED_V_EGO = 0.1       # m/s
STOPPED_V_LEAD = 0.15     # m/s，前車低於此速度視為靜止，更新停車距離
MIN_GAP = 1.5             # m
SOFT_A_LEAD = -0.5        # m/s²，前車輕煞：漸進交還
ABORT_A_LEAD = -1.0       # m/s²，前車急煞：立刻交還
EXIT_V_EGO = 3.0          # m/s
MAX_TIME = 4.0            # s

K_A = 0.7
K_V = 0.5
A_MAX = 1.3
JERK_UP = 3.0             # m/s³，力道上升的最大變化率
JERK_DOWN = 1.5           # m/s³，力道收回的最大變化率（3.0 時重播蠕動情境 0.25 s 內最多掉 0.44）


class LaunchFollow:
  def __init__(self, dt=0.05):
    self.dt = dt
    self.reset()

  def reset(self):
    self.active = False
    self.releasing = False
    self.d_stop = None
    self.t_active = 0.0
    self.a_cmd = 0.0

  @staticmethod
  def _abort(lead, e2e_stop):
    return lead is None or not lead.status or e2e_stop or lead.dRel < MIN_GAP or lead.aLeadK < ABORT_A_LEAD

  def _release(self, v_ego, lead):
    return (lead.vLead <= v_ego + 0.1 or lead.aLeadK < SOFT_A_LEAD or
            v_ego > EXIT_V_EGO or self.t_active > MAX_TIME)

  def _step(self, target):
    self.a_cmd = float(np.clip(target, self.a_cmd - JERK_DOWN * self.dt, self.a_cmd + JERK_UP * self.dt))

  def update(self, v_ego, lead, a_target, should_stop, e2e_stop=False):
    """回傳 (a_target, should_stop)。輸出加速度永遠 >= MPC 的 a_target。"""
    has_lead = lead is not None and lead.status

    if not self.active and not self.releasing:
      if v_ego < STOPPED_V_EGO and has_lead:
        if lead.vLead < STOPPED_V_LEAD:
          # 前車靜止：記住停車時的距離（濾波，吃掉雷達抖動）
          self.d_stop = lead.dRel if self.d_stop is None else 0.8 * self.d_stop + 0.2 * lead.dRel
        elif self.d_stop is not None:
          leaving = (lead.vLead > TRIGGER_V_LEAD and lead.dRel > self.d_stop + TRIGGER_D_GROW and
                     lead.aLeadK > SOFT_A_LEAD and lead.dRel > MIN_GAP and not e2e_stop)
          mpc_agrees = (not should_stop) or a_target > MPC_AGREE_ACCEL
          if leaving and mpc_agrees:
            self.active = True
            self.t_active = 0.0
            self.a_cmd = max(a_target, 0.0)
      else:
        self.d_stop = None
      if not self.active:
        return a_target, should_stop

    if self._abort(lead, e2e_stop):
      self.reset()
      return a_target, should_stop

    if self.active:
      self.t_active += self.dt
      if self._release(v_ego, lead):
        self.active = False
        self.releasing = True
      else:
        a_raw = K_A * max(lead.aLeadK, 0.0) + K_V * max(lead.vLead - v_ego, 0.0)
        self._step(float(np.clip(a_raw, 0.0, A_MAX)))
        return max(a_target, self.a_cmd), False

    # 漸進交還：力道以 JERK_DOWN 降回 MPC；MPC 要停就交給 longcontrol 的停車邏輯
    self._step(a_target)
    if should_stop or self.a_cmd <= a_target:
      self.reset()
      return a_target, should_stop
    return max(a_target, self.a_cmd), should_stop
