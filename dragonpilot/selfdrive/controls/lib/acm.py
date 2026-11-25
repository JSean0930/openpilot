"""
Copyright (c) 2025, Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, and/or sublicense,
for non-commercial purposes only, subject to the following conditions:

- The above copyright notice and this permission notice shall be included in
  all copies or substantial portions of the Software.
- Commercial use (e.g. use in a product, service, or activity intended to
  generate revenue) is prohibited without explicit written permission from
  the copyright holder.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

ACM v2（人性滑行）— 依駕駛者手動資料重寫，2026-11

舊版的兩個問題：
1. self.enabled 從未被設成 True，模組永遠不作用。
2. 它只改 a_desired_trajectory；但 get_accel_from_plan 的輸出是
   2*(v_target - v_now)/t - a_now，抬高 a_now 反而讓煞車更重。

新版直接作用在 MPC 的輸出加速度上（與 e2e 混合之前），只會「放軟」：
- 不會比 MPC 多煞，也不會比 MPC 多加速。
- e2e（紅燈、停止線等）的煞車不受影響，因為 planner 之後仍取 min(e2e, mpc)。

兩種情境：
A. 接近較慢的前車：依物理算出「等速減速剛好在使用者習慣車距前追平」所需的減速度，
   並且至少用自然滑行的減速度。MPC 要的煞車比這個重時才放軟。
   接近中且 TTC < LIFT_OFF_TTC 時不再加油（收油）。
B. 稍微超過設定速度、前方沒有要處理的車：只用自然滑行，不額外踩煞車。
"""

import math
import numpy as np

# ---- 使用者手動資料（routes 53~5b，手動 113 分）----
# 放油門、不踩煞車時的自然減速度 P50（車速 m/s）
COAST_BP = [0.0, 8.3, 11.1, 30.0]
COAST_V = [-0.10, -0.15, -0.32, -0.40]
# 使用者穩態跟車：車距 ≈ 0.68 s × 車速 + 1.3 m；放軟時最多讓車距縮到這裡（另加 0.7 m 餘裕）
USER_T_FOLLOW = 0.68
USER_D0 = 2.0
# 使用者在 TTC 中位數 10.7 s 放油門
LIFT_OFF_TTC = 11.0

# ---- 安全條件：任一不成立就不介入 ----
MIN_V_EGO = 5.0           # m/s，18 km/h 以下交給 MPC（停走已另外調過）
MIN_V_LEAD = 3.0          # m/s，前車靜止或幾乎靜止交給 MPC
MIN_TTC = 4.0             # s，使用者 TTC 3~5 s 時 67% 已在踩煞車
MIN_A_LEAD = -1.0         # m/s²，前車在煞車就不放軟
MAX_MPC_BRAKE = -1.5      # m/s²，MPC 要比這個更重的煞車就不放軟
MIN_GAP_MARGIN = 1.0      # m，離使用者習慣車距少於這個就不放軟
BRAKE_MARGIN = 1.1        # 物理需求的減速度再乘上這個倍數
OVER_CRUISE_MAX = 2.5     # m/s，超過設定速度 9 km/h 以內才只用滑行

# 介入權重的變化率 (1/s)：進入 0.5 s、退出 0.2 s（退出＝恢復 MPC 煞車，要快）
RATE_IN = 2.0
RATE_OUT = 5.0

SOURCE_LEAD = ('lead0', 'lead1')
SOURCE_CRUISE = 'cruise'


def coast_accel(v_ego):
  return float(np.interp(v_ego, COAST_BP, COAST_V))


def pitch_coast_accel(pitch):
  # 與 longitudinal_planner.get_coast_accel 相同的擬合；上坡更負、下坡較不負
  return math.sin(pitch) * -5.65 - 0.3


class ACM:
  def __init__(self, dt=0.05):
    self.dt = dt
    self.weight = 0.0
    self.active = False
    self.mode = ''        # 'lead' / 'cruise' / ''
    self.a_floor = 0.0
    self.a_ceil = 0.0
    self.a_soft_last = 0.0

  def reset(self):
    self.weight = 0.0
    self.active = False
    self.mode = ''

  def _lead_target(self, a_mpc, v_ego, lead):
    """情境 A：回傳 (floor, ceil) 或 None。"""
    if lead is None or not lead.status:
      return None
    d, v_lead, a_lead = lead.dRel, lead.vLead, lead.aLeadK
    if v_ego < MIN_V_EGO or v_lead < MIN_V_LEAD or a_lead < MIN_A_LEAD or a_mpc < MAX_MPC_BRAKE:
      return None
    v_rel = v_lead - v_ego                     # < 0 表示正在接近
    gap = d - (USER_T_FOLLOW * v_lead + USER_D0)
    if gap < MIN_GAP_MARGIN:
      return None

    a_lead_safe = min(a_lead, 0.0)             # 不指望前車會加速
    if v_rel < -0.1:
      ttc = d / -v_rel
      if ttc < MIN_TTC:
        return None
      b_req = v_rel ** 2 / (2.0 * gap)
      floor = min(coast_accel(v_ego), a_lead_safe - BRAKE_MARGIN * b_req)
      ceil = 0.0 if (ttc < LIFT_OFF_TTC and a_lead < 0.3) else math.inf
    else:
      # 沒在接近：拉開距離只用滑行
      floor = coast_accel(v_ego) + a_lead_safe
      ceil = math.inf
    return floor, ceil

  def _cruise_target(self, v_ego, v_cruise, lead, pitch):
    """情境 B：稍微超過設定速度時只滑行。"""
    if v_ego < MIN_V_EGO or not (v_cruise < v_ego < v_cruise + OVER_CRUISE_MAX):
      return None
    if lead is not None and lead.status:
      v_rel = lead.vLead - v_ego
      if v_rel < -0.1 and lead.dRel / -v_rel < LIFT_OFF_TTC:
        return None
    floor = min(pitch_coast_accel(pitch), 0.0)
    return floor, math.inf

  def update(self, a_mpc, v_ego, v_cruise, lead, source, pitch=0.0):
    """回傳放軟後的 MPC 加速度。只會介於 a_mpc 與 a_mpc 的較溫和值之間。"""
    target = None
    mode = ''
    if a_mpc < MAX_MPC_BRAKE:
      # MPC 要重煞：立刻完全交還
      self.weight = 0.0
      self.mode = ''
      self.active = False
      return float(a_mpc)
    if source in SOURCE_LEAD:
      target = self._lead_target(a_mpc, v_ego, lead)
      mode = 'lead'
    elif source == SOURCE_CRUISE:
      target = self._cruise_target(v_ego, v_cruise, lead, pitch)
      mode = 'cruise'

    if target is None:
      # 退出：從上一次的放軟值在 1/RATE_OUT 秒內回到 MPC
      self.weight = max(0.0, self.weight - RATE_OUT * self.dt)
      # 夾在 0 與 a_mpc 之間：退出途中也不會比 MPC 多煞或多加速
      a_soft = float(min(max(self.a_soft_last, min(a_mpc, 0.0)), max(a_mpc, 0.0)))
      if self.weight == 0.0:
        self.mode = ''
    else:
      self.a_floor, self.a_ceil = target
      self.weight = min(1.0, self.weight + RATE_IN * self.dt)
      self.mode = mode
      # floor <= 0、ceil 為 0 或 inf，所以 a_mpc <= 0 時結果仍 <= 0（不會變成加速）
      a_soft = min(max(a_mpc, self.a_floor), self.a_ceil)
      self.a_soft_last = a_soft

    self.active = target is not None
    return float(a_mpc + self.weight * (a_soft - a_mpc))
