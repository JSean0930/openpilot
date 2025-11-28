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

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED.
"""

import time
import numpy as np
from openpilot.selfdrive.modeld.constants import ModelConstants


# ============================================================
# 可調參數集中區（人性化 AEM v2 + 低速跟車減速觸發）
# ============================================================

# --- 1) 觸發與解除時間/權重（平滑淡入淡出） -------------------
# AEM 觸發後的淡入/淡出時間（秒），會依速度再做比例放大
RAMP_UP_TIME_BASE = 0.25     # 觸發後多快「進入」AEM
RAMP_DOWN_TIME_BASE = 0.45   # 解除前多快「退出」AEM

# 速度自適應倍率（v 越高越保守、淡入淡出/冷卻越長）
RAMP_V_BP = [0.0, 6.0, 14.0, 25.0]         # m/s  ≈ 0/22/50/90 km/h
RAMP_V_MULT = [0.8, 1.0, 1.4, 1.8]         # 淡入淡出倍率

# 速度自適應 cooldown（秒）
COOLDOWN_V_BP = [0.0, 6.0, 14.0, 25.0]     # m/s
COOLDOWN_V_VALS = [0.30, 0.55, 0.80, 1.10] # 高速更久

# 超過此權重才「硬覆蓋」mode=blended（維持向下相容）
W_MODE_OVERRIDE = 0.60


# --- 2) 停止線/紅燈判停距離（S 型 / 兩段斜率更像人） ----------
# 速度 m/s -> 判停門檻距離 m
SLOW_DOWN_BP =   [0.0,  2.5,  5.5,  8.5,  11.5,  14.0,  17.0,  20.0,  25.0]
SLOW_DOWN_DIST = [8.0,  18.0, 35.0, 55.0, 70.0,  85.0, 105.0, 130.0, 165.0]


# --- 3) 連續幀確認 / 趨勢判斷 ---------------------------------
TRIGGER_CONFIRM_FRAMES = 3   # 連續幀數
TREND_K = 4                  # 看最後幾段 dx
TREND_DX_MAX = 0.80          # m/step，小於此視為「趨勢在收斂」

TRIGGER_MARGIN = 1.00        # 末端距離觸發裕度（<1 提早觸發）


# --- 4) 解除 hysteresis（避免反覆觸發抖動） -------------------
RELEASE_MARGIN = 1.20        # 解除判斷距離裕度
MIN_ACTIVE_TIME = 0.25       # s，至少 Active 這麼久


# --- 5) lead gate（有可信 lead 時不搶控「停止線觸發」） --------
LEAD_GATE_DIST = 45.0        # m
LEAD_GATE_VREL = -3.0        # m/s（> -3 表示沒有在很快逼近）


# --- 6) 新增：低速跟車減速觸發 AEM -----------------------------
# 有前車 + 車速 < 此值 + 正在減速，啟動實驗模式，讓減速更人性化
LOW_SPEED_LEAD_TRIG_KPH   = 30.0   # km/h 以下才啟動此額外條件
LOW_SPEED_LEAD_DIST_MAX   = 40.0   # m，以內視為「正在跟車」
LOW_SPEED_LEAD_DECEL_TRIG = -0.2   # m/s^2，低於此視為「正在減速中」
# 可依體感微調：
# - 想更容易觸發：提高 DIST_MAX、把 DECEL_TRIG 改得更接近 0（例如 -0.2）
# - 想更保守：降低 DIST_MAX、把 DECEL_TRIG 改成更負（例如 -0.6）


# ============================================================
# AEM v2
# ============================================================
class AEM:
  """
  人性化 AEM：
  - 用模型「末端路徑收斂 + 距離門檻」偵測停止線/紅燈
  - 連續幀確認 + 趨勢判斷避免誤觸
  - 速度自適應 cooldown
  - 權重淡入淡出（w_aem），並維持 get_mode 相容
  - 有可信 lead 時，停止線觸發不搶控（lead gate）
  - 新增：低速跟車 + 減速中 時，也可單獨觸發 AEM，讓跟車減速更柔順
  """

  def __init__(self):
    self._active = False
    self._trigger_cnt = 0
    self._start_time = 0.0
    self._cooldown_end_time = 0.0
    self._last_w = 0.0

    # 為了估計自車減速度，用前一幀速度與時間
    self._last_v_ego = None
    self._last_v_time = None

  # ---------------------------
  # 工具：安全取得 radar lead 資訊
  # ---------------------------
  def _get_lead_info(self, radar_msg):
    lead = getattr(radar_msg, "leadOne", None)
    if lead is None:
      return False, 1e9, 0.0   # no lead
    status = bool(getattr(lead, "status", False))
    d_rel = float(getattr(lead, "dRel", 1e9))
    v_rel = float(getattr(lead, "vRel", 0.0))
    return status, d_rel, v_rel

  # ---------------------------
  # 工具：估計自車加速度（依時間差計算 dv/dt）
  # ---------------------------
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

  # ---------------------------
  # 觸發 AEM（進入 active）
  # ---------------------------
  def _perform_experimental_mode(self, v_ego):
    t_now = time.monotonic()
    cooldown = float(np.interp(v_ego, COOLDOWN_V_BP, COOLDOWN_V_VALS))
    self._active = True
    self._start_time = t_now
    self._cooldown_end_time = t_now + cooldown

  # ---------------------------
  # 計算 AEM 權重（淡入淡出）
  # ---------------------------
  def get_weight(self, v_ego):
    t_now = time.monotonic()
    if not self._active:
      self._last_w = 0.0
      return 0.0

    # 速度倍率（高速淡入淡出更慢、更保守）
    v_mult = float(np.interp(v_ego, RAMP_V_BP, RAMP_V_MULT))
    ramp_up = RAMP_UP_TIME_BASE * v_mult
    ramp_dn = RAMP_DOWN_TIME_BASE * v_mult

    t_start = self._start_time
    t_end = self._cooldown_end_time

    # 淡入
    w_in = np.clip((t_now - t_start) / max(ramp_up, 1e-3), 0.0, 1.0)
    # 淡出
    w_out = np.clip((t_end - t_now) / max(ramp_dn, 1e-3), 0.0, 1.0)

    w = float(w_in * w_out)
    self._last_w = w
    return w

  # ---------------------------
  # 對外：取得 mode（相容原介面）
  # ---------------------------
  def get_mode(self, mode, v_ego=None):
    """
    維持原本的 get_mode(mode) 用法：
    - 權重 w_aem >= W_MODE_OVERRIDE 時，硬覆蓋 blended
    - 否則回傳外部 mode
    若外部願意更細緻混合，可另外讀 get_weight()
    """
    # 若外部沒給 v_ego，就用上一幀權重保底
    w = self._last_w if v_ego is None else self.get_weight(v_ego)

    if w >= W_MODE_OVERRIDE and time.monotonic() < self._cooldown_end_time:
      return "blended"

    # cooldown 過了才真正解除
    if time.monotonic() >= self._cooldown_end_time:
      self._active = False
      self._trigger_cnt = 0

    return mode

  # ---------------------------
  # 主更新：判斷是否觸發/維持
  # ---------------------------
  def update_states(self, model_msg, radar_msg, v_ego):
    """
    觸發條件（需連續 TRIGGER_CONFIRM_FRAMES 幀之一）：
      A) 停止線/紅燈情境：
         - 模型 position/orientation 長度正確
         - 末端距離 x_end < 門檻 * TRIGGER_MARGIN
         - 末段路徑增量趨勢收斂（平均 dx < TREND_DX_MAX）
         - lead gate 未擋住（有可信 lead 且情況穩定時，交給 lead）

      B) 低速跟車減速情境（新條件）：
         - 前方有 lead
         - v_ego < LOW_SPEED_LEAD_TRIG_KPH
         - 自車正在減速（a_ego < LOW_SPEED_LEAD_DECEL_TRIG）
         - lead 距離 < LOW_SPEED_LEAD_DIST_MAX

    解除條件：
      - active 時間 >= MIN_ACTIVE_TIME
      - 且 cooldown 時間到
      - 且末端距離 > 門檻 * RELEASE_MARGIN
    """

    # 估計自車加速度（m/s^2），用來判斷「是否在減速」
    a_ego = self._estimate_ego_accel(v_ego)
    v_ego_kph = v_ego * 3.6

    # -----------------------
    # 0) lead gate / lead 資訊
    # -----------------------
    lead_ok, d_rel, v_rel = self._get_lead_info(radar_msg)

    # 停止線觸發用的 lead gate：
    # 有可信 lead 且距離近、但相對速度沒在「快速逼近」時，避免 AEM 抢控停止線情境
    lead_gate_block = lead_ok and d_rel < LEAD_GATE_DIST and v_rel > LEAD_GATE_VREL

    # 新增：低速跟車減速觸發條件（不受 lead_gate_block 限制）
    low_speed_lead_trigger = (
      lead_ok and
      (v_ego_kph < LOW_SPEED_LEAD_TRIG_KPH) and
      (d_rel < LOW_SPEED_LEAD_DIST_MAX) and
      (a_ego < LOW_SPEED_LEAD_DECEL_TRIG)
    )

    # -----------------------
    # 1) 檢查模型輸出可用
    # -----------------------
    N = ModelConstants.IDX_N
    if not (len(model_msg.orientation.x) == len(model_msg.position.x) == N):
      self._trigger_cnt = 0
      return

    pos_x = np.asarray(model_msg.position.x)
    x_end = float(pos_x[N - 1])

    # -----------------------
    # 2) 計算速度對應門檻距離
    # -----------------------
    slow_dist = float(np.interp(v_ego, SLOW_DOWN_BP, SLOW_DOWN_DIST))
    trigger_th = slow_dist * TRIGGER_MARGIN
    release_th = slow_dist * RELEASE_MARGIN

    # -----------------------
    # 3) 趨勢判斷（末段平均增量）
    # -----------------------
    k = min(TREND_K, N - 1)
    dx_tail = np.diff(pos_x[-(k+1):])
    mean_dx_tail = float(np.mean(dx_tail)) if dx_tail.size else 1e9
    trend_ok = mean_dx_tail < TREND_DX_MAX

    # -----------------------
    # 4) 停止線觸發判斷（原邏輯）
    # -----------------------
    stopline_trigger_now = (x_end < trigger_th) and trend_ok and (not lead_gate_block)

    # 最終觸發：停止線情境 或 低速跟車減速情境
    trigger_now = stopline_trigger_now or low_speed_lead_trigger

    # -----------------------
    # 5) 觸發判定（連續幀確認）
    # -----------------------
    if not self._active:
      if trigger_now:
        self._trigger_cnt += 1
        if self._trigger_cnt >= TRIGGER_CONFIRM_FRAMES:
          self._perform_experimental_mode(v_ego)
          self._trigger_cnt = 0
      else:
        self._trigger_cnt = 0

    # -----------------------
    # 6) 解除 hysteresis（只允許在 cooldown 過後真正解除）
    # -----------------------
    if self._active:
      t_now = time.monotonic()

      # 未到最短 active 時間，不允許過早解除
      if (t_now - self._start_time) < MIN_ACTIVE_TIME:
        return

      # cooldown 到了，且末端距離回到較安全區才解除
      if t_now >= self._cooldown_end_time and x_end > release_th:
        self._active = False
        self._trigger_cnt = 0
