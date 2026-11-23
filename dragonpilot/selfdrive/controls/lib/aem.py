import numpy as np

# ============================================================
# Adaptive Experimental Mode (AEM)
#
# 只在 planner 內部決定縱向模式 ('acc' / 'blended')，**不寫入 ExperimentalMode 參數**：
#   - 不會覆蓋使用者的開關、不落盤、UI 不會跟著跳
#   - 同一 frame 就生效（不必等 selfdriveState.experimentalMode 回傳）
#
# 兩層決策：
#   1. 基礎模式 (get_base_mode)：AEM_SPEED_BASED_BASE=True 時依車速遲滯決定
#      (< 60 km/h → blended、> 70 km/h → acc)；False 時沿用使用者的 Experimental 開關。
#   2. 意圖借用 (get_mode)：依模型 gasPressProb 暫時借用另一模式，含遲滯避免逐 frame 來回切。
# 模式切換造成的 a_target 差異由 planner 端平滑交叉淡化 (crossfade) 處理。
# ============================================================

# [功能 A: 宏觀車速切換] (單位: km/h)
AEM_SPEED_BASED_BASE = True
EXPERIMENTAL_ENABLE_SPEED = 60.0   # 低於此速度：基礎模式 blended
EXPERIMENTAL_DISABLE_SPEED = 70.0  # 高於此速度：基礎模式 acc

# [功能 B: 微觀意圖借用]
THROTTLE_ACC_PROB = 0.45           # blended 基礎：gasPressProb >= 此值 → 借用 ACC
THROTTLE_BLENDED_PROB_BP = [0.0, 5.0]    # m/s
THROTTLE_BLENDED_PROB_V = [0.30, 0.50]   # acc 基礎：gasPressProb <= 此值 → 借用 BLENDED
THROTTLE_BLENDED_PROB = THROTTLE_BLENDED_PROB_V[0]  # 相容舊名稱（靜止時門檻）
BORROW_HYST = 0.05                 # 借用後需反向超過門檻 ± 此值才解除


class AEM:
  def __init__(self):
    self._throttle_prob = 1.0
    self.v_ego = 0.0
    self._speed_base_blended = True   # 車速遲滯狀態（開機多半是低速）
    self._last_base = None
    self._borrowing = False

  def update_states(self, model_msg, radar_msg, v_ego):
    self.v_ego = v_ego
    v_kph = v_ego * 3.6

    if v_kph < EXPERIMENTAL_ENABLE_SPEED:
      self._speed_base_blended = True
    elif v_kph > EXPERIMENTAL_DISABLE_SPEED:
      self._speed_base_blended = False
    # 介於兩者之間：維持原狀態（遲滯）

    probs = model_msg.meta.disengagePredictions.gasPressProbs
    self._throttle_prob = probs[1] if len(probs) > 1 else 1.0

  def get_base_mode(self, user_mode):
    if not AEM_SPEED_BASED_BASE:
      return user_mode
    return 'blended' if self._speed_base_blended else 'acc'

  def get_mode(self, mode):
    if mode != self._last_base:
      self._borrowing = False
      self._last_base = mode

    if mode == 'blended':
      # blended 基礎（市區）：模型明確想給油時借用 ACC
      on_thresh = THROTTLE_ACC_PROB
      off_thresh = THROTTLE_ACC_PROB - BORROW_HYST
      self._borrowing = self._throttle_prob >= (off_thresh if self._borrowing else on_thresh)
      return 'acc' if self._borrowing else 'blended'

    # acc 基礎（高速）：模型明確想收油/減速時借用 BLENDED
    blended_prob = float(np.interp(self.v_ego, THROTTLE_BLENDED_PROB_BP, THROTTLE_BLENDED_PROB_V))
    on_thresh = blended_prob
    off_thresh = blended_prob + BORROW_HYST
    self._borrowing = self._throttle_prob <= (off_thresh if self._borrowing else on_thresh)
    return 'blended' if self._borrowing else 'acc'
