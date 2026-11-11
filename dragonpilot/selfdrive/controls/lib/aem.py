import numpy as np
from openpilot.common.params import Params

# ============================================================
# 可調參數區
# ============================================================
# [功能 A: 宏觀車速切換] (單位: km/h)
EXPERIMENTAL_ENABLE_SPEED = 60.0   # 低於此速度：開啟 Experimental Mode (切換至 Blended)
EXPERIMENTAL_DISABLE_SPEED = 70.0  # 高於此速度：關閉 Experimental Mode (切換至純 ACC)

# [功能 B: 微觀意圖借用]
THROTTLE_ACC_PROB = 0.45           # 神經網路強烈想加速時 (>= 0.55)，瞬間借用 ACC

class AEM:
  def __init__(self):
    # --- 意圖判定所需變數 ---
    self._throttle_prob = 1.0
    self.v_ego = 0.0 

    # --- 底層硬切換所需變數 ---
    self.params = Params()
    # 啟動時先讀取目前的系統開關狀態，避免一開機就盲目寫入
    self._is_experimental_on = self.params.get_bool("ExperimentalMode")

  def _set_experimental_toggle(self, enable: bool):
    """安全寫入防護：只有在狀態真正需要改變時，才執行寫入硬碟的動作"""
    if self._is_experimental_on != enable:
      self.params.put_bool("ExperimentalMode", enable)
      self._is_experimental_on = enable

  def update_states(self, model_msg, radar_msg, v_ego):
    self.v_ego = v_ego
    v_kph = v_ego * 3.6

    # ==========================================
    # 任務 1: 執行「宏觀車速」緩衝區切換邏輯
    # ==========================================
    if v_kph < EXPERIMENTAL_ENABLE_SPEED:
      target_state = True
    elif v_kph > EXPERIMENTAL_DISABLE_SPEED:
      target_state = False
    else:
      target_state = self._is_experimental_on

    # 執行物理狀態切換 (內部有防護機制，不會瘋狂寫入)
    self._set_experimental_toggle(target_state)

    # ==========================================
    # 任務 2: 讀取神經網路油門意圖，供後續動態借用
    # ==========================================
    probs = model_msg.meta.disengagePredictions.gasPressProbs
    self._throttle_prob = probs[1] if len(probs) > 1 else 1.0

  def get_mode(self, mode):
    # =========================================================
    # 任務 3: 執行「微觀意圖」動態 AEM 門檻借用 (Dynamic Threshold)
    # v_ego = 0.0 m/s (靜止時)：門檻降至 0.30 (起步極度敏銳，不猶豫)
    # v_ego = 5.0 m/s (18km/h)：門檻升至 0.50 (提早啟動煞車，極度平順)
    # =========================================================
    dynamic_blended_prob = float(np.interp(self.v_ego, [0.0, 5.0], [0.30, 0.50]))

    # 註：此處傳入的 mode，已經是由上面 _set_experimental_toggle 決定好的基礎狀態
    if mode == 'blended':  
      # 基礎為 Blended (市區)：神經網路覺得前方淨空，瞬間借用 ACC 彈射起步
      return 'acc' if self._throttle_prob >= THROTTLE_ACC_PROB else 'blended'
    
    # 基礎為 ACC (高速)：神經網路發現前方塞車/紅燈，瞬間借用 Blended 柔和煞車
    return 'blended' if self._throttle_prob <= dynamic_blended_prob else 'acc'

