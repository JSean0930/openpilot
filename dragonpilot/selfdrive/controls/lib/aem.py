import numpy as np

THROTTLE_ACC_PROB     = 0.6  # gasPressProb >= this -> model wants throttle -> ACC
# 原本死板的 0.4 我們不用了，我們在函數內動態生成！

class AEM:
  def __init__(self):
    self._throttle_prob = 1.0
    self.v_ego = 0.0  # 🌟 新增：用來記憶當前車速

  def update_states(self, model_msg, radar_msg, v_ego):
    self.v_ego = v_ego  # 🌟 每次更新時，把車速存起來
    
    # Probability the model wants to be on throttle
    probs = model_msg.meta.disengagePredictions.gasPressProbs
    self._throttle_prob = probs[1] if len(probs) > 1 else 1.0

  def get_mode(self, mode):
    
    # =========================================================
    # 🧠 魔改核心：動態 AEM 門檻 (Dynamic Threshold)
    # v_ego = 0.0 m/s (靜止時)：門檻降至 0.30 (極速解鎖起步，不猶豫)
    # v_ego = 5.0 m/s (18km/h)：門檻升至 0.50 (提早啟動煞車，極度平順)
    # =========================================================
    dynamic_blended_prob = float(np.interp(self.v_ego, [0.0, 5.0], [0.30, 0.50]))

    if mode == 'blended':  # blended-first
      return 'acc' if self._throttle_prob >= THROTTLE_ACC_PROB else 'blended'
    
    # acc-first: 使用我們魔改的動態門檻！
    return 'blended' if self._throttle_prob <= dynamic_blended_prob else 'acc'

