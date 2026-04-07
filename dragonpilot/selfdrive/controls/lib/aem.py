import time

class AEM:
  def __init__(self):
    # 記錄車輛是否處於「剛靜止準備起步」的狀態
    self._is_starting_up = False
    # 決定當下是否要強制輸出 'acc'
    self._force_acc = False

  def get_mode(self, mode):
    # 如果符合我們設定的條件，強制覆蓋為 'acc'
    if self._force_acc:
      return 'acc'
    # 其餘狀況維持當下傳入的原有 mode
    return mode

  def update_states(self, model_msg, radar_msg, v_ego):
    v_kph = v_ego * 3.6

    # ==========================================
    # 條件 1：起步階段 (0 -> 3 km/h，僅加速觸發)
    # ==========================================
    # 為了達到「減速不觸發，加速才觸發」，我們用極低速(<= 0.1 km/h)來判定車輛已停止
    if v_kph <= 0.1:
      self._is_starting_up = True
    # 當時速超過 3.0 km/h，起步階段結束，解除狀態
    elif v_kph > 3.0:
      self._is_starting_up = False

    # ==========================================
    # 條件 2：高速階段 (> 60 km/h)
    # ==========================================
    is_high_speed = (v_kph > 30.0)

    # ==========================================
    # 綜合判斷與狀態更新
    # ==========================================
    # 只要處於「剛停止到起步 3km/h 內」或「時速 > 60km/h」，就強制切為 acc
    self._force_acc = self._is_starting_up or is_high_speed


