"""
Copyright (c) 2026

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction... (保留你的版權聲明)
"""

from openpilot.common.params import Params

# ============================================================
# 可調參數區 (單位: km/h)
# ============================================================
EXPERIMENTAL_ENABLE_SPEED = 30.0   # 低於此速度：開啟 Experimental Mode
EXPERIMENTAL_DISABLE_SPEED = 40.0  # 高於此速度：關閉 Experimental Mode (切回一般模式)

class AEM:
  def __init__(self):
    self.params = Params()
    # 啟動時先讀取目前的系統開關狀態，避免一開機就盲目寫入
    self._is_experimental_on = self.params.get_bool("ExperimentalMode")

  def _set_experimental_toggle(self, enable: bool):
    """安全寫入防護：只有在狀態真正需要改變時，才執行寫入硬碟的動作"""
    if self._is_experimental_on != enable:
      self.params.put_bool("ExperimentalMode", enable)
      self._is_experimental_on = enable

  def get_mode(self, mode):
    # 因為我們已經直接從底層 Params 控制了 Experimental Mode 的開關，
    # 所以這裡不需要覆蓋字串，直接把原系統的 mode 丟回去即可。
    return mode

  def update_states(self, model_msg, radar_msg, v_ego):
    v_kph = v_ego * 3.6

    # ==========================================
    # 緩衝區 (Hysteresis) 邏輯判斷
    # ==========================================
    if v_kph < EXPERIMENTAL_ENABLE_SPEED:
      target_state = True
    elif v_kph > EXPERIMENTAL_DISABLE_SPEED:
      target_state = False
    else:
      target_state = self._is_experimental_on

    # 執行狀態切換 (內部有防護機制，不用擔心頻繁寫入)
    self._set_experimental_toggle(target_state)
