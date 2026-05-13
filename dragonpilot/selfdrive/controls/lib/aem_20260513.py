"""
Copyright (c) 2025, Rick Lan

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
... (為節省版面，版權聲明可與之前保持一致) ...
"""

class AEM:
  def __init__(self):
    # 不需要任何狀態變數了
    pass

  def get_mode(self, mode):
    # 無論原本系統傳入什麼 mode，永遠強制回傳 'acc'
    return 'acc'

  def update_states(self, model_msg, radar_msg, v_ego):
    # 因為已經永久鎖定，不需要再讀取車速或計算任何狀態
    # 但必須保留這個函式與三個參數，以免主程式呼叫時發生 TypeError 崩潰
    pass
