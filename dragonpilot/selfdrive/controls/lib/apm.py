from cereal import log

# =========================================================
# 低速區間門檻 (km/h -> m/s)
# 作用：塞車時自動進入戰鬥狀態，緊跟前車
# =========================================================
APM_AGGRESSIVE_ACTIVATE_SPEED = 40 * 1000 / 3600    # 低於 40 km/h — 切換至 aggressive
APM_AGGRESSIVE_DEACTIVATE_SPEED = 50 * 1000 / 3600  # 高於 50 km/h — 解除 aggressive

# =========================================================
# 🌟 新增：高速區間門檻 (km/h -> m/s)
# 作用：高速巡航時拉開距離，避免神經質煞車，提升舒適度
# =========================================================
APM_RELAXED_ACTIVATE_SPEED = 90 * 1000 / 3600       # 高於 90 km/h — 切換至 relaxed
APM_RELAXED_DEACTIVATE_SPEED = 85 * 1000 / 3600     # 低於 85 km/h — 解除 relaxed，還原使用者設定


class APM:
  def __init__(self):
    self._aggressive_active = False
    self._relaxed_active = False

  def get_personality(self, v_ego, personality):
    
    # 1. 更新低速 Aggressive 狀態機
    if self._aggressive_active:
      if v_ego > APM_AGGRESSIVE_DEACTIVATE_SPEED:
        self._aggressive_active = False
    else:
      if v_ego < APM_AGGRESSIVE_ACTIVATE_SPEED:
        self._aggressive_active = True

    # 2. 更新高速 Relaxed 狀態機
    if self._relaxed_active:
      if v_ego < APM_RELAXED_DEACTIVATE_SPEED:
        self._relaxed_active = False
    else:
      if v_ego > APM_RELAXED_ACTIVATE_SPEED:
        self._relaxed_active = True

    # 2026-11-24：跟車時距已改為依車速查表（long_mpc.py T_FOLLOW_*），市區近、高速遠已涵蓋，
    # 不再切換 personality：低速切 aggressive 會把 jerk 成本減半，造成走走停停時煞車又急又重；
    # 高速切 relaxed 會讓車距比原本遠。狀態機保留以便日後需要時恢復。
    return personality
