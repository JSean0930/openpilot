"""
高速平順巡航 — humanlike 2026-11

問題（試車 1128，00000064 @ 05:45~06:40，90~100 km/h 跟車）：
前車本身在 91~104 km/h 間忽快忽慢（雷達 aLeadK ±0.6~0.8），OP 幾乎照單全收，
MPC 目標每 6~8 s 在 +0.8 與 −0.9 之間來回（實際指令 +0.3 / −0.8），像「大踩、放開、又大踩」。
原因：
1. 積極模式（aggressive）把 jerk / 加速度變化成本減半（jerk factor 0.5）
2. 前車加速度（雷達估計，有雜訊、落後約 0.35 s）被完整拿去外推前車軌跡

做法（只在 50 km/h 以上逐漸生效，80 km/h 以上完全生效）：
1. jerk factor 由個人化設定逐步提高到 2.0：加減速更線性、更不跟著晃
2. 前車的輕微加減速只採信一半；前車煞車愈重採信愈多，-1.5 m/s² 以下完全採信（緊急狀況不打折）

閉迴路近似模擬（同一段實際前車軌跡）：加減速來回 12 → 8 次、指令擺幅 −13%、jerk −40%，最小時距不變（0.63 s）。
"""

import numpy as np

HW_BP = [50.0 / 3.6, 80.0 / 3.6]   # m/s

HW_JERK_FACTOR = 2.0

LEAD_ACCEL_TRUST_HW = 0.5          # 高速時前車輕微加減速的採信比例
LEAD_BRAKE_FULL_TRUST = -1.5       # m/s²，前車煞車到這裡以下完全採信
LEAD_BRAKE_PARTIAL = -0.5          # m/s²，-0.5 ~ -1.5 之間線性回到完全採信


def highway_jerk_factor(v_ego, personality_factor):
  """低速維持個人化設定；高速提高到至少 HW_JERK_FACTOR（不會比原本更低）。"""
  hi = max(personality_factor, HW_JERK_FACTOR)
  return float(np.interp(v_ego, HW_BP, [personality_factor, hi]))


def damp_lead_accel(v_ego, a_lead):
  """高速時降低對前車輕微加減速的反應；煞車愈重採信愈多。"""
  trust = float(np.interp(v_ego, HW_BP, [1.0, LEAD_ACCEL_TRUST_HW]))
  if a_lead >= 0.0:
    return a_lead * trust
  trust_brake = float(np.interp(a_lead, [LEAD_BRAKE_FULL_TRUST, LEAD_BRAKE_PARTIAL], [1.0, trust]))
  return a_lead * trust_brake
