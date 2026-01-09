#!/usr/bin/env python3
import math
import numpy as np
from collections import deque
from typing import Any

import capnp
from cereal import messaging, log, car
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.params import Params
from openpilot.common.realtime import DT_MDL, Priority, config_realtime_process
from openpilot.common.swaglog import cloudlog
from openpilot.common.simple_kalman import KF1D


# ====================== 可調參數區（TUNING PARAMS） ======================

# 1) 前車加速度時間常數（aLeadTau）速度相依調整
#   - 低速時用較小 tau → 更敏感
#   - 高速時用較大 tau → 穩定不晃
LEAD_ACCEL_TAU_V_EGO_BP = [0.0, 8.33, 15.0, 30.0]   # 速度分段 (m/s) ≈ [0, 30, 54, 108] km/h
LEAD_ACCEL_TAU_V_EGO_V  = [0.4, 0.7,  1.2,  1.6]     # 對應 tau（秒）

# aLead 被視為「幾乎零加速度」的門檻（m/s^2）
LEAD_ACCEL_CONST_ACCEL_THRESH = 0.2 #0.1

# 2) 視覺 / 雷達 覆蓋邏輯（集中在上方方便日後調整）
VISION_PROB_MIN = 0.35
RADAR_OVERRIDE_PROB = 0.60

# 視覺/雷達切換速度門檻（km/h）
SPEED_SWITCH_KMH = 60.0
SPEED_SWITCH_MS = SPEED_SWITCH_KMH / 3.6

# ====================== 本次修改重點：低速純視覺（不允許雷達覆蓋） ======================
# - v_ego <= SPEED_SWITCH_MS 時：
#   1) 完全不做 match_vision_to_track
#   2) 不允許雷達覆蓋（RADAR_OVERRIDE_PROB 不生效）
#   3) 不做 low_speed_override（低速雷達補強不生效）
LOW_SPEED_VISION_ONLY = True
# =======================================================================


# Default lead acceleration decay（保留原本常數，供舊邏輯或其他模組使用）
_LEAD_ACCEL_TAU = 1.5

# radar tracks
SPEED, ACCEL = 0, 1

# stationary qualification parameters
V_EGO_STATIONARY = 4.

RADAR_TO_CENTER = 2.7
RADAR_TO_CAMERA = 1.52


class KalmanParams:
  def __init__(self, dt: float):
    assert dt > .01 and dt < .2, "Radar time step must be between .01s and 0.2s"
    self.A = [[1.0, dt], [0.0, 1.0]]
    self.C = [1.0, 0.0]

    dts = [i * 0.01 for i in range(1, 21)]
    K0 = [0.12287673, 0.14556536, 0.16522756, 0.18281627, 0.1988689,  0.21372394,
          0.22761098, 0.24069424, 0.253096,   0.26491023, 0.27621103, 0.28705801,
          0.29750003, 0.30757767, 0.31732515, 0.32677158, 0.33594201, 0.34485814,
          0.35353899, 0.36200124]
    K1 = [0.29666309, 0.29330885, 0.29042818, 0.28787125, 0.28555364, 0.28342219,
          0.28144091, 0.27958406, 0.27783249, 0.27617149, 0.27458948, 0.27307714,
          0.27162685, 0.27023228, 0.26888809, 0.26758976, 0.26633338, 0.26511557,
          0.26393339, 0.26278425]
    self.K = [[np.interp(dt, dts, K0)], [np.interp(dt, dts, K1)]]


class Track:
  def __init__(self, identifier: int, v_lead: float, kalman_params: KalmanParams):
    self.identifier = identifier
    self.cnt = 0

    # aLeadTau 改為可被速度動態調整的 filter，初始值仍沿用原本 _LEAD_ACCEL_TAU
    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)

    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float,
             v_lead: float, measured: float, v_ego: float):
    """
    v_ego 新增傳入，讓 aLeadTau 能做速度相依調整：
      - 低速：tau 小，對前車減速/加速變化更敏感
      - 高速：tau 大，維持穩定
    """
    self.dRel = d_rel
    self.yRel = y_rel
    self.vRel = v_rel
    self.vLead = v_lead
    self.measured = measured

    if self.cnt > 0:
      self.kf.update(self.vLead)

    self.vLeadK = float(self.kf.x[SPEED][0])
    self.aLeadK = float(self.kf.x[ACCEL][0])

    base_tau = float(np.interp(v_ego, LEAD_ACCEL_TAU_V_EGO_BP, LEAD_ACCEL_TAU_V_EGO_V))

    if abs(self.aLeadK) < LEAD_ACCEL_CONST_ACCEL_THRESH:
      self.aLeadTau.x = base_tau
    else:
      # 強變化時收斂到 0（等效更小 tau），讓反應更快
      self.aLeadTau.update(0.0)

    self.cnt += 1

  def get_RadarState(self, model_prob: float = 0.0):
    return {
      "dRel": float(self.dRel),
      "yRel": float(self.yRel),
      "vRel": float(self.vRel),
      "vLead": float(self.vLead),
      "vLeadK": float(self.vLeadK),
      "aLeadK": float(self.aLeadK),
      "aLeadTau": float(self.aLeadTau.x),
      "status": True,
      "fcw": self.is_potential_fcw(model_prob),
      "modelProb": model_prob,
      "radar": True,
      "radarTrackId": self.identifier,
    }

  def potential_low_speed_lead(self, v_ego: float):
    # stop for stuff in front of you and low speed, even without model confirmation
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    return f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"


def laplacian_pdf(x: float, mu: float, b: float):
  b = max(b, 1e-4)
  return math.exp(-abs(x-mu)/b)


def match_vision_to_track(v_ego: float, lead: capnp._DynamicStructReader, tracks: dict[int, Track]):
  offset_vision_dist = lead.x[0] - RADAR_TO_CAMERA

  def prob(c):
    prob_d = laplacian_pdf(c.dRel, offset_vision_dist, lead.xStd[0])
    prob_y = laplacian_pdf(c.yRel, -lead.y[0], lead.yStd[0])
    prob_v = laplacian_pdf(c.vRel + v_ego, lead.v[0], lead.vStd[0])
    return prob_d * prob_y * prob_v

  track = max(tracks.values(), key=prob)

  dist_sane = abs(track.dRel - offset_vision_dist) < max([(offset_vision_dist) * .25, 5.0])
  vel_sane = (abs(track.vRel + v_ego - lead.v[0]) < 10) or (v_ego + track.vRel > 3)
  if dist_sane and vel_sane:
    return track
  return None


def get_RadarState_from_vision(lead_msg: capnp._DynamicStructReader, v_ego: float, model_v_ego: float):
  lead_v_rel_pred = lead_msg.v[0] - model_v_ego
  return {
    "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
    "yRel": float(-lead_msg.y[0]),
    "vRel": float(lead_v_rel_pred),
    "vLead": float(v_ego + lead_v_rel_pred),
    "vLeadK": float(v_ego + lead_v_rel_pred),
    "aLeadK": float(lead_msg.a[0]),
    "aLeadTau": 0.3,
    "fcw": False,
    "modelProb": float(lead_msg.prob),
    "status": True,
    "radar": False,
    "radarTrackId": -1,
  }


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, low_speed_override: bool = True) -> dict[str, Any]:
  """
  速度切換策略：
    - v_ego <= SPEED_SWITCH_MS：低速純視覺（本次修改重點：不允許雷達覆蓋/補強）
    - v_ego >  SPEED_SWITCH_MS：高速維持「雷達 + 視覺融合（雷達為主）」原邏輯
  """

  # ====================== 低速：模型純視覺（不允許雷達覆蓋） ======================
  if LOW_SPEED_VISION_ONLY and (v_ego <= SPEED_SWITCH_MS):
    # 低速下：
    # - 不 match_vision_to_track
    # - 不做 RADAR_OVERRIDE_PROB 覆蓋
    # - 不做 low_speed_override（雷達低速補強）
    lead_dict: dict[str, Any] = {"status": False}
    if ready and (lead_msg.prob > VISION_PROB_MIN):
      lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)
    return lead_dict
  # ==============================================================================

  # --- 高速：雷達為主 ---
  if v_ego > SPEED_SWITCH_MS:
    track = None
    if len(tracks) > 0 and ready and lead_msg.prob > .5:
      track = match_vision_to_track(v_ego, lead_msg, tracks)

    lead_dict: dict[str, Any] = {"status": False}
    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob)
    elif ready and (lead_msg.prob > .5):
      lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

    # 低速覆蓋：高速邏輯下保留（通常不會命中，維持原習慣）
    if low_speed_override:
      low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
      if len(low_speed_tracks) > 0:
        closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
        if (not lead_dict.get("status", False)) or (closest_track.dRel < lead_dict["dRel"]):
          lead_dict = closest_track.get_RadarState()

    return lead_dict

  # --- 非高速（但 LOW_SPEED_VISION_ONLY=False 才會走到這裡）：維持原本視覺優先 + 雷達可覆蓋/補強 ---
  lead_dict: dict[str, Any] = {"status": False}

  if ready and (lead_msg.prob > VISION_PROB_MIN):
    lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

    if len(tracks) > 0 and (lead_msg.prob <= RADAR_OVERRIDE_PROB):
      trk = match_vision_to_track(v_ego, lead_msg, tracks)
      if trk is not None:
        lead_dict = trk.get_RadarState(lead_msg.prob)
  else:
    if len(tracks) > 0 and ready and lead_msg.prob > .5:
      track = match_vision_to_track(v_ego, lead_msg, tracks)
    else:
      track = None

    if track is not None:
      lead_dict = track.get_RadarState(lead_msg.prob)
    elif ready and (lead_msg.prob > .5):
      lead_dict = get_RadarState_from_vision(lead_msg, v_ego, model_v_ego)

  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      if (not lead_dict.get("status", False)) or (closest_track.dRel < lead_dict["dRel"]):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.v_ego = 0.0
    self.v_ego_hist = deque([0.0], maxlen=int(round(delay / DT_MDL)) + 1)
    self.last_v_ego_frame = -1

    self.radar_state: capnp._DynamicStructBuilder | None = None
    self.radar_state_valid = False

    self.ready = False

  def update(self, sm: messaging.SubMaster, rr: car.RadarData):
    self.ready = sm.seen["modelV2"]
    self.current_time = 1e-9 * max(sm.logMonoTime.values())

    if sm.recv_frame["carState"] != self.last_v_ego_frame:
      self.v_ego = sm["carState"].vEgo
      self.v_ego_hist.append(self.v_ego)
      self.last_v_ego_frame = sm.recv_frame["carState"]

    ar_pts = {pt.trackId: [pt.dRel, pt.yRel, pt.vRel, pt.measured] for pt in rr.points}

    # *** remove missing points from meta data ***
    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    # *** compute the tracks ***
    for ids in ar_pts:
      rpt = ar_pts[ids]
      # align v_ego by a fixed time to align it with the radar measurement
      v_lead = rpt[2] + self.v_ego_hist[0]

      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)

      # 傳入 v_ego（使用與 v_lead 對齊的歷史值）
      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], self.v_ego_hist[0])

    # *** publish radarState ***
    self.radar_state_valid = sm.all_checks()
    self.radar_state = log.RadarState.new_message()
    self.radar_state.mdMonoTime = sm.logMonoTime["modelV2"]
    self.radar_state.radarErrors = rr.errors
    self.radar_state.carStateMonoTime = sm.logMonoTime["carState"]

    if len(sm["modelV2"].velocity.x):
      model_v_ego = sm["modelV2"].velocity.x[0]
    else:
      model_v_ego = self.v_ego

    leads_v3 = sm["modelV2"].leadsV3
    if len(leads_v3) > 1:
      # leadOne 允許 low_speed_override，但在「低速純視覺」分支會被完全忽略（符合你的需求）
      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                                          low_speed_override=True)
      # leadTwo 通常不做低速補強（原本就 False），低速純視覺同樣生效
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
                                          low_speed_override=False)

  def publish(self, pm: messaging.PubMaster):
    assert self.radar_state is not None
    radar_msg = messaging.new_message("radarState")
    radar_msg.valid = self.radar_state_valid
    radar_msg.radarState = self.radar_state
    pm.send("radarState", radar_msg)


def main() -> None:
  config_realtime_process(5, Priority.CTRL_LOW)

  cloudlog.info("radard is waiting for CarParams")
  CP = messaging.log_from_bytes(Params().get("CarParams", block=True), car.CarParams)
  cloudlog.info("radard got CarParams")

  sm = messaging.SubMaster(["modelV2", "carState", "liveTracks"], poll="modelV2")
  pm = messaging.PubMaster(["radarState"])

  RD = RadarD(CP.radarDelay)

  while 1:
    sm.update()
    RD.update(sm, sm["liveTracks"])
    RD.publish(pm)


if __name__ == "__main__":
  main()