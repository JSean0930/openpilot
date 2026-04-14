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
LEAD_ACCEL_TAU_V_EGO_BP = [0.0, 5.0, 10.0, 30.0]
LEAD_ACCEL_TAU_V_EGO_V  = [0.3, 0.45, 1.50, 2.00]

LEAD_ACCEL_CONST_ACCEL_THRESH = 0.25 

# 2) 視覺 / 雷達 覆蓋邏輯
VISION_PROB_MIN = 0.35
RADAR_OVERRIDE_PROB = 0.60

# ====================== 本次修改重點：視覺與雷達深度融合 ======================
# 關閉低速純視覺，啟用真實的 Sensor Fusion
SENSOR_FUSION_ENABLE = True

# 3) 視覺濾波器參數 (過濾 Vision 產生的神經質高頻抖動，保留作為雷達丟失時的完美備胎)
VISION_V_REL_TAU = 0.10 
VISION_A_LEAD_TAU = 0.15 

# 4) 視覺鎖定磁滯區間 (Hysteresis)
VISION_PROB_LOCK = 0.40   
VISION_PROB_UNLOCK = 0.20 
# =======================================================================


# Default lead acceleration decay
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

    self.aLeadTau = FirstOrderFilter(_LEAD_ACCEL_TAU, 0.45, DT_MDL)

    self.K_A = kalman_params.A
    self.K_C = kalman_params.C
    self.K_K = kalman_params.K
    self.kf = KF1D([[v_lead], [0.0]], self.K_A, self.K_C, self.K_K)

  def update(self, d_rel: float, y_rel: float, v_rel: float,
             v_lead: float, measured: float, v_ego: float):
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
    return abs(self.yRel) < 1.0 and (v_ego < V_EGO_STATIONARY) and (0.75 < self.dRel < 25)

  def is_potential_fcw(self, model_prob: float):
    return model_prob > .9

  def __str__(self):
    return f"x: {self.dRel:4.1f}  y: {self.yRel:4.1f}  v: {self.vRel:4.1f}  a: {self.aLeadK:4.1f}"


class VisionLeadState:
  def __init__(self):
    self.active = False
    self.v_rel_filter = FirstOrderFilter(0.0, VISION_V_REL_TAU, DT_MDL)
    self.a_lead_filter = FirstOrderFilter(0.0, VISION_A_LEAD_TAU, DT_MDL)

  def update(self, prob: float, v_rel_raw: float, a_lead_raw: float) -> bool:
    if self.active:
      if prob < VISION_PROB_UNLOCK:
        self.active = False
    else:
      if prob > VISION_PROB_LOCK:
        self.active = True
        self.v_rel_filter.x = v_rel_raw
        self.a_lead_filter.x = a_lead_raw

    if self.active:
      self.v_rel_filter.update(v_rel_raw)
      self.a_lead_filter.update(a_lead_raw)

    return self.active


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


def get_lead(v_ego: float, ready: bool, tracks: dict[int, Track], lead_msg: capnp._DynamicStructReader,
             model_v_ego: float, vision_state: VisionLeadState, low_speed_override: bool = True) -> dict[str, Any]:
  """
  ✅ 終極優化版：視覺與雷達深度融合 (True Sensor Fusion)
  """

  # ====================== 1. 視覺狀態更新與濾波 (作為完美備胎) ======================
  lead_v_rel_pred_raw = lead_msg.v[0] - model_v_ego
  a_lead_raw = float(lead_msg.a[0])

  vision_active = False
  if ready:
    vision_active = vision_state.update(lead_msg.prob, lead_v_rel_pred_raw, a_lead_raw)

  vision_dict: dict[str, Any] = {"status": False}
  if vision_active:
    base_tau = float(np.interp(v_ego, LEAD_ACCEL_TAU_V_EGO_BP, LEAD_ACCEL_TAU_V_EGO_V))
    if abs(vision_state.a_lead_filter.x) < LEAD_ACCEL_CONST_ACCEL_THRESH:
      vision_tau = base_tau
    else:
      vision_tau = 0.0

    # 靜止鎖定邏輯 (Standstill Lock)
    is_standstill = (v_ego < 0.4) and (abs(v_ego + vision_state.v_rel_filter.x) < 0.5)
    
    if is_standstill:
      final_v_rel = -v_ego  
      final_v_lead = 0.0
      final_a_lead = 0.0
      vision_tau = 4.0      
    else:
      final_v_rel = float(vision_state.v_rel_filter.x)
      final_v_lead = float(v_ego + vision_state.v_rel_filter.x)
      final_a_lead = float(vision_state.a_lead_filter.x)

    vision_dict = {
      "dRel": float(lead_msg.x[0] - RADAR_TO_CAMERA),
      "yRel": float(-lead_msg.y[0]),
      "vRel": final_v_rel,
      "vLead": final_v_lead,
      "vLeadK": final_v_lead,
      "aLeadK": final_a_lead,
      "aLeadTau": float(vision_tau),
      "fcw": False,
      "modelProb": float(lead_msg.prob),
      "status": True,
      "radar": False,
      "radarTrackId": -1,
    }

  # ====================== 2. 真實深度融合 (Sensor Fusion) ======================
  lead_dict: dict[str, Any] = {"status": False}
  
  if SENSOR_FUSION_ENABLE:
    track = None
    # 只要視覺看見車 (prob > MIN)，我們就去尋找有沒有對應的物理雷達訊號
    if len(tracks) > 0 and ready and lead_msg.prob > VISION_PROB_MIN:
      track = match_vision_to_track(v_ego, lead_msg, tracks)

    if track is not None:
      # 找到匹配的雷達！啟動融合模式
      radar_dict = track.get_RadarState(lead_msg.prob)
      
      if vision_active:
        # 【融合核心】
        # 物理運動 (縱向距離、速度、加速度) 100% 交給雷達，消除塞車暴衝
        fused_dict = radar_dict.copy()
        # 空間位置 (橫向偏移、存在機率) 交給視覺，防止跟錯隔壁車道的車
        fused_dict["yRel"] = vision_dict["yRel"]
        fused_dict["modelProb"] = vision_dict["modelProb"]
        
        lead_dict = fused_dict
      else:
        lead_dict = radar_dict
    else:
      # 雷達沒看到 (例如死車、靜止障礙物)，平滑退回使用加強濾波過的視覺狀態
      if vision_active:
        lead_dict = vision_dict

  # ====================== 3. 極低速緊急雷達補強 (防加塞) ======================
  if low_speed_override:
    low_speed_tracks = [c for c in tracks.values() if c.potential_low_speed_lead(v_ego)]
    if len(low_speed_tracks) > 0:
      closest_track = min(low_speed_tracks, key=lambda c: c.dRel)
      # 如果雷達發現了近距離的物體 (例如突然加塞的車)，但視覺沒看到，或是雷達距離比視覺近
      # 直接強行覆蓋，以保證不撞車
      if (not lead_dict.get("status", False)) or (closest_track.dRel < lead_dict.get("dRel", float('inf'))):
        lead_dict = closest_track.get_RadarState()

  return lead_dict


class RadarD:
  def __init__(self, delay: float = 0.0):
    self.current_time = 0.0

    self.tracks: dict[int, Track] = {}
    self.kalman_params = KalmanParams(DT_MDL)

    self.vision_state_lead_one = VisionLeadState()
    self.vision_state_lead_two = VisionLeadState()

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

    for ids in list(self.tracks.keys()):
      if ids not in ar_pts:
        self.tracks.pop(ids, None)

    for ids in ar_pts:
      rpt = ar_pts[ids]
      v_lead = rpt[2] + self.v_ego_hist[0]

      if ids not in self.tracks:
        self.tracks[ids] = Track(ids, v_lead, self.kalman_params)

      self.tracks[ids].update(rpt[0], rpt[1], rpt[2], v_lead, rpt[3], self.v_ego_hist[0])

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
      self.radar_state.leadOne = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[0], model_v_ego,
                                          self.vision_state_lead_one, low_speed_override=True)
      self.radar_state.leadTwo = get_lead(self.v_ego, self.ready, self.tracks, leads_v3[1], model_v_ego,
                                          self.vision_state_lead_two, low_speed_override=False)

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
