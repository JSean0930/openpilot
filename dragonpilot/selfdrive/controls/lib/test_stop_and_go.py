import random
import types

import numpy as np

from dragonpilot.selfdrive.controls.lib.acm import coast_accel
from dragonpilot.selfdrive.controls.lib.stop_and_go import (StopAndGo, model_features, CRUISE, COAST, STOP, STOPPED, LAUNCH,
                                                            STOP_MARGIN, JERK_UP, KPH)

DT = 0.05
T_IDXS = np.array([10.0 * (i / 32) ** 2 for i in range(33)])


def lead(d=0.0, v=0.0, status=True):
  return types.SimpleNamespace(status=status, dRel=d, vLead=v, aLeadK=0.0)


NO_LEAD = lead(status=False)


def cruise_traj(v):
  vt = np.full(len(T_IDXS), v)
  return model_features(T_IDXS, vt, np.cumsum(np.diff(T_IDXS, prepend=0) * vt))


def stop_traj(v, d):
  """模型規劃在 d 公尺外等減速停下。"""
  if d <= 0.05:
    vt = np.zeros(len(T_IDXS))
  else:
    a = v ** 2 / (2 * d)
    vt = np.maximum(v - a * T_IDXS, 0.0)
  x = np.concatenate([[0.0], np.cumsum(np.diff(T_IDXS) * (vt[1:] + vt[:-1]) / 2)])
  return model_features(T_IDXS, vt, x)


def go_traj():
  vt = np.minimum(1.5 * T_IDXS, 10.0)
  x = np.concatenate([[0.0], np.cumsum(np.diff(T_IDXS) * (vt[1:] + vt[:-1]) / 2)])
  return model_features(T_IDXS, vt, x)


def stay_traj():
  return model_features(T_IDXS, np.zeros(len(T_IDXS)), np.zeros(len(T_IDXS)))


def step(sg, v, feats, a_mpc=0.3, e2e=0.0, e2e_stop=False, lead_=NO_LEAD, ss_mpc=False, a_ego=0.0):
  return sg.update(v, a_mpc, ss_mpc, e2e, e2e_stop, feats, lead_, a_ego)


def test_never_more_throttle_than_mpc():
  rng = random.Random(1)
  sg = StopAndGo(DT)
  v = 10.0
  for _ in range(20000):
    a_mpc = rng.uniform(-3.0, 1.5)
    kind = rng.random()
    feats = cruise_traj(v) if kind < 0.4 else (stop_traj(v, rng.uniform(1, 80)) if kind < 0.8 else go_traj())
    l = lead(rng.uniform(2, 100), rng.uniform(0, 20), rng.random() > 0.5)
    out, _, _ = sg.update(v, a_mpc, False, rng.uniform(-3, 1), rng.random() > 0.8, feats, l)
    assert out <= a_mpc + 1e-9
    v = max(0.0, v + rng.uniform(-1, 1))


def test_passthrough_while_cruising():
  sg = StopAndGo(DT)
  for a in [0.5, -0.2, 0.8, -0.6, 0.1]:
    out, ss, used = step(sg, 14.0, cruise_traj(14.0), a_mpc=a, e2e=-0.8)
    assert out == a and not ss and not used
    assert sg.state == CRUISE


def test_highway_never_touched():
  sg = StopAndGo(DT)
  v = 90 * KPH
  for _ in range(100):
    out, _, used = step(sg, v, stop_traj(v, 60.0), a_mpc=0.3, e2e=-2.0)
    assert out == 0.3 and not used
  assert sg.state == CRUISE


def test_short_stop_blip_ignored():
  sg = StopAndGo(DT)
  v = 14.0
  for i in range(200):
    blip = 50 <= i < 57  # 0.35 s
    out, _, _ = step(sg, v, stop_traj(v, 40.0) if blip else cruise_traj(v), a_mpc=0.2, e2e=-0.5 if blip else 0.0)
    assert out == 0.2
  assert sg.state == CRUISE


def test_coast_only_lifts_off():
  sg = StopAndGo(DT)
  v = 12.0
  vt = np.maximum(v - 1.0 * T_IDXS, 0.3 * v - 0.5)   # 明顯減速但不會停
  x = np.cumsum(np.diff(T_IDXS, prepend=0) * vt)
  feats = model_features(T_IDXS, vt, x)
  outs = [step(sg, v, feats, a_mpc=0.4)[0] for _ in range(60)]
  assert sg.state == COAST
  assert abs(outs[-1] - coast_accel(v)) < 1e-6
  assert min(outs) >= coast_accel(v) - 1e-9


def test_closed_loop_red_light_stops_about_one_meter_short():
  # 模型計畫約 1 m/s² 停下；車輛 0.3 s 延遲 + 0.35 s 一階落後
  for v0, d0 in [(14.0, 98.0), (11.0, 60.0), (8.0, 32.0), (5.0, 20.0)]:
    sg = StopAndGo(DT)
    x, v, a = 0.0, v0, 0.0
    hist = [0.0] * 6
    a_min = 0.0
    for _ in range(int(40 / DT)):
      feats = stop_traj(v, max(d0 - x, 0.0)) if v > 0.05 else stay_traj()
      cmd, ss, _ = step(sg, v, feats, a_mpc=0.3, a_ego=a)
      hist.append(cmd)
      a += (hist.pop(0) - a) * DT / 0.35
      if ss and v < 0.5:
        a = min(a, -0.5)
      v = max(0.0, v + a * DT)
      x += v * DT
      if v > 0.5:
        a_min = min(a_min, a)
    assert sg.state == STOPPED
    short = d0 - x
    assert 0.5 < short < 1.5, (v0, d0, short)
    assert a_min > -2.0, (v0, d0, a_min)


def test_stop_profile_is_gentle_and_linear():
  sg = StopAndGo(DT)
  v, d = 14.0, 70.0
  outs = []
  x = 0.0
  for _ in range(40):
    outs.append(step(sg, v, stop_traj(v, d - x), a_mpc=0.3)[0])
    x += v * DT
  assert sg.state == STOP
  expect = -v ** 2 / (2 * (d - x - STOP_MARGIN))
  assert outs[-1] >= expect - 0.15
  assert np.max(np.abs(np.diff(outs))) <= 2.0 * DT + 1e-9


def test_standstill_holds_until_model_says_go():
  sg = StopAndGo(DT)
  for _ in range(40):
    out, ss, _ = step(sg, 0.0, stay_traj(), a_mpc=0.8)
    assert ss and out <= 0.0
  assert sg.holding
  outs = []
  for i in range(10):
    out, ss, _ = step(sg, 0.0, go_traj(), a_mpc=0.8)
    outs.append((out, ss))
  assert sg.state == LAUNCH
  assert outs[0][1] and not outs[-1][1]


def test_go_needs_model_not_saying_stop():
  sg = StopAndGo(DT)
  for _ in range(40):
    step(sg, 0.0, stay_traj())
  for _ in range(40):
    _, ss, _ = step(sg, 0.0, go_traj(), e2e_stop=True)
    assert ss
  assert sg.holding


def test_no_move_then_stop_after_launch():
  sg = StopAndGo(DT)
  for _ in range(40):
    step(sg, 0.0, stay_traj())
  for _ in range(10):
    step(sg, 0.0, go_traj(), a_mpc=0.8)
  assert sg.state == LAUNCH
  v = 3.0
  for _ in range(14):   # 0.7 s 的停止訊號：一般情況會停，起步後不會
    out, _, _ = step(sg, v, stop_traj(v, 15.0), a_mpc=0.8)
  assert sg.state == LAUNCH
  for _ in range(10):
    step(sg, v, stop_traj(v, 12.0), a_mpc=0.8)
  assert sg.state == STOP


def test_lead_owned_stop_left_to_mpc():
  sg = StopAndGo(DT)
  v = 12.0
  for _ in range(60):
    out, _, _ = step(sg, v, stop_traj(v, 30.0), a_mpc=-0.6, lead_=lead(25.0, 0.0))
    assert out == -0.6
  assert sg.state == CRUISE


def test_stopped_behind_lead_not_held():
  sg = StopAndGo(DT)
  for _ in range(40):
    out, ss, _ = step(sg, 0.0, stay_traj(), a_mpc=0.4, lead_=lead(6.0, 0.0), ss_mpc=False)
    assert out == 0.4 and not ss
  assert sg.state == CRUISE


def test_limited_band_needs_longer_signal_and_no_coast():
  sg = StopAndGo(DT)
  v = 70 * KPH
  for _ in range(15):   # 0.75 s
    step(sg, v, stop_traj(v, 80.0), a_mpc=0.2)
  assert sg.state == CRUISE
  for _ in range(10):
    step(sg, v, stop_traj(v, 75.0), a_mpc=0.2)
  assert sg.state == STOP


def test_urgent_e2e_braking_passes_immediately():
  sg = StopAndGo(DT)
  v = 12.0
  step(sg, v, cruise_traj(v), a_mpc=0.5)
  out, _, used = step(sg, v, stop_traj(v, 20.0), a_mpc=0.5, e2e=-2.0)
  assert sg.state == STOP
  assert out < 0.5 - 4.0 * DT


def test_release_ramps_back_to_mpc():
  sg = StopAndGo(DT)
  v = 12.0
  for i in range(40):
    step(sg, v, stop_traj(v, 40.0 - i * v * DT), a_mpc=0.3)
  assert sg.state == STOP
  low = sg.a_last
  outs = [step(sg, v, cruise_traj(v), a_mpc=0.3)[0] for _ in range(80)]
  assert sg.state == CRUISE
  assert np.max(np.diff(outs)) <= JERK_UP * DT + 1e-9
  assert outs[0] < 0.3 and outs[-1] == 0.3 and low < 0
