import itertools
import random
import types

from dragonpilot.selfdrive.controls.lib.acm import ACM, MAX_MPC_BRAKE, USER_T_FOLLOW, USER_D0, coast_accel


def lead(d, v_lead, a_lead=0.0, status=True):
  return types.SimpleNamespace(status=status, dRel=d, vLead=v_lead, aLeadK=a_lead)


def settle(acm, a_mpc, v_ego, lead_, source='lead0', v_cruise=40.0, pitch=0.0, n=40):
  out = a_mpc
  for _ in range(n):
    out = acm.update(a_mpc, v_ego, v_cruise, lead_, source, pitch)
  return out


def test_only_softens_never_adds_brake_or_throttle():
  for a_mpc, v, d, vl, al, src in itertools.product(
      [-2.0, -1.4, -0.8, -0.3, 0.0, 0.4, 1.0], [3.0, 8.0, 14.0, 25.0], [5.0, 15.0, 40.0, 80.0],
      [0.0, 4.0, 10.0, 20.0], [-1.5, -0.5, 0.0, 0.8], ['lead0', 'cruise', '']):
    acm = ACM()
    out = settle(acm, a_mpc, v, lead(d, vl, al), src, v_cruise=v - 1.0)
    if a_mpc <= 0.0:
      assert a_mpc - 1e-9 <= out <= 0.0 + 1e-9, (a_mpc, v, d, vl, al, src, out)
    else:
      assert 0.0 - 1e-9 <= out <= a_mpc + 1e-9, (a_mpc, v, d, vl, al, src, out)


def test_only_softens_with_time_varying_inputs():
  rng = random.Random(0)
  acm = ACM()
  for _ in range(20000):
    a_mpc = rng.uniform(-2.0, 1.2)
    v = rng.uniform(0.0, 30.0)
    l = lead(rng.uniform(2.0, 90.0), rng.uniform(0.0, 30.0), rng.uniform(-2.0, 1.5), rng.random() > 0.1)
    out = acm.update(a_mpc, v, rng.uniform(5.0, 30.0), l, rng.choice(['lead0', 'lead1', 'cruise', '']), rng.uniform(-0.1, 0.1))
    assert min(a_mpc, 0.0) - 1e-9 <= out <= max(a_mpc, 0.0) + 1e-9


def test_heavy_mpc_brake_passes_through_immediately():
  acm = ACM()
  settle(acm, -0.8, 14.0, lead(40.0, 10.0))
  assert acm.active
  assert acm.update(MAX_MPC_BRAKE - 0.1, 14.0, 40.0, lead(40.0, 10.0), 'lead0') == MAX_MPC_BRAKE - 0.1
  assert acm.weight == 0.0


def test_gentle_approach_is_softened_toward_physics_or_coast():
  acm = ACM()
  v_ego, v_lead, d = 14.0, 11.0, 40.0
  out = settle(acm, -0.9, v_ego, lead(d, v_lead))
  gap = d - (USER_T_FOLLOW * v_lead + USER_D0)
  b_req = (v_lead - v_ego) ** 2 / (2 * gap)
  assert out > -0.9
  assert out <= coast_accel(v_ego) + 1e-9
  assert out <= -1.1 * b_req + 1e-9


def test_short_ttc_not_softened():
  acm = ACM()
  out = settle(acm, -1.0, 14.0, lead(15.0, 9.0))  # TTC 3 s
  assert out == -1.0


def test_lead_braking_not_softened():
  acm = ACM()
  assert settle(acm, -1.0, 14.0, lead(40.0, 11.0, a_lead=-1.2)) == -1.0


def test_stopped_lead_not_softened():
  acm = ACM()
  assert settle(acm, -1.0, 10.0, lead(40.0, 0.0)) == -1.0


def test_lift_off_when_closing():
  acm = ACM()
  out = settle(acm, 0.5, 14.0, lead(30.0, 11.0))  # TTC 10 s
  assert out == 0.0


def test_no_lift_off_when_far():
  acm = ACM()
  out = settle(acm, 0.5, 14.0, lead(60.0, 13.0))  # TTC 60 s
  assert out == 0.5


def test_over_cruise_only_coasts():
  acm = ACM()
  out = settle(acm, -0.6, 20.0, lead(0.0, 0.0, status=False), 'cruise', v_cruise=19.0)
  assert abs(out - (-0.3)) < 1e-6


def test_far_over_cruise_not_softened():
  acm = ACM()
  out = settle(acm, -0.6, 25.0, lead(0.0, 0.0, status=False), 'cruise', v_cruise=19.0)
  assert out == -0.6


def test_exit_ramps_back_within_quarter_second():
  acm = ACM()
  settle(acm, -0.9, 14.0, lead(40.0, 11.0))
  outs = [acm.update(-1.0, 14.0, 40.0, lead(15.0, 9.0), 'lead0') for _ in range(5)]
  assert outs[0] > -1.0
  assert outs[-1] == -1.0


def test_closed_loop_gap_never_below_user_gap():
  # 最壞情況：MPC 一直要求比 ACM floor 更重的煞車，車子照 floor 減速，帶 0.3 s 致動延遲
  for v0, vl, d0 in [(14.0, 8.0, 45.0), (20.0, 12.0, 60.0), (11.0, 6.0, 30.0), (25.0, 20.0, 50.0)]:
    acm = ACM()
    v, d, a = v0, d0, 0.0
    hist = [0.0] * 6
    min_margin = 1e9
    for _ in range(600):
      cmd = acm.update(-1.4, v, 40.0, lead(d, vl), 'lead0')
      hist.append(cmd)
      a += (hist.pop(0) - a) * 0.3
      v = max(0.0, v + a * 0.05)
      d += (vl - v) * 0.05
      min_margin = min(min_margin, d - (USER_T_FOLLOW * vl + USER_D0 - 1.0))
    assert min_margin > 0.0, (v0, vl, d0, min_margin)
