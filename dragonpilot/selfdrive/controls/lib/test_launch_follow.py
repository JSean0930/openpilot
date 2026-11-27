import random
import types

from dragonpilot.selfdrive.controls.lib.launch_follow import LaunchFollow, EXIT_V_EGO, JERK_UP, JERK_DOWN


def lead(d, v_lead, a_lead=0.0, status=True):
  return types.SimpleNamespace(status=status, dRel=d, vLead=v_lead, aLeadK=a_lead)


def stopped(lf, d=2.6, n=40):
  for _ in range(n):
    lf.update(0.0, lead(d, 0.0), -0.5, True)


def test_radar_noise_at_standstill_never_starts():
  rng = random.Random(1)
  lf = LaunchFollow()
  for _ in range(4000):
    a, ss = lf.update(0.0, lead(2.6 + rng.uniform(-0.05, 0.05), rng.uniform(-0.25, 0.25)), -0.5, True)
    assert (a, ss) == (-0.5, True)


def test_no_start_while_mpc_still_wants_to_stop():
  lf = LaunchFollow()
  stopped(lf)
  a, ss = lf.update(0.0, lead(3.0, 0.8, 0.5), 0.0, True)
  assert (a, ss) == (0.0, True) and not lf.active


def test_starts_once_mpc_agrees_and_ramps_with_limited_jerk():
  lf = LaunchFollow()
  stopped(lf)
  outs = []
  for _ in range(10):
    a, ss = lf.update(0.0, lead(3.0, 1.0, 1.2), 0.05, True)
    assert ss is False
    outs.append(a)
  assert lf.active
  steps = [b - a for a, b in zip([0.05] + outs, outs)]
  assert max(steps) <= JERK_UP * 0.05 + 1e-9
  assert outs[-1] > 0.6


def test_creeping_lead_gets_small_push():
  # 前車只是蠕動（0.4 m/s、幾乎不加速）：力道應很小，接近你蠕動時的習慣
  lf = LaunchFollow()
  stopped(lf)
  outs = [lf.update(0.0, lead(3.0, 0.4, 0.05), 0.03, False)[0] for _ in range(20)]
  assert max(outs) < 0.3


def test_e2e_stop_blocks_start():
  lf = LaunchFollow()
  stopped(lf)
  a, ss = lf.update(0.0, lead(3.0, 0.8, 0.5), 0.05, False, e2e_stop=True)
  assert not lf.active and (a, ss) == (0.05, False)


def test_abort_immediately_when_lead_brakes_hard():
  lf = LaunchFollow()
  stopped(lf)
  for _ in range(10):
    lf.update(0.0, lead(3.0, 1.0, 1.2), 0.05, False)
  a, ss = lf.update(0.5, lead(3.2, 0.9, -1.2), -0.3, False)
  assert not lf.active and (a, ss) == (-0.3, False)


def test_catching_up_hands_back_without_drop():
  # 追上前車速度（蠕動）：不能一下子掉回 MPC，要以 JERK_DOWN 慢慢降
  lf = LaunchFollow()
  stopped(lf)
  for _ in range(10):
    lf.update(0.0, lead(3.0, 1.0, 1.2), 0.05, False)
  prev = lf.a_cmd
  outs = []
  for _ in range(40):
    a, _ = lf.update(1.0, lead(3.5, 1.0, 0.0), -0.2, False)
    outs.append(a)
  seq = [prev] + outs
  assert all(b >= a - JERK_DOWN * 0.05 - 1e-9 for a, b in zip(seq, seq[1:]))
  assert outs[-1] == -0.2


def test_done_hands_back_and_never_below_mpc():
  lf = LaunchFollow()
  stopped(lf)
  for _ in range(10):
    lf.update(0.0, lead(3.0, 1.0, 1.2), 0.05, False)
  outs = [lf.update(EXIT_V_EGO + 0.1, lead(6.0, 5.0, 1.0), 0.4, False)[0] for _ in range(30)]
  assert all(o >= 0.4 for o in outs)
  assert outs[-1] == 0.4


def test_never_less_accel_than_mpc_random():
  rng = random.Random(0)
  lf = LaunchFollow()
  for _ in range(30000):
    v = rng.choice([0.0, 0.0, 0.05, rng.uniform(0, 5)])
    l = lead(rng.uniform(0.5, 10), rng.choice([0.0, rng.uniform(-0.3, 5)]), rng.uniform(-1.5, 1.5), rng.random() > 0.05)
    a_mpc = rng.uniform(-2.0, 1.5)
    ss = rng.random() < 0.5
    e2e = rng.random() < 0.1
    a, s = lf.update(v, l, a_mpc, ss, e2e)
    assert a >= a_mpc - 1e-9
    if e2e:
      assert (a, s) == (a_mpc, ss)
      assert not lf.active and not lf.releasing


def test_closed_loop_lead_stops_again_no_collision():
  # 前車起步 1 s 後急煞停住；介入應立刻交還，假設 MPC 交還後以 -3 m/s² 煞
  lf = LaunchFollow()
  stopped(lf)
  x_l, v_l, x_e, v_e, a_e = 2.6, 0.0, 0.0, 0.0, 0.0
  hist = [0.0] * 6
  for i in range(200):
    t = i * 0.05
    a_l = 1.5 if t < 1.0 else -4.0
    v_l = max(0.0, v_l + a_l * 0.05)
    x_l += v_l * 0.05
    gap = x_l - x_e
    mpc = 0.05 if t < 1.0 else -3.0
    a_cmd, _ = lf.update(v_e, lead(gap, v_l, a_l if t >= 1.0 else 1.5), mpc, mpc < 0)
    hist.append(a_cmd)
    a_e += (hist.pop(0) - a_e) * 0.3
    v_e = max(0.0, v_e + a_e * 0.05)
    x_e += v_e * 0.05
    assert x_l - x_e > 0.5, (t, x_l - x_e)
