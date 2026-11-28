import numpy as np

from dragonpilot.selfdrive.controls.lib.highway_smooth import (highway_jerk_factor, damp_lead_accel, HW_JERK_FACTOR,
                                                               LEAD_BRAKE_FULL_TRUST)

KPH = 1 / 3.6


def test_jerk_factor_unchanged_in_city():
  for pf in [0.5, 1.0]:
    for v in [0.0, 20 * KPH, 50 * KPH]:
      assert highway_jerk_factor(v, pf) == pf


def test_jerk_factor_raised_on_highway_and_monotonic():
  for pf in [0.5, 1.0]:
    vals = [highway_jerk_factor(v * KPH, pf) for v in range(0, 140, 5)]
    assert vals == sorted(vals)
    assert highway_jerk_factor(100 * KPH, pf) == HW_JERK_FACTOR


def test_jerk_factor_never_lowered():
  assert highway_jerk_factor(100 * KPH, 3.0) == 3.0


def test_lead_accel_unchanged_in_city():
  for a in [-3.0, -1.0, -0.3, 0.0, 0.5, 1.5]:
    assert damp_lead_accel(40 * KPH, a) == a


def test_hard_lead_braking_fully_trusted_on_highway():
  for a in [LEAD_BRAKE_FULL_TRUST, -2.0, -4.0]:
    assert damp_lead_accel(110 * KPH, a) == a


def test_mild_lead_accel_damped_on_highway():
  assert abs(damp_lead_accel(100 * KPH, 0.8) - 0.4) < 1e-9
  assert abs(damp_lead_accel(100 * KPH, -0.4) - (-0.2)) < 1e-9


def test_damped_lead_accel_continuous_and_monotonic():
  for v in [60 * KPH, 100 * KPH]:
    a = np.linspace(-4, 2, 601)
    out = np.array([damp_lead_accel(v, x) for x in a])
    assert np.all(np.diff(out) >= -1e-12)
    assert np.max(np.abs(np.diff(out))) < 0.02
