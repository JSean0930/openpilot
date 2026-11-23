import types

import dragonpilot.selfdrive.controls.lib.aem as aem_mod
from dragonpilot.selfdrive.controls.lib.aem import (AEM, THROTTLE_ACC_PROB, THROTTLE_BLENDED_PROB, BORROW_HYST,
                                                    EXPERIMENTAL_ENABLE_SPEED, EXPERIMENTAL_DISABLE_SPEED)


def model(gas_prob=1.0, have_probs=True):
  probs = [0.0, gas_prob] if have_probs else [0.0]  # index 1 is the near-term gas-press prob
  return types.SimpleNamespace(
    meta=types.SimpleNamespace(
      disengagePredictions=types.SimpleNamespace(gasPressProbs=probs)))


def aem_at(gas_prob=1.0, have_probs=True, v_ego=0.0):
  aem = AEM()
  aem.update_states(model(gas_prob, have_probs), radar_msg=None, v_ego=v_ego)
  return aem


# --- AEM must never touch Params ---

def test_no_params_dependency():
  assert not hasattr(aem_mod, 'Params')
  assert not hasattr(AEM(), 'params')


# --- blended-first: borrow ACC only when clearly wanting throttle ---

def test_blended_first_high_throttle_uses_acc():
  assert aem_at(gas_prob=0.8).get_mode('blended') == 'acc'

def test_blended_first_low_throttle_stays_blended():
  assert aem_at(gas_prob=0.2).get_mode('blended') == 'blended'

def test_blended_first_at_acc_threshold_uses_acc():
  assert aem_at(gas_prob=THROTTLE_ACC_PROB).get_mode('blended') == 'acc'

def test_blended_first_borrow_hysteresis():
  aem = aem_at(gas_prob=THROTTLE_ACC_PROB)
  assert aem.get_mode('blended') == 'acc'
  aem.update_states(model(THROTTLE_ACC_PROB - BORROW_HYST / 2), None, 0.0)
  assert aem.get_mode('blended') == 'acc'      # inside hysteresis band: keep borrowing
  aem.update_states(model(THROTTLE_ACC_PROB - BORROW_HYST * 2), None, 0.0)
  assert aem.get_mode('blended') == 'blended'


# --- acc-first: borrow BLENDED only when clearly easing off ---

def test_acc_first_low_throttle_uses_blended():
  assert aem_at(gas_prob=0.2).get_mode('acc') == 'blended'

def test_acc_first_high_throttle_stays_acc():
  assert aem_at(gas_prob=0.8).get_mode('acc') == 'acc'

def test_acc_first_at_blended_threshold_uses_blended():
  assert aem_at(gas_prob=THROTTLE_BLENDED_PROB).get_mode('acc') == 'blended'


# --- speed based base mode (internal, hysteresis) ---

def test_speed_base_mode_hysteresis():
  aem = AEM()
  aem.update_states(model(), None, (EXPERIMENTAL_ENABLE_SPEED - 5) / 3.6)
  assert aem.get_base_mode('acc') == 'blended'
  aem.update_states(model(), None, (EXPERIMENTAL_ENABLE_SPEED + EXPERIMENTAL_DISABLE_SPEED) / 2 / 3.6)
  assert aem.get_base_mode('acc') == 'blended'  # between thresholds: hold
  aem.update_states(model(), None, (EXPERIMENTAL_DISABLE_SPEED + 5) / 3.6)
  assert aem.get_base_mode('blended') == 'acc'
  aem.update_states(model(), None, (EXPERIMENTAL_ENABLE_SPEED + EXPERIMENTAL_DISABLE_SPEED) / 2 / 3.6)
  assert aem.get_base_mode('blended') == 'acc'

def test_speed_base_mode_disabled_follows_user(monkeypatch):
  monkeypatch.setattr(aem_mod, 'AEM_SPEED_BASED_BASE', False)
  aem = aem_at(v_ego=0.0)
  assert aem.get_base_mode('acc') == 'acc'
  assert aem.get_base_mode('blended') == 'blended'


# --- misc ---

def test_missing_gas_probs_defaults_to_throttle():
  # no usable probs -> assume throttle wanted (1.0): blended-first -> acc, acc-first -> acc
  aem = aem_at(have_probs=False)
  assert aem.get_mode('blended') == 'acc'
  assert aem.get_mode('acc') == 'acc'

def test_update_states_caches_prob():
  aem = aem_at(gas_prob=0.3)
  assert aem._throttle_prob == 0.3
