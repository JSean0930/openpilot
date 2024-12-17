#!/usr/bin/env python3
import math
import numpy as np
from openpilot.common.numpy_fast import clip, interp
from openpilot.common.params import Params

import cereal.messaging as messaging
from openpilot.common.conversions import Conversions as CV
from openpilot.common.filter_simple import FirstOrderFilter
from openpilot.common.realtime import DT_MDL
from openpilot.selfdrive.modeld.constants import ModelConstants
from openpilot.selfdrive.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.selfdrive.controls.lib.longcontrol import LongCtrlState
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import LongitudinalMpc
from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.long_mpc import T_IDXS as T_IDXS_MPC
from openpilot.selfdrive.controls.lib.drive_helpers import V_CRUISE_MAX, CONTROL_N, get_speed_error
from openpilot.common.swaglog import cloudlog
# PFEIFER - SLC {{
from openpilot.selfdrive.controls.speed_limit_controller import slc
# }} PFEIFER - SLC
# PFEIFER - VTSC {{
from openpilot.selfdrive.controls.vtsc import vtsc
# }} PFEIFER - VTSC

LON_MPC_STEP = 0.2  # first step is 0.2s
A_CRUISE_MIN = -1.2
A_CRUISE_MAX_VALS = [1.35, 1.1, 0.8, 0.6] #[1.12, 0.84, 0.8, 0.6] [1.6, 1.2, 0.8, 0.6]
A_CRUISE_MAX_BP = [0., 10.0, 25., 40.]
A_CRUISE_MIN_VALS =    [-1.02,  -0.25, -0.001, -0.01, -0.15, -0.25, -0.33, -0.42, -0.65, -0.65]
A_CRUISE_MIN_BP =      [ 0.,    .01,   .02,    .3,     5.,    8.,    11.,   16.,   28.,   42.]
A_CRUISE_MIN_VALS_DF = [-0.04,  -0.0002,  -0.0002,  -0.19,  -0.19,  -0.30,  -0.40,  -0.45, -0.8, -0.8]
A_CRUISE_MIN_BP_DF =   [0.,    0.01,      0.05,     0.12,    0.30,   5.,    11.,    16.,   28.,  42.]
A_CRUISE_MAX_VALS_DF =     [1.4, 2.4, 2.4, 2.4, 1.53, 1.23, .88, .65, .44, .29, .084]  # Sets the limits of the planner accel, PID may exceed
A_CRUISE_MAX_BP_DF =       [0.,    0.1,   3.,    6.,    8.,    11.,    15.,  20.,   25.,  30.,   55.]
# A_CRUISE_MAX_VALS_TOYOTA = [2.0, 1.68, 1.58, 1.3,  1.15, 0.92, 0.72, 0.52, 0.34, 0.11]  # Sets the limits of the planner accel, PID may exceed
A_CRUISE_MAX_VALS_TOYOTA =   [2.0, 1.7, 1.32, 1.22, 1.02, .87, .73, .58, .38, .24, .082]  # Sets the limits of the planner accel, PID may exceed
# CRUISE_MAX_BP in kmh =     [0.,  4,   10,   20,   30,    40,  53,  72,  90,  107, 150]
A_CRUISE_MAX_BP_TOYOTA =     [0.,  1.,  3.,   6.,   8.,    11., 15., 20., 25., 30., 55.]

# Lookup table for turns
_A_TOTAL_MAX_V = [2.04, 3.84] #[1.7, 3.2]
_A_TOTAL_MAX_BP = [20., 40.]


def get_max_accel(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP, A_CRUISE_MAX_VALS)

def get_min_accel(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP, A_CRUISE_MIN_VALS)

def get_min_accel_df(v_ego):
  return interp(v_ego, A_CRUISE_MIN_BP_DF, A_CRUISE_MIN_VALS_DF)

def get_max_accel_df(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_DF, A_CRUISE_MAX_VALS_DF)

def get_max_accel_toyota(v_ego):
  return interp(v_ego, A_CRUISE_MAX_BP_TOYOTA, A_CRUISE_MAX_VALS_TOYOTA)

def limit_accel_in_turns(v_ego, angle_steers, a_target, CP):
  """
  This function returns a limited long acceleration allowed, depending on the existing lateral acceleration
  this should avoid accelerating when losing the target in turns
  """

  # FIXME: This function to calculate lateral accel is incorrect and should use the VehicleModel
  # The lookup table for turns should also be updated if we do this
  a_total_max = interp(v_ego, _A_TOTAL_MAX_BP, _A_TOTAL_MAX_V)
  a_y = v_ego ** 2 * angle_steers * CV.DEG_TO_RAD / (CP.steerRatio * CP.wheelbase)
  a_x_allowed = math.sqrt(max(a_total_max ** 2 - a_y ** 2, 0.))

  return [a_target[0], min(a_target[1], a_x_allowed)]


class LongitudinalPlanner:
  def __init__(self, CP, init_v=0.0, init_a=0.0, dt=DT_MDL):
    self.CP = CP
    self.mpc = LongitudinalMpc(CP, dt=dt)
    self.fcw = False
    self.dt = dt

    self.a_desired = init_a
    self.v_desired_filter = FirstOrderFilter(init_v, 2.0, self.dt)
    self.v_model_error = 0.0

    self.v_desired_trajectory = np.zeros(CONTROL_N)
    self.a_desired_trajectory = np.zeros(CONTROL_N)
    self.j_desired_trajectory = np.zeros(CONTROL_N)
    self.solverExecutionTime = 0.0
    self.params = Params()
    self.override_slc = False
    self.overridden_speed = 0
    self.slc_target = 0
    self.dynamic_follow = False
    self.dynamic_follow = self.params.get_bool("Marc_Dynamic_Follow")

  @staticmethod
  def parse_model(model_msg, model_error):
    if (len(model_msg.position.x) == ModelConstants.IDX_N and
       len(model_msg.velocity.x) == ModelConstants.IDX_N and
       len(model_msg.acceleration.x) == ModelConstants.IDX_N):
      x = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.position.x) - model_error * T_IDXS_MPC
      v = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.velocity.x) - model_error
      a = np.interp(T_IDXS_MPC, ModelConstants.T_IDXS, model_msg.acceleration.x)
      j = np.zeros(len(T_IDXS_MPC))
    else:
      x = np.zeros(len(T_IDXS_MPC))
      v = np.zeros(len(T_IDXS_MPC))
      a = np.zeros(len(T_IDXS_MPC))
      j = np.zeros(len(T_IDXS_MPC))
    return x, v, a, j

  def update(self, sm):
    self.mpc.mode = 'blended' if sm['controlsState'].experimentalMode else 'acc'

    v_ego = sm['carState'].vEgo
    v_ego_raw = sm['carState'].vEgoRaw
    v_ego_cluster = sm['carState'].vEgoCluster
    v_ego_diff = v_ego_raw - v_ego_cluster if v_ego_cluster > 0 else 0
    v_cruise_kph = min(sm['controlsState'].vCruise, V_CRUISE_MAX)
    v_cruise = v_cruise_kph * CV.KPH_TO_MS

    long_control_off = sm['controlsState'].longControlState == LongCtrlState.off
    force_slow_decel = sm['controlsState'].forceDecel

    # Reset current state when not engaged, or user is controlling the speed
    reset_state = long_control_off if self.CP.openpilotLongitudinalControl else not sm['controlsState'].enabled

    # No change cost when user is controlling the speed, or when standstill
    prev_accel_constraint = not (reset_state or sm['carState'].standstill)

    if self.mpc.mode == 'acc':
      if self.CP.carName == "toyota":
        accel_limits = [get_min_accel(v_ego), get_max_accel_toyota(v_ego)]
      elif self.dynamic_follow:
        accel_limits = [get_min_accel_df(v_ego), get_max_accel_df(v_ego)]
      else:
        accel_limits = [A_CRUISE_MIN, get_max_accel(v_ego)]
      accel_limits_turns = limit_accel_in_turns(v_ego, sm['carState'].steeringAngleDeg, accel_limits, self.CP)
    else:
      accel_limits = [ACCEL_MIN, ACCEL_MAX]
      accel_limits_turns = [ACCEL_MIN, ACCEL_MAX]

    if reset_state:
      self.v_desired_filter.x = v_ego
      # Clip aEgo to cruise limits to prevent large accelerations when becoming active
      self.a_desired = clip(sm['carState'].aEgo, accel_limits[0], accel_limits[1])

    # Prevent divergence, smooth in current v_ego
    self.v_desired_filter.x = max(0.0, self.v_desired_filter.update(v_ego))
    # Compute model v_ego error
    self.v_model_error = get_speed_error(sm['modelV2'], v_ego)

    if force_slow_decel:
      v_cruise = 0.0
    # clip limits, cannot init MPC outside of bounds
    accel_limits_turns[0] = min(accel_limits_turns[0], self.a_desired + 0.05)
    accel_limits_turns[1] = max(accel_limits_turns[1], self.a_desired - 0.05)

    # PFEIFER - SLC {{
    carState = sm['carState']
    enabled = sm['controlsState'].enabled

    if self.params.get_bool("SpeedLimitControl"):
      slc.update_current_max_velocity(v_cruise_kph * CV.KPH_TO_MS, v_ego, sm['carState'].aEgo)
      desired_speed_limit = slc.speed_limit + 1.5 + v_ego_diff

      # Override SLC upon gas pedal press and reset upon brake/cancel button
      self.override_slc |= carState.gasPressed
      self.override_slc &= enabled
      self.override_slc &= v_ego > desired_speed_limit

      # Set the max speed to the manual set speed
      if carState.gasPressed:
        self.overridden_speed = np.clip(v_ego, desired_speed_limit, v_cruise)
      self.overridden_speed *= enabled

      # Use the speed limit if its not being overridden
      if not self.override_slc:
        if slc.speed_limit > 0 and desired_speed_limit < v_cruise:
          self.slc_target = desired_speed_limit
          v_cruise = self.slc_target
      else:
        self.slc_target = self.overridden_speed
    # }} PFEIFER - SLC
    # PFEIFER - VTSC {{
    vtsc.update(prev_accel_constraint, v_ego, sm)
    if vtsc.active and v_cruise > vtsc.v_target:
      v_cruise = vtsc.v_target
    # }} PFEIFER - VTSC

    lead_xv_0 = self.mpc.process_lead(sm['radarState'].leadOne)
    lead_xv_1 = self.mpc.process_lead(sm['radarState'].leadTwo)
    v_lead0 = lead_xv_0[0,1]
    v_lead1 = lead_xv_1[0,1]
    self.mpc.set_weights(prev_accel_constraint, personality=sm['controlsState'].personality, v_lead0=v_lead0, v_lead1=v_lead1)
    self.mpc.set_accel_limits(accel_limits_turns[0], accel_limits_turns[1])
    self.mpc.set_cur_state(self.v_desired_filter.x, self.a_desired)
    x, v, a, j = self.parse_model(sm['modelV2'], self.v_model_error)
    self.mpc.update(sm['radarState'], v_cruise, x, v, a, j, personality=sm['controlsState'].personality, dynamic_follow=self.dynamic_follow)

    self.v_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.v_solution)
    self.a_desired_trajectory_full = np.interp(ModelConstants.T_IDXS, T_IDXS_MPC, self.mpc.a_solution)
    self.v_desired_trajectory = self.v_desired_trajectory_full[:CONTROL_N]
    self.a_desired_trajectory = self.a_desired_trajectory_full[:CONTROL_N]
    self.j_desired_trajectory = np.interp(ModelConstants.T_IDXS[:CONTROL_N], T_IDXS_MPC[:-1], self.mpc.j_solution)

    # TODO counter is only needed because radar is glitchy, remove once radar is gone
    self.fcw = self.mpc.crash_cnt > 2 and not sm['carState'].standstill
    if self.fcw:
      cloudlog.info("FCW triggered")

    # Interpolate 0.05 seconds and save as starting point for next iteration
    a_prev = self.a_desired
    self.a_desired = float(interp(self.dt, ModelConstants.T_IDXS[:CONTROL_N], self.a_desired_trajectory))
    self.v_desired_filter.x = self.v_desired_filter.x + self.dt * (self.a_desired + a_prev) / 2.0

  def publish(self, sm, pm):
    plan_send = messaging.new_message('longitudinalPlan')

    plan_send.valid = sm.all_checks(service_list=['carState', 'controlsState'])

    longitudinalPlan = plan_send.longitudinalPlan
    longitudinalPlan.modelMonoTime = sm.logMonoTime['modelV2']
    longitudinalPlan.processingDelay = (plan_send.logMonoTime / 1e9) - sm.logMonoTime['modelV2']

    longitudinalPlan.speeds = self.v_desired_trajectory.tolist()
    longitudinalPlan.accels = self.a_desired_trajectory.tolist()
    longitudinalPlan.jerks = self.j_desired_trajectory.tolist()

    longitudinalPlan.hasLead = sm['radarState'].leadOne.status
    longitudinalPlan.longitudinalPlanSource = self.mpc.source
    longitudinalPlan.fcw = self.fcw

    longitudinalPlan.solverExecutionTime = self.mpc.solve_time

    pm.send('longitudinalPlan', plan_send)
