#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.constants import CV
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat

MODEL_NAME = 'long'
LONG_MPC_DIR = os.path.dirname(os.path.abspath(__file__))
EXPORT_DIR = os.path.join(LONG_MPC_DIR, "c_generated_code")
JSON_FILE = os.path.join(LONG_MPC_DIR, "acados_ocp_long.json")

SOURCES = ['lead0', 'lead1', 'cruise', 'e2e']

X_DIM = 3
U_DIM = 1
PARAM_DIM = 6
COST_E_DIM = 5
COST_DIM = COST_E_DIM + 1
CONSTR_DIM = 4

# ====================== 可調參數區（TUNING PARAMS） ======================
# 注意：依你要求，不改動 T_IDXS；此檔只做「動態 t_follow / danger factor」提升反應速度

# --- 前車風險判定（越大越早減速） ---
LEAD_BRAKE_A_THRESH = -0.6       # m/s^2：前車減速開始算有感
LEAD_BRAKE_A_STRONG = -1.6       # m/s^2：強減速，風險飽和
CLOSING_VREL_OFFSET = 0.5        # m/s：略過小抖動
CLOSING_VREL_RANGE = 7.0         # m/s：接近速度飽和範圍（越小越敏感）

# --- 動態加大跟車距離（t_follow） ---
EXTRA_T_FOLLOW_MAX = 0.35        # 秒：最大額外 t_follow（建議 0.2~0.5）
EXTRA_T_FOLLOW_V_BP = [0.0, 8.0, 20.0]          # m/s
EXTRA_T_FOLLOW_V_SC = [0.55, 0.85, 1.0]         # 低速不放太大，避免爬行過度保守

# --- 動態提高 danger zone 約束敏感度（lead_danger_factor）---
# lead_danger_factor 越大 → 約束越早生效 → 更早開始減速（更安全/更敏感）
LEAD_DANGER_FACTOR_BASE = 0.75   # 原本常數（沿用）
LEAD_DANGER_FACTOR_MAX = 1.05    # 風險高時的上限（建議 0.95~1.10）

# 只加強 horizon 前段，後段回到 base（避免高速巡航過度緊張）
DANGER_FACTOR_HOLD_S = 1.2       # 秒：前段維持加強
DANGER_FACTOR_FADE_S = 3.0       # 秒：淡出到 base
# =======================================================================

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = LEAD_DANGER_FACTOR_BASE
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

# 依你要求：不改動 T_IDXS
T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])

COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6


def _clip01(x: float) -> float:
  return float(np.clip(x, 0.0, 1.0))


def _lead_risk_from_state(v_ego: float, lead) -> float:
  """
  回傳 0~1 的風險分數：
  - 前車減速越強 → 越接近 1
  - 接近速度越大（v_rel 越負）→ 越接近 1
  """
  if lead is None or (not getattr(lead, 'status', False)):
    return 0.0

  try:
    a_lead = float(getattr(lead, 'aLeadK', 0.0))
    v_lead = float(getattr(lead, 'vLead', v_ego))
  except Exception:
    return 0.0

  v_rel = v_lead - v_ego  # <0 表示正在接近

  # 1) 減速風險（a 更負越高）
  if a_lead >= LEAD_BRAKE_A_THRESH:
    a_score = 0.0
  else:
    denom = (LEAD_BRAKE_A_STRONG - LEAD_BRAKE_A_THRESH)
    a_score = _clip01((LEAD_BRAKE_A_THRESH - a_lead) / max(1e-3, denom))

  # 2) 接近風險（v_rel 越負越高）
  closing = max(0.0, -(v_rel) - CLOSING_VREL_OFFSET)
  v_score = _clip01(closing / max(1e-3, CLOSING_VREL_RANGE))

  return float(max(a_score, v_score))


def _danger_factor_shape(t: np.ndarray) -> np.ndarray:
  """
  前段加強、後段淡出到 0 的形狀（0~1）
  """
  return np.interp(t, [0.0, DANGER_FACTOR_HOLD_S, DANGER_FACTOR_FADE_S], [1.0, 1.0, 0.0])


def _extra_t_follow_scale(v_ego: float) -> float:
  """
  低速縮小 extra t_follow，避免爬行過度保守
  """
  return float(np.interp(v_ego, EXTRA_T_FOLLOW_V_BP, EXTRA_T_FOLLOW_V_SC))


def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality == log.LongitudinalPersonality.standard:
    return 1.0
  elif personality == log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(v_ego, personality=log.LongitudinalPersonality.standard):
  v_kph = float(v_ego * 3.6)

  if personality == log.LongitudinalPersonality.relaxed:
    base = 1.0 + 0.0030 * v_kph
  elif personality == log.LongitudinalPersonality.standard:
    base = 1.0 + 0.0025 * v_kph
  elif personality == log.LongitudinalPersonality.aggressive:
    base = 1.0 + 0.0020 * v_kph
  else:
    raise NotImplementedError("Longitudinal personality not supported")

  return float(base)


def get_stopped_equivalence_factor(v_lead, v_ego):
  """
  目標：
  - 在低速時更積極縮短跟車距離，減少減速後再加速的遲滯
  - offset 對 (v_lead - v_ego) 呈現更線性且可預期的放大
  - 高速時自動收斂到較保守（接近既有邏輯）
  """
  v_lead = np.asarray(v_lead, dtype=float)
  v_ego = float(v_ego)

  v10 = 10.0 * CV.KPH_TO_MS
  v50 = 50.0 * CV.KPH_TO_MS
  v60 = 60.0 * CV.KPH_TO_MS

  delta = v_lead - v_ego

  w_k     = np.clip(1.0 - (v_ego / v60), 0.0, 1.0)
  w_quick = np.clip(1.0 - (v_ego / v50), 0.0, 1.0)
  w_base  = np.clip(1.0 - (v_ego / v10), 0.0, 1.0)

  k_low, k_high = 5.5, 3.5
  k = k_high + (k_low - k_high) * w_k

  quad_gain = 0.35
  quick = quad_gain * (np.clip(delta, 0.0, 5.0) ** 2) * w_quick

  base = k * np.maximum(delta, 0.0) * (0.6 + 0.4 * w_base) * w_base

  v_diff_offset = base + quick
  cap_low, cap_high = 8.0, 5.0
  cap = cap_high + (cap_low - cap_high) * w_k
  v_diff_offset = np.clip(v_diff_offset, 0.0, cap)

  return (v_lead**2) / (2 * COMFORT_BRAKE) + v_diff_offset


def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego**2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW(v_ego)
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead, v_ego)


def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # set up states & controls
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # controls
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor)

  # dynamics model
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  # set dimensions
  ocp.dims.N = N

  # set cost module
  ocp.cost.cost_type = 'NONLINEAR_LS'
  ocp.cost.cost_type_e = 'NONLINEAR_LS'

  QR = np.zeros((COST_DIM, COST_DIM))
  Q = np.zeros((COST_E_DIM, COST_E_DIM))

  ocp.cost.W = QR
  ocp.cost.W_e = Q

  x_ego, v_ego, a_ego = ocp.model.x[0], ocp.model.x[1], ocp.model.x[2]
  j_ego = ocp.model.u[0]

  a_min, a_max = ocp.model.p[0], ocp.model.p[1]
  x_obstacle = ocp.model.p[2]
  prev_a = ocp.model.p[3]
  lead_t_follow = ocp.model.p[4]
  lead_danger_factor = ocp.model.p[5]

  ocp.cost.yref = np.zeros((COST_DIM, ))
  ocp.cost.yref_e = np.zeros((COST_E_DIM, ))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([-1.2, 1.2, 0.0, 0.0, 1.0, LEAD_DANGER_FACTOR])

  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4*np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS  # 不改動

  ocp.code_export_directory = EXPORT_DIR
  return ocp


class LongitudinalMpc:
  def __init__(self, mode='acc', dt=DT_MDL):
    self.mode = mode
    self.dt = dt
    self.solver = AcadosOcpSolverCython(MODEL_NAME, ACADOS_SOLVER_TYPE, N)
    self.reset()
    self.source = SOURCES[2]

  def reset(self):
    self.solver.reset()
    self.v_solution = np.zeros(N+1)
    self.a_solution = np.zeros(N+1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)
    self.yref = np.zeros((N+1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])
    self.x_sol = np.zeros((N+1, X_DIM))
    self.u_sol = np.zeros((N,1))
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))
    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0
    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      W[4,4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    jerk_factor = get_jerk_factor(personality)
    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
      cost_weights = [X_EGO_OBSTACLE_COST, X_EGO_COST, V_EGO_COST, A_EGO_COST,
                      jerk_factor * a_change_cost, jerk_factor * J_EGO_COST]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]
    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')
    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    if abs(v_prev - v) > 2.:
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS**2)/2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def process_lead(self, lead):
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    min_x_lead = ((v_ego + v_lead)/2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)
    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)
    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)
    return lead_xv

  def update(self, radarstate, v_cruise, x, v, a, j, personality=log.LongitudinalPersonality.standard):
    v_ego = self.x0[1]

    # === 1) 先估計 lead 風險（不改 T_IDXS，只改參數輸入讓 MPC 更早反應）===
    risk0 = _lead_risk_from_state(v_ego, radarstate.leadOne)
    risk1 = _lead_risk_from_state(v_ego, radarstate.leadTwo)
    lead_risk = float(max(risk0, risk1))

    # === 2) 動態 t_follow：前車減速/接近時增加跟車距離（更早開始減速）===
    t_follow_base = get_T_FOLLOW(v_ego, personality)
    v_scale = _extra_t_follow_scale(v_ego)
    t_follow = float(t_follow_base + EXTRA_T_FOLLOW_MAX * lead_risk * v_scale)

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    lead_0_obstacle = lead_xv_0[:,0] + get_stopped_equivalence_factor(lead_xv_0[:,1], v_ego)
    lead_1_obstacle = lead_xv_1[:,0] + get_stopped_equivalence_factor(lead_xv_1[:,1], v_ego)

    self.params[:,0] = ACCEL_MIN
    self.params[:,1] = ACCEL_MAX

    # Update in ACC mode or ACC/e2e blend
    if self.mode == 'acc':
      # === 3) 動態 danger factor：風險高時提高敏感度，且只在前段加強 ===
      df_base = LEAD_DANGER_FACTOR_BASE
      df_max = LEAD_DANGER_FACTOR_MAX
      df_boost = (df_max - df_base) * lead_risk
      df_vec = df_base + df_boost * _danger_factor_shape(T_IDXS)
      self.params[:,5] = np.clip(df_vec, 0.60, 1.20)

      v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
      v_upper = v_ego + (T_IDXS * CRUISE_MAX_ACCEL * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1),
                                 v_lower,
                                 v_upper)
      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      # blended 原本就比較嚴格，維持 1.0（不額外變動，避免切換副作用）
      self.params[:,5] = 1.0

      x_obstacles = np.column_stack([lead_0_obstacle,
                                     lead_1_obstacle])
      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1,0] < x_and_cruise[1,1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    self.yref[:,1] = x
    self.yref[:,2] = v
    self.yref[:,3] = a
    self.yref[:,5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    self.params[:,2] = np.min(x_obstacles, axis=1)
    self.params[:,3] = np.copy(self.prev_a)
    self.params[:,4] = t_follow

    self.run()
    if (np.any(lead_xv_0[FCW_IDXS,0] - self.x_sol[FCW_IDXS,0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow)) - self.x_sol[:,0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:,1], t_follow)) - self.x_sol[:,0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N+1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:,1]
    self.a_solution = self.x_sol[:,2]
    self.j_solution = self.u_sol[:,0]

    self.prev_a = np.interp(T_IDXS + self.dt, T_IDXS, self.a_solution)

    t = time.monotonic()
    if self.solution_status != 0:
      if t > self.last_cloudlog_t + 5.0:
        self.last_cloudlog_t = t
        cloudlog.warning(f"Long mpc reset, solution_status: {self.solution_status}")
      self.reset()


if __name__ == "__main__":
  ocp = gen_long_ocp()
  AcadosOcpSolver.generate(ocp, json_file=JSON_FILE)
  # AcadosOcpSolver.build(ocp.code_export_directory, with_cython=True)
