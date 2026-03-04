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

# ============================================================
# long_mpc.py（ACADOS）
# 方法B版本：由 LongitudinalPlanner 明確傳入 a_min / a_max
# - a_min/a_max 會寫入 self.params[:,0/1]，並透過 slack constraint 生效
# - ACC / blended 都會套用同一組 a_min/a_max（因此 blended 也能套用 A_CRUISE_MAX）
#
# 重要：要讓 blended 套用 A_CRUISE_MAX，Planner 必須在 blended 模式也把
#      a_max 設為 get_max_accel(v_ego)（或 per-stage array），並傳進來。
# ============================================================

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

# =========================
# 成本/權重（依你原本設定）
# =========================
X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.8 #0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# =========================
# 時域設定
# =========================
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]
T_IDXS = np.array(T_IDXS_LST)
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])

# =========================
# 物理/限制設定
# =========================
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6


def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  """人格 -> jerk factor（影響 a_change / j 成本）"""
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.5 #1.0
  elif personality == log.LongitudinalPersonality.standard:
    return 1.0
  elif personality == log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(v_ego, personality=log.LongitudinalPersonality.standard):
  """速度相依 T_FOLLOW（你原本邏輯）"""
  v_kph = float(v_ego * 3.6)

  if personality == log.LongitudinalPersonality.relaxed:
    base = 1.0 + 0.0030 * v_kph #0.0026
  elif personality == log.LongitudinalPersonality.standard:
    base = 1.0 + 0.0024 * v_kph #0.0025
  elif personality == log.LongitudinalPersonality.aggressive:
    base = 1.0 #+ 0.0022 * v_kph #0.0020
  else:
    raise NotImplementedError("Longitudinal personality not supported")

  return base


def get_stopped_equivalence_factor(v_lead, v_ego, a_lead, d_rel):
  """
  低速更積極縮短跟車距離（你原本版本，保留）
  加入改良：引入前車加速度抑制與距離天花板，解決塞車走走停停暴衝的問題
  """
  v_lead = np.asarray(v_lead, dtype=float)
  v_ego = float(v_ego)
  a_lead = np.asarray(a_lead, dtype=float)
  d_rel = np.asarray(d_rel, dtype=float)

  v10 = 10.0 * CV.KPH_TO_MS
  v50 = 50.0 * CV.KPH_TO_MS
  v60 = 60.0 * CV.KPH_TO_MS

  delta = v_lead - v_ego

  w_k     = np.clip(1.0 - (v_ego / v60), 0.0, 1.0)
  w_quick = np.clip(1.0 - (v_ego / v50), 0.0, 1.0)
  w_base  = np.clip(1.0 - (v_ego / v10), 0.0, 1.0)

  k_low, k_high = 5.5, 3.5
  k = k_high + (k_low - k_high) * w_k

  quad_gain = 0.75
  # 稍微限制極低速時的二次方增益最大值，平滑起步
  quick = quad_gain * (np.clip(delta, 0.0, 3.0) ** 2) * w_quick

  base = k * np.maximum(delta, 0.0) * (0.6 + 0.4 * w_base) * w_base

  v_diff_offset = base + quick
  
  # --- 新增邏輯 1：前車加速度抑制 ---
  # 如果前車正在煞車 (a_lead < 0)，等比例削弱 offset，提早收油門
  accel_factor = np.clip(1.0 + (a_lead / 2.0), 0.0, 1.0)
  v_diff_offset *= accel_factor

  # --- 新增邏輯 2：真實距離動態上限 ---
  # 不允許虛擬距離無限膨脹，最多只能是當前實際距離的 40%
  dynamic_cap = np.clip(d_rel * 0.4, 0.0, 8.0)
  
  # 保留你原本的靜態上限邏輯作為雙重保險
  cap_low, cap_high = 8.0, 5.0
  static_cap = cap_high + (cap_low - cap_high) * w_k
  
  final_cap = np.minimum(dynamic_cap, static_cap)
  v_diff_offset = np.clip(v_diff_offset, 0.0, final_cap)

  return (v_lead ** 2) / (2 * COMFORT_BRAKE) + v_diff_offset


def get_safe_obstacle_distance(v_ego, t_follow):
  """安全距離：煞停距離 + 跟車時間 + 停車距離"""
  return (v_ego ** 2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  # 由於 get_stopped_equivalence_factor 參數改變，這裡暫時給予預設值 0.0
  # 通常這個函式是外部用來畫圖或 debug 用的，主迴圈中沒用到
  if t_follow is None:
    t_follow = get_T_FOLLOW(v_ego)
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead, v_ego, 0.0, 100.0)


# ============================================================
# ACADOS model / ocp 生成
# ============================================================

def gen_long_model():
  model = AcadosModel()
  model.name = MODEL_NAME

  # states: x, v, a
  x_ego = SX.sym('x_ego')
  v_ego = SX.sym('v_ego')
  a_ego = SX.sym('a_ego')
  model.x = vertcat(x_ego, v_ego, a_ego)

  # control: jerk
  j_ego = SX.sym('j_ego')
  model.u = vertcat(j_ego)

  # xdot
  x_ego_dot = SX.sym('x_ego_dot')
  v_ego_dot = SX.sym('v_ego_dot')
  a_ego_dot = SX.sym('a_ego_dot')
  model.xdot = vertcat(x_ego_dot, v_ego_dot, a_ego_dot)

  # live parameters（會在 run() 逐 stage set('p', params[i])）
  # p = [a_min, a_max, x_obstacle, prev_a, t_follow, lead_danger_factor]
  a_min = SX.sym('a_min')
  a_max = SX.sym('a_max')
  x_obstacle = SX.sym('x_obstacle')
  prev_a = SX.sym('prev_a')
  lead_t_follow = SX.sym('lead_t_follow')
  lead_danger_factor = SX.sym('lead_danger_factor')
  model.p = vertcat(a_min, a_max, x_obstacle, prev_a, lead_t_follow, lead_danger_factor)

  # dynamics
  f_expl = vertcat(v_ego, a_ego, j_ego)
  model.f_impl_expr = model.xdot - f_expl
  model.f_expl_expr = f_expl
  return model


def gen_long_ocp():
  ocp = AcadosOcp()
  ocp.model = gen_long_model()

  Tf = T_IDXS[-1]

  ocp.dims.N = N

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

  ocp.cost.yref = np.zeros((COST_DIM,))
  ocp.cost.yref_e = np.zeros((COST_E_DIM,))

  desired_dist_comfort = get_safe_obstacle_distance(v_ego, lead_t_follow)

  # cost_y: [dist_error, x, v, a, a-prev_a, j]
  costs = [
    ((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
    x_ego,
    v_ego,
    a_ego,
    a_ego - prev_a,
    j_ego
  ]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # slack constraints:
  # 0) v_ego >= 0  -> v_ego
  # 1) a_ego >= a_min -> (a_ego - a_min)
  # 2) a_ego <= a_max -> (a_max - a_ego)
  # 3) keep distance -> ((x_obs-x_ego) - lead_danger_factor*desired_dist)/(...)

  constraints = vertcat(
    v_ego,
    (a_ego - a_min),
    (a_max - a_ego),
    ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.)
  )
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0

  # 初始參數值（runtime 會覆蓋）
  ocp.parameter_values = np.array([ACCEL_MIN, ACCEL_MAX, 0.0, 0.0, 1.0, LEAD_DANGER_FACTOR], dtype=float)

  # slack cost weights：runtime 再 set
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4 * np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

  ocp.code_export_directory = EXPORT_DIR
  return ocp


# ============================================================
# LongitudinalMpc class
# ============================================================

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
    self.u_sol = np.zeros((N, 1))

    # params: (N+1, PARAM_DIM) = [a_min, a_max, x_obs, prev_a, t_follow, lead_danger]
    self.params = np.zeros((N+1, PARAM_DIM))
    for i in range(N+1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0

    # timings
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0

    self.x0 = np.zeros(X_DIM)

    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    """設定 cost matrix 與 slack cost（constraint 的 L2 代價）"""
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # (a - prev_a) 的 cost 在後段衰減，避免 horizon 後段過度抑制加速度變化
      W[4, 4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
      self.solver.cost_set(i, 'W', W)

    # terminal cost（沒有 jerk 那一項）
    self.solver.cost_set(N, 'W', np.copy(W[:COST_E_DIM, :COST_E_DIM]))

    Zl = np.array(constraint_cost_weights, dtype=float)
    for i in range(N):
      self.solver.cost_set(i, 'Zl', Zl)

  def set_weights(self, prev_accel_constraint=True, personality=log.LongitudinalPersonality.standard):
    """依 mode/personality 設定成本權重"""
    jerk_factor = get_jerk_factor(personality)

    if self.mode == 'acc':
      a_change_cost = A_CHANGE_COST if prev_accel_constraint else 0
      cost_weights = [
        X_EGO_OBSTACLE_COST,
        X_EGO_COST,
        V_EGO_COST,
        A_EGO_COST,
        jerk_factor * a_change_cost,
        jerk_factor * J_EGO_COST
      ]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]

    elif self.mode == 'blended':
      a_change_cost = 40.0 if prev_accel_constraint else 0
      cost_weights = [0., 0.1, 0.2, 5.0, a_change_cost, 1.0]
      constraint_cost_weights = [LIMIT_COST, LIMIT_COST, LIMIT_COST, DANGER_ZONE_COST]

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner cost set')

    self.set_cost_weights(cost_weights, constraint_cost_weights)

  def set_cur_state(self, v, a):
    """設定當前狀態（x0）"""
    v_prev = self.x0[1]
    self.x0[1] = v
    self.x0[2] = a
    # 大跳變時，用同一個初始 guess 幫助收斂
    if abs(v_prev - v) > 2.:
      for i in range(N+1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    """前車軌跡外推（a 指數衰減）"""
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS ** 2) / 2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    return np.column_stack((x_lead_traj, v_lead_traj))

  def process_lead(self, lead):
    """把 leadOne/leadTwo 轉成可用的 lead trajectory"""
    v_ego = self.x0[1]
    if lead is not None and lead.status:
      x_lead = lead.dRel
      v_lead = lead.vLead
      a_lead = lead.aLeadK
      a_lead_tau = lead.aLeadTau
    else:
      # 沒有 lead 時，用一台遠且更快的假車，避免模式跳來跳去
      x_lead = 50.0
      v_lead = v_ego + 10.0
      a_lead = 0.0
      a_lead_tau = _LEAD_ACCEL_TAU

    # 避免 MPC 在「立即撞上」情境崩潰：夾住到仍可煞停的最小距離
    min_x_lead = ((v_ego + v_lead) / 2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = np.clip(x_lead, min_x_lead, 1e8)

    v_lead = np.clip(v_lead, 0.0, 1e8)
    a_lead = np.clip(a_lead, -10., 5.)

    return self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

  def _to_stage_array(self, val, default, name: str):
    """
    將輸入參數轉成 (N+1,) stage array
    - val 可為：
      1) None -> default
      2) scalar -> broadcast (N+1,)
      3) array/list (N+1,) -> 使用
    """
    if val is None:
      val = default

    arr = np.asarray(val, dtype=float)
    if arr.ndim == 0:
      return np.full(N+1, float(arr), dtype=float)

    if arr.shape == (N+1,):
      return arr.astype(float, copy=True)

    raise ValueError(f"[long_mpc] {name} shape must be scalar or (N+1,), got {arr.shape}")

  def update(self, radarstate, v_cruise, x, v, a, j,
             personality=log.LongitudinalPersonality.standard,
             a_min=None, a_max=None):
    """
    方法B：由 Planner 明確傳入 a_min/a_max（可 scalar 或 (N+1,)）
    - 這個 a_min/a_max 會在 ACC 與 blended 都生效（因此 blended 也能套用 A_CRUISE_MAX）
    """

    v_ego = self.x0[1]
    t_follow = get_T_FOLLOW(v_ego, personality)

    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0 = self.process_lead(radarstate.leadOne)
    lead_xv_1 = self.process_lead(radarstate.leadTwo)

    # 取出當前的前車加速度給抑制器使用，若無車則設為0
    a_lead_0 = np.clip(radarstate.leadOne.aLeadK, -10., 5.) if radarstate.leadOne.status else 0.0
    a_lead_1 = np.clip(radarstate.leadTwo.aLeadK, -10., 5.) if radarstate.leadTwo.status else 0.0

    # moving lead -> equivalent stopped obstacle distance
    # 將外推的位置 lead_xv_X[:, 0] 作為實際相對距離 d_rel 傳入
    lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor(lead_xv_0[:, 1], v_ego, a_lead_0, lead_xv_0[:, 0])
    lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor(lead_xv_1[:, 1], v_ego, a_lead_1, lead_xv_1[:, 0])

    # ========= 核心：吃進 Planner 給的 a_min/a_max =========
    a_min_arr = self._to_stage_array(a_min, ACCEL_MIN, "a_min")
    a_max_arr = self._to_stage_array(a_max, ACCEL_MAX, "a_max")

    # 防呆：確保 a_min <= a_max
    eps = 1e-3
    a_max_arr = np.maximum(a_max_arr, a_min_arr + eps)

    self.params[:, 0] = a_min_arr
    self.params[:, 1] = a_max_arr
    # =====================================================

    if self.mode == 'acc':
      self.params[:, 5] = LEAD_DANGER_FACTOR

      # cruise obstacle：建議尊重 a_max（避免 reference 假設能更大加速）
      # 用 min(a_max_arr, CRUISE_MAX_ACCEL) 形成每個 stage 可達速度上界
      a_upper_eff = np.minimum(a_max_arr, CRUISE_MAX_ACCEL)

      v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
      v_upper = v_ego + (T_IDXS * a_upper_eff * 1.05)

      v_cruise_clipped = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)

      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # ACC mode 不使用 model x/v/a/j reference
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      self.params[:, 5] = LEAD_DANGER_FACTOR #1.0

      # blended：同樣讓 cruise_target 的「可達速度」尊重 a_max，
      # 避免目標 x 走得太快，造成 MPC 必須靠 constraint 硬頂住（體感可能怪）
      a_upper_eff = np.minimum(a_max_arr, CRUISE_MAX_ACCEL)
      v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
      v_upper = v_ego + (T_IDXS * a_upper_eff * 1.05)
      v_cruise_profile = np.clip(v_cruise * np.ones(N+1), v_lower, v_upper)

      # 兩個 lead 的 obstacle（blended 下只用 lead）
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])

      # cruise_target：用「可達速度曲線」積分得到 x（比 T_IDXS*v_cruise 更一致）
      cruise_target = np.cumsum(T_DIFFS * v_cruise_profile) + x[0]

      # model x：從 v 積分回 x，讓 x 單調且一致
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x_model = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x_model, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1, 0] < x_and_cruise[1, 1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    # === 設定 yref（cost reference）===
    self.yref[:, 1] = x
    self.yref[:, 2] = v
    self.yref[:, 3] = a
    self.yref[:, 5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    # === 設定 params（p）===
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.prev_a)
    self.params[:, 4] = t_follow

    self.run()

    # === FCW heuristic（保留原邏輯）===
    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # blended 下：如果解出來已進入 lead 舒適距離內，標記 source
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:, 1], t_follow)) - self.x_sol[:, 0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:, 1], t_follow)) - self.x_sol[:, 0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    """把 params/states 餵進 solver，solve 後取解"""
    for i in range(N+1):
      self.solver.set(i, 'p', self.params[i])

    # 初始狀態 hard constraint
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

    self.v_solution = self.x_sol[:, 1]
    self.a_solution = self.x_sol[:, 2]
    self.j_solution = self.u_sol[:, 0]

    # prev_a：給下一輪 a_change cost 用
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

