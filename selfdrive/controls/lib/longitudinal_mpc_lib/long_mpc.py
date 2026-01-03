#!/usr/bin/env python3
import os
import time
import numpy as np
from cereal import log
from opendbc.car.interfaces import ACCEL_MIN, ACCEL_MAX
from openpilot.common.realtime import DT_MDL
from openpilot.common.swaglog import cloudlog
# WARNING: imports outside of constants will not trigger a rebuild
from openpilot.selfdrive.modeld.constants import index_function
from openpilot.selfdrive.controls.radard import _LEAD_ACCEL_TAU

if __name__ == '__main__':  # generating code
  from openpilot.third_party.acados.acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver
else:
  from openpilot.selfdrive.controls.lib.longitudinal_mpc_lib.c_generated_code.acados_ocp_solver_pyx import AcadosOcpSolverCython

from casadi import SX, vertcat


# ============================================================================
# long_mpc：配合你已優化的 radard（看得準、看得快）+ longitudinal_planner（SNG guard）
#
# 本版優化目標（只做“配合”，不跟 planner 打架）：
#  1) 對「前車突然再煞停」更敏感：不要把負加速度在 MPC 預測裡太快衰減掉
#  2) moving lead 的 stopped-equivalence 考慮 lead 正在煞：lead 煞越大 → 等效距離更小 → 障礙更近
#  3) 不覆蓋 planner 送進來的 accel 限制（a_min/a_max），讓 DTSC / SNG 上限能真正生效
#
# 注意：
#  - 不改動 T_IDXS（你先前要求）
#  - 起步保守/限制正向加速度：由 planner 負責（這裡只讓“煞車反應更快更真實”）
# ============================================================================


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

X_EGO_OBSTACLE_COST = 3.
X_EGO_COST = 0.
V_EGO_COST = 0.
A_EGO_COST = 0.
J_EGO_COST = 5.0
A_CHANGE_COST = 200.
DANGER_ZONE_COST = 100.
CRASH_DISTANCE = .25
LEAD_DANGER_FACTOR = 0.75
LIMIT_COST = 1e6
ACADOS_SOLVER_TYPE = 'SQP_RTI'

# Fewer timestamps don't hurt performance and lead to
# much better convergence of the MPC with low iterations
N = 12
MAX_T = 10.0
T_IDXS_LST = [index_function(idx, max_val=MAX_T, max_idx=N) for idx in range(N+1)]

T_IDXS = np.array(T_IDXS_LST)          # ⚠️ 不改
FCW_IDXS = T_IDXS < 5.0
T_DIFFS = np.diff(T_IDXS, prepend=[0.])
COMFORT_BRAKE = 2.5
STOP_DISTANCE = 6.0
CRUISE_MIN_ACCEL = -1.2
CRUISE_MAX_ACCEL = 1.6


# ====================== 可調參數區（TUNING PARAMS） ======================
# 1) 是否保留外部（planner）寫入的 a_min/a_max 限制
#    - True：若 planner 有先寫 self.params[:,0/1]（例如 DTSC 或你 SNG 上限收緊），MPC 會使用
#    - False：強制 MPC 自己用 fallback（一般不建議）
USE_CALLER_ACCEL_LIMITS = True

# planner 沒寫限制時的 fallback（維持原本行為）
ACCEL_MIN_FALLBACK = ACCEL_MIN
ACCEL_MAX_FALLBACK = ACCEL_MAX

# 2) SNG（塞車走走停停）下的 lead “煞車敏感化”開關
#    這裡不做正向保守（那由 planner 的 SNG guard 管）
#    只在「lead 變負加速度」時，讓 MPC 更快把障礙拉近、避免慢半拍
MPC_SNG_BRAKE_SENSE_ENABLE = True
MPC_SNG_VEGO_MAX = 8.0          # m/s ≈ 29 km/h（低速才啟用）
MPC_SNG_DREL_MAX = 18.0         # m（近距離才啟用）
MPC_SNG_MODELPROB_MIN = 0.50    # lead 可信度下限（避免亂敏感）

# 2a) lead accel 的衰減（extrapolate_lead 內用 a_lead_tau）
#     你目前公式：a_traj = a0 * exp(-a_lead_tau * (t^2)/2)
#     → a_lead_tau 越大，衰減越快（越快回到 0）
#     所以在「lead 明顯煞」時，我們要把 a_lead_tau 變小，讓負加速度維持久一點 → 更保守
MPC_LEAD_BRAKE_A_THRESH = -0.40         # m/s^2（低於此值視為 lead 在煞）
MPC_A_LEAD_TAU_SCALE_BRAKE = 0.55       # 煞車時 tau 乘上此值（越小越保守/更快反應）
MPC_A_LEAD_TAU_MIN = 0.10              # 避免太小造成發散
MPC_A_LEAD_TAU_MAX = 3.00              # 避免太大導致過度鈍化

# 2b) moving lead 的 stopped-equivalence：lead 正在煞時，等效距離要更小（障礙更近）
#     原本：v^2/(2*COMFORT_BRAKE)（假設 lead 也用舒適煞車）
#     改：若 lead 真的在更大減速度煞車，則用更大的 brake_eff → v^2/(2*brake_eff) 更小 → 更保守
LEAD_BRAKE_EFF_MAX = 6.0                # m/s^2（上限，避免噪聲讓 brake_eff 爆大）
LEAD_BRAKE_EFF_GAIN = 1.00              # 1.0=直接用 |-a_lead|；<1 會比較溫和

# 2c) danger factor（安全距離約束的倍率）在 SNG/煞車時可提高到 1.0
#     constraint：x_obstacle - x_ego >= lead_danger_factor * desired_dist
#     lead_danger_factor 越大 → 越保守
MPC_LEAD_DANGER_FACTOR_SNG = 1.00        # 建議先用 1.0，不要太大避免 slack 過多

# 2d) t_follow 在 SNG/煞車時可微加一點（很小即可，避免變龜）
MPC_T_FOLLOW_SNG_ADD = 0.10             # 秒（0.0~0.2 建議範圍）
# =======================================================================


def get_jerk_factor(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.0
  elif personality == log.LongitudinalPersonality.standard:
    return 1.0
  elif personality == log.LongitudinalPersonality.aggressive:
    return 0.5
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_T_FOLLOW(personality=log.LongitudinalPersonality.standard):
  if personality == log.LongitudinalPersonality.relaxed:
    return 1.75
  elif personality == log.LongitudinalPersonality.standard:
    return 1.45
  elif personality == log.LongitudinalPersonality.aggressive:
    return 1.25
  else:
    raise NotImplementedError("Longitudinal personality not supported")


def get_stopped_equivalence_factor(v_lead, brake_eff=COMFORT_BRAKE):
  """
  moving lead 的“等效停止距離”：
    v^2 / (2*b)
  b 越大（表示 lead 煞越大）→ 等效距離越小 → 障礙更近（更保守）
  """
  b = float(np.clip(brake_eff, 0.1, 50.0))
  return (v_lead ** 2) / (2 * b)


def get_safe_obstacle_distance(v_ego, t_follow):
  return (v_ego ** 2) / (2 * COMFORT_BRAKE) + t_follow * v_ego + STOP_DISTANCE


def desired_follow_distance(v_ego, v_lead, t_follow=None):
  if t_follow is None:
    t_follow = get_T_FOLLOW()
  return get_safe_obstacle_distance(v_ego, t_follow) - get_stopped_equivalence_factor(v_lead)


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

  # The main cost in normal operation is how close you are to the "desired" distance
  # from an obstacle at every timestep.
  costs = [((x_obstacle - x_ego) - (desired_dist_comfort)) / (v_ego + 10.),
           x_ego,
           v_ego,
           a_ego,
           a_ego - prev_a,
           j_ego]
  ocp.model.cost_y_expr = vertcat(*costs)
  ocp.model.cost_y_expr_e = vertcat(*costs[:-1])

  # Constraints:
  constraints = vertcat(v_ego,
                        (a_ego - a_min),
                        (a_max - a_ego),
                        ((x_obstacle - x_ego) - lead_danger_factor * (desired_dist_comfort)) / (v_ego + 10.))
  ocp.model.con_h_expr = constraints

  x0 = np.zeros(X_DIM)
  ocp.constraints.x0 = x0
  ocp.parameter_values = np.array([ACCEL_MIN_FALLBACK, ACCEL_MAX_FALLBACK, 0.0, 0.0, get_T_FOLLOW(), LEAD_DANGER_FACTOR])

  # slack cost weights (runtime set)
  cost_weights = np.zeros(CONSTR_DIM)
  ocp.cost.zl = cost_weights
  ocp.cost.Zl = cost_weights
  ocp.cost.Zu = cost_weights
  ocp.cost.zu = cost_weights

  ocp.constraints.lh = np.zeros(CONSTR_DIM)
  ocp.constraints.uh = 1e4 * np.ones(CONSTR_DIM)
  ocp.constraints.idxsh = np.arange(CONSTR_DIM)

  # solver options
  ocp.solver_options.qp_solver = 'PARTIAL_CONDENSING_HPIPM'
  ocp.solver_options.hessian_approx = 'GAUSS_NEWTON'
  ocp.solver_options.integrator_type = 'ERK'
  ocp.solver_options.nlp_solver_type = ACADOS_SOLVER_TYPE
  ocp.solver_options.qp_solver_cond_N = 1

  ocp.solver_options.qp_solver_iter_max = 10
  ocp.solver_options.qp_tol = 1e-3

  # set prediction horizon
  ocp.solver_options.tf = Tf
  ocp.solver_options.shooting_nodes = T_IDXS

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
    self.v_solution = np.zeros(N + 1)
    self.a_solution = np.zeros(N + 1)
    self.prev_a = np.array(self.a_solution)
    self.j_solution = np.zeros(N)

    self.yref = np.zeros((N + 1, COST_DIM))
    for i in range(N):
      self.solver.cost_set(i, "yref", self.yref[i])
    self.solver.cost_set(N, "yref", self.yref[N][:COST_E_DIM])

    self.x_sol = np.zeros((N + 1, X_DIM))
    self.u_sol = np.zeros((N, 1))

    # params：讓 planner 可以在外部先寫入 a_min/a_max（例如 DTSC 或 SNG guard）
    self.params = np.zeros((N + 1, PARAM_DIM))
    # 給一個合理預設，避免“全 0”時造成 constraint 不合理
    self.params[:, 0] = ACCEL_MIN_FALLBACK
    self.params[:, 1] = ACCEL_MAX_FALLBACK

    for i in range(N + 1):
      self.solver.set(i, 'x', np.zeros(X_DIM))

    self.last_cloudlog_t = 0
    self.status = False
    self.crash_cnt = 0.0
    self.solution_status = 0

    # timers
    self.solve_time = 0.0
    self.time_qp_solution = 0.0
    self.time_linearization = 0.0
    self.time_integrator = 0.0

    self.x0 = np.zeros(X_DIM)
    self.set_weights()

  def set_cost_weights(self, cost_weights, constraint_cost_weights):
    W = np.asfortranarray(np.diag(cost_weights))
    for i in range(N):
      # reduce the cost on (a-a_prev) later in the horizon.
      W[4, 4] = cost_weights[4] * np.interp(T_IDXS[i], [0.0, 1.0, 2.0], [1.0, 1.0, 0.0])
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
      for i in range(N + 1):
        self.solver.set(i, 'x', self.x0)

  @staticmethod
  def extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau):
    """
    注意：這裡的 a_lead_tau 在你目前公式下，是“衰減強度係數”
      a_traj = a0 * exp(-a_lead_tau * t^2 / 2)
    → a_lead_tau 越大，衰減越快（越快回到 0）
    """
    a_lead_traj = a_lead * np.exp(-a_lead_tau * (T_IDXS ** 2) / 2.)
    v_lead_traj = np.clip(v_lead + np.cumsum(T_DIFFS * a_lead_traj), 0.0, 1e8)
    x_lead_traj = x_lead + np.cumsum(T_DIFFS * v_lead_traj)
    lead_xv = np.column_stack((x_lead_traj, v_lead_traj))
    return lead_xv

  def _sng_brake_sense_active(self, v_ego, x_lead, a_lead, model_prob) -> bool:
    """
    判斷是否啟用 “SNG 煞車敏感化”（只在低速近距離且 lead 可信且在煞）
    """
    if not MPC_SNG_BRAKE_SENSE_ENABLE:
      return False
    if v_ego > MPC_SNG_VEGO_MAX:
      return False
    if x_lead > MPC_SNG_DREL_MAX:
      return False
    if model_prob < MPC_SNG_MODELPROB_MIN:
      return False
    return (a_lead < MPC_LEAD_BRAKE_A_THRESH)

  def _get_brake_eff(self, a_lead, model_prob) -> float:
    """
    依 lead 的負加速度決定等效煞車強度（用於 stopped-equivalence）
    - a_lead 越負 → brake_eff 越大 → 等效距離越小（更保守）
    """
    if model_prob < MPC_SNG_MODELPROB_MIN:
      return COMFORT_BRAKE
    if a_lead >= MPC_LEAD_BRAKE_A_THRESH:
      return COMFORT_BRAKE
    b = float(np.clip((-a_lead) * LEAD_BRAKE_EFF_GAIN, COMFORT_BRAKE, LEAD_BRAKE_EFF_MAX))
    return b

  def _get_lead_fields(self, lead, v_ego):
    """
    取出 lead 欄位；若無 lead 則回傳一組“假的遠前車”，讓 MPC 保持可解
    """
    if lead is not None and lead.status:
      x_lead = float(lead.dRel)
      v_lead = float(lead.vLead)
      a_lead = float(lead.aLeadK)
      a_lead_tau = float(lead.aLeadTau)
      model_prob = float(getattr(lead, "modelProb", 0.0))
      return x_lead, v_lead, a_lead, a_lead_tau, model_prob

    # Fake a fast lead car, so mpc can keep running in the same mode
    x_lead = 50.0
    v_lead = v_ego + 10.0
    a_lead = 0.0
    a_lead_tau = _LEAD_ACCEL_TAU
    model_prob = 0.0
    return x_lead, v_lead, a_lead, a_lead_tau, model_prob

  def process_lead(self, lead):
    """
    lead → 產生 lead_xv（x,v trajectory）
    並在 SNG/煞車時做兩個“只增強負向反應”的調整：
      1) a_lead_tau 變小 → 負加速度衰減更慢（更保守）
      2) stopped-equivalence 用更大的 brake_eff → 障礙更近（更保守）
    """
    v_ego = float(self.x0[1])

    x_lead, v_lead, a_lead, a_lead_tau, model_prob = self._get_lead_fields(lead, v_ego)

    # MPC will not converge if immediate crash is expected
    # Clip lead distance to what is still possible to brake for
    min_x_lead = ((v_ego + v_lead) / 2) * (v_ego - v_lead) / (-ACCEL_MIN * 2)
    x_lead = float(np.clip(x_lead, min_x_lead, 1e8))
    v_lead = float(np.clip(v_lead, 0.0, 1e8))
    a_lead = float(np.clip(a_lead, -10., 5.))

    # === SNG 煞車敏感化：減速時讓 a_lead_tau 變小（衰減慢） ===
    if self._sng_brake_sense_active(v_ego, x_lead, a_lead, model_prob):
      a_lead_tau = float(np.clip(a_lead_tau * MPC_A_LEAD_TAU_SCALE_BRAKE, MPC_A_LEAD_TAU_MIN, MPC_A_LEAD_TAU_MAX))
    else:
      a_lead_tau = float(np.clip(a_lead_tau, MPC_A_LEAD_TAU_MIN, MPC_A_LEAD_TAU_MAX))

    lead_xv = self.extrapolate_lead(x_lead, v_lead, a_lead, a_lead_tau)

    # === stopped-equivalence 的等效煞車強度（給 update 產生 obstacle 用） ===
    brake_eff = self._get_brake_eff(a_lead, model_prob)

    return lead_xv, brake_eff, model_prob, x_lead, a_lead

  def _ensure_accel_limits(self):
    """
    確保 self.params[:,0/1]（a_min/a_max）合理：
    - 若 caller 沒寫（全 0），整段回填 fallback
    - 若 caller 有寫，但某些點出現 a_max<=a_min：只修壞掉的點（不要整段覆蓋）
    """
    a_min = self.params[:, 0].copy()
    a_max = self.params[:, 1].copy()

    # 1) caller 完全沒寫（常見：全 0）
    if np.allclose(a_min, 0.0) and np.allclose(a_max, 0.0):
      self.params[:, 0] = ACCEL_MIN_FALLBACK
      self.params[:, 1] = ACCEL_MAX_FALLBACK
      return

    # 2) 逐點清理：限制到 fallback 範圍內（可選，但建議）
    a_min = np.clip(a_min, ACCEL_MIN_FALLBACK, ACCEL_MAX_FALLBACK)
    a_max = np.clip(a_max, ACCEL_MIN_FALLBACK, ACCEL_MAX_FALLBACK)

    # 3) 逐點修復不合法：只修壞掉的 stage
    bad = a_max <= (a_min + 1e-3)
    if np.any(bad):
      # 用 fallback 修壞點，或用 “擴開一點點” 的方式也可
      a_min[bad] = ACCEL_MIN_FALLBACK
      a_max[bad] = ACCEL_MAX_FALLBACK

    self.params[:, 0] = a_min
    self.params[:, 1] = a_max

  def update(self, radarstate, v_cruise, x, v, a, j, personality=log.LongitudinalPersonality.standard):
    v_ego = float(self.x0[1])

    # 先確保 a_min/a_max 合理（避免覆蓋 caller 的限制）
    self._ensure_accel_limits()

    # t_follow：在 SNG/煞車時微加一點（非常小即可）
    t_follow_base = float(get_T_FOLLOW(personality))
    t_follow = t_follow_base

    # lead 狀態
    self.status = radarstate.leadOne.status or radarstate.leadTwo.status

    lead_xv_0, brake_eff_0, prob_0, x0_now, a0_now = self.process_lead(radarstate.leadOne)
    lead_xv_1, brake_eff_1, prob_1, x1_now, a1_now = self.process_lead(radarstate.leadTwo)

    # 若 lead0 在 SNG/煞車敏感化條件內，微加 t_follow（讓距離更保守一點）
    if self._sng_brake_sense_active(v_ego, x0_now, a0_now, prob_0):
      t_follow = float(t_follow_base + MPC_T_FOLLOW_SNG_ADD)

    # moving lead → obstacle：用 brake_eff 產生“更真實”的等效停止距離
    lead_0_obstacle = lead_xv_0[:, 0] + get_stopped_equivalence_factor(lead_xv_0[:, 1], brake_eff_0)
    lead_1_obstacle = lead_xv_1[:, 0] + get_stopped_equivalence_factor(lead_xv_1[:, 1], brake_eff_1)

    # ============ ACC / blended ============
    if self.mode == 'acc':
      # lead_danger_factor：平時用 LEAD_DANGER_FACTOR；SNG/煞車時提升到 1.0（更保守）
      danger_factor = LEAD_DANGER_FACTOR
      if self._sng_brake_sense_active(v_ego, x0_now, a0_now, prob_0):
        danger_factor = max(danger_factor, MPC_LEAD_DANGER_FACTOR_SNG)
      self.params[:, 5] = float(danger_factor)

      # Fake an obstacle for cruise (smooth accel to set speed)
      v_lower = v_ego + (T_IDXS * CRUISE_MIN_ACCEL * 1.05)
      v_upper = v_ego + (T_IDXS * CRUISE_MAX_ACCEL * 1.05)
      v_cruise_clipped = np.clip(v_cruise * np.ones(N + 1), v_lower, v_upper)

      cruise_obstacle = np.cumsum(T_DIFFS * v_cruise_clipped) + get_safe_obstacle_distance(v_cruise_clipped, t_follow)
      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle, cruise_obstacle])
      self.source = SOURCES[np.argmin(x_obstacles[0])]

      # These are not used in ACC mode
      x[:], v[:], a[:], j[:] = 0.0, 0.0, 0.0, 0.0

    elif self.mode == 'blended':
      # blended：原本固定 1.0（保留），但若你想在 SNG/煞車更保守，也可抬到 >=1.0
      danger_factor = 1.0
      if self._sng_brake_sense_active(v_ego, x0_now, a0_now, prob_0):
        danger_factor = max(danger_factor, MPC_LEAD_DANGER_FACTOR_SNG)
      self.params[:, 5] = float(danger_factor)

      x_obstacles = np.column_stack([lead_0_obstacle, lead_1_obstacle])

      cruise_target = T_IDXS * np.clip(v_cruise, v_ego - 2.0, 1e3) + x[0]
      xforward = ((v[1:] + v[:-1]) / 2) * (T_IDXS[1:] - T_IDXS[:-1])
      x = np.cumsum(np.insert(xforward, 0, x[0]))

      x_and_cruise = np.column_stack([x, cruise_target])
      x = np.min(x_and_cruise, axis=1)

      self.source = 'e2e' if x_and_cruise[1, 0] < x_and_cruise[1, 1] else 'cruise'

    else:
      raise NotImplementedError(f'Planner mode {self.mode} not recognized in planner update')

    # yref
    self.yref[:, 1] = x
    self.yref[:, 2] = v
    self.yref[:, 3] = a
    self.yref[:, 5] = j
    for i in range(N):
      self.solver.set(i, "yref", self.yref[i])
    self.solver.set(N, "yref", self.yref[N][:COST_E_DIM])

    # params
    self.params[:, 2] = np.min(x_obstacles, axis=1)
    self.params[:, 3] = np.copy(self.prev_a)
    self.params[:, 4] = float(t_follow)

    self.run()

    # FCW logic（維持原本）
    if (np.any(lead_xv_0[FCW_IDXS, 0] - self.x_sol[FCW_IDXS, 0] < CRASH_DISTANCE) and
            radarstate.leadOne.modelProb > 0.9):
      self.crash_cnt += 1
    else:
      self.crash_cnt = 0

    # Check if it got within lead comfort range (維持原本)
    if self.mode == 'blended':
      if any((lead_0_obstacle - get_safe_obstacle_distance(self.x_sol[:, 1], t_follow)) - self.x_sol[:, 0] < 0.0):
        self.source = 'lead0'
      if any((lead_1_obstacle - get_safe_obstacle_distance(self.x_sol[:, 1], t_follow)) - self.x_sol[:, 0] < 0.0) and \
         (lead_1_obstacle[0] - lead_0_obstacle[0]):
        self.source = 'lead1'

  def run(self):
    for i in range(N + 1):
      self.solver.set(i, 'p', self.params[i])
    self.solver.constraints_set(0, "lbx", self.x0)
    self.solver.constraints_set(0, "ubx", self.x0)

    self.solution_status = self.solver.solve()
    self.solve_time = float(self.solver.get_stats('time_tot')[0])
    self.time_qp_solution = float(self.solver.get_stats('time_qp')[0])
    self.time_linearization = float(self.solver.get_stats('time_lin')[0])
    self.time_integrator = float(self.solver.get_stats('time_sim')[0])

    for i in range(N + 1):
      self.x_sol[i] = self.solver.get(i, 'x')
    for i in range(N):
      self.u_sol[i] = self.solver.get(i, 'u')

    self.v_solution = self.x_sol[:, 1]
    self.a_solution = self.x_sol[:, 2]
    self.j_solution = self.u_sol[:, 0]

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
