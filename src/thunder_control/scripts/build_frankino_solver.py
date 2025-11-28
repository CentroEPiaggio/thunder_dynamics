import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import os

# ===================== SETUP PATHS ==========================
path_to_files = (
    "/home/thunder_dev/thunder_dynamics/src/thunder/build/frankino_generatedFiles/"
)

if not os.path.exists(path_to_files):
    raise FileNotFoundError(f"La cartella {path_to_files} non esiste.")

get_M_func = ca.Function.load(os.path.join(path_to_files, "M.casadi"))
get_C_func = ca.Function.load(os.path.join(path_to_files, "C.casadi"))
get_G_func = ca.Function.load(os.path.join(path_to_files, "G.casadi"))

get_FK_func = ca.Function.load(os.path.join(path_to_files, "T_0_ee.casadi"))


# ===================== DEFINIZIONE MODELLO ===================
def export_franka_model():
    model = AcadosModel()
    model.name = "frankino_throw_catch"

    # Simboli
    n_q = 7
    q = ca.SX.sym("q", n_q)
    q_dot = ca.SX.sym("q_dot", n_q)
    tau = ca.SX.sym("tau", n_q)

    x = ca.vertcat(q, q_dot)
    u = tau

    # Definiamo p_obs come parametro esterno (aggiornabile runtime nel cpp)
    p_obs = ca.SX.sym("p_obs", 3)
    model.p = p_obs

    # Dinamica (M * q_ddot + C * q_dot + G = tau)
    C_tmp = get_C_func(q, q_dot)
    coriolis = C_tmp @ q_dot if (C_tmp.size1() == 7 and C_tmp.size2() == 7) else C_tmp

    rhs = tau - coriolis - get_G_func(q)
    q_ddot = ca.solve(get_M_func(q), rhs)

    f_expl = ca.vertcat(q_dot, q_ddot)

    model.x = x
    model.x = x
    model.u = u
    model.f_expl_expr = f_expl

    T_ee = get_FK_func(q)
    pos_ee = T_ee[0:3, 3]

    # 2. Calcola distanza quadrata dall'ostacolo
    # (x_ee - x_obs)^2 + ...
    dist_sq = ca.sumsqr(pos_ee - p_obs)

    # 3. Assegna l'espressione al vincolo generico h
    model.con_h_expr = dist_sq

    return model


# ===================== DEFINIZIONE OCP =======================
def create_ocp_solver():
    model = export_franka_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = "c_generated_code_frankino_throw"

    # Dimensioni
    nx = 14
    nu = 7
    np_param = 3  # Dimensione del parametro ostacolo (x,y,z)

    # --- TEMPISTICHE ---
    # N=20 e tf=1.0s significa guardare avanti di 1 secondo.
    # Se il lancio deve avvenire tra 0.5s scegliere almeno orizzonte=1.0.
    ocp.dims.N = 20
    ocp.solver_options.tf = 1.0

    # --- COSTI ---
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # 1. Running Cost (0 ... N-1)
    # y = [x, u]
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)

    W_q_run = 100.0  # Peso sulla traiettoria intermedia
    W_v_run = 100.0  # Peso sulla velocità intermedia
    W_tau = 1.0e-2  # Peso basso sullo sforzo (per permettere accelerazioni alte)

    Q_diag_run = np.concatenate([np.full(7, W_q_run), np.full(7, W_v_run)])
    R_diag_run = np.full(7, W_tau)
    ocp.cost.W = np.diag(np.concatenate([Q_diag_run, R_diag_run]))

    # 2. Terminal Cost (Step N) -> QUI IMPONIAMO IL TARGET
    # y_e = [x]
    ocp.model.cost_y_expr_e = model.x

    W_q_end = 1e5  # 1e5: Peso sulla posizione finale
    W_v_end = 1e5  # 1e5: Peso sulla velocità finale (cruciale per il lancio)

    Q_diag_end = np.concatenate([np.full(7, W_q_end), np.full(7, W_v_end)])
    ocp.cost.W_e = np.diag(Q_diag_end)

    # Reference placeholder (saranno settati in C++)
    ocp.cost.yref = np.zeros(nx + nu)
    ocp.cost.yref_e = np.zeros(nx)

    # --- VINCOLI OSTACOLO (h constraints) ---
    # Vogliamo dist_sq >= r_safety^2
    r_safety = 0.1  # Es. 10cm di raggio sicurezza
    min_dist_sq = r_safety**2

    # Impostiamo i limiti: min_dist_sq <= h(x) <= Infinito
    ocp.constraints.lh = np.array([min_dist_sq])
    ocp.constraints.uh = np.array(
        [1000.0]
    )  # Un numero molto grande (praticamente infinito)

    # Indice del vincolo h da ammorbidire (ne abbiamo solo 1, quindi indice 0)
    ocp.constraints.idxsh = np.array([0])

    # Pesi per la slack variable
    # L1 penalty (lineare) e L2 penalty (quadratica) sulla violazione del lower bound
    slack_weight = 10000.0
    ocp.cost.zl = np.array([slack_weight])  # Linear cost on slack lower
    ocp.cost.Zl = np.array([slack_weight])  # Quadratic cost on slack lower
    ocp.cost.zu = np.array([0.0])  # Non ci serve slack sull'upper bound (infinito)
    ocp.cost.Zu = np.array([0.0])  # Non ci serve slack sull'upper bound (infinito)
    # ---Inizializzazione Parametri ---
    # Lo mettiamo lontano (es. 10m, 10m, 10m) così all'inizio non dà fastidio.
    ocp.parameter_values = np.array([10.0, 10.0, 10.0])

    # --- VINCOLI ---
    # Coppia
    tau_limit = 87.0
    ocp.constraints.lbu = np.full(7, -tau_limit)
    ocp.constraints.ubu = np.full(7, +tau_limit)
    ocp.constraints.idxbu = np.arange(nu)

    # Stati (Limiti giunti)
    q_max = np.array([2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89])
    q_min = np.array([-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89])
    # dq_max = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
    # dq_min = -np.array([-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61])
    dq_max = np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0])
    dq_min = -np.array([3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0])

    ocp.constraints.lbx = np.concatenate([q_min, dq_min])
    ocp.constraints.ubx = np.concatenate([q_max, dq_max])
    ocp.constraints.idxbx = np.arange(nx)

    # Stato iniziale
    ocp.constraints.x0 = np.zeros(nx)

    # --- OPZIONI SOLVER ---
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # Veloce Real-Time Iteration

    solver = AcadosOcpSolver(ocp, json_file="acados_ocp_throw.json")
    return solver


if __name__ == "__main__":
    print(" Generazione solver...")
    create_ocp_solver()
    print(" Fatto.")


# Esempio in C++ evitamento ostacolo (da inserire in main_franka_mpc.cpp):
# double p_obs[3] = {x_ostacolo, y_ostacolo, z_ostacolo};

# for (int i = 0; i <= N; i++)
# {
#     ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, i, "p", p_obs); // O funzione equivalente acados_update_params
#     // Nota: verifica la funzione esatta generata nell'interfaccia C (spesso è acados_update_params)
# }
