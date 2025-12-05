import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import os

# ===================== PATHS =====================
path_to_files = (
    "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles"
)
if not os.path.exists(path_to_files):
    raise FileNotFoundError(f"Path non trovato: {path_to_files}")

get_M = ca.Function.load(os.path.join(path_to_files, "M.casadi"))
get_C = ca.Function.load(os.path.join(path_to_files, "C.casadi"))
get_G = ca.Function.load(os.path.join(path_to_files, "G.casadi"))
get_T_0_0 = ca.Function.load(os.path.join(path_to_files, "T_0_0.casadi"))
get_T_0_1 = ca.Function.load(os.path.join(path_to_files, "T_0_1.casadi"))
get_T_0_2 = ca.Function.load(os.path.join(path_to_files, "T_0_2.casadi"))
get_T_0_3 = ca.Function.load(os.path.join(path_to_files, "T_0_3.casadi"))
get_T_0_4 = ca.Function.load(os.path.join(path_to_files, "T_0_4.casadi"))
get_T_0_5 = ca.Function.load(os.path.join(path_to_files, "T_0_5.casadi"))
get_T_0_6 = ca.Function.load(os.path.join(path_to_files, "T_0_6.casadi"))
get_T_0_7 = ca.Function.load(os.path.join(path_to_files, "T_0_7.casadi"))
get_T_0_8 = ca.Function.load(os.path.join(path_to_files, "T_0_8.casadi"))


def export_tracking_model():
    model = AcadosModel()
    model.name = "frankino_tracking_mpc"

    # --- SIMBOLI ---
    q = ca.SX.sym("q", 7)
    dq = ca.SX.sym("dq", 7)
    x = ca.vertcat(q, dq)

    ddq = ca.SX.sym("ddq", 7)
    u = ddq

    p_obs = ca.SX.sym("p_obs", 3)
    model.p = p_obs

    # --- DINAMICA ---
    model.x = x
    model.u = u
    model.f_expl_expr = ca.vertcat(dq, ddq)

    # --- CALCOLO COPPIA ---
    M_val = get_M(q)
    G_val = get_G(q)
    C_temp = get_C(q, dq)
    coriolis = (
        ca.mtimes(C_temp, dq) if C_temp.size1() == 7 and C_temp.size2() == 7 else C_temp
    )
    tau_expr = ca.mtimes(M_val, u) + coriolis + G_val

    # --- VINCOLI ---
    res_T0 = get_T_0_0()
    
    # Se è un dizionario, prendiamo il primo valore (la matrice), altrimenti usiamo il risultato diretto
    T_0_0_val = res_T0["o0"] if isinstance(res_T0, dict) else res_T0

    p_0 = T_0_0_val[0:3, 3]
    p_1 = get_T_0_1(q)[0:3, 3]
    p_2 = get_T_0_2(q)[0:3, 3]
    p_3 = get_T_0_3(q)[0:3, 3]
    p_4 = get_T_0_4(q)[0:3, 3]
    p_5 = get_T_0_5(q)[0:3, 3]
    p_6 = get_T_0_6(q)[0:3, 3]
    p_7 = get_T_0_7(q)[0:3, 3]
    p_8 = get_T_0_8(q)[0:3, 3]

    # Distanza al quadrato tra link e ostacolo
    dist_sq_0 = ca.sumsqr(p_0 - p_obs)
    dist_sq_1 = ca.sumsqr(p_1 - p_obs)
    dist_sq_2 = ca.sumsqr(p_2 - p_obs)
    dist_sq_3 = ca.sumsqr(p_3 - p_obs)
    dist_sq_4 = ca.sumsqr(p_4 - p_obs)
    dist_sq_5 = ca.sumsqr(p_5 - p_obs)
    dist_sq_6 = ca.sumsqr(p_6 - p_obs)
    dist_sq_7 = ca.sumsqr(p_7 - p_obs)
    dist_sq_8 = ca.sumsqr(p_8 - p_obs)
    model.con_h_expr = ca.vertcat(
        tau_expr,
        dist_sq_0,
        dist_sq_1,
        dist_sq_2,
        dist_sq_3,
        dist_sq_4,
        dist_sq_5,
        dist_sq_6,
        dist_sq_7,
        dist_sq_8,
    )
    model.con_h_expr_e = ca.vertcat(
        dist_sq_0,
        dist_sq_1,
        dist_sq_2,
        dist_sq_3,
        dist_sq_4,
        dist_sq_5,
        dist_sq_6,
        dist_sq_7,
        dist_sq_8,
    )

    # --- VETTORE COSTO (y) ---
    # MODIFICA IMPORTANTE: Includiamo 'u' in y per pesarlo con NONLINEAR_LS
    # y = [q, dq, u] -> Dimensione 14 + 7 = 21
    model.cost_y_expr = ca.vertcat(q, dq, u)

    # Costo terminale: solo stato (u non esiste all'ultimo step)
    # y_e = [q, dq] -> Dimensione 14
    model.cost_y_expr_e = ca.vertcat(q, dq)

    return model


def create_solver():
    model = export_tracking_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = "c_generated_code_tracking"

    # --- SETUP ORARIO ---
    N = 20
    Tf = 1.0
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # --- COSTI: RIMANIAMO IN NONLINEAR_LS ---
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"

    # Pesi
    W_q = 500.0  # Tracking posizione
    W_dq = 50.0  # Tracking velocità
    W_u = 0.01  # Penalità accelerazione (smoothness)

    # Matrice W (21x21): [q (7), dq (7), u (7)]
    # Diagonalizziamo i tre blocchi
    W_diag = np.concatenate([np.full(7, W_q), np.full(7, W_dq), np.full(7, W_u)])
    ocp.cost.W = np.diag(W_diag)

    # Reference placeholder (Dimensione 21)
    # Il C++ riempirà i primi 14. Gli ultimi 7 (u_ref) restano 0 per minimizzare l'accelerazione.
    ocp.cost.yref = np.zeros(21)

    # Costo Terminale (Dimensione 14)
    ocp.cost.W_e = np.diag(np.concatenate([np.full(7, 500.0), np.full(7, 50.0)]))
    ocp.cost.yref_e = np.zeros(14)

    # --- VINCOLI ---
    
    #Definiamo quanti vincoli ci sono in totale
    n_tau = 7
    n_dist = 9
    n_h = n_tau + n_dist  # 16
    
    q_max = np.array([2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89])
    q_min = np.array([-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89])
    dq_min = np.array([-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61])
    dq_max = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
    ddq_min = np.array([-15, -7.5, -10, -12.5, -15, -20, -20])
    ddq_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20])

    ocp.constraints.idxbx = np.arange(14)
    ocp.constraints.lbx = np.concatenate([q_min, dq_min])
    ocp.constraints.ubx = np.concatenate([q_max, dq_max])

    ocp.constraints.idxbu = np.arange(7)
    ocp.constraints.lbu = ddq_min
    ocp.constraints.ubu = ddq_max

    # Vincoli h (Torque + Collisione)
    tau_lim = 87.0
    lh_tau = np.full(7, -tau_lim)
    uh_tau = np.full(7, +tau_lim)
    # Link 1 e 2 sono grossi, Link 7 è piccolo
    # Ordine: [L1, L2, L3, L4, L5, L6, L7]
    raggi_robot = np.array([0.10, 0.09, 0.09, 0.07, 0.07, 0.06, 0.06, 0.05, 0.05])
    r_obs = 0.10  # Raggio sfera ostacolo
    raggi = (raggi_robot + r_obs) ** 2  # Somma dei raggi

    # Lower bound = raggio^2 (la distanza minima al quadrato)
    lh_dist = raggi
    # Upper bound = infinito (nessun limite massimo alla distanza)
    uh_dist = np.full(n_dist, 1e9)

    # Unione
    ocp.constraints.lh = np.concatenate([lh_tau, lh_dist])
    ocp.constraints.uh = np.concatenate([uh_tau, uh_dist])
    ocp.constraints.lh_e = lh_dist
    ocp.constraints.uh_e = uh_dist
    ocp.dims.nh_e = n_dist

    # Slacks
    # Numero di slack attive
    n_sh = n_dist  # 9
    # Vogliamo le slack SOLO sugli ultimi 9 vincoli (quelli di distanza)
    # np.arange(start, stop) -> crea array [7, 8, 9, 10, 11, 12, 13, 14, 15]
    ocp.constraints.idxsh = np.arange(n_tau, n_h)
    ocp.constraints.Jsh = np.eye(n_sh)
    ocp.dims.nsh = n_sh

    # I pesi devono avere la dimensione delle slack attive (n_sh), non di n_h!
    # Z = peso quadratico (L2), z = peso lineare (L1)

    # Lower slack weights (zl, Zl): penalizzano la violazione del limite inferiore.
    ocp.cost.zl = np.full(n_sh, 1e2)  # Costo lineare
    ocp.cost.Zl = np.full(n_sh, 1e3)  # Costo quadratico

    # Non verrà mai violato. Mettiamo comunque valori per coerenza, ma sono ininfluenti.
    ocp.cost.zu = np.full(n_sh, 0.0)
    ocp.cost.Zu = np.full(n_sh, 0.0)
    # Solver opts
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"  # 1 solo iterazione per RTI
    ocp.solver_options.tol = 1e-3
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.qp_solver_cond_N = N

    # Regolarizzazione per migliorare condizionamento
    ocp.solver_options.levenberg_marquardt = 1e-1 # Aggiungi regolarizzazione

    # Imposta passo di integrazione
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 5

    ocp.parameter_values = np.array([10.0, 10.0, 10.0])
    ocp.constraints.x0 = np.zeros(14)

    AcadosOcpSolver(ocp, json_file="acados_track.json")


if __name__ == "__main__":
    create_solver()
