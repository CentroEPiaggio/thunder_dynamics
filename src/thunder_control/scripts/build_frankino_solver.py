import casadi as ca
import numpy as np
from acados_template import (
    AcadosOcp,
    AcadosOcpSolver,
    AcadosModel,
    AcadosSimSolver,
    AcadosSim,
)
import os
import DistanceFunctions as geom 
# ===================== PATHS =====================
path_to_files = (
    "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles"
)
if not os.path.exists(path_to_files):
    raise FileNotFoundError(f"Path non trovato: {path_to_files}")

# Carichiamo solo la dinamica necessaria per calcolare le coppie
get_M = ca.Function.load(os.path.join(path_to_files, "M.casadi"))
get_C = ca.Function.load(os.path.join(path_to_files, "C.casadi"))
get_G = ca.Function.load(os.path.join(path_to_files, "G.casadi"))
# Carichiamo le trasformate per la cinematica delle capsule
T_funcs = [ca.Function.load(os.path.join(path_to_files, f"T_0_{i}.casadi")) for i in range(9)]

# ===================== DEFINIZIONE CAPSULE =====================
class CapsuleDefinition:
    def __init__(self, link_index, radius, length, T_offset):
        self.link_index = link_index
        self.radius = radius
        self.length = length
        self.T_offset = np.array(T_offset)

def create_capsule_definitions():
    """Definizioni geometriche dei link del robot Frankino"""
    capsules = []
    # --- Robot Capsule Definitions (Frankino) ---
    capsules.append(CapsuleDefinition(0, 0.09, 0.03, [[0,0,1,-0.075], [0,1,0,0], [-1,0,0,0.06], [0,0,0,1]])) # idx 0
    capsules.append(CapsuleDefinition(1, 0.09, 0.283, [[1,0,0,0], [0,1,0,0], [0,0,1,-0.1915], [0,0,0,1]])) # idx 1
    capsules.append(CapsuleDefinition(2, 0.09, 0.12, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]))      # idx 2
    capsules.append(CapsuleDefinition(3, 0.09, 0.15, [[1,0,0,0], [0,1,0,0], [0,0,1,-0.145], [0,0,0,1]])) # idx 3
    capsules.append(CapsuleDefinition(4, 0.09, 0.12, [[1,0,0,0], [0,1,0,0], [0,0,1,0], [0,0,0,1]]))      # idx 4
    capsules.append(CapsuleDefinition(5, 0.09, 0.10, [[1,0,0,0], [0,1,0,0], [0,0,1,-0.26], [0,0,0,1]]))  # idx 5
    capsules.append(CapsuleDefinition(5, 0.055, 0.14, [[0.9968,-0.0799,0,0], [0.0799,0.9968,0,0.08], [0,0,1,-0.13], [0,0,0,1]])) # idx 6
    capsules.append(CapsuleDefinition(6, 0.08, 0.08, [[1,0,0,0], [0,1,0,0], [0,0,1,-0.03], [0,0,0,1]]))  # idx 7
    capsules.append(CapsuleDefinition(7, 0.07, 0.14, [[1,0,0,0], [0,1,0,0], [0,0,1,0.01], [0,0,0,1]]))  # idx 8
    capsules.append(CapsuleDefinition(7, 0.06, 0.01, [[0,0,1,0.06], [0,1,0,0], [-1,0,0,0.082], [0,0,0,1]])) # idx 9
    capsules.append(CapsuleDefinition(8, 0.07, 0.10, [[1,0,0,0], [0,0,-1,0], [0,1,0,0.04], [0,0,0,1]])) # idx 10
    capsules.append(CapsuleDefinition(8, 0.05, 0.10, [[1,0,0,0], [0,0,-1,0], [0,1,0,0.10], [0,0,0,1]])) # idx 11
    return capsules

def get_self_collision_whitelist():
    # Definiamo le coppie critiche (indice capsula A, indice capsula B)
    return [
        (1, 7), (1, 8), (1, 10), (1, 11),
        (2, 10), (2, 11),
        (3, 10), (3, 11)
    ]

def get_capsule_endpoints(q, cap_def):
    T_link = T_funcs[cap_def.link_index](q)
    T_capsule = ca.mtimes(T_link, cap_def.T_offset)
    O = T_capsule[:3, 3]
    A = O - (cap_def.length)/2 * T_capsule[:3, 2] # Direzione asse Z locale
    B = O + (cap_def.length)/2 * T_capsule[:3, 2] # Direzione asse Z locale
    return A, B, cap_def.radius

# ===================== MODELLO =====================
def export_frankino_model():
    model = AcadosModel()
    model.name = "frankino_tracking_mpc"

    # --- STATO ---
    q = ca.SX.sym("q", 7)
    dq = ca.SX.sym("dq", 7)
    ddq = ca.SX.sym("ddq", 7)
    x = ca.vertcat(q, dq, ddq)
    u = ca.SX.sym("jerk", 7)

    # --- PARAMETRI ---
    Tf = ca.SX.sym("Tf", 1)
    
    # 3 Sfere (Posizione e Raggio)
    p_obs1 = ca.SX.sym("p_obs1", 3)
    r_obs1 = ca.SX.sym("r_obs1", 1)
    
    p_obs2 = ca.SX.sym("p_obs2", 3)
    r_obs2 = ca.SX.sym("r_obs2", 1)
    
    p_obs3 = ca.SX.sym("p_obs3", 3)
    r_obs3 = ca.SX.sym("r_obs3", 1)
    
    # 1 Piano (Punto di passaggio e Vettore normale)
    p_plane = ca.SX.sym("p_plane", 3)
    n_plane = ca.SX.sym("n_plane", 3)

    # Concateniamo tutto nel vettore dei parametri
    model.p = ca.vertcat(
        Tf, 
        p_obs1, r_obs1, 
        p_obs2, r_obs2, 
        p_obs3, r_obs3, 
        p_plane, n_plane
    )

    # --- DINAMICA ---
    model.x = x
    model.u = u
    model.f_expl_expr = Tf * ca.vertcat(dq, ddq, u)

    # --- VINCOLO COPPIA ---
    M_val = get_M(q)
    G_val = get_G(q)
    C_temp = get_C(q, dq)
    coriolis = (
        ca.mtimes(C_temp, dq) if C_temp.size1() == 7 and C_temp.size2() == 7 else C_temp
    )
    tau_expr = ca.mtimes(M_val, ddq) + coriolis + G_val

    # --- COLLISIONI ---
    capsule_defs = create_capsule_definitions()
    dist_constraints = []

    # 1. Autocollisioni da Whitelist
    whitelist = get_self_collision_whitelist()
    for idxA, idxB in whitelist:
        A1, B1, r1 = get_capsule_endpoints(q, capsule_defs[idxA])
        A2, B2, r2 = get_capsule_endpoints(q, capsule_defs[idxB])
        d_self, _, _, _, _ = geom.dist_capsule_capsule(A1, B1, r1, A2, B2, r2)
        dist_constraints.append(d_self)

    # Liste temporanee per raggruppare i vincoli per ostacolo
    dist_obs1_list = []
    dist_obs2_list = []
    dist_obs3_list = []
    dist_plane_list = []

    # 2. Collisione Ostacoli Esterni
    for i, cap_def in enumerate(capsule_defs):
        if cap_def.link_index == 0:
            continue
        
        A, B, r_cap = get_capsule_endpoints(q, cap_def)
        
        # Sfera 1
        dist_obs1, _, _, _, _ = geom.dist_capsule_capsule(A, B, r_cap, p_obs1, p_obs1, r_obs1)
        dist_obs1_list.append(dist_obs1)
        
        # Sfera 2
        dist_obs2, _, _, _, _ = geom.dist_capsule_capsule(A, B, r_cap, p_obs2, p_obs2, r_obs2)
        dist_obs2_list.append(dist_obs2)
        
        # Sfera 3
        dist_obs3, _, _, _, _ = geom.dist_capsule_capsule(A, B, r_cap, p_obs3, p_obs3, r_obs3)
        dist_obs3_list.append(dist_obs3)
        
        # Piano (SOLO per i link dal 3 in poi)
        if cap_def.link_index > 2:
            dist_plane, _ = geom.dist_capsule_plane(A, B, r_cap, p_plane, n_plane)
            dist_plane_list.append(dist_plane)

    # 3. Impiliamo tutto ordinatamente nella lista principale
    dist_constraints.extend(dist_obs1_list)
    dist_constraints.extend(dist_obs2_list)
    dist_constraints.extend(dist_obs3_list)
    dist_constraints.extend(dist_plane_list)
    
    # --- h EXPR ---
    model.con_h_expr = ca.vertcat(tau_expr, *dist_constraints)
    model.con_h_expr_e = ca.vertcat(tau_expr, *dist_constraints)

    model.cost_y_expr = ca.vertcat(q, dq, ddq, u)
    
    # Costo terminale: solo per definizione, ma comanderanno gli Hard Constraints
    model.cost_y_expr_e = ca.vertcat(q, dq,ddq)

    return model

# ===================== SOLVER =====================
def create_solver():
    model = export_frankino_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = "c_generated_code_tracking"
    n_tau = 7
    n_dist = model.con_h_expr.size1() - n_tau

    # --- SETUP ORARIO ---
    N = 20
    ocp.solver_options.N_horizon = N
    ocp.solver_options.tf = 1.0

    # --- COSTI ---
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    W_q = 1e-6
    W_dq = 1e-6
    W_ddq = 1e-1
    W_u = 0.0
    W_diag = np.concatenate([np.full(7, W_q), np.full(7, W_dq), np.full(7, W_ddq), np.full(7, W_u)])
    ocp.cost.W = np.diag(W_diag)

    ocp.cost.yref = np.zeros(28) # Target zero posizione e accelerazione

    # Pesi Terminal Cost
    W_q_e = 1e5
    W_dq_e = 1e5
    W_ddq_e = 1e2
    # Anche se usiamo Hard Constraints, mettiamo un peso per guidare il solver
    W_diag_e = np.concatenate([np.full(7, W_q_e), np.full(7, W_dq_e), np.full(7, W_ddq_e)])
    ocp.cost.W_e = np.diag(W_diag_e) # Peso alto su tutto lo stato finale
    ocp.cost.yref_e = np.zeros(21)   
    # --- VINCOLI --- (Coppie + Distanze)
    # Limiti Fisici Giunti
    q_min = np.array([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973])
    q_max = np.array([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973])
    dq_min = np.array([-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61])
    dq_max = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
    ddq_min = np.array([-15, -7.5, -10, -12.5, -15, -20, -20])
    ddq_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20])
    tau_lim = 87.0
    jerk_lim = 5000.0
    d_safe_obs = 5e-3  # m sicurezza
    d_safe_coll = 2e-3 # m sicurezza per autocollisioni
    
    
     # 1. State Bounds (x)
    ocp.constraints.idxbx = np.arange(21)
    ocp.constraints.lbx = np.concatenate([q_min, dq_min, ddq_min])
    ocp.constraints.ubx = np.concatenate([q_max, dq_max, ddq_max])

    # 2. Input Bounds (u)
    ocp.constraints.idxbu = np.arange(7)
    ocp.constraints.lbu = np.full(7, jerk_lim * -1)
    ocp.constraints.ubu = np.full(7, jerk_lim)

    # 3. Torque Bounds (h) - Solo Stage Constraints
    ocp.constraints.lh = np.concatenate([np.full(n_tau, -tau_lim), np.full(n_dist, d_safe_obs)])
    ocp.constraints.uh = np.concatenate([np.full(n_tau, tau_lim), np.full(n_dist, 1e5)])
    ocp.constraints.lh_e = np.concatenate([np.full(n_tau, -tau_lim), np.full(n_dist, d_safe_coll)])
    ocp.constraints.uh_e = np.concatenate([np.full(n_tau, tau_lim), np.full(n_dist, 1e5)])

    # Slacks per le distanze (fondamentali per non far crashare il solver)
    ocp.dims.nsh = n_dist
    ocp.constraints.idxsh = np.arange(n_tau, n_tau + n_dist)
    ocp.cost.zl = np.full(n_dist, 100.0)
    ocp.cost.Zl = np.full(n_dist, 500.0)
    ocp.cost.zu = np.zeros(n_dist)
    ocp.cost.Zu = np.zeros(n_dist)
    
    ocp.dims.nsh_e = n_dist
    ocp.constraints.idxsh_e = np.arange(n_tau, n_tau + n_dist)
    ocp.cost.zl_e = np.full(n_dist, 100.0)
    ocp.cost.Zl_e = np.full(n_dist, 500.0)
    ocp.cost.zu_e = np.zeros(n_dist)
    ocp.cost.Zu_e = np.zeros(n_dist)
    
    # --- OPZIONI SOLVER ---
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP_RTI" # SQP standard o SQP_RTI
    
    # Per Hard Constraints terminali, a volte serve più iterazioni o tolleranze diverse
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tol = 1e-4

    # Levenberg-Marquardt aiuta se l'Hessiana diventa singolare
    ocp.solver_options.levenberg_marquardt = 1e-3

    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 5

    # Parametri iniziali e condizioni iniziali
    param_init = np.array([
        1.0,                     # Tf
        1000.0, 1000.0, 1000.0, 0.05, # Sfera 1 (x, y, z, r)
        1000.0, 1000.0, 900.0, 0.05, # Sfera 2 (x, y, z, r) 
        1000.0, 1000.0, 800.0, 0.0, # Sfera 3 (x, y, z, r) 
        0.0, 0.0, 0.0,            # Punto del piano 
        0.0, 0.0, 1.0             # Normale del piano 
    ])
    
    ocp.parameter_values = param_init
    ocp.constraints.x0 = np.zeros(21)

    # Crea JSON e genera codice
    AcadosOcpSolver(ocp, json_file="acados_track.json")
    print("Codice generato con successo in c_generated_code_tracking")

    # --- GENERAZIONE SIMULATORE ---
    sim = AcadosSim()
    sim.model = model
    sim.parameter_values = ocp.parameter_values
    sim.code_export_directory = ocp.code_export_directory
    sim.solver_options.T = 0.001  # Deve matchare dt_sim in C++
    sim.solver_options.integrator_type = "ERK"
    sim.solver_options.num_stages = 4
    sim.solver_options.num_steps = 5

    AcadosSimSolver(sim, json_file="acados_sim_frankino.json")
    print("Codice SIMULATORE generato con successo.")


if __name__ == "__main__":
    create_solver()
