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

# Carichiamo solo la dinamica necessaria per calcolare le coppie
get_M = ca.Function.load(os.path.join(path_to_files, "M.casadi"))
get_C = ca.Function.load(os.path.join(path_to_files, "C.casadi"))
get_G = ca.Function.load(os.path.join(path_to_files, "G.casadi"))

def export_clean_model():
    model = AcadosModel()
    model.name = "frankino_tracking_mpc"

    # --- SIMBOLI ---
    # Stato: x = [q, dq] (14 elementi)
    q = ca.SX.sym("q", 7)
    dq = ca.SX.sym("dq", 7)
    x = ca.vertcat(q, dq)

    # Input: u = ddq (7 elementi)
    ddq = ca.SX.sym("ddq", 7)
    u = ddq
    
    # x_final = ca.SX.sym("p_obs", x.size1())
    # model.p = x_final

    # --- DINAMICA ---
    model.x = x
    model.u = u
    # x_dot = [dq, u]
    model.f_expl_expr = ca.vertcat(dq, u)
    
    

    # --- CALCOLO COPPIA (Vincolo Fisico) ---
    # Tau = M(q)*u + C(q,dq)*dq + G(q)
    M_val = get_M(q)
    G_val = get_G(q)
    C_temp = get_C(q, dq)
    coriolis = (
        ca.mtimes(C_temp, dq) if C_temp.size1() == 7 and C_temp.size2() == 7 else C_temp
    )
    tau_expr = ca.mtimes(M_val, u) + coriolis + G_val

    # Aggiungiamo la coppia ai vincoli algebrici (h)
    model.con_h_expr = tau_expr
    
    # error = ca.SX.sym("error", x.size1())
    # error[0] = x[0] - x_final[0]
    # error[1] = x[1] - x_final[1]
    # error[2] = x[2] - x_final[2]
    # error[3] = x[3] - x_final[3]
    # error[4] = x[4] - x_final[4]
    # error[5] = x[5] - x_final[5]
    # error[6] = x[6] - x_final[6]
    # error[7] = x[7] - x_final[7]
    # error[8] = x[8] - x_final[8]
    # error[9] = x[9] - x_final[9]
    # error[10] = x[10] - x_final[10]
    # error[11] = x[11] - x_final[11]
    # error[12] = x[12] - x_final[12]
    # error[13] = x[13] - x_final[13]
    # Nessun vincolo h terminale (la coppia a fine traiettoria è determinata dallo stato finale, di solito fermo)
    # model.con_h_expr_e = ca.SX.zeros(14)

    # --- FUNZIONE DI COSTO ---
    # y = [q, u] -> Minimizziamo posizione e accelerazione
    model.cost_y_expr = ca.vertcat(q, u)
    
    # # Costo terminale: solo per definizione, ma comanderanno gli Hard Constraints
    # model.cost_y_expr_e = ca.vertcat(q, dq)

    return model

def create_solver():
    model = export_clean_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.code_export_directory = "c_generated_code_tracking"

    # --- SETUP ORARIO ---
    N = 20
    Tf = 5.0 # Verrà sovrascritto dal tuo Shrinking Horizon in C++
    ocp.dims.N = N
    ocp.solver_options.tf = Tf

    # --- COSTI ---
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"  

    # Pesi Stage Cost
    # y = [q, u] (14)
    # Obiettivo: Minima posizione e accelerazione (W_q e W_u)
    W_q = 1e-6
    W_u = 1e-1  
    
    # Matrice W (14x14) per y = [q, u]
    ocp.cost.W = np.diag(np.concatenate([np.full(7, W_q), np.full(7, W_u)]))

    ocp.cost.yref = np.zeros(14) # Target zero posizione e accelerazione

    # # Pesi Terminal Cost
    # # Anche se usiamo Hard Constraints, mettiamo un peso per guidare il solver
    # ocp.cost.W_e = np.eye(14) * 1000.0 # Peso alto su tutto lo stato finale
    # ocp.cost.yref_e = np.zeros(14) # Verrà aggiornato in C++

    # --- VINCOLI ---
    
    # Limiti Fisici Giunti
    q_max = np.array([2.89, 1.76, 2.89, -0.06, 2.89, 3.75, 2.89])
    q_min = np.array([-2.89, -1.76, -2.89, -3.07, -2.89, -0.01, -2.89])
    dq_min = np.array([-2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61])
    dq_max = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
    ddq_min = np.array([-15, -7.5, -10, -12.5, -15, -20, -20])
    ddq_max = np.array([15, 7.5, 10, 12.5, 15, 20, 20])
    tau_lim = 87.0

    # 1. State Bounds (x)
    ocp.constraints.idxbx = np.arange(14)
    ocp.constraints.lbx = np.concatenate([q_min, dq_min])
    ocp.constraints.ubx = np.concatenate([q_max, dq_max])

    # 2. Input Bounds (u)
    ocp.constraints.idxbu = np.arange(7)
    ocp.constraints.lbu = ddq_min
    ocp.constraints.ubu = ddq_max

    # 3. Torque Bounds (h) - Solo Stage Constraints
    ocp.constraints.lh = np.full(7, -tau_lim)
    ocp.constraints.uh = np.full(7, +tau_lim)
    
    # 4. HARD CONSTRAINTS TERMINALI (Cruciale per la tua richiesta)
    # Definiamo che all'ultimo nodo (N), TUTTI gli stati (q, dq) sono vincolati.
    # In Python mettiamo dummy values. In C++ aggiornerai lbx_e e ubx_e con il target.
    ocp.constraints.idxbx_e = np.arange(14) 
    ocp.constraints.lbx_e = np.zeros(14)
    ocp.constraints.ubx_e = np.zeros(14)
    
    # ocp.constraints.lh_e = np.zeros(14)
    # ocp.constraints.uh_e = np.zeros(14)

    # --- OPZIONI SOLVER ---
    ocp.solver_options.qp_solver = "FULL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.nlp_solver_type = "SQP" # SQP standard o SQP_RTI
    
    # Per Hard Constraints terminali, a volte serve più iterazioni o tolleranze diverse
    ocp.solver_options.qp_solver_iter_max = 50
    ocp.solver_options.nlp_solver_max_iter = 100
    ocp.solver_options.tol = 1e-4

    # Levenberg-Marquardt aiuta se l'Hessiana diventa singolare
    ocp.solver_options.levenberg_marquardt = 1e-3
    
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 5

    # Parametri iniziali (nessuno ora, ma Acados li vuole se definiti nel model)
    # Nota: nel model clean non ho definito model.p, quindi non serve parameter_values
    # ocp.parameter_values = np.zeros(14)
    ocp.constraints.x0 = np.zeros(14)

    # Crea JSON e genera codice
    AcadosOcpSolver(ocp, json_file="acados_track.json")
    print("Codice generato con successo in c_generated_code_tracking")

if __name__ == "__main__":
    create_solver()
    