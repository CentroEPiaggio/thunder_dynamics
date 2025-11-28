import casadi as ca
import numpy as np
from acados_template import AcadosOcp, AcadosOcpSolver, AcadosModel
import os
import matplotlib.pyplot as plt
import time


q0 = (-0.14724, -0.526, -0.565, -2.1099, -0.312, 1.557, -1.733)
qf = (-1.25962, -0.663669, -0.692637, -2.17138, -0.264125, 1.50759, 0.0630972)


# =============================================================================
# PARTE 1: CONFIGURAZIONE E GENERAZIONE DEL SOLVER
# =============================================================================

path_to_files = (
    "/home/thunder_dev/thunder_dynamics/src/thunder/build/frankino_generatedFiles/"
)

if not os.path.exists(path_to_files):
    raise FileNotFoundError(f"Errore: La cartella {path_to_files} non esiste.")

# Carichiamo le matrici
get_M_func = ca.Function.load(os.path.join(path_to_files, "M.casadi"))
get_C_func = ca.Function.load(os.path.join(path_to_files, "C.casadi"))
get_G_func = ca.Function.load(os.path.join(path_to_files, "G.casadi"))

# Variabili simboliche
n_q = 7
q = ca.SX.sym("q", n_q)
q_dot = ca.SX.sym("q_dot", n_q)
tau = ca.SX.sym("tau", n_q)

x = ca.vertcat(q, q_dot)
u = tau
# ================= PARAMETRI =================
q_target = ca.SX.sym("q_target", n_q)
dq_target = ca.SX.sym("q_dot_target", n_q)
p = ca.vertcat(q_target, dq_target)

# Dinamica
C_tmp = get_C_func(q, q_dot)
if C_tmp.size1() == 7 and C_tmp.size2() == 7:
    coriolis = ca.mtimes(C_tmp, q_dot)
else:
    coriolis = C_tmp

rhs = tau - coriolis - get_G_func(q)
q_ddot = ca.solve(get_M_func(q), rhs)
f_expl = ca.vertcat(q_dot, q_ddot)

# ================= COSTI =================
W_q = 1000.0
W_v = 1000.0
W_tau = 10.0

pos_error = q - q_target
vel_error = q_dot - dq_target

cost_expr = (
    W_q * ca.sumsqr(pos_error) + W_v * ca.sumsqr(vel_error) + W_tau * ca.sumsqr(u)
)

cost_expr_e = 1000.0 * ca.sumsqr(pos_error) + 1000.0 * ca.sumsqr(vel_error)

# ================= MODELLO =================
model = AcadosModel()
model.name = "frankino_pos_ctrl"
model.x = x
model.u = u
model.p = p
model.f_expl_expr = f_expl

model.cost_expr_ext_cost = cost_expr  # cost for intermediate stages
model.cost_expr_ext_cost_0 = cost_expr  # cost for initial stage
model.cost_expr_ext_cost_e = cost_expr_e  # cost for terminal stage

# ================= OCP =================
ocp = AcadosOcp()
ocp.model = model

ocp.solver_options.N_horizon = 20
ocp.solver_options.tf = 1.0

ocp.cost.cost_type = "EXTERNAL"
ocp.cost.cost_type_e = "EXTERNAL"

tau_max = (87, 87, 87, 87, 12, 12, 12)
ocp.constraints.lbu = np.array([-tau for tau in tau_max])
ocp.constraints.ubu = np.array([+tau for tau in tau_max])
ocp.constraints.idxbu = np.arange(7)


ocp.constraints.x0 = np.zeros(14)

ocp.parameter_values = np.zeros(14)


ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
ocp.solver_options.integrator_type = "ERK"
ocp.solver_options.nlp_solver_type = "SQP"
ocp.solver_options.nlp_solver_max_iter = 100

if not os.path.exists("c_generated_code"):
    os.makedirs("c_generated_code")

ocp_solver = AcadosOcpSolver(ocp, json_file="acados_ocp.json")
print("Generazione completata! Inizio simulazione...")

# =============================================================================
# PARTE 2: SIMULAZIONE
# =============================================================================

dt = 0.05
dyn_func = ca.Function("dyn_func", [x, u], [f_expl])


def integrate_rk4(x_curr, u_curr, dt):
    k1 = dyn_func(x_curr, u_curr)
    k2 = dyn_func(x_curr + dt / 2 * k1, u_curr)
    k3 = dyn_func(x_curr + dt / 2 * k2, u_curr)
    k4 = dyn_func(x_curr + dt * k3, u_curr)
    x_next = x_curr + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)
    return np.array(x_next.full()).flatten()


# Setup simulazione
N_sim = 100
n_x = 14
n_u = 7
n_N = 20

x0 = np.zeros(n_x)
q_target_val = np.zeros(7)

q_target_val[0] = 1.0
q_target_val[3] = -1.5
dq_target_val = np.zeros(7)
dq_target_val[0] = 0.5
dq_target_val[3] = -1.0

sim_x = np.zeros((N_sim + 1, n_x))
sim_u = np.zeros((N_sim, n_u))
sim_x[0, :] = x0

x_current = x0.copy()

# WARM START ===============================================
u_init = np.zeros((n_N, n_u))
x_init = np.tile(x_current, (n_N + 1, 1))

print("Avvio loop di controllo...")
start_time = time.time()

# ==========================================================
# CONTROL LOOP
# ==========================================================
for i in range(N_sim):

    # vincolo x0
    ocp_solver.set(0, "lbx", x_current)
    ocp_solver.set(0, "ubx", x_current)

    # Warm-start: stati + comandi + parametri
    for k in range(n_N):
        ocp_solver.set(k, "u", u_init[k])
        ocp_solver.set(k, "x", x_init[k])
        ocp_solver.set(k, "p", np.concatenate((q_target_val, dq_target_val)))

    #  # vincolo x0
    # ocp_solver.set(n_N, "lbx", np.concatenate((q_target_val, dq_target_val)))
    # ocp_solver.set(n_N, "ubx", np.concatenate((q_target_val, dq_target_val)))

    # Solve
    status = ocp_solver.solve()

    if status != 0:
        print(f"[Step {i}]  Solver status: {status}. Uso fallback.")
        try:
            stats = ocp_solver.get_stats()
            print("   Stats solver:", stats)
        except:
            pass
        # fallback: usa ultimo comando valido
        u_opt = u_init[0]
    else:
        u_opt = ocp_solver.get(0, "u")

        # aggiorna warm-start (shift)
        for k in range(n_N - 1):
            u_init[k] = ocp_solver.get(k + 1, "u")
            x_init[k] = ocp_solver.get(k + 1, "x")
        u_init[-1] = u_init[-2]
        x_init[-1] = x_init[-2]

    # Integrazione
    x_next = integrate_rk4(x_current, u_opt, dt)
    sim_u[i, :] = u_opt
    sim_x[i + 1, :] = x_next
    x_current = x_next

total_time = time.time() - start_time
print(f"Finito! Tempo totale: {total_time:.4f}s")

# =============================================================================
# PLOT RISULTATI
# =============================================================================

t_span = np.linspace(0, N_sim * dt, N_sim + 1)

plt.figure(figsize=(10, 8))

plt.subplot(3, 1, 1)
plt.title("Posizione")
plt.plot(t_span, sim_x[:, 0], label="q0")
plt.plot(t_span, sim_x[:, 3], label="q3")
plt.axhline(y=q_target_val[0], color="r", linestyle="--")
plt.axhline(y=q_target_val[3], color="b", linestyle="--")
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 2)
plt.title("Velocità")
plt.plot(t_span, sim_x[:, 7], label="dq0")
plt.plot(t_span, sim_x[:, 10], label="dq3")
plt.axhline(y=dq_target_val[0], color="r", linestyle="--")
plt.axhline(y=dq_target_val[3], color="b", linestyle="--")
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.title("Coppie")
plt.plot(t_span[:-1], sim_u[:, 0], label="tau0")
plt.plot(t_span[:-1], sim_u[:, 3], label="tau3")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig("risultati_simulazione.png", dpi=300)
print("Figura salvata come risultati_simulazione.png")
# plt.show(block=True)
