#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <fstream>
#include <iomanip>
#include "MinJerkTrajectory.h"
#include "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles/thunder_frankino.h"

// --- INCLUSIONI ACADOS ---
#include "acados_solver_frankino_tracking_mpc.h"
#include "acados_c/ocp_nlp_interface.h"

using namespace Eigen;

// --- DEFINIZIONI DIMENSIONI AGGIORNATE ---
#define N_HORIZON 20
#define NX 14  // [q, dq]
#define NU 7   // [ddq]
#define NY 14  // [q, u] nel cost stage
#define NYN 14 // [q, dq] nel cost terminale
#define NP 0   // Nessun parametro ostacoli

// Funzione di utilità per l'errore di predizione sulla traiettoria
double simulate_prediction(
    const VectorXd &q_start,
    const VectorXd &dq_start,
    const MatrixXd &U_sequence,
    const VectorXd &q_target,
    double dt_step,
    int horizon_len)
{
    VectorXd q_corrente = q_start;
    VectorXd dq_corrente = dq_start;
    for (int k = 0; k < horizon_len; ++k)
    {
        VectorXd u_k = U_sequence.col(k);
        q_corrente += dq_corrente * dt_step + 0.5 * u_k * std::pow(dt_step, 2);
        dq_corrente += u_k * dt_step;
    }
    return (q_corrente - q_target).norm();
}

int main()
{
    // ----------------------------------------------------------------
    // 1. SETUP ROBOT & ACADOS
    // ----------------------------------------------------------------
    thunder_frankino robot_sim;
    const std::string conf_file = "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles/frankino_conf.yaml";
    robot_sim.load_conf(conf_file);
    int NJ = robot_sim.get_numJoints();

    frankino_tracking_mpc_solver_capsule *capsule = frankino_tracking_mpc_acados_create_capsule();
    frankino_tracking_mpc_acados_create(capsule);
    auto nlp_config = frankino_tracking_mpc_acados_get_nlp_config(capsule);
    auto nlp_dims = frankino_tracking_mpc_acados_get_nlp_dims(capsule);
    auto nlp_in = frankino_tracking_mpc_acados_get_nlp_in(capsule);
    auto nlp_out = frankino_tracking_mpc_acados_get_nlp_out(capsule);

    // ----------------------------------------------------------------
    // 2. LOGGER SETUP
    // ----------------------------------------------------------------
    std::ofstream data_file("simulation_data.csv"), pred_file("prediction_error.csv");
    data_file << "q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,ddq7,t_solve\n";
    pred_file << "time,prediction_error,dt_mpc_node\n";

    // ----------------------------------------------------------------
    // 3. STATO INIZIALE & TRAIETTORIA
    // ----------------------------------------------------------------
    VectorXd q_curr(NJ), dq_curr(NJ), q_final(NJ), dq_final(NJ), ddq_opt(NJ), ddq_final(NJ);
    q_curr << -1.25962, -0.663669, -0.692637, -2.17138, -0.264125, 1.50759, 0.0630972;
    dq_curr.setZero();
    ddq_opt.setZero();

    q_final = q_curr;
    q_final(0) += 0.5;
    dq_final.setZero();
    ddq_final.setZero();

    double t_curr = 0.0, t_duration = 5.0, t_end = t_duration, dt_sim = 0.001, t_hor_lim = 0.02;

    MinJerkTrajectory planner;
    planner.init(q_curr, q_final, dq_curr, dq_final, ddq_opt, ddq_final, t_curr, t_end);

    double x_target[NX];
    for (int i = 0; i < NJ; i++)
    {
        x_target[i] = q_final(i);
        x_target[NJ + i] = dq_final(i);
    }

    std::cout << ">>> Avvio simulazione..." << std::endl;

    // ----------------------------------------------------------------
    // 4. LOOP DI CONTROLLO
    // ----------------------------------------------------------------
    while (t_curr <= t_end)
    {
        double time_to_go = t_end - t_curr;
        double dt_mpc_node = std::max(time_to_go, t_hor_lim) / N_HORIZON;

        // --- A. FEEDBACK STATO CORRENTE ---
        double x0[NX];
        for (int i = 0; i < NJ; i++)
        {
            x0[i] = q_curr(i);
            x0[NJ + i] = dq_curr(i);
        }
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "x", x0);

        // --- B. LOGICA MPC  ---
        if (time_to_go > t_hor_lim)
        {
            planner.init(q_curr, q_final, dq_curr, dq_final, ddq_opt, ddq_final, t_curr, t_end);

            for (int i = 0; i <= N_HORIZON; i++)
            {
                double ti = t_curr + i * dt_mpc_node;
                auto s = planner.evaluate(ti);
                double x_guess[NX], yref_stage[NY];

                for (int j = 0; j < NJ; j++)
                {
                    x_guess[j] = s.pos(j);
                    x_guess[NJ + j] = s.vel(j);
                    yref_stage[j] = s.pos(j); // Stage cost reference
                }

                // Stampe di debug
                printf("MPC Node %d | t = %.3f s | q_des = [", i, ti);
                for (int j = 0; j < NJ; j++)
                    printf("%.3f%s", s.pos(j), (j == NJ - 1 ? "]\n" : ", "));

                ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &dt_mpc_node);
                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_guess);

                if (i < N_HORIZON)
                {
                    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", s.acc.data());
                    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stage);
                }
                else
                {
                    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "lbx", x_target);
                    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "ubx", x_target);
                    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_HORIZON, "yref", x_target);
                }
            }
        }
        else
        {
            int NODO = (int)(time_to_go / dt_mpc_node);
            for (int j = 0; j < NODO; j++)
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", x_target);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, NODO, "lbx", x_target);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, NODO, "ubx", x_target);
        }

        // --- C. SOLVE ---
        auto start_solve = std::chrono::high_resolution_clock::now();
        int status = frankino_tracking_mpc_acados_solve(capsule);
        auto end_solve = std::chrono::high_resolution_clock::now();
        double solve_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(end_solve - start_solve).count() / 1000.0;

        // --- D. RECUPERO & PREDICTION ERROR ---
        double u0[NU];
        MatrixXd ddq_prev = MatrixXd::Zero(NJ, N_HORIZON);
        for (int r = 0; r < N_HORIZON; r++)
        {
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, r, "u", u0);
            if (r == 0)
                for (int j = 0; j < NJ; j++)
                    ddq_opt(j) = u0[j];
            for (int j = 0; j < NJ; j++)
                ddq_prev(j, r) = u0[j];
        }

        double pred_error = simulate_prediction(q_curr, dq_curr, ddq_prev, q_final, dt_mpc_node, N_HORIZON);
        pred_file << t_curr << "," << pred_error << "," << dt_mpc_node << "\n";

        // --- E. FISICA & LOGGING ---
        robot_sim.set_q(q_curr);
        robot_sim.set_dq(dq_curr);
        VectorXd tau_cmd = robot_sim.get_M() * ddq_opt + (robot_sim.get_C() * dq_curr + robot_sim.get_G());
        VectorXd ddq_real = robot_sim.get_M().llt().solve(tau_cmd - (robot_sim.get_C() * dq_curr + robot_sim.get_G()));

        // Log su file dati
        for (int j = 0; j < NJ; j++)
            data_file << q_curr(j) << ",";
        for (int j = 0; j < NJ; j++)
            data_file << dq_curr(j) << ",";
        for (int j = 0; j < NJ; j++)
            data_file << ddq_real(j) << ",";
        data_file << solve_time_ms << "\n";

        // Stampa a video (Real vs MJ)
        auto s_mj = planner.evaluate(t_curr);
        std::cout << "t=" << std::fixed << std::setprecision(3) << t_curr
                  << " | e_q_norm=" << (q_curr - s_mj.pos).norm()
                  << " | t_solve=" << solve_time_ms << " ms" << std::endl;

        // Integrazione
        q_curr += dq_curr * dt_sim + 0.5 * ddq_real * pow(dt_sim, 2);
        dq_curr += ddq_real * dt_sim;
        t_curr += dt_sim;
    }

    std::cout << ">>> Simulazione finita." << std::endl;
    data_file.close();
    pred_file.close();
    frankino_tracking_mpc_acados_free(capsule);
    return 0;
}