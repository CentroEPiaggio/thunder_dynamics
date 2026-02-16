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
#include "acados_sim_solver_frankino_tracking_mpc.h"
#include "acados_solver_frankino_tracking_mpc.h"
#include "acados_c/ocp_nlp_interface.h"

using namespace Eigen;

// --- DEFINIZIONI DIMENSIONI ---
#define N_HORIZON 20
#define NX 21  // [q, dq, ddq]
#define NU 7   // [jerk]
#define NY 28  // [q, dq, ddq, jerk] nel cost stage
#define NYN 21 // [q,dq,ddq] nel cost terminale
#define NP 1   // Nessun parametro ostacoli

// Funzione di utilità per l'errore di predizione sulla traiettoria
double simulate_prediction(
    const VectorXd &q_start,
    const VectorXd &dq_start,
    const VectorXd &ddq_start,
    const MatrixXd &U_sequence,
    const VectorXd &q_target,
    double dt_step,
    int horizon_len)
{
    VectorXd q_corrente = q_start;
    VectorXd dq_corrente = dq_start;
    VectorXd ddq_corrente = ddq_start;
    for (int k = 0; k < horizon_len; ++k)
    {
        VectorXd u_k = U_sequence.col(k);
        q_corrente += dq_corrente * dt_step + 0.5 * ddq_corrente * std::pow(dt_step, 2) + (1.0 / 6.0) * u_k * std::pow(dt_step, 3);
        dq_corrente += ddq_corrente * dt_step + 0.5 * u_k * std::pow(dt_step, 2);
        ddq_corrente += u_k * dt_step;
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

    frankino_tracking_mpc_sim_solver_capsule *sim_capsule = frankino_tracking_mpc_acados_sim_solver_create_capsule();
    int status_sim = frankino_tracking_mpc_acados_sim_create(sim_capsule);
    if (status_sim != 0)
    {
        std::cout << "Errore creazione simulatore Acados! Status: " << status_sim << std::endl;
        return 1;
    }

    auto sim_config = frankino_tracking_mpc_acados_get_sim_config(sim_capsule);
    auto sim_in = frankino_tracking_mpc_acados_get_sim_in(sim_capsule);
    auto sim_out = frankino_tracking_mpc_acados_get_sim_out(sim_capsule);
    auto sim_dims = frankino_tracking_mpc_acados_get_sim_dims(sim_capsule);
    // ----------------------------------------------------------------
    // 2. LOGGER SETUP
    // ----------------------------------------------------------------
    std::ofstream data_file("simulation_data.csv"), pred_file("prediction_error.csv"), trajs_file("trajectories_history.csv"), debug_file("debug_mismatch.csv");
    data_file << "q1,q2,q3,q4,q5,q6,q7,dq1,dq2,dq3,dq4,dq5,dq6,dq7,ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,ddq7,jerk1,jerk2,jerk3,jerk4,jerk5,jerk6,jerk7,t_solve\n";
    pred_file << "time,prediction_error,dt_mpc_node\n";
    debug_file << "t,q_corrente,q_sim_next,q_mpc_ipotizzata,dt_sim,dt_mpc\n";

    // ----------------------------------------------------------------
    // 3. STATO INIZIALE & TRAIETTORIA
    // ----------------------------------------------------------------
    VectorXd q_curr(NJ), dq_curr(NJ), q_final(NJ), dq_final(NJ), ddq_curr(NJ), ddq_final(NJ), jerk_opt(NJ);
    q_curr << -1.25962, -0.663669, -0.692637, -2.17138, -0.264125, 1.50759, 0.0630972;
    dq_curr.setZero();
    ddq_curr.setZero();
    jerk_opt.setZero();

    q_final = q_curr;
    q_final << -0.14724, -0.526, -0.565, -2.1099, -0.312, 1.557,-1.733;
    dq_final.setZero();
    dq_final(0) += 0.9;
    ddq_final.setZero();

    double t_curr = 0.0, t_duration = 5.0, t_end = t_duration, dt_sim = 0.001, t_hor_lim = 0.2;

    double lb[NX] = {-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973,
                     -2.175, -2.175, -2.175, -2.175, -2.61, -2.61, -2.61,
                     -15, -7.5, -10, -12.5, -15, -20, -20};
    double ub[NX] = {2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973,
                     2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61,
                     15, 7.5, 10, 12.5, 15, 20, 20};

    MinJerkTrajectory planner;
    planner.init(q_curr, q_final, dq_curr, dq_final, ddq_curr, ddq_final, t_curr, t_end);

    double x_target_ub[NX], x_target_lb[NX];
    double tolerance = 1e-6; // Tolleranza per i vincoli hard
    for (int i = 0; i < NJ; i++)
    {
        x_target_ub[i] = q_final(i) + tolerance;
        x_target_lb[i] = q_final(i) - tolerance;
        x_target_ub[NJ + i] = dq_final(i) + tolerance;
        x_target_lb[NJ + i] = dq_final(i) - tolerance;
        x_target_ub[2 * NJ + i] = ddq_final(i) + tolerance;
        x_target_lb[2 * NJ + i] = ddq_final(i) - tolerance;
    }

    std::cout << ">>> Avvio simulazione..." << std::endl;

    // ----------------------------------------------------------------
    // 4. VARIABILI PER IL CONTROLLO A NODO FISSO
    // ----------------------------------------------------------------
    double next_mpc_time = 0.0; // Prossimo istante in cui ricalcolare MPC
    double frequenza = 100;     // Frequenza di ricalcolo MPC (s)
    bool first_mpc_call = true; // True per prima chiamata MPC
    VectorXd current_u0(NJ);    // Controllo corrente da applicare
    current_u0.setZero();

    // Matrice per salvare la sequenza di controllo
    MatrixXd U_sequence(NJ, N_HORIZON);
    U_sequence.setZero();

    double solve_time_ms = 0.0; // Variabile per tempo di risoluzione MPC

    // ----------------------------------------------------------------
    // 5. LOOP DI CONTROLLO
    // ----------------------------------------------------------------
    while (t_curr <= t_end + tolerance)
    {
        double time_to_go = t_end - t_curr;
        double Tf = std::max(time_to_go, t_hor_lim);
        double dt_mpc_node = Tf / N_HORIZON;

        // --- A. FEEDBACK STATO CORRENTE ---
        double x0[NX];
        for (int i = 0; i < NJ; i++)
        {
            x0[i] = q_curr(i);
            x0[NJ + i] = dq_curr(i);
            x0[2 * NJ + i] = ddq_curr(i);
        }

        // --- B. VERIFICO SE È TEMPO DI RICALCOLARE MPC ---
        bool solve_mpc = false;
        if (first_mpc_call || t_curr >= next_mpc_time - 1e-6)
        {
            solve_mpc = true;
            first_mpc_call = false;
            next_mpc_time = t_curr + 1.0 / frequenza;

            std::cout << "\n>>> MPC RICALCOLATO a t = " << t_curr
                      << " s, next MPC a t = " << next_mpc_time
                      << " s, dt_mpc = " << dt_mpc_node << " s" << std::endl;
        }
        // --- C. LOGICA MPC (solo se è tempo di risolvere) ---
        if (solve_mpc)
        {
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
            ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "x", x0);

            // --- B. LOGICA MPC  ---
            if (time_to_go >= t_hor_lim)
            {
                planner.init(q_curr, q_final, dq_curr, dq_final, ddq_curr, ddq_final, t_curr, t_end);
                trajs_file << t_curr;
                double x_prev[NX], u_prev[NU];

                // 1. 21 valori per la MinJerk
                for (int i = 0; i <= N_HORIZON; i++)
                {
                    double ti = t_curr + i * dt_mpc_node;
                    auto s = planner.evaluate(ti);
                    trajs_file << "," << s.pos(0);
                }

                // 2. 21 valori per la Predizione MPC
                double x_node[NX];
                for (int i = 0; i <= N_HORIZON; i++)
                {
                    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_node);
                    trajs_file << "," << x_node[0];
                }
                trajs_file << "\n";

                for (int i = 0; i <= N_HORIZON; i++)
                {
                    double ti = t_curr + i * dt_mpc_node;
                    auto s = planner.evaluate(ti);
                    double x_guess[NX], yref_stage[NY];

                    for (int j = 0; j < NJ; j++)
                    {
                        yref_stage[j] = s.pos(j);      // Stage cost reference
                        yref_stage[NJ + j] = s.vel(j); // Target velocità
                        yref_stage[2 * NJ + j] = 0.0;  // Target accelerazione
                        yref_stage[3 * NJ + j] = 0.0;  // Target jerk qualsiasi tanto W_u = 0
                    }

                    if (i < N_HORIZON)
                    {
                        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stage);
                    }
                    else
                    {
                        // Nodo terminale: usa il target finale (q_final, dq_final)
                        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_HORIZON, "yref", x_target_lb);
                        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "lbx", x_target_lb);
                        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "ubx", x_target_ub);
                    }

                    // Stampe di debug
                    printf("MPC Node %d | t = %.3f s | q_des = [", i, ti);
                    for (int j = 0; j < NJ; j++)
                        printf("%.3f%s", s.pos(j), (j == NJ - 1 ? "]\n" : ", "));

                    // --- B. SET INITIAL GUESS (WARM START) ---
                    if (!first_mpc_call)
                    {
                        // Shifting: usa il nodo (i+1) della soluzione precedente per il nodo (i) attuale
                        if (i < N_HORIZON)
                        {
                            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i + 1, "x", x_prev);
                            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i + 1, "u", u_prev);
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_prev);
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_prev);
                        }
                    }
                    else
                    {
                        // Solo alla prima chiamata assoluta, inizializza con la MinJerk
                        double x_guess[NX];
                        for (int j = 0; j < NJ; j++)
                        {
                            x_guess[j] = s.pos(j);
                            x_guess[NJ + j] = s.vel(j);
                            x_guess[2 * NJ + j] = s.acc(j);
                        }
                        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_guess);
                        if (i < N_HORIZON)
                            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", s.jerk.data());
                    }

                    ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "parameter_values", &Tf);
                }
            }
            else
            {
                trajs_file << t_curr;
                int NODO = (int)std::round(time_to_go / dt_mpc_node);

                for (int i = 0; i <= N_HORIZON; i++)
                {
                    double ti = t_curr + i * dt_mpc_node;
                    auto s = planner.evaluate(ti);
                    trajs_file << "," << s.pos(0);
                }

                double x_node[NX];
                for (int i = 0; i <= N_HORIZON; i++)
                {
                    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, i, "x", x_node);
                    trajs_file << "," << x_node[0];
                }
                trajs_file << "\n";

                std::cout << "NODO = " << NODO << std::endl;
                for (int i = 0; i <= N_HORIZON; i++)
                {

                    if (i == NODO && NODO != 0)
                    {
                        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_target_lb);
                        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", x_target_lb);
                        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", x_target_ub);
                    }
                    else if (i > NODO)
                    {
                        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lb);
                        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ub);
                    }
                }
            }

            // --- D. SOLVE ---
            auto start_solve = std::chrono::high_resolution_clock::now();
            int status = frankino_tracking_mpc_acados_solve(capsule);
            auto end_solve = std::chrono::high_resolution_clock::now();
            solve_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(end_solve - start_solve).count() / 1000.0;

            // --- E. CONTROLLO ERRORI E DEBUG RESIDUI ---
            if (status != ACADOS_SUCCESS)
            {
                printf("\n>>> ERRORE SOLVER! Status: %d a t = %.3f s\n", status, t_curr);
            }

            // Recupera u0
            double u0_temp[NU];
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0_temp);

            // Salva u0 corrente (PRIMO CONTROLLO DELLA SEQUENZA)
            for (int j = 0; j < NJ; j++)
            {
                current_u0(j) = u0_temp[j];
            }

            // Salva tutta la sequenza per calcolare l'errore di predizione
            for (int r = 0; r < N_HORIZON; r++)
            {
                ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, r, "u", u0_temp);
                for (int j = 0; j < NJ; j++)
                {
                    U_sequence(j, r) = u0_temp[j];
                }
            }

            // Calcola errore di predizione
            double pred_error = simulate_prediction(q_curr, dq_curr, ddq_curr, U_sequence, q_final, dt_mpc_node, N_HORIZON);
            pred_file << t_curr << "," << pred_error << "," << dt_mpc_node << "\n";

            std::cout << ">>> MPC solve time: " << solve_time_ms << " ms" << std::endl;
            std::cout << ">>> Pred error: " << pred_error << std::endl;
        }
        else
        {
            // Quando non risolviamo MPC, applichiamo lo stesso u0 calcolato precedentemente
            std::cout << ">>> Applico il precedente u0 (sample-and-hold) at t = " << t_curr
                      << ", prossimo MPC at: " << next_mpc_time << std::endl;
        }

        // --- F. FISICA & INTEGRAZIONE CON ACADOS SIM ---
        double x_current_sim[NX];
        for (int i = 0; i < NJ; i++)
        {
            x_current_sim[i] = q_curr(i);
            x_current_sim[NJ + i] = dq_curr(i);
            x_current_sim[2 * NJ + i] = ddq_curr(i);
        }

        // Settiamo stato attuale e controllo (ddq_curr)
        sim_in_set(sim_config, sim_dims, sim_in, "x", x_current_sim);
        sim_in_set(sim_config, sim_dims, sim_in, "u", current_u0.data());

        // Passo temporale del simulatore uguale a dt_sim
        sim_in_set(sim_config, sim_dims, sim_in, "T", &dt_sim);

        // 2. Esegui il passo di simulazione
        int sim_status = frankino_tracking_mpc_acados_sim_solve(sim_capsule);
        if (sim_status != ACADOS_SUCCESS)
        {
            std::cerr << "Errore nel simulatore Acados!" << std::endl;
            break;
        }

        // 3. Recupera lo stato successivo (xn)
        double x_next[NX];
        sim_out_get(sim_config, sim_dims, sim_out, "xn", x_next);
        auto s_mj = planner.evaluate(t_curr);
        static double last_q_mpc_pred = q_curr(0); // inizializzo con la posizione corrente

        // Log su debug file
        if (solve_mpc)
        {
            // Recupera la predizione MPC per il primo nodo
            double x_pred[NX];
            ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 1, "x", x_pred);
            last_q_mpc_pred = x_pred[0]; // Prima giunto, primo nodo di predizione
        }

        debug_file << t_curr << ","
                   << q_curr(0) << ","       // Dove sono ora
                   << x_next[0] << ","       // Dove sarò tra 1ms (Simulazione)
                   << last_q_mpc_pred << "," // Dove MPC predice che sarò (Solver)
                   << dt_sim << ","
                   << dt_mpc_node << "\n";

        // Log simulation_data.csv
        for (int j = 0; j < NJ; j++)
            data_file << q_curr(j) << ",";
        for (int j = 0; j < NJ; j++)
            data_file << dq_curr(j) << ",";
        for (int j = 0; j < NJ; j++)
            data_file << ddq_curr(j) << ",";
        for (int j = 0; j < NJ; j++)
            data_file << current_u0(j) << ",";
        data_file << solve_time_ms << "\n";

        if (fmod(t_curr, 0.1) < dt_sim)
        {
            std::cout << "t=" << std::fixed << std::setprecision(3) << t_curr
                      << " | e_q_norm=" << (q_curr - s_mj.pos).norm()
                      << " | u0_applied=[" << current_u0.transpose() << "]"
                      << " | time_to_next_mpc=" << (next_mpc_time - t_curr) << " s"
                      << std::endl;
        }
        // --- G. AGGIORNAMENTO STATO ---
        for (int i = 0; i < NJ; i++)
        {
            q_curr(i) = x_next[i];
            dq_curr(i) = x_next[NJ + i];
            ddq_curr(i) = x_next[2 * NJ + i];
        }
        t_curr += dt_sim;
    }

    std::cout << ">>> Simulazione finita." << std::endl;
    data_file.close();
    pred_file.close();
    trajs_file.close();
    frankino_tracking_mpc_acados_free(capsule);
    frankino_tracking_mpc_acados_free_capsule(capsule);
    frankino_tracking_mpc_acados_sim_free(sim_capsule);
    frankino_tracking_mpc_acados_sim_solver_free_capsule(sim_capsule);
    return 0;
}