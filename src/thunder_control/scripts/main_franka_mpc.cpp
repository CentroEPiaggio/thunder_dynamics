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
#define NY 7   // [u] <-- MODIFICATO: Minimizzi solo accelerazione nello stage
#define NYN 14 // [q, dq] nel cost terminale
#define NP 0   // Nessun parametro ostacoli <-- MODIFICATO

const std::string conf_file = "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles/frankino_conf.yaml";

int main()
{
    // ----------------------------------------------------------------
    // 1. SETUP ROBOT
    // ----------------------------------------------------------------
    thunder_frankino robot_sim;
    robot_sim.load_conf(conf_file);
    int NJ = robot_sim.get_numJoints();

    // Logger
    std::ofstream data_file("simulation_data.csv");
    data_file << "q1,q2,q3,q4,q5,q6,q7,"
                 "dq1,dq2,dq3,dq4,dq5,dq6,dq7,"
                 "ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,ddq7,t_solve\n";

    // Vettori di stato
    VectorXd q_curr = VectorXd::Zero(NJ);
    VectorXd dq_curr = VectorXd::Zero(NJ);
    VectorXd ddq_opt = VectorXd::Zero(NJ);
    VectorXd tau_cmd = VectorXd::Zero(NJ);
    double eps = 1e-4; // Tolleranza per i vincoli

    // Posizione Iniziale
    q_curr << -1.25962, -0.663669, -0.692637, -2.17138, -0.264125, 1.50759, 0.0630972;

    // Init dinamica
    robot_sim.set_q(q_curr);
    robot_sim.set_dq(dq_curr);

    // ----------------------------------------------------------------
    // 2. SETUP ACADOS
    // ----------------------------------------------------------------
    std::cout << ">>> Inizializzazione Acados..." << std::endl;

    frankino_tracking_mpc_solver_capsule *capsule = frankino_tracking_mpc_acados_create_capsule();
    if (frankino_tracking_mpc_acados_create(capsule) != 0)
    {
        std::cerr << "Errore creazione Acados!" << std::endl;
        return 1;
    }

    ocp_nlp_config *nlp_config = frankino_tracking_mpc_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = frankino_tracking_mpc_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = frankino_tracking_mpc_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = frankino_tracking_mpc_acados_get_nlp_out(capsule);

    // ----------------------------------------------------------------
    // 3. TRAJECTORY DEFINITION (MinJerk + MPC Target)
    // ----------------------------------------------------------------
    MinJerkTrajectory planner, original_planner;

    double t_start = 0.0;
    double t_duration = 5.0;
    double t_end = t_start + t_duration;

    // Stato iniziale
    VectorXd q_start = q_curr;
    VectorXd dq_start = VectorXd::Zero(NJ);
    VectorXd ddq_start = VectorXd::Zero(NJ);

    // Stato finale
    VectorXd q_final = VectorXd::Zero(NJ);
    VectorXd dq_final = VectorXd::Zero(NJ);
    VectorXd ddq_final = VectorXd::Zero(NJ);

    q_final << -0.157, -0.504, -0.623, -2.258, -0.332, 1.637, -1.728;
    dq_final << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    ddq_final << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    // Target MPC (x_T)
    double x_target[NX];
    for (int i = 0; i < NJ; i++)
    {
        x_target[i] = q_final(i);
        x_target[NJ + i] = dq_final(i);
    }

    // Inizializzazione MinJerk
    planner.init(q_start, q_final,
                 dq_start, dq_final,
                 ddq_start, ddq_final,
                 t_start, t_end);

    original_planner.init(q_start, q_final,
                          dq_start, dq_final,
                          ddq_start, ddq_final,
                          t_start, t_end);

    // ----------------------------------------------------------------
    // 4. LOOP DI CONTROLLO
    // ----------------------------------------------------------------
    double dt_sim = 0.001; // 1 kHz loop fisico
    double t_curr = 0.0;

    std::cout << ">>> Avvio simulazione..." << std::endl;

    while (t_curr <= (t_end))
    {

        // ------------------------------------------------------------
        // [SHRINKING HORIZON LOGIC]
        // ------------------------------------------------------------
        double time_to_go = t_end - t_curr;

        // // Protezione: non scendere troppo (evita divisioni per zero o step minuscoli)
        // // Ma deve essere abbastanza piccolo da convergere. 50ms è un buon lower bound.
        // if (time_to_go < 0.02)
        // {
        //     time_to_go = 0.02;
        // }

        // ------------------------------------------------------------
        // 1. REPLANNING TRAIETTORIA
        // ------------------------------------------------------------

        if (time_to_go > 0.02)
        {
            // Start: Stato corrente del robot (reale o stimato)
            // End: Target fisso
            // Acc Start: Usiamo ddq_opt (l'ultima input applicata) per garantire continuità nell'accelerazione
            planner.init(q_curr, q_final,
                         dq_curr, dq_final,
                         ddq_opt, ddq_final, // non + rilevante... se imposto ddq_real e magari il robot ha subito una collisione e ha un'accelerazione diversa, il planner potrebbe generare un salto improvviso nell'accelerazione elevata o diversa minjerk partirebbe da li
                         t_curr, t_end);

            // auto s_original = original_planner.evaluate(t_curr);
            // auto s_new = planner.evaluate(t_curr);

            // std::cout << "t=" << t_curr
            //           << " | ddq_original=" << s_original.acc.transpose()
            //           << "\n"
            //           << " | ddq_opt=" << ddq_opt.transpose()
            //           << "\n"
            //           << " | ddq_new_minjerk=" << s_new.acc.transpose() << std::endl;
        }
        else if (time_to_go <= 0.02)
        {
            time_to_go = 0.02;
        }

        // Calcolo nuovo dt per nodo MPC
        double dt_mpc_node = time_to_go / N_HORIZON;

        // Set timestep
        for (int i = 0; i < N_HORIZON; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &dt_mpc_node);
        }

        // ------------------------------------------------------------
        // A. VINCOLI INIZIALI (Feedback x0)
        // ------------------------------------------------------------

        double x0[NX];
        for (int i = 0; i < NJ; i++)
        {
            x0[i] = q_curr(i);
            x0[NJ + i] = dq_curr(i);
        }
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "x", x0); // Warm start x0

        // ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "u", ddq_opt); // Warm start u

        // ------------------------------------------------------------
        // C. HARD CONSTRAINTS TERMINALI
        // ------------------------------------------------------------
        // Impostiamo lbx e ubx all'ultimo nodo uguali al target
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "lbx", x_target);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, N_HORIZON, "ubx", x_target);

        // ------------------------------------------------------------
        // B. REFERENCE (STAGE COST)
        // ------------------------------------------------------------
        // Vogliamo minimizzare u durante il tragitto (Minimum Energy/Effort)
        // yref = [ u_ref (7)] = Zeros
        double yref_stage[NY] = {0};
        for (int i = 0; i < N_HORIZON; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_stage);
        }

        // // Terminal cost reference (anche se vincolato, aiuta la convergenza)
        // double yref_e[NYN];
        // for(int i=0; i<NX; i++) yref_e[i] = x_target[i];
        // ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N_HORIZON, "yref", yref_e);

        for (int i = 0; i <= N_HORIZON; i++)
        {
            if (time_to_go > 0.02)
            {
                double ti = t_curr + i * dt_mpc_node;
                auto s = planner.evaluate(ti);

                double x_guess[NX], u_guess[NU];
                for (int j = 0; j < NJ; j++)
                {
                    x_guess[j] = s.pos(j);
                    x_guess[NJ + j] = s.vel(j);
                    u_guess[j] = s.acc(j);
                }
                printf("MPC Node %d | t = %.3f s | q_des = [", i, ti);
                for (int j = 0; j < NJ; j++)
                {
                    printf("%.3f%s", s.pos(j), (j == NJ - 1 ? "]\n" : ", "));
                }

                ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_guess);
                if (i < N_HORIZON)
                {
                    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u_guess);
                }
            }
            else
            {
                int NODO = (int)((t_end - t_curr) / dt_mpc_node);
                // printf(">>> FINE TRAIETTORIA RAGGIUNTA. Blocco stato al nodo %d\n", NODO);
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, NODO, "lbx", x_target);
                ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, NODO, "ubx", x_target);

                // double check_lbx[NX]; // Buffer per leggere i Lower Bound
                // double check_ubx[NX]; // Buffer per leggere gli Upper Bound

                // // Leggiamo indietro dalla struttura di Acados
                // ocp_nlp_constraints_model_get(nlp_config, nlp_dims, nlp_in, NODO, "lbx", check_lbx);
                // ocp_nlp_constraints_model_get(nlp_config, nlp_dims, nlp_in, NODO, "ubx", check_ubx);
                // printf("Lbx Nodo %d: [", NODO);
                // for (int j = 0; j < NX; j++)
                // {
                //     printf("%.3f%s", check_lbx[j], (j == NX - 1 ? "]\n" : ", "));
                // }
            }
        }

        // ------------------------------------------------------------
        // D. SOLVE
        // ------------------------------------------------------------
        auto start_solve = std::chrono::high_resolution_clock::now();

        int status = frankino_tracking_mpc_acados_solve(capsule);

        auto end_solve = std::chrono::high_resolution_clock::now();
        double solve_time_ms = std::chrono::duration_cast<std::chrono::microseconds>(end_solve - start_solve).count() / 1000.0;

        // if (status != 0 && status != 2) // 0: Success, 2: Max Iter (spesso ok in RTI)
        // {
        //     std::cout << "[WARNING] Status: " << status << " | Time to go: " << time_to_go << std::endl;
        // }

        // ------------------------------------------------------------
        // E. RECUPERO ACCELERAZIONE (u0)
        // ------------------------------------------------------------
        double u0[NU];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0);
        for (int j = 0; j < NJ; j++)
            ddq_opt(j) = u0[j];
        // std::cout << "ddq_opt: " << ddq_opt.transpose() << std::endl;

        // ------------------------------------------------------------
        // F. INTEGRAZIONE FISICA (Computed Torque + Sim)
        // ------------------------------------------------------------
        robot_sim.set_q(q_curr);
        robot_sim.set_dq(dq_curr);
        MatrixXd M = robot_sim.get_M();
        MatrixXd C = robot_sim.get_C();
        VectorXd G = robot_sim.get_G();
        VectorXd bias = C * dq_curr + G;

        tau_cmd = M * ddq_opt + bias; // Inverse Dynamics

        // Forward Dynamics (simulazione risposta robot)
        VectorXd ddq_real = M.llt().solve(tau_cmd - bias);
        // Integrazione Eulero semplice
        q_curr += dq_curr * dt_sim + 0.5 * ddq_real * pow(dt_sim, 2);
        dq_curr += ddq_real * dt_sim;

        // // ------------------------------------------------------------
        // // MJ EXPECTED STATE AT t_curr
        // // ------------------------------------------------------------
        // auto s_mj = planner.evaluate(t_curr);

        // // Errori
        // VectorXd e_q = q_curr - s_mj.pos;
        // VectorXd e_dq = dq_curr - s_mj.vel;
        // VectorXd e_ddq = ddq_real - s_mj.acc;

        // double eq_norm = e_q.norm();
        // double edq_norm = e_dq.norm();
        // double eddq_norm = e_ddq.norm();
        // // ------------------------------------------------------------
        // // G. LOGGING (REAL vs MINJERK)
        // // ------------------------------------------------------------
        // std::cout << "t=" << std::fixed << std::setprecision(3) << t_curr
        //           << " | q_real=" << q_curr.transpose() << "\n"
        //           << " | q_mj=" << s_mj.pos.transpose() << "\n"
        //           << " | e_q_norm=" << eq_norm << "\n"
        //           << " | dq_real=" << dq_curr.transpose() << "\n"
        //           << " | dq_mj=" << s_mj.vel.transpose() << "\n"
        //           << " | e_dq_norm=" << edq_norm << "\n"
        //           << " | ddq_real=" << ddq_real.transpose() << "\n"
        //           << " | ddq_mj=" << s_mj.acc.transpose() << "\n"
        //           << " | e_ddq_norm=" << eddq_norm << "\n"
        //           << " | t_solve=" << solve_time_ms << " ms" << "\n"
        //           << std::endl;
        // ------------------------------------------------------------
        // G. LOGGING
        // ------------------------------------------------------------
        data_file << q_curr(0) << "," << q_curr(1) << "," << q_curr(2) << "," << q_curr(3) << "," << q_curr(4) << "," << q_curr(5) << "," << q_curr(6) << ","
                  << dq_curr(0) << "," << dq_curr(1) << "," << dq_curr(2) << "," << dq_curr(3) << "," << dq_curr(4) << "," << dq_curr(5) << "," << dq_curr(6) << ","
                  << ddq_real(0) << "," << ddq_real(1) << "," << ddq_real(2) << "," << ddq_real(3) << "," << ddq_real(4) << "," << ddq_real(5) << "," << ddq_real(6) << ","
                  << solve_time_ms << "\n";

        t_curr += dt_sim;
    }
    std::cout << ">>> Simulazione finita." << std::endl;

    // // ----------------------------------------------------------------
    // // 5. SIMULAZIONE SENZA MPC (Solo MinJerk)
    // // ----------------------------------------------------------------
    // t_curr = 0.0;
    // q_curr = q_start;
    // dq_curr = dq_start;

    // while (t_curr <= (t_end)) // Margine extra per vedere se stabilizza
    // {
    //     double ti = t_curr;
    //     auto s = planner.evaluate(ti);

    //     // ------------------------------------------------------------
    //     // F. INTEGRAZIONE FISICA (Computed Torque + Sim)
    //     // ------------------------------------------------------------
    //     robot_sim.set_q(q_curr);
    //     robot_sim.set_dq(dq_curr);
    //     MatrixXd M = robot_sim.get_M();
    //     MatrixXd C = robot_sim.get_C();
    //     VectorXd G = robot_sim.get_G();
    //     VectorXd bias = C * dq_curr + G;

    //     tau_cmd = M * s.acc + bias; // Inverse Dynamics

    //     // Forward Dynamics (simulazione risposta robot)
    //     VectorXd ddq_real = M.llt().solve(tau_cmd - bias);
    //     // Integrazione Eulero semplice
    //     q_curr += dq_curr * dt_sim + 0.5 * ddq_real * pow(dt_sim, 2);
    //     dq_curr += ddq_real * dt_sim;

    //     // ------------------------------------------------------------
    //     // G. LOGGING
    //     // ------------------------------------------------------------
    //     data_file << q_curr(0) << "," << q_curr(1) << "," << q_curr(2) << "," << q_curr(3) << "," << q_curr(4) << "," << q_curr(5) << "," << q_curr(6) << ","
    //               << dq_curr(0) << "," << dq_curr(1) << "," << dq_curr(2) << "," << dq_curr(3) << "," << dq_curr(4) << "," << dq_curr(5) << "," << dq_curr(6) << ","
    //               << ddq_real(0) << "," << ddq_real(1) << "," << ddq_real(2) << "," << ddq_real(3) << "," << ddq_real(4) << "," << ddq_real(5) << "," << ddq_real(6) << "\n";

    //     t_curr += dt_sim;
    // }
    // std::cout << ">>> Simulazione finita." << std::endl;

    data_file.close();
    // Cleanup
    frankino_tracking_mpc_acados_free(capsule);
    frankino_tracking_mpc_acados_free_capsule(capsule);
    return 0;
}