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

// --- DEFINIZIONI DIMENSIONI (Devono coincidere con il Python generator) ---
#define N_HORIZON 20
#define NX 14  // [q, dq]
#define NU 7   // [ddq] <-- L'input MPC è l'accelerazione
#define NY 21  // [q, dq, u] -> 14 + 7
#define NYN 14 // [q, dq] nel cost terminale
#define NP 3   // [obs_x, obs_y, obs_z]

const std::string conf_file = "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles/frankino_conf.yaml";

int main()
{
    // ----------------------------------------------------------------
    // 1. SETUP ROBOT
    // ----------------------------------------------------------------
    thunder_frankino robot_sim;
    robot_sim.load_conf(conf_file);
    int NJ = robot_sim.get_numJoints();

    // Logger: Creiamo un CSV con i dati che ci servono
    std::ofstream data_file("simulation_data.csv");
    // data_file << "t,ref_q,ref_dq,ref_ddq,q_real,dq_real,ddq_opt,tau_cmd,obs_dist,obs_x,obs_y,obs_z\n";
    data_file << "q1,q2,q3,q4,q5,q6,q7,"
                 "dq1,dq2,dq3,dq4,dq5,dq6,dq7,"
                 "ddq1,ddq2,ddq3,ddq4,ddq5,ddq6,ddq7,t_solve\n";

    // Vettori di stato
    VectorXd q_curr = VectorXd::Zero(NJ);
    VectorXd dq_curr = VectorXd::Zero(NJ);
    VectorXd ddq_opt = VectorXd::Zero(NJ); // Output accelerazione MPC
    VectorXd tau_cmd = VectorXd::Zero(NJ);
    double eps = 1e-6;
    Eigen::VectorXd tol = VectorXd::Constant(NX, eps);

    // Posizione Iniziale
    q_curr << -1.25962, -0.663669, -0.692637, -2.17138, -0.264125, 1.50759, 0.0630972;

    // Init dinamica
    robot_sim.set_q(q_curr);
    robot_sim.set_dq(dq_curr);
    tau_cmd = robot_sim.get_G(); // Compensa gravità all'inizio
    std::cout << "    Stato iniziale q: " << q_curr.transpose() << std::endl;
    std::cout << "    Stato iniziale dq: " << dq_curr.transpose() << std::endl;

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
    // 3. PIANIFICAZIONE (MinJerkTrajectory)
    // ----------------------------------------------------------------
    MinJerkTrajectory planner;
    double t_start = 0.0;                // Tempo iniziale della traiettoria
    double t_duration = 5.0;             // Durata della traiettoria (5 secondi)
    double t_end = t_start + t_duration; // Tempo finale della traiettoria

    VectorXd q_start = q_curr;
    VectorXd q_final = q_curr;

    // // Movimento: Estendiamo il braccio e ruotiamo la base
    // q_final(0) += 1.0; // Base ruota
    // q_final(3) += 1.0; // Gomito si alza

    q_final << -0.157, -0.504, -0.623, -2.258, -0.332, 1.637, -1.728; // Posizione finale target

    // Velocità zero agli estremi
    VectorXd dq_start = VectorXd::Zero(NJ);
    VectorXd dq_final = VectorXd::Zero(NJ);

    // dq_final(0) += 3.0; // Velocità finale del polso in estensione
    // dq_final(3) += 2.0; // Velocità finale del gomito in estensione

    dq_final << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // Velocità finali

    planner.init(q_start, q_final, dq_start, dq_final, t_start, t_end);

    // Posizione iniziale ostacolo (Lontano)
    double obs_p[3] = {0.11, -0.35, 10.53};

    // ----------------------------------------------------------------
    // 4. WARM-UP SOLVER
    // ----------------------------------------------------------------
    std::cout << ">>> Warm-up..." << std::endl;
    for (int k = 0; k < 10; k++)
    {
        // Set stato iniziale
        double x0[NX];
        for (int i = 0; i < NJ; i++)
        {
            x0[i] = q_curr(i);
            x0[NJ + i] = dq_curr(i);
        }
        // Imposta i bound sullo stato iniziale
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "x", x0);

        // Set Reference statico per il warm-up
        for (int i = 0; i <= N_HORIZON; i++)
        {
            frankino_tracking_mpc_acados_update_params(capsule, i, obs_p, NP);

            // Reference Dummy
            if (i < N_HORIZON)
            {
                double yref[NY] = {0};
                for (int j = 0; j < NJ; j++)
                    yref[j] = q_curr(j); // q_ref
                for (int j = 0; j < NJ; j++)
                    yref[NJ + j] = dq_start(j);
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
            }
            else
            {
                double yref_e[NYN] = {0};
                for (int j = 0; j < NJ; j++)
                    yref_e[j] = q_curr(j);
                for (int j = 0; j < NJ; j++)
                    yref_e[NJ + j] = dq_start(j);
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_e);
            }
        }

        int warmup_status = frankino_tracking_mpc_acados_solve(capsule);
        // if (warmup_status != 0 && warmup_status != 2)
        // {
        //     std::cout << "    Warm-up solve status: " << warmup_status << " (iter " << k << ")" << std::endl;
        // }
    }
    std::cout << ">>> Warm-up completato." << std::endl;

    // ----------------------------------------------------------------
    // 5. LOOP DI CONTROLLO
    // ----------------------------------------------------------------
    double dt_sim = 0.001; // 1 kHz loop fisico
    double t_curr = 0.0;

    std::cout << ">>> Avvio simulazione..." << std::endl;

    while (t_curr < (t_end + 0.1)) // +0.1s extra
    {
        // --- A. GESTIONE OSTACOLO DINAMICO ---
        // Facciamo apparire l'ostacolo tra 1.0s e 2.0s
        // Lo posizioniamo "in mezzo" al percorso previsto
        if (t_curr > 2.0)
        {
            obs_p[0] = 0.11;  // X davanti al robot
            obs_p[1] = -0.35; // Y leggermente a lato
            obs_p[2] = 0.53;  // Z altezza critica
        }
        else
        {
            obs_p[0] = 10.0; // Via libera
        }

        // // ------------------------------------------------------------
        // // [SHRINKING HORIZON LOGIC]
        // // ------------------------------------------------------------

        // // 1. Calcolo tempo rimanente
        // double time_to_go = t_end - t_curr;

        // // 2. Protezione: non scendere mai sotto un dt minimo (es. 1ms)
        // // altrimenti il solver esplode numericamente quando t_curr ~ t_end
        // if (time_to_go < 0.02)
        // {
        //     time_to_go = 0.02;
        // }

        // // 3. Calcolo nuovo dt per nodo MPC
        // double dt_mpc_node = time_to_go / N_HORIZON;

        // // 4. Aggiorna il dt ("Ts") dentro Acados per ogni stage
        // for (int i = 0; i < N_HORIZON; i++)
        // {
        //     ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &dt_mpc_node);
        // }

        // // ------------------------------------------------------------

        // --- B. SETUP MPC (Receding Horizon) ---
        // L'MPC predice il futuro con passi di 0.05s
        double dt_mpc_node = 0.05;

        // 1. Feedback Stato Corrente (x0)
        double x0[NX];
        for (int i = 0; i < NJ; i++)
        {
            x0[i] = q_curr(i);
            x0[NJ + i] = dq_curr(i);
        }
        // Imposta i bound sullo stato iniziale
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, 0, "x", x0);

        // 2. Aggiornamento Traiettoria Desiderata nell'orizzonte
        for (int i = 0; i <= N_HORIZON; i++)
        {
            // Il tempo predetto ora usa dt_mpc_node calcolato dinamicamente
            double t_pred = t_curr + i * dt_mpc_node;

            // Usiamo la classe MinJerk
            MinJerkTrajectory::State des = planner.evaluate(t_pred);

            // Passiamo param ostacolo
            frankino_tracking_mpc_acados_update_params(capsule, i, obs_p, NP);
            double yref[NY] = {0};

            if (i < N_HORIZON)
            {

                // Copia posizione e velocità desiderate
                for (int j = 0; j < NJ; j++)
                    yref[j] = des.pos(j);
                for (int j = 0; j < NJ; j++)
                    yref[NJ + j] = des.vel(j);
                for (int j = 0; j < NJ; j++)
                    yref[2 * NJ + j] = des.acc(j); // Riferimento accelerazione a 0.0

                // Gli ultimi 7 valori (indici 14-20) sono riferiti a 'u' (accelerazione).
                // Lasciandoli a 0.0, stiamo dicendo: "Cerca di tenere l'accelerazione bassa".
                // Questo rende il movimento smooth.

                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
            }
            else
            {
                // Terminal cost yref_e = [q, dq] (dimensione NYN = 14)
                double yref_e[NYN] = {0};
                for (int j = 0; j < NJ; j++)
                    yref_e[j] = des.pos(j);
                for (int j = 0; j < NJ; j++)
                    yref_e[NJ + j] = des.vel(j);
                // Per il costo terminale usa "yref_e" non "yref"
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_e);
            }
        }

        auto start_solve = std::chrono::high_resolution_clock::now();
        // --- C. SOLVE MPC ---
        int status = frankino_tracking_mpc_acados_solve(capsule);

        auto end_solve = std::chrono::high_resolution_clock::now();
        auto solve_duration = std::chrono::duration_cast<std::chrono::microseconds>(end_solve - start_solve);

        // Salva tempo di solve per analisi
        double solve_time_us = solve_duration.count();
        double solve_time_ms = solve_time_us / 1000.0;
        // if (status != 0 && status != 2)
        // {
        //     // Status 2 è max iter raggiunto (comune in real-time), va bene.
        //     std::cout << "    MPC solve status: " << status << " at t=" << t_curr << std::endl;
        // }

        // --- D. RECUPERO ACCELERAZIONE OTTIMA (u0) ---
        double u0[NU];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0);
        for (int j = 0; j < NJ; j++)
            ddq_opt(j) = u0[j];

        // --- E. COMPUTED TORQUE (Cascade Control) ---
        // Usiamo l'accelerazione dell'MPC per calcolare la coppia fisica
        robot_sim.set_q(q_curr);
        robot_sim.set_dq(dq_curr);
        MatrixXd M = robot_sim.get_M();
        MatrixXd C = robot_sim.get_C();
        VectorXd G = robot_sim.get_G();
        VectorXd bias = C * dq_curr + G;

        // Legge di controllo: Tau = M * ddq_opt + Bias
        tau_cmd = M * ddq_opt + bias;

        // --- F. SIMULAZIONE (Integrator) ---
        // Simulo la risposta del robot reale
        // ddq_real = inv(M) * (tau - bias) -> In teoria tornerà ddq_opt se il modello è perfetto
        VectorXd ddq_real = M.llt().solve(tau_cmd - bias);

        dq_curr += ddq_real * dt_sim;
        q_curr += dq_curr * dt_sim + 0.5 * ddq_real * pow(dt_sim, 2);

        // --- G. LOGGING ---
        MinJerkTrajectory::State ref_now = planner.evaluate(t_curr);

        // Distanza dall'ostacolo (per logging)
        Vector3d ee_pos = robot_sim.get_T_0_8().block<3, 1>(0, 3);
        double dist_obs = std::sqrt(std::pow(ee_pos(0) - obs_p[0], 2) +
                                    std::pow(ee_pos(1) - obs_p[1], 2) +
                                    std::pow(ee_pos(2) - obs_p[2], 2));

        // Loggiamo i dati relativi al Giunto 3 (Gomito) o 0 (Base) che si muovono molto
        int log_j = 3; // Logghiamo il giunto gomito

        // data_file << t_curr << ","
        //           << ref_now.pos(log_j) << "," << ref_now.vel(log_j) << "," << ref_now.acc(log_j) << ","
        //           << q_curr(log_j) << "," << dq_curr(log_j) << "," << ddq_opt(log_j) << ","
        //           << tau_cmd(log_j) << "," << dist_obs << ","
        //           << obs_p[0] << "," << obs_p[1] << "," << obs_p[2] << "\n";
        data_file << q_curr(0) << "," << q_curr(1) << "," << q_curr(2) << "," << q_curr(3) << "," << q_curr(4) << "," << q_curr(5) << "," << q_curr(6) << ","
                  << dq_curr(0) << "," << dq_curr(1) << "," << dq_curr(2) << "," << dq_curr(3) << "," << dq_curr(4) << "," << dq_curr(5) << "," << dq_curr(6) << ","
                  << ddq_opt(0) << "," << ddq_opt(1) << "," << ddq_opt(2) << "," << ddq_opt(3) << "," << ddq_opt(4) << "," << ddq_opt(5) << "," << ddq_opt(6) << "," <<  solve_time_ms << "\n";
        t_curr += dt_sim;
    }

    std::cout << ">>> Simulazione finita. Dati salvati in simulation_data.csv" << std::endl;
    data_file.close();
    std::cout << ">>> Pulizia Acados..." << std::endl;

    frankino_tracking_mpc_acados_free(capsule);
    frankino_tracking_mpc_acados_free_capsule(capsule);
    return 0;
}