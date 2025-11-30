#include <iostream>
#include <vector>
#include <chrono>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <thread>
#include <fstream>

// --- INCLUSIONI ACADOS ---
#include "acados_solver_frankino_throw_catch.h"
#include "acados_c/ocp_nlp_interface.h"

// --- UTILS ---
#include "MinJerkTrajectory.h"
#include "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles/thunder_frankino.h"

using namespace Eigen;
using namespace std::chrono;

#define N_HORIZON 20
#define NX 14
#define NU 7
#define NY 21
#define NYN 14
#define NP 3

const std::string conf_file = "/home/thunder_dev/thunder_dynamics/src/thunder_control/frankino_generatedFiles/frankino_conf.yaml";

int main()
{
    // ----------------------------------------------------------------
    // 1. INIZIALIZZAZIONE ROBOT
    // ----------------------------------------------------------------
    // Costruzione corretta: costruttore vuoto + load_conf
    thunder_frankino robot_sim;
    robot_sim.load_conf(conf_file);

    thunder_frankino robot_model;
    robot_model.load_conf(conf_file);

    int NJ = robot_sim.get_numJoints(); // Meglio usare il getter

    // SETUP LOGGING
    std::ofstream data_file("simulation_data.csv");
    data_file << "t,ref_q_elbow,ref_v_elbow,q_elbow,dq_elbow,tau_elbow, obstacle_x, obstacle_y, obstacle_z, time_left, current_dt_node \n";

    VectorXd q_curr = VectorXd::Zero(NJ);
    VectorXd dq_curr = VectorXd::Zero(NJ);
    VectorXd tau_cmd = VectorXd::Zero(NJ);

    // Variabile di supporto per setArguments (accelerazioni nulle per update M,C,G base)
    VectorXd zeros = VectorXd::Zero(NJ);

    // Posizione iniziale articolazioni
    q_curr << 0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78;

    robot_sim.setArguments(q_curr, dq_curr, zeros, zeros);
    MatrixXd M = robot_sim.get_M();
    MatrixXd C = robot_sim.get_C();
    VectorXd G = robot_sim.get_G();

    tau_cmd = G; // Manteniamo la posizione iniziale con la gravità compensata

    // ----------------------------------------------------------------
    // 2. INIZIALIZZAZIONE SOLVER ACADOS
    // ----------------------------------------------------------------
    std::cout << ">>> Inizializzazione Solver Acados..." << std::endl;

    frankino_throw_catch_solver_capsule *capsule = frankino_throw_catch_acados_create_capsule();
    int status = frankino_throw_catch_acados_create(capsule);
    if (status)
    {
        std::cerr << "Errore creazione solver: " << status << std::endl;
        return 1;
    }

    ocp_nlp_config *nlp_config = frankino_throw_catch_acados_get_nlp_config(capsule);
    ocp_nlp_dims *nlp_dims = frankino_throw_catch_acados_get_nlp_dims(capsule);
    ocp_nlp_in *nlp_in = frankino_throw_catch_acados_get_nlp_in(capsule);
    ocp_nlp_out *nlp_out = frankino_throw_catch_acados_get_nlp_out(capsule);

    // ----------------------------------------------------------------
    // 3. PIANIFICAZIONE
    // ----------------------------------------------------------------
    MinJerkTrajectory planner;
    double t_wait = 0.2;
    double t_throw_dur = 0.8;
    double t_total_mission = t_wait + t_throw_dur; // Tempo totale della missione

    VectorXd q_start = q_curr;
    VectorXd q_end = q_curr;
    q_end(3) += 1.5; // Estensione gomito

    VectorXd v_start = VectorXd::Zero(NJ);
    VectorXd v_throw = VectorXd::Zero(NJ);
    v_throw(3) = 2.0; // Velocità di lancio target (rad/s)

    planner.init(q_start, q_end, v_start, v_throw, t_wait, t_total_mission);

    // Inizializziamo l'ostacolo lontano. Lo sposteremo in loop.
    double menu_obs_x = 10.0;
    double menu_obs_y = 10.0;
    double menu_obs_z = 10.0;

    // Flag per simulare attivazione ostacolo dinamico
    bool obstacle_active = false;

    std::cout << ">>> Loop avviato. Dati salvati in simulation_data.csv" << std::endl;

    // ----------------------------------------------------------------
    // 3b. WARM-UP SOLVER ACADOS
    // ----------------------------------------------------------------
    std::cout << ">>> Eseguo Warmup Solver (20 iterazioni)..." << std::endl;

    for (int k = 0; k < 20; k++)
    {
        // --- 1. SETTARE X0 (STATO INIZIALE) ---
        // Senza questo, il solver pensa di partire da 0.
        double x0[NX];
        for (int i = 0; i < NJ; i++) x0[i] = q_curr(i);
        for (int i = 0; i < NJ; i++) x0[NJ + i] = dq_curr(i); // che è zero

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);
        // --------------------------------------------------

        double p_warm[NP] = {menu_obs_x, menu_obs_y, menu_obs_z};

        for (int i = 0; i <= N_HORIZON; i++)
        {
            frankino_throw_catch_acados_update_params(capsule, i, p_warm, NP);

            if (i < N_HORIZON)
            {
                double yref[NY];
                for (int j = 0; j < NJ; j++) yref[j] = q_curr(j);
                for (int j = 0; j < NJ; j++) yref[NJ + j] = 0.0;
                for (int j = 0; j < NJ; j++) yref[2 * NJ + j] = G(j); 

                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
            }
            else
            {
                double yref_e[NYN];
                for (int j = 0; j < NJ; j++) yref_e[j] = q_curr(j);
                for (int j = 0; j < NJ; j++) yref_e[NJ + j] = 0.0;
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_e);
            }
        }

        frankino_throw_catch_acados_solve(capsule);
    }

    double u0_warm[NU];
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0_warm);
    for (int i = 0; i < NJ; i++) tau_cmd(i) = u0_warm[i];

    std::cout << ">>> Warmup Finito. Tau Start Gomito: " << tau_cmd(3) << " Nm" << std::endl;


    // ----------------------------------------------------------------
    // 4. CONTROL LOOP
    // ----------------------------------------------------------------
    double dt_control = 0.001; // 1 ms

    bool keep_running = true;
    double t_sim = 0.0;

    // Variabili per debug feedforward
    VectorXd debug_tau_ff = VectorXd::Zero(NJ);

    while (keep_running)
    {
        // A. GESTIONE TEMPO
        t_sim += dt_control;
        double t_abs = t_sim;

        if (t_abs > (t_total_mission + 0.1)) // Aggiungiamo un piccolo buffer
            keep_running = false;

        // --- SHRINKING HORIZON LOGIC ---
        // 1. Quanto manca alla fine?
        double time_left = t_total_mission - t_abs;

        // // 2. Clamp: Potrebbe essere utile non scendere mai sotto un orizzonte minimo (es. 0.1s)
        // // Se scendiamo troppo, il solver diventa instabile (divisioni per dt piccolissimi).
        // // Quando mancano meno di 0.1s, l'orizzonte rimane fisso a 0.1s e guarda "appena dopo" la fine.
        // double min_horizon_duration = 0.1;
        // if (time_left < min_horizon_duration)
        // {
        //     time_left = min_horizon_duration;
        // }

        // 3. Calcolo del nuovo DT per ogni Shooting Node
        double current_dt_node = time_left / (double)N_HORIZON;
        // --------------------------------

        if (obstacle_active)
        {
            // ESEMPIO: L'ostacolo si muove sinusoidalmente davanti al robot_sim
            menu_obs_x = 0.5 + 0.1 * sin(t_abs * 2.0); // Oscilla su X
            menu_obs_y = 0.0;                          // Al centro su Y
            menu_obs_z = 0.5;                          // Altezza gomito/EE
        }
        else
        {
            // Ostacolo disattivato (lo spostiamo lontano)
            menu_obs_x = 10.0;
            menu_obs_y = 10.0;
            menu_obs_z = 10.0;
        }

        double p_current[NP] = {menu_obs_x, menu_obs_y, menu_obs_z};

        // --------------------------------------------------------
        // B. INTEGRATORE FISICO REALE (Forward Dynamics)
        // --------------------------------------------------------

        // 1. Aggiorna stato interno del modello robot_sim
        robot_sim.setArguments(q_curr, dq_curr, zeros, zeros);

        // 2. Recupera matrici
        MatrixXd M = robot_sim.get_M(); // Mass Matrix
        MatrixXd C = robot_sim.get_C(); // Coriolis Matrix
        VectorXd G = robot_sim.get_G(); // Gravity Vector

        // 3. Calcolo Bias
        VectorXd bias = C * dq_curr + G;

        // 4. Attrito viscoso (per stabilizzare la simulazione) tempo fa per le tavole di robotica  lo si aggiungeva al modello
        VectorXd friction = 0.0 * dq_curr;

        // 5. Calcolo Accelerazione: M * ddq = tau - bias - friction
        VectorXd tau_net = tau_cmd - bias - friction;
        VectorXd ddq_calc = M.llt().solve(tau_net); // LLT per simmetria e definitezza positiva Lower-Upper Cholesky Decomposition

        // 6. Integrazione (Eulero semi-implicito)
        dq_curr += ddq_calc * dt_control;
        q_curr += dq_curr * dt_control + 0.5 * ddq_calc * dt_control * dt_control;

        // --------------------------------------------------------

        // C. SET X0 (Feedback per MPC)
        double x0[NX];
        for (int i = 0; i < NJ; i++)
            x0[i] = q_curr(i);
        for (int i = 0; i < NJ; i++)
            x0[NJ + i] = dq_curr(i);

        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", x0);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", x0);

        // D. UPDATE REFERENCE (MPC)
        for (int i = 0; i <= N_HORIZON; i++)
        {
            // --- UPDATE TIME STEP (Shrinking Horizon) ---
            // Acados richiede di settare "Ts" per i nodi da 0 a N-1
            if (i < N_HORIZON)
            {
                ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &current_dt_node);
            }

            // double t_pred = t_abs + i * dt_mpc_step;
            // MinJerkTrajectory::State des = planner.evaluate(t_pred);
            // frankino_throw_catch_acados_update_params(capsule, i, p_current, NP);

            double t_pred = t_abs + i * current_dt_node;
            MinJerkTrajectory::State des = planner.evaluate(t_pred);

            // --- CALCOLO INVERSE DYNAMICS FEEDFORWARD ---
            // Aggiorniamo il robot "modello" con lo stato desiderato
            robot_model.setArguments(des.pos, des.vel, zeros, zeros);

            // Tau_FF = M(q_d)*acc_d + C(q_d, dq_d)*vel_d + g(q_d)
            VectorXd tau_ff = robot_model.get_M() * des.acc +
                              robot_model.get_C() * des.vel +
                              robot_model.get_G();

            // Salviamo per debug il valore corrente (i=0)
            if (i == 0)
                debug_tau_ff = tau_ff;

            // Update ostacoli
            frankino_throw_catch_acados_update_params(capsule, i, p_current, NP);

            if (i < N_HORIZON)
            {
                double yref[NY];
                // Stati
                for (int k = 0; k < NJ; k++)
                    yref[k] = des.pos(k);
                for (int k = 0; k < NJ; k++)
                    yref[NJ + k] = des.vel(k);

                // Input (Coppia)
                for (int k = 0; k < NJ; k++)
                    yref[2 * NJ + k] = tau_ff(k);

                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
            }
            else
            {
                // Terminal cost (solo stati)
                double yref_e[NYN];
                for (int k = 0; k < NJ; k++)
                    yref_e[k] = des.pos(k);
                for (int k = 0; k < NJ; k++)
                    yref_e[NJ + k] = des.vel(k);
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref_e);
            }
        }

        // E. SOLVE
        status = frankino_throw_catch_acados_solve(capsule);
        if (status != 0 && status != 2)
        {
            // Ignoriamo status 2 (max iter) che è comune in real-time
            std::cerr << "Solver error: " << status << std::endl;
        }

        // F. GET CONTROL
        double u0[NU];
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", u0);
        for (int i = 0; i < NJ; i++)
            tau_cmd(i) = u0[i];

        // LOGGING
        MinJerkTrajectory::State current_ref = planner.evaluate(t_abs);

        data_file << t_abs << ","
                  << current_ref.pos(3) << "," << current_ref.vel(3) << ","
                  << q_curr(3) << "," << dq_curr(3) << "," << tau_cmd(3) << ","
                  << menu_obs_x << "," << menu_obs_y << "," << menu_obs_z << ","
                  << time_left << "," << current_dt_node << "\n";

        // // Stampa ogni 100ms
        // int step_count = (int)(t_sim / dt_control);
        // if (step_count % 100 == 0)
        // {
        //     std::cout << "T: " << t_abs
        //               << " | Ref_V: " << current_ref.vel(3)
        //               << " | Real_V: " << dq_curr(3)
        //               << " | Tau: " << tau_cmd(3)
        //               << " | Ref_P: " << current_ref.pos(3)
        //               << " | Real_P: " << q_curr(3)
        //               << " | Time Left: " << time_left
        //               << " | Current Dt Node: " << current_dt_node
        //               << std::endl;
        // }
    }

    // CLEANUP
    std::cout << ">>> Fine simulazione." << std::endl;
    data_file.close();
    frankino_throw_catch_acados_free(capsule);
    frankino_throw_catch_acados_free_capsule(capsule);

    return 0;
}