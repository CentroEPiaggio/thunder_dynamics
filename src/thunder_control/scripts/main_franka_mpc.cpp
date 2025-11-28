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
    data_file << "t,ref_q_elbow,ref_v_elbow,q_elbow,dq_elbow,tau_elbow\n";

    VectorXd q_curr = VectorXd::Zero(NJ);
    VectorXd dq_curr = VectorXd::Zero(NJ);
    VectorXd tau_cmd = VectorXd::Zero(NJ);

    // Variabile di supporto per setArguments (accelerazioni nulle per update M,C,G base)
    VectorXd zeros = VectorXd::Zero(NJ);

    // Posizione iniziale articolazioni
    q_curr << 0.0, -0.78, 0.0, -2.35, 0.0, 1.57, 0.78;

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
    double t_wait = 0.5;
    double t_throw_dur = 0.8;

    VectorXd q_start = q_curr;
    VectorXd q_end = q_curr;
    q_end(3) += 1.5; // Estensione gomito

    VectorXd v_start = VectorXd::Zero(NJ);
    VectorXd v_throw = VectorXd::Zero(NJ);
    v_throw(3) = 2.0; // Velocità di lancio target (rad/s)

    planner.init(q_start, q_end, v_start, v_throw, t_wait, t_wait + t_throw_dur);

    // Inizializziamo l'ostacolo lontano. Lo sposteremo in loop.
    double menu_obs_x = 10.0;
    double menu_obs_y = 10.0;
    double menu_obs_z = 10.0;

    // Flag per simulare attivazione ostacolo dinamico
    bool obstacle_active = false;

    std::cout << ">>> Loop avviato. Dati salvati in simulation_data.csv" << std::endl;

    // ----------------------------------------------------------------
    // 4. CONTROL LOOP
    // ----------------------------------------------------------------
    double dt_control = 0.001;            // 1 ms
    double dt_mpc_step = 1.0 / N_HORIZON; // 0.05 s

    bool keep_running = true;
    double t_sim = 0.0;

    // Variabili per debug feedforward
    VectorXd debug_tau_ff = VectorXd::Zero(NJ);

    while (keep_running)
    {
        // A. GESTIONE TEMPO
        t_sim += dt_control;
        double t_abs = t_sim;

        if (t_abs > (t_wait + t_throw_dur + 0.0))
            keep_running = false;

        if (obstacle_active)
        {
            // ESEMPIO: L'ostacolo si muove sinusoidalmente davanti al robot_sim
            // In un'app reale, leggeresti qui i valori dalla GUI / Tastiera
            menu_obs_x = 0.5 + 0.1 * sin(t_abs * 2.0); // Oscilla su X
            menu_obs_y = 0.0;                          // Al centro su Y
            menu_obs_z = 0.5;                          // Altezza gomito/EE
        }
        else
        {
            // Ostacolo disattivato (lo spostiamo all'infinito)
            menu_obs_x = 10.0;
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
        VectorXd friction = 0.01 * dq_curr;

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
            // double t_pred = t_abs + i * dt_mpc_step;
            // MinJerkTrajectory::State des = planner.evaluate(t_pred);
            // frankino_throw_catch_acados_update_params(capsule, i, p_current, NP);

            double t_pred = t_abs + i * dt_mpc_step;
            MinJerkTrajectory::State des = planner.evaluate(t_pred);
            
            // --- CALCOLO INVERSE DYNAMICS FEEDFORWARD ---
            // Aggiorniamo il robot "modello" con lo stato desiderato
            robot_model.setArguments(des.pos, des.vel, zeros, zeros);
            
            // Tau_FF = M(q_d)*acc_d + C(q_d, dq_d)*vel_d + g(q_d)
            VectorXd tau_ff = robot_model.get_M() * des.acc + 
                              robot_model.get_C() * des.vel + 
                              robot_model.get_G();

            // Salviamo per debug il valore corrente (i=0)
            if (i==0) debug_tau_ff = tau_ff;

            // Update ostacoli
            frankino_throw_catch_acados_update_params(capsule, i, p_current, NP);

            if (i < N_HORIZON)
            {
                double yref[NY];
                // Stati
                for (int k = 0; k < NJ; k++) yref[k] = des.pos(k);
                for (int k = 0; k < NJ; k++) yref[NJ + k] = des.vel(k);
                
                // Input (Coppia)
                for (int k = 0; k < NJ; k++) yref[2 * NJ + k] = tau_ff(k); 
                
                ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
            }
            else
            {
                // Terminal cost (solo stati)
                double yref_e[NYN];
                for (int k = 0; k < NJ; k++) yref_e[k] = des.pos(k);
                for (int k = 0; k < NJ; k++) yref_e[NJ + k] = des.vel(k);
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
                  << menu_obs_x << "," << menu_obs_y << "," << menu_obs_z << "\n";

        // Stampa ogni 100ms
        int step_count = (int)(t_sim / dt_control);
        if (step_count % 100 == 0)
        {
            std::cout << "T: " << t_abs
                      << " | Ref_V: " << current_ref.vel(3)
                      << " | Real_V: " << dq_curr(3)
                      << " | Tau: " << tau_cmd(3)
                      << " | Ref_P: " << current_ref.pos(3)
                      << " | Real_P: " << q_curr(3)
                      << std::endl;
        }
    }

    // CLEANUP
    std::cout << ">>> Fine simulazione." << std::endl;
    data_file.close();
    frankino_throw_catch_acados_free(capsule);
    frankino_throw_catch_acados_free_capsule(capsule);

    return 0;
}