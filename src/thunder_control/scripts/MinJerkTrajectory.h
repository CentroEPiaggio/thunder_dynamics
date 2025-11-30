#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>
#include <stdexcept> 
using namespace Eigen;

class MinJerkTrajectory {
public:
    MinJerkTrajectory() : is_initialized(false) {}

    /**
     * Inizializza la traiettoria.
     * @param q0 Posizione iniziale (rad)
     * @param qf Posizione finale (rad)
     * @param v0 Velocità iniziale (rad/s)
     * @param vf Velocità finale (rad/s) 
     * @param t0 Tempo iniziale assoluto (s)
     * @param tf Tempo finale assoluto (s)
     */
    void init(const VectorXd& q0, const VectorXd& qf, 
              const VectorXd& v0, const VectorXd& vf, 
              double t0, double tf) {
        
        this->t0 = t0;
        this->tf = tf;
        this->T = tf - t0;
        this->q0 = q0;
        this->qf = qf;
        
        int n_joints = q0.size();
        
        // Assumiamo accelerazioni iniziali e finali nulle per semplicità
        VectorXd a0 = VectorXd::Zero(n_joints);
        VectorXd af = VectorXd::Zero(n_joints);

        // Calcolo coefficienti (Vettorizzato)
        c0 = q0;
        c1 = v0;
        c2 = a0 / 2.0;
        
        // Formule standard Minimum Jerk
        c3 = (20.0*(qf - q0) - (8.0*vf + 12.0*v0)*T - (3.0*a0 - af)*T*T) / (2.0*pow(T, 3));
        c4 = (30.0*(q0 - qf) + (14.0*vf + 16.0*v0)*T + (3.0*a0 - 2.0*af)*T*T) / (2.0*pow(T, 4));
        c5 = (12.0*(qf - q0) - 6.0*(vf + v0)*T - (a0 - af)*T*T) / (2.0*pow(T, 5));

        is_initialized = true;
    }

    // Struttura per ritornare lo stato desiderato
    struct State {
        VectorXd pos;
        VectorXd vel;
        VectorXd acc;
    };

    State evaluate(double t) {
        if (!is_initialized) {
            throw std::runtime_error("Traiettoria non inizializzata!");
        }

        // 1. Saturazione per il calcolo del polinomio base
        // Calcoliamo lo stato ESATTAMENTE a tf se t > tf
        double t_poly = std::max(t0, std::min(t, tf));
        double dt = t_poly - t0;

        double dt2 = dt * dt;
        double dt3 = dt2 * dt;
        double dt4 = dt3 * dt;
        double dt5 = dt4 * dt;

        State s;
        s.pos = c0 + c1*dt + c2*dt2 + c3*dt3 + c4*dt4 + c5*dt5;
        s.vel = c1 + 2*c2*dt + 3*c3*dt2 + 4*c4*dt3 + 5*c5*dt4;
        s.acc = 2*c2 + 6*c3*dt + 12*c4*dt2 + 20*c5*dt3;

        // 2. LOGICA FOLLOW THROUGH (Estrapolazione lineare)
        // Se il tempo richiesto è oltre la fine (t > tf), continuiamo a muoverci
        // con la velocità finale costante. Questo evita conflitti nel solver MPC
        // (che altrimenti vedrebbe posizione bloccata ma velocità alta).
        if (t > tf) {
            double dt_extra = t - tf;
            s.pos += s.vel * dt_extra; // Posizione avanza: q = q_f + v_f * delta_t
            s.acc.setZero();           // Accelerazione nulla dopo il lancio
        }

        return s;
    }
    
    double get_final_time() const { return tf; }

private:
    bool is_initialized;
    double t0, tf, T;
    VectorXd q0, qf;
    // Coefficienti polinomiali (ogni vettore ha dimensione n_joints)
    VectorXd c0, c1, c2, c3, c4, c5;
};