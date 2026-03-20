#ifndef KINEMATICS_UTILS_H
#define KINEMATICS_UTILS_H

#include <Eigen/Dense>
#include "thunder_frankino.h"

struct JointStateTarget
{
    Eigen::VectorXd q;
    Eigen::VectorXd dq;
    Eigen::VectorXd ddq;
};

class KinematicsUtils
{
public:
    /**
     * Risolve una CLIK di secondo ordine per trovare lo stato giunti target
     * corrispondente a una conf_cartesiana.
     */
    static JointStateTarget computeFullTarget(
        thunder_frankino &robot,
        const Eigen::Vector3d &p_target,
        const Eigen::Vector3d &v_target,
        const Eigen::VectorXd &q_start,
        const Eigen::VectorXd &dq_start,
        const Eigen::VectorXd &ddq_start,
        double timeout = 1.5)
    {
        int nj = robot.get_numJoints();
        JointStateTarget target;

        // Inizializziamo la "simulazione interna" con lo stato REALE attuale
        target.q = q_start;
        target.dq = dq_start;
        target.ddq = ddq_start;

        double dt = 0.001;
        double Kp = 200.0;
        double Kv = 30.0;

        for (double t = 0; t < timeout; t += dt)
        {
            robot.set_q(target.q);
            robot.set_dq(target.dq);

            Eigen::Vector3d p_curr = robot.get_T_0_ee().block<3, 1>(0, 3);
            Eigen::Matrix<double, 3, 7> J_lin = robot.get_J_ee().block<3, 7>(0, 0);

            // Errore di posizione e velocità
            Eigen::Vector3d error_p = p_target - p_curr;
            Eigen::Vector3d error_v = v_target - (J_lin * target.dq);

            Eigen::MatrixXd J_pinv = J_lin.completeOrthogonalDecomposition().pseudoInverse();

            // Calcolo ddq: se dq_start era alta, Kv smorzerà il moto verso v_target
            target.ddq = J_pinv * (Kp * error_p + Kv * error_v);

            // Integrazione
            target.dq += target.ddq * dt;
            target.q += target.dq * dt + 0.5 * target.ddq * dt * dt;

            if (error_p.norm() < 1e-6 && error_v.norm() < 1e-5)
                break;
        }

        return target;
    }
};

#endif