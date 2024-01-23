#ifndef THUNDERPANDA_H
#define THUNDERPANDA_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>

namespace thunder_ns{

    /* Class thunderPanda 
    Useful functions for adaptive control with Slotine-Li regressor
    Usable only for generated code from casadi library! */
    class thunderPanda{

        private:

            /* Number of joints */
            int num_joints;
            /* Joints' variables */
            Eigen::VectorXd q, dq, dqr, ddqr, param;
            /* Output of generated code */
            Eigen::MatrixXd kin_gen, jac_gen, dotJac_gen, pinvJac_gen, pinvJacPos_gen, dotPinvJac_gen, dotPinvJacPos_gen,
                reg_gen, mass_gen, coriolis_gen, gravity_gen;
            /* Compute generated code */
            void computeKin_gen(), computeJac_gen(), computeDotJac_gen(), computePinvJac_gen(), computeDotPinvJac_gen(), 
                computeReg_gen(), computeMass_gen(), computeCoriolis_gen(), computeGravity_gen(), computePinvJacPos_gen(), computeDotPinvJacPos_gen();

        public:
            
            /* Empty constructor */
            thunderPanda();
            /* Constructor to init variables*/
            thunderPanda(const int);
            /* Destructor*/
            ~thunderPanda(){std::cout<<"thunderPanda destruct\n";};
            
            /* Resize variables */
            void resizeVariables();
            /* Init variables */
            void init(const int);
            /* Set q, dq, dqr, ddqr, to compute Regressor */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&,const Eigen::VectorXd&);
            
            /* Set q, dq, param, to compute M C G */
            void setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& param_);
            /* Set q, dq, to compute pseudo-inverse of jacobian and derivate of pseudo-inverse jacobian */
            void setArguments(const Eigen::VectorXd&,const Eigen::VectorXd&);
            /* Set q to compute forward kinematic */
            void setArguments(const Eigen::VectorXd&);
            
            void setInertialParam(const Eigen::VectorXd& param_);
            /* Get regressor matrix */
            Eigen::MatrixXd getReg_gen(){return reg_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getMass_gen(){return mass_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getCoriolis_gen(){return coriolis_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getGravity_gen(){return gravity_gen;};
            /* Get jacobian matrix */
            Eigen::MatrixXd getJac_gen(){return jac_gen;};
            /* Get derivative of jacobian matrix */
            Eigen::MatrixXd getDotJac_gen(){return dotJac_gen;};
            /* Get pseudo-inverse jacobian matrix */
            Eigen::MatrixXd getPinvJac_gen(){return pinvJac_gen;};
            /* Get derivative of pseudo-inverse jacobian matrix only position */
            Eigen::MatrixXd getPinvJacPos_gen(){return pinvJacPos_gen;};
            /* Get derivative of pseudo-inverse jacobian matrix */
            Eigen::MatrixXd getDotPinvJac_gen(){return dotPinvJac_gen;};
            /* Get derivative of pseudo-inverse jacobian matrix only position */
            Eigen::MatrixXd getDotPinvJacPos_gen(){return dotPinvJacPos_gen;};
            /* Get regressor matrix */
            Eigen::MatrixXd getKin_gen(){return kin_gen;};
    };
}
#endif