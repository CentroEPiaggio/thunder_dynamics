#ifndef ROBKIN_H
#define ROBKIN_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "RobKinBasic.h"
#include "FrameOffset.h"

#define MU 0.02

namespace thunder_ns{
    
    /* Class Forward Kinmeatic */    
    class RobKinAdv : public RobKinBasic{
        
        private:
            /* Override pure virtual function */
            virtual void init() override{};
            /* Initialize and resize variables and function */
            virtual void initVarsFuns();
            /* Update result of kinematic casadi function */
            virtual void compute();
            /* Create casadi function */
            virtual void init_casadi_functions();

            /* Input of casadi function */
            std::vector<casadi::SX> args;

            /* Variable for joints to set arguments */
            Eigen::VectorXd q, dq;

        protected:
            /* Variable for joints velocity */
            casadi::SX _dq_;

            /* Create derivative of jacobian casadi function */
            void DHDotJac();
            /* Create pseudo-inverse of jacobian casadi function */
            void DHPinvJac();
            /* Create pseudo-inverse of jacobian only position casadi function */
            void DHPinvJacPos();
            /* Create derivative of pseudo-inverse of jacobian casadi function */
            void DHDotPinvJac();
            /* Create derivative of pseudo-inverse of jacobian casadi function */
            void DHDotPinvJacPos();

            /* Output of casadi function */
            std::vector<casadi::SX> dotJacobian_res, pinvJacobian_res, pinvJacobianPos_res, dotPinvJacobian_res, dotPinvJacobianPos_res;
            /* Casadi function */
            casadi::Function dotJacobian_fun, pinvJacobian_fun, pinvJacobianPos_fun, dotPinvJacobian_fun, dotPinvJacobianPos_fun;
			// Other
			std::vector<casadi::SX> T_res, J_res;
			// casadi::Function T_fun, J_fun;

            /* Damping coefficient for pseudo-inverse */
            double _mu_;
            
        public:
            /* Empty costructor */
            RobKinAdv();
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link 
            _mu_: damping for pseudo-inverse*/
            RobKinAdv(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame, FrameOffset& ee_frame, const double _mu_=MU);
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame
            _mu_: damping for pseudo-inverse*/
            RobKinAdv(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame, const double _mu_=MU);
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link 
            _mu_: damping for pseudo-inverse*/
            virtual void init(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame, FrameOffset& ee_frame, const double _mu_=MU);
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame
            _mu_: damping for pseudo-inverse*/
            virtual void init(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame,const double _mu_=MU);
            /* Destructor */
            ~RobKinAdv(){};

            /* Set arguments to update the result of casadi functions (forward kinematic and jacobian) */
            virtual void setArguments(const Eigen::VectorXd& q_);
            /* Set arguments to update the result of forward kinematic, jacobian, derivative of jacobian, 
            pseudo-inverse of jacobian, derivative of pseudo-inverse of jacobian */
            virtual void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_);

			/* Get complete kinematics and Jacobians, not in code generation */
			Eigen::Matrix4d getT0i(int index = -1);
			Eigen::MatrixXd getJi(int index = -1);

            /* Get derivative of jacobian matrix */
            Eigen::MatrixXd getDotJacobian();
            /* Get pseudo-inverse of jacobian matrix */
            Eigen::MatrixXd getPinvJacobian();
            /* Get pseudo-inverse of jacobian matrix only position */
            Eigen::MatrixXd getPinvJacobianPos();
            /* Get derivative pseudo-inverse of jacobian matrix */
            Eigen::MatrixXd getDotPinvJacobian();
            /* Get derivative pseudo-inverse of jacobian matrix only position */
            Eigen::MatrixXd getDotPinvJacobianPos();

            /* Generate code */
            virtual void generate_code(std::string&);
            /* Get function name used to generate code in RobKinAdv */
            virtual std::vector<std::string> getFunctionsName();
            /* Get casadi function */
            virtual std::vector<casadi::Function> getCasadiFunctions();

    };

}

#endif