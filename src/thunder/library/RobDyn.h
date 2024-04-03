#ifndef ROBDYN_H
#define ROBDYN_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "RobKinBasic.h"
#include "FrameOffset.h"

namespace thunder_ns{
    
    /* Class Forward Kinmeatic */    
    class RobDyn : public RobKinBasic{
        
        private:
            /* Override pure virtual function */
            virtual void init() override{};
            /* Initialize and resize variables and function */
            virtual void initVarsFuns();
            /* Update result of kinematic casadi function */
            virtual void compute();
            /* Create casadi function */
            virtual void init_casadi_functions();

            /* inertial parameters */
            void setInertialParameters();  

            /* Input of casadi function */
            std::vector<casadi::SX> args;

            /* Matrix SX of regressor */
            casadi::SX SX_mass, SX_coriolis, SX_gravity;
            /* Variable for joints to set arguments*/
            Eigen::VectorXd q, dq, param;
            casadi::SXVector _mass_vec_;
            casadi::SXVector _distCM_;
            casadi::SXVector _J_3x3_;

        protected:
            /* Variable for joints */
            casadi::SX _dq_, _param_;

            /* Create casadi function */
            void mass_coriolis_gravity();

            /* Output of casadi function */
            std::vector<casadi::SX> mass_res, coriolis_res, gravity_res;
            /* Casadi function */
            casadi::Function mass_fun, coriolis_fun, gravity_fun;

            /* Creation of dq_sel matrix */
            casadi::SX dq_select(const casadi::SX& dq_);
            /* Compute Jacobian of Center of mass of links */
            std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(const casadi::SXVector& T0i_vec);
            /* Compute C matrix with Christoffel symbols */
            casadi::SX stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
            /* Compute Regressor of Slotine Li from Denavit-Hartenberg parameterization*/
            casadi::SXVector Dynamic(
                const casadi::SX& q_, const casadi::SX& dq_, const std::string jointsType_, const Eigen::MatrixXd& DHtable_, FrameOffset& base_frame);
            
        public:
            /* Empty costructor */
            RobDyn();
            /* Init variables
            numJoints: number of joints
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            RobDyn(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable, 
                FrameOffset& base_frame, FrameOffset& ee_frame);
            /* Init variables
            numJoints: number of joints
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame */
            RobDyn(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,FrameOffset& base_frame);
            /* Init variables
            numJoints: number of joints
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            virtual void init(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,
                FrameOffset& base_frame, FrameOffset& ee_frame);
            /* Init variables
            numJoints: number of joints
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            jtsType: is string of "R" and "P"
            base_frame: is used to set transformation between link0 and world_frame*/
            virtual void init(const int numJoints,const std::string jointsType,const Eigen::MatrixXd& DHtable,FrameOffset& base_frame);
            /* Destructor */
            ~RobDyn(){};

            /* Set arguments to update the result of casadi functions */
            virtual void setArguments(const Eigen::VectorXd& q_);
            /* Set arguments to update the result of casadi functions */
            virtual void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_);
            /* Set arguments to update the result of casadi functions */
            virtual void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& param_);
            /* Get regressor matrix */
            Eigen::MatrixXd getMass();
            /* Get regressor matrix */
            Eigen::MatrixXd getCoriolis();
            /* Get regressor matrix */
            Eigen::MatrixXd getGravity();
            /* Generate code for regressor */
            virtual void generate_code(std::string&);
            /* Get function name used to generate code in RobDyn */
            virtual std::vector<std::string> getFunctionsName();
            /* Get casadi function */
            virtual std::vector<casadi::Function> getCasadiFunctions();
            /* Set parameter attribute */
    };

}

#endif