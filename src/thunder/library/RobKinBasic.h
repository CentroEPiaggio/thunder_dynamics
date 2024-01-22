#ifndef ROBKINBASIC_H
#define ROBKINBASIC_H

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <eigen3/Eigen/Dense>
#include "CasadiObj.h"
#include "FrameOffset.h"

namespace regrob{
    
    /* Class Forward Kinmeatic */    
    class RobKinBasic : public CasadiObj{
        
        private:
            
            /* Override virtual function */
            virtual void init() override{};
            /* Initialize and resize atrributes of class (variables and function) */
            virtual void initVarsFuns();
            /* Create and initialize casadi function */
            virtual void init_casadi_functions();
            /* Update result of casadi functions */
            virtual void compute();
            
            /* Input of casadi function */
            std::vector<casadi::SX> args;
            /* Variable for joints in set arguments */
            Eigen::VectorXd q;

        protected:

            /* Variable for joints angle */
            casadi::SX _q_;

            /* Number of joints */
            int _numJoints_;
            /* Joints' type string */
            std::string _jointsType_;
            /* Denavit-Hartenberg parameterization table */
            Eigen::MatrixXd _DHtable_;
            /* Frame offset between world-frame and link 0*/
            FrameOffset _lab2L0_;
            /* Frame offset between end-effector and last link */
            FrameOffset _Ln2EE_;
                        
            /* Compute matrix template of transformation from Denavit-Hartenberg parameterization between link i-1 to link i.
            rowDHTable format is [a alpha d theta], jtsType is "R" or "P" */
            casadi::SX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::SX qi, char jtsType);
            /* Compute Forward Kinematic of joints from Denavit-Hartenberg parameterization */
            std::tuple<casadi::SXVector,casadi::SXVector> DHFwKinJoints();
            /* Compute Jacobian from transformation of link i-th respect link_0 frame (obtainable from a DH table) */
            std::tuple<casadi::SXVector,casadi::SXVector> DHJacJoints(const casadi::SXVector& T0i_vec);
            /* Operator hat */
            casadi::SX hat(const casadi::SX& v);

            /* Compute forward kinematic of end-effector respect link_0 frame */
            casadi::SX DHFwKinEE();
            /* Compute Jacobian of end-effector respect link_0 frame */
            casadi::SX DHJacEE();

            /* Output of casadi function */
            std::vector<casadi::SX> kinematic_res, jacobian_res;
            /* Casadi function */
            casadi::Function kinematic_fun, jacobian_fun;

            /* Create kinematic casadi function */
            void DHKin();
            /* Create jacobian casadi function */
            void DHJac();

            
        public:

            /* Empty costructor */
            RobKinBasic();
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            RobKinBasic(const int numJoints, const std::string jointsType,
                const Eigen::MatrixXd& DHtable, FrameOffset& base_frame, FrameOffset& ee_frame);
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame */
            RobKinBasic(const int numJoints, const std::string jointsType, 
                const Eigen::MatrixXd& DHtable, FrameOffset& base_frame);
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame
            ee_frame: is used to set transformation between end-effector and last link */
            virtual void init(const int numJoints,const std::string jointsType,const 
                Eigen::MatrixXd& DHtable, FrameOffset& base_frame, FrameOffset& ee_frame);
            /* Init variables
            numJoints: number of joints
            jointsType: is string of "R" and "P"
            DHTable: Denavit-Hartenberg table format [a alpha d theta]
            base_frame: is used to set transformation between link0 and world_frame */
            virtual void init(const int numJoints,const std::string jointsType,const 
                Eigen::MatrixXd& DHtable, FrameOffset& base_frame);
            
            /* Destructor */
            ~RobKinBasic(){};

            /* Set arguments to update the result of forward kinematic and jacobian */
            virtual void setArguments(const Eigen::VectorXd& q_);

            /* Get forward kinematic matrix */
            Eigen::Matrix4d getKinematic();
            /* Get jacobian matrix */
            Eigen::MatrixXd getJacobian();

            /* Generate code */
            virtual void generate_code(std::string&);
            /* Get function name used to generate code in RobKinBasic */
            virtual std::vector<std::string> getFunctionsName();
            /* Get casadi function */
            virtual std::vector<casadi::Function> getCasadiFunctions();

    };

}

#endif