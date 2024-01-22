/* Test of classes and check validation of regressor compare standard equation of dynamic */

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>
#include <yaml-cpp/yaml.h>

#include "library/RobKinAdv.h"
#include "library/RobReg.h"
#include "library/RobDyn.h"

#define NJ 2
#define PARAM 10

using namespace regrob;
using std::cout;
using std::endl;

bool use_gripper = false;

Eigen::Matrix3d hat(const Eigen::Vector3d v);

int main(){

    Eigen::VectorXd param_REG(PARAM*NJ);
    Eigen::VectorXd param_DYN(PARAM*NJ);

    //-------------------------------Extract Variables-----------------------------------//
    
    try {

        YAML::Node config = YAML::LoadFile("../generatedFiles/inertial_DH.yaml");
        int i = 0;
        for (const auto& node : config) {
            
            if (i==NJ) break;

            std::string linkName = node.first.as<std::string>();
            
            param_DYN[10*i] = node.second["mass"].as<double>();
            param_DYN[10*i+1] = node.second["CoM_x"].as<double>();
            param_DYN[10*i+2] = node.second["CoM_y"].as<double>();
            param_DYN[10*i+3] = node.second["CoM_z"].as<double>();
            param_DYN[10*i+4] = node.second["Ixx"].as<double>();
            param_DYN[10*i+5] = node.second["Ixy"].as<double>();
            param_DYN[10*i+6] = node.second["Ixz"].as<double>();
            param_DYN[10*i+7] = node.second["Iyy"].as<double>();
            param_DYN[10*i+8] = node.second["Iyz"].as<double>();
            param_DYN[10*i+9] = node.second["Izz"].as<double>();
            i++;
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
    }
    std::cout<<"YAML_DH letto"<<std::endl;
    std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;

    //-------------------Obtain param for regressor (no via YAML)------------------------//
    
    Eigen::Matrix3d I0,IG;
    Eigen::Vector3d dOG;
    double m;
    
    for(int i=0;i<NJ;i++){

        m = param_DYN[i*10];
        dOG << param_DYN[i*10+1], param_DYN[i*10+2],param_DYN[i*10+3];
        
        IG(0, 0) = param_DYN[i*10+4];
        IG(0, 1) = param_DYN[i*10+5];
        IG(0, 2) = param_DYN[i*10+6];
        IG(1, 0) = IG(0, 1);
        IG(1, 1) = param_DYN[i*10+7];
        IG(1, 2) = param_DYN[i*10+8];
        IG(2, 0) = IG(0, 2);
        IG(2, 1) = IG(1, 2);
        IG(2, 2) = param_DYN[i*10+9];

        I0 = IG + m * hat(dOG).transpose() * hat(dOG);

        param_REG[i*10] = m;
        param_REG[i*10+1] = m*dOG[0];
        param_REG[i*10+2] = m*dOG[1];
        param_REG[i*10+3] = m*dOG[2];
        param_REG[i*10+4] = I0(0,0);
        param_REG[i*10+5] = I0(0,1);
        param_REG[i*10+6] = I0(0,2);
        param_REG[i*10+7] = I0(1,1);
        param_REG[i*10+8] = I0(1,2);
        param_REG[i*10+9] = I0(2,2);
    }
    std::cout<<"\nparam REG \n"<<param_REG<<std::endl;


    // ---------------------------------------------------------------------------------//
    // ------------------------------TEST CLASSES---------------------------------------//
    // ---------------------------------------------------------------------------------//

    Eigen::Matrix<double,NJ,4> DH_table;
    Eigen::Matrix<double,7,4> DH_table_FULL;

    DH_table_FULL << 0,		-M_PI_2,	0.3330, 0,
                0,      M_PI_2,  	0,      0,
                0.0825, M_PI_2,  	0.3160, 0,
               -0.0825, -M_PI_2,	0,      0,
                0,      M_PI_2,  	0.384,  0,
                0.088,  M_PI_2,  	0,      0,
                0,      0,         	0.107,  0;
    
    DH_table = DH_table_FULL.block(0,0,NJ,4);
    //std::cout<<DH_table<<std::endl;

    /* String of joints' type of robot, R for revolute and P for prismatic */
    std::string jType_FULL = "RRRRRRR"; 
    std::string jType = jType_FULL.substr(0,NJ); 
    //std::cout<<jType<<std::endl;
    
    /* Define frame World to link 0 and offset to link EE (no inertia) */
    FrameOffset base_to_L0({0,0,0},{0,0,0},{0,0,-9.80});
    FrameOffset Ln_to_EE;

    /* Define frame end-effctor respect last joint. Obtained in $(find franka_description)/robots/common/franka_hand.xacro */
    if(use_gripper){
        Ln_to_EE.set_translation({0.0,0.0,0.1034});
        Ln_to_EE.set_ypr({-M_PI_4,0.0,0.0});
    } else {
        Ln_to_EE.set_translation({0.0,0.0,0.0});
        Ln_to_EE.set_ypr({0.0,0.0,0.0});
    }

    /* RobKinAdv, RobReg, RobDyn object */;

    RobKinAdv kinrobot;
    RobReg regrobot;
    RobDyn dynrobot;
    
    kinrobot.init(NJ,jType,DH_table,base_to_L0,Ln_to_EE, 0.001);
    regrobot.init(NJ,jType,DH_table,base_to_L0,Ln_to_EE);
    dynrobot.init(NJ,jType,DH_table,base_to_L0,Ln_to_EE);

    /* Matrix */
    
    Eigen::Matrix<double, NJ, NJ*PARAM> Yr;
    Eigen::Matrix<double, NJ, NJ> myM;
    Eigen::Matrix<double, NJ, NJ> myC;
    Eigen::Matrix<double, NJ, 1> myG;
    Eigen::Matrix<double, 6, NJ> myJac;
    Eigen::Matrix<double, 6, NJ> myJacCM;

    Eigen::Matrix<double, NJ, 1> tau_cmd_dyn;
    Eigen::Matrix<double, NJ, 1> tau_cmd_reg;

    Eigen::VectorXd q(NJ), dq(NJ), dqr(NJ), ddqr(NJ);

    /* Test */
    q = q.setOnes()*0.0;
    dq = dq.setOnes()*0.7;
    dqr = dqr.setOnes()*0.3;
    ddqr = ddqr.setOnes()*0.0;

    kinrobot.setArguments(q,dq);
    regrobot.setArguments(q,dq,dqr,ddqr);
    dynrobot.setArguments(q,dq,param_DYN);

    Yr = regrobot.getRegressor();
    cout<<"\nYr\n"<<Yr;
    myM = dynrobot.getMass();
    cout<<"\nM\n"<<myM;
    myC = dynrobot.getCoriolis();
    cout<<"\nC\n"<<myC;
    myG = dynrobot.getGravity();
    cout<<"\nG\n"<<myG;
    myJac = dynrobot.getJacobian();
    cout<<"\nJac\n"<<myJac;

    tau_cmd_dyn = myM*ddqr + myC*dqr + myG;
    tau_cmd_reg = Yr*param_REG;

    cout<<"\ntau_cmd_dyn:\n"<<tau_cmd_dyn<<endl;
    cout<<"\ntau_cmd_reg:\n"<<tau_cmd_reg<<endl;
    cout<<"\nfunziona diff tau_cmd:\n"<<tau_cmd_dyn-tau_cmd_reg<<endl;

    /* Get Casadi Functions */
    std::vector<casadi::Function> kin_vec, reg_vec, dyn_vec, all_vec;
    kin_vec = kinrobot.getCasadiFunctions();
    reg_vec = regrobot.getCasadiFunctions();
    dyn_vec = dynrobot.getCasadiFunctions();

    /* Merge casadi function */
    int dim1, dim2,dim3;
    dim1 = kin_vec.size();
    dim2 = reg_vec.size();
    dim3 = dyn_vec.size();

    for (int i=2; i<dim1; i++){     // exclude kinematic and jacobian
        all_vec.push_back(kin_vec[i]);
    }
    for (int i=0; i<dim2; i++){
        all_vec.push_back(reg_vec[i]);
    }
    for (int i=2; i<dim3; i++){     // exclude kinematic and jacobian
        all_vec.push_back(dyn_vec[i]);
    }
    if(all_vec.size()!=dim1+dim2+dim3-4) cout<<"Merge Error"<<endl;

    /* Generate merge code */
    std::string relativePath = "";

    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string absolutePath = currentPath / relativePath;
    std::cout << "Absolute path: " << absolutePath << std::endl;

    regrobot.generate_mergeCode(all_vec, absolutePath, "regr_fun_3R_classic");


    return 0;
}

Eigen::Matrix3d hat(const Eigen::Vector3d v){
    Eigen::Matrix3d vhat;
            
    // chech
    if(v.size() != 3 ){
        std::cout<<"in function hat of class FrameOffset invalid dimension of input"<<std::endl;
    }
    
    vhat(0,0) = 0;
    vhat(0,1) = -v[2];
    vhat(0,2) = v[1];
    vhat(1,0) = v[2];
    vhat(1,1) = 0;
    vhat(1,2) = -v[0];
    vhat(2,0) = -v[1];
    vhat(2,1) = v[0];
    vhat(2,2) = 0;

    return vhat;
}
