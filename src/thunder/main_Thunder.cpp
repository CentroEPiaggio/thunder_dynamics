/* Generate C++ code from classes that use casadi library.
In particular generate code to compute for franka emika panda robot:
    - regressor
    - jacobian
    - dot jacobian
    - pseudo-inverse jacobian
    - dot pseudo-inverse jacobian
    - forward kinematic
    - matrix mass
    - matrix coriolis
    - matrix gravity
*/

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>

#include "library/RobKinAdv.h"
#include "library/RobReg.h"
#include "library/RobDyn.h"

using namespace regrob;

using std::cout;
using std::endl;

bool use_gripper = false;
bool copy_flag = true;
#define MU_JACOB 0.0

std::string name_files = "gen_regr_fun";
std::string path_reg = "../generatedFiles/";
std::string path_copy_H = "../../include/utils/gen_regr_fun.h";
std::string path_copy_CPP = "../../src/gen_regr_fun.cpp";

int main(){

    /* Number of joints*/
    const int nj = 7;

    /* Denavit-Hartenberg Table for Franka Emika Panda. Obtained from URDF file in $(find franka_gazebo)/test/launch/panda-gazebo.urdf */
    Eigen::Matrix<double,nj,4> DH_table;
    DH_table << 0,		-M_PI_2,	0.3330, 0,
                0,      M_PI_2,  	0,      0,
                0.0825, M_PI_2,  	0.3160, 0,
               -0.0825, -M_PI_2,	0,      0,
                0,      M_PI_2,  	0.384,  0,
                0.088,  M_PI_2,  	0,      0,
                0,      0,         	0.107,  0; 

    /* String of joints' type of robot, R for revolute and P for prismatic */
    std::string jType = "RRRRRRR"; 

    /* Define frame World to link 0 */
    FrameOffset base_to_L0({0,0,0},{0,0,0},{0,0,-9.81});
    FrameOffset Ln_to_EE;

    /* Define frame end-effctor respect last joint. Obtained in $(find franka_description)/robots/common/franka_hand.xacro */
    if(use_gripper){
        Ln_to_EE.set_translation({0.0,0.0,0.1034});
        Ln_to_EE.set_ypr({-M_PI_4,0.0,0.0});
    } else {
        Ln_to_EE.set_translation({0.0,0.0,0.0});
        Ln_to_EE.set_ypr({0.0,0.0,0.0});
    }

    /* RobKinAdv and RobReg object */;
    RobKinAdv kinrobot;
    RobReg regrobot;
    RobDyn dynrobot;
    
    kinrobot.init(nj,jType,DH_table,base_to_L0,Ln_to_EE, MU_JACOB);
    regrobot.init(nj,jType,DH_table,base_to_L0,Ln_to_EE);
    dynrobot.init(nj,jType,DH_table,base_to_L0,Ln_to_EE);

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
    std::string relativePath = path_reg;

    std::filesystem::path currentPath = std::filesystem::current_path();
    std::string absolutePath = currentPath / relativePath;
    std::cout << "Absolute path: " << absolutePath << std::endl;

    regrobot.generate_mergeCode(all_vec, absolutePath, name_files);

    if(copy_flag){
        /* Copy files in particular path */
        std::filesystem::path sourcePath;
        std::filesystem::path sourceDestPath;
    
        sourcePath = absolutePath + name_files + ".h";
        sourceDestPath = path_copy_H;
        std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

        sourcePath = absolutePath + name_files + ".cpp";
        sourceDestPath = path_copy_CPP;
        std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::overwrite_existing);

    }
    
    return 0;
}