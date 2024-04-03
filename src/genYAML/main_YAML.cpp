/* 
Generate two yaml files of inertial parameters (only for arm + hand) to use standard equation of dynamic and regressor.

    The inertial parameters are referred to joints' frame(!) of denavit parametrization.
    Denavit-Hartenberg parametrization table used is:
        DH_table << 0,      -M_PI_2,  0.3330, 0,
                    0,      M_PI_2,   0,      0,
                    0.0825, M_PI_2,   0.3160, 0,
                    -0.0825,-M_PI_2,  0,      0,
                    0,      M_PI_2,   0.384,  0,
                    0.088,  M_PI_2,   0,      0,
                    0,      0,        0.107,  0;

    For standard equation of dynamic the format of link's parameters are [m, CoM_x, CoMy, CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
    For regressor the format of link's parameters are [m, m*CoM_x, m*CoMy, m*CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].

    Parameters manipulated are obtained with transformation of URDF file from:"
        - $(find franka_gazebo)/test/launch/panda-gazebo.urdf"
        - $(find franka_description)/robots/common/franka_robot.xacro"
        - $(find franka_description)/robots/common/inertial.yaml"
*/

#include <iostream>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>

#include <yaml-cpp/yaml.h>
#include "library/urdf2dh_inertial.h"

#define NUMLINKS 7 // 7 links + hand

using namespace thunder_ns;

using std::cout;
using std::endl;

bool use_gripper = false;
bool copy_flag = false;
std::string path_yaml_DH = "../generatedFiles/inertial_DH.yaml";
std::string path_yaml_DH_REG = "../generatedFiles/inertial_DH_REG";
std::string path_yaml_DH_DYN = "../generatedFiles/inertial_DH_DYN";
std::string path_copy_DH_REG = "../../config/inertial_DH_REG.yaml";

std::string common_comment =             
        "Obtained with transformation of URDF files from:\n"
        "   - $(find franka_gazebo)/test/launch/panda-gazebo.urdf\n"
        "   - $(find franka_description)/robots/common/franka_robot.xacro\n"
        "   - $(find franka_description)/robots/common/inertial.yaml \n"
        "Considering Denavit-Hartenberg parametrization of table:\n"
        "DH_table << 0,      -M_PI_2,  0.3330, 0,\n"
        "            0,      M_PI_2,   0,      0,\n"
        "            0.0825, M_PI_2,   0.3160, 0,\n"
        "            -0.0825,-M_PI_2,  0,      0,\n"
        "            0,      M_PI_2,   0.384,  0,\n"
        "            0.088,  M_PI_2,   0,      0,\n"
        "            0,      0,        0.107,  0;";

void perturbateLinkProp(LinkProp original, LinkProp &perturbate, double percent);

void fillInertialYaml(YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_);

int main(){

    std::vector<std::string> keys_dyn;
    std::vector<std::string> keys_reg;
    keys_dyn.resize(5);
    keys_reg.resize(5);
    keys_dyn[0] = "mass"; keys_dyn[1] = "CoM_"; keys_dyn[2] = "I"; keys_dyn[3] = "DYN", keys_dyn[4] = "standard equation of dynamic";
    keys_reg[0] = "mass"; keys_reg[1] = "m_CoM_"; keys_reg[2] = "I"; keys_reg[3] = "REG"; keys_reg[4] = "regressor";
    
    std::vector<LinkProp> links_prop;
    std::vector<LinkProp> links_prop_DH;
    std::vector<LinkProp> links_prop_DH2REG;
    std::vector<urdf2dh_T> transform;
    
    int numTransform;

    if (use_gripper){
        numTransform = NUMLINKS + 1;
    } else {
        numTransform = NUMLINKS;
    }

    links_prop_DH.resize(numTransform);
    links_prop_DH2REG.resize(NUMLINKS);

    transform.resize(numTransform);
    transform[0].rpy = {+M_PI_2,0,0};   // link transform
    transform[1].rpy = {-M_PI_2,0,0};
    transform[2].rpy = {-M_PI_2,0,0};
    transform[3].rpy = {+M_PI_2,0,0};
    transform[4].rpy = {-M_PI_2,0,0};
    transform[5].rpy = {-M_PI_2,0,0};
    
    transform[0].xyz = {0,0,0};         // link 1
    transform[1].xyz = {0,0,0};
    transform[2].xyz = {-0.0825,0,0};
    transform[3].xyz = {0.0825,0,0};
    transform[4].xyz = {0,0,0};
    transform[5].xyz = {-0.088,0,0};

    if (use_gripper){
        transform[6].rpy = {0,0,0+M_PI_4};
        transform[6].xyz = {0,0,-0.107-0.1034};
        transform[7].rpy = {0,0,0};         // hand
        transform[7].xyz = {0,0,0};         // hand
    } else {
        transform[6].rpy = {0,0,0};
        transform[6].xyz = {0,0,-0.107};
    }

    //------------------------------------Extract Variables-----------------------------------------//

    try {
        YAML::Node config = YAML::LoadFile("yaml/inertial.yaml");
        
        for (const auto& node : config) {

            LinkProp properties;
            std::string linkName = node.first.as<std::string>();
            YAML::Node mass = node.second["mass"];
            YAML::Node parI = node.second["inertia"];
            YAML::Node origin = node.second["origin"];
            std::string xyzStr, rpyStr;
            
            properties.name = linkName;
            properties.mass = mass.as<double>();
            properties.parI[0] = parI["xx"].as<double>();
            properties.parI[1] = parI["xy"].as<double>();
            properties.parI[2] = parI["xz"].as<double>();
            properties.parI[3] = parI["yy"].as<double>();
            properties.parI[4] = parI["yz"].as<double>();
            properties.parI[5] = parI["zz"].as<double>();
            
            if (origin["xyz"]) {
                xyzStr = origin["xyz"].as<std::string>();
                std::istringstream xyzStream(xyzStr);            
                xyzStream >> properties.xyz[0] >> properties.xyz[1] >> properties.xyz[2];
            } else {
                std::cerr << "Missing 'xyz' in the 'origin' node." << std::endl;
                continue;
            }

            if (origin["rpy"]) {
                rpyStr = origin["rpy"].as<std::string>();
                std::istringstream rpyStream(rpyStr);            
                rpyStream >> properties.rpy[0] >> properties.rpy[1] >> properties.rpy[2];
            } else {
                std::cerr << "Missing 'rpy' in the 'origin' node." << std::endl;
                continue;
            }

            links_prop.push_back(properties);
        }
        
    } catch (const YAML::Exception& e) {
        std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
    }

    //-----------------------Convert inertial parameters from URDF to DH----------------------------//

    for (int i = 1; i<=numTransform; i++){
        trasformBodyInertial(transform[i-1].xyz,transform[i-1].rpy,links_prop[i],links_prop_DH[i-1]);
    }

    /* Merge link 7 parameters and hand parameters */

    if (use_gripper){

        LinkProp link_7, link_hand, newlink_7;
        
        link_7 = links_prop_DH[6];
        link_hand = links_prop_DH[7];
        mergeBodyInertial(link_7,link_hand,newlink_7);
        newlink_7.name = link_7.name;
        links_prop_DH[6] = newlink_7;
    }
    
    //----------------!!! Generate DH inertial parameters YAML only for TEST !!!-----------------------//
    
    try {
        YAML::Emitter emitter;
        fillInertialYaml(emitter, links_prop_DH, keys_dyn);
        std::ofstream fout(path_yaml_DH);
        fout << emitter.c_str();
        fout.close();

        std::cout << "Successfully generated YAML file of inertial parameter for DH parametrization"<<std::endl;
        std::cout<< " path: " << path_yaml_DH << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error while generating YAML: " << e.what() << std::endl;
    }
    
    //------------------Generate YAML of inertial parameters for standard dynamic---------------------//

    try {
        YAML::Emitter emitter;
        fillInertialYaml(emitter, links_prop_DH, keys_dyn);
        std::ofstream fout(path_yaml_DH_DYN + ".yaml");
        fout << emitter.c_str();
        fout.close();
        
        std::cout << "Successfully generated YAML file of inertial parameters for dynamics"<<std::endl;
        std::cout<< " path: " << path_yaml_DH_DYN + ".yaml" << std::endl;

    } catch (const YAML::Exception& e) {
        std::cerr << "Error while generating YAML: " << e.what() << std::endl;
    }

    //------------------Generate YAML of inertial parameters for regressor---------------------//
    
    LinkProp tmp_link;
    LinkProp tmp_DH_gauss;
    Eigen::Matrix3d I0,IG;
    Eigen::Vector3d dOG;
    double m;

    // Percentage of perturbation
    Eigen::VectorXd coeff_p(5);
    coeff_p << 0, 2, 5, 10, 20;

    for (int w=0;w<5;w++){

        for(int i=0;i<NUMLINKS;i++){

            tmp_link = links_prop_DH[i];
            perturbateLinkProp(tmp_link,tmp_DH_gauss,coeff_p[w]);

            m = tmp_DH_gauss.mass;
            dOG << tmp_DH_gauss.xyz[0], tmp_DH_gauss.xyz[1], tmp_DH_gauss.xyz[2];
            IG = createI(tmp_DH_gauss.parI);
            I0 = IG + m * hat(dOG) * hat(dOG).transpose();
            dOG = m*dOG;

            links_prop_DH2REG[i].name = tmp_DH_gauss.name;
            links_prop_DH2REG[i].mass = tmp_DH_gauss.mass;
            links_prop_DH2REG[i].xyz = {dOG[0],dOG[1],dOG[2]};
            links_prop_DH2REG[i].parI[0] = I0(0,0);
            links_prop_DH2REG[i].parI[1] = I0(0,1);
            links_prop_DH2REG[i].parI[2] = I0(0,2);
            links_prop_DH2REG[i].parI[3] = I0(1,1);
            links_prop_DH2REG[i].parI[4] = I0(1,2);
            links_prop_DH2REG[i].parI[5] = I0(2,2);
        }

        try {
            YAML::Emitter emitter;
            fillInertialYaml(emitter, links_prop_DH2REG, keys_reg);
            std::string pp;
            if((int)coeff_p[w]==0) pp="";
            else pp = "_p" + std::to_string((int)coeff_p[w]);
            std::ofstream fout(path_yaml_DH_REG + pp + ".yaml");
            fout << emitter.c_str();
            fout.close();

            std::cout << "Successfully generated YAML file of inertial parameters for regressor"<<std::endl;
            std::cout<< " path: " << path_yaml_DH_REG + "_p" + std::to_string((int)coeff_p[w]) + ".yaml"<< std::endl;

        } catch (const YAML::Exception& e) {
            std::cerr << "Error while generating YAML: " << e.what() << std::endl;
        }

        if (copy_flag){
            std::string absolutePath;
            std::filesystem::path sourcePath;
            std::filesystem::path sourceDestPath;
            absolutePath = std::filesystem::current_path();
            sourcePath = absolutePath + "/" + path_yaml_DH_REG + ".yaml";
            sourceDestPath = path_copy_DH_REG;
            std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::update_existing);
            std::cout<<"Files yaml copied"<<std::endl;
        }   
    }

    return 0;
}

void perturbateLinkProp(LinkProp original, LinkProp &perturbate, double percent){

    std::random_device rd;
    std::mt19937 gen(rd());
    std::normal_distribution<double> dist(0.0, percent/100);

    perturbate.name = original.name;
    perturbate.mass = original.mass*(1+dist(gen));
    perturbate.xyz[0] = original.xyz[0]*(1+dist(gen));
    perturbate.xyz[1] = original.xyz[1]*(1+dist(gen));
    perturbate.xyz[2] = original.xyz[2]*(1+dist(gen));
    perturbate.parI[0] = original.parI[0]*(1+dist(gen));
    perturbate.parI[1] = original.parI[1]*(1+dist(gen));
    perturbate.parI[2] = original.parI[2]*(1+dist(gen));
    perturbate.parI[3] = original.parI[3]*(1+dist(gen));
    perturbate.parI[4] = original.parI[4]*(1+dist(gen));
    perturbate.parI[5] = original.parI[5]*(1+dist(gen));
}

void fillInertialYaml(YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_){

    YAML::Node control;

    emitter_.SetIndent(2);
    emitter_.SetSeqFormat(YAML::Flow);
    emitter_ << YAML::Comment(
        "Inertial parameters referred to Denavit-Hartenberg parametrization to use " + keys_[4] + "\n" + common_comment);
    emitter_ << YAML::Newline;

    for (int i=0; i<NUMLINKS; i++) {

        LinkProp link = links_prop_[i];    
        YAML::Node linkNode;
        std::string nodeName;

        nodeName = link.name;
        linkNode[keys_[0]] = link.mass;
        linkNode[keys_[1]+"x"] = link.xyz[0];
        linkNode[keys_[1]+"y"] = link.xyz[1];
        linkNode[keys_[1]+"z"] = link.xyz[2];
        linkNode[keys_[2]+"xx"] = link.parI[0];
        linkNode[keys_[2]+"xy"] = link.parI[1];
        linkNode[keys_[2]+"xz"] = link.parI[2];
        linkNode[keys_[2]+"yy"] = link.parI[3];
        linkNode[keys_[2]+"yz"] = link.parI[4];
        linkNode[keys_[2]+"zz"] = link.parI[5];

        emitter_ << YAML::BeginMap;
        emitter_ << YAML::Key << nodeName;
        emitter_ << linkNode;
        emitter_ << YAML::EndMap << YAML::Newline;
    }
}
