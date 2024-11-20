/* 
Generate two yaml files of inertial parameters to use standard equation of dynamic and regressor.

	The inertial parameters are referred to joints' frame(!) of denavit parametrization.

	For standard equation of dynamic the format of link's parameters are [m, CoM_x, CoMy, CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].
	For regressor the format of link's parameters are [m, m*CoM_x, m*CoMy, m*CoM_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz].

	Parameters manipulated can be obtained with transformation of URDF file
*/

#include <iostream>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <random>

#include <yaml-cpp/yaml.h>
// #include "urdf2dh_inertial.h"
#include "genYaml.h"

// #define num_joints 7 // 7 links + hand

// using namespace thunder_ns;

// using std::cout;
// using std::endl;

// bool use_gripper = false;
// bool copy_flag = false;
// std::string path_yaml_DH = "../generatedFiles/inertial_DH.yaml";
// std::string out_REG_file = "../generatedFiles/inertial_DH_REG";
// std::string path_yaml_DH_DYN = "../generatedFiles/inertial_DH_DYN";
// std::string path_copy_DH_REG = "../../config/inertial_DH_REG.yaml";

std::string common_comment = "";

namespace thunder_ns{

	int genInertial_files(const std::string robot_name, const int num_joints, const std::string config_file, const std::string out_DYN_file, const std::string out_REG_file){

		std::vector<std::string> keys_dyn;
		std::vector<std::string> keys_reg;
		keys_dyn.resize(5);
		keys_reg.resize(5);
		keys_dyn[0] = "mass"; keys_dyn[1] = "CoM_"; keys_dyn[2] = "I"; keys_dyn[3] = "DYN", keys_dyn[4] = "standard equation of dynamics";
		keys_reg[0] = "mass"; keys_reg[1] = "m_CoM_"; keys_reg[2] = "I"; keys_reg[3] = "REG"; keys_reg[4] = "regressor";
		
		// std::vector<LinkProp> links_prop;
		std::vector<LinkProp> links_prop_DH;
		std::vector<LinkProp> links_prop_REG;
		// std::vector<urdf2dh_T> transform;

		links_prop_DH.resize(num_joints);
		links_prop_REG.resize(num_joints);

		//------------------------------------Extract Variables-----------------------------------------//

		try {
			YAML::Node config = YAML::LoadFile(config_file);
			YAML::Node dynamics = config["dynamics"];
			int Dl_order=0;
			if (config["Dl_order"]) Dl_order = config["Dl_order"].as<int>();
			// std::cout<<"Dl_order: " << Dl_order << std::endl;

			int link_index = 0;
			for (const auto& node : dynamics) {

				YAML::Node inertial = node.second["inertial"];
				LinkProp properties;
				std::string linkName = node.first.as<std::string>();
				// YAML::Node mass = node.second["mass"];
				// YAML::Node parI = node.second["inertia"];
				// YAML::Node origin = node.second["origin"];
				// std::string xyzStr, rpyStr;
				
				properties.name = linkName;
				properties.mass = inertial["mass"].as<double>();
				properties.xyz[0] = inertial["CoM_x"].as<double>();
				properties.xyz[1] = inertial["CoM_y"].as<double>();
				properties.xyz[2] = inertial["CoM_z"].as<double>();
				properties.parI[0] = inertial["Ixx"].as<double>();
				properties.parI[1] = inertial["Ixy"].as<double>();
				properties.parI[2] = inertial["Ixz"].as<double>();
				properties.parI[3] = inertial["Iyy"].as<double>();
				properties.parI[4] = inertial["Iyz"].as<double>();
				properties.parI[5] = inertial["Izz"].as<double>();
				// link friction
				if (Dl_order){
					properties.Dl.resize(Dl_order);
					YAML::Node friction = node.second["friction"];
					std::vector<double> Dl = node.second["Dl"].as<std::vector<double>>();
					properties.Dl = Dl;
					// std::cout<<"Dl_read: " << Dl << std::endl;
					// for (int j=0; j<Dl_order; j++){
					// 	properties.Dl[j] = Dl[j];
					// }
				}

				// // debug:
				// std::cout<< linkName + ": "<<std::endl;
				// std::cout<< "Mass: " << properties.mass <<std::endl;
				// std::cout<< "COM : " << properties.xyz.data() << std::endl;
				// std::cout<< "I : " << properties.parI.data() << std::endl<<std::endl;

				// links_prop_DH.push_back(properties);
				links_prop_DH[link_index] = properties;
				link_index ++;
			}

			// --- obtain parameters for regressor --- //
			for(int i=0; i<num_joints; i++){

				LinkProp tmp_link = links_prop_DH[i];
				double m = tmp_link.mass;
				Eigen::Vector3d dOG = {tmp_link.xyz[0], tmp_link.xyz[1], tmp_link.xyz[2]};
				Eigen::Matrix3d IG = createI(tmp_link.parI);
				Eigen::Matrix3d I0 = IG + m * hat(dOG) * hat(dOG).transpose();
				dOG = m*dOG;

				links_prop_REG[i].name = tmp_link.name;
				links_prop_REG[i].mass = m;
				links_prop_REG[i].xyz = {dOG[0],dOG[1],dOG[2]};
				links_prop_REG[i].parI[0] = I0(0,0);
				links_prop_REG[i].parI[1] = I0(0,1);
				links_prop_REG[i].parI[2] = I0(0,2);
				links_prop_REG[i].parI[3] = I0(1,1);
				links_prop_REG[i].parI[4] = I0(1,2);
				links_prop_REG[i].parI[5] = I0(2,2);
				links_prop_REG[i].Dl = tmp_link.Dl;
				// std::cout<<"Dl_par: " << links_prop_REG[i].Dl << std::endl;
			}

			// --- save <robot>_inertial_REG.yaml --- //
			try {
				YAML::Emitter emitter;
				fillInertialYaml(num_joints, emitter, links_prop_REG, keys_reg);
				std::ofstream fout(out_REG_file);
				fout << emitter.c_str();
				fout.close();
			} catch (const YAML::Exception& e) {
				std::cerr << "Error while generating YAML: " << e.what() << std::endl;
				return 0;
			}
			// --- save <robot>_inertial_DYN.yaml --- //
			try {
				YAML::Emitter emitter;
				fillInertialYaml(num_joints, emitter, links_prop_DH, keys_dyn);
				std::ofstream fout(out_DYN_file);
				fout << emitter.c_str();
				fout.close();
			} catch (const YAML::Exception& e) {
				std::cerr << "Error while generating YAML: " << e.what() << std::endl;
				return 0;
			}

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return 0;
		}
		return 1;
	}

	void fillInertialYaml(int num_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_){

		YAML::Node yamlFile;
		YAML::Node dynamicsNode;

		emitter_.SetIndent(2);
		emitter_.SetSeqFormat(YAML::Flow);
		emitter_ << YAML::Comment(
			"Inertial parameters referred to Denavit-Hartenberg parametrization to use " + keys_[4] + "\n" + common_comment);
		emitter_ << YAML::Newline;
		// emitter_ << YAML::BeginMap;
		// emitter_ << YAML::Key << "dynamics";

		for (int i=0; i<num_joints; i++) {

			LinkProp link = links_prop_[i];
			YAML::Node linkNode;
			YAML::Node linkInertia;
			YAML::Node linkFric;
			std::string nodeName;

			nodeName = link.name;
			linkInertia[keys_[0]] = link.mass;
			linkInertia[keys_[1]+"x"] = link.xyz[0];
			linkInertia[keys_[1]+"y"] = link.xyz[1];
			linkInertia[keys_[1]+"z"] = link.xyz[2];
			linkInertia[keys_[2]+"xx"] = link.parI[0];
			linkInertia[keys_[2]+"xy"] = link.parI[1];
			linkInertia[keys_[2]+"xz"] = link.parI[2];
			linkInertia[keys_[2]+"yy"] = link.parI[3];
			linkInertia[keys_[2]+"yz"] = link.parI[4];
			linkInertia[keys_[2]+"zz"] = link.parI[5];
			// link friction
			linkFric["Dl"] = link.Dl;
			// std::cout<<"Dl_fill: " << link.Dl << std::endl;

			linkNode["inertial"] = linkInertia;
			linkNode["friction"] = linkFric;
			dynamicsNode[nodeName] = linkNode;

			// emitter_ << YAML::BeginMap;
			// emitter_ << YAML::Key << nodeName;
			// 	emitter_ << YAML::BeginMap;
			// 	emitter_ << YAML::Key << "inertial" << linkInertia;
			// 		// emitter_ << linkInertia;
			// 	// emitter_ << YAML::BeginMap;
			// 	emitter_ << YAML::Key << "friction" << linkFric;
			// 		// emitter_ << linkFric;
			// 	// emitter_ << YAML::EndMap; // << YAML::Newline;
			// 	emitter_ << YAML::EndMap; // << YAML::Newline;
			// 	// emitter_ << YAML::EndMap << YAML::Newline;
			// emitter_ << YAML::EndMap;// << YAML::Newline;
		}
		yamlFile["dynamics"] = dynamicsNode;
		emitter_ << yamlFile << YAML::Newline;
		// emitter_ << YAML::EndMap << YAML::Newline;
	}

}