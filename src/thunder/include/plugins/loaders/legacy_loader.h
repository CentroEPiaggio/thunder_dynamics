#ifndef LEGACY_LOADER_H
#define LEGACY_LOADER_H

#include <yaml-cpp/yaml.h>

#include "../../plugin_interfaces.h"
#include "../../robot.h"

namespace thunder_ns {

	class LegacyLoader : public BaseLoader {
		private:
			YAML::Node config_;
			std::string robot_name;

		public:
			LegacyLoader() : BaseLoader("Legacy Loader", "Creates a robot with the old yaml structure.") {}

			int configure(const YAML::Node& config) override;

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};

	int LegacyLoader::configure(const YAML::Node& config) {
		config_ = config;
		if (config_["robot_name"]) {
			robot_name = config_["robot_name"].as<std::string>();
		} else {
			robot_name = "";
		}
		// default robot_name?
		debug_log("Configured", VERB_INFO);
		return 0;
	}

	// --- Load function --- //
	std::shared_ptr<Robot> LegacyLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);
		if (robot_name != "") robot->robotName = robot_name;

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints = 0;
			vector<string> jointsType;
			bool ELASTIC = false;
			int K_order = 0;
			int D_order = 0;
			int Dm_order = 0;
			int Dl_order = 0;
			int numElasticJoints = 0;
			vector<short> isElasticJoint;

			const int STD_PAR_LINK = 10;
			robot->add_property<const int>("STD_PAR_LINK", STD_PAR_LINK, "const int", "Standard number of dynamic parameters per link", true);

			// YAML::Node config_ = this->config_yaml;

			// --- Basic Robot properties --- //

			// - numJoints - //
			if (config_["num_joints"]) {
				numJoints = config_["num_joints"].as<int>();
				robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			} else {
				throw std::runtime_error("No num_joints in yaml file.");
			}
			// - jointsType - //
			if (config_["type_joints"]) {
				jointsType = config_["type_joints"].as<vector<string>>();
				if (jointsType.size() != numJoints) {
					throw std::runtime_error("Mismatch between 'num_joints' and the size of 'type_joints' vector.");
				} else {
					robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Number of joints", true);
				}
			} else {
				throw std::runtime_error("No joints type specified in yaml file.");
			}
			// --- Elastic model properties (defaults to false) --- //
			YAML::Node elastic_node;
			ELASTIC = config_["ELASTIC_MODEL"] && config_["ELASTIC_MODEL"].as<bool>();
			robot->add_property<bool>("ELASTIC", ELASTIC, "bool", "Elastic flag", true);
			if (ELASTIC) {
				// - setup elastic properties - //
				if (!config_["K_order"]) throw std::runtime_error("K_order property not present.");
				if (!config_["D_order"]) throw std::runtime_error("D_order property not present.");
				if (!config_["Dm_order"]) throw std::runtime_error("Dm_order property not present.");
				if (!config_["elastic"]) throw std::runtime_error("elastic parameters not present.");
				elastic_node = config_["elastic"];
				K_order = elastic_node["K_order"].as<int>();
				D_order = elastic_node["D_order"].as<int>();
				Dm_order = elastic_node["Dm_order"].as<int>();
				robot->add_property<int>("K_order", K_order, "int", "Order of the coupling stiffness model", true);
				robot->add_property<int>("D_order", D_order, "int", "Order of the coupling friction model", true);
				robot->add_property<int>("Dm_order", Dm_order, "int", "Order of the motor stiffness model", true);

				// - identify elastic joints - //
				numElasticJoints = 0;
				isElasticJoint.resize(numJoints);
				for (int i = 0; i < numJoints; i++) {
					if ((jointsType[i] == "R_SEA") || (jointsType[i] == "P_SEA")) {
						isElasticJoint[i] = 1;
						numElasticJoints++;
					} else {
						isElasticJoint[i] = 0;
					}
				}
				robot->add_property<int>("numElasticJoints", numElasticJoints, "int", "Number of elastic joints", true);
				robot->add_property<vector<short>>("isElasticJoint", isElasticJoint, "vector<short>", "Vector of elastic joint flags", true);
			}


			// --- Variables --- //
			// - Normal joints - //
			robot->add_variable("q", SX::sym("q",numJoints,1), vector<double>(numJoints,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity", true);
			robot->add_variable("ddq", SX::sym("ddq",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration", true);
			robot->add_variable("dqr", SX::sym("dqr",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity reference", true);
			robot->add_variable("ddqr", SX::sym("ddqr",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration reference", true);
			robot->add_variable("d3q", SX::sym("d3q",numJoints,1), vector<double>(numJoints,0), {1}, "Jerk", true);
			robot->add_variable("d4q", SX::sym("d4q",numJoints,1), vector<double>(numJoints,0), {1}, "Snap", true);
			// - Elastic joints - //
			robot->add_variable("x", SX::sym("x",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor angle", true);
			robot->add_variable("dx", SX::sym("dx",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor velocity", true);
			robot->add_variable("ddx", SX::sym("ddx",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor acceleration", true);
			robot->add_variable("ddxr", SX::sym("ddxr",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor acceleration reference", true);
			// - Regressors - //
			robot->add_variable("w", SX::sym("w",6,1), vector<double>(6,1), {1}, "Wrench", true);


			// --- Denavit-Hartenberg --- //
			YAML::Node kinematics = config_["kinematics"];
			vector<double> dh_num = kinematics["DH"].as<vector<double>>();
			int dh_size = dh_num.size();
			// - Symbolic selectivity - //
			vector<short> dh_isSymb;
			if (kinematics["symb"]) dh_isSymb = kinematics["symb"].as<vector<short>>();
			else dh_isSymb.assign(dh_size, 0);
			// - Model - //
			SX dh_symb = SX::sym("DHtable", numJoints * 4);
			// - Add to parameters - //
			robot->add_parameter("par_DHtable", dh_symb, dh_num, dh_isSymb, "DH parameters", true);


			// --- Gravity --- //
			vector<double> gravity_num = {0.0, 0.0, 0.0}; // default no gravity
			if (config_["gravity"]){
				gravity_num = config_["gravity"]["value"].as<vector<double>>();
			}
			// - Symbolic selectivity - //
			vector<short> gravity_isSymb;
			if (config_["gravity"]["symb"]) gravity_isSymb = config_["gravity"]["symb"].as<vector<short>>();
			else gravity_isSymb.assign(3, 0);
			// - Model - //
			SX gravity_symb = SX::sym("gravity", 3);
			// - Add to parameters - //
			robot->add_parameter("par_gravity", gravity_symb, gravity_num, gravity_isSymb, "Gravity on world frame", true);


			// --- Frame Offsets --- //
			YAML::Node frame_base = config_["Base_to_L0"];
			YAML::Node frame_ee = config_["Ln_to_EE"];
			vector<double> world2L0_tr = frame_base["tr"].as<vector<double>>();
			vector<double> world2L0_ypr = frame_base["ypr"].as<vector<double>>();
			vector<double> Ln2EE_tr = frame_ee["tr"].as<vector<double>>();
			vector<double> Ln2EE_ypr = frame_ee["ypr"].as<vector<double>>();
			vector<double> world2L0_num(6,1);
			vector<double> Ln2EE_num(6, 0);
			for (int i = 0; i < 3; i++) {
				world2L0_num[i] = world2L0_tr[i];
				world2L0_num[i + 3] = world2L0_ypr[i];
				Ln2EE_num[i] = Ln2EE_tr[i];
				Ln2EE_num[i + 3] = Ln2EE_ypr[i];
			}
			// - Symbolic selectivity - //
			vector<short> world2L0_isSymb;
			vector<short> Ln2EE_isSymb;
			if (frame_base["symb"]) world2L0_isSymb = frame_base["symb"].as<vector<short>>();
			else world2L0_isSymb.assign(6, 0);
			if (frame_ee["symb"]) Ln2EE_isSymb = frame_ee["symb"].as<vector<short>>();
			else Ln2EE_isSymb.assign(6, 0);
			// - Model - //
			SX world2L0_symb = SX::sym("world2L0", 6);
			SX Ln2EE_symb = SX::sym("Ln2EE", 6);
			// - Numeric - //
			robot->add_parameter("par_world2L0", world2L0_symb, world2L0_num, world2L0_isSymb, "World to base frame", true);
			robot->add_parameter("par_Ln2EE", Ln2EE_symb, Ln2EE_num, Ln2EE_isSymb, "Last link to end-effector frame", true);
			

			// --- Dynamics (Inertial) --- //
			vector<double> par_DYN_num(STD_PAR_LINK*numJoints,0);
			vector<double> par_REG_num(STD_PAR_LINK*numJoints,0);
			vector<short> par_DYN_isSymb(STD_PAR_LINK * numJoints);
			YAML::Node dynamics = config_["dynamics"];
			int idx = 0;
			for (const auto& node : dynamics) {
				YAML::Node inertial = node.second["inertial"];

				if (idx==numJoints) break;	// termination on link number
				string linkName = node.first.as<string>();
				
				// - Numeric - //
				par_DYN_num[STD_PAR_LINK*idx] = inertial["mass"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+1] = inertial["CoM_x"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+2] = inertial["CoM_y"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+3] = inertial["CoM_z"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+4] = inertial["Ixx"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+5] = inertial["Ixy"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+6] = inertial["Ixz"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+7] = inertial["Iyy"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+8] = inertial["Iyz"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+9] = inertial["Izz"].as<double>();

				// - Symbolic selectivity - //
				vector<short> link_isSymb;
				if (inertial["symb"]) {
					link_isSymb = inertial["symb"].as<vector<short>>();
				} else {
					link_isSymb.assign(STD_PAR_LINK, 0); // Default to non-symbolic
				}
				std::copy(link_isSymb.begin(), link_isSymb.end(), par_DYN_isSymb.begin() + idx * STD_PAR_LINK);
				idx++;
			}
			// - Model - //
			SX par_DYN_symb = SX::sym("par_DYN", STD_PAR_LINK*numJoints,1);
			SX par_REG_symb = SX::sym("par_REG", STD_PAR_LINK*numJoints,1);
			// - Add to parameters - //
			robot->add_parameter("par_DYN", par_DYN_symb, par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			vector<short> par_REG_isSymb = par_DYN_isSymb;
			robot->add_parameter("par_REG", par_REG_symb, par_REG_num, par_REG_isSymb, "Dynamic parameters for regressor", true);


			// --- Link friction --- //
			Dl_order = config_["Dl_order"] ? config_["Dl_order"].as<int>() : 0;	// defaults to 0
			robot->add_property<int>("Dl_order", Dl_order, "int", "Order of the link friction model", true);
			vector<double> par_Dl_num;
			vector<short> par_Dl_isSymb;
			if (Dl_order > 0) {
				par_Dl_num.resize(Dl_order * numJoints);
				par_Dl_isSymb.resize(Dl_order * numJoints);
				idx = 0;
				for (const auto& node : dynamics) {
					// - Numeric - //
					if (node.second["friction"]){
						vector<double> Dl = node.second["friction"]["Dl"].as<vector<double>>();
						for (int j=0; j<Dl_order; j++){
							par_Dl_num[Dl_order*idx + j] = Dl[j];
						}
					}

					// - Symbolic selectivity - //
					YAML::Node friction = node.second["friction"];
					vector<int> fric_isSymb;
					if (friction["symb"]) {
						fric_isSymb = friction["symb"].as<vector<int>>();
					} else {
						fric_isSymb.assign(Dl_order, 0);
					}
					std::copy(fric_isSymb.begin(), fric_isSymb.end(), par_Dl_isSymb.begin() + idx * Dl_order);
					idx++;
				}
				// - Model - //
				SX par_Dl_symb = SX::sym("par_Dl", numJoints*Dl_order,1);
				// - Add to parameters - //
				robot->add_parameter("par_Dl", par_Dl_symb, par_Dl_num, par_Dl_isSymb, "Link friction parameters", true);
			}

			// - Elastic joints parameters - //
			vector<double> par_K_num, par_D_num, par_Dm_num, par_Mm_num;
			vector<short> par_K_isSymb, par_D_isSymb, par_Dm_isSymb, par_Mm_isSymb;
			if (ELASTIC) {
				par_K_num.resize(numElasticJoints*K_order);
				par_D_num.resize(numElasticJoints*D_order);
				par_Dm_num.resize(numElasticJoints*Dm_order);
				par_Mm_num.resize(numElasticJoints);
				YAML::Node elastic_joints = config_["elastic"]["joints"];
				int i = 0;
				for (const auto& node : elastic_joints) {
					if (i==numElasticJoints) break; // break if nore joints defined
					string jointName = node.first.as<string>();

					// Helper lambda to parse a symbolic vector
					auto parse_symb_vector = [&](const string& key, int order) {
						vector<short> vec;
						if (node.second[key]){
							vec = node.second[key].as<vector<short>>();
							vec.resize(order);
						} else vec.assign(order, 0);
						return vec;
					};

					// - Numeric - //
					// stiffness
					if (K_order > 0){
						vector<double> K = node.second["K"].as<vector<double>>();
						for (int j=0; j<K_order; j++) par_K_num[K_order*i+j] = K[j];
					}
					// coupling friction
					if (D_order > 0){
						vector<double> D = node.second["D"].as<vector<double>>();
						for (int j=0; j<D_order; j++) par_D_num[D_order*i + j] = D[j];
					}
					// motor friction
					if (Dm_order > 0){
						vector<double> Dm = node.second["Dm"].as<vector<double>>();
						for (int j=0; j<Dm_order; j++) par_Dm_num[Dm_order*i + j] = Dm[j];
					}
					// motor inertia
					par_Mm_num[i] = node.second["Mm"].as<double>();
					
					// - Symbolic selectivity - //
					vector<short> K_symb = parse_symb_vector("K_symb", K_order);
					vector<short> D_symb = parse_symb_vector("D_symb", D_order);
					vector<short> Dm_symb = parse_symb_vector("Dm_symb", Dm_order);
					short Mm_symb = node.second["Mm_symb"] ? node.second["Mm_symb"].as<int>() : 0;
					if (K_order > 0) par_K_isSymb.insert(par_K_isSymb.end(), K_symb.begin(), K_symb.end());
					if (D_order > 0) par_D_isSymb.insert(par_D_isSymb.end(), D_symb.begin(), D_symb.end());
					if (Dm_order > 0) par_Dm_isSymb.insert(par_Dm_isSymb.end(), Dm_symb.begin(), Dm_symb.end());
					par_Mm_isSymb.push_back(Mm_symb);
					i++;
				}
				// - Models - //
				SX par_K_symb = SX::sym("par_K", numElasticJoints*K_order,1);
				SX par_D_symb = SX::sym("par_D", numElasticJoints*D_order,1);
				SX par_Dm_symb = SX::sym("par_Dm", numElasticJoints*Dm_order,1);
				SX par_Mm_symb = SX::sym("par_Mm", numElasticJoints,1);
				// - Add to parameters - //
				robot->add_parameter("par_K", par_K_symb, par_K_num, par_K_isSymb, "Coupling stiffness parameters", true);
				robot->add_parameter("par_D", par_D_symb, par_D_num, par_D_isSymb, "Coupling friction parameters", true);
				robot->add_parameter("par_Dm", par_Dm_symb, par_Dm_num, par_Dm_isSymb, "Motor friction parameters", true);
				robot->add_parameter("par_Mm", par_Mm_symb, par_Mm_num, par_Mm_isSymb, "Motor inertia parameters", true);
			}

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns

#endif // LEGACY_LOADER_H
