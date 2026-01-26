#include "plugins/loaders/kin_loader.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> KinLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints;

			// --- Basic Robot properties --- //
			// - numJoints and jointsType - //
			if (config_["joints_type"]) {
				vector<string> jointsType = config_["joints_type"].as<vector<string>>();
				if ((config_["num_joints"]) && (config_["num_joints"].as<int>() != jointsType.size()))
					throw std::runtime_error("Mismatch between 'num_joints' and the size of 'joints_type' vector.");
				numJoints = jointsType.size();
				robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
				robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			} else {
				debug_log("No joints type specified in yaml file, using from other plugins", VERB_INFO);
			}


			// --- Variables --- //
			// - Normal joints - //
			robot->add_variable("q", SX::sym("q",numJoints,1), vector<double>(numJoints,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity", true);
			robot->add_variable("ddq", SX::sym("ddq",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration", true);
			robot->add_variable("d3q", SX::sym("d3q",numJoints,1), vector<double>(numJoints,0), {1}, "Jerk", true);
			robot->add_variable("d4q", SX::sym("d4q",numJoints,1), vector<double>(numJoints,0), {1}, "Snap", true);


			// --- Kinematics parameters --- //
			if (config_["kinematics"]){
				vector<double> par_KIN_num(6 * numJoints,0);
				vector<short> par_KIN_isSymb(6 * numJoints);
				YAML::Node kinematics = config_["kinematics"];
				
				int idx = 0;
				for (const auto& joint : kinematics) {
					if (idx==numJoints) break;	// termination on link number
					string jointName = joint.first.as<string>();
					
					// - Numeric - //
					if (joint.second["xyzrpy"]){
						vector<double> xyzrpy = joint.second["xyzrpy"].as<vector<double>>();
						for (int i=0; i<6; i++){
							par_KIN_num[6*idx + i] = xyzrpy[i];
						}
					} else if(joint.second["xyz"]){
						vector<double> xyz = joint.second["xyz"].as<vector<double>>();
						vector<double> rpy;
						if (joint.second["rpy"]){
							rpy = joint.second["rpy"].as<vector<double>>();
						} else if(joint.second["ypr"]){
							vector<double> ypr = joint.second["ypr"].as<vector<double>>();
							// conversion from ypr to rpy
							SX ypr_casadi({0,0,0, ypr[0], ypr[1], ypr[2]});
							SX T = get_transform_ypr(ypr_casadi);
							SX rpy_casadi = get_euler_rpy(T);
							rpy = {static_cast<double>(rpy_casadi(0)), static_cast<double>(rpy_casadi(1)), static_cast<double>(rpy_casadi(2))};
						}
						for (int i=0; i<3; i++){
							par_KIN_num[6*idx + i] = xyz[i];
							par_KIN_num[6*idx + i + 3] = rpy[i];
						}
					}
					
					// - Symbolic selectivity - //
					vector<short> joint_isSymb;
					if (joint.second["symb"]) {
						joint_isSymb = joint.second["symb"].as<vector<short>>();
					} else {
						joint_isSymb.assign(6, 0); // Default to non-symbolic
					}
					std::copy(joint_isSymb.begin(), joint_isSymb.end(), par_KIN_isSymb.begin() + 6*idx);
					idx++;
				}
				// - Model - //
				SX par_KIN_symb = SX::sym("par_KIN", 6*numJoints,1);
				// - Add to parameters - //
				robot->add_parameter("par_KIN", par_KIN_symb, par_KIN_num, par_KIN_isSymb, "Kinematic parameters", true);
			}


			// --- Frame Offsets --- //
			// - Base_to_L0 - //
			if (config_["Base_to_L0"]) {
			YAML::Node frame_base = config_["Base_to_L0"];
			vector<double> world2L0_xyz = frame_base["xyz"].as<vector<double>>();
			vector<double> world2L0_ypr = frame_base["ypr"].as<vector<double>>();
			vector<double> world2L0_num(6,1);
			for (int i = 0; i < 3; i++) {
				world2L0_num[i] = world2L0_xyz[i];
				world2L0_num[i + 3] = world2L0_ypr[i];
				}
				// - Symbolic selectivity - //
				vector<short> world2L0_isSymb;
				if (frame_base["symb"]) world2L0_isSymb = frame_base["symb"].as<vector<short>>();
				else world2L0_isSymb.assign(6, 0);
				// - Model - //
				SX world2L0_symb = SX::sym("world2L0", 6);
				// - Numeric - //
				robot->add_parameter("par_world2L0", world2L0_symb, world2L0_num, world2L0_isSymb, "World to base frame", true);
			}
			
			// - Ln_to_EE - //
			if (config_["Ln_to_EE"]) {
				YAML::Node frame_ee = config_["Ln_to_EE"];
				vector<double> Ln2EE_xyz = frame_ee["xyz"].as<vector<double>>();
				vector<double> Ln2EE_ypr = frame_ee["ypr"].as<vector<double>>();
				vector<double> Ln2EE_num(6, 0);
				for (int i = 0; i < 3; i++) {
				Ln2EE_num[i] = Ln2EE_xyz[i];
				Ln2EE_num[i + 3] = Ln2EE_ypr[i];
			}
			// - Symbolic selectivity - //
			vector<short> Ln2EE_isSymb;
			if (frame_ee["symb"]) Ln2EE_isSymb = frame_ee["symb"].as<vector<short>>();
			else Ln2EE_isSymb.assign(6, 0);
			// - Model - //
			SX Ln2EE_symb = SX::sym("Ln2EE", 6);
			// - Numeric - //
			robot->add_parameter("par_Ln2EE", Ln2EE_symb, Ln2EE_num, Ln2EE_isSymb, "Last link to end-effector frame", true);
			}
			
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
