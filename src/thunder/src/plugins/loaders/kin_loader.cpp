#include "plugins/loaders/kin_loader.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> KinLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints = 0;
			vector<string> jointsType;

			// --- Basic Robot properties --- //
			// - numJoints and jointsType - //
			if (config_["type_joints"]) {
				jointsType = config_["type_joints"].as<vector<string>>();
				numJoints = (config_["num_joints"]) ? (config_["num_joints"].as<int>()) : jointsType.size();
				if (jointsType.size() != numJoints) {
					throw std::runtime_error("Mismatch between 'num_joints' and the size of 'type_joints' vector.");
				} else {
					robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
					robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Number of joints", true);
				}
			} else {
				debug_log("No joints type specified in yaml file, using from other plugins", VERB_INFO);
				numJoints = robot->get<int>("numJoints");
				jointsType = robot->get<vector<string>>("jointsType");
			}


			// --- Variables --- //
			// - Normal joints - //
			robot->add_variable("q", SX::sym("q",numJoints,1), vector<double>(numJoints,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity", true);
			robot->add_variable("ddq", SX::sym("ddq",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration", true);
			robot->add_variable("d3q", SX::sym("d3q",numJoints,1), vector<double>(numJoints,0), {1}, "Jerk", true);
			robot->add_variable("d4q", SX::sym("d4q",numJoints,1), vector<double>(numJoints,0), {1}, "Snap", true);


			// --- Kinematics parameters --- //
			if (config_["par_KIN"]){
				// add the kinematics parameters from yaml directly
			}


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
			
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}


		// --- Standard joint functions --- //
		auto numJoints = robot->get<int>("numJoints");
        auto jointsType = robot->get<vector<string>>("jointsType");
		SX q_joint;
		casadi::Slice rot(0, 3);      // [0,1,2] indexes
		SX Ti = SX::eye(4);

		// Prismatic classical joint
		Ti = SX::eye(4);
		q_joint = SX::sym("q_joint");
		Ti(2,3) = q_joint;
		if (!robot->add_function("T_JOINT_P", Ti, {}, "Template transformation of joint P", {q_joint})) {
			std::cerr << "Error adding joint function: T_JOINT_P" << std::endl;
		}

		// Rotoidal classical joint
		Ti = SX::eye(4);
		q_joint = SX::sym("q_joint");
		Ti(rot,rot) = R_z(q_joint);
		if (!robot->add_function("T_JOINT_R", Ti, {}, "Template transformation of joint R", {q_joint})) {
			std::cerr << "Error adding joint function: T_JOINT_R" << std::endl;
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
