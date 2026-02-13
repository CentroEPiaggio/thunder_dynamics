#include "plugins/loaders/dh_loader.h"
#include "utils.h"

using casadi::Slice;


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> DHLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints;
			int ndof = 0;
			vector<string> jointsName;
			vector<string> jointsType;
			vector<int> jointsParent;
			vector<bool> jointsAvailable;
			vector<bool> jointsDerivatives;
			vector<int> jointsDimension;
			vector<vector<double>> jointsAxis;


			// --- Basic Robot properties --- //
			// - numJoints and jointsType - //
			if (config_["joints_type"]) {
				vector<string> jointsType_tmp = config_["joints_type"].as<vector<string>>();
				if ((config_["num_joints"]) && (config_["num_joints"].as<int>() != jointsType_tmp.size()))
					throw std::runtime_error("Mismatch between 'num_joints' and the size of 'joints_type' vector.");
				numJoints = jointsType_tmp.size() + 2;	// base frame and end-effector are fixed joints
				robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
				jointsName.resize(numJoints, "");
				jointsParent.resize(numJoints, -1);
				jointsType.resize(numJoints, "FIXED");
				jointsAvailable.resize(numJoints, false);
				jointsDerivatives.resize(numJoints, false);
				jointsDimension.resize(numJoints, 0);
				jointsAxis.resize(numJoints, vector<double>{0,0,1});
				for (int i=0; i<numJoints-2; i++) {
					jointsName[i+1] = "link" + std::to_string(i);
					jointsType[i+1] = jointsType_tmp[i];
					jointsDimension[i+1] = 1;
					jointsParent[i+1] = i;
					if (jointsType_tmp[i] != "FIXED") ndof++;
				}
				robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
				robot->add_property<int>("ndof", ndof, "int", "Number of degrees of freedom", true);
			} else {
				debug_log("No joints type specified in yaml file, using from other plugins", VERB_INFO);
			}


			// --- Variables --- //
			robot->add_variable("q", SX::sym("q",ndof,1), vector<double>(ndof,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",ndof,1), vector<double>(ndof,0), {1}, "Velocity", true);
			robot->add_variable("ddq", SX::sym("ddq",ndof,1), vector<double>(ndof,0), {1}, "Acceleration", true);
			robot->add_variable("d3q", SX::sym("d3q",ndof,1), vector<double>(ndof,0), {1}, "Jerk", true);
			robot->add_variable("d4q", SX::sym("d4q",ndof,1), vector<double>(ndof,0), {1}, "Snap", true);


			// --- Frame Offsets --- //
			// - Base_to_L0 - //
			if (config_["Base_to_L0"]) {
				jointsName[0] = (config_["Base_to_L0"]["name"]) ? config_["Base_to_L0"]["name"].as<string>() : "world2L0";
				YAML::Node frame_base = config_["Base_to_L0"];
				vector<double> world2L0_xyz = frame_base["xyz"].as<vector<double>>();
				vector<double> world2L0_ypr = frame_base["ypr"].as<vector<double>>();
				vector<double> world2L0_num(6,0);
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
				// // adjust this part --------------------------------------------------------------------------------------
				// if (frame_base["xyzrpy"]){
				// 	par_KIN(Slice(0,6)) = world2L0_symb;
				// 	world2L0_num = frame_base["xyzrpy"].as<vector<double>>();
				// } else if(frame_base["xyz"]){
				// 	vector<double> xyz = frame_base["xyz"].as<vector<double>>();
				// 	vector<double> rpy;
				// 	if (frame_base["rpy"]){
				// 		par_KIN(Slice(3,6)) = world2L0_symb(Slice(3,6));
				// 		rpy = frame_base["rpy"].as<vector<double>>();
				// 	} else if(frame_base["ypr"]){
				// 		vector<double> ypr = frame_base["ypr"].as<vector<double>>();
				// 		// conversion from ypr to rpy
				// 		SX ypr_num({0,0,0, ypr[0], ypr[1], ypr[2]});
				// 		SX T_num = get_transform_ypr(ypr_num);
				// 		SX rpy_num = get_euler_rpy(T_num);
				// 		SX ypr_symb({0,0,0, ypr[0], ypr[1], ypr[2]});
				// 		SX T_symb = get_transform_ypr(ypr_symb);
				// 		SX rpy_num = get_euler_rpy(T_symb);
				// 		rpy = {static_cast<double>(rpy_num(0)), static_cast<double>(rpy_num(1)), static_cast<double>(rpy_num(2))};
				// 	}
				// 	for (int i=0; i<3; i++){
				// 		world2L0_num[i] = xyz[i];
				// 		world2L0_num[i + 3] = rpy[i];
				// 	}
				// }
				// robot->add_function("par_world2L0", ...)
				// // ---------------------------------------------------------------------------------------------------------
			} else {
				jointsName[0] = "world2L0";
				robot->add_parameter("par_world2L0", SX::sym("world2L0", 6), vector<double>(6,0), {0}, "World to base frame", true);
			}


			// --- Denavit-Hartenberg --- //
			if (config_["DH"]) {
				auto dh_config = config_["DH"];
				vector<double> dh_num = dh_config["value"].as<vector<double>>();
				int dh_size = dh_num.size();
				const int nj_dh = dh_size/4;
				if (nj_dh != numJoints-2) throw std::runtime_error("Mismatch joints - size DH.");
				// - Symbolic selectivity - //
				vector<short> dh_isSymb;
				if (dh_config["symb"]) dh_isSymb = dh_config["symb"].as<vector<short>>();
				else dh_isSymb.assign(dh_size, 0);
				// - Model - //
				SX dh_symb = SX::sym("DHtable", dh_size);
				// - Add to parameters - //
				robot->add_parameter("par_DHtable", dh_symb, dh_num, dh_isSymb, "DH parameters", true);
			} else {
				throw std::runtime_error("No 'DH' section in dh_loader config.");
			}

			
			// - Ln_to_EE - //
			if (config_["Ln_to_EE"]) {
				jointsName[numJoints-1] = (config_["Ln2EE"]["name"]) ? config_["Ln2EE"]["name"].as<string>() : "ee";
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
			} else {
				jointsName[numJoints-1] = "ee";
				robot->add_parameter("par_Ln2EE", SX::sym("Ln2EE", 6), vector<double>(6,0), {0}, "Last link to end-effector frame", true);
			}


			// --- Internal kinematics --- //
			SX par_KIN(6*numJoints, 1);

			// - world to base - //
			jointsParent[0] = -1;
			SX world2L0 = robot->get_model("par_world2L0");
			par_KIN(Slice(0,6)) = world2L0;

			// - DH table - //
			casadi::SX DH = robot->get_model("par_DHtable");
			for (int i=0; i<numJoints-2; i++){
				jointsName[i+1] = "link" + std::to_string(i);
				jointsParent[i+1] = i;
				// DH transformation:  T_a * T_alpha * T_d * T_theta
				casadi::SX p_a(3,1);
				casadi::SX p_d(3,1);
				Slice idx_tr(6*(i+1), 3+6*(i+1));      	// [0,1,2]+6*(i+1) indexes, !first joint is not in DH
				Slice idx_or(3+6*(i+1), 6+6*(i+1));     // [0,1,2]+6*(i+1) indexes
				casadi::SX a = DH(4*i);
				casadi::SX alpha = DH(1 + 4*i);
				casadi::SX d = DH(2 + 4*i);
				casadi::SX theta = DH(3 + 4*i);
				p_a(0) = a;
				p_d(2) = d;

				casadi::SX Rx = R_x(alpha);
				casadi::SX pos = casadi::SX::mtimes(Rx, p_d) + p_a;
				casadi::SX rpy(3,1);
				rpy(0) = alpha;
				rpy(2) = theta;
				par_KIN(idx_tr) = pos;
				par_KIN(idx_or) = rpy;
			}

			// - linkn to EE - //
			jointsParent[numJoints-1] = numJoints-2;
			jointsAvailable[numJoints-1] = true;
			jointsDerivatives[numJoints-1] = true;
			SX Ln2EE = robot->get_model("par_Ln2EE");
			par_KIN(Slice(6*(numJoints-1), 6+6*(numJoints-1))) = Ln2EE;

			// - add properties to robot - //
			robot->add_property<vector<string>>("jointsName", jointsName, "vector<string>", "Name of joints", true);
			robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			robot->add_property<vector<bool>>("jointsAvailable", jointsAvailable, "vector<bool>", "Joints that are available in the generated library", true);
			robot->add_property<vector<bool>>("jointsDerivatives", jointsDerivatives, "vector<bool>", "Create jacobian derivatives for these joints", true);
			robot->add_property<vector<vector<double>>>("jointsAxis", jointsAxis, "vector<vector<double>>", "Axes of joints", true);
			robot->add_property<vector<int>>("jointsDimension", jointsDimension, "vector<int>", "Degrees of freedom of each joint", true);
			robot->add_property<vector<int>>("jointsParent", jointsParent, "vector<int>", "Parent Id of joints", true);
			
			// - creating par_KIN function - //
			if (!robot->add_function("par_KIN", par_KIN, {"par_world2L0", "par_DHtable", "par_Ln2EE"}, "Internal kinematic parameters.")) {
				std::cerr << "Error adding kinmatic parameters!" << std::endl;
				return robot;
			}

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
