#include "plugins/loaders/dh_loader.h"
#include "utils.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> DHLoader::load(std::shared_ptr<Robot> robot){
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


			// --- Denavit-Hartenberg --- //
			if (config_["DH"]) {
				auto dh_config = config_["DH"];
				vector<double> dh_num = dh_config["value"].as<vector<double>>();
				int dh_size = dh_num.size();
				const int NJ = dh_size/4;
				// - Symbolic selectivity - //
				vector<short> dh_isSymb;
				if (dh_config["symb"]) dh_isSymb = dh_config["symb"].as<vector<short>>();
				else dh_isSymb.assign(dh_size, 0);
				// - Model - //
				SX dh_symb = SX::sym("DHtable", 4*NJ);
				// - Add to parameters - //
				robot->add_parameter("par_DHtable", dh_symb, dh_num, dh_isSymb, "DH parameters", true);

				// --- internal kinematic parameters (xyzrpy) --- //
				const int SZ = 6*NJ;
				casadi::SX par_KIN(SZ,1);	// output
				casadi::SX DH = robot->get_model("par_DHtable");

				for (int i=0; i<NJ; i++){
					// DH transformation:  T_a * T_alpha * T_d * T_theta
					casadi::SX p_a(3,1);
					casadi::SX p_d(3,1);
					casadi::Slice idx_tr(6*i, 3+6*i);      	// [0,1,2]+6*i indexes
					casadi::Slice idx_or(3+6*i, 6+6*i);     // [0,1,2]+6*i indexes
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
				if (!robot->add_function("par_KIN", par_KIN, {"par_DHtable"}, "Internal kinematic parameters.")) {
					std::cerr << "Error adding kinmatic parameters!" << std::endl;
					return robot;
				}
			} else {
				throw std::runtime_error("No 'DH' section in dh_loader config.");
			}


			// --- Frame Offsets --- //
			// - Base_to_L0 - //
			if (config_["Base_to_L0"]) {
				YAML::Node frame_base = config_["Base_to_L0"];
				vector<double> world2L0_tr = frame_base["tr"].as<vector<double>>();
				vector<double> world2L0_ypr = frame_base["ypr"].as<vector<double>>();
				vector<double> world2L0_num(6,1);
				for (int i = 0; i < 3; i++) {
					world2L0_num[i] = world2L0_tr[i];
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
			
			// - Base_to_L0 - //
			if (config_["Base_to_L0"]) {
				YAML::Node frame_ee = config_["Ln_to_EE"];
				vector<double> Ln2EE_tr = frame_ee["tr"].as<vector<double>>();
				vector<double> Ln2EE_ypr = frame_ee["ypr"].as<vector<double>>();
				vector<double> Ln2EE_num(6, 0);
				for (int i = 0; i < 3; i++) {
					Ln2EE_num[i] = Ln2EE_tr[i];
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
