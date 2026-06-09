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
			// optional prefix for naming DH links ("link" by default)
			string dh_name_prefix = "link";


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

				// read optional DH link name prefix (e.g. "link" -> link0, link1, ...)
				if (config_["DH"] && config_["DH"]["link_names"]) {
					dh_name_prefix = config_["DH"]["link_names"].as<string>();
				}
				for (int i=0; i<numJoints-2; i++) {
					// skip index 0 (base) and reserve last index for end-effector (ee)
					jointsName[i+1] = dh_name_prefix + std::to_string(i);
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
				YAML::Node frame_base = config_["Base_to_L0"];
				jointsName[0] = (frame_base["name"]) ? frame_base["name"].as<string>() : "base";

				vector<short> base_isSymb(6, 0);
				if (frame_base["symb"]) {
					base_isSymb = frame_base["symb"].as<vector<short>>();
					if (base_isSymb.size() != 6) throw std::runtime_error("'Base_to_L0.symb' must have 6 elements.");
				}

				vector<string> world2L0_args;
				SX world2L0_expr;

				if (frame_base["xyzrpy"]) {
					vector<double> world2L0_num = frame_base["xyzrpy"].as<vector<double>>();
					if (world2L0_num.size() != 6) throw std::runtime_error("'Base_to_L0.xyzrpy' must have 6 elements.");
					SX world2L0_symb = SX::sym("Wxyzrpy", 6);
					robot->add_parameter("Wxyzrpy", world2L0_symb, world2L0_num, base_isSymb, "World to base frame in xyzrpy", true);
					world2L0_expr = robot->get_model("Wxyzrpy");
					world2L0_args = {"Wxyzrpy"};
				} else if (frame_base["xyz"]) {
					vector<double> world2L0_xyz = frame_base["xyz"].as<vector<double>>();
					if (world2L0_xyz.size() != 3) throw std::runtime_error("'Base_to_L0.xyz' must have 3 elements.");
					vector<short> xyz_isSymb(base_isSymb.begin(), base_isSymb.begin() + 3);
					vector<short> or_isSymb(base_isSymb.begin() + 3, base_isSymb.end());

					SX xyz_symb = SX::sym("Wxyz", 3);
					robot->add_parameter("Wxyz", xyz_symb, world2L0_xyz, xyz_isSymb, "World to base translation", true);

					if (frame_base["rpy"]) {
						vector<double> world2L0_rpy = frame_base["rpy"].as<vector<double>>();
						if (world2L0_rpy.size() != 3) throw std::runtime_error("'Base_to_L0.rpy' must have 3 elements.");
						SX rpy_symb = SX::sym("Wrpy", 3);
						robot->add_parameter("Wrpy", rpy_symb, world2L0_rpy, or_isSymb, "World to base orientation in rpy", true);

						casadi::SXVector frame_parts(2);
						frame_parts[0] = robot->get_model("Wxyz");
						frame_parts[1] = robot->get_model("Wrpy");
						world2L0_expr = casadi::SX::vertcat({frame_parts});
						world2L0_args = {"Wxyz", "Wrpy"};
					} else if (frame_base["ypr"]) {
						vector<double> world2L0_ypr = frame_base["ypr"].as<vector<double>>();
						if (world2L0_ypr.size() != 3) throw std::runtime_error("'Base_to_L0.ypr' must have 3 elements.");
						SX ypr_symb = SX::sym("Wypr", 3);
						robot->add_parameter("Wypr", ypr_symb, world2L0_ypr, or_isSymb, "World to base orientation in ypr", true);

						SX ypr_frame = SX::zeros(6, 1);
						ypr_frame(Slice(3, 6)) = robot->get_model("Wypr");
						SX rpy_from_ypr = get_euler_rpy(get_transform_ypr(ypr_frame));

						casadi::SXVector frame_parts(2);
						frame_parts[0] = robot->get_model("Wxyz");
						frame_parts[1] = rpy_from_ypr;
						world2L0_expr = casadi::SX::vertcat(frame_parts);
						world2L0_args = {"Wxyz", "Wypr"};
					} else {
						throw std::runtime_error("'Base_to_L0' must define either 'rpy' or 'ypr' when 'xyz' is used.");
					}
				} else {
					throw std::runtime_error("'Base_to_L0' must define either 'xyzrpy' or 'xyz'.");
				}

				if (!robot->add_function("par_world2L0", world2L0_expr, world2L0_args, "World to base frame.")) {
					std::cerr << "Error adding base frame function!" << std::endl;
					return robot;
				}
			} else {
				jointsName[0] = "base";
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
				jointsName[numJoints-1] = (config_["Ln_to_EE"]["name"]) ? config_["Ln_to_EE"]["name"].as<string>() : "ee";
				YAML::Node frame_ee = config_["Ln_to_EE"];

				vector<short> ee_isSymb(6, 0);
				if (frame_ee["symb"]) {
					ee_isSymb = frame_ee["symb"].as<vector<short>>();
					if (ee_isSymb.size() != 6) throw std::runtime_error("'Ln_to_EE.symb' must have 6 elements.");
				}

				vector<string> Ln2EE_args;
				SX Ln2EE_expr;

				if (frame_ee["xyzrpy"]) {
					vector<double> Ln2EE_num = frame_ee["xyzrpy"].as<vector<double>>();
					if (Ln2EE_num.size() != 6) throw std::runtime_error("'Ln_to_EE.xyzrpy' must have 6 elements.");
					SX Ln2EE_symb = SX::sym("Lnxyzrpy", 6);
					robot->add_parameter("Lnxyzrpy", Ln2EE_symb, Ln2EE_num, ee_isSymb, "Last link to end-effector frame in xyzrpy", true);
					Ln2EE_expr = robot->get_model("Lnxyzrpy");
					Ln2EE_args = {"Lnxyzrpy"};
				} else if (frame_ee["xyz"]) {
					vector<double> Ln2EE_xyz = frame_ee["xyz"].as<vector<double>>();
					if (Ln2EE_xyz.size() != 3) throw std::runtime_error("'Ln_to_EE.xyz' must have 3 elements.");
					vector<short> xyz_isSymb(ee_isSymb.begin(), ee_isSymb.begin() + 3);
					vector<short> or_isSymb(ee_isSymb.begin() + 3, ee_isSymb.end());

					SX xyz_symb = SX::sym("Lnxyz", 3);
					robot->add_parameter("Lnxyz", xyz_symb, Ln2EE_xyz, xyz_isSymb, "Last link to end-effector translation", true);

					if (frame_ee["rpy"]) {
						vector<double> Ln2EE_rpy = frame_ee["rpy"].as<vector<double>>();
						if (Ln2EE_rpy.size() != 3) throw std::runtime_error("'Ln_to_EE.rpy' must have 3 elements.");
						SX rpy_symb = SX::sym("Lnrpy", 3);
						robot->add_parameter("Lnrpy", rpy_symb, Ln2EE_rpy, or_isSymb, "Last link to end-effector orientation in rpy", true);

						casadi::SXVector frame_parts(2);
						frame_parts[0] = robot->get_model("Lnxyz");
						frame_parts[1] = robot->get_model("Lnrpy");
						Ln2EE_expr = casadi::SX::vertcat(frame_parts);
						Ln2EE_args = {"Lnxyz", "Lnrpy"};
					} else if (frame_ee["ypr"]) {
						vector<double> Ln2EE_ypr = frame_ee["ypr"].as<vector<double>>();
						if (Ln2EE_ypr.size() != 3) throw std::runtime_error("'Ln_to_EE.ypr' must have 3 elements.");
						SX ypr_symb = SX::sym("Lnypr", 3);
						robot->add_parameter("Lnypr", ypr_symb, Ln2EE_ypr, or_isSymb, "Last link to end-effector orientation in ypr", true);

						SX ypr_frame = SX::zeros(6, 1);
						ypr_frame(Slice(3, 6)) = robot->get_model("Lnypr");
						SX rpy_from_ypr = get_euler_rpy(get_transform_ypr(ypr_frame));

						casadi::SXVector frame_parts(2);
						frame_parts[0] = robot->get_model("Lnxyz");
						frame_parts[1] = rpy_from_ypr;
						Ln2EE_expr = casadi::SX::vertcat(frame_parts);
						Ln2EE_args = {"Lnxyz", "Lnypr"};
					} else {
						throw std::runtime_error("'Ln_to_EE' must define either 'rpy' or 'ypr' when 'xyz' is used.");
					}
				} else {
					throw std::runtime_error("'Ln_to_EE' must define either 'xyzrpy' or 'xyz'.");
				}

				if (!robot->add_function("par_Ln2EE", Ln2EE_expr, Ln2EE_args, "Last link to end-effector frame.")) {
					std::cerr << "Error adding end-effector frame function!" << std::endl;
					return robot;
				}
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
			// use previously computed prefix if available (stored in local variable dh_name_prefix)
			for (int i=0; i<numJoints-2; i++){
				jointsName[i+1] = dh_name_prefix + std::to_string(i);
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
