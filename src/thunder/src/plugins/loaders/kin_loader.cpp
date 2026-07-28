#include "plugins/loaders/kin_loader.h"

using casadi::Slice;


namespace thunder_ns {

	// Build one 6D frame parameterization as [x, y, z, r, p, y].
	// Accepted YAML formats are: xyzrpy, xyz+rpy, or xyz+ypr.
	void KinLoader::parse_frame_parameterization(
		std::shared_ptr<Robot> robot,
		const YAML::Node& frame_node,
		const std::string& frame_prefix,
		casadi::SX& frame_expr,
		std::vector<std::string>& frame_args) {

		vector<short> frame_isSymb(6, 0);
		if (frame_node["symb"]) {
			frame_isSymb = frame_node["symb"].as<vector<short>>();
			if (frame_isSymb.size() != 6) {
				throw std::runtime_error("'" + frame_prefix + ".symb' must have 6 elements.");
			}
		}

		if (frame_node["xyzrpy"]) {
			vector<double> xyzrpy_num = frame_node["xyzrpy"].as<vector<double>>();
			if (xyzrpy_num.size() != 6) {
				throw std::runtime_error("'" + frame_prefix + ".xyzrpy' must have 6 elements.");
			}
			const string xyzrpy_name = frame_prefix + "_xyzrpy";
			SX xyzrpy_symb = SX::sym(xyzrpy_name, 6);
			robot->add_parameter(xyzrpy_name, xyzrpy_symb, xyzrpy_num, frame_isSymb, "Kinematic frame in xyzrpy", true);
			frame_expr = robot->get_model(xyzrpy_name);
			frame_args = {xyzrpy_name};
			return;
		}

		if (!frame_node["xyz"]) {
			throw std::runtime_error("'" + frame_prefix + "' must define either 'xyzrpy' or 'xyz'.");
		}

		vector<double> xyz_num = frame_node["xyz"].as<vector<double>>();
		if (xyz_num.size() != 3) {
			throw std::runtime_error("'" + frame_prefix + ".xyz' must have 3 elements.");
		}

		vector<short> xyz_isSymb(frame_isSymb.begin(), frame_isSymb.begin() + 3);
		vector<short> or_isSymb(frame_isSymb.begin() + 3, frame_isSymb.end());

		const string xyz_name = frame_prefix + "_xyz";
		SX xyz_symb = SX::sym(xyz_name, 3);
		robot->add_parameter(xyz_name, xyz_symb, xyz_num, xyz_isSymb, "Kinematic frame translation", true);

		if (frame_node["rpy"]) {
			vector<double> rpy_num = frame_node["rpy"].as<vector<double>>();
			if (rpy_num.size() != 3) {
				throw std::runtime_error("'" + frame_prefix + ".rpy' must have 3 elements.");
			}
			const string rpy_name = frame_prefix + "_rpy";
			SX rpy_symb = SX::sym(rpy_name, 3);
			robot->add_parameter(rpy_name, rpy_symb, rpy_num, or_isSymb, "Kinematic frame orientation in rpy", true);

			casadi::SXVector frame_parts(2);
			frame_parts[0] = robot->get_model(xyz_name);
			frame_parts[1] = robot->get_model(rpy_name);
			frame_expr = casadi::SX::vertcat(frame_parts);
			frame_args = {xyz_name, rpy_name};
			return;
		}

		if (frame_node["ypr"]) {
			vector<double> ypr_num = frame_node["ypr"].as<vector<double>>();
			if (ypr_num.size() != 3) {
				throw std::runtime_error("'" + frame_prefix + ".ypr' must have 3 elements.");
			}
			const string ypr_name = frame_prefix + "_ypr";
			SX ypr_symb = SX::sym(ypr_name, 3);
			robot->add_parameter(ypr_name, ypr_symb, ypr_num, or_isSymb, "Kinematic frame orientation in ypr", true);

			SX ypr_frame = SX::zeros(6, 1);
			ypr_frame(Slice(3, 6)) = robot->get_model(ypr_name);
			SX rpy_from_ypr = get_euler_rpy(get_transform_ypr(ypr_frame));

			casadi::SXVector frame_parts(2);
			frame_parts[0] = robot->get_model(xyz_name);
			frame_parts[1] = rpy_from_ypr;
			frame_expr = casadi::SX::vertcat(frame_parts);
			frame_args = {xyz_name, ypr_name};
			return;
		}

		throw std::runtime_error("'" + frame_prefix + "' must define either 'rpy' or 'ypr' when 'xyz' is used.");
	}

	// --- Load function --- //
	std::shared_ptr<Robot> KinLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints;
			int ndof;
			vector<string> jointsName;
			vector<string> jointsType;
			vector<string> jointsParentStr;
			vector<int> jointsParent;
			vector<bool> jointsAvailable;
			vector<bool> jointsDerivatives;
			vector<int> jointsDimension;
			vector<vector<double>> jointsAxis;

			// --- Basic Robot properties --- //
			// // - numJoints and jointsType - //
			// if (config_["joints_type"]) {
			// 	vector<string> jointsType = config_["joints_type"].as<vector<string>>();
			// 	if ((config_["num_joints"]) && (config_["num_joints"].as<int>() != jointsType.size()))
			// 		throw std::runtime_error("Mismatch between 'num_joints' and the size of 'joints_type' vector.");
			// 	numJoints = jointsType.size();
			// 	robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			// 	robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			// } else {
			// 	debug_log("No joints type specified in yaml file, using from other plugins", VERB_INFO);
			// }

			// --- Kinematics structure --- //
			if (config_["kinematics"]) {
				auto kinematics = config_["kinematics"];
				ndof = 0;
				for (const auto& joint : kinematics) {
					jointsName.push_back(joint.first.as<string>());
					jointsType.push_back(joint.second["joint_type"] ? joint.second["joint_type"].as<string>() : "FIXED");
					jointsParentStr.push_back(joint.second["parent"] ? joint.second["parent"].as<string>() : "world");
					jointsAvailable.push_back(joint.second["available"] ? joint.second["available"].as<bool>() : false);
					jointsDerivatives.push_back(joint.second["derivatives"] ? joint.second["derivatives"].as<bool>() : false);
					jointsAxis.push_back(joint.second["axis"] ? joint.second["axis"].as<vector<double>>() : vector<double>{0,0,1});
					int jointDof = 0;
					if (joint.second["dimension"]) {
						jointDof = joint.second["dimension"].as<int>();
					} else {
						if (jointsType.back() == "FIXED") jointDof = 0;
						else if (jointsType.back() == "P") jointDof = 1;
						else if (jointsType.back() == "R") jointDof = 1;
						else if (jointsType.back() == "P_SEA") jointDof = 1;
						else if (jointsType.back() == "R_SEA") jointDof = 1;
						else std::cerr << "Cannot obtain the joint dimension of frame " + jointsName.back() << std::endl;
					}
					jointsDimension.push_back(jointDof);
					ndof += jointDof;
				}
				numJoints = jointsType.size();
				// if derivatives set available
				for (int i=0; i<numJoints; i++) if (jointsDerivatives[i]) jointsAvailable[i]=true;
				// add properties
				robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
				robot->add_property<int>("ndof", ndof, "int", "Number of degrees of freedom", true);
				robot->add_property<vector<string>>("jointsName", jointsName, "vector<string>", "Name of joints", true);
				robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
				// robot->add_property<vector<string>>("jointsParentStr", jointsParentStr, "vector<string>", "Parent of joints", true);
				robot->add_property<vector<bool>>("jointsAvailable", jointsAvailable, "vector<bool>", "Joints that are available in the generated library", true);
				robot->add_property<vector<bool>>("jointsDerivatives", jointsDerivatives, "vector<bool>", "Create jacobian derivatives for these joints", true);
				robot->add_property<vector<vector<double>>>("jointsAxis", jointsAxis, "vector<vector<double>>", "Axes of joints", true);
				robot->add_property<vector<int>>("jointsDimension", jointsDimension, "vector<int>", "Degrees of freedom of each joint", true);
			
				// find parent index
				for (int i=0; i<numJoints; i++) {
					string parent = jointsParentStr[i];
					int parent_id = -1;
					if (parent == "world") {
						jointsParent.push_back(parent_id);
					} else {
						while ((++parent_id<numJoints) && (parent != jointsName[parent_id]));
						if (parent_id == numJoints) {
							throw std::runtime_error("Parent joint '" + parent + "' not found for joint '" + jointsName[i] + "'");
						} else {
							jointsParent.push_back(parent_id);
						}
					}
				}
				robot->add_property<vector<int>>("jointsParent", jointsParent, "vector<int>", "Parent Id of joints", true);
				
			} else {
				debug_log("No kinematics specified in yaml file, using from other plugins", VERB_INFO);
			}


			// --- Variables --- //
			// - Normal joints - //
			robot->add_variable("q", SX::sym("q",ndof,1), vector<double>(ndof,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",ndof,1), vector<double>(ndof,0), {1}, "Velocity", true);
			robot->add_variable("ddq", SX::sym("ddq",ndof,1), vector<double>(ndof,0), {1}, "Acceleration", true);
			robot->add_variable("d3q", SX::sym("d3q",ndof,1), vector<double>(ndof,0), {1}, "Jerk", true);
			robot->add_variable("d4q", SX::sym("d4q",ndof,1), vector<double>(ndof,0), {1}, "Snap", true);


			// --- Kinematics parameters --- //
			if (config_["kinematics"]){
				SX par_KIN_expr = SX::zeros(6 * numJoints, 1);
				vector<string> par_KIN_args;
				YAML::Node kinematics = config_["kinematics"];
				
				int idx = 0;
				for (const auto& joint : kinematics) {
					if (idx==numJoints) break;	// termination on link number

					SX joint_expr;
					vector<string> joint_args;
					const string joint_name = joint.first.as<string>();
					const string frame_prefix = "KIN_" + joint_name;
					parse_frame_parameterization(robot, joint.second, frame_prefix, joint_expr, joint_args);

					par_KIN_expr(Slice(6*idx, 6*(idx+1))) = joint_expr;
					par_KIN_args.insert(par_KIN_args.end(), joint_args.begin(), joint_args.end());
					idx++;
				}

				if (!robot->add_function("par_KIN", par_KIN_expr, par_KIN_args, "Kinematic parameters")) {
					std::cerr << "Error adding kinematic parameters function!" << std::endl;
					return robot;
				}
			}


			// // --- Frame Offsets --- //
			// // - Base_to_L0 - //
			// if (config_["Base_to_L0"]) {
			// YAML::Node frame_base = config_["Base_to_L0"];
			// vector<double> world2L0_xyz = frame_base["xyz"].as<vector<double>>();
			// vector<double> world2L0_ypr = frame_base["ypr"].as<vector<double>>();
			// vector<double> world2L0_num(6,1);
			// for (int i = 0; i < 3; i++) {
			// 	world2L0_num[i] = world2L0_xyz[i];
			// 	world2L0_num[i + 3] = world2L0_ypr[i];
			// 	}
			// 	// - Symbolic selectivity - //
			// 	vector<short> world2L0_isSymb;
			// 	if (frame_base["symb"]) world2L0_isSymb = frame_base["symb"].as<vector<short>>();
			// 	else world2L0_isSymb.assign(6, 0);
			// 	// - Model - //
			// 	SX world2L0_symb = SX::sym("world2L0", 6);
			// 	// - Numeric - //
			// 	robot->add_parameter("par_world2L0", world2L0_symb, world2L0_num, world2L0_isSymb, "World to base frame", true);
			// }
			
			// // - Ln_to_EE - //
			// if (config_["Ln_to_EE"]) {
			// 	YAML::Node frame_ee = config_["Ln_to_EE"];
			// 	vector<double> Ln2EE_xyz = frame_ee["xyz"].as<vector<double>>();
			// 	vector<double> Ln2EE_ypr = frame_ee["ypr"].as<vector<double>>();
			// 	vector<double> Ln2EE_num(6, 0);
			// 	for (int i = 0; i < 3; i++) {
			// 	Ln2EE_num[i] = Ln2EE_xyz[i];
			// 	Ln2EE_num[i + 3] = Ln2EE_ypr[i];
			// }
			// // - Symbolic selectivity - //
			// vector<short> Ln2EE_isSymb;
			// if (frame_ee["symb"]) Ln2EE_isSymb = frame_ee["symb"].as<vector<short>>();
			// else Ln2EE_isSymb.assign(6, 0);
			// // - Model - //
			// SX Ln2EE_symb = SX::sym("Ln2EE", 6);
			// // - Numeric - //
			// robot->add_parameter("par_Ln2EE", Ln2EE_symb, Ln2EE_num, Ln2EE_isSymb, "Last link to end-effector frame", true);
			// }
			
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
