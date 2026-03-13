#include "plugins/loaders/urdf_loader.h"
#include "utils.h"
#include <algorithm>
#include <iostream>
#include <filesystem>

namespace thunder_ns {

	void UrdfLoader::accumulateChain(std::shared_ptr<urdf::Link> link, const std::string& base, std::vector<std::shared_ptr<urdf::Link>>& chain) {
		while (link && link->name != base) {
			chain.push_back(link);
			link = link->getParent();
		}
		if (link && link->name == base) {
			chain.push_back(link);
		}
	}

	casadi::SX UrdfLoader::to_casadi_sx(const urdf::Transform& T) {
		casadi::SX R = casadi::SX::zeros(4, 4);
		Eigen::Matrix4d M = T.matrix();
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				R(r, c) = M(r, c);
			}
		}
		return R;
	}

	casadi::DM UrdfLoader::extractKinematicsFromJoint(std::shared_ptr<urdf::Joint> joint) {
		// extract position from joints?
		casadi::DM T_pj = to_casadi_sx(joint->parent_to_joint_transform);
		
		// extract rpy
		casadi::DM rpy = get_euler_rpy(T_pj);

		// extract translation
		casadi::DM xyz = T_pj(casadi::Slice(0,3), 3);

		return casadi::DM::vertcat({xyz, rpy});
	}

	casadi::DM UrdfLoader::extractInertiaFromLink(std::shared_ptr<urdf::Link> link) {
		if (!link->inertial) return casadi::SX::zeros(10,1); // Zero inertia

		casadi::DM m = link->inertial->mass;
		casadi::DM T_li = to_casadi_sx(link->inertial->origin);
		casadi::Slice first3(0, 3);
		casadi::DM CoM = T_li(first3, 3);

		casadi::DM Icom = casadi::SX::zeros(6, 1);
		Icom(0) = link->inertial->ixx;
		Icom(1) = link->inertial->ixy;
		Icom(2) = link->inertial->ixz;
		Icom(3) = link->inertial->iyy;
		Icom(4) = link->inertial->iyz;
		Icom(5) = link->inertial->izz;

		return casadi::DM::vertcat({m, CoM, Icom});
	}

	void UrdfLoader::reset_thunder_chain() {
		numJoints = 0;
		ndof = 0;
		jointsName.resize(0);
		jointsParent.resize(0);
		jointsType.resize(0, "FIXED");
		jointsDimension.resize(0, 0);
		jointsAvailable.resize(0, false);
		jointsDerivatives.resize(0, false);
		jointsAxis.resize(0, {0,0,1});
		par_KIN_num.resize(0, 0);
		par_DYN_num.resize(0, 0);
	}

	bool UrdfLoader::chain_has_link(string link_name) {
		bool has_chain = false;
		for (int i=0; i<chain.size(); i++) {
			if (link_name == chain[i]->name) has_chain = true;
		}
		return has_chain;
	}

	void UrdfLoader::add_chain_from(int parent, std::shared_ptr<urdf::Link> link) {
		// - Fill thunder structures - //
		numJoints++;
		int link_id = jointsName.size();
		jointsName.push_back(link->name);
		jointsParent.push_back(parent);
		parent = link_id++;					// new link index
		auto par_DYN_link = extractInertiaFromLink(link);
		for(int j=0; j<DYN_DIM; j++) par_DYN_num.push_back(static_cast<double>(par_DYN_link(j,0)));

		if ((link->child_joints.empty()) || (ee_link == link)) {		// link is an end-effector
			// cout << "Added end effector: " << link->name << endl;
			jointsType.push_back("FIXED");
			jointsAxis.push_back({0,0,0});
			jointsDimension.push_back(0);
			jointsAvailable.push_back(true);
			jointsDerivatives.push_back(true);
			for(int j=0; j<KIN_DIM; j++) par_KIN_num.push_back(0);
		} else {								// have to explore deeply
			// add kinematic properties for joints
			int i = 0;
			for (auto joint : link->child_joints) {
				if (chain_has_link(joint->child_link_name)) {
					if (i > 0) {
						// Initialize ghost node for branches
						// cout << "_" + std::to_string(i);
						link_id++;
						numJoints++;
						jointsName.push_back(link->name + "_" + std::to_string(i));
						jointsParent.push_back(parent); 							// Attached to same parent as main node
						for(int j=0; j<DYN_DIM; j++) par_DYN_num.push_back(0);		// fictitious link
					}
					// joints origin
					jointsAvailable.push_back(false);
					jointsDerivatives.push_back(false);
					// extract kinematics from joint position
					auto par_KIN_link = extractKinematicsFromJoint(joint);
					for(int j=0; j<KIN_DIM; j++) par_KIN_num.push_back(static_cast<double>(par_KIN_link(j,0)));
					add_joint(joint);
					// cout << endl;
					// add childrens
					if (chain_has_link(joint->child_link_name)) {
						add_chain_from(parent, urdf_model->getLink(joint->child_link_name));
					}
					i++;
				}
			}
		}
	}

	void UrdfLoader::add_joint(std::shared_ptr<urdf::Joint> joint) {
		switch (joint->type) {
			case urdf::JointType::FIXED:
				jointsType.push_back("FIXED");
				jointsDimension.push_back(0);
				jointsAxis.push_back({0,0,0});
				break;
			case urdf::JointType::REVOLUTE:
				jointsType.push_back("R");
				jointsDimension.push_back(1);
				jointsAxis.push_back({joint->axis.x(), joint->axis.y(), joint->axis.z()});
				break;
			case urdf::JointType::CONTINUOUS:
				jointsType.push_back("R");
				jointsDimension.push_back(1);
				jointsAxis.push_back({joint->axis.x(), joint->axis.y(), joint->axis.z()});
				break;
			case urdf::JointType::PRISMATIC:
				jointsType.push_back("P");
				jointsDimension.push_back(1);
				jointsAxis.push_back({joint->axis.x(), joint->axis.y(), joint->axis.z()});
				break;
			case urdf::JointType::FLOATING:
				jointsType.push_back("F");
				// panic
				break;
			case urdf::JointType::PLANAR:
				jointsType.push_back("XY");
				jointsDimension.push_back(2);
				jointsAxis.push_back({0,0,0});
				// panic
				break;
			default:
				debug_log("Detected non-standard joint type for joint '" + joint->name + "'", VERB_INFO);
				jointsType.push_back("UNKNOWN");
				jointsDimension.push_back(0);
				jointsAxis.push_back({0,0,0});
				break;
		}
		ndof += jointsDimension[jointsDimension.size()-1];
	}

	std::shared_ptr<Robot> UrdfLoader::load(std::shared_ptr<Robot> robot) {
		debug_log("URDF Loading started", VERB_INFO);

		try {
			// --- Get configuration --- //
			if (!config_["urdf_path"]) {
				throw std::runtime_error("Missing 'urdf_path' in configuration.");
			}
			auto urdf_path_str = config_["urdf_path"].as<std::string>();
			std::filesystem::path urdf_path(urdf_path_str);
			

			// if the path is relative, we try to resolve it using the config file directory
			if (!urdf_path.is_absolute()) {
				if (config_["config_path"]) {
					// strip the filename, as this ends with the .yaml file
					auto config_dir = std::filesystem::path(config_["config_path"].as<std::string>()).parent_path();
					// resolve wrt config dir	
					urdf_path = config_dir / urdf_path;
					debug_log("Relative URDF path resolved to: " + urdf_path.string(), VERB_DEBUG);
				} else {
					debug_log("URDF path is relative: " + urdf_path_str + " but no config path found. Using current working directory.", VERB_INFO);
					urdf_path = std::filesystem::absolute(urdf_path);
				}
			}

			if (!std::filesystem::exists(urdf_path)) {
				throw std::runtime_error("URDF file not found: " + urdf_path.string());
			}
			
			auto urdf_path_final = urdf_path.string();
			
			auto base_link_name = config_["base_link"] ? config_["base_link"].as<std::string>() : "base_link";
			auto ee_link_name = config_["ee_link"] ? config_["ee_link"].as<std::string>() : "tool0";

			debug_log("Loading URDF from: " + urdf_path_final, VERB_INFO);
			debug_log("Base link: " + base_link_name, VERB_INFO);
			debug_log("End-effector link: " + ee_link_name, VERB_INFO);

			// --- Load URDF --- //
			try {
				urdf_model = urdf::UrdfModel::fromUrdfFile(urdf_path_final.c_str());
			} catch (const std::exception& e) {
				std::cerr << "Failed to load URDF: " << e.what() << std::endl;
				return robot;
			}

			if (!urdf_model) {
				std::cerr << "Failed to load URDF (null model)" << std::endl;
				return robot;
			}

			if (config_["base_link"]){
				try{
					root_link = urdf_model->getLink(config_["base_link"].as<std::string>());
				} catch (const std::exception& e) {
					std::cerr << "Base link '" << base_link_name << "' not found in URDF: " << e.what() << std::endl;
					return robot;
				}
			} else {
				root_link = urdf_model->getRoot();
			}

			// --- Build chain from EE back to base --- //
			
			if (config_["ee_link"]) {
				try{
					ee_link = urdf_model->getLink(config_["ee_link"].as<std::string>());
					if (!ee_link) {
						std::cerr << "End-effector link not found in URDF." << std::endl;
						return robot;
					}
					accumulateChain(ee_link, root_link->name, chain);
					std::reverse(chain.begin(), chain.end());
				} catch (const std::exception& e) {	
					std::cerr << "Error while building kinematic chain: " << e.what() << std::endl;
					return robot;
				}
			} else {
				debug_log("No end-effector link specified, using all links in the model", VERB_INFO);
				urdf_model->getLinks(chain);
			}
			
			debug_log("Chain: ", VERB_DEBUG);
			for (auto& link : chain) {
				debug_log(" - " + link->name, VERB_DEBUG);
			}

			if (chain.empty() || chain.front()->name != base_link_name) {
				std::cerr << "Base link '" << base_link_name << "' not found in chain or chain is empty." << std::endl;
				return robot;
			}

			// --- Identify joints and links --- //
			// std::vector<std::shared_ptr<urdf::Joint>> active_joints;
			// std::vector<casadi::SX> static_transforms; // Transform from parent joint to current joint
			// std::vector<InertialData> active_bodies;
			
			casadi::SX current_cumulative_transform = casadi::SX::eye(4);
			
			debug_log("Chain size: " + std::to_string(chain.size()), VERB_DEBUG);

			// - create thunder structures - //
			reset_thunder_chain();
			// add_chain_from(-1, urdf_model->getLink(base_link_name));
			add_chain_from(-1, root_link);

			// numJoints = jointsName.size();
			
			robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			robot->add_property<int>("ndof", ndof, "int", "Number of degrees of freedom", true);
			robot->add_property<vector<string>>("jointsName", jointsName, "vector<string>", "Name of joints", true);
			robot->add_property<vector<string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			robot->add_property<vector<bool>>("jointsAvailable", jointsAvailable, "vector<bool>", "Joints that are available in the generated library", true);
			robot->add_property<vector<bool>>("jointsDerivatives", jointsDerivatives, "vector<bool>", "Create jacobian derivatives for these joints", true);
			robot->add_property<vector<vector<double>>>("jointsAxis", jointsAxis, "vector<vector<double>>", "Axes of joints", true);
			robot->add_property<vector<int>>("jointsDimension", jointsDimension, "vector<int>", "Degrees of freedom of each joint", true);
			robot->add_property<vector<int>>("jointsParent", jointsParent, "vector<int>", "Parent Id of joints", true);

			// --- Variables --- //
			robot->add_variable("q", casadi::SX::sym("q", ndof, 1), std::vector<double>(ndof, 0), {1}, "Configuration", true);
			robot->add_variable("dq", casadi::SX::sym("dq", ndof, 1), std::vector<double>(ndof, 0), {1}, "Velocity", true);
			robot->add_variable("ddq", casadi::SX::sym("ddq", ndof, 1), std::vector<double>(ndof, 0), {1}, "Acceleration", true);
			robot->add_variable("d3q", casadi::SX::sym("d3q", ndof, 1), std::vector<double>(ndof, 0), {1}, "Jerk", true);
			robot->add_variable("d4q", casadi::SX::sym("d4q", ndof, 1), std::vector<double>(ndof, 0), {1}, "Snap", true);

			// --- Kinematic parameters (par_KIN_num) --- //
			// std::vector<double> par_KIN_num(6 * numJoints, 0);
			// Whether kinematic parameters should be symbolic (1) or numeric (0).
			// Can be overridden per element with `par_KIN_symb`.
			bool kin_symb = config_["symbolic_kinematics"] ? config_["symbolic_kinematics"].as<bool>() : true;
			std::vector<short> par_KIN_isSymb(6 * numJoints, kin_symb ? 1 : 0);
			if (config_["par_KIN_symb"]) {
				par_KIN_isSymb = config_["par_KIN_symb"].as<std::vector<short>>();
			}

			// for (int i = 0; i < numJoints; ++i) {
			// 	const auto& T = static_transforms[i];

			// 	// Translation
			// 	par_KIN_num[6 * i + 0] = static_cast<double>(T(0, 3));
			// 	par_KIN_num[6 * i + 1] = static_cast<double>(T(1, 3));
			// 	par_KIN_num[6 * i + 2] = static_cast<double>(T(2, 3));
				
			// 	// Rotation (RPY)
			// 	casadi::SX rpy = get_euler_rpy(T);
			// 	par_KIN_num[6 * i + 3] = static_cast<double>(rpy(0));
			// 	par_KIN_num[6 * i + 4] = static_cast<double>(rpy(1));
			// 	par_KIN_num[6 * i + 5] = static_cast<double>(rpy(2));
			// }
			robot->add_parameter("par_KIN", casadi::SX::sym("par_KIN", 6 * numJoints, 1), par_KIN_num, par_KIN_isSymb, "Kinematic parameters", true);

			// --- World to L0 (par_world2L0) --- //
			std::vector<double> world2L0_num(6, 0);
			if (config_["Base_to_L0"]) {
				auto base_to_l0 = config_["Base_to_L0"];
				auto xyz = base_to_l0["tr"].as<std::vector<double>>();
				auto ypr = base_to_l0["ypr"].as<std::vector<double>>();
				for (int i = 0; i < 3; ++i) {
					world2L0_num[i] = xyz[i];
					world2L0_num[i + 3] = ypr[i];
				}
			}
			robot->add_parameter("par_world2L0", casadi::SX::sym("world2L0", 6), world2L0_num, std::vector<short>(6, 0), "World to base frame", true);

			// --- Ln to EE (par_Ln2EE) --- //
			std::vector<double> ln2ee_num(6, 0);
			if (config_["Ln_to_EE"]) {
				auto ln_to_ee = config_["Ln_to_EE"];
				auto xyz = ln_to_ee["tr"].as<std::vector<double>>();
				auto ypr = ln_to_ee["ypr"].as<std::vector<double>>();
				for (int i = 0; i < 3; ++i) {
					ln2ee_num[i] = xyz[i];
					ln2ee_num[i + 3] = ypr[i];
				}
			}
			robot->add_parameter("par_Ln2EE", casadi::SX::sym("Ln2EE", 6), ln2ee_num, std::vector<short>(6, 0), "Last link to end-effector frame", true);

			// --- Dynamic parameters (par_DYN_num) --- //
			int STD_PAR_LINK = 10;
			if (robot->properties.count("STD_PAR_LINK")) {
				STD_PAR_LINK = robot->get<int>("STD_PAR_LINK");
			} else {
				robot->add_property<int>("STD_PAR_LINK", STD_PAR_LINK, "int", "Standard number of dynamic parameters per link", true);
			}

			// std::vector<double> par_DYN_num(STD_PAR_LINK * numJoints, 0);
			// Whether dynamic parameters should be symbolic (1) or numeric (0).
			// Can be overridden per-element with `par_DYN_symb`.
			bool dyn_symb = config_["symbolic_dynamics"] ? config_["symbolic_dynamics"].as<bool>() : true;
			std::vector<short> par_DYN_isSymb(STD_PAR_LINK * numJoints, dyn_symb ? 1 : 0);
			if (config_["par_DYN_symb"]) {
				par_DYN_isSymb = config_["par_DYN_symb"].as<std::vector<short>>();
			}

			// for (int i = 0; i < numJoints; ++i) {
			// 	const auto& b = active_bodies[i];
			// 	par_DYN_num[STD_PAR_LINK * i + 0] = static_cast<double>(b.mass);
			// 	par_DYN_num[STD_PAR_LINK * i + 1] = static_cast<double>(b.mass_moment(0));
			// 	par_DYN_num[STD_PAR_LINK * i + 2] = static_cast<double>(b.mass_moment(1));
			// 	par_DYN_num[STD_PAR_LINK * i + 3] = static_cast<double>(b.mass_moment(2));
			// 	par_DYN_num[STD_PAR_LINK * i + 4] = static_cast<double>(b.inertia_at_origin(0, 0)); // Ixx
			// 	par_DYN_num[STD_PAR_LINK * i + 5] = static_cast<double>(b.inertia_at_origin(0, 1)); // Ixy
			// 	par_DYN_num[STD_PAR_LINK * i + 6] = static_cast<double>(b.inertia_at_origin(0, 2)); // Ixz
			// 	par_DYN_num[STD_PAR_LINK * i + 7] = static_cast<double>(b.inertia_at_origin(1, 1)); // Iyy
			// 	par_DYN_num[STD_PAR_LINK * i + 8] = static_cast<double>(b.inertia_at_origin(1, 2)); // Iyz
			// 	par_DYN_num[STD_PAR_LINK * i + 9] = static_cast<double>(b.inertia_at_origin(2, 2)); // Izz
			// }

			robot->add_parameter("par_DYN", casadi::SX::sym("par_DYN", STD_PAR_LINK * numJoints, 1), par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			robot->add_parameter("par_REG", casadi::SX::sym("par_REG", STD_PAR_LINK * numJoints, 1), std::vector<double>(STD_PAR_LINK * numJoints, 0), {1}, "Dynamic parameters for regressor", true);

			// --- Gravity --- //
			std::vector<double> gravity_num = {0.0, 0.0, 0.0}; // default no gravity
			if (config_["gravity"]) {
				gravity_num = config_["gravity"]["value"].as<std::vector<double>>();
			}
			std::vector<short> gravity_isSymb(3, 0);
			if (config_["gravity"] && config_["gravity"]["symb"]) {
				gravity_isSymb = config_["gravity"]["symb"].as<std::vector<short>>();
			}
			robot->add_parameter("par_gravity", casadi::SX::sym("gravity", 3), gravity_num, gravity_isSymb, "Gravity on world frame", true);

		} catch (const std::exception& e) {
			std::cerr << "Error in UrdfLoader::load: " << e.what() << std::endl;
			return robot;
		}

		debug_log("URDF Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
