#include "plugins/loaders/urdf_loader.h"
#include "utils.h"
#include "urdf/model.h"
#include "urdf/link.h"
#include "urdf/joint.h"
#include <algorithm>
#include <iostream>
#include <filesystem>

namespace thunder_ns {

	void accumulateChain(std::shared_ptr<urdf::Link> link, const std::string& base, std::vector<std::shared_ptr<urdf::Link>>& chain) {
		while (link && link->name != base) {
			chain.push_back(link);
			link = link->getParent();
		}
		if (link && link->name == base) {
			chain.push_back(link);
		}
	}

	casadi::SX to_casadi_sx(const urdf::Transform& T) {
		casadi::SX R = casadi::SX::zeros(4, 4);
		Eigen::Matrix4d M = T.matrix();
		for (int r = 0; r < 4; ++r) {
			for (int c = 0; c < 4; ++c) {
				R(r, c) = M(r, c);
			}
		}
		return R;
	}

	inline Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
		Eigen::Matrix3d s;
		s << 0, -v(2), v(1),
			 v(2), 0, -v(0),
			 -v(1), v(0), 0;
		return s;
	}

	struct InertialData {
		double mass = 0.0;
		Eigen::Vector3d mass_moment = Eigen::Vector3d::Zero(); // mc: Mass * Center of Mass
		Eigen::Matrix3d inertia_at_origin = Eigen::Matrix3d::Zero(); // Io: Inertia with respect to link origin

		InertialData() = default;

		// Create inertial data from standard URDF properties (CoM and Inertia at CoM)
		static InertialData FromUrdf(double m, const Eigen::Vector3d& com, const Eigen::Matrix3d& I_com) {
			InertialData d;
			d.mass = m;
			d.mass_moment = m * com;
			
			// Parallel Axis Theorem: Shift from Center of Mass to Link Origin
			// Io = I_com + m * ([c] * [c]^T) where [c] is the skew matrix of CoM
			Eigen::Matrix3d skew_c = skew(com);
			d.inertia_at_origin = I_com + m * (skew_c * skew_c.transpose());
			return d;
		}
	};

	// Transform inertia of a body to another reference frame T (arbitrary frames)
	InertialData transformInertia(const InertialData& original, const Eigen::Matrix4d& T) {
		Eigen::Matrix3d R = T.block<3, 3>(0, 0);
		Eigen::Vector3d p = T.block<3, 1>(0, 3);
		Eigen::Matrix3d skew_p = skew(p);

		InertialData transformed;
		
		// Mass is invariant
		transformed.mass = original.mass;

		// Transform Mass Moment (mc)
		// mc_new = R * mc_old + m * p
		transformed.mass_moment = R * original.mass_moment + original.mass * p;

		// Transform Inertia Tensor (Io)
		// We are moving between two arbitrary frames, we need the Generalized Steiner theorem
		// from https://doi.org/10.1119/1.4994835
		// This is physically equivalent to: [Child Origin -> Child CoM -> Parent Origin]
		// rotate inertia to the new frame 
		Eigen::Matrix3d rotated_I = R * original.inertia_at_origin * R.transpose();
		// compute cross terms in skew
		Eigen::Matrix3d skew_Rmc = skew(R * original.mass_moment);
		Eigen::Matrix3d cross_terms = skew_p * skew_Rmc.transpose();
		// the pure steiner term M(x^2 + y^2)
		Eigen::Matrix3d steiner_pure = original.mass * (skew_p * skew_p.transpose());

		// Io_new = I_rotated + (Coupling + Coupling^T) + Steiner_Point_Mass
		transformed.inertia_at_origin = rotated_I + (cross_terms + cross_terms.transpose()) + steiner_pure;

		return transformed;
	}

	// Merge two bodies that are in the same reference frame
	InertialData mergeInertia(const InertialData& a, const InertialData& b) {
		InertialData merged;
		merged.mass = a.mass + b.mass;
		merged.mass_moment = a.mass_moment + b.mass_moment;
		merged.inertia_at_origin = a.inertia_at_origin + b.inertia_at_origin;
		return merged;
	}

	// Extract inertia from a link
	InertialData extractInertiaFromLink(std::shared_ptr<urdf::Link> link) {
		if (!link->inertial) return InertialData();

		double m = link->inertial->mass;
		Eigen::Matrix4d T_li = link->inertial->origin.matrix();
		Eigen::Vector3d com = T_li.block<3, 1>(0, 3);
		Eigen::Matrix3d R_li = T_li.block<3, 3>(0, 0);

		Eigen::Matrix3d I_com_i;
		I_com_i << link->inertial->ixx, link->inertial->ixy, link->inertial->ixz,
				   link->inertial->ixy, link->inertial->iyy, link->inertial->iyz,
				   link->inertial->ixz, link->inertial->iyz, link->inertial->izz;

		// Rotate inertia from inertial frame to link frame
		Eigen::Matrix3d I_com_l = R_li * I_com_i * R_li.transpose();

		return InertialData::FromUrdf(m, com, I_com_l);
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

			if (!urdf_path.is_absolute()) {
				debug_log("URDF path is relative: " + urdf_path_str + ". Resolving against current working directory.", VERB_INFO);
				urdf_path = std::filesystem::absolute(urdf_path);
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
			std::shared_ptr<urdf::UrdfModel> urdf_model;
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

			// --- Get end-effector link --- //
			auto ee_link = urdf_model->getLink(ee_link_name);
			if (!ee_link) {
				std::cerr << "End-effector link '" << ee_link_name << "' not found in URDF." << std::endl;
				return robot;
			}

			// --- Build chain from EE back to base --- //
			std::vector<std::shared_ptr<urdf::Link>> chain;
			accumulateChain(ee_link, base_link_name, chain);
			std::reverse(chain.begin(), chain.end());
			
			debug_log("Chain: ", VERB_DEBUG);
			for (auto& link : chain) {
				debug_log(" - " + link->name, VERB_DEBUG);
			}

			if (chain.empty() || chain.front()->name != base_link_name) {
				std::cerr << "Base link '" << base_link_name << "' not found in chain or chain is empty." << std::endl;
				return robot;
			}

			// --- Identify joints and links --- //
			std::vector<std::shared_ptr<urdf::Joint>> active_joints;
			std::vector<Eigen::Matrix4d> static_transforms; // Transform from parent joint to current joint
			std::vector<InertialData> active_bodies;
			
			Eigen::Matrix4d current_cumulative_transform = Eigen::Matrix4d::Identity();
			
			debug_log("Chain size: " + std::to_string(chain.size()), VERB_DEBUG);

			for (size_t i = 0; i < chain.size() - 1; i++) {
				auto parent = chain[i]; // parent link
				auto child = chain[i+1]; // child link

				std::shared_ptr<urdf::Joint> joint;
				for (auto& j : parent->child_joints) { // child joints of parent link   
					if (j->child_link_name == child->name) { // child link name matches child joint name
						joint = j;
						break;
					}
				}

				if (!joint) {
					std::cerr << "Joint connecting " << parent->name << " to " << child->name << " not found." << std::endl;
					return robot;
				}

				// Extract inertia of the child link (relative to its origin)
				InertialData child_raw_inertia = extractInertiaFromLink(child);

				if (joint->type == urdf::JointType::FIXED) {
					// --- ACCUMULATION (FIXED JOINT) ---
					// Accumulate kinematic transformation
					current_cumulative_transform = current_cumulative_transform * joint->parent_to_joint_transform.matrix();
					
					// Transform child inertia to the current "rigid block" frame
					InertialData child_transformed = transformInertia(child_raw_inertia, current_cumulative_transform);
					
					// Merge inertia into the current block
					if (!active_bodies.empty()) {
						active_bodies.back() = mergeInertia(active_bodies.back(), child_transformed);
					}
				} else {
					// --- NEW ACTIVE JOINT ---
					active_joints.push_back(joint);
					
					// Save joint kinematics
					static_transforms.push_back(current_cumulative_transform * joint->parent_to_joint_transform.matrix());
					
					// Reset: next active link starts with "clean" inertia
					current_cumulative_transform = Eigen::Matrix4d::Identity();
					active_bodies.push_back(child_raw_inertia);
				}
			}

			int numJoints = active_joints.size();
			robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			debug_log("Detected " + std::to_string(numJoints) + " active joints", VERB_DEBUG); 

			std::vector<std::string> jointsType;
			for (auto& j : active_joints) {

				switch (j->type) {
					case urdf::JointType::REVOLUTE:
						jointsType.push_back("R");
						break;
					case urdf::JointType::CONTINUOUS:
						jointsType.push_back("R");
						break;
					case urdf::JointType::PRISMATIC:
						jointsType.push_back("P");
						break;
					case urdf::JointType::FLOATING:
						jointsType.push_back("F");
						break;
					case urdf::JointType::PLANAR:
						jointsType.push_back("XY");
						break;
					default:
						debug_log("Detected non-standard joint type for joint '" + j->name + "'", VERB_DEBUG);
						// search for map from joint type to 
						jointsType.push_back("UNKNOWN");
						break;
				}
			}
			robot->add_property<std::vector<std::string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			debug_log("Joints types: ", VERB_DEBUG);
			for (auto& t : jointsType) {
				debug_log(" - " + t, VERB_DEBUG);
			}
			
			// --- Variables --- //
			robot->add_variable("q", casadi::SX::sym("q", numJoints, 1), std::vector<double>(numJoints, 0), {1}, "Configuration", true);
			robot->add_variable("dq", casadi::SX::sym("dq", numJoints, 1), std::vector<double>(numJoints, 0), {1}, "Velocity", true);
			robot->add_variable("ddq", casadi::SX::sym("ddq", numJoints, 1), std::vector<double>(numJoints, 0), {1}, "Acceleration", true);
			robot->add_variable("d3q", casadi::SX::sym("d3q", numJoints, 1), std::vector<double>(numJoints, 0), {1}, "Jerk", true);
			robot->add_variable("d4q", casadi::SX::sym("d4q", numJoints, 1), std::vector<double>(numJoints, 0), {1}, "Snap", true);

			// --- Kinematic parameters (par_KIN) --- //
			std::vector<double> par_KIN_num(6 * numJoints, 0);
			std::vector<short> par_KIN_isSymb(6 * numJoints, 0);

			for (int i = 0; i < numJoints; ++i) {
				const auto& T = static_transforms[i];

				// Translation
				par_KIN_num[6 * i + 0] = T(0, 3);
				par_KIN_num[6 * i + 1] = T(1, 3);
				par_KIN_num[6 * i + 2] = T(2, 3);
				
				// Rotation (YPR)
				Eigen::Vector3d ypr = get_euler_angles(T);
				par_KIN_num[6 * i + 3] = ypr(0);
				par_KIN_num[6 * i + 4] = ypr(1);
				par_KIN_num[6 * i + 5] = ypr(2);
			}
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

			// --- Dynamic parameters (par_DYN) --- //
			int STD_PAR_LINK = 10;
			if (robot->properties.count("STD_PAR_LINK")) {
				STD_PAR_LINK = robot->get<int>("STD_PAR_LINK");
			} else {
				robot->add_property<int>("STD_PAR_LINK", STD_PAR_LINK, "int", "Standard number of dynamic parameters per link", true);
			}

			std::vector<double> par_DYN_num(STD_PAR_LINK * numJoints, 0);
			std::vector<short> par_DYN_isSymb(STD_PAR_LINK * numJoints, 0);

			for (int i = 0; i < numJoints; ++i) {
				const auto& b = active_bodies[i];
				par_DYN_num[STD_PAR_LINK * i + 0] = b.mass;
				par_DYN_num[STD_PAR_LINK * i + 1] = b.mass_moment(0);
				par_DYN_num[STD_PAR_LINK * i + 2] = b.mass_moment(1);
				par_DYN_num[STD_PAR_LINK * i + 3] = b.mass_moment(2);
				par_DYN_num[STD_PAR_LINK * i + 4] = b.inertia_at_origin(0, 0); // Ixx
				par_DYN_num[STD_PAR_LINK * i + 5] = b.inertia_at_origin(0, 1); // Ixy
				par_DYN_num[STD_PAR_LINK * i + 6] = b.inertia_at_origin(0, 2); // Ixz
				par_DYN_num[STD_PAR_LINK * i + 7] = b.inertia_at_origin(1, 1); // Iyy
				par_DYN_num[STD_PAR_LINK * i + 8] = b.inertia_at_origin(1, 2); // Iyz
				par_DYN_num[STD_PAR_LINK * i + 9] = b.inertia_at_origin(2, 2); // Izz
			}

			robot->add_parameter("par_DYN", casadi::SX::sym("par_DYN", STD_PAR_LINK * numJoints, 1), par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			robot->add_parameter("par_REG", casadi::SX::sym("par_REG", STD_PAR_LINK * numJoints, 1), std::vector<double>(STD_PAR_LINK * numJoints, 0), par_DYN_isSymb, "Dynamic parameters for regressor", true);

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
