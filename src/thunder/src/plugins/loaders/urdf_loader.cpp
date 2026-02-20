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

	// struct InertialData {
	// 	casadi::SX mass = casadi::SX(0); 					// scalar
	// 	casadi::SX CoM = casadi::SX::zeros(3, 1);			// center of mass
	// 	casadi::SX Icom = casadi::SX::zeros(6, 1); 			// Inertia with respect to center of mass (Ixx,Ixy,Ixz,Iyy,Iyz,Izz)
	// 	// casadi::SX inertia_at_origin = casadi::SX::zeros(3, 3); // Io: Inertia with respect to link origin

	// 	InertialData() = default;

	// 	casadi::SX getLink_parDyn(){
	// 		casadi::SX::vertcat({mass, CoM, Icom});
	// 	}

	// 	// Create inertial data from standard URDF properties (CoM and Inertia at CoM)
	// 	static InertialData FromUrdf(double m, const casadi::SX& com, const casadi::SX& I_com) {
	// 		InertialData d;
	// 		d.mass = casadi::SX(m);
	// 		d.CoM = com;
	// 		d.Icom = I_com;
	// 		// d.mass_moment = d.mass * com;
			
	// 		// // Parallel Axis Theorem: Shift from Center of Mass to Link Origin
	// 		// // Io = I_com + m * ([c] * [c]^T) where [c] is the skew matrix of CoM
	// 		// casadi::SX skew_c = hat(com);
	// 		// d.inertia_at_origin = I_com + d.mass * casadi::SX::mtimes(skew_c, skew_c.T());
	// 		return d;
	// 	}
	// };

	// // Transform inertia of a body to another reference frame T (arbitrary frames)
	// InertialData transformInertia(const InertialData& original, const casadi::SX& T) {
	// 	casadi::Slice first3(0, 3);
	// 	casadi::SX R = T(first3, first3);
	// 	casadi::SX p = T(first3, 3);
	// 	casadi::SX skew_p = hat(p);

	// 	InertialData transformed;
		
	// 	// Mass is invariant
	// 	transformed.mass = original.mass;

	// 	// Transform Mass Moment (mc)
	// 	// mc_new = R * mc_old + m * p
	// 	transformed.mass_moment = casadi::SX::mtimes(R, original.mass_moment) + original.mass * p;

	// 	// Transform Inertia Tensor (Io)
	// 	// We are moving between two arbitrary frames, we need the Generalized Steiner theorem
	// 	// from https://doi.org/10.1119/1.4994835
	// 	// This is physically equivalent to: [Child Origin -> Child CoM -> Parent Origin]
	// 	// rotate inertia to the new frame 
	// 	casadi::SX rotated_I = casadi::SX::mtimes(R, casadi::SX::mtimes(original.inertia_at_origin, R.T()));
	// 	// compute cross terms in skew
	// 	casadi::SX skew_Rmc = hat(casadi::SX::mtimes(R, original.mass_moment));
	// 	casadi::SX cross_terms = casadi::SX::mtimes(skew_p, skew_Rmc.T());
	// 	// the pure steiner term M(x^2 + y^2)
	// 	casadi::SX steiner_pure = original.mass * casadi::SX::mtimes(skew_p, skew_p.T());

	// 	// Io_new = I_rotated + (Coupling + Coupling^T) + Steiner_Point_Mass
	// 	transformed.inertia_at_origin = rotated_I + (cross_terms + cross_terms.T()) + steiner_pure;

	// 	return transformed;
	// }

	// // Merge two bodies that are in the same reference frame
	// InertialData mergeInertia(const InertialData& a, const InertialData& b) {
	// 	InertialData merged;
	// 	merged.mass = a.mass + b.mass;
	// 	merged.mass_moment = a.mass_moment + b.mass_moment;
	// 	merged.inertia_at_origin = a.inertia_at_origin + b.inertia_at_origin;
	// 	return merged;
	// }

	// // Extract inertia from a link
	// InertialData extractInertiaFromLink(std::shared_ptr<urdf::Link> link) {

	// 	//! This is a bit fragile
	// 	//! It's fine as long the massless links are rigidly attached to a proper link
	// 	//! Which is reasonable for valid urdf models. 
	// 	//! But we should check and if that's not the case we should panic
	// 	if (!link->inertial) return InertialData(); // Zero inertia

	// 	double m = link->inertial->mass;
	// 	casadi::SX T_li = to_casadi_sx(link->inertial->origin);
	// 	casadi::Slice first3(0, 3);
	// 	casadi::SX com = T_li(first3, 3);
	// 	casadi::SX R_li = T_li(first3, first3);

	// 	casadi::SX I_com_i = casadi::SX::zeros(3, 3);
	// 	I_com_i(0, 0) = link->inertial->ixx;
	// 	I_com_i(0, 1) = link->inertial->ixy;
	// 	I_com_i(0, 2) = link->inertial->ixz;
	// 	I_com_i(1, 0) = link->inertial->ixy;
	// 	I_com_i(1, 1) = link->inertial->iyy;
	// 	I_com_i(1, 2) = link->inertial->iyz;
	// 	I_com_i(2, 0) = link->inertial->ixz;
	// 	I_com_i(2, 1) = link->inertial->iyz;
	// 	I_com_i(2, 2) = link->inertial->izz;

	// 	// Rotate inertia from inertial frame to link frame
	// 	casadi::SX I_com_l = casadi::SX::mtimes(R_li, casadi::SX::mtimes(I_com_i, R_li.T()));

	// 	return InertialData::FromUrdf(m, com, I_com_l);
	// }

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

	void UrdfLoader::reset_thunder_chain(size_t size) {
		numJoints = size;
		ndof = 0;
		jointsName.resize(size);
		jointsParent.resize(size);
		jointsType.resize(size, "FIXED");
		jointsDimension.resize(size, 0);
		jointsAvailable.resize(size, false);
		jointsDerivatives.resize(size, false);
		jointsAxis.resize(size, {0,0,1});
		par_KIN_num.resize(KIN_DIM*size, 0);
		par_DYN_num.resize(DYN_DIM*size, 0);
	}

	void UrdfLoader::add_chain_from(int& link_id, int parent, std::shared_ptr<urdf::Link> link) {
		if (link_id < numJoints) {
			// - Fill thunder structures - //
			jointsName[link_id] = link->name;
			jointsParent[link_id] = parent;
			// casadi::Slice par_kin_idx(link_id*KIN_DIM, (link_id+1)*KIN_DIM);
			// casadi::Slice par_dyn_idx(link_id*DYN_DIM, (link_id+1)*DYN_DIM);
			// par_DYN_num(par_dyn_idx) = extractInertiaFromLink(link);
			auto par_DYN_link = extractInertiaFromLink(link);
			for(int j=0; j<DYN_DIM; j++) par_DYN_num[link_id*DYN_DIM+j] = static_cast<double>(par_DYN_link(j,0));

			if (link->child_joints.empty()) {		// link is an end-effector
				jointsAvailable[link_id] = true;
				jointsDerivatives[link_id] = true;
			} else {								// have to explore deeply
				// add inertials and link-joint properties
				int new_parent = link_id;
				int i = 0;
				for (auto joint : link->child_joints) {
					if (i > 0) {
						// Initialize ghost node for branches
						link_id++;
						jointsName[link_id] = link->name + "_" + std::to_string(i);
						jointsParent[link_id] = parent; // Attached to same parent as main node
					}
					
					// par_kin_idx = casadi::Slice((link_id)*KIN_DIM, (link_id+1)*KIN_DIM);
					// par_KIN_num[par_kin_idx] = extractKinematicsFromJoint(joint);
					auto par_KIN_link = extractKinematicsFromJoint(joint);
					for(int j=0; j<KIN_DIM; j++) par_KIN_num[link_id*KIN_DIM+j] = static_cast<double>(par_KIN_link(j,0));
					add_joint(link_id, joint);
					add_chain_from(++link_id, new_parent, urdf_model->getLink(joint->child_link_name));
					i++;
				}
			}
		} else {
			std::cerr << "Link out of range: " << link->name << endl;
			return;
		}
	}

	void UrdfLoader::add_joint(int link_id, std::shared_ptr<urdf::Joint> joint) {
		switch (joint->type) {
			case urdf::JointType::REVOLUTE:
				jointsType[link_id] = "R";
				jointsDimension[link_id] = 1;
				jointsAxis[link_id] = {joint->axis.x(), joint->axis.y(), joint->axis.z()};
				break;
			case urdf::JointType::CONTINUOUS:
				jointsType[link_id] = "R";
				jointsDimension[link_id] = 1;
				jointsAxis[link_id] = {joint->axis.x(), joint->axis.y(), joint->axis.z()};
				break;
			case urdf::JointType::PRISMATIC:
				jointsType[link_id] = "P";
				jointsDimension[link_id] = 1;
				jointsAxis[link_id] = {joint->axis.x(), joint->axis.y(), joint->axis.z()};
				break;
			case urdf::JointType::FLOATING:
				jointsType[link_id] = "F";
				// panic
				break;
			case urdf::JointType::PLANAR:
				jointsType[link_id] = "XY";
				// panic
				break;
			default:
				debug_log("Detected non-standard joint type for joint '" + joint->name + "'", VERB_INFO);
				jointsType[link_id] = "UNKNOWN";
				break;
		}
		ndof += jointsDimension[link_id];
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
			// std::vector<std::shared_ptr<urdf::Joint>> active_joints;
			// std::vector<casadi::SX> static_transforms; // Transform from parent joint to current joint
			// std::vector<InertialData> active_bodies;
			
			casadi::SX current_cumulative_transform = casadi::SX::eye(4);
			
			debug_log("Chain size: " + std::to_string(chain.size()), VERB_DEBUG);

			// ----- modified from here ----------------------------------------------------- //
			int numJoints = chain.size();
			// create thunder structures
			reset_thunder_chain(numJoints);
			int link_id = 0;
			add_chain_from(link_id, -1, urdf_model->getLink(base_link_name));
			
			// for (size_t i=0; i<numJoints; i++) {
			// 	auto link = chain[i];
			// 	// handle world
			// 	if (link == "world") {
			// 		jointsParent[i] = -1;
			// 	} else {
			// 		// the interested link is the one before each joints
			// 		auto parent_joint = link.parent_joint(); 	// parent joint
			// 		int parent_id = 0;
			// 		for (size_t i=0; i<numJoints; i++) if (parent_joint.parent.name == chain[i].name) parent_id = i; // maybe could be done better
			// 		if (link.child_joint() == nullptr) {		// no child means link is an end-effector
			// 			jointsName[i] = link_name;
			// 			jointsDerivatives[i] = true;
			// 			jointsAvailable[i] = true;
			// 			// have to identify parent id (the for above), if needed use jointsParentStr ------------------------!
			// 			jointsParent[i] = parent_id;
			// 		}
			// 		auto link = parent_joint.parent_link();

			// 	}
			// }
			// ----- modified to here ------------------------------------------------------- //

			// for (size_t i = 0; i < chain.size() - 1; i++) {
			// 	auto parent = chain[i]; // parent link
			// 	auto child = chain[i+1]; // child link

			// 	std::shared_ptr<urdf::Joint> joint;
			// 	for (auto& j : parent->child_joints) { // child joints of parent link   
			// 		if (j->child_link_name == child->name) { // child link name matches child joint name
			// 			joint = j;
			// 			break;
			// 		}
			// 	}

			// 	if (!joint) {
			// 		std::cerr << "Joint connecting " << parent->name << " to " << child->name << " not found." << std::endl;
			// 		return robot;
			// 	}

			// 	// Extract inertia of the child link (relative to its origin)
			// 	InertialData child_raw_inertia = extractInertiaFromLink(child);
			// 	casadi::SX joint_transform = to_casadi_sx(joint->parent_to_joint_transform);

			// 	if (joint->type == urdf::JointType::FIXED) {
			// 		// --- ACCUMULATION (FIXED JOINT) ---
			// 		// Accumulate kinematic transformation
			// 		current_cumulative_transform = casadi::SX::mtimes(current_cumulative_transform, joint_transform);
					
			// 		// Transform child inertia to the current "rigid block" frame
			// 		InertialData child_transformed = transformInertia(child_raw_inertia, current_cumulative_transform);
					
			// 		// Merge inertia into the current block
			// 		if (!active_bodies.empty()) {
			// 			active_bodies.back() = mergeInertia(active_bodies.back(), child_transformed);
			// 		}
			// 	} else {
			// 		// --- NEW ACTIVE JOINT ---
			// 		active_joints.push_back(joint);
					
			// 		// Save joint kinematics
			// 		static_transforms.push_back(casadi::SX::mtimes(current_cumulative_transform, joint_transform));
					
			// 		// Reset: next active link starts with "clean" inertia
			// 		current_cumulative_transform = casadi::SX::eye(4);
			// 		active_bodies.push_back(child_raw_inertia);
					
			// 	}
			// }

			// int numJoints = active_joints.size();
			// int ndof = active_joints.size();
			// robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			// debug_log("Detected " + std::to_string(numJoints) + " active joints", VERB_DEBUG); 

			// std::vector<std::string> jointsType;
			// std::vector<double> jointsAxis;
			// int j_idx = 0;
			// for (auto& j : active_joints) {

			// 	switch (j->type) {
			// 		case urdf::JointType::REVOLUTE:
			// 			jointsType.push_back("R");
			// 			jointsAxis.push_back(j->axis.x());
			// 			jointsAxis.push_back(j->axis.y());
			// 			jointsAxis.push_back(j->axis.z());
			// 			break;
			// 		case urdf::JointType::CONTINUOUS:
			// 			jointsType.push_back("R");
			// 			jointsAxis.push_back(j->axis.x());
			// 			jointsAxis.push_back(j->axis.y());
			// 			jointsAxis.push_back(j->axis.z());
			// 			break;
			// 		case urdf::JointType::PRISMATIC:
			// 			jointsType.push_back("P");
			// 			jointsAxis.push_back(j->axis.x());
			// 			jointsAxis.push_back(j->axis.y());
			// 			jointsAxis.push_back(j->axis.z());
			// 			break;
			// 		case urdf::JointType::FLOATING:
			// 			jointsType.push_back("F");
			// 			// panic
			// 			break;
			// 		case urdf::JointType::PLANAR:
			// 			jointsType.push_back("XY");
			// 			// panic
			// 			break;
			// 		default:
			// 			debug_log("Detected non-standard joint type for joint '" + j->name + "'", VERB_INFO);
			// 			jointsType.push_back("UNKNOWN");
			// 			break;
			// 	}
			// 	j_idx++;
			// }
			// robot->add_property<std::vector<std::string>>("jointsType", jointsType, "vector<string>", "Type of joints", true);
			// robot->add_parameter("par_jointsAxis", casadi::SX::sym("jointsAxis", 3*numJoints), jointsAxis, std::vector<short>(3 * numJoints, 0), "Joint axes", true);	
			// debug_log("Joints types: ", VERB_DEBUG);
			// for (auto& t : jointsType) {
			// 	debug_log(" - " + t, VERB_DEBUG);
			// }

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
			std::vector<short> par_KIN_isSymb(6 * numJoints, 0);

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
			std::vector<short> par_DYN_isSymb(STD_PAR_LINK * numJoints, 0);

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
