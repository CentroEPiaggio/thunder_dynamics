#include "plugins/builders/kin_builder.h"
#include "utils.h"

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns {

	constexpr double MU = 0.02; //pseudo-inverse damping coeff

    int KinBuilder::create_std_joints(std::shared_ptr<Robot> robot){
		// --- Standard joint functions --- //
		SX q_joint = SX::sym("q_joint");
		SX axis = SX::sym("axis",3,1);
		FunArg q_joint_arg("q_joint", q_joint);
		FunArg axis_arg("axis", axis);
		casadi::Slice rot(0, 3);      // [0,1,2] indexes
		SX Ti = SX::eye(4);

		// // Prismatic classical joint
		// Ti = SX::eye(4);
		// Ti(2,3) = q_joint;
		// if (!robot->add_function("T_JOINT_P", Ti, {}, "Template transformation of joint P", {q_joint})) {
		// 	std::cerr << "Error adding joint function: T_JOINT_P" << std::endl;
		// 	return 0;
		// }

		// // Rotoidal classical joint
		// Ti = SX::eye(4);
		// Ti(rot,rot) = R_z(q_joint);
		// if (!robot->add_function("T_JOINT_R", Ti, {}, "Template transformation of joint R", {q_joint})) {
		// 	std::cerr << "Error adding joint function: T_JOINT_R" << std::endl;
		// 	return 0;
		// }

		// Rotoidal general axis joint
		Ti = SX::eye(4);
		Ti(rot,rot) = R_aa(axis, q_joint);
		if (!robot->add_function("T_JOINT_R", Ti, {}, "Template transformation of general rotoidal joint R", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: T_JOINT_R" << std::endl;
			return 0;
		}

		// Prismatic general axis joint
		Ti = SX::eye(4);
		Ti(casadi::Slice(0,3),3) = casadi::SX::mtimes(axis, q_joint);
		if (!robot->add_function("T_JOINT_P", Ti, {}, "Template transformation of general prismatic joint R", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: T_JOINT_R" << std::endl;
			return 0;
		}

		// Fixed joint
		Ti = SX::eye(4);
		if (!robot->add_function("T_JOINT_FIXED", Ti, {}, "Template transformation of a fixed joint", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: T_JOINT_FIXED" << std::endl;
			return 0;
		}

		return 1;
	}

	SX KinBuilder::apply_joint(std::shared_ptr<Robot> robot, const SX& frame, string joint_type, const SX& qi, const SX& axis){
		SX T(4,4);
		T = casadi::SX::mtimes(get_transform_rpy(frame), robot->get_model("T_JOINT_"+joint_type, {qi, axis}));
		return T;
	}
	
	int KinBuilder::compute_chain(std::shared_ptr<Robot> robot) {

		// parameters from robot
		auto numJoints = robot->get<int>("numJoints");
		vector<string> jointsType = robot->get<vector<string>>("jointsType");
		vector<string> jointsName = robot->get<vector<string>>("jointsName");
		vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		vector<bool> jointsAvailable = robot->get<vector<bool>>("jointsAvailable");
		vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");
		if (!robot->parameters.count("par_jointsAxis")){
			debug_log("par_jointsAxis not defined in robot parameters, using Z axis for all joints", VERB_INFO);
			vector<double> jointsAxis(3* numJoints);
			for (int i=0; i<numJoints; i++){
				jointsAxis[3*i + 2] = 1.0;
			}
			robot->add_parameter("par_jointsAxis", casadi::SX::sym("jointsAxis", 3*numJoints), jointsAxis, {0}, "Joint axes", true);
		}
		auto jointsAxis = robot->get_model("par_jointsAxis");

		auto q = robot->get_model("q");
		auto par_KIN = robot->get_model("par_KIN");
		// auto par_world2L0 = robot->get_model("par_world2L0");
		// auto par_Ln2EE = robot->get_model("par_Ln2EE");

		// computing chain
		casadi::SXVector Ti(numJoints+1);    // Output
		casadi::SXVector Twi(numJoints+2);   // Output
		casadi::Slice allCols(0,4);   
	   
		// Ti is transformation from link i-1 to link i
		// Ti[0] = get_transform_ypr(par_world2L0);
		// Twi is transformation from world 0 to link i
		// Twi[0] = Ti[0];

		std::vector<std::string> arg_list;
		// if (!robot->add_function("T_0", Ti[0], arg_list, "relative transformation from frame world to base")) return 0;
		// arg_list = {"par_world2L0"};
		// if (!robot->add_function("T_w_0", Twi[0], arg_list, "absolute transformation from frame world to base")) return 0;

		for (int i = 0, dof_count=0; i < numJoints; i++) {
			casadi::SX frame = par_KIN(casadi::Slice(i*6, 6+i*6));
			auto axis = jointsAxis(casadi::Slice(i*3, i*3+3));
			int dim = jointsDimension[i];
			SX q_joint = q(casadi::Slice(dof_count, dof_count+dim));	// if dim == 0 Slice have dimension 1, but it do not interfere
			dof_count += dim;
			Ti[i] = apply_joint(robot, frame, jointsType[i], q_joint, axis);
			// std::cout << "axis: " << axis << std::endl;
			// std::cout << "q_joint: " << q_joint << std::endl;
			// std::cout << "dim: " << dim << std::endl;
			// std::cout << "Ti[i]: " << Ti[i] << std::endl;

			int parent_id = jointsParent[i];
			if (parent_id == -1) {
				Twi[i] = Ti[i];
			} else {
				Twi[i] = casadi::SX::mtimes({Twi[parent_id], Ti[i]});
			}
			
			// add functions
			arg_list = {"q", "par_KIN"};
			if (!robot->add_function("T_"+std::to_string(i), Ti[i], arg_list, "relative transformation from frame"+ std::to_string(i-1) +"to frame "+std::to_string(i))) return 0;
			if (!robot->add_function("T_w_"+std::to_string(i), Twi[i], arg_list, "absolute transformation from frame world to frame "+std::to_string(i))) return 0;
			if (jointsAvailable[i]) {
				if (!robot->add_function("T_w_"+jointsName[i], Twi[i], arg_list, "absolute transformation from frame world to frame "+jointsName[i])) return 0;
			}
		}

		// // end-effector transform
		// Twi[numJoints+1] = casadi::SX::mtimes({Twi[numJoints], get_transform_ypr(par_Ln2EE)});

		// arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
		// if (!robot->add_function("T_w_"+std::to_string(numJoints+1), Twi[numJoints+1], arg_list, "absolute transformation from frame base to end_effector")) return 0;
		// if (!robot->add_function("T_w_ee", Twi[numJoints+1], arg_list, "absolute transformation from frame 0 to end_effector")) return 0;
		// // std::cout<<"functions created"<<std::endl;

		return 1;
	}
 
	int KinBuilder::compute_jacobians(std::shared_ptr<Robot> robot) {

		// parameters from robot
		int nj = robot->get<int>("numJoints");
		int ndof = robot->get<int>("ndof");
		vector<string> jointsName = robot->get<vector<string>>("jointsName");
		vector<bool> jointsAvailable = robot->get<vector<bool>>("jointsAvailable");
		
		auto q = robot->get_model("q");
		auto par_KIN = robot->get_model("par_KIN");
		// auto par_world2L0 = robot->get_model("par_world2L0");
		// auto par_Ln2EE = robot->get_model("par_Ln2EE");
		
		// computing jacobians
		casadi::SXVector Ji_v(nj);   // vector of matrix Ji_v + EE
		casadi::SXVector Ji_w(nj);   // vector of matrix Ji_w + EE
		casadi::SXVector Ji(nj);		// complete jacobian
		casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
		casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
		casadi::Slice allRows;              // Select all rows
		// auto world_rot = get_transform_ypr(par_world2L0)(r_rot_idx, r_rot_idx);

		for (int i = 0; i < nj; i++) {
			SX T_wi = robot->get_model("T_w_"+std::to_string(i));

			SX d_0_i = T_wi(r_tra_idx, 3);
			SX R_0_i = T_wi(r_rot_idx, r_rot_idx);
			SX Ji_pos = SX::jacobian(d_0_i, q);
			SX Ji_or(3, ndof);

			// Loop over joints and build columns
			for (int j=0; j<ndof; ++j) {
				// Partial derivative dR/dq_j  (3x3)
				SX dR_dqj = SX::jacobian(SX::reshape(R_0_i, 9, 1), q(j));
				dR_dqj = SX::reshape(dR_dqj, 3, 3);

				// S_j = dR/dq_j * R^T  (3x3 skew-symmetric)
				SX Sj = SX::mtimes(dR_dqj, R_0_i.T());

				// Extract angular velocity vector from skew matrix
				SX wj = vect(Sj);

				// Set column j
				Ji_or(allRows, j) = wj;
			}

			// // Add end-effector transformation (only in the EE Jacobian)
			// if(i==nj){
			// 	casadi::SX R0i = T_wi(r_rot_idx,r_rot_idx);
			// 	casadi::SX ee_tr = get_transform_ypr(par_Ln2EE)(r_tra_idx,3);
			// 	Ji_pos = Ji_pos - casadi::SX::mtimes({R0i,hat(ee_tr),R0i.T(),Ji_or});
			// 	// Ji_pos = Ji_pos - casadi::SX::mtimes(hat(ee_tr), Ji_or);
			// 	// std::cout << "Ji_pos_ee: " << Ji_pos << std::endl;
			// }

			Ji_v[i] = Ji_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			std::vector<std::string> arg_list;
			// if (i<nj){
			// 	arg_list = {"q", "par_KIN", "par_world2L0"};
			// } else {
			// 	arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
			// }
			arg_list = {"q", "par_KIN"};
			if (!robot->add_function("J_"+std::to_string(i), Ji[i], arg_list, "Jacobian of frame "+std::to_string(i))) return 0;
			if (jointsAvailable[i]){
				if (!robot->add_function("J_"+jointsName[i], Ji[i], arg_list, "Jacobian of frame "+jointsName[i])) return 0;
			}
		}

		// std::vector<std::string> arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
		// if (!robot->add_function("J_ee", Ji[nj], arg_list, "Jacobian of the end-effector")) return 0;
		return 1;
	}

	int KinBuilder::compute_kin_adv(std::shared_ptr<Robot> robot){
		auto nj = robot->get<int>("numJoints");
		vector<bool> jointsDerivatives = robot->get<vector<bool>>("jointsDerivatives");
		vector<string> jointsName = robot->get<vector<string>>("jointsName");

		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto ddq = robot->get_model("ddq");

		// jacobian derivatives
		for (int i=0; i<nj; i++){
			if (jointsDerivatives[i]){
				auto Jn = robot->get_model("J_"+jointsName[i]);
				casadi::SX dJn = casadi::SX::jtimes(Jn,q,dq);
				casadi::SX ddJn = casadi::SX::jtimes(dJn,q,dq) + casadi::SX::jtimes(dJn,dq,ddq);
				std::vector<std::string> arg_list = {"q", "dq", "par_KIN"};
				robot->add_function("J_"+jointsName[i]+"_dot", dJn, arg_list, "Time derivative of jacobian matrix of frame "+jointsName[i]);
				arg_list = {"q", "dq", "ddq", "par_KIN"};
				robot->add_function("J_"+jointsName[i]+"_ddot", ddJn, arg_list, "Time second derivative of jacobian matrix of frame "+jointsName[i]);

				// jacobian inverse
				casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn,Jn.T()}) + casadi::SX::eye(6)*MU);
				casadi::SX pinvJn = casadi::SX::mtimes({Jn.T(),invJn_dumped});
				arg_list = {"q", "par_KIN"};
				robot->add_function("J_"+jointsName[i]+"_pinv", pinvJn, arg_list, "Pseudo-Inverse of jacobian matrix of frame "+jointsName[i]);
			}
		}

		return 1;
	}

    void KinBuilder::build(std::shared_ptr<Robot> robot) {
        create_std_joints(robot);
				
		debug_log("Starting kinematic computations", VERB_INFO);

		int ret = 1;
		if (!compute_chain(robot)) ret = 0;
		if (!compute_jacobians(robot)) ret = 0;
		if (1){
			if (!compute_kin_adv(robot)) ret = 0;
		}

		// return ret;
		debug_log("Kinematics computed", VERB_INFO);
    }

}