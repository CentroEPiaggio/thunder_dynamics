#include "plugins/builders/kin_builder.h"
#include "utils.h"

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns {

	constexpr double MU = 0.02; //pseudo-inverse damping coeff

	namespace {
		const casadi::Slice lin(0,3), ang(3,6), all;

		// Solves A X = B with A symmetric positive definite, by LDL^T without pivoting (no sqrt, smaller than SX::inv)
		SX ldl_solve(const SX& A, const SX& B){
			const int n = A.size1();
			SX L = SX::eye(n), D = SX::zeros(n,1);
			for (int j=0; j<n; j++){
				D(j) = A(j,j);
				for (int m=0; m<j; m++) D(j) -= L(j,m)*L(j,m)*D(m);
				for (int i=j+1; i<n; i++){
					SX lij = A(i,j);
					for (int m=0; m<j; m++) lij -= L(i,m)*L(j,m)*D(m);
					L(i,j) = lij / D(j);
				}
			}
			SX X = B;
			for (int i=0; i<n; i++) for (int m=0; m<i; m++) X(i,all) -= L(i,m)*X(m,all);
			for (int i=0; i<n; i++) X(i,all) /= D(i);
			for (int i=n-1; i>=0; i--) for (int m=i+1; m<n; m++) X(i,all) -= L(m,i)*X(m,all);
			return X;
		}

		// Damped pseudo-inverse J^T (J J^T + mu I)^-1 = (J^T J + mu I)^-1 J^T, solved on the smaller of the two matrices.
		// Both are positive definite for mu > 0, so LDL^T needs no pivoting.
		SX damped_pinv(const SX& J, double mu){
			if (J.size2() < J.size1()) return ldl_solve(SX::mtimes(J.T(), J) + SX::eye(J.size2())*mu, J.T());
			return ldl_solve(SX::mtimes(J, J.T()) + SX::eye(J.size1())*mu, J).T();
		}
	}

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

		// Motion subspaces ([linear; angular] twist per unit joint velocity, in the joint frame), used by the rnea dynamics
		SX S_R = SX::vertcat({SX::zeros(3,1), axis / SX::norm_2(axis)});	// R_aa normalises the axis
		if (!robot->add_function("S_JOINT_R", S_R, {}, "Motion subspace of the general rotoidal joint R", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: S_JOINT_R" << std::endl;
			return 0;
		}
		SX S_P = SX::vertcat({axis, SX::zeros(3,1)});
		if (!robot->add_function("S_JOINT_P", S_P, {}, "Motion subspace of the general prismatic joint P", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: S_JOINT_P" << std::endl;
			return 0;
		}

		// Fixed joint (no motion subspace, dimension 0)
		Ti = SX::eye(4);
		if (!robot->add_function("T_JOINT_FIXED", Ti, {}, "Template transformation of a fixed joint", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: T_JOINT_FIXED" << std::endl;
			return 0;
		}

		return 1;
	}

	int KinBuilder::derive_subspaces(std::shared_ptr<Robot> robot){
		// S_JOINT_<type> for the joint types that define only T_JOINT_<type> (Jacobians and rnea need it):
		// column k is the body twist of dT/dq_k, S = vee(T^-1 dT/dq). Exact, but trigonometric identities are
		// not simplified, so it can be a q-dependent expression that is numerically constant (larger code).
		vector<string> jointsType = robot->get<vector<string>>("jointsType");
		vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");
		casadi::Slice sel3(0,3);
		for (int i=0; i<(int)jointsType.size(); i++){
			const string type = jointsType[i];
			if ((jointsDimension[i] == 0) || robot->functions.count("S_JOINT_"+type)) continue;
			if (!robot->functions.count("T_JOINT_"+type)) {
				std::cerr << "Joint type '" << type << "' has neither S_JOINT_ nor T_JOINT_ defined" << std::endl;
				return 0;
			}
			debug_log("S_JOINT_" + type + " not defined, derived from T_JOINT_" + type, VERB_DEBUG);
			const auto& T_joint = robot->functions["T_JOINT_"+type];		// explicit args (q_joint, axis)
			SX T = T_joint.expr;
			SX q_joint = T_joint.explicit_args[0].value;
			SX R = T(sel3,sel3);
			SX S(6, q_joint.size1());
			for (int k=0; k<q_joint.size1(); k++){
				SX dR = SX::reshape(SX::jacobian(SX::reshape(R, 9, 1), q_joint(k)), 3, 3);
				S(lin, k) = SX::mtimes(R.T(), SX::jacobian(T(sel3,3), q_joint(k)));
				S(ang, k) = vect(SX::mtimes(R.T(), dR));
			}
			if (!robot->add_function("S_JOINT_"+type, S, {}, "Motion subspace of joint "+type+", derived from T_JOINT_"+type, T_joint.explicit_args)) return 0;
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
		vector<vector<double>> jointsAxis = robot->get<vector<vector<double>>>("jointsAxis");

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
			auto axis = jointsAxis[i];
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
		// Geometric Jacobians, [linear velocity of the frame origin; angular velocity], both in world coordinates.
		// Joint j (frame j or one of its ancestors) gives the columns [L_j + z_j x (p_i - p_j); z_j] of frame i,
		// with [L_j; z_j] = R_w_j S_j its motion subspace in world coordinates. Derivation in docs/plugins/kin_builder_plugin.md.

		// parameters from robot
		const int nj = robot->get<int>("numJoints");
		const int ndof = robot->get<int>("ndof");
		vector<string> jointsName = robot->get<vector<string>>("jointsName");
		vector<string> jointsType = robot->get<vector<string>>("jointsType");
		vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");
		vector<vector<double>> jointsAxis = robot->get<vector<vector<double>>>("jointsAxis");
		vector<bool> jointsAvailable = robot->get<vector<bool>>("jointsAvailable");
		vector<bool> jointsDerivatives = robot->get<vector<bool>>("jointsDerivatives");

		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto ddq = robot->get_model("ddq");

		// --- Frames and joint subspaces in world coordinates, frame velocities --- //
		casadi::SXVector p(nj), Sw(nj);		// origin of frame i, motion subspace of joint i (6 x dim)
		casadi::SXVector w(nj), v(nj), dSw(nj);	// angular velocity and origin velocity of frame i, d/dt Sw (for J_dot)
		vector<casadi::Slice> qi(nj);		// entries of joint i in q
		for (int i=0, dof_count=0; i<nj; i++){
			const int dim = jointsDimension[i];
			qi[i] = casadi::Slice(dof_count, dof_count + dim);
			dof_count += dim;

			SX T_wi = robot->get_model("T_w_"+std::to_string(i));
			SX R = T_wi(lin, lin);
			p[i] = T_wi(lin, 3);

			const int pa = jointsParent[i];
			SX w_p = (pa < 0) ? SX::zeros(3,1) : w[pa];
			SX v_p = (pa < 0) ? SX::zeros(3,1) : v[pa];
			SX p_p = (pa < 0) ? SX::zeros(3,1) : p[pa];
			w[i] = w_p;
			v[i] = v_p + SX::cross(w_p, p[i] - p_p);
			if (dim == 0) continue;

			SX S = robot->get_model("S_JOINT_"+jointsType[i], {q(qi[i]), SX(casadi::DM(jointsAxis[i]))});
			Sw[i] = SX::vertcat({SX::mtimes(R, S(lin, all)), SX::mtimes(R, S(ang, all))});
			w[i] += SX::mtimes(Sw[i](ang, all), dq(qi[i]));
			v[i] += SX::mtimes(Sw[i](lin, all), dq(qi[i]));
			// d/dt (R S) = w x (R S) + R dS/dt, the jtimes is zero for constant S (R, P joints)
			SX dS = SX::jtimes(S, q, dq);
			dSw[i] = SX::vertcat({SX::mtimes(hat(w[i]), Sw[i](lin, all)) + SX::mtimes(R, dS(lin, all)),
			                      SX::mtimes(hat(w[i]), Sw[i](ang, all)) + SX::mtimes(R, dS(ang, all))});
		}

		// --- Jacobians of each frame, derivatives and pseudo-inverse where asked --- //
		for (int i = 0; i < nj; i++) {
			SX J = SX::zeros(6, ndof);
			for (int j=i; j>=0; j=jointsParent[j]){
				if (jointsDimension[j] == 0) continue;
				J(lin, qi[j]) = Sw[j](lin, all) + SX::mtimes(hat(p[j] - p[i]), Sw[j](ang, all));
				J(ang, qi[j]) = Sw[j](ang, all);
			}

			std::vector<std::string> arg_list = {"q", "par_KIN"};
			if (!robot->add_function("J_"+std::to_string(i), J, arg_list, "Jacobian of frame "+std::to_string(i))) return 0;
			if (jointsAvailable[i]){
				if (!robot->add_function("J_"+jointsName[i], J, arg_list, "Jacobian of frame "+jointsName[i])) return 0;
			}
			if (!jointsDerivatives[i]) continue;

			// time derivative of the columns of J
			SX dJ = SX::zeros(6, ndof);
			for (int j=i; j>=0; j=jointsParent[j]){
				if (jointsDimension[j] == 0) continue;
				dJ(lin, qi[j]) = dSw[j](lin, all) + SX::mtimes(hat(p[j] - p[i]), dSw[j](ang, all)) + SX::mtimes(hat(v[j] - v[i]), Sw[j](ang, all));
				dJ(ang, qi[j]) = dSw[j](ang, all);
			}
			SX ddJ = SX::jtimes(dJ, q, dq) + SX::jtimes(dJ, dq, ddq);

			arg_list = {"q", "dq", "par_KIN"};
			if (!robot->add_function("J_"+jointsName[i]+"_dot", dJ, arg_list, "Time derivative of jacobian matrix of frame "+jointsName[i])) return 0;
			arg_list = {"q", "dq", "ddq", "par_KIN"};
			if (!robot->add_function("J_"+jointsName[i]+"_ddot", ddJ, arg_list, "Time second derivative of jacobian matrix of frame "+jointsName[i])) return 0;
			arg_list = {"q", "par_KIN"};
			if (!robot->add_function("J_"+jointsName[i]+"_pinv", damped_pinv(J, MU), arg_list, "Pseudo-Inverse of jacobian matrix of frame "+jointsName[i])) return 0;
		}

		return 1;
	}

    void KinBuilder::build(std::shared_ptr<Robot> robot) {
        create_std_joints(robot);

		debug_log("Starting kinematic computations", VERB_INFO);

		int ret = 1;
		if (!derive_subspaces(robot)) ret = 0;
		if (!compute_chain(robot)) ret = 0;
		if (!compute_jacobians(robot)) ret = 0;

		// return ret;
		debug_log("Kinematics computed", VERB_INFO);
    }

}
