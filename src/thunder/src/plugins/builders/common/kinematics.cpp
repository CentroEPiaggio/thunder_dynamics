#include "plugins/builders/common/kinematics.h"
#include "plugins/loaders/dh_loader.h"
#include "utils.h"

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns{

	constexpr double MU = 0.02; //pseudo-inverse damping coeff

	SX apply_joint(Robot& robot, const SX& frame, string joint_type, const SX& qi){
		SX T(4,4);
		T = casadi::SX::mtimes(get_transform_rpy(frame), robot.get_model("T_JOINT_"+joint_type, {qi}));
		return T;
	}
	
	int compute_chain(Robot& robot) {

		// parameters from robot
		auto numJoints = robot.get<int>("numJoints");
		vector<string> jointsType = robot.get<vector<string>>("jointsType");
		auto q = robot.get_model("q");
		// auto par_DHtable = robot.get_model("par_DHtable");
		auto par_KIN = robot.get_model("par_KIN");
		auto par_world2L0 = robot.get_model("par_world2L0");
		auto par_Ln2EE = robot.get_model("par_Ln2EE");

		// computing chain
		casadi::SXVector Ti(numJoints+1);    // Output
		casadi::SXVector T0i(numJoints+2);   // Output
		casadi::Slice allCols(0,4);   
	   
		// Ti is transformation from link i-1 to link i
		Ti[0] = get_transform_ypr(par_world2L0);
		// T0i is transformation from link 0 to link i
		T0i[0] = Ti[0];

		std::vector<std::string> arg_list = {"par_world2L0"};
		if (!robot.add_function("T_0", Ti[0], arg_list, "relative transformation from frame base to frame 1")) return 0;
		arg_list = {"par_world2L0"};
		if (!robot.add_function("T_0_0", T0i[0], arg_list, "absolute transformation from frame base to frame 1")) return 0;

		for (int i = 0; i < numJoints; i++) {
			// casadi::Slice row_i(i*4, i*4+4);
			casadi::SX frame = par_KIN(casadi::Slice(i*6, 6+i*6));
			// Ti[i+1] = DHTemplate(par_DHtable(row_i), q(i), jointsType[i]);
			// Ti[i+1] = get_T_Joint(i, jointsType[i]);
			Ti[i+1] = apply_joint(robot, frame, jointsType[i], q(i));
			// Ti[i+1] = casadi::SX::mtimes(get_transform_rpy(frame), robot.get_model("T_JOINT_"+std::to_string(i)));
			T0i[i+1] = casadi::SX::mtimes({T0i[i], Ti[i+1]});
			
			arg_list = {"q", "par_KIN"};
			if (!robot.add_function("T_"+std::to_string(i+1), Ti[i+1], arg_list, "relative transformation from frame"+ std::to_string(i) +"to frame "+std::to_string(i+1))) return 0;
			arg_list = {"q", "par_KIN", "par_world2L0"};
			if (!robot.add_function("T_0_"+std::to_string(i+1), T0i[i+1], arg_list, "absolute transformation from frame base to frame "+std::to_string(i+1))) return 0;
		}

		// end-effector transform
		T0i[numJoints+1] = casadi::SX::mtimes({T0i[numJoints], get_transform_ypr(par_Ln2EE)});

		arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
		if (!robot.add_function("T_0_"+std::to_string(numJoints+1), T0i[numJoints+1], arg_list, "absolute transformation from frame base to end_effector")) return 0;
		if (!robot.add_function("T_0_ee", T0i[numJoints+1], arg_list, "absolute transformation from frame 0 to end_effector")) return 0;
		// std::cout<<"functions created"<<std::endl;

		return 1;
	}
 
	int compute_jacobians(Robot& robot) {

		// parameters from robot
		int nj = robot.get<int>("numJoints");
		// vector<string> jointsType = robot.get<vector<string>>("jointsType");
		auto q = robot.get_model("q");
		auto par_KIN = robot.get_model("par_KIN");
		auto par_world2L0 = robot.get_model("par_world2L0");
		auto par_Ln2EE = robot.get_model("par_Ln2EE");
		
		// computing jacobians
		casadi::SXVector Ji_v(nj+1);   // vector of matrix Ji_v + EE
		casadi::SXVector Ji_w(nj+1);   // vector of matrix Ji_w + EE
		casadi::SXVector Ji(nj+1);		// complete jacobian
		casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
		casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
		casadi::Slice allRows;              // Select all rows
		// auto world_rot = get_transform_ypr(par_world2L0)(r_rot_idx, r_rot_idx);

		for (int i = 0; i <= nj; i++) {
			int i_mod = (i<nj)?i:(nj-1);

			SX T_0i = robot.get_model("T_0_"+std::to_string(i_mod+1));
			
			SX d_0_i = T_0i(r_tra_idx, 3);
			SX R_0_i = T_0i(r_rot_idx, r_rot_idx);
			SX Ji_pos = SX::jacobian(d_0_i, q);
			SX Ji_or(3, nj);

			// Loop over joints and build columns
			for (int j=0; j<nj; ++j) {
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
			
			// Add end-effector transformation (only in the EE Jacobian)
			if(i==nj){
				casadi::SX R0i = T_0i(r_rot_idx,r_rot_idx);
				casadi::SX ee_tr = get_transform_ypr(par_Ln2EE)(r_tra_idx,3);
				Ji_pos = Ji_pos - casadi::SX::mtimes({R0i,hat(ee_tr),R0i.T(),Ji_or});
				// Ji_pos = Ji_pos - casadi::SX::mtimes(hat(ee_tr), Ji_or);
				// std::cout << "Ji_pos_ee: " << Ji_pos << std::endl;
			}
			
			Ji_v[i] = Ji_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			std::vector<std::string> arg_list;
			if (i<nj){
				arg_list = {"q", "par_KIN", "par_world2L0"};
			} else {
				arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
			}
			if (!robot.add_function("J_"+std::to_string(i+1), Ji[i], arg_list, "Jacobian of frame "+std::to_string(i+1))) return 0;
		}

		std::vector<std::string> arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
		if (!robot.add_function("J_ee", Ji[nj], arg_list, "Jacobian of the end-effector")) return 0;
		return 1;
	}

	int compute_kin_adv(Robot& robot){
		auto q = robot.get_model("q");
		auto dq = robot.get_model("dq");
		auto ddq = robot.get_model("ddq");
		auto Jn = robot.get_model("J_ee");

		// jacobian derivatives
		casadi::SX dJn = casadi::SX::jtimes(Jn,q,dq);
		casadi::SX ddJn = casadi::SX::jtimes(dJn,q,dq) + casadi::SX::jtimes(dJn,dq,ddq);
		std::vector<std::string> arg_list = {"q", "dq", "par_KIN", "par_world2L0", "par_Ln2EE"};
		robot.add_function("J_ee_dot", dJn, arg_list, "Time derivative of jacobian matrix");
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_world2L0", "par_Ln2EE"};
		robot.add_function("J_ee_ddot", ddJn, arg_list, "Time second derivative of jacobian matrix");

		// jacobian inverse
		casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn,Jn.T()}) + casadi::SX::eye(6)*MU);
		casadi::SX pinvJn = casadi::SX::mtimes({Jn.T(),invJn_dumped});
		arg_list = {"q", "par_KIN", "par_world2L0", "par_Ln2EE"};
		robot.add_function("J_ee_pinv", pinvJn, arg_list, "Pseudo-Inverse of jacobian matrix");

		return 1;
	}

	// compute everything
	int compute_kinematics(Robot& robot, bool advanced){
		if (!compute_chain(robot)) return 0;
		if (!compute_jacobians(robot)) return 0;
		if (advanced){
			if (!compute_kin_adv(robot)) return 0;
		}

		return 1;
	}

}