#include "kinematics.h"
#include "utils.h"

/* Function name used to generate code */
#define Ti_STRING "T_i"
#define T0i_STRING "T_0_i"
#define JAC_DOT_STRING "J_ee_dot"
#define JAC_PINV_STRING "J_ee_pinv"

namespace thunder_ns{

	constexpr double MU = 0.02; //pseudo-inverse damping coeff
	constexpr double EPSILON = 1e-15; // numerical resolution, below is zero
  
	casadi::SX DHTemplate(const Eigen::MatrixXd& rowDHTable, const casadi::SX qi, std::string jointType) {
		
		double a;
		double alpha;
		casadi::SX d;
		casadi::SX theta;
		
		casadi::SX ct;      // cos(theta)
		casadi::SX st;      // sin(theta)
		double ca;          // cos(alpha)
		double sa;          // sin(alpha)
		
		casadi::SX Ti(4,4); // output
		
		// Check
		if ((jointType == "P")||(jointType == "P_SEA")) {
			d = rowDHTable(2) + qi;
			theta = rowDHTable(3);
		}
		else if ((jointType == "R")||(jointType == "R_SEA")) {
			d = rowDHTable(2);
			theta = rowDHTable(3) + qi;
		}
		else {
			throw std::runtime_error("DHTemplate: Error joint type");
		}

		a = rowDHTable(0);
		alpha = rowDHTable(1);
		
		ca = cos(alpha);
		sa = sin(alpha);
		ct = cos(theta);
		st = sin(theta);

		// if (abs(ca) < EPSILON) ca = 0;
		// if (abs(sa) < EPSILON) sa = 0;
		// if (abs(ct) < EPSILON) ct = 0;
		// if (abs(st) < EPSILON) st = 0;
		
		// original: T_theta*T_d*T_alpha*T_a
		// Ti(0,0) = ct;
		// Ti(0,1) = -ca*st;
		// Ti(0,2) = sa*st;
		// Ti(0,3) = a*ct;
		// Ti(1,0) = st;
		// Ti(1,1) = ca*ct;
		// Ti(1,2) = -sa*ct;
		// Ti(1,3) = a*st;
		// Ti(2,1) = sa;
		// Ti(2,2) = ca;
		// Ti(2,3) = d;
		// Ti(3,3) = 1;

		// modified: T_a*T_alpha*T_d*T_theta
		Ti(0,0) = ct;
		Ti(0,1) = -st;
		Ti(0,2) = 0;
		Ti(0,3) = a;
		Ti(1,0) = ca*st;
		Ti(1,1) = ca*ct;
		Ti(1,2) = -sa;
		Ti(1,3) = -d*sa;
		Ti(2,0) = sa*st;
		Ti(2,1) = sa*ct;
		Ti(2,2) = ca;
		Ti(2,3) = d*ca;
		Ti(3,3) = 1;

		return Ti;
	}
	
	int compute_chain(Robot& robot) {

		// parameters from robot
		auto numJoints = robot.get_numJoints();
		auto jointsType = robot.get_jointsType();
		auto _DHtable_ = robot.get_DHTable();
		auto _world2L0_ = robot.get_world2L0();
		auto _Ln2EE_ = robot.get_Ln2EE();
		auto q = robot.model["q"];

		// computing chain
		casadi::SXVector Ti(numJoints);    // Output
		casadi::SXVector T0i(numJoints+1);   // Output
	   
		// Ti is transformation from link i-1 to link i
		Ti[0] = DHTemplate(_DHtable_.row(0), q(0), jointsType[0]);
		// T0i is transformation from link 0 to link i
		T0i[0] = casadi::SX::mtimes({_world2L0_.get_transform(), Ti[0]});
		if (!robot.add_function("T_0", Ti[0], {"q"}, "relative transformation from frame base to frame 1")) return 0;
		if (!robot.add_function("T_0_0", T0i[0], {"q"}, "absolute transformation from frame base to frame 1")) return 0;

		for (int i = 1; i < numJoints; i++) {
			Ti[i] = DHTemplate(_DHtable_.row(i), q(i), jointsType[i]);
			T0i[i] = casadi::SX::mtimes({T0i[i-1], Ti[i]});
			
			if (!robot.add_function("T_"+std::to_string(i), Ti[i], {"q"}, "relative transformation from frame"+ std::to_string(i) +"to frame "+std::to_string(i+1))) return 0;
			if (!robot.add_function("T_0_"+std::to_string(i), T0i[i], {"q"}, "absolute transformation from frame base to frame "+std::to_string(i+1))) return 0;
		}

		// end-effector transform
		T0i[numJoints] = casadi::SX::mtimes({T0i[numJoints-1], _Ln2EE_.get_transform()});

		if (!robot.add_function("T_0_"+std::to_string(numJoints), T0i[numJoints], {"q"}, "absolute transformation from frame base to end_effector")) return 0;
		if (!robot.add_function("T_0_ee", T0i[numJoints], {"q"}, "absolute transformation from frame 0 to end_effector")) return 0;
		// std::cout<<"functions created"<<std::endl;

		return 1;
	}
 
	int compute_jacobians(Robot& robot) {

		// parameters from robot
		int nj = robot.get_numJoints();
		auto jointsType = robot.get_jointsType();
		auto _world2L0_ = robot.get_world2L0();
		auto _Ln2EE_ = robot.get_Ln2EE();
		// auto _DHtable_ = robot.get_DHTable();
		auto q = robot.model["q"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		// casadi::SXVector T0i_vec(nj+1);
		// for (int i=0; i<nj; i++) {
		// 	T0i_vec[i] = robot.model["T_0_"+std::to_string(i)];
		// }
		
		// computing jacobians
		casadi::SX Ji_pos(3, nj);    // matrix of velocity jacobian
		casadi::SX Ji_or(3, nj);     // matrix of omega jacobian
		casadi::SXVector Ji_v(nj+1);   // vector of matrix Ji_v + EE
		casadi::SXVector Ji_w(nj+1);   // vector of matrix Ji_w + EE
		casadi::SXVector Ji(nj+1);		// complete jacobian
		casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
		casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
		casadi::Slice allRows;              // Select all rows

		for (int i = 0; i <= nj; i++) {
			int i_mod = (i<nj)?i:(nj-1);

			casadi::SX k0(3,1);             // versor of joint i
			casadi::SX O_0i(3,1);           // distance of joint i from joint 0
			casadi::SX T_0i(4,4);           // matrix tranformation of joint i from joint 0

			k0(2,0) = 1;
			// std::cout << "k0: " << k0 << std::endl;
			// T_0i = T0i_vec[i_mod];
			T_0i = robot.model["T_0_"+std::to_string(i_mod)];
			// std::cout << "T_0i: " << T_0i << std::endl;
			O_0i = T_0i(r_tra_idx, 3);
			// std::cout << "O_0i: " << O_0i << std::endl;

			// First column of jacobian
			if ((jointsType[0] == "P")||(jointsType[0] == "P_SEA")) {
				Ji_pos(allRows,0) = k0;
			} else if ((jointsType[0] == "R")||(jointsType[0] == "R_SEA")) {
				Ji_pos(allRows,0) = mtimes(hat(k0),O_0i);
				// std::cout << "Ji_pos: " << Ji_pos << std::endl;
				Ji_or(allRows,0) = k0;
				// std::cout << "Ji_or: " << Ji_or << std::endl;
			} else {
				throw std::runtime_error("DHJac: Error joint type");
				return 0;
			}

			// Rest of columns of jacobian
			for (int j = 1; j <= i_mod; j++) {

				// Init variables of column j-th of jacobian each cycle
				casadi::SX kj_1(3,1);             // versor of joint j-1
				casadi::SX O_j_1i(3,1);           // distance of joint i from joint j-1
				casadi::SX T_0j_1(4,4);           // matrix tranformation of joint i from joint j-1
		
				// T_0j_1 = T0i_vec[j];	// modified from T0i_vec[j-1];
				T_0j_1 = robot.model["T_0_"+std::to_string(j)];
				kj_1 = T_0j_1(r_rot_idx, 2);
				O_j_1i = O_0i - T_0j_1(r_tra_idx, 3);
				// std::cout << "kj_1: " << kj_1 << std::endl;
				// std::cout << "T_0j_1: " << T_0j_1 << std::endl;
				// std::cout << "O_j_1i: " << O_j_1i << std::endl;

				if ((jointsType[j] == "P")||(jointsType[j] == "P_SEA")) {
					Ji_pos(allRows, j) = kj_1;
				} else if ((jointsType[j] == "R")||(jointsType[j] == "R_SEA")) {
					Ji_pos(allRows, j) = mtimes(hat(kj_1),O_j_1i);
					// std::cout << "Ji_pos: " << Ji_pos << std::endl;
					Ji_or(allRows, j) = kj_1;
					// std::cout << "Ji_or: " << Ji_or << std::endl;
				} else {
					throw std::runtime_error("DHJac: Error joint type");
					return 0;
				}
			}
			
			// Add end-effector transformation (only in the EE Jacobian)
			if(i==nj){
				casadi::SX R0i = T_0i(r_rot_idx,r_rot_idx);
				Ji_pos = Ji_pos - casadi::SX::mtimes({R0i,hat(_Ln2EE_.get_translation()),R0i.T(),Ji_or});
				// std::cout << "Ji_pos_ee: " << Ji_pos << std::endl;
			} 
			
			// Add offset from world-frame transformation
			Ji_pos = mtimes(_world2L0_.get_rotation(),Ji_pos);
			// std::cout << "Ji_pos: " << Ji_pos << std::endl;
			Ji_or = mtimes(_world2L0_.get_rotation(),Ji_or);
			// std::cout << "Ji_or: " << Ji_or << std::endl;
			
			Ji_v[i] = Ji_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			if (!robot.add_function("J_"+std::to_string(i), Ji[i], {"q"}, "Jacobian of frame "+std::to_string(i))) return 0;
		}

		if (!robot.add_function("J_ee", Ji[nj], {"q"}, "Jacobian of the end-effector")) return 0;
		return 1;
	}

	int compute_kin_adv(Robot& robot){
		if (robot.model.count("T_0_ee") == 0){
			compute_chain(robot);
		}
		if (robot.model.count("J_ee") == 0){
			compute_jacobians(robot);
		}
		casadi::SX q = robot.model["q"];
		casadi::SX dq = robot.model["dq"];
		casadi::SX Jn = robot.model["J_ee"];

		// jacobian derivatives
		casadi::SX dJn = casadi::SX::jtimes(Jn,q,dq);
		// casadi::SX ddJn = casadi::SX::jtimes(dJn,q,dq) + casadi::SX::jtimes(dJn,dq,_ddq_);
		robot.add_function("J_ee_dot", dJn, {"q","dq"}, "Time derivative of jacobian matrix");

		// jacobian inverse
		casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn,Jn.T()}) + casadi::SX::eye(6)*MU);
		casadi::SX pinvJn = casadi::SX::mtimes({Jn.T(),invJn_dumped});      
		robot.add_function("J_ee_pinv", pinvJn, {"q"}, "Pseudo-Inverse of jacobian matrix");

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