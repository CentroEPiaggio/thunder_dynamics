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
  
	casadi::SX DHTemplate(const casadi::SX& rowDHTable, const casadi::SX& qi, std::string jointType) {
		
		casadi::SX a;
		casadi::SX alpha;
		casadi::SX d;
		casadi::SX theta;
		
		casadi::SX ct;      // cos(theta)
		casadi::SX st;      // sin(theta)
		casadi::SX ca;      // cos(alpha)
		casadi::SX sa;      // sin(alpha)
		
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

	casadi::SX get_transform(casadi::SX frame){
		// frame is traslation and orientation in yaw-pitch-roll
		casadi::SX rotTr = casadi::SX::eye(4);
		
		casadi::SX dx = frame(0);
		casadi::SX dy = frame(1);
		casadi::SX dz = frame(2);
		casadi::SX cy = cos(frame(3));
		casadi::SX sy = sin(frame(3));
		casadi::SX cp = cos(frame(4));
		casadi::SX sp = sin(frame(4));
		casadi::SX cr = cos(frame(5));
		casadi::SX sr = sin(frame(5));

		//template R yaw-pitch-roll
		rotTr(0,0)=cy*cp;
		rotTr(0,1)=cy*sp*sr-sy*cr;
		rotTr(0,2)=cy*sp*cr-sy*sr;
		rotTr(1,0)=sy*cp;
		rotTr(1,1)=sy*sp*sr+cy*cr;
		rotTr(1,2)=sy*sp*cr-cy*sr;
		rotTr(2,0)=-sp;
		rotTr(2,1)=cp*sr;
		rotTr(2,2)=cp*cr;
		rotTr(0,3) = dx;
		rotTr(1,3) = dy;
		rotTr(2,3) = dz;

		return rotTr;
	}
	
	int compute_chain(Robot& robot) {

		// parameters from robot
		auto numJoints = robot.get_numJoints();
		auto jointsType = robot.get_jointsType();
		// auto DHtable = robot.get_DHTable();
		// auto world2L0 = robot.get_world2L0();
		// auto Ln2EE = robot.get_Ln2EE();
		auto& q = robot.model["q"];
		auto& DHtable = robot.model["DHtable"];
		auto& world2L0 = robot.model["world2L0"];
		auto& Ln2EE = robot.model["Ln2EE"];
		// auto gravity = robot.model["gravity"];

		// computing chain
		casadi::SXVector Ti(numJoints);    // Output
		casadi::SXVector T0i(numJoints+1);   // Output
		casadi::Slice allCols(0,4);   
	   
		// Ti is transformation from link i-1 to link i
		Ti[0] = DHTemplate(DHtable(0,allCols), q(0), jointsType[0]);
		// T0i is transformation from link 0 to link i
		T0i[0] = casadi::SX::mtimes({get_transform(world2L0), Ti[0]});
		std::vector<std::string> arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0"});
		// arg_list.push_back({"q"});
		if (!robot.add_function("T_0", Ti[0], arg_list, "relative transformation from frame base to frame 1")) return 0;
		if (!robot.add_function("T_0_0", T0i[0], arg_list, "absolute transformation from frame base to frame 1")) return 0;

		for (int i = 1; i < numJoints; i++) {
			Ti[i] = DHTemplate(DHtable(i,allCols), q(i), jointsType[i]);
			T0i[i] = casadi::SX::mtimes({T0i[i-1], Ti[i]});
			
			arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0"});
			if (!robot.add_function("T_"+std::to_string(i), Ti[i], arg_list, "relative transformation from frame"+ std::to_string(i) +"to frame "+std::to_string(i+1))) return 0;
			if (!robot.add_function("T_0_"+std::to_string(i), T0i[i], arg_list, "absolute transformation from frame base to frame "+std::to_string(i+1))) return 0;
		}

		// end-effector transform
		T0i[numJoints] = casadi::SX::mtimes({T0i[numJoints-1], get_transform(Ln2EE)});

		arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "Ln2EE"});
		if (!robot.add_function("T_0_"+std::to_string(numJoints), T0i[numJoints], arg_list, "absolute transformation from frame base to end_effector")) return 0;
		if (!robot.add_function("T_0_ee", T0i[numJoints], arg_list, "absolute transformation from frame 0 to end_effector")) return 0;
		// std::cout<<"functions created"<<std::endl;

		return 1;
	}
 
	int compute_jacobians(Robot& robot) {

		// parameters from robot
		int nj = robot.get_numJoints();
		auto jointsType = robot.get_jointsType();
		// auto world2L0 = robot.get_world2L0();
		// auto Ln2EE = robot.get_Ln2EE();
		// auto DHtable = robot.get_DHTable();
		auto& q = robot.model["q"];
		auto& DHtable = robot.model["DHtable"];
		auto& world2L0 = robot.model["world2L0"];
		auto& Ln2EE = robot.model["Ln2EE"];
		// auto gravity = robot.model["gravity"];
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
		// auto world_rot = get_transform(world2L0)(r_rot_idx, r_rot_idx);

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
				casadi::SX ee_tr = get_transform(Ln2EE)(r_tra_idx,3);
				Ji_pos = Ji_pos - casadi::SX::mtimes({R0i,hat(ee_tr),R0i.T(),Ji_or});
				// std::cout << "Ji_pos_ee: " << Ji_pos << std::endl;
			} 
			
			// Add offset from world-frame transformation
			// Ji_pos = mtimes(world_rot,Ji_pos);
			// std::cout << "Ji_pos: " << Ji_pos << std::endl;
			// Ji_or = mtimes(world_rot,Ji_or);
			// std::cout << "Ji_or: " << Ji_or << std::endl;
			
			Ji_v[i] = Ji_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			std::vector<std::string> arg_list;
			if (i<nj){
				arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0"});
			} else {
				arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "Ln2EE"});
			}
			if (!robot.add_function("J_"+std::to_string(i), Ji[i], arg_list, "Jacobian of frame "+std::to_string(i))) return 0;
		}

		std::vector<std::string> arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "Ln2EE"});
		if (!robot.add_function("J_ee", Ji[nj], arg_list, "Jacobian of the end-effector")) return 0;
		return 1;
	}

	int compute_kin_adv(Robot& robot){
		if (robot.model.count("T_0_ee") == 0){
			compute_chain(robot);
		}
		if (robot.model.count("J_ee") == 0){
			compute_jacobians(robot);
		}
		casadi::SX& q = robot.model["q"];
		casadi::SX& dq = robot.model["dq"];
		casadi::SX& Jn = robot.model["J_ee"];

		// jacobian derivatives
		casadi::SX dJn = casadi::SX::jtimes(Jn,q,dq);
		// casadi::SX ddJn = casadi::SX::jtimes(dJn,q,dq) + casadi::SX::jtimes(dJn,dq,_ddq_);
		std::vector<std::string> arg_list = robot.obtain_symb_parameters({"q", "dq"}, {"DHtable", "world2L0", "Ln2EE"});
		robot.add_function("J_ee_dot", dJn, arg_list, "Time derivative of jacobian matrix");

		// jacobian inverse
		casadi::SX invJn_dumped = casadi::SX::inv(casadi::SX::mtimes({Jn,Jn.T()}) + casadi::SX::eye(6)*MU);
		casadi::SX pinvJn = casadi::SX::mtimes({Jn.T(),invJn_dumped});
		arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "Ln2EE"});
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