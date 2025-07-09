#include "kinematics.h"
#include "utils.h"

namespace thunder_ns{

	constexpr double MU = 0.02; //pseudo-inverse damping coeff
	constexpr double EPSILON = 1e-15; // numerical resolution, below is zero

	// - set to zero small values (Zero If Small) - //
	casadi::SX ZIS(const casadi::SX& x, double tol = EPSILON) {
		// Check if x is numeric and its absolute value is smaller than tolerance
		if (x.is_constant() && std::abs(static_cast<double>(x)) < tol)
			return casadi::SX(0);
		else
			return x;
	}

	// - Rotations - //
	casadi::SX R_x(const casadi::SX& angle) {
		casadi::SX R = casadi::SX::zeros(3, 3); // Create a 3x3 zero matrix

		// Define the rotation matrix components for rotation around the x-axis
		R(0, 0) = 1;        R(0, 1) = 0;                	R(0, 2) = 0;
		R(1, 0) = 0;        R(1, 1) = ZIS(cos(angle));      R(1, 2) = ZIS(-sin(angle));
		R(2, 0) = 0;        R(2, 1) = ZIS(sin(angle));      R(2, 2) = ZIS(cos(angle));

		return R;
	}
	casadi::SX R_y(const casadi::SX& angle) {
		casadi::SX R = casadi::SX::zeros(3, 3); // Create a 3x3 zero matrix

		// Define the rotation matrix components for rotation around the y-axis
		R(0, 0) = ZIS(cos(angle));      R(0, 1) = 0;		R(0, 2) = ZIS(sin(angle));
		R(1, 0) = 0;                	R(1, 1) = 1;    	R(1, 2) = 0;
		R(2, 0) = ZIS(-sin(angle));     R(2, 1) = 0;    	R(2, 2) = ZIS(cos(angle));

		return R;
	}

	// Function to create a rotation matrix for a given angle about the z-axis
	casadi::SX R_z(const casadi::SX& angle) {
		// Ensure the input is of type casadi::SX
		casadi::SX R = casadi::SX::zeros(3, 3); // Create a 3x3 zero matrix

		// Define the rotation matrix components for rotation around the z-axis
		R(0, 0) = ZIS(cos(angle));      R(0, 1) = ZIS(-sin(angle));     R(0, 2) = 0;
		R(1, 0) = ZIS(sin(angle));      R(1, 1) = ZIS(cos(angle));      R(1, 2) = 0;
		R(2, 0) = 0;                	R(2, 1) = 0;                	R(2, 2) = 1;

		return R;
	}
  
	casadi::SX DHTemplate(const casadi::SX& rowDHTable, const casadi::SX& qi, std::string jointType) {
		
		// Transformation:  T_a * T_alpha * T_d * T_theta
		casadi::SX Ti(4,4); // output
		casadi::SX p_a(3,1);
		casadi::SX p_d(3,1);
		casadi::Slice idx(0, 3);      // [0,1,2] indexes

		casadi::SX a = rowDHTable(0);
		casadi::SX alpha = rowDHTable(1);;
		casadi::SX d;
		casadi::SX theta;
		
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

		p_a(0) = a;
		p_d(2) = d;
		casadi::SX Rx = R_x(alpha);
		casadi::SX Rz = R_z(theta);

		Ti(idx,idx) = casadi::SX::mtimes(Rx, Rz);
		Ti(idx,3) = casadi::SX::mtimes(Rx, p_d) + p_a;
		Ti(3,3) = 1;

		return Ti;
	}

	casadi::SX get_transform(casadi::SX frame){
		// traslation -> xyz, rotation -> yow-pitch-roll
		casadi::SX T(4,4); // output
		casadi::SX R(3,3);
		casadi::SX p(3,1);
		casadi::Slice idx(0, 3);      // [0,1,2] indexes

		p = frame(idx);
		casadi::SX psi = frame(3);
		casadi::SX theta = frame(4);
		casadi::SX phi = frame(5);
		R = casadi::SX::mtimes(casadi::SX::mtimes(R_z(psi), R_y(theta)), R_x(phi));

		T(idx,idx) = R;
		T(idx,3) = p;
		T(3,3) = 1;

		return T;
	}
	
	int compute_chain(Robot& robot) {

		// parameters from robot
		auto numJoints = robot.get_numJoints();
		auto jointsType = robot.get_jointsType();
		const auto& q = robot.model["q"];
		const auto& DHtable = robot.model["DHtable"];
		const auto& world2L0 = robot.model["world2L0"];
		const auto& Ln2EE = robot.model["Ln2EE"];

		// computing chain
		casadi::SXVector Ti(numJoints+1);    // Output
		casadi::SXVector T0i(numJoints+2);   // Output
		casadi::Slice allCols(0,4);   
	   
		// Ti is transformation from link i-1 to link i
		Ti[0] = get_transform(world2L0);
		// T0i is transformation from link 0 to link i
		T0i[0] = Ti[0];

		std::vector<std::string> arg_list = robot.obtain_symb_parameters({}, {"world2L0"});
		if (!robot.add_function("T_0", Ti[0], arg_list, "relative transformation from frame base to frame 1")) return 0;
		arg_list = robot.obtain_symb_parameters({}, {"world2L0"});
		if (!robot.add_function("T_0_0", T0i[0], arg_list, "absolute transformation from frame base to frame 1")) return 0;

		for (int i = 0; i < numJoints; i++) {
			casadi::Slice row_i(i*4, i*4+4);
			Ti[i+1] = DHTemplate(DHtable(row_i), q(i), jointsType[i]);
			T0i[i+1] = casadi::SX::mtimes({T0i[i], Ti[i+1]});
			
			arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable"});
			if (!robot.add_function("T_"+std::to_string(i+1), Ti[i+1], arg_list, "relative transformation from frame"+ std::to_string(i) +"to frame "+std::to_string(i+1))) return 0;
			arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0"});
			if (!robot.add_function("T_0_"+std::to_string(i+1), T0i[i+1], arg_list, "absolute transformation from frame base to frame "+std::to_string(i+1))) return 0;
		}

		// end-effector transform
		T0i[numJoints+1] = casadi::SX::mtimes({T0i[numJoints], get_transform(Ln2EE)});

		arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "Ln2EE"});
		if (!robot.add_function("T_0_"+std::to_string(numJoints+1), T0i[numJoints+1], arg_list, "absolute transformation from frame base to end_effector")) return 0;
		if (!robot.add_function("T_0_ee", T0i[numJoints+1], arg_list, "absolute transformation from frame 0 to end_effector")) return 0;
		// std::cout<<"functions created"<<std::endl;

		return 1;
	}
 
	int compute_jacobians(Robot& robot) {

		// parameters from robot
		int nj = robot.get_numJoints();
		auto jointsType = robot.get_jointsType();
		const auto& q = robot.model["q"];
		const auto& DHtable = robot.model["DHtable"];
		const auto& world2L0 = robot.model["world2L0"];
		const auto& Ln2EE = robot.model["Ln2EE"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		
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

			T_0i = robot.model["T_0_"+std::to_string(i_mod+1)];
			// std::cout << "T_0i: " << T_0i << std::endl;
			k0 = T_0i(r_rot_idx, 2);
			// k0(2,0) = 1;
			// std::cout << "k0: " << k0 << std::endl;
			O_0i = T_0i(r_tra_idx, 3);
			// std::cout << "O_0i: " << O_0i << std::endl;

			// // First column of jacobian
			// if ((jointsType[0] == "P")||(jointsType[0] == "P_SEA")) {
			// 	Ji_pos(allRows,0) = k0;
			// } else if ((jointsType[0] == "R")||(jointsType[0] == "R_SEA")) {
			// 	Ji_pos(allRows,0) = mtimes(hat(k0),O_0i);
			// 	// std::cout << "Ji_pos: " << Ji_pos << std::endl;
			// 	Ji_or(allRows,0) = k0;
			// 	// std::cout << "Ji_or: " << Ji_or << std::endl;
			// } else {
			// 	throw std::runtime_error("DHJac: Error joint type");
			// 	return 0;
			// }

			// Rest of columns of jacobian
			for (int j = 0; j <= i_mod; j++) {

				// Init variables of column j-th of jacobian each cycle
				casadi::SX kj(3,1);             // versor of joint j-1
				casadi::SX O_ji(3,1);           // distance of joint i from joint j-1
				casadi::SX T_0j(4,4);           // matrix tranformation of joint i from joint j-1
		
				// T_0j_1 = T0i_vec[j];	// modified from T0i_vec[j-1];
				T_0j = robot.model["T_0_"+std::to_string(j+1)];
				kj = T_0j(r_rot_idx, 2);
				O_ji = O_0i - T_0j(r_tra_idx, 3);
				// std::cout << "kj: " << kj << std::endl;
				// std::cout << "T_0j: " << T_0j << std::endl;
				// std::cout << "O_ji: " << O_ji << std::endl;

				if ((jointsType[j] == "P")||(jointsType[j] == "P_SEA")) {
					Ji_pos(allRows, j) = kj;
				} else if ((jointsType[j] == "R")||(jointsType[j] == "R_SEA")) {
					Ji_pos(allRows, j) = mtimes(hat(kj),O_ji);
					// std::cout << "Ji_pos: " << Ji_pos << std::endl;
					Ji_or(allRows, j) = kj;
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
			if (!robot.add_function("J_"+std::to_string(i+1), Ji[i], arg_list, "Jacobian of frame "+std::to_string(i+1))) return 0;
		}

		std::vector<std::string> arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "Ln2EE"});
		if (!robot.add_function("J_ee", Ji[nj], arg_list, "Jacobian of the end-effector")) return 0;
		return 1;
	}

	int compute_kin_adv(Robot& robot){
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		if (robot.model.count("J_ee") == 0){
			compute_jacobians(robot);
		}
		casadi::SX& q = robot.model["q"];
		casadi::SX& dq = robot.model["dq"];
		casadi::SX& ddq = robot.model["ddq"];
		casadi::SX& Jn = robot.model["J_ee"];

		// jacobian derivatives
		casadi::SX dJn = casadi::SX::jtimes(Jn,q,dq);
		casadi::SX ddJn = casadi::SX::jtimes(dJn,q,dq) + casadi::SX::jtimes(dJn,dq,ddq);
		std::vector<std::string> arg_list = robot.obtain_symb_parameters({"q", "dq"}, {"DHtable", "world2L0", "Ln2EE"});
		robot.add_function("J_ee_dot", dJn, arg_list, "Time derivative of jacobian matrix");
		arg_list = robot.obtain_symb_parameters({"q", "dq", "ddq"}, {"DHtable", "world2L0", "Ln2EE"});
		robot.add_function("J_ee_ddot", ddJn, arg_list, "Time second derivative of jacobian matrix");

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