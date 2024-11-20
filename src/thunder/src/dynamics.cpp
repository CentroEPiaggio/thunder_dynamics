#include "dynamics.h"
#include "utils.h"
#include "robot.h"
#include "kinematics.h"

/* Function name used to generate code */
// #define Ti_STRING "T_i"
// #define T0i_STRING "T_0_i"
// #define JAC_DOT_STRING "J_ee_dot"
// #define JAC_PINV_STRING "J_ee_pinv"

namespace thunder_ns{

	// constexpr double MU = 0.02; //pseudo-inverse damping coeff
	// constexpr double EPSILON = 1e-15; // numerical resolution, below is zero
	// extern constexpr int nParLink = 10;

	std::tuple<casadi::SXVector, casadi::SXVector, casadi::SXVector> createInertialParameters(int nj, int nParLink, casadi::SX par_DYN){
		
		// dynamics need
		casadi::SXVector _mass_vec_(nj);
		casadi::SXVector _distCM_(nj);
		casadi::SXVector _J_3x3_(nj);

		for (int i=0; i<nj;i++){
			_distCM_[i] = casadi::SX::zeros(3,1);
			_J_3x3_[i] = casadi::SX::zeros(3,3);
		}
		
		casadi::SX tempI(3,3);

		for(int i=0; i<nj; i++){
			
			_mass_vec_[i] = par_DYN(i*nParLink,0);
			for(int j=0; j<3; j++){
				_distCM_[i](j,0) = par_DYN(i*nParLink+j+1,0);
			}

			tempI(0,0) = par_DYN(i*nParLink+4,0);
			tempI(0,1) = par_DYN(i*nParLink+5,0);
			tempI(0,2) = par_DYN(i*nParLink+6,0);
			tempI(1,0) = tempI(0,1);
			tempI(1,1) = par_DYN(i*nParLink+7,0);
			tempI(1,2) = par_DYN(i*nParLink+8,0);
			tempI(2,0) = tempI(0,2);
			tempI(2,1) = tempI(1,2);
			tempI(2,2) = par_DYN(i*nParLink+9,0);

			_J_3x3_[i] = tempI;
		}

		return std::make_tuple(_mass_vec_, _distCM_, _J_3x3_);
	}

	casadi::SX dq_select(const casadi::SX& dq) {
		int n = dq.size1();
		
		casadi::Slice allRows;
		casadi::SX mat_dq = casadi::SX::zeros(n, n * n);
		for (int i = 0; i < n; i++) {
			casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
			mat_dq(allRows, sel_col) = casadi::SX::eye(n)*dq(i);
		}
		
		return mat_dq;
	}

	casadi::SX stdCmatrix(const casadi::SX& M, const casadi::SX& q, const casadi::SX& dq, const casadi::SX& dq_sel_) {
		int n = q.size1();

		casadi::SX jac_M = jacobian(M,q);
		
		casadi::SX C123 = reshape(mtimes(jac_M,dq),n,n);
		casadi::SX C132 = mtimes(dq_sel_,jac_M);
		casadi::SX C231 = C132.T();

		casadi::SX C(n,n);
		C = (C123 + C132 - C231)/2;

		return C;
	}

	casadi::SX stdCmatrix_classic(const casadi::SX& M, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_) {
		// classic C matrix computation, probably have to be C = C/2
		int n = q_.size1();

		casadi::SX C123(n, n);
		casadi::SX C132(n, n);
		// casadi::SX C231(n, n);

		for (int h = 0; h < n; h++) {
			for (int j = 0; j < n; j++) {
				for (int k = 0; k < n; k++) {
					casadi::SX dbhj_dqk = jacobian(M(h, j), q_(k));
					casadi::SX dbhk_dqj = jacobian(M(h, k), q_(j));
					//casadi::SX dbjk_dqh = jacobian(M(j, k), q_(h));
					C123(h, j) = C123(h, j) + 0.5 * (dbhj_dqk) * dq_(k);
					C132(h, j) = C132(h, j) + 0.5 * (dbhk_dqj) * dq_(k);
					//C231(h, j) = C231(h, j) + 0.5 * (dbjk_dqh) * dq_(k);
				}
			}
		}

		// casadi::SXVector C(3);
		// C[0] = C123;
		// C[1] = C132;
		// C[2] = C132.T();
		casadi::SX C(n,n);
    
		C = C123 + C132 - C132.T();

		return C;
	}

	std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(Robot& robot){
		// parameters from robot
		auto numJoints = robot.get_numJoints();
		auto _nParLink_ = robot.STD_PAR_LINK;
		auto jointsType = robot.get_jointsType();
		// auto _DHtable_ = robot.get_DHTable();
		// auto world2L0 = robot.get_world2L0();
		// auto _Ln2EE_ = robot.get_Ln2EE();
		auto q = robot.model["q"];
		auto world2L0 = robot.model["world2L0"];
		auto par_DYN = robot.model["par_DYN"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}

		auto par_inertial = createInertialParameters(numJoints, _nParLink_, par_DYN);
		// casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		// casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);

		casadi::SX Jci_pos(3, numJoints); // matrix of velocity jacobian
		casadi::SX Ji_or(3, numJoints);   // matrix of omega jacobian
		casadi::SXVector Ji_v(numJoints); // vector of matrix Ji_v
		casadi::SXVector Ji_w(numJoints); // vector of matrix Ji_w
		casadi::SXVector Ji(numJoints);		// complete jacobian
		casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
		casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
		casadi::Slice allRows;              // Select all rows
		auto world_rot = get_transform(world2L0)(r_rot_idx, r_rot_idx);

		for (int i = 0; i < numJoints; i++) {
			casadi::SX k0(3,1);             // versor of joint i
			casadi::SX O_0i(3,1);           // distance of joint i from joint 0
			casadi::SX T_0i(4,4);           // matrix tranformation of joint i from joint 0
			
			casadi::SX O_Ci(3,1);
			casadi::SX R0i;
			
			k0(2,0) = 1;
			// T_0i = T0i_vec[i];
			T_0i = robot.model["T_0_"+std::to_string(i)];
			O_0i = T_0i(r_tra_idx, 3);
			
			R0i = T_0i(r_rot_idx,r_rot_idx);
			O_Ci = O_0i + mtimes(R0i,_distCM_[i]);
			
			// First column of jacobian
			if ((jointsType[0] == "P")||(jointsType[0] == "P_SEA")) {
				Jci_pos(allRows,0) = k0;
			} else if ((jointsType[0] == "R")||(jointsType[0] == "R_SEA")) {
				Jci_pos(allRows,0) = mtimes(hat(k0),O_Ci);
				Ji_or(allRows,0) = k0;
			} else {
				throw std::runtime_error("DHJac: Error joint type");
			}

			// Rest of columns of jacobian
			for (int j = 1; j <= i; j++) {
				// Init variables of column j-th of jacobian each cycle
				casadi::SX kj_1(3,1);             // versor of joint j-1
				casadi::SX O_j_1Ci(3,1);           // distance of joint i from joint j-1
				casadi::SX T_0j_1(4,4);           // matrix tranformation of joint i from joint j-1

				// T_0j_1 = T0i_vec[j];	// modified from T0i_vec[j-1];
				T_0j_1 = robot.model["T_0_"+std::to_string(j)];
				kj_1 = T_0j_1(r_rot_idx, 2);
				O_j_1Ci = O_Ci - T_0j_1(r_tra_idx, 3);

				if ((jointsType[j] == "P")||(jointsType[j] == "P_SEA")) {
					Jci_pos(allRows, j) = kj_1;
				} else if ((jointsType[j] == "R")||(jointsType[j] == "R_SEA")) {
					Jci_pos(allRows, j) = mtimes(hat(kj_1),O_j_1Ci);
					Ji_or(allRows, j) = kj_1;
				} else {
					throw std::runtime_error("DHJac: Error joint type");
				}
			}

			// Add offset from world-frame transformation
			// Jci_pos = mtimes(world2L0.get_rotation(),Jci_pos);
			// Ji_or = mtimes(world2L0.get_rotation(),Ji_or);
			Jci_pos = mtimes(world_rot,Jci_pos);
			Ji_or = mtimes(world_rot,Ji_or);
			
			Ji_v[i] = Jci_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			// std::cout<<"Ji[i]: "<<Ji[i]<<std::endl;
			std::vector<std::string> arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "par_DYN"});
			robot.add_function("J_cm_"+std::to_string(i), Ji[i], arg_list, "Jacobian of center of mass of link "+std::to_string(i));
		}

		return std::make_tuple(Ji_v, Ji_w);
	}

	int compute_MCG(Robot& robot){
		// parameters from robot
		auto nj = robot.get_numJoints();
		auto nParLink = robot.STD_PAR_LINK;
		// auto jointsType_ = robot.get_jointsType();
		// auto _DHtable_ = robot.get_DHTable();
		// auto world2L0 = robot.get_world2L0();
		// FrameOffset& base_frame = world2L0;
		// auto _Ln2EE_ = robot.get_Ln2EE();
		auto q = robot.model["q"];
		auto dq = robot.model["dq"];
		auto par_DYN = robot.model["par_DYN"];
		auto gravity = robot.model["gravity"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		auto par_inertial = createInertialParameters(nj, nParLink, par_DYN);
		casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		// casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);
		
		casadi::SX dq_sel_ = dq_select(dq);

		// const int nj = q.size1();
		
		// casadi::SXVector Ti(nj);
		casadi::SX T0i;
		std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

		casadi::SXVector Jci(nj);
		casadi::SXVector Jwi(nj);
		std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

		casadi::SX g = gravity; //base_frame.get_gravity();

		casadi::SX M(nj,nj);
		casadi::SX C(nj,nj);
		casadi::SX C_std(nj,nj);
		casadi::SX G(nj,1);
		
		casadi::SX Mi(nj,nj);
		casadi::SX Gi(1,nj);
		casadi::SX mi(1,1);
		casadi::SX Ii(3,3);
		casadi::Slice selR(0,3);

		// T_tuple = DHFwKinJoints();
		// T0i = std::get<0>(T_tuple);
		//Ti  = std::get<1>(T_tuple);

		J_tuple = DHJacCM(robot);
		Jci = std::get<0>(J_tuple);
		Jwi = std::get<1>(J_tuple);
		
		for (int i=0; i<nj; i++) {
			T0i = robot.model["T_0_"+std::to_string(i)];
			// std::cout<<"T0i: "<<T0i<<std::endl;
			casadi::SX R0i = T0i(selR,selR);
			// std::cout<<"R0i: "<<R0i<<std::endl;

			mi = _mass_vec_[i];
			// std::cout<<"mi: "<<mi<<std::endl;
			Ii = _J_3x3_[i];
			// std::cout<<"Ii: "<<Ii<<std::endl;
			Mi = mi * casadi::SX::mtimes({Jci[i].T(), Jci[i]}) + casadi::SX::mtimes({Jwi[i].T(),R0i,Ii,R0i.T(),Jwi[i]});
			// std::cout<<"Mi: "<<Mi<<std::endl;
			M = M + Mi;
			// std::cout<<"M: "<<M<<std::endl;

			Gi = -mi * casadi::SX::mtimes({g.T(),Jci[i]});
			G = G + Gi.T();
		}
		
		C = stdCmatrix(M,q,dq,dq_sel_);
		C_std = stdCmatrix_classic(M,q,dq,dq_sel_);

		std::vector<std::string> arg_list;
		arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "par_DYN"});
		robot.add_function("M", M, arg_list, "Manipulator mass matrix");
		arg_list = robot.obtain_symb_parameters({"q", "dq"}, {"DHtable", "world2L0", "par_DYN"});
		robot.add_function("C", C, arg_list, "Manipulator Coriolis matrix");
		arg_list = robot.obtain_symb_parameters({"q", "dq"}, {"DHtable", "world2L0", "par_DYN"});
		robot.add_function("C_std", C_std, arg_list, "Classic formulation of the manipulator Coriolis matrix");
		arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "gravity", "par_DYN"});
		robot.add_function("G", G, arg_list, "Manipulator gravity terms");

		return 1;
	}

	int compute_elastic(Robot& robot){
		// parameters from robot
		int nj = robot.get_numJoints();
		bool ELASTIC = robot.get_ELASTIC();

		if (ELASTIC > 0){
			int numElasticJoints = robot.get_numElasticJoints();
			std::vector<int> isElasticJoint = robot.get_isElasticJoint();
			int K_order = robot.get_K_order();
			int D_order = robot.get_D_order();
			int Dm_order = robot.get_Dm_order();
			auto q = robot.model["q"];
			auto x = robot.model["x"];
			auto dq = robot.model["dq"];
			auto dx = robot.model["dx"];
			auto par_K = robot.model["par_K"];
			auto par_D = robot.model["par_D"];
			auto par_Dm = robot.model["par_Dm"];

			casadi::SX K(numElasticJoints,1);
			casadi::SX D(numElasticJoints,1);
			casadi::SX Dm(numElasticJoints,1);
			for (int i=0; i<numElasticJoints; i++){
				for (int ord=0; ord<K_order; ord++){
					K(i) += pow(x(i)-q(i), ord+1) * par_K(i*K_order+ord);
				}
				for (int ord=0; ord<D_order; ord++){
					D(i) += pow(dx(i)-dq(i), ord+1) * par_D(i*D_order+ord);
				}
				for (int ord=0; ord<Dm_order; ord++){
					Dm(i) += pow(dx(i), ord+1) * par_Dm(i*Dm_order+ord);
				}
			}
			std::vector<std::string> arg_list;
			if (K_order > 0) {
				arg_list = robot.obtain_symb_parameters({"q", "x"}, {"par_K"});
				robot.add_function("K", K, arg_list, "SEA manipulator elastic coupling");
			}
			if (D_order > 0) {
				arg_list = robot.obtain_symb_parameters({"dq", "dx"}, {"par_D"});
				robot.add_function("D", D, arg_list, "SEA manipulator dampind coupling");
			}
			if (Dm_order > 0) {
				arg_list = robot.obtain_symb_parameters({"dx"}, {"par_Dm"});
				robot.add_function("Dm", Dm, arg_list, "SEA manipulator motor damping");
			}
			return 1;
		} else return 0;
	}

	int compute_Dl(Robot& robot){
		// parameters from robot
		int Dl_order = robot.get_Dl_order();

		if (Dl_order > 0){
			int nj = robot.get_numJoints();
			auto dq = robot.model["dq"];
			auto par_Dl = robot.model["par_Dl"];

			casadi::SX Dl(nj,1);
			for (int i=0; i<nj; i++){
				for (int ord=0; ord<Dl_order; ord++){
					Dl(i) += pow(dq(i), ord+1) * par_Dl(i*Dl_order+ord);
				}
			}
			std::vector<std::string> arg_list;
			arg_list = robot.obtain_symb_parameters({"dq"}, {"par_Dl"});
			robot.add_function("Dl", Dl, arg_list, "Manipulator link friction");
			return 1;
		} else return 0;
	}

	// compute everything
	int compute_dynamics(Robot& robot, bool advanced){
		if (!compute_MCG(robot)) return 0;
		if (!compute_Dl(robot)) return 0;
		if (!compute_elastic(robot)) return 0;
		
		if (advanced){
			// matrix derivatives and so on
		}

		return 1;
	}

}