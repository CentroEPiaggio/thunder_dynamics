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

	casadi::SX dq_select(const casadi::SX& _dq_) {
		int n = _dq_.size1();
		
		casadi::Slice allRows;
		casadi::SX mat_dq = casadi::SX::zeros(n, n * n);
		for (int i = 0; i < n; i++) {
			casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
			mat_dq(allRows, sel_col) = casadi::SX::eye(n)*_dq_(i);
		}
		
		return mat_dq;
	}

	casadi::SX stdCmatrix(const casadi::SX& M, const casadi::SX& _q_, const casadi::SX& _dq_, const casadi::SX& dq_sel_) {
		int n = _q_.size1();

		casadi::SX jac_M = jacobian(M,_q_);
		
		casadi::SX C123 = reshape(mtimes(jac_M,_dq_),n,n);
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
		C = (C123 + C132 - C132.T())/2;

		return C;
	}

	std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(Robot& robot){
		// parameters from robot
		auto _numJoints_ = robot.get_numJoints();
		auto _nParLink_ = robot.get_numParLink();
		auto _jointsType_ = robot.get_jointsType();
		// auto _DHtable_ = robot.get_DHTable();
		auto _world2L0_ = robot.get_world2L0();
		// auto _Ln2EE_ = robot.get_Ln2EE();
		auto _q_ = robot.model["q"];
		auto _par_DYN_ = robot.model["par_DYN"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}

		auto par_inertial = createInertialParameters(_numJoints_, _nParLink_, _par_DYN_);
		// casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		// casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);

		casadi::SX Jci_pos(3, _numJoints_); // matrix of velocity jacobian
		casadi::SX Ji_or(3, _numJoints_);   // matrix of omega jacobian
		casadi::SXVector Ji_v(_numJoints_); // vector of matrix Ji_v
		casadi::SXVector Ji_w(_numJoints_); // vector of matrix Ji_w
		casadi::SXVector Ji(_numJoints_);		// complete jacobian
		casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
		casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
		casadi::Slice allRows;              // Select all rows

		for (int i = 0; i < _numJoints_; i++) {
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
			if (_jointsType_[0] == 'P') {
				Jci_pos(allRows,0) = k0;
			} else if (_jointsType_[0] == 'R') {
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

				if (_jointsType_[j] == 'P') {
					Jci_pos(allRows, j) = kj_1;
				} else if (_jointsType_[j] == 'R') {
					Jci_pos(allRows, j) = mtimes(hat(kj_1),O_j_1Ci);
					Ji_or(allRows, j) = kj_1;
				} else {
					throw std::runtime_error("DHJac: Error joint type");
				}
			}

			// Add offset from world-frame transformation
			Jci_pos = mtimes(_world2L0_.get_rotation(),Jci_pos);
			Ji_or = mtimes(_world2L0_.get_rotation(),Ji_or);
			
			Ji_v[i] = Jci_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			// std::cout<<"Ji[i]: "<<Ji[i]<<std::endl;
			robot.add_function("J_cm_"+std::to_string(i), Ji[i], {"q", "par_DYN"}, "Jacobian of center of mass of link "+std::to_string(i));
		}

		return std::make_tuple(Ji_v, Ji_w);
	}

	casadi::SXVector compute_dynamics(Robot& robot){
		// parameters from robot
		auto nj = robot.get_numJoints();
		auto nParLink = robot.get_numParLink();
		// auto jointsType_ = robot.get_jointsType();
		// auto _DHtable_ = robot.get_DHTable();
		auto _world2L0_ = robot.get_world2L0();
		FrameOffset& base_frame = _world2L0_;
		// auto _Ln2EE_ = robot.get_Ln2EE();
		auto _q_ = robot.model["q"];
		auto _dq_ = robot.model["dq"];
		auto _par_DYN_ = robot.model["par_DYN"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		auto par_inertial = createInertialParameters(nj, nParLink, _par_DYN_);
		casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		// casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);
		
		casadi::SX dq_sel_ = dq_select(_dq_);

		// const int nj = _q_.size1();
		
		// casadi::SXVector Ti(nj);
		casadi::SX T0i;
		std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

		casadi::SXVector Jci(nj);
		casadi::SXVector Jwi(nj);
		std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

		casadi::SX g = base_frame.get_gravity();

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
		
		C = stdCmatrix(M,_q_,_dq_,dq_sel_);
		C_std = stdCmatrix_classic(M,_q_,_dq_,dq_sel_);

		robot.add_function("M", M, {"q","par_DYN"}, "Manipulator mass matrix");
		robot.add_function("C", C, {"q","dq","par_DYN"}, "Manipulator Coriolis matrix");
		robot.add_function("C_std", C_std, {"q","dq","par_DYN"}, "Classic formulation of the manipulator Coriolis matrix");
		robot.add_function("G", G, {"q","par_DYN"}, "Manipulator gravity terms");

		return {M,C,G};
	}
	
	// int compute_Mass(Robot& robot) {

	// 	// parameters from robot
	// 	auto _numJoints_ = robot.get_numJoints();
	// 	auto _jointsType_ = robot.get_jointsType();
	// 	auto _DHtable_ = robot.get_DHTable();
	// 	auto _world2L0_ = robot.get_world2L0();
	// 	auto _Ln2EE_ = robot.get_Ln2EE();
	// 	auto _q_ = robot.model["q"];

	// 	// computing Mass matrix

	// 	return 1;
	// }

	// compute everything
	// int compute_dynamics(Robot& robot, bool advanced){
	// 	if (!compute_Mass(robot)) return 0;
	// 	if (!compute_Coriolis(robot)) return 0;
	// 	if (!compute_Gravity(robot)) return 0;

	// 	if (advanced){
	// 		// matrix derivatives and so on
	// 	}

	// 	return 1;
	// }

}