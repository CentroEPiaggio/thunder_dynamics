#include "regressors.h"
#include "utils.h"
#include "robot.h"
#include "kinematics.h"
#include "dynamics.h"


namespace thunder_ns{

	// extern constexpr int nParLink = 10;

    casadi::SXVector createQ() {

		casadi::SXVector Q_(3);

		for(int i=0;i<3;i++){
			Q_[i] = casadi::SX::zeros(3,3);
		}
		
		Q_[0](1,2) = -1;
		Q_[0](2,1) = 1;

		Q_[1](0,2) = 1;
		Q_[1](2,0) = -1;

		Q_[2](0,1) = -1;
		Q_[2](1,0) = 1;

		return Q_;
	}
	
	casadi::SXVector createE() {
		
		casadi::SXVector E_(6);

		for(int i=0;i<6;i++){
			E_[i] = casadi::SX::zeros(3,3);
		}
		E_[0](0,0) = 1;

		E_[1](0,1) = 1;
		E_[1](1,0) = 1;

		E_[2](0,2) = 1;
		E_[2](2,0) = 1;

		E_[3](1,1) = 1;

		E_[4](1,2) = 1;
		E_[4](2,1) = 1;

		E_[5](2,2) = 1;

		return E_;
	}

    int compute_regressors(Robot& robot, bool advanced){
		// parameters from robot
		int nj = robot.get_numJoints();
		int nParLink = robot.get_numParLink();
		// auto _jointsType_ = robot.get_jointsType();
		// auto _DHtable_ = robot.get_DHTable();
		auto _world2L0_ = robot.get_world2L0();
		// auto _Ln2EE_ = robot.get_Ln2EE();
		auto _q_ = robot.model["q"];
		auto _dq_ = robot.model["dq"];
		auto _dqr_ = robot.model["dqr"];
		auto _ddqr_ = robot.model["ddqr"];
		auto _par_DYN_ = robot.model["par_DYN"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		if (robot.model.count("J_0") == 0){
			compute_jacobians(robot);
		}
		// auto par_inertial = createInertialParameters(_numJoints_, _par_DYN_);
		// casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		// casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		// casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);
		
		// regressor computation
		casadi::SXVector E_ = createE();
		casadi::SXVector Q_ = createQ();
		casadi::SX dq_sel_ = dq_select(_dq_);
		
		// casadi::SXVector Ti(nj);
		casadi::SX T0i(4,4);
		// std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

		// casadi::SXVector Jvi(nj);
		// casadi::SXVector Jwi(nj);
		casadi::SX Ji(6,nj);
		casadi::SX Jvi(3, nj);
		casadi::SX Jwi(3, nj);
		// std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

		casadi::SX g = _world2L0_.get_gravity();

		casadi::SX Yr(nj,nParLink*nj);        // regressor matrix
		casadi::SX reg_M(nj, nParLink*nj);
		casadi::SX reg_C(nj, nParLink*nj);
		casadi::SX reg_G(nj, nParLink*nj);
				
		casadi::Slice allRows;
		casadi::Slice allCols(0,nj);          
		casadi::Slice selR(0,3);
		casadi::Slice sel_v(0,3);
		casadi::Slice sel_w(3,6);

		// T_tuple = DHFwKinJoints();
		// T0i = std::get<0>(T_tuple);
		// //Ti  = std::get<1>(T_tuple);

		// J_tuple = DHJacJoints(T0i);
		// Jvi = std::get<0>(J_tuple);
		// Jwi = std::get<1>(J_tuple);
		
		for (int i=0; i<nj; i++) {
			
			T0i = robot.model["T_0_"+std::to_string(i)];
			Ji = robot.model["J_"+std::to_string(i)];
			casadi::SX R0i = T0i(selR,selR);
			Jvi = Ji(sel_v, allCols);
			Jwi = Ji(sel_w, allCols);
			// world transform is included in T0i
			// // if(i==(nj-1)){	// end-effector
			// // 	R0i = mtimes(R0i,ee_frame.get_rotation());
			// // } else {
			// // 	R0i = casadi::SX::mtimes({_world2L0_.get_rotation(),R0i,_world2L0_.get_rotation().T()});
			// // }
			// R0i = casadi::SX::mtimes({_world2L0_.get_rotation(),R0i,_world2L0_.get_rotation().T()});

			// ------------------------- Y0r_i -------------------------- //
			
			casadi::SX M0_i = mtimes(Jvi.T(),Jvi);
			casadi::SX C = stdCmatrix(M0_i, _q_, _dq_, dq_sel_);

			casadi::SX dX0r_i = mtimes(M0_i,_ddqr_);
			casadi::SX W0r_i = -mtimes(C,_dqr_);
			casadi::SX Z0r_i = -mtimes(Jvi.T(),g);
			
			casadi::SX Y0r_i = dX0r_i - W0r_i + Z0r_i;
			
			// ------------------------- Y1r_i -------------------------- //
			
			casadi::SX dX1r_i(nj,3);
			casadi::SX W1r_i(nj,3);

			for (int l=0; l<3; l++) {

				casadi::SX Ql = Q_[l];
				casadi::SX M1l_i = casadi::SX::mtimes({Jwi.T(),R0i,Ql,R0i.T(),Jvi}) - 
								   casadi::SX::mtimes({Jvi.T(),R0i,Ql,R0i.T(),Jwi});
				casadi::SX C = stdCmatrix(M1l_i, _q_, _dq_, dq_sel_);

				dX1r_i(allRows,l) = mtimes(M1l_i,_ddqr_);
				W1r_i(allRows,l) = -mtimes(C,_dqr_);
			}
			casadi::SX Z1r_i= -(jacobian(mtimes(R0i.T(),g),_q_)).T();
			
			casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

			// ------------------------- Y2r_i -------------------------- //

			casadi::SX dX2r_i(nj,6);
			casadi::SX W2r_i(nj,6);
			
			for (int l=0; l<6; l++) {

				casadi::SX El = E_[l];
				casadi::SX M2l_i = casadi::SX::mtimes({Jwi.T(),R0i,El,R0i.T(),Jwi});
				casadi::SX C = stdCmatrix(M2l_i, _q_, _dq_, dq_sel_);

				dX2r_i(allRows,l) = mtimes(M2l_i,_ddqr_);
				W2r_i(allRows,l) = -mtimes(C,_dqr_);
			}

			casadi::SX Y2r_i = dX2r_i - W2r_i;

			// ------------------- matrix regressors ------------------- //
			casadi::SX reg_M_i = horzcat(dX0r_i, dX1r_i, dX2r_i);
			casadi::SX reg_C_i = horzcat(-W0r_i, -W1r_i, -W2r_i);
			casadi::SX reg_G_i = horzcat(Z0r_i, Z1r_i, casadi::SX::zeros(nj,6));

			// ------------------------- Yr_i -------------------------- //

			casadi::SX Yr_i = horzcat(Y0r_i,Y1r_i,Y2r_i);
			
			// final regressors 
			casadi::Slice selCols(i*nParLink, (i+1)*nParLink);          // Select current columns of matrix regressor
			Yr(allRows,selCols) = Yr_i;
			reg_M(allRows,selCols) = reg_M_i;
			reg_C(allRows,selCols) = reg_C_i;
			reg_G(allRows,selCols) = reg_G_i;
		}

		if (!robot.add_function("Yr", Yr, {"q", "dq", "dqr", "ddqr"}, "Manipulator regressor matrix")) return 0;
		if (!robot.add_function("reg_M", reg_M, {"q", "ddqr"}, "Regressor matrix of term M*ddqr")) return 0;
		if (!robot.add_function("reg_C", reg_C, {"q", "dq", "dqr"}, "Regressor matrix of term C*dqr")) return 0;
		if (!robot.add_function("reg_G", reg_G, {"q"}, "Regressor matrix of term G")) return 0;

		return 1;
	}

    

}