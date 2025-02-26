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
		Q_[0](2,1) =  1;

		Q_[1](0,2) =  1;
		Q_[1](2,0) = -1;

		Q_[2](0,1) = -1;
		Q_[2](1,0) =  1;

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

	int compute_Yr(Robot& robot){
		// parameters from robot
		int nj = robot.get_numJoints();
		int nParLink = robot.STD_PAR_LINK;
		const auto& q = robot.model["q"];
		const auto& dq = robot.model["dq"];
		const auto& dqr = robot.model["dqr"];
		const auto& ddqr = robot.model["ddqr"];
		const auto& gravity = robot.model["gravity"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		if (robot.model.count("J_1") == 0){
			compute_jacobians(robot);
		}
		
		// regressor computation
		casadi::SXVector E_ = createE();
		casadi::SXVector Q_ = createQ();
		casadi::SX dq_sel_ = dq_select(dq);
		
		casadi::SX T0i(4,4);
		casadi::SX Ji(6,nj);
		casadi::SX Jvi(3, nj);
		casadi::SX Jwi(3, nj);

		casadi::SX g = gravity;

		casadi::SX Yr(nj,nParLink*nj);
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
			
			T0i = robot.model["T_0_"+std::to_string(i+1)];
			Ji = robot.model["J_"+std::to_string(i+1)];
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
			casadi::SX C = stdCmatrix(M0_i, q, dq, dq_sel_);

			casadi::SX dX0r_i = mtimes(M0_i, ddqr);
			casadi::SX W0r_i = -mtimes(C, dqr);
			casadi::SX Z0r_i = -mtimes(Jvi.T(),g);
			
			casadi::SX Y0r_i = dX0r_i - W0r_i + Z0r_i;
			
			// ------------------------- Y1r_i -------------------------- //
			
			casadi::SX dX1r_i(nj,3);
			casadi::SX W1r_i(nj,3);

			for (int l=0; l<3; l++) {

				casadi::SX Ql = Q_[l];
				casadi::SX M1l_i = casadi::SX::mtimes({Jwi.T(),R0i,Ql,R0i.T(),Jvi}) - 
								   casadi::SX::mtimes({Jvi.T(),R0i,Ql,R0i.T(),Jwi});
				casadi::SX C = stdCmatrix(M1l_i, q, dq, dq_sel_);

				dX1r_i(allRows,l) = mtimes(M1l_i, ddqr);
				W1r_i(allRows,l) = -mtimes(C, dqr);
			}
			casadi::SX Z1r_i= -(jacobian(mtimes(R0i.T(),g),q)).T();
			
			casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

			// ------------------------- Y2r_i -------------------------- //

			casadi::SX dX2r_i(nj,6);
			casadi::SX W2r_i(nj,6);
			
			for (int l=0; l<6; l++) {

				casadi::SX El = E_[l];
				casadi::SX M2l_i = casadi::SX::mtimes({Jwi.T(),R0i,El,R0i.T(),Jwi});
				casadi::SX C = stdCmatrix(M2l_i, q, dq, dq_sel_);

				dX2r_i(allRows,l) = mtimes(M2l_i, ddqr);
				W2r_i(allRows,l) = -mtimes(C, dqr);
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
		std::vector<std::string> arg_list;
		arg_list = robot.obtain_symb_parameters({"q", "dq", "dqr", "ddqr"}, {"DHtable", "world2L0", "gravity"});
		if (!robot.add_function("Yr", Yr, arg_list, "Manipulator regressor matrix")) return 0;
		arg_list = robot.obtain_symb_parameters({"q", "ddqr"}, {"DHtable", "world2L0"});
		if (!robot.add_function("reg_M", reg_M, arg_list, "Regressor matrix of term M*ddqr")) return 0;
		arg_list = robot.obtain_symb_parameters({"q", "dq", "dqr"}, {"DHtable", "world2L0"});
		if (!robot.add_function("reg_C", reg_C, arg_list, "Regressor matrix of term C*dqr")) return 0;
		arg_list = robot.obtain_symb_parameters({"q"}, {"DHtable", "world2L0", "gravity"});
		if (!robot.add_function("reg_G", reg_G, arg_list, "Regressor matrix of term G")) return 0;

		return 1;
	}

	int compute_reg_Dl(Robot& robot){
		// parameters from robot
		int nj = robot.get_numJoints();
		int nParLink = robot.STD_PAR_LINK;
		int Dl_order = robot.get_Dl_order();
		const auto& dq = robot.model["dq"];
		if (Dl_order==0) return 0;
		const auto& par_Dl = robot.model["par_Dl"];
		const auto& par_Dl_symb = robot.symb["par_Dl"];
		if (robot.model.count("Dl") == 0){
			compute_Dl(robot);
		}
		const auto& Dl = robot.model["dl"];

		// - symbolic par construction - //
		std::vector<casadi::SX> par_symb;
		// parse par_Dl
		for (int i=0; i<par_Dl.size1(); i++){
			if (par_Dl_symb[i]){
				par_symb.push_back(par_Dl(i));
			}
		}
		casadi::SX par = casadi::SX::vertcat(par_symb);

		casadi::SX reg_Dl = casadi::SX::jacobian(Dl, par);
		if (!robot.add_function("reg_dl", reg_Dl, {"dq"}, "Regressor matrix of the link friction")) return 0;

		return 1;
	}

	int compute_reg_elastic(Robot& robot){
		// parameters from robot
		int nj = robot.get_numJoints();
		int nej = robot.get_numElasticJoints();
		int K_order = robot.get_K_order();
		int D_order = robot.get_D_order();
		int Dm_order = robot.get_Dm_order();
		const auto& dq = robot.model["dq"];
		const auto& dx = robot.model["dx"];
		const auto& ddx = robot.model["ddx"];
		const auto& par_K = robot.model["par_K"];
		const auto& par_D = robot.model["par_D"];
		const auto& par_Dm = robot.model["par_Dm"];
		const auto& par_Mm = robot.model["par_Mm"];
		const auto& par_K_symb = robot.symb["par_K"];
		const auto& par_D_symb = robot.symb["par_D"];
		const auto& par_Dm_symb = robot.symb["par_Dm"];
		const auto& par_Mm_symb = robot.symb["par_Mm"];

		if (robot.model.count("k") == 0){
			compute_elastic(robot);
		}
		const auto& K = robot.model["k"];
		const auto& D = robot.model["d"];
		const auto& Dm = robot.model["dm"];
		const auto& Mm = robot.model["Mm"];

		// - symbolic par construction - //
		std::vector<casadi::SX> par_symb_K;
		std::vector<casadi::SX> par_symb_D;
		std::vector<casadi::SX> par_symb_Dm;
		std::vector<casadi::SX> par_symb_Mm;
		// parse par_K
		for (int i=0; i<par_K.size1(); i++){
			if (par_K_symb[i]){
				par_symb_K.push_back(par_K(i));
			}
		}
		// parse par_D
		for (int i=0; i<par_D.size1(); i++){
			if (par_D_symb[i]){
				par_symb_D.push_back(par_D(i));
			}
		}
		// parse par_Dm
		for (int i=0; i<par_Dm.size1(); i++){
			if (par_Dm_symb[i]){
				par_symb_Dm.push_back(par_Dm(i));
			}
		}
		// parse par_Mm
		for (int i=0; i<par_Mm.size1(); i++){
			if (par_Mm_symb[i]){
				par_symb_Mm.push_back(par_Mm(i));
			}
		}
		casadi::SX par_K_tmp = casadi::SX::vertcat(par_symb_K);
		casadi::SX par_D_tmp = casadi::SX::vertcat(par_symb_D);
		casadi::SX par_Dm_tmp = casadi::SX::vertcat(par_symb_Dm);
		casadi::SX par_Mm_tmp = casadi::SX::vertcat(par_symb_Mm);

		casadi::SX reg_K = casadi::SX::jacobian(K, par_K_tmp);
		casadi::SX reg_D = casadi::SX::jacobian(D, par_D_tmp);
		casadi::SX reg_Dm = casadi::SX::jacobian(Dm, par_Dm_tmp);
		casadi::SX reg_Mm = casadi::SX::jacobian(mtimes(Mm,ddx), par_Mm_tmp);

		if (!robot.add_function("reg_k", reg_K, {"q", "x"}, "Regressor matrix of the coupling stiffness")) return 0;
		if (!robot.add_function("reg_d", reg_D, {"dq", "dx"}, "Regressor matrix of the coupling damping")) return 0;
		if (!robot.add_function("reg_dm", reg_Dm, {"dx"}, "Regressor matrix of the motor friction")) return 0;
		if (!robot.add_function("reg_Mm", reg_Mm, {"ddx"}, "Regressor matrix of the motor friction")) return 0;

		return 1;
	}

	int compute_reg_J(Robot& robot){
		// parameters from robot
		int nj = robot.get_numJoints();
		int nParLink = robot.STD_PAR_LINK;
		const auto& DHtable = robot.model["DHtable"];
		const auto& world2L0 = robot.model["world2L0"];
		const auto& Ln2EE = robot.model["Ln2EE"];
		const auto& DHtable_symb = robot.symb["DHtable"];
		const auto& world2L0_symb = robot.symb["world2L0"];
		const auto& Ln2EE_symb = robot.symb["Ln2EE"];
		const auto& q = robot.model["q"];
		const auto& dq = robot.model["dq"];
		const auto& w = robot.model["w"];
		if (robot.model.count("T_0_0") == 0){
			compute_chain(robot);
		}
		if (robot.model.count("J_ee") == 0){
			compute_jacobians(robot);
		}

		// auto dims = DHtable.size();
		// casadi::SX DH_vect = casadi::SX::reshape(DHtable, dims.first*dims.second, 1);

		casadi::SX J = robot.model["J_ee"];
		// std::cout <<"J: " << J << std::endl;

		// - symbolic par construction of par - //
		std::vector<casadi::SX> par_symb;
		// casadi::SX par = casadi::SX::vertcat({DH_vect, world2L0, Ln2EE});
		// int sz = 0;
		// parse DH
		for (int i=0; i<DHtable.size1(); i++){
			if (DHtable_symb[i]){
				par_symb.push_back(DHtable(i));
			}
		}
		// parse world2L0
		for (int i=0; i<world2L0.size1(); i++){
			if (world2L0_symb[i]){
				par_symb.push_back(world2L0(i));
			}
		}
		// parse Ln2EE
		for (int i=0; i<Ln2EE.size1(); i++){
			if (Ln2EE_symb[i]){
				par_symb.push_back(Ln2EE(i));
			}
		}
		// par.resize(sz,1);
		casadi::SX par = casadi::SX::vertcat(par_symb);
		// std::cout <<"par: " << par << std::endl;
		
		casadi::SX Jdq = casadi::SX::mtimes(J, dq);
		// std::cout <<"Jdq: " << Jdq << std::endl;
		casadi::SX reg_Jdq = casadi::SX::jacobian(Jdq, par);
		// std::cout <<"reg_Jdq: " << reg_Jdq << std::endl;
		// reg_Jdq += casadi::SX::jacobian(Jdq, world2L0);
		// reg_Jdq += casadi::SX::jacobian(Jdq, Ln2EE);
		casadi::SX JTw = casadi::SX::mtimes(J.T(), w);
		// std::cout <<"JTw: " << JTw << std::endl;
		casadi::SX reg_JTw = casadi::SX::jacobian(JTw, par);
		// std::cout <<"reg_JTw: " << reg_JTw << std::endl;

		std::vector<std::string> arg_list;
		arg_list = robot.obtain_symb_parameters({"q", "dq"}, {"DHtable", "world2L0", "Ln2EE"});
		// std::cout << "par_list: " << par_symb << std::endl;
		if (!robot.add_function("reg_Jdq", reg_Jdq, arg_list, "Regressor matrix of the quantity J*dq")) return 0;

		arg_list = robot.obtain_symb_parameters({"q", "w"}, {"DHtable", "world2L0", "Ln2EE"});
		if (!robot.add_function("reg_JTw", reg_JTw, arg_list, "Regressor matrix of the quantity J^T*w")) return 0;

		return 1;
	}

    int compute_regressors(Robot& robot, bool advanced){
		int ret = 1;
		bool ELASTIC = robot.get_ELASTIC();
		int Dl_order = robot.get_Dl_order();

		if (!compute_Yr(robot)) ret=0;
		if (!compute_reg_J(robot)) ret=0;
		if (Dl_order>0){
			if (!compute_reg_Dl(robot)) ret=0;
		}
		if (ELASTIC){
			if (!compute_reg_elastic(robot)) ret=0;
		}
		
		return ret;
	}

    

}