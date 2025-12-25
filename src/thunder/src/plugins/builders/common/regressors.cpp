#include "plugins/builders/common/regressors.h"
#include "plugins/builders/common/kinematics.h"
#include "plugins/builders/common/dynamics.h"
#include "utils.h"

using std::string;
using std::vector;

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
		int nj = robot.get<int>("numJoints");
		const int nParLink = robot.get<const int>("STD_PAR_LINK");
		auto q = robot.get_model("q");
		auto dq = robot.get_model("dq");
		auto dqr = robot.get_model("dqr");
		auto ddqr = robot.get_model("ddqr");
		auto par_gravity = robot.get_model("par_gravity");
		
		// regressor computation
		casadi::SXVector E_ = createE();
		casadi::SXVector Q_ = createQ();
		casadi::SX dq_sel_ = dq_select(dq);
		
		casadi::SX Twi(4,4);
		casadi::SX Ji(6,nj);
		casadi::SX Jvi(3, nj);
		casadi::SX Jwi(3, nj);

		casadi::SX g = par_gravity;

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
		// Twi = std::get<0>(T_tuple);
		// //Ti  = std::get<1>(T_tuple);

		// J_tuple = DHJacJoints(Twi);
		// Jvi = std::get<0>(J_tuple);
		// Jwi = std::get<1>(J_tuple);
		
		for (int i=0; i<nj; i++) {
			
			Twi = robot.get_model("T_w_"+std::to_string(i+1));
			Ji = robot.get_model("J_"+std::to_string(i+1));
			casadi::SX Rwi = Twi(selR,selR);
			Jvi = Ji(sel_v, allCols);
			Jwi = Ji(sel_w, allCols);
			// world transform is included in Twi
			// // if(i==(nj-1)){	// end-effector
			// // 	Rwi = mtimes(Rwi,ee_frame.get_rotation());
			// // } else {
			// // 	Rwi = casadi::SX::mtimes({_world2L0_.get_rotation(),Rwi,_world2L0_.get_rotation().T()});
			// // }
			// Rwi = casadi::SX::mtimes({_world2L0_.get_rotation(),Rwi,_world2L0_.get_rotation().T()});

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
				casadi::SX M1l_i = casadi::SX::mtimes({Jwi.T(),Rwi,Ql,Rwi.T(),Jvi}) - 
								   casadi::SX::mtimes({Jvi.T(),Rwi,Ql,Rwi.T(),Jwi});
				casadi::SX C = stdCmatrix(M1l_i, q, dq, dq_sel_);

				dX1r_i(allRows,l) = mtimes(M1l_i, ddqr);
				W1r_i(allRows,l) = -mtimes(C, dqr);
			}
			casadi::SX Z1r_i= -(jacobian(mtimes(Rwi.T(),g),q)).T();
			
			casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

			// ------------------------- Y2r_i -------------------------- //

			casadi::SX dX2r_i(nj,6);
			casadi::SX W2r_i(nj,6);
			
			for (int l=0; l<6; l++) {

				casadi::SX El = E_[l];
				casadi::SX M2l_i = casadi::SX::mtimes({Jwi.T(),Rwi,El,Rwi.T(),Jwi});
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
		arg_list = {"q", "dq", "dqr", "ddqr", "par_KIN", "par_world2L0", "par_gravity"};
		if (!robot.add_function("Yr", Yr, arg_list, "Manipulator regressor matrix")) return 0;
		arg_list = {"q", "ddqr", "par_KIN", "par_world2L0"};
		if (!robot.add_function("reg_M", reg_M, arg_list, "Regressor matrix of term M*ddqr")) return 0;
		arg_list = {"q", "dq", "dqr", "par_KIN", "par_world2L0"};
		if (!robot.add_function("reg_C", reg_C, arg_list, "Regressor matrix of term C*dqr")) return 0;
		arg_list = {"q", "par_KIN", "par_world2L0", "par_gravity"};
		if (!robot.add_function("reg_G", reg_G, arg_list, "Regressor matrix of term G")) return 0;

		return 1;
	}

	int compute_reg_Dl(Robot& robot){
		// parameters from robot
		int nj = robot.get<int>("numJoints");
		const int nParLink = robot.get<const int>("STD_PAR_LINK");
		int Dl_order = (robot.properties.count("Dl_order")) ? robot.get<int>("Dl_order") : 0;
		auto dq = robot.get_model("dq");
		if (Dl_order==0) return 0;
		auto par_Dl = robot.get_model("par_Dl");
		auto par_Dl_isSymb = robot.parameters["par_Dl"].is_symbolic;
		auto Dl = robot.get_model("dl");

		// - symbolic par construction - //
		std::vector<casadi::SX> par_symb;
		// parse par_Dl
		for (int i=0; i<par_Dl.size1(); i++){
			if (par_Dl_isSymb[i]){
				par_symb.push_back(par_Dl(i));
			}
		}
		casadi::SX par = casadi::SX::vertcat(par_symb);

		if (par.size1() != 0){
			casadi::SX reg_Dl = casadi::SX::jacobian(Dl, par);
			if (!robot.add_function("reg_dl", reg_Dl, {"dq"}, "Regressor matrix of the link friction")) return 0;

		}
		
		return 1;
	}

	int compute_reg_elastic(Robot& robot){
		// parameters from robot
		int nj = robot.get<int>("numJoints");
		int nej = robot.get<int>("numSoftJoints");
		int K_order = robot.get<int>("K_order");
		int D_order = robot.get<int>("D_order");
		int Dm_order = robot.get<int>("Dm_order");
		auto dq = robot.get_model("dq");
		auto dx = robot.get_model("dx");
		auto ddx = robot.get_model("ddx");
		auto par_K = robot.get_model("par_K");
		auto par_D = robot.get_model("par_D");
		auto par_Dm = robot.get_model("par_Dm");
		auto par_Mm = robot.get_model("par_Mm");
		auto par_K_isSymb = robot.parameters["par_K"].is_symbolic;
		auto par_D_isSymb = robot.parameters["par_D"].is_symbolic;
		auto par_Dm_isSymb = robot.parameters["par_Dm"].is_symbolic;
		auto par_Mm_isSymb = robot.parameters["par_Mm"].is_symbolic;

		auto K = K_order ? robot.get_model("k") : 0;
		auto D = D_order ? robot.get_model("d") : 0;
		auto Dm = Dm_order ? robot.get_model("dm") : 0;
		auto Mm = robot.get_model("Mm");

		// - symbolic par construction - //
		std::vector<casadi::SX> par_symb_K;
		std::vector<casadi::SX> par_symb_D;
		std::vector<casadi::SX> par_symb_Dm;
		std::vector<casadi::SX> par_symb_Mm;
		// parse par_K
		for (int i=0; i<par_K.size1(); i++){
			if (par_K_isSymb[i]){
				par_symb_K.push_back(par_K(i));
			}
		}
		// parse par_D
		for (int i=0; i<par_D.size1(); i++){
			if (par_D_isSymb[i]){
				par_symb_D.push_back(par_D(i));
			}
		}
		// parse par_Dm
		for (int i=0; i<par_Dm.size1(); i++){
			if (par_Dm_isSymb[i]){
				par_symb_Dm.push_back(par_Dm(i));
			}
		}
		// parse par_Mm
		for (int i=0; i<par_Mm.size1(); i++){
			if (par_Mm_isSymb[i]){
				par_symb_Mm.push_back(par_Mm(i));
			}
		}
		casadi::SX par_K_tmp = casadi::SX::vertcat(par_symb_K);
		casadi::SX par_D_tmp = casadi::SX::vertcat(par_symb_D);
		casadi::SX par_Dm_tmp = casadi::SX::vertcat(par_symb_Dm);
		casadi::SX par_Mm_tmp = casadi::SX::vertcat(par_symb_Mm);

		if (par_K_tmp.size1() != 0){
			casadi::SX reg_K = casadi::SX::jacobian(K, par_K_tmp);
			if (!robot.add_function("reg_k", reg_K, {"q", "x"}, "Regressor matrix of the coupling stiffness")) return 0;
		}
		if (par_D_tmp.size1() != 0){
			casadi::SX reg_D = casadi::SX::jacobian(D, par_D_tmp);
			if (!robot.add_function("reg_d", reg_D, {"dq", "dx"}, "Regressor matrix of the coupling damping")) return 0;
		}
		if (par_Dm_tmp.size1() != 0){
			casadi::SX reg_Dm = casadi::SX::jacobian(Dm, par_Dm_tmp);
			if (!robot.add_function("reg_dm", reg_Dm, {"dx"}, "Regressor matrix of the motor friction")) return 0;
		}
		if (par_Mm_tmp.size1() != 0){
			casadi::SX reg_Mm = casadi::SX::jacobian(mtimes(Mm,ddx), par_Mm_tmp);
			if (!robot.add_function("reg_Mm", reg_Mm, {"ddx"}, "Regressor matrix of the motor friction")) return 0;
		}
		
		return 1;
	}

	int compute_reg_J(Robot& robot){
		// parameters from robot
		int nj = robot.get<int>("numJoints");
		// auto par_DHtable = robot.get_model("par_DHtable");
		// auto par_world2L0 = robot.get_model("par_world2L0");
		// auto par_Ln2EE = robot.get_model("par_Ln2EE");
		// auto DHtable_isSymb = robot.parameters["par_DHtable"].is_symbolic;
		// auto world2L0_isSymb = robot.parameters["par_world2L0"].is_symbolic;
		// auto Ln2EE_isSymb = robot.parameters["par_Ln2EE"].is_symbolic;
		auto q = robot.get_model("q");
		auto dq = robot.get_model("dq");
		auto w = robot.get_model("w");

		// auto dims = par_DHtable.size();
		// casadi::SX DH_vect = casadi::SX::reshape(par_DHtable, dims.first*dims.second, 1);

		casadi::SX J = robot.get_model("J_ee");
		// std::cout <<"J: " << J << std::endl;

		// - symbolic par construction of par - //
		std::vector<casadi::SX> par_symb;
		// casadi::SX par = casadi::SX::vertcat({DH_vect, par_world2L0, par_Ln2EE});
		// int sz = 0;
		// parse DH
		// for (int i=0; i<par_DHtable.size1(); i++){
		// 	if (DHtable_isSymb[i]){
		// 		par_symb.push_back(par_DHtable(i));
		// 	}
		// }
		// // parse world2L0
		// for (int i=0; i<par_world2L0.size1(); i++){
		// 	if (world2L0_isSymb[i]){
		// 		par_symb.push_back(par_world2L0(i));
		// 	}
		// }
		// // parse Ln2EE
		// for (int i=0; i<par_Ln2EE.size1(); i++){
		// 	if (Ln2EE_isSymb[i]){
		// 		par_symb.push_back(par_Ln2EE(i));
		// 	}
		// }

		for (const auto& arg : robot.functions["J_ee"].args){
			if (arg != "q"){
				par_symb.push_back(robot.parameters[arg].get_symb_resized());
			}
		}

		// par.resize(sz,1);
		casadi::SX par = casadi::SX::vertcat(par_symb);
		// std::cout <<"par: " << par << std::endl;
		
		if (par.size1() != 0){
			casadi::SX Jdq = casadi::SX::mtimes(J, dq);
			// std::cout <<"Jdq: " << Jdq << std::endl;
			casadi::SX reg_Jdq = casadi::SX::jacobian(Jdq, par);
			// std::cout <<"reg_Jdq: " << reg_Jdq << std::endl;
			// reg_Jdq += casadi::SX::jacobian(Jdq, par_world2L0);
			// reg_Jdq += casadi::SX::jacobian(Jdq, par_Ln2EE);
			casadi::SX JTw = casadi::SX::mtimes(J.T(), w);
			// std::cout <<"JTw: " << JTw << std::endl;
			casadi::SX reg_JTw = casadi::SX::jacobian(JTw, par);
			// std::cout <<"reg_JTw: " << reg_JTw << std::endl;

			std::vector<std::string> arg_list;
			arg_list = {"q", "dq", "par_KIN", "par_world2L0", "par_Ln2EE"};
			// std::cout << "par_list: " << par_symb << std::endl;
			if (!robot.add_function("reg_Jdq", reg_Jdq, arg_list, "Regressor matrix of the quantity J*dq")) return 0;

			arg_list = {"q", "w", "par_KIN", "par_world2L0", "par_Ln2EE"};
			if (!robot.add_function("reg_JTw", reg_JTw, arg_list, "Regressor matrix of the quantity J^T*w")) return 0;
		}
		

		return 1;
	}

    int compute_regressors(Robot& robot, bool advanced){
		int ret = 1;
		int numSoftJoints = (robot.properties.count("numSoftJoints")) ? robot.get<int>("numSoftJoints") : 0;
		int Dl_order = (robot.properties.count("Dl_order")) ? robot.get<int>("Dl_order") : 0;

		if (!compute_Yr(robot)) ret=0;
		if (!compute_reg_J(robot)) ret=0;
		if (Dl_order>0){
			if (!compute_reg_Dl(robot)) ret=0;
		}
		if (numSoftJoints){
			if (!compute_reg_elastic(robot)) ret=0;
		}
		
		return ret;
	}

    

}