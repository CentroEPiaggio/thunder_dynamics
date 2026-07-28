#include "plugins/builders/reg_builder.h"
#include "plugins/builders/dyn_builder.h"
#include "utils.h"

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns {

    void RegBuilder::init(std::shared_ptr<Robot> robot){
		// --- Basic Robot properties --- //
		int numJoints = robot->get<int>("numJoints");
		int ndof = robot->get<int>("ndof");

		// - Parameters per link
		int STD_PAR_LINK = 10;
		if (robot->properties.count("STD_PAR_LINK")){
			STD_PAR_LINK = robot->get<int>("STD_PAR_LINK");
		} else {
			robot->add_property<int>("STD_PAR_LINK", STD_PAR_LINK, "int", "Standard number of dynamic parameters per link", true);
		}

		// --- Variables --- //
		// - Slotine regressor - //
		robot->add_variable("dqr", SX::sym("dqr",ndof,1), vector<double>(ndof,0), {1}, "Velocity reference", true);
		robot->add_variable("ddqr", SX::sym("ddqr",ndof,1), vector<double>(ndof,0), {1}, "Acceleration reference", true);
		// - Kinematic regressor - //
		robot->add_variable("w", SX::sym("w",6,1), vector<double>(6,0), {1}, "Wrench", true);
	}

	casadi::SXVector RegBuilder::createQ() {

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
	
	casadi::SXVector RegBuilder::createE() {
		
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

	int RegBuilder::compute_Yr(std::shared_ptr<Robot> robot){
		// parameters from robot
		int nj = robot->get<int>("numJoints");
		int ndof = robot->get<int>("ndof");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		const vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto dqr = robot->get_model("dqr");
		auto ddqr = robot->get_model("ddqr");
		auto par_gravity = robot->get_model("par_gravity");
		DynBuilder dyn;
		
		// regressor computation
		casadi::SXVector E_ = createE();
		casadi::SXVector Q_ = createQ();
		casadi::SX dq_sel_ = dyn.dq_select(dq);
		
		casadi::SX Twi(4,4);
		casadi::SX Ji(6, ndof);
		casadi::SX Jvi(3, ndof);
		casadi::SX Jwi(3, ndof);

		casadi::SX g = par_gravity;

		casadi::SX Yr(ndof, nParLink*nj);
		casadi::SX reg_M(ndof, nParLink*nj);
		casadi::SX reg_C(ndof, nParLink*nj);
		casadi::SX reg_G(ndof, nParLink*nj);
				
		casadi::Slice allRows(0, ndof);
		casadi::Slice allCols(0, ndof);          
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

			//get the parent transform and Jacobian
			int parent_id = jointsParent[i];
			if (parent_id == -1) {
				Twi = SX::eye(4);
				Ji = SX::zeros(6, ndof);
			} else {
				Twi = robot->get_model("T_w_"+std::to_string(parent_id));
				Ji = robot->get_model("J_"+std::to_string(parent_id));
			}
			
			casadi::SX Rwi = Twi(selR,selR);
			Jvi = Ji(sel_v, allCols);
			Jwi = Ji(sel_w, allCols);

			// ------------------------- Y0r_i -------------------------- //
			
			casadi::SX M0_i = mtimes(Jvi.T(),Jvi);
			casadi::SX C = dyn.stdCmatrix(M0_i, q, dq, dq_sel_);

			casadi::SX dX0r_i = mtimes(M0_i, ddqr);
			casadi::SX W0r_i = -mtimes(C, dqr);
			casadi::SX Z0r_i = -mtimes(Jvi.T(),g);
			
			casadi::SX Y0r_i = dX0r_i - W0r_i + Z0r_i;
			
			// ------------------------- Y1r_i -------------------------- //
			
			casadi::SX dX1r_i(ndof,3);
			casadi::SX W1r_i(ndof,3);

			for (int l=0; l<3; l++) {

				casadi::SX Ql = Q_[l];
				casadi::SX M1l_i = casadi::SX::mtimes({Jwi.T(),Rwi,Ql,Rwi.T(),Jvi}) - 
								   casadi::SX::mtimes({Jvi.T(),Rwi,Ql,Rwi.T(),Jwi});
				casadi::SX C = dyn.stdCmatrix(M1l_i, q, dq, dq_sel_);

				dX1r_i(allRows,l) = mtimes(M1l_i, ddqr);
				W1r_i(allRows,l) = -mtimes(C, dqr);
			}
			casadi::SX Z1r_i= -(jacobian(mtimes(Rwi.T(),g),q)).T();
			
			casadi::SX Y1r_i = dX1r_i - W1r_i + Z1r_i;

			// ------------------------- Y2r_i -------------------------- //

			casadi::SX dX2r_i(ndof,6);
			casadi::SX W2r_i(ndof,6);
			
			for (int l=0; l<6; l++) {

				casadi::SX El = E_[l];
				casadi::SX M2l_i = casadi::SX::mtimes({Jwi.T(),Rwi,El,Rwi.T(),Jwi});
				casadi::SX C = dyn.stdCmatrix(M2l_i, q, dq, dq_sel_);

				dX2r_i(allRows,l) = mtimes(M2l_i, ddqr);
				W2r_i(allRows,l) = -mtimes(C, dqr);
			}

			casadi::SX Y2r_i = dX2r_i - W2r_i;

			// ------------------- matrix regressors ------------------- //
			casadi::SX reg_M_i = horzcat(dX0r_i, dX1r_i, dX2r_i);
			casadi::SX reg_C_i = horzcat(-W0r_i, -W1r_i, -W2r_i);
			casadi::SX reg_G_i = horzcat(Z0r_i, Z1r_i, casadi::SX::zeros(ndof,6));

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
		arg_list = {"q", "dq", "dqr", "ddqr", "par_KIN", "par_gravity"};
		if (!robot->add_function("Yr", Yr, arg_list, "Manipulator regressor matrix")) return 0;
		arg_list = {"q", "ddqr", "par_KIN"};
		if (!robot->add_function("reg_M", reg_M, arg_list, "Regressor matrix of term M*ddqr")) return 0;
		arg_list = {"q", "dq", "dqr", "par_KIN"};
		if (!robot->add_function("reg_C", reg_C, arg_list, "Regressor matrix of term C*dqr")) return 0;
		arg_list = {"q", "par_KIN", "par_gravity"};
		if (!robot->add_function("reg_G", reg_G, arg_list, "Regressor matrix of term G")) return 0;

		return 1;
	}

	int RegBuilder::compute_reg_Dl(std::shared_ptr<Robot> robot){
		// parameters from robot
		// int nj = robot->get<int>("numJoints");
		// const int nParLink = robot->get<const int>("STD_PAR_LINK");
		int Dl_order = (robot->properties.count("Dl_order")) ? robot->get<int>("Dl_order") : 0;
		auto dq = robot->get_model("dq");
		if (Dl_order==0) return 0;
		auto par_Dl = robot->get_model("par_Dl");
		auto par_Dl_isSymb = robot->parameters["par_Dl"].is_symbolic;
		auto Dl = robot->get_model("dl");

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
			if (!robot->add_function("reg_dl", reg_Dl, {"dq"}, "Regressor matrix of the link friction")) return 0;

		}
		
		return 1;
	}

	int RegBuilder::compute_reg_elastic(std::shared_ptr<Robot> robot){
		// parameters from robot
		int nj = robot->get<int>("numJoints");
		int nej = robot->get<int>("numSoftJoints");
		int K_order = robot->get<int>("K_order");
		int D_order = robot->get<int>("D_order");
		int Dm_order = robot->get<int>("Dm_order");
		auto dq = robot->get_model("dq");
		auto dx = robot->get_model("dx");
		auto ddx = robot->get_model("ddx");
		auto par_K = robot->get_model("par_K");
		auto par_D = robot->get_model("par_D");
		auto par_Dm = robot->get_model("par_Dm");
		auto par_Mm = robot->get_model("par_Mm");
		auto par_K_isSymb = robot->parameters["par_K"].is_symbolic;
		auto par_D_isSymb = robot->parameters["par_D"].is_symbolic;
		auto par_Dm_isSymb = robot->parameters["par_Dm"].is_symbolic;
		auto par_Mm_isSymb = robot->parameters["par_Mm"].is_symbolic;

		auto K = K_order ? robot->get_model("k") : 0;
		auto D = D_order ? robot->get_model("d") : 0;
		auto Dm = Dm_order ? robot->get_model("dm") : 0;
		auto Mm = robot->get_model("Mm");

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
			if (!robot->add_function("reg_k", reg_K, {"q", "x"}, "Regressor matrix of the coupling stiffness")) return 0;
		}
		if (par_D_tmp.size1() != 0){
			casadi::SX reg_D = casadi::SX::jacobian(D, par_D_tmp);
			if (!robot->add_function("reg_d", reg_D, {"dq", "dx"}, "Regressor matrix of the coupling damping")) return 0;
		}
		if (par_Dm_tmp.size1() != 0){
			casadi::SX reg_Dm = casadi::SX::jacobian(Dm, par_Dm_tmp);
			if (!robot->add_function("reg_dm", reg_Dm, {"dx"}, "Regressor matrix of the motor friction")) return 0;
		}
		if (par_Mm_tmp.size1() != 0){
			casadi::SX reg_Mm = casadi::SX::jacobian(mtimes(Mm,ddx), par_Mm_tmp);
			if (!robot->add_function("reg_Mm", reg_Mm, {"ddx"}, "Regressor matrix of the motor friction")) return 0;
		}
		
		return 1;
	}

	int RegBuilder::compute_reg_J(std::shared_ptr<Robot> robot){
		// parameters from robot
		const int nj = robot->get<int>("numJoints");
		const vector<bool> jointsAvailable = robot->get<vector<bool>>("jointsAvailable");
		const vector<string> jointsName = robot->get<vector<string>>("jointsName");
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto w = robot->get_model("w");

		// - cycle on available joints - //
		for (int i=0; i<nj; i++) {
			if (jointsAvailable[i]) {
				string J_str = "J_"+jointsName[i];
				casadi::SX J = robot->get_model(J_str);
				// std::cout <<"J: " << J << std::endl;

				// - symbolic par construction of par - //
				std::vector<casadi::SX> par_symb;

				for (const auto& arg : robot->functions[J_str].args){
					if (arg != "q"){
						par_symb.push_back(robot->parameters[arg].get_symb_resized());
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
					casadi::SX JTw = casadi::SX::mtimes(J.T(), w);
					// std::cout <<"JTw: " << JTw << std::endl;
					casadi::SX reg_JTw = casadi::SX::jacobian(JTw, par);
					// std::cout <<"reg_JTw: " << reg_JTw << std::endl;

					std::vector<std::string> arg_list;
					arg_list = {"q", "dq", "par_KIN"};
					// std::cout << "par_list: " << par_symb << std::endl;
					if (!robot->add_function("reg_Jdq_"+jointsName[i], reg_Jdq, arg_list, "Regressor matrix of the quantity J*dq of link "+jointsName[i])) return 0;

					arg_list = {"q", "w", "par_KIN"};
					if (!robot->add_function("reg_JTw_"+jointsName[i], reg_JTw, arg_list, "Regressor matrix of the quantity J^T*w of link "+jointsName[i])) return 0;
				}
			}
		}
		return 1;
	}

	int RegBuilder::compute_regressors(std::shared_ptr<Robot> robot, bool advanced){
		int ret = 1;
		int numSoftJoints = (robot->properties.count("numSoftJoints")) ? robot->get<int>("numSoftJoints") : 0;
		int Dl_order = (robot->properties.count("Dl_order")) ? robot->get<int>("Dl_order") : 0;

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

	void RegBuilder::build(std::shared_ptr<Robot> robot) {
		init(robot);
		debug_log("Starting regressor computations", VERB_INFO);
		compute_regressors(robot);
		debug_log("Regressors computed", VERB_INFO);
	}

}