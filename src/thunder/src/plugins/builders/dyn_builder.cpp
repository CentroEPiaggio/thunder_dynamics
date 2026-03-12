#include "plugins/builders/dyn_builder.h"
#include "utils.h"

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns {

    std::tuple<casadi::SXVector, casadi::SXVector, casadi::SXVector> DynBuilder::createInertialParameters(int nj, int nParLink, casadi::SX par_DYN){
		
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

	casadi::SX DynBuilder::dq_select(const casadi::SX& dq) {
		int n = dq.size1();
		
		casadi::Slice allRows;
		casadi::SX mat_dq = casadi::SX::zeros(n, n * n);
		for (int i = 0; i < n; i++) {
			casadi::Slice sel_col(i*n,(i+1)*n);   // Select columns
			mat_dq(allRows, sel_col) = casadi::SX::eye(n)*dq(i);
		}
		
		return mat_dq;
	}

	casadi::SX DynBuilder::stdCmatrix(const casadi::SX& M, const casadi::SX& q, const casadi::SX& dq, const casadi::SX& dq_sel_) {
		int n = q.size1();

		casadi::SX jac_M = jacobian(M,q);
		
		casadi::SX C123 = reshape(mtimes(jac_M,dq),n,n);
		casadi::SX C132 = mtimes(dq_sel_,jac_M);
		casadi::SX C231 = C132.T();

		casadi::SX C(n,n);
		C = (C123 + C132 - C231)/2;

		return C;
	}

	casadi::SX DynBuilder::stdCmatrix_classic(const casadi::SX& M, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_) {
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

		casadi::SX C(n,n);
		C = C123 + C132 - C132.T();

		return C;
	}

	std::tuple<casadi::SXVector,casadi::SXVector> DynBuilder::DHJacCM(std::shared_ptr<Robot> robot){
		// parameters from robot
		int nj = robot->get<int>("numJoints");
		const int _nParLink_ = robot->get<const int>("STD_PAR_LINK");
		vector<string> jointsType = robot->get<vector<string>>("jointsType");
		// const auto& q = robot->model["q"];
		// const auto& par_world2L0 = robot->model["par_world2L0"];
		// const auto& par_DYN = robot->model["par_DYN"];
		auto q = robot->get_model("q");
		auto par_world2L0 = robot->get_model("par_world2L0");
		auto par_DYN = robot->get_model("par_DYN");

		auto par_inertial = createInertialParameters(nj, _nParLink_, par_DYN);
		// casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		// casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);

		casadi::SXVector Ji_v(nj); // vector of matrix Ji_v
		casadi::SXVector Ji_w(nj); // vector of matrix Ji_w
		casadi::SXVector Ji(nj);		// complete jacobian
		casadi::Slice r_tra_idx(0, 3);      // select translation vector of T()
		casadi::Slice r_rot_idx(0, 3);      // select k versor of T()
		casadi::Slice allRows;              // Select all rows
		auto world_rot = get_transform_ypr(par_world2L0)(r_rot_idx, r_rot_idx);

		for (int i = 0; i < nj; i++) {
			SX T_wi = robot->get_model("T_w_"+std::to_string(i+1));

			SX Rwi = T_wi(r_rot_idx, r_rot_idx);
			SX d_Ci = T_wi(r_tra_idx, 3) + mtimes(Rwi,_distCM_[i]);		// center of mass distance
			SX Jci_pos = SX::jacobian(d_Ci, q); 	// matrix of velocity jacobian
			SX Ji_or(3, nj);   				// matrix of omega jacobian

			// Loop over joints and build columns
			for (int j=0; j<nj; ++j) {
				// Partial derivative dR/dq_j  (3x3)
				SX dR_dqj = SX::jacobian(SX::reshape(Rwi, 9, 1), q(j));
				dR_dqj = SX::reshape(dR_dqj, 3, 3);

				// S_j = dR/dq_j * R^T  (3x3 skew-symmetric)
				SX Sj = SX::mtimes(dR_dqj, Rwi.T());

				// Extract angular velocity vector from skew matrix
				SX wj = vect(Sj);

				// Set column j
				Ji_or(allRows, j) = wj;
			}
			
			Ji_v[i] = Jci_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			// std::cout<<"Ji[i]: "<<Ji[i]<<std::endl;
			std::vector<std::string> arg_list = {"q", "par_KIN", "par_world2L0", "par_DYN"};
			robot->add_function("J_cm_"+std::to_string(i+1), Ji[i], arg_list, "Jacobian of center of mass of link "+std::to_string(i+1));
		}

		return std::make_tuple(Ji_v, Ji_w);
	}

	int DynBuilder::compute_MCG(std::shared_ptr<Robot> robot){
		// parameters from robot
		int nj = robot->get<int>("numJoints");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto par_DYN = robot->get_model("par_DYN");
		auto par_gravity = robot->get_model("par_gravity");

		auto par_inertial = createInertialParameters(nj, nParLink, par_DYN);
		casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		// casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);
		
		casadi::SX dq_sel_ = dq_select(dq);
		casadi::SX Twi;
		std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

		casadi::SXVector Jci(nj);
		casadi::SXVector Jwi(nj);
		std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

		casadi::SX g = par_gravity;

		casadi::SX M(nj,nj);
		casadi::SX C(nj,nj);
		casadi::SX C_std(nj,nj);
		casadi::SX G(nj,1);
		
		casadi::SX Mi(nj,nj);
		casadi::SX Gi(1,nj);
		casadi::SX mi(1,1);
		casadi::SX Ii(3,3);
		casadi::Slice selR(0,3);

		J_tuple = DHJacCM(robot);
		Jci = std::get<0>(J_tuple);
		Jwi = std::get<1>(J_tuple);
		
		for (int i=0; i<nj; i++) {
			Twi = robot->get_model("T_w_"+std::to_string(i+1));
			// std::cout<<"Twi: "<<Twi<<std::endl;
			casadi::SX Rwi = Twi(selR,selR);
			// std::cout<<"Rwi: "<<Rwi<<std::endl;

			mi = _mass_vec_[i];
			// std::cout<<"mi: "<<mi<<std::endl;
			Ii = _J_3x3_[i];
			// std::cout<<"Ii: "<<Ii<<std::endl;
			Mi = mi * casadi::SX::mtimes({Jci[i].T(), Jci[i]}) + casadi::SX::mtimes({Jwi[i].T(),Rwi,Ii,Rwi.T(),Jwi[i]});
			// std::cout<<"Mi: "<<Mi<<std::endl;
			M = M + Mi;
			// std::cout<<"M: "<<M<<std::endl;

			Gi = -mi * casadi::SX::mtimes({g.T(),Jci[i]});
			G = G + Gi.T();
		}
		
		C = stdCmatrix(M,q,dq,dq_sel_);
		C_std = stdCmatrix_classic(M,q,dq,dq_sel_);

		std::vector<std::string> arg_list;
		arg_list = {"q", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("M", M, arg_list, "Manipulator mass matrix");
		arg_list = {"q", "dq", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("C", C, arg_list, "Manipulator Coriolis matrix");
		arg_list = {"q", "dq", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("C_std", C_std, arg_list, "Classic formulation of the manipulator Coriolis matrix");
		arg_list = {"q", "par_KIN", "par_world2L0", "par_gravity", "par_DYN"};
		robot->add_function("G", G, arg_list, "Manipulator gravity terms");

		return 1;
	}

	int DynBuilder::compute_Dl(std::shared_ptr<Robot> robot){
		// parameters from robot
		int Dl_order = (robot->properties.count("Dl_order")) ? robot->get<int>("Dl_order") : 0;

		if (Dl_order > 0){
			int nj = robot->get<int>("numJoints");
			const auto& dq = robot->get_model("dq");
			const auto& par_Dl = robot->get_model("par_Dl");

			casadi::SX dl(nj,1);
			std::vector<casadi::SX> Dl_vec(Dl_order);
			for (int i=0; i<nj; i++){
				for (int ord=0; ord<Dl_order; ord++){
					if (ord == 0){
						dl(i) += sign(dq(i)) * par_Dl(i*Dl_order+ord);
					} else if (ord%2 == 0){
						dl(i) += sqrt(pow(dq(i), 2)) * pow(dq(i), ord-1) * par_Dl(i*Dl_order+ord);
					} else {
						dl(i) += pow(dq(i), ord) * par_Dl(i*Dl_order+ord);
					}
					Dl_vec[ord].resize(nj,nj);
					Dl_vec[ord](i,i) = par_Dl(i*Dl_order + ord);
				}
			}
			std::vector<std::string> arg_list;
			arg_list = {"dq", "par_Dl"};
			robot->add_function("dl", dl, arg_list, "Manipulator link friction");
			arg_list = {"par_Dl"};
			for (int ord=0; ord<Dl_order; ord++){ 
				robot->add_function("Dl"+std::to_string(ord+1), Dl_vec[ord], arg_list, "SEA manipulator link damping, order "+std::to_string(ord+1));
			}
			return 1;
		} else return 0;
	}



	int DynBuilder::compute_dyn_derivatives(std::shared_ptr<Robot> robot){
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto ddq = robot->get_model("ddq");
		auto d3q = robot->get_model("d3q");
		auto d4q = robot->get_model("d4q");
		auto M = robot->get_model("M");
		auto C = robot->get_model("C");
		auto G = robot->get_model("G");

		// - Mass derivatives - //
		casadi::SX dM = casadi::SX::jtimes(M,q,dq);
		casadi::SX ddM = casadi::SX::jtimes(dM,q,dq) + casadi::SX::jtimes(dM,dq,ddq);
		std::vector<std::string> arg_list = {"q", "dq", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("M_dot", dM, arg_list, "Time derivative of the mass matrix");
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("M_ddot", ddM, arg_list, "Second time derivative of the mass matrix");

		// - Coriolis derivatives - //
		casadi::SX dC = casadi::SX::jtimes(C,q,dq) + casadi::SX::jtimes(C,dq,ddq);
		casadi::SX ddC = casadi::SX::jtimes(dC,q,dq) + casadi::SX::jtimes(dC,dq,ddq) + casadi::SX::jtimes(dC,ddq,d3q);
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("C_dot", dC, arg_list, "Time derivative of the Coriolis matrix");
		arg_list = {"q", "dq", "ddq", "d3q", "par_KIN", "par_world2L0", "par_DYN"};
		robot->add_function("C_ddot", ddC, arg_list, "Second time derivative of the Coriolis matrix");

		// - Gravity derivatives - //
		casadi::SX dG = casadi::SX::jtimes(G,q,dq);
		casadi::SX ddG = casadi::SX::jtimes(dG,q,dq) + casadi::SX::jtimes(dG,dq,ddq);
		arg_list = {"q", "dq", "par_KIN", "par_world2L0", "par_gravity", "par_DYN"};
		robot->add_function("G_dot", dG, arg_list, "Time derivative of the gravity vector");
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_world2L0", "par_gravity", "par_DYN"};
		robot->add_function("G_ddot", ddG, arg_list, "Second time derivative of the gravity vector");

		return 1;
	}

	// --- REG/DYN conversions --- //
	int DynBuilder::compute_reg_dyn_conversions(std::shared_ptr<Robot> robot){
		int numJoints = robot->get<int>("numJoints");
		const int STD_PAR_LINK = robot->get<const int>("STD_PAR_LINK");

		// - reg2dyn - //
		SX par_REG = robot->get_model("par_REG");
		SX reg2dyn = SX::zeros(par_REG.size());
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			SX p_reg(par_REG(p_idx));
			SX mass = p_reg(0);
			SX CoM = p_reg(casadi::Slice(1,4))/mass;
			SX I_tmp = mass * SX::mtimes(hat(CoM).T(), hat(CoM));
			SX I_reg = p_reg(casadi::Slice(4,10));
			SX I_tmp_v = SX::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			SX I = I_reg - I_tmp_v;
			reg2dyn(p_idx) = SX::vertcat({mass, CoM, I});
		}
		robot->add_function("reg2dyn", reg2dyn, {"par_REG"}, "Conversion from regressor to dynamic parameters");

		// - dyn2reg - //
		SX par_DYN = robot->get_model("par_DYN");
		SX dyn2reg = SX::zeros(par_DYN.size());
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			SX p_dyn(par_DYN(p_idx));
			SX mass = p_dyn(0);
			SX mCoM = mass*p_dyn(casadi::Slice(1,4));
			SX I_tmp = SX::mtimes(hat(mCoM).T(), hat(mCoM))/mass;
			SX I_dyn = p_dyn(casadi::Slice(4,10));
			SX I_tmp_v = SX::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			SX I = I_dyn + I_tmp_v;
			dyn2reg(p_idx) = SX::vertcat({mass, mCoM, I});
		}
		robot->add_function("dyn2reg", dyn2reg, {"par_DYN"}, "Conversion from dynamic to regressor parameters");

		// - Update regressor parameters - //
		robot->set("par_REG", robot->get("dyn2reg"));

		return 1;
	}

    void DynBuilder::build(std::shared_ptr<Robot> robot) {
		debug_log("Starting dynamic computations", VERB_INFO);

		int ret = 1;
		if (!compute_MCG(robot)) ret = 0;
		if (!compute_Dl(robot)) ret = 0;
		if (!compute_reg_dyn_conversions(robot)) ret = 0;
		if (1){
			if (!compute_dyn_derivatives(robot)) ret = 0;
		}

		// return ret;
		debug_log("Dynamics computed", VERB_INFO);
    }
}