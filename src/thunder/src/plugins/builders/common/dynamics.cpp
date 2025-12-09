#include "plugins/builders/common/dynamics.h"
#include "plugins/builders/common/kinematics.h"
#include "utils.h"

using std::string;
using std::vector;

namespace thunder_ns{

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

		casadi::SX C(n,n);
		C = C123 + C132 - C132.T();

		return C;
	}

	std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(Robot& robot){
		// parameters from robot
		int numJoints = robot.get<int>("numJoints");
		const int _nParLink_ = robot.get<const int>("STD_PAR_LINK");
		vector<string> jointsType = robot.get<vector<string>>("jointsType");
		// const auto& q = robot.model["q"];
		// const auto& par_world2L0 = robot.model["par_world2L0"];
		// const auto& par_DYN = robot.model["par_DYN"];
		auto q = robot.get_model("q");
		auto par_world2L0 = robot.get_model("par_world2L0");
		auto par_DYN = robot.get_model("par_DYN");

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
		auto world_rot = get_transform(par_world2L0)(r_rot_idx, r_rot_idx);

		for (int i = 0; i < numJoints; i++) {
			casadi::SX k0(3,1);             // versor of joint i
			casadi::SX O_0i(3,1);           // distance of joint i from joint 0
			casadi::SX T_0i(4,4);           // matrix tranformation of joint i from joint 0
			
			casadi::SX O_Ci(3,1);
			casadi::SX R0i;
			
			T_0i = robot.get_model("T_0_"+std::to_string(i+1));
			k0 = T_0i(r_rot_idx, 2);
			O_0i = T_0i(r_tra_idx, 3);
			
			R0i = T_0i(r_rot_idx,r_rot_idx);
			O_Ci = O_0i + mtimes(R0i,_distCM_[i]);
			
			// // First column of jacobian
			// if ((jointsType[0] == "P")||(jointsType[0] == "P_SEA")) {
			// 	Jci_pos(allRows,0) = k0;
			// } else if ((jointsType[0] == "R")||(jointsType[0] == "R_SEA")) {
			// 	Jci_pos(allRows,0) = mtimes(hat(k0),O_Ci);
			// 	Ji_or(allRows,0) = k0;
			// } else {
			// 	throw std::runtime_error("DHJac: Error joint type");
			// }

			// Rest of columns of jacobian
			for (int j = 0; j <= i; j++) {
				// Init variables of column j-th of jacobian each cycle
				casadi::SX kj(3,1);             // versor of joint j-1
				casadi::SX O_jCi(3,1);          // distance of joint i from joint j-1
				casadi::SX T_0j(4,4);           // matrix tranformation of joint i from joint j-1

				T_0j = robot.get_model("T_0_"+std::to_string(j+1));
				kj = T_0j(r_rot_idx, 2);
				O_jCi = O_Ci - T_0j(r_tra_idx, 3);

				if ((jointsType[j] == "P")||(jointsType[j] == "P_SEA")) {
					Jci_pos(allRows, j) = kj;
				} else if ((jointsType[j] == "R")||(jointsType[j] == "R_SEA")) {
					Jci_pos(allRows, j) = mtimes(hat(kj),O_jCi);
					Ji_or(allRows, j) = kj;
				} else {
					throw std::runtime_error("DHJac: Error joint type");
				}
			}

			// Add offset from world-frame transformation
			// Jci_pos = mtimes(par_world2L0.get_rotation(),Jci_pos);
			// Ji_or = mtimes(par_world2L0.get_rotation(),Ji_or);
			// Jci_pos = mtimes(world_rot,Jci_pos);
			// Ji_or = mtimes(world_rot,Ji_or);
			
			Ji_v[i] = Jci_pos;
			Ji_w[i] = Ji_or;

			Ji[i] = casadi::SX::vertcat({Ji_v[i], Ji_w[i]});
			// std::cout<<"Ji[i]: "<<Ji[i]<<std::endl;
			std::vector<std::string> arg_list = {"q", "par_DHtable", "par_world2L0", "par_DYN"};
			robot.add_function("J_cm_"+std::to_string(i+1), Ji[i], arg_list, "Jacobian of center of mass of link "+std::to_string(i+1));
		}

		return std::make_tuple(Ji_v, Ji_w);
	}

	int compute_MCG(Robot& robot){
		// parameters from robot
		int nj = robot.get<int>("numJoints");
		const int nParLink = robot.get<const int>("STD_PAR_LINK");
		auto q = robot.get_model("q");
		auto dq = robot.get_model("dq");
		auto par_DYN = robot.get_model("par_DYN");
		auto par_gravity = robot.get_model("par_gravity");

		auto par_inertial = createInertialParameters(nj, nParLink, par_DYN);
		casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		// casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);
		
		casadi::SX dq_sel_ = dq_select(dq);
		casadi::SX T0i;
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
			T0i = robot.get_model("T_0_"+std::to_string(i+1));
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
		arg_list = {"q", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("M", M, arg_list, "Manipulator mass matrix");
		arg_list = {"q", "dq", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("C", C, arg_list, "Manipulator Coriolis matrix");
		arg_list = {"q", "dq", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("C_std", C_std, arg_list, "Classic formulation of the manipulator Coriolis matrix");
		arg_list = {"q", "par_DHtable", "par_world2L0", "par_gravity", "par_DYN"};
		robot.add_function("G", G, arg_list, "Manipulator gravity terms");

		return 1;
	}

	int compute_elastic(Robot& robot){
		// parameters from robot
		int nj = robot.get<int>("numJoints");
		bool ELASTIC = robot.get<bool>("ELASTIC");

		if (ELASTIC > 0){
			int numElasticJoints = robot.get<int>("numElasticJoints");
			vector<short> isElasticJoint = robot.get<vector<short>>("isElasticJoint");
			int K_order = robot.get<int>("K_order");
			int D_order = robot.get<int>("D_order");
			int Dm_order = robot.get<int>("Dm_order");
			const auto& q = robot.get_model("q");
			const auto& x = robot.get_model("x");
			const auto& dq = robot.get_model("dq");
			const auto& dx = robot.get_model("dx");
			const auto& par_K = robot.get_model("par_K");
			const auto& par_D = robot.get_model("par_D");
			const auto& par_Dm = robot.get_model("par_Dm");
			const auto& par_Mm = robot.get_model("par_Mm");

			// casadi::SX K(numElasticJoints,1);
			// casadi::SX D(numElasticJoints,1);
			// casadi::SX Dm(numElasticJoints,1);
			// casadi::SX Mm(numElasticJoints,1);
			// for (int i=0; i<numElasticJoints; i++){
			// 	for (int ord=0; ord<K_order; ord++){
			// 		K(i) += pow(x(i)-q(i), ord+1) * par_K(i*K_order+ord);
			// 	}
			// 	for (int ord=0; ord<D_order; ord++){
			// 		D(i) += pow(dx(i)-dq(i), ord+1) * par_D(i*D_order+ord);
			// 	}
			// 	for (int ord=0; ord<Dm_order; ord++){
			// 		Dm(i) += pow(dx(i), ord+1) * par_Dm(i*Dm_order+ord);
			// 	}
			// }
			// function vectors
			casadi::SX k(numElasticJoints,1);
			casadi::SX d(numElasticJoints,1);
			casadi::SX dm(numElasticJoints,1);
			// matrixes
			std::vector<casadi::SX> K_vec(K_order);
			std::vector<casadi::SX> D_vec(D_order);
			std::vector<casadi::SX> Dm_vec(Dm_order);
			// casadi::SX K(numElasticJoints,numElasticJoints);
			// casadi::SX D(numElasticJoints,numElasticJoints);
			// casadi::SX Dm(numElasticJoints,numElasticJoints);
			casadi::SX Mm(numElasticJoints,numElasticJoints);
			for (int i=0; i<numElasticJoints; i++){
				for (int ord=0; ord<K_order; ord++){
					k(i) += pow(x(i)-q(i), 2*ord+1) * par_K(i*K_order+ord);	// ^1,3,5...
					K_vec[ord].resize(numElasticJoints,numElasticJoints);
					K_vec[ord](i,i) = par_K(i*K_order + ord);
				}
				for (int ord=0; ord<D_order; ord++){
					if (ord%2 == 0){
						d(i) += pow(dx(i)-dq(i), ord+1) * par_D(i*D_order+ord);
					} else {
						d(i) += sqrt(pow(dx(i)-dq(i), 2)) * pow(dx(i)-dq(i), ord) * par_D(i*D_order+ord);
					}
					D_vec[ord].resize(numElasticJoints,numElasticJoints);
					D_vec[ord](i,i) = par_D(i*D_order + ord);
				}
				for (int ord=0; ord<Dm_order; ord++){
					if (ord%2 == 0){
						dm(i) += pow(dx(i), ord+1) * par_Dm(i*Dm_order+ord);
					} else {
						dm(i) += sqrt(pow(dx(i), 2)) * pow(dx(i), ord) * par_Dm(i*Dm_order+ord);
					}
					Dm_vec[ord].resize(numElasticJoints,numElasticJoints);
					Dm_vec[ord](i,i) = par_Dm(i*Dm_order + ord);
				}
				Mm(i,i) = par_Mm(i);
			}
			std::vector<std::string> arg_list;
			if (K_order > 0) {
				arg_list = {"q", "x", "par_K"};
				robot.add_function("k", k, arg_list, "SEA manipulator elastic coupling");
				arg_list = {"par_K"};
				for (int ord=0; ord<K_order; ord++){ 
					robot.add_function("K"+std::to_string(2*ord+1), K_vec[ord], arg_list, "SEA manipulator elastic coupling, order "+std::to_string(2*ord+1));
				}
			}
			if (D_order > 0) {
				arg_list = {"dq", "dx", "par_D"};
				robot.add_function("d", d, arg_list, "SEA manipulator dampind coupling");
				arg_list = {"par_D"};
				for (int ord=0; ord<D_order; ord++){ 
					robot.add_function("D"+std::to_string(ord+1), D_vec[ord], arg_list, "SEA manipulator damping coupling, order "+std::to_string(ord+1));
				}
			}
			if (Dm_order > 0) {
				arg_list = {"dx", "par_Dm"};
				robot.add_function("dm", dm, arg_list, "SEA manipulator motor damping");
				arg_list = {"par_Dm"};
				for (int ord=0; ord<Dm_order; ord++){ 
					robot.add_function("Dm"+std::to_string(ord+1), Dm_vec[ord], arg_list, "SEA manipulator motor damping, order "+std::to_string(ord+1));
				}
			}
			arg_list = {"par_Mm"};
			robot.add_function("Mm", Mm, arg_list, "SEA manipulator motor inertia");

			return 1;
		} else return 0;
	}

	int compute_Dl(Robot& robot){
		// parameters from robot
		int Dl_order = robot.get<int>("Dl_order");

		if (Dl_order > 0){
			int nj = robot.get<int>("numJoints");
			const auto& dq = robot.get_model("dq");
			const auto& par_Dl = robot.get_model("par_Dl");

			casadi::SX dl(nj,1);
			std::vector<casadi::SX> Dl_vec(Dl_order);
			for (int i=0; i<nj; i++){
				for (int ord=0; ord<Dl_order; ord++){
					if (ord%2 == 0){
						dl(i) += pow(dq(i), ord+1) * par_Dl(i*Dl_order+ord);
					} else {
						dl(i) += sqrt(pow(dq(i), 2)) * pow(dq(i), ord) * par_Dl(i*Dl_order+ord);
					}
					Dl_vec[ord].resize(nj,nj);
					Dl_vec[ord](i,i) = par_Dl(i*Dl_order + ord);
				}
			}
			std::vector<std::string> arg_list;
			arg_list = {"dq", "par_Dl"};
			robot.add_function("dl", dl, arg_list, "Manipulator link friction");
			arg_list = {"par_Dl"};
			for (int ord=0; ord<Dl_order; ord++){ 
				robot.add_function("Dl"+std::to_string(ord+1), Dl_vec[ord], arg_list, "SEA manipulator link damping, order "+std::to_string(ord+1));
			}
			return 1;
		} else return 0;
	}

	int compute_dyn_derivatives(Robot& robot){
		auto q = robot.get_model("q");
		auto dq = robot.get_model("dq");
		auto ddq = robot.get_model("ddq");
		auto d3q = robot.get_model("d3q");
		auto d4q = robot.get_model("d4q");
		auto M = robot.get_model("M");
		auto C = robot.get_model("C");
		auto G = robot.get_model("G");

		// - Mass derivatives - //
		casadi::SX dM = casadi::SX::jtimes(M,q,dq);
		casadi::SX ddM = casadi::SX::jtimes(dM,q,dq) + casadi::SX::jtimes(dM,dq,ddq);
		std::vector<std::string> arg_list = {"q", "dq", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("M_dot", dM, arg_list, "Time derivative of the mass matrix");
		arg_list = {"q", "dq", "ddq", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("M_ddot", ddM, arg_list, "Second time derivative of the mass matrix");

		// - Coriolis derivatives - //
		casadi::SX dC = casadi::SX::jtimes(C,q,dq) + casadi::SX::jtimes(C,dq,ddq);
		casadi::SX ddC = casadi::SX::jtimes(dC,q,dq) + casadi::SX::jtimes(dC,dq,ddq) + casadi::SX::jtimes(dC,ddq,d3q);
		arg_list = {"q", "dq", "ddq", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("C_dot", dC, arg_list, "Time derivative of the Coriolis matrix");
		arg_list = {"q", "dq", "ddq", "d3q", "par_DHtable", "par_world2L0", "par_DYN"};
		robot.add_function("C_ddot", ddC, arg_list, "Second time derivative of the Coriolis matrix");

		// - Gravity derivatives - //
		casadi::SX dG = casadi::SX::jtimes(G,q,dq);
		casadi::SX ddG = casadi::SX::jtimes(dG,q,dq) + casadi::SX::jtimes(dG,dq,ddq);
		arg_list = {"q", "dq", "par_DHtable", "par_world2L0", "par_gravity", "par_DYN"};
		robot.add_function("G_dot", dG, arg_list, "Time derivative of the gravity vector");
		arg_list = {"q", "dq", "ddq", "par_DHtable", "par_world2L0", "par_gravity", "par_DYN"};
		robot.add_function("G_ddot", ddG, arg_list, "Second time derivative of the gravity vector");

		return 1;
	}

	// --- REG/DYN conversions --- //
	int compute_reg_dyn_conversions(Robot& robot){
		int numJoints = robot.get<int>("numJoints");
		const int STD_PAR_LINK = robot.get<const int>("STD_PAR_LINK");

		// - reg2dyn - //
		SX par_REG = robot.get_model("par_REG");
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
		robot.add_function("reg2dyn", reg2dyn, {"par_REG"}, "Conversion from regressor to dynamic parameters");

		// - dyn2reg - //
		SX par_DYN = robot.get_model("par_DYN");
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
		robot.add_function("dyn2reg", dyn2reg, {"par_DYN"}, "Conversion from dynamic to regressor parameters");

		// - Update regressor parameters - //
		robot.set("par_REG", robot.get("dyn2reg"));

		return 1;
	}


	// - Compute everything - //
	int compute_dynamics(Robot& robot, bool advanced){
		bool ret = true;
		if (!compute_MCG(robot)) ret=false;
		if (!compute_Dl(robot)) ret=false;
		if (!compute_elastic(robot)) ret=false;
		if (!compute_reg_dyn_conversions(robot)) ret=false;
		
		if (advanced){
			if (!compute_dyn_derivatives(robot)) ret=false;
		}

		return ret;
	}

}