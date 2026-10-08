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
		// classic Christoffel C, element by element (same matrix as stdCmatrix, slower to build)
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
		// Centre-of-mass Jacobians from the frame Jacobians of kin_builder: link i moves with frame parent(i),
		// so its centre of mass, at r = R c from the frame origin, has linear velocity v_o + w x r = v_o - hat(r) w.
		const int nj = robot->get<int>("numJoints");
		const int ndof = robot->get<int>("ndof");
		const int _nParLink_ = robot->get<const int>("STD_PAR_LINK");
		const vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		auto par_DYN = robot->get_model("par_DYN");

		auto par_inertial = createInertialParameters(nj, _nParLink_, par_DYN);
		casadi::SXVector _distCM_ = std::get<1>(par_inertial);

		casadi::SXVector Ji_v(nj); // vector of matrix Ji_v
		casadi::SXVector Ji_w(nj); // vector of matrix Ji_w
		casadi::Slice lin(0,3), ang(3,6), all;

		for (int i = 0; i < nj; i++) {
			const int parent_id = jointsParent[i];
			if (parent_id == -1) {		// links on the world frame do not move
				Ji_v[i] = SX::zeros(3, ndof);
				Ji_w[i] = SX::zeros(3, ndof);
				continue;
			}
			SX J = robot->get_model("J_"+std::to_string(parent_id));
			SX Rwi = robot->get_model("T_w_"+std::to_string(parent_id))(lin, lin);
			Ji_w[i] = J(ang, all);
			Ji_v[i] = J(lin, all) - mtimes(hat(mtimes(Rwi, _distCM_[i])), Ji_w[i]);
		}

		return std::make_tuple(Ji_v, Ji_w);
	}

	int DynBuilder::compute_dyn_lagrange(std::shared_ptr<Robot> robot){
		// parameters from robot
		const int nj = robot->get<int>("numJoints");
		const int ndof = robot->get<int>("ndof");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		const vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		// const vector<string> jointsName = robot->get<vector<string>>("jointsName");
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto par_DYN = robot->get_model("par_DYN");
		auto par_gravity = robot->get_model("par_gravity");

		auto par_inertial = createInertialParameters(nj, nParLink, par_DYN);
		casadi::SXVector _mass_vec_ = std::get<0>(par_inertial);
		// casadi::SXVector _distCM_ = std::get<1>(par_inertial);
		casadi::SXVector _J_3x3_ = std::get<2>(par_inertial);
		
		casadi::SX Twi;
		std::tuple<casadi::SXVector, casadi::SXVector> T_tuple;

		casadi::SXVector Jci(nj);
		casadi::SXVector Jwi(nj);
		std::tuple<casadi::SXVector, casadi::SXVector> J_tuple;

		casadi::SX g = par_gravity;

		casadi::SX M(ndof,ndof);
		casadi::SX G(ndof,1);
		
		casadi::SX Mi(ndof,ndof);
		casadi::SX Gi(1,ndof);
		casadi::SX mi(1,1);
		casadi::SX Ii(3,3);
		casadi::Slice selR(0,3);

		J_tuple = DHJacCM(robot);
		Jci = std::get<0>(J_tuple);
		Jwi = std::get<1>(J_tuple);
		
		for (int i=0; i<nj; i++) {
			//get the parent transform
			int parent_id = jointsParent[i];
			if (parent_id == -1) {
				Twi = SX::eye(4);
			} else {
				Twi = robot->get_model("T_w_"+std::to_string(parent_id));
			}
			// Twi = robot->get_model("T_w_"+std::to_string(i));
			// std::cout<<"Twi: "<<Twi<<std::endl;
			casadi::SX Rwi = Twi(selR,selR);
			// std::cout<<"Rwi: "<<Rwi<<std::endl;

			mi = _mass_vec_[i];
			// std::cout<<"mi: "<<mi<<std::endl;
			Ii = _J_3x3_[i];
			// std::cout<<"Ii: "<<Ii<<std::endl;
			Mi = mi * casadi::SX::mtimes({Jci[i].T(), Jci[i]}) + casadi::SX::mtimes({Jwi[i].T(),Rwi,Ii,Rwi.T(),Jwi[i]});
			// std::cout<<"Jci[i]: "<<Jci[i]<<std::endl;
			// std::cout<<"Jwi[i]: "<<Jwi[i]<<std::endl;
			// std::cout<<"Mi: "<<Mi<<std::endl;
			M = M + Mi;
			// std::cout<<"M: "<<M<<std::endl;

			Gi = -mi * casadi::SX::mtimes({g.T(),Jci[i]});
			G = G + Gi.T();
		}
		
		// Coriolis and centrifugal terms from the Lagrangian: C dq = dM/dt dq - 1/2 d(dq^T M dq)/dq
		SX Cdq = mtimes(SX::jtimes(M, q, dq), dq) - 0.5*SX::gradient(SX::mtimes({dq.T(), M, dq}), q);

		return add_dyn(robot, M, Cdq, G);
	}

	int DynBuilder::compute_J_cm(std::shared_ptr<Robot> robot){
		const int nj = robot->get<int>("numJoints");
		auto J_tuple = DHJacCM(robot);
		for (int i=0; i<nj; i++){
			SX Ji = SX::vertcat({std::get<0>(J_tuple)[i], std::get<1>(J_tuple)[i]});
			if (!robot->add_function("J_cm_"+std::to_string(i), Ji, {"q", "par_KIN", "par_DYN"}, "Jacobian of center of mass of link "+std::to_string(i))) return 0;
		}
		return 1;
	}

	// --- Spatial algebra for rnea: motion [v; w], force [f; n], 6x1 in one frame, linear part first --- //
	namespace {
		const casadi::Slice lin(0,3), ang(3,6);

		// v x m
		SX cross_motion(const SX& v, const SX& m){
			return SX::vertcat({cross(v(ang), m(lin)) + cross(v(lin), m(ang)), cross(v(ang), m(ang))});
		}
		// v x* f
		SX cross_force(const SX& v, const SX& f){
			return SX::vertcat({cross(v(ang), f(lin)), cross(v(ang), f(ang)) + cross(v(lin), f(lin))});
		}
		// motion vector from parent to child coordinates (R, r: child frame in the parent frame)
		SX motion_to_child(const SX& R, const SX& r, const SX& m){
			return SX::vertcat({mtimes(R.T(), m(lin) - cross(r, m(ang))), mtimes(R.T(), m(ang))});
		}
		// force vector from child to parent coordinates
		SX force_to_parent(const SX& R, const SX& r, const SX& f){
			SX f_lin = mtimes(R, f(lin));
			return SX::vertcat({f_lin, mtimes(R, f(ang)) + cross(r, f_lin)});
		}
		// link parameters p = [m, h = m CoM, I_O (Ixx Ixy Ixz Iyy Iyz Izz) about the frame origin], as par_REG
		SX inertia_O(const SX& p){
			return SX::vertcat({SX::horzcat({p(4), p(5), p(6)}), SX::horzcat({p(5), p(7), p(8)}), SX::horzcat({p(6), p(8), p(9)})});
		}
		// spatial inertia times a motion vector, linear in p
		SX inertia_times(const SX& p, const SX& u){
			SX h = p(casadi::Slice(1,4));
			return SX::vertcat({p(0)*u(lin) + cross(u(ang), h), mtimes(inertia_O(p), u(ang)) + cross(h, u(lin))});
		}
		// 6x6 spatial inertia and motion transform (parent to child, R, r: child frame in the parent frame), for CRBA
		SX spatial_inertia(const SX& p){
			SX h_hat = hat(p(casadi::Slice(1,4)));
			return SX::vertcat({SX::horzcat({p(0)*SX::eye(3), -h_hat}), SX::horzcat({h_hat, inertia_O(p)})});
		}
		SX motion_transform(const SX& R, const SX& r){
			return SX::vertcat({SX::horzcat({R.T(), -mtimes(R.T(), hat(r))}), SX::horzcat({SX::zeros(3,3), R.T()})});
		}
	}

	casadi::SX DynBuilder::rnea(std::shared_ptr<Robot> robot, const casadi::SX& par, const casadi::SX& dq, const casadi::SX& dqr, const casadi::SX& ddqr, const casadi::SX& g){
		// Modified RNEA (Niemeyer-Slotine): tau = M ddqr + C(q,dq) dqr + G, with C such that dM/dt - 2C is skew.
		// Empty dqr: standard RNEA (dqr = dq) with the plain v x* I v term, smaller expressions. Derivation in notes.md.
		// Same tree convention as compute_dyn_lagrange: joint i moves frame i (T_w_i) w.r.t. frame parent(i),
		// link i is the body attached to frame parent(i) (world if -1). Spatial vectors of frame i are in frame i.
		// par: link parameters in regressor form (dyn2reg), tau is linear in them.
		const int nj = robot->get<int>("numJoints");
		const int ndof = robot->get<int>("ndof");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		const vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		const vector<string> jointsType = robot->get<vector<string>>("jointsType");
		const vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");
		const vector<vector<double>> jointsAxis = robot->get<vector<vector<double>>>("jointsAxis");
		auto q = robot->get_model("q");
		const bool modified = !dqr.is_empty();

		casadi::Slice sel3(0,3);
		casadi::SXVector R(nj), r(nj);		// rotation and origin of frame i in frame parent(i)
		casadi::SXVector S(nj);				// joint motion subspace, 6 x dim
		vector<casadi::Slice> qi(nj);		// entries of joint i in q
		casadi::SXVector v(nj), vr(nj), ar(nj);	// velocity, reference velocity, reference acceleration of frame i

		// --- Forward pass --- //
		for (int i=0, dof_count=0; i<nj; i++){
			const int p = jointsParent[i];
			if (p >= i) throw std::runtime_error("rnea: parent of joint " + std::to_string(i) + " must come before it");
			const int dim = jointsDimension[i];
			qi[i] = casadi::Slice(dof_count, dof_count + dim);
			dof_count += dim;

			SX T = robot->get_model("T_"+std::to_string(i));
			R[i] = T(sel3,sel3);
			r[i] = T(sel3,3);

			// world frame is still, gravity enters as base acceleration
			v[i] = (p < 0) ? SX::zeros(6,1) : motion_to_child(R[i], r[i], v[p]);
			if (modified) vr[i] = (p < 0) ? SX::zeros(6,1) : motion_to_child(R[i], r[i], vr[p]);
			ar[i] = motion_to_child(R[i], r[i], (p < 0) ? SX::vertcat({-g, SX::zeros(3,1)}) : ar[p]);

			if (dim > 0){
				S[i] = robot->get_model("S_JOINT_"+jointsType[i], {q(qi[i]), SX(casadi::DM(jointsAxis[i]))});
				SX Sdq = mtimes(S[i], dq(qi[i]));
				SX Sdqr = modified ? mtimes(S[i], dqr(qi[i])) : Sdq;
				v[i] += Sdq;
				if (modified) vr[i] += Sdqr;
				// jtimes is the dS/dt dqr term, zero when S is constant (R, P joints)
				ar[i] += mtimes(S[i], ddqr(qi[i])) + SX::jtimes(Sdqr, q, dq) + cross_motion(v[i], Sdqr);
			}
			if (!modified) vr[i] = v[i];
		}

		// --- Backward pass: f[i] is the force across joint i, in frame i --- //
		casadi::SXVector f(nj, SX::zeros(6,1));
		SX tau = SX::zeros(ndof,1);
		for (int i=nj-1; i>=0; i--){
			// all children of frame i have index > i, so f[i] is complete here
			if (jointsDimension[i] > 0) tau(qi[i]) = mtimes(S[i].T(), f[i]);

			const int p = jointsParent[i];
			if (p < 0) continue;		// links on the world frame do not load any joint

			// link i moves with frame p: f = I ar + B(v) vr, B(v) = 1/2 [v x* I + (I v) xbar - I v x], B(v) v = v x* I v
			SX par_i = par(casadi::Slice(nParLink*i, nParLink*(i+1)));
			auto inertia = [&](const SX& u){ return inertia_times(par_i, u); };
			SX f_link = modified ?
				inertia(ar[p]) + 0.5*(cross_force(v[p], inertia(vr[p])) + cross_force(vr[p], inertia(v[p])) - inertia(cross_motion(v[p], vr[p]))) :
				inertia(ar[p]) + cross_force(v[p], inertia(v[p]));
			f[p] += f_link + force_to_parent(R[i], r[i], f[i]);
		}

		return tau;
	}

	std::pair<casadi::SX, casadi::SX> DynBuilder::crba(std::shared_ptr<Robot> robot, const casadi::SX& par, const casadi::SX& g){
		// Composite rigid body algorithm for M, and G from the composite mass and first moment. Same tree convention as rnea.
		// Ic[k]: composite inertia of frame k = links attached to frame k + composite inertias of its child frames.
		// m_c[k], h_c[k]: mass and first moment (in frame k) of the same bodies, all that G needs.
		const int nj = robot->get<int>("numJoints");
		const int ndof = robot->get<int>("ndof");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		const vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		const vector<string> jointsType = robot->get<vector<string>>("jointsType");
		const vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");
		const vector<vector<double>> jointsAxis = robot->get<vector<vector<double>>>("jointsAxis");
		auto q = robot->get_model("q");

		casadi::Slice sel3(0,3);
		casadi::SXVector X(nj), S(nj), Ic(nj, SX::zeros(6,6));
		casadi::SXVector g_frame(nj);		// gravity in frame i
		casadi::SXVector m_c(nj, SX::zeros(1,1)), h_c(nj, SX::zeros(3,1));
		vector<casadi::Slice> qi(nj);
		for (int i=0, dof_count=0; i<nj; i++){
			if (jointsParent[i] >= i) throw std::runtime_error("crba: parent of joint " + std::to_string(i) + " must come before it");
			qi[i] = casadi::Slice(dof_count, dof_count + jointsDimension[i]);
			dof_count += jointsDimension[i];
			SX T = robot->get_model("T_"+std::to_string(i));
			X[i] = motion_transform(T(sel3,sel3), T(sel3,3));
			g_frame[i] = mtimes(T(sel3,sel3).T(), (jointsParent[i] < 0) ? g : g_frame[jointsParent[i]]);
			if (jointsDimension[i] > 0) S[i] = robot->get_model("S_JOINT_"+jointsType[i], {q(qi[i]), SX(casadi::DM(jointsAxis[i]))});
		}

		// --- Composite inertias, mass and first moment, leaves to root --- //
		for (int i=nj-1; i>=0; i--){
			const int p = jointsParent[i];
			if (p < 0) continue;
			SX par_i = par(casadi::Slice(nParLink*i, nParLink*(i+1)));
			Ic[p] += spatial_inertia(par_i) + SX::mtimes({X[i].T(), Ic[i], X[i]});
			SX T = robot->get_model("T_"+std::to_string(i));
			m_c[p] += par_i(0) + m_c[i];
			h_c[p] += par_i(casadi::Slice(1,4)) + mtimes(T(sel3,sel3), h_c[i]) + m_c[i]*T(sel3,3);
		}

		// --- M: force of joint i moved up to each ancestor joint j --- //
		SX M = SX::zeros(ndof, ndof);
		for (int i=0; i<nj; i++){
			if (jointsDimension[i] == 0) continue;
			SX F = mtimes(Ic[i], S[i]);
			M(qi[i], qi[i]) = mtimes(S[i].T(), F);
			for (int j=i; jointsParent[j] >= 0; ){
				F = mtimes(X[j].T(), F);		// force to parent coordinates
				j = jointsParent[j];
				if (jointsDimension[j] == 0) continue;
				SX M_ji = mtimes(S[j].T(), F);
				M(qi[j], qi[i]) = M_ji;
				M(qi[i], qi[j]) = M_ji.T();
			}
		}

		// --- G: weight of the subtree of joint i, Ic [-g; 0] with gravity as base acceleration as in rnea --- //
		SX G = SX::zeros(ndof, 1);
		for (int i=0; i<nj; i++){
			if (jointsDimension[i] == 0) continue;
			SX a = -g_frame[i];
			G(qi[i]) = mtimes(S[i].T(), SX::vertcat({m_c[i]*a, cross(h_c[i], a)}));
		}
		return {M, G};
	}

	casadi::SX DynBuilder::cheapest(const string& name, const casadi::SX& a, const string& a_method, const casadi::SX& b, const string& b_method){
		// fewer CasADi instructions, counted as add_function builds the function (with cse)
		auto instructions = [](const SX& e){ return casadi::Function("f", SX::symvar(e), {e}, casadi::Dict{{"cse", true}}).n_instructions(); };
		const casadi_int n_a = instructions(a), n_b = instructions(b);
		const bool pick_a = (n_a <= n_b);
		debug_log("auto: " + name + " by " + (pick_a ? a_method : b_method) + " (" + a_method + " " + std::to_string(n_a) + ", " + b_method + " " + std::to_string(n_b) + " instructions)", VERB_INFO);
		return pick_a ? a : b;
	}

	int DynBuilder::compute_dyn_rnea(std::shared_ptr<Robot> robot, const string& method){
		const int ndof = robot->get<int>("ndof");
		auto dq = robot->get_model("dq");
		auto ddq = robot->get_model("ddq");
		auto par_gravity = robot->get_model("par_gravity");
		SX par = dyn2reg(robot->get_model("par_DYN"), robot->get<int>("numJoints"), robot->get<const int>("STD_PAR_LINK"));
		SX zeros_n = SX::zeros(ndof,1);
		SX zeros_g = SX::zeros(3,1);

		// method "rnea": tau = M ddq + C dq + G, each term is one RNEA call with the others set to zero.
		// method "crba": M and G from the composite inertias. "auto": M and G from the cheapest of the two.
		// Cdq is always by RNEA (velocity terms have no composite form).
		SX M_rnea, G_rnea, M_crba, G_crba;
		if (method != "crba") {
			M_rnea = SX::jacobian(rnea(robot, par, zeros_n, SX(), ddq, zeros_g), ddq);	// tau is linear in ddq
			G_rnea = rnea(robot, par, zeros_n, SX(), zeros_n, par_gravity);
		}
		if (method != "rnea") std::tie(M_crba, G_crba) = crba(robot, par, par_gravity);

		SX M = (method == "rnea") ? M_rnea : (method == "crba") ? M_crba : cheapest("M", M_rnea, "rnea", M_crba, "crba");
		SX G = (method == "rnea") ? G_rnea : (method == "crba") ? G_crba : cheapest("G", G_rnea, "rnea", G_crba, "crba");
		SX Cdq = rnea(robot, par, dq, SX(), zeros_n, zeros_g);

		return add_dyn(robot, M, Cdq, G);
	}

	int DynBuilder::add_dyn(std::shared_ptr<Robot> robot, const casadi::SX& M, const casadi::SX& Cdq, const casadi::SX& G){
		if (!robot->add_function("M", M, {"q", "par_KIN", "par_DYN"}, "Manipulator mass matrix")) return 0;
		if (!robot->add_function("Cdq", Cdq, {"q", "dq", "par_KIN", "par_DYN"}, "Manipulator Coriolis and centrifugal terms C*dq")) return 0;
		if (!robot->add_function("G", G, {"q", "par_KIN", "par_gravity", "par_DYN"}, "Manipulator gravity terms")) return 0;
		return 1;
	}

	int DynBuilder::compute_C(std::shared_ptr<Robot> robot, const string& C_method){
		const int ndof = robot->get<int>("ndof");
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");

		// "christoffel": from the registered M. "rnea": the modified RNEA is linear in the reference velocity, C = d tau / d dqr.
		// "auto": the cheapest of the two.
		SX C_christoffel, C_rnea;
		if (C_method != "rnea") C_christoffel = stdCmatrix(robot->get_model("M"), q, dq, dq_select(dq));
		if (C_method != "christoffel") {
			SX par = dyn2reg(robot->get_model("par_DYN"), robot->get<int>("numJoints"), robot->get<const int>("STD_PAR_LINK"));
			SX dqr = SX::sym("dqr_C", ndof);
			C_rnea = SX::jacobian(rnea(robot, par, dq, dqr, SX::zeros(ndof,1), SX::zeros(3,1)), dqr);
		}
		SX C = (C_method == "rnea") ? C_rnea : (C_method == "christoffel") ? C_christoffel : cheapest("C", C_rnea, "rnea", C_christoffel, "christoffel");
		return robot->add_function("C", C, {"q", "dq", "par_KIN", "par_DYN"}, "Manipulator Coriolis matrix");
	}

	int DynBuilder::compute_C_std(std::shared_ptr<Robot> robot){
		auto q = robot->get_model("q");
		auto dq = robot->get_model("dq");
		auto M = robot->get_model("M");

		SX C_std = stdCmatrix_classic(M, q, dq, dq_select(dq));
		return robot->add_function("C_std", C_std, {"q", "dq", "par_KIN", "par_DYN"}, "Classic formulation of the manipulator Coriolis matrix");
	}

	int DynBuilder::compute_Dl(std::shared_ptr<Robot> robot){
		// parameters from robot
		int Dl_order = 0;
		if (robot->properties.count("Dl_order")) {
			Dl_order = robot->get<int>("Dl_order");
		} else {
			robot->add_property<int>("Dl_order", Dl_order, "int", "Order of the link friction model", true);
		}

		if (Dl_order > 0){
			const int nj = robot->get<int>("numJoints");
			const int ndof = robot->get<int>("ndof");
			const auto& dq = robot->get_model("dq");
			const auto& par_Dl = robot->get_model("par_Dl");

			casadi::SX dl(ndof,1);
			std::vector<casadi::SX> Dl_vec(Dl_order);
			for (int i=0; i<ndof; i++){
				for (int ord=0; ord<Dl_order; ord++){
					if (ord%2 == 0){
						dl(i) += pow(dq(i), ord+1) * par_Dl(i*Dl_order+ord);
					} else {
						dl(i) += sqrt(pow(dq(i), 2)) * pow(dq(i), ord) * par_Dl(i*Dl_order+ord);
					}
					Dl_vec[ord].resize(ndof,ndof);
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
		std::vector<std::string> arg_list = {"q", "dq", "par_KIN", "par_DYN"};
		robot->add_function("M_dot", dM, arg_list, "Time derivative of the mass matrix");
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_DYN"};
		robot->add_function("M_ddot", ddM, arg_list, "Second time derivative of the mass matrix");

		// - Coriolis derivatives - //
		casadi::SX dC = casadi::SX::jtimes(C,q,dq) + casadi::SX::jtimes(C,dq,ddq);
		casadi::SX ddC = casadi::SX::jtimes(dC,q,dq) + casadi::SX::jtimes(dC,dq,ddq) + casadi::SX::jtimes(dC,ddq,d3q);
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_DYN"};
		robot->add_function("C_dot", dC, arg_list, "Time derivative of the Coriolis matrix");
		arg_list = {"q", "dq", "ddq", "d3q", "par_KIN", "par_DYN"};
		robot->add_function("C_ddot", ddC, arg_list, "Second time derivative of the Coriolis matrix");

		// - Gravity derivatives - //
		casadi::SX dG = casadi::SX::jtimes(G,q,dq);
		casadi::SX ddG = casadi::SX::jtimes(dG,q,dq) + casadi::SX::jtimes(dG,dq,ddq);
		arg_list = {"q", "dq", "par_KIN", "par_gravity", "par_DYN"};
		robot->add_function("G_dot", dG, arg_list, "Time derivative of the gravity vector");
		arg_list = {"q", "dq", "ddq", "par_KIN", "par_gravity", "par_DYN"};
		robot->add_function("G_ddot", ddG, arg_list, "Second time derivative of the gravity vector");

		return 1;
	}

	// --- REG/DYN conversions --- //
	casadi::SX DynBuilder::dyn2reg(const casadi::SX& par_DYN, int numJoints, int STD_PAR_LINK){
		// per link [m, CoM, I about CoM] -> [m, m CoM, I about the frame origin] (parallel axis theorem)
		SX par_REG = SX::zeros(par_DYN.size());
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			SX p_dyn(par_DYN(p_idx));
			SX mass = p_dyn(0);
			SX CoM = p_dyn(casadi::Slice(1,4));
			SX mCoM = mass * p_dyn(casadi::Slice(1,4));
			SX I_tmp = mass * SX::mtimes(hat(CoM).T(), hat(CoM));
			SX I_dyn = p_dyn(casadi::Slice(4,10));
			SX I_tmp_v = SX::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			SX I = I_dyn + I_tmp_v;
			par_REG(p_idx) = SX::vertcat({mass, mCoM, I});
		}
		return par_REG;
	}

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
		SX dyn2reg_expr = dyn2reg(robot->get_model("par_DYN"), numJoints, STD_PAR_LINK);
		robot->add_function("dyn2reg", dyn2reg_expr, {"par_DYN"}, "Conversion from dynamic to regressor parameters");

		// - Update regressor parameters - //
		robot->set("par_REG", robot->get("dyn2reg"));

		return 1;
	}

    void DynBuilder::build(std::shared_ptr<Robot> robot) {
		debug_log("Starting dynamic computations", VERB_INFO);

		// dynamics_method: how M, Cdq, G are built, "rnea" (Newton-Euler), "crba" (M and G from composite inertias, Cdq by RNEA),
		// "auto" (M and G from the cheapest of rnea and crba) or "lagrange" (CoM Jacobians)
		const string dynamics_method = config_["dynamics_method"] ? config_["dynamics_method"].as<string>() : "auto";
		// C_method: how the matrix C is built, "rnea" (modified Newton-Euler), "christoffel" (from M) or "auto" (the cheapest)
		const string C_method = config_["C_method"] ? config_["C_method"].as<string>() : "auto";
		// compute_C_std: also add C_std, the element-wise Christoffel version of C (same matrix, slow to build)
		const bool use_C_std = config_["compute_C_std"] ? config_["compute_C_std"].as<bool>() : false;
		// compute_J_cm: also add the centre of mass Jacobians J_cm_<i>
		const bool use_J_cm = config_["compute_J_cm"] ? config_["compute_J_cm"].as<bool>() : false;

		if (dynamics_method != "rnea" && dynamics_method != "crba" && dynamics_method != "auto" && dynamics_method != "lagrange")
			throw std::runtime_error("dyn_builder: unknown dynamics_method '" + dynamics_method + "' (use 'rnea', 'crba', 'auto' or 'lagrange')");
		if (C_method != "rnea" && C_method != "christoffel" && C_method != "auto")
			throw std::runtime_error("dyn_builder: unknown C_method '" + C_method + "' (use 'rnea', 'christoffel' or 'auto')");

		int ret = 1;
		if (dynamics_method == "lagrange") {
			if (!compute_dyn_lagrange(robot)) ret = 0;
		} else {
			if (!compute_dyn_rnea(robot, dynamics_method)) ret = 0;
		}
		if (!compute_C(robot, C_method)) ret = 0;
		if (use_C_std) {
			if (!compute_C_std(robot)) ret = 0;
		}
		if (use_J_cm) {
			if (!compute_J_cm(robot)) ret = 0;
		}
		if (!compute_Dl(robot)) ret = 0;
		if (!compute_reg_dyn_conversions(robot)) ret = 0;
		if (1){
			if (!compute_dyn_derivatives(robot)) ret = 0;
		}

		// return ret;
		debug_log("Dynamics computed", VERB_INFO);
    }

}