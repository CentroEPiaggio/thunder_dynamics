#include "plugins/builders/base_inertial_param_builder.h"
#include "utils.h"

#include <yaml-cpp/yaml.h>

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns {

	void BaseInertialParamBuilder::init(std::shared_ptr<Robot> robot){
		// --- Check DH Usage --- //
		//if (!config_["DH"]){
		//	throw std::runtime_error("The inertial parameters must be defined using the DH convention.");
		//}
		if(robot->parameters.count("par_DHtable") == 0){
			throw std::runtime_error("The inertial parameters must be defined using the DH convention.");
		}
		// --- Check wheter include motor inertia --- //
		//if (config_["par_Mm"]){
		//	debug_log("Including the motor inrtia in the system dynamics", VERB_INFO);
		//	has_motor_inertia = true;
		//}
		if(robot->parameters.count("par_Ia") > 0){
			has_motor_inertia = true;
			debug_log("Including the motor inertia in the system dynamics", VERB_INFO);
		}else{
			has_motor_inertia = false;
			debug_log("Motor inertia not modeled. Proceeding without it", VERB_INFO);
		}

		// --- define r1, r2, p1, p2 parameters from robot (DH must be numeric) --- //
		int n = robot->get<int>("numJoints");
		vector<string> joints_type = robot->get<vector<string>>("jointsType");

		// a) r1 = first revolute joint
		for (int i=0; i<n; i++){
			if (joints_type[i] == "R"){
				robot->add_property<int>("r1", i, "int", "Index of first revolute joint", true);
				debug_log("Found r1 = " + std::to_string(i), VERB_INFO);
				has_r1 = true;
				break;
			}
		}
		// b) r2 = sequent revolute joint not aligned with r1
		if(has_r1){
			int r1 = robot->get<int>("r1");
			auto T_r1 = robot->get_model("T_"+std::to_string(r1)); // transformation from base to link r1
			auto k_r1 = T_r1(casadi::Slice(0,3), casadi::Slice(2,3));  // z axis of link r1
			for (int i=r1+1; i<n; i++){
				if (joints_type[i] == "R"){
					// check alignement
					auto T_i = robot->get_model("T_"+std::to_string(i+1)); // transformation from base to link i
					auto k_i = T_i(casadi::Slice(0,3), casadi::Slice(2,3));  // z axis of link i
					if (!(1-casadi::SX::dot(k_r1, k_i)).is_zero()){
						robot->add_property<int>("r2", i, "int", "Index of sequent revolute joint not aligned with r1", true);
						debug_log("Found r2 = " + std::to_string(i), VERB_INFO);
						has_r2 = true;
						break;
					}
				}
			}
		}
		// c) p1 = first prismatic joint
		for (int i=0; i<n; i++){
			if (joints_type[i] == "P"){
				robot->add_property<int>("p1", i, "int", "Index of first prismatic joint", true);
				debug_log("Found p1 = " + std::to_string(i), VERB_INFO);
				has_p1 = true;
				break;
			}
		}
		// d) rp1 = Last revolute joint before the first prismatic joint
		if(has_p1 && has_r1){
			int p1 = robot->get<int>("p1");
			for (int i=p1-1; i>=0; i--){
				if (joints_type[i] == "R"){
					robot->add_property<int>("rp1", i, "int", "Index of Last revolute before first prismatic joint", true);
					debug_log("Found rp1 = " + std::to_string(i), VERB_INFO);
					has_rp1 = true;
					break;
				}
			}
		}
	}


	casadi::SX BaseInertialParamBuilder::createS(std::shared_ptr<Robot> robot, int link){
		// Extract link parameters
		int numJoints = robot->get<int>("numJoints");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		SX S;

		if (has_motor_inertia){ // Regroup also motor inertial parameters		
			S = SX::zeros(11,numJoints*11);
			casadi::Slice rows(0,10);
			casadi::Slice cols(link*10, (link+1)*10);
			S(rows,cols) = SX::eye(10);
			S(10, numJoints*10 + link) = 1; // selector of motor inertia
			
		}else{// regroup only standard inertial parameters
			S = SX::zeros(10,numJoints*10);
			casadi::Slice rows(0,10);
			casadi::Slice cols(link*10, (link+1)*10);
			S(rows,cols) = SX::eye(10);
		}
		// Ia in Reg
		// S = SX::zeros(nParLink,numJoints*nParLink);
		// casadi::Slice rows(0,nParLink);
		// casadi::Slice cols(link*nParLink, (link+1)*nParLink);
		// S(rows,cols) = SX::eye(nParLink);

		return S;
	}
	casadi::SX BaseInertialParamBuilder::createLambda(const SX& a, const SX& alpha, const SX& d, const SX& theta){
		casadi::SX sa = sin(alpha);
		casadi::SX ca = cos(alpha);
		casadi::SX st = sin(theta);
		casadi::SX ct = cos(theta);

		casadi::SX ssa = sa * sa;
		casadi::SX cca = ca * ca;
		casadi::SX sst = st * st;
		casadi::SX cct = ct * ct;

		casadi::SX csa = ca * sa;
		casadi::SX cst = ct * st;

		casadi::SX L;
		if (has_motor_inertia){
			L = casadi::SX::zeros(11,11);
		}else{
			L = casadi::SX::zeros(10,10);
		}
		// Mass (Column 0)
		L(0, 0) = 1;
		L(1, 0) = a;
		L(2, 0) = -d * sa;
		L(3, 0) = d * ca;
		L(4, 0) = pow(d, 2);
		L(5, 0) = a * d * sa;
		L(6, 0) = -a * d * ca;
		L(7, 0) = pow(a, 2) + pow(d, 2) * cca;
		L(8, 0) = pow(d, 2) * csa;
		L(9, 0) = pow(a, 2) + pow(d, 2) * ssa;

		// m*COM_x (Column 1)
		L(1, 1) = ct;
		L(2, 1) = st * ca;
		L(3, 1) = st * sa;
		L(5, 1) = -a * st * ca + d * ct * sa;
		L(6, 1) = -a * st * sa - d * ct * ca;
		L(7, 1) = 2 * (a * ct + d * st * csa);
		L(8, 1) = d * st * (ssa - cca);
		L(9, 1) = 2 * (a * ct - d * st * csa);

		// m*COM_y (Column 2)
		L(1, 2) = -st;
		L(2, 2) = ct * ca;
		L(3, 2) = ct * sa;
		L(5, 2) = -a * ct * ca + d * st * sa;
		L(6, 2) = -a * ct * sa - d * ct * ca;
		L(7, 2) = 2 * (-a * st + d * ct * csa);
		L(8, 2) = d * ct * (ssa - cca);
		L(9, 2) = 2 * (-a * st - d * ct * csa);

		// m*COM_z (Column 3)
		L(2, 3) = -sa;
		L(3, 3) = ca;
		L(4, 3) = 2 * d;
		L(5, 3) = a * sa;
		L(6, 3) = -a * ca;
		L(7, 3) = 2 * d * cca;
		L(8, 3) = 2 * d * csa;
		L(9, 3) = 2 * d * ssa;

		// Inertia components (Columns 4-9)
		L(4, 4) = cct;   L(5, 4) = cst * ca;       L(6, 4) = cst * sa;       L(7, 4) = sst * cca; L(8, 4) = sst * csa; L(9, 4) = sst * ssa;
		L(4, 5) = -2*cst; L(5, 5) = (cct-sst)*ca; L(6, 5) = (cct-sst)*sa; L(7, 5) = 2*cst*cca; L(8, 5) = 2*cst*csa; L(9, 5) = 2*cst*ssa;
		L(5, 6) = -ct*sa; L(6, 6) = -ct*sa;        L(7, 6) = -2*st*csa;      L(8, 6) = st*(cca-ssa); L(9, 6) = 2*st*csa;
		L(4, 7) = sst;   L(5, 7) = -cst*ca;      L(6, 7) = -cst*sa;      L(7, 7) = cct*cca;   L(8, 7) = cct*csa;   L(9, 7) = cct*ssa;
		L(5, 8) = st*sa;  L(6, 8) = -st*sa;       L(7, 8) = -2*ct*csa;      L(8, 8) = ct*(cca-ssa); L(9, 8) = 2*ct*csa;
		L(7, 9) = sst;   L(8, 9) = -csa;         L(9, 9) = cca;

		return L;
	}
	casadi::SX BaseInertialParamBuilder::del_col(const SX& M, vector<int> elim_idx){
		// 1. Generate the Keep list
		std::vector<int> keep_idx;
		for (int i = 0; i < M.size2(); ++i) {
			// Check if i is NOT in the elimination list
			if (std::find(elim_idx.begin(), elim_idx.end(), i) == elim_idx.end()) {
				keep_idx.push_back(i);
			}
		}
		SX M_red = M(casadi::Slice(), keep_idx);
		return M_red;
	}
	casadi::SX BaseInertialParamBuilder::del_row(const SX& M, vector<int> elim_idx){
		// 1. Generate the Keep list
		std::vector<int> keep_idx;
		for (int i = 0; i < M.size1(); ++i) {
			// Check if i is NOT in the elimination list
			if (std::find(elim_idx.begin(), elim_idx.end(), i) == elim_idx.end()) {
				keep_idx.push_back(i);
			}
		}
		SX M_red = M(keep_idx, casadi::Slice());
		return M_red;
	}


	int BaseInertialParamBuilder::compute_Ia(std::shared_ptr<Robot> robot){
		// parameters from robot
		int nj = robot->get<int>("numJoints");
		const int nParLink = robot->get<const int>("STD_PAR_LINK");
		auto ddq = robot->get_model("ddq");
		
		auto par_Ia = robot->get_model("par_Ia");

		// - symbolic par construction - //
		casadi::SX Ia(nj,1);
		casadi::SX Ia_vec(nj,1);

		// parse par_Ia
		for (int i=0; i<nj; i++){
			Ia(i) += ddq(i) * par_Ia(i);
			Ia_vec(i) = par_Ia(i);
		}
		std::vector<std::string> arg_list;
		arg_list = {"ddq", "par_Ia"};
		robot->add_function("Ia", Ia, arg_list, "Manipulator motor inertia torque");
		
		casadi::SX reg_Ia = casadi::SX::jacobian(Ia, par_Ia);
		if (!robot->add_function("reg_Ia", reg_Ia, {"ddq"}, "Regressor matrix of the motor inertia")) return 0;

		return 1;
	}

	int BaseInertialParamBuilder::compute_par_red(std::shared_ptr<Robot> robot){
		// --- load parameters from robot --- //
		int numJoints = robot->get<int>("numJoints");
		vector<string> joints_type = robot->get<vector<string>>("jointsType");

		auto DHtable = reshape(robot->get_model("par_DHtable"),4,-1).T(); // DH table (a - alplha - d - q)
		auto gravity = robot->get_model("par_gravity"); //shoul
		//debug_log(DHtable.dim(), VERB_INFO);


		int r1 = has_r1 ? robot->get<int>("r1") : -1;
		int r2 = has_r2 ? robot->get<int>("r2") : -1;
		int p1 = has_p1 ? robot->get<int>("p1") : -1;
		int rp1 = has_rp1 ? robot->get<int>("rp1") : -1;

		// Full parameters vector 
		SX par_full;
		if (!has_motor_inertia){
			par_full = robot->get_model("par_REG");
		}else{
			par_full = SX::vertcat({robot->get_model("par_REG"), robot->get_model("par_Ia")});
		}
		par_full = reshape(par_full, -1, 1);
		int par_per_link = par_full.size1()/numJoints;
		
		casadi::SXVector E(numJoints);
		casadi::SXVector H(numJoints);
		casadi::SXVector W(numJoints);

		casadi::SX beta;
		vector<int> reg_linear_dependent_column;

		for (int i = numJoints-1; i >= 0; i--){
			// 1) Extract i-th link parameters
			//  -------------------------------------------------
			SX S_i = createS(robot, i);
			SX par_i = casadi::SX::mtimes({S_i, par_full});

			SX E_full = SX::eye(par_per_link);
			vector<int> elim_idx;

			// 2) Compute parameters that can be eliminated
			//  -------------------------------------------------

			//  ----------- REVOLUTE ----------------------------
			if (joints_type[i] == "R"){
				// 1.a) Reducing linear dependent parameters
				E_full(4,7) = -1; // I_xx' = I_xx - I_yy
				elim_idx.push_back(0);elim_idx.push_back(3);elim_idx.push_back(7); // DELETE: mass, m*COM_z, I_yy
				reg_linear_dependent_column.push_back(10*i+0);
				reg_linear_dependent_column.push_back(10*i+3);
				reg_linear_dependent_column.push_back(10*i+7);
				debug_log("Reducing M, MZ, YY of link " + std::to_string(i), VERB_INFO);


				if (has_motor_inertia){
					if (has_r2 && i == r2){					
						// auto T_r1 = robot->get_model("T_"+std::to_string(r1));
						// auto k_r1 = T_r1(casadi::Slice(0,3), casadi::Slice(2,3)); 
						// auto T_r2 = robot->get_model("T_"+std::to_string(r2)); 
						// auto k_r2 = T_r2(casadi::Slice(0,3), casadi::Slice(2,3));

						SX T_r1_r2 = SX::eye(4); 
						for (int j=r1+1; j<=r2; j++){
							auto T_r1_j = robot->get_model("T_"+std::to_string(j+1)); 
							T_r1_r2 = SX::mtimes({T_r1_r2, T_r1_j});
						}

						// if (dot(k_r1, k_r2).is_zero()){
						if (T_r1_r2(2,2).is_zero()){
							// r2 joint axis orthogonal to r1 joint axis
							E_full(9,10) = 1; 		// I_zz' = I_zz + I_r
							elim_idx.push_back(10); // DELETE: motor inertia
							reg_linear_dependent_column.push_back(10*numJoints + i);
							debug_log("Reducing Ir of link " + std::to_string(i), VERB_INFO);
						}
					}else if(i == r1){
						E_full(9,10) = 1; 		// I_zz' = I_zz + I_r
						elim_idx.push_back(10); // DELETE: motor inertia
						reg_linear_dependent_column.push_back(10*numJoints + i);
						debug_log("Reducing Ir of link " + std::to_string(i), VERB_INFO);
					}
				}


				// 2.a) Reducing parameters with no effect to the dynamic
				if((i>=r1 && !has_r2) || (i>=r1 && i<r2)){
					elim_idx.push_back(4); elim_idx.push_back(5); 
					elim_idx.push_back(6); elim_idx.push_back(8); // DELETE: I_xx, I_xy, I_xz, I_yz
					reg_linear_dependent_column.push_back(10*i+4);
					reg_linear_dependent_column.push_back(10*i+5);
					reg_linear_dependent_column.push_back(10*i+6);
					reg_linear_dependent_column.push_back(10*i+8);
					debug_log("Reducing XX, XY, XZ, YZ of link " + std::to_string(i), VERB_INFO);

				
					auto T_r1 = robot->get_model("T_w_"+std::to_string(r1));
					auto k_r1 = T_r1(casadi::Slice(0,3), casadi::Slice(2,3)); 
					auto u_g = robot->get_model("par_gravity")/norm_1(robot->get_model("par_gravity"));
					
					if ((1-abs(dot(k_r1, u_g))).is_zero()){
						// joint axis aligned with first revolute joint axis and gravity direction
						if (i==r1 && (1-abs(dot(k_r1, u_g))).is_zero()){
							// first rotational joint aligned with gravity direction
							elim_idx.push_back(1); // DELETE: m*COM_x
							elim_idx.push_back(2); // DELETE: m*COM_y
							reg_linear_dependent_column.push_back(10*i+1);
							reg_linear_dependent_column.push_back(10*i+2);
							debug_log("Reducing MX, MY of link " + std::to_string(i), VERB_INFO);
						}
					}
				}			

			}
			//  ----------- PRISMATIC ----------------------------
			else if (joints_type[i] == "P"){
				// 1.a) Reducing linear dependent parameters
				elim_idx.push_back(4);elim_idx.push_back(5);elim_idx.push_back(6); // DELETE: I_xx, I_xy, I_xz, I_yy, I_yz, I_zz
				elim_idx.push_back(7);elim_idx.push_back(8);elim_idx.push_back(9);

				reg_linear_dependent_column.push_back(10*i+4);
				reg_linear_dependent_column.push_back(10*i+5);
				reg_linear_dependent_column.push_back(10*i+6);
				reg_linear_dependent_column.push_back(10*i+7);
				reg_linear_dependent_column.push_back(10*i+8);
				reg_linear_dependent_column.push_back(10*i+9);
				debug_log("Reducing XX, XY, XZ, YY, YZ, ZZ of link " + std::to_string(i), VERB_INFO);
			
				// 2.a) Reducing parameters with no effect to the dynamic
				if( ((has_r1 && has_r2) && (i>r1 && i<r2)) ||
					((has_r1 && !has_r2) && (i>r1))){
					// auto T_r1 = robot->get_model("T_"+std::to_string(r1));
					// auto k_r1 = T_r1(casadi::Slice(0,3), casadi::Slice(2,3)); 
					// auto T_i = robot->get_model("T_"+std::to_string(i)); 
					// auto k_i = T_i(casadi::Slice(0,3), casadi::Slice(2,3));
					SX T_r1_i = SX::eye(4); 
					for (int j=r1+1; j<=i; j++){
						auto T_r1_j = robot->get_model("T_"+std::to_string(j+1)); 
						T_r1_i = SX::mtimes({T_r1_i, T_r1_j});
					}
					auto dot_k_r1_k_i = T_r1_i(2,2);

					//if((1-abs(dot(k_r1, k_i))).is_zero()){
					if((1-abs(dot_k_r1_k_i)).is_zero()){
						// if z_i parallel to z_ri axis
						auto d = DHtable(i,2); auto ct = cos(DHtable(i,3)); auto st = sin(DHtable(i,2));
						E_full(9,1) = 2*d*ct;			// regroup ZZ' = ZZ + 2dct MX - 2dst MY  
						E_full(9,2) = -2*d*st;
						elim_idx.push_back(1); 			// DELETE: m*CoM_x
						elim_idx.push_back(2); 			// DELETE: m*CoM_y
						elim_idx.push_back(3); 			// DELETE: m*CoM_z
						reg_linear_dependent_column.push_back(10*i + 1);
						reg_linear_dependent_column.push_back(10*i + 2);
						reg_linear_dependent_column.push_back(10*i + 3);
						debug_log("Reducing MX, MY, MZ of link " + std::to_string(i), VERB_INFO);
					}else{
						// if z_i NOT parallel to z_ri axis
						auto i_k_r1 = T_r1_i(casadi::Slice(0,3), casadi::Slice(2,3)); 
						if(!i_k_r1(2).is_zero()){
							E_full(1,3) = -i_k_r1(0)/i_k_r1(2);			// regroup MX' = MX + a*MZ  
							E_full(2,3) = -i_k_r1(1)/i_k_r1(2);			// regroup MY' = MY + b*MZ
							elim_idx.push_back(3); 						// DELETE: m*CoM_z
							reg_linear_dependent_column.push_back(10*i + 3);
							debug_log("Reducing MZ of link " + std::to_string(i), VERB_INFO);
						}else if(!i_k_r1(0).is_zero() && !i_k_r1(1).is_zero()){
							E_full(1,2) = -i_k_r1(0)/i_k_r1(1);			// regroup MX' = MX + c*MX
							elim_idx.push_back(2); 						// DELETE: m*CoM_y
							reg_linear_dependent_column.push_back(10*i + 2);
							debug_log("Reducing MY of link " + std::to_string(i), VERB_INFO);
						}else if(!i_k_r1(0).is_zero()){
							elim_idx.push_back(2); 						// DELETE: m*CoM_y
							reg_linear_dependent_column.push_back(10*i + 2);
							debug_log("Reducing MY of link " + std::to_string(i), VERB_INFO);
						}else{
							elim_idx.push_back(1); 						// DELETE: m*CoM_x
							reg_linear_dependent_column.push_back(10*i + 1);
							debug_log("Reducing MX of link " + std::to_string(i), VERB_INFO);
						}
					}
				}else if(has_r1 && i<r1){
					elim_idx.push_back(1); elim_idx.push_back(2); elim_idx.push_back(3); // DELETE: m*CoM_x, m*CoM_y, m*CoM_z
					reg_linear_dependent_column.push_back(10*i+1);
					reg_linear_dependent_column.push_back(10*i+2);
					reg_linear_dependent_column.push_back(10*i+3);
					debug_log("Reducing MX, MY, MZ of link " + std::to_string(i), VERB_INFO);
				}

				if(has_motor_inertia){
					if(has_rp1){
						auto T_p1 = robot->get_model("T_"+std::to_string(p1));
						auto k_p1 = T_p1(casadi::Slice(0,3), casadi::Slice(2,3)); 
						auto T_rp1 = robot->get_model("T_"+std::to_string(rp1)); 
						auto k_rp1 = T_rp1(casadi::Slice(0,3), casadi::Slice(2,3));
						auto u_g = robot->get_model("par_gravity")/norm_1(robot->get_model("par_gravity"));
						if(dot(k_p1, u_g).is_zero()){
							// z-axis of p1 orthogonal to gravity, and if p1=1 or z-axis aligned to z-axes preceedinng 
							E_full(0,10) = 1;			// regroup M' = M + Ir  
							elim_idx.push_back(10); 						// DELETE: Ir
							reg_linear_dependent_column.push_back(10*numJoints + i);
							debug_log("Reducing Ir of link " + std::to_string(i), VERB_INFO);
						}
					}

				}
			}
			else if(joints_type[i]=="R_SEA"){
				throw std::runtime_error("Soft joint will not be reduced.");
			}else{
				throw std::runtime_error("Unkwnown joint type.");
			}

			// 3) Remapping parameters that can be eliminated on previous link
			//  ----------------------------------------------------------------
			SX H_iminus;
			SX iminus_lambda_i = createLambda(DHtable(i,0), DHtable(i,1), DHtable(i,2), DHtable(i,3));
			casadi::Slice all_raws = casadi::Slice();
			if(joints_type[i] == "R"){
				//  ----------- REVOLUTE ----------------------------
				int motor_padding = has_motor_inertia ? 3 : 2;
				H_iminus = SX::horzcat({iminus_lambda_i(all_raws, 0), 
										SX::zeros(par_per_link, 2),  
										iminus_lambda_i(all_raws, 3),
										SX::zeros(par_per_link, 3), 
										iminus_lambda_i(all_raws, 4) + iminus_lambda_i(all_raws, 7), 
										SX::zeros(par_per_link, motor_padding), 
										SX::eye(par_per_link)
										});
			}else if(joints_type[i] == "P"){
				//  ----------- PRISMATIC ----------------------------
				int motor_padding = has_motor_inertia ? 1 : 0;
				H_iminus = SX::horzcat({SX::zeros(par_per_link, 4),
										iminus_lambda_i(all_raws, casadi::Slice(4,10)),
										SX::zeros(par_per_link, motor_padding), 
										SX::eye(par_per_link)
										});
			}
			if(i == numJoints-1){
				// Initialize backpropagation
				W[i] = S_i;
				H[i] = SX::eye(par_per_link);
			}else{
				// backpropagation of reduceable parameters to previous link
				W[i] = SX::vertcat({casadi::SX::mtimes({H[i+1],W[i+1]}),
									S_i});
			}
			E[i] = del_row(E_full,elim_idx);

			if(i>0){H[i-1] = H_iminus;}
		}


		// 4) Computing linear relationship with reduced set of parameters
		//  ----------------------------------------------------------------
		casadi::SXVector beta_blocks;
		for (int i = 0; i < numJoints; ++i) {
			SX product = casadi::SX::mtimes({E[i], H[i], W[i]});
			beta_blocks.push_back(product);
		}
		beta = SX::vertcat(beta_blocks);
		debug_log("Reduced from " + std::to_string(beta.size2()) + " to " + std::to_string(beta.size1()) + " parameters", VERB_INFO);

		// 5) Adding functions 
		//  ----------------------------------------------------------------
		// a.1) beta s.t. par_DYN_red = beta*par_DYN
		std::vector<std::string> arg_list = {};
		if (!robot->add_function("beta", beta, arg_list, "linear relationship between full dyn parameters and the reduced set. beta s.t. par_red = beta*par.")) return 0;
		
		// a.2) par_DYN_red = beta*par_DYN
		// - Add conversion function - //
		casadi::SX par_REG_red = SX::mtimes({beta,par_full});
		if (has_motor_inertia){arg_list = {"par_REG", "par_Ia"};
		}else{arg_list = {"par_REG"};}
		if (!robot->add_function("reg2red", par_REG_red, arg_list, "Conversion from regressor to base inertial parameters.")) return 0;
		
		// - Add symbolic parameters - //
		SX par_REG_red_symb = SX::sym("par_REG_red", par_REG_red.size1());
		vector<short> par_REG_red_isSymb; par_REG_red_isSymb.assign(par_REG_red.size1(),1);
		vector<double> par_REG_red_num; par_REG_red_num.assign(par_REG_red.size1(),0.0);
		robot->add_parameter("par_REG_red", par_REG_red_symb, par_REG_red_num, par_REG_red_isSymb, "Base inertial parameters for reduced regressor.", true);
		robot->set("par_REG_red", robot->get("reg2red"));

		// b.1) beta_pinv s.t. Yr_red = Y_r*beta_pinv
		//SX beta_pinv = mtimes(inv(mtimes(beta.T(), beta)), beta.T());
		// b.2) Yr_red
		casadi::SX Yr_red;
		casadi::SX Yr;
		if(has_motor_inertia){
			const auto& ddqr = robot->get_model("ddqr");
			casadi::SX YIr = SX::zeros(numJoints,numJoints);
			for (int i=0; i<numJoints; i++){
				YIr(i,i) = ddqr(i);
			}
			Yr = SX::horzcat({robot->get_model("Yr"),YIr});
		}else{
			Yr = robot->get_model("Yr");
		}
		// Yr_red = del_col(Yr, reg_linear_dependent_column);
		auto beta_pinv = SX::pinv(beta); 
		Yr_red = SX::mtimes({Yr,beta_pinv}); 
		arg_list = {"q", "dq", "dqr", "ddqr", "par_KIN", "par_world2L0", "par_gravity"};
		if (!robot->add_function("Yr_red", Yr_red, arg_list, "Regressor defined w.r.t the set of base iniertial parameters.")) return 0;

		// b.3) reg_M_red
		casadi::SX reg_M_red;
		reg_M_red = del_col(robot->get_model("reg_M"), reg_linear_dependent_column);
		arg_list = {"q", "ddqr", "par_KIN", "par_world2L0"};
		if (!robot->add_function("reg_M_red", reg_M_red, arg_list, "Regressor of masses defined w.r.t the set of base iniertial parameters.")) return 0;
		// b.4) reg_C_red
		casadi::SX reg_C_red;
		reg_C_red = del_col(robot->get_model("reg_C"), reg_linear_dependent_column);
		arg_list = {"q", "dq", "dqr", "par_KIN", "par_world2L0"};
		if (!robot->add_function("reg_C_red", reg_C_red, arg_list, "Regressor of centripetal defined w.r.t the set of base iniertial parameters.")) return 0;
		// b.5) reg_G_red
		casadi::SX reg_G_red;
		reg_G_red = del_col(robot->get_model("reg_G"), reg_linear_dependent_column);
		arg_list = {"q", "par_KIN", "par_world2L0", "par_gravity"};
		if (!robot->add_function("reg_G_red", reg_G_red, arg_list, "Regressor of gravity defined w.r.t the set of base iniertial parameters.")) return 0;



		// b.3) print Regrouping info
		// std::string description = "";
		// for (int i=0; i < par_REG_red.size1();i++){
		// 	std::stringstream ss;
		// 	bool first_term = true;
		// 	for (int j=0; j < par_full.size1();i++){
		// 		if(!beta(i,j).is_zero()){
		// 			if (!first_term) {
		// 				ss << " + ";
		// 			}
		// 			ss << beta(i,j) << "*" << par_full(i) << " ";
		// 		}
		// 	}
		// 	description += ss.str() + "\n";
		// }


		return 1;
	}

	
	void BaseInertialParamBuilder::build(std::shared_ptr<Robot> robot) {
		debug_log("Starting computation of the Base Inertial Parameters", VERB_INFO);

		init(robot);
		if (has_motor_inertia){
			compute_Ia(robot);
		}
		compute_par_red(robot);

		// return ret;
		debug_log("Base Inertial Parameters computed", VERB_INFO);
	}

}

