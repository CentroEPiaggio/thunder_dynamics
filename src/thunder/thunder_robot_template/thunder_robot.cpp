#include "thunder_robot.h"
#include "robot_gen.h"


/*OPTIONAL PYBIND11 INCLUDE POINT*/


thunder_robot::thunder_robot(){
	resizeVariables();
}

void thunder_robot::resizeVariables(){
	q = Eigen::VectorXd::Zero(n_joints);
	dq = Eigen::VectorXd::Zero(n_joints);
	ddq = Eigen::VectorXd::Zero(n_joints);
	d3q = Eigen::VectorXd::Zero(n_joints);
	d4q = Eigen::VectorXd::Zero(n_joints);
	dqr = Eigen::VectorXd::Zero(n_joints);
	ddqr = Eigen::VectorXd::Zero(n_joints);
	x = Eigen::VectorXd::Zero(numElasticJoints);
	dx = Eigen::VectorXd::Zero(numElasticJoints);
	ddx = Eigen::VectorXd::Zero(numElasticJoints);
	ddxr = Eigen::VectorXd::Zero(numElasticJoints);
	w = Eigen::VectorXd::Zero(6);
	par_REG = Eigen::VectorXd::Zero(STD_PAR_LINK*n_joints);
	par_DYN = Eigen::VectorXd::Zero(STD_PAR_LINK*n_joints);
	par_Dl = Eigen::VectorXd::Zero(Dl_order*n_joints);
	par_K = Eigen::VectorXd::Zero(K_order*numElasticJoints);
	par_D = Eigen::VectorXd::Zero(D_order*numElasticJoints);
	par_Dm = Eigen::VectorXd::Zero(Dm_order*numElasticJoints);
	par_Mm = Eigen::VectorXd::Zero(numElasticJoints);
	par_DHtable = Eigen::VectorXd::Zero(n_joints*4);
	par_world2L0 = Eigen::VectorXd::Zero(6);
	par_Ln2EE = Eigen::VectorXd::Zero(6);
	par_gravity = Eigen::VectorXd::Zero(3);
	DHtable_symb.resize(n_joints*4);
	for (int i=0; i<n_joints*4; i++) DHtable_symb[i] = 0;
	gravity_symb.resize(3);
	for (int i=0; i<3; i++) gravity_symb[i] = 0;
	world2L0_symb.resize(6);
	Ln2EE_symb.resize(6);
	for (int i=0; i<6; i++){
		world2L0_symb[i] = 0;
		Ln2EE_symb[i] = 0;
	}
}

int thunder_robot::get_numJoints() {return n_joints;};
// int thunder_robot::get_numParLink() {return n_joints;};
int thunder_robot::get_numParDYN() {return STD_PAR_LINK*n_joints;};
int thunder_robot::get_numParREG() {return STD_PAR_LINK*n_joints;};
// int thunder_robot::get_numParELA() {return numParELA;};

void thunder_robot::setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_){
	if(q_.size() == n_joints && dq_.size()== n_joints && dqr_.size()==n_joints && ddqr_.size()==n_joints){
		q = q_;
		dq = dq_;
		dqr = dqr_;
		ddqr = ddqr_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_q(const Eigen::VectorXd& q_){
	if(q_.size() == n_joints){
		q = q_;
	} else{
		std::cout<<"in set_q: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_dq(const Eigen::VectorXd& dq_){
	if(dq_.size() == n_joints){
		dq = dq_;
	} else{
		std::cout<<"in set_dq: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_ddq(const Eigen::VectorXd& ddq_){
	if(ddq_.size() == n_joints){
		ddq = ddq_;
	} else{
		std::cout<<"in set_ddq: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_d3q(const Eigen::VectorXd& d3q_){
	if(d3q_.size() == n_joints){
		d3q = d3q_;
	} else{
		std::cout<<"in set_d3q: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_d4q(const Eigen::VectorXd& d4q_){
	if(d4q_.size() == n_joints){
		d4q = d4q_;
	} else{
		std::cout<<"in set_d4q: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_dqr(const Eigen::VectorXd& dqr_){
	if(dqr_.size() == n_joints){
		dqr = dqr_;
	} else{
		std::cout<<"in set_dqr: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_ddqr(const Eigen::VectorXd& ddqr_){
	if(ddqr_.size() == n_joints){
		ddqr = ddqr_;
	} else{
		std::cout<<"in set_ddqr: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_x(const Eigen::VectorXd& x_){
	if(x_.size() == numElasticJoints){
		x = x_;
	} else{
		std::cout<<"in set_x: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_dx(const Eigen::VectorXd& dx_){
	if(dx_.size() == numElasticJoints){
		dx = dx_;
	} else{
		std::cout<<"in set_dx: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_ddx(const Eigen::VectorXd& ddx_){
	if(ddx_.size() == numElasticJoints){
		ddx = ddx_;
	} else{
		std::cout<<"in set_dx: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_ddxr(const Eigen::VectorXd& ddxr_){
	if(ddxr_.size() == numElasticJoints){
		ddxr = ddxr_;
	} else{
		std::cout<<"in set_ddxr: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_w(const Eigen::VectorXd& w_){
	if(w_.size() == 6){
		w = w_;
	} else{
		std::cout<<"in set_w: invalid dimensions of arguments\n";
	}
}

void thunder_robot::update_inertial_DYN(){
	for (int i=0; i<n_joints; i++){
		Eigen::VectorXd p_reg = par_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK);
		double mass = p_reg(0);
		Eigen::Vector3d CoM = {p_reg(1)/mass, p_reg(2)/mass, p_reg(3)/mass};
		Eigen::Matrix3d I_tmp = mass * hat(CoM) * hat(CoM).transpose();
		Eigen::Matrix<double, 6, 1> I_tmp_v;
		I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
		Eigen::Matrix<double, 6, 1> I;
		I << p_reg(4), p_reg(5), p_reg(6), p_reg(7), p_reg(8), p_reg(9);
		par_DYN.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, CoM, I-I_tmp_v;
	}
}

void thunder_robot::update_inertial_REG(){
	for (int i=0; i<n_joints; i++){
		Eigen::VectorXd p_dyn = par_DYN.segment(STD_PAR_LINK*i, STD_PAR_LINK);
		double mass = p_dyn(0);
		Eigen::Vector3d CoM = {p_dyn(1), p_dyn(2), p_dyn(3)};
		Eigen::Vector3d m_CoM = mass * CoM;
		Eigen::Matrix3d I_tmp = mass * hat(CoM) * hat(CoM).transpose();
		Eigen::Matrix<double, 6, 1> I_tmp_v;
		I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
		Eigen::Matrix<double, 6, 1> I;
		I << p_dyn(4), p_dyn(5), p_dyn(6), p_dyn(7), p_dyn(8), p_dyn(9);
		par_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, m_CoM, I+I_tmp_v;
	}
}

void thunder_robot::set_par_DYN(const Eigen::VectorXd& par_, bool update_REG){
	if(par_.size() == par_DYN.size()){
		par_DYN = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
	// conversion from REG to DYN
	if (update_REG)	update_inertial_REG();
}

void thunder_robot::set_par_REG(const Eigen::VectorXd& par_, bool update_DYN){
	if(par_.size() == par_REG.size()){
		par_REG = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
	// conversion from REG to DYN
	if (update_DYN)	update_inertial_DYN();
}

void thunder_robot::set_par_K(const Eigen::VectorXd& par_){
	if(par_.size() == par_K.size()){
		par_K = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_D(const Eigen::VectorXd& par_){
	if(par_.size() == par_D.size()){
		par_D = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_Dm(const Eigen::VectorXd& par_){
	if(par_.size() == par_Dm.size()){
		par_Dm = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_Mm(const Eigen::VectorXd& par_){
	if(par_.size() == par_Mm.size()){
		par_Mm = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_Dl(const Eigen::VectorXd& par_){
	if(par_.size() == par_Dl.size()){
		par_Dl = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_DHtable(const Eigen::MatrixXd& par_){
	if(par_.size() == par_DHtable.size()){
		par_DHtable = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_gravity(const Eigen::VectorXd& par_){
	if(par_.size() == par_gravity.size()){
		par_gravity = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_world2L0(const Eigen::VectorXd& par_){
	if(par_.size() == par_world2L0.size()){
		par_world2L0 = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_Ln2EE(const Eigen::VectorXd& par_){
	if(par_.size() == par_Ln2EE.size()){
		par_Ln2EE = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

Eigen::VectorXd thunder_robot::get_par_DYN(){
	return par_DYN;
}

Eigen::VectorXd thunder_robot::get_par_REG(){
	return par_REG;
}

Eigen::VectorXd thunder_robot::get_par_K(){
	return par_K;
}

Eigen::VectorXd thunder_robot::get_par_D(){
	return par_D;
}

Eigen::VectorXd thunder_robot::get_par_Dm(){
	return par_Dm;
}

Eigen::VectorXd thunder_robot::get_par_Mm(){
	return par_Mm;
}

Eigen::VectorXd thunder_robot::get_par_Dl(){
	return par_Dl;
}

Eigen::MatrixXd thunder_robot::get_par_DHtable(){
	return par_DHtable;
}

Eigen::VectorXd thunder_robot::get_par_gravity(){
	return par_gravity;
}

Eigen::VectorXd thunder_robot::get_par_world2L0(){
	return par_world2L0;
}

Eigen::VectorXd thunder_robot::get_par_Ln2EE(){
	return par_Ln2EE;
}

Eigen::VectorXd thunder_robot::load_par_REG(std::string file_path, bool update_DYN){
	try {
		YAML::Node config = YAML::LoadFile(file_path);
		int i;
		// --- Parse dynamics REG --- //
		if (config["dynamics"]){
			YAML::Node dynamics = config["dynamics"];
			double mass, m_cmx, m_cmy, m_cmz, xx, xy, xz, yy, yz, zz;
			i = 0;
			for (const auto& node : dynamics) {
				if (i==n_joints) break;
				YAML::Node inertial = node.second["inertial"];
				std::string linkName = node.first.as<std::string>();
				mass = inertial["mass"].as<double>();
				m_cmx = inertial["m_CoM_x"].as<double>();
				m_cmy = inertial["m_CoM_y"].as<double>();
				m_cmz = inertial["m_CoM_z"].as<double>();
				xx = inertial["Ixx"].as<double>();
				xy = inertial["Ixy"].as<double>();
				xz = inertial["Ixz"].as<double>();
				yy = inertial["Iyy"].as<double>();
				yz = inertial["Iyz"].as<double>();
				zz = inertial["Izz"].as<double>();

				par_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass,m_cmx,m_cmy,m_cmz,xx,xy,xz,yy,yz,zz;
				
				i++;
			}
			if (update_DYN) update_inertial_DYN();
		}
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}
	return par_REG;
}

void thunder_robot::load_conf(std::string file_path, bool update_REG){
	try {
		YAML::Node config = YAML::LoadFile(file_path);
		int index;
		// --- Parse kinematics --- //
		if (config["kinematics"]){
			par_DHtable.resize(n_joints*4);
			// Denavit-Hartenberg
			YAML::Node kinematics = config["kinematics"];
			std::vector<double> dh_vect = kinematics["DH"].as<std::vector<double>>();
			// par_DHtable = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);
			if (kinematics["symb"]){
				DHtable_symb = kinematics["symb"].as<std::vector<int>>();
			} else {
				// no parameters here
			}
			int sz = dh_vect.size();
			// par_DHtable.resize(sz);
			// for (int i=0; i<sz; i++){
			// 		par_DHtable(i) = dh_vect[i];
			// 	}
			// }
			// Eigen::VectorXd gravity_new(sz);
			int sz1 = 0;
			for (int i=0; i<sz; i++){
				if (DHtable_symb[i]){
					par_DHtable(sz1) = dh_vect[i];
					sz1++;
				}
			}
			par_DHtable.conservativeResize(sz1);
		}
		// - Frame Offsets - //
		if (config["Base_to_L0"]){
			par_world2L0.resize(6);
			YAML::Node frame_base = config["Base_to_L0"];
			if (frame_base["symb"]){
				world2L0_symb = frame_base["symb"].as<std::vector<int>>();
			} else {
				// no parameters here
			}
			// int sz = 0;
			// for (int v : world2L0_symb) if (v) sz++;
			// std::cout << "sz: " << sz << std::endl;
			std::vector<double> tr = frame_base["tr"].as<std::vector<double>>();
			std::vector<double> ypr = frame_base["ypr"].as<std::vector<double>>();
			// Eigen::VectorXd world2L0_new(sz);
			int sz1=0, sz2=0;
			for (int i=0; i<3; i++){
				if (world2L0_symb[i]){
					par_world2L0(sz1) = tr[i];
					sz1++;
				}
			}
			for (int i=0; i<3; i++){
				if (world2L0_symb[i+3]){
					par_world2L0(sz1+sz2) = ypr[i];
					sz2++;
				}
			}
			par_world2L0.conservativeResize(sz1+sz2);
			// world2L0 = world2L0_new;
		}

		if (config["Ln_to_EE"]){
			par_Ln2EE.resize(6);
			YAML::Node frame_ee = config["Ln_to_EE"];
			if (frame_ee["symb"]){
				Ln2EE_symb = frame_ee["symb"].as<std::vector<int>>();
			} else {
				// no parameters here
			}
			// int sz = 0;
			// for (int v : Ln2EE_symb) if (v) sz++;
			// std::cout << "sz: " << sz << std::endl;
			std::vector<double> tr = frame_ee["tr"].as<std::vector<double>>();
			std::vector<double> ypr = frame_ee["ypr"].as<std::vector<double>>();
			// Eigen::VectorXd Ln2EE_new(sz);
			int sz1=0, sz2=0;
			for (int i=0; i<3; i++){
				if (Ln2EE_symb[i]){
					par_Ln2EE(sz1) = tr[i];
					sz1++;
				}
			}
			for (int i=0; i<3; i++){
				if (Ln2EE_symb[i+3]){
					par_Ln2EE(sz1+sz2) = ypr[i];
					sz2++;
				}
			}
			par_Ln2EE.conservativeResize(sz1+sz2);
			// par_Ln2EE = Ln2EE_new;
		}
		
		// --- Parse dynamics --- //
		if (config["dynamics"]){
			YAML::Node dynamics = config["dynamics"];
			double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
			index=0;
			for (const auto& node : dynamics) {
				if (index==n_joints) break;
				YAML::Node inertial = node.second["inertial"];
				// - inertial parameters - //
				std::string linkName = node.first.as<std::string>();
				mass = inertial["mass"].as<double>();
				cmx = inertial["CoM_x"].as<double>();
				cmy = inertial["CoM_y"].as<double>();
				cmz = inertial["CoM_z"].as<double>();
				xx = inertial["Ixx"].as<double>();
				xy = inertial["Ixy"].as<double>();
				xz = inertial["Ixz"].as<double>();
				yy = inertial["Iyy"].as<double>();
				yz = inertial["Iyz"].as<double>();
				zz = inertial["Izz"].as<double>();

				par_DYN.segment(STD_PAR_LINK*index, STD_PAR_LINK) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
				
				// - link friction - //
				if (node.second["friction"]){
					std::vector<double> Dl = node.second["friction"]["Dl"].as<std::vector<double>>();
					for (int j=0; j<Dl_order; j++){
						par_Dl(Dl_order*index + j) = Dl[j];
					}
				}
				index++;
			}
			if (update_REG) update_inertial_REG();
		}
		// - Gravity - //
		if (config["gravity"]){
			par_gravity.resize(3);
			YAML::Node gravity_node = config["gravity"];
			if (gravity_node["symb"]){
				gravity_symb = gravity_node["symb"].as<std::vector<int>>();
			} else {
				// no parameters here
			}
			// int sz = 0;
			// for (int v : gravity_symb) if (v) sz++;
			std::vector<double> grav = gravity_node["value"].as<std::vector<double>>();
			// Eigen::VectorXd gravity_new(sz);
			int sz1 = 0;
			for (int i=0; i<3; i++){
				if (gravity_symb[i]){
					par_gravity(sz1) = grav[i];
					sz1++;
				}
			}
			par_gravity.conservativeResize(sz1);
			// par_gravity = gravity_new;
		}

		// --- Parse elastic --- //
		if (config["elastic"]){
			YAML::Node elastic = config["elastic"];
			index = 0;
			for (const auto& node : elastic["joints"]) {
				if (index==numElasticJoints) break;
				std::string jointName = node.first.as<std::string>();
				// stiffness
				for (int j=0; j<K_order; j++){
					std::vector<double> K = node.second["K"].as<std::vector<double>>();
					par_K(K_order*index+j) = K[j];
					// std::cout<<"K_"<<j<<": "<<K[j] << std::endl;
				}
				// coupling friction
				for (int j=0; j<D_order; j++){
					std::vector<double> D = node.second["D"].as<std::vector<double>>();
					par_D(D_order*index + j) = D[j];
					// std::cout<<"D_"<<j<<": "<<D[j] << std::endl;
				}
				// motor friction
				for (int j=0; j<Dm_order; j++){
					std::vector<double> Dm = node.second["Dm"].as<std::vector<double>>();
					par_Dm(Dm_order*index + j) = Dm[j];
					// std::cout<<"Dm_"<<j<<": "<<Dm[j] << std::endl;
				}
				// motor Inertia
				par_Mm(index) = node.second["Mm"].as<double>();
				// std::cout<<"Dm_"<<j<<": "<<Dm[j] << std::endl;
				index++;
			}
		}
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}
}

// void thunder_robot::load_par_elastic(std::string file_path){
// 	// ----- parsing yaml elastic ----- //
// 	try {
// 		// load yaml
// 		YAML::Node config_file = YAML::LoadFile(file_path);
// 		// parse elastic
// 		YAML::Node elastic = config_file["elastic"];
// 		int i = 0;
// 		for (const auto& node : elastic["joints"]) {
			
// 			if (i==numElasticJoints) break;
// 			std::string jointName = node.first.as<std::string>();
// 			// stiffness
// 			for (int j=0; j<K_order; j++){
// 				std::vector<double> K = node.second["K"].as<std::vector<double>>();
// 				par_K(K_order*i+j) = K[j];
// 				std::cout<<"K_"<<j<<": "<<K[j] << std::endl;
// 			}
// 			// coupling friction
// 			for (int j=0; j<D_order; j++){
// 				std::vector<double> D = node.second["D"].as<std::vector<double>>();
// 				par_D(D_order*i + j) = D[j];
// 				std::cout<<"D_"<<j<<": "<<D[j] << std::endl;
// 			}
// 			// motor friction
// 			for (int j=0; j<Dm_order; j++){
// 				std::vector<double> Dm = node.second["Dm"].as<std::vector<double>>();
// 				par_Dm(Dm_order*i + j) = Dm[j];
// 				std::cout<<"Dm_"<<j<<": "<<Dm[j] << std::endl;
// 			}
// 			i++;
// 		}
// 		// std::cout<<"YAML_DH letto"<<std::endl;
// 		// std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;
// 	} catch (const YAML::Exception& e) {
// 		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
// 	}
// }

void thunder_robot::save_par_REG(std::string path_yaml_DH_REG){
	std::vector<std::string> keys_reg;
	keys_reg.resize(5);
	keys_reg[0] = "mass"; keys_reg[1] = "m_CoM_"; keys_reg[2] = "I"; keys_reg[3] = "REG"; keys_reg[4] = "regressor";
	std::vector<LinkProp> links_prop_REG;
	links_prop_REG.resize(n_joints);

	for(int i=0; i<n_joints; i++){
		links_prop_REG[i].Dl.resize(Dl_order);
		links_prop_REG[i].name = "link" + std::to_string(i+1);
		links_prop_REG[i].mass = par_REG[STD_PAR_LINK*i + 0];
		links_prop_REG[i].xyz = {par_REG[STD_PAR_LINK*i + 1], par_REG[STD_PAR_LINK*i + 2], par_REG[STD_PAR_LINK*i + 3]};
		links_prop_REG[i].parI[0] = par_REG[STD_PAR_LINK*i + 4];
		links_prop_REG[i].parI[1] = par_REG[STD_PAR_LINK*i + 5];
		links_prop_REG[i].parI[2] = par_REG[STD_PAR_LINK*i + 6];
		links_prop_REG[i].parI[3] = par_REG[STD_PAR_LINK*i + 7];
		links_prop_REG[i].parI[4] = par_REG[STD_PAR_LINK*i + 8];
		links_prop_REG[i].parI[5] = par_REG[STD_PAR_LINK*i + 9];
		for (int j=0; j<Dl_order; j++){
			links_prop_REG[i].Dl[j] = par_Dl[Dl_order*i + j];
		}
	}
	// create file
	try {
		YAML::Emitter emitter;
		fillInertialYaml(n_joints, emitter, links_prop_REG, keys_reg);
		std::ofstream fout(path_yaml_DH_REG);
		fout << emitter.c_str();
		fout.close();

		std::cout << "par_REG saved on path: " << path_yaml_DH_REG << std::endl;

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
	}
}

void thunder_robot::save_par_DYN(std::string path_yaml_DH_DYN){
	std::vector<std::string> keys_reg;
	keys_reg.resize(5);
	keys_reg[0] = "mass"; keys_reg[1] = "CoM_"; keys_reg[2] = "I"; keys_reg[3] = "DYN"; keys_reg[4] = "dynamics";
	std::vector<LinkProp> links_prop_DYN;
	links_prop_DYN.resize(n_joints);

	for(int i=0; i<n_joints; i++){
		links_prop_DYN[i].Dl.resize(Dl_order);
		links_prop_DYN[i].name = "link" + std::to_string(i+1);
		links_prop_DYN[i].mass = par_DYN[STD_PAR_LINK*i + 0];
		links_prop_DYN[i].xyz = {par_DYN[STD_PAR_LINK*i + 1], par_DYN[STD_PAR_LINK*i + 2], par_DYN[STD_PAR_LINK*i + 3]};
		links_prop_DYN[i].parI[0] = par_DYN[STD_PAR_LINK*i + 4];
		links_prop_DYN[i].parI[1] = par_DYN[STD_PAR_LINK*i + 5];
		links_prop_DYN[i].parI[2] = par_DYN[STD_PAR_LINK*i + 6];
		links_prop_DYN[i].parI[3] = par_DYN[STD_PAR_LINK*i + 7];
		links_prop_DYN[i].parI[4] = par_DYN[STD_PAR_LINK*i + 8];
		links_prop_DYN[i].parI[5] = par_DYN[STD_PAR_LINK*i + 9];
		for (int j=0; j<Dl_order; j++){
			links_prop_DYN[i].Dl[j] = par_Dl[Dl_order*i + j];
		}
	}
	// create file
	try {
		YAML::Emitter emitter;
		fillInertialYaml(n_joints, emitter, links_prop_DYN, keys_reg);
		std::ofstream fout(path_yaml_DH_DYN);
		fout << emitter.c_str();
		fout.close();

		std::cout << "par_DYN saved on path: " << path_yaml_DH_DYN << std::endl;

	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
	}
}

int thunder_robot::save_par(std::string par_file){
	try {
		YAML::Emitter emitter;
		emitter.SetIndent(2);
		emitter.SetSeqFormat(YAML::Flow);

		YAML::Node yamlFile;

		// - save DHtable - //
		std::vector<double> DHtable_vect(par_DHtable.data(), par_DHtable.data() + par_DHtable.rows() * par_DHtable.cols());
		yamlFile["DHtable"] = DHtable_vect;

		// - save world2L0 - //
		std::vector<double> world2L0_vect(par_world2L0.data(), par_world2L0.data() + par_world2L0.rows() * par_world2L0.cols());
		yamlFile["world2L0"] = world2L0_vect;

		// - save Ln2EE - //
		std::vector<double> Ln2EE_vect(par_Ln2EE.data(), par_Ln2EE.data() + par_Ln2EE.rows() * par_Ln2EE.cols());
		yamlFile["Ln2EE"] = Ln2EE_vect;

		// - save gravity - //
		std::vector<double> gravity_vect(par_gravity.data(), par_gravity.data() + par_gravity.rows() * par_gravity.cols());
		yamlFile["gravity"] = gravity_vect;

		// - save par_DYN - //
		std::vector<double> par_DYN_vect(par_DYN.data(), par_DYN.data() + par_DYN.rows() * par_DYN.cols());
		yamlFile["par_DYN"] = par_DYN_vect;

		// - save par_REG - //
		std::vector<double> par_REG_vect(par_REG.data(), par_REG.data() + par_REG.rows() * par_REG.cols());
		yamlFile["par_REG"] = par_REG_vect;

		// - save par_K - //
		std::vector<double> par_K_vect(par_K.data(), par_K.data() + par_K.rows() * par_K.cols());
		yamlFile["par_K"] = par_K_vect;

		// - save par_Dl - //
		std::vector<double> par_Dl_vect(par_Dl.data(), par_Dl.data() + par_Dl.rows() * par_Dl.cols());
		yamlFile["par_Dl"] = par_Dl_vect;

		// - save par_D - //
		std::vector<double> par_D_vect(par_D.data(), par_D.data() + par_D.rows() * par_D.cols());
		yamlFile["par_D"] = par_D_vect;

		// - save par_Dm - //
		std::vector<double> par_Dm_vect(par_Dm.data(), par_Dm.data() + par_Dm.rows() * par_Dm.cols());
		yamlFile["par_Dm"] = par_Dm_vect;

		// - save par_Mm - //
		std::vector<double> par_Mm_vect(par_Mm.data(), par_Mm.data() + par_Mm.rows() * par_Mm.cols());
		yamlFile["par_Mm"] = par_Mm_vect;

		emitter << yamlFile << YAML::Newline;

		std::ofstream fout(par_file);
		fout << emitter.c_str();
		fout.close();
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		return 0;
	}
	return 1;
}

// Other functions
void thunder_robot::fillInertialYaml(int n_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_){
	YAML::Node yamlFile;
	YAML::Node dynamicsNode;

	emitter_.SetIndent(2);
	emitter_.SetSeqFormat(YAML::Flow);
	emitter_ << YAML::Comment(
		"Inertial parameters referred to Denavit-Hartenberg parametrization to use " + keys_[4] + "\n");
	emitter_ << YAML::Newline;

	for (int i=0;  i<n_joints;  i++) {

		LinkProp link = links_prop_[i];    
		YAML::Node linkNode;
		std::string nodeName;
		YAML::Node linkInertia;
		YAML::Node linkFric;

		nodeName = link.name;
		linkInertia[keys_[0]] = link.mass;
		linkInertia[keys_[1]+"x"] = link.xyz[0];
		linkInertia[keys_[1]+"y"] = link.xyz[1];
		linkInertia[keys_[1]+"z"] = link.xyz[2];
		linkInertia[keys_[2]+"xx"] = link.parI[0];
		linkInertia[keys_[2]+"xy"] = link.parI[1];
		linkInertia[keys_[2]+"xz"] = link.parI[2];
		linkInertia[keys_[2]+"yy"] = link.parI[3];
		linkInertia[keys_[2]+"yz"] = link.parI[4];
		linkInertia[keys_[2]+"zz"] = link.parI[5];
		// link friction
		linkFric["Dl"] = link.Dl;

		linkNode["inertial"] = linkInertia;
		linkNode["friction"] = linkFric;
		dynamicsNode[nodeName] = linkNode;
	}
	yamlFile["dynamics"] = dynamicsNode;
	emitter_ << yamlFile << YAML::Newline;
}

Eigen::Matrix3d thunder_robot::hat(const Eigen::Vector3d v){
	Eigen::Matrix3d vhat;
			
	// chech
	if(v.size() != 3 ){
		std::cout<<"in function hat of class FrameOffset invalid dimension of input"<<std::endl;
	}
	
	vhat(0,0) = 0;
	vhat(0,1) = -v[2];
	vhat(0,2) = v[1];
	vhat(1,0) = v[2];
	vhat(1,1) = 0;
	vhat(1,2) = -v[0];
	vhat(2,0) = -v[1];
	vhat(2,1) = v[0];
	vhat(2,2) = 0;

	return vhat;
}

Eigen::Matrix3d thunder_robot::rpyRot(const std::vector<double> rpy){
	Eigen::Matrix3d rotTr;
	
	double cy = cos(rpy[2]);
	double sy = sin(rpy[2]);
	double cp = cos(rpy[1]);
	double sp = sin(rpy[1]);
	double cr = cos(rpy[0]);
	double sr = sin(rpy[0]);

	//template R yaw-pitch-roll
	rotTr(0,0)=cy*cp;
	rotTr(0,1)=cy*sp*sr-sy*cr;
	rotTr(0,2)=cy*sp*cr-sy*sr;
	rotTr(1,0)=sy*cp;
	rotTr(1,1)=sy*sp*sr+cy*cr;
	rotTr(1,2)=sy*sp*cr-cy*sr;
	rotTr(2,0)=-sp;
	rotTr(2,1)=cp*sr;
	rotTr(2,2)=cp*cr;

	return rotTr;
}

Eigen::Matrix3d thunder_robot::createI(const std::vector<double> parI){
	Eigen::Matrix3d I;
	I(0, 0) = parI[0];
	I(0, 1) = parI[1];
	I(0, 2) = parI[2];
	I(1, 0) = parI[1];
	I(1, 1) = parI[3];
	I(1, 2) = parI[4];
	I(2, 0) = parI[2];
	I(2, 1) = parI[4];
	I(2, 2) = parI[5];

	return I;
}

// ----- generated functions ----- //
/*#-FUNCTIONS_CPP-#*/


/*#-OPTIONAL SPACE FOR PYTHON BINDINGS-#*/

