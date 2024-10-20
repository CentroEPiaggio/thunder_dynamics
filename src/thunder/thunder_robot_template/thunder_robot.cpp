#include "thunder_robot.h"
#include "robot_gen.h"

thunder_robot::thunder_robot(){
	resizeVariables();
}

void thunder_robot::resizeVariables(){
	q = Eigen::VectorXd::Zero(n_joints);
	dq = Eigen::VectorXd::Zero(n_joints);
	dqr = Eigen::VectorXd::Zero(n_joints);
	ddqr = Eigen::VectorXd::Zero(n_joints);
	x = Eigen::VectorXd::Zero(numElasticJoints);
	dx = Eigen::VectorXd::Zero(numElasticJoints);
	ddxr = Eigen::VectorXd::Zero(numElasticJoints);
	par_REG = Eigen::VectorXd::Zero(STD_PAR_LINK*n_joints);
	par_DYN = Eigen::VectorXd::Zero(STD_PAR_LINK*n_joints);
	par_Dl = Eigen::VectorXd::Zero(Dl_order*n_joints);
	par_K = Eigen::VectorXd::Zero(K_order*numElasticJoints);
	par_D = Eigen::VectorXd::Zero(D_order*numElasticJoints);
	par_Dm = Eigen::VectorXd::Zero(Dm_order*numElasticJoints);
	// par_ELA = Eigen::VectorXd::Zero(numParELA);
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

void thunder_robot::set_ddxr(const Eigen::VectorXd& ddxr_){
	if(ddxr_.size() == numElasticJoints){
		ddxr = ddxr_;
	} else{
		std::cout<<"in set_ddxr: invalid dimensions of arguments\n";
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
		par_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, CoM, I+I_tmp_v;
	}
}

void thunder_robot::set_par_DYN(const Eigen::VectorXd& par_){
	if(par_.size() == STD_PAR_LINK*n_joints){
		par_DYN = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
	// conversion from REG to DYN
	update_inertial_REG();
}

void thunder_robot::set_par_REG(const Eigen::VectorXd& par_){
	if(par_.size() == STD_PAR_LINK*n_joints){
		par_REG = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
	// conversion from REG to DYN
	update_inertial_DYN();
}

void thunder_robot::set_par_K(const Eigen::VectorXd& par_){
	if(par_.size() == K_order*numElasticJoints){
		par_K = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_D(const Eigen::VectorXd& par_){
	if(par_.size() == D_order*numElasticJoints){
		par_D = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_Dm(const Eigen::VectorXd& par_){
	if(par_.size() == Dm_order*numElasticJoints){
		par_Dm = par_;
	} else{
		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	}
}

void thunder_robot::set_par_Dl(const Eigen::VectorXd& par_){
	if(par_.size() == Dl_order*n_joints){
		par_Dl = par_;
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

Eigen::VectorXd thunder_robot::get_par_Dl(){
	return par_Dl;
}

void thunder_robot::load_par_REG(std::string file_path){
	try {
		YAML::Node config = YAML::LoadFile(file_path);
		
		double mass, m_cmx, m_cmy, m_cmz, xx, xy, xz, yy, yz, zz;
		int i = 0;
		for (const auto& node : config) {
			std::string linkName = node.first.as<std::string>();
			mass = node.second["mass"].as<double>();
			m_cmx = node.second["m_CoM_x"].as<double>();
			m_cmy = node.second["m_CoM_y"].as<double>();
			m_cmz = node.second["m_CoM_z"].as<double>();
			xx = node.second["Ixx"].as<double>();
			xy = node.second["Ixy"].as<double>();
			xz = node.second["Ixz"].as<double>();
			yy = node.second["Iyy"].as<double>();
			yz = node.second["Iyz"].as<double>();
			zz = node.second["Izz"].as<double>();

			par_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass,m_cmx,m_cmy,m_cmz,xx,xy,xz,yy,yz,zz;
			
			// link friction
			for (int j=0; j<Dl_order; j++){
				std::vector<double> Dl = node.second["Dl"].as<std::vector<double>>();
				par_Dl(Dl_order*i+j) = Dl[j];
				std::cout<<"Dl_"<<j<<": "<<Dl[j] << std::endl;
			}

			i++;
		}
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}
	update_inertial_DYN();
}

void thunder_robot::load_par_DYN(std::string file_path){
	try {
		YAML::Node config = YAML::LoadFile(file_path);
		
		double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
		int i = 0;
		for (const auto& node : config) {
			// inertial parameters
			std::string linkName = node.first.as<std::string>();
			mass = node.second["mass"].as<double>();
			cmx = node.second["CoM_x"].as<double>();
			cmy = node.second["CoM_y"].as<double>();
			cmz = node.second["CoM_z"].as<double>();
			xx = node.second["Ixx"].as<double>();
			xy = node.second["Ixy"].as<double>();
			xz = node.second["Ixz"].as<double>();
			yy = node.second["Iyy"].as<double>();
			yz = node.second["Iyz"].as<double>();
			zz = node.second["Izz"].as<double>();

			par_DYN.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
			
			// link friction
			for (int j=0; j<Dl_order; j++){
				std::vector<double> Dl = node.second["Dl"].as<std::vector<double>>();
				par_Dl(Dl_order*i+j) = Dl[j];
				std::cout<<"Dl_"<<j<<": "<<Dl[j] << std::endl;
			}

			i++;
		}
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}
	update_inertial_REG();
}

void thunder_robot::load_par_elastic(std::string file_path){
	// ----- parsing yaml elastic ----- //
	try {
		// load yaml
		YAML::Node config_file = YAML::LoadFile(file_path);
		// parse elastic
		YAML::Node elastic = config_file["elastic"];
		int i = 0;
		for (const auto& node : elastic["joints"]) {
			
			if (i==numElasticJoints) break;
			std::string jointName = node.first.as<std::string>();
			// stiffness
			for (int j=0; j<K_order; j++){
				std::vector<double> K = node.second["K"].as<std::vector<double>>();
				par_K(K_order*i+j) = K[j];
				std::cout<<"K_"<<j<<": "<<K[j] << std::endl;
			}
			// coupling friction
			for (int j=0; j<D_order; j++){
				std::vector<double> D = node.second["D"].as<std::vector<double>>();
				par_D(D_order*i + j) = D[j];
				std::cout<<"D_"<<j<<": "<<D[j] << std::endl;
			}
			// motor friction
			for (int j=0; j<Dm_order; j++){
				std::vector<double> Dm = node.second["Dm"].as<std::vector<double>>();
				par_Dm(Dm_order*i + j) = Dm[j];
				std::cout<<"Dm_"<<j<<": "<<Dm[j] << std::endl;
			}

			i++;
		}
		// std::cout<<"YAML_DH letto"<<std::endl;
		// std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;
	} catch (const YAML::Exception& e) {
		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	}
}

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

// Other functions
void thunder_robot::fillInertialYaml(int n_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_){
	YAML::Node control;

	emitter_.SetIndent(2);
	emitter_.SetSeqFormat(YAML::Flow);
	emitter_ << YAML::Comment(
		"Inertial parameters referred to Denavit-Hartenberg parametrization to use " + keys_[4] + "\n");
	emitter_ << YAML::Newline;

	for (int i=0;  i<n_joints;  i++) {

		LinkProp link = links_prop_[i];    
		YAML::Node linkNode;
		std::string nodeName;

		nodeName = link.name;
		linkNode[keys_[0]] = link.mass;
		linkNode[keys_[1]+"x"] = link.xyz[0];
		linkNode[keys_[1]+"y"] = link.xyz[1];
		linkNode[keys_[1]+"z"] = link.xyz[2];
		linkNode[keys_[2]+"xx"] = link.parI[0];
		linkNode[keys_[2]+"xy"] = link.parI[1];
		linkNode[keys_[2]+"xz"] = link.parI[2];
		linkNode[keys_[2]+"yy"] = link.parI[3];
		linkNode[keys_[2]+"yz"] = link.parI[4];
		linkNode[keys_[2]+"zz"] = link.parI[5];
		// link friction
		linkNode["Dl"] = link.Dl;

		emitter_ << YAML::BeginMap;
		emitter_ << YAML::Key << nodeName;
		emitter_ << linkNode;
		emitter_ << YAML::EndMap << YAML::Newline;
	}
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