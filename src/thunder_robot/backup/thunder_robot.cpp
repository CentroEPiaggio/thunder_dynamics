#include "../library/thunder_robot.h"
#include "../library/robot_gen.h"

// constexpr std::string path_yaml_DH_REG = "../robots/franka/generatedFiles/inertial_REG_stored";
// constexpr std::string path_copy_DH_REG = "../robots/franka/generatedFiles/inertial_REG_stored_copy";

const int N_JOINTS = 7;
const int N_PAR_LINK = 10;

using namespace thunder_ns;

namespace thunder_ns{
	// thunder_robot::thunder_robot(){};

	// thunder_robot::thunder_robot(const int nj_):num_joints(nj_){ 
	// 	resizeVariables();
	// }

	thunder_robot::thunder_robot(){
		num_joints = N_JOINTS;
		resizeVariables();
	}
   
	// void thunder_robot::init(const int nj_){
	// 	num_joints = nj_;
	// 	resizeVariables();
	// }

	void thunder_robot::resizeVariables(){
		q = Eigen::VectorXd::Zero(num_joints);
		dq = Eigen::VectorXd::Zero(num_joints);
		dqr = Eigen::VectorXd::Zero(num_joints);
		ddqr = Eigen::VectorXd::Zero(num_joints);
		param = Eigen::VectorXd::Zero(N_PAR_LINK*num_joints);

		reg_gen.resize(num_joints,N_PAR_LINK*num_joints);
		jac_gen.resize(6,num_joints);
		dotJac_gen.resize(6,num_joints);
		pinvJac_gen.resize(num_joints,6);
		pinvJacPos_gen.resize(num_joints,3);
		dotPinvJac_gen.resize(num_joints,6);
		dotPinvJacPos_gen.resize(num_joints,3);
		kin_gen.resize(4,4);
		mass_gen.resize(num_joints,num_joints);
		coriolis_gen.resize(num_joints,num_joints);
		gravity_gen.resize(num_joints,1);
	}

	int thunder_robot::get_numJoints() {return num_joints;};
	int thunder_robot::get_numParams() {return param.size();};
	
	void thunder_robot::setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_){
		if(q_.size() == num_joints && dq_.size()== num_joints && dqr_.size()==num_joints && ddqr_.size()==num_joints){
			q = q_;
			dq = dq_;
			dqr = dqr_;
			ddqr = ddqr_;
		} else{
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
		}
		// computeKin_gen();
		// computeJac_gen();
		// computeReg_gen();
		// computeMass_gen();
		// computeCoriolis_gen();
		// computeGravity_gen();
	}

	void thunder_robot::setInertialParam(const Eigen::VectorXd& param_){
		if(param_.size() == N_PAR_LINK*num_joints){
			param = param_;
		} else{
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
		}
		// computeMass_gen();
		// computeCoriolis_gen();
		// computeGravity_gen();
	}

	Eigen::VectorXd thunder_robot::getInertialParams(){
		return param;
	}
	
	// void thunder_robot::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_,const Eigen::VectorXd& param_){
	// 	if(q_.size() == num_joints && dq_.size()== num_joints && param_.size()== N_PAR_LINK*num_joints){
	// 		q = q_;
	// 		dq = dq_;
	// 		param = param_;
	// 	} else{
	// 		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	// 	}
	// 	// computeMass_gen();
	// 	// computeCoriolis_gen();
	// 	// computeGravity_gen();
	// }

	// void thunder_robot::setArguments(const Eigen::VectorXd& q_,const Eigen::VectorXd& dq_){
	// 	if(q_.size() == num_joints && dq_.size()== num_joints){
	// 		q = q_;
	// 		dq = dq_;
	// 	} else{
	// 		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	// 	}
	// 	// computeJac_gen();
	// 	// computeDotJac_gen();
	// 	// computePinvJac_gen();
	// 	// computeDotPinvJac_gen();
	// 	// computePinvJacPos_gen();
	// 	// computeDotPinvJacPos_gen();
	// 	// computeKin_gen();
	// 	// //computeMass_gen();
	// 	// //computeCoriolis_gen();
	// 	// //computeGravity_gen();
	// }
	
	// void thunder_robot::setArguments(const Eigen::VectorXd& q_){
	// 	if(q_.size() == num_joints){
	// 		q = q_;
	// 	} else{
	// 		std::cout<<"in setArguments: invalid dimensions of arguments\n";
	// 	}
	// 	// computeJac_gen();
	// 	// //computePinvJac_gen();
	// 	// computeKin_gen();
	// 	// //computeMass_gen();
	// 	// //computeGravity_gen();
	// }
	
	void thunder_robot::computeReg_gen(){
		long long p3[regr_fun_SZ_IW];
		double p4[regr_fun_SZ_W];

		const double* input_[] = {q.data(), dq.data(), dqr.data(), ddqr.data()};
		double* output_[] = {reg_gen.data()};
		
		int check = regr_fun(input_, output_, p3, p4, 0);
	}
	
	void thunder_robot::computeMass_gen(){
		long long p3[mass_fun_SZ_IW];
		double p4[mass_fun_SZ_W];

		const double* input_[] = {q.data(), param.data()};
		double* output_[] = {mass_gen.data()};
		
		int check = mass_fun(input_, output_, p3, p4, 0);
	}
	
	void thunder_robot::computeCoriolis_gen(){
		long long p3[coriolis_fun_SZ_IW];
		double p4[coriolis_fun_SZ_W];

		const double* input_[] = {q.data(), dq.data(), param.data()};
		double* output_[] = {coriolis_gen.data()};
		
		int check = coriolis_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computeGravity_gen(){
		long long p3[gravity_fun_SZ_IW];
		double p4[gravity_fun_SZ_W];

		const double* input_[] = {q.data(), param.data()};
		double* output_[] = {gravity_gen.data()};
		int check = gravity_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computeJac_gen(){
		long long p3[jac_fun_SZ_IW];
		double p4[jac_fun_SZ_W];

		const double* input_[] = {q.data()};
		double* output_[] = {jac_gen.data()};

		int check = jac_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computeDotJac_gen(){
		long long p3[dotJac_fun_SZ_IW];
		double p4[dotJac_fun_SZ_W];

		const double* input_[] = {q.data(), dq.data()};
		double* output_[] = {dotJac_gen.data()};

		int check = dotJac_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computePinvJac_gen(){
		long long p3[pinvJac_fun_SZ_IW];
		double p4[pinvJac_fun_SZ_W];

		const double* input_[] = {q.data()};
		double* output_[] = {pinvJac_gen.data()};

		int check = pinvJac_fun(input_, output_, p3, p4, 0);
	}
	
	void thunder_robot::computePinvJacPos_gen(){
		long long p3[pinvJacPos_fun_SZ_IW];
		double p4[pinvJacPos_fun_SZ_W];

		const double* input_[] = {q.data()};
		double* output_[] = {pinvJacPos_gen.data()};

		int check = pinvJacPos_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computeDotPinvJac_gen(){
		long long p3[dotPinvJac_fun_SZ_IW];
		double p4[dotPinvJac_fun_SZ_W];

		const double* input_[] = {q.data(),dq.data()};
		double* output_[] = {dotPinvJac_gen.data()};

		int check = dotPinvJac_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computeDotPinvJacPos_gen(){
		long long p3[dotPinvJacPos_fun_SZ_IW];
		double p4[dotPinvJacPos_fun_SZ_W];

		const double* input_[] = {q.data(),dq.data()};
		double* output_[] = {dotPinvJacPos_gen.data()};

		int check = dotPinvJacPos_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::computeKin_gen(){
		long long p3[kin_fun_SZ_IW];
		double p4[kin_fun_SZ_W];

		const double* input_[] = {q.data()};
		double* output_[] = {kin_gen.data()};

		int check = kin_fun(input_, output_, p3, p4, 0);
	}

	void thunder_robot::loadInertialParam(std::string file_path){
		try {
			YAML::Node config = YAML::LoadFile(file_path);
			
			double mass, m_cmx, m_cmy, m_cmz, xx, xy, xz, yy, yz, zz;
			int i = 0;
			for (const auto& node : config) {
				std::string linkName = node.first.as<std::string>();
				std::cout<<"link_nake:"<<linkName<<std::endl;
				mass = node.second["mass"].as<double>();
				m_cmx = node.second["m_CoM_x"].as<double>();
				m_cmy = node.second["m_CoM_y"].as<double>();
				m_cmy = node.second["m_CoM_y"].as<double>();
				xx = node.second["Ixx"].as<double>();
				xy = node.second["Ixy"].as<double>();
				xz = node.second["Ixz"].as<double>();
				yy = node.second["Iyy"].as<double>();
				yz = node.second["Iyz"].as<double>();
				zz = node.second["Izz"].as<double>();

				param.segment(N_PAR_LINK*i, N_PAR_LINK) << mass,m_cmx,m_cmy,m_cmz,xx,xy,xz,yy,yz,zz;
				i++;
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
	}

	void thunder_robot::saveInertialParam(std::string path_yaml_DH_REG){
		std::vector<std::string> keys_reg;
		keys_reg.resize(5);
		keys_reg[0] = "mass"; keys_reg[1] = "m_CoM_"; keys_reg[2] = "I"; keys_reg[3] = "REG"; keys_reg[4] = "regressor";
		std::vector<LinkProp> links_prop_DH2REG;
		// LinkProp tmp_link;
		// LinkProp tmp_DH_gauss;
		// Eigen::Matrix3d I0,IG;
		// Eigen::Vector3d dOG;
		// double m;
		// // Percentage of perturbation
		// Eigen::VectorXd coeff_p(5);
		// coeff_p << 0, 2, 5, 10, 20;

		for(int i=0; i<num_joints; i++){

			// tmp_link = links_prop_DH[i];
			// perturbateLinkProp(tmp_link,tmp_DH_gauss,coeff_p[w]);

			// m = tmp_DH_gauss.mass;
			// dOG << tmp_DH_gauss.xyz[0], tmp_DH_gauss.xyz[1], tmp_DH_gauss.xyz[2];
			// IG = createI(tmp_DH_gauss.parI);
			// I0 = IG + m * hat(dOG) * hat(dOG).transpose();
			// dOG = m*dOG;

			// links_prop_DH2REG[i].name = tmp_DH_gauss.name;
			// links_prop_DH2REG[i].mass = tmp_DH_gauss.mass;
			// links_prop_DH2REG[i].xyz = {dOG[0],dOG[1],dOG[2]};
			// links_prop_DH2REG[i].parI[0] = I0(0,0);
			// links_prop_DH2REG[i].parI[1] = I0(0,1);
			// links_prop_DH2REG[i].parI[2] = I0(0,2);
			// links_prop_DH2REG[i].parI[3] = I0(1,1);
			// links_prop_DH2REG[i].parI[4] = I0(1,2);
			// links_prop_DH2REG[i].parI[5] = I0(2,2);
			links_prop_DH2REG[i].name = "link" + std::to_string(i);
			links_prop_DH2REG[i].mass = param[N_PAR_LINK*i + 0];
			links_prop_DH2REG[i].xyz = {param[N_PAR_LINK*i + 1], param[N_PAR_LINK*i + 2], param[N_PAR_LINK*i + 3]};
			links_prop_DH2REG[i].parI[0] = param[N_PAR_LINK*i + 4];
			links_prop_DH2REG[i].parI[1] = param[N_PAR_LINK*i + 5];
			links_prop_DH2REG[i].parI[2] = param[N_PAR_LINK*i + 6];
			links_prop_DH2REG[i].parI[3] = param[N_PAR_LINK*i + 7];
			links_prop_DH2REG[i].parI[4] = param[N_PAR_LINK*i + 8];
			links_prop_DH2REG[i].parI[5] = param[N_PAR_LINK*i + 9];
		}

		try {
			YAML::Emitter emitter;
			fillInertialYaml(emitter, links_prop_DH2REG, keys_reg);
			// std::string pp = "";
			// if((int)coeff_p[w]==0) pp="";
			// else pp = "_p" + std::to_string((int)coeff_p[w]);
			// std::ofstream fout(path_yaml_DH_REG + ".yaml");
			std::ofstream fout(path_yaml_DH_REG);
			fout << emitter.c_str();
			fout.close();

			std::cout << "Successfully generated YAML file of inertial parameters for regressor"<<std::endl;
			std::cout<< " path: " << path_yaml_DH_REG<< std::endl;

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		}
		// if (copy_flag){
		// 	std::string absolutePath;
		// 	std::filesystem::path sourcePath;
		// 	std::filesystem::path sourceDestPath;
		// 	absolutePath = std::filesystem::current_path();
		// 	sourcePath = absolutePath + "/" + path_yaml_DH_REG + ".yaml";
		// 	sourceDestPath = path_copy_DH_REG;
		// 	std::filesystem::copy_file(sourcePath, sourceDestPath, std::filesystem::copy_options::update_existing);
		// 	std::cout<<"Files yaml copied"<<std::endl;
		// }
	}

	/* Get regressor matrix */
	Eigen::MatrixXd thunder_robot::getReg_gen(){computeReg_gen(); return reg_gen;};
	/* Get regressor matrix */
	Eigen::MatrixXd thunder_robot::getMass_gen(){computeMass_gen(); return mass_gen;};
	/* Get regressor matrix */
	Eigen::MatrixXd thunder_robot::getCoriolis_gen(){computeCoriolis_gen(); return coriolis_gen;};
	/* Get regressor matrix */
	Eigen::MatrixXd thunder_robot::getGravity_gen(){computeGravity_gen(); return gravity_gen;};
	/* Get jacobian matrix */
	Eigen::MatrixXd thunder_robot::getJac_gen(){computeJac_gen(); return jac_gen;};
	/* Get derivative of jacobian matrix */
	Eigen::MatrixXd thunder_robot::getDotJac_gen(){computeDotJac_gen(); return dotJac_gen;};
	/* Get pseudo-inverse jacobian matrix */
	Eigen::MatrixXd thunder_robot::getPinvJac_gen(){computePinvJac_gen(); return pinvJac_gen;};
	/* Get derivative of pseudo-inverse jacobian matrix only position */
	Eigen::MatrixXd thunder_robot::getPinvJacPos_gen(){computePinvJacPos_gen(); return pinvJacPos_gen;};
	/* Get derivative of pseudo-inverse jacobian matrix */
	Eigen::MatrixXd thunder_robot::getDotPinvJac_gen(){computeDotPinvJac_gen(); return dotPinvJac_gen;};
	/* Get derivative of pseudo-inverse jacobian matrix only position */
	Eigen::MatrixXd thunder_robot::getDotPinvJacPos_gen(){computeDotPinvJacPos_gen(); return dotPinvJacPos_gen;};
	/* Get regressor matrix */
	Eigen::MatrixXd thunder_robot::getKin_gen(){computeKin_gen(); return kin_gen;};

	// Other functions
	void thunder_robot::fillInertialYaml(YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_){
		YAML::Node control;

		emitter_.SetIndent(2);
		emitter_.SetSeqFormat(YAML::Flow);
		emitter_ << YAML::Comment(
			"Inertial parameters referred to Denavit-Hartenberg parametrization to use " + keys_[4] + "\n");
		emitter_ << YAML::Newline;

		for (int i=0;  i<num_joints;  i++) {

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

			emitter_ << YAML::BeginMap;
			emitter_ << YAML::Key << nodeName;
			emitter_ << linkNode;
			emitter_ << YAML::EndMap << YAML::Newline;
		}
	}

	void thunder_robot::trasformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body){
        Eigen::Vector3d OuGi; 
        Eigen::Vector3d OiGi;
        Eigen::Vector3d dist_i;
        Eigen::Matrix3d Riu = rpyRot(rpy_i);
        Eigen::Matrix3d Rub = rpyRot(body_urdf.rpy);
        Eigen::Matrix3d Rib = Riu*Rub;

        Eigen::Matrix3d IGi_B = createI(body_urdf.parI);
        Eigen::Matrix3d IOi_i;
        Eigen::Matrix3d d_hat;

        OuGi << body_urdf.xyz[0], body_urdf.xyz[1],body_urdf.xyz[2]; 
        dist_i << d_i[0], d_i[1], d_i[2];
        OiGi = dist_i + Riu*OuGi;
        d_hat = hat(OiGi);

        IOi_i = Rib*IGi_B*Rib.transpose();

        body.mass = body_urdf.mass;
        body.xyz = {OiGi(0),OiGi(1),OiGi(2)};
        body.parI = {IOi_i(0,0),IOi_i(0,1),IOi_i(0,2),IOi_i(1,1),IOi_i(1,2),IOi_i(2,2)};
        body.name = body_urdf.name;
    }

    void thunder_robot::mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody){
        Eigen::Vector3d G1Gnew;
        Eigen::Vector3d G2Gnew;
        Eigen::Vector3d O1G1;
        Eigen::Vector3d O2G2;
        Eigen::Vector3d newCoM;        
        Eigen::Matrix3d d_hat1;
        Eigen::Matrix3d d_hat2;
        Eigen::Matrix3d newI;
        Eigen::Matrix3d IG1 = createI(body1.parI);
        Eigen::Matrix3d IG2 = createI(body2.parI);
        
        O1G1 << body1.xyz[0], body1.xyz[1], body1.xyz[2]; 
        O2G2 << body2.xyz[0], body2.xyz[1], body2.xyz[2]; 
        
        newCoM = (body1.mass*O1G1 + body2.mass*O2G2)/(body1.mass + body2.mass);

        G1Gnew = newCoM-O1G1;
        G2Gnew = newCoM-O2G2;
        
        d_hat1 = hat(G1Gnew);
        d_hat2 = hat(G2Gnew);

        newI = IG1 + body1.mass*d_hat1*d_hat1.transpose() + IG2 + body2.mass*d_hat2*d_hat2.transpose();

        newBody.mass = body1.mass + body2.mass;
        newBody.xyz = {newCoM(0),newCoM(1),newCoM(2)};
        newBody.parI = {newI(0,0),newI(0,1),newI(0,2),newI(1,1),newI(1,2),newI(2,2)};
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
}