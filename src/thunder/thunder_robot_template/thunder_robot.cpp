#include "thunder_robot.h"
#include "robot_gen.h"

// constexpr std::string path_yaml_DH_REG = "../robots/franka/generatedFiles/inertial_REG_stored";
// constexpr std::string path_copy_DH_REG = "../robots/franka/generatedFiles/inertial_REG_stored_copy";

const int N_JOINTS = 3;
const int N_PAR_LINK = 10;

using namespace thunder_ns;

namespace thunder_ns{

	thunder_robot::thunder_robot(){
		num_joints = N_JOINTS;
		resizeVariables();
	}

	void thunder_robot::resizeVariables(){
		q = Eigen::VectorXd::Zero(num_joints);
		dq = Eigen::VectorXd::Zero(num_joints);
		dqr = Eigen::VectorXd::Zero(num_joints);
		ddqr = Eigen::VectorXd::Zero(num_joints);
		par_REG = Eigen::VectorXd::Zero(N_PAR_LINK*num_joints);
		par_DYN = Eigen::VectorXd::Zero(N_PAR_LINK*num_joints);
	}

	int thunder_robot::get_numJoints() {return num_joints;};
	int thunder_robot::get_numParams() {return par_REG.size();};
	
	void thunder_robot::setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_){
		if(q_.size() == num_joints && dq_.size()== num_joints && dqr_.size()==num_joints && ddqr_.size()==num_joints){
			q = q_;
			dq = dq_;
			dqr = dqr_;
			ddqr = ddqr_;
		} else{
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
		}
	}

	void thunder_robot::update_inertial_DYN(){
		for (int i=0; i<num_joints; i++){
			Eigen::VectorXd p_reg = par_REG.segment(N_PAR_LINK*i, N_PAR_LINK);
			double mass = p_reg(0);
			Eigen::Vector3d CoM = {p_reg(1)/mass, p_reg(2)/mass, p_reg(3)/mass};
			Eigen::Matrix3d I_tmp = mass * hat(CoM) * hat(CoM).transpose();
			Eigen::Vector<double, 6> I_tmp_v;
			I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
			Eigen::Vector<double, 6> I;
			I << p_reg(4), p_reg(5), p_reg(6), p_reg(7), p_reg(8), p_reg(9);
			par_DYN.segment(N_PAR_LINK*i, N_PAR_LINK) << mass, CoM, I-I_tmp_v;
		}
	}

	void thunder_robot::update_inertial_REG(){
		for (int i=0; i<num_joints; i++){
			Eigen::VectorXd p_dyn = par_DYN.segment(N_PAR_LINK*i, N_PAR_LINK);
			double mass = p_dyn(0);
			Eigen::Vector3d CoM = {p_dyn(1), p_dyn(2), p_dyn(3)};
			Eigen::Vector3d m_CoM = mass * CoM;
			Eigen::Matrix3d I_tmp = mass * hat(CoM) * hat(CoM).transpose();
			Eigen::Vector<double, 6> I_tmp_v;
			I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
			Eigen::Vector<double, 6> I;
			I << p_dyn(4), p_dyn(5), p_dyn(6), p_dyn(7), p_dyn(8), p_dyn(9);
			par_REG.segment(N_PAR_LINK*i, N_PAR_LINK) << mass, CoM, I+I_tmp_v;
		}
	}

	void thunder_robot::set_inertial_REG(const Eigen::VectorXd& par_){
		if(par_.size() == N_PAR_LINK*num_joints){
			par_REG = par_;
		} else{
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
		}
		// conversion from REG to DYN
		update_inertial_DYN();
	}

	void thunder_robot::set_inertial_DYN(const Eigen::VectorXd& par_){
		if(par_.size() == N_PAR_LINK*num_joints){
			par_DYN = par_;
		} else{
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
		}
		// conversion from REG to DYN
		update_inertial_REG();
	}

	Eigen::VectorXd thunder_robot::get_inertial_REG(){
		return par_REG;
	}

	Eigen::VectorXd thunder_robot::get_inertial_DYN(){
		return par_DYN;
	}

	void thunder_robot::load_inertial_REG(std::string file_path){
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

				par_REG.segment(N_PAR_LINK*i, N_PAR_LINK) << mass,m_cmx,m_cmy,m_cmz,xx,xy,xz,yy,yz,zz;
				i++;
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		update_inertial_DYN();
	}

	void thunder_robot::load_inertial_DYN(std::string file_path){
		try {
			YAML::Node config = YAML::LoadFile(file_path);
			
			double mass, cmx, cmy, cmz, xx, xy, xz, yy, yz, zz;
			int i = 0;
			for (const auto& node : config) {
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

				par_DYN.segment(N_PAR_LINK*i, N_PAR_LINK) << mass,cmx,cmy,cmz,xx,xy,xz,yy,yz,zz;
				i++;
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		update_inertial_REG();
	}

	void thunder_robot::save_inertial_REG(std::string path_yaml_DH_REG){
		std::vector<std::string> keys_reg;
		keys_reg.resize(5);
		keys_reg[0] = "mass"; keys_reg[1] = "m_CoM_"; keys_reg[2] = "I"; keys_reg[3] = "REG"; keys_reg[4] = "regressor";
		std::vector<LinkProp> links_prop_REG;
		links_prop_REG.resize(num_joints);

		for(int i=0; i<num_joints; i++){
			links_prop_REG[i].name = "link" + std::to_string(i+1);
			links_prop_REG[i].mass = par_REG[N_PAR_LINK*i + 0];
			links_prop_REG[i].xyz = {par_REG[N_PAR_LINK*i + 1], par_REG[N_PAR_LINK*i + 2], par_REG[N_PAR_LINK*i + 3]};
			links_prop_REG[i].parI[0] = par_REG[N_PAR_LINK*i + 4];
			links_prop_REG[i].parI[1] = par_REG[N_PAR_LINK*i + 5];
			links_prop_REG[i].parI[2] = par_REG[N_PAR_LINK*i + 6];
			links_prop_REG[i].parI[3] = par_REG[N_PAR_LINK*i + 7];
			links_prop_REG[i].parI[4] = par_REG[N_PAR_LINK*i + 8];
			links_prop_REG[i].parI[5] = par_REG[N_PAR_LINK*i + 9];
		}
		// create file
		try {
			YAML::Emitter emitter;
			fillInertialYaml(num_joints, emitter, links_prop_REG, keys_reg);
			std::ofstream fout(path_yaml_DH_REG);
			fout << emitter.c_str();
			fout.close();

			std::cout << "par_REG saved on path: " << path_yaml_DH_REG << std::endl;

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		}
	}

	void thunder_robot::save_inertial_DYN(std::string path_yaml_DH_DYN){
		std::vector<std::string> keys_reg;
		keys_reg.resize(5);
		keys_reg[0] = "mass"; keys_reg[1] = "CoM_"; keys_reg[2] = "I"; keys_reg[3] = "DYN"; keys_reg[4] = "dynamics";
		std::vector<LinkProp> links_prop_DYN;
		links_prop_DYN.resize(num_joints);

		for(int i=0; i<num_joints; i++){
			links_prop_DYN[i].name = "link" + std::to_string(i+1);
			links_prop_DYN[i].mass = par_DYN[N_PAR_LINK*i + 0];
			links_prop_DYN[i].xyz = {par_DYN[N_PAR_LINK*i + 1], par_DYN[N_PAR_LINK*i + 2], par_DYN[N_PAR_LINK*i + 3]};
			links_prop_DYN[i].parI[0] = par_DYN[N_PAR_LINK*i + 4];
			links_prop_DYN[i].parI[1] = par_DYN[N_PAR_LINK*i + 5];
			links_prop_DYN[i].parI[2] = par_DYN[N_PAR_LINK*i + 6];
			links_prop_DYN[i].parI[3] = par_DYN[N_PAR_LINK*i + 7];
			links_prop_DYN[i].parI[4] = par_DYN[N_PAR_LINK*i + 8];
			links_prop_DYN[i].parI[5] = par_DYN[N_PAR_LINK*i + 9];
		}
		// create file
		try {
			YAML::Emitter emitter;
			fillInertialYaml(num_joints, emitter, links_prop_DYN, keys_reg);
			std::ofstream fout(path_yaml_DH_DYN);
			fout << emitter.c_str();
			fout.close();

			std::cout << "par_DYN saved on path: " << path_yaml_DH_DYN << std::endl;

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while generating YAML: " << e.what() << std::endl;
		}
	}

	// Other functions
	void thunder_robot::fillInertialYaml(int num_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_){
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
//#-FUNCTIONS_CPP-#//
}