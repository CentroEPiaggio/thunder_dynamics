#include <yaml-cpp/yaml.h>
#include "../library/robot.h"
#include "../library/kinematics.h"
#include "../library/dynamics.h"
#include "../library/regressors.h"
#include "../library/utils.h"
#include "../library/ConfigLoader.h"
#include "../library/DHConfigLoader.h"
#include "../library/URDFConfigLoader.h"

/* File name of generated code */
// #define GENERATED_FILE "kin_basic_fun.cpp"
// #define GENERATED_FILE "kinematics_fun.cpp"
// #define GENERATED_FILE "dynamics_fun.cpp"
#define GENERATED_FILE "robot_gen.cpp"

/* Define number of function generable */
// #define NUMBER_FUNCTIONS 10
// #define MU 0.02
// #define N_PAR_LINK = 10

using std::cout;
using std::endl;

namespace thunder_ns{

	// constexpr double MU = 0.02; //pseudo-inverse damping coeff
	// constexpr unsigned int N_PAR_LINK = 10; // number of link+joint parameters
	// constexpr unsigned int NUMBER_FUNCTIONS = 10; // number of generable functions

	Robot::Robot(const Config conf){
		numJoints = conf.numJoints;
		jointsType = conf.jointsType;
		// numParLink.resize(numJoints);
		ELASTIC = conf.ELASTIC;
		K_order = conf.K_order;
		D_order = conf.D_order;
		Dl_order = conf.Dl_order;
		Dm_order = conf.Dm_order;
		// obtain number of parameters
		// PAR_DYN_LINK = STD_PAR_LINK + Dl_order;
		// PAR_REG_LINK = STD_PAR_LINK + Dl_order;
		// PAR_ELA_LINK = K_order + D_order + Dm_order;
		// numParDYN = STD_PAR_LINK*numJoints;
		// numParREG = numParDYN;
		if (jointsType.size()!=numJoints){
			// problem on number of joints or joints type !!!
			// --- To handle!!! --- //
		}
		numElasticJoints = 0;
		isElasticJoint.resize(numJoints);
		for (int i=0; i<numJoints; i++){
			isElasticJoint[i] = 0;
			// numParLink[i] = STD_PAR_LINK;
			if ((jointsType[i] == "R_SEA")||(jointsType[i] == "P_SEA")) {
				// numParELA += K_order + D_order + Dm_order;
				// numParLink[i] += K_order + D_order;
				isElasticJoint[i] = 1;
				numElasticJoints++;
			}
		}
		// numParELA = numElasticJoints*(PAR_ELA_LINK);

		// N_PAR_LINK = 10;
		// jointsType = jointsType;
		// _DHtable_ = conf.DHtable;
		_world2L0_ = conf.base_frame;
		// N_PAR_LINK = 10;
		// gravity = base_frame.get_gravity();
		_Ln2EE_ = conf.ee_frame;
		valid = 1;
		// _mu_ = MU;

		initVarsFuns();
		// compute();
	}

	void Robot::initVarsFuns(){
		
		// if (_DHtable_.rows() != numJoints || _DHtable_.cols() != 4){
		// 	throw std::runtime_error("DHTemplate: Error size of DH table");
		// }
		if ((int)jointsType.size()!=numJoints){
			throw std::runtime_error("DHFwkinJoints: Error size of joints string");
		}

		casadi::SX q = casadi::SX::sym("q", numJoints,1);
		casadi::SX dq = casadi::SX::sym("dq", numJoints,1);
		casadi::SX dqr = casadi::SX::sym("dqr", numJoints,1);
		casadi::SX ddqr = casadi::SX::sym("ddqr", numJoints,1);
		casadi::SX x = casadi::SX::sym("x", numElasticJoints,1);
		casadi::SX dx = casadi::SX::sym("dx", numElasticJoints,1);
		casadi::SX ddxr = casadi::SX::sym("ddxr", numElasticJoints,1);
		// _par_KIN_ = casadi::SX::sym("par_DYN", N_PAR_KIN,1);
		casadi::SX par_DYN = casadi::SX::sym("par_DYN", STD_PAR_LINK*numJoints,1);
		casadi::SX par_REG = casadi::SX::sym("par_REG", STD_PAR_LINK*numJoints,1);
		casadi::SX par_Dl = casadi::SX::sym("par_Dl", numJoints*Dl_order,1);
		casadi::SX par_K = casadi::SX::sym("par_K", numElasticJoints*K_order,1);
		casadi::SX par_D = casadi::SX::sym("par_D", numElasticJoints*D_order,1);
		casadi::SX par_Dm = casadi::SX::sym("par_Dm", numElasticJoints*Dm_order,1);
		
		// model update
		model.insert({"q", q});
		model.insert({"dq", dq});
		model.insert({"dqr", dqr});
		model.insert({"ddqr", ddqr});
		// model.insert({"par_KIN", _par_KIN_});
		model.insert({"par_DYN", par_DYN});
		model.insert({"par_REG", par_REG});
		if (Dl_order > 0){
			model.insert({"par_Dl", par_Dl});
		}
		if (ELASTIC){
			model.insert({"x", x});
			model.insert({"dx", dx});
			model.insert({"ddxr", ddxr});
			model.insert({"par_K", par_K});
			model.insert({"par_D", par_D});
			model.insert({"par_Dm", par_Dm});
		}

		args.insert({"q", casadi::SX::zeros(numJoints)});
		args.insert({"dq", casadi::SX::zeros(numJoints)});
		args.insert({"dqr", casadi::SX::zeros(numJoints)});
		args.insert({"ddqr", casadi::SX::zeros(numJoints)});
		// args.insert({"par_KIN", par_KIN});
		args.insert({"par_DYN", casadi::SX::zeros(STD_PAR_LINK*numJoints)});
		args.insert({"par_REG", casadi::SX::zeros(STD_PAR_LINK*numJoints)});
		if (Dl_order > 0){
			args.insert({"par_Dl", casadi::SX::zeros(Dl_order*numJoints)});
		}
		if (ELASTIC){
			args.insert({"x", casadi::SX::zeros(numElasticJoints)});
			args.insert({"dx", casadi::SX::zeros(numElasticJoints)});
			args.insert({"ddxr", casadi::SX::zeros(numElasticJoints)});
			args.insert({"par_K", casadi::SX::zeros(K_order*numElasticJoints)});
			args.insert({"par_D", casadi::SX::zeros(D_order*numElasticJoints)});
			args.insert({"par_Dm", casadi::SX::zeros(Dm_order*numElasticJoints)});
		}
		// args.insert({"par_ELA", casadi::SX::zeros(numParELA)});
	}

	Eigen::MatrixXd Robot::get(std::string name){
		std::vector<casadi::SX> result;
		// Eigen::MatrixXd result_num;
		if (fun_args.count(name)){
    		// key exists
			// cout<<"ok1, name: "<<name<<endl;
			Eigen::MatrixXd result_num(model[name].rows(), model[name].columns());
			// cout<<"ok1.5, output_size: "<<result_num.size()<<endl;
			auto f_args = fun_args[name];
			int sz = f_args.size();
			// cout<<"ok2, f_args:"<<f_args<<", size: "<<sz<<endl;
			casadi::SXVector inputs(sz);
			int i=0;
			for (const auto& arg : f_args) {
				inputs[i] = args[arg];
				i++;
			}
			// cout<<"ok3, args: "<<inputs<<endl;
			casadi::Function fun = casadi_fun[name];
			// cout<<"ok3.5, fun: "<<fun<<endl;
			fun.call(inputs, result);
			// cout<<"ok4, result: "<<result<<endl;
			std::vector<casadi::SXElem> res_elements = result[0].get_elements();
			// cout<<"ok5, elements: "<<res_elements<<endl;
			std::transform(res_elements.begin(), res_elements.end(), result_num.data(), mapFunction);
			// cout<<"ok6, result: "<<result_num<<endl;
			return result_num;
		} else {
			cout<<name + " not recognised"<<endl;
			// result_num.resize(1,1);
			// result_num << 0;
			return Eigen::Matrix<double, 1, 1>::Zero();
		}
		
		// return result_num;
	}

	int Robot::set_arg(std::string name, Eigen::VectorXd value){
		int sz = value.size();
		casadi::SX& arg = args[name];
		arg = casadi::SX::zeros(sz);
		for (int i=0; i<sz; i++){
			arg(i) = value(i);
		}

		return 1;
	}

	int Robot::set_q(Eigen::VectorXd value){
		casadi::SX& q = args["q"];
		if (value.size() == numJoints){
			for (int i=0; i<numJoints; i++){
				q(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_dq(Eigen::VectorXd value){
		casadi::SX& dq = args["dq"];
		if (value.size() == numJoints){
			for (int i=0; i<numJoints; i++){
				dq(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_dqr(Eigen::VectorXd value){
		casadi::SX& dqr = args["dqr"];
		if (value.size() == numJoints){
			for (int i=0; i<numJoints; i++){
				dqr(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_ddqr(Eigen::VectorXd value){
		casadi::SX& ddqr = args["ddqr"];
		if (value.size() == numJoints){
			for (int i=0; i<numJoints; i++){
				ddqr(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_x(Eigen::VectorXd value){
		casadi::SX& x = args["x"];
		if (value.size() == numElasticJoints){
			for (int i=0; i<numElasticJoints; i++){
				x(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_dx(Eigen::VectorXd value){
		casadi::SX& dx = args["dx"];
		if (value.size() == numElasticJoints){
			for (int i=0; i<numElasticJoints; i++){
				dx(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_ddxr(Eigen::VectorXd value){
		casadi::SX& ddxr = args["ddxr"];
		if (value.size() == numElasticJoints){
			for (int i=0; i<numElasticJoints; i++){
				ddxr(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_par_DYN(Eigen::VectorXd value){
		casadi::SX& par_DYN = args["par_DYN"];
		int numPar = STD_PAR_LINK*numJoints;
		if (value.size() == numPar){
			for (int i=0; i<numPar; i++){
				par_DYN(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
		// change par_REG
		update_inertial_REG();
	}

	int Robot::set_par_REG(Eigen::VectorXd value){
		casadi::SX& par_REG = args["par_REG"];
		int numPar = STD_PAR_LINK*numJoints;
		if (value.size() == numPar){
			for (int i=0; i<numPar; i++){
				par_REG(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
		update_inertial_DYN();
	}

	int Robot::set_par_K(Eigen::VectorXd value){
		casadi::SX& par_K = args["par_K"];
		int numPar = K_order*numElasticJoints;
		if (value.size() == numPar){
			for (int i=0; i<numPar; i++){
				par_K(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_par_D(Eigen::VectorXd value){
		casadi::SX& par_D = args["par_D"];
		int numPar = D_order*numElasticJoints;
		if (value.size() == numPar){
			for (int i=0; i<numPar; i++){
				par_D(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_par_Dm(Eigen::VectorXd value){
		casadi::SX& par_Dm = args["par_Dm"];
		int numPar = Dm_order*numElasticJoints;
		if (value.size() == numPar){
			for (int i=0; i<numPar; i++){
				par_Dm(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	int Robot::set_par_Dl(Eigen::VectorXd value){
		casadi::SX& par_Dl = args["par_Dl"];
		int numPar = Dl_order*numJoints;
		if (value.size() == numPar){
			for (int i=0; i<numPar; i++){
				par_Dl(i) = value(i);
			}
			return 1;
		} else {
			std::cout<<"in setArguments: invalid dimensions of arguments\n";
			return 0;
		}
	}

	Eigen::VectorXd Robot::get_par_DYN(){
		const casadi::SX& par_DYN_casadi = args["par_DYN"];
		int numPar = STD_PAR_LINK*numJoints;
		Eigen::VectorXd param_DYN(numPar);
		std::vector<casadi::SXElem> res_elements = par_DYN_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param_DYN.data(), mapFunction);
		return param_DYN;
	}

	Eigen::VectorXd Robot::get_par_REG(){
		const casadi::SX& par_REG_casadi = args["par_REG"];
		int numPar = STD_PAR_LINK*numJoints;
		Eigen::VectorXd param_REG(numPar);
		std::vector<casadi::SXElem> res_elements = par_REG_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param_REG.data(), mapFunction);
		return param_REG;
	}

	Eigen::VectorXd Robot::get_par_K(){
		const casadi::SX& par_K_casadi = args["par_K"];
		int numPar = K_order*numElasticJoints;
		Eigen::VectorXd param_K(numPar);
		std::vector<casadi::SXElem> res_elements = par_K_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param_K.data(), mapFunction);
		return param_K;
	}

	Eigen::VectorXd Robot::get_par_D(){
		const casadi::SX& par_D_casadi = args["par_D"];
		int numPar = D_order*numElasticJoints;
		Eigen::VectorXd param_D(numPar);
		std::vector<casadi::SXElem> res_elements = par_D_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param_D.data(), mapFunction);
		return param_D;
	}

	Eigen::VectorXd Robot::get_par_Dm(){
		const casadi::SX& par_Dm_casadi = args["par_Dm"];
		int numPar = Dm_order*numElasticJoints;
		Eigen::VectorXd param_Dm(numPar);
		std::vector<casadi::SXElem> res_elements = par_Dm_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param_Dm.data(), mapFunction);
		return param_Dm;
	}

	Eigen::VectorXd Robot::get_par_Dl(){
		const casadi::SX& par_Dl_casadi = args["par_Dl"];
		int numPar = Dl_order*numJoints;
		Eigen::VectorXd param_Dl(numPar);
		std::vector<casadi::SXElem> res_elements = par_Dl_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param_Dl.data(), mapFunction);
		return param_Dl;
	}

	// Eigen::VectorXd Robot::get_par_ELA(){
	// 	const casadi::SX& par_REG_casadi = args["par_ELA"];
	// 	int numPar = STD_PAR_LINK*numJoints;
	// 	Eigen::VectorXd par_ELA(sz);
	// 	std::vector<casadi::SXElem> res_elements = par_ELA_casadi.get_elements();
	// 	std::transform(res_elements.begin(), res_elements.end(), par_ELA.data(), mapFunction);
	// 	return par_ELA;
	// }

	std::vector<fun_obj> Robot::get_functions(bool onlyNames) {
		std::vector<fun_obj> functions;
		int sz = casadi_fun.size();
		functions.resize(sz);
		int i=0;
		for (auto &f : casadi_fun){
			std::string name = f.first;
			functions[i].name = f.first;
			functions[i].description = fun_descr[name];
			functions[i].args = fun_args[name];
			functions[i].out_size.resize(2);
			functions[i].out_size[0] = model[name].size1();
			functions[i].out_size[1] = model[name].size2();
			if (!onlyNames){
				functions[i].expr = model[name];
				functions[i].fun = f.second;
			}
			i++;
		}
		return functions;
	}

	int Robot::get_numJoints(){
		return numJoints;
	}

	bool Robot::get_ELASTIC(){ return ELASTIC;	};
	int Robot::get_K_order(){ return K_order; };
	int Robot::get_D_order(){ return D_order; };
	int Robot::get_Dl_order(){ return Dl_order; };
	int Robot::get_Dm_order(){ return Dm_order; };
	int Robot::get_numElasticJoints(){ return numElasticJoints; };
	std::vector<int> Robot::get_isElasticJoint(){ return isElasticJoint; };

	// std::vector<int> Robot::get_numParLink(){
	// 	return numParLink;
	// }

	// int Robot::get_numParLink(int i){
	// 	return numParLink[i];
	// }

	int Robot::get_numParDYN(){
		return STD_PAR_LINK*numJoints;
	}

	int Robot::get_numParREG(){
		return STD_PAR_LINK*numJoints;
	}

	// int Robot::get_numParELA(){
	// 	return numParELA;
	// }

	std::vector<std::string> Robot::get_jointsType(){
		return jointsType;
	}

	std::vector<std::vector<casadi::SX>> Robot::get_kin_pars(){
		return _KinParams_;
	}

	FrameOffset Robot::get_world2L0(){
		return _world2L0_;
	}

	FrameOffset Robot::get_Ln2EE(){
		return _Ln2EE_;
	}

	int Robot::load_par_DYN(std::string file){
		// Eigen::VectorXd param_DYN;
		casadi::SX param_DYN(STD_PAR_LINK*numJoints,1);
		casadi::SX param_Dl(Dl_order*numJoints,1);
		// ----- parsing yaml inertial ----- //
		try {
			// load yaml
			YAML::Node config_file = YAML::LoadFile(file);
			// load inertial
			YAML::Node inertial = config_file["inertial"];
			int i = 0;
			for (const auto& node : inertial) {
				
				if (i==numJoints) break;
				std::string linkName = node.first.as<std::string>();

				// standard parameters
				param_DYN(STD_PAR_LINK*i) = node.second["mass"].as<double>();
				param_DYN(STD_PAR_LINK*i+1) = node.second["CoM_x"].as<double>();
				param_DYN(STD_PAR_LINK*i+2) = node.second["CoM_y"].as<double>();
				param_DYN(STD_PAR_LINK*i+3) = node.second["CoM_z"].as<double>();
				param_DYN(STD_PAR_LINK*i+4) = node.second["Ixx"].as<double>();
				param_DYN(STD_PAR_LINK*i+5) = node.second["Ixy"].as<double>();
				param_DYN(STD_PAR_LINK*i+6) = node.second["Ixz"].as<double>();
				param_DYN(STD_PAR_LINK*i+7) = node.second["Iyy"].as<double>();
				param_DYN(STD_PAR_LINK*i+8) = node.second["Iyz"].as<double>();
				param_DYN(STD_PAR_LINK*i+9) = node.second["Izz"].as<double>();
				
				// link friction
				if (node.second["Dl"]){
					std::vector<float> Dl = node.second["Dl"].as<std::vector<float>>();
					for (int j=0; j<Dl_order; j++){
						param_Dl(Dl_order*i + j) = Dl[j];
					}
				}

				i++;
			}
			// std::cout<<"YAML_DH letto"<<std::endl;
			// std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		args["par_DYN"] = param_DYN;
		if (Dl_order > 0) args["par_Dl"] = param_Dl;
		update_inertial_REG();
		return 1;
	}

	int Robot::load_par_REG(std::string file){
		// Eigen::VectorXd param_REG;
		casadi::SX param_REG(STD_PAR_LINK*numJoints,1);
		casadi::SX param_Dl(Dl_order*numJoints,1);
		// casadi::SX::zeros(numJoints,1)
		// ----- parsing yaml inertial ----- //
		try {
			// load yaml
			YAML::Node config_file = YAML::LoadFile(file);
			// load inertial
			YAML::Node inertial = config_file["inertial"];
			int i = 0;
			for (const auto& node : inertial) {
				
				if (i==numJoints) break;
				std::string linkName = node.first.as<std::string>();
				
				// standard parameters
				param_REG(STD_PAR_LINK*i) = node.second["mass"].as<double>();
				param_REG(STD_PAR_LINK*i+1) = node.second["m_CoM_x"].as<double>();
				param_REG(STD_PAR_LINK*i+2) = node.second["m_CoM_y"].as<double>();
				param_REG(STD_PAR_LINK*i+3) = node.second["m_CoM_z"].as<double>();
				param_REG(STD_PAR_LINK*i+4) = node.second["Ixx"].as<double>();
				param_REG(STD_PAR_LINK*i+5) = node.second["Ixy"].as<double>();
				param_REG(STD_PAR_LINK*i+6) = node.second["Ixz"].as<double>();
				param_REG(STD_PAR_LINK*i+7) = node.second["Iyy"].as<double>();
				param_REG(STD_PAR_LINK*i+8) = node.second["Iyz"].as<double>();
				param_REG(STD_PAR_LINK*i+9) = node.second["Izz"].as<double>();

				// link friction
				if (node.second["Dl"]){
					std::vector<float> Dl = node.second["Dl"].as<std::vector<float>>();
					for (int j=0; j<Dl_order; j++){
						param_Dl(Dl_order*i + j) = Dl[j];
					}
				}

				i++;
			}
			// std::cout<<"YAML_DH letto"<<std::endl;
			// std::cout<<"\nparam REG \n"<<param_REG<<std::endl;
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		args["par_REG"] = param_REG;
		if (Dl_order > 0) args["par_Dl"] = param_Dl;
		update_inertial_DYN();
		return 1;
	}

	// to modify this file, load_par_elastic
	int Robot::load_par_elastic(std::string file){
		casadi::SX param_K(numElasticJoints*K_order);
		casadi::SX param_D(numElasticJoints*D_order);
		casadi::SX param_Dm(numElasticJoints*Dm_order);
		// ----- parsing yaml elastic ----- //
		try {
			// load yaml
			YAML::Node config_file = YAML::LoadFile(file);
			// parse elastic
			YAML::Node elastic = config_file["elastic"];
			int i = 0;
			for (const auto& node : elastic["joints"]) {
				
				if (i==numElasticJoints) break;
				std::string jointName = node.first.as<std::string>();
				// stiffness
				for (int j=0; j<K_order; j++){
					std::vector<float> K = node.second["K"].as<std::vector<float>>();
					param_K(K_order*i+j) = K[j];
				}
				// coupling friction
				for (int j=0; j<D_order; j++){
					std::vector<float> D = node.second["D"].as<std::vector<float>>();
					param_D(D_order*i + j) = D[j];
				}
				// motor friction
				for (int j=0; j<Dm_order; j++){
					std::vector<float> Dm = node.second["Dm"].as<std::vector<float>>();
					param_Dm(Dm_order*i + j) = Dm[j];
				}

				i++;
			}
			// std::cout<<"YAML_DH letto"<<std::endl;
			// std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		args["par_K"] = param_K;
		args["par_D"] = param_D;
		args["par_Dm"] = param_Dm;
		return 1;
	}

	int Robot::update_inertial_DYN(){
		// not efficient, should be made in casadi
		Eigen::VectorXd param_REG = get_par_REG();
		Eigen::VectorXd param_DYN(STD_PAR_LINK*numJoints);
		for (int i=0; i<numJoints; i++){
			Eigen::VectorXd p_reg = param_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK);
			double mass = p_reg(0);
			Eigen::Vector3d CoM = {p_reg(1)/mass, p_reg(2)/mass, p_reg(3)/mass};
			Eigen::Matrix3d I_tmp = mass * hat(CoM) * hat(CoM).transpose();
			Eigen::Matrix<double, 6, 1> I_tmp_v;
			I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
			Eigen::Matrix<double, 6, 1> I;
			I << p_reg(4), p_reg(5), p_reg(6), p_reg(7), p_reg(8), p_reg(9);
			// Eigen::VectorXd Dl;
			// Dl.resize(Dl_order);
			// for(int j=0; j<Dl_order; j++){
			// 	Dl(j) = param_REG(STD_PAR_LINK*i + STD_PAR_LINK + j);
			// }
			param_DYN.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, CoM, I-I_tmp_v;
		}
		set_par_DYN(param_DYN);
		return 1;
	}

	int Robot::update_inertial_REG(){
		// not efficient, should be made in casadi
		Eigen::VectorXd param_DYN = get_par_DYN();
		// cout<<"param_DYN:"<<endl<<param_DYN<<endl<<endl;
		Eigen::VectorXd param_REG(STD_PAR_LINK*numJoints);
		for (int i=0; i<numJoints; i++){
			Eigen::VectorXd p_dyn = param_DYN.segment(STD_PAR_LINK*i, STD_PAR_LINK);
			// cout<<"p_dyn:"<<endl<<p_dyn<<endl<<endl;
			double mass = p_dyn(0);
			Eigen::Vector3d CoM = {p_dyn(1), p_dyn(2), p_dyn(3)};
			cout<<"CoM:"<<endl<<CoM<<endl<<endl;
			Eigen::Vector3d m_CoM = mass * CoM;
			cout<<"m_CoM:"<<endl<<m_CoM<<endl<<endl;
			Eigen::Matrix3d I_tmp = mass * (hat(CoM) * hat(CoM).transpose());
			cout<<"I_tmp:"<<endl<<I_tmp<<endl<<endl;
			Eigen::Matrix<double, 6, 1> I_tmp_v;
			I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
			cout<<"I_tmp_v:"<<endl<<I_tmp_v<<endl<<endl;
			Eigen::Matrix<double, 6, 1> I;
			I << p_dyn(4), p_dyn(5), p_dyn(6), p_dyn(7), p_dyn(8), p_dyn(9);
			cout<<"I:"<<endl<<I<<endl<<endl;
			cout<<"I+I_tmp_v:"<<endl<<I+I_tmp_v<<endl<<endl;
			// Eigen::VectorXd Dl;
			// Dl.resize(Dl_order);
			// for(int j=0; j<Dl_order; j++){
			// 	Dl(j) = param_DYN(STD_PAR_LINK*i + STD_PAR_LINK + j);
			// }
			param_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, CoM, I+I_tmp_v;
		}
		set_par_REG(param_REG);
		return 1;
	}

	int Robot::add_function(std::string name, casadi::SX expr, std::vector<std::string> f_args, std::string descr){
		// maybe directly model[name] = ...?
		if (model.count(name)){
			// key already exists
			return 0;
		} else {
			model[name] = expr;
			fun_args[name] = f_args;
			fun_descr[name] = descr;

			casadi::SXVector inputs(f_args.size());
			// for (const auto& arg : f_args) {
			// 	inputs.push_back(model[arg]);
			// }
			int i=0;
			for (const auto& arg : f_args) {
				inputs[i] = model[arg];
				i++;
			}

			casadi::Function fun(name+"_fun", inputs, {densify(expr)});
			// cout<<"fun: "<<fun<<endl;
			casadi_fun[name] = fun;
		}

		return 1;
	}

	void Robot::generate_library(const std::string& savePath, const std::string& name_file){
		// Options for c-code auto generation
		casadi::Dict opts = casadi::Dict();
		opts["cpp"] = true;
		opts["with_header"] = true;
		
		// generate functions in c code
		casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(name_file, opts);
		// cout<<"casadi_fun: "<<casadi_fun<<endl;

		for (const auto& f : casadi_fun) {
			myCodeGen.add(f.second);
			// cout<<"f_name: "<<f.first<<endl;
			// cout<<"fun: "<<f.second<<endl<<endl;
		}
		myCodeGen.generate(savePath);
	}


    Config load_config(std::string file) {
        std::unique_ptr<ConfigLoader> loader;

        if (file.ends_with(".urdf")) {
            loader = std::make_unique<URDFConfigLoader>();
        } else if (file.ends_with(".dh")) {
            loader = std::make_unique<DHConfigLoader>();
        } else {
            throw std::runtime_error("Unsupported file type");
        }

        return loader->load(file);
    }
	

	Robot robot_from_file(std::string file, bool compute){
		// create config
		Config conf = load_config(file);
		cout<<"Configuration loaded!"<<endl;
		Robot robot(conf);
		cout<<"Robot created!"<<endl;
		if (compute){
			compute_kinematics(robot);
			cout<<"Kinematics ok!"<<endl;
			compute_dynamics(robot);
			cout<<"Dynamics ok!"<<endl;
			compute_regressors(robot);
			cout<<"Regressors ok!"<<endl;
		}

		// --- load parameters --- //
		robot.load_par_DYN(file);
		if (conf.ELASTIC){
			robot.load_par_elastic(file);
		}

		return robot;
	}



}