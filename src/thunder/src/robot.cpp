#include "../library/robot.h"
#include "../library/kinematics.h"
#include "../library/dynamics.h"
#include "../library/regressors.h"
#include "../library/utils.h"
#include "../library/userDefined.h"

using std::cout;
using std::endl;

namespace thunder_ns{

	// constexpr double MU = 0.02; //pseudo-inverse damping coeff
	// constexpr unsigned int N_PAR_LINK = 10; // number of link+joint parameters
	// constexpr unsigned int NUMBER_FUNCTIONS = 10; // number of generable functions

	Robot::Robot(const std::string config_file){
		load_config(config_file); 	// load config and parameters from yaml file
		initVarsFuns();				// initialize variables
	}

	Robot::Robot(const YAML::Node yaml){
		load_config(yaml); 		// load config and parameters from yaml file
		initVarsFuns();			// initialize variables
	}


	YAML::Node Robot::load_config(std::string config_file){
		try {
			// load yaml
			config_yaml = YAML::LoadFile(config_file);
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while loading YAML: " << e.what() << std::endl;
		}
		parse_config();
		return config_yaml;
	}
	int Robot::load_config(YAML::Node yaml){
		// load yaml
		config_yaml = yaml;
		if (parse_config()) return 1; else return 0;
	}

	int Robot::parse_config() {
		// Local variables for parsing
		int nj;
		int STD_PAR_LINK = Robot::STD_PAR_LINK;
		FrameOffset Base_to_L0;
		FrameOffset Ln_to_EE;

		// ----- Parsing YAML File ----- //
		try {
			YAML::Node config_file = this->config_yaml;

			// ---- Basic Robot Properties ---- //
			nj = config_file["num_joints"].as<int>();
			this->numJoints = nj;
			this->jointsType = config_file["type_joints"].as<std::vector<std::string>>();

			// Friction model order (defaults to 0)
			this->Dl_order = config_file["Dl_order"] ? config_file["Dl_order"].as<int>() : 0;

			// Elastic model properties (defaults to false)
			this->ELASTIC = false;
			this->K_order = 0;
			this->D_order = 0;
			this->Dm_order = 0;
			if (config_file["ELASTIC_MODEL"] && config_file["ELASTIC_MODEL"].as<bool>()) {
				this->ELASTIC = true;
				YAML::Node elastic_node = config_file["elastic"];
				this->K_order = elastic_node["K_order"].as<int>();
				this->D_order = elastic_node["D_order"].as<int>();
				this->Dm_order = elastic_node["Dm_order"].as<int>();
			}
			// identify elastic joints
			this->numElasticJoints = 0;
			this->isElasticJoint.resize(this->numJoints);
			for (int i = 0; i < this->numJoints; i++) {
				if ((this->jointsType[i] == "R_SEA") || (this->jointsType[i] == "P_SEA")) {
					this->isElasticJoint[i] = 1;
					this->numElasticJoints++;
				} else {
					this->isElasticJoint[i] = 0;
				}
			}

			// Denavit-Hartenberg
			YAML::Node kinematics = config_file["kinematics"];
			this->kin_type  = kinematics["type"].as<std::string>();

			if (this->kin_type == "DH"){
				// Save Denavit-Hartenberg parameters
				std::vector<double> dh_vect = kinematics["DH"].as<std::vector<double>>();
				int dh_size = dh_vect.size();
				casadi::SX DHtable_numerical(dh_size, 1);
				for (int i = 0; i < dh_size; i++) {
					DHtable_numerical(i) = dh_vect[i];
				}
				this->args.insert({"DHtable", DHtable_numerical});
				// populate symbolic selectivity flag. We do it here since we branched the kinematics type
				if (kinematics["symb"]) this->symb["par_DHtable"] = kinematics["symb"].as<std::vector<int>>();
				else this->symb["par_DHtable"].assign(dh_size, 0);
				
			}else if(this->kin_type == "URDF"){
				// save path to urdf file
				this->urdf_path = kinematics["urdf_path"].as<std::string>();

				//get base and link frames
				this->base_link = kinematics["base_link"].as<std::string>();
				this->ee_link = kinematics["ee_link"].as<std::string>();
			}else{
				throw std::runtime_error("Unknown kinematics type: " + kin_type);
			}

			// Gravity
			std::vector<double> gravity_vect = {0.0, 0.0, 0.0}; // default no gravity
			if (config_file["gravity"]){
				std::vector<double> gravity_vect = config_file["gravity"]["value"].as<std::vector<double>>();
			}

			// Frame Offsets
			YAML::Node frame_base = config_file["Base_to_L0"];
			Base_to_L0.set_translation(frame_base["tr"].as<std::vector<double>>());
			Base_to_L0.set_ypr(frame_base["ypr"].as<std::vector<double>>());
			Base_to_L0.set_gravity(gravity_vect);

			YAML::Node frame_ee = config_file["Ln_to_EE"];
			Ln_to_EE.set_translation(frame_ee["tr"].as<std::vector<double>>());
			Ln_to_EE.set_ypr(frame_ee["ypr"].as<std::vector<double>>());


			// ---- Symbolic Selectivity Vectors ---- //
			// These vectors determine which parameters are treated as symbolic variables.
			
			// Dynamics (Inertia)
			std::vector<int> par_DYN_symb(STD_PAR_LINK * nj);
			YAML::Node dynamics = config_file["dynamics"];
			int link_index = 0;
			for (const auto& node : dynamics) {
				YAML::Node inertial = node.second["inertial"];
				std::vector<int> link_symb;
				if (inertial["symb"]) {
					link_symb = inertial["symb"].as<std::vector<int>>();
				} else {
					link_symb.assign(STD_PAR_LINK, 0); // Default to non-symbolic
				}
				std::copy(link_symb.begin(), link_symb.end(), par_DYN_symb.begin() + link_index * STD_PAR_LINK);
				link_index++;
			}
			
			// Link Friction
			std::vector<int> par_Dl_symb;
			if (this->Dl_order > 0) {
				par_Dl_symb.resize(this->Dl_order * nj);
				link_index = 0;
				for (const auto& node : dynamics) {
					YAML::Node friction = node.second["friction"];
					std::vector<int> fric_symb;
					if (friction["symb"]) {
						fric_symb = friction["symb"].as<std::vector<int>>();
					} else {
						fric_symb.assign(this->Dl_order, 0);
					}
					std::copy(fric_symb.begin(), fric_symb.end(), par_Dl_symb.begin() + link_index * this->Dl_order);
					link_index++;
				}
			}
			
			// Elasticity (Stiffness, Damping, etc.)
			std::vector<int> par_K_symb, par_D_symb, par_Dm_symb, par_Mm_symb;
			if (this->ELASTIC) {
				YAML::Node elastic_joints = config_file["elastic"]["joints"];
				int i = 0;
				for (const auto& node : elastic_joints) {
					if (i==numElasticJoints) break; // break if nore joints defined
					// Helper lambda to parse a symbolic vector
					auto parse_symb_vector = [&](const std::string& key, int order) {
						std::vector<int> vec;
						if (node.second[key]){
							vec = node.second[key].as<std::vector<int>>();
							vec.resize(order);
						} else vec.assign(order, 0);
						return vec;
					};

					std::vector<int> K_symb = parse_symb_vector("K_symb", this->K_order);
					if (K_order > 0) par_K_symb.insert(par_K_symb.end(), K_symb.begin(), K_symb.end());

					std::vector<int> D_symb = parse_symb_vector("D_symb", this->D_order);
					if (D_order > 0) par_D_symb.insert(par_D_symb.end(), D_symb.begin(), D_symb.end());

					std::vector<int> Dm_symb = parse_symb_vector("Dm_symb", this->Dm_order);
					if (Dm_order > 0) par_Dm_symb.insert(par_Dm_symb.end(), Dm_symb.begin(), Dm_symb.end());

					int Mm_symb = node.second["Mm_symb"] ? node.second["Mm_symb"].as<int>() : 0;
					par_Mm_symb.push_back(Mm_symb);

					i++;
				}
			}

			// ---- Populate the Symbolic Selectivity Map (`symb`) ---- //
			this->symb["par_DYN"] = par_DYN_symb;
			this->symb["par_Dl"] = par_Dl_symb;
			this->symb["par_K"] = par_K_symb;
			this->symb["par_D"] = par_D_symb;
			this->symb["par_Dm"] = par_Dm_symb;
			this->symb["par_Mm"] = par_Mm_symb;



			if (frame_base["symb"]) this->symb["par_world2L0"] = frame_base["symb"].as<std::vector<int>>();
			else this->symb["par_world2L0"].assign(6, 0);
			
			if (frame_ee["symb"]) this->symb["par_Ln2EE"] = frame_ee["symb"].as<std::vector<int>>();
			else this->symb["par_Ln2EE"].assign(6, 0);

			if (config_file["gravity"]["symb"]) this->symb["par_gravity"] = config_file["gravity"]["symb"].as<std::vector<int>>();
			else this->symb["par_gravity"].assign(3, 0);


			// ---- Finalize Robot State and CasADi Model ---- //
			if (this->jointsType.size() != this->numJoints) {
				throw std::runtime_error("Mismatch between 'num_joints' and the size of 'type_joints' vector.");
			}
			
			// Define symbolic variables for the model
			this->model = {
				{"par_DHtable", casadi::SX::sym("DHtable", this->numJoints * 4)},
				{"par_world2L0", casadi::SX::sym("world2L0", 6, 1)},
				{"par_Ln2EE", casadi::SX::sym("Ln2EE", 6, 1)},
				{"par_gravity", casadi::SX::sym("gravity", 3, 1)}
			};

			// Create CasADi numerical vectors from parsed data
			casadi::SX world2L0_numerical(6, 1);
			casadi::SX Ln2EE_numerical(6, 1);
			casadi::SX gravity_numerical(3, 1);
			for (int i = 0; i < 3; i++) {
				world2L0_numerical(i) = Base_to_L0.translation[i];
				world2L0_numerical(i + 3) = Base_to_L0.ypr[i];
				Ln2EE_numerical(i) = Ln_to_EE.translation[i];
				Ln2EE_numerical(i + 3) = Ln_to_EE.ypr[i];
				gravity_numerical(i) = Base_to_L0.gravity[i];
			}

			// Populate the 'args' map with numerical values
			this->args.insert({"par_world2L0", world2L0_numerical});
			this->args.insert({"par_Ln2EE", Ln2EE_numerical});
			this->args.insert({"par_gravity", gravity_numerical});

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return 0; // Indicate failure
		}
		return 1; // Indicate success
	}

	void Robot::initVarsFuns(){
		// if (_DHtable_.rows() != numJoints || _DHtable_.columns() != 4){
		// 	throw std::runtime_error("DHTemplate: Error size of DH table");
		// }
		if ((int)jointsType.size()!=numJoints){
			throw std::runtime_error("DHFwkinJoints: Error size of joints string");
		}

		casadi::SX q = casadi::SX::sym("q", numJoints,1);
		casadi::SX dq = casadi::SX::sym("dq", numJoints,1);
		casadi::SX ddq = casadi::SX::sym("ddq", numJoints,1);
		casadi::SX d3q = casadi::SX::sym("d3q", numJoints,1);
		casadi::SX d4q = casadi::SX::sym("d4q", numJoints,1);
		casadi::SX dqr = casadi::SX::sym("dqr", numJoints,1);
		casadi::SX ddqr = casadi::SX::sym("ddqr", numJoints,1);
		casadi::SX x = casadi::SX::sym("x", numElasticJoints,1);
		casadi::SX dx = casadi::SX::sym("dx", numElasticJoints,1);
		casadi::SX ddx = casadi::SX::sym("ddx", numElasticJoints,1);
		casadi::SX ddxr = casadi::SX::sym("ddxr", numElasticJoints,1);
		// _par_KIN_ = casadi::SX::sym("par_DYN", N_PAR_KIN,1);
		casadi::SX par_DYN = casadi::SX::sym("par_DYN", STD_PAR_LINK*numJoints,1);
		casadi::SX par_REG = casadi::SX::sym("par_REG", STD_PAR_LINK*numJoints,1);
		casadi::SX par_Dl = casadi::SX::sym("par_Dl", numJoints*Dl_order,1);
		casadi::SX par_K = casadi::SX::sym("par_K", numElasticJoints*K_order,1);
		casadi::SX par_D = casadi::SX::sym("par_D", numElasticJoints*D_order,1);
		casadi::SX par_Dm = casadi::SX::sym("par_Dm", numElasticJoints*Dm_order,1);
		casadi::SX par_Mm = casadi::SX::sym("par_Mm", numElasticJoints,1);
		casadi::SX w = casadi::SX::sym("w", 6,1);
		
		// model update
		model.insert({"q", q});
		model.insert({"dq", dq});
		model.insert({"ddq", ddq});
		model.insert({"d3q", d3q});
		model.insert({"d4q", d4q});
		model.insert({"dqr", dqr});
		model.insert({"ddqr", ddqr});
		model.insert({"w", w});
		// model.insert({"par_KIN", _par_KIN_});
		model.insert({"par_DYN", par_DYN});
		model.insert({"par_REG", par_REG});
		if (Dl_order > 0){
			model.insert({"par_Dl", par_Dl});
		}
		if (ELASTIC){
			model.insert({"x", x});
			model.insert({"dx", dx});
			model.insert({"ddx", ddx});
			model.insert({"ddxr", ddxr});
			model.insert({"par_K", par_K});
			model.insert({"par_D", par_D});
			model.insert({"par_Dm", par_Dm});
			model.insert({"par_Mm", par_Mm});
		}

		args.insert({"q", casadi::SX::zeros(numJoints,1)});
		args.insert({"dq", casadi::SX::zeros(numJoints,1)});
		args.insert({"ddq", casadi::SX::zeros(numJoints,1)});
		args.insert({"d3q", casadi::SX::zeros(numJoints,1)});
		args.insert({"d4q", casadi::SX::zeros(numJoints,1)});
		args.insert({"dqr", casadi::SX::zeros(numJoints,1)});
		args.insert({"ddqr", casadi::SX::zeros(numJoints,1)});
		args.insert({"w", casadi::SX::zeros(6)});
		// args.insert({"par_KIN", par_KIN});
		args.insert({"par_DYN", casadi::SX::zeros(STD_PAR_LINK*numJoints,1)});
		args.insert({"par_REG", casadi::SX::zeros(STD_PAR_LINK*numJoints,1)});
		if (Dl_order > 0){
			args.insert({"par_Dl", casadi::SX::zeros(Dl_order*numJoints,1)});
		}
		if (ELASTIC){
			args.insert({"x", casadi::SX::zeros(numElasticJoints,1)});
			args.insert({"dx", casadi::SX::zeros(numElasticJoints,1)});
			args.insert({"ddx", casadi::SX::zeros(numElasticJoints,1)});
			args.insert({"ddxr", casadi::SX::zeros(numElasticJoints,1)});
			args.insert({"par_K", casadi::SX::zeros(K_order*numElasticJoints,1)});
			args.insert({"par_D", casadi::SX::zeros(D_order*numElasticJoints,1)});
			args.insert({"par_Dm", casadi::SX::zeros(Dm_order*numElasticJoints,1)});
			args.insert({"par_Mm", casadi::SX::zeros(numElasticJoints,1)});
		}
		// args.insert({"par_ELA", casadi::SX::zeros(numParELA)});

		// // - symbolics - //
		// symb.insert({"q", casadi::SX::zeros(numElasticJoints)})
		std::vector<int> q_symb(numJoints, 1);
		std::vector<int> dq_symb(numJoints, 1);
		std::vector<int> ddq_symb(numJoints, 1);
		std::vector<int> d3q_symb(numJoints, 1);
		std::vector<int> d4q_symb(numJoints, 1);
		std::vector<int> dqr_symb(numJoints, 1);
		std::vector<int> ddqr_symb(numJoints, 1);
		std::vector<int> w_symb(6, 1);
		symb.insert({"q", q_symb});
		symb.insert({"dq", dq_symb});
		symb.insert({"ddq", ddq_symb});
		symb.insert({"d3q", d3q_symb});
		symb.insert({"d4q", d4q_symb});
		symb.insert({"dqr", dqr_symb});
		symb.insert({"ddqr", ddqr_symb});
		symb.insert({"w", w_symb});
		if (ELASTIC){
			std::vector<int> x_symb(numElasticJoints, 1);
			std::vector<int> dx_symb(numElasticJoints, 1);
			std::vector<int> ddx_symb(numElasticJoints, 1);
			std::vector<int> ddxr_symb(numElasticJoints, 1);
			symb.insert({"x", x_symb});
			symb.insert({"dx", dx_symb});
			symb.insert({"ddx", ddx_symb});
			symb.insert({"ddxr", ddxr_symb});
		}

	}

	Eigen::MatrixXd Robot::get(std::string name){
		std::vector<casadi::SX> result;
		// Eigen::MatrixXd result_num;
		if (fun_args.count(name)){
    		// key exists
			// cout<<"name: "<<name<<endl;
			Eigen::MatrixXd result_num(model[name].rows(), model[name].columns());
			// cout<<"output_size: "<<result_num.size()<<endl;
			auto f_args = fun_args[name];
			int sz = f_args.size();
			// cout<<"f_args:"<<f_args<<", size: "<<sz<<endl;
			casadi::SXVector inputs(sz);
			int i=0;
			for (const auto& arg : f_args) {
				inputs[i] = args[arg];
				i++;
			}
			// cout<<"args: "<<inputs<<endl;
			// casadi::Function fun = casadi_fun[name];
			// cout<<"fun: "<<fun<<endl;
			casadi_fun[name].call(inputs, result);
			// cout<<"result: "<<result<<endl;
			std::vector<casadi::SXElem> res_elements = result[0].get_elements();
			// cout<<"elements: "<<res_elements<<endl;
			std::transform(res_elements.begin(), res_elements.end(), result_num.data(), mapFunction);
			// cout<<"result: "<<result_num<<endl;
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

	int Robot::set_ddx(Eigen::VectorXd value){
		casadi::SX& ddx = args["ddx"];
		if (value.size() == numElasticJoints){
			for (int i=0; i<numElasticJoints; i++){
				ddx(i) = value(i);
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


	Eigen::VectorXd Robot::get_arg(std::string par){
		const casadi::SX& par_casadi = args[par];
		int numPar = par_casadi.size1() * par_casadi.size2();
		Eigen::VectorXd param(numPar);
		std::vector<casadi::SXElem> res_elements = par_casadi.get_elements();
		std::transform(res_elements.begin(), res_elements.end(), param.data(), mapFunction);
		return param;
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

	// casadi::SX Robot::get_DHTable(){
	// 	return _DHtable_;
	// }

	// FrameOffset Robot::get_world2L0(){
	// 	return _world2L0_;
	// }

	// FrameOffset Robot::get_Ln2EE(){
	// 	return _Ln2EE_;
	// }

	int Robot::load_conf_par(std::string file, bool update_REG){
		// Eigen::VectorXd param_DYN;
		casadi::SX param_DYN(STD_PAR_LINK*numJoints,1);
		casadi::SX param_Dl(Dl_order*numJoints,1);
		casadi::SX param_K(numElasticJoints*K_order,1);
		casadi::SX param_D(numElasticJoints*D_order,1);
		casadi::SX param_Dm(numElasticJoints*Dm_order,1);
		casadi::SX param_Mm(numElasticJoints,1);
		// ----- parsing yaml inertial ----- //
		try {
			// load yaml
			YAML::Node config_file = YAML::LoadFile(file);
			// load inertial
			YAML::Node dynamics = config_file["dynamics"];
			int i = 0;
			for (const auto& node : dynamics) {
				YAML::Node inertial = node.second["inertial"];
				
				if (i==numJoints) break;
				std::string linkName = node.first.as<std::string>();

				// standard parameters
				param_DYN(STD_PAR_LINK*i) = inertial["mass"].as<double>();
				param_DYN(STD_PAR_LINK*i+1) = inertial["CoM_x"].as<double>();
				param_DYN(STD_PAR_LINK*i+2) = inertial["CoM_y"].as<double>();
				param_DYN(STD_PAR_LINK*i+3) = inertial["CoM_z"].as<double>();
				param_DYN(STD_PAR_LINK*i+4) = inertial["Ixx"].as<double>();
				param_DYN(STD_PAR_LINK*i+5) = inertial["Ixy"].as<double>();
				param_DYN(STD_PAR_LINK*i+6) = inertial["Ixz"].as<double>();
				param_DYN(STD_PAR_LINK*i+7) = inertial["Iyy"].as<double>();
				param_DYN(STD_PAR_LINK*i+8) = inertial["Iyz"].as<double>();
				param_DYN(STD_PAR_LINK*i+9) = inertial["Izz"].as<double>();
				
				// link friction
				if (node.second["friction"]){
					std::vector<double> Dl = node.second["friction"]["Dl"].as<std::vector<double>>();
					for (int j=0; j<Dl_order; j++){
						param_Dl(Dl_order*i + j) = Dl[j];
					}
				}

				i++;
			}

			// ----- parsing yaml elastic ----- //
			if ((ELASTIC) && (config_file["elastic"])){
				YAML::Node elastic = config_file["elastic"];
				i = 0;
				for (const auto& node : elastic["joints"]) {
					
					if (i==numElasticJoints) break;
					std::string jointName = node.first.as<std::string>();
					// stiffness
					if (K_order > 0){
						std::vector<double> K = node.second["K"].as<std::vector<double>>();
						for (int j=0; j<K_order; j++) param_K(K_order*i+j) = K[j];
					}
					// coupling friction
					if (D_order > 0){
						std::vector<double> D = node.second["D"].as<std::vector<double>>();
						for (int j=0; j<D_order; j++) param_D(D_order*i + j) = D[j];
					}
					// motor friction
					if (Dm_order > 0){
						std::vector<double> Dm = node.second["Dm"].as<std::vector<double>>();
						for (int j=0; j<Dm_order; j++) param_Dm(Dm_order*i + j) = Dm[j];
					}
					// motor inertia
					param_Mm(i) = node.second["Mm"].as<double>();

					i++;
				}
				// std::cout<<"YAML_DH letto"<<std::endl;
				// std::cout<<"\nparam DYN \n"<<param_DYN<<std::endl;
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		args["par_DYN"] = param_DYN;
		if (Dl_order > 0) args["par_Dl"] = param_Dl;
		if (K_order>0) args["par_K"] = param_K;
		if (D_order>0) args["par_D"] = param_D;
		if (Dm_order>0) args["par_Dm"] = param_Dm;
		if (ELASTIC) args["par_Mm"] = param_Mm;
		if (update_REG) update_inertial_REG();
		return 1;
	}

	casadi::SX Robot::load_par_REG(std::string file, bool update_DYN){
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
					std::vector<double> Dl = node.second["Dl"].as<std::vector<double>>();
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
		if (update_DYN) update_inertial_DYN();
		return param_REG;
	}

	int Robot::load_par(std::string par_file, std::vector<std::string> par_list){
		try {
			// load yaml
			YAML::Node yamlFile = YAML::LoadFile(par_file);
			if (par_list.size() == 0){
				for (const auto& node : yamlFile){
					std::vector<double> par_vect = node.second.as<std::vector<double>>();
					casadi::SX& par = args[node.first.as<std::string>()];
					for (int i=0; i<par_vect.size(); i++){
						par(i) = par_vect[i];
					}
				}
			} else {
				for (std::string par_str : par_list){
					std::vector<double> par_vect = yamlFile[par_str].as<std::vector<double>>();
					casadi::SX& par = args[par_str];
					for (int i=0; i<par_vect.size(); i++){
						par(i) = par_vect[i];
					}
				}
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while loading parameters: " << e.what() << std::endl;
			return 0;
		}
		return 1;
	}

	// to modify this file, load_par_elastic
	// int Robot::load_par_elastic(std::string file){		
	// 	return 1;
	// }

	void Robot::update_conf(){
		// to do!
	}

	int Robot::save_conf(std::string par_file){
		update_conf();
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			emitter << config_yaml << YAML::Newline;

			std::ofstream fout(par_file);
			fout << emitter.c_str();
			fout.close();
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while generating YAML: " << e.what() << std::endl;
			return 0;
		}
		return 1;
	}

	int Robot::save_par_REG(std::string par_file){
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			YAML::Node yamlFile;
			YAML::Node dynamicsNode;

			emitter << YAML::Comment(
				"Inertial parameters referred to Denavit-Hartenberg parametrization to use with regressor matrix\n");
			emitter << YAML::Newline;

			casadi::SX par_REG = args["par_REG"];
			for (int i=0;  i<numJoints;  i++) {

				// LinkProp link = links_prop_[i];    
				YAML::Node linkNode;
				std::string nodeName;
				YAML::Node linkInertia;
				YAML::Node linkFric;

				nodeName = "link" + std::to_string(i+1);
				linkInertia["mass"] = (double)par_REG(i*STD_PAR_LINK+0);
				linkInertia["m_CoM_x"] = (double)par_REG(i*STD_PAR_LINK+1);
				linkInertia["m_CoM_y"] = (double)par_REG(i*STD_PAR_LINK+2);
				linkInertia["m_CoM_z"] = (double)par_REG(i*STD_PAR_LINK+3);
				linkInertia["Ixx"] = (double)par_REG(i*STD_PAR_LINK+4);
				linkInertia["Ixy"] = (double)par_REG(i*STD_PAR_LINK+5);
				linkInertia["Ixz"] = (double)par_REG(i*STD_PAR_LINK+6);
				linkInertia["Iyy"] = (double)par_REG(i*STD_PAR_LINK+7);
				linkInertia["Iyz"] = (double)par_REG(i*STD_PAR_LINK+8);
				linkInertia["Izz"] = (double)par_REG(i*STD_PAR_LINK+9);
				// link friction
				// linkFric["Dl"] = link.Dl;

				linkNode["inertial"] = linkInertia;
				// linkNode["friction"] = linkFric;
				dynamicsNode[nodeName] = linkNode;
			}

			yamlFile["dynamics"] = dynamicsNode;
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

	int Robot::save_par(std::string par_file, std::vector<std::string> par_list){
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			YAML::Node yamlFile;

			for (auto& par : par_list){
				// YAML::Node par_node;
				// par_node[par] = args[par];
				// emitter << par_node << YAML::Newline;
				// yamlFile[par] = args[par];

				// std::cout << par + "_sx: " << args[par] << endl;
				Eigen::VectorXd vect_eig = get_arg(par);
				// std::cout << par + "_eig: " << vect_eig << endl;
				std::vector<double> vect_std(vect_eig.data(), vect_eig.data() + vect_eig.rows() * vect_eig.cols());
				yamlFile[par] = vect_std;
				
			}
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

	int Robot::update_inertial_DYN(){
		// not efficient, should be made in casadi
		Eigen::VectorXd param_REG = get_par_REG();
		Eigen::VectorXd param_DYN(STD_PAR_LINK*numJoints);
		for (int i=0; i<numJoints; i++){
			Eigen::VectorXd p_reg = param_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK);
			double mass = p_reg(0);
			Eigen::Vector3d CoM = {p_reg(1)/mass, p_reg(2)/mass, p_reg(3)/mass};
			Eigen::Matrix3d I_tmp = mass * (hat(CoM).transpose() * hat(CoM));
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
			// cout<<"CoM:"<<endl<<CoM<<endl<<endl;
			Eigen::Vector3d m_CoM = mass * CoM;
			// cout<<"m_CoM:"<<endl<<m_CoM<<endl<<endl;
			Eigen::Matrix3d I_tmp = mass * (hat(CoM).transpose() * hat(CoM));
			// cout<<"I_tmp:"<<endl<<I_tmp<<endl<<endl;
			Eigen::Matrix<double, 6, 1> I_tmp_v;
			I_tmp_v << I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2);
			// cout<<"I_tmp_v:"<<endl<<I_tmp_v<<endl<<endl;
			Eigen::Matrix<double, 6, 1> I;
			I << p_dyn(4), p_dyn(5), p_dyn(6), p_dyn(7), p_dyn(8), p_dyn(9);
			// cout<<"I:"<<endl<<I<<endl<<endl;
			// cout<<"I+I_tmp_v:"<<endl<<I+I_tmp_v<<endl<<endl;
			// Eigen::VectorXd Dl;
			// Dl.resize(Dl_order);
			// for(int j=0; j<Dl_order; j++){
			// 	Dl(j) = param_DYN(STD_PAR_LINK*i + STD_PAR_LINK + j);
			// }
			param_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, m_CoM, I+I_tmp_v;
		}
		set_par_REG(param_REG);
		return 1;
	}

	int Robot::add_function(std::string f_name, casadi::SX expr, std::vector<std::string> f_args, std::string descr){
		// maybe directly model[f_name] = ...?
		if (model.count(f_name)){
			// key already exists
			return 0;
		} else {
			model[f_name] = expr;
			fun_args[f_name] = f_args;
			fun_descr[f_name] = descr;

			casadi::SXVector inputs(f_args.size());
			// for (const auto& arg : f_args) {
			// 	inputs.push_back(model[arg]);
			// }
			int arg_index=0;
			for (const auto& arg : f_args) {
				// - resize parameters - //
				// int sz_original = args[arg].size();
				// std::cout << "fun: " << f_name << std::endl;
				// std::cout << "arg: " << arg << std::endl;
				std::vector<int>& symb_flag = symb[arg];
				std::vector<casadi::SX> par_symb;
				casadi::SX& par_model = model[arg];
				// cout << "model[arg]: " << par_model << endl;
				// cout << "symb_flag: " << symb_flag << endl;
				int sz_original = par_model.size().first;
				// casadi::SX par_model_new = casadi::SX::zeros(sz_original);
				// casadi::SX new_par(sz_original,1);
				int sz = 0;
				for (int i=0; i<sz_original; i++){
					if (symb_flag[i]){
						par_symb.push_back(par_model(i));
						// par_model_new(sz) = par_model(i);
						sz++;
					}
				}
				casadi::SX par_symb_new = casadi::SX::vertcat(par_symb);
				// casadi::Slice newsize(0,sz);
				// par_model = par_model(newsize,1);
				// par_arg = par_arg(newsize,1);
				// par_model_new.resize(sz,1);
				// cout << "par_model_new: " << par_symb_new << endl;

				inputs[arg_index] = par_symb_new;
				// inputs[arg_index] = model[arg](newsize, 0);
				arg_index++;
			}

			casadi::Function fun(robotName+"_"+f_name+"_fun", inputs, {densify(expr)});
			// cout<<"fun: "<<fun<<endl;
			casadi_fun[f_name] = fun;
		}

		return 1;
	}

	void Robot::generate_library(const std::string& savePath, const std::string& name_file, const bool SAVE_CASADI){
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

		if(SAVE_CASADI){
			// Save CasADi functions
			for (const auto& f : casadi_fun) {
				std::string function_file = savePath + "/" + f.first + ".casadi";
				// std::ofstream file(function_file, std::ios::binary);
				f.second.save(function_file);
				// file.close();
			}
		}
	}

	int Robot::subs_symb_par(std::string par){
		std::vector<int> symbolic = symb[par];
		// cout << "par: " << par << endl;
		// cout << "symbolic: " << symbolic << endl;
		// cout << "model: " << model[par] << endl;
		// cout << "args: " << args[par] << endl;
		// cout << "size: " << symbolic.size() << endl;

		for (int i=0; i<symbolic.size(); i++){
			if (symbolic[i] == 0){
				model[par](i) = (double)args[par](i);
			}
		}

		// cout<<"model: "<< model[par] << endl;
		// cout<<"arg: "<< args[par] << endl;
		return 1;
	}

	int Robot::init_symb_parameters(){
		// - substitute non-symbolic variables with numbers - //
		for (auto par : symb){
			subs_symb_par(par.first);
		}
		return 1;
	}

	std::vector<std::string> Robot::obtain_symb_parameters(std::vector<std::string> par_sure, std::vector<std::string> par_possible){
		// - obtain parameters that have symbolic values inside - //
		std::vector<std::string> arg_list = par_sure;
		for (auto& par : par_possible){
			bool is_symb = false;
			for (int v : symb[par]){
				if (v) is_symb = true;
			}
			if (is_symb){
				arg_list.push_back(par);
			}
		}
		return arg_list;
	}

	int Robot::update_symb_parameters(){
		// - resize parameter variables to right dimension - //
		for (auto par_symb : symb){
			int sz_original = par_symb.second.size();
			std::string par_name = par_symb.first;
			casadi::SX& par_model = model[par_name];
			casadi::SX& par_arg = args[par_name];
			// casadi::SX new_par(sz_original,1);
			int sz = 0;
			for (int i=0; i<sz_original; i++){
				if (par_symb.second[i]){
					par_model(sz) = par_model(i);
					par_arg(sz) = par_arg(i);
					sz++;
				}
			}
			par_model.resize(sz,1);
			par_arg.resize(sz,1);
			// cout << "par_model: " << par_model << endl;
			// cout << "par_arg: " << par_arg << endl;
		}
		return 1;
	}


	Robot robot_from_file(std::string robot_name, std::string file, bool compute){
		// create config
		// Config conf = load_config(file);
		bool advanced = true;
		Robot robot(file);
		robot.robotName = robot_name;
		// --- load parameters --- //
		robot.load_conf_par(file);
		cout<<"Configuration loaded!"<<endl;
		// --- compute functions --- //
		if (compute){
			// - symbolic selectivity - //
			robot.init_symb_parameters();
			cout<<"symbolic parameters ok!"<<endl;

			// - compute functions - //
			compute_kinematics(robot, advanced);
			cout<<"Kinematics ok!"<<endl;
			compute_dynamics(robot, advanced);
			cout<<"Dynamics ok!"<<endl;
			// compute_regressors(robot);
			// cout<<"Regressors ok!"<<endl;
			// compute_userDefined(robot);
			// cout<<"User defined functions ok!"<<endl;

			// - update parameters - //
			robot.update_symb_parameters();
			cout<<"symbolic parameters ready!"<<endl;
		}
		
		cout<<"Robot created!"<<endl;

		return robot;
	}

}