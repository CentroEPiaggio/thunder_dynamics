#include "../library/robot.h"
#include "../library/kinematics.h"
#include "../library/dynamics.h"
#include "../library/regressors.h"
#include "../library/utils.h"
#include "../library/userDefined.h"

using std::cout;
using std::endl;
using casadi::SX;

namespace thunder_ns{

	Robot::Robot(const string config_file){
		load_config(config_file); 	// load config and parameters from yaml file
	}

	Robot::Robot(const YAML::Node yaml){
		load_config(yaml); 		// load config and parameters from yaml file
	}

	YAML::Node Robot::load_config(string config_file){
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


			// --- Basic Robot properties --- //
			nj = config_file["num_joints"].as<int>();
			this->numJoints = nj;
			this->jointsType = config_file["type_joints"].as<vector<string>>();
			if (this->jointsType.size() != this->numJoints) {
				throw std::runtime_error("Mismatch between 'num_joints' and the size of 'type_joints' vector.");
			}
			// --- Elastic model properties (defaults to false) --- //
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
			// - identify elastic joints - //
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


			// --- Variables --- //
			// - Normal joints - //
			add_variable("q", SX::sym("q",numJoints,1), vector<double>(numJoints,0), {1}, "Configuration", true);
			add_variable("dq", SX::sym("dq",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity", true);
			add_variable("ddq", SX::sym("ddq",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration", true);
			add_variable("dqr", SX::sym("dqr",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity reference", true);
			add_variable("ddqr", SX::sym("ddqr",numJoints,1), vector<double>(numJoints,0), {1}, "Acceleration reference", true);
			add_variable("d3q", SX::sym("d3q",numJoints,1), vector<double>(numJoints,0), {1}, "Jerk", true);
			add_variable("d4q", SX::sym("d4q",numJoints,1), vector<double>(numJoints,0), {1}, "Snap", true);
			// - Elastic joints - //
			add_variable("x", SX::sym("x",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor angle", true);
			add_variable("dx", SX::sym("dx",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor velocity", true);
			add_variable("ddx", SX::sym("ddx",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor acceleration", true);
			add_variable("ddxr", SX::sym("ddxr",numElasticJoints,1), vector<double>(numElasticJoints,0), {1}, "Motor acceleration reference", true);
			// - Regressors - //
			add_variable("w", SX::sym("w",6,1), vector<double>(6,1), {1}, "Wrench", true);


			// --- Denavit-Hartenberg --- //
			YAML::Node kinematics = config_file["kinematics"];
			vector<double> dh_num = kinematics["DH"].as<vector<double>>();
			int dh_size = dh_num.size();
			// - Symbolic selectivity - //
			vector<short> dh_isSymb;
			if (kinematics["symb"]) dh_isSymb = kinematics["symb"].as<vector<short>>();
			else dh_isSymb.assign(dh_size, 0);
			// - Model - //
			SX dh_symb = SX::sym("DHtable", this->numJoints * 4);
			// - Add to parameters - //
			add_parameter("par_DHtable", dh_symb, dh_num, dh_isSymb, "DH parameters", true);


			// --- Gravity --- //
			vector<double> gravity_num = {0.0, 0.0, 0.0}; // default no gravity
			if (config_file["gravity"]){
				gravity_num = config_file["gravity"]["value"].as<vector<double>>();
			}
			// - Symbolic selectivity - //
			vector<short> gravity_isSymb;
			if (config_file["gravity"]["symb"]) gravity_isSymb = config_file["gravity"]["symb"].as<vector<short>>();
			else gravity_isSymb.assign(3, 0);
			// - Model - //
			SX gravity_symb = SX::sym("gravity", 3);
			// - Add to parameters - //
			add_parameter("par_gravity", gravity_symb, gravity_num, gravity_isSymb, "Gravity on world frame", true);


			// --- Frame Offsets --- //
			YAML::Node frame_base = config_file["Base_to_L0"];
			YAML::Node frame_ee = config_file["Ln_to_EE"];
			Base_to_L0.set_translation(frame_base["tr"].as<vector<double>>());
			Base_to_L0.set_ypr(frame_base["ypr"].as<vector<double>>());
			Ln_to_EE.set_translation(frame_ee["tr"].as<vector<double>>());
			Ln_to_EE.set_ypr(frame_ee["ypr"].as<vector<double>>());
			vector<double> world2L0_num(6, 0);
			vector<double> Ln2EE_num(6, 0);
			for (int i = 0; i < 3; i++) {
				world2L0_num[i] = Base_to_L0.translation[i];
				world2L0_num[i + 3] = Base_to_L0.ypr[i];
				Ln2EE_num[i] = Ln_to_EE.translation[i];
				Ln2EE_num[i + 3] = Ln_to_EE.ypr[i];
			}
			// - Symbolic selectivity - //
			vector<short> world2L0_isSymb;
			vector<short> Ln2EE_isSymb;
			if (frame_base["symb"]) world2L0_isSymb = frame_base["symb"].as<vector<short>>();
			else world2L0_isSymb.assign(6, 0);
			if (frame_ee["symb"]) Ln2EE_isSymb = frame_ee["symb"].as<vector<short>>();
			else Ln2EE_isSymb.assign(6, 0);
			// - Model - //
			SX world2L0_symb = SX::sym("world2L0", 6);
			SX Ln2EE_symb = SX::sym("Ln2EE", 6);
			// - Numeric - //
			add_parameter("par_world2L0", world2L0_symb, world2L0_num, world2L0_isSymb, "World to base frame", true);
			add_parameter("par_Ln2EE", Ln2EE_symb, Ln2EE_num, Ln2EE_isSymb, "Last link to end-effector frame", true);
			

			// --- Dynamics (Inertial) --- //
			vector<double> par_DYN_num(STD_PAR_LINK*numJoints,0);
			vector<double> par_REG_num(STD_PAR_LINK*numJoints,0);
			vector<short> par_DYN_isSymb(STD_PAR_LINK * nj);
			YAML::Node dynamics = config_file["dynamics"];
			int idx = 0;
			for (const auto& node : dynamics) {
				YAML::Node inertial = node.second["inertial"];

				if (idx==numJoints) break;	// termination on link number
				string linkName = node.first.as<string>();
				
				// - Numeric - //
				par_DYN_num[STD_PAR_LINK*idx] = inertial["mass"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+1] = inertial["CoM_x"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+2] = inertial["CoM_y"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+3] = inertial["CoM_z"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+4] = inertial["Ixx"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+5] = inertial["Ixy"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+6] = inertial["Ixz"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+7] = inertial["Iyy"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+8] = inertial["Iyz"].as<double>();
				par_DYN_num[STD_PAR_LINK*idx+9] = inertial["Izz"].as<double>();

				// - Symbolic selectivity - //
				vector<short> link_isSymb;
				if (inertial["symb"]) {
					link_isSymb = inertial["symb"].as<vector<short>>();
				} else {
					link_isSymb.assign(STD_PAR_LINK, 0); // Default to non-symbolic
				}
				std::copy(link_isSymb.begin(), link_isSymb.end(), par_DYN_isSymb.begin() + idx * STD_PAR_LINK);
				idx++;
			}
			// - Model - //
			SX par_DYN_symb = SX::sym("par_DYN", STD_PAR_LINK*numJoints,1);
			SX par_REG_symb = SX::sym("par_REG", STD_PAR_LINK*numJoints,1);
			// - Add to parameters - //
			add_parameter("par_DYN", par_DYN_symb, par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			vector<short> par_REG_isSymb = par_DYN_isSymb;
			add_parameter("par_REG", par_REG_symb, par_REG_num, par_REG_isSymb, "Dynamic parameters for regressor", true);
			update_inertial_REG();


			// --- Link friction --- //
			this->Dl_order = config_file["Dl_order"] ? config_file["Dl_order"].as<int>() : 0;	// defaults to 0
			vector<double> par_Dl_num;
			vector<short> par_Dl_isSymb;
			if (this->Dl_order > 0) {
				par_Dl_num.resize(this->Dl_order * nj);
				par_Dl_isSymb.resize(this->Dl_order * nj);
				idx = 0;
				for (const auto& node : dynamics) {
					// - Numeric - //
					if (node.second["friction"]){
						vector<double> Dl = node.second["friction"]["Dl"].as<vector<double>>();
						for (int j=0; j<Dl_order; j++){
							par_Dl_num[Dl_order*idx + j] = Dl[j];
						}
					}

					// - Symbolic selectivity - //
					YAML::Node friction = node.second["friction"];
					vector<int> fric_isSymb;
					if (friction["symb"]) {
						fric_isSymb = friction["symb"].as<vector<int>>();
					} else {
						fric_isSymb.assign(this->Dl_order, 0);
					}
					std::copy(fric_isSymb.begin(), fric_isSymb.end(), par_Dl_isSymb.begin() + idx * this->Dl_order);
					idx++;
				}
				// - Model - //
				SX par_Dl_symb = SX::sym("par_Dl", numJoints*Dl_order,1);
				// - Add to parameters - //
				add_parameter("par_Dl", par_Dl_symb, par_Dl_num, par_Dl_isSymb, "Link friction parameters", true);
			}

			// - Elastic joints parameters - //
			vector<double> par_K_num, par_D_num, par_Dm_num, par_Mm_num;
			vector<short> par_K_isSymb, par_D_isSymb, par_Dm_isSymb, par_Mm_isSymb;
			if (this->ELASTIC) {
				par_K_num.resize(numElasticJoints*K_order);
				par_D_num.resize(numElasticJoints*D_order);
				par_Dm_num.resize(numElasticJoints*Dm_order);
				par_Mm_num.resize(numElasticJoints);
				YAML::Node elastic_joints = config_file["elastic"]["joints"];
				int i = 0;
				for (const auto& node : elastic_joints) {
					if (i==numElasticJoints) break; // break if nore joints defined
					string jointName = node.first.as<string>();

					// Helper lambda to parse a symbolic vector
					auto parse_symb_vector = [&](const string& key, int order) {
						vector<short> vec;
						if (node.second[key]){
							vec = node.second[key].as<vector<short>>();
							vec.resize(order);
						} else vec.assign(order, 0);
						return vec;
					};

					// - Numeric - //
					// stiffness
					if (K_order > 0){
						vector<double> K = node.second["K"].as<vector<double>>();
						for (int j=0; j<K_order; j++) par_K_num[K_order*i+j] = K[j];
					}
					// coupling friction
					if (D_order > 0){
						vector<double> D = node.second["D"].as<vector<double>>();
						for (int j=0; j<D_order; j++) par_D_num[D_order*i + j] = D[j];
					}
					// motor friction
					if (Dm_order > 0){
						vector<double> Dm = node.second["Dm"].as<vector<double>>();
						for (int j=0; j<Dm_order; j++) par_Dm_num[Dm_order*i + j] = Dm[j];
					}
					// motor inertia
					par_Mm_num[i] = node.second["Mm"].as<double>();
					
					// - Symbolic selectivity - //
					vector<short> K_symb = parse_symb_vector("K_symb", this->K_order);
					vector<short> D_symb = parse_symb_vector("D_symb", this->D_order);
					vector<short> Dm_symb = parse_symb_vector("Dm_symb", this->Dm_order);
					short Mm_symb = node.second["Mm_symb"] ? node.second["Mm_symb"].as<int>() : 0;
					if (K_order > 0) par_K_isSymb.insert(par_K_isSymb.end(), K_symb.begin(), K_symb.end());
					if (D_order > 0) par_D_isSymb.insert(par_D_isSymb.end(), D_symb.begin(), D_symb.end());
					if (Dm_order > 0) par_Dm_isSymb.insert(par_Dm_isSymb.end(), Dm_symb.begin(), Dm_symb.end());
					par_Mm_isSymb.push_back(Mm_symb);
					i++;
				}
				// - Models - //
				SX par_K_symb = SX::sym("par_K", numElasticJoints*K_order,1);
				SX par_D_symb = SX::sym("par_D", numElasticJoints*D_order,1);
				SX par_Dm_symb = SX::sym("par_Dm", numElasticJoints*Dm_order,1);
				SX par_Mm_symb = SX::sym("par_Mm", numElasticJoints,1);
				// - Add to parameters - //
				add_parameter("par_K", par_K_symb, par_K_num, par_K_isSymb, "Coupling stiffness parameters", true);
				add_parameter("par_D", par_D_symb, par_D_num, par_D_isSymb, "Coupling friction parameters", true);
				add_parameter("par_Dm", par_Dm_symb, par_Dm_num, par_Dm_isSymb, "Motor friction parameters", true);
				add_parameter("par_Mm", par_Mm_symb, par_Mm_num, par_Mm_isSymb, "Motor inertia parameters", true);
			}

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return 0; // Indicate failure
		}
		return 1; // Indicate success
	}

	DM Robot::get_value(string name){
		if (parameters.count(name)){					// parameter exists
			return parameters[name].num;
		} else if (functions.count(name)){				// function exists
			vector<DM> result;
			cout<<"name: "<<name<<endl;
			auto f_args = functions[name].args;
			int sz = f_args.size();
			cout<<"f_args:"<<f_args<<", size: "<<sz<<endl;
			casadi::DMVector inputs(sz);
			cout << "arg_names: ";
			int i=0;
			for (const auto& arg : f_args) {
				cout << arg << ", " << endl;
				inputs[i] = parameters[arg].num;
				i++;
			}
			cout<<"args: "<<inputs<<endl;
			casadi::Function fun = functions[name].fun;
			cout<<"fun: "<<fun<<endl;
			functions[name].fun.call(inputs, result);
			cout<<"result: "<<result<<endl;
			return DM::vertcat(result);
		} else {
			cout<<name + " not recognised"<<endl;
			// result_num.resize(1,1);
			// result_num << 0;
			return DM::zeros(1,1);
		}
		
		// return result_num;
	}

	int Robot::set_par(string name, DM value){
		if (parameters.count(name)){
			if (value.size() == parameters[name].num.size()){
				parameters[name].num = value;
			} else {
				std::cerr << "Size mismatch when setting parameter: " << name << endl;
				return 0;
			}
		} else {
			std::cerr << "Parameter not found: " << name << endl;
			return 0;
		}
		return 1;
	}

	DM Robot::get_par(string par){
		return parameters[par].num;
	}

	vector<fun_obj> Robot::get_functions(bool onlyNames) {
		vector<fun_obj> fun_vect;
		int sz = functions.size();
		fun_vect.resize(sz);
		int i=0;
		for (auto &f : functions){
			string name = f.first;
			fun_vect[i].name = f.first;
			fun_vect[i].description = f.second.description;
			fun_vect[i].args = f.second.args;
			fun_vect[i].out_size.resize(2);
			fun_vect[i].out_size[0] = model[name].size1();
			fun_vect[i].out_size[1] = model[name].size2();
			if (!onlyNames){
				fun_vect[i].expr = model[name];
				fun_vect[i].fun = f.second.fun;
			}
			i++;
		}
		return fun_vect;
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
	vector<int> Robot::get_isElasticJoint(){ return isElasticJoint; };

	vector<string> Robot::get_jointsType(){
		return jointsType;
	}

	casadi::SX Robot::load_par_REG(string file, bool update_DYN){
		vector<double> par_REG_num(STD_PAR_LINK*numJoints,0);
		// ----- parsing yaml inertial ----- //
		try {
			// load yaml
			YAML::Node config_file = YAML::LoadFile(file);
			// load inertial
			YAML::Node inertial = config_file["inertial"];
			int i = 0;
			for (const auto& node : inertial) {
				
				if (i==numJoints) break;
				string linkName = node.first.as<string>();
				
				// standard parameters
				par_REG_num[STD_PAR_LINK*i] = node.second["mass"].as<double>();
				par_REG_num[STD_PAR_LINK*i+1] = node.second["m_CoM_x"].as<double>();
				par_REG_num[STD_PAR_LINK*i+2] = node.second["m_CoM_y"].as<double>();
				par_REG_num[STD_PAR_LINK*i+3] = node.second["m_CoM_z"].as<double>();
				par_REG_num[STD_PAR_LINK*i+4] = node.second["Ixx"].as<double>();
				par_REG_num[STD_PAR_LINK*i+5] = node.second["Ixy"].as<double>();
				par_REG_num[STD_PAR_LINK*i+6] = node.second["Ixz"].as<double>();
				par_REG_num[STD_PAR_LINK*i+7] = node.second["Iyy"].as<double>();
				par_REG_num[STD_PAR_LINK*i+8] = node.second["Iyz"].as<double>();
				par_REG_num[STD_PAR_LINK*i+9] = node.second["Izz"].as<double>();

				i++;
			}
			// std::cout<<"\nparam REG \n"<<par_REG_num<<std::endl;
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		parameters["par_REG"].num = par_REG_num;
		if (update_DYN) update_inertial_DYN();
		return par_REG_num;
	}

	int Robot::load_par(string par_file, vector<string> par_list){
		try {
			// load yaml
			YAML::Node yamlFile = YAML::LoadFile(par_file);
			if (par_list.size() == 0){
				for (const auto& node : yamlFile){
					string key = node.first.as<string>();
					if (parameters.count(key)){
						parameters[key].num = node.second.as<vector<double>>();
					}
				}
			} else {
				for (string key : par_list){
					if (parameters.count(key)){
						parameters[key].num = yamlFile[key].as<vector<double>>();
					} else {
						std::cerr << "Parameter does not exist: " << key << std::endl;
					}
				}
			}
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while loading parameters: " << e.what() << std::endl;
			return 0;
		}
		return 1;
	}

	// void Robot::update_conf(){
	// 	// to do!
	// }

	// int Robot::save_conf(string par_file){
	// 	update_conf();
	// 	try {
	// 		YAML::Emitter emitter;
	// 		emitter.SetIndent(2);
	// 		emitter.SetSeqFormat(YAML::Flow);

	// 		emitter << config_yaml << YAML::Newline;

	// 		std::ofstream fout(par_file);
	// 		fout << emitter.c_str();
	// 		fout.close();
	// 	} catch (const YAML::Exception& e) {
	// 		std::cerr << "Error while generating YAML: " << e.what() << std::endl;
	// 		return 0;
	// 	}
	// 	return 1;
	// }

	int Robot::save_par_REG(string par_file){
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			YAML::Node yamlFile;
			YAML::Node dynamicsNode;

			emitter << YAML::Comment(
				"Inertial parameters referred to Denavit-Hartenberg parametrization to use with regressor matrix\n");
			emitter << YAML::Newline;

			DM par_REG = parameters["par_REG"].num;
			for (int i=0;  i<numJoints;  i++) {

				// LinkProp link = links_prop_[i];    
				YAML::Node linkNode;
				string nodeName;
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

	int Robot::save_par(string par_file, vector<string> par_list){
		try {
			YAML::Emitter emitter;
			emitter.SetIndent(2);
			emitter.SetSeqFormat(YAML::Flow);

			YAML::Node yamlFile;

			if (par_list.size() == 0){
				for (auto& par : parameters){
					string par_name = par.first;
					vector<double> vect_std = par.second.num.get_elements();
					yamlFile[par.first] = vect_std;
				}
			} else {
				for (auto& par : par_list){
					// YAML::Node par_node;
					// par_node[par] = args[par];
					// emitter << par_node << YAML::Newline;
					// yamlFile[par] = args[par];

					// std::cout << par + "_sx: " << args[par] << endl;
					// Eigen::VectorXd vect_eig = get_arg(par);
					// std::cout << par + "_eig: " << vect_eig << endl;
					if (parameters.count(par)){
						vector<double> vect_std = parameters[par].num.get_elements();
						yamlFile[par] = vect_std;
					} else {
						std::cerr << "Parameter does not exist: " << par << std::endl;
					}
				}
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
		DM& par_REG = parameters["par_REG"].num;
		DM& par_DYN = parameters["par_REG"].num;
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			DM p_reg(par_REG(p_idx));
			DM mass = p_reg(0);
			DM CoM = p_reg(casadi::Slice(1,4))/mass;
			DM I_tmp = mass * DM::mtimes(hat(CoM).T(), hat(CoM));
			DM I_reg = p_reg(casadi::Slice(4,10));
			DM I_tmp_v = DM::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			DM I = I_reg - I_tmp_v;
			par_DYN(p_idx) = DM::vertcat({mass, CoM, I});
		}
		return 1;
	}

	int Robot::update_inertial_REG(){
		DM& par_DYN = parameters["par_DYN"].num;
		DM& par_REG = parameters["par_REG"].num;
		for (int i=0; i<numJoints; i++){
			casadi::Slice p_idx(STD_PAR_LINK*i,STD_PAR_LINK*(i+1));
			DM p_dyn(par_DYN(p_idx));
			DM mass = p_dyn(0);
			DM mCoM = mass*p_dyn(casadi::Slice(1,4));
			DM I_tmp = DM::mtimes(hat(mCoM).T(), hat(mCoM))/mass;
			DM I_dyn = p_dyn(casadi::Slice(4,10));
			DM I_tmp_v = DM::vertcat({I_tmp(0,0), I_tmp(0,1), I_tmp(0,2), I_tmp(1,1), I_tmp(1,2), I_tmp(2,2)});
			DM I = I_dyn + I_tmp_v;
			par_REG(p_idx) = DM::vertcat({mass, mCoM, I});
		}
		return 1;
	}

	int Robot::add_variable(string v_name, SX symb, vector<double> num, vector<short> is_symbolic, string descr, bool overwrite){
		int ret = add_parameter(v_name, symb, num, is_symbolic, descr, overwrite);
		return ret;
	}

	int Robot::add_parameter(string p_name, SX symb, vector<double> num, vector<short> is_symbolic, string descr, bool overwrite){
		if ((!overwrite) && model.count(p_name)){
			// key already exists
			return 0;
		} else {
			par_obj param;
			param.name = p_name;
			param.description = descr;
			param.symb = symb;
			int size = symb.size1();
			param.size = size;

			if (num.size()!=size){
				std::cerr << "Error dimension of numeric SX: " << std::endl;
				return 0;
			} else {
				param.num = num;
			}

			if (is_symbolic.size()==size){ 				// normal initialization
				param.is_symbolic = is_symbolic;
			}else if (is_symbolic.size() == 1) { 		// one value initialization
				param.is_symbolic.resize(size);
				for (int i=0; i<size; i++){
					param.is_symbolic[i] = is_symbolic[0];
				}
			} else {							
				std::cerr << "Error dimension of symbolic vector: " << p_name << std::endl;
				return 0;
			}

			// add to parameters map
			parameters[p_name] = param;
			model[p_name] = param.get_value_all();
		}
		return 1;
	}

	int Robot::add_function(string f_name, casadi::SX expr, vector<string> args, string descr, bool overwrite){
		if ((!overwrite) && model.count(f_name)){
			// key already exists
			return 0;
		} else {
			fun_obj fun_struct;
			model[f_name] = expr;
			fun_struct.args = args;
			fun_struct.description = descr;

			casadi::SXVector inputs(args.size());
			// for (const auto& arg : args) {
			// 	inputs.push_back(model[arg]);
			// }
			int arg_index=0;
			for (const auto& arg : args) {
				// - resize parameters - //
				// int sz_original = args[arg].size();
				// std::cout << "fun: " << f_name << std::endl;
				// std::cout << "arg: " << arg << std::endl;
				vector<short>& symb_flag = parameters[arg].is_symbolic;
				vector<casadi::SX> par_symb;
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
			fun_struct.fun = fun;
			functions[f_name] = fun_struct;
		}

		return 1;
	}

	void Robot::generate_library(const string& savePath, const string& name_file, const bool SAVE_CASADI){
		// Options for c-code auto generation
		casadi::Dict opts = casadi::Dict();
		opts["cpp"] = true;
		opts["with_header"] = true;
		
		// generate functions in c code
		casadi::CodeGenerator myCodeGen = casadi::CodeGenerator(name_file, opts);
		// cout<<"casadi_fun: "<<casadi_fun<<endl;

		for (const auto& f : functions) {
			myCodeGen.add(f.second.fun);
			// cout<<"f_name: "<<f.first<<endl;
			// cout<<"fun: "<<f.second<<endl<<endl;
		}
		myCodeGen.generate(savePath);

		if(SAVE_CASADI){
			// Save CasADi functions
			for (const auto& f : functions) {
				string function_file = savePath + "/" + f.first + ".casadi";
				// std::ofstream file(function_file, std::ios::binary);
				f.second.fun.save(function_file);
				// file.close();
			}
		}
	}

	int Robot::subs_symb_par(string par){
		vector<short> symbolic = parameters[par].is_symbolic;
		// cout << "par: " << par << endl;
		// cout << "symbolic: " << symbolic << endl;
		// cout << "model: " << model[par] << endl;
		// cout << "args: " << args[par] << endl;
		// cout << "size: " << symbolic.size() << endl;

		for (int i=0; i<symbolic.size(); i++){
			if (symbolic[i] == 0){
				model[par](i) = (double)parameters[par].num(i);
			}
		}

		// cout<<"model: "<< model[par] << endl;
		// cout<<"arg: "<< args[par] << endl;
		return 1;
	}

	int Robot::init_symb_parameters(){
		// - substitute non-symbolic variables with numbers - //
		for (auto par : parameters){
			subs_symb_par(par.first);
		}
		return 1;
	}

	vector<string> Robot::obtain_symb_parameters(vector<string> par_sure, vector<string> par_possible){
		// - obtain parameters that have symbolic values inside - //
		vector<string> arg_list = par_sure;
		for (auto& par : par_possible){
			bool is_symb = false;
			for (short v : parameters[par].is_symbolic){
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
		for (auto par : parameters){
			auto par_isSymb = par.second.is_symbolic;
			int sz_original = par_isSymb.size();
			string par_name = par.first;
			casadi::SX& par_model = model[par_name];
			DM& par_num = par.second.num;
			int sz = 0;
			for (int i=0; i<sz_original; i++){
				if (par_isSymb[i]){
					par_model(sz) = par_model(i);
					par_num(sz) = par_num(i);
					sz++;
				}
			}
			par_model.resize(sz,1);
			par_num.resize(sz,1);
			// cout << "par_model: " << par_model << endl;
			// cout << "par_num: " << par_num << endl;
			// cout << "par_isSymb: " << par_isSymb << endl;
		}
		return 1;
	}


	Robot robot_from_file(string robot_name, string file, bool compute){
		bool advanced = true;
		Robot robot(file);
		robot.robotName = robot_name;
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
			compute_regressors(robot);
			cout<<"Regressors ok!"<<endl;
			compute_userDefined(robot);
			cout<<"User defined functions ok!"<<endl;

			// - update parameters - //
			robot.update_symb_parameters();
			cout<<"symbolic parameters ready!"<<endl;
		}
		
		cout<<"Robot created!"<<endl;

		return robot;
	}

}