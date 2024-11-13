#include "../library/robot.h"
#include "../library/kinematics.h"
#include "../library/dynamics.h"
#include "../library/regressors.h"
#include "../library/utils.h"

/* File name of generated code */
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

	Robot::Robot(const std::string config_file){
		load_config(config_file); 	// load config and parameters from yaml file
		initVarsFuns();				// initialize variables
	}

	Robot::Robot(const YAML::Node yaml){
		load_config(yaml); 		// load config and parameters from yaml file
		initVarsFuns();			// initialize variables
	}

	// Robot::Robot(const Config conf){
	// 	numJoints = conf.numJoints;
	// 	jointsType = conf.jointsType;
	// 	// numParLink.resize(numJoints);
	// 	ELASTIC = conf.ELASTIC;
	// 	K_order = conf.K_order;
	// 	D_order = conf.D_order;
	// 	Dl_order = conf.Dl_order;
	// 	Dm_order = conf.Dm_order;
	// 	// obtain number of parameters
	// 	// PAR_DYN_LINK = STD_PAR_LINK + Dl_order;
	// 	// PAR_REG_LINK = STD_PAR_LINK + Dl_order;
	// 	// PAR_ELA_LINK = K_order + D_order + Dm_order;
	// 	// numParDYN = STD_PAR_LINK*numJoints;
	// 	// numParREG = numParDYN;
	// 	if (jointsType.size()!=numJoints){
	// 		// problem on number of joints or joints type !!!
	// 		// --- To handle!!! --- //
	// 	}
	// 	numElasticJoints = 0;
	// 	isElasticJoint.resize(numJoints);
	// 	// elasticSel = casadi::SX::zeros(numJoints,numJoints);
	// 	for (int i=0; i<numJoints; i++){
	// 		isElasticJoint[i] = 0;
	// 		// numParLink[i] = STD_PAR_LINK;
	// 		if ((jointsType[i] == "R_SEA")||(jointsType[i] == "P_SEA")) {
	// 			// numParELA += K_order + D_order + Dm_order;
	// 			// numParLink[i] += K_order + D_order;
	// 			isElasticJoint[i] = 1;
	// 			numElasticJoints++;
	// 			// elasticSel(i,i) = 1;
	// 		}
	// 	}
	// 	// numParELA = numElasticJoints*(PAR_ELA_LINK);

	// 	// // N_PAR_LINK = 10;
	// 	// // jointsType = jointsType;
	// 	// _DHtable_ = conf.DHtable;
	// 	// _world2L0_ = conf.base_frame;
	// 	// // N_PAR_LINK = 10;
	// 	// // gravity = base_frame.get_gravity();
	// 	// _Ln2EE_ = conf.ee_frame;
	// 	valid = 1;
	// 	// _mu_ = MU;

	// 	// symbolic selectivity
	// 	symb["DHtable"] = conf.DHtable_symb;
	// 	symb["par_DYN"] = conf.par_DYN_symb;
	// 	symb["par_Dl"] = conf.par_Dl_symb;
	// 	symb["par_K"] = conf.par_K_symb;
	// 	symb["par_D"] = conf.par_D_symb;
	// 	symb["par_Dm"] = conf.par_Dm_symb;
	// 	symb["world2L0"] = conf.world2L0_symb;
	// 	symb["Ln2EE"] = conf.Ln2EE_symb;
	// 	symb["gravity"] = conf.gravity_symb;

	// 	// - parameters - //
	// 	casadi::SX DHtable = casadi::SX::sym("DHtable", numJoints,4);
	// 	casadi::SX world2L0 = casadi::SX::sym("world2L0", 6,1);
	// 	casadi::SX Ln2EE = casadi::SX::sym("Ln2EE", 6,1);
	// 	casadi::SX gravity = casadi::SX::sym("gravity", 3,1);
	// 	model.insert({"DHtable", DHtable});
	// 	model.insert({"world2L0", world2L0});
	// 	model.insert({"Ln2EE", Ln2EE});
	// 	model.insert({"gravity", gravity});

	// 	for (int i=0; i<3; i++){
	// 		world2L0(i) = casadi::SX(conf.base_frame.translation[i]);
	// 		world2L0(i+3) = conf.base_frame.ypr[i];
	// 		Ln2EE(i) = conf.ee_frame.translation[i];
	// 		Ln2EE(i+3) = conf.ee_frame.ypr[i];
	// 		gravity(i) = conf.base_frame.gravity[i];
	// 	}
	// 	args.insert({"DHtable", conf.DHtable});
	// 	args.insert({"world2L0", world2L0});
	// 	args.insert({"Ln2EE", Ln2EE});
	// 	args.insert({"gravity", gravity});

	// 	// - init all other variables - //
	// 	initVarsFuns();

	// 	// compute();
	// }

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

	int Robot::parse_config(){
		int nj;
		int STD_PAR_LINK = Robot::STD_PAR_LINK;
		FrameOffset Base_to_L0;
		FrameOffset Ln_to_EE;
		Config conf;
		// ----- parsing yaml file ----- //
		try {
			// load yaml
			YAML::Node config_file = config_yaml;

			// Number of joints
			nj = config_file["num_joints"].as<int>();
			conf.numJoints = nj;
			int Dl_order = 0;

			// joints_type
			// YAML::Node type_joints = config_file["type_joints"];
			// jType = type_joints.as<std::string>();
			conf.jointsType = config_file["type_joints"].as<std::vector<std::string>>();
			std::vector<std::string> jType = conf.jointsType;

			if (config_file["Dl_order"]) Dl_order = config_file["Dl_order"].as<int>();
			conf.Dl_order = Dl_order;
			if (config_file["ELASTIC_MODEL"]){
				conf.ELASTIC = config_file["ELASTIC_MODEL"].as<bool>();
				if (conf.ELASTIC){
					conf.K_order = config_file["elastic"]["K_order"].as<int>();
					conf.D_order = config_file["elastic"]["D_order"].as<int>();
					conf.Dm_order = config_file["elastic"]["Dm_order"].as<int>();
				}
			}
			// Denavit-Hartenberg
			YAML::Node kinematics = config_file["kinematics"];
			std::vector<double> dh_vect = kinematics["DH"].as<std::vector<double>>();
			// conf.DHtable = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);
			casadi::SX DHtable_tmp(nj,4);
			for (int i=0; i<nj; i++){
				for (int j=0; j<4; j++){
					DHtable_tmp(i,j) = dh_vect[4*i + j];
				}
			}
			conf.DHtable = DHtable_tmp;
			// cout<<"dhtable: "<<DHtable_tmp<<endl;

			// - Gravity - //
			YAML::Node gravity_node = config_file["gravity"];
			std::vector<double> gravity_vect = gravity_node["value"].as<std::vector<double>>();

			// frames offsets
			YAML::Node frame_base = config_file["Base_to_L0"];
			YAML::Node frame_ee = config_file["Ln_to_EE"];

			std::vector<double> tr = frame_base["tr"].as<std::vector<double>>();
			std::vector<double> ypr = frame_base["ypr"].as<std::vector<double>>();
			Base_to_L0.set_translation(tr);
			Base_to_L0.set_ypr(ypr);
			Base_to_L0.set_gravity(gravity_vect);
			conf.base_frame = Base_to_L0;

			tr = frame_ee["tr"].as<std::vector<double>>();
			ypr = frame_ee["ypr"].as<std::vector<double>>();
			Ln_to_EE.set_translation(tr);
			Ln_to_EE.set_ypr(ypr);
			conf.ee_frame = Ln_to_EE;

			// - Dynamics - //
			YAML::Node dynamics = config_file["dynamics"];
			// std::vector<LinkProp> links(nj);
			conf.links_DYN.resize(nj);
			std::vector<int> par_DYN_symb;
			std::vector<int> par_Dl_symb;
			par_DYN_symb.resize(STD_PAR_LINK*nj);
			par_Dl_symb.resize(Dl_order*nj);
			int link_index = 0;
			for (const auto& node : dynamics){
				std::string linkName = node.first.as<std::string>();
				conf.links_DYN[link_index].name = linkName;
				YAML::Node inertial = node.second["inertial"];
				// inertial
				// conf.links_DYN[link_index].mass = inertial["mass"].as<double>();
				// conf.links_DYN[link_index].xyz[0] = inertial["CoM_x"].as<double>();
				// conf.links_DYN[link_index].xyz[1] = inertial["CoM_y"].as<double>();
				// conf.links_DYN[link_index].xyz[2] = inertial["CoM_z"].as<double>();
				// conf.links_DYN[link_index].parI[0] = inertial["Ixx"].as<double>();
				// conf.links_DYN[link_index].parI[1] = inertial["Ixy"].as<double>();
				// conf.links_DYN[link_index].parI[2] = inertial["Ixz"].as<double>();
				// conf.links_DYN[link_index].parI[3] = inertial["Iyy"].as<double>();
				// conf.links_DYN[link_index].parI[4] = inertial["Iyz"].as<double>();
				// conf.links_DYN[link_index].parI[5] = inertial["Izz"].as<double>();
				// symbolic selectivity
				std::vector<int> link_symb;
				if (inertial["symb"]){
					link_symb = inertial["symb"].as<std::vector<int>>();
				} else {
					link_symb.resize(STD_PAR_LINK);
					for (int i=0; i<STD_PAR_LINK; i++) link_symb[i] = 0;
				}
				for (int j=0; j<STD_PAR_LINK; j++){
					par_DYN_symb[link_index*STD_PAR_LINK + j] = link_symb[j];
				}
				// friction
				if (Dl_order){
					conf.links_DYN[link_index].Dl.resize(Dl_order);
					YAML::Node friction = node.second["friction"];
					// std::vector<double> Dl = friction["Dl"].as<std::vector<double>>();
					// conf.links_DYN[link_index].Dl = Dl;
					// symbolic selectivity
					std::vector<int> fric_symb;
					if (friction["symb"]){
						fric_symb = friction["symb"].as<std::vector<int>>();
					} else {
						fric_symb.resize(Dl_order);
						for (int i=0; i<Dl_order; i++) fric_symb[i] = 0;
					}
					// std::vector<int> fric_symb = friction["symb"].as<std::vector<int>>();
					for (int j=0; j<Dl_order; j++){
						par_Dl_symb[link_index*Dl_order + j] = fric_symb[j];
					}
				}
				link_index++;
			}

			// parse elastic
			std::vector<int> par_K_symb;
			std::vector<int> par_D_symb;
			std::vector<int> par_Dm_symb;
			if (conf.ELASTIC){
				YAML::Node elastic = config_file["elastic"];
				int index = 0;
				int K_order = conf.K_order;
				int D_order = conf.D_order;
				int Dm_order = conf.Dm_order;
				for (const auto& node : elastic["joints"]) {
					std::string jointName = node.first.as<std::string>();
					// stiffness
					std::vector<int> K_symb;					
					if (node.second["K_symb"]){
						K_symb = node.second["K_symb"].as<std::vector<int>>();
					} else {
						K_symb.resize(K_order);
						for (int i=0; i<K_order; i++) K_symb[i] = 0;
					}
					for (int v : K_symb){
						par_K_symb.push_back(v);
					}
					// coupling friction
					std::vector<int> D_symb;
					if (node.second["D_symb"]){
						D_symb = node.second["D_symb"].as<std::vector<int>>();
					} else {
						D_symb.resize(D_order);
						for (int i=0; i<D_order; i++) D_symb[i] = 0;
					}
					for (int v : D_symb){
						par_D_symb.push_back(v);
					}
					// motor friction
					std::vector<int> Dm_symb;
					if (node.second["Dm_symb"]){
						Dm_symb = node.second["Dm_symb"].as<std::vector<int>>();
					} else {
						Dm_symb.resize(Dm_order);
						for (int i=0; i<Dm_order; i++) Dm_symb[i] = 0;
					}
					for (int v : Dm_symb){
						par_Dm_symb.push_back(v);
					}

					index++;
				}
			}
			
			// symbolic selectivity
			if (config_file["kinematics"]["symb"]){
				conf.DHtable_symb = config_file["kinematics"]["symb"].as<std::vector<int>>();
			} else {
				conf.DHtable_symb.resize(4*nj);
				for (int i=0; i<4*nj; i++) conf.DHtable_symb[i] = 0;
			}
			conf.par_DYN_symb = par_DYN_symb;
			conf.par_Dl_symb = par_Dl_symb;
			conf.par_K_symb = par_K_symb;
			conf.par_D_symb = par_D_symb;
			conf.par_Dm_symb = par_Dm_symb;
			if (config_file["Base_to_L0"]["symb"]){
				conf.world2L0_symb = config_file["Base_to_L0"]["symb"].as<std::vector<int>>();
			} else {
				conf.world2L0_symb.resize(6);
				for (int i=0; i<6; i++) conf.world2L0_symb[i] = 0;
			}
			// conf.world2L0_symb = config_file["Base_to_L0"]["symb"].as<std::vector<int>>();
			if (config_file["Ln_to_EE"]["symb"]){
				conf.Ln2EE_symb = config_file["Ln_to_EE"]["symb"].as<std::vector<int>>();
			} else {
				conf.Ln2EE_symb.resize(6);
				for (int i=0; i<6; i++) conf.Ln2EE_symb[i] = 0;
			}
			// conf.Ln2EE_symb = config_file["Ln_to_EE"]["symb"].as<std::vector<int>>();
			if (config_file["gravity"]["symb"]){
				conf.gravity_symb = config_file["gravity"]["symb"].as<std::vector<int>>();
			} else {
				conf.gravity_symb.resize(3);
				for (int i=0; i<3; i++) conf.gravity_symb[i] = 0;
			}
			// conf.gravity_symb = config_file["gravity"]["symb"].as<std::vector<int>>();
			
			// std::vector<double> dh_vect = kinematics["DH"].as<std::vector<double>>();
			// // conf.DHtable = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);
			// casadi::SX DHtable_tmp(nj,4);
			// for (int i=0; i<nj; i++){
			// 	for (int j=0; j<4; j++){
			// 		DHtable_tmp(i,j) = dh_vect[4*i + j];
			// 	}
			// }
			// conf.DHtable = DHtable_tmp;

			// ----- Apply config ----- //
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
			// elasticSel = casadi::SX::zeros(numJoints,numJoints);
			for (int i=0; i<numJoints; i++){
				isElasticJoint[i] = 0;
				// numParLink[i] = STD_PAR_LINK;
				if ((jointsType[i] == "R_SEA")||(jointsType[i] == "P_SEA")) {
					// numParELA += K_order + D_order + Dm_order;
					// numParLink[i] += K_order + D_order;
					isElasticJoint[i] = 1;
					numElasticJoints++;
					// elasticSel(i,i) = 1;
				}
			}

			// symbolic selectivity
			symb["DHtable"] = conf.DHtable_symb;
			symb["par_DYN"] = conf.par_DYN_symb;
			symb["par_Dl"] = conf.par_Dl_symb;
			symb["par_K"] = conf.par_K_symb;
			symb["par_D"] = conf.par_D_symb;
			symb["par_Dm"] = conf.par_Dm_symb;
			symb["world2L0"] = conf.world2L0_symb;
			symb["Ln2EE"] = conf.Ln2EE_symb;
			symb["gravity"] = conf.gravity_symb;

			// - parameters - //
			casadi::SX DHtable = casadi::SX::sym("DHtable", numJoints,4);
			casadi::SX world2L0 = casadi::SX::sym("world2L0", 6,1);
			casadi::SX Ln2EE = casadi::SX::sym("Ln2EE", 6,1);
			casadi::SX gravity = casadi::SX::sym("gravity", 3,1);
			model.insert({"DHtable", DHtable});
			model.insert({"world2L0", world2L0});
			model.insert({"Ln2EE", Ln2EE});
			model.insert({"gravity", gravity});

			for (int i=0; i<3; i++){
				world2L0(i) = casadi::SX(conf.base_frame.translation[i]);
				world2L0(i+3) = conf.base_frame.ypr[i];
				Ln2EE(i) = conf.ee_frame.translation[i];
				Ln2EE(i+3) = conf.ee_frame.ypr[i];
				gravity(i) = conf.base_frame.gravity[i];
			}
			args.insert({"DHtable", conf.DHtable});
			args.insert({"world2L0", world2L0});
			args.insert({"Ln2EE", Ln2EE});
			args.insert({"gravity", gravity});

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}
		return 1;
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

		// to change ----------------------------------------------------------------------------!
		// _DHtable_ = conf.DHtable;
		// _world2L0_ = conf.base_frame;
		// _Ln2EE_ = conf.ee_frame;
		// std::vector<int> DHtable_symb;
		// std::vector<int> world2L0_symb;
		// std::vector<int> Ln2EE_symb;
		// std::vector<int> gravity_symb;
		//------------------------------------------------------------------------------
		
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
			// casadi::Function fun = casadi_fun[name];
			// cout<<"ok3.5, fun: "<<fun<<endl;
			casadi_fun[name].call(inputs, result);
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

	Eigen::VectorXd Robot::get_par(std::string par){
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

	int Robot::load_par(std::string file, bool update_REG){
		// Eigen::VectorXd param_DYN;
		casadi::SX param_DYN(STD_PAR_LINK*numJoints,1);
		casadi::SX param_Dl(Dl_order*numJoints,1);
		casadi::SX param_K(numElasticJoints*K_order,1);
		casadi::SX param_D(numElasticJoints*D_order,1);
		casadi::SX param_Dm(numElasticJoints*Dm_order,1);
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
					for (int j=0; j<K_order; j++){
						std::vector<double> K = node.second["K"].as<std::vector<double>>();
						param_K(K_order*i+j) = K[j];
					}
					// coupling friction
					for (int j=0; j<D_order; j++){
						std::vector<double> D = node.second["D"].as<std::vector<double>>();
						param_D(D_order*i + j) = D[j];
					}
					// motor friction
					for (int j=0; j<Dm_order; j++){
						std::vector<double> Dm = node.second["Dm"].as<std::vector<double>>();
						param_Dm(Dm_order*i + j) = Dm[j];
					}
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

	int Robot::save_par(std::vector<std::string> par_list, std::string par_file){
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

				std::cout << par + "_sx: " << args[par] << endl;
				Eigen::VectorXd vect_eig = get_par(par);
				std::cout << par + "_eig: " << vect_eig << endl;
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
			// cout<<"CoM:"<<endl<<CoM<<endl<<endl;
			Eigen::Vector3d m_CoM = mass * CoM;
			// cout<<"m_CoM:"<<endl<<m_CoM<<endl<<endl;
			Eigen::Matrix3d I_tmp = mass * (hat(CoM) * hat(CoM).transpose());
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
			param_REG.segment(STD_PAR_LINK*i, STD_PAR_LINK) << mass, CoM, I+I_tmp_v;
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
			int i=0;
			for (const auto& arg : f_args) {
				inputs[i] = model[arg];
				i++;
			}

			casadi::Function fun(robotName+"_"+f_name+"_fun", inputs, {densify(expr)});
			// cout<<"fun: "<<fun<<endl;
			casadi_fun[f_name] = fun;
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

	int Robot::subs_symb_par(std::string par){
		std::vector<int> symbolic = symb[par];
		
		for (int i=0; i<symbolic.size(); i++){
			if (symbolic[i] == 0){
				model[par](i) = args[par](i);
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
		// std::vector<int> symbolic = symb["DHtable"];
		// for (int i=0; i<symbolic.size(); i++){
		// 	if (symbolic[i] == 0){
		// 		model["DHtable"][i] = args["DHtable"][i];
		// 	}
		// }
		// std::vector<int> DHtable_symb;
		// std::vector<int> par_DYN_symb;
		// std::vector<int> par_Dl_symb;
		// std::vector<int> par_K_symb;
		// std::vector<int> par_D_symb;
		// std::vector<int> par_Dm_symb;
		// std::vector<int> world2L0_symb;
		// std::vector<int> Ln2EE_symb;
		// std::vector<int> gravity_symb;
		return 1;
	}

	std::vector<std::string> Robot::obtain_symb_parameters(std::vector<std::string> par, std::vector<std::string> possible_par){
		// - obtain parameters that have symbolic values inside - //
		std::vector<std::string> arg_list = par;
		for (auto& par : possible_par){
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
		return 1;
	}

	// Config load_config(std::string file){
	// 	int nj;
	// 	int STD_PAR_LINK = Robot::STD_PAR_LINK;
	// 	FrameOffset Base_to_L0;
	// 	FrameOffset Ln_to_EE;
	// 	Config conf;
	// 	// ----- parsing yaml file ----- //
	// 	try {
	// 		// load yaml
	// 		YAML::Node config_file = YAML::LoadFile(file);

	// 		// Number of joints
	// 		nj = config_file["num_joints"].as<int>();
	// 		conf.numJoints = nj;
	// 		int Dl_order = 0;

	// 		// joints_type
	// 		// YAML::Node type_joints = config_file["type_joints"];
	// 		// jType = type_joints.as<std::string>();
	// 		conf.jointsType = config_file["type_joints"].as<std::vector<std::string>>();
	// 		std::vector<std::string> jType = conf.jointsType;

	// 		if (config_file["Dl_order"]) Dl_order = config_file["Dl_order"].as<int>();
	// 		conf.Dl_order = Dl_order;
	// 		if (config_file["ELASTIC_MODEL"]){
	// 			conf.ELASTIC = config_file["ELASTIC_MODEL"].as<bool>();
	// 			if (conf.ELASTIC){
	// 				conf.K_order = config_file["elastic"]["K_order"].as<int>();
	// 				conf.D_order = config_file["elastic"]["D_order"].as<int>();
	// 				conf.Dm_order = config_file["elastic"]["Dm_order"].as<int>();
	// 			}
	// 		}
	// 		// Denavit-Hartenberg
	// 		YAML::Node kinematics = config_file["kinematics"];
	// 		std::vector<double> dh_vect = kinematics["DH"].as<std::vector<double>>();
	// 		// conf.DHtable = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);
	// 		casadi::SX DHtable_tmp(nj,4);
	// 		for (int i=0; i<nj; i++){
	// 			for (int j=0; j<4; j++){
	// 				DHtable_tmp(i,j) = dh_vect[4*i + j];
	// 			}
	// 		}
	// 		conf.DHtable = DHtable_tmp;
	// 		// cout<<"dhtable: "<<DHtable_tmp<<endl;

	// 		// - Gravity - //
	// 		YAML::Node gravity = config_file["gravity"];
	// 		std::vector<double> gravity_vect = gravity["value"].as<std::vector<double>>();

	// 		// frames offsets
	// 		YAML::Node frame_base = config_file["Base_to_L0"];
	// 		YAML::Node frame_ee = config_file["Ln_to_EE"];

	// 		std::vector<double> tr = frame_base["tr"].as<std::vector<double>>();
	// 		std::vector<double> ypr = frame_base["ypr"].as<std::vector<double>>();
	// 		Base_to_L0.set_translation(tr);
	// 		Base_to_L0.set_ypr(ypr);
	// 		Base_to_L0.set_gravity(gravity_vect);
	// 		conf.base_frame = Base_to_L0;

	// 		tr = frame_ee["tr"].as<std::vector<double>>();
	// 		ypr = frame_ee["ypr"].as<std::vector<double>>();
	// 		Ln_to_EE.set_translation(tr);
	// 		Ln_to_EE.set_ypr(ypr);
	// 		conf.ee_frame = Ln_to_EE;

	// 		// - Dynamics - //
	// 		YAML::Node dynamics = config_file["dynamics"];
	// 		// std::vector<LinkProp> links(nj);
	// 		conf.links_DYN.resize(nj);
	// 		std::vector<int> par_DYN_symb;
	// 		std::vector<int> par_Dl_symb;
	// 		par_DYN_symb.resize(STD_PAR_LINK*nj);
	// 		par_Dl_symb.resize(Dl_order*nj);
	// 		int link_index = 0;
	// 		for (const auto& node : dynamics){
	// 			std::string linkName = node.first.as<std::string>();
	// 			conf.links_DYN[link_index].name = linkName;
	// 			YAML::Node inertial = node.second["inertial"];
	// 			// inertial
	// 			// conf.links_DYN[link_index].mass = inertial["mass"].as<double>();
	// 			// conf.links_DYN[link_index].xyz[0] = inertial["CoM_x"].as<double>();
	// 			// conf.links_DYN[link_index].xyz[1] = inertial["CoM_y"].as<double>();
	// 			// conf.links_DYN[link_index].xyz[2] = inertial["CoM_z"].as<double>();
	// 			// conf.links_DYN[link_index].parI[0] = inertial["Ixx"].as<double>();
	// 			// conf.links_DYN[link_index].parI[1] = inertial["Ixy"].as<double>();
	// 			// conf.links_DYN[link_index].parI[2] = inertial["Ixz"].as<double>();
	// 			// conf.links_DYN[link_index].parI[3] = inertial["Iyy"].as<double>();
	// 			// conf.links_DYN[link_index].parI[4] = inertial["Iyz"].as<double>();
	// 			// conf.links_DYN[link_index].parI[5] = inertial["Izz"].as<double>();
	// 			// symbolic selectivity
	// 			std::vector<int> link_symb;
	// 			if (inertial["symb"]){
	// 				link_symb = inertial["symb"].as<std::vector<int>>();
	// 			} else {
	// 				link_symb.resize(STD_PAR_LINK);
	// 				for (int i=0; i<STD_PAR_LINK; i++) link_symb[i] = 0;
	// 			}
	// 			for (int j=0; j<STD_PAR_LINK; j++){
	// 				par_DYN_symb[link_index*STD_PAR_LINK + j] = link_symb[j];
	// 			}
	// 			// friction
	// 			if (Dl_order){
	// 				conf.links_DYN[link_index].Dl.resize(Dl_order);
	// 				YAML::Node friction = node.second["friction"];
	// 				// std::vector<double> Dl = friction["Dl"].as<std::vector<double>>();
	// 				// conf.links_DYN[link_index].Dl = Dl;
	// 				// symbolic selectivity
	// 				std::vector<int> fric_symb;
	// 				if (friction["symb"]){
	// 					fric_symb = friction["symb"].as<std::vector<int>>();
	// 				} else {
	// 					fric_symb.resize(Dl_order);
	// 					for (int i=0; i<Dl_order; i++) fric_symb[i] = 0;
	// 				}
	// 				// std::vector<int> fric_symb = friction["symb"].as<std::vector<int>>();
	// 				for (int j=0; j<Dl_order; j++){
	// 					par_Dl_symb[link_index*Dl_order + j] = fric_symb[j];
	// 				}
	// 			}
	// 			link_index++;
	// 		}

	// 		// parse elastic
	// 		std::vector<int> par_K_symb;
	// 		std::vector<int> par_D_symb;
	// 		std::vector<int> par_Dm_symb;
	// 		if (conf.ELASTIC){
	// 			YAML::Node elastic = config_file["elastic"];
	// 			int index = 0;
	// 			int K_order = conf.K_order;
	// 			int D_order = conf.D_order;
	// 			int Dm_order = conf.Dm_order;
	// 			for (const auto& node : elastic["joints"]) {
	// 				std::string jointName = node.first.as<std::string>();
	// 				// stiffness
	// 				std::vector<int> K_symb;					
	// 				if (node.second["K_symb"]){
	// 					K_symb = node.second["K_symb"].as<std::vector<int>>();
	// 				} else {
	// 					K_symb.resize(K_order);
	// 					for (int i=0; i<K_order; i++) K_symb[i] = 0;
	// 				}
	// 				for (int v : K_symb){
	// 					par_K_symb.push_back(v);
	// 				}
	// 				// coupling friction
	// 				std::vector<int> D_symb;
	// 				if (node.second["D_symb"]){
	// 					D_symb = node.second["D_symb"].as<std::vector<int>>();
	// 				} else {
	// 					D_symb.resize(D_order);
	// 					for (int i=0; i<D_order; i++) D_symb[i] = 0;
	// 				}
	// 				for (int v : D_symb){
	// 					par_D_symb.push_back(v);
	// 				}
	// 				// motor friction
	// 				std::vector<int> Dm_symb;
	// 				if (node.second["Dm_symb"]){
	// 					Dm_symb = node.second["Dm_symb"].as<std::vector<int>>();
	// 				} else {
	// 					Dm_symb.resize(Dm_order);
	// 					for (int i=0; i<Dm_order; i++) Dm_symb[i] = 0;
	// 				}
	// 				for (int v : Dm_symb){
	// 					par_Dm_symb.push_back(v);
	// 				}

	// 				index++;
	// 			}
	// 		}
			
	// 		// symbolic selectivity
	// 		if (config_file["kinematics"]["symb"]){
	// 			conf.DHtable_symb = config_file["kinematics"]["symb"].as<std::vector<int>>();
	// 		} else {
	// 			conf.DHtable_symb.resize(4*nj);
	// 			for (int i=0; i<4*nj; i++) conf.DHtable_symb[i] = 0;
	// 		}
	// 		conf.par_DYN_symb = par_DYN_symb;
	// 		conf.par_Dl_symb = par_Dl_symb;
	// 		conf.par_K_symb = par_K_symb;
	// 		conf.par_D_symb = par_D_symb;
	// 		conf.par_Dm_symb = par_Dm_symb;
	// 		if (config_file["Base_to_L0"]["symb"]){
	// 			conf.world2L0_symb = config_file["Base_to_L0"]["symb"].as<std::vector<int>>();
	// 		} else {
	// 			conf.world2L0_symb.resize(6);
	// 			for (int i=0; i<6; i++) conf.world2L0_symb[i] = 0;
	// 		}
	// 		// conf.world2L0_symb = config_file["Base_to_L0"]["symb"].as<std::vector<int>>();
	// 		if (config_file["Ln_to_EE"]["symb"]){
	// 			conf.Ln2EE_symb = config_file["Ln_to_EE"]["symb"].as<std::vector<int>>();
	// 		} else {
	// 			conf.Ln2EE_symb.resize(6);
	// 			for (int i=0; i<6; i++) conf.Ln2EE_symb[i] = 0;
	// 		}
	// 		// conf.Ln2EE_symb = config_file["Ln_to_EE"]["symb"].as<std::vector<int>>();
	// 		if (config_file["gravity"]["symb"]){
	// 			conf.gravity_symb = config_file["gravity"]["symb"].as<std::vector<int>>();
	// 		} else {
	// 			conf.gravity_symb.resize(3);
	// 			for (int i=0; i<3; i++) conf.gravity_symb[i] = 0;
	// 		}
	// 		// conf.gravity_symb = config_file["gravity"]["symb"].as<std::vector<int>>();
			
	// 		// std::vector<double> dh_vect = kinematics["DH"].as<std::vector<double>>();
	// 		// // conf.DHtable = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);
	// 		// casadi::SX DHtable_tmp(nj,4);
	// 		// for (int i=0; i<nj; i++){
	// 		// 	for (int j=0; j<4; j++){
	// 		// 		DHtable_tmp(i,j) = dh_vect[4*i + j];
	// 		// 	}
	// 		// }
	// 		// conf.DHtable = DHtable_tmp;

	// 	} catch (const YAML::Exception& e) {
	// 		std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
	// 	}

	// 	return conf;
	// }

	Robot robot_from_file(std::string robot_name, std::string file, bool compute){
		// create config
		// Config conf = load_config(file);
		// cout<<"Configuration loaded!"<<endl;
		Robot robot(file);
		robot.robotName = robot_name;
		// --- load parameters --- //
		robot.load_par(file);
		// --- compute functions --- //
		if (compute){
			// - symbolic selectivity - //
			robot.init_symb_parameters();
			cout<<"symbolic parameters ok!"<<endl;
			// - compute functions - //
			compute_kinematics(robot);
			cout<<"Kinematics ok!"<<endl;
			compute_dynamics(robot);
			cout<<"Dynamics ok!"<<endl;
			compute_regressors(robot);
			cout<<"Regressors ok!"<<endl;
			// - update parameters - //
			robot.update_symb_parameters();
			cout<<"symbolic parameters ready!"<<endl;
		}
		
		cout<<"Robot created!"<<endl;

		return robot;
	}

}