#include "plugins/loaders/dyn_loader.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> DynLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML ----- //
		try {
			// --- Take from robot --- //
			const int numJoints = robot->get<int>("numJoints");
			const int ndof = robot->get<int>("ndof");
			const vector<string> jointsName = robot->get<vector<string>>("jointsName");
			const vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");


			// --- Basic robot properties --- //
			int STD_PAR_LINK = 10;
			if (robot->properties.count("STD_PAR_LINK")){
				STD_PAR_LINK = robot->get<int>("STD_PAR_LINK");
			} else {
				robot->add_property<int>("STD_PAR_LINK", STD_PAR_LINK, "int", "Standard number of dynamic parameters per link", true);
			}


			// --- Gravity --- //
			vector<double> gravity_num = {0.0, 0.0, 0.0}; // default no gravity
			if (config_["gravity"]){
				gravity_num = config_["gravity"]["value"].as<vector<double>>();
			}
			// - Symbolic selectivity - //
			vector<short> gravity_isSymb;
			if (config_["gravity"]["symb"]) gravity_isSymb = config_["gravity"]["symb"].as<vector<short>>();
			else gravity_isSymb.assign(3, 0);
			// - Model - //
			SX gravity_symb = SX::sym("gravity", 3);
			// - Add to parameters - //
			robot->add_parameter("par_gravity", gravity_symb, gravity_num, gravity_isSymb, "Gravity on world frame", true);


			// --- Dynamics (Inertial) --- //
			vector<double> par_DYN_num(STD_PAR_LINK*numJoints, 0);
			vector<double> par_REG_num(STD_PAR_LINK*numJoints, 0);
			vector<short> par_DYN_isSymb(STD_PAR_LINK*numJoints, 0);
			YAML::Node dynamics = config_["dynamics"];

			for (int i=0; i<numJoints; i++) {
				vector<double> link_dyn(STD_PAR_LINK, 0.0);
				if (dynamics[jointsName[i]]) {
					YAML::Node inertial = dynamics[jointsName[i]]["inertial"];

					// - Numeric - //
					// Mass
					par_DYN_num[STD_PAR_LINK*i] = inertial["mass"].as<double>();
					// Center of Mass
					if (inertial["CoM"]) {
						vector<double> CoM = inertial["CoM"].as<vector<double>>();
						par_DYN_num[STD_PAR_LINK*i+1] = CoM[0];
						par_DYN_num[STD_PAR_LINK*i+2] = CoM[1];
						par_DYN_num[STD_PAR_LINK*i+3] = CoM[2];
					} else {
						par_DYN_num[STD_PAR_LINK*i+1] = inertial["CoM_x"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+2] = inertial["CoM_y"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+3] = inertial["CoM_z"].as<double>();
					}
					// Inertia terms
					if (inertial["I"]) {
						vector<double> I = inertial["I"].as<vector<double>>();
						for (int j=4; j<10; j++) {
							par_DYN_num[STD_PAR_LINK*i+j] = I[j];
						}
					} else {
						par_DYN_num[STD_PAR_LINK*i+4] = inertial["Ixx"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+5] = inertial["Ixy"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+6] = inertial["Ixz"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+7] = inertial["Iyy"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+8] = inertial["Iyz"].as<double>();
						par_DYN_num[STD_PAR_LINK*i+9] = inertial["Izz"].as<double>();
					}

					// - Symbolic selectivity - //
					vector<short> link_isSymb;
					if (inertial["symb"]) {
						link_isSymb = inertial["symb"].as<vector<short>>();
						std::copy(link_isSymb.begin(), link_isSymb.end(), par_DYN_isSymb.begin() + i * STD_PAR_LINK);
					}					
				}
			}

			// - Model - //
			SX par_DYN_symb = SX::sym("par_DYN", STD_PAR_LINK*numJoints,1);
			SX par_REG_symb = SX::sym("par_REG", STD_PAR_LINK*numJoints,1);
			// - Add to parameters - //
			robot->add_parameter("par_DYN", par_DYN_symb, par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			// vector<short> par_REG_isSymb = par_DYN_isSymb;
			robot->add_parameter("par_REG", par_REG_symb, par_REG_num, {1}, "Dynamic parameters for regressor", true);


			// --- Link friction --- //
			int Dl_order = config_["Dl_order"] ? config_["Dl_order"].as<int>() : 0;	// defaults to 0
			robot->add_property<int>("Dl_order", Dl_order, "int", "Order of the link friction model", true);
			if (Dl_order > 0) {
				vector<double> par_Dl_num(Dl_order*ndof, 0.0);
				vector<short> par_Dl_isSymb(Dl_order*ndof, 0);
				for (int i=0, count=0; i<numJoints; i++) {
					int j_dim = jointsDimension[i];
					if (dynamics[jointsName[i]]) {
						if (dynamics[jointsName[i]]["friction"]){
							// - Numeric - //
							YAML::Node friction = dynamics[jointsName[i]]["friction"];
							vector<double> Dl = friction["Dl"].as<vector<double>>();
							for (int j=0; j<Dl_order*j_dim; j++){
								par_Dl_num[Dl_order*count + j] = Dl[j];
							}

							// - Symbolic selectivity - //
							vector<int> fric_isSymb;
							if (friction["symb"]) {
								fric_isSymb = friction["symb"].as<vector<int>>();
								for (int j=0; j<Dl_order*j_dim; j++){
									par_Dl_isSymb[Dl_order*count + j] = fric_isSymb[j];
								}
								// std::copy(fric_isSymb.begin(), fric_isSymb.end(), par_Dl_isSymb.begin() + count*Dl_order);
							}
						}
					}
					count += j_dim;
				}
				// - Model - //
				SX par_Dl_symb = SX::sym("par_Dl", ndof*Dl_order,1);
				// - Add to parameters - //
				robot->add_parameter("par_Dl", par_Dl_symb, par_Dl_num, par_Dl_isSymb, "Link friction parameters", true);
			}
			
		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
