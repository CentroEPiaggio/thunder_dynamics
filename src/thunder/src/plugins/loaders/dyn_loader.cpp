#include "plugins/loaders/dyn_loader.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> DynLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML ----- //
		try {
			// --- Take from robot --- //
			const int numJoints = robot->get<int>("numJoints");


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
			vector<double> par_DYN_num(STD_PAR_LINK*numJoints,0);
			vector<double> par_REG_num(STD_PAR_LINK*numJoints,0);
			vector<short> par_DYN_isSymb(STD_PAR_LINK * numJoints);
			YAML::Node dynamics = config_["dynamics"];
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
			robot->add_parameter("par_DYN", par_DYN_symb, par_DYN_num, par_DYN_isSymb, "Dynamic parameters", true);
			vector<short> par_REG_isSymb = par_DYN_isSymb;
			robot->add_parameter("par_REG", par_REG_symb, par_REG_num, par_REG_isSymb, "Dynamic parameters for regressor", true);


			// --- Link friction --- //
			int Dl_order = config_["Dl_order"] ? config_["Dl_order"].as<int>() : 0;	// defaults to 0
			robot->add_property<int>("Dl_order", Dl_order, "int", "Order of the link friction model", true);
			vector<double> par_Dl_num;
			vector<short> par_Dl_isSymb;
			if (Dl_order > 0) {
				par_Dl_num.resize(Dl_order * numJoints);
				par_Dl_isSymb.resize(Dl_order * numJoints);
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
						fric_isSymb.assign(Dl_order, 0);
					}
					std::copy(fric_isSymb.begin(), fric_isSymb.end(), par_Dl_isSymb.begin() + idx * Dl_order);
					idx++;
				}
				// - Model - //
				SX par_Dl_symb = SX::sym("par_Dl", numJoints*Dl_order,1);
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
