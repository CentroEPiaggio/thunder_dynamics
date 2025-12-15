#include "plugins/loaders/dh_loader.h"
#include "utils.h"


namespace thunder_ns {

	// --- Load function --- //
	std::shared_ptr<Robot> DHLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// --- Denavit-Hartenberg --- //
			vector<double> dh_num = config_["DH"].as<vector<double>>();
			int dh_size = dh_num.size();
			const int NJ = dh_size/4;
			// - Symbolic selectivity - //
			vector<short> dh_isSymb;
			if (config_["symb"]) dh_isSymb = config_["symb"].as<vector<short>>();
			else dh_isSymb.assign(dh_size, 0);
			// - Model - //
			SX dh_symb = SX::sym("DHtable", 4*NJ);
			// - Add to parameters - //
			robot->add_parameter("par_DHtable", dh_symb, dh_num, dh_isSymb, "DH parameters", true);

			// --- internal kinematic parameters (xyzrpy) --- //
			const int SZ = 6*NJ;
			casadi::SX par_KIN(SZ,1);	// output
			casadi::SX DH = robot->get_model("par_DHtable");

			for (int i=0; i<NJ; i++){
				// DH transformation:  T_a * T_alpha * T_d * T_theta
				casadi::SX p_a(3,1);
				casadi::SX p_d(3,1);
				casadi::Slice idx_tr(6*i, 3+6*i);      	// [0,1,2]+6*i indexes
				casadi::Slice idx_or(3+6*i, 6+6*i);     // [0,1,2]+6*i indexes
				casadi::SX a = DH(4*i);
				casadi::SX alpha = DH(1 + 4*i);
				casadi::SX d = DH(2 + 4*i);
				casadi::SX theta = DH(3 + 4*i);
				p_a(0) = a;
				p_d(2) = d;

				casadi::SX Rx = R_x(alpha);
				casadi::SX pos = casadi::SX::mtimes(Rx, p_d) + p_a;
				casadi::SX rpy(3,1);
				rpy(0) = alpha;
				rpy(2) = theta;
				par_KIN(idx_tr) = pos;
				par_KIN(idx_or) = rpy;
			}
			if (!robot->add_function("par_KIN", par_KIN, {"par_DHtable"}, "Internal kinematic parameters.")) {
				std::cerr << "Error adding kinmatic parameters!" << std::endl;
				return robot;
			}

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns