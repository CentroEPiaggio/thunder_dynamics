#include "plugins/loaders/dh_loader.h"
#include "utils.h"


namespace thunder_ns {

	// --- DH template for joints --- //
	casadi::SX DHTemplate(const casadi::SX& rowDHTable, const casadi::SX& qi, std::string jointType) {
		// Transformation:  T_a * T_alpha * T_d * T_theta
		casadi::SX Ti(4,4); // output
		casadi::SX p_a(3,1);
		casadi::SX p_d(3,1);
		casadi::Slice idx(0, 3);      // [0,1,2] indexes

		casadi::SX a = rowDHTable(0);
		casadi::SX alpha = rowDHTable(1);;
		casadi::SX d;
		casadi::SX theta;
		
		// Check
		if ((jointType == "P")||(jointType == "P_SEA")) {
			d = rowDHTable(2) + qi;
			theta = rowDHTable(3);
		}
		else if ((jointType == "R")||(jointType == "R_SEA")) {
			d = rowDHTable(2);
			theta = rowDHTable(3) + qi;
		}
		else {
			throw std::runtime_error("DHTemplate: Error joint type");
		}

		p_a(0) = a;
		p_d(2) = d;
		casadi::SX Rx = R_x(alpha);
		casadi::SX Rz = R_z(theta);

		Ti(idx,idx) = casadi::SX::mtimes(Rx, Rz);
		Ti(idx,3) = casadi::SX::mtimes(Rx, p_d) + p_a;
		Ti(3,3) = 1;

		return Ti;
	}

	// --- Load function --- //
	std::shared_ptr<Robot> DHLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// ----- Parsing YAML File ----- //
		try {
			// Local properties for parsing
			int numJoints = 0;
			vector<string> jointsType;


			// --- Basic Robot properties --- //

			// - numJoints - //
			if (config_["num_joints"]) {
				numJoints = config_["num_joints"].as<int>();
				robot->add_property<int>("numJoints", numJoints, "int", "Number of joints", true);
			} else {
				throw std::runtime_error("No num_joints in yaml file.");
			}
			
			
			// --- Variables --- //
			// - Normal joints - //
			robot->add_variable("q", SX::sym("q",numJoints,1), vector<double>(numJoints,0), {1}, "Configuration", true);
			robot->add_variable("dq", SX::sym("dq",numJoints,1), vector<double>(numJoints,0), {1}, "Velocity", true);


			// --- Denavit-Hartenberg --- //
			vector<double> dh_num = config_["DH"].as<vector<double>>();
			int dh_size = dh_num.size();
			// - Symbolic selectivity - //
			vector<short> dh_isSymb;
			if (config_["symb"]) dh_isSymb = config_["symb"].as<vector<short>>();
			else dh_isSymb.assign(dh_size, 0);
			// - Model - //
			SX dh_symb = SX::sym("DHtable", numJoints * 4);
			// - Add to parameters - //
			robot->add_parameter("par_DHtable", dh_symb, dh_num, dh_isSymb, "DH parameters", true);

			// --- internal kinematic parameters (xyzrpy) --- //
			int sz = 6*numJoints;
			casadi::SX par_KIN(sz,1);	// output
			casadi::SX DH = robot->get_model("par_DHtable");

			for (int i=0; i<numJoints; i++){
				// DH transformation:  T_a * T_alpha * T_d * T_theta
				casadi::SX p_a(3,1);
				casadi::SX p_d(3,1);
				casadi::Slice idx_tr(i*numJoints, 3+i*numJoints);      	// [0,1,2]+i*nj indexes
				casadi::Slice idx_or(4+i*numJoints, 6+i*numJoints);      	// [0,1,2]+i*nj indexes
				casadi::SX a = DH(i*numJoints);
				casadi::SX alpha = DH(1 + i*numJoints);
				casadi::SX d = DH(2 + i*numJoints);
				casadi::SX theta = DH(3 + i*numJoints);
				p_a(0) = a;
				p_d(2) = d;

				casadi::SX Rx = R_x(alpha);
				casadi::SX pos = casadi::SX::mtimes(Rx, p_d) + p_a;
				par_KIN(idx_tr) = pos;
				par_KIN(idx_or)(0) = alpha;
				par_KIN(idx_or)(2) = theta;
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