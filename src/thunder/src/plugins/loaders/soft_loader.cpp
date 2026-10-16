#include "plugins/loaders/soft_loader.h"


namespace thunder_ns {
	// using namespace legacy;

	void SoftLoader::create_elastic_joints(std::shared_ptr<Robot> robot){
		// --- Standard joint functions --- //
		SX q_joint = SX::sym("q_joint");
		SX axis = SX::sym("axis",3,1);
		FunArg q_joint_arg("q_joint", q_joint);
		FunArg axis_arg("axis", axis);
		casadi::Slice rot(0, 3);      // [0,1,2] indexes
		SX Ti = SX::eye(4);

		// Rotoidal classical joint
		Ti = SX::eye(4);
		Ti(rot,rot) = R_aa(axis, q_joint);
		if (!robot->add_function("T_JOINT_R_SEA", Ti, {}, "Template transformation of rotoidal joint with elasticity", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: T_JOINT_R_SEA" << std::endl;
		}

		// Prismatic classical joint
		Ti = SX::eye(4);
		Ti(casadi::Slice(0,3),3) = casadi::SX::mtimes(axis, q_joint);
		if (!robot->add_function("T_JOINT_P_SEA", Ti, {}, "Template transformation of prismatic joint with elasticity", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: T_JOINT_P_SEA" << std::endl;
		}

		// Motion subspaces, same as the rigid joints R and P
		SX S_R = SX::vertcat({SX::zeros(3,1), axis / SX::norm_2(axis)});
		if (!robot->add_function("S_JOINT_R_SEA", S_R, {}, "Motion subspace of rotoidal joint with elasticity", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: S_JOINT_R_SEA" << std::endl;
		}
		SX S_P = SX::vertcat({axis, SX::zeros(3,1)});
		if (!robot->add_function("S_JOINT_P_SEA", S_P, {}, "Motion subspace of prismatic joint with elasticity", {q_joint_arg, axis_arg})) {
			std::cerr << "Error adding joint function: S_JOINT_P_SEA" << std::endl;
		}

	}

	// --- Load function --- //
	std::shared_ptr<Robot> SoftLoader::load(std::shared_ptr<Robot> robot){
		debug_log("Loading started", VERB_INFO);

		// --- add elastic joint definitions --- //
		create_elastic_joints(robot);

		// ----- Parsing YAML File ----- //
		try {
			int K_order = 0;
			int D_order = 0;
			int Dm_order = 0;

			// - Load from robot - //
			int numJoints = robot->get<int>("numJoints");
			int ndof = robot->get<int>("ndof");
			vector<string> jointsType = robot->get<vector<string>>("jointsType");
			vector<string> jointsName = robot->get<vector<string>>("jointsName");
			vector<int> jointsDimension = robot->get<vector<int>>("jointsDimension");

			// - identify elastic joints - //
			// isSoftJoint has one flag per joint variable (entry of q): soft_builder takes the link side of each elastic joint from it
			int numSoftJoints = 0;
			vector<short> isSoftJoint;
			vector<string> softJointsName;		// in tree order, the order of x and of the parameters
			isSoftJoint.reserve(ndof);
			for (int i = 0; i < numJoints; i++) {
				const bool isSoft = jointsType[i] == "R_SEA" || jointsType[i] == "P_SEA";
				if (isSoft && jointsDimension[i] != 1) throw std::runtime_error("Elastic joint '" + jointsName[i] + "' must have dimension 1.");
				for (int k = 0; k < jointsDimension[i]; k++) isSoftJoint.push_back(isSoft);
				if (isSoft) {
					softJointsName.push_back(jointsName[i]);
					numSoftJoints++;
				}
			}

			// --- Soft model properties (defaults to false) --- //
			// ELASTIC = config_["ELASTIC_MODEL"] && config_["ELASTIC_MODEL"].as<bool>();
			// robot->add_property<bool>("ELASTIC", ELASTIC, "bool", "Elastic flag", true);
			if (numSoftJoints) {
				// - setup elastic properties - //
				if (!config_["K_order"]) throw std::runtime_error("K_order property not present.");
				if (!config_["D_order"]) throw std::runtime_error("D_order property not present.");
				if (!config_["Dm_order"]) throw std::runtime_error("Dm_order property not present.");
				K_order = config_["K_order"].as<int>();
				D_order = config_["D_order"].as<int>();
				Dm_order = config_["Dm_order"].as<int>();
				robot->add_property<int>("K_order", K_order, "int", "Order of the coupling stiffness model", true);
				robot->add_property<int>("D_order", D_order, "int", "Order of the coupling friction model", true);
				robot->add_property<int>("Dm_order", Dm_order, "int", "Order of the motor stiffness model", true);

				robot->add_property<int>("numSoftJoints", numSoftJoints, "int", "Number of soft joints", true);
				robot->add_property<vector<short>>("isSoftJoint", isSoftJoint, "vector<short>", "Vector of soft joint flags", true);
			}


			// --- Variables --- //
			// - Soft joints - //
			robot->add_variable("x", SX::sym("x",numSoftJoints,1), vector<double>(numSoftJoints,0), {1}, "Motor angle", true);
			robot->add_variable("dx", SX::sym("dx",numSoftJoints,1), vector<double>(numSoftJoints,0), {1}, "Motor velocity", true);
			robot->add_variable("ddx", SX::sym("ddx",numSoftJoints,1), vector<double>(numSoftJoints,0), {1}, "Motor acceleration", true);
			robot->add_variable("ddxr", SX::sym("ddxr",numSoftJoints,1), vector<double>(numSoftJoints,0), {1}, "Motor acceleration reference", true);
			
			// - Soft joints parameters - //
			vector<double> par_K_num, par_D_num, par_Dm_num, par_Mm_num;
			vector<short> par_K_isSymb, par_D_isSymb, par_Dm_isSymb, par_Mm_isSymb;
			if (numSoftJoints) {
				par_K_num.resize(numSoftJoints*K_order);
				par_D_num.resize(numSoftJoints*D_order);
				par_Dm_num.resize(numSoftJoints*Dm_order);
				par_Mm_num.resize(numSoftJoints);
				for (int i = 0; i < numSoftJoints; i++) {
					// parameters of each elastic joint, by node name
					const string& jointName = softJointsName[i];
					const YAML::Node joint_cfg = config_["joints"][jointName];
					if (!joint_cfg) throw std::runtime_error("No entry in soft_loader 'joints' for elastic joint '" + jointName + "'.");

					// Helper lambda to parse a symbolic vector
					auto parse_symb_vector = [&](const string& key, int order) {
						vector<short> vec;
						if (joint_cfg[key]){
							vec = joint_cfg[key].as<vector<short>>();
							vec.resize(order);
						} else vec.assign(order, 0);
						return vec;
					};

					// - Numeric - //
					// stiffness
					if (K_order > 0){
						vector<double> K = joint_cfg["K"].as<vector<double>>();
						for (int j=0; j<K_order; j++) par_K_num[K_order*i+j] = K[j];
					}
					// coupling friction
					if (D_order > 0){
						vector<double> D = joint_cfg["D"].as<vector<double>>();
						for (int j=0; j<D_order; j++) par_D_num[D_order*i + j] = D[j];
					}
					// motor friction
					if (Dm_order > 0){
						vector<double> Dm = joint_cfg["Dm"].as<vector<double>>();
						for (int j=0; j<Dm_order; j++) par_Dm_num[Dm_order*i + j] = Dm[j];
					}
					// motor inertia
					par_Mm_num[i] = joint_cfg["Mm"].as<double>();
					
					// - Symbolic selectivity - //
					vector<short> K_symb = parse_symb_vector("K_symb", K_order);
					vector<short> D_symb = parse_symb_vector("D_symb", D_order);
					vector<short> Dm_symb = parse_symb_vector("Dm_symb", Dm_order);
					short Mm_symb = joint_cfg["Mm_symb"] ? joint_cfg["Mm_symb"].as<int>() : 0;
					if (K_order > 0) par_K_isSymb.insert(par_K_isSymb.end(), K_symb.begin(), K_symb.end());
					if (D_order > 0) par_D_isSymb.insert(par_D_isSymb.end(), D_symb.begin(), D_symb.end());
					if (Dm_order > 0) par_Dm_isSymb.insert(par_Dm_isSymb.end(), Dm_symb.begin(), Dm_symb.end());
					par_Mm_isSymb.push_back(Mm_symb);
				}
				// - Models - //
				SX par_K_symb = SX::sym("par_K", numSoftJoints*K_order,1);
				SX par_D_symb = SX::sym("par_D", numSoftJoints*D_order,1);
				SX par_Dm_symb = SX::sym("par_Dm", numSoftJoints*Dm_order,1);
				SX par_Mm_symb = SX::sym("par_Mm", numSoftJoints,1);
				// - Add to parameters - //
				robot->add_parameter("par_K", par_K_symb, par_K_num, par_K_isSymb, "Coupling stiffness parameters", true);
				robot->add_parameter("par_D", par_D_symb, par_D_num, par_D_isSymb, "Coupling friction parameters", true);
				robot->add_parameter("par_Dm", par_Dm_symb, par_Dm_num, par_Dm_isSymb, "Motor friction parameters", true);
				robot->add_parameter("par_Mm", par_Mm_symb, par_Mm_num, par_Mm_isSymb, "Motor inertia parameters", true);
			}

		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
			return robot; // Indicate failure
		}

		debug_log("Loading finished", VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
