#include "plugins/builders/example_builder.h"
#include "utils.h"

#include <yaml-cpp/yaml.h>

using std::string;
using std::vector;
using casadi::SX;

namespace thunder_ns {

	int ExampleBuilder::compute_userDefined(std::shared_ptr<Robot> robot) {

		// --------------------- //
		// ----- YOUR CODE ----- //
		// --------------------- //

		// if (compute_example_fun(robot)) return 1;

		return 0;

	}

	int ExampleBuilder::compute_example_fun(std::shared_ptr<Robot> robot) {

		// ----- minimal example, to try: 
			// - uncomment the function into compute_userDefined(std::shared_ptr<Robot> robot)
			// - copy 'q0: [...]' into yaml file

		// --- parameters from robot --- //
		int numJoints = robot->get<int>("numJoints");
		// - use <robot>.get_model("<term>") to access the model term you want
		auto q = robot->get_model("q");

		// --- parameters from yaml --- //
		if (robot->config_yaml["q0"]){
			// --- take q0 from yaml --- //
			std::vector<double> q0_vect = robot->config_yaml["q0"].as<std::vector<double>>();
			casadi::SX q0(numJoints,1);
			for (int i=0; i<numJoints; i++){
				q0(i) = q0_vect[i];
			}

			// --- compute the distance --- //
			casadi::SX q0_dist = casadi::SX::norm_2(q-q0);

			// --- save function into robot --- //
			std::vector<std::string> arg_list = {"q", "q0"};	// valid also with model terms
			if (!robot->add_function("q0_dist", q0_dist, arg_list, "distance from q0")) return 0;
			// when you create the library, the function "get_q_dist()" into thunder_<robot> will return the distance from q0
		}
		
		return 1;
		// - after robot->add_function the new expression is added to the robot->model and can be reused in the same way
	}

	void ExampleBuilder::build(std::shared_ptr<Robot> robot) {
		debug_log("Starting example computations", VERB_INFO);

		compute_userDefined(robot);

		// return ret;
		debug_log("Example computed", VERB_INFO);
	}

}