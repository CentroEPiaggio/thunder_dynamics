#include "plugins/builders/std_joint_builder.h"
#include "utils.h"

using std::vector;
using std::string;

namespace thunder_ns {

	// --- Create joint functions --- //
	void StdJointBuilder::build(std::shared_ptr<Robot> robot) {
        // --- get properties --- //
		auto numJoints = robot->get<int>("numJoints");
        auto jointsType = robot->get<vector<string>>("jointsType");
		auto q = robot->get_model("q");
		casadi::Slice rot(0, 3);      // [0,1,2] indexes

		for (int i=0; i<numJoints; i++){
			casadi::SX Ti = casadi::SX::eye(4);
			
			if ((jointsType[i] == "P")||(jointsType[i] == "P_SEA")) {
				Ti(2,3) = q(i);
			}
			else if ((jointsType[i] == "R")||(jointsType[i] == "R_SEA")) {
				Ti(rot,rot) = R_z(q(i));
			}
			else {
				throw std::runtime_error("Error: joint type not available");
			}

			if (!robot->add_function("T_J"+std::to_string(i), Ti, {"q"}, "Joint "+std::to_string(i)+" transformation.")) {
				std::cerr << "Error adding joint function: " << "T_J"+std::to_string(i) << std::endl;
			}
		}
	}

} // namespace thunder_ns