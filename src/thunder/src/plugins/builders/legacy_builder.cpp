#include "plugins/builders/legacy_builder.h"

namespace thunder_ns {
    using namespace legacy;

    void LegacyBuilder::build(std::shared_ptr<Robot> robot) {
        debug_log("Starting kinematic computations", VERB_INFO);
        compute_kinematics(*robot, 1);
        debug_log("Kinematics computed", VERB_INFO);
        compute_dynamics(*robot, 1);
        debug_log("Dynamics computed", VERB_INFO);
        compute_regressors(*robot);
        debug_log("Regressors computed", VERB_INFO);
        compute_userDefined(*robot);
        debug_log("User functions computed", VERB_INFO);
    }

}