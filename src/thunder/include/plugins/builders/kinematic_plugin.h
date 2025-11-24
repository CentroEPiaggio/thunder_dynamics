#ifndef KIN_BUILDER_H
#define KIN_BUILDER_H

#include "../plugin_interfaces.h"
#include "../robot.h"
#include "../../library/robot.h"
#include "../../library/kinematics.h"
#include "../../library/dynamics.h"
#include "../../library/regressors.h"
#include "../../library/utils.h"
#include "../../library/userDefined.h"
#include <yaml-cpp/yaml.h>


namespace thunder_ns {


    class KinBuilder : public BaseBuilder {
        public:
        KinBuilder() : BaseBuilder("Kinematic_blilder", "Build kinematics and differential kinematics expressions, like Jacobians and Transform matrixes T.") {}
        
        int configure(const YAML::Node& config) override{
            debug_log("Configured", VERB_INFO);
            return 0;
        }

        void execute(std::shared_ptr<Robot> robot) override{
            debug_log("Starting kinematic computations", VERB_INFO);
            compute_kinematics(*robot, 1);
			compute_dynamics(*robot, 1);
			compute_regressors(*robot);
            debug_log("Kinematics computed", VERB_INFO);
        }

    };

} // namespace thunder_ns

#endif // KIN_BUILDER_H