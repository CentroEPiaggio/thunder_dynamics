#ifndef DYN_BUILDER_H
#define DYN_BUILDER_H

#include "plugin_interfaces.h"
#include "plugins/builders/common/dynamics.h"


namespace thunder_ns {

	class DynBuilder : public BaseBuilder {

		public:
			DynBuilder() : BaseBuilder("Dynamics Builder", "Build the robot dynamics.") {}

			void build(std::shared_ptr<Robot> robot) override {
				debug_log("Starting dynamic computations", VERB_INFO);
				compute_dynamics(*robot, 1);
				debug_log("Dynamics computed", VERB_INFO);
			}
			
	};

	
} // namespace thunder_ns

#endif // DYN_BUILDER_H