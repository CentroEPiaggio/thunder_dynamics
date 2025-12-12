#ifndef STD_JOINT_BUILDER_H
#define STD_JOINT_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class StdJointBuilder : public BaseBuilder {

		public:
			StdJointBuilder() : BaseBuilder("Standard Joint Builder", "Build joint functions to create the kinematics.") {}

			void build(std::shared_ptr<Robot> robot) override;
			
	};

	
} // namespace thunder_ns

#endif // KIN_BUILDER_H