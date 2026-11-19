#ifndef ROS_CONTROL_GENERATOR_H
#define ROS_CONTROL_GENERATOR_H

#include "plugin_interfaces.h"

using std::string;


namespace thunder_ns {

	class ROSControlGenerator : public BaseGenerator {

		public:
			ROSControlGenerator() : BaseGenerator("ROS Control Generator", "Generates a ros2_control controller package (MACT) that links the robot library") {}

			void generate(const std::shared_ptr<Robot> robot) override;

	};


} // namespace thunder_ns

#endif // ROS_CONTROL_GENERATOR_H
