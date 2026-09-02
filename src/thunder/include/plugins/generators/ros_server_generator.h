#ifndef ROS_SERVER_GENERATOR_H
#define ROS_SERVER_GENERATOR_H

#include "plugin_interfaces.h"

using std::string;


namespace thunder_ns {

	class ROSServerGenerator : public BaseGenerator {
		
		private:

		public:
			ROSServerGenerator() : BaseGenerator("ROS Server Generator", "Generates a ROS 2 package that exposes robot quantities through services and topics") {}
			


			void generate(const std::shared_ptr<Robot> robot) override;

	};


} // namespace thunder_ns

#endif // ROS_SERVER_GENERATOR_H