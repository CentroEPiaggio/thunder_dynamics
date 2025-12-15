#ifndef STD_JOINT_LOADER_H
#define STD_JOINT_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class StdJointLoader : public BaseLoader {

		public:
			StdJointLoader() : BaseLoader("Standard joint Loader", "Load the joint type from the configuration file.") {}
		
			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};

	
} // namespace thunder_ns

#endif // STD_JOINT_LOADER_H
