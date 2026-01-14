#ifndef KIN_LOADER_H
#define KIN_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class KinLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			KinLoader() : BaseLoader("Kinematics Loader", "Load the robot kinematics and structure.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // KIN_LOADER_H
