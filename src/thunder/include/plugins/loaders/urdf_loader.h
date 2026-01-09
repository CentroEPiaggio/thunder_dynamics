#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class UrdfLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			UrdfLoader() : BaseLoader("URDF Loader", "Load a robot from URDF file.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // URDF_LOADER_H
