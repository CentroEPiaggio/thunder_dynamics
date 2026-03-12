#ifndef DYN_LOADER_H
#define DYN_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class DynLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			DynLoader() : BaseLoader("Dynamics Loader", "Load the robot dynamics.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // DYN_LOADER_H
