#ifndef DYNTREE_LOADER_H
#define DYNTREE_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class DynTreeLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			DynTreeLoader() : BaseLoader("Dynamics Loader", "Load the robot dynamics, with trees.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // DYNTREE_LOADER_H
