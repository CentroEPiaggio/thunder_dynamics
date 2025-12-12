#ifndef LEGACY_LOADER_H
#define LEGACY_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class LegacyLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			LegacyLoader() : BaseLoader("Legacy Loader", "Creates a robot with the old yaml structure.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // LEGACY_LOADER_H
