#ifndef SOFT_LOADER_H
#define SOFT_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class SoftLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			SoftLoader() : BaseLoader("Soft-Robots Loader", "Load elastic joint structure.") {}

			void create_elastic_joints(std::shared_ptr<Robot> robot);

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // SOFT_LOADER_H
