#ifndef KINTREE_LOADER_H
#define KINTREE_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class KinTreeLoader : public BaseLoader {
		
		private:
			std::string robot_name;

		public:
			KinTreeLoader() : BaseLoader("Kinematics Loader", "Load the robot kinematics and structure, with tree support.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // KINTREE_LOADER_H
