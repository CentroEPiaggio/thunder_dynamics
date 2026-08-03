#ifndef KIN_LOADER_H
#define KIN_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class KinLoader : public BaseLoader {
		
		private:
			std::string robot_name;
			void parse_frame_parameterization(
				std::shared_ptr<Robot> robot,
				const YAML::Node& frame_node,
				const std::string& frame_prefix,
				casadi::SX& frame_expr,
				std::vector<std::string>& frame_args);

		public:
			KinLoader() : BaseLoader("Kinematics Loader", "Load the robot kinematics and structure.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // KIN_LOADER_H
