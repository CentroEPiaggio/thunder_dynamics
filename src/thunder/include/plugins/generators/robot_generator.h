#ifndef ROBOT_GEN_H
#define ROBOT_GEN_H

#include "plugin_interfaces.h"

using std::string;


namespace thunder_ns {

	class RobotGenerator : public BaseGenerator {
		
		private:
			bool GEN_CASADI = false;		// generate casadi functions
			bool GEN_PYTHON = false;		// generate python bindings
			bool GEN_ROBOT = true;			// generate thunder_<robot> class
			bool COPY_GEN = false;			// used to copy generated files into thunder_robot project

			int copy_to(string robot_name, string path_from, string path_conf, string path_par, string path_h, string path_cpp);
			int update_template(const string from_robot, const string to_robot, const string file_path);

		public:
			RobotGenerator() : BaseGenerator("Robot Generator", "Generates an Eigen C++ library for Robot, with python bindings and casadi optional") {}
			
			void init();

			void generate(const std::shared_ptr<Robot> robot) override;

	};


} // namespace thunder_ns

#endif // ROBOT_GEN_H
