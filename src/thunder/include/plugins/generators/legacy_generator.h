#ifndef LEGACY_GEN_H
#define LEGACY_GEN_H

#include "plugin_interfaces.h"

using std::string;


namespace thunder_ns {

	class LegacyGenerator : public BaseGenerator {
		
		private:
			bool GEN_CASADI;		// generate casadi functions
			bool GEN_PYTHON;		// generate python bindings
			bool COPY_GEN;			// used to copy generated files into thunder_robot project

			void init();
			int copy_to(string robot_name, string path_from, string path_conf, string path_par, string path_h, string path_cpp);
			int update_cmake(const string from_robot, const string to_robot, const string file_path);

		public:
			LegacyGenerator() : BaseGenerator("Legacy Generator", "Generates an Eigen C++ library for Robot, with python bindings and casadi optional") {}
			
			void generate(const std::shared_ptr<Robot> robot) override;

	};


} // namespace thunder_ns

#endif // LEGACY_GEN_H
