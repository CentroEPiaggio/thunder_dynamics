#ifndef DH_LOADER_H
#define DH_LOADER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	casadi::SX DHTemplate(const casadi::SX& rowDHTable, const casadi::SX& qi, std::string jointType);

	class DHLoader : public BaseLoader {

		private:
			std::string robot_name;

		public:
			DHLoader() : BaseLoader("DH Loader", "Creates a robot structure using DH parameters (modified convention).") {}
		
			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};

	
} // namespace thunder_ns

#endif // DH_LOADER_H
