#ifndef C_GEN_H
#define C_GEN_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class CGenerator : public BaseGenerator {

		private:
			bool GEN_CASADI;		// generate casadi functions
			bool COPY_GEN;			// used to copy generated files into thunder_robot project

		public:
			CGenerator() : BaseGenerator("C Generator", "Generates a plain C library for Robot") {}
		
			void generate(const std::shared_ptr<Robot> robot) override;

	};

	
} // namespace thunder_ns

#endif // C_GEN_H
