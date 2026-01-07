#ifndef REG_BUILDER_H
#define REG_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class RegBuilder : public BaseBuilder {

		public:
			RegBuilder() : BaseBuilder("Regressors Builder", "Build the kinematic and dynamic regressors.") {}

			void init(std::shared_ptr<Robot> robot);

			casadi::SXVector createQ();
			casadi::SXVector createE();
			int compute_Yr(std::shared_ptr<Robot> robot);
			int compute_reg_Dl(std::shared_ptr<Robot> robot);
			int compute_reg_elastic(std::shared_ptr<Robot> robot);
			int compute_reg_J(std::shared_ptr<Robot> robot);

			int compute_regressors(std::shared_ptr<Robot> robot, bool advanced=1);
		
			void build(std::shared_ptr<Robot> robot) override;

	};


} // namespace thunder_ns

#endif // REG_BUILDER_H