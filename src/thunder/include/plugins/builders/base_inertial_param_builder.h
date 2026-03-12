#ifndef BASE_INERTIAL_PARAM_BUILDER_H
#define BASE_INERTIAL_PARAM_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class BaseInertialParamBuilder : public BaseBuilder {

		public:
			BaseInertialParamBuilder() : BaseBuilder("BaseInertialParam Builder", "Build the Reduced set of inertial parameters.") {}

			void init(std::shared_ptr<Robot> robot);

			casadi::SX createS(std::shared_ptr<Robot> robot, int link);
			casadi::SX createLambda(const SX& a, const SX& alpha, const SX& d, const SX& theta);
			casadi::SX del_row(const SX& M, vector<int> elim_idx);
			casadi::SX del_col(const SX& M, vector<int> elim_idx);
			int compute_par_red(std::shared_ptr<Robot> robot);
			int compute_Ia(std::shared_ptr<Robot> robot);

			void build(std::shared_ptr<Robot> robot) override;
		private:
			bool has_r1 = false;
			bool has_r2 = false;
			bool has_p1 = false;
			bool has_rp1 = false;
			bool has_motor_inertia = false;
	};

	
} // namespace thunder_ns

#endif // EXAMPLE_BUILDER_H