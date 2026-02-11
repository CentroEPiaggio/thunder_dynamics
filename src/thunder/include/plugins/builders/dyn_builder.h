#ifndef DYNTREE_BUILDER_H
#define DYNTREE_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class DynTreeBuilder : public BaseBuilder {

		public:
			DynTreeBuilder() : BaseBuilder("Dynamics Builder", "Build the robot dynamics, with trees.") {}

			std::tuple<casadi::SXVector,casadi::SXVector, casadi::SXVector> createInertialParameters(int nj, int nParLink, casadi::SX);
			casadi::SX dq_select(const casadi::SX& dq_);
			casadi::SX stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
			casadi::SX stdCmatrix_classic(const casadi::SX& M, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
			std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(std::shared_ptr<Robot> robot);
			int compute_MCG(std::shared_ptr<Robot> robot);
			int compute_Dl(std::shared_ptr<Robot> robot);
			int compute_dyn_derivatives(std::shared_ptr<Robot> robot);
			int compute_reg_dyn_conversions(std::shared_ptr<Robot> robot);

			void build(std::shared_ptr<Robot> robot) override;
			
	};

	
} // namespace thunder_ns

#endif // DYNTREE_BUILDER_H