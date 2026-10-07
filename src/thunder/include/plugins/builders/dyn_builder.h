#ifndef DYN_BUILDER_H
#define DYN_BUILDER_H

#include "plugin_interfaces.h"


namespace thunder_ns {

	class DynBuilder : public BaseBuilder {

		public:
			DynBuilder() : BaseBuilder("Dynamics Builder", "Build the robot dynamics.") {}

			std::tuple<casadi::SXVector,casadi::SXVector, casadi::SXVector> createInertialParameters(int nj, int nParLink, casadi::SX);
			casadi::SX dq_select(const casadi::SX& dq_);
			casadi::SX stdCmatrix(const casadi::SX& B, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
			casadi::SX stdCmatrix_classic(const casadi::SX& M, const casadi::SX& q_, const casadi::SX& dq_, const casadi::SX& dq_sel_);
			std::tuple<casadi::SXVector,casadi::SXVector> DHJacCM(std::shared_ptr<Robot> robot);
			int compute_dyn_lagrange(std::shared_ptr<Robot> robot);
			casadi::SX joint_subspace(std::shared_ptr<Robot> robot, const std::string& type, const casadi::SX& q_joint, const casadi::SX& axis);
			casadi::SX rnea(std::shared_ptr<Robot> robot, const casadi::SX& dq, const casadi::SX& dqr, const casadi::SX& ddqr, const casadi::SX& g);
			int compute_dyn_rnea(std::shared_ptr<Robot> robot);
			int add_dyn(std::shared_ptr<Robot> robot, const casadi::SX& M, const casadi::SX& Cdq, const casadi::SX& G);
			int compute_C(std::shared_ptr<Robot> robot, const std::string& C_method);
			int compute_C_std(std::shared_ptr<Robot> robot);
			int compute_J_cm(std::shared_ptr<Robot> robot);
			int compute_Dl(std::shared_ptr<Robot> robot);
			int compute_dyn_derivatives(std::shared_ptr<Robot> robot);
			int compute_reg_dyn_conversions(std::shared_ptr<Robot> robot);

			void build(std::shared_ptr<Robot> robot) override;
			
	};

	
} // namespace thunder_ns

#endif // DYN_BUILDER_H