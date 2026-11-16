#ifndef FLOATING_BASE_H
#define FLOATING_BASE_H

#include <memory>
#include <string>
#include <vector>

#include "robot.h"

namespace thunder_ns {

	/**
	 * Floating base, declared by floating_loader. The base is the body on frame -1 (the root of the structure), with
	 *   pose_base  = [p; quat]   pose of the base frame in the world, quaternion xyzw (unit, Hamilton)
	 *   twist_base = [v; w]      velocity of the base origin and angular velocity, in world axes (twist: world,
	 *                            default) or in base axes (twist: body, i.e. [R^T dp; R^T w], as Pinocchio)
	 *   acc_base   = [dv; dw]    time derivative of the twist_base components (world: classical acceleration;
	 *                            body: body acceleration, not the classical one)
	 * plus twist_base_r, acc_base_r (Slotine-Li reference) and jerk_base (second time derivatives).
	 *
	 * Builders work on generalized vectors u = [twist_base; dq] of size nv = 6 + ndof and give outputs stacked
	 * [base; joints]. Without a floating base every helper returns the joint quantity (u = dq, nv = ndof, ...),
	 * so the builders have one code path. Derivations: docs/floating_base.md.
	 */
	namespace floating {
		bool is_floating(std::shared_ptr<Robot> robot);
		bool body_twist(std::shared_ptr<Robot> robot);				// twist_base in base axes (floating_loader twist: body)
		int offset(std::shared_ptr<Robot> robot);					// first joint entry of u: 6 or 0
		int nv(std::shared_ptr<Robot> robot);						// size of u: ndof + offset

		SX velocity(std::shared_ptr<Robot> robot);					// u   = [twist_base; dq]
		SX acceleration(std::shared_ptr<Robot> robot);				// du  = [acc_base; ddq]
		SX jerk(std::shared_ptr<Robot> robot);						// ddu = [jerk_base; d3q]
		SX ref_velocity(std::shared_ptr<Robot> robot);				// ur  = [twist_base_r; dqr]
		SX ref_acceleration(std::shared_ptr<Robot> robot);			// dur = [acc_base_r; ddqr]

		// function arguments: list, plus base_vars when the base is floating
		std::vector<std::string> args(std::shared_ptr<Robot> robot, std::vector<std::string> list, const std::vector<std::string>& base_vars);

		SX R_base(std::shared_ptr<Robot> robot);					// rotation of pose_base, identity if fixed
		SX T_base(std::shared_ptr<Robot> robot);					// transform of pose_base, identity if fixed

		// d/dt expr along the motion. Configuration z = [pose_base; q] moves with dz = [dp; dquat; dq]: world twist
		// dp = v, dquat = 1/2 [w; 0] (x) quat; body twist dp = R v, dquat = 1/2 quat (x) [w; 0].
		// Velocities u move with du, accelerations du with ddu. Exact for any state (no chart).
		SX time_derivative(std::shared_ptr<Robot> robot, const SX& expr);
	}

	SX quat_mult(const SX& a, const SX& b);		// Hamilton product, xyzw
	SX quat2rot(const SX& quat);				// rotation matrix of a unit quaternion, xyzw

}

#endif
