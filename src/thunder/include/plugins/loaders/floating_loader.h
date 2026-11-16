#ifndef FLOATING_LOADER_H
#define FLOATING_LOADER_H

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "plugin_interfaces.h"

namespace thunder_ns {

	/**
	 * Declares a floating base: the root of the structure loaded before (urdf_loader, kin_loader, dh_loader, ...)
	 * moves freely in the world. Adds the variables pose_base, twist_base, acc_base, twist_base_r, acc_base_r,
	 * jerk_base and the properties floating = true, floating_twist, read by the builders through the helpers below.
	 *
	 *   pipeline:
	 *     loaders: [urdf_loader, floating_loader]
	 *   floating_loader:
	 *     base: base_Link      # optional, the root of the structure; only the root can float for now
	 *     twist: world         # world (default): twist_base = [v; w] in world axes, body: in base axes
	 */
	class FloatingLoader : public BaseLoader {
		public:
			FloatingLoader() : BaseLoader("Floating base Loader", "Make the root of the structure a floating base.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot) override;
	};

	/**
	 * Helpers for the builders. The floating base is the body on frame -1 (the root of the structure), with
	 *   pose_base  = [p; quat]   pose of the base frame in the world, quaternion xyzw (unit, Hamilton)
	 *   twist_base = [v; w]      velocity of the base origin and angular velocity, in world axes (twist: world,
	 *                            default) or in base axes (twist: body, i.e. [R^T dp; R^T w], as Pinocchio)
	 *   acc_base   = [dv; dw]    time derivative of the twist_base components (world: classical acceleration;
	 *                            body: body acceleration, not the classical one)
	 * plus twist_base_r, acc_base_r (Slotine-Li reference) and jerk_base (second time derivatives).
	 *
	 * Builders work on generalized vectors u = [twist_base; dq] of size nv = 6 + ndof and give outputs stacked
	 * [base; joints]. Without floating_loader the base has no dofs: every helper returns the joint quantity
	 * (u = dq, nv = ndof), the still world, or nothing for the base block, so the builders have one code path.
	 * The base enters the builders as a 6-dof joint at the root with motion subspace S_b (base velocity in base
	 * coordinates = S_b twist_base): S_b = I for a body twist, blkdiag(R^T, R^T) for a world twist. S_b and the
	 * terms that come from it are written in closed form per convention. Derivations: docs/floating_base.md.
	 */
	namespace floating {
		bool is_floating(std::shared_ptr<Robot> robot);
		bool body_twist(std::shared_ptr<Robot> robot);				// twist_base in base axes (floating_loader twist: body)
		int nv(std::shared_ptr<Robot> robot);						// size of u: 6 + ndof, or ndof

		// generalized vectors
		SX velocity(std::shared_ptr<Robot> robot);					// u   = [twist_base; dq]
		SX acceleration(std::shared_ptr<Robot> robot);				// du  = [acc_base; ddq]
		SX jerk(std::shared_ptr<Robot> robot);						// ddu = [jerk_base; d3q]
		SX ref_velocity(std::shared_ptr<Robot> robot);				// ur  = [twist_base_r; dqr]
		SX ref_acceleration(std::shared_ptr<Robot> robot);			// dur = [acc_base_r; ddqr]
		std::pair<SX, SX> split(std::shared_ptr<Robot> robot, const SX& u);		// {base rows, joint rows} of u (base 0 x 1 if fixed)
		SX stack_rows(std::shared_ptr<Robot> robot, const SX& base, const SX& joints);		// [base; joints]
		SX stack_cols(std::shared_ptr<Robot> robot, const SX& base, const SX& joints);		// [base, joints]
		SX stack_blocks(std::shared_ptr<Robot> robot, const SX& bb, const SX& bj, const SX& jj);	// [bb, bj; bj^T, jj]

		// function arguments: each base variable right before its joint counterpart (q: pose_base, dq: twist_base,
		// ddq: acc_base, d3q: jerk_base, dqr: twist_base_r, ddqr: acc_base_r)
		std::vector<std::string> add_base_args(std::shared_ptr<Robot> robot, const std::vector<std::string>& list);

		// base pose: identity if fixed
		SX R_base(std::shared_ptr<Robot> robot);
		SX T_base(std::shared_ptr<Robot> robot);

		// base motion subspace in world coordinates at the base origin (kinematics), and its time derivative:
		// world twist I, 0; body twist blkdiag(R, R), blkdiag(w^ R, w^ R) (w in world axes); 6 x 0 if fixed
		SX S_world(std::shared_ptr<Robot> robot);
		SX dS_world(std::shared_ptr<Robot> robot);
		// base twist [v; w] of the base origin in world axes, S_world twist_base; zeros if fixed
		SX twist_world(std::shared_ptr<Robot> robot);
		// S_b m: base rows (twist, acceleration) to base coordinates; zeros(6, cols) if fixed
		SX motion(std::shared_ptr<Robot> robot, const SX& m);
		// S_b^T f: forces in base coordinates to base rows; 0 x cols if fixed
		SX force(std::shared_ptr<Robot> robot, const SX& f);
		// velocity terms of the base acceleration in base coordinates, d/dt(S_b) ur_b + v x vr, with v = S_b u_b and
		// vr = S_b ur_b (empty vr: plain RNEA, vr = v): world twist [v_lin x vr_ang; 0], body twist v x vr; 0 if fixed
		SX bias(std::shared_ptr<Robot> robot, const SX& v, const SX& vr);

		// d/dt expr along the motion. Configuration z = [pose_base; q] moves with dz = [dp; dquat; dq]: world twist
		// dp = v, dquat = 1/2 [w; 0] (x) quat; body twist dp = R v, dquat = 1/2 quat (x) [w; 0].
		// Velocities u move with du, accelerations du with ddu. Exact for any state (no chart).
		SX time_derivative(std::shared_ptr<Robot> robot, const SX& expr);
	}

} // namespace thunder_ns

#endif // FLOATING_LOADER_H
