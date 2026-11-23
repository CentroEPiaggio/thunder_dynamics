#ifndef LOAD_LIMITS_H
#define LOAD_LIMITS_H

#include <memory>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "robot.h"

namespace thunder_ns {

	/**
	 * @brief Joint limits, one entry per joint variable (order of q).
	 *
	 * Added to the robot as the symbolic parameters q_min, q_max, dq_max, ddq_max, tau_max, dtau_max.
	 * A limit that is not given is -inf (q_min) or +inf (the others): no limit.
	 */
	struct JointLimits {
		std::vector<double> q_min, q_max, dq_max, ddq_max, tau_max, dtau_max;
	};

	/// No limits for ndof joints: q_min = -inf, all the others +inf.
	JointLimits no_limits(int ndof);

	/**
	 * @brief Add the joint limits to the robot, after overwriting them with the configuration.
	 *
	 * Used by every loader that creates q: kin_loader and dh_loader start from no_limits(), urdf_loader from the
	 * <limit> of the URDF joints. Each key of config["limits"] (q_min, q_max, dq_max, ddq_max, tau_max, dtau_max)
	 * replaces that limit; it takes ndof values (.inf and -.inf allowed).
	 *
	 * @param robot robot
	 * @param ndof size of q
	 * @param config the loader configuration
	 * @param limits initial limits, ndof entries each
	 * @param loader name of the calling loader, for the error messages
	 * @throws std::runtime_error on unknown keys, wrong sizes, q_min > q_max or a negative maximum
	 */
	void load_limits(std::shared_ptr<Robot> robot, int ndof, const YAML::Node& config, JointLimits limits, const std::string& loader);

} // namespace thunder_ns

#endif // LOAD_LIMITS_H
