#include "plugins/loaders/common/load_limits.h"

#include <limits>
#include <map>
#include <stdexcept>

namespace thunder_ns {

JointLimits no_limits(int ndof) {
	const double inf = std::numeric_limits<double>::infinity();
	JointLimits limits;
	limits.q_min.assign(ndof, -inf);
	limits.q_max.assign(ndof, inf);
	limits.dq_max.assign(ndof, inf);
	limits.ddq_max.assign(ndof, inf);
	limits.tau_max.assign(ndof, inf);
	limits.dtau_max.assign(ndof, inf);
	return limits;
}

void load_limits(std::shared_ptr<Robot> robot, int ndof, const YAML::Node& config, JointLimits limits, const std::string& loader) {
	const std::map<std::string, std::pair<std::vector<double>*, std::string>> entries = {
		{"q_min", {&limits.q_min, "Lower position limit of each joint"}},
		{"q_max", {&limits.q_max, "Upper position limit of each joint"}},
		{"dq_max", {&limits.dq_max, "Velocity limit of each joint"}},
		{"ddq_max", {&limits.ddq_max, "Acceleration limit of each joint"}},
		{"tau_max", {&limits.tau_max, "Effort limit of each joint"}},
		{"dtau_max", {&limits.dtau_max, "Effort rate limit of each joint"}},
	};

	// - overwrite with the configuration - //
	if (config["limits"]) {
		for (const auto& entry : config["limits"]) {
			const std::string key = entry.first.as<std::string>();
			if (!entries.count(key)) throw std::runtime_error(loader + ".limits: unknown limit '" + key + "' (q_min, q_max, dq_max, ddq_max, tau_max, dtau_max).");
			*entries.at(key).first = entry.second.as<std::vector<double>>();
		}
	}

	// - check - //
	for (const auto& [name, entry] : entries) {
		const std::vector<double>& values = *entry.first;
		if (static_cast<int>(values.size()) != ndof) {
			throw std::runtime_error(loader + ": limit '" + name + "' has " + std::to_string(values.size()) + " values, the robot has " + std::to_string(ndof) + " joints.");
		}
		const bool is_position = (name == "q_min" || name == "q_max");
		for (double value : values) {
			if (!is_position && value < 0) throw std::runtime_error(loader + ": limit '" + name + "' must be non-negative.");
		}
	}
	for (int i = 0; i < ndof; i++) {
		if (limits.q_min[i] > limits.q_max[i]) throw std::runtime_error(loader + ": q_min > q_max for joint " + std::to_string(i) + ".");
	}

	// - add the parameters, always symbolic (settable at runtime) - //
	for (const auto& [name, entry] : entries) {
		robot->add_parameter(name, casadi::SX::sym(name, ndof, 1), *entry.first, {1}, entry.second, true);
	}
}

} // namespace thunder_ns
