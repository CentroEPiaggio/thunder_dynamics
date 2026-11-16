#include "plugins/loaders/floating_loader.h"
#include "floating_base.h"

namespace thunder_ns {

	std::shared_ptr<Robot> FloatingLoader::load(std::shared_ptr<Robot> robot) {
		if (!robot->properties.count("jointsParent") || !robot->parameters.count("q"))
			throw std::runtime_error("floating_loader: no structure loaded, put it after urdf_loader, kin_loader or dh_loader");
		if (floating::is_floating(robot))
			throw std::runtime_error("floating_loader: the floating base is already declared");

		// the floating body is the one on frame -1: the root node (urdf_loader names it after the root link)
		const vector<int> jointsParent = robot->get<vector<int>>("jointsParent");
		const vector<string> jointsName = robot->get<vector<string>>("jointsName");
		const string root = jointsName.at(0);
		if (jointsParent.at(0) != -1)
			throw std::runtime_error("floating_loader: the first node '" + root + "' is not a root");
		const string base = config_["base"] ? config_["base"].as<string>() : root;
		// convention of twist_base and of its derivatives: world-aligned (default) or body
		const string twist = config_["twist"] ? config_["twist"].as<string>() : "world";
		if (twist != "world" && twist != "body")
			throw std::runtime_error("floating_loader: twist must be 'world' or 'body', not '" + twist + "'");
		if (base != root)
			throw std::runtime_error("floating_loader: base '" + base + "' is not the root '" + root + "' (only the root can float for now)");

		// pose_base is the pose of frame -1: a constant world-to-root frame would be a second, hidden base frame
		if (robot->functions.count("par_world2L0")) {
			const DM world2L0 = robot->get("par_world2L0");
			if (!robot->functions["par_world2L0"].args.empty() || casadi::DM::norm_inf(world2L0).scalar() != 0.0)
				throw std::runtime_error("floating_loader: Base_to_L0 must be the identity (the base pose is the state)");
		}

		robot->add_variable("pose_base", SX::sym("pose_base", 7, 1), {0, 0, 0, 0, 0, 0, 1}, {1}, "Base pose [p; quat xyzw] in the world", true);
		const string axes = (twist == "world") ? "world axes" : "base axes";
		robot->add_variable("twist_base", SX::sym("twist_base", 6, 1), vector<double>(6, 0), {1}, "Base twist [v; w] of the base origin, " + axes, true);
		robot->add_variable("acc_base", SX::sym("acc_base", 6, 1), vector<double>(6, 0), {1}, "Base acceleration, d/dt of the twist_base components", true);
		robot->add_variable("jerk_base", SX::sym("jerk_base", 6, 1), vector<double>(6, 0), {1}, "Base jerk, d/dt acc_base", true);
		robot->add_variable("twist_base_r", SX::sym("twist_base_r", 6, 1), vector<double>(6, 0), {1}, "Base reference twist", true);
		robot->add_variable("acc_base_r", SX::sym("acc_base_r", 6, 1), vector<double>(6, 0), {1}, "Base reference acceleration", true);
		robot->add_property<bool>("floating", true, "bool", "The root of the structure is a floating base", true);
		robot->add_property<string>("floating_base", base, "string", "Name of the floating base", true);
		robot->add_property<string>("floating_twist", twist, "string", "Axes of twist_base: world or body", true);

		debug_log("Floating base: " + base + ", twist in " + axes, VERB_INFO);
		return robot;
	}

} // namespace thunder_ns
