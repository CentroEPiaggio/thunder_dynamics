#include "plugins/loaders/floating_loader.h"

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
		robot->add_property<string>("floating_twist", twist, "string", "Axes of twist_base: world or body", true);

		debug_log("Floating base: " + base + ", twist in " + axes, VERB_INFO);
		return robot;
	}

	// --- Helpers for the builders (floating_loader.h) --- //
	namespace floating {

		bool is_floating(std::shared_ptr<Robot> robot){
			return robot->properties.count("floating") && robot->get<bool>("floating");
		}

		bool body_twist(std::shared_ptr<Robot> robot){
			return is_floating(robot) && robot->get<string>("floating_twist") == "body";
		}

		int nv(std::shared_ptr<Robot> robot){
			return robot->get<int>("ndof") + (is_floating(robot) ? 6 : 0);
		}

		// --- Generalized vectors --- //
		namespace {
			SX stacked(std::shared_ptr<Robot> robot, const string& base, const string& joints){
				SX q_joints = robot->get_model(joints);
				return is_floating(robot) ? SX::vertcat({robot->get_model(base), q_joints}) : q_joints;
			}
		}

		SX velocity(std::shared_ptr<Robot> robot){ return stacked(robot, "twist_base", "dq"); }
		SX acceleration(std::shared_ptr<Robot> robot){ return stacked(robot, "acc_base", "ddq"); }
		SX jerk(std::shared_ptr<Robot> robot){ return stacked(robot, "jerk_base", "d3q"); }
		SX ref_velocity(std::shared_ptr<Robot> robot){ return stacked(robot, "twist_base_r", "dqr"); }
		SX ref_acceleration(std::shared_ptr<Robot> robot){ return stacked(robot, "acc_base_r", "ddqr"); }

		std::pair<SX, SX> split(std::shared_ptr<Robot> robot, const SX& u){
			if (!is_floating(robot)) return {SX(0, 1), u};
			return {u(casadi::Slice(0, 6)), u(casadi::Slice(6, u.size1()))};
		}

		SX stack_rows(std::shared_ptr<Robot> robot, const SX& base, const SX& joints){
			return is_floating(robot) ? SX::vertcat({base, joints}) : joints;
		}

		SX stack_cols(std::shared_ptr<Robot> robot, const SX& base, const SX& joints){
			return is_floating(robot) ? SX::horzcat({base, joints}) : joints;
		}

		SX stack_blocks(std::shared_ptr<Robot> robot, const SX& bb, const SX& bj, const SX& jj){
			return is_floating(robot) ? SX::vertcat({SX::horzcat({bb, bj}), SX::horzcat({bj.T(), jj})}) : jj;
		}

		// --- Function arguments --- //
		std::vector<std::string> add_base_args(std::shared_ptr<Robot> robot, const std::vector<std::string>& list){
			if (!is_floating(robot)) return list;
			static const std::map<string, string> base_of = {{"q", "pose_base"}, {"dq", "twist_base"}, {"ddq", "acc_base"},
			                                                  {"d3q", "jerk_base"}, {"dqr", "twist_base_r"}, {"ddqr", "acc_base_r"}};
			std::vector<std::string> out;
			for (const auto& name : list){
				if (base_of.count(name)) out.push_back(base_of.at(name));
				out.push_back(name);
			}
			return out;
		}

		// --- Base pose --- //
		SX R_base(std::shared_ptr<Robot> robot){
			if (!is_floating(robot)) return SX::eye(3);
			return quat2rot(robot->get_model("pose_base")(casadi::Slice(3,7)));
		}

		SX T_base(std::shared_ptr<Robot> robot){
			SX T = SX::eye(4);
			if (!is_floating(robot)) return T;
			T(lin, lin) = R_base(robot);
			T(lin, 3) = robot->get_model("pose_base")(lin);
			return T;
		}

		// --- Base motion model, closed form per convention (docs/floating_base.md) --- //
		SX S_world(std::shared_ptr<Robot> robot){
			if (!is_floating(robot)) return SX(6, 0);
			if (!body_twist(robot)) return SX::eye(6);
			SX R = R_base(robot);
			return SX::diagcat({R, R});
		}

		SX dS_world(std::shared_ptr<Robot> robot){
			if (!is_floating(robot)) return SX(6, 0);
			if (!body_twist(robot)) return SX::zeros(6, 6);
			SX R = R_base(robot);
			SX w_hat_R = SX::mtimes(hat(SX::mtimes(R, robot->get_model("twist_base")(ang))), R);	// w in world axes
			return SX::diagcat({w_hat_R, w_hat_R});
		}

		SX twist_world(std::shared_ptr<Robot> robot){
			if (!is_floating(robot)) return SX::zeros(6, 1);
			return SX::mtimes(S_world(robot), robot->get_model("twist_base"));
		}

		SX motion(std::shared_ptr<Robot> robot, const SX& m){
			if (!is_floating(robot)) return SX::zeros(6, m.size2());
			if (body_twist(robot)) return m;
			SX Rt = R_base(robot).T();
			return SX::vertcat({SX::mtimes(Rt, m(lin, casadi::Slice())), SX::mtimes(Rt, m(ang, casadi::Slice()))});
		}

		SX force(std::shared_ptr<Robot> robot, const SX& f){
			if (!is_floating(robot)) return SX(0, f.size2());
			if (body_twist(robot)) return f;
			SX R = R_base(robot);
			return SX::vertcat({SX::mtimes(R, f(lin, casadi::Slice())), SX::mtimes(R, f(ang, casadi::Slice()))});
		}

		SX bias(std::shared_ptr<Robot> robot, const SX& v, const SX& vr){
			if (!is_floating(robot)) return SX::zeros(6, 1);
			const bool plain = vr.is_empty();
			if (body_twist(robot)) {
				// S_b = I: only the self term of the base block, v x vr (zero in the plain RNEA)
				if (plain) return SX::zeros(6, 1);
				return SX::vertcat({SX::cross(v(ang), vr(lin)) + SX::cross(v(lin), vr(ang)), SX::cross(v(ang), vr(ang))});
			}
			// world twist: d/dt(S_b) ur_b = -[w x vr_lin; w x vr_ang] cancels the angular parts of v x vr
			const SX& vr_ = plain ? v : vr;
			return SX::vertcat({SX::cross(v(lin), vr_(ang)), SX::zeros(3, 1)});
		}

		// --- Time derivative along the motion --- //
		SX time_derivative(std::shared_ptr<Robot> robot, const SX& expr){
			SX q = robot->get_model("q"), dq = robot->get_model("dq");
			SX z = q, dz = dq;
			if (is_floating(robot)){
				SX pose = robot->get_model("pose_base"), twist = robot->get_model("twist_base");
				SX quat = pose(casadi::Slice(3,7)), w = SX::vertcat({twist(ang), 0});
				const bool body = body_twist(robot);
				SX dp = body ? SX::mtimes(R_base(robot), twist(lin)) : twist(lin);
				SX dquat = body ? 0.5*quat_mult(quat, w) : 0.5*quat_mult(w, quat);	// body: right product, world: left
				z = SX::vertcat({pose, q});
				dz = SX::vertcat({dp, dquat, dq});
			}
			SX u = velocity(robot), du = acceleration(robot), ddu = jerk(robot);
			return SX::jtimes(expr, z, dz) + SX::jtimes(expr, u, du) + SX::jtimes(expr, du, ddu);
		}

	}

} // namespace thunder_ns
