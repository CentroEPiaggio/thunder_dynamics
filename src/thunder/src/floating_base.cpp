#include "floating_base.h"

namespace thunder_ns {

	namespace {
		const casadi::Slice lin(0,3), ang(3,6);
	}

	SX quat_mult(const SX& a, const SX& b){
		SX av = a(lin), bv = b(lin), aw = a(3), bw = b(3);
		return SX::vertcat({aw*bv + bw*av + SX::cross(av, bv), aw*bw - SX::dot(av, bv)});
	}

	SX quat2rot(const SX& quat){
		// exact on |quat| = 1 only: the caller normalises
		SX v = quat(lin), w = quat(3);
		return (w*w - SX::dot(v, v))*SX::eye(3) + 2*SX::mtimes(v, v.T()) + 2*w*hat(v);
	}

	namespace floating {

		bool is_floating(std::shared_ptr<Robot> robot){
			return robot->properties.count("floating") && robot->get<bool>("floating");
		}

		bool body_twist(std::shared_ptr<Robot> robot){
			return is_floating(robot) && robot->get<string>("floating_twist") == "body";
		}

		int offset(std::shared_ptr<Robot> robot){
			return is_floating(robot) ? 6 : 0;
		}

		int nv(std::shared_ptr<Robot> robot){
			return robot->get<int>("ndof") + offset(robot);
		}

		namespace {
			SX stacked(std::shared_ptr<Robot> robot, const std::string& base, const std::string& joints){
				SX q_joints = robot->get_model(joints);
				return is_floating(robot) ? SX::vertcat({robot->get_model(base), q_joints}) : q_joints;
			}
		}

		SX velocity(std::shared_ptr<Robot> robot){ return stacked(robot, "twist_base", "dq"); }
		SX acceleration(std::shared_ptr<Robot> robot){ return stacked(robot, "acc_base", "ddq"); }
		SX jerk(std::shared_ptr<Robot> robot){ return stacked(robot, "jerk_base", "d3q"); }
		SX ref_velocity(std::shared_ptr<Robot> robot){ return stacked(robot, "twist_base_r", "dqr"); }
		SX ref_acceleration(std::shared_ptr<Robot> robot){ return stacked(robot, "acc_base_r", "ddqr"); }

		std::vector<std::string> args(std::shared_ptr<Robot> robot, std::vector<std::string> list, const std::vector<std::string>& base_vars){
			if (is_floating(robot)) list.insert(list.end(), base_vars.begin(), base_vars.end());
			return list;
		}

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

}
