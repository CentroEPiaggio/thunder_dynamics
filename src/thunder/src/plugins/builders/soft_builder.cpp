#include "plugins/builders/soft_builder.h"

namespace thunder_ns {

    int SoftBuilder::compute_elastic(std::shared_ptr<Robot> robot){
		// parameters from robot
		int nj = robot->get<int>("ndof");
		int numSoftJoints = 0;
		if (robot->properties.count("numSoftJoints")) {
			numSoftJoints = robot->get<int>("numSoftJoints");
		} else {
			robot->add_property<int>("numSoftJoints", numSoftJoints, "int", "Number of soft joints", true);
		}

		if (numSoftJoints){
			vector<short> isSoftJoint = robot->get<vector<short>>("isSoftJoint");
			int K_order = 0;
			int D_order = 0;
			int Dm_order = 0;
			// - K_order - //
			if (robot->properties.count("K_order")) {
				K_order = robot->get<int>("K_order");
			} else {
				robot->add_property<int>("K_order", K_order, "int", "Number of soft joints", true);
			}
			// - D_order - //
			if (robot->properties.count("D_order")) {
				D_order = robot->get<int>("D_order");
			} else {
				robot->add_property<int>("D_order", D_order, "int", "Number of soft joints", true);
			}
			// - Dm_order - //
			if (robot->properties.count("Dm_order")) {
				Dm_order = robot->get<int>("Dm_order");
			} else {
				robot->add_property<int>("Dm_order", Dm_order, "int", "Number of soft joints", true);
			}
			const auto& q = robot->get_model("q");
			const auto& x = robot->get_model("x");
			const auto& dq = robot->get_model("dq");
			const auto& dx = robot->get_model("dx");
			const auto& par_K = robot->get_model("par_K");
			const auto& par_D = robot->get_model("par_D");
			const auto& par_Dm = robot->get_model("par_Dm");
			const auto& par_Mm = robot->get_model("par_Mm");

			// casadi::SX K(numSoftJoints,1);
			// casadi::SX D(numSoftJoints,1);
			// casadi::SX Dm(numSoftJoints,1);
			// casadi::SX Mm(numSoftJoints,1);
			// for (int i=0; i<numSoftJoints; i++){
			// 	for (int ord=0; ord<K_order; ord++){
			// 		K(i) += pow(x(i)-q(i), ord+1) * par_K(i*K_order+ord);
			// 	}
			// 	for (int ord=0; ord<D_order; ord++){
			// 		D(i) += pow(dx(i)-dq(i), ord+1) * par_D(i*D_order+ord);
			// 	}
			// 	for (int ord=0; ord<Dm_order; ord++){
			// 		Dm(i) += pow(dx(i), ord+1) * par_Dm(i*Dm_order+ord);
			// 	}
			// }
			// function vectors
			casadi::SX k(numSoftJoints,1);
			casadi::SX d(numSoftJoints,1);
			casadi::SX dm(numSoftJoints,1);
			// matrixes
			std::vector<casadi::SX> K_vec(K_order);
			std::vector<casadi::SX> D_vec(D_order);
			std::vector<casadi::SX> Dm_vec(Dm_order);
			// casadi::SX K(numSoftJoints,numSoftJoints);
			// casadi::SX D(numSoftJoints,numSoftJoints);
			// casadi::SX Dm(numSoftJoints,numSoftJoints);
			casadi::SX Mm(numSoftJoints,numSoftJoints);
			for (int i=0; i<numSoftJoints; i++){
				for (int ord=0; ord<K_order; ord++){
					k(i) += pow(x(i)-q(i), 2*ord+1) * par_K(i*K_order+ord);	// ^1,3,5...
					K_vec[ord].resize(numSoftJoints,numSoftJoints);
					K_vec[ord](i,i) = par_K(i*K_order + ord);
				}
				for (int ord=0; ord<D_order; ord++){
					if (ord%2 == 0){
						d(i) += pow(dx(i)-dq(i), ord+1) * par_D(i*D_order+ord);
					} else {
						d(i) += sqrt(pow(dx(i)-dq(i), 2)) * pow(dx(i)-dq(i), ord) * par_D(i*D_order+ord);
					}
					D_vec[ord].resize(numSoftJoints,numSoftJoints);
					D_vec[ord](i,i) = par_D(i*D_order + ord);
				}
				for (int ord=0; ord<Dm_order; ord++){
					if (ord%2 == 0){
						dm(i) += pow(dx(i), ord+1) * par_Dm(i*Dm_order+ord);
					} else {
						dm(i) += sqrt(pow(dx(i), 2)) * pow(dx(i), ord) * par_Dm(i*Dm_order+ord);
					}
					Dm_vec[ord].resize(numSoftJoints,numSoftJoints);
					Dm_vec[ord](i,i) = par_Dm(i*Dm_order + ord);
				}
				Mm(i,i) = par_Mm(i);
			}
			std::vector<std::string> arg_list;
			if (K_order > 0) {
				arg_list = {"q", "x", "par_K"};
				robot->add_function("k", k, arg_list, "SEA manipulator elastic coupling");
				arg_list = {"par_K"};
				for (int ord=0; ord<K_order; ord++){ 
					robot->add_function("K"+std::to_string(2*ord+1), K_vec[ord], arg_list, "SEA manipulator elastic coupling, order "+std::to_string(2*ord+1));
				}
			}
			if (D_order > 0) {
				arg_list = {"dq", "dx", "par_D"};
				robot->add_function("d", d, arg_list, "SEA manipulator dampind coupling");
				arg_list = {"par_D"};
				for (int ord=0; ord<D_order; ord++){ 
					robot->add_function("D"+std::to_string(ord+1), D_vec[ord], arg_list, "SEA manipulator damping coupling, order "+std::to_string(ord+1));
				}
			}
			if (Dm_order > 0) {
				arg_list = {"dx", "par_Dm"};
				robot->add_function("dm", dm, arg_list, "SEA manipulator motor damping");
				arg_list = {"par_Dm"};
				for (int ord=0; ord<Dm_order; ord++){ 
					robot->add_function("Dm"+std::to_string(ord+1), Dm_vec[ord], arg_list, "SEA manipulator motor damping, order "+std::to_string(ord+1));
				}
			}
			arg_list = {"par_Mm"};
			robot->add_function("Mm", Mm, arg_list, "SEA manipulator motor inertia");

			return 1;
		} else return 0;
	}

    void SoftBuilder::build(std::shared_ptr<Robot> robot) {
        debug_log("Starting soft-robotics computations", VERB_INFO);
        compute_elastic(robot);
        debug_log("Soft-robotics computed", VERB_INFO);
    }

}