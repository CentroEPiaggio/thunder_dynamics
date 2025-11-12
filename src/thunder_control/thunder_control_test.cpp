/* Test of classes and check validation of regressor compare standard equation of dynamic */

#include <iostream>
#include <string>
#include <casadi/casadi.hpp>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <filesystem>
#include <stdexcept>
#include <chrono>
// #include <yaml-cpp/yaml.h>

#include "thunder_control.h"
// #include "thunder_robot.h"
#include "thunder_RRR.h"
// #include "thunder_franka.h"
// #include "thunder_seaRRR.h"
// #include "thunder_egoArm.h"
// #include "thunder_frankaWrist.h"

const std::string conf_file = "../robots/RRR_conf.yaml";
// const std::string conf_file = "../robots/seaRRR_conf.yaml";
// const std::string conf_file = "../robots/franka_conf.yaml";
// const std::string conf_file = "../robots/egoArm_conf.yaml";
// const std::string conf_file = "../robots/frankaWrist_conf.yaml";
const std::string saved_inertial_file = "../robots/saved_robot_inertial_DYN.yaml";

using namespace std::chrono;
using std::cout;
using std::endl;

// bool use_gripper = false;

// Eigen::Matrix3d hat(const Eigen::Vector3d v);

int main(){

	int n_rep = 10;
	auto time_start = high_resolution_clock::now();
	auto time_stop = high_resolution_clock::now();
	auto duration = duration_cast<nanoseconds>(time_stop - time_start).count();

	// - setup robot - //
	thunder_RRR robot;
	robot.load_conf(conf_file);
	int nj = robot.get_numJoints();
	// - setup controller - //
	thunder_control<thunder_RRR> controller(&robot);
	controller.init();

	// - starting state - //
	VectorXd q(nj);
	VectorXd dq(nj);
	VectorXd dqr(nj);
	VectorXd ddqr(nj);
	q.setOnes();
	dq.setZero();
	dqr.setZero();
	ddqr.setZero();

	robot.setArguments(q, dq, dqr, ddqr);
	controller.start();

	// - controller loop test - //
	time_start = high_resolution_clock::now();
	controller.update();
	VectorXd tau = controller.get_action();
	time_stop = high_resolution_clock::now();
	duration = duration_cast<nanoseconds>(time_stop - time_start).count();
	cout << "tau: " << tau.transpose() << endl;
	cout << "Controller cycle time: " << duration << " ns" << endl;

	return 0;
}




// extern "C" {
// #include "acados_solver_simple_model.h"
// }

// #include <iostream>

// int main() {
//     // Sover creation
//     void *solver = acados_create();

//     // Initial conditions
//     double x0[1] = {2.0};
//     acados_solver_simple_model_set(solver, 0, "x", x0);

//     // Solver execution
//     int status = acados_solver_simple_model_solve(solver);

//     if (status == 0) {
//         std::cout << "Solver converged!" << std::endl;
//     } else {
//         std::cout << "Solver error: " << status << std::endl;
//     }

//     // Obtain the solution
//     double x_opt[1];
//     acados_solver_simple_model_get(solver, 0, "x", x_opt);

//     std::cout << "Optimal x: " << x_opt[0] << std::endl;

//     // Resource freeing
//     acados_free(solver);

//     return 0;
// }

