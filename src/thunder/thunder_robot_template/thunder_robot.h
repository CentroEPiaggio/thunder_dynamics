#ifndef THUNDERROBOT_H
#define THUNDERROBOT_H

#include <iostream>
#include <string>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <fstream>


/* Class thunder_robot 
Useful functions for adaptive control with Slotine-Li regressor
Usable only for generated code from casadi library! */
class thunder_robot{

	private:
		// standard number of parameters
		const int STD_PAR_LINK = 10;
		/* Joints' variables */
		Eigen::VectorXd q, dq, ddq, d3q, d4q, dqr, ddqr, par_DYN, par_REG, par_Dl;
		Eigen::VectorXd x, dx, ddx, ddxr, par_K, par_D, par_Dm, par_Mm;
		Eigen::VectorXd w;
		Eigen::VectorXd DHtable, gravity, world2L0, Ln2EE;
		std::vector<int> DHtable_symb, gravity_symb, world2L0_symb, Ln2EE_symb;

		void update_inertial_DYN();
		void update_inertial_REG();

		struct LinkProp {
			double mass;
			std::vector<double> parI = std::vector<double>(6); // Inertia in the order xx, xy, xz, yy, yz, zz
			std::vector<double> xyz = std::vector<double>(3); // Origin xyz as std::vector
			std::vector<double> rpy = std::vector<double>(3);
			std::vector<double> Dl = std::vector<double>(1);
			std::string name;
		};

		struct urdf2dh_T{
			std::vector<double> xyz;
			std::vector<double> rpy;
		};

		void fillInertialYaml(int num_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_);
		// void transformBodyInertial(std::vector<double> d_i, std::vector<double> rpy_i, const LinkProp body_urdf, LinkProp &body);
		// void mergeBodyInertial(const LinkProp body1, const LinkProp body2, LinkProp &newBody);

		Eigen::Matrix3d rpyRot(const std::vector<double> rpy);
		Eigen::Matrix3d createI(const std::vector<double> parI);
		Eigen::Matrix3d hat(const Eigen::Vector3d v);
		
	public:
		const std::string robotName = "/*#-ROBOT_NAME-#*/";
		const int n_joints = /*#-n_joints-#*/;
		const bool ELASTIC = /*#-ELASTIC-#*/;
		const int numElasticJoints = /*#-numElasticJoints-#*/;
		const int K_order = /*#-K_order-#*/;
		const int D_order = /*#-D_order-#*/;
		const int Dl_order = /*#-Dl_order-#*/;
		const int Dm_order = /*#-Dm_order-#*/;
		const int numParDYN = STD_PAR_LINK*n_joints;
		const int numParREG = STD_PAR_LINK*n_joints;
		// const int numParELA = /*#-numParELA-#*/;
		const int isElasticJoint[/*#-n_joints-#*/] = /*#-isElasticJoint-#*/;

		/* Empty constructor, initialization inside */
		thunder_robot();
		/* Constructor to init variables*/
		// thunder_robot(const int);
		/* Destructor*/
		~thunder_robot() = default;
		
		/* Resize variables */
		void resizeVariables();
		/* Init variables from number of joints*/
		// void init(const int);
		/* Set q, dq, dqr, ddqr, to compute Regressor and update state*/
		void setArguments(const Eigen::VectorXd& q_, const Eigen::VectorXd& dq_, const Eigen::VectorXd& dqr_, const Eigen::VectorXd& ddqr_);
		void set_q(const Eigen::VectorXd& q_);
		void set_dq(const Eigen::VectorXd& dq_);
		void set_ddq(const Eigen::VectorXd& ddq_);
		void set_d3q(const Eigen::VectorXd& d3q_);
		void set_d4q(const Eigen::VectorXd& d4q_);
		void set_dqr(const Eigen::VectorXd& dqr_);
		void set_ddqr(const Eigen::VectorXd& ddqr_);
		void set_x(const Eigen::VectorXd& x_);
		void set_dx(const Eigen::VectorXd& dx_);
		void set_ddx(const Eigen::VectorXd& ddx_);
		void set_ddxr(const Eigen::VectorXd& ddxr_);
		void set_w(const Eigen::VectorXd& w_);
		void set_par_REG(const Eigen::VectorXd& par_, bool update_DYN = true);
		void set_par_DYN(const Eigen::VectorXd& par_, bool update_REG = true);
		void set_par_K(const Eigen::VectorXd& par_);
		void set_par_D(const Eigen::VectorXd& par_);
		void set_par_Dm(const Eigen::VectorXd& par_);
		void set_par_Mm(const Eigen::VectorXd& par_);
		void set_par_Dl(const Eigen::VectorXd& par_);
		void set_DHtable(const Eigen::MatrixXd& par_);
		void set_gravity(const Eigen::VectorXd& par_);
		void set_world2L0(const Eigen::VectorXd& par_);
		void set_Ln2EE(const Eigen::VectorXd& par_);
		// void set_par_ELA(const Eigen::VectorXd& par_);
		Eigen::VectorXd get_par_REG();
		Eigen::VectorXd get_par_DYN();
		Eigen::VectorXd get_par_K();
		Eigen::VectorXd get_par_D();
		Eigen::VectorXd get_par_Dm();
		Eigen::VectorXd get_par_Mm();
		Eigen::VectorXd get_par_Dl();
		Eigen::MatrixXd get_DHtable();
		Eigen::VectorXd get_gravity();
		Eigen::VectorXd get_world2L0();
		Eigen::VectorXd get_Ln2EE();
		// Eigen::VectorXd get_par_ELA();

		Eigen::VectorXd load_par_REG(std::string, bool update_DYN = true);
		void save_par_REG(std::string);
		void load_conf(std::string, bool update_REG = true);
		void save_par_DYN(std::string);
		int save_par(std::string par_file);
		// void load_par_DYN(std::string);
		// void save_par_DYN(std::string);
		// void load_par_elastic(std::string);
		// void load_par_ELA(std::string);
		// void save_par_ELA(std::string);

		int get_numJoints();
		// int get_numParLink();
		int get_numParDYN();
		int get_numParREG();
		// int get_numParELA();

		// ----- generated functions ----- //
		/*#-FUNCTIONS_H-#*/
};

#endif