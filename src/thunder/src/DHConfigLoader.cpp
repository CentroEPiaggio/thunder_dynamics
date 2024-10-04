#include <iostream>
#include <yaml-cpp/yaml.h>
#include "DHConfigLoader.h"
#include "../library/robot.h"
#include "../library/kinematics.h"
#include "../library/dynamics.h"
#include "../library/regressors.h"
#include "../library/utils.h"
/*
%       - Links start from 0, first bring the base frame to the first
%           joint, last bring the last joint to the end effector
%       - Frames start from 0, there are as many frames as links
%       - Frames have the origins on the phisical joints (0 and ee
%           excluded)
%       - Joints start from 1, frames and joints have same origins and
%           indexes
%       - Link Li brings the Joint {Ji} to the Joint {Ji+1} with a 
%           rototraslational matrix, defined by a traslation and a 
%           rotation using the XYZ parametrization
%       - Center of mass of link Li is expressed in frame {Si}
%       - Inertia tensor of link Li is defined on the center of mass in the
%           frame {Si}
%       - the joint variables are included in the trasformation matrices
*/
namespace thunder_ns {

    Config DHConfigLoader::load(const std::string& file) const {
        std::cout << "Loading DH config from " << file << std::endl;
        		int nj;
		FrameOffset Base_to_L0;
		FrameOffset Ln_to_EE;
		Config conf;
		// ----- parsing yaml file ----- //
		try {
			// load yaml
			YAML::Node config_file = YAML::LoadFile(file);

			// Number of joints
			nj = config_file["num_joints"].as<int>();
			conf.numJoints = nj;

			// joints_type
			// YAML::Node type_joints = config_file["type_joints"];
			// jType = type_joints.as<std::string>();
			conf.jointsType = config_file["type_joints"].as<std::vector<std::string>>();
			std::vector<std::string> jType = conf.jointsType;

			if (config_file["Dl_order"]) conf.Dl_order = config_file["Dl_order"].as<int>();
			if (config_file["ELASTIC_MODEL"]){
				conf.ELASTIC = config_file["ELASTIC_MODEL"].as<bool>();
				if (conf.ELASTIC){
					conf.K_order = config_file["elastic"]["K_order"].as<int>();
					conf.D_order = config_file["elastic"]["D_order"].as<int>();
					conf.Dm_order = config_file["elastic"]["Dm_order"].as<int>();
				}
			}
			// gravity
			std::vector<double> gravity = config_file["gravity"].as<std::vector<double>>();

			// frames offsets
			YAML::Node frame_base = config_file["Base_to_L0"];
			YAML::Node frame_ee = config_file["Ln_to_EE"];

			std::vector<double> tr = frame_base["tr"].as<std::vector<double>>();
			std::vector<double> ypr = frame_base["ypr"].as<std::vector<double>>();
			Base_to_L0.set_translation(tr);
			Base_to_L0.set_ypr(ypr);
			Base_to_L0.set_gravity(gravity);
			conf.base_frame = Base_to_L0;

			tr = frame_ee["tr"].as<std::vector<double>>();
			ypr = frame_ee["ypr"].as<std::vector<double>>();
			Ln_to_EE.set_translation(tr);
			Ln_to_EE.set_ypr(ypr);
			conf.ee_frame = Ln_to_EE;

			// Denavit-Hartenberg
            std::vector<double> dh_vect = config_file["DH"].as<std::vector<double>>();
			auto DHtable = Eigen::Map<Eigen::VectorXd>(&dh_vect[0], nj*4).reshaped<Eigen::RowMajor>(nj, 4);


            // computing chain
            casadi::SXVector Ti(nj);    // Output
            casadi::SXVector T0i(nj+1);   // Output
        
            // Ti is transformation from link i-1 to link i
            Ti[0] = DHConfigLoader::DHMatrix(DHtable.row(0));
            // T0i is transformation from link 0 to link i
            T0i[0] = casadi::SX::mtimes({Base_to_L0.get_transform(), Ti[0]});   

            std::vector<std::vector<casadi::SX>> kin_params;
            // load the matrix T0i in the kin_params.
            // first deal with R: extract diag vector, then extract skew-simm elements and then extract the translation vector
            auto R = T0i[0](casadi::Slice(0, 3), casadi::Slice(0, 3));
            auto diag = casadi::SX::diag(T0i[0](casadi::Slice(0, 3), casadi::Slice(0, 3)));
            auto skew_symm = casadi::SX::skew(R);
            auto d = T0i[0](casadi::Slice(0, 3), 3);
            auto Ksx = casadi::SX::vertcat({diag, skew_symm, d});

            std::vector<casadi::SX> kin_param = {Ksx(0), Ksx(1), Ksx(2), Ksx(3), Ksx(4), Ksx(5), Ksx(6), Ksx(7), Ksx(8), Ksx(9)};
            
            kin_params.push_back(kin_param);

            
            for (int i = 1; i < nj; i++) {
                Ti[i] = DHConfigLoader::DHMatrix(DHtable.row(i));
                T0i[i] = casadi::SX::mtimes({T0i[i-1], Ti[i]});
                // load in kin params
                kin_param = {T0i[i](0, 0), T0i[i](0, 1), T0i[i](0, 2), T0i[i](1, 1), T0i[i](1, 2), T0i[i](2, 2), T0i[i](0, 3), T0i[i](1, 3), T0i[i](2, 3), T0i[i](3, 3)};
                kin_params.push_back(kin_param);
            }



		} catch (const YAML::Exception& e) {
			std::cerr << "Error while parsing YAML: " << e.what() << std::endl;
		}

		return conf;
    }

	casadi::SX DHConfigLoader::DHMatrix(const Eigen::MatrixXd &rowDHTable) const {

            double a;
            double alpha;
            casadi::SX d;
            casadi::SX theta;

            casadi::SX ct; // cos(theta)
            casadi::SX st; // sin(theta)
            double ca;     // cos(alpha)
            double sa;     // sin(alpha)

            casadi::SX Ti(4, 4); // output

            a = rowDHTable(0);
            alpha = rowDHTable(1);

            ca = cos(alpha);
            sa = sin(alpha);
            ct = cos(theta);
            st = sin(theta);

            // modified: T_a*T_alpha*T_d*T_theta
            Ti(0, 0) = ct;
            Ti(0, 1) = -st;
            Ti(0, 2) = 0;
            Ti(0, 3) = a;
            Ti(1, 0) = ca * st;
            Ti(1, 1) = ca * ct;
            Ti(1, 2) = -sa;
            Ti(1, 3) = -d * sa;
            Ti(2, 0) = sa * st;
            Ti(2, 1) = sa * ct;
            Ti(2, 2) = ca;
            Ti(2, 3) = d * ca;
            Ti(3, 3) = 1;

            return Ti;
        }

}