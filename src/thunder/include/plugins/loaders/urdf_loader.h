#ifndef URDF_LOADER_H
#define URDF_LOADER_H

#include "plugin_interfaces.h"
#include "urdf/model.h"
#include "urdf/link.h"
#include "urdf/joint.h"

namespace thunder_ns {

	class UrdfLoader : public BaseLoader {
		
		private:
			std::shared_ptr<urdf::UrdfModel> urdf_model;
			std::vector<std::shared_ptr<urdf::Link>> chain;
			std::shared_ptr<urdf::Link> root_link;
			std::shared_ptr<urdf::Link> ee_link;
			std::string robot_name;
			int numJoints = 0;
			int ndof = 0;
			const int KIN_DIM = 6;
			const int DYN_DIM = 10;
			vector<double> par_KIN_num;
			vector<double> par_DYN_num;			

			// robot properties
			vector<string> jointsName;
			vector<string> jointsType;
			vector<string> jointsParentStr;
			vector<int> jointsParent;
			vector<bool> jointsAvailable;
			vector<bool> jointsDerivatives;
			vector<int> jointsDimension;
			vector<vector<double>> jointsAxis;

			void accumulateChain(std::shared_ptr<urdf::Link> link, const std::string& base, std::vector<std::shared_ptr<urdf::Link>>& chain);
			casadi::SX to_casadi_sx(const urdf::Transform& T);
			void reset_thunder_chain();
			casadi::DM extractKinematicsFromJoint(std::shared_ptr<urdf::Joint> joint);
			casadi::DM extractInertiaFromLink(std::shared_ptr<urdf::Link> link);
			void add_joint(std::shared_ptr<urdf::Joint> joint);
			bool chain_has_link(string link_name);
			void add_chain_from(int parent, std::shared_ptr<urdf::Link> link);
			void parse_frame_parameterization(
				std::shared_ptr<Robot> robot,
				const YAML::Node& frame_node,
				const std::string& frame_prefix,
				const std::vector<double>& default_xyzrpy,
				const std::vector<short>& default_symb,
				casadi::SX& frame_expr,
				std::vector<std::string>& frame_args,
				const std::string& description_prefix);

		public:
			UrdfLoader() : BaseLoader("URDF Loader", "Load a robot from URDF file.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // URDF_LOADER_H
