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
			void reset_thunder_chain(size_t size);
			casadi::DM extractKinematicsFromJoint(std::shared_ptr<urdf::Joint> joint);
			casadi::DM extractInertiaFromLink(std::shared_ptr<urdf::Link> link);
			void add_joint(int link_id, std::shared_ptr<urdf::Joint> joint);
			void add_chain_from(int& link_id, int parent, std::shared_ptr<urdf::Link> link);

		public:
			UrdfLoader() : BaseLoader("URDF Loader", "Load a robot from URDF file.") {}

			std::shared_ptr<Robot> load(std::shared_ptr<Robot> robot_ptr) override;

	};


} // namespace thunder_ns

#endif // URDF_LOADER_H
