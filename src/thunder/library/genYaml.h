#ifndef GEN_YAML
#define GEN_YAML

#include "utils.h"

namespace thunder_ns{

	// void perturbateLinkProp(LinkProp original, LinkProp &perturbate, double percent);

	void fillInertialYaml(int num_joints, YAML::Emitter &emitter_, std::vector<LinkProp> &links_prop_, std::vector<std::string> keys_);

	int genInertial_files(const std::string robot_name, const int num_joints, const std::string path_inertial, const std::string path_inertial_DYN, const std::string path_inertial_REG);

}

#endif