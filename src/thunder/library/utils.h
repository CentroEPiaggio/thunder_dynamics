#ifndef THUNDER_UTILS
#define THUNDER_UTILS

#include <string>

namespace thunder_ns{

	int change_to_robot(const std::string from_robot, const std::string to_robot, int n_joints, const std::string file_path_h, const std::string file_path_cpp);

	void replace_all(std::string& str, const std::string& from_str, const std::string& to_str);

}

#endif