#ifndef ROS_LIBRARY_PACKAGE_H
#define ROS_LIBRARY_PACKAGE_H

#include <filesystem>
#include <string>

namespace thunder_ns {

	/**
	 * @brief Name of the ament package holding the robot library: thunder_<robot> in lowercase.
	 *
	 * @throws std::runtime_error if the robot name cannot give a valid ROS 2 package name
	 */
	std::string ros_library_package_name(const std::string& robot_name);

	/**
	 * @brief Write the ament package thunder_<robot> in output_root: the C++ robot written by robot_generator
	 * (thunder_<robot>.h/.cpp, <robot>_gen.h/.cpp, <robot>_conf.yaml, <robot>_par.yaml) as a shared library,
	 * exported as thunder_<robot>::thunder_<robot>. Used by the ROS generators, which link it.
	 *
	 * @param output_root <robot>_generatedFiles, where robot_generator wrote the robot
	 * @param robot_name robot name
	 * @param generator name of the calling generator, for the error messages
	 * @return the package name
	 * @throws std::runtime_error if the files of robot_generator are missing
	 */
	std::string write_ros_library_package(const std::filesystem::path& output_root, const std::string& robot_name,
	                                      const std::string& generator);

} // namespace thunder_ns

#endif // ROS_LIBRARY_PACKAGE_H
