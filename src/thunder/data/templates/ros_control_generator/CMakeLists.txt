cmake_minimum_required(VERSION 3.16)
project(<PACKAGE>)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(DEPENDENCIES
  control_msgs
  controller_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  std_msgs
  std_srvs
  trajectory_msgs
  urdf
)

find_package(ament_cmake REQUIRED)
foreach(dependency IN ITEMS ${DEPENDENCIES})
  find_package(${dependency} REQUIRED)
endforeach()
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)

# the controller and the generated robot library, in one plugin
add_library(${PROJECT_NAME} SHARED
  src/controller.cpp
  src/thunder_<ROBOT>.cpp
  src/<ROBOT>_gen.cpp
)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
# the generated robot sources include their headers without the package prefix
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME})
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen yaml-cpp)
ament_target_dependencies(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

pluginlib_export_plugin_description_file(controller_interface <PACKAGE>.xml)

install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # the algorithm without ROS: quintic, limit check, filter, MACT terms
  ament_add_gtest(test_algorithm test/test_algorithm.cpp)
  target_include_directories(test_algorithm PRIVATE include)
  target_link_libraries(test_algorithm Eigen3::Eigen)
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES} Eigen3 yaml-cpp)
ament_package()
