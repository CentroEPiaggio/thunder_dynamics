cmake_minimum_required(VERSION 3.16)
project(rrr_server_test)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rrr_server REQUIRED)

add_executable(rrr_service_client src/rrr_service_client.cpp)
target_compile_features(rrr_service_client PUBLIC cxx_std_17)
ament_target_dependencies(rrr_service_client rclcpp rrr_server)

install(TARGETS rrr_service_client DESTINATION lib/${PROJECT_NAME})
ament_package()
