
cmake_minimum_required(VERSION 3.10)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG <tag_name> # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)
project(thunder_robot_py)

set(CMAKE_CXX_STANDARD 14)

find_package(yaml-cpp REQUIRED)
find_package(Python 3.7 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)


pybind11_add_module(thunder_robot_py thunder_robot.cpp robot_gen.cpp)

target_link_libraries(thunder_robot_py PRIVATE yaml-cpp::yaml-cpp pybind11::module Eigen3::Eigen)