
cmake_minimum_required(VERSION 3.10)
project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master # Can be a tag (yaml-cpp-x.x.x), a commit hash, or a branch name (master)
)
FetchContent_MakeAvailable(yaml-cpp)

set(CMAKE_CXX_STANDARD 14)

set(Python_FIND_STRATEGY LOCATION)
set(Python_FIND_VIRTUALENV FIRST)

find_package(Python 3.7 COMPONENTS Interpreter Development REQUIRED)
find_package(pybind11 REQUIRED)
find_package(Eigen3 REQUIRED)


pybind11_add_module(thunder_robot thunder_robot.cpp robot_gen.cpp)

target_link_libraries(thunder_robot PRIVATE yaml-cpp::yaml-cpp pybind11::module Eigen3::Eigen)
install(TARGETS thunder_robot LIBRARY DESTINATION thunder_robot_py)