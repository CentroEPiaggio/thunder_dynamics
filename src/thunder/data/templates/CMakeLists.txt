
cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED SKBUILD_PROJECT_NAME)
  set(SKBUILD_PROJECT_NAME "thunder_robot")
endif()

project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master
)
FetchContent_MakeAvailable(yaml-cpp)

# Set TRUE to build python bindings, default is OFF but will be handled in the pyproject
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)

set(CMAKE_CXX_STANDARD 14)

find_package(Eigen3 REQUIRED)

if(BUILD_PYTHON_BINDINGS)
  set(Python_FIND_STRATEGY LOCATION)
  set(Python_FIND_VIRTUALENV FIRST)
  find_package(Python 3.7 COMPONENTS Interpreter Development REQUIRED)
  find_package(pybind11 REQUIRED)

  add_compile_definitions(ENABLE_PYBIND11)

  pybind11_add_module(thunder_robot thunder_robot.cpp robot_gen.cpp)
  target_link_libraries(thunder_robot PRIVATE yaml-cpp::yaml-cpp pybind11::module Eigen3::Eigen)
  install(TARGETS thunder_robot LIBRARY DESTINATION thunder_robot_py)

else()
  add_library(thunder_robot SHARED thunder_robot.cpp robot_gen.cpp)
  target_link_libraries(thunder_robot PUBLIC yaml-cpp::yaml-cpp Eigen3::Eigen)
  target_include_directories(thunder_robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install(TARGETS thunder_robot DESTINATION lib)
  install(FILES thunder_robot.h robot_gen.h DESTINATION include)
endif()
