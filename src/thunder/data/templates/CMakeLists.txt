
cmake_minimum_required(VERSION 3.10)

if(NOT DEFINED SKBUILD_PROJECT_NAME)
  set(SKBUILD_PROJECT_NAME "thunder_robot")
endif()

project(${SKBUILD_PROJECT_NAME} LANGUAGES CXX)

# --- Build optimization flags --- 
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Optimization flags for faster compilation
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  string(APPEND CMAKE_CXX_FLAGS_RELEASE " -O3 -march=native")
  # Enable Link Time Optimization (LTO) for smaller, faster binaries
  string(APPEND CMAKE_CXX_FLAGS_RELEASE " -flto=auto")
endif()

# Enable parallel compilation with ninja/make
set(CMAKE_JOB_POOLS "default=4")
set(CMAKE_JOB_POOL_COMPILE "default")

# Set TRUE to build python bindings, default is OFF but will be handled in the pyproject
option(BUILD_PYTHON_BINDINGS "Build python bindings" OFF)
option(USE_SYSTEM_YAML_CPP "Use system yaml-cpp if available" ON)

find_package(Eigen3 REQUIRED)

# Try system yaml-cpp first, then fetch
if(USE_SYSTEM_YAML_CPP)
  find_package(yaml-cpp CONFIG QUIET)
endif()

if(NOT yaml-cpp_FOUND)
  include(FetchContent)
  FetchContent_Declare(
    yaml-cpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG master
  )
  FetchContent_MakeAvailable(yaml-cpp)
endif()

if(BUILD_PYTHON_BINDINGS)
  set(Python_FIND_STRATEGY LOCATION)
  set(Python_FIND_VIRTUALENV FIRST)
  find_package(Python 3.8 COMPONENTS Interpreter Development.Module REQUIRED)
  find_package(pybind11 CONFIG REQUIRED)

  add_compile_definitions(ENABLE_PYBIND11)

  # Use pybind11_add_module for automatic module configuration
  pybind11_add_module(thunder_robot 
    MODULE
    thunder_robot.cpp 
    robot_gen.cpp
  )
  
  # Enable LTO and optimizations for the Python module
  target_compile_options(thunder_robot PRIVATE -O3 -march=native)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(thunder_robot PRIVATE -flto=auto)
  endif()
  
  target_link_libraries(thunder_robot PRIVATE 
    yaml-cpp::yaml-cpp 
    pybind11::module 
    Eigen3::Eigen
  )
  
  # Strip debug symbols for smaller wheels (users can install -dbg packages if needed)
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_custom_command(TARGET thunder_robot POST_BUILD
      COMMAND ${CMAKE_STRIP} -s $<TARGET_FILE:thunder_robot> || true
      COMMENT "Stripping debug symbols from Python module"
    )
  endif()
  
  install(TARGETS thunder_robot LIBRARY DESTINATION thunder_robot_py)

else()
  add_library(thunder_robot SHARED thunder_robot.cpp robot_gen.cpp)
  
  # Enable optimizations for shared library
  target_compile_options(thunder_robot PRIVATE -O3 -march=native)
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(thunder_robot PRIVATE -flto=auto)
  endif()
  
  target_link_libraries(thunder_robot PUBLIC yaml-cpp::yaml-cpp Eigen3::Eigen)
  target_include_directories(thunder_robot PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

  install(TARGETS thunder_robot DESTINATION lib)
  install(FILES thunder_robot.h robot_gen.h DESTINATION include)
endif()
