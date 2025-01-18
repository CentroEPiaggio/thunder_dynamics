# cmake and c++ version
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# project
project(thunder_dynamics VERSION 0.2)

# Set the output directories for compiled binaries and libraries
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set (CMAKE_CXX_FLAGS "-g3 -lstdc++fs -std=c++17")
# set (CMAKE_CXX_FLAGS_DEBUG "-g3")

# Include the "library" directory for header file search
include_directories(${CMAKE_SOURCE_DIR}/library)

# Add all source files in the "src" directory to the SOURCES variable
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Define the library target with the collected source files
add_library(library ${SOURCES})

# Set the output name for the library
set_target_properties(library PROPERTIES OUTPUT_NAME "library")

# Make the library's headers available to other projects
target_include_directories(library PUBLIC ${CMAKE_SOURCE_DIR}/library)

# Find and link the CasADi library
find_package(casadi REQUIRED)

# Include FetchContent module for fetching external dependencies
include(FetchContent)

FetchContent_Declare(
  yaml-cpp
  GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
  GIT_TAG master 
)
FetchContent_MakeAvailable(yaml-cpp)

find_package(yaml-cpp REQUIRED)
target_link_libraries(library casadi yaml-cpp::yaml-cpp)

# Add executables and link them with the library
add_executable(thunder_test ${CMAKE_SOURCE_DIR}/thunder_test.cpp)
target_link_libraries(thunder_test library)

add_executable(thunder_test_chrono ${CMAKE_SOURCE_DIR}/thunder_test_chrono.cpp)
target_link_libraries(thunder_test_chrono library)

add_executable(thunder ${CMAKE_SOURCE_DIR}/thunder_main.cpp)
target_link_libraries(thunder library)


# fetch latest argparse
FetchContent_Declare(
    argparse
    GIT_REPOSITORY https://github.com/p-ranav/argparse.git
)
FetchContent_MakeAvailable(argparse)
target_link_libraries(thunder argparse)

# Retrieve the CasADi library path
find_library(CASADI_LIB_PATH libcasadi.so HINTS ${casadi_DIR} PATH_SUFFIXES lib)
if (CASADI_LIB_PATH)
  message(STATUS "CasADi library found at ${CASADI_LIB_PATH}")
  
  # Extract the directory from the library path
  get_filename_component(CASADI_LIB_DIR ${CASADI_LIB_PATH} DIRECTORY)
  message(STATUS "CasADi library directory: ${CASADI_LIB_DIR}")

  # Set rpath to include the CasADi library directory
  set(CMAKE_INSTALL_RPATH "${CASADI_LIB_DIR}")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
else()
  message(FATAL_ERROR "CasADi library not found!")
endif()

set_target_properties(thunder PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
)
# Set default install prefix if not specified
# if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
# 	set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "default install path" FORCE)
# endif()
# set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "default install path")
set(CMAKE_INSTALL_PREFIX "/usr/local" CACHE PATH "default install path" FORCE)
# Install the executable and the template directory
install(TARGETS thunder RUNTIME DESTINATION ${CMAKE_INSTALL_PREFIX}/bin)
install(DIRECTORY ${CMAKE_SOURCE_DIR}/thunder_robot_template DESTINATION ${CMAKE_INSTALL_PREFIX}/share/thunder_dynamics)