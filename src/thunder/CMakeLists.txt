# cmake and c++ version
cmake_minimum_required(VERSION 3.18)
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
include_directories(${CMAKE_SOURCE_DIR}/include)

# Add all source files in the "src" directory to the SOURCES variable
file(GLOB_RECURSE SOURCES
  ${CMAKE_SOURCE_DIR}/src/*.cpp
)
# Exclude Python proxy 
list(FILTER SOURCES EXCLUDE REGEX ".*/plugins/python/.*")

# Define the library target with the collected source files
add_library(library ${SOURCES})

# Set the output name for the library
set_target_properties(library PROPERTIES OUTPUT_NAME "library")
set_target_properties(library PROPERTIES POSITION_INDEPENDENT_CODE ON)

# Make the library's headers available to other projects
target_include_directories(library PUBLIC ${CMAKE_SOURCE_DIR}/include)

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

# Fetch URDF parser
FetchContent_Declare(
    urdfparser
    GIT_REPOSITORY https://github.com/tolomeis/URDF_Parser.git
    GIT_TAG dev/eigen
)
set(BUILD_SHARED_LIBS OFF CACHE BOOL "Build static libs" FORCE)

FetchContent_MakeAvailable(urdfparser)

target_link_libraries(library casadi yaml-cpp::yaml-cpp urdfparser)



# Add executables and link them with the library
add_executable(thunder_test ${CMAKE_SOURCE_DIR}/thunder_test.cpp)
target_link_libraries(thunder_test library)

add_executable(thunder_test_chrono ${CMAKE_SOURCE_DIR}/thunder_test_chrono.cpp)
target_link_libraries(thunder_test_chrono library)

add_executable(thunder_symbolic_test ${CMAKE_SOURCE_DIR}/symbolic_mask_test.cpp)
target_link_libraries(thunder_symbolic_test library)
target_compile_definitions(thunder_symbolic_test PRIVATE THUNDER_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}/../..\")

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
set_target_properties(thunder_test PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
)
set_target_properties(thunder_symbolic_test PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
)
set_target_properties(thunder_test_chrono PROPERTIES
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
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/templates DESTINATION ${CMAKE_INSTALL_PREFIX}/share/thunder_dynamics)

# =============================== #
# --- Python Plugin Support  --- #
# =============================== #
option(BUILD_PYTHON_PLUGINS "Build Python plugin support with nanobind" OFF)

if(BUILD_PYTHON_PLUGINS)
  find_package(Python 3.8 COMPONENTS Interpreter Development REQUIRED)
  find_package(SWIG REQUIRED)
  include(${SWIG_USE_FILE})

  # Generate swigpyrun.h (SWIG runtime header for pointer extraction)
  set(SWIGPYRUN_DIR "${CMAKE_CURRENT_BINARY_DIR}/swigpyrun")
  set(SWIGPYRUN_OUTPUT "${SWIGPYRUN_DIR}/swigpyrun.h")
  file(MAKE_DIRECTORY ${SWIGPYRUN_DIR})
  add_custom_command(
    OUTPUT ${SWIGPYRUN_OUTPUT}
    COMMAND ${SWIG_EXECUTABLE} -python -external-runtime ${SWIGPYRUN_OUTPUT}
    COMMENT "Generating swigpyrun.h for CasADi type bridge"
    VERBATIM
  )
  add_custom_target(generate_swigpyrun ALL DEPENDS ${SWIGPYRUN_OUTPUT})

  # Fetch nanobind
  FetchContent_Declare(
    nanobind
    GIT_REPOSITORY https://github.com/wjakob/nanobind.git
    GIT_TAG v2.4.0
  )
  FetchContent_MakeAvailable(nanobind)

  # thunder_core._bindings Python module (Robot bindings + CasADi type casters)
  nanobind_add_module(_bindings
    ${CMAKE_SOURCE_DIR}/python/bindings.cpp
  )
  target_link_libraries(_bindings PRIVATE library)
  target_include_directories(_bindings PRIVATE
    ${CMAKE_SOURCE_DIR}/python
    ${SWIGPYRUN_DIR}
  )
  add_dependencies(_bindings generate_swigpyrun)

  # Add Python proxy plugin sources to the main library
  target_sources(library PRIVATE
    ${CMAKE_SOURCE_DIR}/src/plugins/python/py_plugin_proxy.cpp
  )
  target_link_libraries(library Python::Python)
  target_include_directories(library PRIVATE ${SWIGPYRUN_DIR})
  target_compile_definitions(library PUBLIC THUNDER_PYTHON_PLUGINS)
  add_dependencies(library generate_swigpyrun)

  # Also give the thunder executable nanobind/Python include paths
  target_include_directories(thunder PRIVATE ${SWIGPYRUN_DIR})

  # Install thunder_core package to Python site-packages
  execute_process(
    COMMAND ${Python_EXECUTABLE} -c "import sysconfig; print(sysconfig.get_path('purelib'))"
    OUTPUT_VARIABLE PYTHON_SITE_PACKAGES
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  # Install compiled _bindings module into thunder_core package directory
  install(TARGETS _bindings LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}/thunder_core)

  # Install thunder_core Python package (pure-Python parts)
  install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../python/thunder_core
    DESTINATION ${PYTHON_SITE_PACKAGES})

  message(STATUS "Python plugin support ENABLED")
  message(STATUS "  Python: ${Python_EXECUTABLE} (${Python_VERSION})")
  message(STATUS "  SWIG:   ${SWIG_EXECUTABLE} (${SWIG_VERSION})")
  message(STATUS "  Install to: ${PYTHON_SITE_PACKAGES}")
endif()
