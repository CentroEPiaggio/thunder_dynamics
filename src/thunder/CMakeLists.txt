# cmake and c++ version
cmake_minimum_required(VERSION 3.18)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# project
project(thunder_dynamics VERSION 1.0.0)

include(GNUInstallDirs)

option(WITH_TESTS "Build C++ unit tests (GoogleTest)" OFF)

# Set the output directories for compiled binaries and libraries
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_FLAGS "-lstdc++fs -std=c++17")
# set (CMAKE_CXX_FLAGS_DEBUG "-g3")
set(CMAKE_BUILD_TYPE Release)

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
  set(THUNDER_PRIVATE_LIB_DIR "lib/thunder_dynamics")

  # Make the test executables and shared library runnable from the build tree.
  set_target_properties(library PROPERTIES
    BUILD_RPATH "${CASADI_LIB_DIR}"
  )
  set(CMAKE_BUILD_RPATH "${CASADI_LIB_DIR}")

  # Set rpath to include the CasADi library directory
  # set(CMAKE_INSTALL_RPATH "${CASADI_LIB_DIR}")
  # set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  # Use $ORIGIN so the binary looks for the lib relative to its own path (bin/..)
  # We use the escaped version \$ORIGIN so it literalizes in the ELF header
  set(CMAKE_INSTALL_RPATH "\$ORIGIN/../${THUNDER_PRIVATE_LIB_DIR}")
  set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
  # Install CasADi files so they get included in the .deb
  file(GLOB CASADI_LIB_VARIANTS "${CASADI_LIB_DIR}/libcasadi.so*")
  install(FILES ${CASADI_LIB_VARIANTS} DESTINATION ${THUNDER_PRIVATE_LIB_DIR})
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
if(WITH_TESTS)
  enable_testing()

  FetchContent_Declare(
    googletest
    GIT_REPOSITORY https://github.com/google/googletest.git
    GIT_TAG main
  )
  set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
  FetchContent_MakeAvailable(googletest)

  add_executable(thunder_no_generation_test ${CMAKE_SOURCE_DIR}/tests/cpp/test_no_generation.cpp)
  target_link_libraries(thunder_no_generation_test PRIVATE library gtest_main)
  target_compile_definitions(thunder_no_generation_test PRIVATE THUNDER_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}/../..\")

  add_executable(thunder_generator_smoke_test ${CMAKE_SOURCE_DIR}/tests/cpp/test_generator_smoke.cpp)
  target_link_libraries(thunder_generator_smoke_test PRIVATE library gtest_main)
  target_compile_definitions(thunder_generator_smoke_test PRIVATE THUNDER_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}/../..\")

  add_executable(thunder_tree_kinematics_test ${CMAKE_SOURCE_DIR}/tests/cpp/test_tree_kinematics.cpp)
  target_link_libraries(thunder_tree_kinematics_test PRIVATE library gtest_main)
  target_compile_definitions(thunder_tree_kinematics_test PRIVATE THUNDER_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}/../..\")

  add_executable(thunder_robot_comparison_gtest
    ${CMAKE_SOURCE_DIR}/tests/cpp/robot_comparison_gtest.cpp
  )
  target_link_libraries(thunder_robot_comparison_gtest PRIVATE library gtest_main)
  target_compile_definitions(thunder_robot_comparison_gtest PRIVATE
    THUNDER_SOURCE_DIR=\"${CMAKE_SOURCE_DIR}/../..\"
    PINOCCHIO_HELPER_SCRIPT=\"${CMAKE_SOURCE_DIR}/tests/python/pinocchio_helper.py\"
  )

  set_target_properties(thunder_robot_comparison_gtest PROPERTIES
      INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
      BUILD_WITH_INSTALL_RPATH TRUE
  )

  set_target_properties(thunder_no_generation_test PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  set_target_properties(thunder_generator_smoke_test PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )
  set_target_properties(thunder_tree_kinematics_test PROPERTIES
    INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
    BUILD_WITH_INSTALL_RPATH TRUE
  )

  add_test(NAME thunder_no_generation_test COMMAND thunder_no_generation_test)
  add_test(NAME thunder_generator_smoke_test COMMAND thunder_generator_smoke_test)
  add_test(NAME thunder_tree_kinematics_test COMMAND thunder_tree_kinematics_test)
  add_test(NAME thunder_robot_comparison_gtest COMMAND thunder_robot_comparison_gtest)

  set_tests_properties(
    thunder_no_generation_test
    thunder_generator_smoke_test
    thunder_tree_kinematics_test
    thunder_robot_comparison_gtest
    PROPERTIES
      ENVIRONMENT "LD_LIBRARY_PATH=${CASADI_LIB_DIR}:$ENV{LD_LIBRARY_PATH}"
  )
endif()

# Set default install prefix if not specified
# if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
# 	set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "default install path" FORCE)
# endif()
# set(CMAKE_INSTALL_PREFIX "$ENV{HOME}/.local" CACHE PATH "default install path")
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "/usr" CACHE PATH "default install path" FORCE)
endif()
# Install the executable and the template directory
install(TARGETS thunder RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
install(DIRECTORY ${CMAKE_SOURCE_DIR}/data/templates DESTINATION ${CMAKE_INSTALL_DATADIR}/thunder_dynamics)

# =============================== #
# --- Python Plugin Support  --- #
# =============================== #

option(BUILD_PYTHON_PLUGINS "Build Python plugin support with nanobind" ON)
option(THUNDER_USE_LOCAL_PYTHON_FILES "Use local thunder_core/thunder_plugins sources instead of installing Python packages" OFF)
set(THUNDER_LOCAL_PYTHON_ROOT "${CMAKE_SOURCE_DIR}/../../python" CACHE PATH "Path to local Python source root (must contain thunder_core and thunder_plugins)")

if(BUILD_PYTHON_PLUGINS)
  message(STATUS "Configuring Python plugin support...")
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

  if(THUNDER_USE_LOCAL_PYTHON_FILES)
    #  write compiled module next to local thunder_core package.
    set_target_properties(_bindings PROPERTIES
      LIBRARY_OUTPUT_DIRECTORY "${THUNDER_LOCAL_PYTHON_ROOT}/thunder_core")
  endif()

  target_link_libraries(_bindings PRIVATE library)
  target_include_directories(_bindings PRIVATE
    ${CMAKE_SOURCE_DIR}/python
    ${SWIGPYRUN_DIR}
  )
  # Resolve bundled shared libs from the extension module directory at runtime.
  set_target_properties(_bindings PROPERTIES
    INSTALL_RPATH "\$ORIGIN"
    BUILD_WITH_INSTALL_RPATH TRUE
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

  if(THUNDER_USE_LOCAL_PYTHON_FILES)
    message(STATUS "  Local Python source mode ENABLED")
    message(STATUS "  Local root: ${THUNDER_LOCAL_PYTHON_ROOT}")
    message(STATUS "  _bindings output: ${THUNDER_LOCAL_PYTHON_ROOT}/thunder_core")
    message(STATUS "  Skipping install of thunder_core/thunder_plugins into site-packages")
  else()
    # Install compiled _bindings module into thunder_core package directory
    install(TARGETS _bindings LIBRARY DESTINATION ${PYTHON_SITE_PACKAGES}/thunder_core)
    # Install CasADi runtime libraries alongside _bindings so Python imports work
    # without requiring system-wide CasADi linker configuration.
    install(FILES ${CASADI_LIB_VARIANTS} DESTINATION ${PYTHON_SITE_PACKAGES}/thunder_core)
    # Install thunder_core Python package (pure-Python parts)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../python/thunder_core
      DESTINATION ${PYTHON_SITE_PACKAGES})
    # Install thunder_plugins package (Python plugin system)
    install(DIRECTORY ${CMAKE_SOURCE_DIR}/../../python/thunder_plugins
      DESTINATION ${PYTHON_SITE_PACKAGES})
  endif()

  message(STATUS "Python plugin support ENABLED")
  message(STATUS "  Python: ${Python_EXECUTABLE} (${Python_VERSION})")
  message(STATUS "  SWIG:   ${SWIG_EXECUTABLE} (${SWIG_VERSION})")
  message(STATUS "  Install to: ${PYTHON_SITE_PACKAGES}")
  
  if(WITH_TESTS)
    add_test(NAME thunder_python_api_test
      COMMAND ${CMAKE_COMMAND} -E env
        LD_LIBRARY_PATH=${CASADI_LIB_DIR}:$ENV{LD_LIBRARY_PATH}
        PYTHONPATH=${CMAKE_BINARY_DIR}:${CMAKE_SOURCE_DIR}/../../python
        ${Python_EXECUTABLE} -m unittest discover -s ${CMAKE_SOURCE_DIR}/tests/python -p test_*.py -v)
  endif()
endif()

# =============================== #
# ------- Package Support ------- #
# =============================== #

set(CPACK_GENERATOR "DEB")
set(CPACK_PACKAGE_NAME "thunder-dynamics")
set(CPACK_PACKAGE_VENDOR "thunder")
set(CPACK_PACKAGE_VERSION ${PROJECT_VERSION})
set(CPACK_PACKAGE_CONTACT "maintainers@thunder-dynamics.local")
set(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Thunder dynamics executable, templates, and optional Python plugin support")

# Keep absolute install destinations (e.g. Python site-packages) valid in packages.
set(CPACK_SET_DESTDIR ON)
set(CPACK_PACKAGING_INSTALL_PREFIX "/")

set(CPACK_DEBIAN_FILE_NAME DEB-DEFAULT)

# Let CPack infer shared-library runtime deps automatically.
set(CPACK_DEBIAN_PACKAGE_SHLIBDEPS ON)

if(BUILD_PYTHON_PLUGINS AND NOT THUNDER_USE_LOCAL_PYTHON_FILES)
  set(CPACK_DEBIAN_PACKAGE_DEPENDS "python3")
endif()

include(CPack)
