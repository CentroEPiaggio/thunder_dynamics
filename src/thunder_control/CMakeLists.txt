# cmake and c++ version
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(thunder_control VERSION 0.2)

set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(CMAKE_CXX_FLAGS "-g3 -lstdc++fs -std=c++17")

# ==============================================================================
# 0. DIPENDENZE ESTERNE
# ==============================================================================
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# 1. CONFIGURAZIONE ACADOS
# ==============================================================================
set(ACADOS_INSTALL_DIR "/home/thunder_dev/acados") 
set(ACADOS_GENERATED_DIR "${CMAKE_SOURCE_DIR}/scripts/c_generated_code_frankino_throw")

if(NOT EXISTS "${ACADOS_INSTALL_DIR}")
    message(FATAL_ERROR "ACADOS_INSTALL_DIR non trovato: ${ACADOS_INSTALL_DIR}")
endif()

# Cartella dove si trovano thunder_frankino.cpp e frankino_gen.cpp
# Modifica questo path se i file sono altrove!
set(ROBOT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/frankino_generatedFiles")

include_directories(
    ${CMAKE_SOURCE_DIR}/library
    ${CMAKE_SOURCE_DIR}/control_header
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/scripts  
    ${ROBOT_SOURCE_DIR}          # <--- Include per thunder_frankino.h
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/include/blasfeo/include
    ${ACADOS_INSTALL_DIR}/include/hpipm/include
    ${ACADOS_GENERATED_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

link_directories(${ACADOS_INSTALL_DIR}/lib)

# ==============================================================================
# 2. RACCOLTA SORGENTI
# ==============================================================================

# A. I tuoi sorgenti generici in src/
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# B. I sorgenti del Robot (thunder_frankino.cpp, frankino_gen.cpp, ecc.)
file(GLOB ROBOT_SOURCES 
    "${ROBOT_SOURCE_DIR}/*.cpp" 
    "${ROBOT_SOURCE_DIR}/*.c"
)

# C. I sorgenti del Solver Acados (inclusi quelli annidati)
file(GLOB_RECURSE ACADOS_SOLVER_SOURCES "${ACADOS_GENERATED_DIR}/*.c")

# ==============================================================================
# 3. CREAZIONE LIBRERIA PRINCIPALE
# ==============================================================================
# Includiamo TUTTI i sorgenti nella libreria thunder_control
add_library(thunder_control ${SOURCES} ${ROBOT_SOURCES} ${ACADOS_SOLVER_SOURCES})

set_target_properties(thunder_control PROPERTIES OUTPUT_NAME "thunder_control")

target_include_directories(thunder_control PUBLIC 
    ${CMAKE_SOURCE_DIR}/library
    ${ACADOS_GENERATED_DIR}
    ${ROBOT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(thunder_control 
    PUBLIC
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    acados
    hpipm
    blasfeo
    m
)

set_target_properties(thunder_control PROPERTIES 
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${ACADOS_INSTALL_DIR}/lib"
)

# ==============================================================================
# 4. ESEGUIBILE MPC
# ==============================================================================
add_executable(franka_mpc_run ${CMAKE_SOURCE_DIR}/scripts/main_franka_mpc.cpp)
target_link_libraries(franka_mpc_run thunder_control)