# cmake and c++ version
cmake_minimum_required(VERSION 3.14)

# Impostiamo lo standard C++
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

project(thunder_control VERSION 0.2)

# Setup Output
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# Flags: -O3 per ottimizzazione (importante per MPC), -g3 per debug se serve
set(CMAKE_CXX_FLAGS "-O3 -g3 -lstdc++fs -std=c++17 -fPIC")

# ==============================================================================
# 0. DIPENDENZE ESTERNE
# ==============================================================================
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)

# ==============================================================================
# 1. CONFIGURAZIONE PATH (Punto Critico!)
# ==============================================================================
set(ACADOS_INSTALL_DIR "/home/thunder_dev/acados") 

# ATTENZIONE: Questo deve coincidere con 'ocp.code_export_directory' nel tuo Python!
# Se nel python hai messo "c_generated_code_tracking", metti lo stesso nome qui sotto.
set(ACADOS_GENERATED_DIR "${CMAKE_SOURCE_DIR}/scripts/c_generated_code_tracking")

if(NOT EXISTS "${ACADOS_INSTALL_DIR}")
    message(FATAL_ERROR "ACADOS_INSTALL_DIR non trovato: ${ACADOS_INSTALL_DIR}")
endif()

# Cartella primitive robot (thunder_frankino.cpp, etc.)
set(ROBOT_SOURCE_DIR "${CMAKE_SOURCE_DIR}/frankino_generatedFiles")

# Directory per Linker e Include
link_directories(${ACADOS_INSTALL_DIR}/lib)

include_directories(
    ${CMAKE_SOURCE_DIR}/library
    ${CMAKE_SOURCE_DIR}/control_header
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/scripts       # Per trovare header nella cartella scripts
    ${ROBOT_SOURCE_DIR}               # Per includere "thunder_frankino.h" direttamente
    ${ACADOS_INSTALL_DIR}/include
    ${ACADOS_INSTALL_DIR}/include/blasfeo/include
    ${ACADOS_INSTALL_DIR}/include/hpipm/include
    ${ACADOS_GENERATED_DIR}           # Per trovare acados_solver_xxx.h
    ${EIGEN3_INCLUDE_DIRS}
)

# ==============================================================================
# 2. RACCOLTA SORGENTI
# ==============================================================================

# A. Sorgenti generici
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# B. Sorgenti Robot (Primitive CasADi esportate)
file(GLOB ROBOT_SOURCES 
    "${ROBOT_SOURCE_DIR}/*.cpp" 
    "${ROBOT_SOURCE_DIR}/*.c"
)
# C. Sorgenti Solver Acados Generati (C Code)
file(GLOB_RECURSE ACADOS_SOLVER_SOURCES "${ACADOS_GENERATED_DIR}/*.c")

# --- FIX: RIMUOVI I MAIN GENERATI DA ACADOS ---
# Filtriamo via dalla lista qualsiasi file che contenga "main_" nel nome
list(FILTER ACADOS_SOLVER_SOURCES EXCLUDE REGEX "/main_.*\\.c$")

# ==============================================================================
# 3. LIBRERIA CORE (thunder_control)
# ==============================================================================
add_library(thunder_control SHARED ${SOURCES} ${ROBOT_SOURCES} ${ACADOS_SOLVER_SOURCES})

set_target_properties(thunder_control PROPERTIES OUTPUT_NAME "thunder_control")

target_link_libraries(thunder_control 
    PUBLIC
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    acados
    hpipm
    blasfeo
    m
)

# RPATH per trovare le lib di acados a runtime senza settare LD_LIBRARY_PATH
set_target_properties(thunder_control PROPERTIES 
    BUILD_WITH_INSTALL_RPATH TRUE
    INSTALL_RPATH "${ACADOS_INSTALL_DIR}/lib"
)

# ==============================================================================
# 4. ESEGUIBILE MPC (Cascade Control)
# ==============================================================================
# Compila il main che hai creato
add_executable(franka_mpc_run ${CMAKE_SOURCE_DIR}/scripts/main_franka_mpc.cpp)

# Linka contro la tua libreria core (che contiene robot + solver)
target_link_libraries(franka_mpc_run thunder_control)