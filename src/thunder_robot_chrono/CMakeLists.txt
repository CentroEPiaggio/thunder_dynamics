# cmake and c++ version
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# project
project(thunder_robot VERSION 0.2)
# project(library)

# Specifica la cartella di output per i file compilati
# set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set (CMAKE_CXX_FLAGS "-g3 -lstdc++fs -std=c++17")
# set (CMAKE_CXX_FLAGS_DEBUG "-g3")

# Aggiungi la cartella "library" per cercare gli header
include_directories(${CMAKE_SOURCE_DIR}/library)

# Aggiungi tutti i file di origine nella cartella "src"
file(GLOB SOURCES ${CMAKE_SOURCE_DIR}/src/*.cpp)

# Definisci la tua libreria
add_library(thunder_robot ${SOURCES})

# Specifica il nome dell'output della libreria
set_target_properties(thunder_robot PROPERTIES OUTPUT_NAME "thunder_robot")

# Indica che altri progetti possono trovare questa libreria
target_include_directories(thunder_robot PUBLIC ${CMAKE_SOURCE_DIR}/library)

find_package(yaml-cpp REQUIRED)
target_link_libraries(thunder_robot yaml-cpp::yaml-cpp)

# Puoi aggiungere altre impostazioni di compilazione qui, se necessario

# Esempio di aggiunta di dipendenze esterne:
# find_package(OpenGL REQUIRED)
# target_link_libraries(library ${OPENGL_LIBRARIES})

# Esempio di aggiunta di altre librerie personalizzate:
# target_link_libraries(library my_custom_library)

# Esempio di specifica di file di origine aggiuntivi:
# file(GLOB MORE_SOURCES ${CMAKE_SOURCE_DIR}/src/more/*.cpp)
# target_sources(library PRIVATE ${MORE_SOURCES})

# Esempio di definizione di variabili di definizione del preprocessore:
# target_compile_definitions(library PRIVATE -DSOME_DEFINE)

# Esempio di creazione di un eseguibile che utilizza la tua libreria
add_executable(thunder_robot_test_chrono ${CMAKE_SOURCE_DIR}/thunder_robot_test_chrono.cpp)
target_link_libraries(thunder_robot_test_chrono thunder_robot)
