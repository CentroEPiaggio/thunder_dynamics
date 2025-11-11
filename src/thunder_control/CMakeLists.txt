# cmake and c++ version
cmake_minimum_required(VERSION 3.14)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# project
project(thunder_control VERSION 0.2)
# project(library)

# Specifica la cartella di output per i file compilati
# set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/build)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set (CMAKE_CXX_FLAGS "-g3 -lstdc++fs -std=c++17")
# set (CMAKE_CXX_FLAGS_DEBUG "-g3")

# Aggiungi la cartella "library" per cercare gli header
include_directories(
    ${CMAKE_SOURCE_DIR}/library
    ${CMAKE_SOURCE_DIR}/control_header
)

# Aggiungi tutti i file di origine nella cartella "src"
file(GLOB SOURCES 
    ${CMAKE_SOURCE_DIR}/src/*.cpp
)

# Definisci la tua libreria
add_library(thunder_control ${SOURCES})

# Specifica il nome dell'output della libreria
set_target_properties(thunder_control PROPERTIES OUTPUT_NAME "thunder_control")

# Indica che altri progetti possono trovare questa libreria
target_include_directories(thunder_control PUBLIC ${CMAKE_SOURCE_DIR}/library)

find_package(yaml-cpp REQUIRED)
target_link_libraries(thunder_control yaml-cpp::yaml-cpp)

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
add_executable(thunder_control_test ${CMAKE_SOURCE_DIR}/thunder_control_test.cpp)
target_link_libraries(thunder_control_test thunder_control)
