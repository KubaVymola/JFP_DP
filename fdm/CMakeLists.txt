cmake_minimum_required(VERSION 3.13)

# ccmake
# cmake -G <ninja|make> ..
# cmake --build .

################################################################################
# Define CMAKE constants
################################################################################

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

################################################################################
# Define Project
################################################################################

project(fdm VERSION 0.1 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(functions)


################################################################################
# Define the main executable
################################################################################

add_subdirectory(lib)

add_subdirectory(src)
