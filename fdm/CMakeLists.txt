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

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(BUILD_DOCS OFF CACHE INTERNAL "")
set(tinyxml2_BUILD_TESTING OFF CACHE INTERNAL "")

################################################################################
# Define Project
################################################################################

project(fdm VERSION 0.1 LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
include(functions)

################################################################################
# Find installed dependencies
################################################################################

find_package(websocketpp REQUIRED)

################################################################################
# Add local dependencies
# TODO there should be a separate CMakeLists.txt in lib directory
# TODO have external libs directory and a top-level CMakeLists.txt to share
# ^ the libraries between projects
################################################################################

add_subdirectory(lib/jsbsim)
add_subdirectory(lib/json)
add_subdirectory(lib/tinyxml2)

################################################################################
# Define the main executable
################################################################################

add_executable(fdm src/main.cpp src/ws_server.cpp src/jsbsim_interface.cpp src/fcs_interface.cpp)

list(APPEND JFP_LIBS websocketpp::websocketpp)
list(APPEND JFP_LIBS libJSBSim)
list(APPEND JFP_LIBS dl)
list(APPEND JFP_LIBS nlohmann_json::nlohmann_json)
list(APPEND JFP_LIBS tinyxml2::tinyxml2)
list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/jsbsim/src)
list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/json/single_include)
list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/tinyxml2)

target_link_libraries(fdm PUBLIC "${JFP_LIBS}")
target_include_directories(fdm PUBLIC "${JFP_INCLUDE_DIRS}")
