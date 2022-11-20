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

# list(APPEND JFP_SOURCES
#     src/main.cpp
#     src/ws_server.cpp
#     src/jsbsim_interface.cpp
#     src/sitl_interface.cpp
#     src/sim_events.cpp)

# add_executable(fdm "${JFP_SOURCES}")

# list(APPEND JFP_LIBS websocketpp::websocketpp)
# list(APPEND JFP_LIBS libJSBSim)
# list(APPEND JFP_LIBS dl)
# list(APPEND JFP_LIBS nlohmann_json::nlohmann_json)
# list(APPEND JFP_LIBS tinyxml2::tinyxml2)
# list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
# list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/jsbsim/src)
# list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/json/single_include)
# list(APPEND JFP_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/tinyxml2)

# target_link_libraries(fdm PUBLIC "${JFP_LIBS}")
# target_include_directories(fdm PUBLIC "${JFP_INCLUDE_DIRS}")

add_subdirectory(lib)

add_subdirectory(src)
