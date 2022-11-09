cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

set(JSON_BuildTests OFF CACHE INTERNAL "")

project(fcs VERSION 0.1 LANGUAGES CXX)

add_subdirectory(lib/json)
add_subdirectory(lib/tinyxml2)

add_library(fcs SHARED fcs.cpp)

list(APPEND FCS_LIBS nlohmann_json::nlohmann_json)
list(APPEND FCS_LIBS tinyxml2::tinyxml2)
list(APPEND FCS_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src)
list(APPEND FCS_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/json/single_include)
list(APPEND FCS_INCLUDE_DIRS ${CMAKE_CURRENT_LIST_DIR}/lib/tinyxml2)

target_link_libraries(fcs PUBLIC "${FCS_LIBS}")
target_include_directories(fcs PUBLIC "${FCS_INCLUDE_DIRS}")
