add_library(app_external_deps INTERFACE)

set(BUILD_DOCS OFF CACHE INTERNAL "")
set(JSON_BuildTests OFF CACHE INTERNAL "")
set(tinyxml2_BUILD_TESTING OFF CACHE INTERNAL "")

add_subdirectory(jsbsim)
add_subdirectory(json)
add_subdirectory(tinyxml2)

find_package(websocketpp REQUIRED)

target_link_libraries(app_external_deps INTERFACE
    websocketpp::websocketpp
    libJSBSim
    dl
    nlohmann_json::nlohmann_json
    tinyxml2::tinyxml2
)

target_include_directories(app_external_deps INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/jsbsim/src
    ${CMAKE_CURRENT_LIST_DIR}/json/single_include
    ${CMAKE_CURRENT_LIST_DIR}/tinyxml2
)
