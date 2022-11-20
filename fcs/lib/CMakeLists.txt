add_library(app_external_deps INTERFACE)

set(JSON_BuildTests OFF CACHE INTERNAL "")
set(tinyxml2_BUILD_TESTING OFF CACHE INTERNAL "")

add_subdirectory(json)
add_subdirectory(madgwick_algorithm_c)
add_subdirectory(tinyxml2)

target_link_libraries(app_external_deps INTERFACE
    nlohmann_json::nlohmann_json
    tinyxml2::tinyxml2
    madgwick_algo::madgwick_algo
)

target_include_directories(app_external_deps INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/json/single_include
    ${CMAKE_CURRENT_LIST_DIR}/tinyxml2
    ${CMAKE_CURRENT_LIST_DIR}/madgwick_algorithm_c
)
