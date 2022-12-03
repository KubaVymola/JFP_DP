add_library(app_external_deps INTERFACE)

set(tinyxml2_BUILD_TESTING OFF CACHE INTERNAL "")

add_subdirectory(tinyxml2)

target_link_libraries(app_external_deps INTERFACE
    tinyxml2::tinyxml2
)

target_include_directories(app_external_deps INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/tinyxml2
)
