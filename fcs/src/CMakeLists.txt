add_library(fcs SHARED
    fcs.cpp
)

target_link_libraries(fcs PRIVATE app_external_deps)
