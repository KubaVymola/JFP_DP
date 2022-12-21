add_executable(fdm "${JFP_SOURCES}"
    main.cpp
    ws_server.cpp
    jsbsim_interface.cpp
    sitl_interface.cpp
    hitl_interface.cpp
    sim_events.cpp
    timer.cpp
)

target_link_libraries(fdm PRIVATE app_external_deps)
