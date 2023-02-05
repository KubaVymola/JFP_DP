add_executable(fdm "${JFP_SOURCES}"
    command_interface.cpp
    # j_packets.cpp
    jsbsim_interface.cpp
    main.cpp
    realtime_loop.cpp
    serial_interface.cpp
    sim_events.cpp
    sitl_interface.cpp
    state_logger.cpp
    timer.cpp
    ws_server.cpp
    ../../shared/j_packets.cpp
)

target_include_directories(fdm PRIVATE ../../shared)

target_link_libraries(fdm PRIVATE app_external_deps)


