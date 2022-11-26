add_library(fcs SHARED
    fcs.cpp
    Adafruit_AHRS_NXPFusion.cpp
    Adafruit_AHRS_NXPmatrix.cpp
    Adafruit_AHRS_Madgwick.cpp
)

target_link_libraries(fcs PRIVATE app_external_deps)
