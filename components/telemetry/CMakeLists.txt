idf_component_register(SRCS "telemetry.c" "telem_line.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver common params ina226 vl53l0x)