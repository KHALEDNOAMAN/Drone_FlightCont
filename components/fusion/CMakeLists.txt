idf_component_register(SRCS "fusion.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver common lsm6dsox dps310 ms5611 mmc5603 ina226 vl53l0x esp_timer telemetry params)