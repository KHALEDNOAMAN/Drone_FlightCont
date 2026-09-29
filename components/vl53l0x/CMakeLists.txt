idf_component_register(SRCS "vl53l0x.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver i2c_all esp_timer)
