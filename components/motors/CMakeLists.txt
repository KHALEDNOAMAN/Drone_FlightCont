idf_component_register(
    SRCS "motors.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES esp_driver_ledc
)
