idf_component_register(SRCS "pmw3901.c" "pmw3901_calc.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_spi
                       PRIV_REQUIRES common esp_driver_gpio esp_timer)
