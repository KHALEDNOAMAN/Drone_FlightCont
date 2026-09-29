idf_component_register(
    SRCS "blackbox.c"
    INCLUDE_DIRS "."
    PRIV_REQUIRES params common esp_driver_spi esp_driver_sdspi esp_driver_gpio sdmmc esp_timer
)
