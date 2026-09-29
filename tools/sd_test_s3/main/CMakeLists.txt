idf_component_register(SRCS "main.c"
                       PRIV_REQUIRES esp_driver_spi esp_driver_sdspi
                                     esp_driver_gpio sdmmc esp_timer
                       INCLUDE_DIRS "")
