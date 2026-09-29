idf_component_register(SRCS "mmc5603.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver nvs_flash esp_timer i2c_all)
