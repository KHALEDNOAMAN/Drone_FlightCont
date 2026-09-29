idf_component_register(SRCS "tbsM10.c" "ubx.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver common esp_timer gnss_guard)
