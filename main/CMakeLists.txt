idf_component_register(SRCS "main.c" "fc_tasks.c" "debug_stream.c"
                        INCLUDE_DIRS ""
                        REQUIRES sys_ctrl ibus motors mixer common pid fusion
                                 dps310 mmc5603 lsm6dsox telemetry params blackbox ina226 vl53l0x
                                 i2c_all pmw3901 tbsM10 gnss_guard
                        PRIV_REQUIRES nvs_flash driver freertos esp_timer esp_driver_usb_serial_jtag)
