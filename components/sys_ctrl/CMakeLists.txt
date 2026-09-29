idf_component_register(SRCS "sys_ctrl.c"
                       INCLUDE_DIRS "."
                       REQUIRES common i2c_all fusion tbsM10 telemetry driver ina226 vl53l0x params)




