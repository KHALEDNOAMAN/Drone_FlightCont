idf_component_register(SRCS "ms5611.c" "ms5611_calc.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver i2c_all)
