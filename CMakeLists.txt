cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(drone_fc)

# S3 FPU is single-precision only; every accidental double is a soft-float
# library call in the 250 Hz loop. Warn on any implicit float->double in
# project code (IDF's own components are left alone - not ours to fix).
foreach(comp main common i2c_all ibus mixer motors pid sys_ctrl tbsM10 ms5611
             telemetry lsm6dsox dps310 mmc5603 fusion pmw3901 blackbox params
             ina226 vl53l0x gnss_guard)
    idf_component_get_property(lib ${comp} COMPONENT_LIB)
    target_compile_options(${lib} PRIVATE -Wdouble-promotion)
endforeach()
