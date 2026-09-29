cmake_minimum_required(VERSION 3.16)
# Reuse the flight firmware's ibus component (GPIO 7 wiring unchanged).
# COMPONENTS is restricted so none of the other flight components get
# built - several of them include headers from the flight main/ dir.
set(EXTRA_COMPONENT_DIRS "../../components")
set(COMPONENTS main ibus)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esc_cal_s3)
