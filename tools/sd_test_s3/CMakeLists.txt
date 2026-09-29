cmake_minimum_required(VERSION 3.16)
# Standalone: nothing from the flight firmware is built, so the SD module
# is tested against ESP-IDF's own sdspi driver - the same driver the
# blackbox uses - and nothing else.
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sd_test_s3)
