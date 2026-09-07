cmake_minimum_required(VERSION 3.16)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(oled_test)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/components/mqtt/esp-mqtt)