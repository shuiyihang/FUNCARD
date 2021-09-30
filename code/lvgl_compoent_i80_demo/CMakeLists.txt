# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(DIRS "$ENV{IDF_PATH}/projects/new_lvgl")
list(APPEND EXTRA_COMPONENT_DIRS 
                                ${DIRS}/components
                                ${DIRS}/components/audio
                                ${DIRS}/examples/common_components
                                ${DIRS}/components/bus
                                ${DIRS}/components/button
                                ${DIRS}/components/display
                                ${DIRS}/components/display/digital_tube
                                ${DIRS}/components/expander/io_expander
                                ${DIRS}/components/gui
                                ${DIRS}/components/led
                                ${DIRS}/components/motor
                                ${DIRS}/components/sensors
                                ${DIRS}/components/sensors/gesture
                                ${DIRS}/components/sensors/humiture
                                ${DIRS}/components/sensors/imu
                                ${DIRS}/components/sensors/light_sensor
                                ${DIRS}/components/sensors/pressure
                                ${DIRS}/components/storage
                                ${DIRS}/components/storage/eeprom)




include($ENV{IDF_PATH}/tools/cmake/project.cmake)

message(STATUS "App \"${DIRS}\" version: ${DIRS}")
project(pro_lvgl)
