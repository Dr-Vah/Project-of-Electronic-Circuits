cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS
    "${CMAKE_CURRENT_LIST_DIR}/../Task2/components/car_control")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(WhiteBallDetection)
