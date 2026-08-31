cmake_minimum_required(VERSION 3.16)

# Keep this diagnostic firmware small and independent of the car application.
set(COMPONENTS main)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(camera-uvc-test)
