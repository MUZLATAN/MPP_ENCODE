cmake_minimum_required(VERSION 3.5.1)


# This file is used for cross compilation on hisi chip platform

set(CMAKE_SYSTEM_NAME Linux)

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)


project(MPPEncode)

set(CMAKE_VERBOSE_MAKEFILE ON)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_CXX_FLAGS "-O0 -g")

include_directories(
        ${CMAKE_CURRENT_LIST_DIR}/dev/include
        ${CMAKE_CURRENT_LIST_DIR}/dev/include/rockchip
)

link_directories(
        ${CMAKE_CURRENT_LIST_DIR}/dev/lib
)
AUX_SOURCE_DIRECTORY(. SRCS)
add_executable(MPPEncode ${SRCS} MppEncoder.cpp MppEncoder.h main.cpp)

target_link_libraries(MPPEncode
        rockchip_mpp
        utils
        pthread
        c
        # opencv_imgcodecs
        # opencv_core
        # opencv_imgproc
        # opencv_highgui
        # opencv_videoio
        )