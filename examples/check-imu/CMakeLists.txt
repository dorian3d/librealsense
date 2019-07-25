# License: Apache 2.0. See LICENSE file in root directory.
# Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesCheckIMU)

if(BUILD_EXAMPLES)
    add_executable(rs-check-imu rs-check-imu.cpp)
    set_property(TARGET rs-check-imu PROPERTY CXX_STANDARD 11)
    target_link_libraries(rs-check-imu ${DEPENDENCIES})
    set_target_properties (rs-check-imu PROPERTIES FOLDER Examples)
    install(TARGETS rs-check-imu RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()
