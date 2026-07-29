#-------------------------------------------------------------------------------
# Zephyr Example Application
#
# Copyright (c) 2021 Nordic Semiconductor ASA
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)

# ZEPHYR_EXTRA_MODULES must be set BEFORE find_package(Zephyr) so that
# the module is discovered and its CMakeLists.txt (zephyr_include_directories,
# zephyr_library_sources, …) is processed during the Zephyr configuration.
list(APPEND ZEPHYR_EXTRA_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/model)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C CXX)

target_sources(app PRIVATE
    src/main.c
    src/kws.cpp
    src/pitch.c
)
