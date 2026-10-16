# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(remote_thermometer)

target_sources(app PRIVATE
  src/main.c
  src/app_config.c
  src/temp_service.c
  src/max31865.c
  )
