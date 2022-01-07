#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

target_sources(app PRIVATE ${app_sources})

target_sources(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/applications/nrf5340_audio/nrf5340_audio_app/src/utils/data_fifo.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/applications/nrf5340_audio/nrf5340_audio_app/src/utils/
  ${ZEPHYR_BASE}/../nrf/applications/nrf5340_audio/nrf5340_audio_app/src/utils/macros/
  )
