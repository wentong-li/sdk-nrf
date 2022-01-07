#
# Copyright (c) 2020 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

add_compile_definitions(CONFIG_SW_CODEC_LC3_ENC_SAMPLE_RATE_48KHZ_SUPPORT=1)
add_compile_definitions(CONFIG_SW_CODEC_LC3_DEC_SAMPLE_RATE_48KHZ_SUPPORT=1)
add_compile_definitions(CONFIG_SW_CODEC_LC3_7_5_MS_FRAMESIZE=1)

target_sources(app PRIVATE ${app_sources})

target_sources(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/modules/lc3/nrf5340_audio/nrf5340_audio_app/src/utils/tone.c
  )

target_include_directories(app
  PRIVATE
  ${ZEPHYR_BASE}/../nrf/modules/lc3/include/
  )
