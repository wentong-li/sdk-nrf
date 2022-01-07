#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.13.1)

set(REQ_C_COMPILER_VERSION 9.2.1)
set(REQ_C_COMPILER arm-none-eabi-gcc)
set(DESIRED_ZEPHYR_SDK_VERSION 0.13.1)
# Checks which compiler is used. GNU Arm Embedded Toolchain is strongly preferred.
# https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads
# E.g. for compiler version 10.2.1: the "10-2020-q4-major" release includes this. You have to check which
# GNU release contains the correct compiler.

if(DEFINED ENV{ZEPHYR_TOOLCHAIN_VARIANT})
    if ($ENV{ZEPHYR_TOOLCHAIN_VARIANT} STREQUAL "gnuarmemb")
        if(NOT DEFINED ENV{GNUARMEMB_TOOLCHAIN_PATH})
                message(FATAL_ERROR "The env variable GNUARMEMB_TOOLCHAIN_PATH is not set. It must point to the gnuarmemb folder")
        endif()
    else()
        message(FATAL_ERROR "The env variable ZEPHYR_TOOLCHAIN_VARIANT must be gnuarmemb. Value now: $ENV{ZEPHYR_TOOLCHAIN_VARIANT}" )
    endif()
else()
    message(FATAL_ERROR "The env variable ZEPHYR_TOOLCHAIN_VARIANT is not set. Should preferably be set to gnuarmemb")
endif()

if (NOT ${CMAKE_C_COMPILER} MATCHES ${REQ_C_COMPILER})
        message(FATAL_ERROR "The compiler is ${CMAKE_C_COMPILER} must be of type: ${REQ_C_COMPILER}")
endif()

if(NOT CMAKE_C_COMPILER_VERSION)
    message(FATAL_ERROR "CMAKE_C_COMPILER_VERSION was not automatically detected. Is GNU Arm Embedded Toolchain installed?")
elseif(NOT (CMAKE_C_COMPILER_VERSION VERSION_EQUAL ${REQ_C_COMPILER_VERSION}))
    message(FATAL_ERROR "The C compiler version is: ${CMAKE_C_COMPILER_VERSION}. It must be: ${REQ_C_COMPILER_VERSION}")
endif()

# The ZEPHYR_SDK_INSTALL_DIR may not be defined on all systems. If it exists, check version
if(DEFINED ENV{ZEPHYR_SDK_INSTALL_DIR})
    set(ZEPHYR_SDK_VERSION 0)
    file(READ $ENV{ZEPHYR_SDK_INSTALL_DIR}/sdk_version ZEPHYR_SDK_VERSION)
    if (NOT ZEPHYR_SDK_VERSION VERSION_EQUAL DESIRED_ZEPHYR_SDK_VERSION)
        message(WARNING "The Zephyr SDK version is: ${ZEPHYR_SDK_VERSION} - It should preferably be: ${DESIRED_ZEPHYR_SDK_VERSION}")
    else()
        message("Zephyr SDK version is ${ZEPHYR_SDK_VERSION}")
    endif()
endif()

