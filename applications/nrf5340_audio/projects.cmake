#
# Copyright (c) 2018 Nordic Semiconductor ASA
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

if("x-$ENV{ZEPHYR_BASE}" STREQUAL "x-")
    message(FATAL_ERROR
   "         ERROR            \n\
     YOU HAVE TO SET THE ZEPHYR_BASE ENV VARIABLE\n\
     This is usually set by running west update ")
endif("x-$ENV{ZEPHYR_BASE}" STREQUAL "x-")

# Set the nRF5340 Audio version. x.x.99 indicates master branch/cutting edge
set(NRF5340_AUDIO_RELEASE_VERSION 0.4.99)

# Set paths
set(NRF5340_AUDIO_REPO_ROOT ${CMAKE_CURRENT_LIST_DIR} CACHE PATH "nRF5340 Audio root directory")
set(ENV{NRF5340_AUDIO_REPO_ROOT} $NRF5340_AUDIO_REPO_ROOT)
message("NRF5340_AUDIO_REPO_ROOT is set to: ${NRF5340_AUDIO_REPO_ROOT}")

set(NRF5340_AUDIO_SRC ${NRF5340_AUDIO_REPO_ROOT}/nrf5340_audio_app/src)
set(EXT_ROOT ${NRF5340_AUDIO_REPO_ROOT}/ext)
set(LC3_ROOT ${NRF5340_AUDIO_REPO_ROOT}/../../../modules/lib/lc3)
set(CIRRUS_ROOT ${EXT_ROOT}/cirrus_logic)

# Add core specific config
set(CONF_FILE "${CONF_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/prj.conf")

# Check if -DCMAKE_BUILD_TYPE=RELEASE, if so, add the prj_release.conf to the list of configs.
if(CMAKE_BUILD_TYPE MATCHES RELEASE)
    # Add generic release config
    set(CONF_FILE "${CONF_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/prj_release.conf")
elseif(CMAKE_BUILD_TYPE MATCHES DEBUG)
    message("
          ------------------------------------------------------------
          ---     DEBUG mode, enabling power hungry debug features ---
          ------------------------------------------------------------
          ")
    add_definitions(-DDEBUG)
    # Add core specific debug config
    set(CONF_FILE "${CONF_FILE} ${CMAKE_CURRENT_SOURCE_DIR}/prj_debug.conf")
else()
    message(WARNING "CMAKE_BUILD_TYPE is neither RELEASE nor DEBUG. This may be OK for tests")
endif()
message("*.conf files used: ${CONF_FILE}")
