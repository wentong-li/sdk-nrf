cmake_minimum_required(VERSION 3.10.0)

target_sources(app PRIVATE ${CIRRUS_ROOT}/cs47l63/cs47l63.c)
target_sources(app PRIVATE ${CIRRUS_ROOT}/cs47l63/config/cs47l63_syscfg_regs.c)
target_sources(app PRIVATE ${CIRRUS_ROOT}/common/fw_img_v1.c)

zephyr_library_include_directories(app PRIVATE ${CIRRUS_ROOT}/common/)
zephyr_library_include_directories(app PRIVATE ${CIRRUS_ROOT}/cs47l63)
zephyr_library_include_directories(app PRIVATE ${CIRRUS_ROOT}/cs47l63/bsp)
zephyr_library_include_directories(app PRIVATE ${CIRRUS_ROOT}/cs47l63/config)
zephyr_library_include_directories(app PRIVATE ${CIRRUS_ROOT}/cs47l63/fw_pt)
