#===============================================================================
# @brief    cmake file for DEMO01 (Vehicle End)
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
#===============================================================================
set(SLE_UART_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../vendor/BearPi-Pico_H3863/products/sle_uart")

set(SOURCES
    "${SOURCES}"
    "${CMAKE_CURRENT_SOURCE_DIR}/vehicle_main.c"
PARENT_SCOPE)
