#===============================================================================
# @brief    cmake file
# Copyright (c) HiSilicon (Shanghai) Technologies Co., Ltd. 2023-2023. All rights reserved.
#===============================================================================
set(SLE_UART_ROOT
    "${CMAKE_CURRENT_SOURCE_DIR}/../../../../../vendor/BearPi-Pico_H3863/products/sle_uart")

set(SOURCES 
    "${SOURCES}"
    "${CMAKE_CURRENT_SOURCE_DIR}/mobile_main.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/ssd1306_fonts.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/ssd1306.c"
PARENT_SCOPE)
