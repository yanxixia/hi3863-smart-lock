/*
 * Copyright (c) 2024 HiSilicon Technologies CO., Ltd.
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <securec.h>
#include "i2c.h"
#include "soc_osal.h"
#include "ssd1306.h"

#define CONFIG_I2C_MASTER_BUS_ID    1
#define I2C_SLAVE2_ADDR             0x3C
#define SSD1306_CTRL_CMD            0x00
#define SSD1306_CTRL_DATA           0x40
#define SSD1306_MASK_CONT           (0x1 << 7)
#define DOUBLE                      2

#if defined(SSD1306_USE_I2C)

void ssd1306_Reset(void)
{
    /* for I2C - do nothing, caller should delay before first use */
}

static uint32_t ssd1306_SendData(uint8_t *buffer, uint32_t size)
{
    uint16_t dev_addr = I2C_SLAVE2_ADDR;
    i2c_data_t data = {0};
    data.send_buf = buffer;
    data.send_len = size;
    uint32_t retval = uapi_i2c_master_write(CONFIG_I2C_MASTER_BUS_ID, dev_addr, &data);
    if (retval != 0) {
        printf("I2cWrite(%02X) failed, %0X!\n", data.send_buf[1], retval);
        return retval;
    }
    return 0;
}

static uint32_t ssd1306_WiteByte(uint8_t regAddr, uint8_t byte)
{
    uint8_t buffer[] = {regAddr, byte};
    return ssd1306_SendData(buffer, sizeof(buffer));
}

/* Send command byte */
void ssd1306_WriteCommand(uint8_t byte)
{
    ssd1306_WiteByte(SSD1306_CTRL_CMD, byte);
}

/* Send data bytes (page by page, 128 bytes max per call) */
void ssd1306_WriteData(uint8_t *buffer, uint32_t buff_size)
{
    /* Each data byte needs a control byte prefix.
     * Buffer: [0x40, D0, 0x40, D1, ..., 0x40, Dn]  */
    uint32_t total = buff_size * 2;
    uint8_t *data = (uint8_t *)osal_kmalloc(total, OSAL_GFP_ATOMIC);
    if (data == NULL) return;
    (void)memset_s(data, total, 0, total);
    for (uint32_t i = 0; i < buff_size; i++) {
        data[i * DOUBLE]     = SSD1306_CTRL_DATA | SSD1306_MASK_CONT;
        data[i * DOUBLE + 1] = buffer[i];
    }
    data[(buff_size - 1) * DOUBLE] = SSD1306_CTRL_DATA;
    ssd1306_SendData(data, total);
    osal_kfree(data);
}

#else
#error "Define SSD1306_USE_I2C or SSD1306_USE_SPI in ssd1306.h"
#endif

/* Screen buffer */
static uint8_t SSD1306_Buffer[SSD1306_BUFFER_SIZE];

/* Screen object */
static SSD1306_t SSD1306;

/* Fills the screen buffer */
SSD1306_Error_t ssd1306_FillBuffer(uint8_t *buf, uint32_t len)
{
    SSD1306_Error_t ret = SSD1306_ERR;
    if (len <= SSD1306_BUFFER_SIZE) {
        memcpy_s(SSD1306_Buffer, len + 1, buf, len);
        ret = SSD1306_OK;
    }
    return ret;
}

void ssd1306_Init_CMD(void)
{
    ssd1306_WriteCommand(0xA4); /* Output follows RAM content */

    ssd1306_WriteCommand(0xD3); /* Set display offset */
    ssd1306_WriteCommand(0x00); /* Not offset */

    ssd1306_WriteCommand(0xD5); /* Set display clock divide ratio */
    ssd1306_WriteCommand(0xF0); /* Ratio */

    ssd1306_WriteCommand(0xD9); /* Set pre-charge period */
    ssd1306_WriteCommand(0x11); /* 0x22 by default */

    ssd1306_WriteCommand(0xDA); /* Set COM pins hardware config */
#if (SSD1306_HEIGHT == 32)
    ssd1306_WriteCommand(0x02);
#elif (SSD1306_HEIGHT == 64)
    ssd1306_WriteCommand(0x12);
#elif (SSD1306_HEIGHT == 128)
    ssd1306_WriteCommand(0x12);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    ssd1306_WriteCommand(0xDB); /* Set VCOMH */
    ssd1306_WriteCommand(0x30); /* 0.83×Vcc */

    ssd1306_WriteCommand(0x8D); /* Enable DC-DC */
    ssd1306_WriteCommand(0x14);
    ssd1306_SetDisplayOn(1);    /* Turn on panel */
}

/* Initialize the OLED screen */
void ssd1306_Init(void)
{
    ssd1306_Reset();

    ssd1306_SetDisplayOn(0); /* display off */

    ssd1306_WriteCommand(0x20); /* Set Memory Addressing Mode */
    ssd1306_WriteCommand(0x00); /* Horizontal addressing mode */

    ssd1306_WriteCommand(0xB0); /* Set page start address */

#ifdef SSD1306_MIRROR_VERT
    ssd1306_WriteCommand(0xC0); /* Mirror vertically */
#else
    ssd1306_WriteCommand(0xC8); /* COM scan direction normal */
#endif

    ssd1306_WriteCommand(0x00); /* Set low column address */
    ssd1306_WriteCommand(0x10); /* Set high column address */

    ssd1306_WriteCommand(0x40); /* Set start line address */

    ssd1306_SetContrast(0xFF);

#ifdef SSD1306_MIRROR_HORIZ
    ssd1306_WriteCommand(0xA0); /* Mirror horizontally */
#else
    ssd1306_WriteCommand(0xA1); /* Segment re-map 0 to 127 */
#endif

#ifdef SSD1306_INVERSE_COLOR
    ssd1306_WriteCommand(0xA7); /* Inverse color */
#else
    ssd1306_WriteCommand(0xA6); /* Normal color */
#endif

    /* Set multiplex ratio */
#if (SSD1306_HEIGHT == 128)
    ssd1306_WriteCommand(0xFF);
#else
    ssd1306_WriteCommand(0xA8);
#endif

#if (SSD1306_HEIGHT == 32)
    ssd1306_WriteCommand(0x1F);
#elif (SSD1306_HEIGHT == 64)
    ssd1306_WriteCommand(0x3F);
#elif (SSD1306_HEIGHT == 128)
    ssd1306_WriteCommand(0x3F);
#else
#error "Only 32, 64, or 128 lines of height are supported!"
#endif

    ssd1306_Init_CMD();

    /* Clear screen */
    ssd1306_Fill(Black);
    ssd1306_UpdateScreen();

    SSD1306.CurrentX    = 0;
    SSD1306.CurrentY    = 0;
    SSD1306.Initialized = 1;
}

/* Fill entire screen with color */
void ssd1306_Fill(SSD1306_COLOR color)
{
    uint32_t i;
    for (i = 0; i < sizeof(SSD1306_Buffer); i++) {
        SSD1306_Buffer[i] = (color == Black) ? 0x00 : 0xFF;
    }
}

/* Write screen buffer to display (page-by-page, safe I2C transfer) */
void ssd1306_UpdateScreen(void)
{
    uint8_t pages = SSD1306_HEIGHT / 8;
    for (uint8_t i = 0; i < pages; i++) {
        ssd1306_WriteCommand(0xB0 + i); /* Set page address */
        ssd1306_WriteCommand(0x00 + SSD1306_X_OFFSET_LOWER);
        ssd1306_WriteCommand(0x10 + SSD1306_X_OFFSET_UPPER);
        ssd1306_WriteData(&SSD1306_Buffer[SSD1306_WIDTH * i], SSD1306_WIDTH);
    }
}

/* Draw a pixel in the screen buffer */
void ssd1306_DrawPixel(uint8_t x, uint8_t y, SSD1306_COLOR color)
{
    if (x >= SSD1306_WIDTH || y >= SSD1306_HEIGHT) return;

    SSD1306_COLOR c = color;
    if (SSD1306.Inverted) {
        c = (SSD1306_COLOR)!c;
    }

    if (c == White) {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] |= (1 << (y & 7));
    } else {
        SSD1306_Buffer[x + (y / 8) * SSD1306_WIDTH] &= ~(1 << (y & 7));
    }
}

/* Draw a character */
char ssd1306_DrawChar(char ch, FontDef Font, SSD1306_COLOR color)
{
    uint32_t i, b, j;

    if ((uint32_t)ch < 32 || (uint32_t)ch > 126) return 0;

    const uint32_t ch_min = 32;
    if (SSD1306_WIDTH < (SSD1306.CurrentX + Font.FontWidth) ||
        SSD1306_HEIGHT < (SSD1306.CurrentY + Font.FontHeight)) {
        return 0;
    }

    for (i = 0; i < Font.FontHeight; i++) {
        b = Font.data[(ch - ch_min) * Font.FontHeight + i];
        for (j = 0; j < Font.FontWidth; j++) {
            if ((b << j) & 0x8000) {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), color);
            } else {
                ssd1306_DrawPixel(SSD1306.CurrentX + j, (SSD1306.CurrentY + i), (SSD1306_COLOR)!color);
            }
        }
    }

    SSD1306.CurrentX += Font.FontWidth;
    return ch;
}

/* Draw a string */
char ssd1306_DrawString(char *str, FontDef Font, SSD1306_COLOR color)
{
    char *p = str;
    while (*p) {
        if (ssd1306_DrawChar(*p, Font, color) != *p) {
            return *p;
        }
        p++;
    }
    return *p;
}

/* Set cursor position */
void ssd1306_SetCursor(uint8_t x, uint8_t y)
{
    SSD1306.CurrentX = x;
    SSD1306.CurrentY = y;
}

/* Draw line (Bresenham) */
void ssd1306_DrawLine(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    int32_t deltaX = abs(x2 - x1);
    int32_t deltaY = abs(y2 - y1);
    int32_t signX  = ((x1 < x2) ? 1 : -1);
    int32_t signY  = ((y1 < y2) ? 1 : -1);
    int32_t error  = deltaX - deltaY;
    int32_t error2;

    ssd1306_DrawPixel(x2, y2, color);
    while ((x1 != x2) || (y1 != y2)) {
        ssd1306_DrawPixel(x1, y1, color);
        error2 = error * 2;
        if (error2 > -deltaY) { error -= deltaY; x1 += signX; }
        if (error2 <  deltaX) { error += deltaX; y1 += signY; }
    }
}

/* Draw polyline */
void ssd1306_DrawPolyline(const SSD1306_VERTEX *par_vertex, uint16_t par_size, SSD1306_COLOR color)
{
    if (par_vertex == NULL) return;
    for (uint16_t i = 1; i < par_size; i++) {
        ssd1306_DrawLine(par_vertex[i - 1].x, par_vertex[i - 1].y,
                         par_vertex[i].x, par_vertex[i].y, color);
    }
}

/* Draw circle */
void ssd1306_DrawCircle(uint8_t par_x, uint8_t par_y, uint8_t par_r, SSD1306_COLOR par_color)
{
    if (par_x >= SSD1306_WIDTH || par_y >= SSD1306_HEIGHT) return;

    int32_t x = -par_r;
    int32_t y = 0;
    int32_t err = 2 - 2 * par_r;
    int32_t e2;

    do {
        ssd1306_DrawPixel(par_x - x, par_y + y, par_color);
        ssd1306_DrawPixel(par_x + x, par_y + y, par_color);
        ssd1306_DrawPixel(par_x + x, par_y - y, par_color);
        ssd1306_DrawPixel(par_x - x, par_y - y, par_color);
        e2 = err;
        if (e2 <= y) {
            y++;
            err = err + (y * 2 + 1);
        }
        if (e2 > x || err > y) {
            x++;
            err = err + (x * 2 + 1);
        }
    } while (x <= 0);
}

/* Draw rectangle */
void ssd1306_DrawRectangle(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, SSD1306_COLOR color)
{
    ssd1306_DrawLine(x1, y1, x2, y1, color);
    ssd1306_DrawLine(x2, y1, x2, y2, color);
    ssd1306_DrawLine(x2, y2, x1, y2, color);
    ssd1306_DrawLine(x1, y2, x1, y1, color);
}

/* Draw bitmap */
void ssd1306_DrawBitmap(const uint8_t *bitmap, uint32_t size)
{
    uint8_t rows = (uint8_t)(size * 8 / SSD1306_WIDTH);
    if (rows > SSD1306_HEIGHT) rows = SSD1306_HEIGHT;

    for (uint8_t y = 0; y < rows; y++) {
        for (uint8_t x = 0; x < SSD1306_WIDTH; x++) {
            uint8_t byte = bitmap[(y * SSD1306_WIDTH / 8) + (x / 8)];
            uint8_t bit  = byte & (0x80 >> (x & 7));
            ssd1306_DrawPixel(x, y, bit ? White : Black);
        }
    }
}

void ssd1306_DrawRegion(uint8_t x, uint8_t y, uint8_t w, const uint8_t *data, uint32_t size)
{
    if (x + w > SSD1306_WIDTH || y + w > SSD1306_HEIGHT || w == 0) {
        printf("%dx%d @ %d,%d out of range or invalid!\r\n", w, w, x, y);
        return;
    }

    uint8_t rows = (uint8_t)(size * 8 / w);
    for (uint8_t i = 0; i < rows; i++) {
        uint32_t base = i * w / 8;
        for (uint8_t j = 0; j < w; j++) {
            uint32_t idx  = base + (j / 8);
            uint8_t  byte = (idx < size) ? data[idx] : 0;
            uint8_t  bit  = byte & (0x80 >> (j & 7));
            ssd1306_DrawPixel(x + j, y + i, bit ? White : Black);
        }
    }
}

void ssd1306_SetContrast(const uint8_t value)
{
    ssd1306_WriteCommand(0x81);
    ssd1306_WriteCommand(value);
}

void ssd1306_SetDisplayOn(const uint8_t on)
{
    if (on) {
        ssd1306_WriteCommand(0xAF);
        SSD1306.DisplayOn = 1;
    } else {
        ssd1306_WriteCommand(0xAE);
        SSD1306.DisplayOn = 0;
    }
}

uint8_t ssd1306_GetDisplayOn(void)
{
    return SSD1306.DisplayOn;
}

int g_ssd1306_current_loc_v = 0;
#define SSD1306_INTERVAL_V (15)

void ssd1306_ClearOLED(void)
{
    ssd1306_Fill(Black);
    g_ssd1306_current_loc_v = 0;
}

void ssd1306_printf(char *fmt, ...)
{
    if (fmt == NULL) return;
    char buffer[24];
    va_list argList;
    va_start(argList, fmt);
    int ret = vsnprintf_s(buffer, sizeof(buffer), sizeof(buffer) - 1, fmt, argList);
    va_end(argList);
    if (ret < 0) {
        printf("buffer is null\r\n");
        return;
    }
    ssd1306_SetCursor(0, g_ssd1306_current_loc_v);
    ssd1306_DrawString(buffer, Font_7x10, White);
    ssd1306_UpdateScreen();
    g_ssd1306_current_loc_v += SSD1306_INTERVAL_V;
}
