/*
 * Copyright (c) 2011, The Linux Foundation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name of The Linux Foundation nor
 *    the names of its contributors may be used to endorse or promote
 *    products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <linux/gpio.h>
#include <linux/delay.h>

#define VEE_RESET              20
#define LCD_RESET              180
#define PANEL_640x360_60	1
#define PANEL_640x400_60	0
#define PANEL_640x360_50	0
#define PANEL_640x360_60_14	0

#define PANEL_GPIO_BASE	902
#define SPI_CS_N1	PANEL_GPIO_BASE + 18
#define SPI_CS_N2	PANEL_GPIO_BASE + 32
#define SPI_SCLK	PANEL_GPIO_BASE + 19
#define SPI_MOSI	PANEL_GPIO_BASE + 16
#define SPI_MISO	PANEL_GPIO_BASE + 17

#define GPIO26_GPIO_CNTRL 0x169	/* backlight */
#define PANEL1_ON	0x09 // mclkpol 1:0x07 0:0x05
#define PANEL2_ON	0x05 // mclkpol 1:0x0b 0:0x09
#define PANEL1_OFF	0x06 // mclkpol 1:0x06 0:0x04
#define PANEL2_OFF	0x0a // mclkpol 1:0x0a 0:0x08

#define REG_BRTLUM_DIS	0x08
#define REG_BRT			0x25
#define REG_RBRT		0x26
#define REG_GBRT		0x27
#define REG_BBRT		0x28
#define REG_LUM			0x2a
#define MAX_LUM			255 //org default 255

struct panel_spi_data {
	unsigned addr;
	unsigned data;
};

static struct panel_spi_data init_sequence[] = {
#if PANEL_640x360_60
	{0x00, 0x0F},
	{0x01, 0x0C},
	{0x02, 0x00},
	{0x03, 0x00},
	{0x04, 0x3F},
	{0x05, 0xC8},
	{0x06, 0x00},
	{0x07, 0x40},
	{0x08, 0x00},// invalid preset lum:4 brt:1 
	{0x09, 0x00},
	{0x0A, 0x10},
	{0x0B, 0x00},
	{0x0C, 0x00},
	{0x0D, 0x00},
	{0x0E, 0x00},
	{0x0F, 0x56},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x16, 0x00},
	{0x17, 0x00},
	{0x18, 0x00},
	{0x19, 0x00},
	{0x1A, 0x00},
	{0x1B, 0x00},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x1E, 0x00},
	{0x1F, 0x00},
	{0x20, 0x01},
	{0x21, 0x00},
	{0x22, 0x40},
	{0x23, 0x40},
	{0x24, 0x40},
	{0x25, 0x80},
	{0x26, 0x40},
	{0x27, 0x40},
	{0x28, 0x40},
	{0x29, 0x0B},
	{0x2A, 0xBE},
	{0x2B, 0x3C},
	{0x2C, 0x02},
	{0x2D, 0x7A},
	{0x2E, 0x02},
	{0x2F, 0xFA},
	{0x30, 0x26},
	{0x31, 0x01},
	{0x32, 0x8E},
	{0x33, 0x00},
	{0x34, 0x03},
	{0x35, 0x5A},
	{0x36, 0x00},
	{0x37, 0x76},
	{0x38, 0x02},
	{0x39, 0xFE},
	{0x3A, 0x02},
	{0x3B, 0x0D},
	{0x3C, 0x00},
	{0x3D, 0x1B},
	{0x3E, 0x00},
	{0x3F, 0x1C},
	{0x40, 0x01},
	{0x41, 0xF3},
	{0x42, 0x01},
	{0x43, 0xF4},
	{0x44, 0x80},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x41},
	{0x48, 0x08},
	{0x49, 0x02},
	{0x4A, 0xFC},
	{0x4B, 0x08},
	{0x4C, 0x16},
	{0x4D, 0x08},
	{0x4E, 0x00},
	{0x4F, 0x4E},
	{0x50, 0x02},
	{0x51, 0xC2},
	{0x52, 0x01},
	{0x53, 0x2D},
	{0x54, 0x01},
	{0x55, 0x2B},
	{0x56, 0x00},
	{0x57, 0x2B},
	{0x58, 0x23},
	{0x59, 0x02},
	{0x5A, 0x25},
	{0x5B, 0x02},
	{0x5C, 0x25},
	{0x5D, 0x02},
	{0x5E, 0x1D},
	{0x5F, 0x00},
	{0x60, 0x23},
	{0x61, 0x02},
	{0x62, 0x1D},
	{0x63, 0x00},
	{0x64, 0x1A},
	{0x65, 0x03},
	{0x66, 0x0A},
	{0x67, 0xF0},
	{0x68, 0x00},
	{0x69, 0xF0},
	{0x6A, 0x00},
	{0x6B, 0x00},
	{0x6C, 0x00},
	{0x6D, 0xF0},
	{0x6E, 0x00},
	{0x6F, 0x60},
	{0x70, 0x00},
	{0x71, 0x00},
	{0x72, 0x00},
	{0x73, 0x00},
	{0x74, 0x00},
	{0x75, 0x00},
	{0x76, 0x00},
	{0x77, 0x00},
	{0x78, 0x00},
	{0x79, 0x68},
	{0x7A, 0x00},
	{0x7B, 0x00},
	{0x7C, 0x00},
	{0x7D, 0x00},
	{0x7E, 0x00},
	{0x7F, 0x00}
#elif PANEL_640x400_60
	{0x00, 0x0F},
	{0x01, 0x00},
	{0x02, 0x00},
	{0x03, 0x00},
	{0x04, 0x3F},
	{0x05, 0xC8},
	{0x06, 0x00},
	{0x07, 0x40},
	{0x08, 0x00},
	{0x09, 0x00},
	{0x0A, 0x10},
	{0x0B, 0x00},
	{0x0C, 0x00},
	{0x0D, 0x00},
	{0x0E, 0x00},
	{0x0F, 0x56},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x16, 0x00},
	{0x17, 0x00},
	{0x18, 0x00},
	{0x19, 0x00},
	{0x1A, 0x00},
	{0x1B, 0x00},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x1E, 0x00},
	{0x1F, 0x00},
	{0x20, 0x01},
	{0x21, 0x00},
	{0x22, 0x40},
	{0x23, 0x40},
	{0x24, 0x40},
	{0x25, 0x80},
	{0x26, 0x40},
	{0x27, 0x40},
	{0x28, 0x40},
	{0x29, 0x0B},
	{0x2A, 0xBE},
	{0x2B, 0x3C},
	{0x2C, 0x02},
	{0x2D, 0x7A},
	{0x2E, 0x02},
	{0x2F, 0xFA},
	{0x30, 0x26},
	{0x31, 0x01},
	{0x32, 0xB6},
	{0x33, 0x00},
	{0x34, 0x03},
	{0x35, 0x5A},
	{0x36, 0x00},
	{0x37, 0x76},
	{0x38, 0x02},
	{0x39, 0xFE},
	{0x3A, 0x02},
	{0x3B, 0x0D},
	{0x3C, 0x00},
	{0x3D, 0x1B},
	{0x3E, 0x00},
	{0x3F, 0x1C},
	{0x40, 0x01},
	{0x41, 0xF3},
	{0x42, 0x01},
	{0x43, 0xF4},
	{0x44, 0x80},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x41},
	{0x48, 0x08},
	{0x49, 0x02},
	{0x4A, 0xFC},
	{0x4B, 0x08},
	{0x4C, 0x16},
	{0x4D, 0x08},
	{0x4E, 0x00},
	{0x4F, 0x4E},
	{0x50, 0x02},
	{0x51, 0xC2},
	{0x52, 0x01},
	{0x53, 0x2D},
	{0x54, 0x01},
	{0x55, 0x2B},
	{0x56, 0x00},
	{0x57, 0x2B},
	{0x58, 0x23},
	{0x59, 0x02},
	{0x5A, 0x25},
	{0x5B, 0x02},
	{0x5C, 0x25},
	{0x5D, 0x02},
	{0x5E, 0x1D},
	{0x5F, 0x00},
	{0x60, 0x23},
	{0x61, 0x02},
	{0x62, 0x1D},
	{0x63, 0x00},
	{0x64, 0x1A},
	{0x65, 0x03},
	{0x66, 0x0A},
	{0x67, 0xF0},
	{0x68, 0x00},
	{0x69, 0xF0},
	{0x6A, 0x00},
	{0x6B, 0x00},
	{0x6C, 0x00},
	{0x6D, 0xF0},
	{0x6E, 0x00},
	{0x6F, 0x60},
	{0x70, 0x00},
	{0x71, 0x00},
	{0x72, 0x00},
	{0x73, 0x00},
	{0x74, 0x00},
	{0x75, 0x00},
	{0x76, 0x00},
	{0x77, 0x00},
	{0x78, 0x00},
	{0x79, 0x68},
	{0x7A, 0x00},
	{0x7B, 0x00},
	{0x7C, 0x00},
	{0x7D, 0x00},
	{0x7E, 0x00},
	{0x7F, 0x00}
#elif PANEL_640x360_50
	{0x00, 0x0F},
	{0x01, 0x0C},
	{0x02, 0x00},
	{0x03, 0x00},
	{0x04, 0x3F},
	{0x05, 0xC8},
	{0x06, 0x00},
	{0x07, 0x40},
	{0x08, 0x00},
	{0x09, 0x00},
	{0x0A, 0x10},
	{0x0B, 0x00},
	{0x0C, 0x00},
	{0x0D, 0x00},
	{0x0E, 0x00},
	{0x0F, 0x56},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x16, 0x00},
	{0x17, 0x00},
	{0x18, 0x00},
	{0x19, 0x00},
	{0x1A, 0x00},
	{0x1B, 0x00},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x1E, 0x00},
	{0x1F, 0x00},
	{0x20, 0x01},
	{0x21, 0x00},
	{0x22, 0x40},
	{0x23, 0x40},
	{0x24, 0x40},
	{0x25, 0x80},
	{0x26, 0x40},
	{0x27, 0x40},
	{0x28, 0x40},
	{0x29, 0x0B},
	{0x2A, 0xBE},
	{0x2B, 0x3C},
	{0x2C, 0x02},
	{0x2D, 0x7A},
	{0x2E, 0x02},
	{0x2F, 0xFA},
	{0x30, 0x26},
	{0x31, 0x01},
	{0x32, 0x8E},
	{0x33, 0x00},
	{0x34, 0x03},
	{0x35, 0x60},
	{0x36, 0x00},
	{0x37, 0x76},
	{0x38, 0x02},
	{0x39, 0xFE},
	{0x3A, 0x02},
	{0x3B, 0x71},
	{0x3C, 0x00},
	{0x3D, 0x1B},
	{0x3E, 0x00},
	{0x3F, 0x1C},
	{0x40, 0x02},
	{0x41, 0x4D},
	{0x42, 0x02},
	{0x43, 0x4E},
	{0x44, 0x80},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x41},
	{0x48, 0x08},
	{0x49, 0x02},
	{0x4A, 0xFC},
	{0x4B, 0x08},
	{0x4C, 0x16},
	{0x4D, 0x08},
	{0x4E, 0x00},
	{0x4F, 0x4E},
	{0x50, 0x02},
	{0x51, 0xC2},
	{0x52, 0x01},
	{0x53, 0x2D},
	{0x54, 0x01},
	{0x55, 0x2B},
	{0x56, 0x00},
	{0x57, 0x2B},
	{0x58, 0x23},
	{0x59, 0x02},
	{0x5A, 0x25},
	{0x5B, 0x02},
	{0x5C, 0x25},
	{0x5D, 0x02},
	{0x5E, 0x1D},
	{0x5F, 0x00},
	{0x60, 0x23},
	{0x61, 0x02},
	{0x62, 0x1D},
	{0x63, 0x00},
	{0x64, 0x1A},
	{0x65, 0x03},
	{0x66, 0x0A},
	{0x67, 0xF0},
	{0x68, 0x00},
	{0x69, 0xF0},
	{0x6A, 0x00},
	{0x6B, 0x00},
	{0x6C, 0x00},
	{0x6D, 0xF0},
	{0x6E, 0x00},
	{0x6F, 0x60},
	{0x70, 0x00},
	{0x71, 0x00},
	{0x72, 0x00},
	{0x73, 0x00},
	{0x74, 0x00},
	{0x75, 0x00},
	{0x76, 0x00},
	{0x77, 0x00},
	{0x78, 0x00},
	{0x79, 0x68},
	{0x7A, 0x00},
	{0x7B, 0x00},
	{0x7C, 0x00},
	{0x7D, 0x00},
	{0x7E, 0x00},
	{0x7F, 0x00}
#elif PANEL_640x360_60_14
	{0x00, 0x0F},
	{0x01, 0x0C},
	{0x02, 0x00},
	{0x03, 0x00},
	{0x04, 0x3F},
	{0x05, 0xC8},
	{0x06, 0x00},
	{0x07, 0x40},
	{0x08, 0x00},
	{0x09, 0x00},
	{0x0A, 0x10},
	{0x0B, 0x00},
	{0x0C, 0x00},
	{0x0D, 0x00},
	{0x0E, 0x00},
	{0x0F, 0x56},
	{0x10, 0x00},
	{0x11, 0x00},
	{0x12, 0x00},
	{0x13, 0x00},
	{0x14, 0x00},
	{0x15, 0x00},
	{0x16, 0x00},
	{0x17, 0x00},
	{0x18, 0x00},
	{0x19, 0x00},
	{0x1A, 0x00},
	{0x1B, 0x00},
	{0x1C, 0x00},
	{0x1D, 0x00},
	{0x1E, 0x00},
	{0x1F, 0x00},
	{0x20, 0x01},
	{0x21, 0x00},
	{0x22, 0x40},
	{0x23, 0x40},
	{0x24, 0x40},
	{0x25, 0x80},
	{0x26, 0x40},
	{0x27, 0x40},
	{0x28, 0x40},
	{0x29, 0x0B},
	{0x2A, 0xBE},
	{0x2B, 0x3C},
	{0x2C, 0x02},
	{0x2D, 0x7C},
	{0x2E, 0x02},
	{0x2F, 0xFC},
	{0x30, 0x26},
	{0x31, 0x01},
	{0x32, 0x8E},
	{0x33, 0x00},
	{0x34, 0x03},
	{0x35, 0x5A},
	{0x36, 0x00},
	{0x37, 0x78},
	{0x38, 0x3 },
	{0x39, 0x0 },
	{0x3A, 0x02},
	{0x3B, 0x0D},
	{0x3C, 0x00},
	{0x3D, 0x1B},
	{0x3E, 0x00},
	{0x3F, 0x1C},
	{0x40, 0x01},
	{0x41, 0xF3},
	{0x42, 0x01},
	{0x43, 0xF4},
	{0x44, 0x80},
	{0x45, 0x00},
	{0x46, 0x00},
	{0x47, 0x41},
	{0x48, 0x08},
	{0x49, 0x02},
	{0x4A, 0xFC},
	{0x4B, 0x08},
	{0x4C, 0x16},
	{0x4D, 0x08},
	{0x4E, 0x00},
	{0x4F, 0x4E},
	{0x50, 0x02},
	{0x51, 0xC2},
	{0x52, 0x01},
	{0x53, 0x2D},
	{0x54, 0x01},
	{0x55, 0x2B},
	{0x56, 0x00},
	{0x57, 0x2B},
	{0x58, 0x23},
	{0x59, 0x02},
	{0x5A, 0x25},
	{0x5B, 0x02},
	{0x5C, 0x25},
	{0x5D, 0x02},
	{0x5E, 0x1D},
	{0x5F, 0x00},
	{0x60, 0x23},
	{0x61, 0x02},
	{0x62, 0x1D},
	{0x63, 0x00},
	{0x64, 0x1A},
	{0x65, 0x03},
	{0x66, 0x0A},
	{0x67, 0xF0},
	{0x68, 0x00},
	{0x69, 0xF0},
	{0x6A, 0x00},
	{0x6B, 0x00},
	{0x6C, 0x00},
	{0x6D, 0xF0},
	{0x6E, 0x00},
	{0x6F, 0x60},
	{0x70, 0x00},
	{0x71, 0x00},
	{0x72, 0x00},
	{0x73, 0x00},
	{0x74, 0x00},
	{0x75, 0x00},
	{0x76, 0x00},
	{0x77, 0x00},
	{0x78, 0x00},
	{0x79, 0x68},
	{0x7A, 0x00},
	{0x7B, 0x00},
	{0x7C, 0x00},
	{0x7D, 0x00},
	{0x7E, 0x00},
	{0x7F, 0x00}
#endif
};

#if 0
static unsigned char bit_shift[8] = { (1 << 7),	/* MSB */
	(1 << 6),
	(1 << 5),
	(1 << 4),
	(1 << 3),
	(1 << 2),
	(1 << 1),
	(1 << 0)		/* LSB */
};
#else
static unsigned char bit_shift[8] = { (1 << 0),	/* LSB */
	(1 << 1),
	(1 << 2),
	(1 << 3),
	(1 << 4),
	(1 << 5),
	(1 << 6),
	(1 << 7)		/* MSB */
};
#endif

unsigned int SPI_CS = 0;
char hud_gpio_init = 0;

unsigned int panel_spi_read_byte(void)
{
	int i;
	unsigned int val = 0;
	/* Clock should be Low before entering */
	for (i = 0; i < 8; i++) {
		gpio_set_value(SPI_SCLK, 0);
		/* #1: Drive the Data (High or Low) */
		if(gpio_get_value(SPI_MISO))
			val |= bit_shift[i];

		/* #2: Drive the Clk High and then Low */
		gpio_set_value(SPI_SCLK, 1);
//		udelay(1);
//		udelay(1);
	}
	return val;
}

static void panel_spi_write_byte(unsigned int val)
{
	int i;

	/* Clock should be Low before entering */
	for (i = 0; i < 8; i++) {
		gpio_set_value(SPI_SCLK, 0);
		/* #1: Drive the Data (High or Low) */
		if (val & bit_shift[i])
			gpio_set_value(SPI_MOSI, 1);
		else
			gpio_set_value(SPI_MOSI, 0);

		/* #2: Drive the Clk High and then Low */
		gpio_set_value(SPI_SCLK, 1);
//		udelay(1);
//		udelay(1);
	}
}

static int panel_spi_out(unsigned int reg, unsigned int data)
{
	/* Enable the Chip Select - low */
	gpio_set_value(SPI_CS, 0);
	udelay(1);

	/* Transmit register address first, then data */
	panel_spi_write_byte(reg);

	/* Idle state of MOSI is Low */
//	gpio_set_value(SPI_MOSI, 0);
//	udelay(1);
	panel_spi_write_byte(data);

	udelay(1);
	gpio_set_value(SPI_MOSI, 0);
	gpio_set_value(SPI_CS, 1);
	udelay(10);
	return 0;
}

unsigned int panel_spi_in(unsigned int reg)
{
	unsigned int data = 0;

	/* Set read mode */
	panel_spi_out(0x80, 1);

	/* Set read address */
	panel_spi_out(0x81, reg);

	/* Read from 0x81 */
	gpio_set_value(SPI_CS, 0);
	udelay(1);
	panel_spi_write_byte(0x81);
	data = panel_spi_read_byte();

	udelay(1);
	/* Idle state of MOSI is Low */
	gpio_set_value(SPI_MOSI, 0);
	gpio_set_value(SPI_CS, 1);
//	mdelay(1);
	return data;
}

void panel_set_brightness(int value)
{
	unsigned int val;

	if(value > MAX_LUM) val = MAX_LUM;
	else val = value;

	SPI_CS = SPI_CS_N1;
	panel_spi_out(0x80, 0);
	mdelay(1);	
	panel_spi_out(REG_BRTLUM_DIS, 4);
	panel_spi_out(REG_LUM, val);
//	panel_spi_out(REG_BRT, val);
	mdelay(1);
//	printk("HUD1 rd LUM:%d\n",panel_spi_in(REG_LUM));
//	printk("HUD1 rd LUM:%d\n",panel_spi_in(REG_BRT));
	SPI_CS = SPI_CS_N2;
	panel_spi_out(0x80, 0);
	mdelay(1);	
	panel_spi_out(REG_BRTLUM_DIS, 4);
	panel_spi_out(REG_LUM, val);
//	panel_spi_out(REG_BRT, val);
	mdelay(1);	
//	printk("HUD2 rd LUM:%d\n",panel_spi_in(REG_LUM));
//	printk("HUD2 rd LUM:%d\n",panel_spi_in(REG_BRT));
}

void panel_spi_gpio_init(void)
{
	int ret;
	gpio_direction_output(SPI_CS_N1, 1);
	gpio_direction_output(SPI_CS_N2, 1);
	gpio_direction_output(SPI_SCLK, 1);
	gpio_direction_output(SPI_MOSI, 0);
//	gpio_direction_output(SPI_MISO, 0);
	ret = gpio_direction_input(SPI_MISO);
	panel_set_brightness(200);
	hud_gpio_init = 1;
	mdelay(5);
}

void panel_hud_disp_on(void)
{
	unsigned int pop_data = 0;
	unsigned int i;
//	int write_again = 0;
	
	if(!hud_gpio_init) panel_spi_gpio_init();
// write to panel 1
//WRITE_P1:
	SPI_CS = SPI_CS_N1;
	for (i = 0; i < ARRAY_SIZE(init_sequence); i++) {
		if(i==0) init_sequence[i].data = PANEL1_OFF;
		panel_spi_out(init_sequence[i].addr, init_sequence[i].data);
	}
	mdelay(10);
	for (i = 0; i < 128; i++) {
		pop_data = panel_spi_in(i);
		if (pop_data != init_sequence[i].data){
//			write_again = 1;
//			printk("eztest panel spi N1****************CS:%d addr:%x init:[%x] pop:[%x]\n",SPI_CS,i,init_sequence[i].data,pop_data);
		}
	}
	/* Set read mode off*/
	panel_spi_out(0x80, 0);
	mdelay(1);
#if 0
	if(write_again){
		write_again =0;
		msleep(10);
		panel_spi_gpio_init();
		goto WRITE_P1;
	}
#endif
// write to panel 2
//WRITE_P2:
	SPI_CS = SPI_CS_N2;
	for (i = 0; i < ARRAY_SIZE(init_sequence); i++) {
		if(i==0) init_sequence[i].data = PANEL2_OFF;
		panel_spi_out(init_sequence[i].addr, init_sequence[i].data);
	}
	mdelay(10);
	for (i = 0; i < 128; i++) {
		pop_data = panel_spi_in(i);
		if (pop_data != init_sequence[i].data){
//			write_again = 1;
//			printk("eztest panel spi N2****************CS:%d addr:%x init:[%x] pop:[%x]\n",SPI_CS,i,init_sequence[i].data,pop_data);
		}
	}
	/* Set read mode off*/
	panel_spi_out(0x80, 0);
	mdelay(1);
#if 0
	if(write_again){
		write_again =0;
		msleep(10);
		panel_spi_gpio_init();
		goto WRITE_P2;
	}
#endif
	SPI_CS = SPI_CS_N1;
	panel_spi_out(0,PANEL1_ON);// mclkpol 1:0x07 0:0x05
	mdelay(1);
#if 0
	for (i = 0; i < 128; i++) {
		pop_data = panel_spi_in(i);
		if (pop_data != init_sequence[i].data) printk("eztest panel spi *********CS:%d addr:%x init:[%x] pop:[%x]\n",SPI_CS,i,init_sequence[i].data,pop_data);
		else printk("eztest panel spi -------->CS:%d addr:%x init:[%x] pop:[%x]\n",SPI_CS,i,init_sequence[i].data,pop_data);
	}
#else
	pop_data = panel_spi_in(0);
	printk("eztest panel spi -------->pop panel:%d addr:0x00:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(1);
	printk("eztest panel spi -------->pop panel:%d addr:0x01:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(2);
	printk("eztest panel spi -------->pop panel:%d addr:0x02:[%x]\n",SPI_CS,pop_data);
#endif
	/* Set read mode off*/
	panel_spi_out(0x80, 0);
	mdelay(1);

	SPI_CS = SPI_CS_N2;
	panel_spi_out(0,PANEL2_ON);// mclkpol 1:0x0b 0:0x09
	mdelay(1);
#if 0
	for (i = 0; i < 128; i++) {
		pop_data = panel_spi_in(i);
		if (pop_data != init_sequence[i].data) printk("eztest panel spi *********CS:%d addr:%x init:[%x] pop:[%x]\n",SPI_CS,i,init_sequence[i].data,pop_data);
		else printk("eztest panel spi -------->CS:%d addr:%x init:[%x] pop:[%x]\n",SPI_CS,i,init_sequence[i].data,pop_data);
	}
#else
	pop_data = panel_spi_in(0);
	printk("eztest panel spi -------->pop panel:%d addr:0x00:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(1);
	printk("eztest panel spi -------->pop panel:%d addr:0x01:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(2);
	printk("eztest panel spi -------->pop panel:%d addr:0x02:[%x]\n",SPI_CS,pop_data);
#endif
	/* Set read mode off*/
	panel_spi_out(0x80, 0);
	mdelay(1);
}

void panel_hud_disp_off(void)
{
	unsigned int pop_data = 0;

	SPI_CS = SPI_CS_N1;
	panel_spi_out(0,PANEL1_OFF);// mclkpol 1:0x06 0:0x04
	mdelay(1);
	pop_data = panel_spi_in(0);
	printk("eztest panel spi -------->pop panel:%d addr:0x00:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(1);
	printk("eztest panel spi -------->pop panel:%d addr:0x01:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(2);
	printk("eztest panel spi -------->pop panel:%d addr:0x02:[%x]\n",SPI_CS,pop_data);
	/* Set read mode off*/
	panel_spi_out(0x80, 0);

	SPI_CS = SPI_CS_N2;
	panel_spi_out(0,PANEL2_OFF);// mclkpol 1:0x0a 0:0x08
	mdelay(1);
	pop_data = panel_spi_in(0);
	printk("eztest panel spi -------->pop panel:%d addr:0x00:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(1);
	printk("eztest panel spi -------->pop panel:%d addr:0x01:[%x]\n",SPI_CS,pop_data);
	pop_data = panel_spi_in(2);
	printk("eztest panel spi -------->pop panel:%d addr:0x02:[%x]\n",SPI_CS,pop_data);
	/* Set read mode off*/
	panel_spi_out(0x80, 0);
	mdelay(1);
}
