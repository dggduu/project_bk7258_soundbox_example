// Copyright 2026
// ILI9341 240x320 单线 SPI 面板定义
//
// 初始化命令表直接搬自 projects/app_bk_pi/ref/il9341/lcd.c 的 LCD_Init()：
// 那是 QDtech 2.8 寸 ILI9341 模块（240x320，SPI 接口）在 STM32 上验证过的
// 表，除了把 delay_ms() 换成 SDK 的 {0x00, {ms}, 0xFF} 延时项外没有改动。
//
// SDK 的 lcd_spi 驱动会在 init 命令发完后再自己发一遍 0x2A/0x2B（建立整屏
// 显存窗口）和 0x2C，所以这里保留的 0x2A/0x2B 只是照抄参考时序，不影响显示。

#include "bsp_screen.h"
#include "bsp_board_cfg.h"

static const lcd_qspi_init_cmd_t s_ili9341_init_cmds[] = {
	/* Power control */
	{ 0xCF, { 0x00, 0xC9, 0x30 }, 3 },
	{ 0xED, { 0x64, 0x03, 0x12, 0x81 }, 4 },
	{ 0xE8, { 0x85, 0x10, 0x7A }, 3 },
	{ 0xCB, { 0x39, 0x2C, 0x00, 0x34, 0x02 }, 5 },
	{ 0xF7, { 0x20 }, 1 },
	{ 0xEA, { 0x00, 0x00 }, 2 },

	{ 0xC0, { 0x1B }, 1 },              /* VRH[5:0]      */
	{ 0xC1, { 0x00 }, 1 },              /* SAP[2:0];BT[3:0] */
	{ 0xC5, { 0x30, 0x30 }, 2 },        /* VCM control   */
	{ 0xC7, { 0xB7 }, 1 },              /* VCM control2  */

	{ 0x36, { BSP_LCD_MADCTL }, 1 },    /* Memory Access Control */
	{ 0x3A, { 0x55 }, 1 },              /* 16 bit/pixel  */

	{ 0xB1, { 0x00, 0x1A }, 2 },        /* Frame rate control */
	{ 0xB6, { 0x0A, 0xA2 }, 2 },        /* Display Function Control */
	{ 0xF2, { 0x00 }, 1 },              /* 3Gamma Function Disable */
	{ 0x26, { 0x01 }, 1 },              /* Gamma curve selected */

	/* Set Gamma */
	{ 0xE0, { 0x0F, 0x2A, 0x28, 0x08, 0x0E, 0x08, 0x54, 0xA9,
	          0x43, 0x0A, 0x0F, 0x00, 0x00, 0x00, 0x00 }, 15 },
	{ 0xE1, { 0x00, 0x15, 0x17, 0x07, 0x11, 0x06, 0x2B, 0x56,
	          0x3C, 0x05, 0x10, 0x0F, 0x3F, 0x3F, 0x0F }, 15 },

	/* 显存窗口（整屏），SDK 驱动随后还会再设一遍 */
	{ 0x2B, { 0x00, 0x00, 0x01, 0x3F }, 4 },
	{ 0x2A, { 0x00, 0x00, 0x00, 0xEF }, 4 },

	{ 0x11, { 0x00 }, 0 },               /* Exit Sleep     */
	{ 0x00, { 120 }, 0xFF },             /* delay 120ms    */
	{ 0x29, { 0x00 }, 0 },               /* Display on     */
};

const bsp_screen_panel_t bsp_panel_ili9341_240x320 = {
	.name = "ili9341",
	.width = BSP_LCD_H_RES,
	.height = BSP_LCD_V_RES,
	/* 2.8 寸模块排线不长，32M 是保守值；如果出现花屏/错位可以降到 30M，
	 * 相反确认模块能跑也可以试 LCD_QSPI_40M / LCD_QSPI_48M。 */
	.clk = LCD_QSPI_32M,
	.init_cmd = s_ili9341_init_cmds,
	.init_cmd_len = sizeof(s_ili9341_init_cmds) / sizeof(s_ili9341_init_cmds[0]),
};
