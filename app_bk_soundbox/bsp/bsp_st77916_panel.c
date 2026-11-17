// Copyright 2026
// ST77916 360x360 圆形 QSPI 屏面板定义
//
// 面板：BOE 1.8" 圆屏，ST77916 控制器，QSPI 4 线，360x360，RGB565。
//
// 初始化命令表在 bsp_st77916_boe_init.h（192 项，厂商例程 1:1 转换）。
// 本文件只负责把这些参数组装成 SDK 的 lcd_device_t。
//
// 【时钟】当前固定 8MHz。
//
// 这块屏的排线没有阻抗控制，16MHz 在 app_test_uart1 上能跑，但在
// app_bk_pi 上出现花屏，怀疑是同一块屏在不同板况（走线/负载/供电）
// 下的余量差异。先把速度压到 8MHz 建立"确定能显示"的基线：
//   4 线 @8MHz -> 约 4MB/s，整帧 259200B ≈ 65ms（≈15fps）
// UI 会明显变慢但能用。确认 8M 不花屏后再逐档往上试：
//   LCD_QSPI_8M -> LCD_QSPI_12M -> LCD_QSPI_16M
// 每换一档只改这一个枚举，别的都不用动。
//
// 如果 8M 仍然花屏，那就**不是通信速度问题**，别再往下压 ——
// 转而看 bsp_screen.c 里那两条自检日志：
//   selftest: COLMOD=0x??  <- 不是 0x55 就是初始化命令没进屏
//   selftest: red/green/blue <- 三色是否按顺序显示、颜色对不对
//
// QSPI 时序参数（与 app_test_uart1 的 s_boe_qspi 一致）：
//   reg_write_cmd = 0x02、reg_read_cmd = 0x0B、无 dummy clk、
//   像素写命令字 0x32/0x00/0x2C/0x00、REFRESH_BY_FRAME。

#include "bsp_screen.h"
#include "bsp_board_cfg.h"

#include <driver/lcd_types.h>
#include <driver/lcd_qspi_types.h>

#include "bsp_st77916_boe_init.h"

/* QSPI 4 线写像素的命令字（末字节 0x2C = RAMWR）。
 * 前两个 0x32/0x00 是 ST77916 的 QSPI 像素写"引导字"，不能省。 */
static const uint8_t s_pixel_cmd[4] = { 0x32, 0x00, 0x2C, 0x00 };

const bsp_screen_panel_t bsp_panel_st77916_360x360 = {
	.name = "st77916_boe_360x360",
	.width = BSP_LCD_H_RES,
	.height = BSP_LCD_V_RES,
	.qspi_id = BOARD_LCD_QSPI_ID,
	.clk = LCD_QSPI_8M,
	.refresh_method = LCD_QSPI_REFRESH_BY_FRAME,
	.reg_write_cmd = 0x02,
	.reg_read_cmd = 0x0B,
	.pixel_cmd = s_pixel_cmd,
	.pixel_cmd_len = sizeof(s_pixel_cmd),
	.init_cmd = s_boe_init_cmds,
	.init_cmd_len = sizeof(s_boe_init_cmds) / sizeof(s_boe_init_cmds[0]),
	.reset_pin = GPIO_NUM,   /* RST 接 CSN，跳过硬复位 */
};
