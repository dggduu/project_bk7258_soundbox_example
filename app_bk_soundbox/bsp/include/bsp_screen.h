// Copyright 2026
// 屏幕 BSP（ST77916 360x360 圆形 QSPI 屏，背光 P23，屏电源 P51）
//
// 硬件事实（来自 app_bk_pi 原理图网表 + 本次换屏接线）：
//   - 排针 LCD 组就是 QSPI1 的引脚，4 线全用：
//     P2=CLK  P3=CSN  P4=IO0  P5=IO1  P6=IO2  P7=IO3
//   - RST 不再单独引出（接在 CSN 上），所以 reset_pin 传 GPIO_NUM 跳过硬复位
//   - 背光 LCD_BLC 直连 P23，无 MOS 管，高电平点亮，没接 PWM
//   - 屏电源 SCR_3V3 由 P51 经 PMOS Q5 控制，低电平导通
//
// 驱动路线：SDK 的 lcd_qspi 驱动（bk_lcd_qspi_init / frame_display /
// wait_display_complete）。**不是** lcd_spi —— 换屏前那套单线 SPI 已经不用了。
//
// 【刷帧的时序约束】必须 frame_display() 紧随 wait_display_complete()：
// frame_display() 只是把 DMA 踢起来，wait 才真正等这一帧进 GRAM。
// 反过来（先 wait 再 display）会 5 秒超时返回 BK_FAIL，且帧从未提交。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>
#include <driver/lcd_types.h>
#include <driver/lcd_qspi_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * 屏幕面板描述。换屏时填一个结构体注册即可，不用改 SDK。
 *
 * 字段全部对应 SDK 的 lcd_qspi_t，含义见 driver/lcd_qspi_types.h。
 */
typedef struct {
	const char *name;
	uint16_t width;
	uint16_t height;

	/* --- QSPI 总线参数 --- */
	uint8_t qspi_id;                        /* QSPI_ID_0 / QSPI_ID_1 */
	lcd_qspi_clk_t clk;                     /* 本板只能 16MHz */
	lcd_qspi_refresh_method_t refresh_method;
	uint8_t reg_write_cmd;                  /* 写寄存器命令，ST77916 = 0x02 */
	uint8_t reg_read_cmd;                   /* 读寄存器命令，ST77916 = 0x0B */
	const uint8_t *pixel_cmd;               /* 4 线写像素引导字，4 字节 */
	uint8_t pixel_cmd_len;

	/* --- 面板初始化 --- */
	const lcd_qspi_init_cmd_t *init_cmd;
	uint32_t init_cmd_len;

	/* --- 复位脚：GPIO_NUM 表示不单独复位（RST 接 CSN） --- */
	uint8_t reset_pin;
} bsp_screen_panel_t;

/* 本板面板：ST77916 360x360 圆屏，QSPI 4 线，RGB565 */
extern const bsp_screen_panel_t bsp_panel_st77916_360x360;

/**
 * @brief 用默认面板初始化屏幕。
 *
 * 顺序：关背光 -> 屏 3V3 上电 -> bk_lcd_qspi_init() 配 QSPI 并回放初始化表
 * -> 开背光。
 */
bk_err_t bsp_screen_init(void);

/**
 * @brief 用指定面板初始化屏幕（会先关掉旧的初始化）。
 */
bk_err_t bsp_screen_init_with(const bsp_screen_panel_t *panel);

/**
 * @brief 关闭屏幕（关背光 + 关屏电源）。
 */
bk_err_t bsp_screen_deinit(void);

/**
 * @brief 屏幕重新初始化（重发初始化命令）。
 */
bk_err_t bsp_screen_reset(void);

/**
 * @brief 背光开关（GPIO 直驱，无 PWM 调光）。
 */
bk_err_t bsp_screen_backlight_on(void);
bk_err_t bsp_screen_backlight_off(void);

/**
 * @brief 背光亮度 0~100（GPIO 模式下仅 0=灭、非 0=亮）。
 */
bk_err_t bsp_screen_backlight_set(uint8_t percent);

/**
 * @brief 背光当前是否点亮（软件状态）。
 */
bool bsp_screen_backlight_is_on(void);

/**
 * @brief 显示开/关（面板 0x29/0x28）。
 */
bk_err_t bsp_screen_display_on(void);
bk_err_t bsp_screen_display_off(void);

/**
 * @brief 向屏幕送一帧数据（RGB565），阻塞到 DMA 发送完成。
 *
 * @param data 像素数据，必须 4 字节对齐，长度是 4 的整数倍
 * @param len  字节数（= 宽 * 高 * 2 = 259200）
 */
bk_err_t bsp_screen_flush(const void *data, uint32_t len);

/**
 * @brief 向屏幕的指定矩形区域送一块 RGB565 数据（分块刷新用）。
 *
 * 内部走 partial display：先设窗口（CASET/RASET）再起 DMA，最后等完成。
 * 调用方要保证 data 地址 4 字节对齐且 w*h*2 是 4 的整数倍 ——
 * 这两个条件不满足时调用方应先拷到自己的对齐缓冲（见 bsp_lvgl.c
 * 的 disp_flush），本接口不代劳，避免每次都白拷一遍。
 *
 * @param x    区域左上角 x
 * @param y    区域左上角 y
 * @param w    宽（像素）
 * @param h    高（像素）
 * @param data 像素数据，行宽 = w，连续存放
 */
bk_err_t bsp_screen_flush_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               const void *data);

/**
 * @brief 清屏为指定颜色（RGB565）。
 */
bk_err_t bsp_screen_fill(uint16_t color);

/**
 * @brief 发一条面板命令（+可选参数），调试用。
 */
bk_err_t bsp_screen_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len);

/**
 * @brief 当前面板信息。
 */
const bsp_screen_panel_t *bsp_screen_get_panel(void);

/**
 * @brief 屏幕是否已初始化。
 */
bool bsp_screen_is_ready(void);

#ifdef __cplusplus
}
#endif
