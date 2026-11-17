// Copyright 2026
// CST816S 电容触摸（I2C1: P42/P43，INT = P53，地址 0x15）
//
// 参考 espressif esp_lcd_touch_cst816s 的寄存器用法：
//   0x01 GestureID / 0x02 TouchPoints / 0x03~0x06 X,Y
//   0xFE DisAutoSleep / 0xFA IrqCtl / 0xA7 ChipID

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t x;
	uint16_t y;
	uint8_t strength;   /* 按压力度，0 表示无 */
	uint8_t gesture;    /* CST816S 手势 ID，0 表示无 */
	bool pressed;
} bsp_touch_point_t;

/**
 * @brief 初始化触摸：探测 chip id、关闭自动休眠、配置中断上报。
 *
 * 触摸与屏共用 CEN/电源，没有独立复位脚，初始化前需保证屏已上电。
 */
bk_err_t bsp_touch_init(void);

/**
 * @brief 释放 I2C 总线引用。
 */
bk_err_t bsp_touch_deinit(void);

/**
 * @brief 读一次触摸点（坐标已按面板方向换算）。
 *
 * @param pt 输出，pressed=false 表示当前无触摸
 */
bk_err_t bsp_touch_read(bsp_touch_point_t *pt);

/**
 * @brief 读芯片 ID（CST816S 为 0xB5）。
 */
bk_err_t bsp_touch_chip_id(uint8_t *id);

/**
 * @brief 坐标方向配置（不同屏贴合方向不同，默认不交换不镜像）。
 */
void bsp_touch_set_transform(bool swap_xy, bool mirror_x, bool mirror_y);

/**
 * @brief 触摸是否已初始化。
 */
bool bsp_touch_is_ready(void);

#ifdef __cplusplus
}
#endif
