// Copyright 2026
// 板级电源控制（各外设的电源使能脚）

#pragma once

#include <common/bk_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	BSP_PWR_RAIL_SCR = 0,   /* 屏幕 3V3，PMOS Q5，低有效 */
	BSP_PWR_RAIL_CAM_2V8,   /* 摄像头 2V8，LDO U10，高有效 */
	BSP_PWR_RAIL_CAM_1V5,   /* 摄像头 1V5，LDO U9，高有效 */
	BSP_PWR_RAIL_MIC_1V8,   /* 麦克风 1V8，PMOS（原 LDO U8），低有效 */
	BSP_PWR_RAIL_PA,        /* 功放 NS4150B CTRL，高有效 */
	BSP_PWR_RAIL_MAX,
} bsp_pwr_rail_t;

/* PM 框架投票用的模块 ID（应用区 10~31，见 gpio_ctrl_ldo_module_e）。
 * bsp_power_enable() 会把每路电源的最终电平登记给 PM，
 * 低功耗睡眠时由 PM 维持，唤醒后自动恢复。 */
#define BSP_PM_MODULE_MIC    (10 + 0)   /* GPIO_CTRL_LDO_MODULE_APP_BASE + 0 */
#define BSP_PM_MODULE_BATADC (10 + 1)   /* 电池分压通路 */

/**
 * @brief 初始化所有电源使能脚为默认关闭状态（输出、无效电平）。
 */
bk_err_t bsp_power_init(void);

/**
 * @brief 打开/关闭某一路电源。
 */
bk_err_t bsp_power_enable(bsp_pwr_rail_t rail, bool on);

/**
 * @brief 查询某一路电源当前是否打开（软件状态）。
 */
bool bsp_power_is_enabled(bsp_pwr_rail_t rail);

/**
 * @brief 读该路使能脚 pad 的实际电平：1/0，参数非法返回 -1。
 *
 * 用于区分"软件写了但引脚没动"（复用/被别的模块占用）和"引脚动了但外设没反应"（硬件）。
 */
int bsp_power_get_pad(bsp_pwr_rail_t rail);

/**
 * @brief 摄像头上电（1V5 -> 2V8，带稳定延时）。
 */
bk_err_t bsp_power_camera_on(void);

/**
 * @brief 摄像头下电（2V8 -> 1V5）。
 */
bk_err_t bsp_power_camera_off(void);

/**
 * @brief 屏幕上电（SCR_3V3，带稳定延时）。屏幕无独立复位脚，靠断电复位。
 */
bk_err_t bsp_power_screen_on(void);

/**
 * @brief 屏幕下电。
 */
bk_err_t bsp_power_screen_off(void);

/**
 * @brief 屏幕断电再上电，用于硬复位（≥ BOARD_SCR_PWR_SETTLE_MS）。
 */
bk_err_t bsp_power_screen_reset(void);

#ifdef __cplusplus
}
#endif
