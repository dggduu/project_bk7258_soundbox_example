// Copyright 2026
// 电池电压检测（BAT_ADC = P22/ADC5，R21 220k / R24 110k = 1:3 分压，直采无使能脚）
//
// 电压换算走 SDK 的芯片出厂标定（和按键 ADC 同一口径），raw 只用于判饱和
// 和打诊断日志。为什么不再用 raw * 1559 / 8191，见 bsp_battery.c 文件头。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电池检测（仅打日志，采样时按需切 P22 到 ADC 功能）。
 */
bk_err_t bsp_battery_init(void);

/**
 * @brief 读取电池电压（mV），单次采样。
 */
bk_err_t bsp_battery_get_mv(uint32_t *mv);

/**
 * @brief 读取电池电压（mV），多次采样取平均。
 *
 * 分压常接，直接读 ADC5、按 SDK 标定换算成 mV 再乘 BOARD_BAT_ADC_DIV_NUM。
 * 分压点低于 800mV（VBAT 约 2.4V）视为无电池/分压开路，返回
 * BK_ERR_NOT_FOUND；raw 饱和或分压点 >2200mV 返回 BK_ERR_PARAM
 * （分压比不对 / 过压）。
 *
 * @param sample_num 采样次数（取算术平均），>=1；建议 8 次
 * @param mv         输出 VBAT，单位 mV
 */
bk_err_t bsp_battery_get_mv_avg(uint32_t sample_num, uint32_t *mv);

/**
 * @brief 由电压估算电量百分比（0~100，单节锂电 3.0V~4.2V 线性映射）。
 */
uint8_t bsp_battery_mv_to_percent(uint32_t mv);

/**
 * @brief 读电压并换算电量。
 */
bk_err_t bsp_battery_get_percent(uint8_t *percent);

/**
 * @brief 低功耗钩子（分压常接，软件无法断开，保留空实现）。
 */
bk_err_t bsp_battery_power_down(void);

#ifdef __cplusplus
}
#endif
