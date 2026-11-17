// Copyright 2026
// SC7A20 三轴加速度计（兼容 LIS2DH/LIS3DH 寄存器，I2C1: P42/P43）
//   INT1 = P55 (SC_INT1)，INT2 = P54 (SC_INT2)

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	BSP_SC7A20_RANGE_2G = 0,
	BSP_SC7A20_RANGE_4G,
	BSP_SC7A20_RANGE_8G,
	BSP_SC7A20_RANGE_16G,
} bsp_sc7a20_range_t;

typedef enum {
	BSP_SC7A20_ODR_POWER_DOWN = 0,
	BSP_SC7A20_ODR_1HZ,
	BSP_SC7A20_ODR_10HZ,
	BSP_SC7A20_ODR_25HZ,
	BSP_SC7A20_ODR_50HZ,
	BSP_SC7A20_ODR_100HZ,
	BSP_SC7A20_ODR_200HZ,
	BSP_SC7A20_ODR_400HZ,
	BSP_SC7A20_ODR_1620HZ,
} bsp_sc7a20_odr_t;

typedef struct {
	int16_t x;   /* mg */
	int16_t y;
	int16_t z;
} bsp_sc7a20_accel_t;

/**
 * @brief 初始化（探测 0x18/0x19，默认 ±2g / 100Hz / 12bit）。
 */
bk_err_t bsp_sc7a20_init(void);

/**
 * @brief 反初始化（进入 power-down）。
 */
bk_err_t bsp_sc7a20_deinit(void);

/**
 * @brief 读取加速度，单位 mg。
 */
bk_err_t bsp_sc7a20_read(bsp_sc7a20_accel_t *out);

/**
 * @brief 读取原始 LSB 值。
 */
bk_err_t bsp_sc7a20_read_raw(int16_t *x, int16_t *y, int16_t *z);

/**
 * @brief 设置量程。
 */
bk_err_t bsp_sc7a20_set_range(bsp_sc7a20_range_t range);

/**
 * @brief 设置输出速率。
 */
bk_err_t bsp_sc7a20_set_odr(bsp_sc7a20_odr_t odr);

/**
 * @brief 读 WHO_AM_I（SC7A20 应为 0x11）。
 */
bk_err_t bsp_sc7a20_who_am_i(uint8_t *id);

/**
 * @brief 使能运动（AOI1）中断到 INT1 引脚。
 *
 * 配置：CTRL2 高通滤波(去重力) -> INT1_CFG 0x2A（任一轴超阈值，OR）
 *       -> INT1_THS 阈值 -> INT1_DURATION 持续时间 -> CTRL3 IA1 使能
 *       -> CTRL5 LIR_INT1 锁存（读 INT1_SRC 清除）。
 * 中断信号高有效，主控按上升沿触发。
 *
 * @param ths       阈值，1 LSB ≈ 16mg @±2g（128mg -> 8）
 * @param duration  持续时间，1 LSB = 1/ODR（100Hz -> 10ms）
 */
bk_err_t bsp_sc7a20_motion_int_enable(uint8_t ths, uint8_t duration);

/**
 * @brief 关闭运动中断（INT1_CFG=0、CTRL3 复位）。
 */
bk_err_t bsp_sc7a20_motion_int_disable(void);

/**
 * @brief 读 INT1_SRC 清除锁存的中断（LIR 模式下必须调用，否则不再产生新中断）。
 *
 * @param src_out 可为 NULL；非 NULL 时返回寄存器原值（bit6 = IA 有效）
 */
bk_err_t bsp_sc7a20_int_src_read(uint8_t *src_out);

/**
 * @brief 使用的 7bit I2C 地址（init 后有效）。
 */
uint8_t bsp_sc7a20_get_addr(void);

#ifdef __cplusplus
}
#endif
