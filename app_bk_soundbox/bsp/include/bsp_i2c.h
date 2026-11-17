// Copyright 2026
// 板级 I2C 总线管理（I2C0 = 摄像头 SCCB，I2C1 = 触摸 + SC7A20）

#pragma once

#include <common/bk_err.h>
#include <driver/i2c.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	BSP_I2C_BUS_CAM = 0,   /* I2C0: SCL=P20, SDA=P21 */
	BSP_I2C_BUS_SYS,       /* I2C1: SCL=P42, SDA=P43 */
	BSP_I2C_BUS_MAX,
} bsp_i2c_bus_t;

/**
 * @brief 初始化总线（幂等，重复调用只增加引用计数）。
 *
 * I2C1 会先把引脚切到 GPIO42/43 复用组。
 */
bk_err_t bsp_i2c_init(bsp_i2c_bus_t bus);

/**
 * @brief 释放总线（引用计数归零时才真正 deinit）。
 */
bk_err_t bsp_i2c_deinit(bsp_i2c_bus_t bus);

/**
 * @brief 总线是否已初始化。
 */
bool bsp_i2c_is_ready(bsp_i2c_bus_t bus);

/**
 * @brief 读 8bit 寄存器。
 */
bk_err_t bsp_i2c_read_reg8(bsp_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t *val);

/**
 * @brief 写 8bit 寄存器。
 */
bk_err_t bsp_i2c_write_reg8(bsp_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t val);

/**
 * @brief 连续读多个 8bit 寄存器（地址自增）。
 */
bk_err_t bsp_i2c_read_buf(bsp_i2c_bus_t bus, uint8_t dev_addr, uint8_t reg, uint8_t *buf, uint32_t len);

/**
 * @brief 探测设备是否应答。
 */
bool bsp_i2c_probe(bsp_i2c_bus_t bus, uint8_t dev_addr);

#ifdef __cplusplus
}
#endif
