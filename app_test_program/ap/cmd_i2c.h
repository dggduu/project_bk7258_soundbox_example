// Copyright 2026
// cmd_i2c.c 暴露给其它器件调试命令的总线 helper。
//
// 为什么不各自 bk_i2c_init()：cmd_i2c.c 里的 s_hw_inited[] 记录着总线状态，
// 多个模块各自维护一份会互相 deinit；统一走这里，两个命令文件的行为一致。
// 说明：bus=0 -> I2C_ID_0（摄像头 SCCB），bus=1 -> I2C_ID_1（触摸/加速度计）。

#pragma once

#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

bk_err_t i2c_bus_init(int bus);

/**
 * @brief 设置总线波特率并立即生效（已初始化的总线会 deinit + 重建）。
 * @param hz 10000..400000；SCCB 实际上限 400k，信号差的板子建议 50k~100k
 */
bk_err_t i2c_bus_set_baud(int bus, uint32_t hz);

/** @brief 查询总线当前波特率（未设置过返回默认 100k） */
uint32_t i2c_bus_get_baud(int bus);

/** @brief 无视缓存强制重建总线（deinit -> init -> 重新映射引脚）。
 *  第三方代码调过 bk_i2c_init/deinit 之后用它恢复引脚复用。 */
bk_err_t i2c_bus_force_rebuild(int bus);

/** @brief 总线恢复：9 个 SCL 脉冲 + STOP，释放被从机拉住的 SCL/SDA，
 *  然后恢复 I2C 复用并重建控制器。从机卡在半截传输里时只有这个办法。 */
bk_err_t i2c_bus_clear(int bus);

void     i2c_bus_deinit(int bus);

#ifdef __cplusplus
}
#endif
