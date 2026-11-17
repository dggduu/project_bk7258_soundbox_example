// Copyright 2026
// SD 卡 BSP：SDIO 4bit（P14~P19）+ 卡检测（P40）+ FATFS 挂载
//
// 与 app_test_uart1 的 sd 测试保持一致：
//   1) 先给卡座 3V3 LDO 投票（GPIO13）
//   2) bk_sd_card_init() 完成卡初始化
//   3) f_mount("1:") 挂 FATFS
// 盘符是 1（DISK_NUMBER_SDIO_SD），不是 0。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_SD_MOUNT_POINT  "1:"

/**
 * @brief 检测卡座上是否有卡（读 P40，低有效）。
 */
bool bsp_sd_is_inserted(void);

/**
 * @brief 初始化 SDIO 并挂载 FATFS；已挂载时直接返回成功。
 */
bk_err_t bsp_sd_mount(void);

/**
 * @brief 卸载 FATFS 并释放 SDIO。
 */
bk_err_t bsp_sd_unmount(void);

/**
 * @brief 卡是否已挂载可用。
 */
bool bsp_sd_is_mounted(void);

/**
 * @brief 卡容量（MB）。
 */
bk_err_t bsp_sd_capacity_mb(uint32_t *mb);

#ifdef __cplusplus
}
#endif
