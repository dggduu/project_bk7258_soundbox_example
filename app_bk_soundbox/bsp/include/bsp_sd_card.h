// Copyright 2026
// SD 卡 BSP：SDIO 4bit（P14~P19）+ 卡检测（P40）+ FATFS 挂载

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* FATFS 卷号：DISK_NUMBER_SDIO_SD = 1 -> "1:" */
#define BSP_SD_MOUNT_POINT  "1:"

/**
 * @brief 检测卡座上是否有卡（读 P40/低有效）。
 */
bool bsp_sd_card_is_inserted(void);

/**
 * @brief 初始化 SDIO 并挂载 FATFS。
 *
 * 已挂载时重复调用直接返回成功。
 */
bk_err_t bsp_sd_card_init(void);

/**
 * @brief 卸载 FATFS 并释放 SDIO。
 */
bk_err_t bsp_sd_card_deinit(void);

/**
 * @brief 卡是否已挂载可用。
 */
bool bsp_sd_card_is_mounted(void);

/**
 * @brief 获取卡容量（MB）。
 */
bk_err_t bsp_sd_card_get_capacity_mb(uint32_t *mb);

/**
 * @brief 获取卡信息（版本/类型/速度等级）。
 */
bk_err_t bsp_sd_card_get_info(uint32_t *version, uint32_t *type, uint32_t *speed_class);

/**
 * @brief 自检：写文件再读回比对，用于验证 FATFS 通路。
 *
 * @param path 例如 "1:/bsp_sd_test.txt"
 */
bk_err_t bsp_sd_card_self_test(const char *path);

#ifdef __cplusplus
}
#endif
