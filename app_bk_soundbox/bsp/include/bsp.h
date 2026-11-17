// Copyright 2026
// app_bk_pi 三模音箱 BSP 统一入口
//
// 只包含本产品实际使用的外设：
//   power / adc key / battery / screen / LVGL / DAC+PA audio / SDIO SD
// 摄像头、触摸、MIC 不在本工程启用。

#pragma once

#include <common/bk_err.h>
#include "bsp_board_cfg.h"
#include "bsp_power.h"
#include "bsp_adc_util.h"
#include "bsp_battery.h"
#include "bsp_key.h"
#include "bsp_screen.h"
#include "bsp_audio.h"
#include "bsp_mic.h"
#include "bsp_sd.h"
#include "bsp_lvgl.h"
#include "bsp_font.h"
#include "bsp_sc7a20.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 基础板级初始化：电源轨默认关闭、控制脚就绪。
 *
 * 屏幕/音频/SD/LVGL 由应用按需初始化。
 */
bk_err_t bsp_init(void);

/**
 * @brief 打印关键引脚映射（调试用）。
 */
void bsp_dump_pinmap(void);

#ifdef __cplusplus
}
#endif
