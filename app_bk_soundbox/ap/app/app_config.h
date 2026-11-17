// Copyright 2026
// 用户配置持久化（easyflash V4）
//
// 保存：音量、背光、摇一摇开关、字体来源（SD TTF / 内置）。
// 用单个 blob 存（4 字节 + magic），减少 easyflash 的 key 数量与擦写次数。
// 写入是"延迟去抖"的：改音量会连续触发，静默 2s 后才真正落盘一次。

#pragma once

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 初始化 easyflash 并把保存的配置应用回 app_state（app_state_init 之后调用）。 */
void app_config_load(void);

/** @brief 有配置变化，请求保存（去抖，实际写入由 app_config_poll 完成）。 */
void app_config_save_request(void);

/** @brief 周期调用（app_mode 的 poll，200ms 节奏）：静默 2s 后落盘。 */
void app_config_poll(void);

/** @brief 按 app_state.font_sd 应用字体来源（bsp_font_init 之后调用）。 */
void app_config_apply_font(void);

#ifdef __cplusplus
}
#endif
