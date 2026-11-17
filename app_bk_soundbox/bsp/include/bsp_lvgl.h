// Copyright 2026
// LVGL 8 显示后端：240x320 RGB565，direct mode 全屏帧缓冲（PSRAM）
//
// 屏幕是 ILI9341 单线 SPI，整帧 150KB，flush 一次约 40ms（32MHz SPI）。
// 对于音箱 UI（进度条 1s 更新、歌词 1s 更新）这个刷新率足够，
// 换来的是不需要处理 partial area 的 4 字节对齐问题。

#pragma once

#include <common/bk_err.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 每轮 LVGL 主循环前调用（在 LVGL 任务上下文里）。 */
typedef void (*bsp_lvgl_loop_hook_t)(void);

/**
 * @brief 初始化 LVGL、注册显示驱动，帧缓冲放 PSRAM。
 *
 * @param hook 可选，LVGL 任务每轮调用一次；用于处理按键队列、刷新业务状态。
 */
bk_err_t bsp_lvgl_init(bsp_lvgl_loop_hook_t hook);

/**
 * @brief 启动 LVGL 任务（5ms 周期）。
 */
bk_err_t bsp_lvgl_start(void);

/**
 * @brief 停止 LVGL 任务。
 */
bk_err_t bsp_lvgl_stop(void);

/**
 * @brief 释放帧缓冲。
 */
bk_err_t bsp_lvgl_deinit(void);

bool bsp_lvgl_is_ready(void);

#ifdef __cplusplus
}
#endif
