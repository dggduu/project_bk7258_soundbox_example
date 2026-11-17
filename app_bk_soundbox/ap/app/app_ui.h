// Copyright 2026
// LVGL 竖屏界面：播放（封面+歌词）/ SD 文件 / 模式切换 / 设置 + 底部 Tab
//
// 所有 LVGL 调用都发生在 LVGL 任务上下文（由 bsp_lvgl 的 loop hook 驱动），
// 其他线程只更新 app_state，通过 app_state_copy() 读取快照。

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 创建界面（LVGL 初始化之后、bsp_lvgl_start 之前调用） */
void app_ui_init(void);

/* 每轮 LVGL 主循环调用（处理按键队列 + 刷新状态） */
void app_ui_loop(void);

/* 按键任务投递按键事件（线程安全） */
void app_ui_post_key(uint8_t key, uint8_t evt);

/* 请求亮屏（熄屏时拿起设备由加速度计回调触发；线程安全，只置标志） */
void app_ui_wakeup_request(void);

#ifdef __cplusplus
}
#endif
