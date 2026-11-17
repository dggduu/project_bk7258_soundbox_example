// Copyright 2026
// 三模（BT / USB / SD）模式管理与全局命令队列

#pragma once

#include <common/bk_err.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 创建应用任务、初始化三个模式 */
void app_mode_init(void);

/* 上电默认模式（先跑 UI，再进业务） */
void app_mode_start(void);

/* 自动请求（BT/USB 检测到外部开始播放时调用），SD 正在播放时不抢占 */
void app_mode_request(app_mode_t mode);

/* 手动切换（按键/界面选择），强制生效 */
void app_mode_switch_manual(app_mode_t mode);

/* 播放控制（UI 线程调用，内部只投递消息） */
void app_mode_play_pause(void);
void app_mode_next(void);
void app_mode_prev(void);
/* 快进/快退（毫秒）：SD 为精确定位，BT 为 AVRCP 尽力而为，USB 由主机控制 */
void app_mode_seek(int delta_ms);
void app_mode_volume_step(int delta);
void app_mode_volume_set(uint8_t volume);
void app_mode_play_file(const char *path);
void app_mode_bt_pairing(void);
void app_mode_backlight_toggle(void);

/* 摇一摇切歌（app_shake 检测到摇动时调用；是否生效由开关 + 当前模式决定） */
void app_mode_shake_next(void);
/* 设置页：打开/关闭摇一摇切歌 */
void app_mode_shake_toggle(void);

/* 请求 app 主循环立刻跑一轮 poll（打断 100ms 队列等待）。
 * 给 USB 用：主机 SET_INTERFACE(alt=1) 开录音流后必须尽快启动 DMIC，
 * 否则主机收到的头几包全是静音。可在中断上下文调用（非阻塞入队）。 */
void app_mode_poll_now(void);

/* 当前模式是否正在出声（自动切换策略用） */
bool app_mode_is_busy(void);

#ifdef __cplusplus
}
#endif
