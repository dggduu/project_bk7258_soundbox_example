// Copyright 2026
// 蓝牙模式封装：A2DP sink + AVRCP CT（车载歌词）

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 早期 Bluetooth 协议栈初始化（main 里、media_service_init 之后调用，
 * 对应 SDK headset 例程的 bt_manager_init；只做名字/可发现，不阻塞） */
void app_bt_early_init(void);

/* 创建 A2DP/AVRCP 初始化线程（约需数秒，不阻塞 UI） */
void app_bt_init(void);

/* 进入/离开蓝牙模式：离开时暂停手机播放并让出 DAC */
void app_bt_set_active(bool active);

bool app_bt_is_ready(void);
bool app_bt_is_connected(void);

/* 手机侧播放控制（AVRCP passthrough） */
void app_bt_play_pause(void);
void app_bt_next(void);
void app_bt_prev(void);

/* 快进/快退（AVRCP REWIND / FAST_FORWARD）。delta_ms 只取符号，
 * 手机侧决定实际跳转量，无法像 SD 本地文件那样精确定位。 */
void app_bt_seek(int delta_ms);

/* 让手机重新进入可发现/可连接状态 */
void app_bt_enter_pairing(void);

/* 本机音量变化后同步给对端（AVRCP TG 音量通知），
 * 手机/Windows 的音量滑块会跟着动；未连接时静默忽略 */
void app_bt_report_volume(uint8_t percent);

/* 由 app 任务周期调用：同步连接状态、周期性拉取 AVRCP TITLE */
void app_bt_poll(void);

#ifdef __cplusplus
}
#endif
