// Copyright 2026
// A2DP/AVRCP 底层与 app_bt 之间的钩子
//
// a2dp_sink_demo.c（移植自 SDK headset 示例）在 A2DP / AVRCP 事件里调用
// 这些函数，把手机侧状态送到应用层。所有回调都运行在蓝牙协议栈任务上下文，
// 实现里只做拷贝/置位，不做阻塞操作。

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* A2DP 音频开始/停止（参数是协商出来的采样率和声道数） */
void app_bt_on_a2dp_start(uint32_t sample_rate, uint8_t channels);
void app_bt_on_a2dp_stop(void);
void app_bt_on_a2dp_connected(void);

/* AVRCP 连接状态 */
void app_bt_on_avrcp_connected(bool connected);

/* 车载歌词：手机把当前歌词行放进 TITLE（也可能只是普通歌名） */
void app_bt_on_avrcp_attr(uint32_t attr_id, const char *text, uint32_t len,
                          uint16_t charset);

/* AVRCP 播放状态 / 播放进度 */
void app_bt_on_play_status(uint8_t status);
void app_bt_on_play_pos(uint32_t pos_ms);

/* 曲目切换（elm_id 变化） */
void app_bt_on_track_change(void);

/* 让 A2DP speaker 线程允许/禁止占用 DAC（离开 BT 模式时置 false） */
void bt_audio_sink_set_output_active(bool active);
bool bt_audio_sink_is_output_active(void);
bool bt_audio_sink_is_audio_opened(void);

/* A2DP 音频健康度快照：给 app_bt_poll() 的心跳日志用。
 *   pkt    累计到达包数（在数据回调入口无条件自增）
 *   drop   分配失败/入队失败丢弃的包数
 *   resets 音频通路被看门狗重建的次数
 * 判据：pkt 不再涨 = 主机侧断流；pkt 在涨但 resets 也在涨 = 本机音频通路反复死。 */
void bt_audio_sink_stats(uint32_t *pkt, uint32_t *drop, uint32_t *resets);

#ifdef __cplusplus
}
#endif
