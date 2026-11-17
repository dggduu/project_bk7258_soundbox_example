/**
 * @file a2dp_sink_demo.h
 *
 */

#ifndef A2DP_SINK_DEMO_H
#define A2DP_SINK_DEMO_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

int a2dp_sink_demo_init(uint8_t aac_supported);
int32_t bk_bt_app_avrcp_ct_get_attr(uint32_t attr);
int32_t bt_avrcp_poll_attrs(void);
int32_t bt_avrcp_request_cover_art(void);

/* AVRCP passthrough（手机侧播放控制） */
void bk_bt_app_avrcp_ct_play(void);
void bk_bt_app_avrcp_ct_pause(void);
void bk_bt_app_avrcp_ct_next(void);
void bk_bt_app_avrcp_ct_prev(void);

/* 快退/快进：按下 REWIND / FAST_FORWARD 保持 ms 毫秒后松开。
 * 实际跳多少由手机 App 决定，蓝牙侧无法保证精确 ±10s。 */
void bk_bt_app_avrcp_ct_rewind(uint32_t ms);
void bk_bt_app_avrcp_ct_fast_forward(uint32_t ms);

/* 本机音量变化后同步给对端（AVRCP TG VOLUME_CHANGE 通知） */
void bt_avrcp_report_volume_percent(uint8_t percent);

/* 手机拖动音量时的回发去抖：由 app_bt_poll 周期调用（~200ms），
 * 值稳定 80ms 后才回一个 CHANGED，避免每步都占链路导致音量滞后。 */
void bt_avrcp_volume_poll(void);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif /*A2DP_SINK_DEMO_H*/
