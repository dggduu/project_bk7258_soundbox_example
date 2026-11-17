// Copyright 2026
// 应用层音频路由：同一时刻只允许一个模式占用 DAC/功放
//
// BT / SD / USB 三个模式在开始出声前调用 app_audio_open()，
// 退出模式时调用 app_audio_close()。音量统一走 app_state。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

bk_err_t app_audio_open(uint32_t samp_rate, uint8_t channels);
bk_err_t app_audio_write(const void *pcm, uint32_t len);
bk_err_t app_audio_pause(void);
bk_err_t app_audio_resume(void);
void app_audio_close(void);

bool app_audio_is_open(void);
uint32_t app_audio_rate(void);

/* 音量 0~100，内部换算成 DAC 增益；未打开时只记录到全局状态。 */
bk_err_t app_audio_set_volume(uint8_t volume);
uint8_t app_audio_get_volume(void);

#ifdef __cplusplus
}
#endif
