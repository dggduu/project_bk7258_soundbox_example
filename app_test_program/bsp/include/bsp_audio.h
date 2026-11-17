// Copyright 2026
// 音频播放 BSP（内部 DAC -> AUDLP/AUDN -> NS4150B 差分功放 -> CN3 喇叭）
//
// 板级职责：PA_EN(P52) 上电时序 + 封装 SDK 的 audio_play 抽象层。
// 数据格式：PCM 16bit，采样率/声道可配（单声道/立体声）。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t samp_rate;     /* 默认 16000，音乐可用 44100/48000 */
	uint8_t channels;       /* 1 = 单声道，2 = 立体声 */
	uint8_t bits;           /* 默认 16 */
	uint8_t volume;         /* 0~100，默认 70 */
	uint32_t frame_size;    /* 每帧字节数，0 = 按采样率自动算（约 10ms） */
	uint32_t pool_size;     /* 播放环形缓冲字节数，0 = 8 帧 */
} bsp_audio_cfg_t;

/**
 * @brief 填充默认配置（16kHz / 单声道 / 16bit / 音量 70）。
 */
void bsp_audio_default_cfg(bsp_audio_cfg_t *cfg);

/**
 * @brief 初始化播放：打开 PA 电源、创建并打开 onboard speaker 播放通道。
 *
 * 内部会等功放稳定后再返回，之后可直接 write。
 * 如果已经在播放，返回 BK_OK（重复初始化无副作用）。
 */
bk_err_t bsp_audio_init(const bsp_audio_cfg_t *cfg);

/**
 * @brief 写入 PCM 数据（阻塞，直到数据全部送入播放缓冲）。
 */
bk_err_t bsp_audio_write(const void *pcm, uint32_t len);

/**
 * @brief 设置音量（0~100，内部换算成 DAC 数字增益 0x00~0x3F）。
 */
bk_err_t bsp_audio_set_volume(uint8_t percent);

/**
 * @brief 暂停 / 恢复播放。
 */
bk_err_t bsp_audio_pause(void);
bk_err_t bsp_audio_resume(void);

/**
 * @brief 停止播放并关闭 PA（喇叭静音，消除底噪）。
 */
bk_err_t bsp_audio_stop(void);

/**
 * @brief 完全释放（close + destroy + 关 PA）。
 */
bk_err_t bsp_audio_deinit(void);

/**
 * @brief 播放通道是否已打开。
 */
bool bsp_audio_is_ready(void);

/**
 * @brief 单独控制功放使能（调试用，正常流程由 init/stop 管理）。
 */
bk_err_t bsp_audio_pa_enable(bool on);

#ifdef __cplusplus
}
#endif
