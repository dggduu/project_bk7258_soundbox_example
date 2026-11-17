// Copyright 2026
// USB 麦克风采集 BSP（DMIC -> PCM16）
//
// 硬件：U15 / U16 = MEMS 数字麦，CLK 共用 P8，DATA 共用 P9（单线双通道）
//   U16 的 L/R 脚接 GND  -> 左声道
//   U15 的 L/R 脚接 1V8  -> 右声道
//   1V8 供电由 P48（BSP_PWR_RAIL_MIC_1V8，低有效 PMOS）控制
//
// 实现基于 SDK 官方组件 onboard_dual_dmic_mic_stream，
// 输出 16bit L/R 交织 PCM，采样率 16k/32k/48k 任选。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint32_t samp_rate;     /* 16000 / 32000 / 48000，默认 48000 */
	uint16_t frame_ms;      /* 每帧时长，默认 20ms */
	bool stereo;            /* true=双麦 L/R，false=只输出左声道 */
} bsp_mic_cfg_t;

/**
 * @brief 填充默认配置（48kHz / 20ms / 双声道）。
 */
void bsp_mic_default_cfg(bsp_mic_cfg_t *cfg);

/**
 * @brief 初始化：MIC 1V8 上电、创建 DMIC 采集元素、建输出 ringbuffer。
 *
 * 初始化完成后不会自动采集，需再调 bsp_mic_start()。
 */
bk_err_t bsp_mic_init(const bsp_mic_cfg_t *cfg);

/**
 * @brief 启动采集（DMA/DMIC/ADC 全部起来）。
 */
bk_err_t bsp_mic_start(void);

/**
 * @brief 停止采集（保留资源，可再次 start）。
 */
bk_err_t bsp_mic_stop(void);

/**
 * @brief 释放资源并给 MIC 断电。
 */
bk_err_t bsp_mic_deinit(void);

/**
 * @brief 读一帧 16bit PCM（L/R 交织）。
 *
 * @param buf         输出缓冲，长度需 >= bsp_mic_frame_bytes()
 * @param len         缓冲长度
 * @param timeout_ms  等待超时
 * @return BK_OK 成功；BK_ERR_TIMEOUT 数据还没到（正常情况，调用方重试即可）
 */
bk_err_t bsp_mic_read(void *buf, uint32_t len, uint32_t timeout_ms);

/**
 * @brief 输出端口当前可读字节数（诊断用）。<0 表示未初始化。
 */
int bsp_mic_filled(void);

/**
 * @brief 单帧字节数（block 大小）/ 单帧每声道采样点数。
 */
uint32_t bsp_mic_frame_bytes(void);
uint32_t bsp_mic_frame_samples(void);

/**
 * @brief 当前采样率 / 声道数。
 */
uint32_t bsp_mic_sample_rate(void);
uint32_t bsp_mic_channels(void);

/**
 * @brief 是否已启动采集。
 */
bool bsp_mic_is_running(void);

#ifdef __cplusplus
}
#endif
