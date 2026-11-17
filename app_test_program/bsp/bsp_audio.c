// Copyright 2026
// 音频播放 BSP 实现

#include "bsp_audio.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#include "audio_play.h"

#define TAG "bsp_audio"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define BSP_AUDIO_MAX_GAIN  0x3F

static audio_play_t *s_play = NULL;
static bsp_audio_cfg_t s_cfg;
static uint8_t s_gain = 0x2D;
static bool s_ready = false;

/* 0~100 映射到 DAC 数字增益 0x00~0x3F（0x2D 约 0dB） */
static uint8_t bsp_audio_percent_to_gain(uint8_t percent)
{
	if (percent > 100)
		percent = 100;

	return (uint8_t)(((uint32_t)percent * BSP_AUDIO_MAX_GAIN + 50) / 100);
}

void bsp_audio_default_cfg(bsp_audio_cfg_t *cfg)
{
	if (!cfg)
		return;

	cfg->samp_rate = 16000;
	cfg->channels = 1;
	cfg->bits = 16;
	cfg->volume = 70;
	cfg->frame_size = 0;
	cfg->pool_size = 0;
}

bk_err_t bsp_audio_pa_enable(bool on)
{
	return bsp_power_enable(BSP_PWR_RAIL_PA, on);
}

bk_err_t bsp_audio_init(const bsp_audio_cfg_t *cfg)
{
	audio_play_cfg_t play_cfg;
	bsp_audio_cfg_t c;
	uint32_t bytes_per_sec;
	bk_err_t ret;

	if (s_ready)
		return BK_OK;

	if (cfg)
		c = *cfg;
	else
		bsp_audio_default_cfg(&c);

	if ((c.channels != 1) && (c.channels != 2)) {
		BSP_LOGE("unsupported channels: %d\r\n", c.channels);
		return BK_FAIL;
	}
	if (c.bits != 16) {
		BSP_LOGE("unsupported bits: %d (only 16bit)\r\n", c.bits);
		return BK_FAIL;
	}
	if (c.samp_rate == 0) {
		BSP_LOGE("invalid sample rate\r\n");
		return BK_FAIL;
	}

	bytes_per_sec = c.samp_rate * c.channels * (c.bits / 8);

	/* 默认一帧约 10ms，按 4 字节对齐（立体声 16bit 一个采样点 4 字节） */
	if (c.frame_size == 0) {
		c.frame_size = (bytes_per_sec / 100) & ~3U;
		if (c.frame_size < 64)
			c.frame_size = 64;
	}
	if (c.pool_size == 0)
		c.pool_size = c.frame_size * 8;

	s_gain = bsp_audio_percent_to_gain(c.volume);
	s_cfg = c;
	s_cfg.volume = c.volume;

	/* 1. 功放上电（NS4150B CTRL 高有效） */
	ret = bsp_audio_pa_enable(true);
	if (ret != BK_OK) {
		BSP_LOGE("PA enable failed: %d\r\n", ret);
		return ret;
	}
	rtos_delay_milliseconds(10);   /* 等功放偏置稳定，避免开机 pop 声 */

	/* 2. 创建 onboard speaker 播放通道 */
	os_memset(&play_cfg, 0, sizeof(play_cfg));
	play_cfg.port = 0;
	play_cfg.nChans = c.channels;
	play_cfg.sampRate = c.samp_rate;
	play_cfg.bitsPerSample = c.bits;
	play_cfg.volume = s_gain;
	play_cfg.play_mode = AUDIO_PLAY_MODE_DIFFEN;
	play_cfg.frame_size = c.frame_size;
	play_cfg.pool_size = c.pool_size;

	s_play = audio_play_create(AUDIO_PLAY_ONBOARD_SPEAKER, &play_cfg);
	if (!s_play) {
		BSP_LOGE("audio_play_create failed\r\n");
		bsp_audio_pa_enable(false);
		return BK_FAIL;
	}

	ret = audio_play_open(s_play);
	if (ret != BK_OK) {
		BSP_LOGE("audio_play_open failed: %d\r\n", ret);
		audio_play_destroy(s_play);
		s_play = NULL;
		bsp_audio_pa_enable(false);
		return ret;
	}

	s_ready = true;
	BSP_LOGI("audio ready: %uHz %uch %ubit vol=%u gain=0x%02x frame=%u pool=%u\r\n",
	         c.samp_rate, c.channels, c.bits, c.volume, s_gain,
	         c.frame_size, c.pool_size);
	return BK_OK;
}

bk_err_t bsp_audio_write(const void *pcm, uint32_t len)
{
	bk_err_t ret;

	if (!s_ready || !s_play)
		return BK_ERR_NOT_INIT;
	if (!pcm || !len)
		return BK_ERR_NULL_PARAM;

	/* 注意：audio_play_write_data() 成功时返回的是写入长度（正数），
	 * 失败才返回 BK_FAIL；这里统一归一成 BK_OK / 负数错误码。 */
	ret = audio_play_write_data(s_play, (char *)pcm, len);
	if (ret == (bk_err_t)len)
		return BK_OK;

	return (ret < 0) ? ret : BK_FAIL;
}

bk_err_t bsp_audio_set_volume(uint8_t percent)
{
	bk_err_t ret;

	if (percent > 100)
		percent = 100;

	if (!s_ready || !s_play)
		return BK_ERR_NOT_INIT;

	ret = audio_play_set_volume(s_play, bsp_audio_percent_to_gain(percent));
	if (ret == BK_OK) {
		s_cfg.volume = percent;
		s_gain = bsp_audio_percent_to_gain(percent);
	}

	return ret;
}

bk_err_t bsp_audio_pause(void)
{
	if (!s_ready || !s_play)
		return BK_ERR_NOT_INIT;

	return audio_play_control(s_play, AUDIO_PLAY_PAUSE);
}

bk_err_t bsp_audio_resume(void)
{
	if (!s_ready || !s_play)
		return BK_ERR_NOT_INIT;

	return audio_play_control(s_play, AUDIO_PLAY_RESUME);
}

bk_err_t bsp_audio_stop(void)
{
	bk_err_t ret = BK_OK;

	if (s_play) {
		ret = audio_play_close(s_play);
		audio_play_destroy(s_play);
		s_play = NULL;
	}

	s_ready = false;

	/* 关 PA，避免喇叭持续底噪 */
	bsp_audio_pa_enable(false);
	return ret;
}

bk_err_t bsp_audio_deinit(void)
{
	return bsp_audio_stop();
}

bool bsp_audio_is_ready(void)
{
	return s_ready;
}
