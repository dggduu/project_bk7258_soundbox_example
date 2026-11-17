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

#define BSP_AUDIO_MAX_GAIN  0x2D   /* 约 0dB；0x3F 会数字削顶/破音 */

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
	if (c.pool_size == 0) {
		/* 池子 = 帧数 × 单帧时长。原来 12 帧（10ms/帧 → 120ms）在
		 * “UI 整帧刷屏(单线 SPI ~38ms) + SD 读卡 + BT 解码”同时发生时不够，
		 * onboard_speaker 的供数线程在池子空时会**补一整帧静音**，
		 * 每补一次就是一个 10ms 的静音缺口 —— 听感就是周期性的“嗡/咔”。
		 * 加到 24 帧（240ms）后，常见抖动都能被吸收。 */
		c.pool_size = c.frame_size * 24;
	}

	s_gain = bsp_audio_percent_to_gain(c.volume);
	s_cfg = c;
	s_cfg.volume = c.volume;

	/* ==================== 开机爆音的根因与顺序 ====================
	 *
	 * NS4150B 这类差分功放只要 CTRL 拉高就立刻开始放大输入端的一切，
	 * 包括 DAC 上电过程中那段**直流偏置建立**的瞬态。原来的顺序是
	 *
	 *     pa_enable(true) -> 10ms -> audio_play_create/open
	 *
	 * 也就是"先给功放上电，再让 DAC 起来"，DAC 从 0V 爬到中点电压的那
	 * 一下被功放完整放大 -> 喇叭"啪"一声。这跟有没有音频数据无关，
	 * 所以每次 open 都会响，音量越小越不明显、开大音量尤其刺耳。
	 *
	 * 正确顺序（也是绝大多数功放的推荐上电时序）：
	 *   1) 先把 DAC 通道建起来
	 *   2) **在功放还关着的时候**把 DAC 拉到静音，等它偏置稳定
	 *   3) 再开功放 —— 此时功放看到的是稳定的静音电平，没有阶跃
	 *   4) 最后解除静音，开始送真实数据
	 *
	 * 第 2 步的 30ms 是给 DAC 输出耦合电容和内部参考建立用的，
	 * 短了仍会有轻微"哒"声。 */

	/* 1. 创建 onboard speaker 播放通道（此时功放仍然关着） */
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
		return BK_FAIL;
	}

	ret = audio_play_open(s_play);
	if (ret != BK_OK) {
		BSP_LOGE("audio_play_open failed: %d\r\n", ret);
		audio_play_destroy(s_play);
		s_play = NULL;
		return ret;
	}

	/* 2. 功放关着的时候先静音，并等 DAC 偏置稳定 */
	audio_play_control(s_play, AUDIO_PLAY_MUTE);
	rtos_delay_milliseconds(30);

	/* 3. 现在才开功放：它看到的是稳定静音，不会放大上电瞬态 */
	ret = bsp_audio_pa_enable(true);
	if (ret != BK_OK) {
		BSP_LOGE("PA enable failed: %d\r\n", ret);
		audio_play_close(s_play);
		audio_play_destroy(s_play);
		s_play = NULL;
		return ret;
	}
	rtos_delay_milliseconds(10);   /* 等功放自身软启动完成 */

	s_ready = true;

	/* 4. 解除静音，开始送真实数据。
	 *
	 * 注意这里的条件与"上一步的静音"是配对的：volume == 0 时保持静音，
	 * 否则一定要 UNMUTE —— 否则上面那次 AUDIO_PLAY_MUTE 没人解开，
	 * 表现为"播放正常但没有声音"（比爆音更难查，因为状态全对）。 */
	if (c.volume == 0)
		audio_play_control(s_play, AUDIO_PLAY_MUTE);
	else
		audio_play_control(s_play, AUDIO_PLAY_UNMUTE);

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

	s_gain = bsp_audio_percent_to_gain(percent);

	/* DAC 数字增益 0 并不等于静音（还能听到残留/底噪），音量 0 必须走
	 * 硬件的 DAC mute 位（bk_aud_dac_mute → sys_drv_aud_dac_dacmute_en）。
	 * 这是"音量为 0 还有声音"的根因。 */
	if (percent == 0) {
		audio_play_set_volume(s_play, 0);
		ret = audio_play_control(s_play, AUDIO_PLAY_MUTE);
	} else {
		audio_play_control(s_play, AUDIO_PLAY_UNMUTE);
		ret = audio_play_set_volume(s_play, s_gain);
	}

	if (ret != BK_OK)
		BSP_LOGE("set volume %u failed: %d\r\n", percent, ret);
	else
		s_cfg.volume = percent;

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

	if (s_play && s_ready) {
		/* 1. 数字音量分 6 步降到 0：避免"突然断流"式的破音 */
		uint8_t vol = s_cfg.volume;

		for (int i = 1; i <= 6; i++) {
			uint8_t v = (uint8_t)(vol - (vol * i) / 6);

			audio_play_set_volume(s_play, bsp_audio_percent_to_gain(v));
			rtos_delay_milliseconds(3);
		}

		/* 2. 再 mute，确保模拟输出被彻底切断（音量 0 只是数字静音，
		 *    DAC 仍输出中点直流） */
		audio_play_control(s_play, AUDIO_PLAY_MUTE);

		/* 3. 等 DMA 把队列里剩下的帧排空、DAC 输出稳定在中点。
		 *    20ms ≈ 两个 10ms 帧，少了会把还没播完的数据硬切断。 */
		rtos_delay_milliseconds(20);
	}

	/* 4. 【先关功放，再关 DAC】
	 *
	 * 反过来（原来就是这么写的：close -> destroy -> pa_off）会让功放在
	 * DAC 停止输出的那一瞬间还在放大 —— DAC 从"有信号"跳到"无输出"
	 * 是个阶跃，功放照放不误，喇叭"啪"一声。现在 DAC 已经静音且稳定，
	 * 功放断电时输入是平的，就没有可放大的东西。 */
	bsp_audio_pa_enable(false);
	rtos_delay_milliseconds(5);

	/* 5. 最后才拆 DAC 通道 */
	if (s_play) {
		ret = audio_play_close(s_play);
		audio_play_destroy(s_play);
		s_play = NULL;
	}

	s_ready = false;
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
