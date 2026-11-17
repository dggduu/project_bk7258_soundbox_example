// Copyright 2026
// 应用层音频路由实现

#include "app_audio.h"
#include "app_state.h"
#include "bsp.h"

#include <os/os.h>
#include <components/log.h>
#include <driver/pwr_clk.h>       /* bk_pm_module_vote_cpu_freq() */
#include <modules/pm.h>           /* bk_pm_module_vote_sleep_ctrl() */

#define TAG "app_audio"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

static bool s_open;
static uint32_t s_rate;
static uint8_t s_ch;
static beken_mutex_t s_lock;

/* 播放时提频到 240M（解码/重采样余量），关闭时投回默认频率省电。
 * 三模共用：BT SBC、UAC PCM、SD 解码全走这里。 */
static void audio_cpu_vote(bool high)
{
	if (high)
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_240M);
	else
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_DEFAULT);
}

/* 【播放期间禁止进入休眠】
 *
 * sleep_state 的语义容易反：0x1 = "我睡了"，0x0 = "我还在跑"。
 * 后者会清掉 APP 这一路的睡眠票，PM 就不会在 BTSP/APP 全票时把整颗
 * 芯片降到低电压（CP 侧 CONFIG_PM_ENTER_LV_TIME_OUT_PERIOD_MS=30000，
 * 30 秒无活动就会尝试）。降电压期间 DAC DMA / PSRAM 时序最容易出问题，
 * 表现正是"放一会儿就卡住"。
 *
 * AP 侧这个调用会经 mailbox 转给 CP，由 CP 的 PM 汇总所有模块的票。 */
static void audio_sleep_vote(bool allow_sleep)
{
	bk_pm_module_vote_sleep_ctrl(PM_SLEEP_MODULE_NAME_APP,
	                             allow_sleep ? 0x1 : 0x0, 0);
}

static void audio_lock_init(void)
{
	if (!s_lock)
		rtos_init_mutex(&s_lock);
}

bk_err_t app_audio_open(uint32_t samp_rate, uint8_t channels)
{
	bsp_audio_cfg_t cfg;
	bk_err_t ret;
	bool was_open;

	if ((samp_rate < 8000) || (samp_rate > 48000) ||
	    ((channels != 1) && (channels != 2))) {
		LOGE("bad format %uHz %uch\r\n", (unsigned)samp_rate, channels);
		return BK_ERR_PARAM;
	}

	audio_lock_init();
	if (s_lock)
		rtos_lock_mutex(&s_lock);

	was_open = s_open;

	if (s_open && (s_rate == samp_rate) && (s_ch == channels)) {
		if (s_lock)
			rtos_unlock_mutex(&s_lock);
		return BK_OK;
	}

	if (s_open) {
		bsp_audio_stop();
		s_open = false;
	}

	bsp_audio_default_cfg(&cfg);
	cfg.samp_rate = samp_rate;
	cfg.channels = channels;
	cfg.bits = 16;
	cfg.volume = app_state_volume();
	cfg.frame_size = 0;     /* BSP 按采样率自动算 10ms */
	cfg.pool_size = 0;

	ret = bsp_audio_init(&cfg);
	if (ret != BK_OK) {
		LOGE("audio open failed: %d\r\n", ret);
	} else {
		s_open = true;
		s_rate = samp_rate;
		s_ch = channels;
		if (!was_open) {
			audio_cpu_vote(true);
			audio_sleep_vote(false);   /* 播放期间禁止休眠 */
		}
		LOGI("audio open %uHz %uch vol=%u\r\n",
		     (unsigned)samp_rate, channels, cfg.volume);
	}

	if (s_lock)
		rtos_unlock_mutex(&s_lock);
	return ret;
}

bk_err_t app_audio_write(const void *pcm, uint32_t len)
{
	if (!s_open)
		return BK_ERR_NOT_INIT;
	return bsp_audio_write(pcm, len);
}

bk_err_t app_audio_pause(void)
{
	if (!s_open)
		return BK_ERR_NOT_INIT;
	return bsp_audio_pause();
}

bk_err_t app_audio_resume(void)
{
	if (!s_open)
		return BK_ERR_NOT_INIT;
	return bsp_audio_resume();
}

void app_audio_close(void)
{
	audio_lock_init();
	if (s_lock)
		rtos_lock_mutex(&s_lock);

	if (s_open) {
		bsp_audio_stop();
		s_open = false;
		LOGI("audio closed\r\n");
		audio_cpu_vote(false);   /* 无音频时降回默认频率省电 */
		audio_sleep_vote(true);  /* 放开休眠票，空闲时允许 PM 降电压 */
	}

	if (s_lock)
		rtos_unlock_mutex(&s_lock);
}

bool app_audio_is_open(void)
{
	return s_open;
}

uint32_t app_audio_rate(void)
{
	return s_rate;
}

bk_err_t app_audio_set_volume(uint8_t volume)
{
	if (volume > 100)
		volume = 100;

	app_state_set_volume(volume);

	if (s_open)
		return bsp_audio_set_volume(volume);

	return BK_OK;
}

uint8_t app_audio_get_volume(void)
{
	return app_state_volume();
}
