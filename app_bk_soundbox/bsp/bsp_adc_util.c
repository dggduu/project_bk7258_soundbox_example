// Copyright 2026
// ADC 公共读取封装实现

#include "bsp_adc_util.h"

#include <driver/gpio.h>
#include "gpio_driver.h"      /* gpio_dev_map() */

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#define TAG "bsp_adc"
#define BSP_LOGD(...) BK_LOGD(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define BSP_ADC_TIMEOUT_MS 200

static beken_mutex_t s_adc_mutex = NULL;

/* 本工程用到的 ADC 通道 -> 引脚映射（BK7258，见 gpio_map.h 的 func 表）：
 *   ADC_1=P25  ADC_2=P24  ADC_3=P23  ADC_4=P28  ADC_5=P22  ADC_6=P21
 * 只登记实际用到的两个通道；未登记的通道回退到 SDK 的
 * bk_adc_chan_init_gpio()（走 IPC 到 CP）。
 *
 * 为什么要自己动手而不是直接调 SDK 的 bk_adc_chan_init_gpio()：
 *   ① 那个接口走 IPC，每次读都调有一次跨核往返；
 *   ② 它内部对"已经是模拟通道"的通道会直接 return（不切引脚）；
 *   ③ AP 侧的 gpio_dev_map() 已经把该做的都做了 —— 关输出/输入/上下拉/
 *      中断 + 写 func_sel，与 CP 侧 adc_init_gpio() 的 unmap+map 等价。
 * 前提：这两个引脚在本工程里只给 ADC 用，不会被别的模块抢；
 * 如果以后要复用给别的外设，需改回 SDK 路径（CP 那边才有占用登记）。 */
static const struct {
	adc_chan_t chan;
	gpio_id_t  pin;
	gpio_dev_t dev;
} s_adc_pins[] = {
	{ ADC_4, GPIO_28, GPIO_DEV_ADC4 },   /* 按键 AD 键盘 */
	{ ADC_5, GPIO_22, GPIO_DEV_ADC5 },   /* 电池分压 */
};

static bool adc_pin_map_direct(adc_chan_t chan)
{
	for (size_t i = 0; i < sizeof(s_adc_pins) / sizeof(s_adc_pins[0]); i++) {
		if (s_adc_pins[i].chan != chan)
			continue;

		/* 引脚默认是 LCD 第二功能（P22=LCD_R4 / P28=I2S1_MCLK），
		 * gpio_dev_map 会覆盖 func_sel，不需要先 unmap。 */
		return gpio_dev_map(s_adc_pins[i].pin, s_adc_pins[i].dev) == BK_OK;
	}

	return false;
}

static bk_err_t adc_lock_init(void)
{
	if (s_adc_mutex)
		return BK_OK;

	return rtos_init_mutex(&s_adc_mutex);
}

static void adc_config_fill(adc_config_t *cfg, adc_chan_t chan)
{
	os_memset(cfg, 0, sizeof(*cfg));
	cfg->chan = chan;
	cfg->adc_mode = ADC_CONTINUOUS_MODE;
	cfg->src_clk = ADC_SCLK_XTAL_26M;
	cfg->clk = 0x30A0C5;
	cfg->saturate_mode = ADC_SATURATE_MODE_3;
	cfg->steady_ctrl = 7;
	cfg->adc_filter = 0;
	cfg->sample_rate = 0;
	cfg->is_open = 0;
	cfg->vol_div = ADC_VOL_DIV_NONE;
}

/* 公共采样核心：把 acquire/init/config/start/read/deinit/release 全做完，
 * 由调用者决定怎么换算（SDK mV 或 raw 码值）。
 *
 * want_raw = false -> sum_mv 累加 bk_adc_data_calculate()（SDK 3300mV 口径）
 * want_raw = true  -> sum_raw 累加原始码值（0..8191） */
static bk_err_t adc_sample(adc_chan_t chan, uint32_t sample_num,
                           uint32_t *sum_mv, uint32_t *sum_raw, uint32_t *n_out)
{
	bk_err_t ret;
	adc_config_t cfg;
	uint32_t n = 0;

	ret = adc_lock_init();
	if (ret != BK_OK)
		return ret;

	rtos_lock_mutex(&s_adc_mutex);

	ret = bk_adc_acquire();
	if (ret != BK_OK) {
		BSP_LOGE("acquire failed: %d\r\n", ret);
		goto out_unlock;
	}

	/* 每次读之前重新把引脚切到 ADC 功能。
	 *
	 * 不能只做一次：P22/P28 的默认配置都是
	 * `GPIO_SECOND_FUNC_ENABLE + GPIO_DEV_LCD_Rx / I2S1_MCLK`（见 gpio_map.h），
	 * 屏幕/音频初始化会把它们占走。每次重新 map 只是一次寄存器写，
	 * 比走 IPC 的 bk_adc_chan_init_gpio() 便宜得多。 */
	if (!adc_pin_map_direct(chan)) {
		BSP_LOGD("ch%d not in local map, fallback to SDK\r\n", chan);
		bk_adc_chan_init_gpio(chan);
	}

	ret = bk_adc_init(chan);
	if (ret != BK_OK) {
		BSP_LOGE("init ch%d failed: %d\r\n", chan, ret);
		goto out_release;
	}

	adc_config_fill(&cfg, chan);
	ret = bk_adc_set_config(&cfg);
	if (ret != BK_OK) {
		BSP_LOGE("set config ch%d failed: %d\r\n", chan, ret);
		goto out_deinit;
	}

	bk_adc_enable_bypass_clalibration();

	ret = bk_adc_start();
	if (ret != BK_OK) {
		BSP_LOGE("start ch%d failed: %d\r\n", chan, ret);
		goto out_deinit;
	}

	for (n = 0; n < sample_num; n++) {
		uint16_t raw = 0;

		ret = bk_adc_read(&raw, BSP_ADC_TIMEOUT_MS);
		if (ret != BK_OK) {
			BSP_LOGE("read ch%d failed: %d\r\n", chan, ret);
			break;
		}
		if (sum_raw)
			*sum_raw += raw;
		if (sum_mv)
			*sum_mv += bk_adc_data_calculate(raw, chan);
	}

	bk_adc_stop();

	if (n == 0) {
		goto out_deinit;
	}

	*n_out = n;
	ret = BK_OK;

out_deinit:
	bk_adc_deinit(chan);
out_release:
	bk_adc_release();
out_unlock:
	rtos_unlock_mutex(&s_adc_mutex);
	return ret;
}

bk_err_t bsp_adc_read_mv(adc_chan_t chan, uint32_t sample_num, uint32_t *mv_out)
{
	uint32_t sum = 0;
	uint32_t n = 0;
	bk_err_t ret;

	if (!mv_out || sample_num == 0)
		return BK_ERR_NULL_PARAM;

	ret = adc_sample(chan, sample_num, &sum, NULL, &n);
	if (ret != BK_OK)
		return ret;

	*mv_out = sum / n;
	BSP_LOGD("ch%d avg %dmV (%u samples)\r\n", chan, *mv_out, (unsigned)n);
	return BK_OK;
}

bk_err_t bsp_adc_read_raw(adc_chan_t chan, uint32_t sample_num, uint32_t *raw_out)
{
	uint32_t sum = 0;
	uint32_t n = 0;
	bk_err_t ret;

	if (!raw_out || sample_num == 0)
		return BK_ERR_NULL_PARAM;

	ret = adc_sample(chan, sample_num, NULL, &sum, &n);
	if (ret != BK_OK)
		return ret;

	*raw_out = sum / n;
	BSP_LOGD("ch%d avg raw=%u (%u samples)\r\n", chan, (unsigned)*raw_out,
	         (unsigned)n);
	return BK_OK;
}

bk_err_t bsp_adc_read_raw_mv(adc_chan_t chan, uint32_t sample_num,
                             uint32_t *raw_out, uint32_t *mv_out)
{
	uint32_t sum_raw = 0;
	uint32_t sum_mv = 0;
	uint32_t n = 0;
	bk_err_t ret;

	if ((!raw_out && !mv_out) || sample_num == 0)
		return BK_ERR_NULL_PARAM;

	/* 两个累加器同时打开：adc_sample 在一次采集循环里既累加 raw
	 * 也累加 SDK 换算的 mV，raw 和 mV 一定来自同一批采样。 */
	ret = adc_sample(chan, sample_num,
	                 mv_out ? &sum_mv : NULL,
	                 raw_out ? &sum_raw : NULL, &n);
	if (ret != BK_OK)
		return ret;

	if (raw_out)
		*raw_out = sum_raw / n;
	if (mv_out)
		*mv_out = sum_mv / n;

	BSP_LOGD("ch%d avg raw=%u %dmV (%u samples)\r\n", chan,
	         (unsigned)(raw_out ? *raw_out : 0),
	         (unsigned)(mv_out ? *mv_out : 0), (unsigned)n);
	return BK_OK;
}
