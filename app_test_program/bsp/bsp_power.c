// Copyright 2026
// 板级电源控制实现

#include "bsp_power.h"
#include "bsp_board_cfg.h"

#include <driver/gpio.h>
#include "gpio_driver.h"      /* gpio_dev_unmap() */
#include <os/os.h>
#include <components/log.h>

#define TAG "bsp_power"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

typedef struct {
	gpio_id_t pin;
	uint8_t active_low;
	bool enabled;
} bsp_pwr_ctrl_t;

static bsp_pwr_ctrl_t s_pwr[BSP_PWR_RAIL_MAX] = {
	[BSP_PWR_RAIL_SCR]     = { BOARD_PWR_SCR_EN_PIN,     BOARD_PWR_SCR_EN_ACTIVE_LOW,     false },
	[BSP_PWR_RAIL_CAM_2V8] = { BOARD_PWR_2V8_EN_PIN,     BOARD_PWR_2V8_EN_ACTIVE_LOW,     false },
	[BSP_PWR_RAIL_CAM_1V5] = { BOARD_PWR_1V5_EN_PIN,     BOARD_PWR_1V5_EN_ACTIVE_LOW,     false },
	[BSP_PWR_RAIL_MIC_1V8] = { BOARD_PWR_MIC_EN_PIN,     BOARD_PWR_MIC_EN_ACTIVE_LOW,     false },
	[BSP_PWR_RAIL_PA]      = { BOARD_PA_EN_PIN,          BOARD_PA_EN_ACTIVE_LOW,          false },
};

static void pwr_write(bsp_pwr_ctrl_t *c, bool on)
{
	uint32_t level = on ? 1 : 0;

	if (c->active_low)
		level = !level;

	/* 注意：不能用 bk_gpio_set_value()，那是整字写配置寄存器
	 * （bit0 输入值 / bit1 输出值 / bit3 输出使能 ...），会把输出使能一起清掉，
	 * 引脚根本不会动。这里只用 bit1 的 set_output_high/low。 */
	if (level)
		bk_gpio_set_output_high(c->pin);
	else
		bk_gpio_set_output_low(c->pin);
	c->enabled = on;
}

/* 每次操作前重新确认引脚是 GPIO 输出。
 * 低电压睡眠唤醒、或别的模块误把这个脚复用走之后，只写输出寄存器是没用的
 * （pad 由复用功能驱动），所以这里反复解复用 + 使能输出。
 * 同时使能输入，方便回读 pad 实际电平（bk_gpio_get_input 要求 input_en）。 */
static void pwr_pin_config(bsp_pwr_ctrl_t *c)
{
	gpio_dev_unmap(c->pin);
	bk_gpio_disable_pull(c->pin);
	bk_gpio_enable_output(c->pin);
	bk_gpio_enable_input(c->pin);
}

bk_err_t bsp_power_init(void)
{
	bk_err_t ret = BK_OK;

	for (int i = 0; i < BSP_PWR_RAIL_MAX; i++) {
		bsp_pwr_ctrl_t *c = &s_pwr[i];

		/* 先输出无效电平再切输出，避免上电毛刺；使能输出后再写一次确保生效 */
		pwr_write(c, false);
		pwr_pin_config(c);
		ret = bk_gpio_enable_output(c->pin);
		if (ret != BK_OK) {
			BSP_LOGE("rail %d gpio %d output enable failed: %d\r\n", i, c->pin, ret);
			return ret;
		}
		pwr_write(c, false);

		BSP_LOGI("rail %d gpio %d init, pad=%d\r\n", i, c->pin, bk_gpio_get_input(c->pin));
	}

	BSP_LOGI("power rails init done, all off\r\n");
	return BK_OK;
}

bk_err_t bsp_power_enable(bsp_pwr_rail_t rail, bool on)
{
	if (rail >= BSP_PWR_RAIL_MAX)
		return BK_ERR_PARAM;

	pwr_pin_config(&s_pwr[rail]);      /* 防止引脚被复用走/睡眠后丢失配置 */
	pwr_write(&s_pwr[rail], on);

	BSP_LOGI("rail %d gpio %d -> %s, pad=%d\r\n",
	         rail, s_pwr[rail].pin, on ? "on" : "off",
	         bk_gpio_get_input(s_pwr[rail].pin));
	return BK_OK;
}

/* pad 实际电平（1/0），供 shell 命令显示 */
int bsp_power_get_pad(bsp_pwr_rail_t rail)
{
	if (rail >= BSP_PWR_RAIL_MAX)
		return -1;

	return bk_gpio_get_input(s_pwr[rail].pin) ? 1 : 0;
}

bool bsp_power_is_enabled(bsp_pwr_rail_t rail)
{
	if (rail >= BSP_PWR_RAIL_MAX)
		return false;

	return s_pwr[rail].enabled;
}

bk_err_t bsp_power_camera_on(void)
{
	bk_err_t ret;

	/* OV3660 datasheet power-up 时序：DOVDD/AVDD(2V8) 先于 DVDD(1.5V)。
	 * 之前 1V5->2V8 的顺序会让 sensor 在 DVDD 先加电时进入未知状态，
	 * 表现为 SCCB 无应答；且只有真正断电重上才会触发（带电状态下
	 * probe 只是重拉 EN，电源没掉，掩盖了顺序错误）。 */
	ret = bsp_power_enable(BSP_PWR_RAIL_CAM_2V8, true);
	if (ret != BK_OK)
		return ret;
	rtos_delay_milliseconds(BOARD_PWR_SETTLE_MS);

	ret = bsp_power_enable(BSP_PWR_RAIL_CAM_1V5, true);
	if (ret != BK_OK)
		return ret;
	rtos_delay_milliseconds(BOARD_CAM_PWR_SETTLE_MS);

	return BK_OK;
}

bk_err_t bsp_power_camera_off(void)
{
	/* 与上电顺序相反：DVDD(1V5) 先断 -> AVDD/DOVDD(2V8) 后断。
	 * 断电后要等足够久（电容放电），否则马上重上电时 sensor
	 * 内部状态没复位，'power cycle 复活' 就不成立。 */
	bsp_power_enable(BSP_PWR_RAIL_CAM_1V5, false);
	rtos_delay_milliseconds(5);
	bsp_power_enable(BSP_PWR_RAIL_CAM_2V8, false);
	rtos_delay_milliseconds(BOARD_CAM_PWR_OFF_MS);
	return BK_OK;
}

bk_err_t bsp_power_screen_on(void)
{
	bk_err_t ret = bsp_power_enable(BSP_PWR_RAIL_SCR, true);

	if (ret == BK_OK)
		rtos_delay_milliseconds(BOARD_SCR_PWR_SETTLE_MS);

	return ret;
}

bk_err_t bsp_power_screen_off(void)
{
	return bsp_power_enable(BSP_PWR_RAIL_SCR, false);
}

bk_err_t bsp_power_screen_reset(void)
{
	bk_err_t ret = bsp_power_screen_off();

	rtos_delay_milliseconds(BOARD_SCR_PWR_SETTLE_MS);
	if (ret != BK_OK)
		return ret;

	return bsp_power_screen_on();
}
