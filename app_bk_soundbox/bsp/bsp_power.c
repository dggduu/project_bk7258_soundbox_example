// Copyright 2026
// 板级电源控制实现

#include "bsp_power.h"
#include "bsp_board_cfg.h"

#include <driver/gpio.h>
#include <driver/pwr_clk.h>       /* bk_pm_module_vote_ctrl_external_ldo() */
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
 * 同时使能输入，方便回读 pad 实际电平（bk_gpio_get_input 要求 input_en）。
 *
 * 这里是**推挽**驱动，适用于源极电压低于 VDDIO(3.3V) 的 PMOS 轨道
 * （MIC 1V8）：输出高 3.3V 时 Vgs = 3.3 - 1.8 = +1.5V，可靠截止。
 * 若某条轨道的 PMOS 源极直接接 VBAT(最高 4.2V)，不能用推挽高关断
 * （Vgs 只有 -0.9V 左右），必须改成"释放成高阻 + 外部 VBAT 上拉"。
 * 本工程现有轨道都不属于这种情况（最高源极是 3V3）。 */
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

	/* MIC 电源轨已从 LDO 换成 PMOS，控制逻辑接入 SDK PM：
	 * 把最终电平（pad 实际输出）登记给 PM，睡眠期间 PM 维持该电平、
	 * 唤醒后自动恢复，避免睡眠把 PMOS 翻回导通造成 MIC 常供电漏电。 */
	if (rail == BSP_PWR_RAIL_MIC_1V8) {
		gpio_output_state_e pm_state = bk_gpio_get_input(s_pwr[rail].pin)
		                               ? GPIO_OUTPUT_STATE_HIGH
		                               : GPIO_OUTPUT_STATE_LOW;

		bk_pm_module_vote_ctrl_external_ldo(BSP_PM_MODULE_MIC,
		                                    s_pwr[rail].pin, pm_state);
	}

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

	ret = bsp_power_enable(BSP_PWR_RAIL_CAM_1V5, true);
	if (ret != BK_OK)
		return ret;
	rtos_delay_milliseconds(BOARD_PWR_SETTLE_MS);

	ret = bsp_power_enable(BSP_PWR_RAIL_CAM_2V8, true);
	if (ret != BK_OK)
		return ret;
	rtos_delay_milliseconds(BOARD_CAM_PWR_SETTLE_MS);

	return BK_OK;
}

bk_err_t bsp_power_camera_off(void)
{
	bsp_power_enable(BSP_PWR_RAIL_CAM_2V8, false);
	rtos_delay_milliseconds(5);
	bsp_power_enable(BSP_PWR_RAIL_CAM_1V5, false);
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
