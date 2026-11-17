// Copyright 2026
// CherrySH 命令：ldo —— 板上几路电源使能（LDO_EN）测试
//
//   ldo                        列出所有电源轨和当前状态
//   ldo <name>                 查询单路
//   ldo <name> on|off|toggle   控制单路
//   ldo all on|off             同时控制所有
//
//   gpio <pin>                 读任意 GPIO 的 pad 电平（调试用）
//   gpio <pin> 0|1             把任意 GPIO 配成输出并驱动
//   gpio <pin> z               释放成输入（高阻）
//
// 引脚来自 app_bk_pi 的原理图网表（bsp_board_cfg.h）：
//   scr = P51 屏幕 3V3   （PMOS Q5，低有效）
//   2v8 = P50 摄像头 2V8 （LDO U10，高有效）
//   1v5 = P26 摄像头 1V5 （LDO U9，高有效）
//   mic = P48 MIC 1V8    （LDO U8，高有效）
//   pa  = P52 功放 EN    （NS4150B CTRL，高有效）
//
// 命令注册在 ap/csh_port.c 的 s_cmd_table[] 里（不走链接段自动注册，
// 原因见那里的注释）。

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"
#include "gpio_driver.h"     /* gpio_dev_unmap() */

typedef struct {
	const char *name;
	bsp_pwr_rail_t rail;
	gpio_id_t pin;
	bool active_low;
	const char *desc;
} ldo_rail_t;

static const ldo_rail_t s_rails[] = {
	{ "scr", BSP_PWR_RAIL_SCR,     BOARD_PWR_SCR_EN_PIN, BOARD_PWR_SCR_EN_ACTIVE_LOW, "screen 3V3 (PMOS)" },
	{ "2v8", BSP_PWR_RAIL_CAM_2V8, BOARD_PWR_2V8_EN_PIN, BOARD_PWR_2V8_EN_ACTIVE_LOW, "camera 2V8 (LDO)"  },
	{ "1v5", BSP_PWR_RAIL_CAM_1V5, BOARD_PWR_1V5_EN_PIN, BOARD_PWR_1V5_EN_ACTIVE_LOW, "camera 1V5 (LDO)"  },
	{ "mic", BSP_PWR_RAIL_MIC_1V8, BOARD_PWR_MIC_EN_PIN, BOARD_PWR_MIC_EN_ACTIVE_LOW, "mic 1V8 (LDO)"     },
	{ "pa",  BSP_PWR_RAIL_PA,      BOARD_PA_EN_PIN,      BOARD_PA_EN_ACTIVE_LOW,      "pa enable"         },
};

#define LDO_RAIL_CNT (sizeof(s_rails) / sizeof(s_rails[0]))

static void ldo_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  ldo                        list all rails\r\n");
	csh_printf(csh, "  ldo <name>                 show one rail\r\n");
	csh_printf(csh, "  ldo <name> on|off|toggle   control one rail\r\n");
	csh_printf(csh, "  ldo all on|off             control all rails\r\n");
	csh_printf(csh, "name:");
	for (uint32_t i = 0; i < LDO_RAIL_CNT; i++)
		csh_printf(csh, " %s", s_rails[i].name);
	csh_printf(csh, "\r\n");
}

static void ldo_show(chry_shell_t *csh, const ldo_rail_t *r)
{
	csh_printf(csh, "  %-4s  P%-2d  %-7s  %-3s  pad=%d  %s\r\n",
	           r->name, (int)r->pin, r->active_low ? "low_on" : "high_on",
	           bsp_power_is_enabled(r->rail) ? "ON" : "off",
	           bsp_power_get_pad(r->rail), r->desc);
}

static int ldo_parse_level(const char *arg, bool *on)
{
	if (!strcmp(arg, "on") || !strcmp(arg, "1")) {
		*on = true;
		return 0;
	}
	if (!strcmp(arg, "off") || !strcmp(arg, "0")) {
		*on = false;
		return 0;
	}
	return -1;
}

static const ldo_rail_t *ldo_find(const char *name)
{
	for (uint32_t i = 0; i < LDO_RAIL_CNT; i++) {
		if (!strcmp(name, s_rails[i].name))
			return &s_rails[i];
	}
	return NULL;
}

static const char *ldo_state(bsp_pwr_rail_t rail)
{
	return bsp_power_is_enabled(rail) ? "ON" : "off";
}

int ldo_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];

	/* ldo */
	if (argc == 1) {
		csh_printf(csh, "name  pin   polarity  state  pad  desc\r\n");
		for (uint32_t i = 0; i < LDO_RAIL_CNT; i++)
			ldo_show(csh, &s_rails[i]);
		return 0;
	}

	const char *name = argv[1];

	/* ldo all on|off */
	if (!strcmp(name, "all")) {
		bool on;

		if ((argc < 3) || (ldo_parse_level(argv[2], &on) != 0)) {
			ldo_usage(csh);
			return -1;
		}

		for (uint32_t i = 0; i < LDO_RAIL_CNT; i++) {
			bsp_power_enable(s_rails[i].rail, on);
			ldo_show(csh, &s_rails[i]);
		}
		return 0;
	}

	const ldo_rail_t *r = ldo_find(name);
	if (r == NULL) {
		csh_printf(csh, "unknown rail '%s'\r\n", name);
		ldo_usage(csh);
		return -1;
	}

	/* ldo <name> */
	if (argc < 3) {
		ldo_show(csh, r);
		return 0;
	}

	bool on;

	if (!strcmp(argv[2], "toggle")) {
		on = !bsp_power_is_enabled(r->rail);
	} else if (ldo_parse_level(argv[2], &on) != 0) {
		ldo_usage(csh);
		return -1;
	}

	bsp_power_enable(r->rail, on);
	csh_printf(csh, "%s -> %s\r\n", r->name, ldo_state(r->rail));
	ldo_show(csh, r);
	return 0;
}

/* 板级信息：把几路电源使能脚和主 clock 打印出来，方便对着原理图测 */
int board_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];

	(void)argc;
	csh_printf(csh, "BK7258 (app_bk_pi board)\r\n");
	csh_printf(csh, "  shell uart : UART1 TX=P0 RX=P1 115200 8N1\r\n");
	csh_printf(csh, "  log uart   : UART0 TX=P11 RX=P10 (via CP)\r\n");
	csh_printf(csh, "  power en   : scr=P51 2v8=P50 1v5=P26 mic=P48 pa=P52\r\n");
	csh_printf(csh, "  ldo state  :\r\n");
	for (uint32_t i = 0; i < LDO_RAIL_CNT; i++)
		ldo_show(csh, &s_rails[i]);
	return 0;
}

/* 通用 GPIO 调试命令：怀疑某一路 EN 不是配置里的脚时，可以直接拿这个试任意脚。
 *
 *   gpio <pin>        读 pad 电平
 *   gpio <pin> 0|1    配成推挽输出并驱动
 *   gpio <pin> z      释放成输入（高阻）
 *
 * 注意：会强行解复用并驱动该脚，只适合在测试板上用。 */
int gpio_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	int pin;

	if (argc < 2) {
		csh_printf(csh, "usage:\r\n");
		csh_printf(csh, "  gpio <pin>        read pad\r\n");
		csh_printf(csh, "  gpio <pin> 0|1    drive output\r\n");
		csh_printf(csh, "  gpio <pin> z      release (input/hiz)\r\n");
		return -1;
	}

	pin = atoi(argv[1]);
	if ((pin < 0) || (pin >= (int)GPIO_NUM)) {
		csh_printf(csh, "invalid pin %d\r\n", pin);
		return -1;
	}

	if (argc < 3) {
		bk_gpio_enable_input((gpio_id_t)pin);
		csh_printf(csh, "GPIO%d pad=%d\r\n", pin, bk_gpio_get_input((gpio_id_t)pin));
		return 0;
	}

	if (!strcmp(argv[2], "z")) {
		bk_gpio_disable_output((gpio_id_t)pin);
		bk_gpio_disable_pull((gpio_id_t)pin);
		bk_gpio_enable_input((gpio_id_t)pin);
		csh_printf(csh, "GPIO%d -> input/hiz\r\n", pin);
		return 0;
	}

	{
		int v = atoi(argv[2]) ? 1 : 0;

		gpio_dev_unmap((gpio_id_t)pin);
		bk_gpio_disable_pull((gpio_id_t)pin);
		bk_gpio_enable_input((gpio_id_t)pin);
		/* 不能用 bk_gpio_set_value()：整字写会把输出使能清掉 */
		if (v)
			bk_gpio_set_output_high((gpio_id_t)pin);
		else
			bk_gpio_set_output_low((gpio_id_t)pin);
		bk_gpio_enable_output((gpio_id_t)pin);

		csh_printf(csh, "GPIO%d -> %d, pad=%d\r\n", pin, v,
		           bk_gpio_get_input((gpio_id_t)pin));
	}
	return 0;
}
