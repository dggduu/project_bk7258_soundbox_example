// Copyright 2026
// CherrySH 命令：lcdq —— QSPI 屏（ST77916 360x360）调试
//
// 为什么用 bk_lcd_qspi_* 而不是 bk_display_*：
//   bk_display 会另起显示任务、投票 CPU 480M / PSRAM，适合产品上层；
//   调试命令直接用 driver 层公开 API 更可控，且 app_bk_pi 的旧版 QSPI 屏
//   通路（见 bsp_screen.c 顶部注释）本来就是 bk_lcd_qspi_init() +
//   bk_lcd_qspi_frame_display()。
//
// 硬件（bsp_board_cfg.h / qspi_ll.h）：
//   QSPI1 : P2=CLK P3=CSN P4=IO0 P5=IO1 P6=IO2 P7=IO3
//           CONFIG_QSPI_LINE_MODE=4，bk_qspi_init() 内部会自己映射这 6 根脚，
//           usr_gpio_cfg.h 里 P2~P7 故意留 GPIO_DEV_INVALID（表里释放）。
//   屏电源: P51 PWR_SCR_EN，低有效 PMOS Q5。屏 RST 与模组 CEN 同网络，
//           软件不能单独拉 → bk_lcd_qspi_init() 的 reset_pin 传 GPIO_NUM
//           跳过硬复位，靠 bsp_power_screen_reset()（断电 30ms 再上电）复位。
//   背光  : P23 GPIO 直连，高有效（与 ESP32 参考板的低有效不同）。
//
// 三套初始化表（单变量 A/B，实测哪张能点亮就用哪张）：
//   boe   : 换屏后 BOE 1.8" 360x360 圆屏厂商例程
//           ref/TFT-BOE1.8-ST77916-...-zch/BOE1.8-ST77916-初始化.txt，
//           192 项原样回放，见 lcdq_init_boe.h —— 当前默认
//   esp32 : 上一块屏的 ESP32 参考驱动 bsp_lcd.c vendor_specific_init_yysj
//           （185 项）+ 补 0x36/0x3A 共 187 项，见 lcdq_init_esp32.h
//   sdk   : SDK 自带 ap/components/bk_peripheral/src/lcd/qspi/
//           lcd_qspi_st77916.c（需 CONFIG_LCD_QSPI_ST77916=y，本工程已打开）
//   三者同为 ST77916 360x360 QSPI，但寄存器数值差异很大，对应不同玻璃/模组厂。
//   驱动有 lcd_qspi_is_init 保护，换表必须先 'lcdq deinit' 再 'lcdq init'。
//
// BOE 表要点（与另两张表的区别）：
//   - 厂商表自带 COLMOD 0x3A=0x55，但位置在 0x29(Display On) 之后；
//     照厂商顺序原样回放，不提前、不重复下发。
//   - 没有 0x36 MADCTL（上电默认 0x00），故不注入。
//   - 0x11(Sleep Out) 之后有 120ms 延时，用 SDK 的 {0x00,{120},0xFF} 表示。
//   - 像素写命令字、RGB565 大端字节序与 SDK pixel_write_config 完全一致，
//     所以 driver 层参数不用改，只是 init_cmd 换表。
//
// 为什么 esp32 表要补 0x3A：ESP32 的 esp_lcd_st77916 组件会根据
// esp_lcd_panel_dev_config_t 的 .bits_per_pixel=16 在组件内部下发 COLMOD，
// vendor 表里没有 0x3A；而 BK7258 的 bk_lcd_qspi_init() 只回放 init_cmd，
// 不会自己设像素格式。缺 0x3A=0x55 时屏幕保持默认（通常 18bit/RGB666），
// 360*360*2 字节的 RGB565 帧只够填 2/3 GRAM —— 实测现象就是上面 2/5
// 花屏、下面 3/5 有色条但颜色不对（发灰）。0x36/0x3A 取自 SDK 自带
// st77916 表，插在 0x11(Sleep Out) 之前，由 tools/gen_lcdq_init.py 注入。
//
// 命令：
//   lcdq                          状态
//   lcdq init [boe|esp32|sdk]     断电复位 + 上电 + 初始化（默认沿用当前表）
//   lcdq deinit                   关背光 / 断电 / 反初始化 / 释放帧缓冲
//   lcdq profile [boe|esp32|sdk]      查看或切换初始化表（切换后需重新 init）
//   lcdq power on|off             仅屏 3V3（P51）
//   lcdq bl on|off                仅背光（P23）
//   lcdq fill <color>             整屏纯色：0xXXXX=RGB565，0xRRGGBB 自动转 RGB565
//   lcdq bars                     8 色条（照 ESP32 bsp_lcd_rgb_test）
//   lcdq cmd <reg> [hex ...]      发一条 QSPI 寄存器写命令（0x02 前缀由驱动加）
//   lcdq read <reg> [len]         读 QSPI 寄存器（4 线读，dummy=0，仅诊断）
//   lcdq help
//
// 命令注册在 ap/csh_port.c 的 s_cmd_table[]（不走链接段自动注册）。

#include <string.h>

#include "cmd_lcdq.h"
#include <stdlib.h>
#include <stdbool.h>
#include <stdint.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"
#include "gpio_driver.h"          /* gpio_dev_unmap() */

#include <driver/gpio.h>
#include <driver/lcd_qspi.h>
#include <driver/lcd_types.h>
#include <driver/lcd_qspi_types.h>
#include <driver/hal/hal_qspi_types.h>
#include <components/media_types.h>
#include <os/os.h>
#include <os/mem.h>

#include "lcdq_init_esp32.h"
#include "lcdq_init_boe.h"

/* SDK 自带 ST77916 设备：CONFIG_LCD_QSPI_ST77916=y 时由
 * ap/components/bk_peripheral/src/lcd/qspi/lcd_qspi_st77916.c 提供，
 * 并被 lcd_panel_devices.c 的全局设备表引用，符号一定在镜像里。 */
extern const lcd_device_t lcd_device_st77916;

#define LCDQ_W              360
#define LCDQ_H              360
#define LCDQ_FRAME_BYTES    (LCDQ_W * LCDQ_H * 2)
#define LCDQ_QSPI_ID        QSPI_ID_1
#define LCDQ_RESET_PIN      GPIO_NUM      /* 跳过硬复位，本板 RST 接 CEN */
#define LCDQ_CMD_MAX        16

enum {
	LCDQ_PROF_BOE   = 0,    /* BOE 1.8" 厂商例程（当前在用，默认） */
	LCDQ_PROF_ESP32 = 1,
	LCDQ_PROF_SDK   = 2,
};

/* QSPI 4 线写像素的命令字（0x2C = RAMWR）。与 ESP32 版
 * lcd_cmd = cmd<<8 | 0x02<<24 中的 0x02 是同一个东西：
 * SDK 的 reg_write_cmd=0x02 由 bk_lcd_qspi_send_cmd() 拼进命令字。 */
static uint8_t s_pixel_cmd[4] = {0x32, 0x00, 0x2C, 0x00};

static lcd_qspi_t s_esp32_qspi = {
	.clk                 = LCD_QSPI_16M,
	.refresh_method      = LCD_QSPI_REFRESH_BY_FRAME,
	.reg_write_cmd       = 0x02,
	.reg_read_cmd        = 0x0B,
	.reg_read_config     = {0, LCD_QSPI_NO_INSERT_DUMMMY_CLK},
	.pixel_write_config  = {s_pixel_cmd, sizeof(s_pixel_cmd)},
	.init_cmd            = s_esp32_init_cmds,
	.device_init_cmd_len = LCDQ_ESP32_INIT_CMD_CNT,
	.refresh_config      = {0},
	.frame_len           = LCDQ_FRAME_BYTES,
};

static const lcd_device_t s_esp32_dev = {
	.id      = LCD_DEVICE_ST77916,
	.name    = "st77916_esp32",
	.type    = LCD_TYPE_QSPI,
	.width   = LCDQ_W,
	.height  = LCDQ_H,
	.src_fmt = PIXEL_FMT_RGB565,
	.out_fmt = PIXEL_FMT_RGB565,
	.qspi    = &s_esp32_qspi,
	.init    = NULL,
	.off     = NULL,
};

/* BOE 1.8" 360x360 圆屏（厂商例程 TFT-BOE1.8-ST77916-...-zch）。
 * 与 ESP32 表共用同一套 QSPI 时序参数：像素写命令字 0x32/0x00/0x2C/0x00、
 * reg_write_cmd=0x02、reg_read_cmd=0x0B、48MHz、BY_FRAME，只有 init_cmd 不同。
 * 厂商表是逐条 SPI_WriteComm/WriteData dump，已 1:1 转成 192 项
 * （含 0x11 后 120ms 延时，以及末尾 0x29 之后的 0x3A=0x55）。 */
static lcd_qspi_t s_boe_qspi = {
	.clk                 = LCD_QSPI_16M,
	.refresh_method      = LCD_QSPI_REFRESH_BY_FRAME,
	.reg_write_cmd       = 0x02,
	.reg_read_cmd        = 0x0B,
	.reg_read_config     = {0, LCD_QSPI_NO_INSERT_DUMMMY_CLK},
	.pixel_write_config  = {s_pixel_cmd, sizeof(s_pixel_cmd)},
	.init_cmd            = s_boe_init_cmds,
	.device_init_cmd_len = LCDQ_BOE_INIT_CMD_CNT,
	.refresh_config      = {0},
	.frame_len           = LCDQ_FRAME_BYTES,
};

static const lcd_device_t s_boe_dev = {
	.id      = LCD_DEVICE_ST77916,
	.name    = "st77916_boe",
	.type    = LCD_TYPE_QSPI,
	.width   = LCDQ_W,
	.height  = LCDQ_H,
	.src_fmt = PIXEL_FMT_RGB565,
	.out_fmt = PIXEL_FMT_RGB565,
	.qspi    = &s_boe_qspi,
	.init    = NULL,
	.off     = NULL,
};

static const lcd_device_t *s_dev;
static int      s_profile = LCDQ_PROF_BOE;
/* 本板实测只有 16MHz 稳定（排线无阻抗控制，48M 下 DMA 能完成但屏不认数据）。
 * 需要提速可用 'lcdq clk' 临时改，但会黑屏/花屏。 */
static lcd_qspi_clk_t s_clk = LCD_QSPI_16M;
static bool     s_inited;
static bool     s_bl_inited;
static bool     s_bl_on;
static uint16_t *s_frame;

/* ---------------- 小工具 ---------------- */

static const char *lcdq_prof_name(int p)
{
	switch (p) {
	case LCDQ_PROF_BOE:   return "boe";
	case LCDQ_PROF_SDK:   return "sdk";
	default:              return "esp32";
	}
}

static const char *lcdq_prof_desc(int p)
{
	switch (p) {
	case LCDQ_PROF_BOE:
		return "BOE 1.8\" vendor table, 192 cmds (current panel)";
	case LCDQ_PROF_SDK:
		return "SDK lcd_qspi_st77916.c";
	default:
		return "ESP32 vendor table + 0x36/0x3A (187 cmds)";
	}
}

static int lcdq_parse_profile(const char *s)
{
	if (!strcmp(s, "boe"))
		return LCDQ_PROF_BOE;
	if (!strcmp(s, "esp32"))
		return LCDQ_PROF_ESP32;
	if (!strcmp(s, "sdk"))
		return LCDQ_PROF_SDK;
	return -1;
}

static const lcd_device_t *lcdq_device(int p)
{
	switch (p) {
	case LCDQ_PROF_BOE:   return &s_boe_dev;
	case LCDQ_PROF_SDK:   return &lcd_device_st77916;
	default:              return &s_esp32_dev;
	}
}

/* 把 s_clk 写进自建的两张表。SDK 的 lcd_device_st77916 是 const，改不了，
 * 所以 sdk profile 固定 48M（真要降速就用 boe / esp32 表）。 */
static void lcdq_apply_clk(void)
{
	s_boe_qspi.clk = s_clk;
	s_esp32_qspi.clk = s_clk;
}

static int lcdq_parse_on_off(const char *s, bool *on)
{
	if (!strcmp(s, "on") || !strcmp(s, "1")) {
		*on = true;
		return 0;
	}
	if (!strcmp(s, "off") || !strcmp(s, "0")) {
		*on = false;
		return 0;
	}
	return -1;
}

/* 本板背光 P23 是 GPIO 直连、高有效；ESP32 参考板是低有效，别照抄。 */
static void lcdq_bl_set(bool on)
{
	if (!s_bl_inited) {
		gpio_dev_unmap(BOARD_LCD_BACKLIGHT_PIN);
		bk_gpio_disable_pull(BOARD_LCD_BACKLIGHT_PIN);
		bk_gpio_enable_output(BOARD_LCD_BACKLIGHT_PIN);
		s_bl_inited = true;
	}

	if (on)
		bk_gpio_set_output_high(BOARD_LCD_BACKLIGHT_PIN);
	else
		bk_gpio_set_output_low(BOARD_LCD_BACKLIGHT_PIN);

	s_bl_on = on;
}

/* 帧缓冲按大端填 RGB565：官方 qspi_lcd_example 的
 * lcd_qspi_display_fill_pure_color() 就是 data[0]=color>>8, data[1]=color，
 * 直接写 uint16_t 数组在小端 ARM 上会把字节序反过来。 */
static void lcdq_frame_fill(uint16_t color)
{
	uint8_t *p = (uint8_t *)s_frame;
	uint8_t hi = (uint8_t)(color >> 8);
	uint8_t lo = (uint8_t)(color & 0xFF);
	uint32_t i, n = LCDQ_W * LCDQ_H;

	for (i = 0; i < n; i++) {
		p[2 * i]     = hi;
		p[2 * i + 1] = lo;
	}
}

static void lcdq_frame_bars(void)
{
	static const uint16_t colors[] = {
		0xF800, 0x07E0, 0x001F, 0xFFE0,
		0xF81F, 0x07FF, 0xFFFF, 0x0000,
	};
	uint8_t *p = (uint8_t *)s_frame;
	uint32_t rows = LCDQ_H / (sizeof(colors) / sizeof(colors[0]));
	uint32_t i, j;

	for (i = 0; i < sizeof(colors) / sizeof(colors[0]); i++) {
		uint8_t hi = (uint8_t)(colors[i] >> 8);
		uint8_t lo = (uint8_t)(colors[i] & 0xFF);
		uint32_t base = i * rows * LCDQ_W;

		for (j = 0; j < rows * LCDQ_W; j++) {
			p[2 * (base + j)]     = hi;
			p[2 * (base + j) + 1] = lo;
		}
	}
}

/* 0xXXXX 当 RGB565 直接用，0xRRGGBB 转成 RGB565。 */
static uint16_t lcdq_parse_color(const char *s)
{
	unsigned long v = strtoul(s, NULL, 0);

	if (v > 0xFFFFUL) {
		uint8_t r = (uint8_t)((v >> 16) & 0xFF);
		uint8_t g = (uint8_t)((v >> 8) & 0xFF);
		uint8_t b = (uint8_t)(v & 0xFF);

		return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
	}

	return (uint16_t)v;
}

/* ---------------- 初始化 / 反初始化 ---------------- */

static void lcdq_status(chry_shell_t *csh)
{
	csh_printf(csh, "QSPI LCD (ST77916 %dx%d RGB565)\r\n", LCDQ_W, LCDQ_H);
	csh_printf(csh, "  bus      : QSPI1 P2=CLK P3=CSN P4..P7=IO0..IO3, %dMHz, 4-line\r\n",
           (int)s_clk);
	csh_printf(csh, "  profile  : %-5s  %s\r\n",
	           lcdq_prof_name(s_profile), lcdq_prof_desc(s_profile));
	csh_printf(csh, "  power    : %-3s  P51 pad=%d\r\n",
	           bsp_power_is_enabled(BSP_PWR_RAIL_SCR) ? "ON" : "off",
	           bsp_power_get_pad(BSP_PWR_RAIL_SCR));
	csh_printf(csh, "  backlight: %-3s  P23 (high-active)\r\n", s_bl_on ? "ON" : "off");
	csh_printf(csh, "  inited   : %s\r\n", s_inited ? "yes" : "no");
	csh_printf(csh, "  frame    : %s  %u bytes\r\n",
	           s_frame ? "alloc" : "none", (unsigned)LCDQ_FRAME_BYTES);
}

static void lcdq_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  lcdq                        status\r\n");
	csh_printf(csh, "  lcdq init [boe|esp32|sdk]       power-cycle + qspi init (default: current profile)\r\n");
	csh_printf(csh, "  lcdq deinit                 backlight off + power off + qspi deinit\r\n");
	csh_printf(csh, "  lcdq profile [boe|esp32|sdk]    show / switch init table (then re-init)\r\n");
	csh_printf(csh, "  lcdq clk [80..16]           show / set qspi clock (then re-init)\r\n");
	csh_printf(csh, "  lcdq power on|off           screen 3V3 only (P51)\r\n");
	csh_printf(csh, "  lcdq bl on|off              backlight only (P23)\r\n");
	csh_printf(csh, "  lcdq fill <color>           solid color, 0xXXXX or 0xRRGGBB\r\n");
	csh_printf(csh, "  lcdq bars                   8 color bars (ESP32 rgb_test)\r\n");
	csh_printf(csh, "  lcdq cmd <reg> [hex ...]    raw qspi register write\r\n");
	csh_printf(csh, "  lcdq read <reg> [len]       raw qspi register read (diagnostic only)\r\n");
	csh_printf(csh, "examples:\r\n");
	csh_printf(csh, "  lcdq init                   # start here (ESP32 table)\r\n");
	csh_printf(csh, "  lcdq bars\r\n");
	csh_printf(csh, "  lcdq deinit && lcdq init sdk && lcdq bars   # A/B the SDK table\r\n");
}

static int lcdq_do_deinit(chry_shell_t *csh)
{
	lcdq_bl_set(false);

	if (s_inited) {
		bk_lcd_qspi_deinit(LCDQ_QSPI_ID, (uint8_t)LCDQ_RESET_PIN);
		s_inited = false;
	}

	bsp_power_screen_off();

	if (s_frame) {
		os_free(s_frame);
		s_frame = NULL;
	}

	csh_printf(csh, "deinit done\r\n");
	return 0;
}

static int lcdq_do_init(chry_shell_t *csh, int profile)
{
	bk_err_t ret;

	/* 驱动用 s_qspi_disp[].lcd_qspi_is_init 保护，重复 init 会直接返回 BK_OK
	 * 而不会重发新表，所以换表必须先 deinit。 */
	if (s_inited) {
		csh_printf(csh, "init: already inited, deinit first\r\n");
		lcdq_do_deinit(csh);
	}

	s_profile = profile;
	s_dev = lcdq_device(profile);
	lcdq_apply_clk();      /* 让 init 用上最新的 s_clk */

	csh_printf(csh, "init: profile=%s (%s), clk=%dMHz\r\n",
	           lcdq_prof_name(profile), lcdq_prof_desc(profile), (int)s_clk);

	/* 上电/复位期间先关背光，避免闪花屏 */
	lcdq_bl_set(false);

	/* 屏 RST 与模组 CEN 同网络，软件不能单独拉；断电再上电就是硬复位。
	 * bsp_power_screen_reset(): off -> 30ms -> on -> 30ms settle。 */
	ret = bsp_power_screen_reset();
	if (ret != BK_OK) {
		csh_printf(csh, "  screen power reset failed: %d\r\n", ret);
		return -1;
	}
	csh_printf(csh, "  screen power ON: scr=%s pad=%d\r\n",
	           bsp_power_is_enabled(BSP_PWR_RAIL_SCR) ? "ON" : "off",
	           bsp_power_get_pad(BSP_PWR_RAIL_SCR));

	if (!s_frame) {
		s_frame = (uint16_t *)psram_malloc(LCDQ_FRAME_BYTES);
		if (!s_frame) {
			csh_printf(csh, "  frame buffer malloc failed (%u bytes)\r\n",
			           (unsigned)LCDQ_FRAME_BYTES);
			return -1;
		}
	}
	csh_printf(csh, "  frame buf %u bytes @ 0x%08X\r\n",
	           (unsigned)LCDQ_FRAME_BYTES, (unsigned)(uint32_t)s_frame);

	/* reset_pin = GPIO_NUM -> lcd_qspi_hardware_reset() 直接跳过；
	 * 该函数内部还会：init DMA、soft reset、bk_qspi_init() 映射 P2~P7、
	 * 设全屏窗口 0x2A/0x2B、逐条发 init_cmd（data_len==0xFF 视为延时）。 */
	ret = bk_lcd_qspi_init(LCDQ_QSPI_ID, s_dev, (uint8_t)LCDQ_RESET_PIN);
	if (ret != BK_OK) {
		csh_printf(csh, "  bk_lcd_qspi_init failed: %d\r\n", ret);
		return -1;
	}

	s_inited = true;
	csh_printf(csh, "  bk_lcd_qspi_init OK (%dx%d, %u init cmds)\r\n",
	           s_dev->width, s_dev->height,
	           (unsigned)s_dev->qspi->device_init_cmd_len);

	lcdq_bl_set(true);
	csh_printf(csh, "  backlight ON (P23, high-active)\r\n");
	csh_printf(csh, "init done. try 'lcdq bars' or 'lcdq fill 0xF800'\r\n");
	return 0;
}

/* ---------------- 刷一帧 ---------------- */

#define LCDQ_FILL_SOLID 0
#define LCDQ_FILL_BARS  1

static int lcdq_need_init(chry_shell_t *csh)
{
	if (!s_inited || !s_frame || !s_dev) {
		csh_printf(csh, "not inited, run 'lcdq init' first\r\n");
		return -1;
	}
	return 0;
}

/* QSPI 刷帧（BY_FRAME + repeat DMA）。
 *
 * dma_sema 由 lcd_qspi_dma_init() 的 rtos_init_semaphore(&sema, 1) 创建 ——
 * 第二个参数是**最大计数**，初值为 0（不是 1）。因此：
 *   ① 绝不能在 frame_display() 之前 wait：此时没有任何帧在跑，中断不会
 *      set 信号，必然 5 秒超时（实测打印 "wait prev frame failed: -1"，
 *      -1 = BK_FAIL）；
 *   ② 每帧必须 frame_display() -> wait() 一一对应，wait 里的
 *      bk_lcd_qspi_quad_write_stop() 才负责收尾。
 *
 * repeat 模式的完成条件（lcd_qspi_driver.c:74-84）：DMA 写地址走到
 * src_pause_addr = data + data_len 时触发 pause 中断，handler 里判断
 * bk_dma_get_repeat_wr_pause() 为真才 set semaphore。所以 data_len 必须
 * 精确等于帧字节数（360*360*2 = 259200），否则永远等不到完成。
 *
 * 帧缓冲"先填后提交"：函数返回时上一帧已停止，连续调用不会出现
 * 「DMA 还在读、CPU 已改写」的撕裂。 */
static int lcdq_flush(chry_shell_t *csh, int mode, uint16_t color)
{
	bk_err_t ret;

	if (lcdq_need_init(csh) != 0)
		return -1;

	if (mode == LCDQ_FILL_BARS)
		lcdq_frame_bars();
	else
		lcdq_frame_fill(color);

	/* 传字节数（与官方 bk_display_qspi_ctlr.c 传 frame->size 一致）。
	 * 259200 > 0x10000，驱动走 repeat DMA 模式。 */
	ret = bk_lcd_qspi_frame_display(LCDQ_QSPI_ID, s_dev,
	                                (uint32_t *)s_frame, LCDQ_FRAME_BYTES);
	if (ret != BK_OK) {
		csh_printf(csh, "frame_display failed: %d\r\n", ret);
		return -1;
	}

	ret = bk_lcd_qspi_wait_display_complete(LCDQ_QSPI_ID, s_dev);
	if (ret != BK_OK) {
		csh_printf(csh, "wait frame failed: %d (DMA 未完成)\r\n", ret);
		return -1;
	}

	return 0;
}

/* ---------------- 命令入口 ---------------- */

int lcdq_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	const char *sub;

	if (argc == 1) {
		lcdq_status(csh);
		return 0;
	}

	sub = argv[1];

	if (!strcmp(sub, "help")) {
		lcdq_usage(csh);
		return 0;
	}

	if (!strcmp(sub, "init")) {
		int p = s_profile;

		if (argc > 2) {
			p = lcdq_parse_profile(argv[2]);
			if (p < 0) {
				csh_printf(csh, "unknown profile '%s' (boe|esp32|sdk)\r\n", argv[2]);
				return -1;
			}
		}
		return lcdq_do_init(csh, p);
	}

	if (!strcmp(sub, "deinit"))
		return lcdq_do_deinit(csh);

	if (!strcmp(sub, "clk")) {
		if (argc < 3) {
			csh_printf(csh, "clk = %dMHz\r\n", (int)s_clk);
			csh_printf(csh, "  available: 80 64 60 53 48 40 32 30 24 20 16 (MHz)\r\n");
			csh_printf(csh, "  change -> 'lcdq deinit' then 'lcdq init' to apply\r\n");
			return 0;
		}

		{
			int mhz = atoi(argv[2]);

			switch (mhz) {
			case 80: s_clk = LCD_QSPI_80M; break;
			case 64: s_clk = LCD_QSPI_64M; break;
			case 60: s_clk = LCD_QSPI_60M; break;
			case 53: s_clk = LCD_QSPI_53M; break;
			case 48: s_clk = LCD_QSPI_48M; break;
			case 40: s_clk = LCD_QSPI_40M; break;
			case 32: s_clk = LCD_QSPI_32M; break;
			case 30: s_clk = LCD_QSPI_30M; break;
			case 24: s_clk = LCD_QSPI_24M; break;
			case 20: s_clk = LCD_QSPI_20M; break;
			case 16: s_clk = LCD_QSPI_16M; break;
			default:
				csh_printf(csh, "unsupported clk %d MHz\r\n", mhz);
				csh_printf(csh, "  available: 80 64 60 53 48 40 32 30 24 20 16\r\n");
				return -1;
			}
		}

		lcdq_apply_clk();
		csh_printf(csh, "clk -> %dMHz; run 'lcdq deinit' then 'lcdq init' to apply\r\n",
		           (int)s_clk);
		if (s_profile == LCDQ_PROF_SDK)
			csh_printf(csh, "  note: 'sdk' profile is a const SDK table, stays 48MHz\r\n");
		return 0;
	}

	if (!strcmp(sub, "profile")) {
		int p;

		if (argc < 3) {
			csh_printf(csh, "profile = %s (%s)\r\n",
			           lcdq_prof_name(s_profile), lcdq_prof_desc(s_profile));
			return 0;
		}

		p = lcdq_parse_profile(argv[2]);
		if (p < 0) {
			csh_printf(csh, "unknown profile '%s' (boe|esp32|sdk)\r\n", argv[2]);
			return -1;
		}
		s_profile = p;
		csh_printf(csh, "profile -> %s; run 'lcdq init' to apply\r\n",
		           lcdq_prof_name(s_profile));
		return 0;
	}

	if (!strcmp(sub, "power")) {
		bool on;
		bk_err_t ret;

		if ((argc < 3) || (lcdq_parse_on_off(argv[2], &on) != 0)) {
			csh_printf(csh, "usage: lcdq power on|off\r\n");
			return -1;
		}

		ret = on ? bsp_power_screen_on() : bsp_power_screen_off();
		csh_printf(csh, "screen power %s -> %s (pad=%d, ret=%d)\r\n",
		           on ? "on" : "off",
		           bsp_power_is_enabled(BSP_PWR_RAIL_SCR) ? "ON" : "off",
		           bsp_power_get_pad(BSP_PWR_RAIL_SCR), ret);
		return 0;
	}

	if (!strcmp(sub, "bl")) {
		bool on;

		if ((argc < 3) || (lcdq_parse_on_off(argv[2], &on) != 0)) {
			csh_printf(csh, "usage: lcdq bl on|off\r\n");
			return -1;
		}

		lcdq_bl_set(on);
		csh_printf(csh, "backlight P%d -> %s\r\n",
		           (int)BOARD_LCD_BACKLIGHT_PIN, on ? "ON" : "off");
		return 0;
	}

	if (!strcmp(sub, "fill")) {
		uint16_t color;

		if (argc < 3) {
			csh_printf(csh, "usage: lcdq fill <0xXXXX|0xRRGGBB>\r\n");
			return -1;
		}

		color = lcdq_parse_color(argv[2]);
		csh_printf(csh, "fill 0x%04X ...\r\n", color);
		return lcdq_flush(csh, LCDQ_FILL_SOLID, color);
	}

	if (!strcmp(sub, "bars")) {
		csh_printf(csh, "8 color bars ...\r\n");
		return lcdq_flush(csh, LCDQ_FILL_BARS, 0);
	}

	if (!strcmp(sub, "cmd")) {
		uint8_t data[LCDQ_CMD_MAX];
		uint8_t reg;
		int n, i;
		bk_err_t ret;

		if (lcdq_need_init(csh) != 0)
			return -1;
		if (argc < 3) {
			csh_printf(csh, "usage: lcdq cmd <reg> [hex ...]  (e.g. lcdq cmd 0x11)\r\n");
			return -1;
		}

		reg = (uint8_t)strtoul(argv[2], NULL, 0);
		n = argc - 3;
		if (n > LCDQ_CMD_MAX)
			n = LCDQ_CMD_MAX;
		for (i = 0; i < n; i++)
			data[i] = (uint8_t)strtoul(argv[3 + i], NULL, 0);

		ret = bk_lcd_qspi_send_cmd(LCDQ_QSPI_ID, s_dev->qspi->reg_write_cmd,
		                           reg, n ? data : NULL, (uint8_t)n);
		csh_printf(csh, "cmd 0x02 / reg 0x%02X + %d bytes -> %d\r\n", reg, n, ret);
		return (ret == BK_OK) ? 0 : -1;
	}

	if (!strcmp(sub, "read")) {
		uint8_t buf[LCDQ_CMD_MAX];
		uint8_t reg;
		int len, i;
		bk_err_t ret;

		if (lcdq_need_init(csh) != 0)
			return -1;
		if (argc < 3) {
			csh_printf(csh, "usage: lcdq read <reg> [len]\r\n");
			return -1;
		}

		reg = (uint8_t)strtoul(argv[2], NULL, 0);
		len = (argc > 3) ? atoi(argv[3]) : 1;
		if ((len < 1) || (len > LCDQ_CMD_MAX))
			len = 1;

		ret = bk_lcd_qspi_read_data(LCDQ_QSPI_ID, buf, s_dev, reg, (uint8_t)len);
		csh_printf(csh, "read reg 0x%02X ret=%d:", reg, ret);
		for (i = 0; i < len; i++)
			csh_printf(csh, " %02X", buf[i]);
		csh_printf(csh, "\r\n");
		csh_printf(csh, "  hint: QSPI 4-line read needs dummy clocks / line order;\r\n");
		csh_printf(csh, "        all-zero here does NOT prove the panel is dead.\r\n");
		csh_printf(csh, "        judge with 'lcdq bars' instead.\r\n");
		return 0;
	}

	csh_printf(csh, "unknown subcmd '%s'\r\n", sub);
	lcdq_usage(csh);
	return -1;
}

/* ---------------- 对外接口（camera 等复用） ---------------- */

int lcdq_is_ready(void)
{
	return (s_inited && s_frame && s_dev) ? 1 : 0;
}

void *lcdq_frame_buf(void)
{
	return (s_inited && s_frame) ? (void *)s_frame : NULL;
}

int lcdq_display_rgb565(const void *src, uint32_t len)
{
	bk_err_t ret;

	if (!s_inited || !s_frame || !s_dev)
		return -1;
	if (!src || len == 0 || len > LCDQ_FRAME_BYTES)
		return -1;

	if (src != (const void *)s_frame)   /* 同缓冲时跳过自拷贝 */
		os_memcpy(s_frame, src, len);

	/* 与 lcdq_flush 相同的顺序：提交 -> 等本帧完成（semaphore 初值 0） */
	ret = bk_lcd_qspi_frame_display(LCDQ_QSPI_ID, s_dev,
	                                (uint32_t *)s_frame, LCDQ_FRAME_BYTES);
	if (ret != BK_OK)
		return (int)ret;

	ret = bk_lcd_qspi_wait_display_complete(LCDQ_QSPI_ID, s_dev);
	if (ret != BK_OK)
		return (int)ret;

	return 0;
}
