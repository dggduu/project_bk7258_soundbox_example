// Copyright 2026
// 屏幕 BSP 实现（ST77916 360x360 圆形 QSPI 屏）
//
// 【驱动路线】
// 换屏前用的是 ILI9341 + SDK 的 lcd_spi 驱动（复用 QSPI1 硬件做单线 SPI）。
// 现在换成 ST77916 + QSPI 4 线，走 SDK 的 lcd_qspi 驱动：
//   bk_lcd_qspi_init(qspi_id, dev, reset_pin)
//       配 QSPI 控制器、映射 P2~P7、逐条回放 192 项初始化命令、设整屏窗口
//   bk_lcd_qspi_frame_display(qspi_id, dev, data, len)
//       把整帧 RGB565 踢给 DMA
//   bk_lcd_qspi_wait_display_complete(qspi_id, dev)
//       等这一帧真正进 GRAM
//
// 【两个必须遵守的时序事实】（app_test_uart1 上实测得出，别改）
//
//  ① 必须 display -> wait 一一对应。
//     rtos_init_semaphore(&sema, 1) 的第二个参数是**最大计数**，初值为 0。
//     所以 frame_display() 之后才能 wait；在 display 之前 wait 会 5 秒超时
//     返回 BK_FAIL，而那一帧根本没提交，屏幕停在 GRAM 随机值（花屏）。
//
//  ② data_len 必须**精确等于**帧字节数（360*360*2 = 259200）。
//     repeat DMA 的完成条件是写地址走到 src_pause = data + data_len 才 set
//     信号（lcd_qspi_driver.c:74-84），差一个字节都等不到完成中断。
//
// 【16MHz 是硬上限】见 bsp_st77916_panel.c 顶部注释。

#include "bsp_screen.h"
#include "bsp_board_cfg.h"
#include "bsp_power.h"

#include <driver/gpio.h>
#include "gpio_driver.h"      /* gpio_dev_unmap() */
#include <driver/lcd.h>
#include <driver/lcd_qspi.h>
#include <driver/lcd_types.h>
#include <driver/lcd_qspi_types.h>
#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#define TAG "bsp_screen"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define BSP_LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)

/* 本板 LCD 挂在 QSPI1 上 */
#define BSP_LCD_QSPI_ID     ((qspi_id_t)BOARD_LCD_QSPI_ID)

#define BSP_LCD_COLOR_BYTES (BSP_LCD_BPP / 8)   /* RGB565 -> 2 字节 */

/* ---------------- 运行时状态 ---------------- */
static lcd_qspi_t s_qspi_cfg;
static lcd_device_t s_lcd_dev;
static const bsp_screen_panel_t *s_panel = NULL;
static bool s_ready = false;
static bool s_bl_inited = false;
static bool s_bl_on = false;

/* ---------------- 背光 ---------------- */
static void backlight_pin_init(void)
{
	if (s_bl_inited)
		return;

	/* 先解除复用，确保是 GPIO 功能而不是默认表里的 LCD 第二功能 */
	gpio_dev_unmap(BOARD_LCD_BACKLIGHT_PIN);
	bk_gpio_disable_pull(BOARD_LCD_BACKLIGHT_PIN);
	bk_gpio_enable_output(BOARD_LCD_BACKLIGHT_PIN);
	bk_gpio_set_output_low(BOARD_LCD_BACKLIGHT_PIN);
	s_bl_inited = true;
}

bk_err_t bsp_screen_backlight_on(void)
{
	backlight_pin_init();
	s_bl_on = true;
	return bk_gpio_set_output_high(BOARD_LCD_BACKLIGHT_PIN);
}

bk_err_t bsp_screen_backlight_off(void)
{
	backlight_pin_init();
	s_bl_on = false;
	return bk_gpio_set_output_low(BOARD_LCD_BACKLIGHT_PIN);
}

bk_err_t bsp_screen_backlight_set(uint8_t percent)
{
	/* 硬件上背光直连 GPIO，无 PWM 调光管；此处按开关处理。
	 * 若后续改用 PWM 调光，只需在这里换成 bk_pwm_set_period_duty()。 */
	if (percent == 0)
		return bsp_screen_backlight_off();

	return bsp_screen_backlight_on();
}

bool bsp_screen_backlight_is_on(void)
{
	return s_bl_on;
}

/* ---------------- GRAM 窗口维护 ----------------
 *
 * 为什么 BSP 里要自己发窗口命令：
 *   bk_lcd_qspi_frame_display() **不设窗口**，它只发像素引导字 + 踢 DMA。
 *   窗口是上一次 partial_display / init 留下的状态。只要中途调过一次
 *   bsp_screen_fill()（逐行 partial display），整帧 flush 就会灌进最后那
 *   一行的窗口里。
 *   SDK 里的 lcd_qspi_disp_area_config() 是 static 的，外部调不到，所以
 *   这里按它的实现（lcd_qspi_driver.c:368-390）原样复刻一份：
 *   0x2A = CASET（列地址），0x2B = RASET（行地址），各 4 字节大端。 */
#define LCD_QSPI_DEVICE_CASET   0x2A
#define LCD_QSPI_DEVICE_RASET   0x2B

static void set_display_area(uint16_t xs, uint16_t xe, uint16_t ys, uint16_t ye)
{
	uint8_t col[4] = {
		(uint8_t)((xs >> 8) & 0xFF), (uint8_t)(xs & 0xFF),
		(uint8_t)((xe >> 8) & 0xFF), (uint8_t)(xe & 0xFF)
	};
	uint8_t row[4] = {
		(uint8_t)((ys >> 8) & 0xFF), (uint8_t)(ys & 0xFF),
		(uint8_t)((ye >> 8) & 0xFF), (uint8_t)(ye & 0xFF)
	};

	bk_lcd_qspi_send_cmd(BSP_LCD_QSPI_ID, s_qspi_cfg.reg_write_cmd,
	                     LCD_QSPI_DEVICE_CASET, col, sizeof(col));
	bk_lcd_qspi_send_cmd(BSP_LCD_QSPI_ID, s_qspi_cfg.reg_write_cmd,
	                     LCD_QSPI_DEVICE_RASET, row, sizeof(row));
}

/* ---------------- 启动自检：刷纯色 ----------------
 *
 * 【为什么需要】换屏后第一次上板出现花屏，而串口只有 CP 日志、拿不到
 * AP 侧输出时，光靠"看起来花"无法区分故障层：
 *   - 屏驱动/初始化表有问题   -> 纯色也刷不出来（还是花屏/全黑）
 *   - 驱动没问题、数据源有问题 -> 纯色能刷出来，UI 却是花屏
 * 刷一段纯红/纯绿/纯蓝是成本最低的判别手段，而且顺带能验证字节序：
 * 刷红显蓝 = RGB565 高低字节反了；刷红显青 = 色序错位。
 *
 * 用 frame_display 而不是 partial_display：frame_display 是 app_test_uart1
 * 上验证过的路径，结论才可信；partial 模式在本工程还没验证。
 *
 * 缓冲从 PSRAM 取（259200B 放不进 AP 内部 RAM），取不到就跳过自检，
 * 不影响正常启动。
 *
 * 【发布前关掉】把 BSP_SCREEN_SELFTEST 置 0。
 */
#define BSP_SCREEN_SELFTEST   0
#define SELFTEST_HOLD_MS      250

static void screen_selftest_colors(void)
{
	static const struct {
		uint16_t color;
		const char *name;
	} seq[] = {
		{ 0xF800, "red"   },
		{ 0x07E0, "green" },
		{ 0x001F, "blue"  },
	};
	uint32_t pixels = (uint32_t)s_panel->width * s_panel->height;
	uint16_t *buf;
	void *probe;

	/* PSRAM 堆是懒初始化的，先 probe 一次 */
	probe = psram_malloc(4);
	if (!probe) {
		BSP_LOGW("selftest skipped: psram unavailable\r\n");
		return;
	}
	psram_free(probe);

	buf = (uint16_t *)psram_malloc(pixels * 2);
	if (!buf) {
		BSP_LOGW("selftest skipped: psram alloc %u failed\r\n",
		         (unsigned)(pixels * 2));
		return;
	}

	for (size_t s = 0; s < sizeof(seq) / sizeof(seq[0]); s++) {
		for (uint32_t i = 0; i < pixels; i++)
			buf[i] = seq[s].color;

		if (bsp_screen_flush(buf, pixels * 2) != BK_OK)
			BSP_LOGE("selftest %s flush failed\r\n", seq[s].name);
		else
			BSP_LOGI("selftest: %s\r\n", seq[s].name);

		rtos_delay_milliseconds(SELFTEST_HOLD_MS);
	}

	psram_free(buf);

	/* 自检结束。这里不主动清屏：紧接着 bsp_lvgl_init() 会把它自己的
	 * 帧缓冲 memset 0 并整帧推上来，屏幕自然会变成干净的黑底。 */
	BSP_LOGI("selftest done\r\n");
}

/* ---------------- 屏幕方向 ----------------
 *
 * ST77916 的显示方向由 MADCTL(0x36) 决定，位定义：
 *   D7 MY  D6 MX  D5 MV  D4 ML  D3 RGB  D2 MH
 * 旋转 180° = MY|MX 同时置位 = 0xC0。
 *
 * 【为什么不在初始化表里加】`bsp_st77916_boe_init.h` 是从厂商例程 1:1
 * 转出来的 192 项，本该保持原样（表里压根没有 0x36，靠上电默认 0x00）。
 * 往里面插条目会让"这张表 = 厂商表"这个前提失效，以后对照厂商例程排查
 * 时容易误判，所以方向这类"本工程策略"统一放在 BSP 里下发。
 *
 * 【为什么先 DISPOFF】部分控制器在 DISPON 期间改 MADCTL 不会立刻生效，
 * 必须重新使能一次显示。这里走 DISPOFF -> MADCTL -> DISPON 的完整序列，
 * 只在初始化时做一次，肉眼只是一次极短的亮度变化。
 *
 * 若上板后红蓝互换，说明面板是 BGR 排列，把 0xC0 改成 0xC8（置 RGB 位）。
 */
#define BSP_LCD_MADCTL_180   0xC0

static void apply_rotation_180(void)
{
	const uint8_t madctl = BSP_LCD_MADCTL_180;

	bsp_screen_send_cmd(0x28, NULL, 0);          /* DISPOFF */
	bsp_screen_send_cmd(0x36, &madctl, 1);       /* MADCTL  */
	bsp_screen_send_cmd(0x29, NULL, 0);          /* DISPON  */
	BSP_LOGI("rotation 180 (MADCTL=0x%02x)\r\n", madctl);
}

/* ---------------- 面板配置构造 ---------------- */
static bk_err_t build_device(const bsp_screen_panel_t *p)
{
	if (!p || !p->init_cmd || !p->init_cmd_len)
		return BK_ERR_NULL_PARAM;

	os_memset(&s_qspi_cfg, 0, sizeof(s_qspi_cfg));
	os_memset(&s_lcd_dev, 0, sizeof(s_lcd_dev));

	s_qspi_cfg.clk = p->clk;
	s_qspi_cfg.refresh_method = p->refresh_method;
	s_qspi_cfg.reg_write_cmd = p->reg_write_cmd;
	s_qspi_cfg.reg_read_cmd = p->reg_read_cmd;
	s_qspi_cfg.reg_read_config.dummy_clk = 0;
	s_qspi_cfg.reg_read_config.dummy_mode = LCD_QSPI_NO_INSERT_DUMMMY_CLK;
	s_qspi_cfg.pixel_write_config.cmd = (uint8_t *)p->pixel_cmd;
	s_qspi_cfg.pixel_write_config.cmd_len = p->pixel_cmd_len;
	s_qspi_cfg.init_cmd = p->init_cmd;
	s_qspi_cfg.device_init_cmd_len = p->init_cmd_len;
	s_qspi_cfg.frame_len = p->width * p->height * BSP_LCD_COLOR_BYTES;

	s_lcd_dev.id = LCD_DEVICE_UNKNOW;   /* 走通用 QSPI 流程，不依赖 SDK 面板表 */
	s_lcd_dev.name = (char *)p->name;
	s_lcd_dev.type = LCD_TYPE_QSPI;
	s_lcd_dev.width = p->width;
	s_lcd_dev.height = p->height;
	s_lcd_dev.src_fmt = PIXEL_FMT_RGB565;
	s_lcd_dev.out_fmt = PIXEL_FMT_RGB565;
	s_lcd_dev.qspi = &s_qspi_cfg;
	s_lcd_dev.init = NULL;
	s_lcd_dev.off = NULL;

	return BK_OK;
}

bk_err_t bsp_screen_init_with(const bsp_screen_panel_t *panel)
{
	bk_err_t ret;

	if (!panel || !panel->init_cmd)
		return BK_ERR_NULL_PARAM;

	if (s_ready)
		bsp_screen_deinit();

	ret = build_device(panel);
	if (ret != BK_OK)
		return ret;

	/* 屏背光先关，避免上电/复位时闪花 */
	bsp_screen_backlight_off();

	/* 【先把 PSRAM 堆唤醒再初始化屏】
	 *
	 * app_test_uart1 是在系统完全跑起来之后用 `lcdq init` 手动初始化屏的，
	 * 而本工程在 main() 早期就初始化。两者的差别不只是"时间早晚"：
	 * PSRAM 堆是懒初始化的，直到第一次 psram_malloc 才会去配 PSRAM
	 * 控制器和它的时钟。如果 QSPI 与 PSRAM 共用 PLL/时钟域，那么
	 * "先配 QSPI、后配 PSRAM" 和 "先 PSRAM、后 QSPI" 得到的 QSPI 时钟
	 * 可能不同 —— 前者会让屏收到错误波特率的数据，表现为花屏。
	 *
	 * 这里主动探一次，把 PSRAM 初始化提前，消除这个顺序变量。
	 * 探不到也不报错：屏初始化本身不依赖 PSRAM（只有自检刷色需要）。 */
	{
		void *probe = psram_malloc(4);

		if (probe) {
			psram_free(probe);
		} else {
			BSP_LOGW("psram probe failed, continue anyway\r\n");
		}
	}

	/* 【必须做一次真正的 power cycle】
	 *
	 * 本板屏的 RST 接在 CSN 上，没有独立复位脚，所以 bk_lcd_qspi_init()
	 * 的 reset_pin 只能传 GPIO_NUM 跳过硬复位 —— 也就是说屏的复位
	 * **完全依赖掉电**。
	 *
	 * 如果不先关一次电源：重新烧录/软复位后 P51 可能仍是导通状态，
	 * 屏既没掉电也没收到复位，内部寄存器停在上一轮的配置上（还可能
	 * 处在 sleep-in 或 partial mode），接着再灌 192 条初始化命令就会
	 * 得到一份"半新半旧"的配置 —— 典型症状就是花屏、或者只有部分
	 * 区域显示正确。
	 *
	 * 先 off 再 on，把这段状态彻底清掉。 */
	bsp_power_screen_off();
	rtos_delay_milliseconds(BOARD_SCR_PWR_SETTLE_MS);

	ret = bsp_power_screen_on();
	if (ret != BK_OK) {
		BSP_LOGE("screen power on failed: %d\r\n", ret);
		return ret;
	}

	/* bsp_power_screen_on() 内部已按 BOARD_SCR_PWR_SETTLE_MS 等 120ms
	 * （ST77916 的 power-on sequence 要求，少等会导致整段初始化命令
	 * 被屏忽略 —— 症状也是花屏）。 */

	/* 再补 20ms 冗余：PMOS 栅极 RC + 屏内部 LDO 起振的离散性，
	 * 120ms 是手册下限，卡着下限跑在温度/批次上有风险。 */
	rtos_delay_milliseconds(20);

	/* bk_lcd_qspi_init() 内部会：
	 *   1) 分配并配置 DMA（REPEAT 模式，源/目的都打 SEC 属性）
	 *   2) qspi_hal 软复位 + 引脚映射 P2~P7 到 QSPI1
	 *   3) 按 reset_pin 做硬复位（本板 RST 接 CSN，传 GPIO_NUM 跳过）
	 *   4) lcd_qspi_driver_init() 配时钟与分频
	 *   5) 设整屏显示窗口
	 *   6) 逐条回放 192 项初始化命令（含 0x11 后 120ms 延时）
	 */
	ret = bk_lcd_qspi_init(BSP_LCD_QSPI_ID, &s_lcd_dev, panel->reset_pin);
	if (ret != BK_OK) {
		BSP_LOGE("bk_lcd_qspi_init failed: %d\r\n", ret);
		bsp_power_screen_off();
		return ret;
	}

	s_panel = panel;
	s_ready = true;

	/* 方向必须在开背光前定下来，否则用户会看到一次方向跳变 */
	apply_rotation_180();

	bsp_screen_backlight_on();

	BSP_LOGI("screen ready: %s %dx%d, qspi%u %dMHz, %u init cmds, reset=P%u\r\n",
	         panel->name, panel->width, panel->height, BOARD_LCD_QSPI_ID,
	         (int)panel->clk, (unsigned)panel->init_cmd_len,
	         (unsigned)panel->reset_pin);

#if BSP_SCREEN_SELFTEST
	/* 读回 COLMOD（命令 0x0B）确认识别格式。
	 *
	 * 这是"花屏"最常见根因的**确定性判据**：ST77916 上电默认是 18bit
	 * RGB666，必须由初始化表里的 `0x3A = 0x55` 改成 16bit RGB565。
	 * 若没写进去，360*360*2 的 RGB565 帧只填了 2/3 的 GRAM，
	 * 表现为"上 2/5 花屏、下 3/5 有色条但发灰"。
	 *
	 * 读到 0x55 -> COLMOD 正确，花屏不是这个原因；
	 * 读到 0x00/0xFF/其它 -> 初始化命令根本没进屏（查引脚/CS/时钟）。
	 */
	{
		uint8_t colmod = 0;
		bk_err_t rret = bk_lcd_qspi_read_data(BSP_LCD_QSPI_ID, &colmod,
		                                      &s_lcd_dev, 0x0B, 1);

		if (rret == BK_OK)
			BSP_LOGI("selftest: COLMOD=0x%02x (expect 0x55)\r\n", colmod);
		else
			BSP_LOGW("selftest: read COLMOD failed (%d)\n", rret);
	}

	screen_selftest_colors();
#endif

	return BK_OK;
}

bk_err_t bsp_screen_init(void)
{
	return bsp_screen_init_with(&bsp_panel_st77916_360x360);
}

bk_err_t bsp_screen_deinit(void)
{
	bsp_screen_backlight_off();

	if (s_ready) {
		bk_lcd_qspi_deinit(BSP_LCD_QSPI_ID,
		                   s_panel ? s_panel->reset_pin : GPIO_NUM);
		s_ready = false;
	}

	bsp_power_screen_off();
	return BK_OK;
}

bk_err_t bsp_screen_reset(void)
{
	const bsp_screen_panel_t *p = s_panel;

	if (!p)
		return BK_ERR_NOT_INIT;

	return bsp_screen_init_with(p);
}

bk_err_t bsp_screen_display_on(void)
{
	if (!s_ready)
		return BK_ERR_NOT_INIT;

	/* 0x29 = DISPON。走 QSPI 的寄存器写通道，不带参数。 */
	return bk_lcd_qspi_send_cmd(BSP_LCD_QSPI_ID, s_qspi_cfg.reg_write_cmd,
	                            0x29, NULL, 0);
}

bk_err_t bsp_screen_display_off(void)
{
	if (!s_ready)
		return BK_ERR_NOT_INIT;

	return bk_lcd_qspi_send_cmd(BSP_LCD_QSPI_ID, s_qspi_cfg.reg_write_cmd,
	                            0x28, NULL, 0);
}

bk_err_t bsp_screen_flush(const void *data, uint32_t len)
{
	bk_err_t ret;

	if (!s_ready || !s_panel)
		return BK_ERR_NOT_INIT;
	if (!data || !len)
		return BK_ERR_NULL_PARAM;
	/* DMA 按 32bit 搬数据，长度必须是 4 的整数倍 */
	if ((len & 0x3) != 0) {
		BSP_LOGE("flush len %u not 4-byte aligned\r\n", (unsigned)len);
		return BK_ERR_PARAM;
	}
	/* len 必须精确等于帧长，否则 DMA 完成信号永远不来（见文件头注释②） */
	if (len != s_qspi_cfg.frame_len) {
		BSP_LOGE("flush len %u != frame_len %u\r\n",
		         (unsigned)len, (unsigned)s_qspi_cfg.frame_len);
		return BK_ERR_PARAM;
	}

	/* frame_display() 只是把 DMA 踢起来，立刻 wait 才是正确配对。
	 *
	 * 【不要在这里发 CASET/RASET】曾尝试每帧设一次整屏窗口"求稳"，
	 * 结果多了两条同步命令，而且是在 DMA 已配置好的状态下发 —— 收益为零
	 * （init 已经设过整屏窗口，flush 本身不改窗口），风险反而更大。
	 * 只有 bsp_screen_fill() 会改窗口，它自己在结束时负责恢复。 */
	ret = bk_lcd_qspi_frame_display(BSP_LCD_QSPI_ID, &s_lcd_dev,
	                                (uint32_t *)(uintptr_t)data, len);
	if (ret != BK_OK) {
		BSP_LOGE("frame display failed: %d\r\n", ret);
		return ret;
	}

	ret = bk_lcd_qspi_wait_display_complete(BSP_LCD_QSPI_ID, &s_lcd_dev);
	if (ret != BK_OK) {
		BSP_LOGE("wait display complete failed: %d\r\n", ret);
		return ret;
	}

	return BK_OK;
}

bk_err_t bsp_screen_flush_area(uint16_t x, uint16_t y, uint16_t w, uint16_t h,
                               const void *data)
{
	lcd_display_area_t area;
	bk_err_t ret;

	if (!s_ready || !s_panel)
		return BK_ERR_NOT_INIT;
	if (!data || !w || !h)
		return BK_ERR_NULL_PARAM;
	if ((uint32_t)x + w > s_panel->width || (uint32_t)y + h > s_panel->height) {
		BSP_LOGE("flush area %u,%u %ux%d out of %ux%u\r\n",
		         (unsigned)x, (unsigned)y, (unsigned)w, (unsigned)h,
		         s_panel->width, s_panel->height);
		return BK_ERR_PARAM;
	}

	/* partial_display() 内部先设窗口再起 single 模式 DMA：
	 *   lcd_qspi_driver.c:640-653
	 *     lcd_qspi_disp_area_config(area)          <- 设 CASET/RASET
	 *     lcd_qspi_dma_single_mode_config(len = w*h*2)
	 *     quad_write_start + bk_dma_start
	 * len = w*h*2，360 宽的块恒为 4 的倍数（720 的倍数）；
	 * w <= 360、h <= LVGL draw_buf 行数（40）时 len <= 28800 < 0x10000，
	 * 恒走 single 模式，不会落进 repeat 模式分支。 */
	area.x_start = x;
	area.y_start = y;
	area.x_end = (uint16_t)(x + w - 1);
	area.y_end = (uint16_t)(y + h - 1);

	ret = bk_lcd_qspi_partial_display(BSP_LCD_QSPI_ID, &s_lcd_dev,
	                                  &area, (uint32_t *)(uintptr_t)data);
	if (ret != BK_OK) {
		BSP_LOGE("partial display %u,%u %ux%d failed: %d\r\n",
		         (unsigned)x, (unsigned)y, (unsigned)w, (unsigned)h, ret);
		return ret;
	}

	ret = bk_lcd_qspi_wait_display_complete(BSP_LCD_QSPI_ID, &s_lcd_dev);
	if (ret != BK_OK) {
		BSP_LOGE("wait partial failed: %d\r\n", ret);
		return ret;
	}

	return BK_OK;
}

bk_err_t bsp_screen_fill(uint16_t color)
{
	static uint16_t line[BSP_LCD_H_RES];
	lcd_display_area_t area;
	uint32_t w, h, i;
	bk_err_t ret = BK_OK;

	if (!s_ready || !s_panel)
		return BK_ERR_NOT_INIT;

	w = s_panel->width;
	h = s_panel->height;
	if (w > sizeof(line) / sizeof(line[0]))
		w = sizeof(line) / sizeof(line[0]);

	for (i = 0; i < w; i++)
		line[i] = color;

	/* 一行一行刷：partial display 会重设窗口，速度比整帧慢，够调试用。
	 * 注意 partial_display 的 data 形参是 uint32_t*，必须显式转换。 */
	for (i = 0; i < h; i++) {
		area.x_start = 0;
		area.x_end = (uint16_t)(w - 1);
		area.y_start = (uint16_t)i;
		area.y_end = (uint16_t)i;
		ret = bk_lcd_qspi_partial_display(BSP_LCD_QSPI_ID, &s_lcd_dev,
		                                  &area, (uint32_t *)line);
		if (ret != BK_OK)
			break;
		ret = bk_lcd_qspi_wait_display_complete(BSP_LCD_QSPI_ID, &s_lcd_dev);
		if (ret != BK_OK)
			break;
	}

	/* 恢复整屏窗口，免得后面的整帧 flush 用到残窗口。
	 * bsp_screen_flush() 自己也会设一遍，这里是双保险（fill 之后
	 * 可能有人直接调 partial display）。 */
	if (s_ready)
		set_display_area(0, (uint16_t)(s_panel->width - 1),
		                 0, (uint16_t)(s_panel->height - 1));

	return ret;
}

bk_err_t bsp_screen_send_cmd(uint8_t cmd, const uint8_t *data, uint8_t len)
{
	if (!s_ready)
		return BK_ERR_NOT_INIT;

	return bk_lcd_qspi_send_cmd(BSP_LCD_QSPI_ID, s_qspi_cfg.reg_write_cmd,
	                            cmd, (uint8_t *)data, len);
}

const bsp_screen_panel_t *bsp_screen_get_panel(void)
{
	return s_panel;
}

bool bsp_screen_is_ready(void)
{
	return s_ready;
}
