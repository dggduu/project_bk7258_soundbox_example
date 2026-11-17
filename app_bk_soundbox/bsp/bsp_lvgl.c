// Copyright 2026
// LVGL 8 显示后端实现（partial mode + PSRAM 分块缓冲）
//
// 【2026-11 从 direct mode 改成 partial mode】
//
// 原 direct mode：draw_buf = 整帧 259200B，flush 每次全帧推。
//   8MHz QSPI 下全帧 65ms —— 哪怕只动了进度条，也要花 65ms 重灌整屏。
//
// 现在 partial mode：draw_buf = 360 x LV_BUF_LINES 行的小块，
//   LVGL 只把"这一轮重画的区域"渲染进块里，flush 按区域推。
//   进度条一行 (216x20) ≈ 8.6KB -> 2.2ms；整页切换也只推变化的部分。
//   帧缓冲占用从 259KB 降到 28.8KB（PSRAM），pad 缓冲同样大小。
//
// 【DMA 的两个对齐硬条件】（partial_display 走 single 模式 DMA，32bit）
//   1. color_p 首地址 4 字节对齐
//   2. 长度 w*h*2 是 4 的整数倍
// LVGL 的脏区尺寸由控件决定，两个条件都可能不满足：
//   x1 为奇数 -> color_p = draw_buf_base + y_off*720 + x1*2 -> 2 对齐；
//   w 为奇数 -> len = 奇*偶 -> 2 对齐。
// 命中任一条件就把这块拷进自备 pad 缓冲：宽度向上取偶、行尾补一列
// 同色像素（视觉无痕），再刷 pad。正常路径（整宽块）零拷贝直刷。

#include "bsp_lvgl.h"
#include "bsp_screen.h"
#include "bsp_board_cfg.h"

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#include "lvgl.h"
#include "app_ui_theme.h"

#define TAG "bsp_lvgl"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#ifndef CONFIG_LVGL_TASK_PRIORITY
#define CONFIG_LVGL_TASK_PRIORITY 4
#endif

#define LVGL_TASK_STACK     8192
#define LVGL_TASK_PRIO      CONFIG_LVGL_TASK_PRIORITY
#define LVGL_TASK_PERIOD_MS 5

/* draw_buf 的行数。越大单块越大（一次 flush 搬得多、次数少），
 * 越小占用越低。40 行 = 28800B，是"进度条级局部刷新"与"整页重绘
 * 次数"的折中；整页重绘会被 LVGL 自动切成 ceil(360/40)=9 块。 */
#define LV_BUF_LINES        40
#define LV_BUF_BYTES        (BSP_LCD_H_RES * LV_BUF_LINES * sizeof(lv_color_t))

static lv_color_t *s_fb;         /* draw_buf（PSRAM，28.8KB） */
static lv_color_t *s_pad;        /* 奇数宽/非对齐场景的 pad 缓冲（PSRAM，28.8KB） */
static lv_disp_draw_buf_t s_draw_buf;
static lv_disp_drv_t s_disp_drv;
static beken_thread_t s_task;
static volatile bool s_running;
static bool s_ready;
static bsp_lvgl_loop_hook_t s_hook;

static void disp_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_p)
{
	uint16_t x = (uint16_t)area->x1;
	uint16_t y = (uint16_t)area->y1;
	uint16_t w = (uint16_t)(area->x2 - area->x1 + 1);
	uint16_t h = (uint16_t)(area->y2 - area->y1 + 1);

	if (s_fb && s_pad) {
		if (((uintptr_t)color_p & 3) || (w & 1)) {
			/* 对齐修复路径：拷进 pad 缓冲。
			 * pad 行宽向上取偶（w 偶则 w，w 奇则 w+1），行尾补一列
			 * 同色像素 —— 多出的 1px 与原边界同色，视觉无痕。 */
			uint16_t pw = (uint16_t)((w + 1) & ~1u);
			uint16_t row, col;

			for (row = 0; row < h; row++) {
				const lv_color_t *src = color_p + (uint32_t)row * w;
				lv_color_t *dst = s_pad + (uint32_t)row * pw;

				os_memcpy(dst, src, (uint32_t)w * sizeof(lv_color_t));
				for (col = w; col < pw; col++)
					dst[col] = src[w - 1];
			}

			bsp_screen_flush_area(x, y, pw, h, s_pad);
		} else {
			/* 正常路径：color_p 本身就是对齐的连续块，零拷贝直刷 */
			bsp_screen_flush_area(x, y, w, h, color_p);
		}
	}

	lv_disp_flush_ready(drv);
}

static void lvgl_task(void *arg)
{
	(void)arg;

	while (s_running) {
		if (s_hook)
			s_hook();

		lv_timer_handler();
		rtos_delay_milliseconds(LVGL_TASK_PERIOD_MS);
	}

	s_task = NULL;
	rtos_delete_thread(NULL);
}

bk_err_t bsp_lvgl_init(bsp_lvgl_loop_hook_t hook)
{
	void *probe;

	if (s_ready)
		return BK_OK;

	if (!bsp_screen_is_ready()) {
		LOGE("screen not ready\r\n");
		return BK_ERR_NOT_INIT;
	}

	/* PSRAM 堆是懒初始化的：第一次 psram_malloc 之前 free_size 返回 0。
	 * 先 probe 一次让堆就绪，再检查 256KB LVGL pool + 两个块缓冲。 */
	probe = psram_malloc(4);

	if (!probe) {
		LOGE("psram init/probe failed\r\n");
		return BK_FAIL;
	}
	psram_free(probe);

	if (rtos_get_psram_free_heap_size() <
	    (256U * 1024U + 2u * (uint32_t)LV_BUF_BYTES)) {
		LOGE("psram too low: free=%u need=%u\r\n",
		     (unsigned)rtos_get_psram_free_heap_size(),
		     (unsigned)(256U * 1024U + 2u * (uint32_t)LV_BUF_BYTES));
		return BK_FAIL;
	}

	lv_init();

	s_fb = (lv_color_t *)psram_malloc(LV_BUF_BYTES);
	s_pad = (lv_color_t *)psram_malloc(LV_BUF_BYTES);
	if (!s_fb || !s_pad) {
		LOGE("buf malloc %u failed (fb=%p pad=%p)\r\n",
		     (unsigned)LV_BUF_BYTES, s_fb, s_pad);
		if (s_fb) { psram_free(s_fb); s_fb = NULL; }
		if (s_pad) { psram_free(s_pad); s_pad = NULL; }
		return BK_FAIL;
	}
	os_memset(s_fb, 0, LV_BUF_BYTES);

	lv_disp_draw_buf_init(&s_draw_buf, s_fb, NULL,
	                      BSP_LCD_H_RES * LV_BUF_LINES);

	lv_disp_drv_init(&s_disp_drv);
	s_disp_drv.hor_res = BSP_LCD_H_RES;
	s_disp_drv.ver_res = BSP_LCD_V_RES;
	s_disp_drv.flush_cb = disp_flush;
	s_disp_drv.draw_buf = &s_draw_buf;
	s_disp_drv.direct_mode = 0;     /* partial：只刷脏区 */
	s_disp_drv.full_refresh = 0;
	lv_disp_drv_register(&s_disp_drv);

	/* 换掉 LVGL 自带的 lv_theme_default：它在本工程占 17.9KB Flash，且会把
	 * chart/meter/colorwheel/calendar/keyboard/table/menu 等未使用的 widget
	 * 通过 `lv_xxx_class` 引用一起拖进来。改用 app_ui_theme.c 的极简主题。
	 * 必须放在 lv_disp_drv_register() 之后 —— 注册时 LVGL 只会按
	 * LV_USE_THEME_DEFAULT 初始化默认主题，这里再覆盖掉。 */
	app_ui_theme_init(lv_disp_get_default());

	s_hook = hook;
	s_ready = true;

	LOGI("lvgl ready %dx%d, partial buf %ux%u = %u bytes in psram\r\n",
	     BSP_LCD_H_RES, BSP_LCD_V_RES,
	     BSP_LCD_H_RES, (unsigned)LV_BUF_LINES, (unsigned)LV_BUF_BYTES);
	return BK_OK;
}

bk_err_t bsp_lvgl_start(void)
{
	bk_err_t ret;

	if (!s_ready)
		return BK_ERR_NOT_INIT;
	if (s_running)
		return BK_OK;

	s_running = true;
	ret = rtos_create_thread(&s_task, LVGL_TASK_PRIO, "bsp_lvgl",
	                         (beken_thread_function_t)lvgl_task,
	                         LVGL_TASK_STACK, NULL);
	if (ret != BK_OK) {
		s_running = false;
		LOGE("create lvgl task failed: %d\r\n", ret);
		return ret;
	}

	return BK_OK;
}

bk_err_t bsp_lvgl_stop(void)
{
	if (!s_running)
		return BK_OK;

	s_running = false;
	rtos_delay_milliseconds(LVGL_TASK_PERIOD_MS * 3 + 20);
	return BK_OK;
}

bk_err_t bsp_lvgl_deinit(void)
{
	bsp_lvgl_stop();

	if (s_fb) {
		psram_free(s_fb);
		s_fb = NULL;
	}
	if (s_pad) {
		psram_free(s_pad);
		s_pad = NULL;
	}

	s_ready = false;
	return BK_OK;
}

bool bsp_lvgl_is_ready(void)
{
	return s_ready;
}
