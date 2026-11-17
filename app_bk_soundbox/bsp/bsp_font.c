// Copyright 2026
// 双来源字体实现：SD 卡 TTF（stb_truetype） + 内置点阵字库
//
// stb 的实现只在本文件里展开（STBTT_STATIC + IMPLEMENTATION），
// 这样不用改 SDK 的 LVGL 组件，也不会和别的库抢符号。

#include "bsp_font.h"
#include "bsp_board_cfg.h"

#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include <components/log.h>
#include <stdio.h>
#include <string.h>

#include "ff.h"

#define TAG "bsp_font"
#define FONT_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define FONT_LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define FONT_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

LV_FONT_DECLARE(font_sc16);
LV_FONT_DECLARE(font_sc12);

/* ---------------- stb_truetype（单文件实现） ----------------
 * 第三方头自带大量未用到的 API，编译开关上先把常见告警关掉，
 * 免得整个组件被 -Werror 拖死（工程的告警策略不适用于第三方代码）。 */
#if defined(__GNUC__)
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wunused-function"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wunused-const-variable"
#pragma GCC diagnostic ignored "-Wunused-macros"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wshadow"
#pragma GCC diagnostic ignored "-Wpointer-sign"
#pragma GCC diagnostic ignored "-Wcast-qual"
#pragma GCC diagnostic ignored "-Wmaybe-uninitialized"
#pragma GCC diagnostic ignored "-Wold-style-declaration"
#endif

#define STB_RECT_PACK_IMPLEMENTATION
#define STBRP_STATIC
#define STBTT_STATIC
#define STB_TRUETYPE_IMPLEMENTATION
#define STBTT_malloc(x, u) ((void)(u), psram_malloc(x))
#define STBTT_free(x, u)   ((void)(u), psram_free(x))
#include "third_party/stb/stb_rect_pack.h"
#include "third_party/stb/stb_truetype_htcw.h"

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif

/* ---------------- 配置 ---------------- */

#define FONT_MAX_BYTES      (4 * 1024 * 1024)   /* 整个 TTF 读进 PSRAM */
#define FONT_BMP_MAX        (64 * 64)           /* 单字形位图上限（8bpp） */
#define FONT_READ_CHUNK     16384

/* 与内置字库对齐的行高：这样现有 label 的高度/位置不用改 */
static const int8_t s_line_h[BSP_FONT_ID_MAX] = {
	[BSP_FONT_SMALL] = 15,
	[BSP_FONT_NORMAL] = 19,
};
static const uint16_t s_px[BSP_FONT_ID_MAX] = {
	[BSP_FONT_SMALL] = 12,
	[BSP_FONT_NORMAL] = 16,
};

typedef struct {
	lv_font_t font;            /* 必须是第一个成员，回调里靠它反推自己 */
	stbtt_fontinfo info;
	float scale;
	uint16_t px;
	int16_t ascent_px;

	uint8_t *bmp;              /* 单个字形的 8bpp 缓冲 */

	/* get_glyph_dsc 刚算过的字形，get_glyph_bitmap 直接复用 */
	uint32_t last_cp;
	int last_gid;
	int last_w;
	int last_h;
} bsp_ttf_font_t;

static bsp_ttf_font_t s_ttf[BSP_FONT_ID_MAX];
static const lv_font_t *s_sel[BSP_FONT_ID_MAX] = { &font_sc12, &font_sc16 };
static bool s_ttf_ok;
static char s_source[32] = "builtin";
static char s_ttf_path[24] = "";     /* TTF 原始路径，切换来源时回显用 */
static uint8_t *s_ttf_data;
static uint32_t s_ttf_len;

/* ---------------- v8 字体回调 ---------------- */

static bool ttf_get_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc,
                              uint32_t letter, uint32_t letter_next)
{
	bsp_ttf_font_t *f = (bsp_ttf_font_t *)font;
	int gid, adv = 0, lsb = 0;
	int x0 = 0, y0 = 0, x1 = 0, y1 = 0;
	int adv16;

	if (!dsc || (f->scale <= 0.0f))
		return false;

	gid = stbtt_FindGlyphIndex(&f->info, (int)letter);
	if (gid <= 0) {
		f->last_gid = -1;
		return false;
	}

	stbtt_GetGlyphHMetrics(&f->info, gid, &adv, &lsb);
	stbtt_GetGlyphBitmapBox(&f->info, gid, f->scale, f->scale, &x0, &y0, &x1, &y1);

	adv16 = (int)((float)adv * f->scale * 16.0f + 0.5f);

	if (letter_next) {
		int gn = stbtt_FindGlyphIndex(&f->info, (int)letter_next);

		if (gn > 0)
			adv16 += (int)((float)stbtt_GetGlyphKernAdvance(&f->info, gid, gn) *
			               f->scale * 16.0f);
	}

	dsc->adv_w = (uint16_t)(adv16 < 0 ? 0 : adv16);
	dsc->box_w = (uint16_t)(x1 > x0 ? (x1 - x0) : 0);
	dsc->box_h = (uint16_t)(y1 > y0 ? (y1 - y0) : 0);
	dsc->ofs_x = (int16_t)x0;
	/* LVGL v8 绘制公式：位图底边 = 基线 - ofs_y。
	 * stbtt 的 y0 就是"位图底部相对基线的偏移（正=基线上方）"，
	 * 恰好等于 ofs_y——与内置 font_sc16 的数据完全一致
	 * （如 '"'：基线上方 9px，生成器 ofs_y=9）。
	 * 之前写成 -y1（位图顶部相对基线），整行字形会下沉 box 高度。 */
	dsc->ofs_y = (int16_t)y0;
	dsc->bpp = 8;
	dsc->is_placeholder = 0;

	f->last_cp = letter;
	f->last_gid = gid;
	f->last_w = dsc->box_w;
	f->last_h = dsc->box_h;

	return true;
}

static const uint8_t *ttf_get_glyph_bitmap(const lv_font_t *font, uint32_t letter)
{
	bsp_ttf_font_t *f = (bsp_ttf_font_t *)font;
	int gid, w, h;
	int x0 = 0, y0 = 0, x1 = 0, y1 = 0;

	if (!f->bmp)
		return NULL;

	if (letter == f->last_cp) {
		gid = f->last_gid;
		w = f->last_w;
		h = f->last_h;
	} else {
		gid = stbtt_FindGlyphIndex(&f->info, (int)letter);
		if (gid <= 0)
			return NULL;
		stbtt_GetGlyphBitmapBox(&f->info, gid, f->scale, f->scale, &x0, &y0, &x1, &y1);
		w = (x1 > x0) ? (x1 - x0) : 0;
		h = (y1 > y0) ? (y1 - y0) : 0;
	}

	if ((w <= 0) || (h <= 0))
		return f->bmp;                      /* 空格之类：空位图 */

	if ((w * h) > FONT_BMP_MAX) {
		FONT_LOGW("glyph 0x%lx too big %dx%d\r\n", (unsigned long)letter, w, h);
		return NULL;
	}

	os_memset(f->bmp, 0, (size_t)(w * h));
	stbtt_MakeGlyphBitmap(&f->info, f->bmp, w, h, w, f->scale, f->scale, gid);
	return f->bmp;
}

/* ---------------- 从 TTF 内存建字体 ---------------- */

/* 读 name 表的 family name，判断是不是简体中文面。
 * .ttc 里 face 顺序常见 JP/KR/SC/TC/HK…，拿 face 0 会显示日文字形
 * （"骨""直""黑"等写法不同），所以要挑 SC/CN 面。 */
static bool face_name_is_sc(const stbtt_fontinfo *info)
{
	static const char *const needles[] = { "SC", "Simplified", "CN", "Chinese" };
	int len = 0;
	stbtt_uint32 off;
	const unsigned char *p;

	off = stbtt_GetFontNameString(info, &len, 3, 1, 0x409, 1);   /* Unicode BMP */
	if (!off || len <= 0)
		off = stbtt_GetFontNameString(info, &len, 1, 0, 0, 1);   /* Macintosh ASCII */
	if (!off || len <= 0)
		return false;

	p = info->data + off;

	for (size_t k = 0; k < sizeof(needles) / sizeof(needles[0]); k++) {
		int nlen = (int)strlen(needles[k]);

		/* UTF-16BE */
		for (int i = 0; (i + 2 * nlen) <= len; i += 2) {
			int j = 0;

			while ((j < nlen) && (p[i + 2 * j] == 0) &&
			       (p[i + 2 * j + 1] == (unsigned char)needles[k][j]))
				j++;
			if (j == nlen)
				return true;
		}
		/* ASCII */
		for (int i = 0; (i + nlen) <= len; i++) {
			if (!memcmp(p + i, needles[k], (size_t)nlen))
				return true;
		}
	}
	return false;
}

static bool ttf_build(bsp_font_id_t id, uint8_t *data, uint32_t len)
{
	bsp_ttf_font_t *f = &s_ttf[id];
	int best_off = -1, a = 0, d = 0, g = 0;
	int ascent_px;

	(void)len;
	os_memset(f, 0, sizeof(*f));

	/* .ttc 里可能有多个 face：优先挑简体中文面（SC/CN），否则用第一个能解析的。
	 * stb 的这个 fork 同时支持 glyf 与 CFF（OTTO/OTF），所以 .ttf/.otf/.ttc
	 * 都能吃。 */
	for (int idx = 0; idx < 8; idx++) {
		int off = stbtt_GetFontOffsetForIndex(data, idx);

		if (off < 0)
			break;
		if (!stbtt_InitFont(&f->info, data, off))
			continue;
		if (best_off < 0)
			best_off = off;
		if (face_name_is_sc(&f->info)) {
			best_off = off;
			break;
		}
	}

	if (best_off < 0) {
		FONT_LOGE("stbtt_InitFont failed for all faces\r\n");
		return false;
	}
	stbtt_InitFont(&f->info, data, best_off);

	f->px = s_px[id];
	f->scale = stbtt_ScaleForPixelHeight(&f->info, (float)f->px);
	stbtt_GetFontVMetrics(&f->info, &a, &d, &g);

	ascent_px = (int)((float)a * f->scale + 0.5f);
	f->ascent_px = (int16_t)ascent_px;

	f->bmp = psram_malloc(FONT_BMP_MAX);
	if (!f->bmp) {
		FONT_LOGE("glyph buffer malloc failed\r\n");
		return false;
	}

	/* 行高固定成与内置字库一致（15/19），保证现有布局不跑版；
	 * base_line = 从行盒底部到基线的距离 = 行高 - 上升高度。
	 * TTF 的 ascent 可能大于行高（此时为负，字形上溢行盒），
	 * LVGL v8 允许负值，不要钳制——之前钳到 0 会把字形整体压低。 */
	f->font.get_glyph_dsc = ttf_get_glyph_dsc;
	f->font.get_glyph_bitmap = ttf_get_glyph_bitmap;
	f->font.line_height = s_line_h[id];
	f->font.base_line = (int16_t)(s_line_h[id] - ascent_px);

	f->font.subpx = LV_FONT_SUBPX_NONE;
	f->font.underline_position = -2;
	f->font.underline_thickness = 1;
	f->font.dsc = NULL;
	f->font.fallback = NULL;

	FONT_LOGI("ttf %dpx: ascent=%d line_h=%d base_line=%d scale=%.3f\r\n",
	          f->px, ascent_px, f->font.line_height, f->font.base_line,
	          (double)f->scale);
	return true;
}

/* ---------------- 从 SD 卡读 ---------------- */

static uint8_t *read_file(const char *path, uint32_t *out_len)
{
	static FIL f;
	FSIZE_t size;
	UINT br = 0;
	uint8_t *buf;

	if (f_open(&f, path, FA_READ) != FR_OK)
		return NULL;

	size = f_size(&f);
	if ((size == 0) || (size > FONT_MAX_BYTES)) {
		FONT_LOGW("%s: size %u out of range (limit %uKB) — use tools/mkfont.py\r\n",
		          path, (unsigned)size, (unsigned)(FONT_MAX_BYTES / 1024));
		f_close(&f);
		return NULL;
	}

	buf = psram_malloc((uint32_t)size);
	if (!buf) {
		FONT_LOGE("malloc %u failed\r\n", (unsigned)size);
		f_close(&f);
		return NULL;
	}

	if ((f_read(&f, buf, (UINT)size, &br) != FR_OK) || (br != size)) {
		FONT_LOGE("read %s failed (%u/%u)\r\n", path, (unsigned)br,
		          (unsigned)size);
		psram_free(buf);
		f_close(&f);
		return NULL;
	}
	f_close(&f);

	*out_len = (uint32_t)size;
	return buf;
}

/* ---------------- 对外 ---------------- */

void bsp_font_init(void)
{
	static const char *const cands[] = {
		"1:/font.ttf",
		"1:/fonts/font.ttf",
		"1:/FONT.TTF",
	};
	uint32_t t0 = rtos_get_time();
	uint8_t *data = NULL;
	uint32_t len = 0;

	if (s_ttf_ok)
		return;

	for (size_t i = 0; i < sizeof(cands) / sizeof(cands[0]); i++) {
		data = read_file(cands[i], &len);
		if (data) {
			os_strncpy(s_ttf_path, cands[i], sizeof(s_ttf_path) - 1);
			s_ttf_path[sizeof(s_ttf_path) - 1] = '\0';
			snprintf(s_source, sizeof(s_source), "sd:%s", cands[i]);
			break;
		}
	}

	if (!data) {
		FONT_LOGI("no sd ttf, use builtin bitmap fonts\r\n");
		return;
	}

	if (!ttf_build(BSP_FONT_SMALL, data, len) ||
	    !ttf_build(BSP_FONT_NORMAL, data, len)) {
		FONT_LOGE("ttf build failed, fallback to builtin\r\n");
		if (s_ttf[0].bmp) {
			psram_free(s_ttf[0].bmp);
			s_ttf[0].bmp = NULL;
		}
		if (s_ttf[1].bmp) {
			psram_free(s_ttf[1].bmp);
			s_ttf[1].bmp = NULL;
		}
		psram_free(data);
		os_strncpy(s_source, "builtin", sizeof(s_source) - 1);
		return;
	}

	s_ttf_data = data;
	s_ttf_len = len;
	s_sel[BSP_FONT_SMALL] = &s_ttf[BSP_FONT_SMALL].font;
	s_sel[BSP_FONT_NORMAL] = &s_ttf[BSP_FONT_NORMAL].font;
	s_ttf_ok = true;

	FONT_LOGI("use %s (%u bytes, %ums)\r\n", s_source, (unsigned)len,
	          (unsigned)(rtos_get_time() - t0));
}

const lv_font_t *bsp_font_get(bsp_font_id_t id)
{
	if (id >= BSP_FONT_ID_MAX)
		return &font_sc16;
	return s_sel[id];
}

bool bsp_font_is_ttf(void)
{
	return s_ttf_ok && (s_sel[BSP_FONT_NORMAL] == &s_ttf[BSP_FONT_NORMAL].font);
}

bool bsp_font_ttf_available(void)
{
	return s_ttf_ok;
}

void bsp_font_set_ttf(bool use_ttf)
{
	if (!s_ttf_ok)
		use_ttf = false;    /* 没加载成功就只能用内置 */

	s_sel[BSP_FONT_SMALL]  = use_ttf ? &s_ttf[BSP_FONT_SMALL].font  : &font_sc12;
	s_sel[BSP_FONT_NORMAL] = use_ttf ? &s_ttf[BSP_FONT_NORMAL].font : &font_sc16;

	if (use_ttf)
		snprintf(s_source, sizeof(s_source), "sd:%s", s_ttf_path);
	else
		os_strncpy(s_source, "builtin", sizeof(s_source) - 1);

	FONT_LOGI("font source -> %s\r\n", s_source);
}

const char *bsp_font_source(void)
{
	return s_source;
}
