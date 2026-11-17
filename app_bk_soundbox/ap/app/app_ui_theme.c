// Copyright 2026
// 极简 LVGL 主题实现 —— 见 app_ui_theme.h 顶部的说明（为什么要自己写）。
//
// 设计取舍：
//   * 只给"屏幕根对象"铺底色，其余对象一律不动。本工程 app_ui.c 里每个控件的
//     颜色、字号、边框都是显式设置的，主题只是兜底，不需要"好看的默认样式"。
//   * 字号统一用 LV_FONT_DEFAULT（= montserrat_14）。app_ui.c 的所有 label 都会
//     显式 lv_obj_set_style_text_font()，主题里的字号实际不会被用到，但 LVGL
//     要求 theme->font_* 非空（部分 API 会解引用），所以还是要填。
//   * 不引用任何 `lv_xxx_class`，这样不会通过 vtable 把未使用的 widget 拖进 Flash。

#include "app_ui_theme.h"

#include <string.h>

/* 与 app_ui.c 里的 C_BG 保持一致（0x0B0C10）。这里不能 include app_ui.h
 * ——那会把整个 UI 接口带进来，主题模块要保持独立。 */
#define THEME_BG_COLOR   0x0B0C10

typedef struct {
	lv_style_t scr;      /* 屏幕根对象：纯深底，无滚动条 */
} app_theme_styles_t;

static app_theme_styles_t *s_styles;
static lv_theme_t s_theme;
static bool s_inited;

static void theme_apply(lv_theme_t *th, lv_obj_t *obj)
{
	LV_UNUSED(th);

	/* 只处理屏幕根对象（没有父对象）；其它对象不注入任何样式。
	 * 判断方式跟 LVGL 自带主题一致：根对象的 parent 为 NULL。 */
	if (lv_obj_get_parent(obj) == NULL) {
		lv_obj_add_style(obj, &s_styles->scr, 0);
	}
}

void app_ui_theme_init(lv_disp_t *disp)
{
	if (disp == NULL)
		disp = lv_disp_get_default();
	if (disp == NULL) {
		return;
	}

	if (!s_inited) {
		/* 用 lv_mem_alloc 而不是 static：与 LVGL 自带主题的写法一致，
		 * 也避免 lv_style_t 被编译器当成常量放进 .data 额外占 Flash。 */
		s_styles = (app_theme_styles_t *)lv_mem_alloc(sizeof(app_theme_styles_t));
		if (s_styles == NULL) {
			return;
		}
		memset(s_styles, 0, sizeof(app_theme_styles_t));

		lv_style_init(&s_styles->scr);
		lv_style_set_bg_opa(&s_styles->scr, LV_OPA_COVER);
		lv_style_set_bg_color(&s_styles->scr, lv_color_hex(THEME_BG_COLOR));
		lv_style_set_border_width(&s_styles->scr, 0);
		lv_style_set_pad_all(&s_styles->scr, 0);
		lv_style_set_radius(&s_styles->scr, 0);

		s_inited = true;
	}

	s_theme.disp = disp;
	s_theme.color_primary = lv_color_hex(0x1DB954);      /* 与 C_ACCENT 一致 */
	s_theme.color_secondary = lv_color_hex(0x1DB954);
	s_theme.font_small = LV_FONT_DEFAULT;
	s_theme.font_normal = LV_FONT_DEFAULT;
	s_theme.font_large = LV_FONT_DEFAULT;
	s_theme.flags = 0;
	s_theme.user_data = NULL;
	s_theme.apply_cb = theme_apply;

	/* LVGL v8 没有 lv_disp_set_theme()，直接赋值字段；lv_disp_t 在 lvgl.h
	 * 里是完整定义，可以访问。 */
	disp->theme = &s_theme;

	/* 通知已有对象重新应用样式（此时 UI 还没创建，这里基本是空操作）。 */
	lv_obj_report_style_change(NULL);
}
