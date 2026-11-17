// Copyright 2026
// 极简 LVGL 主题（app_bk_pi 专用）
//
// 为什么自己写而不用 LVGL 自带的 lv_theme_default：
//   1. 体积。lv_theme_default.c 在本工程里占 **17.9KB**（text），而其中绝大部分
//      是给 chart / meter / colorwheel / calendar / keyboard / table / menu /
//      spinbox / roller / tabview ... 这些本工程**一个都没用**的控件做样式。
//   2. LVGL v8 的默认主题在 `lv_disp_drv_register()` 里被**无条件**初始化
//      （`lv_hal_disp.c:217` `#if LV_USE_THEME_DEFAULT`），所以只要
//      `LV_USE_THEME_DEFAULT=1` 就一定会被链接进来，应用层无法靠"我不调用"省掉。
//   3. 主题里的 `lv_xxx_class` 引用会把对应 widget 的实现一起拉进 Flash
//      —— 这是"没用到的控件也占空间"的主要传导路径。
//
// 本主题只覆盖本工程真正用到的 4 类对象：obj / btn / label / image。
// 其余对象（含未来新增）不加任何默认样式，需要样式的地方由 app_ui.c 显式设置，
// 那本来就是本工程的写法（所有颜色/字号都在 app_ui.c 里写死）。
//
// 注册方式：`lv_disp_drv_register()` 之后调用 `app_ui_theme_init(disp)`。
// 注意 LVGL v8 没有 `lv_disp_set_theme()` 这个 API，只能直接赋值
// `disp->theme`（`lv_disp_t` 在 lvgl.h 里是完整定义，可以直接访问该字段）。

#ifndef __APP_UI_THEME_H__
#define __APP_UI_THEME_H__

#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

/* 给显示设备装上本工程的极简主题。disp 传 NULL 表示用默认显示。 */
void app_ui_theme_init(lv_disp_t *disp);

#ifdef __cplusplus
}
#endif

#endif /* __APP_UI_THEME_H__ */
