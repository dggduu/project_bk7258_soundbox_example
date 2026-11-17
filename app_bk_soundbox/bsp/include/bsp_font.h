// Copyright 2026
// 双来源字体：SD 卡 TTF（优先）/ 内置点阵字库（兜底）
//
// 分层：
//   1) SD 卡上的字体（`1:/font.ttf` 或 `1:/fonts/font.ttf`，.ttf/.otf/.ttc 皆可），
//      运行时用 stb_truetype 栅格化，用户想换字体只要换文件；
//   2) 内置 font_sc16 / font_sc12（中 + 日 + 英，随固件烧写，不依赖 SD 卡）。
//
// 约束：stb_truetype 只支持 **TrueType 轮廓（glyf）**，不支持 OTF/CFF
//       （思源/Noto 的 .otf、.ttc 取 CFF face 都不行，需要 .ttf 版本）。
//       字体文件上限 4MB（整份读进 PSRAM，stbtt 要求全量数据在内存）。

#pragma once

#include <stdbool.h>
#include "lvgl.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	BSP_FONT_SMALL = 0,    /* 12px：歌词译文、状态栏、列表尺寸 */
	BSP_FONT_NORMAL,       /* 16px：标题、歌名、当前歌词、设置项 */
	BSP_FONT_ID_MAX,
} bsp_font_id_t;

/**
 * @brief 初始化字体来源：先试 SD 卡 TTF，失败则用内置字库。
 *
 * 必须在 SD 卡挂载之后、创建 UI 之前调用（main 里 app_mode_init 之后）。
 */
void bsp_font_init(void);

/** @brief 取字体对象；未初始化时返回内置字库。 */
const lv_font_t *bsp_font_get(bsp_font_id_t id);

/** @brief 当前是否用的 SD 卡 TTF。 */
bool bsp_font_is_ttf(void);

/** @brief SD 卡 TTF 是否加载成功（决定切换开关是否有效）。 */
bool bsp_font_ttf_available(void);

/**
 * @brief 运行时切换字体来源（设置页"字体"项用）。
 *
 * TTF 渲染有问题时可切回内置点阵字库；use_ttf=true 仅在
 * bsp_font_ttf_available() 时生效。切换只改内部指针，UI 下一帧即生效。
 */
void bsp_font_set_ttf(bool use_ttf);

/** @brief 字体来源描述，日志/设置页用："sd:1:/font.ttf" 或 "builtin"。 */
const char *bsp_font_source(void);

#ifdef __cplusplus
}
#endif
