// Copyright 2026
// LVGL 界面实现（360x360 圆形屏，无触摸）
//
// ============================ 交互模型 ============================
//
// 新屏没有触摸，**全部操作依赖 ADC 分压键盘的四个实体键**。按键语义
// 按"当前页面"复用，不设全局固定的功能键（只有 KEY4 的长按是全局的）：
//
//   KEY1  短按  上一项 / 快退 10s（播放页）
//         长按  上一首（播放页）/ 音量 -5（其它页）
//         连发  音量 -2（非播放页，长按后持续按住）
//   KEY2  短按  确认 / 播放暂停（播放页）/ 打开文件 / 切换模式 / 调整设置项
//         长按  摇一摇开关（播放页）/ 蓝牙配对（其它页）
//   KEY3  短按  下一项 / 快进 10s（播放页）
//         长按  下一首（播放页）/ 音量 +5（其它页）
//         连发  音量 +2（非播放页）
//   KEY4  短按  切换到下一个 Tab（播放 → 文件 → 模式 → 设置 → 播放）
//         长按  循环切换播放模式（蓝牙 / USB / SD）
//
// Tab 只能单向循环。四个 Tab 最多按 4 次回到原处，比加一个"反向键"
// 简单 —— 四键里已经没有空位了。
//
// 熄屏（60s 无操作）期间第一次按键只负责亮屏，不执行动作，防误触。
//
// ============================ 布局约束 ============================
//
// 圆屏四角物理上不发光，所有坐标必须落在圆内。几何量集中在文件开头的
// "屏幕几何"段，改布局前先看那里的可见宽度表。

#include "app_ui.h"

#include <string.h>
#include <strings.h>
#include <stdio.h>

#include <os/os.h>
#include <os/str.h>
#include <os/mem.h>
#include <components/log.h>

#include "lvgl.h"

#include "app_state.h"
#include "bt_cover_art.h"
#include "app_mode.h"
#include "app_bt.h"
#include "app_sd.h"
#include "bsp.h"
#include "bsp_font.h"

#define TAG "app_ui"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

extern const lv_img_dsc_t img_cover_default;

/* ============================ 屏幕几何 ============================
 *
 * 360x360 圆形屏（ST77916）。**所有布局必须避开四个角** —— 圆外区域
 * 物理上不发光，放东西进去等于白放（而且 LVGL 还以为它在显示）。
 *
 * 可见区判定：圆心 (180,180)、半径 180，对任意 y 有
 *     可见半宽 half_w(y) = sqrt(180^2 - (y-180)^2)
 *     可见范围 x ∈ [180 - half_w, 180 + half_w]
 *
 * 关键高度上的实测半宽：
 *   y=20  -> half_w 78.4   x∈[102,258]（宽 157）
 *   y=36  -> half_w 108.0  x∈[ 72,288]（宽 216）
 *   y=56  -> half_w 118.4  x∈[ 62,298]（宽 236）
 *   y=76  -> half_w 129.0  x∈[ 51,309]（宽 258）
 *   y=180 -> half_w 180.0  x∈[  0,360]（宽 360，最宽）
 *   y=284 -> half_w 147.6  x∈[ 32,328]（宽 295）
 *   y=298 -> half_w 134.0  x∈[ 46,314]（宽 268）
 *   y=324 -> half_w 108.0  x∈[ 72,288]（宽 216）
 *
 * 据此定三条"安全带"，所有页面只用这三条的几何量：
 *   ① 状态栏   y 36..56   宽 216（x 72..288）
 *   ② 内容区   y 60..292  宽 256（x 52..308）—— 上下都留了余量
 *   ③ Tab 栏   y 298..324 宽 216（x 72..288）
 *
 * 内容区 x 取 52..308 的依据：y=60 时可见 x∈[46,314]、y=292 时
 * x∈[43,317]，内缩 6px 作为视觉呼吸，全程不越界。
 */
#define SCR_W       360
#define SCR_H       360
#define SCR_CX      (SCR_W / 2)

#define STATUS_Y    36
#define STATUS_H    20
#define STATUS_W    216
#define STATUS_X    ((SCR_W - STATUS_W) / 2)

/* 状态栏右侧：左半段是文字（模式/音量/电量），最右边是自绘的电量图标。
 *
 * 为什么要自绘图标：内置 font_sc12 是 CJK 子集点阵，LV_SYMBOL_* 落在
 * FontAwesome 私有区（U+F000 以上），字库里没有字形，会渲染成方框；
 * SD 卡 TTF 也不保证带电池字形。用 3 个 LVGL 图元拼一个 17x10 的空心电池
 * + 正极端 + 按百分比伸缩的填充条，零字体依赖，还省 Flash。
 *
 * 宽度预算（STATUS_W=216）：
 *   左侧模式文字 104 + 右侧文字 86 + 图标(17+3) + 6px 间隙 = 216
 * "VOL 100 100%" 用 font_sc12 量约 79px，放得下 86，且和图标留 6px 呼吸。
 */
#define STATUS_LEFT_W   104
#define BAT_BODY_W      17
#define BAT_BODY_H      10
#define BAT_CAP_W       3
#define BAT_CAP_H       4
#define BAT_FILL_W      13      /* 电池内腔 15px，左右各留 1px */
#define BAT_FILL_H      6
#define BAT_ICON_X      (STATUS_W - BAT_CAP_W - BAT_BODY_W)
#define STATUS_RIGHT_W  (BAT_ICON_X - 6 - STATUS_LEFT_W)
#define BAT_LOW_PCT     20      /* <=20% 用红色提示 */

#define CONTENT_Y   60
#define CONTENT_W   256
#define CONTENT_X   ((SCR_W - CONTENT_W) / 2)
#define CONTENT_H   232                 /* 60..292 */

#define TAB_Y       298
#define TAB_H       26
#define TAB_W       216                 /* x 72..288，4 格各 54 */
#define TAB_X       ((SCR_W - TAB_W) / 2)
#define TAB_ITEM_W  (TAB_W / 4)

/* 封面：圆屏正中做"圆相框"。
 *
 * 【为什么图片本身只做 104】封面数据源的尺寸是硬约束：
 *   - SD 卡封面：app_sd.c 的 s_cover 是 APP_SD_COVER_SIZE(104) 见方，
 *     app_sd_copy_cover() 只会把左上 min(请求, 104) 的区域拷出来，
 *     请求 148 的话右下 44x44 是未初始化数据，显示出来是一块脏图。
 *   - 蓝牙封面：bt_cover_art 同样固定 104x104。
 * 所以 COVER_SIZE 必须保持 104；圆屏观感靠**加大圆形底卡**来做，
 * 底卡 140 圆 + 封面 104 居中，视觉上就是一枚圆相框。
 *
 * 垂直预算（内容区 60..292，共 232px）：
 *   底卡 140  62..202
 *   标题  20  204..224
 *   艺术家 16 224..240
 *   歌词  22  242..264
 *   进度条 4  268..272
 *   时间  14  272..286
 *   余量      286..292
 */
#define COVER_SIZE  104
#define COVER_BG    140
#define COVER_Y     62

#define C_BG        0x0B0C10
#define C_CARD      0x171922
#define C_CARD_SEL  0x252A38
#define C_ACCENT    0x1DB954
#define C_LOW       0xE05252     /* 低电量（状态栏电量数字/图标） */
#define C_TEXT      0xF2F2F2
#define C_DIM       0x8A8A93
#define C_DIMMER    0x4A4A55

/* 字体来源在运行时决定：SD 卡 TTF 优先，没有则用内置点阵字库
 * （见 bsp_font.c）。这里用指针而不是编译期宏，切字体不用改 UI 代码。 */
static const lv_font_t *s_f16;
static const lv_font_t *s_f12;
#define F16         (s_f16)
#define F12         (s_f12)

/* 【Flash 优化】原先这里单独挂在 lv_font_montserrat_14 上，只用来画数字和
 * LV_SYMBOL 图标。为了省掉那份 montserrat_14 点阵（只有被引用才会链接），
 * 现在统一走 F16（内置 font_sc16 或 SD 卡 TTF）；lv_conf.h 的
 * LV_FONT_DEFAULT 也一并改成了 font_sc16。
 *
 * 代价：内置 font_sc16 是 1bpp 点阵，ASCII 区不像 montserrat 那样等宽，
 * 而且 LV_SYMBOL 的 FontAwesome 图标在 CJK 子集里**没有**对应字形，
 * 会显示为方框 —— 这一点必须靠 SD 卡 TTF（含图标字形）或改成中文字/自绘
 * 图形来解决，见 app_ui.c 里各图标的用法。 */
#define F_NUM       (s_f16)

#define KEY_NONE    0
#define KEY_1       1
#define KEY_2       2
#define KEY_3       3
#define KEY_4       4
#define EVT_DOWN    0
#define EVT_UP      1
#define EVT_LONG    2
#define EVT_REPEAT  3

typedef struct {
	uint8_t key;
	uint8_t evt;
} ui_key_t;

/* ============================ 状态 ============================ */

static lv_obj_t *s_scr;

/* 状态栏 */
static lv_obj_t *s_status_left;
static lv_obj_t *s_status_right;
static lv_obj_t *s_bat_body;      /* 电池外壳（带 1px 边框） */
static lv_obj_t *s_bat_cap;       /* 正极小凸起 */
static lv_obj_t *s_bat_fill;      /* 电量填充条，宽度随百分比变化 */
static bool s_bat_icon_valid;
static uint8_t s_bat_icon_pct = 0xFF;   /* 0xFF = 强制第一帧刷新 */
static uint32_t s_status_right_color = 0xFFFFFFFF;  /* 缓存，避免每 100ms 刷样式 */

/* 播放页 */
static lv_obj_t *s_page_play;
static lv_obj_t *s_cover_bg;
static lv_obj_t *s_cover_img;
static lv_obj_t *s_title;
static lv_obj_t *s_codec_tag;    /* 封面右上角格式胶囊 */
static lv_obj_t *s_codec_lbl;
static lv_obj_t *s_artist;
static lv_obj_t *s_lyric_prev;
static lv_obj_t *s_lyric_cur;
static lv_obj_t *s_lyric_sub;
static lv_obj_t *s_progress;
static lv_obj_t *s_time_cur;
static lv_obj_t *s_time_total;
static lv_obj_t *s_play_icon;

/* ==================== 播放状态图标（自绘位图） ====================
 *
 * 【为什么不用 LV_SYMBOL_PLAY / LV_SYMBOL_PAUSE】
 * 本工程 `LV_FONT_DEFAULT` 是 font_sc16，一个 CJK 子集点阵字库 ——
 * LV_SYMBOL_* 落在 FontAwesome 私有区（U+F000 以上），字库里没有这些
 * 字形，渲染出来是一对空方框。只有切到 SD 卡的 TTF 字体（FreeType）
 * 才有，而字体来源默认是 builtin。
 *
 * 【为什么也不用 GB2312 符号】
 * GBK 的 A1 符号区有 ▲ → ● ■ ‖，但**没有右指实心三角 ▶**(U+25B6)，
 * 用 ▲ 表示播放方向不对，用 → 又像"下一首"。
 *
 * 所以直接画 12x14 的 ALPHA_8BIT 位图：0=透明，255=不透明，
 * 颜色由 `style_img_recolor` 决定（跟随主题强调色）。
 * 两张图共 336 字节，比任何字体方案都省。
 *
 * 数据是算出来的，不是手搓的：播放三角第 y 行宽度 = 1 + 2*min(y, 13-y)，
 * 左边缘固定在 x=2；暂停是 x=2..3 与 x=6..7 两条 2px 竖条。 */
/* 播放：12x14 右指三角 */
static const uint8_t s_icon_play_map[120] = {
	  0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   /* y=0 */
	  0, 255, 255, 255,   0,   0,   0,   0,   0,   0,   /* y=1 */
	  0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   /* y=2 */
	  0, 255, 255, 255, 255, 255, 255, 255,   0,   0,   /* y=3 */
	  0, 255, 255, 255, 255, 255, 255, 255, 255,   0,   /* y=4 */
	  0, 255, 255, 255, 255, 255, 255, 255, 255,   0,   /* y=5 */
	  0, 255, 255, 255, 255, 255, 255, 255, 255,   0,   /* y=6 */
	  0, 255, 255, 255, 255, 255, 255, 255, 255,   0,   /* y=7 */
	  0, 255, 255, 255, 255, 255, 255, 255,   0,   0,   /* y=8 */
	  0, 255, 255, 255, 255, 255,   0,   0,   0,   0,   /* y=9 */
	  0, 255, 255, 255,   0,   0,   0,   0,   0,   0,   /* y=10 */
	  0, 255,   0,   0,   0,   0,   0,   0,   0,   0,   /* y=11 */
};

/* 暂停：12x14 双竖条 */
static const uint8_t s_icon_pause_map[120] = {
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=0 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=1 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=2 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=3 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=4 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=5 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=6 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=7 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=8 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=9 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=10 */
	  0, 255, 255,   0,   0, 255, 255,   0,   0,   0,   /* y=11 */
};

static const lv_img_dsc_t s_icon_play = {
	.header = { .cf = LV_IMG_CF_ALPHA_8BIT, .w = 10, .h = 12 },
	.data_size = sizeof(s_icon_play_map),
	.data = s_icon_play_map,
};

static const lv_img_dsc_t s_icon_pause = {
	.header = { .cf = LV_IMG_CF_ALPHA_8BIT, .w = 10, .h = 12 },
	.data_size = sizeof(s_icon_pause_map),
	.data = s_icon_pause_map,
};
static lv_obj_t *s_vol_label;

/* 文件页 */
static lv_obj_t *s_page_files;
static lv_obj_t *s_files_path_lbl;
static lv_obj_t *s_files_page_lbl;      /* 右侧页码 "2/5" */
static lv_obj_t *s_files_rows[7];
static lv_obj_t *s_files_names[7];
static lv_obj_t *s_files_sizes[7];
static lv_obj_t *s_files_hint;

/* 模式页 */
static lv_obj_t *s_page_mode;
static lv_obj_t *s_mode_cards[APP_MODE_MAX];
static lv_obj_t *s_mode_names[APP_MODE_MAX];
static lv_obj_t *s_mode_status[APP_MODE_MAX];

/* 设置页 */
static lv_obj_t *s_page_settings;
static lv_obj_t *s_set_rows[6];
static lv_obj_t *s_set_names[6];
static lv_obj_t *s_set_values[6];
static lv_obj_t *s_set_hint;

/* Tab 栏 */
static lv_obj_t *s_tab_icons[4];
static lv_obj_t *s_tab_texts[4];
static lv_obj_t *s_tab_bars[4];

/* Toast */
static lv_obj_t *s_toast;
static uint32_t s_toast_seq;
static uint32_t s_toast_until;

/* 输入队列 */
static volatile ui_key_t s_key_q[16];
static volatile uint8_t s_key_wr;
static volatile uint8_t s_key_rd;

/* 当前 UI 选择 */
static app_tab_t s_tab = APP_TAB_PLAY;
static uint8_t s_mode_sel;
static uint8_t s_set_sel;

#define FILES_VISIBLE 7
static char s_files_path[APP_PATH_LEN] = "1:/";
static app_sd_entry_t s_files_entries[APP_SD_MAX_ENTRIES];
static int s_files_count;
static int s_files_sel;
static int s_files_top;
static app_tab_t s_last_tab = APP_TAB_MAX;

/* 封面 */
/* 封面缓冲。【2026-11 从静态数组改 PSRAM 动态分配】
 *
 * 原来是 `static uint16_t s_cover_buf[COVER_SIZE*COVER_SIZE]`（104x104x2
 * = 21632B），落在 .bss，吃的是**内部 RAM**。SD 播放日志显示内部堆只剩
 * 6KB 时音频 pipeline 全部 malloc fail，这块 21.6KB 挪进 PSRAM 就是
 * 最便宜的堆救援：对 PSRAM（4MB 级）无关痛痒，对内部堆是 3 倍增量。
 * 分配失败时 s_cover_buf 保持 NULL，update_cover() 自动回退默认图标。 */
static uint16_t *s_cover_buf;
static lv_img_dsc_t s_cover_dsc;
static uint32_t s_last_cover_gen = 0xFFFFFFFF;
static uint32_t s_last_bt_cover_gen = 0xFFFFFFFF;
static app_mode_t s_last_cover_mode = APP_MODE_MAX;

/* 状态刷新节流 */
static uint32_t s_last_refresh;
static app_state_t s_last;
static bool s_need_redraw;

/* ============================ 小工具 ============================ */

static lv_obj_t *make_obj(lv_obj_t *parent)
{
	lv_obj_t *obj = lv_obj_create(parent);

	lv_obj_remove_style_all(obj);
	lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
	lv_obj_set_style_border_width(obj, 0, 0);
	lv_obj_set_style_pad_all(obj, 0, 0);
	lv_obj_clear_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
	return obj;
}

static lv_obj_t *make_label(lv_obj_t *parent, const char *text,
                            lv_coord_t x, lv_coord_t y, lv_coord_t w, lv_coord_t h,
                            const lv_font_t *font, uint32_t color)
{
	lv_obj_t *lbl = lv_label_create(parent);

	lv_label_set_text(lbl, text ? text : "");
	lv_obj_set_pos(lbl, x, y);
	lv_obj_set_size(lbl, w, h);
	lv_obj_set_style_text_font(lbl, font, 0);
	lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
	lv_label_set_long_mode(lbl, LV_LABEL_LONG_CLIP);
	return lbl;
}

static void set_text(lv_obj_t *lbl, const char *text)
{
	const char *old;

	if (!lbl)
		return;

	old = lv_label_get_text(lbl);
	if (!text)
		text = "";
	if (old && !strcmp(old, text))
		return;

	lv_label_set_text(lbl, text);
}

static void set_text_color(lv_obj_t *lbl, uint32_t color)
{
	if (lbl)
		lv_obj_set_style_text_color(lbl, lv_color_hex(color), 0);
}

/* ---------------------------------------------------------------- 歌词排版
 *
 * 1. 单行过长时自动降一档字体（16px -> 12px），仍放不下就交给
 *    LV_LABEL_LONG_DOT 打省略号，不再靠 WRAP 折行后裁掉半行；
 * 2. 歌词统一左对齐（与卡拉 OK 的阅读习惯一致）；
 * 3. 双语歌词：把 “原文 / 译文”、含换行、含 “|” 的一行拆成
 *    主行（大字）+ 副行（小字）两行显示。
 */

/* 方括号里的内容是不是 LRC 标签/时间戳（[00:12.34] [ti:歌名] [offset:500]…） */
static bool bracket_is_tag(const char *s, size_t n)
{
	static const char *const tags[] = {
		"ti:", "ar:", "al:", "by:", "offset:", "re:", "ve:", "kana:",
	};
	bool only_digits = true;
	bool has_colon = false;

	if ((n == 0) || (n > 12))
		return false;

	for (size_t i = 0; i < n; i++) {
		char c = s[i];

		if (c == ':') {
			has_colon = true;
			continue;
		}
		if ((c == '.') || (c == '-') || (c == '/'))
			continue;
		if ((c >= '0') && (c <= '9'))
			continue;
		only_digits = false;
		break;
	}
	if (only_digits && has_colon)
		return true;                    /* [00:12.34] / [1:23] */

	for (size_t i = 0; i < sizeof(tags) / sizeof(tags[0]); i++) {
		size_t tlen = strlen(tags[i]);

		if ((n > tlen) && !strncasecmp(s, tags[i], tlen))
			return true;
	}
	return false;
}

/* 剥掉行首残留的标签：LRC 解析对不规范格式（[mm:ss:xx]、一行多组前缀、
 * 缺右括号的残缺行）会漏掉，显示前再清一层。 */
static const char *lyric_skip_tags(const char *p)
{
	size_t n;

	if (!p)
		return "";
	if (((uint8_t)p[0] == 0xEF) && ((uint8_t)p[1] == 0xBB) &&
	    ((uint8_t)p[2] == 0xBF))
		p += 3;

	for (;;) {
		const char *close;

		while ((*p == ' ') || (*p == '\t') || (*p == '\r') || (*p == '\n'))
			p++;
		if (*p != '[')
			break;
		close = strchr(p, ']');
		if (!close)
			break;
		n = (size_t)(close - p) - 1;
		if (!bracket_is_tag(p + 1, n))
			break;
		p = close + 1;
	}
	return p;
}

/* 找一个两行歌词的分隔点，找不到返回 NULL；*sep_len 返回分隔符字节数 */
static const char *lyric_split_point(const char *s, size_t *sep_len)
{
	static const struct {
		const char *sep;
		size_t len;
	} seps[] = {
		{ "\n", 1 },
		{ "\r", 1 },
		/* 有些 LRC/标签歌词用**字面**两个字符 \n 表示换行
		 * （下载来的 txt→lrc 特别常见），必须一起当分隔符 */
		{ "\\n", 2 },
		{ "\\r", 2 },
		{ " / ", 3 },
		{ " | ", 3 },
		{ "|", 1 },
		{ "\xEF\xBC\x8F", 3 },   /* 全角斜杠 ／ */
		/* 日/中双语 LRC 常用「排版空格」把原文与译文放在同一行。
		 * 普通空格不能当分隔符（歌词内部本来就有空格），这里只认
		 * 这些在正文里几乎不出现的 Unicode 空格，且排在显式换行符之后。 */
		{ "\xE2\x80\x89", 3 },   /* U+2009 THIN SPACE（内嵌 USLT 实测） */
		{ "\xE2\x80\x82", 3 },   /* U+2002 EN SPACE */
		{ "\xE2\x80\x83", 3 },   /* U+2003 EM SPACE */
		{ "\xE2\x80\x86", 3 },   /* U+2006 SIX-PER-EM SPACE */
		{ "\xC2\xA0", 2 },       /* U+00A0 NO-BREAK SPACE */
	};

	for (size_t i = 0; i < sizeof(seps) / sizeof(seps[0]); i++) {
		const char *p = strstr(s, seps[i].sep);

		if (p) {
			*sep_len = seps[i].len;
			return p;
		}
	}
	return NULL;
}

static void lyric_split(const char *in, char *main_line, size_t main_len,
                        char *sub_line, size_t sub_len)
{
	const char *sep;
	size_t sep_len = 0;
	size_t n;

	if (main_line && main_len)
		main_line[0] = '\0';
	if (sub_line && sub_len)
		sub_line[0] = '\0';
	if (!in || !in[0])
		return;

	in = lyric_skip_tags(in);
	if (!in[0])
		return;

	sep = lyric_split_point(in, &sep_len);
	if (!sep) {
		snprintf(main_line, main_len, "%s", in);
		return;
	}

	n = (size_t)(sep - in);
	if (n >= main_len)
		n = main_len - 1;
	memcpy(main_line, in, n);
	main_line[n] = '\0';

	sep += sep_len;
	while (*sep == ' ' || *sep == '\t')
		sep++;
	if (sub_line && sub_len)
		snprintf(sub_line, sub_len, "%s", sep);

	/* 去掉主行行尾空白 */
	n = strlen(main_line);
	while (n > 0 && (main_line[n - 1] == ' ' || main_line[n - 1] == '\t'))
		main_line[--n] = '\0';
}

/* 按可用宽度选字体：超宽就降一档，再超宽由 LONG_DOT 省略 */
static void set_text_fit(lv_obj_t *lbl, const char *text, const lv_font_t *big,
                         const lv_font_t *small, lv_coord_t width)
{
	const lv_font_t *use = big;

	if (!lbl)
		return;
	if (!text)
		text = "";

	if (text[0] && (lv_txt_get_width(text, (uint32_t)strlen(text), big, 0,
	                                 LV_TEXT_FLAG_NONE) > width))
		use = small;

	if (lv_obj_get_style_text_font(lbl, 0) != use)
		lv_obj_set_style_text_font(lbl, use, 0);
	set_text(lbl, text);
}

static void fmt_time(uint32_t ms, char *buf, size_t len)
{
	uint32_t sec = ms / 1000;

	snprintf(buf, len, "%u:%02u", (unsigned)(sec / 60), (unsigned)(sec % 60));
}

static const char *mode_name(app_mode_t m)
{
	switch (m) {
	case APP_MODE_BT:  return "蓝牙";
	case APP_MODE_USB: return "USB";
	case APP_MODE_SD:  return "SD 卡";
	case APP_MODE_USB_MIC: return "USB 麦";
	default:           return "?";
	}
}

static void ui_rebuild(void);   /* settings_activate 里切字体时用到 */

static const char *mode_icon(app_mode_t m)
{
	switch (m) {
	case APP_MODE_BT:  return LV_SYMBOL_BLUETOOTH;
	case APP_MODE_USB: return LV_SYMBOL_USB;
	case APP_MODE_SD:  return LV_SYMBOL_SD_CARD;
	default:           return LV_SYMBOL_AUDIO;
	}
}

static const app_track_t *active_track(const app_state_t *st)
{
	switch (st->mode) {
	case APP_MODE_BT:  return &st->bt;
	case APP_MODE_USB: return &st->usb;
	case APP_MODE_SD:  return &st->sd;
	case APP_MODE_USB_MIC: return &st->usb;
	default:           return &st->bt;
	}
}

/* ============================ 输入队列 ============================ */

void app_ui_post_key(uint8_t key, uint8_t evt)
{
	uint8_t wr = s_key_wr;
	uint8_t next = (uint8_t)((wr + 1) & 0x0F);

	if (next == s_key_rd)
		return;

	s_key_q[wr].key = key;
	s_key_q[wr].evt = evt;
	s_key_wr = next;
}

static bool pop_key(ui_key_t *out)
{
	uint8_t rd = s_key_rd;

	if (rd == s_key_wr)
		return false;

	*out = s_key_q[rd];
	s_key_rd = (uint8_t)((rd + 1) & 0x0F);
	return true;
}

/* ============================ Tab / 页面 ============================ */

static void refresh_files(void);

static void update_tabs(void)
{
	for (int i = 0; i < 4; i++) {
		bool sel = (i == (int)s_tab);

		set_text_color(s_tab_icons[i], sel ? C_ACCENT : C_DIMMER);
		set_text_color(s_tab_texts[i], sel ? C_ACCENT : C_DIM);
		if (s_tab_bars[i])
			lv_obj_set_style_bg_opa(s_tab_bars[i], sel ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
	}
}

static void show_tab(app_tab_t tab, bool force)
{
	if ((tab == s_tab) && !force)
		return;

	s_tab = tab;
	app_state_set_tab(tab);

	lv_obj_add_flag(s_page_play, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(s_page_files, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(s_page_mode, LV_OBJ_FLAG_HIDDEN);
	lv_obj_add_flag(s_page_settings, LV_OBJ_FLAG_HIDDEN);

	switch (tab) {
	case APP_TAB_PLAY:
		lv_obj_clear_flag(s_page_play, LV_OBJ_FLAG_HIDDEN);
		break;
	case APP_TAB_FILES:
		lv_obj_clear_flag(s_page_files, LV_OBJ_FLAG_HIDDEN);
		refresh_files();
		break;
	case APP_TAB_MODE:
		lv_obj_clear_flag(s_page_mode, LV_OBJ_FLAG_HIDDEN);
		s_mode_sel = (uint8_t)app_state_mode();
		break;
	case APP_TAB_SETTINGS:
		lv_obj_clear_flag(s_page_settings, LV_OBJ_FLAG_HIDDEN);
		break;
	default:
		break;
	}

	update_tabs();
	s_need_redraw = true;
}

/* ============================ 播放页 ============================ */

static void build_play_page(void)
{
	s_page_play = make_obj(s_scr);
	lv_obj_set_pos(s_page_play, 0, CONTENT_Y);
	lv_obj_set_size(s_page_play, SCR_W, CONTENT_H);

	/* 封面底卡 + 音符占位：即使图片解码失败/蓝牙没有封面，也不会是一块空白。
	 * 底卡是 140 的圆形，图片 104 居中 —— 圆屏上形成"圆相框"的观感。 */
	s_cover_bg = make_obj(s_page_play);
	lv_obj_set_pos(s_cover_bg, (SCR_W - COVER_BG) / 2, COVER_Y - CONTENT_Y);
	lv_obj_set_size(s_cover_bg, COVER_BG, COVER_BG);
	lv_obj_set_style_radius(s_cover_bg, COVER_BG / 2, 0);
	lv_obj_set_style_bg_color(s_cover_bg, lv_color_hex(C_CARD), 0);
	lv_obj_set_style_bg_opa(s_cover_bg, LV_OPA_COVER, 0);
	{
		lv_obj_t *note = make_label(s_cover_bg, "♪", 0, 46, COVER_BG, 48,
		                            F16, C_DIMMER);

		lv_obj_set_style_text_align(note, LV_TEXT_ALIGN_CENTER, 0);
	}

	s_cover_img = lv_img_create(s_page_play);
	lv_img_set_src(s_cover_img, &img_cover_default);
	lv_obj_set_pos(s_cover_img, (SCR_W - COVER_SIZE) / 2,
	               COVER_Y - CONTENT_Y + (COVER_BG - COVER_SIZE) / 2);

	/* 封面右上角的格式胶囊（FLAC / WAV / MP3 / SBC / PCM）。
	 * 贴在底卡右上沿内侧，不越出圆形可见区。 */
	{
		lv_obj_t *tag = make_obj(s_page_play);

		lv_obj_set_pos(tag, SCR_CX + 30, COVER_Y - CONTENT_Y + 14);
		lv_obj_set_size(tag, 52, 18);
		lv_obj_set_style_radius(tag, 9, 0);
		lv_obj_set_style_bg_color(tag, lv_color_hex(0x2A2E3A), 0);
		lv_obj_set_style_bg_opa(tag, LV_OPA_80, 0);

		s_codec_lbl = make_label(tag, "", 0, 1, 52, 16, F12, C_DIM);
		lv_obj_set_style_text_align(s_codec_lbl, LV_TEXT_ALIGN_CENTER, 0);
		lv_obj_add_flag(tag, LV_OBJ_FLAG_HIDDEN);
		lv_obj_set_user_data(tag, s_codec_lbl);
		s_codec_tag = tag;
	}

	/* ---- 下方文字区 ----
	 * 【坐标基准】s_page_play 的宽是 SCR_W(360)，位置在 (0, CONTENT_Y)，
	 * 所以子对象的 x 是**屏幕绝对 x**，居中要用 (SCR_W - w) / 2，
	 * 不能用 CONTENT_W —— 内容区只是"安全内容列"，不是父对象宽度。
	 * y 则是相对页面的，等于绝对 y - CONTENT_Y。
	 */
	s_title = make_label(s_page_play, "等待播放", (SCR_W - 240) / 2, 144,
	                     240, 20, F16, C_TEXT);
	lv_obj_set_style_text_align(s_title, LV_TEXT_ALIGN_CENTER, 0);
	lv_label_set_long_mode(s_title, LV_LABEL_LONG_DOT);

	s_artist = make_label(s_page_play, "", (SCR_W - 240) / 2, 164,
	                      240, 16, F12, C_DIM);
	lv_obj_set_style_text_align(s_artist, LV_TEXT_ALIGN_CENTER, 0);
	lv_label_set_long_mode(s_artist, LV_LABEL_LONG_DOT);

	/* 当前歌词一行（居中）。圆屏没有空间放"上一句/译文"两行，
	 * 那两行在 240x320 上是 46px 的垂直预算，这里要留给封面和进度条。 */
	s_lyric_cur = make_label(s_page_play, "选择模式开始播放", (SCR_W - 240) / 2,
	                         182, 240, 22, F16, C_ACCENT);
	lv_obj_set_style_text_align(s_lyric_cur, LV_TEXT_ALIGN_CENTER, 0);
	lv_label_set_long_mode(s_lyric_cur, LV_LABEL_LONG_DOT);

	/* 这两个标签保留对象（上层 update 逻辑会 set_text），但在圆屏上
	 * 没有位置显示 —— 直接隐藏，避免 layout 改动时漏掉。 */
	s_lyric_prev = make_label(s_page_play, "", (SCR_W - 240) / 2, 182,
	                          240, 16, F12, C_DIMMER);
	lv_obj_add_flag(s_lyric_prev, LV_OBJ_FLAG_HIDDEN);
	s_lyric_sub = make_label(s_page_play, "", (SCR_W - 240) / 2, 182,
	                         240, 16, F12, C_DIM);
	lv_obj_add_flag(s_lyric_sub, LV_OBJ_FLAG_HIDDEN);

	/* 进度条：y=208（绝对 268），此处可见宽度 302，216 宽居中很安全 */
	s_progress = lv_bar_create(s_page_play);
	lv_obj_set_pos(s_progress, (SCR_W - 216) / 2, 208);
	lv_obj_set_size(s_progress, 216, 4);
	lv_bar_set_range(s_progress, 0, 1000);
	lv_bar_set_value(s_progress, 0, LV_ANIM_OFF);
	lv_obj_set_style_bg_color(s_progress, lv_color_hex(0x2A2A33), 0);
	lv_obj_set_style_bg_opa(s_progress, LV_OPA_COVER, 0);
	lv_obj_set_style_radius(s_progress, 2, 0);
	lv_obj_set_style_bg_color(s_progress, lv_color_hex(C_ACCENT), LV_PART_INDICATOR);
	lv_obj_set_style_bg_opa(s_progress, LV_OPA_COVER, LV_PART_INDICATOR);
	lv_obj_set_style_radius(s_progress, 2, LV_PART_INDICATOR);

	s_time_cur = make_label(s_page_play, "0:00", (SCR_W - 216) / 2, 212,
	                        50, 14, F_NUM, C_DIM);
	s_time_total = make_label(s_page_play, "0:00", (SCR_W - 216) / 2 + 166,
	                          212, 50, 14, F_NUM, C_DIM);
	lv_obj_set_style_text_align(s_time_total, LV_TEXT_ALIGN_RIGHT, 0);

	/* 播放状态图标：放在时间行正中间那块空档里。
	 *
	 * 时间行布局是 `0:00`（x 72..122）+ 空档 + `3:45`（x 238..288），
	 * 中间 116px 是空的 —— 图标（12x14）居中放在 x=174，正好形成
	 * "0:00 ▶ 3:45" 的观感，**不额外占垂直空间**（圆屏最缺的就是纵向）。
	 * 这也是为什么之前把状态并进艺术家行、这里只留文字。 */
	s_play_icon = lv_img_create(s_page_play);
	lv_img_set_src(s_play_icon, &s_icon_play);
	lv_obj_set_pos(s_play_icon, SCR_CX - 5, 213);
	lv_obj_set_style_img_recolor(s_play_icon, lv_color_hex(C_ACCENT), 0);
	lv_obj_set_style_img_recolor_opa(s_play_icon, LV_OPA_COVER, 0);
	lv_obj_add_flag(s_play_icon, LV_OBJ_FLAG_HIDDEN);

	s_vol_label = make_label(s_page_play, "VOL 60", 0, 0, 1, 1, F12, C_DIM);
	lv_obj_add_flag(s_vol_label, LV_OBJ_FLAG_HIDDEN);   /* 音量改在状态栏显示 */
}

static void update_cover(const app_state_t *st)
{
	bool from_sd = false;
	bool from_bt = false;

	if ((st->mode == s_last_cover_mode) &&
	    (st->mode != APP_MODE_SD || st->sd_cover_gen == s_last_cover_gen) &&
	    (st->mode != APP_MODE_BT || st->bt_cover_gen == s_last_bt_cover_gen))
		return;

	/* s_cover_buf 在 PSRAM（见声明处注释），分配失败就退回默认图标 */
	if (s_cover_buf) {
		if ((st->mode == APP_MODE_SD) && st->sd.has_cover)
			from_sd = app_sd_copy_cover(s_cover_buf, COVER_SIZE, COVER_SIZE);
		if ((st->mode == APP_MODE_BT) && st->bt.has_cover)
			from_bt = bt_cover_art_copy_cover(s_cover_buf, COVER_SIZE, COVER_SIZE);
	}

	if (from_sd || from_bt) {
		s_cover_dsc.header.cf = LV_IMG_CF_TRUE_COLOR;
		s_cover_dsc.header.always_zero = 0;
		s_cover_dsc.header.reserved = 0;
		s_cover_dsc.header.w = COVER_SIZE;
		s_cover_dsc.header.h = COVER_SIZE;
		s_cover_dsc.data_size = COVER_SIZE * COVER_SIZE * 2;
		s_cover_dsc.data = (const uint8_t *)s_cover_buf;
		lv_img_set_src(s_cover_img, &s_cover_dsc);
	} else {
		lv_img_set_src(s_cover_img, &img_cover_default);
	}

	s_last_cover_mode = st->mode;
	s_last_cover_gen = st->sd_cover_gen;
	s_last_bt_cover_gen = st->bt_cover_gen;
}

/* 渲染当前歌词。
 *
 * 【圆屏限制】240x320 时代这里是三行（上一句 / 当前句 / 译文），共占
 * 46px 垂直预算。360x360 圆屏上封面放大到 148px，剩下的高度只够一行，
 * 所以 s_lyric_prev / s_lyric_sub 两个 label 创建后就被隐藏，
 * 信息合并到单行：主行为空时用译文顶替。
 *
 * prev 参数保留只为兼容调用点（上层仍传上一句），本函数不再使用。 */
static void update_lyric(const char *cur, const char *prev)
{
	char main_line[APP_LYRIC_LEN];
	char sub_line[APP_LYRIC_LEN];

	(void)prev;

	lyric_split(cur, main_line, sizeof(main_line), sub_line, sizeof(sub_line));

	if (!main_line[0] && sub_line[0])
		set_text_fit(s_lyric_cur, sub_line, F16, F12, CONTENT_W - 16);
	else
		set_text_fit(s_lyric_cur, main_line, F16, F12, CONTENT_W - 16);
}

static void update_play_page(const app_state_t *st)
{
	const app_track_t *tr = active_track(st);
	char buf[64];
	uint32_t dur = tr->dur_ms;
	uint32_t pos = tr->pos_ms;

	/* 标题 */
	if (tr->title[0])
		set_text(s_title, tr->title);
	else if (st->mode == APP_MODE_USB)
		set_text(s_title, "bk7258_soundbox");
	else if (st->mode == APP_MODE_BT)
		set_text(s_title, "无音乐");
	else
		set_text(s_title, "等待播放");

	/* 艺术家行 + 播放状态。
	 *
	 * 圆屏的垂直预算不允许播放状态单独占一行（见 COVER_SIZE 处的预算表），
	 * 所以并进这一行："艺术家 · 播放中"。
	 * 不用 LV_SYMBOL_PLAY/PAUSE —— 内置 font_sc16 是 CJK 子集，没有
	 * FontAwesome 私有区字形，只会渲染成方框。 */
	{
		const char *st_txt;

		switch (st->play) {
		case APP_PLAY_PLAYING: st_txt = "播放中"; break;
		case APP_PLAY_PAUSED:  st_txt = "已暂停"; break;
		default:               st_txt = "已停止"; break;
		}

		if (st->mode == APP_MODE_USB) {
			snprintf(buf, sizeof(buf), "电脑%s · %s",
			         st->usb_streaming ? "正在播放" : "已连接", st_txt);
			set_text(s_artist, buf);
		} else if (st->mode == APP_MODE_BT) {
			snprintf(buf, sizeof(buf), "%s · %s",
			         tr->artist[0] ? tr->artist : "蓝牙音乐", st_txt);
			set_text(s_artist, buf);
		} else {
			snprintf(buf, sizeof(buf), "%s · %s",
			         tr->artist[0] ? tr->artist : "SD 卡", st_txt);
			set_text(s_artist, buf);
		}
	}

	/* 格式胶囊：有格式就显示，没有就隐藏 */
	if (s_codec_tag && s_codec_lbl) {
		if (st->codec[0]) {
			set_text(s_codec_lbl, st->codec);
			lv_obj_clear_flag(s_codec_tag, LV_OBJ_FLAG_HIDDEN);
		} else {
			lv_obj_add_flag(s_codec_tag, LV_OBJ_FLAG_HIDDEN);
		}
	}

	/* 歌词 */
	if (st->mode == APP_MODE_BT) {
		if (tr->has_lyric)
			update_lyric(tr->lyric_cur, tr->lyric_prev);
		else
			update_lyric(tr->title[0] ? tr->title : "无音乐", "");
	} else if (st->mode == APP_MODE_SD) {
		if (tr->has_lyric)
			update_lyric(tr->lyric_cur, tr->lyric_prev);
		else
			update_lyric(tr->title[0] ? "无歌词" : "选择歌曲", "");
	} else {
		update_lyric(st->usb_streaming ? "电脑正在播放" : "等待电脑播放", "");
	}

	/* 进度 */
	int32_t val = 0;

	if (dur > 0) {
		val = (int32_t)((uint64_t)pos * 1000 / dur);
		if (val > 1000) val = 1000;
	}
	if (lv_bar_get_value(s_progress) != val)
		lv_bar_set_value(s_progress, val, LV_ANIM_OFF);

	fmt_time(pos, buf, sizeof(buf));
	set_text(s_time_cur, buf);
	fmt_time(dur, buf, sizeof(buf));
	set_text(s_time_total, buf);

	/* 播放状态图标：播放=强调色三角，暂停=主文字色双竖条，停止=隐藏。
	 *
	 * USB 模式下 app_state 的 play 字段不会随主机播放状态更新（那边只有
	 * usb_streaming 标志），所以单独判一下，否则插着电脑放歌时图标是空的。 */
	{
		bool playing, paused;

		if (st->mode == APP_MODE_USB) {
			playing = st->usb_streaming;
			paused = false;
		} else {
			playing = (st->play == APP_PLAY_PLAYING);
			paused = (st->play == APP_PLAY_PAUSED);
		}

		if (playing) {
			lv_img_set_src(s_play_icon, &s_icon_play);
			lv_obj_set_style_img_recolor(s_play_icon, lv_color_hex(C_ACCENT), 0);
			lv_obj_clear_flag(s_play_icon, LV_OBJ_FLAG_HIDDEN);
		} else if (paused) {
			lv_img_set_src(s_play_icon, &s_icon_pause);
			lv_obj_set_style_img_recolor(s_play_icon, lv_color_hex(C_TEXT), 0);
			lv_obj_clear_flag(s_play_icon, LV_OBJ_FLAG_HIDDEN);
		} else {
			lv_obj_add_flag(s_play_icon, LV_OBJ_FLAG_HIDDEN);
		}
	}

	update_cover(st);
}

/* ============================ 文件页 ============================ */

static void files_path_up(void)
{
	int n = (int)strlen(s_files_path);

	/* "1:/dir1"     -> n=3，path[2]=='/' -> 根目录
	 * "1:/dir1/sub" -> n=8，path[7]=='/' -> "1:/dir1"
	 * "1:/dir1/"    -> n=8，path[7]=='/' -> "1:/dir1" */
	while (n > 1 && s_files_path[n - 1] != '/')
		n--;

	if (n <= 3) {
		os_strncpy(s_files_path, "1:/", sizeof(s_files_path));
		s_files_path[sizeof(s_files_path) - 1] = '\0';
	} else {
		s_files_path[n - 1] = '\0';
	}
}

static void files_join(const char *name, char *out, size_t outlen)
{
	size_t n = strlen(s_files_path);

	if (n && s_files_path[n - 1] == '/')
		snprintf(out, outlen, "%s%s", s_files_path, name);
	else
		snprintf(out, outlen, "%s/%s", s_files_path, name);
}

static void refresh_files(void)
{
	int rows = 0;

	if (!bsp_sd_is_mounted()) {
		app_sd_set_active(true);
	}

	s_files_count = app_sd_list_dir(s_files_path, s_files_entries,
	                                APP_SD_MAX_ENTRIES);
	if (s_files_count < 0) {
		s_files_count = 0;
		set_text(s_files_hint, "SD 卡不可用，按 KEY4 切模式重试");
	} else {
		set_text(s_files_hint, "KEY1/3 选择   KEY2 打开   KEY4 切模式");
	}

	/* 目录非根时，第一行是 ".." */
	rows = s_files_count + (strcmp(s_files_path, "1:/") ? 1 : 0);
	if (s_files_sel >= rows)
		s_files_sel = rows > 0 ? rows - 1 : 0;
	if (s_files_sel < 0)
		s_files_sel = 0;

	/* 【分页，不是滚动】
	 *
	 * 原来 s_files_top 是"跟随选择的最小滚动量"（选中项一到屏幕边缘
	 * 就挪一行），7 行的窗口里看着是一格一格地蹭，用户完全看不出
	 * 后面还有多少内容。现在改成按页对齐：光标在同一页内移动时页面
	 * 不动，跨页才整屏跳一页，配合右上角的 "当前页/总页" 显示，
	 * 位置感很明确。
	 *
	 * 约束：s_files_top 必须始终是 FILES_VISIBLE 的整数倍，
	 * 否则 update_files_rows() 算出来的页码会错位。 */
	s_files_top = (s_files_sel / FILES_VISIBLE) * FILES_VISIBLE;

	set_text(s_files_path_lbl, s_files_path);
	(void)rows;
}

static void update_files_rows(void)
{
	bool has_parent = strcmp(s_files_path, "1:/") != 0;
	int rows = s_files_count + (has_parent ? 1 : 0);
	char name[APP_NAME_LEN];
	char size[24];

	set_text(s_files_path_lbl, s_files_path);

	/* 页码：只在超过一页时显示，单页时不占视觉空间 */
	if (s_files_page_lbl) {
		int pages = (rows + FILES_VISIBLE - 1) / FILES_VISIBLE;

		if (pages > 1) {
			char pg[16];
			int cur = s_files_top / FILES_VISIBLE + 1;

			if (cur > pages)
				cur = pages;
			snprintf(pg, sizeof(pg), "%d/%d", cur, pages);
			set_text(s_files_page_lbl, pg);
		} else {
			set_text(s_files_page_lbl, "");
		}
	}

	for (int i = 0; i < FILES_VISIBLE; i++) {
		int idx = s_files_top + i;

		if (idx >= rows) {
			lv_obj_add_flag(s_files_rows[i], LV_OBJ_FLAG_HIDDEN);
			continue;
		}

		lv_obj_clear_flag(s_files_rows[i], LV_OBJ_FLAG_HIDDEN);

		if (has_parent && idx == 0) {
			snprintf(name, sizeof(name), "..");
			size[0] = '\0';
		} else {
			int e = idx - (has_parent ? 1 : 0);

			if (e < 0 || e >= s_files_count)
				continue;
			snprintf(name, sizeof(name), "%s", s_files_entries[e].name);
			if (s_files_entries[e].is_dir)
				snprintf(size, sizeof(size), "<DIR>");
			else
				snprintf(size, sizeof(size), "%uK",
				         (unsigned)(s_files_entries[e].size / 1024));
		}

		set_text(s_files_names[i], name);
		set_text(s_files_sizes[i], size);
		set_text_color(s_files_names[i], (idx == s_files_sel) ? C_TEXT : C_DIM);

		lv_obj_set_style_bg_opa(s_files_rows[i],
		                        (idx == s_files_sel) ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
	}
}

static void files_activate(void)
{
	bool has_parent = strcmp(s_files_path, "1:/") != 0;

	if (has_parent && s_files_sel == 0) {
		files_path_up();
		s_files_sel = 0;
		s_files_top = 0;
		refresh_files();
		return;
	}

	int e = s_files_sel - (has_parent ? 1 : 0);

	if (e < 0 || e >= s_files_count)
		return;

	if (s_files_entries[e].is_dir) {
		char next[APP_PATH_LEN];

		files_join(s_files_entries[e].name, next, sizeof(next));
		os_strncpy(s_files_path, next, sizeof(s_files_path) - 1);
		s_files_path[sizeof(s_files_path) - 1] = '\0';
		s_files_sel = 0;
		s_files_top = 0;
		refresh_files();
	} else {
		char path[APP_PATH_LEN];

		files_join(s_files_entries[e].name, path, sizeof(path));
		app_mode_play_file(path);
		show_tab(APP_TAB_PLAY, true);
	}
}

static void build_files_page(void)
{
	s_page_files = make_obj(s_scr);
	lv_obj_set_pos(s_page_files, 0, CONTENT_Y);
	lv_obj_set_size(s_page_files, SCR_W, CONTENT_H);

	/* 内容区宽 256（x 52..308）：
	 *   y=60  可见 x∈[46,314]  -> 内缩 6px 安全
	 *   y=292 可见 x∈[43,317]  -> 同样安全 */
	make_label(s_page_files, "SD 卡", CONTENT_X, 0, 60, 16, F12, C_DIMMER);
	s_files_path_lbl = make_label(s_page_files, "1:/", CONTENT_X + 52, 0,
	                              CONTENT_W - 52 - 46, 16, F12, C_DIM);
	lv_label_set_long_mode(s_files_path_lbl, LV_LABEL_LONG_DOT);

	/* 右侧页码。只在条目超过一页时才有文字，见 update_files_rows()。 */
	s_files_page_lbl = make_label(s_page_files, "", CONTENT_X + CONTENT_W - 46, 0,
	                              46, 16, F12, C_ACCENT);
	lv_obj_set_style_text_align(s_files_page_lbl, LV_TEXT_ALIGN_RIGHT, 0);

	/* 7 行 × 30px = 210，从 y=24 到 234，内容区高 232 刚好放下 */
	for (int i = 0; i < FILES_VISIBLE; i++) {
		lv_obj_t *row = make_obj(s_page_files);
		lv_obj_set_pos(row, CONTENT_X, 24 + i * 30);
		lv_obj_set_size(row, CONTENT_W, 28);
		lv_obj_set_style_radius(row, 8, 0);
		lv_obj_set_style_bg_color(row, lv_color_hex(C_CARD_SEL), 0);

		s_files_rows[i] = row;
		/* 字体 line_height=19，行框 28 高 -> y=4 时上下各留 4/5 px */
		s_files_names[i] = make_label(row, "", 10, 4, CONTENT_W - 76, 20,
		                              F16, C_DIM);
		lv_label_set_long_mode(s_files_names[i], LV_LABEL_LONG_DOT);
		s_files_sizes[i] = make_label(row, "", CONTENT_W - 66, 8, 58, 14,
		                              F12, C_DIMMER);
		lv_obj_set_style_text_align(s_files_sizes[i], LV_TEXT_ALIGN_RIGHT, 0);
	}

	/* 按键提示去掉：圆屏内容区已经满了，而且四键的操作逻辑在四个页面
	 * 是统一的（KEY1/3 移动、KEY2 确认、KEY4 换页），见 app_ui 顶部说明。 */
	s_files_hint = make_label(s_page_files, "", 0, 0, 1, 1, F12, C_DIMMER);
	lv_obj_add_flag(s_files_hint, LV_OBJ_FLAG_HIDDEN);
}

/* ============================ 模式页 ============================ */

static void build_mode_page(void)
{
	s_page_mode = make_obj(s_scr);
	lv_obj_set_pos(s_page_mode, 0, CONTENT_Y);
	lv_obj_set_size(s_page_mode, SCR_W, CONTENT_H);

	make_label(s_page_mode, "播放模式", CONTENT_X + 4, 0, 120, 20, F16, C_TEXT);

	/* 4 张卡片（蓝牙 / USB / SD / USB 麦），每张 44 高、间距 52：
	 * y = 22 / 74 / 126 / 178，最后一张到 222，内容区 232 还有余量。
	 * 圆屏在这个纵向区间可见宽度都 ≥ 300px，卡宽 256 不会被裁。 */
	for (int i = 0; i < APP_MODE_MAX; i++) {
		lv_obj_t *card = make_obj(s_page_mode);
		lv_obj_set_pos(card, CONTENT_X, 22 + i * 52);
		lv_obj_set_size(card, CONTENT_W, 44);
		lv_obj_set_style_bg_color(card, lv_color_hex(C_CARD), 0);
		lv_obj_set_style_bg_opa(card, LV_OPA_COVER, 0);
		lv_obj_set_style_radius(card, 14, 0);
		lv_obj_set_style_border_width(card, 1, 0);
		lv_obj_set_style_border_color(card, lv_color_hex(0x2A2E3A), 0);
		s_mode_cards[i] = card;

		/* 图标位置让给一个序号圆点 —— LV_SYMBOL 在内置字库下是方框，
		 * 而圆屏上放方框特别难看。序号既无字体依赖，又能提示
		 * KEY1/KEY3 的移动顺序。序号不参与选中态（选中由卡片边框体现）。 */
		lv_obj_t *dot = make_obj(card);
		lv_obj_set_pos(dot, 10, 10);
		lv_obj_set_size(dot, 24, 24);
		lv_obj_set_style_radius(dot, 12, 0);
		lv_obj_set_style_bg_color(dot, lv_color_hex(0x2A2E3A), 0);
		lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
		{
			char num[2] = { (char)('1' + i), '\0' };
			lv_obj_t *nl = make_label(dot, num, 0, 3, 24, 18, F12, C_DIM);

			lv_obj_set_style_text_align(nl, LV_TEXT_ALIGN_CENTER, 0);
		}

		s_mode_names[i] = make_label(card, mode_name((app_mode_t)i), 44, 2,
		                             CONTENT_W - 56, 20, F16, C_TEXT);
		s_mode_status[i] = make_label(card, "", 44, 21, CONTENT_W - 56, 16,
		                              F12, C_DIM);
		lv_label_set_long_mode(s_mode_status[i], LV_LABEL_LONG_DOT);
	}
}

static void update_mode_page(const app_state_t *st)
{
	char buf[96];

	for (int i = 0; i < APP_MODE_MAX; i++) {
		bool sel = (i == s_mode_sel);

		lv_obj_set_style_border_color(s_mode_cards[i],
		                              lv_color_hex(sel ? C_ACCENT : 0x2A2E3A), 0);
		lv_obj_set_style_border_width(s_mode_cards[i], sel ? 2 : 1, 0);
		lv_obj_set_style_bg_color(s_mode_cards[i],
		                          lv_color_hex(sel ? C_CARD_SEL : C_CARD), 0);
	}

	if (st->bt_connected)
		snprintf(buf, sizeof(buf), "已连接%s%s", st->bt_name[0] ? " " : "", st->bt_name);
	else
		snprintf(buf, sizeof(buf), "可配对 / 等待手机连接");
	set_text(s_mode_status[APP_MODE_BT], buf);

	if (st->usb_streaming)
		set_text(s_mode_status[APP_MODE_USB], "电脑正在播放");
	else if (st->usb_connected)
		set_text(s_mode_status[APP_MODE_USB], "已连接，等待播放");
	else
		set_text(s_mode_status[APP_MODE_USB], "插入 USB 连接电脑");

	if (st->usb_mic_streaming)
		set_text(s_mode_status[APP_MODE_USB_MIC], "电脑录音中");
	else if (st->usb_connected)
		set_text(s_mode_status[APP_MODE_USB_MIC], "已连接，等待录音");
	else
		set_text(s_mode_status[APP_MODE_USB_MIC], "插入 USB 当麦克风");

	if (st->sd_mounted)
		set_text(s_mode_status[APP_MODE_SD], "SD 卡已挂载");
	else if (st->sd_present)
		set_text(s_mode_status[APP_MODE_SD], "检测到卡，挂载失败");
	else
		set_text(s_mode_status[APP_MODE_SD], "未插入 SD 卡");
}

/* ============================ 设置页 ============================ */

#define SET_ITEMS 6

static void build_settings_page(void)
{
	static const char *names[SET_ITEMS] = {
		"音量", "背光", "蓝牙配对", "摇一摇切歌", "字体", "关于"
	};

	s_page_settings = make_obj(s_scr);
	lv_obj_set_pos(s_page_settings, 0, CONTENT_Y);
	lv_obj_set_size(s_page_settings, SCR_W, CONTENT_H);

	make_label(s_page_settings, "设置", CONTENT_X + 4, 0, 120, 20, F16, C_TEXT);

	/* 6 行 × 34 = 204，从 y=26 到 230，内容区高 232 刚好。
	 * 每行 256 宽（内容区满宽），此处 y 全程可见宽 ≥ 290。 */
	for (int i = 0; i < SET_ITEMS; i++) {
		lv_obj_t *row = make_obj(s_page_settings);
		lv_obj_set_pos(row, CONTENT_X, 26 + i * 34);
		lv_obj_set_size(row, CONTENT_W, 30);
		lv_obj_set_style_radius(row, 9, 0);
		lv_obj_set_style_bg_color(row, lv_color_hex(C_CARD_SEL), 0);
		s_set_rows[i] = row;

		/* 30 高的行 + 19 的行高 -> y=6 时文字上下各留 6/5 px */
		s_set_names[i] = make_label(row, names[i], 12, 6, 130, 20, F16, C_TEXT);
		s_set_values[i] = make_label(row, "", 142, 7, CONTENT_W - 154, 18,
		                             F12, C_DIM);
		lv_label_set_long_mode(s_set_values[i], LV_LABEL_LONG_CLIP);
		lv_obj_set_style_text_align(s_set_values[i], LV_TEXT_ALIGN_RIGHT, 0);
	}

	/* 按键提示不单独占一行（内容区 232 高已被 6 行用掉 230），
	 * 操作逻辑与其它页一致：KEY1/3 选择、KEY2 调整、KEY4 换页。 */
	s_set_hint = make_label(s_page_settings, "", 0, 0, 1, 1, F12, C_DIMMER);
	lv_obj_add_flag(s_set_hint, LV_OBJ_FLAG_HIDDEN);
}

static void update_settings_page(const app_state_t *st)
{
	char buf[32];

	for (int i = 0; i < SET_ITEMS; i++) {
		bool sel = (i == s_set_sel);

		lv_obj_set_style_bg_opa(s_set_rows[i], sel ? LV_OPA_COVER : LV_OPA_TRANSP, 0);
		lv_obj_set_style_bg_color(s_set_rows[i], lv_color_hex(C_CARD_SEL), 0);
	}

	snprintf(buf, sizeof(buf), "%u", (unsigned)st->volume);
	set_text(s_set_values[0], buf);
	set_text(s_set_values[1], st->backlight ? "开" : "关");
	set_text(s_set_values[2], "KEY2 配对");
	set_text(s_set_values[3], !st->shake_ready ? "无"
	                                          : (st->shake_next ? "开" : "关"));
	/* 字体项：TTF 没加载成功时显示"无SD字体"，开关无效 */
	if (!bsp_font_ttf_available())
		set_text(s_set_values[4], "无SD字体");
	else
		set_text(s_set_values[4], st->font_sd ? "SD字体" : "内置");
	set_text(s_set_values[5], "BK7258 v1.0");
}

static void settings_activate(const app_state_t *st)
{
	switch (s_set_sel) {
	case 0: {
		/* 音量每次 +10，超过 100 回 0 */
		int v = (int)st->volume + 10;

		if (v > 100)
			v = 0;
		app_mode_volume_set((uint8_t)v);
		break;
	}
	case 1:
		app_mode_backlight_toggle();
		break;
	case 2:
		app_mode_bt_pairing();
		break;
	case 3:
		app_mode_shake_toggle();
		break;
	case 4: {
		/* 字体：内置点阵 <-> SD 卡 TTF（TTF 渲染有问题时切回内置） */
		bool use_sd = !st->font_sd;

		if (!bsp_font_ttf_available()) {
			app_state_toast("无 SD 字体，先放 1:/font.ttf");
			break;
		}
		bsp_font_set_ttf(use_sd);
		app_state_set_font_sd(use_sd);
		ui_rebuild();     /* 所有 label 都在创建时绑定了字体指针，必须重建 */
		app_state_toast(use_sd ? "字体: SD TTF" : "字体: 内置");
		break;
	}
	case 5:
		/* 版本 + 电池实测值。本工程没有 shell，这里也是现场唯一能直接
		 * 看到"ADC 到底读了多少 mV"的地方 —— 校分压比/BAT_ADC_FS_MV 时
		 * 拿万用表量 VBAT，对照这个数就能判断换算是否偏了。 */
		if (st->battery_valid)
			app_state_toast("v1.0 BAT %umV %u%%",
			                (unsigned)st->battery_mv,
			                (unsigned)st->battery_pct);
		else
			app_state_toast("v1.0 BAT -- (无电池)");
		break;
	default:
		break;
	}
}

/* ============================ 状态栏 / Tab / Toast ============================ */

static void build_status_bar(void)
{
	lv_obj_t *bar = make_obj(s_scr);
	lv_obj_set_pos(bar, STATUS_X, STATUS_Y);
	lv_obj_set_size(bar, STATUS_W, STATUS_H);

	/* 圆屏顶部很窄（y=36 只有 216px 可见宽），所以状态栏不做整行底色，
	 * 只放两段文字，靠 CONTENT 的深色背景衬托。加底色反而会露出直边，
	 * 和圆形轮廓打架。 */
	s_status_left = make_label(bar, "蓝牙", 0, 2, STATUS_LEFT_W, 16, F12, C_ACCENT);
	s_status_right = make_label(bar, "VOL 60", STATUS_LEFT_W, 2, STATUS_RIGHT_W, 16,
	                            F12, C_DIM);
	lv_obj_set_style_text_align(s_status_right, LV_TEXT_ALIGN_RIGHT, 0);
	s_status_right_color = C_DIM;

	/* 电量图标（自绘，理由见 BAT_BODY_W 处的注释）：
	 * 外壳 17x10 空心圆角矩形 + 3x4 正极帽 + 内部按百分比伸缩的填充条。 */
	s_bat_body = make_obj(bar);
	lv_obj_set_pos(s_bat_body, BAT_ICON_X, (STATUS_H - BAT_BODY_H) / 2);
	lv_obj_set_size(s_bat_body, BAT_BODY_W, BAT_BODY_H);
	lv_obj_set_style_border_width(s_bat_body, 1, 0);
	lv_obj_set_style_border_color(s_bat_body, lv_color_hex(C_DIM), 0);
	lv_obj_set_style_radius(s_bat_body, 2, 0);

	s_bat_cap = make_obj(bar);
	lv_obj_set_pos(s_bat_cap, BAT_ICON_X + BAT_BODY_W,
	               (STATUS_H - BAT_CAP_H) / 2);
	lv_obj_set_size(s_bat_cap, BAT_CAP_W, BAT_CAP_H);
	lv_obj_set_style_bg_color(s_bat_cap, lv_color_hex(C_DIM), 0);
	lv_obj_set_style_bg_opa(s_bat_cap, LV_OPA_COVER, 0);
	lv_obj_set_style_radius(s_bat_cap, 1, 0);

	/* 子对象坐标相对父对象的**内容区**（border+padding 之后），外壳有
	 * 1px 边框，所以 (1,1) 正好贴住内腔左上角；内腔 15x8，填充条 13x6
	 * 右/下各留 1px，看起来才像"电量没充满"。 */
	s_bat_fill = make_obj(s_bat_body);
	lv_obj_set_pos(s_bat_fill, 1, 1);
	lv_obj_set_size(s_bat_fill, BAT_FILL_W, BAT_FILL_H);
	lv_obj_set_style_bg_color(s_bat_fill, lv_color_hex(C_ACCENT), 0);
	lv_obj_set_style_bg_opa(s_bat_fill, LV_OPA_COVER, 0);
	lv_obj_set_style_radius(s_bat_fill, 1, 0);
	/* 第一帧还不知道电量：先藏起来，别闪一下"满电" */
	lv_obj_add_flag(s_bat_fill, LV_OBJ_FLAG_HIDDEN);

	s_bat_icon_pct = 0xFF;    /* 新对象：下一帧强制套用当前电量 */
	s_bat_icon_valid = false;
}

static void build_tab_bar(void)
{
	static const char *icons[4] = {
		LV_SYMBOL_PLAY, LV_SYMBOL_DIRECTORY, LV_SYMBOL_AUDIO, LV_SYMBOL_SETTINGS
	};
	static const char *texts[4] = { "播放", "文件", "模式", "设置" };

	lv_obj_t *bar = make_obj(s_scr);
	lv_obj_set_pos(bar, TAB_X, TAB_Y);
	lv_obj_set_size(bar, TAB_W, TAB_H);
	/* 同样不铺满整行：圆屏底部 y=298 处可见宽 268，铺 216 宽刚好，
	 * 加一条整宽顶边会从左到右割出直线，破坏圆形观感。 */

	for (int i = 0; i < 4; i++) {
		lv_coord_t x = (lv_coord_t)(i * TAB_ITEM_W);

		/* 选中指示：图标上方一条短横线 */
		s_tab_bars[i] = make_obj(bar);
		lv_obj_set_pos(s_tab_bars[i], x + (TAB_ITEM_W - 14) / 2, 0);
		lv_obj_set_size(s_tab_bars[i], 14, 2);
		lv_obj_set_style_bg_color(s_tab_bars[i], lv_color_hex(C_ACCENT), 0);
		lv_obj_set_style_bg_opa(s_tab_bars[i], LV_OPA_TRANSP, 0);

		/* 26px 的栏高里塞图标+文字太挤，只留文字；
		 * 图标（LV_SYMBOL）在内置 CJK 字库里没有字形，本来就显示方框。 */
		s_tab_icons[i] = make_label(bar, icons[i], x, 0, TAB_ITEM_W, 1,
		                            F_NUM, C_DIMMER);
		lv_obj_add_flag(s_tab_icons[i], LV_OBJ_FLAG_HIDDEN);

		s_tab_texts[i] = make_label(bar, texts[i], x, 5, TAB_ITEM_W, 18,
		                            F12, C_DIM);
		lv_obj_set_style_text_align(s_tab_texts[i], LV_TEXT_ALIGN_CENTER, 0);
	}
}

static void build_toast(void)
{
	s_toast = make_obj(s_scr);
	/* 圆屏正中偏下：y=246 处可见宽 336，200 宽居中不受限。
	 * 不放最底部的原因是和 Tab 栏抢位置。 */
	lv_obj_set_pos(s_toast, (SCR_W - 200) / 2, 246);
	lv_obj_set_size(s_toast, 200, 26);
	lv_obj_set_style_bg_color(s_toast, lv_color_hex(0x2A2E3A), 0);
	lv_obj_set_style_bg_opa(s_toast, LV_OPA_90, 0);
	lv_obj_set_style_radius(s_toast, 13, 0);
	lv_obj_add_flag(s_toast, LV_OBJ_FLAG_HIDDEN);

	lv_obj_t *lbl = make_label(s_toast, "", 6, 5, 188, 16, F12, C_TEXT);
	lv_obj_set_style_text_align(lbl, LV_TEXT_ALIGN_CENTER, 0);
	lv_obj_set_user_data(s_toast, lbl);
}

/* 字体切换后重建全部页面：所有 label 在 make_label 时就绑定了字体指针，
 * 单独 set_text 不会换字体。toast 没有动画/定时器，clean 是安全的。 */
static void ui_rebuild(void)
{
	s_f12 = bsp_font_get(BSP_FONT_SMALL);
	s_f16 = bsp_font_get(BSP_FONT_NORMAL);

	lv_obj_clean(s_scr);

	build_status_bar();
	build_play_page();
	build_files_page();
	build_mode_page();
	build_settings_page();
	build_tab_bar();
	build_toast();

	s_last_tab = APP_TAB_MAX;                 /* 强制 show_tab 重画 */
	show_tab((app_tab_t)s_last.tab, true);
	s_need_redraw = true;
	LOGI("ui rebuilt, font: %s\r\n", bsp_font_source());
}

static void update_toast(const app_state_t *st)
{
	uint32_t now = rtos_get_time();

	if (st->toast_seq != s_toast_seq) {
		s_toast_seq = st->toast_seq;
		s_toast_until = now + 2200;
		lv_obj_t *lbl = (lv_obj_t *)lv_obj_get_user_data(s_toast);

		if (lbl)
			set_text(lbl, st->toast);
		lv_obj_clear_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
	}

	if (s_toast_until && (now > s_toast_until)) {
		s_toast_until = 0;
		lv_obj_add_flag(s_toast, LV_OBJ_FLAG_HIDDEN);
	}
}

/* 按电量刷新图标：有效 -> 绿/红填充条；无效 -> 空壳 + 暗色（表示"未知"，
 * 不用红色，红色只用来表示"电量低"）。
 *
 * 只有 (valid, pct) 变化时才写 LVGL 样式：状态栏每 100ms 就会走一次
 * update_status_bar()，无条件 set_style 会让这块区域一直标脏重绘。 */
static void update_battery_icon(bool valid, uint8_t pct)
{
	lv_coord_t w;
	uint32_t col;

	if (!s_bat_body || !s_bat_cap || !s_bat_fill)
		return;

	if ((s_bat_icon_valid == valid) && (s_bat_icon_pct == pct))
		return;

	s_bat_icon_valid = valid;
	s_bat_icon_pct = pct;

	if (!valid) {
		lv_obj_add_flag(s_bat_fill, LV_OBJ_FLAG_HIDDEN);
		lv_obj_set_style_border_color(s_bat_body, lv_color_hex(C_DIMMER), 0);
		lv_obj_set_style_bg_color(s_bat_cap, lv_color_hex(C_DIMMER), 0);
		return;
	}

	col = (pct <= BAT_LOW_PCT) ? C_LOW : C_ACCENT;
	lv_obj_set_style_border_color(s_bat_body, lv_color_hex(C_DIM), 0);
	lv_obj_set_style_bg_color(s_bat_cap, lv_color_hex(C_DIM), 0);
	lv_obj_set_style_bg_color(s_bat_fill, lv_color_hex(col), 0);

	if (pct == 0) {
		lv_obj_add_flag(s_bat_fill, LV_OBJ_FLAG_HIDDEN);
		return;
	}

	lv_obj_clear_flag(s_bat_fill, LV_OBJ_FLAG_HIDDEN);
	w = (lv_coord_t)((BAT_FILL_W * pct) / 100);
	if (w < 1)
		w = 1;                /* 1%~7% 也留一小格，别显示成空电池 */
	lv_obj_set_width(s_bat_fill, w);
}

static void update_status_bar(const app_state_t *st)
{
	char buf[96];
	const char *state = "";

	switch (st->mode) {
	case APP_MODE_BT:
		state = st->bt_connected ? " 已连接" : " 可配对";
		break;
	case APP_MODE_USB:
		state = st->usb_streaming ? " 播放中" : (st->usb_connected ? " 已连接" : " 等待");
		break;
	case APP_MODE_SD:
		state = st->sd_mounted ? "" : (st->sd_present ? " 异常" : " 未插卡");
		break;
	case APP_MODE_USB_MIC:
		state = st->usb_mic_streaming ? " 录音中"
		        : (st->usb_connected ? " 已连接" : " 等待");
		break;
	default:
		break;
	}

	snprintf(buf, sizeof(buf), "%s%s", mode_name(st->mode), state);
	set_text(s_status_left, buf);

	/* 右侧：音量 + 电量百分比。电量无效（没接电池/分压异常）时显示
	 * "--%"，不要显示 0% —— 那会让人以为电池已经耗尽。 */
	if (st->battery_valid)
		snprintf(buf, sizeof(buf), "VOL %u %u%%", (unsigned)st->volume,
		         (unsigned)st->battery_pct);
	else
		snprintf(buf, sizeof(buf), "VOL %u --%%", (unsigned)st->volume);
	set_text(s_status_right, buf);

	/* 文字颜色同样做缓存：低电量才变红 */
	{
		uint32_t col = (st->battery_valid && st->battery_pct <= BAT_LOW_PCT)
		                   ? C_LOW : C_DIM;

		if (col != s_status_right_color) {
			s_status_right_color = col;
			set_text_color(s_status_right, col);
		}
	}

	update_battery_icon(st->battery_valid, st->battery_pct);
}

/* ============================ 按键处理 ============================ */

static void cycle_mode(void)
{
	app_mode_t next = (app_mode_t)((app_state_mode() + 1) % APP_MODE_MAX);

	app_mode_switch_manual(next);
	show_tab(APP_TAB_PLAY, true);
}

static void next_tab(void)
{
	app_tab_t t = (app_tab_t)((s_tab + 1) % APP_TAB_MAX);

	show_tab(t, true);
}

static void move_mode_sel(int delta)
{
	int v = (int)s_mode_sel + delta;

	/* 模式数是 APP_MODE_MAX（现在是 4：蓝牙 / USB / SD / USB 麦），
	 * 不能写死 0..2，否则新模式选不中。 */
	if (v < 0)
		v = APP_MODE_MAX - 1;
	if (v >= APP_MODE_MAX)
		v = 0;
	s_mode_sel = (uint8_t)v;
}

static void move_set_sel(int delta)
{
	int v = (int)s_set_sel + delta;

	if (v < 0) v = SET_ITEMS - 1;
	if (v >= SET_ITEMS) v = 0;
	s_set_sel = (uint8_t)v;
}

static void move_file_sel(int delta)
{
	bool has_parent = strcmp(s_files_path, "1:/") != 0;
	int rows = s_files_count + (has_parent ? 1 : 0);

	if (rows <= 0)
		return;

	s_files_sel += delta;
	if (s_files_sel < 0)
		s_files_sel = rows - 1;
	if (s_files_sel >= rows)
		s_files_sel = 0;

	/* 【分页必须在这里同步页起点】
	 *
	 * 上一版只在 refresh_files()（进目录/换盘/初始化）里算 s_files_top，
	 * 而光标移动走的是这个函数，压根不碰 top —— 结果是光标走出当前
	 * 7 行之后，选中项就在屏幕外了（看得到高亮吗？看不到：整页还是
	 * 原来那 7 行，只是没有任何一行是选中的），页码也不动。
	 * 这就是"分页有问题"的直接原因。
	 *
	 * 公式与 refresh_files() 保持一致：top 永远是 FILES_VISIBLE 的整数倍。 */
	s_files_top = (s_files_sel / FILES_VISIBLE) * FILES_VISIBLE;
}

/* 短按（松开且未触发长按时执行）。
 * 播放页 KEY1/KEY3 = 快退/快进 10s；其它页仍是上移/下移光标。 */
static void handle_key_click(uint8_t key)
{
	static app_state_t st;

	switch (key) {
	case KEY_4:
		next_tab();
		return;
	case KEY_2:
		app_state_copy(&st);
		switch (s_tab) {
		case APP_TAB_PLAY:
			app_mode_play_pause();
			break;
		case APP_TAB_FILES:
			files_activate();
			break;
		case APP_TAB_MODE:
			app_mode_switch_manual((app_mode_t)s_mode_sel);
			show_tab(APP_TAB_PLAY, true);
			break;
		case APP_TAB_SETTINGS:
			settings_activate(&st);
			break;
		default:
			break;
		}
		return;
	case KEY_1:
	case KEY_3: {
		int delta = (key == KEY_3) ? 1 : -1;

		switch (s_tab) {
		case APP_TAB_PLAY:
			app_mode_seek(key == KEY_3 ? 10000 : -10000);
			break;
		case APP_TAB_FILES:
			move_file_sel(delta);
			break;
		case APP_TAB_MODE:
			move_mode_sel(delta);
			break;
		case APP_TAB_SETTINGS:
			move_set_sel(delta);
			break;
		default:
			break;
		}
		return;
	}
	default:
		break;
	}
}

/* 长按：播放页 KEY1/KEY3 切上一首/下一首；播放页 KEY2 = 摇一摇开关
 * （摇的时候不方便进设置页）；其它页 KEY2 仍是蓝牙配对。
 * KEY4 切模式与当前页面无关。 */
static void handle_key_long(uint8_t key)
{
	if (key == KEY_4) {
		cycle_mode();
		return;
	}
	if (key == KEY_2) {
		if (s_tab == APP_TAB_PLAY)
			app_mode_shake_toggle();
		else
			app_mode_bt_pairing();
		return;
	}
	if ((key != KEY_1) && (key != KEY_3))
		return;

	if (s_tab == APP_TAB_PLAY) {
		if (key == KEY_1)
			app_mode_prev();
		else
			app_mode_next();
	} else {
		app_mode_volume_step(key == KEY_1 ? -5 : 5);
	}
}

/* 长按后的连发：播放页已经切过歌，继续按住不再做任何事（尤其不能连着调音量）；
 * 其它页保留原来的音量连发。 */
static void handle_key_repeat(uint8_t key)
{
	if (s_tab == APP_TAB_PLAY)
		return;

	if (key == KEY_1)
		app_mode_volume_step(-2);
	else if (key == KEY_3)
		app_mode_volume_step(+2);
}

/* 短按/长按互斥：
 * 底层在 EVT_DOWN 就报「按下」，900ms 后报 EVT_LONG。若直接在 DOWN 执行短按，
 * 一次长按会先跳 10s 再切歌。这里把短按推迟到 EVT_UP，且 LONG 到达时撤销它。 */
static struct {
	uint8_t key;
	bool pending;
} s_click;

/* ---- 熄屏省电 + 拿起唤醒 ----
 * 无操作 DIM_TIMEOUT_MS 后关背光（屏内容保持，功耗大头是背光 LED）；
 * 熄屏期间加速度计仍在采样，拿起（运动）由 app_shake 回调触发
 * app_ui_wakeup_request() 亮屏。第一次按键也只亮屏不执行动作。 */
#define DIM_TIMEOUT_MS   60000

static volatile bool s_dimmed;
static uint32_t s_last_activity;
static volatile bool s_wakeup_req;

static void wake_screen(void)
{
	s_dimmed = false;
	s_last_activity = rtos_get_time();
	bsp_screen_backlight_set(100);

	app_state_lock();
	app_state_raw()->backlight = 100;
	app_state_unlock();
	LOGI("screen wake\r\n");
}

static void dim_screen(void)
{
	s_dimmed = true;
	bsp_screen_backlight_set(0);

	app_state_lock();
	app_state_raw()->backlight = 0;
	app_state_unlock();
	LOGI("screen dim (idle %ums)\r\n", (unsigned)DIM_TIMEOUT_MS);
}

void app_ui_wakeup_request(void)
{
	s_wakeup_req = true;
}

static void process_keys(void)
{
	ui_key_t k;

	while (pop_key(&k)) {
		s_last_activity = rtos_get_time();

		/* 熄屏期间第一次按键只负责亮屏（防误触），不执行动作 */
		if (s_dimmed) {
			wake_screen();
			continue;
		}

		s_need_redraw = true;
		switch (k.evt) {
		case EVT_DOWN:
			s_click.key = k.key;
			s_click.pending = true;
			break;
		case EVT_UP:
			if (s_click.pending && (s_click.key == k.key)) {
				s_click.pending = false;
				handle_key_click(k.key);
			}
			break;
		case EVT_LONG:
			s_click.pending = false;
			handle_key_long(k.key);
			break;
		case EVT_REPEAT:
			handle_key_repeat(k.key);
			break;
		default:
			break;
		}
	}
}

/* ============================ 状态刷新 ============================ */

static bool state_dirty(const app_state_t *st)
{
	if (st->mode != s_last.mode || st->play != s_last.play ||
	    st->volume != s_last.volume || st->backlight != s_last.backlight ||
	    st->shake_next != s_last.shake_next ||
	    st->shake_ready != s_last.shake_ready ||
	    st->font_sd != s_last.font_sd ||
	    st->bt_connected != s_last.bt_connected ||
	    st->bt_streaming != s_last.bt_streaming ||
	    st->usb_connected != s_last.usb_connected ||
	    st->usb_streaming != s_last.usb_streaming ||
	    st->usb_mic_streaming != s_last.usb_mic_streaming ||
	    st->sd_present != s_last.sd_present ||
	    st->sd_mounted != s_last.sd_mounted ||
	    st->sd_cover_gen != s_last.sd_cover_gen ||
	    st->toast_seq != s_last.toast_seq)
		return true;

	if (memcmp(&st->bt, &s_last.bt, sizeof(app_track_t)))
		return true;
	if (memcmp(&st->usb, &s_last.usb, sizeof(app_track_t)))
		return true;
	if (memcmp(&st->sd, &s_last.sd, sizeof(app_track_t)))
		return true;
	if (memcmp(st->codec, s_last.codec, sizeof(st->codec)))
		return true;
	return false;
}

static void refresh_state(void)
{
	static app_state_t st;
	uint32_t now = rtos_get_time();

	if ((now - s_last_refresh) < 100)
		return;
	s_last_refresh = now;

	app_state_copy(&st);
	update_status_bar(&st);
	update_toast(&st);

	if (!state_dirty(&st) && !s_need_redraw && (s_tab == s_last_tab))
		return;

	/* 模式被外部（手机/USB）自动切走时，界面跟到播放页 */
	if (st.mode != s_last.mode) {
		if (s_tab == APP_TAB_MODE)
			s_mode_sel = (uint8_t)st.mode;
	}

	switch (s_tab) {
	case APP_TAB_PLAY:
		update_play_page(&st);
		break;
	case APP_TAB_FILES:
		if (s_tab != s_last_tab)
			refresh_files();
		update_files_rows();
		break;
	case APP_TAB_MODE:
		update_mode_page(&st);
		break;
	case APP_TAB_SETTINGS:
		update_settings_page(&st);
		break;
	default:
		break;
	}

	/* 模式页/设置页选中项变化时也要刷新 */
	if ((s_tab == APP_TAB_MODE) && (st.mode != s_last.mode))
		update_mode_page(&st);

	s_last = st;
	s_last_tab = s_tab;
	s_need_redraw = false;
}

/* ============================ 对外接口 ============================ */

void app_ui_init(void)
{
	static app_state_t st;

	/* 封面缓冲放 PSRAM（21.6KB，见 s_cover_buf 处注释）。
	 * bsp_lvgl_init 已经 probe 过 PSRAM，这里直接分。失败不致命，
	 * update_cover() 会回退默认图标。 */
	if (!s_cover_buf)
		s_cover_buf = (uint16_t *)psram_malloc(COVER_SIZE * COVER_SIZE * sizeof(uint16_t));

	/* 字体在 UI 之前定下来（SD 卡 TTF 可能已经在 bsp_font_init 里加载好） */
	s_f12 = bsp_font_get(BSP_FONT_SMALL);
	s_f16 = bsp_font_get(BSP_FONT_NORMAL);
	LOGI("font source: %s\r\n", bsp_font_source());

	app_state_copy(&st);

	s_scr = lv_scr_act();
	lv_obj_set_style_bg_color(s_scr, lv_color_hex(C_BG), 0);
	lv_obj_set_style_bg_opa(s_scr, LV_OPA_COVER, 0);
	lv_obj_clear_flag(s_scr, LV_OBJ_FLAG_SCROLLABLE);

	build_status_bar();
	build_play_page();
	build_files_page();
	build_mode_page();
	build_settings_page();
	build_tab_bar();
	build_toast();

	s_mode_sel = (uint8_t)st.mode;
	s_last = st;
	s_last_tab = APP_TAB_MAX;
	s_last_activity = rtos_get_time();   /* 上电即开始计时熄屏 */

	show_tab((app_tab_t)st.tab, true);

	LOGI("ui created\r\n");
}

/* 熄屏定时：设置页背光开关、模式切换（ACMD_BACKLIGHT）直接改
 * app_state 的 backlight，这里以它为准做收敛。
 *
 * 【播放中不熄屏】只要任一通路在出声（SD 播放 / BT 串流 / USB 串流），
 * 就把"最后活动时间"往前推，屏幕保持点亮。用户明确要求"播放时不要
 * 进入休眠状态"；而且熄屏后按键的第一下只负责亮屏，等于多一次操作。 */
static void sleep_tick(void)
{
	uint32_t now = rtos_get_time();
	bool bl_on;
	bool playing;

	if (s_wakeup_req) {
		s_wakeup_req = false;
		if (s_dimmed)
			wake_screen();
	}

	app_state_lock();
	bl_on = (app_state_raw()->backlight != 0);
	playing = (app_state_raw()->play == APP_PLAY_PLAYING) ||
	          app_state_raw()->bt_streaming ||
	          app_state_raw()->usb_streaming;
	app_state_unlock();

	if (!s_dimmed && bl_on) {
		if (playing)
			s_last_activity = now;
		else if ((now - s_last_activity) >= DIM_TIMEOUT_MS)
			dim_screen();
	}
}

void app_ui_loop(void)
{
	process_keys();
	sleep_tick();
	refresh_state();
}
