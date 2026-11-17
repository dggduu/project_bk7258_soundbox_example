// Copyright 2026
// 全局应用状态：三种模式共用的 UI/控制快照
//
// 所有模块通过 app_state_lock() / app_state_unlock() 访问，UI 线程只读。
// 这样 BT/SD/USB 线程更新状态时不会直接碰 LVGL。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	APP_MODE_BT = 0,
	APP_MODE_USB,
	APP_MODE_SD,
	APP_MODE_USB_MIC,   /* USB 麦克风：只采音，不出声 */
	APP_MODE_MAX,
} app_mode_t;

typedef enum {
	APP_TAB_PLAY = 0,
	APP_TAB_FILES,
	APP_TAB_MODE,
	APP_TAB_SETTINGS,
	APP_TAB_MAX,
} app_tab_t;

typedef enum {
	APP_PLAY_STOPPED = 0,
	APP_PLAY_PLAYING,
	APP_PLAY_PAUSED,
} app_play_state_t;

#define APP_TITLE_LEN   96
#define APP_ARTIST_LEN  64
#define APP_ALBUM_LEN   64
#define APP_LYRIC_LEN   160
#define APP_PATH_LEN    256
#define APP_NAME_LEN    96

typedef struct {
	char title[APP_TITLE_LEN];
	char artist[APP_ARTIST_LEN];
	char album[APP_ALBUM_LEN];
	char lyric_cur[APP_LYRIC_LEN];      /* 当前歌词（BT=AVRCP TITLE；SD=LRC） */
	char lyric_prev[APP_LYRIC_LEN];     /* 上一句，用于车载歌词的上下文 */
	uint32_t pos_ms;
	uint32_t dur_ms;
	bool has_lyric;
	bool has_cover;
} app_track_t;

typedef struct {
	app_mode_t mode;
	app_tab_t tab;
	uint8_t volume;             /* 0~100 */
	uint8_t backlight;          /* 0~100（本板只有开/关） */
	bool muted;
	bool shake_next;            /* 摇一摇切歌（设置页开关，默认开） */
	bool shake_ready;           /* 加速度计是否可用（不可用时设置页显示“无”） */
	bool font_sd;               /* 字体来源：true=SD 卡 TTF，false=内置点阵 */
	char codec[8];              /* 当前播放格式胶囊：FLAC/WAV/MP3/SBC/PCM */

	/* 电池（P22/ADC5，见 bsp_battery.c） */
	bool battery_valid;         /* false = 没接电池 / 分压异常 / ADC 读失败 */
	uint8_t battery_pct;        /* 0~100，单节锂电 3.0~4.2V 线性映射 */
	uint32_t battery_mv;        /* VBAT，单位 mV */

	/* 蓝牙 */
	bool bt_connected;
	bool bt_streaming;
	bool bt_pairing;
	char bt_name[APP_NAME_LEN];
	app_track_t bt;
	uint32_t bt_cover_gen;

	/* USB UAC */
	bool usb_connected;
	bool usb_streaming;
	bool usb_mic_streaming;     /* USB 麦克风模式：主机正在录音 */
	app_track_t usb;
	uint32_t usb_cover_gen;

	/* SD */
	bool sd_present;
	bool sd_mounted;
	app_track_t sd;
	uint32_t sd_cover_gen;

	app_play_state_t play;      /* 当前模式播放状态 */

	/* 蓝牙免提通话（HFP）。call_active 与 call_ringing 互斥地为 UI
	 * 提供"该显示通话页还是来电页"的依据；两者都为 false 表示没通话。 */
	bool call_active;           /* 通话已建立（含去电） */
	bool call_ringing;          /* 来电振铃中 */

	char toast[64];             /* 一次性提示，UI 读取后清空 */
	uint32_t toast_seq;
} app_state_t;

void app_state_init(void);
void app_state_lock(void);
void app_state_unlock(void);

/* 调用者必须已加锁 */
app_state_t *app_state_raw(void);

/* 线程安全拷贝 */
void app_state_copy(app_state_t *out);

/* 当前活动模式的 track 指针（调用者加锁） */
app_track_t *app_state_active_track(app_state_t *st);
const app_track_t *app_state_active_track_const(const app_state_t *st);

/* 当前模式/页面 */
app_mode_t app_state_mode(void);
void app_state_set_mode(app_mode_t mode);
app_tab_t app_state_tab(void);
void app_state_set_tab(app_tab_t tab);

/* 提示条（线程安全） */
void app_state_toast(const char *fmt, ...);

/* 默认音量 / 背光 */
uint8_t app_state_volume(void);
void app_state_set_volume(uint8_t vol);

/* 电池读数（app_mode 后台任务写，UI 线程读快照） */
void app_state_set_battery(uint32_t mv, uint8_t pct, bool valid);

/* 摇一摇切歌开关 / 加速度计可用性 */
bool app_state_shake_next(void);
void app_state_set_shake_next(bool on);
bool app_state_shake_ready(void);
void app_state_set_shake_ready(bool ready);
bool app_state_font_sd(void);
void app_state_set_font_sd(bool use_sd);

/* 蓝牙通话状态（active=通话中，ringing=来电振铃；两者不会同时为 true） */
void app_state_set_call(bool active, bool ringing);
bool app_state_call_active(void);
bool app_state_call_ringing(void);

#ifdef __cplusplus
}
#endif
