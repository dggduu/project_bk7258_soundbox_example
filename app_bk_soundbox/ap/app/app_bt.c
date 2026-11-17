// Copyright 2026
// 蓝牙模式封装实现（A2DP sink + AVRCP CT 车载歌词）

#include "app_bt.h"
#include "app_bt_link.h"
#include "app_state.h"
#include "app_mode.h"

#include <string.h>
#include <strings.h>
#include <os/os.h>
#include <os/str.h>
#include <stdio.h>
#include <components/log.h>
#include <components/bluetooth/bk_dm_bluetooth.h>
#include <components/bluetooth/bk_dm_avrcp_types.h>

#include "bt_manager.h"
#include "bt_cover_art.h"
#include "a2dp_sink_demo.h"

#define TAG "app_bt"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define BT_INIT_STACK     5120
#define BT_POLL_ATTR_MS   1000
#define BT_TITLE_LOCK_MS  1200
/* 换歌去抖：手机重发 TRACK_CHANGE（重注册通知、暂停恢复、每首多次）很常见 */
#define BT_TRACK_CHANGE_MS 3000
/* 换歌窗口：这段时间内的额外 TITLE 既不当歌名也不当歌词。
 * 手机切歌瞬间常常先给歌名、几百 ms 后开始推首行歌词，若不设窗口，
 * 第二条 TITLE 就会变成"歌词"，而第三条又被当成新歌名。 */
#define BT_TITLE_GUARD_MS  1500
/* 属性变化确认换歌的最短间隔，避免 TRACK_CHANGE 与属性变化重复触发 */
#define BT_CONFIRM_MIN_MS  1200

static volatile bool s_ready;
static volatile bool s_stack_ready;
static volatile bool s_active;
static volatile bool s_connected;
static volatile bool s_streaming;
static volatile bool s_avrcp_connected;
static volatile bool s_playing;
static volatile bool s_want_active;

static char s_song_title[APP_TITLE_LEN];
static char s_artist[APP_ARTIST_LEN];
static char s_album[APP_ALBUM_LEN];
static char s_lyric_cur[APP_LYRIC_LEN];
static char s_lyric_prev[APP_LYRIC_LEN];
static bool s_has_lyric;
static bool s_title_locked;
static uint32_t s_track_start_ms;
static uint32_t s_last_title_ms;
static uint32_t s_dur_ms;
static uint32_t s_pos_ms;
static uint32_t s_last_pos_ms;
static uint32_t s_last_poll_ms;
static uint32_t s_last_pairing_ms;
static uint32_t s_last_track_change_ms;

static beken_thread_t s_init_thread;
static beken_mutex_t s_lock;

static void bt_lock(void)
{
	if (!s_lock)
		rtos_init_mutex(&s_lock);
	if (s_lock)
		rtos_lock_mutex(&s_lock);
}

static void bt_unlock(void)
{
	if (s_lock)
		rtos_unlock_mutex(&s_lock);
}


static int utf8_put_cp(uint32_t cp, char *out, int pos, int outlen)
{
	if (cp < 0x80) {
		if (pos + 1 < outlen) out[pos++] = (char)cp;
	} else if (cp < 0x800) {
		if (pos + 2 < outlen) {
			out[pos++] = (char)(0xC0 | (cp >> 6));
			out[pos++] = (char)(0x80 | (cp & 0x3F));
		}
	} else if (cp < 0x10000) {
		if (pos + 3 < outlen) {
			out[pos++] = (char)(0xE0 | (cp >> 12));
			out[pos++] = (char)(0x80 | ((cp >> 6) & 0x3F));
			out[pos++] = (char)(0x80 | (cp & 0x3F));
		}
	} else {
		if (pos + 4 < outlen) {
			out[pos++] = (char)(0xF0 | (cp >> 18));
			out[pos++] = (char)(0x80 | ((cp >> 12) & 0x3F));
			out[pos++] = (char)(0x80 | ((cp >> 6) & 0x3F));
			out[pos++] = (char)(0x80 | (cp & 0x3F));
		}
	}
	return pos;
}

/* AVRCP 的 TITLE 可能是 UTF-8，也可能是 UTF-16。手机厂实现不一致，
 * 这里按字符集 + 零字节分布做兜底识别。 */
static void avrcp_text_to_utf8(const char *in, uint32_t len, uint16_t charset,
                               char *out, size_t outlen)
{
	uint32_t zeros = 0;
	bool be = false;
	int pos = 0;

	if (!in || !out || outlen == 0)
		return;

	out[0] = '\0';

	if (charset == 3 || charset == 4 || charset == 106) {
		uint32_t n = len;

		if (n >= outlen)
			n = (uint32_t)outlen - 1;
		memcpy(out, in, n);
		out[n] = '\0';
		return;
	}

	for (uint32_t i = 0; i < len && i < 64; i++) {
		if (in[i] == 0)
			zeros++;
	}

	if ((len >= 2) && (zeros > (len / 8))) {
		uint32_t i = 0;

		if ((in[0] == (char)0xFE) && (in[1] == (char)0xFF)) {
			be = true;
			i = 2;
		} else if ((in[0] == (char)0xFF) && (in[1] == (char)0xFE)) {
			be = false;
			i = 2;
		} else if ((len >= 4) && (in[0] == 0) && (in[1] != 0)) {
			be = true;
		}

		for (; i + 1 < len; i += 2) {
			uint32_t u = be ? (((uint8_t)in[i] << 8) | (uint8_t)in[i + 1])
			                : ((uint8_t)in[i] | ((uint8_t)in[i + 1] << 8));

			if (u == 0)
				break;
			pos = utf8_put_cp(u, out, pos, (int)outlen);
		}
		out[pos] = '\0';
		return;
	}

	{
		uint32_t n = len;

		if (n >= outlen)
			n = (uint32_t)outlen - 1;
		memcpy(out, in, n);
		out[n] = '\0';
	}
}

static void trim_copy(char *dst, size_t dst_len, const char *src, uint32_t len)
{
	size_t n = 0;

	if (!dst || dst_len == 0)
		return;
	if (!src)
		len = 0;

	while (n + 1 < dst_len && n < len && src[n]) {
		char c = src[n];

		if ((c == '\r') || (c == '\n') || (c == '\t'))
			c = ' ';
		dst[n] = c;
		n++;
	}
	dst[n] = '\0';

	/* 去尾部空格 */
	while (n > 0 && dst[n - 1] == ' ')
		dst[--n] = '\0';
}

static void publish_locked(void)
{
	app_state_t *st = app_state_raw();
	app_track_t *tr = &st->bt;

	st->bt_connected = s_connected;
	st->bt_streaming = s_streaming;
	st->bt_pairing = !s_connected;
	/* A2DP sink 默认 SBC；格式胶囊只反映"当前有音频流" */
	os_strncpy(st->codec, s_streaming ? "SBC" : "", sizeof(st->codec) - 1);
	st->codec[sizeof(st->codec) - 1] = '\0';
	os_strncpy(tr->title, s_song_title, sizeof(tr->title) - 1);
	tr->title[sizeof(tr->title) - 1] = '\0';
	os_strncpy(tr->artist, s_artist, sizeof(tr->artist) - 1);
	tr->artist[sizeof(tr->artist) - 1] = '\0';
	os_strncpy(tr->album, s_album, sizeof(tr->album) - 1);
	tr->album[sizeof(tr->album) - 1] = '\0';
	os_strncpy(tr->lyric_cur, s_lyric_cur, sizeof(tr->lyric_cur) - 1);
	tr->lyric_cur[sizeof(tr->lyric_cur) - 1] = '\0';
	os_strncpy(tr->lyric_prev, s_lyric_prev, sizeof(tr->lyric_prev) - 1);
	tr->lyric_prev[sizeof(tr->lyric_prev) - 1] = '\0';
	tr->pos_ms = s_pos_ms;
	tr->dur_ms = s_dur_ms;
	tr->has_lyric = s_has_lyric;

	if (st->mode == APP_MODE_BT) {
		if (s_streaming && s_playing)
			st->play = APP_PLAY_PLAYING;
		else if (s_streaming)
			st->play = APP_PLAY_PAUSED;
		else
			st->play = APP_PLAY_STOPPED;
	}
}

static void publish(void)
{
	bt_lock();
	app_state_lock();
	publish_locked();
	app_state_unlock();
	bt_unlock();
}

/* ==================== 元数据攒批：歌名防抖的核心 ====================
 *
 * 现场现象（AVRCP 日志）：换歌瞬间手机会连发多条 GET_ELEMENT_ATTRIBUTES
 * 响应，其中常常先来一条 count=0 的空包，紧接着才是真数据；而且每秒的
 * 轮询都会把**完全相同的四个属性**再发一遍。
 *
 * 旧实现的两个毛病：
 *   ① 换歌时 reset_lyrics() 直接把歌名清空 → 界面先闪一下空白，
 *      8ms 后才被新歌名填上（"抖一下"）；
 *   ② ARTIST/ALBUM 分支无条件 changed=true → 每秒一次 publish，
 *      整块 BT 文本区每秒重绘一次（看起来像持续轻微闪烁）。
 *
 * 新做法：
 *   · 换歌进入 COLLECTING，TITLE/ARTIST/ALBUM 只写暂存区，**不落库、不 publish**；
 *   · 三个字段到齐、或超过 BT_META_COMMIT_MS 后**一次性提交**，
 *     并且只在值真的变了时才 publish —— 旧歌到新歌只有一次可见变化；
 *   · 换歌时**不清旧歌名**，界面一直显示上一首直到新值落地，杜绝空白帧；
 *   · 空值/占位符（"Unknown"/"Not Provided"）一律丢弃，绝不覆盖旧值；
 *   · 与已应用值相同的响应直接忽略（手机每秒重发同样的属性）。
 */
#define BT_META_COMMIT_MS  700

static char s_stage_title[APP_TITLE_LEN];
static char s_stage_artist[APP_ARTIST_LEN];
static char s_stage_album[APP_ALBUM_LEN];
static bool s_stage_has_title;
static bool s_stage_has_artist;
static bool s_stage_has_album;
static uint32_t s_stage_ms;          /* COLLECTING 的起点 */
static bool s_meta_collecting;

static void stage_reset_locked(void)
{
	s_stage_has_title = false;
	s_stage_has_artist = false;
	s_stage_has_album = false;
	s_stage_title[0] = '\0';
	s_stage_artist[0] = '\0';
	s_stage_album[0] = '\0';
}

/* 【歌词占位串：绝不能当成歌名】
 *
 * 国内音乐 App 的"车载歌词"会把 AVRCP 的 TITLE 改写成当前歌词行。
 * 纯音乐/无歌词的曲目拿不到歌词，App 就会塞一条占位串，例如
 *   纯音乐 / 纯音乐，请欣赏 / 此歌曲为没有填词的纯音乐，请欣赏 /
 *   暂无歌词 / 无歌词 / 该歌曲没有歌词
 * 旧逻辑把"换歌后的第一条 TITLE"无条件当作歌名，于是这类曲目的歌名
 * 就变成了"纯音乐"。这里把它们识别出来，只送进歌词区，不锁歌名。
 *
 * 匹配规则用**前缀**而不是"包含"：真实歌名里可能出现"纯音乐"三个字
 * （如"纯音乐合集"），但极少会以它开头。 */
static bool title_is_lyric_placeholder(const char *s)
{
	static const char *const prefix_marks[] = {
		"纯音乐",
		"暂无歌词",
		"无歌词",
		"没有歌词",
		"该歌曲没有",
		"此歌曲为",
		"请欣赏",
		"instrumental",
		"no lyrics",
		"karaoke",
		"lyrics not",
	};

	for (size_t i = 0; i < sizeof(prefix_marks) / sizeof(prefix_marks[0]); i++) {
		size_t n = strlen(prefix_marks[i]);

		if (!strncasecmp(s, prefix_marks[i], n))
			return true;
	}
	return false;
}

/* 把暂存区一次性提交（调用者需持 bt_lock）。
 * 只覆盖"有值且不同"的字段；空值不会把已显示的内容抹掉。 */
static void meta_commit_locked(void)
{
	bool diff = false;

	if (s_stage_has_title) {
		if (title_is_lyric_placeholder(s_stage_title)) {
			/* "纯音乐，请欣赏"这类是歌词占位，不是歌名：
			 * 送进歌词区显示，**不设 s_title_locked**，等真正的歌名到来。
			 * 一直等不到也没关系 —— 保留上一首的歌名，总比把"纯音乐"
			 * 当歌名显示要合理。 */
			if (os_strcmp(s_stage_title, s_lyric_cur) != 0) {
				os_strncpy(s_lyric_cur, s_stage_title, sizeof(s_lyric_cur) - 1);
				s_lyric_cur[sizeof(s_lyric_cur) - 1] = '\0';
				s_has_lyric = true;
				diff = true;
			}
		} else if (s_stage_title[0] && (os_strcmp(s_stage_title, s_song_title) != 0)) {
			os_strncpy(s_song_title, s_stage_title, sizeof(s_song_title) - 1);
			s_song_title[sizeof(s_song_title) - 1] = '\0';
			s_title_locked = true;
			diff = true;
		} else if (s_stage_title[0]) {
			/* 值相同也要标记已锁定，否则后续 TITLE 会被当成歌词 */
			s_title_locked = true;
		}
	}

	if (s_stage_has_artist && s_stage_artist[0] &&
	    (os_strcmp(s_stage_artist, s_artist) != 0)) {
		os_strncpy(s_artist, s_stage_artist, sizeof(s_artist) - 1);
		s_artist[sizeof(s_artist) - 1] = '\0';
		diff = true;
	}

	if (s_stage_has_album && s_stage_album[0] &&
	    (os_strcmp(s_stage_album, s_album) != 0)) {
		os_strncpy(s_album, s_stage_album, sizeof(s_album) - 1);
		s_album[sizeof(s_album) - 1] = '\0';
		diff = true;
	}

	s_meta_collecting = false;
	stage_reset_locked();
	s_stage_ms = 0;

	if (diff) {
		app_state_lock();
		publish_locked();
		app_state_unlock();
	}
}

/* 到齐或超时就提交（调用者需持 bt_lock）。由属性回调和 app_bt_poll 共同驱动：
 * 手机只给 TITLE 不给 ARTIST 时，得靠轮询里的超时提交兜底。 */
static void meta_try_commit_locked(uint32_t now)
{
	if (!s_meta_collecting)
		return;

	if (s_stage_has_title && s_stage_has_artist && s_stage_has_album) {
		meta_commit_locked();
		return;
	}

	if (s_stage_ms && ((now - s_stage_ms) >= BT_META_COMMIT_MS))
		meta_commit_locked();
}

static void reset_track_locked(bool clear_meta)
{
	s_has_lyric = false;
	s_title_locked = false;
	if (clear_meta)
		s_song_title[0] = '\0';
	s_lyric_cur[0] = '\0';
	s_lyric_prev[0] = '\0';
	s_track_start_ms = rtos_get_time();

	stage_reset_locked();
	if (clear_meta) {
		s_meta_collecting = false;
		s_stage_ms = 0;
	} else {
		s_meta_collecting = true;    /* 换歌：开始攒新元数据 */
		s_stage_ms = s_track_start_ms;
	}
}

/* 换歌的统一落地点（调用者需持 bt_lock）。
 *
 * 三个触发源：
 *   ① TRACK_CHANGE 通知        —— 最直接，但会重复/缺失
 *   ② ARTIST 文本变化          —— 歌词行绝不会改 ARTIST，最可靠
 *   ③ PLAYING_TIME（曲目总长） —— 歌词行也绝不会改它，次可靠
 *
 * 注意这里**不清歌名/歌手/专辑**：继续显示上一首，等新元数据攒齐后
 * 由 meta_commit_locked() 一次性替换。清空会让界面闪一下空白。 */
static void confirm_track_change_locked(const char *reason)
{
	reset_track_locked(false);
	s_pos_ms = 0;
	s_last_pos_ms = 0;
	s_dur_ms = 0;
	s_last_track_change_ms = s_track_start_ms;
	app_state_lock();
	publish_locked();      /* 只为了让歌词区立刻清空 */
	app_state_unlock();
	LOGI("track changed (%s), collecting meta\r\n", reason);
}

static void bt_ensure_discoverable(void)
{
	bt_err_t err = bk_bt_gap_set_visibility(BK_BT_CONNECTABLE, BK_BT_DISCOVERABLE);

	if (err)
		LOGW("set visibility/discoverable failed: %d\r\n", err);
}

void app_bt_early_init(void)
{
	int ret;

	if (s_stack_ready)
		return;

	if (!s_lock)
		rtos_init_mutex(&s_lock);

	ret = bt_manager_init();
	if (ret != 0) {
		LOGE("bt_manager_init (early) failed: %d\r\n", ret);
		return;
	}

	s_stack_ready = true;
	LOGI("bt stack early ready\r\n");
}

static void bt_init_thread(void *arg)
{
	int ret;

	(void)arg;

	LOGI("bt init start\r\n");

	if (!s_stack_ready) {
		ret = bt_manager_init();
		if (ret != 0) {
			LOGE("bt_manager_init failed: %d\r\n", ret);
			app_state_toast("蓝牙初始化失败");
			goto out;
		}
		s_stack_ready = true;
		LOGI("bt stack ready\r\n");
	}

	/* 先保证一次可发现，即使后面 A2DP 初始化失败，手机也能搜到 */
	if (s_want_active && !s_connected) {
		bt_ensure_discoverable();
		bt_manager_set_mode(BT_MNG_MODE_PAIRING);
	}

	ret = a2dp_sink_demo_init(0);
	if (ret != 0) {
		LOGE("a2dp_sink_demo_init failed: %d\r\n", ret);
		app_state_toast("A2DP 初始化失败");
		goto out;
	}

	/* 【HFP 已停用】原实现在 app_hfp.c（16k + mSBC + 软件 AEC）。
	 * 停用原因：现场没有可测试的通话环境，而它和 A2DP 共用 DAC 和
	 * BT 链路，出问题时无法区分是通话抢占还是 A2DP 本身的问题。
	 * 源码保留在 ap/app/app_hfp.c，但已从 CMakeLists 移除、不再编译。 */

	s_ready = true;
	LOGI("bt ready\r\n");

	bt_lock();
	{
		uint8_t mac[6] = {0};
		char name[APP_NAME_LEN];

		bk_bluetooth_get_address(mac);
		snprintf(name, sizeof(name), "bk7258_soundbox");
		(void)mac;
		app_state_lock();
		os_strncpy(app_state_raw()->bt_name, name,
		           sizeof(app_state_raw()->bt_name) - 1);
		app_state_unlock();
	}
	bt_unlock();

	if (s_want_active)
		app_bt_set_active(true);

out:
	s_init_thread = NULL;
	rtos_delete_thread(NULL);
}

void app_bt_init(void)
{
	if (!s_lock)
		rtos_init_mutex(&s_lock);

	s_active = false;
	s_connected = false;
	s_streaming = false;
	s_ready = false;
	/* 不要清 s_stack_ready：main 里 app_bt_early_init() 可能已经初始化过协议栈 */
	reset_track_locked(true);   /* 初始化：连歌名一起清，且不进入攒批状态 */
	s_artist[0] = '\0';
	s_album[0] = '\0';

	if (rtos_create_thread(&s_init_thread, BEKEN_DEFAULT_WORKER_PRIORITY - 1,
	                       "app_bt_init", (beken_thread_function_t)bt_init_thread,
	                       BT_INIT_STACK, NULL) != kNoErr) {
		s_init_thread = NULL;
		LOGE("create bt init thread failed\r\n");
	}
}

bool app_bt_is_ready(void)
{
	return s_ready;
}

bool app_bt_is_connected(void)
{
	return s_connected;
}

void app_bt_set_active(bool active)
{
	s_want_active = active;
	s_active = active;
	bt_audio_sink_set_output_active(active);

	if (!s_stack_ready)
		return;

	if (active) {
		LOGI("enter bt mode\r\n");
		/* 未连接时始终可发现；即使后台还在回连旧手机，也要让用户能搜到 */
		if (!s_connected) {
			bt_ensure_discoverable();
			bt_manager_set_mode(BT_MNG_MODE_PAIRING);
		}
	} else {
		LOGI("leave bt mode\r\n");
		/* 离开时让手机暂停，避免 A2DP 数据一直占蓝牙带宽 */
		if (s_connected && s_avrcp_connected && s_playing)
			bk_bt_app_avrcp_ct_pause();
		s_streaming = false;
		s_playing = false;
		publish();
	}
}

void app_bt_play_pause(void)
{
	if (!s_ready || !s_connected || !s_avrcp_connected) {
		app_state_toast("蓝牙未连接 / AVRCP 未就绪");
		return;
	}

	if (s_playing) {
		bk_bt_app_avrcp_ct_pause();
		s_playing = false;
	} else {
		bk_bt_app_avrcp_ct_play();
		s_playing = true;
	}
	publish();
}

void app_bt_next(void)
{
	if (s_ready && s_connected && s_avrcp_connected)
		bk_bt_app_avrcp_ct_next();
}

void app_bt_prev(void)
{
	if (s_ready && s_connected && s_avrcp_connected)
		bk_bt_app_avrcp_ct_prev();
}

void app_bt_seek(int delta_ms)
{
	/* AVRCP 只有 REWIND / FAST_FORWARD 的「按下-保持-松开」，没有绝对定位命令。
	 * 保持 250ms 让手机识别为一次快进/快退；实际跳多少取决于手机 App
	 * （多数按一次跳 10~30s），所以蓝牙下无法保证精确 ±10s。
	 * 这里不发 toast：短按是高频操作，避免刷屏重绘。 */
	if (!s_ready || !s_connected || !s_avrcp_connected)
		return;

	if (delta_ms > 0)
		bk_bt_app_avrcp_ct_fast_forward(250);
	else if (delta_ms < 0)
		bk_bt_app_avrcp_ct_rewind(250);
}

void app_bt_enter_pairing(void)
{
	if (!s_ready) {
		app_state_toast("蓝牙尚未就绪");
		return;
	}

	bk_bt_enter_pairing_mode();
	bt_manager_set_mode(BT_MNG_MODE_PAIRING);
	app_state_toast("蓝牙配对模式");
}

void app_bt_report_volume(uint8_t percent)
{
	if (!s_ready)
		return;

	bt_avrcp_report_volume_percent(percent);
}

void app_bt_poll(void)
{
	uint32_t now;
	uint8_t st;

	if (!s_stack_ready)
		return;

	now = rtos_get_time();
	st = bt_manager_get_connect_state();

	/* 手机拖动音量后的 CHANGED 回发（去抖 80ms），见 a2dp_sink_demo.c */
	bt_avrcp_volume_poll();

	/* 换歌元数据攒批的超时提交：手机可能只给 TITLE（不给 ARTIST/ALBUM），
	 * 那种情况只能靠这里在 700ms 后把已到的字段落地。
	 * poll 周期 ~200ms，所以超时误差不超过一个 poll。 */
	if (s_meta_collecting) {
		bt_lock();
		meta_try_commit_locked(now);
		bt_unlock();
	}

	/* ==================== 心跳（每 5 秒一行，无条件打印） ====================
	 *
	 * 排查"蓝牙放一会儿卡住"用。之前只在有包时打日志，一旦卡住就完全静默，
	 * 分不清是主机不送数据还是本机音频通路死了 —— 这一行不管有没有音频都会打：
	 *
	 *   pkt 不再增长            -> 主机侧断流（手机暂停 / A2DP 断了 / 链路卡）
	 *   pkt 在涨、rst 也在涨     -> 本机音频通路反复被看门狗重建
	 *   pkt 在涨、open=1、fill 正常但没声音 -> DAC/功放那一侧
	 *
	 * conn/stream 是本地记录的状态机，open 是 A2DP speaker 线程实际持有的
	 * 音频通路，heap/psram 用来看是否又被内存拖死。 */
	{
		static uint32_t s_beat_ms;
		static uint32_t s_pkt_last;

		if ((now - s_beat_ms) >= 5000) {
			uint32_t pkt = 0, drop = 0, rst = 0;

			s_beat_ms = now;
			bt_audio_sink_stats(&pkt, &drop, &rst);
			LOGI("bt beat: conn=%d stream=%d play=%d pkt=%u(+%u) drop=%u rst=%u open=%d heap=%uK psram=%uK\r\n",
			     (int)s_connected, (int)s_streaming, (int)s_playing,
			     (unsigned)pkt, (unsigned)(pkt - s_pkt_last),
			     (unsigned)drop, (unsigned)rst,
			     (int)bt_audio_sink_is_audio_opened(),
			     (unsigned)(rtos_get_free_heap_size() / 1024),
			     (unsigned)(rtos_get_psram_free_heap_size() / 1024));
			s_pkt_last = pkt;
		}
	}

	if ((st == BT_STATE_PROFILE_CONNECTED) || (st == BT_STATE_LINK_CONNECTED)) {
		if (!s_connected) {
			s_connected = true;
			publish();
		}
	} else if (s_connected) {
		s_connected = false;
		s_streaming = false;
		s_playing = false;
		publish();
	}

	if (!s_active)
		return;

	/* 未连接时每 3 秒确保一次可发现/可连接，避免回连旧手机期间搜不到。
	 * 直接写 visibility，不依赖 btm_env.mode，防止状态机提前 return。 */
	if (!s_connected && ((now - s_last_pairing_ms) >= 3000)) {
		s_last_pairing_ms = now;
		bt_ensure_discoverable();
		bt_manager_set_mode(BT_MNG_MODE_PAIRING);
	}

	if (!s_avrcp_connected)
		return;

	if ((now - s_last_poll_ms) >= BT_POLL_ATTR_MS) {
		s_last_poll_ms = now;
		bt_avrcp_poll_attrs();
	}

	/* 暂停时进度不涨，保持即可 */
}

/* ==========================================================================
 * 以下函数由 a2dp_sink_demo.c 调用（蓝牙协议栈上下文）
 * ========================================================================== */

void app_bt_on_a2dp_connected(void)
{
	bool need_pub = !s_connected;

	s_connected = true;
	if (need_pub) {
		LOGI("a2dp connected, heap=%u psram=%u\r\n",
		     (unsigned)rtos_get_free_heap_size(),
		     (unsigned)rtos_get_psram_free_heap_size());
		publish();
	}

	/* 手机连上并开始出声：如果当前不在播放别的模式，自动切到蓝牙 */
	app_mode_request(APP_MODE_BT);
}

void app_bt_on_a2dp_start(uint32_t sample_rate, uint8_t channels)
{
	(void)sample_rate;
	(void)channels;

	s_streaming = true;
	s_playing = true;
	publish();
	app_mode_request(APP_MODE_BT);
}

void app_bt_on_a2dp_stop(void)
{
	s_streaming = false;
	s_playing = false;
	publish();
}

void app_bt_on_avrcp_connected(bool connected)
{
	s_avrcp_connected = connected;
	if (connected) {
		s_last_poll_ms = 0;
		/* 先把自己的音量同步给对端：对端拿到的第一个 INTERIM 值就是本机
		 * 实际音量，避免手机/Windows 滑块停在存储里的旧值上。 */
		bt_avrcp_report_volume_percent(app_state_volume());
		bt_avrcp_poll_attrs();
		bt_avrcp_request_cover_art();
	}
}

void app_bt_on_track_change(void)
{
	uint32_t now = rtos_get_time();

	/* 部分手机/Windows 会连着发 TRACK_CHANGE（有的甚至每首歌、每次重注册通知
	 * 都发一次）。重复通知只清歌词上下文，**不动歌名** —— 否则紧接着轮询到的
	 * TITLE（其实是歌词行）就会被当成新歌名。 */
	if (s_last_track_change_ms && ((now - s_last_track_change_ms) < BT_TRACK_CHANGE_MS)) {
		bt_lock();
		s_lyric_cur[0] = '\0';
		s_lyric_prev[0] = '\0';
		s_has_lyric = false;
		s_pos_ms = 0;
		s_last_pos_ms = 0;
		app_state_lock();
		publish_locked();
		app_state_unlock();
		bt_unlock();
		LOGW("track change debounced (%ums), keep title\r\n",
		     (unsigned)(now - s_last_track_change_ms));
		return;
	}

	bt_lock();
	confirm_track_change_locked("TRACK_CHANGE");
	bt_unlock();

	bt_avrcp_request_cover_art();

	/* 【换歌后立刻取一次属性，别等下一个 1 秒轮询】
	 *
	 * 手机切歌时，音乐 App 可能已经把 AVRCP 的 TITLE 改写成"当前歌词行"
	 * 甚至"纯音乐，请欣赏"这类占位串。越晚取样，拿到真实歌名的机会越小。
	 * 这里主动踢一次，把取样点尽量提前到切歌瞬间。
	 * （s_last_poll_ms 清零，让主循环的定时轮询重新计时，避免紧接着
	 *  又发一条重复请求。） */
	s_last_poll_ms = 0;
	bt_avrcp_poll_attrs();
}

void app_bt_on_play_status(uint8_t status)
{
	switch (status) {
	case BK_AVRCP_PLAYBACK_PLAYING:
		s_playing = true;
		break;
	case BK_AVRCP_PLAYBACK_PAUSED:
		s_playing = false;
		break;
	case BK_AVRCP_PLAYBACK_STOPPED:
		s_playing = false;
		s_streaming = false;
		break;
	default:
		break;
	}
	publish();
}

void app_bt_on_play_pos(uint32_t pos_ms)
{
	/* AVRCP 里 0xFFFFFFFF 表示“播放位置不可用”，Windows 每次通知都会带这个值。
	 * 旧代码把它当成“进度大幅回退 -> 换歌”，于是每次都重新锁定歌名，
	 * 下一次轮询到的 TITLE（其实是歌词行）就把歌名覆盖掉了。
	 * 现在：非法值直接丢弃，且不再用进度回退推断换歌 —— 只认 TRACK_CHANGE。 */
	if ((pos_ms == 0xFFFFFFFFu) || (pos_ms > (24u * 3600u * 1000u)))
		return;

	s_last_pos_ms = pos_ms;
	s_pos_ms = pos_ms;
	publish();
}

static bool avrcp_text_is_placeholder(const char *s)
{
	static const char *const words[] = {
		"not provided", "not provide", "not available",
		"unknown", "n/a", "none",
	};

	for (size_t i = 0; i < sizeof(words) / sizeof(words[0]); i++) {
		if (!strcasecmp(s, words[i]))
			return true;
	}
	return false;
}

void app_bt_on_avrcp_attr(uint32_t attr_id, const char *text, uint32_t len,
                          uint16_t charset)
{
	char tmp[APP_LYRIC_LEN];
	uint32_t now = rtos_get_time();
	bool changed = false;

	/* Cover Art image handle 是 7 字节 ASCII，不要走 UTF-8 转换 */
	if (attr_id == BK_AVRCP_MEDIA_ATTR_ID_DEFAULT_COVER_ART) {
		char handle[16];
		uint32_t n = (len < sizeof(handle) - 1) ? len : sizeof(handle) - 1;

		memcpy(handle, text, n);
		handle[n] = '\0';
		while (n > 0 && (handle[n - 1] == ' ' || handle[n - 1] == '\r' || handle[n - 1] == '\n'))
			handle[--n] = '\0';
		if (n > 0)
			bt_cover_art_on_handle(handle, n);
		return;
	}

	avrcp_text_to_utf8(text, len, charset, tmp, sizeof(tmp));
	trim_copy(tmp, sizeof(tmp), tmp, strlen(tmp));
	if (tmp[0] == '\0')
		return;

	/* 占位符（"Unknown"/"Not Provided"/"N/A"）一律丢弃，**不再清空已有内容**。
	 * 旧实现会把 TITLE 清空，于是每次手机重发一条占位属性，界面上歌名就
	 * 闪一下空白 —— 也是"歌名抖动"的成因之一。宁可保留上一首的信息，
	 * 等真正的值到了再替换。 */
	if (avrcp_text_is_placeholder(tmp))
		return;

	bt_lock();

	switch (attr_id) {
	case BK_AVRCP_MEDIA_ATTR_ID_TITLE:
	{
		static char s_last_title_log[APP_LYRIC_LEN];

		if (strcmp(s_last_title_log, tmp)) {
			os_strncpy(s_last_title_log, tmp, sizeof(s_last_title_log) - 1);
			s_last_title_log[sizeof(s_last_title_log) - 1] = '\0';
			LOGI("avrcp TITLE: %s\r\n", tmp);
		}

		/* ① 换歌攒批窗口：只写暂存区，不落库、不 publish。
		 *    三字段到齐或 700ms 超时后由 meta_commit_locked() 一次性提交，
		 *    这样标题/歌手/专辑是同帧替换，不会出现"标题先变、歌手还是旧的"。 */
		if (s_meta_collecting) {
			os_strncpy(s_stage_title, tmp, sizeof(s_stage_title) - 1);
			s_stage_title[sizeof(s_stage_title) - 1] = '\0';
			s_stage_has_title = true;
			meta_try_commit_locked(now);
			break;
		}

		/* 车载歌词协议：手机在同一首歌内不断改写 TITLE。
		 * 换歌后的第一条 TITLE 永久保存为歌名；之后的 TITLE 变化才当歌词。
		 * 注意这里**不把歌名写进歌词**：歌名已经在标题栏，写进歌词会让
		 * 歌词区先闪一下歌名再跳到首行歌词（看起来像"歌名/歌词串行"）。 */
		if (!s_title_locked) {
			/* "纯音乐，请欣赏"这类歌词占位串：只进歌词区，不当歌名，
			 * 也不锁歌名（等真正的歌名；等不到就保留上一首的）。 */
			if (title_is_lyric_placeholder(tmp)) {
				if (os_strcmp(tmp, s_lyric_cur) != 0) {
					os_strncpy(s_lyric_cur, tmp, sizeof(s_lyric_cur) - 1);
					s_lyric_cur[sizeof(s_lyric_cur) - 1] = '\0';
					s_has_lyric = true;
					changed = true;
				}
				break;
			}

			/* 与已显示内容相同就不算变化（手机每秒重发同样的属性） */
			if (os_strcmp(tmp, s_song_title) != 0) {
				os_strncpy(s_song_title, tmp, sizeof(s_song_title) - 1);
				s_song_title[sizeof(s_song_title) - 1] = '\0';
				changed = true;
			}
			s_title_locked = true;
			s_last_title_ms = now;
		} else if (os_strcmp(tmp, s_song_title) != 0) {
			/* 换歌窗口内：既不是第一条歌名，也不该当歌词。
			 * 手机切歌瞬间常见"歌名 → 首行歌词 → 第二行"的快速序列，
			 * 窗口内的额外值一律丢弃，避免把首行歌词当歌名、或让歌词乱跳。 */
			if ((now - s_track_start_ms) < BT_TITLE_GUARD_MS) {
				LOGW("title in guard window, ignored: %s\r\n", tmp);
				break;
			}

			/* 同一首歌内 TITLE 变化 -> 当前歌词行。
			 * 防抖三件套：
			 *  ① 距上次更新 <400ms 视为抖动（轮询 1s 一次，正常不会走到）；
			 *  ② 与当前行相同 -> 重复值，忽略（不 publish）；
			 *  ③ 与“上一行”相同 -> A->B->A 回跳（有的 App 在主标题/副标题
			 *     之间来回切），忽略；否则歌词会来回闪。 */
			if (((now - s_last_title_ms) >= 400) &&
			    (os_strcmp(tmp, s_lyric_cur) != 0) &&
			    (os_strcmp(tmp, s_lyric_prev) != 0)) {
				os_strncpy(s_lyric_prev, s_lyric_cur, sizeof(s_lyric_prev) - 1);
				s_lyric_prev[sizeof(s_lyric_prev) - 1] = '\0';
				os_strncpy(s_lyric_cur, tmp, sizeof(s_lyric_cur) - 1);
				s_lyric_cur[sizeof(s_lyric_cur) - 1] = '\0';
				s_has_lyric = true;
				s_last_title_ms = now;
				changed = true;
			}
		}
	}
	break;

	case BK_AVRCP_MEDIA_ATTR_ID_ARTIST:
		/* 换歌攒批窗口：暂存，等 TITLE/ALBUM 一起提交 */
		if (s_meta_collecting) {
			os_strncpy(s_stage_artist, tmp, sizeof(s_stage_artist) - 1);
			s_stage_artist[sizeof(s_stage_artist) - 1] = '\0';
			s_stage_has_artist = true;
			meta_try_commit_locked(now);
			break;
		}

		/* 歌词行只会改 TITLE，绝不会改 ARTIST。ARTIST 变了就是真换歌
		 * （有的手机/播放器根本不发 TRACK_CHANGE，只能靠这里兜住）。
		 * 判成换歌后本次值先不落库：进入攒批窗口，等这一轮的 TITLE
		 * 一起提交，避免"歌手先变、歌名还是上一首"的中间态。 */
		if (s_artist[0] && (os_strcmp(s_artist, tmp) != 0) &&
		    ((now - s_track_start_ms) > BT_CONFIRM_MIN_MS)) {
			confirm_track_change_locked("ARTIST");
			break;
		}

		if (os_strcmp(s_artist, tmp) != 0) {
			os_strncpy(s_artist, tmp, sizeof(s_artist) - 1);
			s_artist[sizeof(s_artist) - 1] = '\0';
			changed = true;
		}
		break;

	case BK_AVRCP_MEDIA_ATTR_ID_ALBUM:
		if (s_meta_collecting) {
			os_strncpy(s_stage_album, tmp, sizeof(s_stage_album) - 1);
			s_stage_album[sizeof(s_stage_album) - 1] = '\0';
			s_stage_has_album = true;
			meta_try_commit_locked(now);
			break;
		}

		if (os_strcmp(s_album, tmp) != 0) {
			os_strncpy(s_album, tmp, sizeof(s_album) - 1);
			s_album[sizeof(s_album) - 1] = '\0';
			changed = true;
		}
		break;

	case BK_AVRCP_MEDIA_ATTR_ID_PLAYING_TIME:
	{
		uint32_t total = 0;

		for (uint32_t i = 0; (i < len) && (text[i] >= '0') && (text[i] <= '9'); i++)
			total = total * 10 + (uint32_t)(text[i] - '0');

		if (total > 0) {
			/* 曲目总长变化超过 1.5s 视为换歌（歌词刷新不会改总长） */
			uint32_t diff = (total > s_dur_ms) ? (total - s_dur_ms)
			                                   : (s_dur_ms - total);

			if (s_dur_ms && (diff > 1500) &&
			    ((now - s_track_start_ms) > BT_CONFIRM_MIN_MS)) {
				confirm_track_change_locked("PLAYING_TIME");
				/* 新曲目的时长直接落库：它不属于"元数据攒批"三件套 */
				s_dur_ms = total;
				changed = true;
				break;
			}

			/* 值相同就不 publish —— 手机每秒都会重发同样的 PLAYING_TIME */
			if (total != s_dur_ms) {
				s_dur_ms = total;
				changed = true;
			}
		}
	}
	break;

	default:
		break;
	}

	if (changed) {
		app_state_lock();
		publish_locked();
		app_state_unlock();
	}

	bt_unlock();
}
