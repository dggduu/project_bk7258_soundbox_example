// Copyright 2026
// SD 卡模式实现：目录浏览 + 播放（bk_audio_player 组件）+ LRC/封面
//
// 解码与播放状态机由 SDK bk_audio_player 承担（见 sd_player.c），
// 本文件负责：文件浏览/播放列表、标签与歌词/封面（media_meta）、
// 状态发布（app_state）以及 seek 的 UI 语义。

#include "app_sd.h"
#include "png_cover.h"
#include "app_audio.h"
#include "app_mode.h"
#include "media_meta.h"
#include "sd_player.h"
#include "bsp.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include <components/log.h>

#include "ff.h"
#include "lv_jpeg_sw_decode.h"
#include <components/media_types.h>
#include <components/bk_jpeg_decode/bk_jpeg_decode_hw.h>
#include <components/bk_jpeg_decode/bk_jpeg_decode_types_hw.h>

#define TAG "app_sd"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define SD_PROGRESS_MS     200
#define SD_SUB_GAP_MS      400      /* 双语歌词两行的最大时间差 */
#define SD_SEEK_MARGIN_MS  1200     /* 快进时距曲尾的最小余量，避免 seek 触发切歌 */

static bool s_mounted;
static uint32_t s_pos_ms;
static uint32_t s_dur_ms;
static char s_cur_path[APP_PATH_LEN];

static media_info_t s_info;
static uint16_t *s_cover;
static uint32_t s_cover_gen;
static bool s_cover_valid;
static beken_mutex_t s_cover_lock;

static char s_playlist[APP_SD_MAX_PLAYLIST][APP_PATH_LEN];
static int s_pl_count;
static int s_pl_index;

/* ============================ 工具 ============================ */

static bool ext_is(const char *name, const char *ext)
{
	const char *dot = strrchr(name, '.');
	size_t n = strlen(ext);

	if (!dot)
		return false;
	if (strlen(dot + 1) != n)
		return false;
	for (size_t i = 0; i < n; i++) {
		if (tolower((unsigned char)dot[1 + i]) != tolower((unsigned char)ext[i]))
			return false;
	}
	return true;
}

static bool is_audio(const char *name)
{
	return ext_is(name, "mp3") || ext_is(name, "wav") || ext_is(name, "flac");
}

static void path_dir(const char *path, char *out, size_t outlen)
{
	const char *slash = strrchr(path, '/');

	if (!slash) {
		os_strncpy(out, "1:/", outlen);
		out[outlen - 1] = '\0';
		return;
	}

	size_t n = (size_t)(slash - path) + 1;

	if (n >= outlen)
		n = outlen - 1;
	memcpy(out, path, n);
	out[n] = '\0';
}

static void path_join(const char *dir, const char *name, char *out, size_t outlen)
{
	size_t n = strlen(dir);

	if (n && dir[n - 1] == '/')
		snprintf(out, outlen, "%s%s", dir, name);
	else
		snprintf(out, outlen, "%s/%s", dir, name);
}

static void str_tolower(char *s)
{
	for (; *s; s++)
		*s = (char)tolower((unsigned char)*s);
}

/* 行内是否含中日文（UTF-8：E3 = 假名/日文标点，E4~E9 = CJK 汉字） */
static bool line_has_cjk(const char *s)
{
	while (s && *s) {
		unsigned char c = (unsigned char)*s;

		if ((c == 0xE3) || ((c >= 0xE4) && (c <= 0xE9)))
			return true;
		s++;
	}
	return false;
}

/* 两行是否构成“原文 + 译文”：
 * 时间差 ≤400ms，且一行含中日文、另一行完全不含 —— 这样不会把
 * 正常的相邻两句歌词误配成双语（快歌每行 300ms 也不会连成串）。 */
static bool lyric_lines_pair(const media_lrc_line_t *a,
                             const media_lrc_line_t *b)
{
	uint32_t gap;

	if (!a || !b || !a->text[0] || !b->text[0])
		return false;

	gap = (b->time_ms >= a->time_ms) ? (b->time_ms - a->time_ms)
	                                 : (a->time_ms - b->time_ms);
	if (gap > SD_SUB_GAP_MS)
		return false;

	return (line_has_cjk(a->text) != line_has_cjk(b->text));
}

static void publish_track(void)
{
	app_state_t *st;
	app_track_t *tr;
	int idx;
	bool playing, paused;
	sd_player_state_t pst = sd_player_state();

	playing = (pst != SD_PLAYER_STOPPED);
	paused = (pst == SD_PLAYER_PAUSED);

	idx = media_lrc_index_at(&s_info, s_pos_ms);

	app_state_lock();
	st = app_state_raw();
	tr = &st->sd;

	st->sd_present = bsp_sd_is_inserted();
	st->sd_mounted = s_mounted;

	os_strncpy(tr->title, s_info.title, sizeof(tr->title) - 1);
	tr->title[sizeof(tr->title) - 1] = '\0';
	os_strncpy(tr->artist, s_info.artist, sizeof(tr->artist) - 1);
	tr->artist[sizeof(tr->artist) - 1] = '\0';
	os_strncpy(tr->album, s_info.album, sizeof(tr->album) - 1);
	tr->album[sizeof(tr->album) - 1] = '\0';
	tr->pos_ms = s_pos_ms;
	tr->dur_ms = s_dur_ms;
	tr->has_lyric = (s_info.line_count > 0);
	tr->has_cover = s_cover_valid;

	if (idx >= 0) {
		/* 双语 LRC 的两种常见写法：
		 *   a) 同一时间戳连着两行（原文 + 译文）
		 *   b) 两行时间戳相差几十~几百 ms，且一行是中日文、另一行是纯拉丁
		 * 命中就把“主行\n副行”塞进 lyric_cur，由 UI 拆成大字 + 小字两行。
		 * 只回退/配对一步，避免快歌（每行 300ms）被串成一串。 */
		int sub = -1;

		if ((idx > 0) && lyric_lines_pair(&s_info.lines[idx - 1],
		                                  &s_info.lines[idx]))
			idx--;

		if ((idx + 1 < s_info.line_count) &&
		    lyric_lines_pair(&s_info.lines[idx], &s_info.lines[idx + 1]))
			sub = idx + 1;

		if (sub >= 0)
			snprintf(tr->lyric_cur, sizeof(tr->lyric_cur), "%s\n%s",
			         s_info.lines[idx].text, s_info.lines[sub].text);
		else
			os_strncpy(tr->lyric_cur, s_info.lines[idx].text,
			           sizeof(tr->lyric_cur) - 1);
		tr->lyric_cur[sizeof(tr->lyric_cur) - 1] = '\0';

		if (idx > 0) {
			os_strncpy(tr->lyric_prev, s_info.lines[idx - 1].text,
			           sizeof(tr->lyric_prev) - 1);
			tr->lyric_prev[sizeof(tr->lyric_prev) - 1] = '\0';
		} else {
			tr->lyric_prev[0] = '\0';
		}
	} else {
		tr->lyric_cur[0] = '\0';
		tr->lyric_prev[0] = '\0';
	}

	st->sd_cover_gen = s_cover_gen;

	if (st->mode == APP_MODE_SD) {
		if (playing)
			st->play = paused ? APP_PLAY_PAUSED : APP_PLAY_PLAYING;
		else
			st->play = APP_PLAY_STOPPED;
	}

	app_state_unlock();
}

static void publish_path_status(void)
{
	app_state_t *st;

	app_state_lock();
	st = app_state_raw();
	st->sd_present = bsp_sd_is_inserted();
	st->sd_mounted = s_mounted;
	app_state_unlock();
}

/* ============================ 封面 ============================ */

/* 硬件 JPEG 解码（bk_jpeg_decoder / CONFIG_JPEG_DECODER_HW）：
 *   - 比软件解码快一个数量级，不占 CPU（软解 1000x1000 要几百 ms，
 *     期间 UI/按键全卡住），大封面才敢解；
 *   - 输出 **YUYV 全尺寸**到 PSRAM（每像素 2 字节），本文件再降采样到
 *     104x104 RGB565，所以峰值内存 = 原图 w*h*2，必须按 PSRAM 余量放行；
 *   - 渐进式 JPEG 硬件不支持，失败时回退软件解码。
 * 绝对尺寸上限只是防溢出，真正的门限是下面的 PSRAM 余量检查。 */
#define COVER_HW_MAX_DIM 4096

static bk_jpeg_decode_hw_handle_t s_jpeg_hw;
static bool s_jpeg_hw_ok;

/* YUV(JPEG 全范围) -> RGB565，并交换字节。
 * 交换是为了与软解路径（byte_swap=true）产出的字节序保持一致，
 * 否则同一个封面走两条路径会显示成颜色错乱。 */
static inline uint16_t yuv_to_rgb565_swapped(int y, int u, int v)
{
	int d = u - 128;
	int e = v - 128;
	int r = y + ((1436 * e) >> 10);            /* 1.402 */
	int g = y - ((352 * d + 731 * e) >> 10);   /* 0.344 / 0.714 */
	int b = y + ((1814 * d) >> 10);            /* 1.772 */
	uint16_t c;

	if (r < 0)
		r = 0;
	else if (r > 255)
		r = 255;
	if (g < 0)
		g = 0;
	else if (g > 255)
		g = 255;
	if (b < 0)
		b = 0;
	else if (b > 255)
		b = 255;

	c = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
	return (uint16_t)((c >> 8) | (c << 8));
}

/* YUYV 全图 -> dst(w*h) RGB565。
 * 亮度按目标像素对应的源矩形做**块平均**（大图直接点采样会有严重摩尔纹），
 * 色度取该矩形中心点的 U/V（人眼对色度分辨率不敏感）。
 * 复杂度和源图像素数成正比，3000x3000 约 9M 次字节访问，百毫秒级。 */
static void yuyv_to_rgb565(const uint8_t *src, uint32_t sw, uint32_t sh,
                           uint16_t *dst, int w, int h)
{
	for (int ty = 0; ty < h; ty++) {
		uint32_t sy0 = (uint32_t)ty * sh / (uint32_t)h;
		uint32_t sy1 = (uint32_t)(ty + 1) * sh / (uint32_t)h;

		if (sy1 <= sy0)
			sy1 = sy0 + 1;
		if (sy1 > sh)
			sy1 = sh;

		for (int tx = 0; tx < w; tx++) {
			uint32_t sx0 = (uint32_t)tx * sw / (uint32_t)w;
			uint32_t sx1 = (uint32_t)(tx + 1) * sw / (uint32_t)w;
			uint32_t sum = 0, n = 0, cx, cy, g;
			int y, u, v;

			if (sx1 <= sx0)
				sx1 = sx0 + 1;
			if (sx1 > sw)
				sx1 = sw;

			for (uint32_t sy = sy0; sy < sy1; sy++) {
				const uint8_t *row = src + sy * sw * 2;

				for (uint32_t sx = sx0; sx < sx1; sx++)
					sum += row[sx * 2];
				n += sx1 - sx0;
			}
			y = (int)(sum / (n ? n : 1));

			/* YUYV 以 2 像素为一组：Y0 U Y1 V，色度取组对齐的地址 */
			cx = (sx0 + sx1) / 2;
			cy = (sy0 + sy1) / 2;
			g = cx & ~1u;
			if ((g + 1) >= sw)
				g = (sw >= 2) ? (sw - 2) : 0;
			u = src[cy * sw * 2 + g * 2 + 1];
			v = src[cy * sw * 2 + g * 2 + 3];

			dst[ty * w + tx] = yuv_to_rgb565_swapped(y, u, v);
		}
	}
}

static bool jpeg_hw_init(void)
{
	bk_jpeg_decode_hw_config_t cfg;

	if (s_jpeg_hw_ok)
		return true;

	os_memset(&cfg, 0, sizeof(cfg));
	if (bk_hardware_jpeg_decode_new(&s_jpeg_hw, &cfg) != AVDK_ERR_OK) {
		LOGE("jpeg hw new failed\r\n");
		return false;
	}
	if (bk_jpeg_decode_hw_open(s_jpeg_hw) != AVDK_ERR_OK) {
		LOGE("jpeg hw open failed\r\n");
		bk_jpeg_decode_hw_delete(s_jpeg_hw);
		s_jpeg_hw = NULL;
		return false;
	}

	s_jpeg_hw_ok = true;
	return true;
}

static bool hw_cover_decode(const uint8_t *data, uint32_t len, uint16_t *dst,
                            int w, int h)
{
	frame_buffer_t in, out;
	bk_jpeg_decode_img_info_t info;
	uint32_t sw, sh, need, avail;
	uint32_t t0;

	if (!jpeg_hw_init())
		return false;

	os_memset(&in, 0, sizeof(in));
	in.frame = (uint8_t *)data;
	in.length = len;
	in.size = len;

	/* 先只解析 SOF 头拿尺寸，不分配任何输出缓冲 */
	os_memset(&info, 0, sizeof(info));
	info.frame = &in;
	if (bk_jpeg_decode_hw_get_img_info(s_jpeg_hw, &info) != AVDK_ERR_OK) {
		LOGW("cover: bad jpeg header\r\n");
		return false;
	}

	sw = info.width;
	sh = info.height;
	if (!sw || !sh || (sw > COVER_HW_MAX_DIM) || (sh > COVER_HW_MAX_DIM)) {
		LOGW("cover: %ux%u out of range\r\n", (unsigned)sw, (unsigned)sh);
		return false;
	}

	need = sw * sh * 2;
	avail = rtos_get_psram_free_heap_size();
	if (need > (avail / 2)) {          /* 留一半给系统/其它模块 */
		LOGW("cover: %ux%u needs %uKB, psram free %uKB, skip\r\n",
		     (unsigned)sw, (unsigned)sh, (unsigned)(need / 1024),
		     (unsigned)(avail / 1024));
		return false;
	}

	os_memset(&out, 0, sizeof(out));
	out.frame = (uint8_t *)psram_malloc(need);
	if (!out.frame) {
		LOGE("cover: psram alloc %uKB failed\r\n", (unsigned)(need / 1024));
		return false;
	}
	out.size = need;

	t0 = rtos_get_time();
	if (bk_jpeg_decode_hw_decode(s_jpeg_hw, &in, &out) != AVDK_ERR_OK) {
		LOGE("cover: hw decode failed %ux%u\r\n", (unsigned)sw, (unsigned)sh);
		os_free(out.frame);
		return false;
	}

	yuyv_to_rgb565(out.frame, sw, sh, dst, w, h);
	os_free(out.frame);

	LOGI("cover: hw %ux%u -> %dx%d (%ums, psram free %uKB)\r\n",
	     (unsigned)sw, (unsigned)sh, w, h, (unsigned)(rtos_get_time() - t0),
	     (unsigned)(rtos_get_psram_free_heap_size() / 1024));
	return true;
}

/* 软件解码兜底（渐进式 JPEG / 硬件未启用时）。
 * 注意：输出是全尺寸 RGB565，PSRAM 占用与硬件路径相同。 */
static bool sw_cover_decode(const uint8_t *data, uint32_t len, uint16_t *dst,
                            int w, int h)
{
	static bool s_jpeg_inited;
	frame_buffer_t jpg;
	lv_img_dsc_t img;
	uint32_t sw, sh;
	const uint16_t *src;

	if (!s_jpeg_inited) {
		if (lv_jpeg_sw_decode_init() != BK_OK) {
			LOGE("jpeg decoder init failed\r\n");
			return false;
		}
		s_jpeg_inited = true;
	}

	os_memset(&jpg, 0, sizeof(jpg));
	jpg.frame = (uint8_t *)data;
	jpg.size = len;
	jpg.length = len;

	os_memset(&img, 0, sizeof(img));
	if (lv_jpeg_sw_decode_start(&jpg, &img, true) != BK_OK || !img.data) {
		LOGE("embedded cover decode failed\r\n");
		return false;
	}

	sw = img.header.w;
	sh = img.header.h;
	if (!sw || !sh || (sw > COVER_HW_MAX_DIM) || (sh > COVER_HW_MAX_DIM)) {
		LOGE("embedded cover too large %ux%u\r\n", (unsigned)sw, (unsigned)sh);
		os_free((void *)img.data);
		return false;
	}

	src = (const uint16_t *)img.data;
	for (int y = 0; y < h; y++) {
		uint32_t sy = (uint32_t)y * sh / (uint32_t)h;

		for (int x = 0; x < w; x++) {
			uint32_t sx = (uint32_t)x * sw / (uint32_t)w;

			dst[y * w + x] = src[sy * sw + sx];
		}
	}
	os_free((void *)img.data);

	LOGI("embedded cover %ux%u -> %dx%d (sw)\r\n",
	     (unsigned)sw, (unsigned)sh, w, h);
	return true;
}

/* 内嵌封面（ID3 APIC / FLAC PICTURE）：硬件解码优先，失败回退软解 */
static bool embedded_cover_load(const media_info_t *info, uint16_t *dst, int w,
                                int h)
{
	const uint8_t *data = NULL;
	uint32_t len = 0;
	bool is_png = false;

	if (!media_get_embedded_cover(info, &data, &len, &is_png))
		return false;

	if (is_png) {
		/* PNG 走流式解码：边解压边降采样到目标尺寸，
		 * 内存与图像尺寸基本无关（32KB 窗口 + 2 条源行），
		 * 3000x3000 也能解。JPEG 无法这样做（所有官方解码器
		 * 都要求整帧输出缓冲），所以超大 JPEG 仍然走 skip。 */
		bool pok = png_cover_decode(data, len, dst, w, h);

		if (pok)
			return true;
		LOGW("embedded cover is PNG, decode failed/unsupported\r\n");
		return false;
	}

	if (hw_cover_decode(data, len, dst, w, h))
		return true;

	return sw_cover_decode(data, len, dst, w, h);
}

static void cover_load(const char *audio_path)
{
	bool found = false;
	uint32_t t0, t1, t2;
	uint32_t apic = 0;
	bool is_png = false;

	/* 【入口日志】上一版这里没有日志，用户反馈"UART0 看不到 jpeg 信息"，
	 * 无法区分是"没走到这里"还是"走到但提前返回"。先无条件打一行。 */
	LOGI("cover: enter %s (buf=%p psram=%uKB)\r\n", audio_path,
	     s_cover, (unsigned)(rtos_get_psram_free_heap_size() / 1024));

	if (!s_cover) {
		LOGE("cover: s_cover is NULL (psram alloc failed at init)\r\n");
		return;
	}

	t0 = rtos_get_time();

	/* 优先级：同名 .bmp / cover.bmp / folder.bmp > 内嵌 APIC/PICTURE */
	if (media_load_bmp_cover(audio_path, s_cover, APP_SD_COVER_SIZE,
	                         APP_SD_COVER_SIZE)) {
		found = true;
	}
	t1 = rtos_get_time();

	if (!found) {
		const uint8_t *cp = NULL;

		/* 只为了拿到大小做日志，数据指针仍然走 embedded_cover_load */
		(void)media_get_embedded_cover(&s_info, &cp, &apic, &is_png);
		if (embedded_cover_load(&s_info, s_cover, APP_SD_COVER_SIZE,
		                        APP_SD_COVER_SIZE))
			found = true;
	}
	t2 = rtos_get_time();

	/* 分段计时：封面"有的快有的慢"的原因全在这三个数上。
	 *   bmp_ms 大      -> SD 卡上同目录有 BMP 封面，解码慢
	 *   embedded_ms 大 -> 读 APIC（大小见 apic_kb）+ 解码 + 缩放
	 *   apic_kb 大     -> ID3v2 里塞了超大图，是主要开销来源
	 * 没有这一行就只能靠猜，之前一直没法定量。 */
	LOGI("cover: bmp=%ums embedded=%ums total=%ums apic=%uKB png=%d found=%d\r\n",
	     (unsigned)(t1 - t0), (unsigned)(t2 - t1), (unsigned)(t2 - t0),
	     (unsigned)(apic / 1024), (int)is_png, (int)found);

	if (s_cover_lock)
		rtos_lock_mutex(&s_cover_lock);
	s_cover_valid = found;
	s_cover_gen++;
	if (s_cover_lock)
		rtos_unlock_mutex(&s_cover_lock);
}

bool app_sd_copy_cover(uint16_t *dst, int w, int h)
{
	bool ok = false;

	if (!dst || !s_cover)
		return false;

	if (s_cover_lock)
		rtos_lock_mutex(&s_cover_lock);

	if (s_cover_valid) {
		int copy_w = (w < APP_SD_COVER_SIZE) ? w : APP_SD_COVER_SIZE;
		int copy_h = (h < APP_SD_COVER_SIZE) ? h : APP_SD_COVER_SIZE;

		for (int y = 0; y < copy_h; y++)
			memcpy(dst + y * w, s_cover + y * APP_SD_COVER_SIZE,
			       (size_t)copy_w * 2);
		ok = true;
	}

	if (s_cover_lock)
		rtos_unlock_mutex(&s_cover_lock);

	return ok;
}

uint32_t app_sd_cover_gen(void)
{
	return s_cover_gen;
}

/* ============================ 播放进度 ============================ */

static void progress_update(uint32_t pos_ms)
{
	uint32_t now = rtos_get_time();
	static uint32_t last = 0;

	s_pos_ms = pos_ms;

	if ((now - last) < SD_PROGRESS_MS)
		return;
	last = now;
	publish_track();
}

/* ============================ 组件事件回调 ============================ */
/* 以下回调都在 bk_audio_player 内部任务上下文触发：只允许更新本文件的状态变量
 * 或向 app_mode 队列投递命令，禁止同步调用 sd_player_* 控制接口（会死锁）。 */

static void publish_stopped(void)
{
	app_state_t *st;

	app_state_lock();
	st = app_state_raw();
	if (st->mode == APP_MODE_SD)
		st->play = APP_PLAY_STOPPED;
	app_state_unlock();
}

static void sdpl_on_start(const char *url)
{
	(void)url;
	s_pos_ms = 0;

	/* media 元数据给不出时长时（如部分 VBR MP3）用解码器回报的时长兜底 */
	if (s_dur_ms == 0)
		s_dur_ms = sd_player_total_ms();

	publish_track();
}

static void sdpl_on_finished(bool failure)
{
	if (failure) {
		app_state_toast("播放失败");
		publish_stopped();
		return;
	}

	/* 与旧播放循环一致：只有一首就停，多首自动下一曲（post 到 app 任务，非阻塞） */
	if (s_pl_count > 1)
		app_mode_next();
	else
		publish_stopped();
}

static void sdpl_on_seek_done(bool ok, int second)
{
	if (!ok)
		return;

	s_pos_ms = (uint32_t)second * 1000u;
	publish_track();
}

static void sdpl_on_paused(bool paused)
{
	(void)paused;
	publish_track();
}

static const sd_player_cb_t s_sdpl_cb = {
	.on_start    = sdpl_on_start,
	.on_finished = sdpl_on_finished,
	.on_seek_done = sdpl_on_seek_done,
	.on_paused   = sdpl_on_paused,
};

/* ============================ 播放列表 ============================ */

static void build_playlist(const char *file_path)
{
	char dir[APP_PATH_LEN];
	/* 这个数组 ~5KB，不能放 app 任务（4KB 栈）里 */
	static app_sd_entry_t entries[APP_SD_MAX_ENTRIES];
	int n;

	s_pl_count = 0;
	s_pl_index = 0;

	path_dir(file_path, dir, sizeof(dir));
	n = app_sd_list_dir(dir, entries, APP_SD_MAX_ENTRIES);
	if (n < 0)
		return;

	for (int i = 0; i < n && s_pl_count < APP_SD_MAX_PLAYLIST; i++) {
		if (entries[i].is_dir || !is_audio(entries[i].name))
			continue;

		path_join(dir, entries[i].name, s_playlist[s_pl_count],
		          sizeof(s_playlist[0]));
		if (!strcmp(s_playlist[s_pl_count], file_path))
			s_pl_index = s_pl_count;
		s_pl_count++;
	}

	/* 文件不在目录列表里（大小写/路径差异）时至少保留它自己 */
	if (s_pl_count == 0) {
		os_strncpy(s_playlist[0], file_path, sizeof(s_playlist[0]) - 1);
		s_pl_count = 1;
		s_pl_index = 0;
	}
}

static bool stop_current(void)
{
	/* 组件 stop 是同步命令：内部任务完成 teardown（含关音频）后才返回 */
	return sd_player_stop() == BK_OK;
}

static void start_current(void)
{
	if ((s_pl_count <= 0) || (s_pl_index < 0) || (s_pl_index >= s_pl_count))
		return;

	os_strncpy(s_cur_path, s_playlist[s_pl_index], sizeof(s_cur_path) - 1);
	s_pos_ms = 0;
	s_dur_ms = 0;

	/* 播放格式胶囊：按扩展名（解码器只认这三种） */
	app_state_lock();
	if (ext_is(s_cur_path, "flac"))
		os_strncpy(app_state_raw()->codec, "FLAC", sizeof(app_state_raw()->codec) - 1);
	else if (ext_is(s_cur_path, "wav"))
		os_strncpy(app_state_raw()->codec, "WAV", sizeof(app_state_raw()->codec) - 1);
	else if (ext_is(s_cur_path, "mp3"))
		os_strncpy(app_state_raw()->codec, "MP3", sizeof(app_state_raw()->codec) - 1);
	else
		app_state_raw()->codec[0] = '\0';
	app_state_unlock();

	media_info_free(&s_info);
	media_info_reset(&s_info);
	media_parse_tags(s_cur_path, &s_info);
	media_parse_lrc(s_cur_path, &s_info);
	s_dur_ms = s_info.duration_ms;

	/* 【先出声，后贴封面】
	 *
	 * 原来是 cover_load() 在 sd_player_play_file() 之前调。封面的代价很
	 * 不均匀：ID3 里的 APIC 从几十 KB 到 1MB 都有（实测素材里就有
	 * 130KB / 438KB / 1055KB 三档），读 APIC + 分配全尺寸 YUV 缓冲 +
	 * 硬解 + 块平均缩放加起来是几百毫秒量级 —— 全都卡在"按键确认"到
	 * "出声"之间，用户感知就是"有的歌点下去半天没反应"。
	 *
	 * 现在把顺序倒过来：播放器先起来（出声优先），再解析封面。
	 * 封面晚几百毫秒出现，但对手感的关键路径没有影响。 */
	publish_track();

	LOGI("play %s: %s / %s (heap=%u psram=%u)\r\n", s_cur_path,
	     s_info.title, s_info.artist,
	     (unsigned)rtos_get_free_heap_size(),
	     (unsigned)rtos_get_psram_free_heap_size());

	if (sd_player_play_file(s_cur_path) != BK_OK) {
		LOGE("start failed: %s\r\n", s_cur_path);
		app_state_toast("播放失败");
		publish_stopped();
		return;
	}

	cover_load(s_cur_path);

	/* 【必须再 publish 一次，否则整页都不刷新】
	 *
	 * UI 的重绘判断靠 app_state 里的 `sd_cover_gen`（refresh_state() ->
	 * state_dirty()），而 cover_load() 只在**自己内部**把 s_cover_gen
	 * 加一，不会同步到 app_state —— 同步动作在 publish_track() 里。
	 *
	 * 上一版把 cover_load() 挪到 play 之后却没补这一次 publish，结果：
	 *   publish_track（播放前）-> s_cover_gen 还是上一首的旧值
	 *   sdpl_on_start 的回调 -> 同样还是旧值
	 *   cover_load -> 更新了 s_cover_gen，但没人再通知 UI
	 * state_dirty 认为"什么都没变"，于是**标题、歌词、封面全都停在上一首**。
	 * 这就是"改动后不显示歌名和歌词"的全部原因。 */
	publish_track();
}

/* ============================ 对外接口 ============================ */

void app_sd_init(void)
{
	media_info_reset(&s_info);
	s_mounted = false;

	s_cover = (uint16_t *)psram_malloc(APP_SD_COVER_SIZE * APP_SD_COVER_SIZE * 2);
	if (s_cover)
		os_memset(s_cover, 0, APP_SD_COVER_SIZE * APP_SD_COVER_SIZE * 2);

	rtos_init_mutex(&s_cover_lock);

	/* bk_audio_player 实例与插件（失败只影响 SD 播放，不影响其它模式） */
	if (sd_player_init(&s_sdpl_cb) != BK_OK)
		LOGE("audio player init failed\r\n");

	if (bsp_sd_mount() == BK_OK)
		s_mounted = true;
	else
		LOGE("sd mount failed\r\n");

	publish_path_status();
}

void app_sd_set_active(bool active)
{
	if (active) {
		if (!s_mounted && (bsp_sd_mount() == BK_OK))
			s_mounted = true;
		publish_path_status();
	}
}

int app_sd_list_dir(const char *path, app_sd_entry_t *out, int max)
{
	DIR dir;
	FILINFO fi;
	FRESULT fr;
	int n = 0;

	if (!s_mounted || !out || max <= 0)
		return -1;

	fr = f_opendir(&dir, path);
	if (fr != FR_OK) {
		LOGE("opendir %s failed: %d\r\n", path, fr);
		return -1;
	}

	while (n < max) {
		fr = f_readdir(&dir, &fi);
		if ((fr != FR_OK) || (fi.fname[0] == 0))
			break;
		if (!strcmp(fi.fname, ".") || !strcmp(fi.fname, ".."))
			continue;

		if (fi.fattrib & AM_DIR) {
			out[n].is_dir = true;
		} else {
			if (!is_audio(fi.fname))
				continue;
			out[n].is_dir = false;
		}

		os_strncpy(out[n].name, fi.fname, sizeof(out[n].name) - 1);
		out[n].name[sizeof(out[n].name) - 1] = '\0';
		out[n].size = (uint32_t)fi.fsize;
		n++;
	}
	f_closedir(&dir);

	/* 目录优先，其次忽略大小写按名字排序 */
	for (int i = 1; i < n; i++) {
		app_sd_entry_t key = out[i];
		char ka[APP_NAME_LEN], kb[APP_NAME_LEN];
		int j = i - 1;

		os_strncpy(ka, key.name, sizeof(ka) - 1);
		ka[sizeof(ka) - 1] = '\0';
		str_tolower(ka);

		while (j >= 0) {
			os_strncpy(kb, out[j].name, sizeof(kb) - 1);
			kb[sizeof(kb) - 1] = '\0';
			str_tolower(kb);

			bool key_dir = key.is_dir;
			bool cur_dir = out[j].is_dir;

			if ((key_dir && !cur_dir) ||
			    (key_dir == cur_dir && strcmp(ka, kb) < 0)) {
				out[j + 1] = out[j];
				j--;
			} else {
				break;
			}
		}
		out[j + 1] = key;
	}

	return n;
}

void app_sd_play(const char *path)
{
	uint32_t t0 = rtos_get_time();
	uint32_t t1, t2, t3;

	if (!path || !path[0]) {
		/* 文件页选中项为空时走到这里（路径拼接失败/目录项异常）。
		 * 以前这里静默 return，界面毫无反应，看起来就是"点了没反应"。 */
		LOGE("play: empty path\r\n");
		app_state_toast("路径无效");
		return;
	}

	if (!s_mounted && (bsp_sd_mount() != BK_OK)) {
		/* 三个失败分支各自留下日志：只看 toast 分不清是没挂载、
		 * 停上一首超时、还是解码器打不开。 */
		LOGE("play: sd not mounted (%s, present=%d)\r\n", path,
		     (int)bsp_sd_is_inserted());
		app_state_toast("SD 卡未挂载");
		return;
	}

	if (!stop_current()) {
		LOGE("play: stop previous failed, state=%d\r\n", (int)sd_player_state());
		app_state_toast("停止上一首超时");
		return;
	}
	t1 = rtos_get_time();

	LOGI("play: open %s\r\n", path);
	build_playlist(path);
	t2 = rtos_get_time();
	start_current();
	t3 = rtos_get_time();

	/* 【分段耗时】"频繁切歌会卡住"需要知道卡在哪一段，这三段语义不同：
	 *   stop_ms    等 audio_player 组件 teardown（含关音频、释放 pipeline）
	 *   pl_ms      遍历目录建播放列表（FATFS 目录扫描）
	 *   start_ms   解析 ID3/LRC + 起播放器 + 解码封面（最重的一段）
	 * 若 stop_ms 经常接近超时上限，说明组件的 stop 没及时响应；
	 * 若 start_ms 在切几次后越来越大，说明 pipeline 或 PSRAM 在泄漏。 */
	LOGI("play: timing stop=%ums pl=%ums start=%ums total=%ums\r\n",
	     (unsigned)(t1 - t0), (unsigned)(t2 - t1), (unsigned)(t3 - t2),
	     (unsigned)(t3 - t0));
}

void app_sd_play_pause(void)
{
	sd_player_state_t st = sd_player_state();

	if (st == SD_PLAYER_STOPPED) {
		if (s_cur_path[0])
			app_sd_play(s_cur_path);
		else
			app_state_toast("请选择歌曲");
		return;
	}

	/* pause/resume 同步执行，事件回调里已刷新状态 */
	if (st == SD_PLAYER_PLAYING)
		sd_player_pause();
	else
		sd_player_resume();
	publish_track();
}

bool app_sd_stop(void)
{
	bool ok = stop_current();

	s_pos_ms = 0;
	publish_track();
	return ok;
}

void app_sd_next(void)
{
	if (s_pl_count <= 0) {
		if (s_cur_path[0])
			app_sd_play(s_cur_path);
		return;
	}

	if (!stop_current()) {
		app_state_toast("停止上一首超时");
		return;
	}
	s_pl_index = (s_pl_index + 1) % s_pl_count;
	start_current();
}

void app_sd_prev(void)
{
	if (s_pl_count <= 0) {
		if (s_cur_path[0])
			app_sd_play(s_cur_path);
		return;
	}

	if (!stop_current()) {
		app_state_toast("停止上一首超时");
		return;
	}
	s_pl_index = (s_pl_index + s_pl_count - 1) % s_pl_count;
	start_current();
}

void app_sd_seek(int delta_ms)
{
	sd_player_state_t st = sd_player_state();
	uint32_t dur;
	int64_t target;

	if (st == SD_PLAYER_STOPPED) {
		app_state_toast("未在播放");
		return;
	}

	dur = s_dur_ms ? s_dur_ms : sd_player_total_ms();
	target = (int64_t)sd_player_pos_ms() + delta_ms;

	if (target < 0)
		target = 0;
	if (dur && (target > (int64_t)dur - SD_SEEK_MARGIN_MS))
		target = (int64_t)dur - SD_SEEK_MARGIN_MS;

	/* 组件按绝对秒定位；暂停状态下请求会被组件挂起，恢复播放后生效 */
	if (sd_player_seek_abs((int)(target / 1000)) != BK_OK)
		LOGW("seek busy, target=%ds\r\n", (int)(target / 1000));
}

void app_sd_poll(void)
{
	bool present = bsp_sd_is_inserted();
	static bool last_present = true;

	if (present != last_present) {
		last_present = present;
		publish_path_status();
	}

	/* 播放中刷新进度（progress_update 内部 200ms 节流），歌词随之前滚 */
	if (sd_player_state() == SD_PLAYER_PLAYING) {
		s_pos_ms = sd_player_pos_ms();
		if (s_dur_ms && (s_pos_ms > s_dur_ms))
			s_pos_ms = s_dur_ms;
		progress_update(s_pos_ms);
	}

	if (!present && s_mounted) {
		/* 卡被拔掉：必须先停播（同步，内部任务已退出文件访问）再卸载，
		 * 否则 f_read 还在访问卷会导致 FatFS 崩溃。 */
		if (app_sd_stop()) {
			bsp_sd_unmount();
			s_mounted = false;
		}
		publish_path_status();
	}
}

bool app_sd_is_playing(void)
{
	return sd_player_state() != SD_PLAYER_STOPPED;
}
