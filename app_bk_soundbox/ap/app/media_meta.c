// Copyright 2026
// SD 音频元数据实现

#include "media_meta.h"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include <components/log.h>

#include "ff.h"
#include "conv_utf8_pub.h"

#define TAG "media"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)

/* ============================ 小工具 ============================ */

static const char *base_name(const char *path)
{
	const char *p = path;

	if (!p)
		return "";

	for (const char *q = path; *q; q++) {
		if ((*q == '/') || (*q == ':'))
			p = q + 1;
	}
	return p;
}

static void strip_ext(char *name)
{
	char *dot = strrchr(name, '.');

	if (dot && (dot != name))
		*dot = '\0';
}

static const char *get_ext(const char *path)
{
	const char *dot = strrchr(path, '.');

	return dot ? dot + 1 : "";
}

static bool ext_is(const char *path, const char *ext)
{
	const char *e = get_ext(path);
	size_t n = strlen(ext);

	if (strlen(e) != n)
		return false;

	for (size_t i = 0; i < n; i++) {
		if (tolower((unsigned char)e[i]) != tolower((unsigned char)ext[i]))
			return false;
	}
	return true;
}

static void dir_of(const char *path, char *out, size_t outlen)
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

static void make_sidecar(const char *audio_path, const char *ext, char *out, size_t outlen)
{
	size_t n = strlen(audio_path);
	size_t e = strlen(ext);

	if (outlen == 0)
		return;
	out[0] = '\0';
	if (n + 1 + e + 1 > outlen)
		return;

	memcpy(out, audio_path, n + 1);
	char *dot = strrchr(out, '.');

	if (dot && (dot > out))
		*dot = '\0';
	strncat(out, ".", outlen - strlen(out) - 1);
	strncat(out, ext, outlen - strlen(out) - 1);
}

static void trim_right(char *s)
{
	size_t n = strlen(s);

	while (n > 0 && ((s[n - 1] == ' ') || (s[n - 1] == '\r') ||
	                 (s[n - 1] == '\n') || (s[n - 1] == '\t')))
		s[--n] = '\0';
}

void media_info_reset(media_info_t *info)
{
	if (!info)
		return;

	os_memset(info, 0, sizeof(*info));
	info->lines = NULL;
	info->line_count = 0;
	info->offset_ms = 0;
}

void media_info_free(media_info_t *info)
{
	if (!info)
		return;

	if (info->cover_data) {
		os_free(info->cover_data);
		info->cover_data = NULL;
	}
	info->cover_len = 0;
	info->cover_is_png = false;

	if (info->lines) {
		if (info->lines_in_psram)
			psram_free(info->lines);
		else
			os_free(info->lines);
		info->lines = NULL;
	}
	info->line_count = 0;
	info->lines_in_psram = false;
	info->lines_from_tag = false;
}

/* ============================ 编码转换 ============================ */

static int utf8_char_len(uint8_t c)
{
	if (c < 0x80) return 1;
	if ((c & 0xE0) == 0xC0) return 2;
	if ((c & 0xF0) == 0xE0) return 3;
	if ((c & 0xF8) == 0xF0) return 4;
	return 0;
}

static bool utf8_valid(const uint8_t *s, int len)
{
	int i = 0;

	while (i < len && s[i]) {
		int n = utf8_char_len(s[i]);

		if (n == 0 || (i + n) > len)
			return false;
		for (int k = 1; k < n; k++) {
			if ((s[i + k] & 0xC0) != 0x80)
				return false;
		}
		i += n;
	}
	return true;
}

static int utf8_put(uint32_t cp, char *out, int pos, int outlen)
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

static void utf16_to_utf8(const uint8_t *in, int len, bool big_endian,
                          char *out, int outlen)
{
	int i = 0, pos = 0;

	while (i + 1 < len) {
		uint32_t u = big_endian ? ((in[i] << 8) | in[i + 1])
		                        : (in[i] | (in[i + 1] << 8));

		i += 2;
		if (u == 0)
			break;

		if ((u >= 0xD800) && (u <= 0xDBFF) && (i + 1 < len)) {
			uint32_t lo = big_endian ? ((in[i] << 8) | in[i + 1])
			                         : (in[i] | (in[i + 1] << 8));
			i += 2;
			if ((lo >= 0xDC00) && (lo <= 0xDFFF))
				u = 0x10000 + ((u - 0xD800) << 10) + (lo - 0xDC00);
		}
		pos = utf8_put(u, out, pos, outlen);
	}
	out[pos] = '\0';
}

/* enc + 正文 -> UTF-8。正文长度按字节给，内部按 UTF-16 时成对处理。 */
static void id3_body_to_utf8(uint8_t enc, const uint8_t *data, int len,
                             char *out, int outlen)
{
	int i = 0;

	if (!data || len <= 0 || outlen <= 0) {
		if (outlen > 0) out[0] = '\0';
		return;
	}

	out[0] = '\0';

	switch (enc) {
	case 0: {   /* ISO-8859-1 */
		int pos = 0;

		for (; i < len && data[i]; i++) {
			uint32_t c = data[i];

			if (c < 0x80) {
				if (pos + 1 < outlen) out[pos++] = (char)c;
			} else {
				pos = utf8_put(c, out, pos, outlen);
			}
		}
		out[pos] = '\0';
		break;
	}
	case 1: {   /* UTF-16 + BOM */
		bool be = false;

		if ((i + 1 < len) && (data[i] == 0xFE) && (data[i + 1] == 0xFF)) {
			be = true;
			i += 2;
		} else if ((i + 1 < len) && (data[i] == 0xFF) && (data[i + 1] == 0xFE)) {
			be = false;
			i += 2;
		}
		utf16_to_utf8(data + i, len - i, be, out, outlen);
		break;
	}
	case 2:     /* UTF-16BE */
		utf16_to_utf8(data + i, len - i, true, out, outlen);
		break;
	default:    /* UTF-8 / 未知按 UTF-8 */
	{
		int n = len - i;

		if (n >= outlen)
			n = outlen - 1;
		memcpy(out, data + i, n);
		out[n] = '\0';
		break;
	}
	}

	trim_right(out);
}

static void id3_text_to_utf8(const uint8_t *data, int len, char *out, int outlen)
{
	if (!data || len <= 1 || outlen <= 0) {
		if (outlen > 0) out[0] = '\0';
		return;
	}

	id3_body_to_utf8(data[0], data + 1, len - 1, out, outlen);
}

static void text_to_utf8(char *s, int len, char *out, int outlen)
{
	if (utf8_valid((const uint8_t *)s, len)) {
		int n = len;

		if (n >= outlen)
			n = outlen - 1;
		memcpy(out, s, n);
		out[n] = '\0';
		return;
	}

	unsigned char *conv = conv_utf8((unsigned char *)s);

	if (conv) {
		os_strncpy(out, (char *)conv, outlen - 1);
		out[outlen - 1] = '\0';
		os_free(conv);
	} else {
		int n = len;

		if (n >= outlen)
			n = outlen - 1;
		memcpy(out, s, n);
		out[n] = '\0';
	}
}

/* ============================ ID3 ============================ */

static uint32_t be32(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | p[3];
}

static uint32_t syncsafe32(const uint8_t *p)
{
	return ((uint32_t)(p[0] & 0x7F) << 21) |
	       ((uint32_t)(p[1] & 0x7F) << 14) |
	       ((uint32_t)(p[2] & 0x7F) << 7) |
	       (uint32_t)(p[3] & 0x7F);
}

/* ============ 内嵌歌词 / 内嵌封面 ============
 *
 * 歌词来源优先级：同名 .lrc > ID3v2 SYLT（同步）> ID3v2 USLT（非同步）
 *                 > FLAC VorbisComment 的 LYRICS/UNSYNCEDLYRICS
 * 封面来源优先级：同名 .bmp / cover.bmp / folder.bmp > ID3v2 APIC > FLAC PICTURE
 *
 * 注意：很多 MP3 的 USLT 装的其实是**完整 LRC 文本**（含 [mm:ss.xx]），
 * 甚至用 U+2009 细空格在同一行里塞「原文 + 译文」。这类内容必须先走
 * LRC 解析，不能当纯文本；只有确实没有时间戳时才退回“整首均摊”的伪时间轴。
 */

static void lrc_add(media_info_t *info, uint32_t time_ms, const char *text);
static bool lrc_from_text(media_info_t *info, const char *text, uint32_t len);

static bool lines_ensure(media_info_t *info)
{
	if (info->lines)
		return true;

	info->lines = (media_lrc_line_t *)psram_malloc(
		sizeof(media_lrc_line_t) * MEDIA_MAX_LRC_LINES);
	if (info->lines) {
		info->lines_in_psram = true;
	} else {
		info->lines = (media_lrc_line_t *)os_malloc(
			sizeof(media_lrc_line_t) * MEDIA_MAX_LRC_LINES);
		info->lines_in_psram = false;
	}
	return info->lines != NULL;
}

/* 内嵌歌词文本 -> 歌词行。
 * 含 LRC 时间戳时按真实时间轴解析；纯文本才按整首均摊造伪时间轴。 */
static void lyrics_from_text(media_info_t *info, const char *text, uint32_t len,
                             uint32_t dur_ms)
{
	int first, n;
	uint32_t step;
	char line[APP_LYRIC_LEN];

	if (!text || !len)
		return;

	/* USLT / FLAC LYRICS 里经常直接放整份 LRC，优先走时间戳解析 */
	if (lrc_from_text(info, text, len))
		return;

	if (!lines_ensure(info))
		return;

	first = info->line_count;

	while (len > 0 && info->line_count < MEDIA_MAX_LRC_LINES) {
		uint32_t i = 0;
		int pos = 0;

		while ((i < len) && (text[i] != '\n') && (text[i] != '\r')) {
			if (pos + 1 < (int)sizeof(line))
				line[pos++] = text[i];
			i++;
		}
		line[pos] = '\0';
		trim_right(line);
		if (line[0])
			lrc_add(info, 0, line);

		/* 跳过行尾的 \r\n / \n */
		while ((i < len) && ((text[i] == '\n') || (text[i] == '\r')))
			i++;
		text += i;
		len -= i;
	}

	n = info->line_count - first;
	if (n <= 0)
		return;

	/* 均摊：知道时长就用时长，否则按 3 秒/行估 */
	step = dur_ms ? (dur_ms / (uint32_t)n) : 3000u;
	for (int k = 0; k < n; k++) {
		uint32_t t = step * (uint32_t)k;

		if (info->lines[first + k].time_ms == 0)
			info->lines[first + k].time_ms = t;
	}

	info->lines_from_tag = true;
	LOGI("lyrics from tag: %d lines (step %ums)\r\n", n, (unsigned)step);
}

static void cover_store(media_info_t *info, const uint8_t *data, uint32_t len,
                        bool is_png)
{
	uint8_t *p;
	uint32_t avail;

	if (!data || !len || (len > MEDIA_MAX_COVER_BYTES))
		return;

	/* 只有 JPEG 能解码；已有 JPEG 时不再被 PNG 覆盖 */
	if (info->cover_data && !info->cover_is_png)
		return;

	/* 原图要常驻 PSRAM，而解码时还要再要一块 w*h*2 的输出缓冲。
	 * 这里先保证原图不超过可用量的一半，剩下的留给解码，
	 * 否则会出现"封面存下了但解不出来"的哑巴情况。 */
	avail = rtos_get_psram_free_heap_size();
	if (len > (avail / 2)) {
		LOGW("cover %u bytes > psram free/2 (%u), skip\r\n",
		     (unsigned)len, (unsigned)(avail / 2));
		return;
	}

	if (info->cover_data) {
		os_free(info->cover_data);
		info->cover_data = NULL;
		info->cover_len = 0;
	}

	p = psram_malloc(len);
	if (!p)
		p = os_malloc(len);
	if (!p)
		return;

	memcpy(p, data, len);
	info->cover_data = p;
	info->cover_len = len;
	info->cover_is_png = is_png;

	LOGI("embedded cover: %u bytes (%s)\r\n", (unsigned)len,
	     is_png ? "png" : "jpeg");
}

/* 跳过 ID3 的“按编码结尾”的字符串（描述字段），返回下一个字节位置 */
static uint32_t id3_skip_string(const uint8_t *buf, uint32_t p, uint32_t len,
                                uint8_t enc)
{
	if (enc == 1 || enc == 2) {
		while ((p + 1) < len && !((buf[p] == 0) && (buf[p + 1] == 0)))
			p += 2;
		return (p + 1 < len) ? (p + 2) : len;
	}

	while ((p < len) && buf[p])
		p++;
	return (p < len) ? (p + 1) : len;
}

/* APIC: enc(1) mime(0 结尾) type(1) desc(0 结尾) data */
static void id3_apic_parse(media_info_t *info, const uint8_t *buf, uint32_t len)
{
	uint8_t enc;
	uint32_t p = 0;
	uint32_t mime_start, mime_len;
	bool is_png = false;

	if (len < 8)
		return;

	enc = buf[0];
	mime_start = 1;
	mime_len = 0;
	while ((mime_start + mime_len) < len && buf[mime_start + mime_len])
		mime_len++;
	if ((mime_len + mime_start) >= len)
		return;

	for (uint32_t i = 0; i + 2 < mime_len; i++) {
		const char *m = (const char *)buf + mime_start + i;

		if ((m[0] == 'p' || m[0] == 'P') && (m[1] == 'n' || m[1] == 'N') &&
		    (m[2] == 'g' || m[2] == 'G')) {
			is_png = true;
			break;
		}
	}

	p = mime_start + mime_len + 1;      /* 跳过 mime 的 0 */
	if (p >= len)
		return;
	p++;                                /* picture type */
	p = id3_skip_string(buf, p, len, enc);
	if (p >= len)
		return;

	cover_store(info, buf + p, len - p, is_png);
}

/* USLT: enc(1) lang(3) desc(0 结尾) lyrics
 *
 * 正文可能是一整份 LRC（本工程实测样例 2952 字节 UTF-16 -> 3338 字节 UTF-8），
 * 固定 1KB 的栈缓冲会把后半首歌词整段截掉，所以按输入长度在堆上开。 */
static void id3_uslt_parse(media_info_t *info, const uint8_t *buf, uint32_t len,
                           uint32_t dur_ms)
{
	uint8_t enc;
	uint32_t p, cap;
	char *text;

	if (len < 6)
		return;

	enc = buf[0];
	p = id3_skip_string(buf, 4, len, enc);
	if (p >= len)
		return;

	/* UTF-16 每 2 字节最多变 3 字节 UTF-8，Latin1 每字节最多变 2 字节 */
	cap = (len - p) * 2 + 8;
	text = psram_malloc(cap);
	if (!text)
		text = os_malloc(cap);
	if (!text)
		return;

	id3_body_to_utf8(enc, buf + p, (int)(len - p), text, (int)cap);
	if (text[0])
		lyrics_from_text(info, text, (uint32_t)strlen(text), dur_ms);
	os_free(text);
}

/* SYLT: enc(1) lang(3) ts_format(1) content_type(1) desc(0 结尾)
 *       [ text(0 结尾) time(4B BE) ]... */
static void id3_sylt_parse(media_info_t *info, const uint8_t *buf, uint32_t len)
{
	uint8_t enc;
	uint8_t ts_format;
	uint32_t p;

	if (len < 8)
		return;

	enc = buf[0];
	ts_format = buf[4];
	if (ts_format != 2) {
		/* 1 = MPEG 帧数，需要知道帧长才能换算，直接用会错位 */
		LOGW("SYLT timestamp format %u unsupported\r\n", ts_format);
		return;
	}

	p = id3_skip_string(buf, 6, len, enc);
	if (p >= len)
		return;

	while ((p < len) && (info->line_count < MEDIA_MAX_LRC_LINES)) {
		uint32_t tstart = p;
		uint32_t tend;
		char text[APP_LYRIC_LEN];

		if (enc == 1 || enc == 2) {
			tend = p;
			while ((tend + 1) < len && !((buf[tend] == 0) && (buf[tend + 1] == 0)))
				tend += 2;
			if ((tend + 1) >= len)
				break;
			p = tend + 2;
		} else {
			tend = p;
			while ((tend < len) && buf[tend])
				tend++;
			if (tend >= len)
				break;
			p = tend + 1;
		}

		if ((p + 4) > len)
			break;

		id3_body_to_utf8(enc, buf + tstart, (int)(tend - tstart), text, sizeof(text));
		if (text[0])
			lrc_add(info, be32(buf + p), text);
		p += 4;
	}

	if (info->line_count > 0) {
		info->lines_from_tag = true;
		LOGI("lyrics from SYLT: %d lines\r\n", info->line_count);
	}
}

static void parse_id3v2(FIL *f, media_info_t *info)
{
	uint8_t hdr[10];
	UINT br = 0;
	uint32_t tag_size, end;
	int version;

	f_lseek(f, 0);
	if ((f_read(f, hdr, 10, &br) != FR_OK) || (br != 10) || memcmp(hdr, "ID3", 3))
		return;

	version = hdr[3];
	tag_size = syncsafe32(hdr + 6);
	end = 10 + tag_size;

	while (f_tell(f) + 10 < end) {
		uint8_t fh[10];
		uint32_t fsize, next;
		char id[5] = {0};

		if ((f_read(f, fh, 10, &br) != FR_OK) || (br != 10))
			break;
		if (fh[0] == 0)
			break;

		memcpy(id, fh, 4);
		if (version >= 4)
			fsize = syncsafe32(fh + 4);
		else
			fsize = be32(fh + 4);

		if (fsize == 0)
			break;

		{
			FSIZE_t pos = f_tell(f);

			/* 不信任帧长度：越界一律截到 tag 末尾，避免 seek 到文件尾外 */
			if (fsize > (FSIZE_t)(end - pos))
				next = end;
			else
				next = pos + fsize;
		}
		if (next <= f_tell(f)) {
			f_lseek(f, next);
			continue;
		}

		/* 内嵌封面：APIC 可能几百 KB，单独放行 */
		if (!memcmp(id, "APIC", 4) && (fsize <= MEDIA_MAX_COVER_BYTES)) {
			uint8_t *buf = psram_malloc(fsize);

			if (!buf)
				buf = os_malloc(fsize);
			if (buf) {
				if ((f_read(f, buf, fsize, &br) == FR_OK) && (br == fsize))
					id3_apic_parse(info, buf, fsize);
				os_free(buf);
			}
			f_lseek(f, next);
			continue;
		}

		/* 内嵌歌词 */
		if ((!memcmp(id, "USLT", 4) || !memcmp(id, "SYLT", 4)) &&
		    (fsize <= 16384)) {
			uint8_t *buf = psram_malloc(fsize + 1);

			if (!buf)
				buf = os_malloc(fsize + 1);
			if (buf) {
				if ((f_read(f, buf, fsize, &br) == FR_OK) && (br == fsize)) {
					if (!memcmp(id, "USLT", 4))
						id3_uslt_parse(info, buf, fsize,
						               info->duration_ms);
					else
						id3_sylt_parse(info, buf, fsize);
				}
				os_free(buf);
			}
			f_lseek(f, next);
			continue;
		}

		if (fsize > 16384) {
			f_lseek(f, next);
			continue;
		}

		if (!memcmp(id, "TIT2", 4) || !memcmp(id, "TPE1", 4) ||
		    !memcmp(id, "TALB", 4) || !memcmp(id, "TLEN", 4)) {
			uint8_t *buf = psram_malloc(fsize + 1);

			if (!buf)
				buf = os_malloc(fsize + 1);
			if (buf) {
				if ((f_read(f, buf, fsize, &br) == FR_OK) && (br == fsize)) {
					char txt[APP_TITLE_LEN];

					id3_text_to_utf8(buf, (int)fsize, txt, sizeof(txt));
					if (!memcmp(id, "TIT2", 4))
						os_strncpy(info->title, txt, sizeof(info->title) - 1);
					else if (!memcmp(id, "TPE1", 4))
						os_strncpy(info->artist, txt, sizeof(info->artist) - 1);
					else if (!memcmp(id, "TALB", 4))
						os_strncpy(info->album, txt, sizeof(info->album) - 1);
					else if (!memcmp(id, "TLEN", 4))
						info->duration_ms = (uint32_t)strtoul(txt, NULL, 10);
				}
				os_free(buf);
			}
			f_lseek(f, next);
		} else {
			f_lseek(f, next);
		}
	}
}

static void parse_id3v1(FIL *f, media_info_t *info)
{
	uint8_t tag[128];
	FSIZE_t size = f_size(f);
	UINT br = 0;
	char tmp[64];

	if (size < 128)
		return;

	f_lseek(f, size - 128);
	if ((f_read(f, tag, sizeof(tag), &br) != FR_OK) || br != sizeof(tag))
		return;
	if (memcmp(tag, "TAG", 3))
		return;

	if (info->title[0] == '\0') {
		memcpy(tmp, tag + 3, 30);
		tmp[30] = '\0';
		trim_right(tmp);
		if (tmp[0]) {
			char u[APP_TITLE_LEN];

			text_to_utf8(tmp, strlen(tmp), u, sizeof(u));
			os_strncpy(info->title, u, sizeof(info->title) - 1);
		}
	}
	if (info->artist[0] == '\0') {
		memcpy(tmp, tag + 33, 30);
		tmp[30] = '\0';
		trim_right(tmp);
		if (tmp[0]) {
			char u[APP_ARTIST_LEN];

			text_to_utf8(tmp, strlen(tmp), u, sizeof(u));
			os_strncpy(info->artist, u, sizeof(info->artist) - 1);
		}
	}
	if (info->album[0] == '\0') {
		memcpy(tmp, tag + 63, 30);
		tmp[30] = '\0';
		trim_right(tmp);
		if (tmp[0]) {
			char u[APP_ALBUM_LEN];

			text_to_utf8(tmp, strlen(tmp), u, sizeof(u));
			os_strncpy(info->album, u, sizeof(info->album) - 1);
		}
	}
}

static uint32_t rd_le32(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static void flac_parse_comment(const uint8_t *buf, uint32_t len, media_info_t *info)
{
	uint32_t p = 0;
	uint32_t vendor_len;
	uint32_t count;

	if (len < 8)
		return;
	vendor_len = rd_le32(buf);
	if (vendor_len > len - 8)
		return;
	p = 4 + vendor_len;
	count = rd_le32(buf + p);
	p += 4;
	if (count > 256)
		count = 256;

	for (uint32_t i = 0; i < count && (p + 4) <= len; i++) {
		uint32_t n = rd_le32(buf + p);
		const uint8_t *kv;
		const uint8_t *eq;
		uint32_t klen;
		uint32_t vlen;
		char val[APP_TITLE_LEN];

		p += 4;
		if (n > len - p)
			break;
		kv = buf + p;
		eq = memchr(kv, '=', n);
		if (eq) {
			klen = (uint32_t)(eq - kv);
			vlen = n - klen - 1;

			/* 歌词注释：整段交给 lyrics_from_text（不能塞进 96 字节的 val） */
			if (((klen == 7) && !memcmp(kv, "LYRICS", 7)) ||
			    ((klen == 14) && !memcmp(kv, "UNSYNCEDLYRICS", 14))) {
				uint32_t raw = vlen;

				if (raw > 16384)
					raw = 16384;
				lyrics_from_text(info, (const char *)(eq + 1), raw,
				                 info->duration_ms);
				p += n;
				continue;
			}

			if (vlen > sizeof(val) - 1)
				vlen = sizeof(val) - 1;
			memcpy(val, eq + 1, vlen);
			val[vlen] = '\0';

			if ((klen == 5) && !memcmp(kv, "TITLE", 5))
				os_strncpy(info->title, val, sizeof(info->title) - 1);
			else if ((klen == 6) && !memcmp(kv, "ARTIST", 6))
				os_strncpy(info->artist, val, sizeof(info->artist) - 1);
			else if ((klen == 5) && !memcmp(kv, "ALBUM", 5))
				os_strncpy(info->album, val, sizeof(info->album) - 1);
		}
		p += n;
	}
}

/* PICTURE: type(4) mime_len(4) mime desc_len(4) desc w/h/depth/colors(16)
 *          data_len(4) data —— 全部大端 */
static void flac_picture_parse(media_info_t *info, const uint8_t *buf, uint32_t len)
{
	uint32_t p = 4;
	uint32_t mime_len, desc_len, data_len;
	bool is_png = false;

	if (len < 32)
		return;

	mime_len = be32(buf + p); p += 4;
	if (mime_len > (len - p))
		return;
	for (uint32_t i = 0; i + 2 < mime_len; i++) {
		const char *m = (const char *)buf + p + i;

		if ((m[0] == 'p' || m[0] == 'P') && (m[1] == 'n' || m[1] == 'N') &&
		    (m[2] == 'g' || m[2] == 'G')) {
			is_png = true;
			break;
		}
	}
	p += mime_len;

	desc_len = be32(buf + p); p += 4;
	if (desc_len > (len - p))
		return;
	p += desc_len + 16;                 /* 描述 + w/h/depth/colors */
	if ((p + 4) > len)
		return;

	data_len = be32(buf + p); p += 4;
	if (data_len > (len - p))
		data_len = len - p;

	cover_store(info, buf + p, data_len, is_png);
}

static void parse_flac_meta(FIL *f, media_info_t *info)
{
	uint8_t hdr[4];
	UINT br = 0;
	bool last = false;

	f_lseek(f, 0);
	if ((f_read(f, hdr, 4, &br) != FR_OK) || br != 4 || memcmp(hdr, "fLaC", 4))
		return;

	while (!last) {
		uint32_t len;
		uint8_t type;

		if ((f_read(f, hdr, 4, &br) != FR_OK) || br != 4)
			break;
		last = (hdr[0] & 0x80) != 0;
		type = hdr[0] & 0x7F;
		len = ((uint32_t)hdr[1] << 16) | ((uint32_t)hdr[2] << 8) | hdr[3];

		if (type == 0 && len >= 34) {
			uint8_t si[34];

			if ((f_read(f, si, 34, &br) != FR_OK) || br != 34)
				break;
			uint32_t rate = ((uint32_t)si[10] << 12) |
			                ((uint32_t)si[11] << 4) |
			                ((uint32_t)si[12] >> 4);
			uint32_t ch = ((si[12] >> 1) & 0x07) + 1;
			uint32_t bps = (((uint32_t)(si[12] & 0x01) << 4) |
			                (si[13] >> 4)) + 1;
			uint64_t total = ((uint64_t)(si[13] & 0x0F) << 32) |
			                 ((uint64_t)si[14] << 24) |
			                 ((uint64_t)si[15] << 16) |
			                 ((uint64_t)si[16] << 8) | si[17];

			(void)ch;
			(void)bps;
			if (rate && total)
				info->duration_ms = (uint32_t)(total * 1000ULL / rate);
			if (len > 34)
				f_lseek(f, f_tell(f) + (len - 34));
		} else if (type == 4 && len > 0 && len <= 16384) {
			uint8_t *buf = psram_malloc(len);

			if (!buf)
				buf = os_malloc(len);
			if (buf) {
				if ((f_read(f, buf, len, &br) == FR_OK) && br == len)
					flac_parse_comment(buf, len, info);
				os_free(buf);
			} else {
				f_lseek(f, f_tell(f) + len);
			}
		} else if ((type == 6) && (len > 32) && (len <= MEDIA_MAX_COVER_BYTES)) {
			uint8_t *buf = psram_malloc(len);

			if (!buf)
				buf = os_malloc(len);
			if (buf) {
				if ((f_read(f, buf, len, &br) == FR_OK) && br == len)
					flac_picture_parse(info, buf, len);
				os_free(buf);
			} else {
				f_lseek(f, f_tell(f) + len);
			}
		} else {
			f_lseek(f, f_tell(f) + len);
		}
	}
}

/* ============================ MP3 ============================ */

/* 【为什么需要这一段】
 *
 * 原来 MP3 只解析 ID3v2 的 TLEN 帧来取时长。但 TLEN 是**可选帧**，
 * LAME 等主流编码器默认根本不写 —— 于是 info->duration_ms 恒为 0，
 * 界面上总时长永远显示 0:00、进度条分母为 0（除零保护后干脆不动）。
 * FLAC 不受影响，因为它从 STREAMINFO 块里拿到精确的 total_samples。
 *
 * 这里按 MP3 帧头估算：
 *   1) 跳过 ID3v2 区（头 10 字节 + 同步安全整数表示的长度）
 *   2) 逐字节找第一个合法帧头（同步字 11 个 1，且 Layer III、
 *      比特率/采样率索引都合法 —— 三重校验能避免 ID3 里的封面数据
 *      被误认成帧头）
 *   3) 按 CBR 反推：时长 = 音频字节数 × 8 ÷ 比特率（kbps）
 *
 * 精度说明：对 CBR 文件是准的；VBR 文件用首帧比特率估算会偏小，
 * 但量级正确、进度条能正常工作。做逐帧精确统计要遍历上万个帧
 * （每帧一次 f_lseek + f_read），SD 卡上要几百毫秒，开机/切歌
 * 都能感知到卡顿，不值得。有 TLEN 的文件仍然优先用 TLEN。 */
static const uint16_t s_mp3_br_v1l3[16] = {
	0, 32, 40, 48, 56, 64, 80, 96, 112, 128, 160, 192, 224, 256, 320, 0
};
static const uint16_t s_mp3_br_v2l3[16] = {
	0, 8, 16, 24, 32, 40, 48, 56, 64, 80, 96, 112, 128, 144, 160, 0
};
static const uint16_t s_mp3_sr[3] = { 44100, 48000, 32000 };

/* 返回音频数据起点；无 ID3v2 时返回 0 */
static uint32_t mp3_audio_start(FIL *f)
{
	uint8_t h[10];
	UINT br = 0;
	uint32_t sz;

	if (f_lseek(f, 0) != FR_OK)
		return 0;
	if (f_read(f, h, 10, &br) != FR_OK || br != 10)
		return 0;
	if (memcmp(h, "ID3", 3))
		return 0;

	/* ID3v2 长度是"同步安全整数"：每字节只用低 7 位 */
	sz = ((uint32_t)(h[6] & 0x7F) << 21) | ((uint32_t)(h[7] & 0x7F) << 14) |
	     ((uint32_t)(h[8] & 0x7F) << 7) | (uint32_t)(h[9] & 0x7F);

	return 10 + sz;
}

static void parse_mp3_frames(FIL *f, media_info_t *info)
{
	uint8_t h[4];
	UINT br = 0;
	uint32_t fsize = (uint32_t)f_size(f);
	uint32_t off = mp3_audio_start(f);
	uint32_t bitrate = 0, rate = 0, audio_bytes;
	bool found = false;

	/* 最多往后找 256KB：足以跳过 ID3 里的大块封面，又不至于在
	 * 损坏文件上把整首扫完 */
	for (uint32_t guard = 0; off + 4 < fsize && guard < (256u * 1024u); guard++) {
		uint8_t ver, layer, br_idx, sr_idx;

		if (f_lseek(f, off) != FR_OK)
			return;
		if (f_read(f, h, 4, &br) != FR_OK || br != 4)
			return;

		if (h[0] != 0xFF || (h[1] & 0xE0) != 0xE0) {
			off++;
			continue;
		}

		ver = (uint8_t)((h[1] >> 3) & 0x03);   /* 3=MPEG1 2=MPEG2 0=MPEG2.5 */
		layer = (uint8_t)((h[1] >> 1) & 0x03); /* 1=Layer III */
		br_idx = (uint8_t)((h[2] >> 4) & 0x0F);
		sr_idx = (uint8_t)((h[2] >> 2) & 0x03);

		if (layer != 1 || br_idx == 0 || br_idx >= 15 || sr_idx >= 3) {
			off++;
			continue;
		}

		bitrate = (ver == 3) ? s_mp3_br_v1l3[br_idx] : s_mp3_br_v2l3[br_idx];
		rate = s_mp3_sr[sr_idx];
		if (ver == 2)
			rate >>= 1;
		else if (ver == 0)
			rate >>= 2;
		found = true;
		break;
	}

	if (!found || !bitrate || !rate)
		return;

	/* TLEN 已经给了精确值，不覆盖 */
	if (info->duration_ms)
		return;

	audio_bytes = (fsize > off) ? (fsize - off) : 0;
	info->duration_ms = (uint32_t)((uint64_t)audio_bytes * 8u / bitrate);
}

/* ============================ 入口 ============================ */

void media_parse_tags(const char *path, media_info_t *info)
{
	/* FIL 含 4KB 缓冲，不能放栈上；这些解析函数只在 SD 播放线程调用 */
	static FIL f;
	char name[APP_NAME_LEN];

	media_info_reset(info);

	os_strncpy(name, base_name(path), sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';
	strip_ext(name);
	os_strncpy(info->title, name, sizeof(info->title) - 1);

	if (!ext_is(path, "mp3") && !ext_is(path, "flac"))
		return;

	if (f_open(&f, path, FA_READ) != FR_OK)
		return;

	if (ext_is(path, "flac")) {
		parse_flac_meta(&f, info);
	} else {
		parse_id3v2(&f, info);
		parse_id3v1(&f, info);
		/* ID3 里通常没有 TLEN，这里补上帧头估算的时长 */
		parse_mp3_frames(&f, info);
	}
	f_close(&f);

	if (info->title[0] == '\0') {
		os_strncpy(info->title, name, sizeof(info->title) - 1);
		info->title[sizeof(info->title) - 1] = '\0';
	}
}

/* ============================ LRC ============================ */

static bool parse_lrc_tags(char *line, media_info_t *info)
{
	if (!strncmp(line, "[ti:", 4)) {
		char *e = strchr(line + 4, ']');
		char t[APP_TITLE_LEN];

		if (e) {
			*e = '\0';
			text_to_utf8(line + 4, strlen(line + 4), t, sizeof(t));
			os_strncpy(info->title, t, sizeof(info->title) - 1);
			*e = ']';
			return true;
		}
	} else if (!strncmp(line, "[ar:", 4)) {
		char *e = strchr(line + 4, ']');
		char t[APP_ARTIST_LEN];

		if (e) {
			*e = '\0';
			text_to_utf8(line + 4, strlen(line + 4), t, sizeof(t));
			os_strncpy(info->artist, t, sizeof(info->artist) - 1);
			*e = ']';
			return true;
		}
	} else if (!strncmp(line, "[al:", 4)) {
		char *e = strchr(line + 4, ']');
		char t[APP_ALBUM_LEN];

		if (e) {
			*e = '\0';
			text_to_utf8(line + 4, strlen(line + 4), t, sizeof(t));
			os_strncpy(info->album, t, sizeof(info->album) - 1);
			*e = ']';
			return true;
		}
	} else if (!strncmp(line, "[offset:", 8)) {
		info->offset_ms = atoi(line + 8);
		return true;
	}
	return false;
}

static void lrc_add(media_info_t *info, uint32_t time_ms, const char *text)
{
	media_lrc_line_t *ln;

	if (!info->lines || info->line_count >= MEDIA_MAX_LRC_LINES)
		return;
	if (!text || !text[0])
		return;

	ln = &info->lines[info->line_count];
	ln->time_ms = time_ms;
	os_strncpy(ln->text, text, sizeof(ln->text) - 1);
	ln->text[sizeof(ln->text) - 1] = '\0';
	info->line_count++;
}

static void lrc_sort(media_info_t *info)
{
	for (int i = 1; i < info->line_count; i++) {
		media_lrc_line_t key = info->lines[i];
		int j = i - 1;

		while (j >= 0 && info->lines[j].time_ms > key.time_ms) {
			info->lines[j + 1] = info->lines[j];
			j--;
		}
		info->lines[j + 1] = key;
	}
}

/* 逐行解析一份 UTF-8 LRC 文本。
 * buf 必须可写、NUL 结尾、已剥 BOM；sidecar .lrc 与内嵌 USLT /
 * FLAC LYRICS 共用这条路径，保证两者的时间轴行为完全一致。 */
static void lrc_parse_text(media_info_t *info, char *buf)
{
	char *line, *save;

	if (!lines_ensure(info))
		return;

	line = strtok_r(buf, "\n", &save);
	while (line) {
		char *p = line;
		char text[APP_LYRIC_LEN] = {0};
		uint32_t times[8];
		int ntimes = 0;

		trim_right(p);
		if (p[0] == '[' && parse_lrc_tags(p, info)) {
			line = strtok_r(NULL, "\n", &save);
			continue;
		}

		/* 解析连续的 [mm:ss.xx] 前缀 */
		while (p[0] == '[' && ntimes < 8) {
			int mm = 0, ss = 0, xx = 0;
			char *close = strchr(p, ']');

			if (!close)
				break;
			if (sscanf(p, "[%d:%d.%d]", &mm, &ss, &xx) == 3 ||
			    sscanf(p, "[%d:%d]", &mm, &ss) >= 2) {
				times[ntimes++] = (uint32_t)mm * 60000u +
				                  (uint32_t)ss * 1000u +
				                  (uint32_t)(xx < 100 ? xx * 10 : xx);
				p = close + 1;
			} else {
				break;
			}
		}

		while (*p == ' ')
			p++;
		os_strncpy(text, p, sizeof(text) - 1);

		for (int i = 0; i < ntimes; i++)
			lrc_add(info, times[i], text);

		line = strtok_r(NULL, "\n", &save);
	}

	if (info->offset_ms) {
		for (int i = 0; i < info->line_count; i++) {
			int64_t t = (int64_t)info->lines[i].time_ms + info->offset_ms;

			if (t < 0) t = 0;
			info->lines[i].time_ms = (uint32_t)t;
		}
	}

	lrc_sort(info);
}

/* 文本里是否出现 [mm:ss.xx] 时间戳。
 * 只认「数字 / : / . / -」组成的括号内容，避免把 [by:xxx]、[ti:xxx]
 * 这类元数据行误判成 LRC 而走上错误的时间轴。 */
static bool lrc_text_has_time(const char *s, uint32_t len)
{
	uint32_t i = 0;

	while (i < len) {
		const char *open = memchr(s + i, '[', len - i);
		const char *close;
		uint32_t n, k;
		bool digits = false, colon = false, ok;

		if (!open)
			return false;
		close = memchr(open, ']', (size_t)(s + len - open));
		if (!close)
			return false;
		n = (uint32_t)(close - open - 1);
		ok = (n >= 3) && (n <= 12);
		if (ok) {
			for (k = 0; k < n; k++) {
				char c = open[1 + k];

				if ((c >= '0') && (c <= '9'))
					digits = true;
				else if (c == ':')
					colon = true;
				else if ((c != '.') && (c != '-')) {
					ok = false;
					break;
				}
			}
		}
		if (ok && digits && colon)
			return true;
		i = (uint32_t)(close - s) + 1;
	}
	return false;
}

/* 内嵌歌词若本身是 LRC，按真实时间轴解析；成功返回 true，
 * 失败（纯文本 / 无有效行）由调用方退回“整首均摊”。 */
static bool lrc_from_text(media_info_t *info, const char *text, uint32_t len)
{
	char *buf;
	int first;
	bool ok;

	if (!text || !len || !lrc_text_has_time(text, len))
		return false;

	buf = psram_malloc((size_t)len + 4);
	if (!buf)
		buf = os_malloc((size_t)len + 4);
	if (!buf)
		return false;
	memcpy(buf, text, len);
	buf[len] = '\0';

	/* ID3 UTF-16 正文常带 BOM，转 UTF-8 后就是 EF BB BF */
	if ((len >= 3) && ((uint8_t)buf[0] == 0xEF) &&
	    ((uint8_t)buf[1] == 0xBB) && ((uint8_t)buf[2] == 0xBF))
		memmove(buf, buf + 3, len - 2);

	if (!lines_ensure(info)) {
		os_free(buf);
		return false;
	}

	first = info->line_count;
	lrc_parse_text(info, buf);
	ok = (info->line_count > first);
	os_free(buf);

	if (ok) {
		info->lines_from_tag = true;
		LOGI("lyrics from tag: %d lines (LRC time axis)\r\n",
		     info->line_count - first);
	}
	return ok;
}

void media_parse_lrc(const char *audio_path, media_info_t *info)
{
	char lrc_path[APP_PATH_LEN];
	static FIL f;
	FSIZE_t size;
	UINT br = 0;
	char *buf = NULL;
	int converted_offset = 0;

	make_sidecar(audio_path, "lrc", lrc_path, sizeof(lrc_path));
	if (f_open(&f, lrc_path, FA_READ) != FR_OK)
		return;

	/* 同名 .lrc 优先级高于 ID3 USLT/SYLT 与 FLAC LYRICS：丢掉 tag 已经填的行 */
	info->line_count = 0;
	info->lines_from_tag = false;

	size = f_size(&f);
	if ((size == 0) || (size > 32768)) {
		f_close(&f);
		return;
	}

	buf = psram_malloc((size_t)size + 1);
	if (!buf)
		buf = os_malloc((size_t)size + 1);
	if (!buf) {
		f_close(&f);
		return;
	}

	if (f_read(&f, buf, (UINT)size, &br) != FR_OK) {
		os_free(buf);
		f_close(&f);
		return;
	}
	f_close(&f);
	buf[br] = '\0';

	if ((br >= 3) && ((uint8_t)buf[0] == 0xEF) &&
	    ((uint8_t)buf[1] == 0xBB) && ((uint8_t)buf[2] == 0xBF))
		converted_offset = 3;

	if (!utf8_valid((const uint8_t *)buf + converted_offset, (int)(br - converted_offset))) {
		unsigned char *conv = conv_utf8((unsigned char *)buf + converted_offset);

		if (conv) {
			os_free(buf);
			buf = (char *)conv;
		}
	} else if (converted_offset) {
		memmove(buf, buf + converted_offset, br - converted_offset + 1);
	}

	/* 逐行解析 + offset + 排序：与内嵌 LRC 走同一条路径 */
	lrc_parse_text(info, buf);
	os_free(buf);
}

int media_lrc_index_at(const media_info_t *info, uint32_t pos_ms)
{
	int lo = 0, hi, ans = -1;

	if (!info || !info->lines || info->line_count == 0)
		return -1;

	hi = info->line_count - 1;
	while (lo <= hi) {
		int mid = (lo + hi) / 2;

		if (info->lines[mid].time_ms <= pos_ms) {
			ans = mid;
			lo = mid + 1;
		} else {
			hi = mid - 1;
		}
	}
	return ans;
}

/* ============================ BMP 封面 ============================ */

static uint16_t rgb_to_565(uint8_t r, uint8_t g, uint8_t b)
{
	uint16_t v = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));

	/* ILI9341 单线 SPI 按高字节先发；LVGL 打开 LV_COLOR_16_SWAP 后，
	 * TRUE_COLOR 图像数据也必须是高字节在前，否则红蓝/绿色会错。 */
#if defined(CONFIG_LV_COLOR_16_SWAP) && CONFIG_LV_COLOR_16_SWAP
	v = (uint16_t)((v >> 8) | (v << 8));
#endif
	return v;
}

static bool bmp_load(const char *path, uint16_t *dst, int dw, int dh)
{
	static FIL f;
	uint8_t fh[14], ih[40];
	UINT br = 0;
	uint32_t pixel_off;
	int32_t w, h;
	uint16_t bpp;
	uint32_t compression;
	bool bottom_up;
	uint32_t stride;
	uint8_t *row = NULL;
	bool ok = false;

	if (f_open(&f, path, FA_READ) != FR_OK)
		return false;

	if ((f_read(&f, fh, 14, &br) != FR_OK) || br != 14 || memcmp(fh, "BM", 2))
		goto out;
	if ((f_read(&f, ih, 40, &br) != FR_OK) || br != 40)
		goto out;

	pixel_off = (uint32_t)fh[10] | ((uint32_t)fh[11] << 8) |
	            ((uint32_t)fh[12] << 16) | ((uint32_t)fh[13] << 24);
	w = (int32_t)((uint32_t)ih[4] | ((uint32_t)ih[5] << 8) |
	              ((uint32_t)ih[6] << 16) | ((uint32_t)ih[7] << 24));
	h = (int32_t)((uint32_t)ih[8] | ((uint32_t)ih[9] << 8) |
	              ((uint32_t)ih[10] << 16) | ((uint32_t)ih[11] << 24));
	bpp = (uint16_t)(ih[14] | (ih[15] << 8));
	compression = (uint32_t)ih[16] | ((uint32_t)ih[17] << 8) |
	              ((uint32_t)ih[18] << 16) | ((uint32_t)ih[19] << 24);

	if ((w <= 0) || (h == 0) || (w > 2048) || (h > 2048) || (h < -2048))
		goto out;
	if ((bpp != 24) && (bpp != 32))
		goto out;
	if (compression != 0)
		goto out;

	bottom_up = h > 0;
	if (h < 0)
		h = -h;
	if ((w * (bpp / 8)) > 8192)
		goto out;

	stride = ((uint32_t)w * (bpp / 8) + 3) & ~3u;
	row = psram_malloc(stride);
	if (!row)
		row = os_malloc(stride);
	if (!row)
		goto out;

	for (int y = 0; y < dh; y++) {
		int sy = (int)((int64_t)y * h / dh);
		uint32_t file_row = bottom_up ? (uint32_t)(h - 1 - sy) : (uint32_t)sy;

		if (f_lseek(&f, pixel_off + file_row * stride) != FR_OK)
			break;
		if ((f_read(&f, row, stride, &br) != FR_OK) || br != stride)
			break;

		for (int x = 0; x < dw; x++) {
			int sx = (int)((int64_t)x * w / dw);
			uint8_t *p = row + sx * (bpp / 8);

			dst[y * dw + x] = rgb_to_565(p[2], p[1], p[0]);
		}
	}
	ok = true;

out:
	if (row)
		os_free(row);
	f_close(&f);
	return ok;
}

/* ============================ 内嵌封面取用 ============================ */

bool media_get_embedded_cover(const media_info_t *info, const uint8_t **data,
                              uint32_t *len, bool *is_png)
{
	if (!info || !info->cover_data || !info->cover_len)
		return false;

	if (data)
		*data = info->cover_data;
	if (len)
		*len = info->cover_len;
	if (is_png)
		*is_png = info->cover_is_png;
	return true;
}

bool media_load_bmp_cover(const char *audio_path, uint16_t *dst, int w, int h)
{
	char dir[APP_PATH_LEN], path[APP_PATH_LEN];

	if (!audio_path || !dst || w <= 0 || h <= 0)
		return false;

	make_sidecar(audio_path, "bmp", path, sizeof(path));
	if (bmp_load(path, dst, w, h))
		return true;

	dir_of(audio_path, dir, sizeof(dir));
	snprintf(path, sizeof(path), "%scover.bmp", dir);
	if (bmp_load(path, dst, w, h))
		return true;

	snprintf(path, sizeof(path), "%sfolder.bmp", dir);
	if (bmp_load(path, dst, w, h))
		return true;

	return false;
}
