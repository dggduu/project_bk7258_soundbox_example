// Copyright 2026
// SD 音频元数据：ID3v1/v2、LRC 歌词、BMP 封面

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define MEDIA_MAX_LRC_LINES 200
#define MEDIA_MAX_COVER_BYTES (2 * 1024 * 1024)   /* 内嵌封面原图上限（PSRAM） */

typedef struct {
	uint32_t time_ms;
	char text[APP_LYRIC_LEN];
} media_lrc_line_t;

typedef struct {
	char title[APP_TITLE_LEN];
	char artist[APP_ARTIST_LEN];
	char album[APP_ALBUM_LEN];
	uint32_t duration_ms;
	media_lrc_line_t *lines;
	int line_count;
	int offset_ms;
	bool lines_in_psram;
	bool lines_from_tag;      /* 来自 ID3 USLT/SYLT 或 FLAC LYRICS，而非 .lrc */

	/* 内嵌封面（ID3v2 APIC / FLAC PICTURE），原始压缩数据 */
	uint8_t *cover_data;
	uint32_t cover_len;
	bool cover_is_png;
} media_info_t;

void media_info_reset(media_info_t *info);
void media_info_free(media_info_t *info);

/* 解析 MP3 的 ID3v2/ID3v1、FLAC 的 STREAMINFO/VorbisComment/PICTURE；
 * 顺带把内嵌歌词（USLT/SYLT/LYRICS）填进 lines。文件名兜底。 */
void media_parse_tags(const char *path, media_info_t *info);

/* 找同名 .lrc 并解析（UTF-8 或 GB2312）；有 .lrc 时覆盖内嵌歌词 */
void media_parse_lrc(const char *audio_path, media_info_t *info);

/* 取内嵌封面原始数据（JPEG/PNG），没有则返回 false */
bool media_get_embedded_cover(const media_info_t *info, const uint8_t **data,
                              uint32_t *len, bool *is_png);

/* pos_ms 对应的歌词行，-1 表示还没有歌词 */
int media_lrc_index_at(const media_info_t *info, uint32_t pos_ms);

/* 读同名 .bmp / cover.bmp / folder.bmp，缩放到 w*h RGB565 */
bool media_load_bmp_cover(const char *audio_path, uint16_t *dst, int w, int h);

#ifdef __cplusplus
}
#endif
