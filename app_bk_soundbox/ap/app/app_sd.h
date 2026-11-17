// Copyright 2026
// SD 卡模式：FATFS 浏览 + WAV/MP3 播放 + LRC/封面

#pragma once

#include <common/bk_err.h>
#include <stdbool.h>
#include <stdint.h>

#include "app_state.h"

#ifdef __cplusplus
extern "C" {
#endif

#define APP_SD_MAX_ENTRIES   48
#define APP_SD_MAX_PLAYLIST  48
#define APP_SD_COVER_SIZE    104

typedef struct {
	char name[APP_NAME_LEN];
	bool is_dir;
	uint32_t size;
} app_sd_entry_t;

/* 挂载 SD 卡并初始化模块（同步，约几十 ms） */
void app_sd_init(void);

/* 进入/离开 SD 模式 */
void app_sd_set_active(bool active);

/* 列目录，返回条目数（<0 出错）。path 形如 "1:/" */
int app_sd_list_dir(const char *path, app_sd_entry_t *out, int max);

/* 播放指定文件（app 任务上下文调用，会先停掉当前播放并等线程退出） */
void app_sd_play(const char *path);

void app_sd_play_pause(void);
bool app_sd_stop(void);
void app_sd_next(void);
void app_sd_prev(void);

/* 相对当前位置跳转（毫秒，正=快进 / 负=快退），由 bk_audio_player_seek 实现 */
void app_sd_seek(int delta_ms);

/* 由 app 任务周期调用 */
void app_sd_poll(void);

bool app_sd_is_playing(void);

/* 把当前封面（116x116 RGB565）拷到 UI 自己的缓冲区；返回是否有效 */
bool app_sd_copy_cover(uint16_t *dst, int w, int h);
uint32_t app_sd_cover_gen(void);

#ifdef __cplusplus
}
#endif
