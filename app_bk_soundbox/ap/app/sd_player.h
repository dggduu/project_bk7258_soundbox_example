// Copyright 2026
// SD 播放适配层：把 SDK bk_audio_player 组件接到本工程的 FatFs / app_audio 上
//
// 组件负责播放状态机、解码（WAV/MP3/FLAC，复用 SDK 自带解码库）与 seek 字节定位；
// 数据源用 FatFs（路径形如 "1:/music/x.mp3"），输出接到统一音频路由 app_audio，
// 这样 SD / 蓝牙 / USB 三模共用 DAC、音量与 mute 逻辑，组件自带的 ADK 输出栈不参与。
//
// 线程模型：组件内部任务跑解码循环；本层所有公开 API 从 app 任务调用。
// 事件回调在组件内部任务上下文触发，回调里只允许改本层变量或投队列，禁止再调
// 本层的同步控制 API（会死锁：回调等返回、控制命令等内部任务处理完）。

#pragma once

#include <common/bk_err.h>
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	SD_PLAYER_STOPPED = 0,
	SD_PLAYER_PLAYING,
	SD_PLAYER_PAUSED,
} sd_player_state_t;

/* 事件回调（组件内部任务上下文，必须非阻塞） */
typedef struct {
	void (*on_start)(const char *url);           /* 管线建立完成，开始解码 */
	void (*on_finished)(bool failure);           /* 播完（failure=false）或失败 */
	void (*on_seek_done)(bool ok, int second);   /* SEEK_COMPLETE，second 为目标秒 */
	void (*on_paused)(bool paused);              /* SONG_PAUSE / SONG_RESUME */
} sd_player_cb_t;

/* 创建组件实例并注册 FatFs source / app_audio sink / 三个解码器 */
bk_err_t sd_player_init(const sd_player_cb_t *cb);

/* 清空列表、加入单曲并开播（同步：等组件内部任务把管线建好，约几十~几百 ms） */
bk_err_t sd_player_play_file(const char *path);

bk_err_t sd_player_pause(void);
bk_err_t sd_player_resume(void);
bk_err_t sd_player_stop(void);          /* 同步等待组件 teardown（关音频） */

/* 绝对秒跳转（非阻塞，SEEK_COMPLETE 事件报结果） */
bk_err_t sd_player_seek_abs(int second);

sd_player_state_t sd_player_state(void);

/* 当前播放位置（毫秒）。由 sink 实际写入的解码输出字节数累计，seek 完成时重置基准，
 * 精度为一个解码 chunk（几十 ms），比组件秒级 SONG_TICK 事件更平滑。 */
uint32_t sd_player_pos_ms(void);

/* 本曲总时长（毫秒）；解码器给不出时返回 0 */
uint32_t sd_player_total_ms(void);

#ifdef __cplusplus
}
#endif
