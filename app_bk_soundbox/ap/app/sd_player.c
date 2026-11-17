// Copyright 2026
// bk_audio_player 组件适配层实现
//
// 自定义插件：
//   source —— FatFs 直读（f_open/f_read/f_lseek），支持 SEEK_SET/CUR/END，
//             组件的 MP3 解码器会 seek(-128, SEEK_END) 读 ID3v1，必须处理负偏移。
//   sink   —— 接 app_audio（统一三模音频路由）。非 16bit 样本（24bit FLAC 等）
//             在此处转成 16bit 再写入；写入字节按“解码输出格式”累计用于进度。
//
// 进度：pos = seek_base_ms + (written - seek_base_written) / bytes_per_sec，
// bytes_per_sec 来自解码器 get_info（ch * rate * bits / 8）。

#include "sd_player.h"

#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include <unistd.h>          /* SEEK_SET / SEEK_CUR / SEEK_END */

#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include <components/log.h>

#include "ff.h"

#include <components/bk_audio_player/bk_audio_player.h>
#include <components/bk_audio_player/bk_audio_player_types.h>
#include "source_api.h"
#include "sink_api.h"
#include "codec_api.h"
#include "player_mem.h"

#include "app_audio.h"

#define TAG "sd_player"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define SDPL_TASK_STACK   16384      /* 与组件默认一致：MP3 解码需要余量 */
#define SDPL_TASK_PRIO    (BEKEN_DEFAULT_WORKER_PRIORITY - 1)
#define SDPL_CVT_SAMPLES  2048       /* 非 16bit 转换块大小（样本数） */

/* ============================ 状态 ============================ */

static bk_audio_player_handle_t s_handle;
static sd_player_cb_t s_cb;
static bool s_inited;
static volatile sd_player_state_t s_state_cache = SD_PLAYER_STOPPED;

static volatile uint32_t s_written;        /* 解码输出累计字节（seek 基准前） */
static volatile uint32_t s_bytes_per_sec;  /* 解码输出字节率（输入格式口径） */
static volatile uint32_t s_seek_base_ms;
static volatile uint32_t s_seek_base_written;
static audio_info_t s_dec_info;            /* 最近一次解码器流信息 */

/* ============================ FatFs source 插件 ============================ */

typedef struct {
	FIL f;
	uint32_t file_size;
	int codec_type;
} fatfs_src_priv_t;

static int fatfs_src_open(char *url, bk_audio_player_source_t **source_pp)
{
	bk_audio_player_source_t *source;
	fatfs_src_priv_t *priv;
	FRESULT fr;
	char *ext;

	if (!url || !url[0])
		return AUDIO_PLAYER_INVALID;

	source = audio_source_new(sizeof(fatfs_src_priv_t));
	if (!source)
		return AUDIO_PLAYER_NO_MEM;

	priv = (fatfs_src_priv_t *)source->source_priv;

	ext = strrchr(url, '.');
	priv->codec_type = audio_codec_get_type(ext);
	if (priv->codec_type == AUDIO_FORMAT_UNKNOWN) {
		LOGW("unsupported ext: %s\r\n", ext ? ext : "(none)");
		player_free(source);
		return AUDIO_PLAYER_ERR;
	}

	fr = f_open(&priv->f, url, FA_READ);
	if (fr != FR_OK) {
		LOGE("f_open %s: %d\r\n", url, fr);
		player_free(source);
		return AUDIO_PLAYER_ERR;
	}

	priv->file_size = (uint32_t)f_size(&priv->f);
	*source_pp = source;

	LOGI("open %s (%u bytes)\r\n", url, (unsigned)priv->file_size);
	return AUDIO_PLAYER_OK;
}

static int fatfs_src_close(bk_audio_player_source_t *source)
{
	fatfs_src_priv_t *priv = (fatfs_src_priv_t *)source->source_priv;

	f_close(&priv->f);
	return AUDIO_PLAYER_OK;
}

static int fatfs_src_get_codec_type(bk_audio_player_source_t *source)
{
	return ((fatfs_src_priv_t *)source->source_priv)->codec_type;
}

static uint32_t fatfs_src_get_total_bytes(bk_audio_player_source_t *source)
{
	return ((fatfs_src_priv_t *)source->source_priv)->file_size;
}

static int fatfs_src_read(bk_audio_player_source_t *source, char *buffer, int len)
{
	fatfs_src_priv_t *priv = (fatfs_src_priv_t *)source->source_priv;
	UINT br = 0;

	if ((len <= 0) || !buffer)
		return 0;

	if (f_read(&priv->f, buffer, (UINT)len, &br) != FR_OK)
		return 0;

	return (int)br;
}

static int fatfs_src_seek(bk_audio_player_source_t *source, int offset, uint32_t whence)
{
	fatfs_src_priv_t *priv = (fatfs_src_priv_t *)source->source_priv;
	uint32_t target;
	FRESULT fr;

	switch (whence) {
	case SEEK_SET:
		target = (offset >= 0) ? (uint32_t)offset : 0u;
		break;
	case SEEK_CUR:
		target = (uint32_t)f_tell(&priv->f) + (uint32_t)offset;  /* offset 可为负 */
		break;
	case SEEK_END:
		/* 典型用途：mp3 解码器 seek(-128, SEEK_END) 读 ID3v1 */
		if (offset < 0) {
			uint32_t back = (uint32_t)(-(int64_t)offset);

			target = (back >= priv->file_size) ? 0u : (priv->file_size - back);
		} else {
			target = priv->file_size + (uint32_t)offset;
		}
		break;
	default:
		return AUDIO_PLAYER_INVALID;
	}

	fr = f_lseek(&priv->f, target);
	if (fr != FR_OK)
		return AUDIO_PLAYER_ERR;

	return AUDIO_PLAYER_OK;
}

static const bk_audio_player_source_ops_t s_fatfs_src_ops = {
	.open            = fatfs_src_open,
	.get_codec_type  = fatfs_src_get_codec_type,
	.get_total_bytes = fatfs_src_get_total_bytes,
	.read            = fatfs_src_read,
	.seek            = fatfs_src_seek,
	.close           = fatfs_src_close,
};

/* ============================ app_audio sink 插件 ============================ */

typedef struct {
	uint8_t bits;              /* 解码输出样本位宽（8/16/24） */
	uint16_t chans;
	uint32_t rate;
} appaudio_sink_priv_t;

/* 把解码输出统一转成 16bit 后写 app_audio；返回 0=成功 */
static int sink_write_16(appaudio_sink_priv_t *priv, const char *buf, uint32_t len)
{
	static int16_t cvt[SDPL_CVT_SAMPLES];
	uint32_t bps = priv->bits / 8;
	uint32_t samples = len / bps;
	uint32_t off = 0;

	while (off < samples) {
		uint32_t n = samples - off;

		if (n > SDPL_CVT_SAMPLES)
			n = SDPL_CVT_SAMPLES;

		for (uint32_t i = 0; i < n; i++) {
			const uint8_t *p = (const uint8_t *)buf + (size_t)(off + i) * bps;

			if (priv->bits == 24) {
				int32_t v = (int32_t)(p[0] | (p[1] << 8) | (p[2] << 16));

				if (v & 0x800000)
					v |= ~0xFFFFFF;
				cvt[i] = (int16_t)(v >> 8);
			} else { /* 8bit 无符号 */
				cvt[i] = (int16_t)(((int16_t)p[0] - 128) << 8);
			}
		}

		if (app_audio_write(cvt, n * sizeof(int16_t)) != BK_OK)
			return -1;

		off += n;
	}

	return 0;
}

static int appaudio_sink_open(audio_sink_type_t sink_type, void *param,
                              bk_audio_player_sink_t **sink_pp)
{
	bk_audio_player_sink_t *sink;
	appaudio_sink_priv_t *priv;
	device_sink_param_t *dev = (device_sink_param_t *)param;
	audio_info_t *info;
	bk_err_t ret;

	if (sink_type != AUDIO_SINK_DEVICE)
		return AUDIO_PLAYER_INVALID;

	if (!dev || !dev->info)
		return AUDIO_PLAYER_ERR;

	info = dev->info;
	if ((info->sample_rate < 8000) || (info->sample_rate > 48000) ||
	    ((info->channel_number != 1) && (info->channel_number != 2)) ||
	    (info->sample_bits != 8 && info->sample_bits != 16 && info->sample_bits != 24)) {
		LOGE("unsupported stream: %dHz %dch %dbit\r\n",
		     info->sample_rate, info->channel_number, info->sample_bits);
		return AUDIO_PLAYER_ERR;
	}

	sink = audio_sink_new(sizeof(appaudio_sink_priv_t));
	if (!sink)
		return AUDIO_PLAYER_NO_MEM;

	priv = (appaudio_sink_priv_t *)sink->sink_priv;
	priv->rate = (uint32_t)info->sample_rate;
	priv->chans = (uint16_t)info->channel_number;
	priv->bits = (uint8_t)info->sample_bits;

	ret = app_audio_open(priv->rate, (uint8_t)priv->chans);
	if (ret != BK_OK) {
		player_free(sink);
		return AUDIO_PLAYER_ERR;
	}

	/* 记录解码器流信息，供进度与总时长使用 */
	os_memcpy(&s_dec_info, info, sizeof(*info));
	s_written = 0;
	s_seek_base_ms = 0;
	s_seek_base_written = 0;
	s_bytes_per_sec = (uint32_t)info->channel_number * info->sample_rate *
	                  info->sample_bits / 8;
	if (s_bytes_per_sec == 0)
		s_bytes_per_sec = 1;    /* 防 0 除，位置表现为原地不动 */

	LOGI("sink open %uHz %uch %ubit (%u B/s)\r\n",
	     priv->rate, priv->chans, priv->bits, (unsigned)s_bytes_per_sec);

	*sink_pp = sink;
	return AUDIO_PLAYER_OK;
}

static int appaudio_sink_write(bk_audio_player_sink_t *sink, char *buffer, int len)
{
	appaudio_sink_priv_t *priv = (appaudio_sink_priv_t *)sink->sink_priv;
	int ret;

	if ((len <= 0) || !buffer)
		return len;

	if (priv->bits == 16) {
		ret = app_audio_write(buffer, (uint32_t)len) == BK_OK ? 0 : -1;
	} else {
		ret = sink_write_16(priv, buffer, (uint32_t)len);
	}

	if (ret != 0) {
		LOGE("audio write failed\r\n");
		return -1;
	}

	s_written += (uint32_t)len;
	return len;
}

static int appaudio_sink_control(bk_audio_player_sink_t *sink, audio_sink_control_t control)
{
	appaudio_sink_priv_t *priv = (appaudio_sink_priv_t *)sink->sink_priv;

	switch (control) {
	case AUDIO_SINK_PAUSE:
		app_audio_pause();
		break;
	case AUDIO_SINK_RESUME:
		app_audio_resume();
		break;
	case AUDIO_SINK_FRAME_INFO_CHANGE:
		/* 采样格式中途变化（少见）：sink->info 已被组件更新，重开音频并刷新字节率 */
		app_audio_open((uint32_t)sink->info.sampRate, (uint8_t)sink->info.nChans);
		priv->rate = (uint32_t)sink->info.sampRate;
		priv->chans = (uint16_t)sink->info.nChans;
		priv->bits = (uint8_t)sink->info.bitsPerSample;
		s_bytes_per_sec = (uint32_t)sink->info.nChans * sink->info.sampRate *
		                  sink->info.bitsPerSample / 8;
		if (s_bytes_per_sec == 0)
			s_bytes_per_sec = 1;
		LOGI("frame change %uHz %uch %ubit\r\n", priv->rate, priv->chans, priv->bits);
		break;
	case AUDIO_SINK_MUTE:
	case AUDIO_SINK_UNMUTE:
	case AUDIO_SINK_SET_VOLUME:
		/* 音量/mute 由 app_audio 统一管理，组件侧无需重复控制 */
		break;
	default:
		break;
	}

	return AUDIO_PLAYER_OK;
}

static int appaudio_sink_close(bk_audio_player_sink_t *sink)
{
	(void)sink;
	app_audio_close();
	return AUDIO_PLAYER_OK;
}

static const bk_audio_player_sink_ops_t s_appaudio_sink_ops = {
	.open    = appaudio_sink_open,
	.write   = appaudio_sink_write,
	.control = appaudio_sink_control,
	.close   = appaudio_sink_close,
};

/* ============================ 事件分发 ============================ */

static void sdpl_event(audio_player_event_type_t event, void *extra_info, void *args)
{
	(void)args;

	switch (event) {
	case AUDIO_PLAYER_EVENT_SONG_START:
		s_state_cache = SD_PLAYER_PLAYING;
		s_written = 0;
		s_seek_base_ms = 0;
		s_seek_base_written = 0;
		if (s_cb.on_start)
			s_cb.on_start(NULL);
		break;

	case AUDIO_PLAYER_EVENT_SONG_FINISH:
		s_state_cache = SD_PLAYER_STOPPED;
		if (s_cb.on_finished)
			s_cb.on_finished(false);
		break;

	case AUDIO_PLAYER_EVENT_SONG_FAILURE:
		s_state_cache = SD_PLAYER_STOPPED;
		if (s_cb.on_finished)
			s_cb.on_finished(true);
		break;

	case AUDIO_PLAYER_EVENT_SONG_PAUSE:
		s_state_cache = SD_PLAYER_PAUSED;
		if (s_cb.on_paused)
			s_cb.on_paused(true);
		break;

	case AUDIO_PLAYER_EVENT_SONG_RESUME:
		s_state_cache = SD_PLAYER_PLAYING;
		if (s_cb.on_paused)
			s_cb.on_paused(false);
		break;

	case AUDIO_PLAYER_EVENT_SEEK_COMPLETE: {
		audio_player_seek_result_t *r = (audio_player_seek_result_t *)extra_info;

		if (r && (r->status == AUDIO_PLAYER_OK)) {
			/* 组件已把 consumed_bytes 设为 target*bytes_per_sec，
			 * 这里把进度基准对齐到目标秒 */
			s_seek_base_ms = (uint32_t)r->second * 1000u;
			s_seek_base_written = s_written;
		}
		if (s_cb.on_seek_done)
			s_cb.on_seek_done(r && (r->status == AUDIO_PLAYER_OK),
			                  r ? r->second : -1);
		break;
	}

	default:
		break;
	}
}

/* ============================ 对外接口 ============================ */

bk_err_t sd_player_init(const sd_player_cb_t *cb)
{
	bk_audio_player_cfg_t cfg = {
		.task_stack = SDPL_TASK_STACK,
		.task_prio = SDPL_TASK_PRIO,
		.event_handler = sdpl_event,
		.args = NULL,
	};
	int ret;

	if (s_inited)
		return BK_OK;

	if (cb)
		s_cb = *cb;
	else
		os_memset(&s_cb, 0, sizeof(s_cb));

	ret = bk_audio_player_new(&s_handle, &cfg);
	if (ret != AUDIO_PLAYER_OK) {
		LOGE("player new: %d\r\n", ret);
		return BK_FAIL;
	}

	/* 插件逐个注册：任一失败都说明组件配置异常 */
	ret = bk_audio_player_register_source(s_handle, &s_fatfs_src_ops);
	if (ret != AUDIO_PLAYER_OK)
		goto out_fail;

	ret = bk_audio_player_register_sink(s_handle, &s_appaudio_sink_ops);
	if (ret != AUDIO_PLAYER_OK)
		goto out_fail;

	{
		extern const bk_audio_player_decoder_ops_t *bk_audio_player_get_wav_decoder_ops(void);
		extern const bk_audio_player_decoder_ops_t *bk_audio_player_get_mp3_decoder_ops(void);
		extern const bk_audio_player_decoder_ops_t *bk_audio_player_get_flac_decoder_ops(void);

		ret = bk_audio_player_register_decoder(s_handle, bk_audio_player_get_wav_decoder_ops());
		if (ret == AUDIO_PLAYER_OK)
			ret = bk_audio_player_register_decoder(s_handle, bk_audio_player_get_mp3_decoder_ops());
		if (ret == AUDIO_PLAYER_OK)
			ret = bk_audio_player_register_decoder(s_handle, bk_audio_player_get_flac_decoder_ops());
		if (ret != AUDIO_PLAYER_OK)
			goto out_fail;
	}

	ret = bk_audio_player_set_play_mode(s_handle, AUDIO_PLAYER_MODE_ONE_SONG);
	if (ret != AUDIO_PLAYER_OK)
		goto out_fail;

	s_inited = true;
	LOGI("audio player ready\r\n");
	return BK_OK;

out_fail:
	LOGE("register plugin: %d\r\n", ret);
	bk_audio_player_delete(s_handle);
	s_handle = NULL;
	return BK_FAIL;
}

bk_err_t sd_player_play_file(const char *path)
{
	int ret;
	char name[64];
	const char *base;

	if (!s_inited || !path || !path[0])
		return BK_ERR_PARAM;

	base = strrchr(path, '/');
	os_strncpy(name, base ? base + 1 : path, sizeof(name) - 1);
	name[sizeof(name) - 1] = '\0';

	ret = bk_audio_player_clear_music_list(s_handle);
	if (ret != AUDIO_PLAYER_OK)
		return BK_FAIL;

	ret = bk_audio_player_add_music(s_handle, name, (char *)path);
	if (ret != AUDIO_PLAYER_OK)
		return BK_FAIL;

	ret = bk_audio_player_start(s_handle);
	if (ret != AUDIO_PLAYER_OK) {
		LOGE("start %s: %d\r\n", path, ret);
		return BK_FAIL;
	}

	return BK_OK;
}

bk_err_t sd_player_pause(void)
{
	if (!s_inited)
		return BK_ERR_NOT_INIT;
	return (bk_audio_player_pause(s_handle) == AUDIO_PLAYER_OK) ? BK_OK : BK_FAIL;
}

bk_err_t sd_player_resume(void)
{
	if (!s_inited)
		return BK_ERR_NOT_INIT;
	return (bk_audio_player_resume(s_handle) == AUDIO_PLAYER_OK) ? BK_OK : BK_FAIL;
}

bk_err_t sd_player_stop(void)
{
	if (!s_inited)
		return BK_OK;

	/* stop 是同步命令：teardown 不发事件，这里直接把状态缓存归位 */
	if (bk_audio_player_stop(s_handle) == AUDIO_PLAYER_OK) {
		s_state_cache = SD_PLAYER_STOPPED;
		return BK_OK;
	}
	return BK_FAIL;
}

bk_err_t sd_player_seek_abs(int second)
{
	if (!s_inited)
		return BK_ERR_NOT_INIT;
	return (bk_audio_player_seek(s_handle, second) == AUDIO_PLAYER_OK) ? BK_OK : BK_FAIL;
}

sd_player_state_t sd_player_state(void)
{
	if (!s_inited)
		return SD_PLAYER_STOPPED;

	/* 组件状态机经事件间接同步：PAUSE/RESUME/START/FINISH 都有回调，
	 * 这里不再直接读内部状态，避免依赖私有结构布局。 */
	return s_state_cache;
}

uint32_t sd_player_pos_ms(void)
{
	uint32_t written = s_written;
	uint32_t base_written = s_seek_base_written;
	uint32_t delta_ms;

	if (written < base_written)
		written = base_written;    /* 防御：基准重置与累计的竞争窗口 */

	delta_ms = (uint32_t)((uint64_t)(written - base_written) * 1000u /
	                      s_bytes_per_sec);

	return s_seek_base_ms + delta_ms;
}

uint32_t sd_player_total_ms(void)
{
	uint32_t total_bytes = s_dec_info.total_bytes - s_dec_info.header_bytes;
	uint32_t bps = (uint32_t)s_dec_info.bps;

	if (s_dec_info.duration > 0.0)
		return (uint32_t)s_dec_info.duration;

	if ((bps > 0) && (total_bytes > 0))
		return (uint32_t)((uint64_t)total_bytes * 8000u / bps);

	return 0;
}
