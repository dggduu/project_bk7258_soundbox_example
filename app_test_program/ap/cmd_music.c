// Copyright 2026
// CherrySH 命令：music —— 从 SD 卡播放音乐（WAV / MP3）
//
//   music play <file>    播放 SD 卡上的 .wav / .mp3（后台任务，shell 不阻塞）
//   music stop           停止播放
//   music pause/resume   暂停 / 继续
//   music vol <0-100>    设置音量（默认 70）
//   music list [path]    列出 SD 卡上的音频文件
//   music status         查看当前状态
//
// 音频通路：SD 卡 -> FATFS -> PCM -> audio_play -> 内部 DAC(AUDLP/AUDN)
//           -> NS4150B 差分功放 -> CN3 喇叭；PA_EN = P52（高有效）。
//
// 格式支持：
//   - WAV：PCM，16bit，单/双声道（44.1k / 48k / 32k …）
//   - MP3：SDK 自带的 Helix 解码器（libhelix.a），自动识别采样率/声道
//
// 依赖：CONFIG_AUDIO_PLAY=y、CONFIG_AUDIO_DAC=y、CONFIG_FATFS_SDCARD=y，
//       SDIO 引脚映射见 ap/config/bk7258_ap/usr_gpio_cfg.h。

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_audio.h"
#include "bsp_board_cfg.h"

#include <os/os.h>
#include <os/mem.h>
#include <driver/sd_card.h>
#include <driver/pwr_clk.h>
#include <modules/mp3dec.h>
#include "ff.h"

#define MUSIC_PATH_LEN      128
#define MUSIC_COPY_CHUNK    4096
#define MUSIC_TASK_STACK    8192
#define MUSIC_STOP_TIMEOUT  3000
#define MUSIC_DRAIN_MS      150      /* 播完后等环形缓冲排空，避免截尾 */

typedef enum {
	MUSIC_TYPE_NONE = 0,
	MUSIC_TYPE_WAV,
	MUSIC_TYPE_MP3,
} music_type_t;

typedef enum {
	MUSIC_STATE_IDLE = 0,
	MUSIC_STATE_PLAYING,
} music_state_t;

static FATFS s_fs;
static FIL s_fil;
static bool s_fs_mounted;

static volatile music_state_t s_state = MUSIC_STATE_IDLE;
static volatile bool s_stop_req;
static music_type_t s_type;
static char s_path[MUSIC_PATH_LEN];
static uint8_t s_volume = 70;

static beken_thread_t s_task = NULL;
static beken_semaphore_t s_done_sem = NULL;

/* ---------------- 小工具 ---------------- */

static uint16_t rd16le(const uint8_t *p)
{
	return (uint16_t)(p[0] | (p[1] << 8));
}

static uint32_t rd32le(const uint8_t *p)
{
	return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
	       ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static bool str_ends_with_ci(const char *s, const char *suffix)
{
	size_t ls = strlen(s), lf = strlen(suffix);

	if (ls < lf)
		return false;

	for (size_t i = 0; i < lf; i++) {
		char a = s[ls - lf + i], b = suffix[i];
		if ((a >= 'A') && (a <= 'Z'))
			a = (char)(a - 'A' + 'a');
		if ((b >= 'A') && (b <= 'Z'))
			b = (char)(b - 'A' + 'a');
		if (a != b)
			return false;
	}
	return true;
}

static bool is_audio_file(const char *name)
{
	return str_ends_with_ci(name, ".wav") || str_ends_with_ci(name, ".mp3");
}

static music_type_t detect_type(const char *name)
{
	if (str_ends_with_ci(name, ".wav"))
		return MUSIC_TYPE_WAV;
	if (str_ends_with_ci(name, ".mp3"))
		return MUSIC_TYPE_MP3;
	return MUSIC_TYPE_NONE;
}

/* 支持 music/a.mp3、/music/a.mp3、1:/a.mp3 三种写法 */
static int build_path(const char *arg, char *out, size_t outlen)
{
	if ((arg == NULL) || (arg[0] == '\0'))
		return -1;

	if (arg[0] == '/')
		snprintf(out, outlen, "1:%s", arg);
	else if ((strlen(arg) > 1) && (arg[1] == ':'))
		snprintf(out, outlen, "%s", arg);
	else
		snprintf(out, outlen, "1:/%s", arg);

	return 0;
}

/* ---------------- SD / FATFS ---------------- */

static int music_sd_mount(chry_shell_t *csh)
{
	FRESULT fr;
	bk_err_t ret;

	if (s_fs_mounted)
		return 0;

	/* 卡座 3V3 LDO（和 cmd_sd.c 里一样，重复调用无副作用） */
	bk_pm_module_vote_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO,
	                                    CONFIG_LDO3V3_CTRL_GPIO,
	                                    GPIO_OUTPUT_STATE_HIGH);
	rtos_delay_milliseconds(5);

	ret = bk_sd_card_init();
	if (ret != BK_OK) {
		csh_printf(csh, "music: sd init failed: %d\r\n", ret);
		return -1;
	}

	fr = f_mount(&s_fs, "1:", 1);
	if (fr != FR_OK) {
		csh_printf(csh, "music: f_mount failed: %d（卡没插/不是 FAT 分区）\r\n", fr);
		return -1;
	}
	s_fs_mounted = true;
	return 0;
}

/* ---------------- WAV ---------------- */

static bk_err_t play_wav(FIL *f, chry_shell_t *csh)
{
	uint8_t hdr[12], ck[8], fmt[16];
	UINT br = 0;
	FSIZE_t file_size = f_size(f);
	FSIZE_t data_off = 0;
	uint32_t data_len = 0;
	uint16_t audio_fmt = 0, chans = 0, bits = 0;
	uint32_t rate = 0;
	bool have_fmt = false, have_data = false;
	bsp_audio_cfg_t cfg;
	uint8_t *buf = NULL;
	uint32_t left;
	bk_err_t ret = BK_FAIL;

	f_lseek(f, 0);
	if ((f_read(f, hdr, 12, &br) != FR_OK) || (br != 12))
		goto out;
	if (memcmp(hdr, "RIFF", 4) || memcmp(hdr + 8, "WAVE", 4)) {
		csh_printf(csh, "music: not a RIFF/WAVE file\r\n");
		goto out;
	}

	while (f_read(f, ck, 8, &br) == FR_OK && br == 8) {
		uint32_t sz = rd32le(ck + 4);

		if (!memcmp(ck, "fmt ", 4)) {
			UINT want = (sz < sizeof(fmt)) ? sz : sizeof(fmt);

			if ((f_read(f, fmt, want, &br) != FR_OK) || (br != want) || (want < 16))
				goto out;

			audio_fmt = rd16le(fmt + 0);
			chans = rd16le(fmt + 2);
			rate = rd32le(fmt + 4);
			bits = rd16le(fmt + 14);
			have_fmt = true;

			if (sz > want)
				f_lseek(f, f_tell(f) + (sz - want));
		} else if (!memcmp(ck, "data", 4)) {
			data_off = f_tell(f);
			data_len = sz;
			have_data = true;
			break;
		} else {
			f_lseek(f, f_tell(f) + sz);
		}

		if (sz & 1)
			f_lseek(f, f_tell(f) + 1);
		if (f_tell(f) >= file_size)
			break;
	}

	if (!have_fmt || !have_data) {
		csh_printf(csh, "music: wav missing fmt/data chunk\r\n");
		goto out;
	}
	/* 0xFFFE = WAVE_FORMAT_EXTENSIBLE，实际还是 PCM（16bit） */
	if ((audio_fmt != 1) && (audio_fmt != 0xFFFE)) {
		csh_printf(csh, "music: wav format %u not supported (PCM only)\r\n", audio_fmt);
		goto out;
	}
	if ((chans != 1) && (chans != 2)) {
		csh_printf(csh, "music: wav channels %u not supported\r\n", chans);
		goto out;
	}
	if (bits != 16) {
		csh_printf(csh, "music: wav %u-bit not supported (16bit only)\r\n", bits);
		goto out;
	}
	if ((rate < 8000) || (rate > 48000)) {
		csh_printf(csh, "music: wav sample rate %u not supported\r\n", rate);
		goto out;
	}
	if (data_off + data_len > file_size)
		data_len = (uint32_t)(file_size - data_off);

	bsp_audio_default_cfg(&cfg);
	cfg.samp_rate = rate;
	cfg.channels = (uint8_t)chans;
	cfg.bits = 16;
	cfg.volume = s_volume;
	/* frame_size / pool_size = 0：由 BSP 按采样率自动算 */

	ret = bsp_audio_init(&cfg);
	if (ret != BK_OK) {
		csh_printf(csh, "music: audio init failed: %d\r\n", ret);
		goto out;
	}

	csh_printf(csh, "music: wav %uHz %uch 16bit %u KB\r\n",
	           rate, chans, (unsigned)(data_len / 1024));

	buf = os_malloc(MUSIC_COPY_CHUNK);
	if (buf == NULL) {
		csh_printf(csh, "music: malloc failed\r\n");
		ret = BK_FAIL;
		goto out;
	}

	left = data_len;
	f_lseek(f, data_off);
	ret = BK_OK;
	while ((left > 0) && !s_stop_req) {
		UINT want = (left > MUSIC_COPY_CHUNK) ? MUSIC_COPY_CHUNK : left;

		if ((f_read(f, buf, want, &br) != FR_OK) || (br == 0))
			break;

		if (bsp_audio_write(buf, br) != BK_OK) {
			csh_printf(csh, "music: audio write failed\r\n");
			ret = BK_FAIL;
			break;
		}
		left -= br;
	}

	if (!s_stop_req && (ret == BK_OK))
		rtos_delay_milliseconds(MUSIC_DRAIN_MS);

out:
	if (buf)
		os_free(buf);
	return ret;
}

/* ---------------- MP3 ---------------- */

static bk_err_t play_mp3(FIL *f, chry_shell_t *csh)
{
	HMP3Decoder dec = NULL;
	MP3FrameInfo info;
	unsigned char *rbuf = NULL;
	short *pcm = NULL;
	unsigned char *rp;
	int bytes_left = 0;
	int bad_frames = 0;
	bool audio_started = false;
	uint8_t tag[10];
	UINT br = 0;
	bk_err_t ret = BK_FAIL;

	dec = MP3InitDecoder();
	if (dec == NULL) {
		csh_printf(csh, "music: MP3 decoder init failed\r\n");
		goto out;
	}

	rbuf = os_malloc(MAINBUF_SIZE);
	pcm = os_malloc(MAX_NSAMP * MAX_NCHAN * MAX_NGRAN * sizeof(short));
	if ((rbuf == NULL) || (pcm == NULL)) {
		csh_printf(csh, "music: malloc failed\r\n");
		goto out;
	}
	rp = rbuf;

	/* 跳过 ID3v2 头 */
	f_lseek(f, 0);
	if ((f_read(f, tag, 10, &br) == FR_OK) && (br == 10) && !memcmp(tag, "ID3", 3)) {
		int tag_size = ((tag[6] & 0x7F) << 21) | ((tag[7] & 0x7F) << 14) |
		               ((tag[8] & 0x7F) << 7) | (tag[9] & 0x7F);
		f_lseek(f, (FSIZE_t)(tag_size + 10));
	} else {
		f_lseek(f, 0);
	}

	ret = BK_OK;
	while (!s_stop_req) {
		if (bytes_left < MAINBUF_SIZE) {
			if (bytes_left > 0)
				os_memmove(rbuf, rp, bytes_left);

			if (f_read(f, rbuf + bytes_left, MAINBUF_SIZE - bytes_left, &br) != FR_OK) {
				ret = BK_FAIL;
				break;
			}
			bytes_left += br;
			rp = rbuf;

			if (bytes_left == 0)      /* 文件读完 */
				break;
		}

		int off = MP3FindSyncWord(rp, bytes_left);
		if (off < 0) {
			bytes_left = 0;
			continue;
		}
		rp += off;
		bytes_left -= off;

		int rc = MP3Decode(dec, &rp, &bytes_left, pcm, 0);
		if (rc != ERR_MP3_NONE) {
			if (++bad_frames > 16) {
				csh_printf(csh, "music: too many bad mp3 frames\r\n");
				ret = BK_FAIL;
				break;
			}
			continue;
		}
		bad_frames = 0;

		MP3GetLastFrameInfo(dec, &info);
		if (info.outputSamps <= 0)
			continue;

		if (!audio_started) {
			bsp_audio_cfg_t cfg;

			bsp_audio_default_cfg(&cfg);
			cfg.samp_rate = (uint32_t)info.samprate;
			cfg.channels = (uint8_t)info.nChans;
			cfg.bits = 16;
			cfg.volume = s_volume;

			ret = bsp_audio_init(&cfg);
			if (ret != BK_OK) {
				csh_printf(csh, "music: audio init failed: %d\r\n", ret);
				break;
			}
			audio_started = true;

			csh_printf(csh, "music: mp3 %dHz %dch %dkbps\r\n",
			           info.samprate, info.nChans, info.bitrate / 1000);
		}

		if (bsp_audio_write(pcm, (uint32_t)info.outputSamps * 2) != BK_OK) {
			csh_printf(csh, "music: audio write failed\r\n");
			ret = BK_FAIL;
			break;
		}
	}

	if (audio_started && !s_stop_req && (ret == BK_OK))
		rtos_delay_milliseconds(MUSIC_DRAIN_MS);

out:
	if (dec)
		MP3FreeDecoder(dec);
	if (rbuf)
		os_free(rbuf);
	if (pcm)
		os_free(pcm);
	return ret;
}

/* ---------------- 播放任务 ---------------- */

static void music_task(void *arg)
{
	chry_shell_t *csh = csh_port_shell();
	FRESULT fr = FR_OK;
	bk_err_t ret = BK_FAIL;

	(void)arg;

	fr = f_open(&s_fil, s_path, FA_OPEN_EXISTING | FA_READ);
	if (fr != FR_OK) {
		csh_printf(csh, "music: open %s failed: %d\r\n", s_path, fr);
		goto out;
	}

	if (s_type == MUSIC_TYPE_WAV)
		ret = play_wav(&s_fil, csh);
	else if (s_type == MUSIC_TYPE_MP3)
		ret = play_mp3(&s_fil, csh);

	f_close(&s_fil);

	if (!s_stop_req) {
		if (ret == BK_OK)
			csh_printf(csh, "music: done\r\n");
		else
			csh_printf(csh, "music: playback failed: %d\r\n", ret);
	}

out:
	bsp_audio_stop();
	s_state = MUSIC_STATE_IDLE;
	s_task = NULL;
	if (s_done_sem)
		rtos_set_semaphore(&s_done_sem);
	rtos_delete_thread(NULL);
}

static int music_start(chry_shell_t *csh, const char *file)
{
	music_type_t type = detect_type(file);

	if (type == MUSIC_TYPE_NONE) {
		csh_printf(csh, "music: only .wav / .mp3 supported\r\n");
		return -1;
	}

	if ((s_state != MUSIC_STATE_IDLE) || (s_task != NULL)) {
		csh_printf(csh, "music: already playing, run 'music stop' first\r\n");
		return -1;
	}

	if (build_path(file, s_path, sizeof(s_path)) != 0) {
		csh_printf(csh, "music: bad file name\r\n");
		return -1;
	}

	if (music_sd_mount(csh) != 0)
		return -1;

	if (s_done_sem == NULL) {
		if (rtos_init_semaphore(&s_done_sem, 1) != kNoErr) {
			csh_printf(csh, "music: create semaphore failed\r\n");
			return -1;
		}
	} else {
		/* 清掉上一次播放结束留下的信号量，避免 stop 立刻“完成” */
		while (rtos_get_semaphore(&s_done_sem, 0) == kNoErr)
			;
	}

	s_type = type;
	s_stop_req = false;
	s_state = MUSIC_STATE_PLAYING;

	if (rtos_create_thread(&s_task, BEKEN_DEFAULT_WORKER_PRIORITY, "music",
	                       (beken_thread_function_t)music_task,
	                       MUSIC_TASK_STACK, NULL) != kNoErr) {
		s_task = NULL;
		s_state = MUSIC_STATE_IDLE;
		csh_printf(csh, "music: create task failed\r\n");
		return -1;
	}

	csh_printf(csh, "music: playing %s\r\n", s_path);
	return 0;
}

static int music_stop(chry_shell_t *csh)
{
	uint32_t waited = 0;

	if ((s_state == MUSIC_STATE_IDLE) && (s_task == NULL)) {
		csh_printf(csh, "music: not playing\r\n");
		return 0;
	}

	/* 如果当前是暂停状态，先恢复，否则写环形缓冲会一直阻塞 */
	bsp_audio_resume();
	s_stop_req = true;

	while ((s_task != NULL) && (waited < MUSIC_STOP_TIMEOUT)) {
		if (rtos_get_semaphore(&s_done_sem, 50) == kNoErr)
			break;
		waited += 50;
	}

	if (s_task != NULL) {
		csh_printf(csh, "music: stop timeout, still running\r\n");
		return -1;
	}

	csh_printf(csh, "music: stopped\r\n");
	return 0;
}

static int music_list(chry_shell_t *csh, const char *arg)
{
	char path[32];
	DIR dir;
	FILINFO fi;
	FRESULT fr;
	int n = 0;

	if (music_sd_mount(csh) != 0)
		return -1;

	if ((arg == NULL) || (arg[0] == '\0'))
		snprintf(path, sizeof(path), "1:/");
	else if (arg[1] == ':')
		snprintf(path, sizeof(path), "%s", arg);
	else
		snprintf(path, sizeof(path), "1:/%s", arg);

	fr = f_opendir(&dir, path);
	if (fr != FR_OK) {
		csh_printf(csh, "music: open dir %s failed: %d\r\n", path, fr);
		return -1;
	}

	csh_printf(csh, "music: %s\r\n", path);
	while (1) {
		fr = f_readdir(&dir, &fi);
		if ((fr != FR_OK) || (fi.fname[0] == 0))
			break;
		if ((fi.fattrib & AM_DIR) || !is_audio_file(fi.fname))
			continue;
		csh_printf(csh, "  %-8u  %s\r\n", (unsigned)fi.fsize, fi.fname);
		n++;
	}
	f_closedir(&dir);

	csh_printf(csh, "  %d audio file(s)\r\n", n);
	return 0;
}

static int music_volume(chry_shell_t *csh, const char *arg)
{
	int vol;

	if ((arg == NULL) || (arg[0] == '\0')) {
		csh_printf(csh, "music: volume %u\r\n", s_volume);
		return 0;
	}

	vol = atoi(arg);
	if (vol < 0)
		vol = 0;
	if (vol > 100)
		vol = 100;

	s_volume = (uint8_t)vol;
	if (bsp_audio_is_ready())
		bsp_audio_set_volume(s_volume);

	csh_printf(csh, "music: volume %u\r\n", s_volume);
	return 0;
}

static void music_help(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  music play <file>     play .wav/.mp3 from SD (drive 1:)\r\n");
	csh_printf(csh, "  music stop            stop playback\r\n");
	csh_printf(csh, "  music pause|resume    pause/resume\r\n");
	csh_printf(csh, "  music vol [0-100]     set/show volume\r\n");
	csh_printf(csh, "  music list [path]     list audio files on SD\r\n");
	csh_printf(csh, "  music status          show state\r\n");
	csh_printf(csh, "examples:\r\n");
	csh_printf(csh, "  music play test.wav\r\n");
	csh_printf(csh, "  music play /music/a.mp3\r\n");
}

int music_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];

	if (csh == NULL)
		return -1;

	if (argc < 2) {
		music_help(csh);
		return 0;
	}

	if (!strcmp(argv[1], "help")) {
		music_help(csh);
		return 0;
	}

	if (!strcmp(argv[1], "play")) {
		if (argc < 3) {
			csh_printf(csh, "usage: music play <file>\r\n");
			return -1;
		}
		return music_start(csh, argv[2]);
	}

	if (!strcmp(argv[1], "stop"))
		return music_stop(csh);

	if (!strcmp(argv[1], "pause")) {
		if (bsp_audio_pause() != BK_OK) {
			csh_printf(csh, "music: not playing\r\n");
			return -1;
		}
		csh_printf(csh, "music: paused\r\n");
		return 0;
	}

	if (!strcmp(argv[1], "resume")) {
		if (bsp_audio_resume() != BK_OK) {
			csh_printf(csh, "music: not playing\r\n");
			return -1;
		}
		csh_printf(csh, "music: resumed\r\n");
		return 0;
	}

	if (!strcmp(argv[1], "vol"))
		return music_volume(csh, (argc > 2) ? argv[2] : NULL);

	if (!strcmp(argv[1], "list"))
		return music_list(csh, (argc > 2) ? argv[2] : NULL);

	if (!strcmp(argv[1], "status")) {
		csh_printf(csh, "music: %s%s%s, volume %u\r\n",
		           (s_state == MUSIC_STATE_PLAYING) ? "playing" : "idle",
		           (s_state == MUSIC_STATE_PLAYING) ? " " : "",
		           (s_state == MUSIC_STATE_PLAYING) ? s_path : "",
		           s_volume);
		return 0;
	}

	csh_printf(csh, "music: unknown subcommand '%s', try 'music help'\r\n", argv[1]);
	return -1;
}
