// Copyright 2026
// 三模模式管理实现

#include "app_mode.h"
#include "app_audio.h"
#include "app_bt.h"
#include "app_bt_link.h"
#include "app_usb.h"
#include "app_sd.h"
#include "app_shake.h"
#include "app_config.h"
#include "bsp.h"

#include <os/os.h>
#include <os/str.h>
#include <components/log.h>

#define TAG "app_mode"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define APP_TASK_STACK   8192
#define APP_QUEUE_DEPTH  12
#define APP_POLL_MS      200

/* 电池采样：5s 一次、8 次平均（与 app_test_uart1 的 `bat` 默认参数一致）。
 * 分压电阻常接，一次读要过 ADC 的 acquire/map/config/start/stop，
 * 单次采样很快（bsp_key 每 30ms 就采 4 次），不会拖慢主循环。 */
#define APP_BAT_POLL_MS  5000
#define APP_BAT_SAMPLES  8

typedef enum {
	ACMD_MODE = 1,
	ACMD_PLAY_PAUSE,
	ACMD_NEXT,
	ACMD_PREV,
	ACMD_SEEK,
	ACMD_VOL_STEP,
	ACMD_VOL_SET,
	ACMD_PLAY_FILE,
	ACMD_BT_PAIRING,
	ACMD_BACKLIGHT,
	ACMD_SHAKE_NEXT,
	ACMD_SHAKE_TOGGLE,
	/* 只用来打断 s_queue 的 100ms 等待，让主循环立刻跑一轮 poll。
	 * USB 主机开录音流时用：usbd_audio_open() 在中断上下文置位
	 * s_mic_streaming 之后，必须尽快启动 DMIC，否则主机收到的头几包
	 * 全是静音（详见 app_usb.c 里 usbd_audio_open 的注释）。 */
	ACMD_POLL_NOW,
} app_cmd_t;

typedef struct {
	uint8_t cmd;
	uint8_t force;
	int16_t arg;
	char path[APP_PATH_LEN];
} app_msg_t;

static beken_queue_t s_queue;
static beken_thread_t s_task;
static volatile bool s_running;
static app_mode_t s_mode = APP_MODE_BT;

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

static bool mode_is_busy(app_mode_t m)
{
	bool busy = false;

	app_state_lock();
	switch (m) {
	case APP_MODE_BT:
		busy = app_state_raw()->bt_streaming;
		break;
	case APP_MODE_USB:
		busy = app_state_raw()->usb_streaming;
		break;
	case APP_MODE_SD:
		busy = (app_state_raw()->play == APP_PLAY_PLAYING);
		break;
	case APP_MODE_USB_MIC:
		busy = app_state_raw()->usb_mic_streaming;
		break;
	default:
		break;
	}
	app_state_unlock();
	return busy;
}

bool app_mode_is_busy(void)
{
	bool busy;

	app_state_lock();
	busy = (app_state_raw()->play == APP_PLAY_PLAYING);
	app_state_unlock();
	return busy;
}

static bool mode_leave(app_mode_t m)
{
	switch (m) {
	case APP_MODE_BT:
		app_bt_set_active(false);
		/* 等 BT speaker 线程真正关掉 audio_play，避免和下一个模式抢 DAC */
		for (int i = 0; i < 20 && bt_audio_sink_is_audio_opened(); i++)
			rtos_delay_milliseconds(10);
		break;
	case APP_MODE_USB:
		app_usb_set_active(false);
		break;
	case APP_MODE_SD:
		return app_sd_stop();
	case APP_MODE_USB_MIC:
		app_usb_set_mic_mode(false);
		break;
	default:
		break;
	}
	return true;
}

static void mode_enter(app_mode_t m)
{
	switch (m) {
	case APP_MODE_BT:
		app_bt_set_active(true);
		break;
	case APP_MODE_USB:
		app_usb_set_active(true);
		break;
	case APP_MODE_SD:
		/* SD 模式只切界面，具体播放由文件页触发 */
		break;
	case APP_MODE_USB_MIC:
		/* 只采音不出声：不打开 DAC，只允许 UAC 的 IN 端点出数据 */
		app_usb_set_mic_mode(true);
		break;
	default:
		break;
	}
}

static void do_mode_switch(app_mode_t m)
{
	if (m >= APP_MODE_MAX)
		return;

	if (m == s_mode) {
		mode_enter(m);
		return;
	}

	LOGI("mode %s -> %s\r\n", mode_name(s_mode), mode_name(m));

	if (!mode_leave(s_mode)) {
		app_state_toast("停止播放超时，暂时不能切换");
		return;
	}
	app_audio_close();

	s_mode = m;
	app_state_set_mode(m);

	/* 切到 SD 模式时刷新一次文件列表 */
	if (m == APP_MODE_SD)
		app_sd_set_active(true);

	mode_enter(m);

	{
		bool playing;

		app_state_lock();
		playing = (app_state_raw()->play == APP_PLAY_PLAYING);
		app_state_unlock();
		if (playing)
			app_state_toast("切换到%s", mode_name(m));
	}
}

static void do_play_pause(void)
{
	switch (s_mode) {
	case APP_MODE_BT:
		app_bt_play_pause();
		break;
	case APP_MODE_SD:
		app_sd_play_pause();
		break;
	case APP_MODE_USB:
		/* USB 声卡的播放由主机控制，设备侧只提示 */
		app_state_toast("USB 播放由电脑控制");
		break;
	case APP_MODE_USB_MIC:
		app_state_toast("麦克风录音由电脑控制");
		break;
	default:
		break;
	}
}

static void do_next(void)
{
	switch (s_mode) {
	case APP_MODE_BT: app_bt_next(); break;
	case APP_MODE_SD: app_sd_next(); break;
	default: break;
	}
}

static void do_prev(void)
{
	switch (s_mode) {
	case APP_MODE_BT: app_bt_prev(); break;
	case APP_MODE_SD: app_sd_prev(); break;
	default: break;
	}
}

static void do_seek(int delta_ms)
{
	switch (s_mode) {
	case APP_MODE_SD:
		/* 本地文件：bk_audio_player_seek 按秒绝对定位，三种格式都支持 */
		app_sd_seek(delta_ms);
		break;
	case APP_MODE_BT:
		/* 手机侧播放：只能发 AVRCP 快进/快退，跳多少由 App 决定 */
		app_bt_seek(delta_ms);
		break;
	case APP_MODE_USB:
		/* UAC 是纯 PCM 流，设备侧没有播放列表也没有进度概念 */
		app_state_toast("USB 进度由电脑控制");
		break;
	default:
		break;
	}
}

static void do_volume(int delta)
{
	int vol = app_state_volume() + delta;

	if (vol < 0)
		vol = 0;
	if (vol > 100)
		vol = 100;

	app_audio_set_volume((uint8_t)vol);
	/* 蓝牙下把新音量同步给手机/电脑，避免两端各自维护一个音量 */
	if (s_mode == APP_MODE_BT)
		app_bt_report_volume((uint8_t)vol);
}

/* 电池电压/电量：读分压点 -> 还原 VBAT -> 线性映射百分比，结果写进
 * app_state 供状态栏显示。读失败（没接电池/分压异常）也要写一次
 * invalid，UI 才会从上一帧的百分比切到 "--%"。
 *
 * 放在 app_task 而不是 UI 线程：ADC 访问有 mutex，和按键扫描串行化，
 * 不能阻塞 LVGL 的 loop hook。换算/标定细节见 bsp_battery.c。
 *
 * 为什么还要再做一次 EMA（4 次 ≈ 20s 窗口）：
 *   bsp_battery 里的 8 次平均只压掉 ADC 噪声，压不掉"功放拉电流导致
 *   的电压跌落"—— 放歌时 VBAT 瞬间能掉 100mV 以上，换算成百分比接近
 *   10%，状态栏会随着鼓点跳变。20s 的滑动平均对电量变化几乎无感
 *   （满电到空电要几小时），但能把这些瞬时跌落抹平。 */
static void battery_poll(void)
{
	static uint32_t last_ms;
	static uint32_t mv_ema;
	uint32_t now = rtos_get_time();
	uint32_t mv = 0;
	uint8_t pct;

	if (last_ms && ((now - last_ms) < APP_BAT_POLL_MS))
		return;
	last_ms = now;

	if (bsp_battery_get_mv_avg(APP_BAT_SAMPLES, &mv) != BK_OK) {
		mv_ema = 0;               /* 下次有效读数重新起算，别混用旧值 */
		app_state_set_battery(0, 0, false);
		return;
	}

	mv_ema = mv_ema ? ((mv_ema * 3 + mv) / 4) : mv;

	pct = bsp_battery_mv_to_percent(mv_ema);
	app_state_set_battery(mv_ema, pct, true);
}

static void handle_msg(app_msg_t *msg)
{
	switch (msg->cmd) {
	case ACMD_MODE:
		do_mode_switch((app_mode_t)msg->arg);
		break;
	case ACMD_PLAY_PAUSE:
		do_play_pause();
		break;
	case ACMD_NEXT:
		do_next();
		break;
	case ACMD_PREV:
		do_prev();
		break;
	case ACMD_SEEK:
		do_seek(msg->arg);
		break;
	case ACMD_VOL_STEP:
		do_volume(msg->arg);
		break;
	case ACMD_VOL_SET:
		app_audio_set_volume((uint8_t)msg->arg);
		if (s_mode == APP_MODE_BT)
			app_bt_report_volume((uint8_t)msg->arg);
		break;
	case ACMD_PLAY_FILE:
		if (s_mode != APP_MODE_SD)
			do_mode_switch(APP_MODE_SD);
		app_sd_play(msg->path);
		break;
	case ACMD_BT_PAIRING:
		if (s_mode != APP_MODE_BT)
			do_mode_switch(APP_MODE_BT);
		app_bt_enter_pairing();
		break;
	case ACMD_BACKLIGHT:
	{
		uint8_t val;

		app_state_lock();
		val = app_state_raw()->backlight ? 0 : 100;
		app_state_raw()->backlight = val;
		app_state_unlock();
		bsp_screen_backlight_set(val);
		break;
	}
	case ACMD_SHAKE_NEXT:
		/* HFP 已停用，摇一摇不再需要让通话优先消费。 */

		/* USB 声卡没有“下一首”概念，主机的播放列表不受设备控制。
		 * 这里故意不发 toast：摇一摇是高频手势，误触发时刷 UI 会让
		 * 屏幕一直全帧重绘（单线 SPI 一帧 ~38ms）。 */
		if (!app_state_shake_next() || (s_mode == APP_MODE_USB))
			break;
		do_next();
		break;
	case ACMD_SHAKE_TOGGLE:
	{
		bool on;

		app_state_lock();
		on = !app_state_raw()->shake_next;
		app_state_raw()->shake_next = on;
		app_state_unlock();
		app_shake_set_enabled(on);
		app_state_toast(on ? "摇一摇切歌：开" : "摇一摇切歌：关");
		break;
	}
	case ACMD_POLL_NOW:
		/* USB 主机刚开录音流：立刻跑一轮 USB poll 把 DMIC 拉起来，
		 * 别等 APP_POLL_MS。当前就在 app_task 上下文，直接调用安全。 */
		app_usb_poll();
		break;
	default:
		break;
	}
}

static void app_task(void *arg)
{
	app_msg_t msg;
	uint32_t last_poll = 0;

	(void)arg;

	while (s_running) {
		if (rtos_pop_from_queue(&s_queue, &msg, 100) == kNoErr)
			handle_msg(&msg);

		if ((rtos_get_time() - last_poll) >= APP_POLL_MS) {
			last_poll = rtos_get_time();
			app_bt_poll();
			app_usb_poll();
			app_sd_poll();
			app_config_poll();
			battery_poll();
		}
	}

	s_task = NULL;
	rtos_delete_thread(NULL);
}

static void post_msg(const app_msg_t *msg)
{
	if (!s_queue)
		return;
	rtos_push_to_queue(&s_queue, (void *)msg, 0);
}

static void post_simple(app_cmd_t cmd, int arg)
{
	app_msg_t msg;

	os_memset(&msg, 0, sizeof(msg));
	msg.cmd = (uint8_t)cmd;
	msg.arg = (int16_t)arg;
	post_msg(&msg);
}

void app_mode_init(void)
{
	app_state_init();
	/* 恢复 easyflash 里保存的用户配置（音量/摇一摇/字体来源） */
	app_config_load();

	if (rtos_init_queue(&s_queue, "app_mode_q", sizeof(app_msg_t),
	                    APP_QUEUE_DEPTH) != kNoErr) {
		LOGE("queue init failed\r\n");
		s_queue = NULL;
		return;
	}

	app_usb_start();
	app_sd_init();
	app_bt_init();

	/* 加速度计（摇一摇切歌）：失败不影响其它功能，只在设置页显示“无” */
	app_shake_init();

	/* 电池（P22/ADC5）：这里只打一行配置日志，实际采样在 app_task 里 5s 一次 */
	bsp_battery_init();

	s_running = true;
	if (rtos_create_thread(&s_task, BEKEN_DEFAULT_WORKER_PRIORITY, "app_mode",
	                       (beken_thread_function_t)app_task,
	                       APP_TASK_STACK, NULL) != kNoErr) {
		LOGE("create app task failed\r\n");
		s_running = false;
	}
}

void app_mode_start(void)
{
	app_mode_switch_manual(APP_MODE_BT);
}

void app_mode_request(app_mode_t mode)
{
	if (mode >= APP_MODE_MAX)
		return;
	if (mode == s_mode)
		return;

	/* 正在播放的模式不被自动抢占 */
	if (mode_is_busy(s_mode))
		return;

	/* USB 主机正在推流时，不要让手机连上来把 USB 抢走 */
	if ((mode == APP_MODE_BT) && mode_is_busy(APP_MODE_USB))
		return;

	/* USB 麦克风是用户手动选定的"只采音"模式：
	 * UAC 是全双工设备，电脑很可能同时打开扬声器流，此时会触发
	 * app_mode_request(APP_MODE_USB) —— 不能因此把用户选的麦克风模式抢走。 */
	if ((s_mode == APP_MODE_USB_MIC) && (mode == APP_MODE_USB))
		return;

	post_simple(ACMD_MODE, mode);
}

void app_mode_switch_manual(app_mode_t mode)
{
	post_simple(ACMD_MODE, mode);
}

void app_mode_play_pause(void)
{
	post_simple(ACMD_PLAY_PAUSE, 0);
}

void app_mode_next(void)
{
	post_simple(ACMD_NEXT, 0);
}

void app_mode_prev(void)
{
	post_simple(ACMD_PREV, 0);
}

void app_mode_seek(int delta_ms)
{
	post_simple(ACMD_SEEK, delta_ms);
}

void app_mode_volume_step(int delta)
{
	post_simple(ACMD_VOL_STEP, delta);
}

void app_mode_volume_set(uint8_t volume)
{
	post_simple(ACMD_VOL_SET, volume);
}

void app_mode_play_file(const char *path)
{
	app_msg_t msg;

	if (!path)
		return;

	os_memset(&msg, 0, sizeof(msg));
	msg.cmd = ACMD_PLAY_FILE;
	os_strncpy(msg.path, path, sizeof(msg.path) - 1);
	post_msg(&msg);
}

void app_mode_bt_pairing(void)
{
	post_simple(ACMD_BT_PAIRING, 0);
}

void app_mode_backlight_toggle(void)
{
	post_simple(ACMD_BACKLIGHT, 0);
}

void app_mode_shake_next(void)
{
	post_simple(ACMD_SHAKE_NEXT, 0);
}

void app_mode_shake_toggle(void)
{
	post_simple(ACMD_SHAKE_TOGGLE, 0);
}

/* 打断 app_task 里 s_queue 的 100ms 等待，让主循环立刻跑一轮 poll。
 * 只在 USB 中断上下文调用（不能阻塞、不能拿锁），post_msg 用 0 超时入队。 */
void app_mode_poll_now(void)
{
	post_simple(ACMD_POLL_NOW, 0);
}
