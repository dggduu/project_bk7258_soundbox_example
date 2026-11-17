// Copyright 2026
// 全局应用状态实现

#include "app_state.h"
#include "app_config.h"
#include "bsp.h"

#include <os/os.h>
#include <os/str.h>
#include <stdarg.h>
#include <stdio.h>

static app_state_t s_state;
static beken_mutex_t s_lock;
static bool s_inited;

void app_state_init(void)
{
	if (s_inited)
		return;

	if (rtos_init_mutex(&s_lock) != kNoErr) {
		s_lock = NULL;
		return;
	}

	os_memset(&s_state, 0, sizeof(s_state));
	s_state.mode = APP_MODE_BT;
	s_state.tab = APP_TAB_PLAY;
	s_state.volume = BOARD_DEFAULT_VOLUME;
	s_state.backlight = BOARD_DEFAULT_BACKLIGHT;
	s_state.shake_next = true;
	s_state.shake_ready = false;
	s_state.play = APP_PLAY_STOPPED;
	s_inited = true;
}

void app_state_lock(void)
{
	if (s_lock)
		rtos_lock_mutex(&s_lock);
}

void app_state_unlock(void)
{
	if (s_lock)
		rtos_unlock_mutex(&s_lock);
}

app_state_t *app_state_raw(void)
{
	return &s_state;
}

void app_state_copy(app_state_t *out)
{
	if (!out)
		return;

	app_state_lock();
	*out = s_state;
	app_state_unlock();
}

app_track_t *app_state_active_track(app_state_t *st)
{
	if (!st)
		return NULL;

	switch (st->mode) {
	case APP_MODE_BT:  return &st->bt;
	case APP_MODE_USB: return &st->usb;
	case APP_MODE_SD:  return &st->sd;
	case APP_MODE_USB_MIC: return &st->usb;   /* 复用 usb 槽位显示"USB 麦克风" */
	default:           return &st->bt;
	}
}

const app_track_t *app_state_active_track_const(const app_state_t *st)
{
	return app_state_active_track((app_state_t *)st);
}

app_mode_t app_state_mode(void)
{
	app_mode_t m;

	app_state_lock();
	m = s_state.mode;
	app_state_unlock();
	return m;
}

void app_state_set_mode(app_mode_t mode)
{
	if (mode >= APP_MODE_MAX)
		return;

	app_state_lock();
	s_state.mode = mode;
	app_state_unlock();
}

app_tab_t app_state_tab(void)
{
	app_tab_t t;

	app_state_lock();
	t = s_state.tab;
	app_state_unlock();
	return t;
}

void app_state_set_tab(app_tab_t tab)
{
	if (tab >= APP_TAB_MAX)
		return;

	app_state_lock();
	s_state.tab = tab;
	app_state_unlock();
}

void app_state_toast(const char *fmt, ...)
{
	static uint32_t s_last_ms;
	va_list ap;
	char buf[sizeof(s_state.toast)];

	if (!fmt)
		return;

	va_start(ap, fmt);
	vsnprintf(buf, sizeof(buf), fmt, ap);
	va_end(ap);

	app_state_lock();
	/* 同一句提示 1s 内不再触发：UI 是“toast_seq 变了就重绘整页”，
	 * 高频重复提示会让单线 SPI（整帧 ~38ms）一直满负荷。 */
	if ((s_state.toast_seq != 0) && !os_strcmp(s_state.toast, buf) &&
	    ((rtos_get_time() - s_last_ms) < 1000)) {
		app_state_unlock();
		return;
	}
	os_strncpy(s_state.toast, buf, sizeof(s_state.toast) - 1);
	s_state.toast[sizeof(s_state.toast) - 1] = '\0';
	s_state.toast_seq++;
	app_state_unlock();

	s_last_ms = rtos_get_time();
}

uint8_t app_state_volume(void)
{
	uint8_t v;

	app_state_lock();
	v = s_state.volume;
	app_state_unlock();
	return v;
}

void app_state_set_volume(uint8_t vol)
{
	if (vol > 100)
		vol = 100;

	app_state_lock();
	s_state.volume = vol;
	app_state_unlock();
	app_config_save_request();
}

/* 电池读数由 app_mode 的后台 poll 周期写入，UI 只读快照。
 * 不做 app_config_save_request()：电量是易变的运行时状态，不该落盘。 */
void app_state_set_battery(uint32_t mv, uint8_t pct, bool valid)
{
	if (pct > 100)
		pct = 100;

	app_state_lock();
	s_state.battery_mv = mv;
	s_state.battery_pct = pct;
	s_state.battery_valid = valid;
	app_state_unlock();
}

bool app_state_shake_next(void)
{
	bool on;

	app_state_lock();
	on = s_state.shake_next;
	app_state_unlock();
	return on;
}

void app_state_set_shake_next(bool on)
{
	app_state_lock();
	s_state.shake_next = on;
	app_state_unlock();
	app_config_save_request();
}

bool app_state_shake_ready(void)
{
	bool ready;

	app_state_lock();
	ready = s_state.shake_ready;
	app_state_unlock();
	return ready;
}

void app_state_set_shake_ready(bool ready)
{
	app_state_lock();
	s_state.shake_ready = ready;
	app_state_unlock();
}

bool app_state_font_sd(void)
{
	bool use_sd;

	app_state_lock();
	use_sd = s_state.font_sd;
	app_state_unlock();
	return use_sd;
}

void app_state_set_font_sd(bool use_sd)
{
	app_state_lock();
	s_state.font_sd = use_sd;
	app_state_unlock();
	app_config_save_request();
}

/* 通话状态：active / ringing 是互斥的两个展示态，这里统一收口，
 * 避免调用方出现"既在通话又来电"的自相矛盾状态。 */
void app_state_set_call(bool active, bool ringing)
{
	app_state_lock();
	/* ringing 优先：来电时即便 AG 同时报了 call present 也要先显示来电页 */
	s_state.call_ringing = ringing;
	s_state.call_active = active && !ringing;
	app_state_unlock();
}

bool app_state_call_active(void)
{
	bool v;

	app_state_lock();
	v = s_state.call_active;
	app_state_unlock();
	return v;
}

bool app_state_call_ringing(void)
{
	bool v;

	app_state_lock();
	v = s_state.call_ringing;
	app_state_unlock();
	return v;
}
