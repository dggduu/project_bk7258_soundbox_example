// Copyright 2026
// 摇一摇检测实现
//
// 采样由 SDK g_sensor 框架驱动（`gs_sc7a20`，见 gsensor_sc7a20_glue.c）：
//   加速度计 AOI 运动中断 -> INT1(P55) ISR -> 事件计数 -> 本文件回调
//   -> 窗口内数过阈值的次数 -> 达到次数就认为是一次"摇动"。
// 不再轮询读 XYZ：静置时零 I2C/零 CPU，且不会因任务调度延迟漏掉抖动。

#include "app_shake.h"
#include "app_mode.h"
#include "app_state.h"
#include "app_ui.h"

#include <components/bk_gsensor.h>

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#define TAG "app_shake"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define SHAKE_DEV_NAME   "sc7a20"

/* gsensor_sc7a20_glue.c 提供的中断事件回调注册（本工程扩展接口） */
extern void gs_sc7a20_set_motion_cb(void (*cb)(uint32_t events));

static app_shake_cfg_t s_cfg;
static void *s_handle;
static app_shake_det_t s_det;
static volatile bool s_ready;
static volatile bool s_started;
static volatile bool s_enabled = true;
static uint32_t s_enable_at;      /* 上电后这段时间内不响应 */

void app_shake_cfg_default(app_shake_cfg_t *cfg)
{
	if (!cfg)
		return;

	/* 中断模型的"门限"在 SC7A20 里（GLUE_MOTION_THS≈128mg），
	 * 这里只保留次数/窗口/冷却：320ms 内 3 次运动中断 ≈ 10Hz 往复，
	 * 轻拿起只有 1~2 次不会误切歌。 */
	cfg->trig_mg = 128;       /* 仅用于日志，与 GLUE_MOTION_THS 对应 */
	cfg->min_peaks = 3;
	cfg->window_ms = 320;
	cfg->cooldown_ms = 1500;
	cfg->lpf_shift = 0;       /* 中断模型不做重力估计 */
}

void app_shake_det_init(app_shake_det_t *det, const app_shake_cfg_t *cfg)
{
	if (!det)
		return;

	os_memset(det, 0, sizeof(*det));
	if (cfg)
		det->cfg = *cfg;
	else
		app_shake_cfg_default(&det->cfg);
}

/* |a| + |b| + |c|，避免 sqrt/浮点 */
static int32_t l1(int32_t a, int32_t b, int32_t c)
{
	if (a < 0)
		a = -a;
	if (b < 0)
		b = -b;
	if (c < 0)
		c = -c;
	return a + b + c;
}

bool app_shake_det_feed(app_shake_det_t *det, int16_t x, int16_t y, int16_t z,
                        uint32_t now_ms)
{
	int32_t dx, dy, dz, mag;
	bool ret = false;

	if (!det)
		return false;

	/* 没调用 app_shake_det_init() 时 cfg 全是 0：trig_mg=0 让 mag>0 恒成立、
	 * min_peaks=0 让判定恒成立，于是每个采样都判成“摇动”（上电日志
	 * 每 20ms 一条 "shake -> next" 的根因）。这里做一次硬保护。 */
	if ((det->cfg.trig_mg <= 0) || (det->cfg.min_peaks == 0) ||
	    (det->cfg.window_ms == 0) || (det->cfg.cooldown_ms == 0))
		return false;

	det->triggered = false;

	if (!det->primed) {
		/* 用第一帧直接作为重力初值，省掉几百 ms 的收敛过程 */
		det->gx = x;
		det->gy = y;
		det->gz = z;
		det->primed = true;
		det->win_start_ms = now_ms;
		return false;
	}

	det->gx += ((int32_t)x - det->gx) >> det->cfg.lpf_shift;
	det->gy += ((int32_t)y - det->gy) >> det->cfg.lpf_shift;
	det->gz += ((int32_t)z - det->gz) >> det->cfg.lpf_shift;

	dx = (int32_t)x - det->gx;
	dy = (int32_t)y - det->gy;
	dz = (int32_t)z - det->gz;
	mag = l1(dx, dy, dz);
	det->last_mag = (int16_t)(mag > 32767 ? 32767 : mag);
	if (det->last_mag > det->peak_mag)
		det->peak_mag = det->last_mag;

	/* 冷却期内连统计都不做，避免“冷却一结束立刻又满足条件”连发 */
	if (det->last_trig_ms &&
	    ((uint32_t)(now_ms - det->last_trig_ms) < det->cfg.cooldown_ms)) {
		det->peaks = 0;
		det->win_start_ms = now_ms;
		return false;
	}

	/* 窗口过期就重新开始计数 */
	if ((uint32_t)(now_ms - det->win_start_ms) > det->cfg.window_ms) {
		det->win_start_ms = now_ms;
		det->peaks = 0;
	}

	if (mag > det->cfg.trig_mg) {
		if (det->peaks < 255)
			det->peaks++;
		if (det->peaks == 1)
			det->win_start_ms = now_ms;
	}

	if (det->peaks >= det->cfg.min_peaks) {
		det->peaks = 0;
		det->win_start_ms = now_ms;
		det->last_trig_ms = now_ms;
		det->triggered = true;
		ret = true;
	}

	return ret;
}

/* 中断事件版本：只有"过阈值"的计数，没有 XYZ。
 * 判定逻辑与 feed() 的窗口/冷却部分一致。 */
bool app_shake_det_feed_event(app_shake_det_t *det, uint32_t now_ms)
{
	if (!det)
		return false;

	if ((det->cfg.min_peaks == 0) || (det->cfg.window_ms == 0) ||
	    (det->cfg.cooldown_ms == 0))
		return false;

	det->triggered = false;

	/* 冷却期内不计峰 */
	if (det->last_trig_ms &&
	    ((uint32_t)(now_ms - det->last_trig_ms) < det->cfg.cooldown_ms)) {
		det->peaks = 0;
		det->win_start_ms = now_ms;
		return false;
	}

	if ((uint32_t)(now_ms - det->win_start_ms) > det->cfg.window_ms) {
		det->win_start_ms = now_ms;
		det->peaks = 0;
	}

	if (det->peaks < 255)
		det->peaks++;
	if (det->peaks == 1)
		det->win_start_ms = now_ms;

	if (det->peaks >= det->cfg.min_peaks) {
		det->peaks = 0;
		det->win_start_ms = now_ms;
		det->last_trig_ms = now_ms;
		det->triggered = true;
		return true;
	}

	return false;
}

bool app_shake_is_ready(void)
{
	return s_ready;
}

void app_shake_set_enabled(bool on)
{
	s_enabled = on;
}

bool app_shake_is_enabled(void)
{
	return s_enabled;
}

/* 运动中断事件（在 glue 的事件任务上下文，可安全投队列） */
static void shake_motion_cb(uint32_t events)
{
	uint32_t now = rtos_get_time();

	if (!s_ready || (events == 0))
		return;

	for (uint32_t i = 0; i < events; i++) {
		if (!app_shake_det_feed_event(&s_det, now))
			continue;

		/* 上电/进入 SD 后 3 秒内的搬动不算；开关关闭时只统计不动 */
		if (s_enabled && ((int32_t)(now - s_enable_at) > 0)) {
			LOGW("shake detected, next track\r\n");
			app_mode_shake_next();
		}
	}

	/* 熄屏拿起唤醒：任何一次运动中断都请求亮屏（UI 侧只在熄屏时响应）。
	 * 门限 128mg 已滤掉桌面振动，不会因为放着不动而乱亮。 */
	app_ui_wakeup_request();
}

void app_shake_init(void)
{
	if (s_started)
		return;
	s_started = true;

	/* 通过 SDK g_sensor 框架打开驱动（"sc7a20" 由 gsensor_sc7a20_glue.c 注册） */
	s_handle = bk_gsensor_init(SHAKE_DEV_NAME);
	if (!s_handle) {
		LOGE("gsensor %s not available, shake disabled\r\n", SHAKE_DEV_NAME);
		s_ready = false;
		app_state_set_shake_ready(false);
		return;
	}

	app_shake_cfg_default(&s_cfg);
	app_shake_det_init(&s_det, &s_cfg);

	bk_gsensor_setDateRange(s_handle, GSENSOR_RANGE_2G);
	/* 运动中断事件（不是标准的逐样本数据回调） */
	gs_sc7a20_set_motion_cb(shake_motion_cb);

	if (bk_gsensor_open(s_handle) != 0) {
		LOGE("gsensor open failed, shake disabled\r\n");
		s_ready = false;
		app_state_set_shake_ready(false);
		return;
	}

	/* 上电后 3 秒不响应：这段时间蓝牙在回连、SD 在挂载，搬动不该触发 */
	s_enable_at = rtos_get_time() + 3000;

	s_ready = true;
	s_enabled = app_state_shake_next();
	app_state_set_shake_ready(true);

	LOGI("shake ready (motion int): ths=%dmg peaks=%u window=%ums cooldown=%ums\r\n",
	     s_cfg.trig_mg, s_cfg.min_peaks, (unsigned)s_cfg.window_ms,
	     (unsigned)s_cfg.cooldown_ms);
}
