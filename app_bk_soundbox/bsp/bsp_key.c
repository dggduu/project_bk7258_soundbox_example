// Copyright 2026
// ADC 分压键盘实现

#include "bsp_key.h"
#include "bsp_board_cfg.h"
#include "bsp_adc_util.h"

#include <os/os.h>
#include <components/log.h>

#define TAG "bsp_key"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* 判定方式：离哪个档位的标称值最近就是哪个键。
 *
 * 原来用“相邻两档中点”做门限，KEY1 与 KEY2 的中点只有 1959mV，而 KEY1 的
 * 实测电压约 2.1V（标称 2.269V），容差只剩 150mV —— ADC 噪声或按键接触电阻
 * 一变就跨到 KEY2，表现为“KEY1 不灵敏/要用力按”。
 *
 * 最近邻的容差是按相邻间距给的（最小间距 527mV 的一半 ≈ 264mV），
 * 这里取 330mV：既覆盖 KEY1 偏低到 2.1V，也不会把“无键”(≈3300mV) 误判成按键。
 */
#define KEY_MATCH_MV        330
#define KEY_SCAN_SAMPLES    4       /* 单次判定取 4 次平均，压低 ADC 噪声 */

/* 回调里会打 LOG（vsnprintf 有 256B 的栈缓冲），2K 偏紧，这里放到 3K */
#define KEY_TASK_STACK  3072
#define KEY_TASK_PRIO   BEKEN_DEFAULT_WORKER_PRIORITY

static bsp_key_id_t mv_to_key(uint32_t mv)
{
	static const struct {
		uint8_t id;
		uint32_t mv;
	} tbl[] = {
		{ BSP_KEY_1, BOARD_KEY1_MV },
		{ BSP_KEY_2, BOARD_KEY2_MV },
		{ BSP_KEY_3, BOARD_KEY3_MV },
		{ BSP_KEY_4, BOARD_KEY4_MV },
	};
	uint32_t best = 0xFFFFFFFFu;
	bsp_key_id_t id = BSP_KEY_NONE;

	for (size_t i = 0; i < sizeof(tbl) / sizeof(tbl[0]); i++) {
		uint32_t d = (mv > tbl[i].mv) ? (mv - tbl[i].mv) : (tbl[i].mv - mv);

		if (d < best) {
			best = d;
			id = (bsp_key_id_t)tbl[i].id;
		}
	}

	if (best > KEY_MATCH_MV)
		return BSP_KEY_NONE;

	return id;
}

bsp_key_id_t bsp_key_scan(void)
{
	uint32_t mv = BOARD_KEY_ADC_VREF_MV;

	if (bsp_adc_read_mv(BOARD_KEY_ADC_CHAN, KEY_SCAN_SAMPLES, &mv) != BK_OK)
		return BSP_KEY_NONE;

	return mv_to_key(mv);
}

typedef struct {
	bsp_key_cfg_t cfg;
	bsp_key_cb_t cb;
	void *arg;

	beken_thread_t thread;
	bool running;

	bsp_key_id_t stable;      /* 消抖后的键值 */
	bsp_key_id_t candidate;   /* 待确认键值 */
	uint32_t candidate_cnt;

	uint32_t press_ms;        /* 当前按下持续时间 */
	bool long_fired;
	uint32_t repeat_ms_acc;
} bsp_key_ctx_t;

static bsp_key_ctx_t s_key = {
	.cfg = { .poll_ms = 30, .debounce_cnt = 2, .long_ms = 1000, .repeat_ms = 200 },
};

bk_err_t bsp_key_get_mv(uint32_t *mv)
{
	if (!mv)
		return BK_ERR_NULL_PARAM;

	return bsp_adc_read_mv(BOARD_KEY_ADC_CHAN, 4, mv);
}

bsp_key_id_t bsp_key_get_state(void)
{
	return s_key.stable;
}

static void key_emit(bsp_key_id_t id, bsp_key_evt_t evt)
{
	if (s_key.cb)
		s_key.cb(id, evt, s_key.arg);
}

static void key_task(void *arg)
{
	bsp_key_id_t raw;

	while (s_key.running) {
		raw = bsp_key_scan();

		/* 消抖：连续 debounce_cnt 次相同才认可 */
		if (raw == s_key.candidate) {
			if (s_key.candidate_cnt < s_key.cfg.debounce_cnt)
				s_key.candidate_cnt++;
		} else {
			s_key.candidate = raw;
			s_key.candidate_cnt = 1;
		}

		if (s_key.candidate_cnt >= s_key.cfg.debounce_cnt && s_key.candidate != s_key.stable) {
			bsp_key_id_t old = s_key.stable;

			s_key.stable = s_key.candidate;

			if (old != BSP_KEY_NONE)
				key_emit(old, BSP_KEY_EVT_UP);
			if (s_key.stable != BSP_KEY_NONE) {
				s_key.press_ms = 0;
				s_key.long_fired = false;
				s_key.repeat_ms_acc = 0;
				key_emit(s_key.stable, BSP_KEY_EVT_DOWN);
			}
		}

		/* 长按与连发 */
		if (s_key.stable != BSP_KEY_NONE) {
			s_key.press_ms += s_key.cfg.poll_ms;

			if (!s_key.long_fired && s_key.press_ms >= s_key.cfg.long_ms) {
				s_key.long_fired = true;
				s_key.repeat_ms_acc = 0;
				key_emit(s_key.stable, BSP_KEY_EVT_LONG);
			} else if (s_key.long_fired && s_key.cfg.repeat_ms) {
				s_key.repeat_ms_acc += s_key.cfg.poll_ms;
				if (s_key.repeat_ms_acc >= s_key.cfg.repeat_ms) {
					s_key.repeat_ms_acc = 0;
					key_emit(s_key.stable, BSP_KEY_EVT_REPEAT);
				}
			}
		}

		rtos_delay_milliseconds(s_key.cfg.poll_ms);
	}

	rtos_delete_thread(NULL);
}

bk_err_t bsp_key_init(const bsp_key_cfg_t *cfg, bsp_key_cb_t cb, void *arg)
{
	bk_err_t ret;

	if (s_key.running)
		return BK_OK;

	if (cfg)
		s_key.cfg = *cfg;
	if (s_key.cfg.poll_ms == 0)
		s_key.cfg.poll_ms = 30;
	if (s_key.cfg.debounce_cnt == 0)
		s_key.cfg.debounce_cnt = 2;
	if (s_key.cfg.long_ms == 0)
		s_key.cfg.long_ms = 1000;

	s_key.cb = cb;
	s_key.arg = arg;
	s_key.stable = BSP_KEY_NONE;
	s_key.candidate = BSP_KEY_NONE;
	s_key.candidate_cnt = 0;
	s_key.running = true;

	ret = rtos_create_thread(&s_key.thread, KEY_TASK_PRIO, "bsp_key",
	                         (beken_thread_function_t)key_task,
	                         KEY_TASK_STACK, NULL);
	if (ret != BK_OK) {
		s_key.running = false;
		BSP_LOGE("create key task failed: %d\r\n", ret);
		return ret;
	}

	BSP_LOGI("key init done (ADC ch%d, poll %dms)\r\n", BOARD_KEY_ADC_CHAN, s_key.cfg.poll_ms);
	return BK_OK;
}

bk_err_t bsp_key_deinit(void)
{
	if (!s_key.running)
		return BK_OK;

	s_key.running = false;
	/* 等待线程自行退出 */
	rtos_delay_milliseconds(s_key.cfg.poll_ms * 2 + 10);
	s_key.thread = NULL;
	return BK_OK;
}
