// Copyright 2026
// ADC 按键接入实现

#include "app_key.h"
#include "app_ui.h"
#include "bsp.h"

#include <components/log.h>

#define TAG "app_key"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)

static void key_cb(bsp_key_id_t id, bsp_key_evt_t evt, void *arg)
{
	uint8_t key = 0;
	uint8_t e;

	(void)arg;

	switch (id) {
	case BSP_KEY_1: key = 1; break;
	case BSP_KEY_2: key = 2; break;
	case BSP_KEY_3: key = 3; break;
	case BSP_KEY_4: key = 4; break;
	default: return;
	}

	switch (evt) {
	case BSP_KEY_EVT_DOWN:   e = 0; break;
	case BSP_KEY_EVT_UP:     e = 1; break;
	case BSP_KEY_EVT_LONG:   e = 2; break;
	case BSP_KEY_EVT_REPEAT: e = 3; break;
	default: return;
	}

	app_ui_post_key(key, e);
}

void app_key_init(void)
{
	bsp_key_cfg_t cfg = {
		.poll_ms = 30,
		.debounce_cnt = 2,
		.long_ms = 900,
		.repeat_ms = 250,
	};

	if (bsp_key_init(&cfg, key_cb, NULL) != BK_OK)
		LOGI("key init failed\r\n");
	else
		LOGI("key ready\r\n");
}
