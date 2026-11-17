// Copyright 2026
// 用户配置持久化实现（easyflash V4 blob）

#include "app_config.h"
#include "app_state.h"
#include "bsp.h"

#include "easyflash.h"     /* easy_flash_V4.X/inc */
#include "bk_ef.h"         /* Beken 封装：bk_set_env_enhance / bk_get_env_enhance */

#include <os/os.h>
#include <os/str.h>
#include <components/log.h>

#define TAG "app_config"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define CFG_KEY        "app_cfg"
#define CFG_MAGIC      0xA5
#define CFG_SAVE_QUIET_MS  2000   /* 静默 2s 后落盘，避免音量连按时反复擦写 */

/* 注意：**背光不持久化**。熄屏省电会把 backlight 写成 0，
 * 若存下来下次开机就是黑屏；背光统一按默认 100 开机。 */
typedef struct {
	uint8_t magic;
	uint8_t volume;
	uint8_t reserved;
	uint8_t flags;       /* bit0 = 摇一摇开关, bit1 = 字体用 SD TTF */
} app_cfg_blob_t;

#define FLAG_SHAKE   0x01
#define FLAG_FONT_SD 0x02

static volatile bool s_dirty;
static volatile uint32_t s_dirty_ms;

void app_config_load(void)
{
	app_cfg_blob_t cfg;
	int n;

	if (easyflash_init() != EF_NO_ERR) {
		LOGE("easyflash init failed, use defaults\r\n");
		return;
	}

	os_memset(&cfg, 0, sizeof(cfg));
	n = bk_get_env_enhance(CFG_KEY, &cfg, sizeof(cfg));
	if ((n != (int)sizeof(cfg)) || (cfg.magic != CFG_MAGIC)) {
		LOGI("no saved config (n=%d), use defaults\r\n", n);
		return;
	}

	/* 边界检查：坏数据不应用 */
	if (cfg.volume > 100) {
		LOGW("bad config, ignored\r\n");
		return;
	}

	app_state_lock();
	app_state_raw()->volume = cfg.volume;
	app_state_raw()->shake_next = (cfg.flags & FLAG_SHAKE) != 0;
	app_state_raw()->font_sd = (cfg.flags & FLAG_FONT_SD) != 0;
	app_state_unlock();

	LOGI("loaded: vol=%u shake=%d font_sd=%d\r\n", cfg.volume,
	     (cfg.flags & FLAG_SHAKE) ? 1 : 0,
	     (cfg.flags & FLAG_FONT_SD) ? 1 : 0);
}

void app_config_save_request(void)
{
	s_dirty = true;
	s_dirty_ms = rtos_get_time();
}

void app_config_poll(void)
{
	app_cfg_blob_t cfg;

	if (!s_dirty)
		return;
	if ((rtos_get_time() - s_dirty_ms) < CFG_SAVE_QUIET_MS)
		return;

	s_dirty = false;

	app_state_lock();
	cfg.magic = CFG_MAGIC;
	cfg.volume = app_state_raw()->volume;
	cfg.reserved = 0;
	cfg.flags = (app_state_raw()->shake_next ? FLAG_SHAKE : 0) |
	            (app_state_raw()->font_sd ? FLAG_FONT_SD : 0);
	app_state_unlock();

	if (bk_set_env_enhance(CFG_KEY, &cfg, sizeof(cfg)) != EF_NO_ERR)
		LOGE("save failed\r\n");
	else
		LOGI("saved: vol=%u flags=0x%02x\r\n", cfg.volume, cfg.flags);
}

void app_config_apply_font(void)
{
	bool use_sd = app_state_font_sd();

	if (use_sd && !bsp_font_ttf_available())
		use_sd = false;      /* SD 卡里没有字体，保持内置 */

	bsp_font_set_ttf(use_sd);
	LOGI("font source applied: %s\r\n", bsp_font_source());
}
