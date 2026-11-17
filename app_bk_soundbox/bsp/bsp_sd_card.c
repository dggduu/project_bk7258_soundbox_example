// Copyright 2026
// SD 卡 BSP 实现（SDIO 4bit + FATFS）

#include "bsp_sd_card.h"
#include "bsp_board_cfg.h"

#include <driver/gpio.h>
#include <driver/sd_card.h>
#include <driver/sd_card_types.h>
#include <os/os.h>
#include <os/mem.h>
#include <string.h>
#include <components/log.h>
#include "ff.h"

#define TAG "bsp_sd"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

static FATFS s_fs;
static bool s_mounted = false;
static bool s_det_inited = false;

static void det_pin_init(void)
{
	if (s_det_inited)
		return;

	bk_gpio_disable_output(BOARD_SD_DETECT_PIN);
	bk_gpio_enable_input(BOARD_SD_DETECT_PIN);
	bk_gpio_pull_up(BOARD_SD_DETECT_PIN);
	s_det_inited = true;
}

bool bsp_sd_card_is_inserted(void)
{
	uint32_t level;

	det_pin_init();
	level = bk_gpio_get_input(BOARD_SD_DETECT_PIN);

#if BOARD_SD_DETECT_ACTIVE_LOW
	return level == 0;
#else
	return level != 0;
#endif
}

bool bsp_sd_card_is_mounted(void)
{
	return s_mounted;
}

bk_err_t bsp_sd_card_init(void)
{
	FRESULT fr;
	bk_err_t ret;
	DWORD free_clust = 0;
	FATFS *fs_ptr = NULL;

	if (s_mounted)
		return BK_OK;

	if (!bsp_sd_card_is_inserted()) {
		BSP_LOGE("no card inserted (SD_DET=GPIO%d)\r\n", BOARD_SD_DETECT_PIN);
		return BK_ERR_NOT_SUPPORT;
	}

	/* 让 SDIO 驱动完成卡初始化：f_mount/f_getfree 内部会调用 bk_sd_card_init() */
	fr = f_mount(&s_fs, BSP_SD_MOUNT_POINT, 1);
	if (fr == FR_OK)
		fr = f_getfree(BSP_SD_MOUNT_POINT, &free_clust, &fs_ptr);

	if (fr != FR_OK) {
		BSP_LOGE("first mount failed: %d, retry with explicit sd init\r\n", fr);
		f_mount(NULL, BSP_SD_MOUNT_POINT, 0);
		ret = bk_sd_card_init();
		BSP_LOGI("bk_sd_card_init -> %d\r\n", ret);
		rtos_delay_milliseconds(50);
		fr = f_mount(&s_fs, BSP_SD_MOUNT_POINT, 1);
		if (fr == FR_OK)
			fr = f_getfree(BSP_SD_MOUNT_POINT, &free_clust, &fs_ptr);
	}

	if (fr != FR_OK) {
		BSP_LOGE("mount %s failed: %d\r\n", BSP_SD_MOUNT_POINT, fr);
		f_mount(NULL, BSP_SD_MOUNT_POINT, 0);
		return BK_FAIL;
	}

	s_mounted = true;
	BSP_LOGI("SD card mounted on %s\r\n", BSP_SD_MOUNT_POINT);
	return BK_OK;
}

bk_err_t bsp_sd_card_deinit(void)
{
	if (s_mounted) {
		f_mount(NULL, BSP_SD_MOUNT_POINT, 0);
		s_mounted = false;
	}

	bk_sd_card_deinit();
	return BK_OK;
}

bk_err_t bsp_sd_card_get_capacity_mb(uint32_t *mb)
{
	uint32_t sectors;

	if (!mb)
		return BK_ERR_NULL_PARAM;

	if (!s_mounted)
		return BK_ERR_NOT_INIT;

	sectors = bk_sd_card_get_card_size();
	/* 1 sector = 512B -> 1MB = 2048 sectors */
	*mb = sectors / 2048;
	return BK_OK;
}

bk_err_t bsp_sd_card_get_info(uint32_t *version, uint32_t *type, uint32_t *speed_class)
{
	sd_card_info_t info;
	bk_err_t ret;

	if (!s_mounted)
		return BK_ERR_NOT_INIT;

	ret = bk_sd_card_get_card_info(&info);
	if (ret != BK_OK)
		return ret;

	if (version)
		*version = info.card_version;
	if (type)
		*type = info.card_type;
	if (speed_class)
		*speed_class = info.class;

	return BK_OK;
}

bk_err_t bsp_sd_card_self_test(const char *path)
{
	static const char pattern[] = "bsp_sd_card self test\n";
	char buf[sizeof(pattern)] = { 0 };
	FIL fp;
	FRESULT fr;
	UINT bw = 0, br = 0;
	bk_err_t ret = BK_OK;

	if (!path)
		return BK_ERR_NULL_PARAM;

	if (!s_mounted) {
		ret = bsp_sd_card_init();
		if (ret != BK_OK)
			return ret;
	}

	fr = f_open(&fp, path, FA_CREATE_ALWAYS | FA_WRITE);
	if (fr != FR_OK) {
		BSP_LOGE("open(w) %s failed: %d\r\n", path, fr);
		return BK_FAIL;
	}
	fr = f_write(&fp, pattern, sizeof(pattern) - 1, &bw);
	f_close(&fp);
	if (fr != FR_OK || bw != sizeof(pattern) - 1) {
		BSP_LOGE("write %s failed: fr=%d bw=%d\r\n", path, fr, bw);
		return BK_FAIL;
	}

	fr = f_open(&fp, path, FA_READ);
	if (fr != FR_OK) {
		BSP_LOGE("open(r) %s failed: %d\r\n", path, fr);
		return BK_FAIL;
	}
	fr = f_read(&fp, buf, sizeof(pattern) - 1, &br);
	f_close(&fp);
	if (fr != FR_OK || br != sizeof(pattern) - 1) {
		BSP_LOGE("read %s failed: fr=%d br=%d\r\n", path, fr, br);
		return BK_FAIL;
	}

	if (memcmp(buf, pattern, sizeof(pattern) - 1) != 0) {
		BSP_LOGE("data mismatch on %s\r\n", path);
		return BK_FAIL;
	}

	f_unlink(path);
	BSP_LOGI("self test ok\r\n");
	return BK_OK;
}
