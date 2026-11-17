// Copyright 2026
// SD 卡 BSP 实现（SDIO 4bit + FATFS）

#include "bsp_sd.h"
#include "bsp_board_cfg.h"

#include <driver/gpio.h>
#include <driver/sd_card.h>
#include <driver/sdio_host.h>
#include <driver/pwr_clk.h>       /* bk_pm_module_vote_ctrl_external_ldo() */
#include "gpio_driver.h"
#include <os/os.h>
#include <components/log.h>

#include "ff.h"

#define TAG "bsp_sd"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

static FATFS s_fs;
static bool s_mounted;
static bool s_det_inited;

static void det_pin_init(void)
{
	if (s_det_inited)
		return;

	gpio_dev_unmap(BOARD_SD_DETECT_PIN);
	bk_gpio_disable_output(BOARD_SD_DETECT_PIN);
	bk_gpio_disable_pull(BOARD_SD_DETECT_PIN);
	bk_gpio_pull_up(BOARD_SD_DETECT_PIN);
	bk_gpio_enable_input(BOARD_SD_DETECT_PIN);
	s_det_inited = true;
}

bool bsp_sd_is_inserted(void)
{
	uint32_t level;

	det_pin_init();
	rtos_delay_milliseconds(2);
	level = bk_gpio_get_input(BOARD_SD_DETECT_PIN);

#if BOARD_SD_DETECT_ACTIVE_LOW
	return level == 0;
#else
	return level != 0;
#endif
}

bool bsp_sd_is_mounted(void)
{
	return s_mounted;
}

bk_err_t bsp_sd_mount(void)
{
	FRESULT fr;
	bk_err_t ret;

	if (s_mounted)
		return BK_OK;

	if (!bsp_sd_is_inserted()) {
		LOGE("no card (SD_DET=P%d)\r\n", (int)BOARD_SD_DETECT_PIN);
		return BK_ERR_NOT_SUPPORT;
	}

	/* 卡座 3V3 LDO：CONFIG_LDO3V3_CTRL_GPIO 必须是 13（见板级说明） */
	bk_pm_module_vote_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO,
	                                    CONFIG_LDO3V3_CTRL_GPIO,
	                                    GPIO_OUTPUT_STATE_HIGH);
	rtos_delay_milliseconds(5);

	ret = bk_sd_card_init();
	if (ret != BK_OK) {
		LOGE("sd init failed: %d\r\n", ret);
		bk_sdio_host_deinit();
		return BK_FAIL;
	}

	fr = f_mount(&s_fs, BSP_SD_MOUNT_POINT, 1);
	if (fr != FR_OK) {
		LOGE("f_mount failed: %d\r\n", fr);
		bk_sd_card_deinit();
		return BK_FAIL;
	}

	s_mounted = true;
	LOGI("sd mounted on %s\r\n", BSP_SD_MOUNT_POINT);
	return BK_OK;
}

bk_err_t bsp_sd_unmount(void)
{
	if (!s_mounted)
		return BK_OK;

	f_mount(NULL, BSP_SD_MOUNT_POINT, 0);
	bk_sd_card_deinit();
	s_mounted = false;
	return BK_OK;
}

bk_err_t bsp_sd_capacity_mb(uint32_t *mb)
{
	uint32_t sectors;

	if (!mb)
		return BK_ERR_NULL_PARAM;
	if (!s_mounted)
		return BK_ERR_NOT_INIT;

	sectors = bk_sd_card_get_card_size();
	*mb = sectors / 2048;
	return BK_OK;
}
