// Copyright 2026
// 板级支持包统一初始化

#include "bsp.h"

#include <components/log.h>

#define TAG "bsp"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)

bk_err_t bsp_init(void)
{
	bk_err_t ret;

	ret = bsp_power_init();
	if (ret != BK_OK) {
		LOGI("power init failed: %d\r\n", ret);
		return ret;
	}

	LOGI("bsp init done\r\n");
	return BK_OK;
}

void bsp_dump_pinmap(void)
{
	LOGI("---- app_bk_pi tri-mode speaker pin map ----\r\n");
	LOGI("LCD   : ST77916 %dx%d round, QSPI%u 4-line: CLK=P2 CSN=P3 IO0..3=P4..P7\r\n",
	     BSP_LCD_H_RES, BSP_LCD_V_RES, BOARD_LCD_QSPI_ID);
	LOGI("LCD   : BL=P23, 3V3_EN=P51(low on), 16MHz only\r\n");
	LOGI("ADCKEY: P28/ADC4 (2.269/1.650/1.055/0 V) -> KEY1..KEY4, no touch\r\n");
	LOGI("BAT   : P22/ADC5, R21 220k / R24 110k = 1:%d divider\r\n",
	     BOARD_BAT_ADC_DIV_NUM);
	LOGI("SD    : CLK=P14 CMD=P15 D0..D3=P16..P19, DET=P40, LDO_EN=P13\r\n");
	LOGI("AUDIO : DAC -> NS4150B, PA_EN=P52(high on)\r\n");
	LOGI("USB   : dedicated DP/DM, UAC speaker (no GPIO)\r\n");
	LOGI("BT    : A2DP sink + AVRCP CT (car lyric via TITLE)\r\n");
}
