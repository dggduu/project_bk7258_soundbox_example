// Copyright 2026
// ADC 分压键盘（4 键共用 P28/ADC4）
//
// 分压网络：R29 100k 上拉到 3V3，按键各自经电阻下拉到 GND
//   KEY1 = 2.26875V (R28 220k)   KEY2 = 1.65V (R30 100k)
//   KEY3 = 1.055V   (R33 47k)    KEY4 = 0V   (直连)
// 无按键时 ADC 读到 3.3V（上拉满量程）。

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	BSP_KEY_NONE = 0,
	BSP_KEY_1,
	BSP_KEY_2,
	BSP_KEY_3,
	BSP_KEY_4,
	BSP_KEY_MAX,
} bsp_key_id_t;

typedef enum {
	BSP_KEY_EVT_DOWN = 0,   /* 按下（已消抖） */
	BSP_KEY_EVT_UP,         /* 抬起 */
	BSP_KEY_EVT_LONG,       /* 长按触发（默认 1000ms，只触发一次） */
	BSP_KEY_EVT_REPEAT,     /* 长按后连发（默认 200ms 一次） */
} bsp_key_evt_t;

typedef void (*bsp_key_cb_t)(bsp_key_id_t id, bsp_key_evt_t evt, void *arg);

typedef struct {
	uint32_t poll_ms;       /* 轮询周期，默认 30ms */
	uint32_t debounce_cnt;  /* 消抖次数，默认 2 */
	uint32_t long_ms;       /* 长按门限，默认 1000ms */
	uint32_t repeat_ms;     /* 连发周期，默认 200ms，0 关闭 */
} bsp_key_cfg_t;

/**
 * @brief 初始化按键扫描（内部创建轮询线程）。回调可为 NULL，此时仅用 bsp_key_get_state 查询。
 */
bk_err_t bsp_key_init(const bsp_key_cfg_t *cfg, bsp_key_cb_t cb, void *arg);

/**
 * @brief 反初始化，停止轮询线程。
 */
bk_err_t bsp_key_deinit(void);

/**
 * @brief 立即采样一次 ADC，返回当前按下的键（不做消抖）。
 */
bsp_key_id_t bsp_key_scan(void);

/**
 * @brief 获取消抖后的当前按键状态。
 */
bsp_key_id_t bsp_key_get_state(void);

/**
 * @brief 读取当前 ADC 原始电压（mV），调试用。
 */
bk_err_t bsp_key_get_mv(uint32_t *mv);

#ifdef __cplusplus
}
#endif
