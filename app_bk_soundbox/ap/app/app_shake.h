// Copyright 2026
// 摇一摇检测（加速度计 -> 动作）
//
// 分层：
//   bsp_sc7a20.c    只负责 I2C 寄存器读写，输出 mg 加速度
//   app_shake.c     纯算法（app_shake_det_*）+ 采样任务（app_shake_init）
//   app_mode.c      决定“摇一摇”在当前模式下该做什么（下一首/忽略）
//
// 算法与 RTOS 解耦：app_shake_det_feed() 只吃 (x,y,z,now_ms)，不碰任何
// 队列/线程，方便在 PC 上回放数据做单测；采样率与门限全部在 cfg 里。

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	int16_t  trig_mg;      /* 去重力后合加速度的峰值门限（mg） */
	uint8_t  min_peaks;    /* 判定窗口内需要几次过门限 */
	uint32_t window_ms;    /* 判定窗口长度 */
	uint32_t cooldown_ms;  /* 触发后静默期，防止一次摇动连发 */
	uint16_t lpf_shift;    /* 重力低通系数 = 1/(1<<shift)，50Hz 下 4 ≈ 3Hz */
} app_shake_cfg_t;

typedef struct {
	app_shake_cfg_t cfg;

	/* 重力估计（mg） */
	int32_t gx;
	int32_t gy;
	int32_t gz;
	bool    primed;

	/* 判定窗口 */
	uint8_t  peaks;
	uint32_t win_start_ms;
	uint32_t last_trig_ms;
	bool     triggered;

	/* 统计量，串口调试用 */
	int16_t  last_mag;
	int16_t  peak_mag;
} app_shake_det_t;

/* 默认参数（50Hz 采样、300ms 窗口、3 次过门限、1.5s 静默） */
void app_shake_cfg_default(app_shake_cfg_t *cfg);

void app_shake_det_init(app_shake_det_t *det, const app_shake_cfg_t *cfg);

/**
 * @brief 喂一个样本。返回 true 表示本次样本构成一次“摇一摇”事件。
 * @param now_ms 单调递增毫秒时间（rtos_get_time()）
 */
bool app_shake_det_feed(app_shake_det_t *det, int16_t x, int16_t y, int16_t z,
                        uint32_t now_ms);

/**
 * @brief 喂一个"运动中断"事件（加速度计 AOI 中断，已按阈值判过）。
 *
 * 与 app_shake_det_feed 的区别：不需要 XYZ/重力估计，直接用中断次数
 * 做峰值计数。返回 true 表示本次事件构成一次"摇一摇"。
 */
bool app_shake_det_feed_event(app_shake_det_t *det, uint32_t now_ms);

/* 采样任务：40Hz（关闭时降到 10Hz），检测到摇动后调用 app_mode_shake_next() */
void app_shake_init(void);

/* 开关缓存：检测任务在热路径上只读这个 volatile bool，不再去拿 app_state 的锁 */
void app_shake_set_enabled(bool on);
bool app_shake_is_enabled(void);

/* 加速度计是否探测成功（UI 用来显示“无”） */
bool app_shake_is_ready(void);

#ifdef __cplusplus
}
#endif
