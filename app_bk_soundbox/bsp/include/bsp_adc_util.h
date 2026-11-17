// Copyright 2026
// ADC 公共读取封装（内部使用）：串行化多通道访问，避免 acquire/release 冲突。

#pragma once

#include <common/bk_err.h>
#include <driver/adc.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 读取指定 ADC 通道的电压（mV），内部完成 acquire/init/config/start/read/deinit/release。
 *
 * 换算走 SDK 的 bk_adc_data_calculate()（按 3300mV 满量程）。
 * **按键阈值 BOARD_KEYx_MV 就是按这个口径标定的，不要改这里的换算**，
 * 否则四个按键会一次性全部失效。
 *
 * @param chan        通道号
 * @param sample_num  采样次数（取算术平均），>=1
 * @param mv_out      输出平均电压，单位 mV
 */
bk_err_t bsp_adc_read_mv(adc_chan_t chan, uint32_t sample_num, uint32_t *mv_out);

/**
 * @brief 读取指定 ADC 通道的**原始码值**（多次平均）。
 *
 * 什么时候需要 raw：判饱和、排查分压电阻/量程、打诊断日志。
 * **换算电压不要自己按 raw 套固定系数**——用上面的 bsp_adc_read_mv()
 * （芯片出厂标定，按键阈值 BOARD_KEYx_MV 也是按这个口径标定的）。
 * 电池这一路就同时要 raw（诊断）和 mV（换算），用下面的
 * bsp_adc_read_raw_mv() 一次采集拿两个值即可。
 *
 * raw 是 13bit（0..8191）：raw 接近 8191 说明输入进饱和区，读数已封顶。
 *
 * @param chan        通道号
 * @param sample_num  采样次数（取算术平均），>=1
 * @param raw_out     输出平均原始码值（0..8191）
 */
bk_err_t bsp_adc_read_raw(adc_chan_t chan, uint32_t sample_num, uint32_t *raw_out);

/**
 * @brief 一次采集同时输出平均 raw 码值和平均电压（mV）。
 *
 * 两个值来自同一批采样，不会出现"raw 和 mV 对不上"的错位。
 * 电池检测用这个：mV 做换算，raw 做饱和判断和日志。
 *
 * @param chan        通道号
 * @param sample_num  采样次数（取算术平均），>=1
 * @param raw_out     输出平均原始码值（0..8191），可为 NULL
 * @param mv_out      输出平均电压（mV，SDK 标定口径），可为 NULL
 */
bk_err_t bsp_adc_read_raw_mv(adc_chan_t chan, uint32_t sample_num,
                             uint32_t *raw_out, uint32_t *mv_out);

#ifdef __cplusplus
}
#endif
