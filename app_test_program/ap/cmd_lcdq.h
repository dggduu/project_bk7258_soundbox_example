// Copyright 2026
// lcdq 命令对外接口（供 camera 等其它命令复用同一个 QSPI 面板实例）

#pragma once

#include <common/bk_err.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 把一张 360x360 RGB565（大端字节序，即 {hi,lo} 排列）图送到屏上。
 *
 * 复用 lcdq 已分配的帧缓冲与已初始化的面板，调用前必须先 `lcdq init`。
 * 函数返回时本帧已发送完成（内部 frame_display + wait）。
 *
 * @param src 像素数据，长度 = 360*360*2 = 259200
 * @param len 字节数
 * @return 0 成功，负值失败
 */
int lcdq_display_rgb565(const void *src, uint32_t len);

/**
 * @brief 面板是否已初始化（供调用方提前判断）。
 */
int lcdq_is_ready(void);

/**
 * @brief 取 lcdq 帧缓冲指针（360x360 RGB565 大端，259200B，PSRAM）。
 * 供 camera 等直接作 image_convert 的 dst，省一份 259KB 中间缓冲。
 * 未初始化返回 NULL。
 */
void *lcdq_frame_buf(void);

#ifdef __cplusplus
}
#endif
