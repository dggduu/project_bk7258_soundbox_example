// Copyright 2026
// 摄像头 BSP（OV3660，DVP 8bit，电源由 P26/P50 控制）

#pragma once

#include <common/bk_err.h>
#include <stdint.h>
#include <stdbool.h>
#include <driver/media_types.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
	uint16_t width;
	uint16_t height;
	uint8_t fps;          /* frame_fps_t */
	uint16_t format;      /* image_format_t: IMAGE_YUV / IMAGE_MJPEG / IMAGE_H264 */
	media_rotate_t rotate;
} bsp_camera_cfg_t;

/**
 * @brief 摄像头 BSP 初始化：给 1V5/2V8 上电，并把 OV3660 注册进 DVP sensor 列表。
 *
 * 不修改 SDK：通过 bk_dvp_set_devices_list() 覆盖默认列表。
 */
bk_err_t bsp_camera_init(void);

/**
 * @brief 断电并注销自定义 sensor 列表。
 */
bk_err_t bsp_camera_deinit(void);

/**
 * @brief 探测 OV3660 是否在位（读 SCCB PID）。
 *
 * 会临时初始化 I2C0，读完释放，不干扰后续 SDK 初始化。
 */
bool bsp_camera_probe(void);

/**
 * @brief 打开摄像头（内部调 bk_dvp_camera_open）。
 */
bk_err_t bsp_camera_open(const bsp_camera_cfg_t *cfg, camera_handle_t *handle);

/**
 * @brief 关闭摄像头。
 */
bk_err_t bsp_camera_close(camera_handle_t handle);

/**
 * @brief 当前使用的 sensor 名称。
 */
const char *bsp_camera_name(void);

/**
 * @brief 默认配置：640x480 / 15fps / YUV。
 */
void bsp_camera_default_cfg(bsp_camera_cfg_t *cfg);

#ifdef __cplusplus
}
#endif
