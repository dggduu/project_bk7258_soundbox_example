// Copyright 2026
// OV3660 DVP sensor 驱动（放在项目 BSP 内，不修改 SDK）
//
// 参考：
//   1. esp32-camera/sensors/ov3660.c + private_include/ov3660_settings.h
//   2. SDK 自带 dvp_ov2640.c 的 dvp_sensor_config_t 组织方式
//
// 与 SDK 的对接方式：bsp_camera_init() 调 bk_dvp_set_devices_list() 覆盖默认 sensor 列表，
// 之后 SDK 的 bk_dvp_detect()/bk_dvp_init() 就会走这里注册的 OV3660。

#pragma once

#include <driver/dvp_camera_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* OV3660 作为自定义 sensor 注册项 */
extern const dvp_sensor_config_t bsp_sensor_ov3660;

/**
 * @brief 读芯片 ID（0x300A=0x36 / 0x300B=0x60）。
 */
bool bsp_ov3660_detect(void);

/**
 * @brief 软件复位 + 加载默认寄存器 + 配置 YUV422。
 */
int bsp_ov3660_init(void);

/**
 * @brief 设置输出分辨率。
 */
int bsp_ov3660_set_ppi(media_ppi_t ppi);

/**
 * @brief 设置帧率。
 */
int bsp_ov3660_set_fps(frame_fps_t fps);

/**
 * @brief 打印关键寄存器，调试用。
 */
int bsp_ov3660_dump(media_ppi_t ppi);

#ifdef __cplusplus
}
#endif
