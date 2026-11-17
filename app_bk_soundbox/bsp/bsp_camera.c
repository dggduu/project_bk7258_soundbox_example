// Copyright 2026
// 摄像头 BSP 实现

#include "bsp_camera.h"
#include "bsp_ov3660.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"
#include "bsp_i2c.h"

#include <driver/dvp_camera.h>
#include <driver/dvp_camera_types.h>
#include <os/os.h>
#include <components/log.h>
#include "camera.h"

#define TAG "bsp_cam"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

static bool s_cam_inited = false;

/* 自定义 sensor 列表：只有 OV3660 */
static const dvp_sensor_config_t *s_sensor_list[] = {
	&bsp_sensor_ov3660,
};

void bsp_camera_default_cfg(bsp_camera_cfg_t *cfg)
{
	if (!cfg)
		return;

	cfg->width = 640;
	cfg->height = 480;
	cfg->fps = FPS15;
	cfg->format = IMAGE_YUV;
	cfg->rotate = ROTATE_NONE;
}

const char *bsp_camera_name(void)
{
	return bsp_sensor_ov3660.name;
}

bk_err_t bsp_camera_init(void)
{
	bk_err_t ret;

	if (s_cam_inited)
		return BK_OK;

	/* 1. 摄像头供电：1V5 -> 2V8 */
	ret = bsp_power_camera_on();
	if (ret != BK_OK) {
		BSP_LOGE("camera power on failed: %d\r\n", ret);
		return ret;
	}

	/* 2. 用自定义列表覆盖 SDK 默认 sensor 列表（不改 SDK 源码） */
	bk_dvp_set_devices_list(s_sensor_list, sizeof(s_sensor_list) / sizeof(s_sensor_list[0]));

	s_cam_inited = true;
	BSP_LOGI("camera bsp init, sensor=%s\r\n", bsp_sensor_ov3660.name);
	return BK_OK;
}

bk_err_t bsp_camera_deinit(void)
{
	if (!s_cam_inited)
		return BK_OK;

	/* 恢复为 NULL 列表，避免指向已下电的 sensor */
	bk_dvp_set_devices_list(NULL, 0);
	bsp_power_camera_off();
	s_cam_inited = false;
	return BK_OK;
}

bool bsp_camera_probe(void)
{
	bool ok;

	/* dvp_camera_i2c_* 走 CONFIG_DVP_CAMERA_I2C_ID(0)，这里只保证 I2C0 已初始化 */
	if (bsp_i2c_init(BSP_I2C_BUS_CAM) != BK_OK) {
		BSP_LOGE("cam i2c init failed\r\n");
		return false;
	}

	ok = bsp_ov3660_detect();

	bsp_i2c_deinit(BSP_I2C_BUS_CAM);

	BSP_LOGI("probe %s\r\n", ok ? "ok" : "failed");
	return ok;
}

bk_err_t bsp_camera_open(const bsp_camera_cfg_t *cfg, camera_handle_t *handle)
{
	media_camera_device_t dev;
	bsp_camera_cfg_t def;
	bk_err_t ret;

	if (!handle)
		return BK_ERR_NULL_PARAM;

	if (!s_cam_inited) {
		ret = bsp_camera_init();
		if (ret != BK_OK)
			return ret;
	}

	if (cfg)
		def = *cfg;
	else
		bsp_camera_default_cfg(&def);

	os_memset(&dev, 0, sizeof(dev));
	dev.type = DVP_CAMERA;
	dev.port = 0;
	dev.width = def.width;
	dev.height = def.height;
	dev.fps = def.fps;
	dev.format = def.format;
	dev.rotate = def.rotate;

	ret = bk_dvp_camera_open(handle, &dev);
	if (ret != BK_OK) {
		BSP_LOGE("dvp open failed: %d\r\n", ret);
		return ret;
	}

	BSP_LOGI("camera opened %dx%d fps=0x%x fmt=0x%x\r\n",
	         def.width, def.height, def.fps, def.format);
	return BK_OK;
}

bk_err_t bsp_camera_close(camera_handle_t handle)
{
	if (!handle)
		return BK_ERR_NULL_PARAM;

	return bk_dvp_camera_close(handle);
}
