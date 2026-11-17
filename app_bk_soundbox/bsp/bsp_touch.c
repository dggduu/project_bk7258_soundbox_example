// Copyright 2026
// CST816S 触摸实现

#include "bsp_touch.h"
#include "bsp_i2c.h"
#include "bsp_board_cfg.h"

#include <driver/gpio.h>
#include <os/os.h>
#include <components/log.h>

#define TAG "bsp_touch"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* 寄存器 */
#define CST816S_REG_GESTURE      0x01
#define CST816S_REG_POINTS       0x02
#define CST816S_REG_CHIP_ID      0xA7
#define CST816S_REG_IRQ_CTL      0xFA
#define CST816S_REG_DIS_AUTOSLEEP 0xFE

#define CST816S_CHIP_ID_VAL      0xB5
#define CST816S_ADDR             BOARD_TP_I2C_ADDR

/* 从 0x02 起一次读 6 字节：points / xh / xl / yh / yl / strength */
#define CST816S_READ_LEN         6

static bool s_ready = false;
static bool s_swap_xy = false;
static bool s_mirror_x = false;
static bool s_mirror_y = false;

static bk_err_t touch_write(uint8_t reg, uint8_t val)
{
	return bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, CST816S_ADDR, reg, val);
}

static bk_err_t touch_read(uint8_t reg, uint8_t *buf, uint32_t len)
{
	return bsp_i2c_read_buf(BSP_I2C_BUS_SYS, CST816S_ADDR, reg, buf, len);
}

bk_err_t bsp_touch_chip_id(uint8_t *id)
{
	if (!id)
		return BK_ERR_NULL_PARAM;

	return touch_read(CST816S_REG_CHIP_ID, id, 1);
}

bool bsp_touch_is_ready(void)
{
	return s_ready;
}

void bsp_touch_set_transform(bool swap_xy, bool mirror_x, bool mirror_y)
{
	s_swap_xy = swap_xy;
	s_mirror_x = mirror_x;
	s_mirror_y = mirror_y;
}

bk_err_t bsp_touch_init(void)
{
	bk_err_t ret;
	uint8_t id = 0;

	if (s_ready)
		return BK_OK;

	ret = bsp_i2c_init(BSP_I2C_BUS_SYS);
	if (ret != BK_OK) {
		BSP_LOGE("i2c init failed: %d\r\n", ret);
		return ret;
	}

	/* 触摸中断脚：输入上拉，中断由应用层注册 */
	bk_gpio_disable_output(BOARD_TP_INT_PIN);
	bk_gpio_enable_input(BOARD_TP_INT_PIN);
	bk_gpio_pull_up(BOARD_TP_INT_PIN);

	ret = bsp_touch_chip_id(&id);
	if (ret != BK_OK) {
		BSP_LOGE("read chip id failed: %d\r\n", ret);
		bsp_i2c_deinit(BSP_I2C_BUS_SYS);
		return ret;
	}

	BSP_LOGI("chip id 0x%02X\r\n", id);
	if (id != CST816S_CHIP_ID_VAL)
		BSP_LOGE("unexpected chip id, expect 0x%02X\r\n", CST816S_CHIP_ID_VAL);

	/* 关闭自动休眠，保证 INT 持续上报 */
	touch_write(CST816S_REG_DIS_AUTOSLEEP, 0x01);
	/* 触摸按下/抬起都触发中断 */
	touch_write(CST816S_REG_IRQ_CTL, 0x01);

	rtos_delay_milliseconds(10);
	s_ready = true;
	BSP_LOGI("CST816S ready (%dx%d)\r\n", BSP_LCD_H_RES, BSP_LCD_V_RES);
	return BK_OK;
}

bk_err_t bsp_touch_read(bsp_touch_point_t *pt)
{
	uint8_t buf[CST816S_READ_LEN] = { 0 };
	bk_err_t ret;
	uint16_t x, y;

	if (!pt)
		return BK_ERR_NULL_PARAM;

	if (!s_ready)
		return BK_ERR_NOT_INIT;

	os_memset(pt, 0, sizeof(*pt));

	ret = touch_read(CST816S_REG_POINTS, buf, sizeof(buf));
	if (ret != BK_OK)
		return ret;

	if ((buf[0] & 0x0F) == 0)
		return BK_OK;   /* 无触摸 */

	x = ((uint16_t)(buf[1] & 0x0F) << 8) | buf[2];
	y = ((uint16_t)(buf[3] & 0x0F) << 8) | buf[4];

	if (s_swap_xy) {
		uint16_t t = x;
		x = y;
		y = t;
	}
	if (s_mirror_x)
		x = (BSP_LCD_H_RES > x) ? (BSP_LCD_H_RES - 1 - x) : 0;
	if (s_mirror_y)
		y = (BSP_LCD_V_RES > y) ? (BSP_LCD_V_RES - 1 - y) : 0;

	pt->x = x;
	pt->y = y;
	pt->strength = buf[5];
	pt->pressed = true;

	/* 手势单独读，失败不影响坐标 */
	touch_read(CST816S_REG_GESTURE, &pt->gesture, 1);

	return BK_OK;
}

bk_err_t bsp_touch_deinit(void)
{
	if (!s_ready)
		return BK_OK;

	s_ready = false;
	bsp_i2c_deinit(BSP_I2C_BUS_SYS);
	return BK_OK;
}
