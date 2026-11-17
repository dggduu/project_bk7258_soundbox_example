// Copyright 2026
// SC7A20 三轴加速度计实现

#include "bsp_sc7a20.h"
#include "bsp_i2c.h"
#include "bsp_board_cfg.h"

#include <os/os.h>
#include <components/log.h>

#define TAG "sc7a20"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* 寄存器（与 LIS2DH12 兼容） */
#define SC7A20_REG_WHO_AM_I   0x0F
#define SC7A20_REG_CTRL1      0x20
#define SC7A20_REG_CTRL2      0x21
#define SC7A20_REG_CTRL3      0x22
#define SC7A20_REG_CTRL4      0x23
#define SC7A20_REG_CTRL5      0x24
#define SC7A20_REG_STATUS     0x27
#define SC7A20_REG_OUT_X_L    0x28
#define SC7A20_REG_INT1_CFG   0x30
#define SC7A20_REG_INT1_SRC   0x31
#define SC7A20_REG_INT1_THS   0x32
#define SC7A20_REG_INT1_DUR   0x33
#define SC7A20_REG_INT2_CFG   0x34
#define SC7A20_REG_CTRL6      0x25

#define SC7A20_WHO_AM_I_VAL   0x11
#define SC7A20_AUTO_INC       0x80

/* CTRL_REG1: ODR[7:4] | LPen[3] | Zen[2] | Yen[1] | Xen[0] */
#define SC7A20_CTRL1_AXES_EN  0x07

/* CTRL_REG4: BDU[7] | BLE[6] | FS[5:4] | HR[3] | ST[2:0] */
#define SC7A20_CTRL4_BDU      0x80
#define SC7A20_CTRL4_HR       0x08

static bool s_inited = false;
static uint8_t s_addr = BOARD_SC7A20_I2C_ADDR;
static bsp_sc7a20_range_t s_range = BSP_SC7A20_RANGE_2G;

static uint8_t range_to_fs(bsp_sc7a20_range_t r)
{
	switch (r) {
	case BSP_SC7A20_RANGE_4G:  return 0x01;
	case BSP_SC7A20_RANGE_8G:  return 0x02;
	case BSP_SC7A20_RANGE_16G: return 0x03;
	case BSP_SC7A20_RANGE_2G:
	default:                   return 0x00;
	}
}

/* 12bit 高分辨率下的 mg/LSB */
static uint16_t range_to_mg_lsb(bsp_sc7a20_range_t r)
{
	switch (r) {
	case BSP_SC7A20_RANGE_4G:  return 2;
	case BSP_SC7A20_RANGE_8G:  return 4;
	case BSP_SC7A20_RANGE_16G: return 12;
	case BSP_SC7A20_RANGE_2G:
	default:                   return 1;
	}
}

bk_err_t bsp_sc7a20_who_am_i(uint8_t *id)
{
	if (!id)
		return BK_ERR_NULL_PARAM;

	return bsp_i2c_read_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_WHO_AM_I, id);
}

uint8_t bsp_sc7a20_get_addr(void)
{
	return s_addr;
}

bk_err_t bsp_sc7a20_set_range(bsp_sc7a20_range_t range)
{
	bk_err_t ret;
	uint8_t reg = SC7A20_CTRL4_BDU | SC7A20_CTRL4_HR | (range_to_fs(range) << 4);

	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL4, reg);
	if (ret == BK_OK)
		s_range = range;

	return ret;
}

bk_err_t bsp_sc7a20_set_odr(bsp_sc7a20_odr_t odr)
{
	uint8_t reg = ((uint8_t)odr << 4) | SC7A20_CTRL1_AXES_EN;

	return bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL1, reg);
}

bk_err_t bsp_sc7a20_init(void)
{
	bk_err_t ret;
	uint8_t id = 0;
	uint8_t addr_list[2] = { BOARD_SC7A20_I2C_ADDR, BOARD_SC7A20_I2C_ADDR_ALT };
	bool found = false;

	ret = bsp_i2c_init(BSP_I2C_BUS_SYS);
	if (ret != BK_OK)
		return ret;

	for (int i = 0; i < 2; i++) {
		s_addr = addr_list[i];
		if (bsp_i2c_read_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_WHO_AM_I, &id) == BK_OK) {
			BSP_LOGI("probe 0x%02X -> WHO_AM_I 0x%02X\r\n", s_addr, id);
			if (id == SC7A20_WHO_AM_I_VAL) {
				found = true;
				break;
			}
		}
	}

	if (!found) {
		BSP_LOGE("SC7A20 not found on I2C1 (tried 0x18/0x19)\r\n");
		bsp_i2c_deinit(BSP_I2C_BUS_SYS);
		return BK_ERR_NOT_SUPPORT;
	}

	/* BDU + 高分辨率 + ±2g */
	ret = bsp_sc7a20_set_range(BSP_SC7A20_RANGE_2G);
	if (ret != BK_OK)
		goto fail;

	/* 100Hz，三轴使能 */
	ret = bsp_sc7a20_set_odr(BSP_SC7A20_ODR_100HZ);
	if (ret != BK_OK)
		goto fail;

	/* CTRL2: 高通滤波关闭 */
	bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL2, 0x00);
	/* CTRL3: INT1 推挽有效高 */
	bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL3, 0x08);
	/* 默认关闭中断 */
	bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT1_CFG, 0x00);
	bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT2_CFG, 0x00);

	rtos_delay_milliseconds(10);
	s_inited = true;
	BSP_LOGI("SC7A20 init ok @0x%02X, +-2g/100Hz/12bit\r\n", s_addr);
	return BK_OK;

fail:
	BSP_LOGE("SC7A20 config failed: %d\r\n", ret);
	bsp_i2c_deinit(BSP_I2C_BUS_SYS);
	return ret;
}

bk_err_t bsp_sc7a20_read_raw(int16_t *x, int16_t *y, int16_t *z)
{
	uint8_t buf[6] = { 0 };
	bk_err_t ret;

	if (!x || !y || !z)
		return BK_ERR_NULL_PARAM;

	if (!s_inited)
		return BK_ERR_NOT_INIT;

	/* 多字节读需置地址最高位开启自增 */
	ret = bsp_i2c_read_buf(BSP_I2C_BUS_SYS, s_addr,
	                       SC7A20_REG_OUT_X_L | SC7A20_AUTO_INC, buf, sizeof(buf));
	if (ret != BK_OK)
		return ret;

	*x = (int16_t)((buf[1] << 8) | buf[0]);
	*y = (int16_t)((buf[3] << 8) | buf[2]);
	*z = (int16_t)((buf[5] << 8) | buf[4]);

	/* 12bit 左对齐，右移 4 位得到有效值 */
	*x >>= 4;
	*y >>= 4;
	*z >>= 4;
	return BK_OK;
}

bk_err_t bsp_sc7a20_read(bsp_sc7a20_accel_t *out)
{
	int16_t x, y, z;
	bk_err_t ret;
	uint16_t lsb;

	if (!out)
		return BK_ERR_NULL_PARAM;

	ret = bsp_sc7a20_read_raw(&x, &y, &z);
	if (ret != BK_OK)
		return ret;

	lsb = range_to_mg_lsb(s_range);
	out->x = (int16_t)(x * lsb);
	out->y = (int16_t)(y * lsb);
	out->z = (int16_t)(z * lsb);
	return BK_OK;
}

bk_err_t bsp_sc7a20_deinit(void)
{
	bk_err_t ret = BK_OK;

	if (s_inited) {
		ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL1, 0x00);
		s_inited = false;
	}

	bsp_i2c_deinit(BSP_I2C_BUS_SYS);
	return ret;
}

/* ---------------- 运动（AOI1）中断 ---------------- */

/* 运动中断：任一轴超过阈值持续 duration 个采样周期就拉高 INT1。
 * 比起 40Hz 轮询读 XYZ，中断方式 CPU/I2C 开销几乎为零，
 * 且不会因为采样任务被调度延迟而漏掉快速抖动。 */
bk_err_t bsp_sc7a20_motion_int_enable(uint8_t ths, uint8_t duration)
{
	bk_err_t ret;

	if (!s_inited)
		return BK_ERR_NOT_INIT;

	/* CTRL2=0x01：AOI 走高通（去掉重力分量，判"运动"而不是"姿态倾角"） */
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL2, 0x01);
	if (ret != BK_OK)
		return ret;

	/* CTRL5=0x08：LIR_INT1 锁存，中断保持到读 INT1_SRC，避免边沿被漏掉 */
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL5, 0x08);
	if (ret != BK_OK)
		return ret;

	/* CTRL6=0x00：INT 引脚高有效 */
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL6, 0x00);
	if (ret != BK_OK)
		return ret;

	/* 阈值 / 持续时间 */
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT1_THS, ths);
	if (ret != BK_OK)
		return ret;
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT1_DUR, duration);
	if (ret != BK_OK)
		return ret;

	/* INT1_CFG=0x2A：AOI=0(OR 组合)、6D=0、XHIE|YHIE|ZHIE 使能（高事件） */
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT1_CFG, 0x2A);
	if (ret != BK_OK)
		return ret;

	/* CTRL3=0x40：IA1 路由到 INT1 引脚 */
	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL3, 0x40);

	BSP_LOGI("motion int on: ths=%u(~%umg) dur=%u\r\n",
	         ths, (unsigned)ths * 16, duration);
	return ret;
}

bk_err_t bsp_sc7a20_motion_int_disable(void)
{
	bk_err_t ret;

	if (!s_inited)
		return BK_ERR_NOT_INIT;

	ret = bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT1_CFG, 0x00);
	bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL3, 0x08);
	bsp_i2c_write_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_CTRL5, 0x00);
	return ret;
}

bk_err_t bsp_sc7a20_int_src_read(uint8_t *src_out)
{
	uint8_t src = 0;
	bk_err_t ret;

	if (!s_inited)
		return BK_ERR_NOT_INIT;

	ret = bsp_i2c_read_reg8(BSP_I2C_BUS_SYS, s_addr, SC7A20_REG_INT1_SRC, &src);
	if (src_out)
		*src_out = src;
	return ret;
}
