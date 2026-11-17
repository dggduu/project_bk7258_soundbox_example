// Copyright 2026
// OV3660 DVP sensor 驱动实现

#include "bsp_ov3660.h"
#include "bsp_board_cfg.h"

#include <driver/dvp_camera.h>
#include <driver/dvp_camera_types.h>
#include <driver/media_types.h>
#include "dvp_sensor_devices.h"
#include <os/os.h>
#include <components/log.h>

#define TAG "ov3660"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGD(...) BK_LOGD(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* SCCB 7bit 地址：写 0x3C / 读 0x3D */
#define OV3660_ADDR          BOARD_OV3660_I2C_ADDR
#define OV3660_PIDH_VAL      0x36
#define OV3660_PIDL_VAL      0x60

/* 常用寄存器 */
#define OV3660_REG_SYS_CTRL0     0x3008
#define OV3660_REG_PIDH          0x300A
#define OV3660_REG_PIDL          0x300B
#define OV3660_REG_DRIVE_CAP     0x302C
#define OV3660_REG_CLK_POL       0x4740
#define OV3660_REG_FORMAT_CTRL   0x501F
#define OV3660_REG_FORMAT_CTRL00 0x4300
#define OV3660_REG_ISP_CTRL01    0x5001
#define OV3660_REG_X_ADDR_ST_H   0x3800
#define OV3660_REG_X_ADDR_END_H  0x3804
#define OV3660_REG_X_OUT_SIZE_H  0x3808
#define OV3660_REG_X_TOTAL_H     0x380C
#define OV3660_REG_X_OFFSET_H    0x3810
#define OV3660_REG_VTS_H         0x380E
#define OV3660_REG_PLL_CTRL0     0x303A
#define OV3660_REG_PLL_CTRL1     0x303B
#define OV3660_REG_PLL_CTRL2     0x303C
#define OV3660_REG_PLL_CTRL3     0x303D
#define OV3660_REG_PCLK_RATIO    0x3824
#define OV3660_REG_VFIFO_CTRL0C  0x460C

/* 表里用 0xFFFF 表示"延时 val 毫秒" */
#define OV3660_REG_DELAY         0xFFFF

typedef struct {
	uint16_t reg;
	uint8_t val;
} ov3660_reg_t;

static bool s_read_verify = false;

static int ov3660_write(uint16_t reg, uint8_t val)
{
	return dvp_camera_i2c_write_uint16(OV3660_ADDR, reg, val);
}

static int ov3660_read(uint16_t reg, uint8_t *val)
{
	return dvp_camera_i2c_read_uint16(OV3660_ADDR, reg, val);
}

static void ov3660_write_verify(uint16_t reg, uint8_t val)
{
	if (!s_read_verify)
		return;

	uint8_t v = 0;
	rtos_delay_milliseconds(1);
	if (ov3660_read(reg, &v) == 0 && v != val)
		LOGD("reg 0x%04X write 0x%02X read 0x%02X\r\n", reg, val, v);
}

static int ov3660_write_regs(const ov3660_reg_t *regs, uint32_t num)
{
	for (uint32_t i = 0; i < num; i++) {
		if (regs[i].reg == OV3660_REG_DELAY) {
			rtos_delay_milliseconds(regs[i].val);
			continue;
		}
		ov3660_write(regs[i].reg, regs[i].val);
		ov3660_write_verify(regs[i].reg, regs[i].val);
	}
	return 0;
}

/* 写 16bit 地址寄存器对：H 在前，L 在后 */
static void ov3660_write_addr(uint16_t reg_h, uint16_t vh, uint16_t vl)
{
	ov3660_write(reg_h, (uint8_t)(vh & 0xFF));
	ov3660_write(reg_h + 1, (uint8_t)(vl & 0xFF));
}

/* ---------------- 默认寄存器表（来自 esp32-camera ov3660_settings.h） ---------------- */
static const ov3660_reg_t ov3660_default_regs[] = {
	{OV3660_REG_SYS_CTRL0, 0x82},      /* software reset */
	{OV3660_REG_DELAY, 10},

	{0x3103, 0x13},
	{OV3660_REG_SYS_CTRL0, 0x42},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{OV3660_REG_DRIVE_CAP, 0xc3},
	{OV3660_REG_CLK_POL, 0x21},

	{0x3611, 0x01},
	{0x3612, 0x2d},
	{0x3032, 0x00},
	{0x3614, 0x80},
	{0x3618, 0x00},
	{0x3619, 0x75},
	{0x3622, 0x80},
	{0x3623, 0x00},
	{0x3624, 0x03},
	{0x3630, 0x52},
	{0x3632, 0x07},
	{0x3633, 0xd2},
	{0x3704, 0x80},
	{0x3708, 0x66},
	{0x3709, 0x12},
	{0x370b, 0x12},
	{0x3717, 0x00},
	{0x371b, 0x60},
	{0x371c, 0x00},
	{0x3901, 0x13},

	{0x3600, 0x08},
	{0x3620, 0x43},
	{0x3702, 0x20},
	{0x3739, 0x48},
	{0x3730, 0x20},
	{0x370c, 0x0c},

	{0x3a18, 0x00},
	{0x3a19, 0xf8},

	{0x3000, 0x10},
	{0x3004, 0xef},

	{0x6700, 0x05},
	{0x6701, 0x19},
	{0x6702, 0xfd},
	{0x6703, 0xd1},
	{0x6704, 0xff},
	{0x6705, 0xff},

	{0x3c01, 0x80},
	{0x3c00, 0x04},
	{0x3a08, 0x00}, {0x3a09, 0x62},
	{0x3a0e, 0x08},
	{0x3a0a, 0x00}, {0x3a0b, 0x52},
	{0x3a0d, 0x09},

	{0x3a00, 0x3a},
	{0x3a14, 0x09},
	{0x3a15, 0x30},
	{0x3a02, 0x09},
	{0x3a03, 0x30},

	{0x440e, 0x08},
	{0x4520, 0x0b},
	{0x460b, 0x37},
	{0x4713, 0x02},
	{0x471c, 0xd0},
	{0x5086, 0x00},

	{0x5002, 0x00},
	{0x501f, 0x00},

	{OV3660_REG_SYS_CTRL0, 0x02},

	{0x5180, 0xff},
	{0x5181, 0xf2},
	{0x5182, 0x00},
	{0x5183, 0x14},
	{0x5184, 0x25},
	{0x5185, 0x24},
	{0x5186, 0x16},
	{0x5187, 0x16},
	{0x5188, 0x16},
	{0x5189, 0x68},
	{0x518a, 0x60},
	{0x518b, 0xe0},
	{0x518c, 0xb2},
	{0x518d, 0x42},
	{0x518e, 0x35},
	{0x518f, 0x56},
	{0x5190, 0x56},
	{0x5191, 0xf8},
	{0x5192, 0x04},
	{0x5193, 0x70},
	{0x5194, 0xf0},
	{0x5195, 0xf0},
	{0x5196, 0x03},
	{0x5197, 0x01},
	{0x5198, 0x04},
	{0x5199, 0x12},
	{0x519a, 0x04},
	{0x519b, 0x00},
	{0x519c, 0x06},
	{0x519d, 0x82},
	{0x519e, 0x38},

	{0x5381, 0x1d},
	{0x5382, 0x60},
	{0x5383, 0x03},
	{0x5384, 0x0c},
	{0x5385, 0x78},
	{0x5386, 0x84},
	{0x5387, 0x7d},
	{0x5388, 0x6b},
	{0x5389, 0x12},
	{0x538a, 0x01},
	{0x538b, 0x98},

	{0x5480, 0x01},

	{0x5000, 0xa7},
	{0x5800, 0x0C},
	{0x5801, 0x09},
	{0x5802, 0x0C},
	{0x5803, 0x0C},
	{0x5804, 0x0D},
	{0x5805, 0x17},
	{0x5806, 0x06},
	{0x5807, 0x05},
	{0x5808, 0x04},
	{0x5809, 0x06},
	{0x580a, 0x09},
	{0x580b, 0x0E},
	{0x580c, 0x05},
	{0x580d, 0x01},
	{0x580e, 0x01},
	{0x580f, 0x01},
	{0x5810, 0x05},
	{0x5811, 0x0D},
	{0x5812, 0x05},
	{0x5813, 0x01},
	{0x5814, 0x01},
	{0x5815, 0x01},
	{0x5816, 0x05},
	{0x5817, 0x0D},
	{0x5818, 0x08},
	{0x5819, 0x06},
	{0x581a, 0x05},
	{0x581b, 0x07},
	{0x581c, 0x0B},
	{0x581d, 0x0D},
	{0x581e, 0x12},
	{0x581f, 0x0D},
	{0x5820, 0x0E},
	{0x5821, 0x10},
	{0x5822, 0x10},
	{0x5823, 0x1E},
	{0x5824, 0x53},
	{0x5825, 0x15},
	{0x5826, 0x05},
	{0x5827, 0x14},
	{0x5828, 0x54},
	{0x5829, 0x25},
	{0x582a, 0x33},
	{0x582b, 0x33},
	{0x582c, 0x34},
	{0x582d, 0x16},
	{0x582e, 0x24},
	{0x582f, 0x41},
	{0x5830, 0x50},
	{0x5831, 0x42},
	{0x5832, 0x15},
	{0x5833, 0x25},
	{0x5834, 0x34},
	{0x5835, 0x33},
	{0x5836, 0x24},
	{0x5837, 0x26},
	{0x5838, 0x54},
	{0x5839, 0x25},
	{0x583a, 0x15},
	{0x583b, 0x25},
	{0x583c, 0x53},
	{0x583d, 0xCF},

	{0x3a0f, 0x30},
	{0x3a10, 0x28},
	{0x3a1b, 0x30},
	{0x3a1e, 0x28},
	{0x3a11, 0x60},
	{0x3a1f, 0x14},

	{0x5302, 0x28},
	{0x5303, 0x20},
	{0x5306, 0x1c},
	{0x5307, 0x28},

	{0x4002, 0xc5},
	{0x4003, 0x81},
	{0x4005, 0x12},

	{0x5688, 0x11},
	{0x5689, 0x11},
	{0x568a, 0x11},
	{0x568b, 0x11},
	{0x568c, 0x11},
	{0x568d, 0x11},
	{0x568e, 0x11},
	{0x568f, 0x11},

	{0x5580, 0x06},
	{0x5588, 0x00},
	{0x5583, 0x40},
	{0x5584, 0x2c},

	{OV3660_REG_ISP_CTRL01, 0x83},   /* color matrix + awb + SDE */
};

/* YUV422 (YUYV) 输出 */
static const ov3660_reg_t ov3660_fmt_yuv422[] = {
	{OV3660_REG_FORMAT_CTRL, 0x00},
	{OV3660_REG_FORMAT_CTRL00, 0x30},
};

bool bsp_ov3660_detect(void)
{
	uint8_t pidh = 0, pidl = 0;

	ov3660_read(OV3660_REG_PIDH, &pidh);
	ov3660_read(OV3660_REG_PIDL, &pidl);

	LOGI("detect id: 0x%02X%02X (expect 0x%02X%02X)\r\n",
	     pidh, pidl, OV3660_PIDH_VAL, OV3660_PIDL_VAL);

	return (pidh == OV3660_PIDH_VAL && pidl == OV3660_PIDL_VAL);
}

/* PLL：按 16MHz XCLK 配置，输出 PCLK 8MHz。
 * 参数含义见 esp32-camera set_pll()：
 *   multiplier=8, sys_div=1, pre_div=0(1x), root_2x=false, seld5=2(2x), pclk_manual=true, pclk_div=4
 */
static void ov3660_set_pll_16m(void)
{
	ov3660_write(OV3660_REG_PLL_CTRL0, 0x00);          /* bypass off */
	ov3660_write(OV3660_REG_PLL_CTRL1, 8 & 0x1f);      /* multiplier */
	ov3660_write(OV3660_REG_PLL_CTRL2, 0x10 | (1 & 0x0f)); /* charge pump | sys_div */
	ov3660_write(OV3660_REG_PLL_CTRL3, (0 & 0x3) << 4 | 2); /* pre_div=1x, seld5=2 */
	ov3660_write(OV3660_REG_PCLK_RATIO, 4 & 0x1f);
	ov3660_write(OV3660_REG_VFIFO_CTRL0C, 0x22);       /* pclk manual */
}

int bsp_ov3660_init(void)
{
	LOGI("init\r\n");

	/* 软复位 */
	ov3660_write(OV3660_REG_SYS_CTRL0, 0x82);
	rtos_delay_milliseconds(10);

	ov3660_write_regs(ov3660_default_regs,
	                  sizeof(ov3660_default_regs) / sizeof(ov3660_default_regs[0]));

	/* YUV422 输出 */
	ov3660_write_regs(ov3660_fmt_yuv422,
	                  sizeof(ov3660_fmt_yuv422) / sizeof(ov3660_fmt_yuv422[0]));

	/* PCLK 配置 */
	ov3660_set_pll_16m();

	rtos_delay_milliseconds(BOARD_SENSOR_RESET_MS);
	LOGI("init done\r\n");
	return 0;
}

/* 4:3 全窗口参数（来自 esp32-camera ratio_table） */
#define OV3660_43_START_X   0
#define OV3660_43_START_Y   0
#define OV3660_43_END_X     2079
#define OV3660_43_END_Y     1547
#define OV3660_43_TOTAL_X   2300
#define OV3660_43_TOTAL_Y   1564

static void ov3660_set_window(uint16_t w, uint16_t h, bool scale_en)
{
	/* 开窗 */
	ov3660_write_addr(OV3660_REG_X_ADDR_ST_H, OV3660_43_START_X, OV3660_43_START_Y);
	ov3660_write_addr(OV3660_REG_X_ADDR_END_H, OV3660_43_END_X, OV3660_43_END_Y);
	/* 输出尺寸 */
	ov3660_write_addr(OV3660_REG_X_OUT_SIZE_H, w, h);
	/* 总尺寸 */
	ov3660_write_addr(OV3660_REG_X_TOTAL_H, OV3660_43_TOTAL_X, OV3660_43_TOTAL_Y);
	/* ISP 偏移 */
	ov3660_write_addr(OV3660_REG_X_OFFSET_H, 16, 6);

	/* ISP_CONTROL_01 bit5: scale enable */
	ov3660_write(OV3660_REG_ISP_CTRL01, scale_en ? 0xA3 : 0x83);
}

int bsp_ov3660_set_ppi(media_ppi_t ppi)
{
	uint16_t w = ppi >> 16;
	uint16_t h = ppi & 0xFFFF;
	bool scale = !(w == 2048 && h == 1536);

	LOGI("set_ppi %dx%d\r\n", w, h);
	ov3660_set_window(w, h, scale);
	rtos_delay_milliseconds(5);
	return 0;
}

int bsp_ov3660_set_fps(frame_fps_t fps)
{
	/* 通过 VTS 调整帧率，基准 1564 行 @ 30fps 附近 */
	uint16_t vts = OV3660_43_TOTAL_Y;

	switch (fps) {
	case FPS5:  vts = 3128; break;
	case FPS10: vts = 2346; break;
	case FPS15: vts = 1955; break;
	case FPS20: vts = 1759; break;
	case FPS25: vts = 1642; break;
	case FPS30: vts = 1564; break;
	default:    vts = 1564; break;
	}

	ov3660_write_addr(OV3660_REG_VTS_H, (vts >> 8) & 0xFF, vts & 0xFF);
	LOGD("set_fps %d, vts=%d\r\n", fps, vts);
	return 0;
}

int bsp_ov3660_dump(media_ppi_t ppi)
{
	(void)ppi;
	uint8_t pidh = 0, pidl = 0;

	ov3660_read(OV3660_REG_PIDH, &pidh);
	ov3660_read(OV3660_REG_PIDL, &pidl);
	LOGI("PID=0x%02X%02X\r\n", pidh, pidl);
	return 0;
}

static void ov3660_read_enable(bool enable)
{
	s_read_verify = enable;
}

/* 注册项：id 用 ID_UNKNOW 之外的自定义值不影响 SDK 内部逻辑（SDK 仅按 id 查找表项），
 * 这里沿用 ID_UNKNOW 会被 bk_dvp_detect 误判，故用未占用值 0x3C。 */
const dvp_sensor_config_t bsp_sensor_ov3660 = {
	.name = "ov3660",
	.clk = MCLK_16M,
	.fmt = PIXEL_FMT_YUYV,
	.vsync = SYNC_HIGH_LEVEL,
	.hsync = SYNC_HIGH_LEVEL,
	.def_ppi = PPI_640X480,
	.def_fps = FPS15,
	.fps_cap = FPS5 | FPS10 | FPS15 | FPS20 | FPS25 | FPS30,
	.ppi_cap = PPI_CAP_640X480 | PPI_CAP_1280X720 | PPI_CAP_1600X1200,
	.id = 0x3C,
	.address = OV3660_ADDR,
	.init = bsp_ov3660_init,
	.detect = bsp_ov3660_detect,
	.set_ppi = bsp_ov3660_set_ppi,
	.set_fps = bsp_ov3660_set_fps,
	.power_down = NULL,
	.dump_register = bsp_ov3660_dump,
	.read_register = ov3660_read_enable,
};
