// Copyright 2026
// CherrySH 命令：ov3660（别名 cam）—— OV3660 DVP sensor 寄存器级调试
//
//   ov3660                     状态：电源 / MCLK / 上次读到的 PID
//   ov3660 on|off              摄像头上电 / 下电（1V5 -> 2V8，带稳定延时）
//   ov3660 mclk on|off|<MHz>   GPIO27 输出 MCLK，16MHz 默认，可选 15/16/20/24/30/32/40/48
//   ov3660 probe               上电 + 开 MCLK + 读 PID（最常用的一条）
//   ov3660 id                  只读 PID，不改电源/时钟
//   ov3660 read <reg> [len]    读 16bit 寄存器（reg 十六进制，len 1..16）
//   ov3660 write <reg> <val>   写 16bit 寄存器，并回读校验
//   ov3660 dump [all]          dump 关键寄存器；all 再扫 0x3000..0x30FF
//   ov3660 reset               软复位（0x3008 = 0x82）
//   ov3660 init                复位 + 默认寄存器表 + YUV422 + PLL(16MHz XCLK)
//   ov3660 ppi <w> <h>         设置输出窗口尺寸（十进制）
//   ov3660 fps <5|10|15|20|25|30>  通过 VTS 调帧率
//   ov3660 test [n]            连续读 PID n 次（默认 10），查 SCCB 稳定性
//   ov3660 help                用法
//
// ---------------------------------------------------------------------------
// 实现取舍（重要，改动前先读）
//
// 1) 本工程 **没有** 打开 CONFIG_DVP_CAMERA，ap/components/bk_dvp 下的
//    bk_dvp.c/dvp_common.c 不参与编译，所以不能调 bk_dvp_camera_open()，
//    也不能用依赖 CONFIG_DVP_CAMERA_I2C_ID 的 dvp_camera_i2c_*() 封装。
//    这里直接用 SDK 的 bk_i2c_memory_read/write（I2C_MEM_ADDR_SIZE_16BIT），
//    与 ap/components/bk_peripheral/src/dvp/dvp_sensor_devices.c 里官方
//    SCCB helper 的做法完全一致（16bit 地址，高字节先发）。
//
// 2) OV3660 的 SCCB 需要 XCLK 才能可靠访问寄存器，所以 probe 会先按官方
//    dvp_common.c::dvp_camera_mclk_enable() 的写法开 MCLK：
//      gpio_dev_map(GPIO_27, GPIO_DEV_CLK_AUXS_CIS)
//      sys_drv_set_auxs_cis(3, div) + sys_drv_set_cis_auxs_clk_en(1)
//    用的是 SDK 的 sys_drv 高层 API + gpio_dev_map，不是直接写 LL 寄存器。
//    时钟分频表照抄官方 dvp_common.c。
//
// 3) I2C 总线复用 cmd_i2c.c 的 i2c_bus_init/deinit（见 ap/cmd_i2c.h），
//    共用一份 s_hw_inited[] 状态，避免互相 deinit。bus 0 = I2C_ID_0，
//    SCL=P20 / SDA=P21 恰好是板上摄像头 SCCB。
//    注意：CONFIG_GPIO_DEFAULT_SET_SUPPORT=y 下 SDK 的 i2c_init_gpio()
//    不编译，cmd_i2c.c 的 hw_init() 已补上 P20/P21 -> GPIO_DEV_I2C0_*
//    的显式映射（否则 SCCB 无波形、读回恒 0x00 且驱动不报错）。
//    probe 特殊：它自己管 I2C 生命周期（排在上电 + MCLK 之后），并在
//    命令返回前补写一次电源，用来定位/规避「probe 完 LDO 掉电」。
//
// 4) 寄存器表来自本仓库 app_bk_pi/bsp/bsp_ov3660.c（其来源为
//    esp32-camera/sensors/ov3660_settings.h），不是新编的。
//
// 5) 命令注册在 ap/csh_port.c 的 s_cmd_table[]（显式注册，不走链接段），
//    ov3660 和 cam 两个名字指向同一个处理函数。
// ---------------------------------------------------------------------------

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "csh.h"
#include "csh_port.h"
#include "cmd_i2c.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"
#include "gpio_driver.h"      /* gpio_dev_map() / gpio_dev_unmap() */
#include "sys_driver.h"       /* sys_drv_set_auxs_cis(), sys_drv_set_cis_auxs_clk_en() */
#include <driver/gpio.h>
#include <driver/hal/hal_gpio_types.h>   /* GPIO_DEV_CLK_AUXS_CIS */
#include <driver/i2c.h>
#include <os/os.h>
#include <os/mem.h>

/* 板级配置里摄像头 SCCB 挂在 I2C0，这里是硬编码 I2C_ID_0，改板子时别漏 */
#if BOARD_CAM_I2C_ID != 0
#error "cmd_ov3660.c assumes camera SCCB on I2C0 (BOARD_CAM_I2C_ID == 0)"
#endif

#define OV3660_I2C_BUS      0
#define OV3660_I2C_ID       I2C_ID_0
#define OV3660_ADDR         BOARD_OV3660_I2C_ADDR   /* 7bit 0x3C */
#define OV3660_TIMEOUT_MS   50
#define OV3660_READ_MAX     16

/* 关键寄存器（16bit 地址） */
#define REG_SYS_CTRL0       0x3008
#define REG_PIDH            0x300A
#define REG_PIDL            0x300B
#define REG_DRIVE_CAP       0x302C
#define REG_CLK_POL         0x4740
#define REG_FORMAT_CTRL     0x501F
#define REG_FORMAT_CTRL00   0x4300
#define REG_ISP_CTRL01      0x5001
#define REG_X_ADDR_ST_H     0x3800
#define REG_X_ADDR_END_H    0x3804
#define REG_X_OUT_SIZE_H    0x3808
#define REG_X_TOTAL_H       0x380C
#define REG_VTS_H           0x380E
#define REG_X_OFFSET_H      0x3810
#define REG_PLL_CTRL0       0x303A
#define REG_PLL_CTRL1       0x303B
#define REG_PLL_CTRL2       0x303C
#define REG_PLL_CTRL3       0x303D
#define REG_PCLK_RATIO      0x3824
#define REG_VFIFO_CTRL0C    0x460C

#define PIDH_VAL            0x36
#define PIDL_VAL            0x60
#define REG_DELAY           0xFFFF      /* 表里用 0xFFFF 表示「延时 val 毫秒」 */

/* esp32-camera 的 OV3660 YUV PLL 按 16MHz XCLK 调，camera 命令也用 16MHz，
 * 这里保持一致，免得 probe/init 用 20MHz 时 PCLK 偏 25%。 */
#define MCLK_DEF_MHZ     16
#define PPI_43_TOTAL_X      2300
#define PPI_43_TOTAL_Y      1564

typedef struct {
	uint16_t reg;
	uint8_t  val;
} ov3660_reg_t;

/* 软件状态：MCLK 没有硬件回读，用软件变量；电源直接查 bsp_power */
static bool s_mclk_on;
static int  s_mclk_mhz = MCLK_DEF_MHZ;
static bool s_id_ok;
static uint8_t s_pidh, s_pidl;

/* =====================================================================
 * 一、MCLK（GPIO27）
 *
 * 分频表照抄 SDK ap/components/bk_dvp/src/dvp_common.c::
 * dvp_camera_mclk_enable()，cksel 固定 3。
 * ===================================================================== */

static const struct {
	int      mhz;
	uint32_t div;
} s_mclk_tbl[] = {
	{ 15, 31 }, { 16, 29 }, { 20, 23 }, { 24, 19 },
	{ 30, 15 }, { 32, 14 }, { 40, 11 }, { 48,  9 },
};
#define MCLK_TBL_CNT ((int)(sizeof(s_mclk_tbl) / sizeof(s_mclk_tbl[0])))

static int mclk_div(int mhz)
{
	int i;

	for (i = 0; i < MCLK_TBL_CNT; i++) {
		if (s_mclk_tbl[i].mhz == mhz)
			return (int)s_mclk_tbl[i].div;
	}
	return 0;
}

/* 导出给 camera 命令用：bk_dvp_open 内部的 dvp_camera_mclk_enable 把 P27
 * 复用到 JPEG_MCLK，若该复用不输出 AUXS_CIS 分频时钟则 sensor 无 XCLK
 * （实测 0 帧），这里用 probe 验证过的 CLK_AUXS_CIS 路径恢复 */
int ov_mclk_enable(int mhz)
{
	int div = mclk_div(mhz);

	if (!div)
		return -1;

	/* 官方 dvp_common.c 的顺序：先解复用再映射到 AUXS_CIS，最后开时钟 */
	gpio_dev_unmap(BOARD_CIS_MCLK_PIN);
	gpio_dev_map(BOARD_CIS_MCLK_PIN, GPIO_DEV_CLK_AUXS_CIS);
	sys_drv_set_auxs_cis(3, div);
	sys_drv_set_cis_auxs_clk_en(1);

	s_mclk_on  = true;
	s_mclk_mhz = mhz;
	return 0;
}

static void ov_mclk_disable(void)
{
	sys_drv_set_cis_auxs_clk_en(0);
	s_mclk_on = false;
}

/* =====================================================================
 * 二、SCCB 读写（16bit 寄存器地址）
 * ===================================================================== */

/* SCCB 与标准 I2C 的差别：不支持多字节连续读（没有可靠的地址自增），
 * 每次读都要单独发 16bit 寄存器地址。所以这里按 len 逐个寄存器读，
 * 而不是把 data_size 设成 len 让驱动连读——那样在某些 OV 传感器上
 * 会一直读回同一个寄存器的值。 */
static int ov_read(uint16_t reg, uint8_t *buf, int len)
{
	int i, ret;

	for (i = 0; i < len; i++) {
		i2c_mem_param_t p;

		os_memset(&p, 0, sizeof(p));
		p.dev_addr      = OV3660_ADDR;
		p.mem_addr      = (uint16_t)(reg + i);
		p.mem_addr_size = I2C_MEM_ADDR_SIZE_16BIT;
		p.data          = &buf[i];
		p.data_size     = 1;
		p.timeout_ms    = OV3660_TIMEOUT_MS;

		ret = bk_i2c_memory_read(OV3660_I2C_ID, &p);
		if (ret != BK_OK)
			return ret;
	}
	return BK_OK;
}

static int ov_write(uint16_t reg, uint8_t val)
{
	i2c_mem_param_t p;

	os_memset(&p, 0, sizeof(p));
	p.dev_addr      = OV3660_ADDR;
	p.mem_addr      = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_16BIT;
	p.data          = &val;
	p.data_size     = 1;
	p.timeout_ms    = OV3660_TIMEOUT_MS;

	return bk_i2c_memory_write(OV3660_I2C_ID, &p);
}

static int ov_write_regs(const ov3660_reg_t *regs, int num)
{
	int i;

	for (i = 0; i < num; i++) {
		if (regs[i].reg == REG_DELAY) {
			rtos_delay_milliseconds(regs[i].val);
			continue;
		}
		ov_write(regs[i].reg, regs[i].val);
	}
	return 0;
}

/* 写一个 16bit 寄存器对：reg 存高字节，reg+1 存低字节（大端） */
static void ov_write_reg16(uint16_t reg, uint16_t val)
{
	ov_write(reg,     (uint8_t)((val >> 8) & 0xFF));
	ov_write(reg + 1, (uint8_t)(val & 0xFF));
}

/* 写 X/Y 一对 16bit 值：reg 是 X 的 H 寄存器，Y 的 H 在 reg+2。
 * 旧版把"值的低字节"当 H 字节写，640x480 会变成 0x80E0，窗口全错；
 * bsp_ov3660.c 侧同样修正，见那里的注释。 */
static void ov_write_addr(uint16_t reg, uint16_t x, uint16_t y)
{
	ov_write_reg16(reg, x);
	ov_write_reg16(reg + 2, y);
}

/* =====================================================================
 * 三、默认寄存器表（app_bk_pi/bsp/bsp_ov3660.c，源自 esp32-camera）
 * ===================================================================== */

static const ov3660_reg_t s_default_regs[] = {
	{REG_SYS_CTRL0, 0x82},      /* software reset */
	{REG_DELAY, 10},

	{0x3103, 0x13},
	{REG_SYS_CTRL0, 0x42},
	{0x3017, 0xff},
	{0x3018, 0xff},
	{REG_DRIVE_CAP, 0xc3},
	{REG_CLK_POL, 0x21},

	{0x3611, 0x01}, {0x3612, 0x2d}, {0x3032, 0x00}, {0x3614, 0x80},
	{0x3618, 0x00}, {0x3619, 0x75}, {0x3622, 0x80}, {0x3623, 0x00},
	{0x3624, 0x03}, {0x3630, 0x52}, {0x3632, 0x07}, {0x3633, 0xd2},
	{0x3704, 0x80}, {0x3708, 0x66}, {0x3709, 0x12}, {0x370b, 0x12},
	{0x3717, 0x00}, {0x371b, 0x60}, {0x371c, 0x00}, {0x3901, 0x13},

	{0x3600, 0x08}, {0x3620, 0x43}, {0x3702, 0x20}, {0x3739, 0x48},
	{0x3730, 0x20}, {0x370c, 0x0c},

	{0x3a18, 0x00}, {0x3a19, 0xf8},
	{0x3000, 0x10}, {0x3004, 0xef},

	{0x6700, 0x05}, {0x6701, 0x19}, {0x6702, 0xfd}, {0x6703, 0xd1},
	{0x6704, 0xff}, {0x6705, 0xff},

	{0x3c01, 0x80}, {0x3c00, 0x04},
	{0x3a08, 0x00}, {0x3a09, 0x62}, {0x3a0e, 0x08},
	{0x3a0a, 0x00}, {0x3a0b, 0x52}, {0x3a0d, 0x09},
	{0x3a00, 0x3a}, {0x3a14, 0x09}, {0x3a15, 0x30},
	{0x3a02, 0x09}, {0x3a03, 0x30},

	{0x440e, 0x08}, {0x4520, 0x0b}, {0x460b, 0x37},
	{0x4713, 0x02}, {0x471c, 0xd0}, {0x5086, 0x00},
	{0x5002, 0x00}, {REG_FORMAT_CTRL, 0x00},
	{REG_SYS_CTRL0, 0x02},

	{0x5180, 0xff}, {0x5181, 0xf2}, {0x5182, 0x00}, {0x5183, 0x14},
	{0x5184, 0x25}, {0x5185, 0x24}, {0x5186, 0x16}, {0x5187, 0x16},
	{0x5188, 0x16}, {0x5189, 0x68}, {0x518a, 0x60}, {0x518b, 0xe0},
	{0x518c, 0xb2}, {0x518d, 0x42}, {0x518e, 0x35}, {0x518f, 0x56},
	{0x5190, 0x56}, {0x5191, 0xf8}, {0x5192, 0x04}, {0x5193, 0x70},
	{0x5194, 0xf0}, {0x5195, 0xf0}, {0x5196, 0x03}, {0x5197, 0x01},
	{0x5198, 0x04}, {0x5199, 0x12}, {0x519a, 0x04}, {0x519b, 0x00},
	{0x519c, 0x06}, {0x519d, 0x82}, {0x519e, 0x38},

	{0x5381, 0x1d}, {0x5382, 0x60}, {0x5383, 0x03}, {0x5384, 0x0c},
	{0x5385, 0x78}, {0x5386, 0x84}, {0x5387, 0x7d}, {0x5388, 0x6b},
	{0x5389, 0x12}, {0x538a, 0x01}, {0x538b, 0x98},
	{0x5480, 0x01}, {0x5000, 0xa7},

	{0x5800, 0x0C}, {0x5801, 0x09}, {0x5802, 0x0C}, {0x5803, 0x0C},
	{0x5804, 0x0D}, {0x5805, 0x17}, {0x5806, 0x06}, {0x5807, 0x05},
	{0x5808, 0x04}, {0x5809, 0x06}, {0x580a, 0x09}, {0x580b, 0x0E},
	{0x580c, 0x05}, {0x580d, 0x01}, {0x580e, 0x01}, {0x580f, 0x01},
	{0x5810, 0x05}, {0x5811, 0x0D}, {0x5812, 0x05}, {0x5813, 0x01},
	{0x5814, 0x01}, {0x5815, 0x01}, {0x5816, 0x05}, {0x5817, 0x0D},
	{0x5818, 0x08}, {0x5819, 0x06}, {0x581a, 0x05}, {0x581b, 0x07},
	{0x581c, 0x0B}, {0x581d, 0x0D}, {0x581e, 0x12}, {0x581f, 0x0D},
	{0x5820, 0x0E}, {0x5821, 0x10}, {0x5822, 0x10}, {0x5823, 0x1E},
	{0x5824, 0x53}, {0x5825, 0x15}, {0x5826, 0x05}, {0x5827, 0x14},
	{0x5828, 0x54}, {0x5829, 0x25}, {0x582a, 0x33}, {0x582b, 0x33},
	{0x582c, 0x34}, {0x582d, 0x16}, {0x582e, 0x24}, {0x582f, 0x41},
	{0x5830, 0x50}, {0x5831, 0x42}, {0x5832, 0x15}, {0x5833, 0x25},
	{0x5834, 0x34}, {0x5835, 0x33}, {0x5836, 0x24}, {0x5837, 0x26},
	{0x5838, 0x54}, {0x5839, 0x25}, {0x583a, 0x15}, {0x583b, 0x25},
	{0x583c, 0x53}, {0x583d, 0xCF},

	{0x3a0f, 0x30}, {0x3a10, 0x28}, {0x3a1b, 0x30}, {0x3a1e, 0x28},
	{0x3a11, 0x60}, {0x3a1f, 0x14},
	{0x5302, 0x28}, {0x5303, 0x20}, {0x5306, 0x1c}, {0x5307, 0x28},
	{0x4002, 0xc5}, {0x4003, 0x81}, {0x4005, 0x12},
	{0x5688, 0x11}, {0x5689, 0x11}, {0x568a, 0x11}, {0x568b, 0x11},
	{0x568c, 0x11}, {0x568d, 0x11}, {0x568e, 0x11}, {0x568f, 0x11},
	{0x5580, 0x06}, {0x5588, 0x00}, {0x5583, 0x40}, {0x5584, 0x2c},
	{REG_ISP_CTRL01, 0x83},      /* color matrix + awb + SDE */
};

/* YUV422 (YUYV) 输出 */
static const ov3660_reg_t s_fmt_yuv422[] = {
	{REG_FORMAT_CTRL,   0x00},
	{REG_FORMAT_CTRL00, 0x30},
};

/* PLL：按 16MHz XCLK 配置，输出 PCLK 8MHz。
 * multiplier=8, sys_div=1, pre_div=1x, seld5=2x, pclk_div=4 */
static void ov_set_pll_16m(void)
{
	ov_write(REG_PLL_CTRL0,    0x00);              /* bypass off */
	ov_write(REG_PLL_CTRL1,    8 & 0x1f);          /* multiplier */
	ov_write(REG_PLL_CTRL2,    0x10 | (1 & 0x0f)); /* charge pump | sys_div */
	ov_write(REG_PLL_CTRL3,    (0 & 0x3) << 4 | 2);/* pre_div=1x, seld5=2 */
	ov_write(REG_PCLK_RATIO,   4 & 0x1f);
	ov_write(REG_VFIFO_CTRL0C, 0x22);              /* pclk manual */
}

/* 4:3 全窗口，来自 esp32-camera ratio_table */
static void ov_set_window(uint16_t w, uint16_t h, bool scale_en)
{
	ov_write_addr(REG_X_ADDR_ST_H,  0, 0);
	ov_write_addr(REG_X_ADDR_END_H, 2079, 1547);
	ov_write_addr(REG_X_OUT_SIZE_H, w, h);
	ov_write_addr(REG_X_TOTAL_H,    PPI_43_TOTAL_X, PPI_43_TOTAL_Y);
	ov_write_addr(REG_X_OFFSET_H,   16, 6);
	ov_write(REG_ISP_CTRL01, scale_en ? 0xA3 : 0x83);  /* bit5: scale enable */
}

static const struct {
	int      fps;
	uint16_t vts;
} s_vts_tbl[] = {
	{ 5, 3128 }, { 10, 2346 }, { 15, 1955 },
	{ 20, 1759 }, { 25, 1642 }, { 30, 1564 },
};
#define VTS_CNT ((int)(sizeof(s_vts_tbl) / sizeof(s_vts_tbl[0])))

/* =====================================================================
 * 四、子命令实现
 * ===================================================================== */

static void ov_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  ov3660                     status (power/mclk/pid)\r\n");
	csh_printf(csh, "  ov3660 on|off              camera power 1v5->2v8\r\n");
	csh_printf(csh, "  ov3660 mclk on|off|<MHz>   GPIO27 MCLK, 15/16/20/24/30/32/40/48\r\n");
	csh_printf(csh, "  ov3660 probe               power + mclk + read PID\r\n");
	csh_printf(csh, "  ov3660 id                  read PID only\r\n");
	csh_printf(csh, "  ov3660 read <reg> [len]    read 16bit reg (hex, len 1..16)\r\n");
	csh_printf(csh, "  ov3660 write <reg> <val>   write + read back verify\r\n");
	csh_printf(csh, "  ov3660 dump [all]          key regs / also scan 0x3000..0x30FF\r\n");
	csh_printf(csh, "  ov3660 reset               software reset (0x3008=0x82)\r\n");
	csh_printf(csh, "  ov3660 init                reset + default regs + YUV422 + PLL\r\n");
	csh_printf(csh, "  ov3660 ppi <w> <h>         set output window size\r\n");
	csh_printf(csh, "  ov3660 fps <n>             set fps via VTS (5/10/15/20/25/30)\r\n");
	csh_printf(csh, "  ov3660 test [n]            read PID n times (default 10)\r\n");
	csh_printf(csh, "  ov3660 sccb [hz]           show / set SCCB baud (10k..400k)\r\n");
	csh_printf(csh, "alias: cam\r\n");
	csh_printf(csh, "SCCB: i2c0 0x%02X, SCL=P%d SDA=P%d, 16bit reg addr\r\n",
	           OV3660_ADDR, (int)BOARD_CAM_I2C_SCL_PIN, (int)BOARD_CAM_I2C_SDA_PIN);
}

static int ov_status(chry_shell_t *csh)
{
	csh_printf(csh, "OV3660 (SCCB i2c0 @0x%02X, SCL=P%d SDA=P%d)\r\n",
	           OV3660_ADDR, (int)BOARD_CAM_I2C_SCL_PIN, (int)BOARD_CAM_I2C_SDA_PIN);
	csh_printf(csh, "  power  1v5=P%-2d %-3s pad=%d | 2v8=P%-2d %-3s pad=%d\r\n",
	           (int)BOARD_PWR_1V5_EN_PIN,
	           bsp_power_is_enabled(BSP_PWR_RAIL_CAM_1V5) ? "ON" : "off",
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_1V5),
	           (int)BOARD_PWR_2V8_EN_PIN,
	           bsp_power_is_enabled(BSP_PWR_RAIL_CAM_2V8) ? "ON" : "off",
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_2V8));
	csh_printf(csh, "  mclk   P%-2d %s %dMHz\r\n",
	           (int)BOARD_CIS_MCLK_PIN, s_mclk_on ? "ON" : "off", s_mclk_mhz);
	csh_printf(csh, "  pid    %s", s_id_ok ? "0x" : "not probed yet");
	if (s_id_ok)
		csh_printf(csh, "%02X%02X (OV3660)\r\n", s_pidh, s_pidl);
	else
		csh_printf(csh, "\r\n");
	csh_printf(csh, "  tip    'ov3660 probe' = power on + mclk + read PID\r\n");
	return 0;
}

static int ov_power(chry_shell_t *csh, bool on)
{
	bk_err_t ret = on ? bsp_power_camera_on() : bsp_power_camera_off();

	if (ret != BK_OK) {
		csh_printf(csh, "camera power %s failed: %d\r\n", on ? "on" : "off", ret);
		return -1;
	}
	csh_printf(csh, "camera power %s: 1v5=%s(pad=%d) 2v8=%s(pad=%d)\r\n",
	           on ? "ON" : "OFF",
	           bsp_power_is_enabled(BSP_PWR_RAIL_CAM_1V5) ? "ON" : "off",
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_1V5),
	           bsp_power_is_enabled(BSP_PWR_RAIL_CAM_2V8) ? "ON" : "off",
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_2V8));
	return 0;
}

static int ov_mclk_cmd(chry_shell_t *csh, const char *arg)
{
	int mhz;

	if (!strcmp(arg, "off")) {
		ov_mclk_disable();
		csh_printf(csh, "mclk P%d OFF\r\n", (int)BOARD_CIS_MCLK_PIN);
		return 0;
	}

	if (!strcmp(arg, "on") || !strcmp(arg, "1"))
		mhz = s_mclk_mhz;
	else
		mhz = atoi(arg);

	if (!mclk_div(mhz)) {
		csh_printf(csh, "unsupported MCLK %dMHz, use:", mhz);
		for (int i = 0; i < MCLK_TBL_CNT; i++)
			csh_printf(csh, " %d", s_mclk_tbl[i].mhz);
		csh_printf(csh, "\r\n");
		return -1;
	}

	if (ov_mclk_enable(mhz) != 0) {
		csh_printf(csh, "mclk enable failed\r\n");
		return -1;
	}
	csh_printf(csh, "mclk P%d -> %dMHz (auxs_cis div=%d)\r\n",
	           (int)BOARD_CIS_MCLK_PIN, mhz, mclk_div(mhz));
	return 0;
}

/* 读 PID，成功时更新缓存。返回 0 = ID 正确 */
static int ov_read_pid(chry_shell_t *csh, bool verbose)
{
	uint8_t pidh = 0, pidl = 0;
	int ret;

	ret = ov_read(REG_PIDH, &pidh, 1);
	if (ret == BK_OK)
		ret = ov_read(REG_PIDL, &pidl, 1);

	s_pidh = pidh;
	s_pidl = pidl;
	s_id_ok = (ret == BK_OK && pidh == PIDH_VAL && pidl == PIDL_VAL);

	if (verbose) {
		if (ret != BK_OK) {
			csh_printf(csh, "SCCB read failed: %d ", ret);
			csh_printf(csh, "(power/mclk/wiring? try 'ov3660 probe')\r\n");
			return -1;
		}
		csh_printf(csh, "PID 0x%02X%02X (expect 0x%02X%02X) -> %s\r\n",
		           pidh, pidl, PIDH_VAL, PIDL_VAL,
		           s_id_ok ? "OV3660 OK" : "MISMATCH");
		if (pidh == 0 && pidl == 0)
			csh_printf(csh, "  hint: 0x0000 with no ACK error = no waveform on SCL/SDA "
			                "(pinmux / power / MCLK). try 'i2c scan 0'\r\n");
	}
	return s_id_ok ? 0 : -1;
}

/* 分步打印两路电源 pad，用来定位电源是在哪一步掉的 */
/* SCCB 失败后的定位手段：扫一遍 I2C0 上的 7bit 地址。
 *   有设备（尤其 0x3C）但读 PID 失败 -> 传感器状态/寄存器协议问题
 *   一个都没有                       -> 电源 / MCLK / 排线 / 焊接
 * 探测方式：对每个地址读 OV3660 的 PIDH(0x300A) 一个字节，能完成即认为在线。 */
static void ov_i2c_scan(chry_shell_t *csh)
{
	int found = 0;

	csh_printf(csh, "sccb scan 0x08..0x77 (read reg 0x300A, 20ms each):\r\n");

	for (uint8_t a = 0x08; a <= 0x77; a++) {
		i2c_mem_param_t p;
		uint8_t v = 0;

		os_memset(&p, 0, sizeof(p));
		p.dev_addr      = a;
		p.mem_addr      = 0x300A;
		p.mem_addr_size = I2C_MEM_ADDR_SIZE_16BIT;
		p.data          = &v;
		p.data_size     = 1;
		p.timeout_ms    = 20;

		if (bk_i2c_memory_read(OV3660_I2C_BUS, &p) == BK_OK) {
			csh_printf(csh, "  found 0x%02X (pidh=0x%02X)\r\n", a, v);
			found++;
		}
	}

	if (!found) {
		csh_printf(csh, "  no device answered on i2c0\r\n");
		csh_printf(csh, "  check: 1) 24pin 排线是否插紧 / 焊点是否短路\r\n");
		csh_printf(csh, "         2) 万用表量模组 2V8 / 1V5 实测电压(不是 pad 电平)\r\n");
		csh_printf(csh, "         3) P27 是否有 16MHz 波形(示波器)，无波形则 sensor 不会响应\r\n");
	}
}

static void ov_print_pads(chry_shell_t *csh, const char *tag)
{
	csh_printf(csh, "  pad %-11s 1v5=%d 2v8=%d\r\n", tag,
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_1V5),
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_2V8));
}

static int ov_probe(chry_shell_t *csh)
{
	int ret;

	csh_printf(csh, "probe: power on ...\r\n");
	if (ov_power(csh, true) != 0)
		return -1;

	csh_printf(csh, "probe: mclk %dMHz ...\r\n", MCLK_DEF_MHZ);
	if (ov_mclk_enable(MCLK_DEF_MHZ) != 0) {
		csh_printf(csh, "mclk enable failed\r\n");
		return -1;
	}
	csh_printf(csh, "mclk P%d -> %dMHz\r\n", (int)BOARD_CIS_MCLK_PIN, MCLK_DEF_MHZ);
	ov_print_pads(csh, "after mclk");

	/* 有些模组在 XCLK 起来后还需要几个 ms 才响应 SCCB */
	rtos_delay_milliseconds(BOARD_SENSOR_RESET_MS);

	/* 与旧版不同：I2C 生命周期放在 probe 内部、且排在上电 + MCLK 之后。
	 * 旧版由命令外层先 i2c_bus_init() 再上电，任何 I2C 初始化副作用都
	 * 落在电源刚建立之后；现在顺序严格是 power -> mclk -> i2c -> read。 */
	csh_printf(csh, "probe: sccb i2c0 @0x%02X ...\r\n", OV3660_ADDR);
	ret = i2c_bus_init(OV3660_I2C_BUS);
	if (ret != BK_OK) {
		csh_printf(csh, "i2c0 init failed: %d\r\n", ret);
		return -1;
	}

	ret = ov_read_pid(csh, true);
	if (ret != BK_OK) {
		/* power-cycle 重试：彻底断电（含电容放电）再重新上电。
		 * sensor 若因上电顺序/闩锁进入坏状态，只有真掉电才能救回来。 */
		csh_printf(csh, "probe: power-cycle retry (off %dms -> on) ...\r\n",
		           BOARD_CAM_PWR_OFF_MS);
		bsp_power_camera_off();
		if (ov_power(csh, true) != 0)
			return -1;
		ov_mclk_enable(MCLK_DEF_MHZ);
		/* 读失败后 I2C 控制器可能残留 busy/SDA 卡低状态，重建一次 */
		i2c_bus_force_rebuild(OV3660_I2C_BUS);
		rtos_delay_milliseconds(BOARD_SENSOR_RESET_MS);
		ret = ov_read_pid(csh, true);
	}

	if (ret != BK_OK)
		ov_i2c_scan(csh);
	i2c_bus_deinit(OV3660_I2C_BUS);
	ov_print_pads(csh, "after sccb");

	/* 关键：命令返回前重新写一次两路电源。
	 * 现场反馈「probe 执行完 LDO 没电压，而 ldo 单独控制有电压」，
	 * 这里补写一次并打印最终 pad：既是防御，也把掉电点暴露在日志里。 */
	csh_printf(csh, "probe: re-assert camera power ...\r\n");
	if (ov_power(csh, true) != 0)
		return -1;

	return ret;
}

static int ov_read_cmd(chry_shell_t *csh, const char *sreg, int len)
{
	uint16_t reg = (uint16_t)strtoul(sreg, NULL, 16);
	uint8_t  buf[OV3660_READ_MAX];
	int i, ret;

	if (len < 1)
		len = 1;
	if (len > OV3660_READ_MAX)
		len = OV3660_READ_MAX;

	os_memset(buf, 0, sizeof(buf));
	ret = ov_read(reg, buf, len);
	if (ret != BK_OK) {
		csh_printf(csh, "read 0x%04X failed: %d\r\n", reg, ret);
		return -1;
	}

	csh_printf(csh, "0x%04X:", reg);
	for (i = 0; i < len; i++)
		csh_printf(csh, " %02X", buf[i]);
	csh_printf(csh, "\r\n");
	return 0;
}

static int ov_write_cmd(chry_shell_t *csh, const char *sreg, const char *sval)
{
	uint16_t reg = (uint16_t)strtoul(sreg, NULL, 16);
	uint8_t  val = (uint8_t)strtoul(sval, NULL, 16);
	uint8_t  rb = 0;
	int ret;

	ret = ov_write(reg, val);
	if (ret != BK_OK) {
		csh_printf(csh, "write 0x%04X failed: %d\r\n", reg, ret);
		return -1;
	}

	/* 回读校验：SCCB 写失败常常不报错，只看 ACK 会漏 */
	rtos_delay_milliseconds(1);
	ret = ov_read(reg, &rb, 1);
	if (ret != BK_OK) {
		csh_printf(csh, "write 0x%04X ok, read back failed: %d\r\n", reg, ret);
		return -1;
	}

	csh_printf(csh, "0x%04X <- 0x%02X, read back 0x%02X %s\r\n",
	           reg, val, rb, (rb == val) ? "OK" : "MISMATCH");
	return (rb == val) ? 0 : -1;
}

/* dump 用的关键寄存器表 */
static const struct {
	uint16_t reg;
	const char *name;
} s_key_regs[] = {
	{REG_PIDH, "PIDH"}, {REG_PIDL, "PIDL"}, {REG_SYS_CTRL0, "SYS_CTRL0"},
	{0x3103, "SYS_CLK"}, {REG_DRIVE_CAP, "DRIVE_CAP"}, {REG_CLK_POL, "CLK_POL"},
	{REG_PLL_CTRL0, "PLL0"}, {REG_PLL_CTRL1, "PLL1"},
	{REG_PLL_CTRL2, "PLL2"}, {REG_PLL_CTRL3, "PLL3"},
	{REG_PCLK_RATIO, "PCLK_RATIO"}, {REG_VFIFO_CTRL0C, "VFIFO_CTRL0C"},
	{REG_FORMAT_CTRL, "FORMAT_CTRL"}, {REG_FORMAT_CTRL00, "FORMAT_CTRL00"},
	{REG_ISP_CTRL01, "ISP_CTRL01"}, {0x5000, "ISP_CTRL00"},
	{REG_X_ADDR_ST_H, "X_ADDR_ST_H"}, {REG_X_ADDR_ST_H + 1, "X_ADDR_ST_L"},
	{REG_X_ADDR_END_H, "X_ADDR_END_H"}, {REG_X_ADDR_END_H + 1, "X_ADDR_END_L"},
	{REG_X_OUT_SIZE_H, "X_OUT_SIZE_H"}, {REG_X_OUT_SIZE_H + 1, "X_OUT_SIZE_L"},
	{REG_X_TOTAL_H, "X_TOTAL_H"}, {REG_X_TOTAL_H + 1, "X_TOTAL_L"},
	{REG_VTS_H, "VTS_H"}, {REG_VTS_H + 1, "VTS_L"},
	{0x3a00, "AEC_CTRL00"}, {0x3a08, "AEC_B50_STEP_H"},
	{0x3a09, "AEC_B50_STEP_L"}, {0x3a0f, "AEC_CTRL0F"},
	{0x3a11, "AEC_CTRL11"}, {0x3a1f, "AEC_CTRL1F"},
	{0x3500, "EXPOSURE_H"}, {0x3501, "EXPOSURE_M"}, {0x3502, "EXPOSURE_L"},
	{0x350a, "GAIN_H"}, {0x350b, "GAIN_L"},
};
#define KEY_REG_CNT ((int)(sizeof(s_key_regs) / sizeof(s_key_regs[0])))

static int ov_dump(chry_shell_t *csh, bool all)
{
	int i;

	for (i = 0; i < KEY_REG_CNT; i++) {
		uint8_t v = 0;

		if (ov_read(s_key_regs[i].reg, &v, 1) != BK_OK) {
			csh_printf(csh, "dump aborted at 0x%04X (SCCB error)\r\n",
			           s_key_regs[i].reg);
			return -1;
		}
		csh_printf(csh, "  0x%04X %-16s = 0x%02X\r\n",
		           s_key_regs[i].reg, s_key_regs[i].name, v);
	}

	if (!all)
		return 0;

	csh_printf(csh, "-- 0x3000..0x30FF (one reg at a time; SCCB has no burst read) --\r\n");
	for (uint16_t base = 0x3000; base <= 0x30F0; base += 16) {
		uint8_t b[16];
		int j;

		os_memset(b, 0, sizeof(b));
		if (ov_read(base, b, 16) != BK_OK) {
			csh_printf(csh, "  0x%04X read failed\r\n", base);
			continue;
		}
		csh_printf(csh, "  0x%04X:", base);
		for (j = 0; j < 16; j++)
			csh_printf(csh, " %02X", b[j]);
		csh_printf(csh, "\r\n");
	}
	return 0;
}

static int ov_init(chry_shell_t *csh)
{
	csh_printf(csh, "init: soft reset ...\r\n");
	ov_write(REG_SYS_CTRL0, 0x82);
	rtos_delay_milliseconds(10);

	csh_printf(csh, "init: %d default regs ...\r\n",
	           (int)(sizeof(s_default_regs) / sizeof(s_default_regs[0])));
	ov_write_regs(s_default_regs,
	              (int)(sizeof(s_default_regs) / sizeof(s_default_regs[0])));

	csh_printf(csh, "init: YUV422 + PLL(16MHz XCLK) ...\r\n");
	ov_write_regs(s_fmt_yuv422,
	              (int)(sizeof(s_fmt_yuv422) / sizeof(s_fmt_yuv422[0])));
	ov_set_pll_16m();

	rtos_delay_milliseconds(BOARD_SENSOR_RESET_MS);
	csh_printf(csh, "init done, ");
	return ov_read_pid(csh, true);
}

static int ov_ppi(chry_shell_t *csh, int w, int h)
{
	bool scale = !(w == 2048 && h == 1536);

	if (w < 16 || h < 16 || w > 2048 || h > 1536) {
		csh_printf(csh, "invalid ppi %dx%d (16..2048 / 16..1536)\r\n", w, h);
		return -1;
	}

	ov_set_window((uint16_t)w, (uint16_t)h, scale);
	rtos_delay_milliseconds(5);
	csh_printf(csh, "ppi %dx%d, scale=%s\r\n", w, h, scale ? "on" : "off");
	return 0;
}

static int ov_fps(chry_shell_t *csh, int fps)
{
	int i;

	for (i = 0; i < VTS_CNT; i++) {
		if (s_vts_tbl[i].fps != fps)
			continue;

		ov_write_reg16(REG_VTS_H, s_vts_tbl[i].vts);
		csh_printf(csh, "fps %d -> VTS=%d (0x380E/0x380F)\r\n",
		           fps, s_vts_tbl[i].vts);
		return 0;
	}

	csh_printf(csh, "unsupported fps %d, use 5/10/15/20/25/30\r\n", fps);
	return -1;
}

static int ov_test(chry_shell_t *csh, int n)
{
	int i, ok = 0;

	for (i = 0; i < n; i++) {
		uint8_t pidh = 0, pidl = 0;
		int ret = ov_read(REG_PIDH, &pidh, 1);

		if (ret == BK_OK)
			ret = ov_read(REG_PIDL, &pidl, 1);

		if (ret == BK_OK && pidh == PIDH_VAL && pidl == PIDL_VAL) {
			ok++;
			csh_printf(csh, "  #%-2d 0x%02X%02X OK\r\n", i, pidh, pidl);
		} else {
			csh_printf(csh, "  #%-2d 0x%02X%02X FAIL ret=%d\r\n",
			           i, pidh, pidl, ret);
		}
		rtos_delay_milliseconds(20);
	}

	csh_printf(csh, "test: %d/%d OK\r\n", ok, n);
	return (ok == n) ? 0 : -1;
}

/* =====================================================================
 * 五、分发与命令入口
 * ===================================================================== */

static bool ov_need_i2c(const char *sub)
{
	/* probe 不在表里：它自己管 I2C 生命周期（上电 + MCLK 之后才 init），
	 * 见 ov_probe()。其余子命令仍走命令外层的统一 init/deinit。 */
	static const char *const subs[] = {
		"id", "read", "write", "dump", "reset",
		"init", "ppi", "fps", "test",
	};
	int i;

	for (i = 0; i < (int)(sizeof(subs) / sizeof(subs[0])); i++) {
		if (!strcmp(sub, subs[i]))
			return true;
	}
	return false;
}

static int ov_dispatch(chry_shell_t *csh, int argc, char **argv, const char *sub)
{
	/* ov3660 */
	if (argc == 1)
		return ov_status(csh);

	/* ov3660 help */
	if (!strcmp(sub, "help")) {
		ov_usage(csh);
		return 0;
	}

	/* 电源 / MCLK：不碰 SCCB */
	if (!strcmp(sub, "on"))
		return ov_power(csh, true);
	if (!strcmp(sub, "off"))
		return ov_power(csh, false);
	if (!strcmp(sub, "mclk")) {
		if (argc < 3) {
			csh_printf(csh, "usage: ov3660 mclk on|off|<MHz>\r\n");
			return -1;
		}
		return ov_mclk_cmd(csh, argv[2]);
	}

	/* SCCB 波特率：本板信号完整性差，传感器 ACK 不稳时先降速再排查硬件。
	 * 标准 SCCB 上限 400k；实测建议 100k 起步，不行就 50k / 20k。 */
	if (!strcmp(sub, "sccb")) {
		if (argc < 3) {
			csh_printf(csh, "sccb baud = %u Hz (0 = default 100k)\r\n",
			           (unsigned)i2c_bus_get_baud(OV3660_I2C_BUS));
			csh_printf(csh, "  usage: ov3660 sccb <hz>   e.g. 50000 / 20000 / 100000\r\n");
			csh_printf(csh, "  note : applies to ov3660 cmds immediately;\r\n");
			csh_printf(csh, "         'camera sccb <hz>' is separate (dvp path)\r\n");
			return 0;
		}

		{
			uint32_t hz = (uint32_t)atoi(argv[2]);
			bk_err_t r = i2c_bus_set_baud(OV3660_I2C_BUS, hz);

			if (r != BK_OK) {
				csh_printf(csh, "set baud %u failed: %d (range 10k..400k)\r\n",
				           (unsigned)hz, r);
				return -1;
			}
			csh_printf(csh, "sccb baud -> %u Hz (i2c0 rebuilt)\r\n",
			           (unsigned)i2c_bus_get_baud(OV3660_I2C_BUS));
			csh_printf(csh, "try 'ov3660 id' to verify, or 'ov3660 test 20'\r\n");
		}
		return 0;
	}

	if (!strcmp(sub, "probe"))
		return ov_probe(csh);
	if (!strcmp(sub, "id"))
		return ov_read_pid(csh, true);

	if (!strcmp(sub, "read")) {
		int len = (argc > 3) ? atoi(argv[3]) : 1;

		if (argc < 3) {
			csh_printf(csh, "usage: ov3660 read <reg> [len]\r\n");
			return -1;
		}
		return ov_read_cmd(csh, argv[2], len);
	}

	if (!strcmp(sub, "write")) {
		if (argc < 4) {
			csh_printf(csh, "usage: ov3660 write <reg> <val>\r\n");
			return -1;
		}
		return ov_write_cmd(csh, argv[2], argv[3]);
	}

	if (!strcmp(sub, "dump"))
		return ov_dump(csh, (argc > 2 && !strcmp(argv[2], "all")));

	if (!strcmp(sub, "reset")) {
		int ret = ov_write(REG_SYS_CTRL0, 0x82);

		if (ret != BK_OK) {
			csh_printf(csh, "reset write failed: %d\r\n", ret);
			return -1;
		}
		rtos_delay_milliseconds(10);
		csh_printf(csh, "soft reset done (0x3008=0x82)\r\n");
		return 0;
	}

	if (!strcmp(sub, "init"))
		return ov_init(csh);

	if (!strcmp(sub, "ppi")) {
		if (argc < 4) {
			csh_printf(csh, "usage: ov3660 ppi <w> <h>\r\n");
			return -1;
		}
		return ov_ppi(csh, atoi(argv[2]), atoi(argv[3]));
	}

	if (!strcmp(sub, "fps")) {
		if (argc < 3) {
			csh_printf(csh, "usage: ov3660 fps <5|10|15|20|25|30>\r\n");
			return -1;
		}
		return ov_fps(csh, atoi(argv[2]));
	}

	if (!strcmp(sub, "test")) {
		int n = (argc > 2) ? atoi(argv[2]) : 10;

		if (n < 1 || n > 100)
			n = 10;
		return ov_test(csh, n);
	}

	csh_printf(csh, "unknown subcommand '%s'\r\n", sub);
	ov_usage(csh);
	return -1;
}

int ov3660_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	const char *sub = (argc > 1) ? argv[1] : "";
	bool need_i2c = ov_need_i2c(sub);
	int ret;

	if (need_i2c) {
		bk_err_t r = i2c_bus_init(OV3660_I2C_BUS);

		if (r != BK_OK) {
			csh_printf(csh, "i2c0 init failed: %d\r\n", r);
			return -1;
		}
	}

	ret = ov_dispatch(csh, argc, argv, sub);

	if (need_i2c)
		i2c_bus_deinit(OV3660_I2C_BUS);

	return ret;
}
