// Copyright 2026
// OV3660 DVP sensor 驱动实现

#include "bsp_ov3660.h"
#include "bsp_board_cfg.h"

#include <components/dvp_camera.h>
#include <components/dvp_camera_types.h>
#include <components/media_types.h>
#include "dvp_sensor_devices.h"
#include "gpio_driver.h"        /* gpio_dev_map/unmap：总线恢复 */
#include <driver/gpio.h>
#include <driver/hal/hal_gpio_types.h>  /* GPIO_DEV_I2C0_SCL/SDA */
#include <driver/i2c.h>
#include "bk_misc.h"            /* bk_delay_us() */
#include <os/os.h>
#include <os/mem.h>
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
#define OV3660_REG_X_INCREMENT   0x3814
#define OV3660_REG_Y_INCREMENT   0x3815
#define OV3660_REG_TIMING_TC20   0x3820   /* binning / vflip */
#define OV3660_REG_TIMING_TC21   0x3821   /* binning / hmirror */
#define OV3660_REG_4514          0x4514
#define OV3660_REG_4520          0x4520
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

/* 供 cmd_camera 在 bk_dvp_open 前同步（两边用同一个 SCCB 速率，恢复时
 * 重建控制器才不会悄悄改回别的速率）。 */
static uint32_t s_sccb_baud = 100000;

void bsp_ov3660_set_sccb_baud(uint32_t hz)
{
	if (hz >= 10000 && hz <= 400000)
		s_sccb_baud = hz;
}

static int ov3660_write(uint16_t reg, uint8_t val);

/* PCLK 分频覆盖：0 = 由 set_ppi 按分辨率自动决定。
 * 本板信号完整性差时，`camera pclk 4/8` 直接把 sensor 的 PCLK（线上
 * 数据速率）降一半/四分之一，比降 MCLK 更直接（MCLK 最低只到 15MHz）。 */
static uint8_t s_pclk_div_override;

void bsp_ov3660_set_pclk_div(uint8_t div)
{
	s_pclk_div_override = div;
	if (div == 0)
		return;

	/* 立即生效：PCLK ratio + 手动 PCLK 模式 */
	ov3660_write(OV3660_REG_PCLK_RATIO, div & 0x1f);
	ov3660_write(OV3660_REG_VFIFO_CTRL0C, 0x22);
	LOGI("pclk divider -> %u\r\n", div);
}

/* SCCB 控制器恢复（只用 SDK API，不碰 LL 寄存器）。
 *
 * 现场现象：一次传输没有等到 STOP（sensor 切 PLL/复位时可能拉住总线），
 * 之后 sm_bus_status.busy 一直为 1，SDK 的每次传输都在 i2c_wait_sm_bus_idle()
 * 里等 50ms 然后返回 BK_ERR_I2C_SM_BUS_BUSY —— 而 SDK 不会自己恢复，
 * 于是初始化表从某一条开始全部失败，sensor 没配置好，DVP 自然 0 帧。
 *
 * 关键：sensor 卡在"半截传输"里时会一直**拉住 SCL**，这时只 bk_i2c_init()
 * 复位主控没有用（实测 ov3660 id 仍然 -11523 总线忙）。必须先做一次
 * 标准 I2C 总线恢复：把 SCL/SDA 切成 GPIO，补 9 个 SCL 时钟 + 一个 STOP，
 * 把从机的移位寄存器顶空，然后再恢复复用、软复位主控。
 * 这里不做 bk_i2c_deinit()：deinit 会销毁信号量，而卡住的那次传输的 ISR
 * 之后还可能来一次，销毁后释放会踩空；重新 init 复用信号量更安全。 */
static void ov3660_sccb_bus_clear(void)
{
	const gpio_id_t scl = BOARD_CAM_I2C_SCL_PIN;
	const gpio_id_t sda = BOARD_CAM_I2C_SDA_PIN;
	int i;

	/* 1. 引脚从 I2C 切到 GPIO（unmap 会关掉第二功能，主控暂时悬空） */
	gpio_dev_unmap(scl);
	gpio_dev_unmap(sda);
	bk_gpio_disable_output(scl);
	bk_gpio_disable_output(sda);
	bk_gpio_disable_pull(scl);
	bk_gpio_disable_pull(sda);
	bk_gpio_pull_up(scl);
	bk_gpio_pull_up(sda);
	bk_gpio_enable_input(scl);
	bk_gpio_enable_input(sda);
	bk_delay_us(10);

	/* 2. 最多 9 个 SCL 脉冲：从机如果卡在某一位，8 个脉冲内会把它移出去 */
	for (i = 0; i < 9; i++) {
		bk_gpio_set_output_low(scl);
		bk_gpio_enable_output(scl);
		bk_delay_us(5);
		bk_gpio_disable_output(scl);     /* 上拉释放 -> 高 */
		bk_delay_us(5);
	}

	/* 3. 补一个 STOP：SCL 低 -> SDA 低 -> SCL 高 -> SDA 高 */
	bk_gpio_set_output_low(scl);
	bk_gpio_enable_output(scl);
	bk_delay_us(5);
	bk_gpio_set_output_low(sda);
	bk_gpio_enable_output(sda);
	bk_delay_us(5);
	bk_gpio_disable_output(scl);
	bk_delay_us(5);
	bk_gpio_disable_output(sda);
	bk_delay_us(5);

	/* 4. 恢复 I2C 复用（保持和 cmd_i2c.c 一致的内部上拉） */
	gpio_dev_map(scl, GPIO_DEV_I2C0_SCL);
	gpio_dev_map(sda, GPIO_DEV_I2C0_SDA);
	bk_gpio_pull_up(scl);
	bk_gpio_pull_up(sda);
}

static void ov3660_sccb_recover(void)
{
	i2c_config_t cfg;

	/* 先做总线级恢复：从机拉住 SCL 时，只复位主控是不够的 */
	ov3660_sccb_bus_clear();

	os_memset(&cfg, 0, sizeof(cfg));
	cfg.baud_rate = s_sccb_baud;
	cfg.addr_mode = I2C_ADDR_MODE_7BIT;

	if (bk_i2c_init(CONFIG_DVP_CAMERA_I2C_ID, &cfg) != BK_OK)
		LOGE("sccb recover failed\r\n");
}

/* 判断一次传输是否真的干净结束。
 * SDK 在"发出 STOP"时就释放 tx/rx 信号量，并不等 STOP 真正完成；若 STOP
 * 没走完（sensor 拉住 SCL），函数仍返回 BK_OK，但 busy 已经挂住。这里留
 * 1ms 让 STOP 收尾，再查控制器 busy —— 正常传输十几 us 就结束了。 */
static bool ov3660_sccb_done(void)
{
	rtos_delay_milliseconds(1);
	return !bk_i2c_is_bus_busy(CONFIG_DVP_CAMERA_I2C_ID);
}

/* 一条正常 SCCB 传输 <1ms。SDK 的 i2c_hardware_memory_write/read 有个坑：
 * rtos_get_semaphore() 超时后返回值被忽略，函数仍然返回 BK_OK（err_code 一直
 * 是 BK_OK），于是 50ms 超时被当成"写成功/读成功"，数据其实没动。
 * 这里用耗时兜底：>=20ms 一律按失败处理，恢复控制器后重试，绝不放过。
 * 现象对应现场日志 "sccb write 0x302C=C3 took 54ms" + regs 读回全 0。 */
#define OV3660_XFER_TIMEOUT_MS   20

static int ov3660_write(uint16_t reg, uint8_t val)
{
	int ret = BK_FAIL;
	static uint8_t s_slow_log;

	for (int t = 0; t < 3; t++) {
		uint32_t t0 = rtos_get_time();
		uint32_t dt;

		ret = dvp_camera_i2c_write_uint16(OV3660_ADDR, reg, val);
		dt = rtos_get_time() - t0;

		if (ret == BK_OK && dt < OV3660_XFER_TIMEOUT_MS &&
		    ov3660_sccb_done())
			return BK_OK;

		if (s_slow_log < 5 && dt >= OV3660_XFER_TIMEOUT_MS) {
			s_slow_log++;
			LOGE("sccb write 0x%04X=%02X timeout (%ums), retry\r\n",
			     reg, val, (unsigned)dt);
		}

		ov3660_sccb_recover();
		rtos_delay_milliseconds(2);
	}

	return (ret == BK_OK) ? BK_FAIL : ret;
}

static int ov3660_read(uint16_t reg, uint8_t *val)
{
	int ret = BK_FAIL;

	for (int t = 0; t < 3; t++) {
		uint32_t t0 = rtos_get_time();
		uint32_t dt;

		*val = 0;
		ret = dvp_camera_i2c_read_uint16(OV3660_ADDR, reg, val);
		dt = rtos_get_time() - t0;

		if (ret == BK_OK && dt < OV3660_XFER_TIMEOUT_MS &&
		    ov3660_sccb_done())
			return BK_OK;

		ov3660_sccb_recover();
		rtos_delay_milliseconds(2);
	}

	*val = 0;
	return (ret == BK_OK) ? BK_FAIL : ret;
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

/* 初始化表写入（带逐条诊断）：
 * 之前返回值被忽略、每条写还附带读回校验，出现 "i2c sm bus is busy"
 * 时既不知道是哪条寄存器失败、也不知道失败后 sensor/总线已挂。
 * 现在：失败即打印首错寄存器与错误码，连续 3 条失败终止（继续写无意义）；
 * 校验只在写成功后进行，节省一半 I2C 操作。 */
static int ov3660_write_regs(const ov3660_reg_t *regs, uint32_t num)
{
	uint32_t fails = 0;

	for (uint32_t i = 0; i < num; i++) {
		int ret;

		if (regs[i].reg == OV3660_REG_DELAY) {
			rtos_delay_milliseconds(regs[i].val);
			continue;
		}

		ret = ov3660_write(regs[i].reg, regs[i].val);
		if (ret != BK_OK) {
			if (fails == 0)
				LOGE("init table: first write fail at 0x%04X=0x%02X ret=%d (idx %u/%u)\r\n",
				     regs[i].reg, regs[i].val, ret, i, num);
			fails++;
			if (fails >= 3) {
				LOGE("init abort: %u consecutive fails (idx %u/%u)\r\n",
				     fails, i, num);
				return -1;
			}
			continue;
		}

		ov3660_write_verify(regs[i].reg, regs[i].val);
	}

	LOGI("init table done, %u/%u written\r\n", num - fails, num);
	return (fails == 0) ? 0 : -1;
}

/* 写一个 16bit 寄存器对：reg 存高字节，reg+1 存低字节（大端） */
static void ov3660_write_reg16(uint16_t reg, uint16_t val)
{
	ov3660_write(reg,     (uint8_t)((val >> 8) & 0xFF));
	ov3660_write(reg + 1, (uint8_t)(val & 0xFF));
}

/* 写 X/Y 一对 16bit 值：reg 是 X 的 H 寄存器，Y 的 H 在 reg+2。
 *
 * 【2026-11 修正】原来这里写的是 (vh & 0xFF) 和 (vl & 0xFF)，把"值的
 * 低字节"直接塞进 H 寄存器：
 *   X_OUT_SIZE (0x3808/09) = 640/480 被写成 0x80/0xE0 -> 尺寸 0x80E0，
 *   X_ADDR_END(0x3804/05)  = 2079/1547 写成 0x1F/0x0B，
 *   而且 Y 的 H/L(0x3806/07) 根本没写。
 * 窗口、输出尺寸、总尺寸全错，DVP 自然一帧都收不到。 */
static void ov3660_write_addr(uint16_t reg, uint16_t x, uint16_t y)
{
	ov3660_write_reg16(reg, x);
	ov3660_write_reg16(reg + 2, y);
}

/* ---------------- 默认寄存器表（来自 esp32-camera ov3660_settings.h） ---------------- */
static const ov3660_reg_t ov3660_default_regs[] = {
	{OV3660_REG_SYS_CTRL0, 0x82},      /* software reset */
	/* 复位期间 SCCB 挂起并可能拉住 SDA：10ms 实测不够，idx7(0x4740) 起全总线
	 * busy 死锁（-11523 = I2C_SM_BUS_BUSY）。OV3660 datasheet 要求
	 * SW reset 后等内部初始化完成再继续 SCCB，这里给足 100ms。 */
	{OV3660_REG_DELAY, 100},

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

	/* MCLK 刚打开时 sensor 的 SCCB 还没就绪：现场实测前 ~2ms 的读
	 * 全是 NACK（一次 detect 会打 8 条 master_read get ack failed）。
	 * 先等 10ms，既省掉这串告警，也避免 NACK 把总线 SDA 留在低位。 */
	rtos_delay_milliseconds(10);

	ov3660_read(OV3660_REG_PIDH, &pidh);
	ov3660_read(OV3660_REG_PIDL, &pidl);

	LOGI("detect id: 0x%02X%02X (expect 0x%02X%02X)\r\n",
	     pidh, pidl, OV3660_PIDH_VAL, OV3660_PIDL_VAL);

	return (pidh == OV3660_PIDH_VAL && pidl == OV3660_PIDL_VAL);
}

/* PLL：参数含义见 esp32-camera set_pll()。
 * YUV 模式按 **16MHz XCLK** 调（esp32-camera 的注释 "tuned for 16MHz XCLK
 * and 8MHz PCLK"），不同分辨率的 multiplier / pclk_div 不同（与
 * esp32-camera set_framesize 的分支逐项对应）。
 * 注意：本工程 cam_open 必须用 MCLK_16M 才能和这里对上。 */
static void ov3660_set_pll(uint8_t multiplier, uint8_t seld5, uint8_t pclk_div)
{
	ov3660_write(OV3660_REG_PLL_CTRL0, 0x00);              /* bypass off */
	ov3660_write(OV3660_REG_PLL_CTRL1, multiplier & 0x1f);
	ov3660_write(OV3660_REG_PLL_CTRL2, 0x10 | 1);          /* charge pump | sys_div=1 */
	ov3660_write(OV3660_REG_PLL_CTRL3,
	             (uint8_t)(((0 & 0x3) << 4) | (seld5 & 0x3))); /* pre_div=0(1x) */
	ov3660_write(OV3660_REG_PCLK_RATIO, pclk_div & 0x1f);
	ov3660_write(OV3660_REG_VFIFO_CTRL0C, 0x22);           /* pclk manual */
}

int bsp_ov3660_init(void)
{
	int ret;

	LOGI("init\r\n");

	/* 软复位。esp32-camera 的 reset() 在 0x82 之后等 100ms 才写表，
	 * 这里照做：sensor 内部初始化没完成时写表容易被 NACK/拉住总线。 */
	if (ov3660_write(OV3660_REG_SYS_CTRL0, 0x82) != BK_OK) {
		LOGE("soft reset failed\r\n");
		return -1;
	}
	rtos_delay_milliseconds(100);

	ret = ov3660_write_regs(ov3660_default_regs,
	                        sizeof(ov3660_default_regs) / sizeof(ov3660_default_regs[0]));
	if (ret != 0) {
		LOGE("init aborted: default regs failed\r\n");
		return -1;
	}

	/* YUV422 输出 */
	ret = ov3660_write_regs(ov3660_fmt_yuv422,
	                        sizeof(ov3660_fmt_yuv422) / sizeof(ov3660_fmt_yuv422[0]));
	if (ret != 0) {
		LOGE("init aborted: yuv422 failed\r\n");
		return -1;
	}

	/* PLL / 开窗 / binning 都放在 set_ppi() 里按分辨率配置，
	 * 与 esp32-camera 的 set_framesize() 一致（init 里不再写死 PLL）。 */
	rtos_delay_milliseconds(BOARD_SENSOR_RESET_MS);
	LOGI("init done\r\n");
	return 0;
}

/* 4:3 全窗口参数（来自 esp32-camera ratio_table[4x3]） */
#define OV3660_43_START_X   0
#define OV3660_43_START_Y   0
#define OV3660_43_END_X     2079
#define OV3660_43_END_Y     1547
#define OV3660_43_TOTAL_X   2300
#define OV3660_43_TOTAL_Y   1564
#define OV3660_43_MAX_W     2048
#define OV3660_43_MAX_H     1536

/* 与 esp32-camera set_image_options() 对齐。
 * 640x480 / 320x240 都满足 w<=max/2 && h<=max/2，esp32-camera 走 binning
 * 分支：0x3820/0x3821 的 binning 位、0x4514=0xaa、0x4520=0x0b、
 * X/Y_INCREMENT=0x31；否则 0x40/0x00/0x88/0xb0/0x11。 */
static void ov3660_set_image_options(bool binning)
{
	ov3660_write(OV3660_REG_TIMING_TC20, binning ? 0x01 : 0x40);
	ov3660_write(OV3660_REG_TIMING_TC21, binning ? 0x01 : 0x00);
	ov3660_write(OV3660_REG_4514, binning ? 0xaa : 0x88);
	ov3660_write(OV3660_REG_4520, binning ? 0x0b : 0xb0);
	ov3660_write(OV3660_REG_X_INCREMENT, binning ? 0x31 : 0x11);
	ov3660_write(OV3660_REG_Y_INCREMENT, binning ? 0x31 : 0x11);
}

/* 现场核对：读回 YUV 输出最关键的一组寄存器，一行打印。
 * 如果 PCLK/同步都有了但 DVP 还是 0 字节，先看这行确认 sensor 到底有没有
 * 被配置成功（对比 set_ppi 打印的期望值）。
 * 开头的 pid 用来确认"读通路"本身还活着：pid 不对说明 I2C 又挂了，
 * 此时后面的 0 不能当成"寄存器没写进去"。 */
static void ov3660_log_key_regs(void)
{
	static const uint16_t regs[] = {
		0x300A, 0x300B,                   /* PID */
		0x3808, 0x3809, 0x380A, 0x380B,   /* 输出宽高 */
		0x380E, 0x380F,                   /* VTS */
		0x3820, 0x3821, 0x3814,           /* binning / 增量 */
		0x303B, 0x303D, 0x3824,           /* PLL mult / PLL3 / PCLK ratio */
	};
	uint8_t v[sizeof(regs) / sizeof(regs[0])];
	unsigned fails = 0;

	os_memset(v, 0, sizeof(v));
	for (unsigned i = 0; i < sizeof(regs) / sizeof(regs[0]); i++) {
		if (ov3660_read(regs[i], &v[i]) != BK_OK)
			fails++;
	}

	LOGI("regs(%u read fail): pid=%02X%02X out=%ux%u vts=%u bin=%02X/%02X "
	     "inc=%02X pll=%02X/%02X/%02X\r\n",
	     fails, v[0], v[1],
	     (unsigned)((v[2] << 8) | v[3]), (unsigned)((v[4] << 8) | v[5]),
	     (unsigned)((v[6] << 8) | v[7]),
	     v[8], v[9], v[10], v[11], v[12], v[13]);
}

static void ov3660_set_window(uint16_t w, uint16_t h, bool binning, bool scale_en)
{
	/* esp32-camera set_framesize()：binning 时 VTS 取一半、ISP 偏移 8/2 */
	uint16_t total_y = binning ? (uint16_t)(OV3660_43_TOTAL_Y / 2 + 1)
	                           : OV3660_43_TOTAL_Y;
	uint16_t off_x = binning ? 8 : 16;
	uint16_t off_y = binning ? 2 : 6;

	/* 开窗 */
	ov3660_write_addr(OV3660_REG_X_ADDR_ST_H, OV3660_43_START_X, OV3660_43_START_Y);
	ov3660_write_addr(OV3660_REG_X_ADDR_END_H, OV3660_43_END_X, OV3660_43_END_Y);
	/* 输出尺寸 */
	ov3660_write_addr(OV3660_REG_X_OUT_SIZE_H, w, h);
	/* 总尺寸 */
	ov3660_write_addr(OV3660_REG_X_TOTAL_H, OV3660_43_TOTAL_X, total_y);
	/* ISP 偏移 */
	ov3660_write_addr(OV3660_REG_X_OFFSET_H, off_x, off_y);

	/* ISP_CONTROL_01 bit5: scale enable */
	ov3660_write(OV3660_REG_ISP_CTRL01, scale_en ? 0xA3 : 0x83);
}

int bsp_ov3660_set_ppi(media_ppi_t ppi)
{
	uint16_t w = ppi >> 16;
	uint16_t h = ppi & 0xFFFF;
	bool binning = (w <= (OV3660_43_MAX_W / 2) && h <= (OV3660_43_MAX_H / 2));
	/* 与 esp32-camera 一致：只有"全尺寸"和"正好 1/2 尺寸"不用 scaler，
	 * 其余（含 640x480）都要打开 ISP 的 scale。 */
	bool scale = !((w == OV3660_43_MAX_W && h == OV3660_43_MAX_H) ||
	               (w == OV3660_43_MAX_W / 2 && h == OV3660_43_MAX_H / 2));
	uint8_t mult, seld5, pclk_div;

	if (w < 16 || h < 16 || w > OV3660_43_MAX_W || h > OV3660_43_MAX_H) {
		LOGE("unsupported ppi %dx%d\r\n", w, h);
		return -1;
	}

	/* YUV 模式的 PLL 分支（esp32-camera set_framesize）：
	 *   >  HVGA(480x320) -> mult 4, seld5 2, pclk_div 2
	 *   >= QVGA(320x240) -> mult 8, seld5 2, pclk_div 4
	 *   <  QVGA          -> mult 8, seld5 0, pclk_div 8 */
	if (w > 480 || h > 320) {
		mult = 4; seld5 = 2; pclk_div = 2;
	} else if (w >= 320 && h >= 240) {
		mult = 8; seld5 = 2; pclk_div = 4;
	} else {
		mult = 8; seld5 = 0; pclk_div = 8;
	}

	if (s_pclk_div_override)
		pclk_div = s_pclk_div_override;

	LOGI("set_ppi %dx%d (binning=%d scale=%d pll=%u/%u pclk_div=%u)\r\n",
	     w, h, binning, scale, mult, seld5, pclk_div);

	ov3660_set_window(w, h, binning, scale);
	ov3660_set_image_options(binning);
	ov3660_set_pll(mult, seld5, pclk_div);
	rtos_delay_milliseconds(5);
	ov3660_log_key_regs();
	return 0;
}

int bsp_ov3660_set_fps(frame_fps_t fps)
{
	/* esp32-camera 的 OV3660 没有 set_fps：帧率由 set_framesize 里的
	 * HTS/VTS 和 PLL 决定（YUV 640x480 在 16MHz XCLK 下约 4.4fps）。
	 * 这里保持 VTS 不动。旧实现把 4:3 表的 1564（其实是 HTS/VTS 的
	 * 基准）当"30fps 基准"再写 0x380E，只会把帧率拖慢，没有收益。 */
	LOGD("set_fps %d ignored (VTS set by set_ppi)\r\n", (int)fps);
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

/* 当前 SDK 的 dvp_sensor_config_t 要求：
 *   int (*read_register)(uint32_t reg, uint32_t *val);
 *   int (*write_register)(uint32_t reg, uint32_t val);
 * 旧版 app_bk_pi 用的是 void (*)(bool) 的 read_enable，签名不兼容。 */
static int ov3660_read_register(uint32_t reg, uint32_t *val)
{
	uint8_t v = 0;
	int ret;

	if (!val)
		return -1;

	ret = ov3660_read((uint16_t)reg, &v);
	if (ret == 0)
		*val = v;
	return ret;
}

static int ov3660_write_register(uint32_t reg, uint32_t val)
{
	return ov3660_write((uint16_t)reg, (uint8_t)val);
}

/* SDK 官方探测路径：bk_dvp_detect() 遍历链接段 .camera_sensor_detect_function_list
 * （BK_CAMERA_SENSOR_DETECT_SECTION 宏注册），bk_dvp_set_devices_list() 对这条
 * 路径不生效 —— 所以必须用宏把 OV3660 挂进链接段，否则 bk_dvp_open 永远
 * 报 "no dvp camera found"。
 * 约定：f 是返回 const dvp_sensor_config_t* 的函数；带 __attribute__((used))，
 * 不怕 --gc-sections 剔除。 */
/* 运行时同步极性覆盖。
 *
 * dvp_sensor_config_t 里的 vsync/hsync 是 const，但 bk_dvp_open 打开时会从
 * 探测函数返回的指针读它们。这里让 detect 返回一份**可写副本**，
 * `camera sync` 改副本、下一次 camera off/show 生效 —— 不用反复改代码重烧，
 * 方便逐个试 4 种极性组合。 */
static dvp_sensor_config_t s_sensor_cfg;
static bool s_sensor_cfg_ready;

void bsp_ov3660_set_sync(sync_level_t vsync, sync_level_t hsync)
{
	if (!s_sensor_cfg_ready) {
		s_sensor_cfg = bsp_sensor_ov3660;
		s_sensor_cfg_ready = true;
	}
	s_sensor_cfg.vsync = vsync;
	s_sensor_cfg.hsync = hsync;
	LOGI("sync -> vsync=%d hsync=%d\r\n", (int)vsync, (int)hsync);
}

const dvp_sensor_config_t *ov3660_detect_sensor(void)
{
	if (!s_sensor_cfg_ready) {
		s_sensor_cfg = bsp_sensor_ov3660;
		s_sensor_cfg_ready = true;
	}
	if (bsp_ov3660_detect())
		return &s_sensor_cfg;
	return NULL;
}

BK_CAMERA_SENSOR_DETECT_SECTION(ov3660_detect_sensor);

/* 注册项：id 用 ID_UNKNOW 之外的自定义值不影响 SDK 内部逻辑（SDK 仅按 id 查找表项），
 * 这里沿用 ID_UNKNOW 会被 bk_dvp_detect 误判，故用未占用值 0x3C。 */
const dvp_sensor_config_t bsp_sensor_ov3660 = {
	.name = "ov3660",
	.clk = MCLK_16M,
	.fmt = PIXEL_FMT_YUYV,
	/* 同步极性：默认按"高有效、不做反转"（和 SDK 的 OV2640 配置一致）。
	 * init 表里 0x4740=0x21，按寄存器注释 bit1=0 应该是 HREF 低有效，
	 * 但实测按 LOW（yuv_buf 打开 hsync_rev）时 yuv_buf 报
	 * "sensor's yuyv data resoltion is not right" 且只收到 <1KB 数据，
	 * 说明这条总线上 HREF 的极性/连接和注释不一致。
	 * 运行时可 `camera sync <vsync> <hsync>` 四种组合逐个试，不用重烧。 */
	.vsync = SYNC_HIGH_LEVEL,
	.hsync = SYNC_HIGH_LEVEL,
	.def_ppi = PPI_640X480,
	.def_fps = FPS15,
	.fps_cap = FPS5 | FPS10 | FPS15 | FPS20 | FPS25 | FPS30,
	.ppi_cap = PPI_CAP_320X240 | PPI_CAP_640X480 | PPI_CAP_1280X720 | PPI_CAP_1600X1200,
	.id = 0x3C,
	.address = OV3660_ADDR,
	.init = bsp_ov3660_init,
	.detect = bsp_ov3660_detect,
	.set_ppi = bsp_ov3660_set_ppi,
	.set_fps = bsp_ov3660_set_fps,
	.power_down = NULL,
	.read_register = ov3660_read_register,
	.write_register = ov3660_write_register,
};
