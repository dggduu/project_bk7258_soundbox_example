// Copyright 2026
// CherrySH 命令：i2c —— I2C 总线扫描 / 读寄存器
//
//   i2c                                列出板上的两条 I2C 总线
//   i2c scan <bus>                     用 SDK 硬件 I2C 驱动扫描 0x08..0x77
//   i2c bb <bus>                       用 GPIO 模拟 I2C（bit-bang）扫描，对比用
//   i2c read <bus> <addr> <reg> [len]  用硬件 I2C 读从机寄存器
//
// 关于 I2C0 的坑（最坑，ov3660 读不到 PID 的根因）：
//   本工程 CONFIG_GPIO_DEFAULT_SET_SUPPORT=y，SDK 的 i2c_init_gpio() 不编译，
//   引脚全靠 usr_gpio_cfg.h 的 GPIO_DEFAULT_DEV_CONFIG。该表里 P20/P21 是
//   GPIO_SECOND_FUNC_DISABLE + GPIO_DEV_INVALID，没映射到 I2C0。
//   结果：控制器 init 成功，但 SCL/SDA 没连上，读回恒 0x00 且驱动不报错。
//   所以 hw_init() 里必须显式 gpio_dev_map(P20/P21, GPIO_DEV_I2C0_*)。
//
// 关于 I2C1 的坑（很关键）：
//   SDK 的 I2C1 默认复用 GPIO0/GPIO1（i2c_ll.h: I2C1_LL_SCL_PIN/SDA_PIN），
//   而这板子 GPIO0/1 是 UART1 控制台（CherrySH 就挂在上面）。
//   bk_i2c_init(I2C_ID_1) 会把这两个脚抢走。所以这里在 init 之后立刻
//   把 P42/P43 挂到 I2C1，再把 P0/P1 还给 UART1（gpio_dev_map 只改引脚复用，
//   I2C 控制器没有 pin 选择寄存器，i2c_ll_set_pin() 是空函数）。
//
// 为什么保留 bit-bang：
//   硬件 I2C 依赖 pinmux/时钟都对；bit-bang 只碰 SCL/SDA，能在
//   "驱动起不来 / 引脚被别人占用" 时做对照。两个结果一致才可信。
//
// 注意：驱动层**没有** scan 接口（cli_i2c.c 也只有 init/read/write），
// 这里的 scan 是用 bk_i2c_master_read() 逐个地址探测实现的：
// 无应答时驱动返回 BK_ERR_I2C_ACK_TIMEOUT，不会卡满 timeout。

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_board_cfg.h"
#include "gpio_driver.h"     /* gpio_dev_map() / gpio_dev_unmap() */
#include "bk_misc.h"         /* bk_delay_us() */
#include <driver/gpio.h>
#include <driver/hal/hal_gpio_types.h>  /* GPIO_DEV_I2C0_SCL/SDA */
#include <driver/i2c.h>
#include <os/os.h>
#include <os/mem.h>

#define I2C_HALF_US         5      /* bit-bang 半周期 ≈5us，约 100kHz */
#define I2C_PROBE_TIMEOUT   20     /* 硬件探测单地址超时 ms */
#define I2C_MEM_TIMEOUT     50
#define I2C_SCAN_FIRST      0x08
#define I2C_SCAN_LAST       0x77
#define I2C_READ_MAX        16

typedef struct {
	const char *name;
	const char *desc;
	i2c_id_t id;
	gpio_id_t scl;
	gpio_id_t sda;
} bus_desc_t;

static const bus_desc_t s_bus[] = {
	{ "i2c0", "camera SCCB",  I2C_ID_0, BOARD_CAM_I2C_SCL_PIN, BOARD_CAM_I2C_SDA_PIN },
	{ "i2c1", "touch/SC7A20", I2C_ID_1, BOARD_TP_I2C_SCL_PIN,  BOARD_TP_I2C_SDA_PIN  },
};
#define BUS_CNT ((int)(sizeof(s_bus) / sizeof(s_bus[0])))

/* 板上已知器件（来自 app_bk_pi 的原理图网表） */
static const struct {
	int bus;
	uint8_t addr;
	const char *name;
} s_known[] = {
	{ 1, 0x15, "CST816S touch" },
	{ 1, 0x18, "SC7A20 accel" },
	{ 1, 0x19, "SC7A20 accel (alt)" },
	{ 0, 0x3C, "OV3660 camera" },
};
#define KNOWN_CNT ((int)(sizeof(s_known) / sizeof(s_known[0])))

static const char *known_name(int bus, uint8_t addr)
{
	int i;

	for (i = 0; i < KNOWN_CNT; i++) {
		if (s_known[i].bus == bus && s_known[i].addr == addr)
			return s_known[i].name;
	}
	return NULL;
}

/* =====================================================================
 * 一、硬件 I2C（SDK 驱动）
 * ===================================================================== */

static bool s_hw_inited[BUS_CNT];

/* 波特率缓存：s_baud[bus]==0 表示未设置过，hw_init 里回落 100k。
 * 必须放在 hw_init 之前——C 的 static 变量要先声明后使用。 */
static uint32_t s_baud[BUS_CNT];

/* SDK 初始化 I2C1 会把 P0/P1 抢走，这里把 UART1 的两个脚放回去。
 * 只在 I2C1 上需要；I2C0 的默认脚就是 P20/P21，和板子一致。 */
static void uart1_pin_restore(void)
{
	gpio_dev_map(GPIO_0, GPIO_DEV_UART1_TXD);
	gpio_dev_map(GPIO_1, GPIO_DEV_UART1_RXD);
	bk_gpio_pull_up(GPIO_0);
	bk_gpio_pull_up(GPIO_1);
}

static bk_err_t hw_init(int bus)
{
	const bus_desc_t *b = &s_bus[bus];
	i2c_config_t cfg;
	bk_err_t ret;

	if (s_hw_inited[bus])
		return BK_OK;

	os_memset(&cfg, 0, sizeof(cfg));
	/* 波特率可调（本板信号完整性差，SCCB 可能要降到 50k 甚至更低），
	 * 默认 100k；i2c_bus_set_baud() 修改后对已初始化的总线立即重建。 */
	cfg.baud_rate = s_baud[bus] ? s_baud[bus] : I2C_BAUD_RATE_100KHZ;
	cfg.addr_mode = I2C_ADDR_MODE_7BIT;

	ret = bk_i2c_init(b->id, &cfg);
	if (ret != BK_OK)
		return ret;

	/* 关键修复：本工程 CONFIG_GPIO_DEFAULT_SET_SUPPORT=y，i2c_driver.c 里的
	 * i2c_init_gpio() 被 #if 裁掉、根本不编译，I2C 引脚完全依赖
	 * usr_gpio_cfg.h 的 GPIO_DEFAULT_DEV_CONFIG。而该表里 P20/P21（以及
	 * P42/P43）都是 GPIO_SECOND_FUNC_DISABLE + GPIO_DEV_INVALID，即
	 * 没有任何引脚被映射到 I2C 外设。
	 *
	 * 后果：bk_i2c_init() 成功、bk_i2c_memory_read() 也返回 BK_OK，但
	 * SCL/SDA 没连到 I2C 控制器，总线上没有波形，读回恒为 0x00——
	 * 这正是 ov3660 probe 得到 PID 0x0000 却不报 SCCB 错误的原因。
	 * 所以两条总线都必须在这里显式 gpio_dev_map。
	 *
	 * 先 unmap 再 map：清掉 gpio_2_func_en，避免重复 init 时
	 * gpio_hal_map_check() 返回 BK_ERR_GPIO_INTERNAL_USED 而 map 失败。 */
	gpio_dev_unmap(b->scl);
	gpio_dev_unmap(b->sda);

	if (b->id == I2C_ID_0) {             /* I2C0：P20/P21 -> 摄像头 SCCB */
		gpio_dev_map(b->scl, GPIO_DEV_I2C0_SCL);
		gpio_dev_map(b->sda, GPIO_DEV_I2C0_SDA);
		bk_gpio_pull_up(b->scl);
		bk_gpio_pull_up(b->sda);
	} else if (b->id == I2C_ID_1) {      /* I2C1：P42/P43，再把 P0/P1 还给 UART1 */
		gpio_dev_map(b->scl, GPIO_DEV_I2C1_SCL);
		gpio_dev_map(b->sda, GPIO_DEV_I2C1_SDA);
		bk_gpio_pull_up(b->scl);
		bk_gpio_pull_up(b->sda);
		uart1_pin_restore();
	}

	s_hw_inited[bus] = true;
	return BK_OK;
}

static void hw_deinit(int bus)
{
	if (!s_hw_inited[bus])
		return;

	bk_i2c_deinit(s_bus[bus].id);
	if (s_bus[bus].id == I2C_ID_1)
		uart1_pin_restore();
	s_hw_inited[bus] = false;
}

bk_err_t i2c_bus_set_baud(int bus, uint32_t hz)
{
	if (bus < 0 || bus >= BUS_CNT)
		return BK_ERR_PARAM;
	/* I2C 标准/快速模式范围；SCCB 实际上限 400k，本板建议 50k~100k */
	if (hz < 10000 || hz > 400000)
		return BK_ERR_PARAM;

	s_baud[bus] = hz;

	/* 已初始化的总线立即重建：deinit -> init(新速率) -> 重新 map 引脚 */
	if (s_hw_inited[bus]) {
		hw_deinit(bus);
		return hw_init(bus);
	}

	return BK_OK;
}

bk_err_t i2c_bus_clear(int bus);   /* 实现在下面的 bit-bang 段（需要 pin_low 等 helper） */

bk_err_t i2c_bus_force_rebuild(int bus)
{
	if (bus < 0 || bus >= BUS_CNT)
		return BK_ERR_PARAM;

	/* 无视 s_hw_inited 缓存强制重建，并且先做一次总线恢复：
	 * 第三方代码（如 bk_dvp_open / sensor 初始化）可能把控制器留在 busy，
	 * 或从机卡在半截传输里拉住 SCL —— 那种状态下只 deinit/init 主控没用。 */
	return i2c_bus_clear(bus);
}

uint32_t i2c_bus_get_baud(int bus)
{
	if (bus < 0 || bus >= BUS_CNT)
		return 0;
	return s_baud[bus] ? s_baud[bus] : I2C_BAUD_RATE_100KHZ;
}

/* 逐个地址探测：只发地址+读 1 字节，不往从机写数据。
 * 有应答返回 BK_OK；无应答驱动立刻返回 BK_ERR_I2C_ACK_TIMEOUT。 */
static int hw_probe(int bus, uint8_t addr)
{
	uint8_t dummy = 0;

	return bk_i2c_master_read(s_bus[bus].id, addr, &dummy, 1, I2C_PROBE_TIMEOUT) == BK_OK;
}

static int hw_scan(chry_shell_t *csh, int bus)
{
	const bus_desc_t *b = &s_bus[bus];
	bk_err_t ret;
	int addr, found = 0;

	ret = hw_init(bus);
	if (ret != BK_OK) {
		csh_printf(csh, "i2c%d init failed: %d\r\n", bus, ret);
		return -1;
	}

	csh_printf(csh, "scan %s (%s) hw 0x%02X..0x%02X\r\n",
	           b->name, b->desc, I2C_SCAN_FIRST, I2C_SCAN_LAST);

	for (addr = I2C_SCAN_FIRST; addr <= I2C_SCAN_LAST; addr++) {
		const char *name;

		if (!hw_probe(bus, (uint8_t)addr))
			continue;

		name = known_name(bus, (uint8_t)addr);
		if (name)
			csh_printf(csh, "  0x%02X ACK  <- %s\r\n", addr, name);
		else
			csh_printf(csh, "  0x%02X ACK\r\n", addr);
		found++;
	}

	hw_deinit(bus);
	csh_printf(csh, "%s: %d device(s) found\r\n", b->name, found);
	return found;
}

static int hw_read(chry_shell_t *csh, int bus, uint8_t addr, uint8_t reg, int len)
{
	i2c_mem_param_t p;
	uint8_t buf[I2C_READ_MAX];
	bk_err_t ret;
	int i;

	ret = hw_init(bus);
	if (ret != BK_OK) {
		csh_printf(csh, "i2c%d init failed: %d\r\n", bus, ret);
		return -1;
	}

	os_memset(&p, 0, sizeof(p));
	p.dev_addr = addr;
	p.mem_addr = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
	p.data = buf;
	p.data_size = len;
	p.timeout_ms = I2C_MEM_TIMEOUT;

	ret = bk_i2c_memory_read(s_bus[bus].id, &p);
	hw_deinit(bus);

	if (ret != BK_OK) {
		csh_printf(csh, "0x%02X read failed: %d (no device?)\r\n", addr, ret);
		return -1;
	}

	csh_printf(csh, "0x%02X reg 0x%02X:", addr, reg);
	for (i = 0; i < len; i++)
		csh_printf(csh, " %02X", buf[i]);
	csh_printf(csh, "\r\n");
	return 0;
}

/* =====================================================================
 * 二、GPIO 模拟 I2C（bit-bang）
 *
 * 开漏模拟：写 0 = 输出低；写 1 = 释放（靠上拉）。
 * 注意用 bk_gpio_set_output_low() 而不是 bk_gpio_set_value()：
 * 后者是整字写配置寄存器（bit0 输入值/bit1 输出值/bit3 输出使能...），
 * 会把输出使能一起清掉，引脚根本不会动。
 * ===================================================================== */

static void bb_delay(void)
{
	bk_delay_us(I2C_HALF_US);
}

static void pin_low(gpio_id_t p)
{
	bk_gpio_set_output_low(p);           /* 先写值，再开输出，避免毛刺 */
	bk_gpio_enable_output(p);
}

static void pin_release(gpio_id_t p)
{
	bk_gpio_disable_output(p);
}

static int scl_read(const bus_desc_t *b)
{
	return bk_gpio_get_input(b->scl) ? 1 : 0;
}

static int sda_read(const bus_desc_t *b)
{
	return bk_gpio_get_input(b->sda) ? 1 : 0;
}

static void bb_init(const bus_desc_t *b)
{
	gpio_dev_unmap(b->scl);
	gpio_dev_unmap(b->sda);
	bk_gpio_disable_pull(b->scl);
	bk_gpio_disable_pull(b->sda);
	bk_gpio_pull_up(b->scl);             /* 板上一般有外部上拉，这里兜底 */
	bk_gpio_pull_up(b->sda);
	bk_gpio_enable_input(b->scl);
	bk_gpio_enable_input(b->sda);
	pin_release(b->scl);
	pin_release(b->sda);
	bb_delay();
}

/* 空闲时 SCL/SDA 都应该被上拉成高；不是的话补 9 个时钟把从机顶出来 */
static int bb_bus_ok(const bus_desc_t *b)
{
	int i;

	bb_delay();
	if (scl_read(b) && sda_read(b))
		return 1;

	for (i = 0; i < 9; i++) {
		pin_low(b->scl);
		bb_delay();
		pin_release(b->scl);
		bb_delay();
	}

	bb_delay();
	return (scl_read(b) && sda_read(b)) ? 1 : 0;
}

static void bb_start(const bus_desc_t *b)
{
	pin_release(b->sda);
	pin_release(b->scl);
	bb_delay();
	pin_low(b->sda);                     /* SCL 高时 SDA 下降 = START */
	bb_delay();
	pin_low(b->scl);
	bb_delay();
}

static void bb_stop(const bus_desc_t *b)
{
	pin_low(b->scl);
	bb_delay();
	pin_low(b->sda);
	bb_delay();
	pin_release(b->scl);                 /* SCL 高时 SDA 上升 = STOP */
	bb_delay();
	pin_release(b->sda);
	bb_delay();
}

/* 硬件 I2C 总线恢复（给 ov3660 / camera 等用）。
 *
 * 从机卡在"半截传输"里时会一直拉住 SCL（或 SDA），这时主控的
 * bk_i2c_init()/deinit() 都救不回来 —— 它们只复位主控，总线电平还是低的。
 * 标准做法：把引脚切成 GPIO，补 9 个 SCL 时钟让从机把当前字节移完，
 * 再补一个 STOP，最后恢复 I2C 复用并重建控制器。 */
bk_err_t i2c_bus_clear(int bus)
{
	const bus_desc_t *b;
	int i;

	if (bus < 0 || bus >= BUS_CNT)
		return BK_ERR_PARAM;
	b = &s_bus[bus];

	/* 1. 引脚切 GPIO（unmap 关掉 I2C 第二功能，主控不再驱动） */
	gpio_dev_unmap(b->scl);
	gpio_dev_unmap(b->sda);
	bk_gpio_disable_output(b->scl);
	bk_gpio_disable_output(b->sda);
	bk_gpio_disable_pull(b->scl);
	bk_gpio_disable_pull(b->sda);
	bk_gpio_pull_up(b->scl);
	bk_gpio_pull_up(b->sda);
	bk_gpio_enable_input(b->scl);
	bk_gpio_enable_input(b->sda);
	bb_delay();

	/* 2. 9 个 SCL 脉冲：把卡住的从机移位寄存器顶空 */
	for (i = 0; i < 9; i++) {
		pin_low(b->scl);
		bb_delay();
		pin_release(b->scl);
		bb_delay();
	}

	/* 3. 补 STOP：SCL 低 -> SDA 低 -> SCL 高 -> SDA 高 */
	pin_low(b->scl);
	bb_delay();
	pin_low(b->sda);
	bb_delay();
	pin_release(b->scl);
	bb_delay();
	pin_release(b->sda);
	bb_delay();

	/* 4. 强制重新映射 I2C 引脚并重建控制器 */
	s_hw_inited[bus] = false;
	return hw_init(bus);
}

/* 返回 0 表示从机 ACK */
static int bb_write_byte(const bus_desc_t *b, uint8_t val)
{
	int bit, nack;

	for (bit = 7; bit >= 0; bit--) {
		if (val & (1u << bit))
			pin_release(b->sda);
		else
			pin_low(b->sda);
		bb_delay();
		pin_release(b->scl);
		bb_delay();
		pin_low(b->scl);
		bb_delay();
	}

	pin_release(b->sda);                 /* 第 9 个时钟：读 ACK */
	bb_delay();
	pin_release(b->scl);
	bb_delay();
	nack = sda_read(b);
	pin_low(b->scl);
	bb_delay();

	return nack;
}

static int bb_scan(chry_shell_t *csh, int bus)
{
	const bus_desc_t *b = &s_bus[bus];
	int addr, found = 0;

	bb_init(b);
	if (!bb_bus_ok(b)) {
		csh_printf(csh, "i2c%d bus stuck (SCL=%d SDA=%d), check pull-up / wiring\r\n",
		           bus, scl_read(b), sda_read(b));
		return -1;
	}

	csh_printf(csh, "scan %s (%s) bb SCL=P%d SDA=P%d, 0x%02X..0x%02X\r\n",
	           b->name, b->desc, (int)b->scl, (int)b->sda,
	           I2C_SCAN_FIRST, I2C_SCAN_LAST);

	for (addr = I2C_SCAN_FIRST; addr <= I2C_SCAN_LAST; addr++) {
		const char *name;
		int nack;

		bb_start(b);
		nack = bb_write_byte(b, (uint8_t)(addr << 1));
		bb_stop(b);

		if (nack != 0)
			continue;

		name = known_name(bus, (uint8_t)addr);
		if (name)
			csh_printf(csh, "  0x%02X ACK  <- %s\r\n", addr, name);
		else
			csh_printf(csh, "  0x%02X ACK\r\n", addr);
		found++;
	}

	csh_printf(csh, "%s: %d device(s) found\r\n", b->name, found);
	return found;
}

/* =====================================================================
 * 三、命令
 * ===================================================================== */

static void i2c_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  i2c                                 list buses\r\n");
	csh_printf(csh, "  i2c scan <bus>                      scan with hw i2c driver\r\n");
	csh_printf(csh, "  i2c bb <bus>                        scan with gpio bit-bang\r\n");
	csh_printf(csh, "  i2c read <bus> <addr> <reg> [len]   read register (hw)\r\n");
	csh_printf(csh, "bus: 0=i2c0(SCL=P%d SDA=P%d)  1=i2c1(SCL=P%d SDA=P%d)\r\n",
	           (int)s_bus[0].scl, (int)s_bus[0].sda,
	           (int)s_bus[1].scl, (int)s_bus[1].sda);
	csh_printf(csh, "note: hw scan will print 'get ack failed' on UART0 for empty addresses\r\n");
}

int i2c_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	int bus;

	/* i2c —— 列总线 */
	if (argc == 1) {
		csh_printf(csh, "bus  scl  sda  desc\r\n");
		for (bus = 0; bus < BUS_CNT; bus++) {
			csh_printf(csh, "  %d  P%-2d  P%-2d  %s\r\n", bus,
			           (int)s_bus[bus].scl, (int)s_bus[bus].sda,
			           s_bus[bus].desc);
		}
		csh_printf(csh, "known devices:\r\n");
		for (bus = 0; bus < KNOWN_CNT; bus++) {
			csh_printf(csh, "  i2c%d 0x%02X  %s\r\n",
			           s_known[bus].bus, s_known[bus].addr, s_known[bus].name);
		}
		return 0;
	}

	if (argc < 3) {
		i2c_usage(csh);
		return -1;
	}

	bus = atoi(argv[2]);
	if ((bus < 0) || (bus >= BUS_CNT)) {
		csh_printf(csh, "invalid bus %d\r\n", bus);
		i2c_usage(csh);
		return -1;
	}

	if (!strcmp(argv[1], "scan"))
		return hw_scan(csh, bus);

	if (!strcmp(argv[1], "bb"))
		return bb_scan(csh, bus);

	if (!strcmp(argv[1], "read")) {
		uint8_t addr, reg;
		int len = 1;

		if (argc < 5) {
			i2c_usage(csh);
			return -1;
		}

		addr = (uint8_t)strtoul(argv[3], NULL, 16);
		reg = (uint8_t)strtoul(argv[4], NULL, 16);
		if (argc > 5) {
			len = atoi(argv[5]);
			if (len < 1)
				len = 1;
			if (len > I2C_READ_MAX)
				len = I2C_READ_MAX;
		}

		return hw_read(csh, bus, addr, reg, len);
	}

	i2c_usage(csh);
	return -1;
}

/* =====================================================================
 * 四、SC7A20 加速度计（I2C1：SCL=P42 / SDA=P43，7bit 地址 0x18 / 0x19）
 *
 *   accel                   探测 + 读 WHO_AM_I + 配置 + 读一次 XYZ
 *   accel poll [n]          连续读 n 次（默认 10 次，100ms 一次）
 *   accel reg <reg>         读一个寄存器（排查用）
 *
 * 寄存器定义和 CTRL_REG1 的取值来自 SDK 自带的
 * ap/components/bk_gsensor/gsensor_sc7a20.c：数据寄存器 0x28 要或上 0x80
 * 才能连续读，WHO_AM_I(0x0F) 期望 0x11。
 * ===================================================================== */

#define ACC_BUS             1        /* I2C1 */
#define ACC_ADDR            0x18     /* SDO 下拉 -> 0x18 */
#define ACC_ADDR_ALT        0x19     /* SDO 上拉 -> 0x19 */
#define ACC_REG_CHIP_ID     0x0F
#define ACC_CHIP_ID         0x11     /* SC7A20 的 WHO_AM_I */
#define ACC_REG_CTRL1       0x20
#define ACC_REG_CTRL4       0x23
#define ACC_REG_OUT_X_L     0x28
#define ACC_CTRL1_100HZ_XYZ 0x57     /* ODR=100Hz + X/Y/Z 使能 */
#define ACC_CTRL4_HR_2G     0x88     /* BDU + 高分辨率(12bit) + ±2g，1mg/LSB */

static uint8_t s_acc_addr;

static bk_err_t acc_read(uint8_t addr, uint8_t reg, uint8_t *buf, int len)
{
	i2c_mem_param_t p;

	os_memset(&p, 0, sizeof(p));
	p.dev_addr = addr;
	p.mem_addr = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
	p.data = buf;
	p.data_size = len;
	p.timeout_ms = 50;

	return bk_i2c_memory_read(s_bus[ACC_BUS].id, &p);
}

static bk_err_t acc_write(uint8_t addr, uint8_t reg, uint8_t val)
{
	i2c_mem_param_t p;

	os_memset(&p, 0, sizeof(p));
	p.dev_addr = addr;
	p.mem_addr = reg;
	p.mem_addr_size = I2C_MEM_ADDR_SIZE_8BIT;
	p.data = &val;
	p.data_size = 1;
	p.timeout_ms = 50;

	return bk_i2c_memory_write(s_bus[ACC_BUS].id, &p);
}

/* 读一帧 XYZ，返回 raw（12bit，左对齐）和换算后的 mg */
static bk_err_t acc_read_xyz(int16_t *raw, int32_t *mg)
{
	uint8_t b[6];
	bk_err_t ret;

	/* 连续读必须把子地址最高位置 1（LIS2DH/SC7A20 的规矩） */
	ret = acc_read(s_acc_addr, (uint8_t)(ACC_REG_OUT_X_L | 0x80), b, 6);
	if (ret != BK_OK)
		return ret;

	raw[0] = (int16_t)((b[1] << 8) | b[0]);
	raw[1] = (int16_t)((b[3] << 8) | b[2]);
	raw[2] = (int16_t)((b[5] << 8) | b[4]);

	/* 高分辨率 12bit 左对齐：右移 4 位，±2g 时 1mg/LSB */
	for (int i = 0; i < 3; i++)
		mg[i] = (raw[i] >> 4);

	return BK_OK;
}

static int acc_probe(chry_shell_t *csh)
{
	static const uint8_t cand[] = { ACC_ADDR, ACC_ADDR_ALT };
	uint8_t id = 0;
	int i;

	for (i = 0; i < (int)(sizeof(cand) / sizeof(cand[0])); i++) {
		id = 0;
		if (acc_read(cand[i], ACC_REG_CHIP_ID, &id, 1) != BK_OK)
			continue;

		s_acc_addr = cand[i];
		csh_printf(csh, "accel: addr 0x%02X, WHO_AM_I(0x0F)=0x%02X", cand[i], id);
		if (id == ACC_CHIP_ID)
			csh_printf(csh, "  ok (SC7A20)\r\n");
		else
			csh_printf(csh, "  ? expected 0x%02X\r\n", ACC_CHIP_ID);
		return 0;
	}

	csh_printf(csh, "accel: no device at 0x%02X/0x%02X\r\n", ACC_ADDR, ACC_ADDR_ALT);
	return -1;
}

int accel_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	bk_err_t ret;
	int i, n = 1;

	if (argc > 1 && !strcmp(argv[1], "reg")) {
		uint8_t reg, val = 0;

		if (argc < 3) {
			csh_printf(csh, "usage: accel reg <reg>\r\n");
			return -1;
		}
		reg = (uint8_t)strtoul(argv[2], NULL, 16);

		ret = hw_init(ACC_BUS);
		if (ret != BK_OK) {
			csh_printf(csh, "i2c%d init failed: %d\r\n", ACC_BUS, ret);
			return -1;
		}
		if (acc_probe(csh) != 0) {
			hw_deinit(ACC_BUS);
			return -1;
		}
		ret = acc_read(s_acc_addr, reg, &val, 1);
		hw_deinit(ACC_BUS);
		if (ret != BK_OK) {
			csh_printf(csh, "read reg 0x%02X failed: %d\r\n", reg, ret);
			return -1;
		}
		csh_printf(csh, "0x%02X reg 0x%02X = 0x%02X\r\n", s_acc_addr, reg, val);
		return 0;
	}

	if (argc > 1 && !strcmp(argv[1], "help")) {
		csh_printf(csh, "usage:\r\n");
		csh_printf(csh, "  accel                probe + configure + read xyz once\r\n");
		csh_printf(csh, "  accel poll [n]       read xyz n times (100ms)\r\n");
		csh_printf(csh, "  accel reg <reg>      read one register (hex)\r\n");
		return 0;
	}

	if (argc > 1 && !strcmp(argv[1], "poll")) {
		if (argc > 2) {
			n = atoi(argv[2]);
			if (n < 1 || n > 1000)
				n = 8;
		} else {
			n = 8;
		}
	}

	ret = hw_init(ACC_BUS);
	if (ret != BK_OK) {
		csh_printf(csh, "i2c%d init failed: %d\r\n", ACC_BUS, ret);
		return -1;
	}

	if (acc_probe(csh) != 0) {
		hw_deinit(ACC_BUS);
		return -1;
	}

	acc_write(s_acc_addr, ACC_REG_CTRL1, ACC_CTRL1_100HZ_XYZ);
	acc_write(s_acc_addr, ACC_REG_CTRL4, ACC_CTRL4_HR_2G);
	rtos_delay_milliseconds(20);

	for (i = 0; i < n; i++) {
		int16_t raw[3];
		int32_t mg[3];

		ret = acc_read_xyz(raw, mg);
		if (ret != BK_OK) {
			csh_printf(csh, "read xyz failed: %d\r\n", ret);
			break;
		}

		csh_printf(csh, "#%-2d raw=(%6d,%6d,%6d) mg=(%5d,%5d,%5d)\r\n",
		           i, raw[0], raw[1], raw[2], mg[0], mg[1], mg[2]);

		if (i + 1 < n)
			rtos_delay_milliseconds(100);
	}

	hw_deinit(ACC_BUS);
	return 0;
}

/* =====================================================================
 * 五、给其它器件调试命令复用的总线 helper（声明在 ap/cmd_i2c.h）
 *
 * 包一层而不是把 hw_init/hw_deinit 改成全局：那两个函数是本文件内部实现，
 * 改成全局要动十几处调用点；包装后其它命令文件只需 include cmd_i2c.h，
 * 总线状态仍由这里的 s_hw_inited[] 统一管理。
 * ===================================================================== */

bk_err_t i2c_bus_init(int bus)
{
	if (bus < 0 || bus >= BUS_CNT)
		return BK_ERR_PARAM;

	/* 已初始化：正常直接复用；但总线若被卡成 busy（从机拉住 SCL /
	 * 上次传输没做完），先做总线恢复，否则后面的读写全是 -11523。 */
	if (s_hw_inited[bus] && bk_i2c_is_bus_busy(s_bus[bus].id))
		return i2c_bus_clear(bus);

	return hw_init(bus);
}

void i2c_bus_deinit(int bus)
{
	if (bus < 0 || bus >= BUS_CNT)
		return;
	hw_deinit(bus);
}
