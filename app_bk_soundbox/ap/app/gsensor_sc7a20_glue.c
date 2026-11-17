// Copyright 2026
// SC7A20 注册为 SDK g_sensor 框架的驱动实现
//
// SDK 的 g_sensor 框架（ap/components/bk_gsensor/bk_gsensor.c）通过
// `gsensor_device_t` 抽象设备，表里固定 extern 了 `gs_sc7a20`：
//
//     extern const gsensor_device_t gs_sc7a20;
//
// SDK 自带的 gsensor_sc7a20.c 把 I2C 口写死成 I2C_ID_0，并依赖 INT 引脚 +
// 450ms 定时器批量读 FIFO；本板的 SC7A20 挂在 **I2C1（P42/P43）**，所以这里
// 提供同名 `gs_sc7a20` 的定义：
//   - 寄存器操作复用已验证的 `bsp_sc7a20_*`（I2C1、0x18/0x19 自探测）
//   - **运动中断驱动**：SC7A20 的 AOI1 中断 -> INT1(P55) 上升沿 -> GPIO ISR
//     计数 -> 10ms 轻量任务取事件并回调（不做 40Hz XYZ 轮询，CPU/I2C 近零）
//
// 因此 **不要** 同时打开 CONFIG_GSENSOR_SC7A20_ENABLE（会符号重定义）。

#include <components/bk_gsensor.h>

#include <driver/gpio.h>
#include "gpio_driver.h"      /* gpio_dev_unmap() */
#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#include "bsp.h"
#include "bsp_sc7a20.h"

#define TAG "gs_sc7a20"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define GLUE_TICK_MS      10      /* 任务只查中断计数，不做 I2C 轮询 */
#define GLUE_STACK        1536
#define GLUE_PRIO         (BEKEN_DEFAULT_WORKER_PRIORITY - 1)

/* 运动中断阈值：1 LSB ≈ 16mg @±2g，128mg 能滤掉桌面振动/静置噪声，
 * 又足以在"拿起/摇动"时触发。持续时间 1 个采样周期(~10ms)防瞬时尖峰。 */
#define GLUE_MOTION_THS   8
#define GLUE_MOTION_DUR   1

/* 运动事件回调（非 gsensor_device_t 标准接口，本工程扩展）：
 * events 为自上次回调以来累计的中断次数。 */
typedef void (*gs_motion_cb_t)(uint32_t events);

static gsensor_cb s_cb;
static gs_motion_cb_t s_motion_cb;
static volatile bool s_running;
static volatile bool s_stop;
static beken_thread_t s_task;
static gsensor_dr_t s_dr = GSENSOR_DR_100HZ;
static gsensor_range_t s_range = GSENSOR_RANGE_2G;

static volatile uint32_t s_irq_cnt;      /* ISR 累加 */
static uint32_t s_irq_seen;              /* 任务已处理 */

/* SDK 框架（bk_gsensor.c）extern 的就是这个名字，定义在文件末尾 */
extern const gsensor_device_t gs_sc7a20;

static void glue_close(void);   /* glue_deinit 里先用到 */

/* 单样本回调载荷：count=1 + 一个 xyz。布局必须与 gsensor_data_t 一致
 * （count 后紧跟 xyz[]），否则上层按数组下标取会读到垃圾。 */
typedef struct {
	uint32_t count;
	gsensor_xyz_t xyz[1];
} glue_sample_t;

static bsp_sc7a20_odr_t to_bsp_odr(gsensor_dr_t dr)
{
	switch (dr) {
	case GSENSOR_DR_1HZ:   return BSP_SC7A20_ODR_1HZ;
	case GSENSOR_DR_10HZ:  return BSP_SC7A20_ODR_10HZ;
	case GSENSOR_DR_25HZ:  return BSP_SC7A20_ODR_25HZ;
	case GSENSOR_DR_50HZ:  return BSP_SC7A20_ODR_50HZ;
	case GSENSOR_DR_100HZ: return BSP_SC7A20_ODR_100HZ;
	case GSENSOR_DR_200HZ: return BSP_SC7A20_ODR_200HZ;
	case GSENSOR_DR_400HZ: return BSP_SC7A20_ODR_400HZ;
	default:               return BSP_SC7A20_ODR_100HZ;
	}
}

static bsp_sc7a20_range_t to_bsp_range(gsensor_range_t rg)
{
	switch (rg) {
	case GSENSOR_RANGE_2G:  return BSP_SC7A20_RANGE_2G;
	case GSENSOR_RANGE_4G:  return BSP_SC7A20_RANGE_4G;
	case GSENSOR_RANGE_8G:  return BSP_SC7A20_RANGE_8G;
	case GSENSOR_RANGE_16G: return BSP_SC7A20_RANGE_16G;
	default:                return BSP_SC7A20_RANGE_2G;
	}
}

static void glue_isr(gpio_id_t id)
{
	(void)id;
	s_irq_cnt++;             /* ISR 里只累加，清中断/回调放到任务里 */
}

static void glue_task(void *arg)
{
	uint32_t events;

	(void)arg;

	while (!s_stop) {
		events = s_irq_cnt;
		if (events != s_irq_seen) {
			uint32_t delta = events - s_irq_seen;   /* 无符号回绕安全 */

			s_irq_seen = events;

			/* LIR 锁存模式：读 INT1_SRC 清掉锁存，否则不会再产生新中断 */
			bsp_sc7a20_int_src_read(NULL);

			if (s_motion_cb)
				s_motion_cb(delta);
		}

		rtos_delay_milliseconds(GLUE_TICK_MS);
	}

	s_running = false;
	s_task = NULL;
	rtos_delete_thread(NULL);
}

static int glue_init(void)
{
	if (bsp_sc7a20_init() != BK_OK) {
		LOGE("sc7a20 init failed (I2C1 P42/P43, addr 0x18/0x19)\r\n");
		return -1;
	}
	s_dr = GSENSOR_DR_100HZ;
	s_range = GSENSOR_RANGE_2G;
	bsp_sc7a20_set_range(BSP_SC7A20_RANGE_2G);
	bsp_sc7a20_set_odr(BSP_SC7A20_ODR_100HZ);
	return 0;
}

static void glue_deinit(void)
{
	glue_close();
	bsp_sc7a20_deinit();
}

static int glue_open(void)
{
	gpio_config_t cfg = {
		.io_mode = GPIO_INPUT_ENABLE,
		.pull_mode = GPIO_PULL_DISABLE,
		.func_mode = GPIO_SECOND_FUNC_DISABLE,
	};

	if (s_running)
		return 0;

	if (bsp_sc7a20_motion_int_enable(GLUE_MOTION_THS, GLUE_MOTION_DUR) != BK_OK) {
		LOGE("motion int enable failed\r\n");
		return -1;
	}

	/* INT1 引脚：GPIO 输入 + 上升沿中断（SC7A20 INT 高有效） */
	gpio_dev_unmap(BOARD_SC7A20_INT1_PIN);
	bk_gpio_set_config(BOARD_SC7A20_INT1_PIN, &cfg);
	bk_gpio_register_isr(BOARD_SC7A20_INT1_PIN, glue_isr);
	bk_gpio_set_interrupt_type(BOARD_SC7A20_INT1_PIN, GPIO_INT_TYPE_RISING_EDGE);
	bk_gpio_enable_interrupt(BOARD_SC7A20_INT1_PIN);

	s_irq_cnt = 0;
	s_irq_seen = 0;
	s_stop = false;
	if (rtos_create_thread(&s_task, GLUE_PRIO, "gs_sc7a20",
	                       (beken_thread_function_t)glue_task, GLUE_STACK,
	                       NULL) != kNoErr) {
		LOGE("create event task failed\r\n");
		s_task = NULL;
		bk_gpio_disable_interrupt(BOARD_SC7A20_INT1_PIN);
		bsp_sc7a20_motion_int_disable();
		return -1;
	}
	s_running = true;
	LOGI("motion int on INT1(P%d), ths=%u(~%umg)\r\n",
	     (int)BOARD_SC7A20_INT1_PIN, GLUE_MOTION_THS,
	     (unsigned)GLUE_MOTION_THS * 16);
	return 0;
}

static void glue_close(void)
{
	if (!s_running)
		return;

	bk_gpio_disable_interrupt(BOARD_SC7A20_INT1_PIN);
	bk_gpio_set_interrupt_type(BOARD_SC7A20_INT1_PIN, GPIO_INT_TYPE_LOW_LEVEL);
	bsp_sc7a20_motion_int_disable();

	s_stop = true;
	/* 事件任务 10ms 一个循环，最多等 100ms 让它自己退出 */
	for (int i = 0; i < 100 && s_running; i++)
		rtos_delay_milliseconds(2);
	bsp_sc7a20_set_odr(BSP_SC7A20_ODR_POWER_DOWN);
}

static int glue_set_datarate(gsensor_dr_t dr)
{
	s_dr = dr;
	return bsp_sc7a20_set_odr(to_bsp_odr(dr));
}

static int glue_set_mode(gsensor_mode_t mode)
{
	/* 本板没接 INT 引脚，低功耗唤醒模式不可用；NORMAL 恒成立 */
	return (mode == GSENSOR_MODE_NOMAL) ? 0 : -1;
}

static int glue_set_range(gsensor_range_t rg)
{
	s_range = rg;
	return bsp_sc7a20_set_range(to_bsp_range(rg));
}

static int glue_register_cb(gsensor_cb cb)
{
	s_cb = cb;
	return 0;
}

/* 本工程扩展：注册运动中断事件回调（不是 gsensor_device_t 标准数据回调） */
void gs_sc7a20_set_motion_cb(gs_motion_cb_t cb)
{
	s_motion_cb = cb;
}

const gsensor_device_t gs_sc7a20 = {
	.name            = "sc7a20",
	.init            = glue_init,
	.deinit          = glue_deinit,
	.open            = glue_open,
	.close           = glue_close,
	.setDatarate     = glue_set_datarate,
	.setMode         = glue_set_mode,
	.setDataRange    = glue_set_range,
	.registerCallback = glue_register_cb,
};
