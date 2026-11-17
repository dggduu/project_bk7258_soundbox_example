// Copyright 2026
// CherrySH 命令：camera —— DVP 摄像头出图测试（OV3660）
//
//   camera                 状态（sensor / 分辨率 / 帧计数 / 屏与 SD 状态）
//   camera show            拍一帧 -> 缩放到 360x360 -> 显示到 QSPI 屏
//   camera save [name]     拍一帧 -> 存 24bit BMP 到 SD 卡（默认 1:/cam.bmp）
//   camera size <w> <h>    设置采集分辨率（默认 320x240）
//   camera on|off          显式打开/关闭 DVP
//   camera help
//
// 与 ov3660 命令的区别：
//   `ov3660` 是**寄存器级**调试（SCCB 读写、PID、初始化表），不开 DVP 通路；
//   `camera` 走官方 bk_camera 组件（bk_camera_dvp_ctlr_new/open），真的出图。
//
// 前置条件：
//   1. `CONFIG_DVP_CAMERA=y` + `CONFIG_YUV_BUF=y`（本工程已开），
//      否则 bk_dvp 组件不编译、dvp_camera_i2c_* 也没有；
//   2. 显示要 `lcdq init` 过；保存要先 `sd list` 挂载 FATFS 到 1:。
//
// PSRAM 预算（AP_PSRAM_HEAP = 640KB）：
//   lcdq 帧缓冲 259200B + 640x480 YUYV 614400B 走官方 display 池(5.5MB)；
//
// 帧所有权（照官方 dvp_example）：
//   malloc  -> dvp_frame_queue_malloc()
//   complete-> 好帧 dvp_frame_queue_complete()（进 ready 队列），
//              坏帧 dvp_frame_queue_free()
//   命令侧  -> dvp_frame_queue_get_frame() 取，用完 dvp_frame_queue_free()。
//   队列自带 free/ready 限流，不需要自己维护"最新一帧"指针（SMP 竞态源）。

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_board_cfg.h"
#include "bsp_power.h"
#include "bsp_ov3660.h"
#include "cmd_lcdq.h"

#include <components/dvp_camera.h>
#include <components/dvp_camera_types.h>
#include <components/media_types.h>
#include <components/bk_camera_ctlr.h>   /* 官方 bk_camera 控制层 */
#include "dvp_frame_list.h"              /* 官方 dvp_example 帧队列 */
#include "cmd_i2c.h"            /* i2c_bus_force_rebuild / get_baud */
#include "gpio_driver.h"        /* gpio_dev_unmap：camera pins 用 */
#include <driver/gpio.h>
#include <driver/yuv_buf.h>     /* bk_yuv_buf_set_mclk_div：DVP 减速 */
/* 注意 SDK 缺陷：image_convert.h 只声明函数、不 include codec_img_t 的定义头，
 * 必须先包含 image_scale_types.h（codec_img_t / PIXEL_MSB_* / media_rotate_t） */
#include <modules/image_scale_types.h>
#include "frame_buffer.h"          /* SDK 官方帧缓冲池（PSRAM display 分区，32 对齐） */
#include <psram_mem_slab.h>        /* bk_psram_frame_buffer_init：池初始化 */
/* dvp_camera_io_init 声明在 bk_dvp 组件的 dvp_private.h（Armino 对该组件的
 * include 目录传播不可靠，这里直接 extern；符号在 libbk_dvp.a 里） */
extern void dvp_camera_io_init(bk_dvp_io_config_t *io_config);
/* cmd_ov3660.c 的 XCLK 输出（CLK_AUXS_CIS 复用，probe 同款路径） */
extern int ov_mclk_enable(int mhz);
#include "image_convert.h"      /* SDK 官方 YUYV->RGB565 + resize */
#include <os/os.h>
#include <os/mem.h>
#include "ff.h"

#define CAM_W_DEF       640
#define CAM_H_DEF       480   /* = sensor def_ppi；YUV 帧走官方 display 池(5.5MB) */
#define CAM_FPS_DEF     FPS15
/* esp32-camera 的 OV3660 **YUV** 模式 PLL 是按 16MHz XCLK 调的
 * （ov3660.c: "tuned for 16MHz XCLK and 8MHz PCLK"），bsp_ov3660_set_ppi()
 * 里的 PLL 分频也按 16MHz 算，两边必须一致，否则 PCLK 偏 25%。 */
#define CAM_MCLK_DEF    MCLK_16M

#define CAM_WAIT_MS     3000
#define LCD_W           360
#define LCD_H           360
#define LCD_FRAME_BYTES (LCD_W * LCD_H * 2)

static bk_camera_ctlr_handle_t s_cam;
static bool             s_opened;
static uint16_t         s_w = CAM_W_DEF;
static uint16_t         s_h = CAM_H_DEF;

/* 速度可调（本板信号完整性差）：
 *   s_i2c_hz : DVP 通路 SCCB 波特率（bk_dvp_open 内部 bk_i2c_init 用），
 *              ov3660 命令走 i2c_bus_set_baud()，两者互相独立；
 *   s_mclk   : MCLK 频率枚举，bk_dvp_open 内部 dvp_camera_mclk_enable 用，
 *              PCLK 与 MCLK 同比例，降 MCLK = 降 DVP 数据带宽。 */
static uint32_t         s_i2c_hz = 100000;
static mclk_freq_t      s_mclk   = CAM_MCLK_DEF;
/* yuv_buf 模块时钟分频：SDK 默认 /3。改大（4/6）就是"DVP 减速"，
 * 用来验证 0 帧是不是采样时钟太快/信号完整性导致。 */
static mclk_div_t       s_yuv_div = YUV_MCLK_DIV_3;
/* DVP 同步极性：yuv_buf 报"分辨率不对"时，用 camera sync 逐个试 4 种组合 */
static sync_level_t     s_vsync = SYNC_HIGH_LEVEL;
static sync_level_t     s_hsync = SYNC_HIGH_LEVEL;
/* sensor PCLK 分频（0=按分辨率自动）。本板信号差时用来降线上数据速率。 */
static uint8_t          s_pclk_div;
/* 屏幕 RGB565 高低字节交换。本板实测要用 swap=1 颜色才对（配合 UYVY），
 * 所以默认打开；camera swap 0/1 可切。 */
static bool             s_swap = true;
/* true = 中心裁 360x360 + CPU 转 RGB565（默认，可靠）。
 * false = 走 SDK image_convert(if_resize=1)，注意该分支实际不做转换。 */
static bool             s_crop = true;
/* sensor YUV422 的字节序。实测本 OV3660 输出 **UYVY**（U Y V Y）：
 * 按 UYVY 解 U/V 均值≈127（中性）、画面正常；按 YUYV 解会把亮度当色度，
 * 整片紫红（用户说的"红外相机"效果）。 */
static img_fmt          s_yuv_fmt = PIXEL_MSB_UYVY;
static bool             s_fb_pool_inited;/* 官方 PSRAM 帧池只初始化一次 */

static volatile uint32_t s_seq;               /* 收到帧计数 */
static volatile uint32_t s_drop;              /* 被丢弃的帧数 */

/* ---------------- camera 引脚（虚焊排查用） ----------------
 * MCLK/PCLK/HSYNC/VSYNC/D0~D7 这 11 根线是 DVP 通路的全部信号线。
 * `camera pins high|low` 把它们临时切成 GPIO 推挽输出，方便拿万用表
 * 在模组端/排线端逐根量通断（注意先 `ov3660 off` 给模组断电，避免两边
 * 对着驱动）。 */
static const struct {
	uint8_t pin;
	const char *name;
} s_cam_pins[] = {
	{ BOARD_CIS_MCLK_PIN,  "MCLK"  },
	{ BOARD_CIS_PCLK_PIN,  "PCLK"  },
	{ BOARD_CIS_HSYNC_PIN, "HSYNC" },
	{ BOARD_CIS_VSYNC_PIN, "VSYNC" },
	{ BOARD_CIS_D0_PIN,    "D0"    },
	{ BOARD_CIS_D1_PIN,    "D1"    },
	{ BOARD_CIS_D2_PIN,    "D2"    },
	{ BOARD_CIS_D3_PIN,    "D3"    },
	{ BOARD_CIS_D4_PIN,    "D4"    },
	{ BOARD_CIS_D5_PIN,    "D5"    },
	{ BOARD_CIS_D6_PIN,    "D6"    },
	{ BOARD_CIS_D7_PIN,    "D7"    },
};
#define CAM_PIN_CNT ((int)(sizeof(s_cam_pins) / sizeof(s_cam_pins[0])))

static int yuv_div_to_n(mclk_div_t d);

/* 转换直接写 lcdq 的帧缓冲（lcdq_frame_buf()），不再自备 259KB 中间缓冲：
 * PSRAM heap 640KB 里 lcdq 259200 + 320x240 YUV 153600 = 412KB 才放得下 */

/* ---------------- DVP 回调（照官方 dvp_example） ----------------
 *
 * 官方做法（projects/dvp_example/ap/dvp_test/src/dvp_api_test.c）：
 *   malloc   -> dvp_frame_queue_malloc()
 *   complete -> 好帧 dvp_frame_queue_complete()（进 ready 队列）
 *               坏帧 dvp_frame_queue_free()
 * 消费者用 dvp_frame_queue_get_frame() 取、dvp_frame_queue_free() 还。
 * 队列本身负责限流/引用，避免了之前自己维护 s_frame 在 SMP 下的竞态。 */
static frame_buffer_t *cam_malloc(image_format_t format, uint32_t size)
{
	frame_buffer_t *frame = dvp_frame_queue_malloc(format, size);

	if (frame) {
		frame->sequence = 0;
		frame->length = 0;
		frame->timestamp = 0;
		frame->size = size;
	}

	return frame;
}

static void cam_complete(image_format_t format, frame_buffer_t *frame, int result)
{
	if (!frame)
		return;

	if (result != 0) {
		s_drop++;
		dvp_frame_queue_free(format, frame);
	} else {
		s_seq++;
		dvp_frame_queue_complete(format, frame);
	}
}

static const bk_dvp_callback_t s_cb = {
	.malloc   = cam_malloc,
	.complete = cam_complete,
};

/* ---------------- 打开 / 关闭 ---------------- */

/* mclk_freq_t 是顺序枚举（15M=0..48M=7），打印要换算回 MHz */
static int cam_mclk_to_mhz(mclk_freq_t m)
{
	switch (m) {
	case MCLK_15M: return 15;
	case MCLK_16M: return 16;
	case MCLK_20M: return 20;
	case MCLK_24M: return 24;
	case MCLK_30M: return 30;
	case MCLK_32M: return 32;
	case MCLK_40M: return 40;
	case MCLK_48M: return 48;
	default:       return 0;
	}
}

static const uint8_t s_data_pins[8] = {
	BOARD_CIS_D0_PIN, BOARD_CIS_D1_PIN, BOARD_CIS_D2_PIN, BOARD_CIS_D3_PIN,
	BOARD_CIS_D4_PIN, BOARD_CIS_D5_PIN, BOARD_CIS_D6_PIN, BOARD_CIS_D7_PIN,
};

static bk_err_t cam_open(chry_shell_t *csh)
{
	bk_dvp_config_t cfg;
	bk_err_t ret;

	if (s_opened)
		return BK_OK;

	/* 官方 PSRAM 帧池初始化（bk_dvp_open 的 malloc 回调从
	 * PSRAM_MEM_SLAB_DISPLAY 分区分配）：不初始化时 heap_size==0，
	 * bk_psram_frame_buffer_malloc 里 BK_ASSERT 直接整机复位。
	 * 幂等保护：只跑一次（运行中重建会丢未释放块）。 */
	if (!s_fb_pool_inited) {
		bk_psram_frame_buffer_init();
		s_fb_pool_inited = true;
	}

	ret = bsp_power_camera_on();
	if (ret != BK_OK) {
		csh_printf(csh, "camera power on failed: %d\r\n", ret);
		return ret;
	}

	/* 用自定义列表覆盖 SDK 默认 sensor 列表（不改 SDK 源码） */
	{
		static const dvp_sensor_config_t *list[] = { &bsp_sensor_ov3660 };
		bk_dvp_set_devices_list(list, 1);
	}

	os_memset(&cfg, 0, sizeof(cfg));

	cfg.i2c_config.id        = BOARD_CAM_I2C_ID;
	cfg.i2c_config.scl_pin   = BOARD_CAM_I2C_SCL_PIN;
	cfg.i2c_config.sda_pin   = BOARD_CAM_I2C_SDA_PIN;
	cfg.i2c_config.baud_rate = s_i2c_hz;

	/* SDK 约定 0xFF = "无引脚"：dvp_sensor_reset(uint8_t,...) 里按 !=0xFF
	 * 判断。GPIO_NUM 是枚举值(=56)，截断成 uint8_t 后仍非 0xFF，会让 SDK
	 * 对越界的 P56 做 unmap/输出操作 —— 之前传 GPIO_NUM 是错的。 */
	cfg.reset_pin = 0xFF;           /* 本板没有独立 RST，靠断电 */
	cfg.pwdn_pin  = 0xFF;

	cfg.io_config.data_width = SENSOR_BITS_WIDTH_8BIT;
	for (int i = 0; i < 8; i++)
		cfg.io_config.data_pin[i] = s_data_pins[i];
	cfg.io_config.vsync_pin = BOARD_CIS_VSYNC_PIN;
	cfg.io_config.hsync_pin = BOARD_CIS_HSYNC_PIN;
	cfg.io_config.xclk_pin  = BOARD_CIS_MCLK_PIN;
	cfg.io_config.pclk_pin  = BOARD_CIS_PCLK_PIN;

	cfg.clk_source = s_mclk;
	cfg.img_format = IMAGE_YUV;
	cfg.width      = s_w;
	cfg.height     = s_h;
	cfg.fps        = CAM_FPS_DEF;

	s_seq = 0;
	s_drop = 0;

	/* 官方帧队列（dvp_example/dvp_frame_list）：建 free/ready 队列，幂等 */
	dvp_frame_queue_init_all();

	/* 关键：先把我们这侧的 I2C0 状态拆掉。bk_dvp_open 内部会 bk_i2c_init，
	 * 若控制器已被我们 hw_init 过，重复 init 会破坏 P20/P21 复用
	 * （CONFIG_GPIO_DEFAULT_SET_SUPPORT=y 下 SDK 不重新映射引脚），
	 * 之后整条总线无应答。deinit 后 SDK 的 init 是干净的首次初始化；
	 * 成功/失败路径随后都由 force_rebuild 恢复引脚映射。 */
	i2c_bus_deinit(0);

	/* SCCB 速率同步给 sensor 驱动：它在 busy/超时后会用 SDK 的
	 * bk_i2c_init() 重建控制器，重建时的 baud 必须和这里一致。 */
	bsp_ov3660_set_sccb_baud(s_i2c_hz);

	/* 同步极性覆盖（camera sync 可改，下次打开生效） */
	bsp_ov3660_set_sync(s_vsync, s_hsync);

	/* DVP 数据脚映射：CONFIG_GPIO_DEFAULT_SET_SUPPORT=y 时
	 * bk_dvp_detect 里的 dvp_camera_io_init 被 #ifndef 裁掉，
	 * 而 usr_gpio_cfg 里 P29~P39 是 INVALID，必须自己调 SDK 函数映射，
	 * 否则 DVP 控制器收不到任何数据（实测 dvp:0[0 0KB 0Kbps]） */
	dvp_camera_io_init(&cfg.io_config);

	/* 照官方 dvp_example：通过 bk_camera 组件打开（内部就是 bk_dvp_open，
	 * 额外做了 ctlr 句柄/编码 buffer/生命周期管理）。 */
	{
		bk_dvp_ctlr_config_t ctl_cfg;

		ctl_cfg.config = cfg;
		ctl_cfg.cbs = &s_cb;

		ret = bk_camera_dvp_ctlr_new(&s_cam, &ctl_cfg);
		if (ret == AVDK_ERR_OK)
			ret = bk_camera_open(s_cam);
	}
	if (ret != AVDK_ERR_OK) {
		csh_printf(csh, "bk_camera open failed: %d\r\n", ret);
		csh_printf(csh, "  run 'camera detect' to split detect vs yuv_mode failure\r\n");
		csh_printf(csh, "  restoring i2c0 (bk_dvp_open touched it) ...\r\n");
		if (s_cam) {
			bk_camera_delete(s_cam);
			s_cam = NULL;
		}
		i2c_bus_force_rebuild(0);
		bsp_power_camera_off();
		return ret;
	}

	s_opened = true;
	/* bk_dvp_open 内部 bk_i2c_init(0) 过了（且没映射引脚），这里按
	 * 本次使用的 SCCB 速率重建一次：既把 P20/P21 复用恢复，也让后续
	 * ov3660 命令用和摄像头一致的 baud。 */
	i2c_bus_set_baud(0, s_i2c_hz);
	i2c_bus_force_rebuild(0);

	/* 应用 yuv_buf 模块时钟分频（SDK 默认 /3，camera div 可改）。
	 * 此时 yuv_buf 已由 bk_dvp_open 初始化，寄存器可立即生效。 */
	if (bk_yuv_buf_set_mclk_div(s_yuv_div) != BK_OK)
		csh_printf(csh, "warn: set yuv_buf mclk_div failed\r\n");

	csh_printf(csh, "camera opened: %s %ux%u YUV, mclk=%dMHz sccb=%uHz "
	           "yuv_div=/%d\r\n",
	           bsp_sensor_ov3660.name, (unsigned)s_w, (unsigned)s_h,
	           cam_mclk_to_mhz(s_mclk), (unsigned)s_i2c_hz,
	           yuv_div_to_n(s_yuv_div));
	return BK_OK;
}

static void cam_close(chry_shell_t *csh)
{
	if (!s_opened)
		return;

	/* 官方 bk_camera 生命周期：close -> delete（delete 内部释放 ctlr） */
	if (s_cam) {
		bk_camera_close(s_cam);
		bk_camera_delete(s_cam);
		s_cam = NULL;
	}
	s_opened = false;

	bk_dvp_set_devices_list(NULL, 0);
	bsp_power_camera_off();

	if (csh)
		csh_printf(csh, "camera closed\r\n");
}

/* 等一帧（超时返回 NULL）。取走后调用方负责 dvp_frame_queue_free()。 */
static frame_buffer_t *cam_grab(chry_shell_t *csh, uint32_t timeout_ms)
{
	frame_buffer_t *f = dvp_frame_queue_get_frame(IMAGE_YUV, timeout_ms);
	frame_buffer_t *n;

	if (!f) {
		csh_printf(csh, "timeout waiting frame (%ums), seq=%u drop=%u\r\n",
		           (unsigned)timeout_ms, (unsigned)s_seq, (unsigned)s_drop);
		return NULL;
	}

	/* ready 队列里可能压着好几帧（最多 3 帧）。取最旧的一帧显示会"慢半拍"，
	 * 运动物体看起来就像残影。这里把旧的都丢掉，只保留最新一帧。 */
	while ((n = dvp_frame_queue_get_frame(IMAGE_YUV, 0)) != NULL) {
		dvp_frame_queue_free(IMAGE_YUV, f);
		f = n;
		s_drop++;
	}

	return f;
}

/* ---------------- 像素转换 ---------------- */

/* YUYV -> RGB565 + 缩放：用 SDK 的 image_convert 组件（codec_img_t），
 * 不再手写转换（遵循"封装高的 SDK 尽量用组件"的原则）。
 * SDK 输出的 RGB565 字节序未验证，若屏上颜色错乱用 'camera swap 1' 交换。 */
/* YUV422(YUYV 排列) 单像素 -> RGB565，BT.601 limited range */
static inline uint16_t cam_yuv2rgb565(uint8_t y, uint8_t u, uint8_t v)
{
	int c = (int)y - 16;
	int d = (int)u - 128;
	int e = (int)v - 128;
	int r = (298 * c + 409 * e + 128) >> 8;
	int g = (298 * c - 100 * d - 208 * e + 128) >> 8;
	int b = (298 * c + 516 * d + 128) >> 8;

	if (r < 0) r = 0; else if (r > 255) r = 255;
	if (g < 0) g = 0; else if (g > 255) g = 255;
	if (b < 0) b = 0; else if (b > 255) b = 255;

	return (uint16_t)(((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3));
}

/* 中心裁 360x360 + CPU 转 RGB565。
 *
 * 不用 SDK 的 image_convert(if_resize=1)：那个分支只把 new_width/new_height
 * 拷回结构体，**根本没做像素转换**（image_convert.c 里 resize 分支是空的），
 * 所以 camera show 一直"shown"但 LCD 缓冲还是旧的 —— 屏不更新。
 * 裁切而不是缩放，保持 4:3 不拉伸；一次 129600 像素，CPU 几 ms 完成。 */
static int cam_crop_to_lcd(frame_buffer_t *f)
{
	uint16_t *dst = (uint16_t *)lcdq_frame_buf();
	uint32_t x0 = (s_w - LCD_W) / 2;
	uint32_t y0 = (s_h - LCD_H) / 2;

	if (!dst)
		return -1;

	for (uint32_t y = 0; y < LCD_H; y++) {
		const uint8_t *src = f->frame + ((y0 + y) * s_w + x0) * 2;
		uint16_t *d = dst + y * LCD_W;

		for (uint32_t x = 0; x < LCD_W; x += 2) {
			uint8_t b0 = src[0], b1 = src[1], b2 = src[2], b3 = src[3];
			uint8_t yy0, yy1, uu, vv;

			/* 4 字节是 [A Y B Y] 的顺序，A/B 是 U/V；
			 * 实测本 sensor 输出 UYVY（U Y V Y），按 YUYV 解会把
			 * 亮度当色度 -> 整片紫红，像红外相机。 */
			switch (s_yuv_fmt) {
			case PIXEL_MSB_UYVY:
				uu = b0; yy0 = b1; vv = b2; yy1 = b3;
				break;
			case PIXEL_MSB_VUYY:
				vv = b0; uu = b1; yy0 = b2; yy1 = b3;
				break;
			case PIXEL_MSB_VYUY:
				vv = b0; yy0 = b1; uu = b2; yy1 = b3;
				break;
			case PIXEL_MSB_YYUV:
				yy0 = b0; yy1 = b1; uu = b2; vv = b3;
				break;
			case PIXEL_MSB_YUYV:
			default:
				yy0 = b0; uu = b1; yy1 = b2; vv = b3;
				break;
			}

			d[x]     = cam_yuv2rgb565(yy0, uu, vv);
			d[x + 1] = cam_yuv2rgb565(yy1, uu, vv);
			src += 4;
		}
	}

	return 0;
}

static int cam_convert_to_lcd(frame_buffer_t *f)
{
	codec_img_t img;
	uint16_t *dst = (uint16_t *)lcdq_frame_buf();

	if (!dst)
		return -1;

	/* 默认走 CPU 裁切转换（可靠）；camera crop 0 才用 SDK 的 image_convert */
	if (s_crop && s_w >= LCD_W && s_h >= LCD_H)
		return cam_crop_to_lcd(f);

	os_memset(&img, 0, sizeof(img));
	img.src_buf    = f->frame;
	img.dst_buf    = (uint8_t *)dst;
	img.input_type  = s_yuv_fmt;
	img.output_type = PIXEL_MSB_RGB565;
	img.width      = s_w;
	img.height     = s_h;
	img.rotate     = ROTATE_NONE;
	img.if_resize  = 1;
	img.new_width  = LCD_W;
	img.new_height = LCD_H;

	/* image_convert 返回 bk_err_t：BK_OK 成功，BK_ERR_NOT_SUPPORT 等 */
	{
		bk_err_t r = image_convert(&img);
		return (r == BK_OK) ? 0 : (int)r;
	}
}

/* SDK 输出小端 uint16 时，QSPI 屏需要大端字节 -> 逐像素交换 */
static void cam_swap_bytes(void)
{
	uint16_t *p = (uint16_t *)lcdq_frame_buf();

	if (!p)
		return;
	for (int i = 0; i < LCD_W * LCD_H; i++) {
		uint16_t c = p[i];
		p[i] = (uint16_t)((c >> 8) | (c << 8));
	}
}

/* ---------------- BMP 落盘 ---------------- */

static void put_u32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
	p[2] = (uint8_t)(v >> 16);
	p[3] = (uint8_t)(v >> 24);
}

static void put_u16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)v;
	p[1] = (uint8_t)(v >> 8);
}

/* 24bit BMP（top-down，行宽 360*3=1080 已是 4 的倍数，无需 padding） */
static FATFS s_cam_fatfs;

/* 'sd'/'sd list' 是 mount -> 列举 -> unmount 的，命令返回后 1: 处于未挂载
 * 状态，直接 f_open 会得到 FR_NOT_ENABLED(12)。这里自己挂一次，用完摘掉。 */
static FRESULT cam_sd_mount(void)
{
	return f_mount(&s_cam_fatfs, "1:", 1);
}

static int write_bmp(chry_shell_t *csh, const char *path,
                     const uint16_t *rgb565_be, int w, int h)
{
	uint8_t hdr[54];
	FIL f;
	FRESULT fr;
	UINT bw;
	uint32_t row_bytes = (uint32_t)w * 3;
	uint8_t *row;

	os_memset(hdr, 0, sizeof(hdr));
	hdr[0] = 'B';
	hdr[1] = 'M';
	put_u32(hdr + 2, 54 + row_bytes * (uint32_t)h);
	put_u32(hdr + 10, 54);
	put_u32(hdr + 14, 40);
	put_u32(hdr + 18, (uint32_t)w);
	put_u32(hdr + 22, (uint32_t)(-(int32_t)h));   /* 负数 = top-down */
	put_u16(hdr + 26, 1);
	put_u16(hdr + 28, 24);
	put_u32(hdr + 34, row_bytes * (uint32_t)h);
	put_u32(hdr + 38, 2835);
	put_u32(hdr + 42, 2835);

	row = (uint8_t *)psram_malloc(row_bytes);
	if (!row) {
		csh_printf(csh, "bmp row buf malloc failed\r\n");
		return -1;
	}

	fr = cam_sd_mount();
	if (fr != FR_OK) {
		csh_printf(csh, "sd mount failed: %d (run 'sd' first)\r\n", fr);
		psram_free(row);
		return -1;
	}

	fr = f_open(&f, path, FA_WRITE | FA_CREATE_ALWAYS);
	if (fr != FR_OK) {
		csh_printf(csh, "f_open(%s) failed: %d (mount with 'sd list' first)\r\n",
		           path, fr);
		f_unmount(1, "1:", 0);
		psram_free(row);
		return -1;
	}

	fr = f_write(&f, hdr, sizeof(hdr), &bw);
	if (fr == FR_OK) {
		for (int y = 0; y < h && fr == FR_OK; y++) {
			const uint16_t *src = rgb565_be + (uint32_t)y * w;

			for (int x = 0; x < w; x++) {
				uint16_t c = src[x];        /* 存储为大端：读出来直接是 RGB565 */
				uint8_t r = (uint8_t)((c >> 11) & 0x1F);
				uint8_t g = (uint8_t)((c >> 5) & 0x3F);
				uint8_t b = (uint8_t)(c & 0x1F);

				row[x * 3 + 0] = (uint8_t)((b << 3) | (b >> 2));
				row[x * 3 + 1] = (uint8_t)((g << 2) | (g >> 4));
				row[x * 3 + 2] = (uint8_t)((r << 3) | (r >> 2));
			}
			fr = f_write(&f, row, row_bytes, &bw);
		}
	}

	f_close(&f);
	f_unmount(1, "1:", 0);
	psram_free(row);

	if (fr != FR_OK) {
		csh_printf(csh, "f_write failed: %d\r\n", fr);
		return -1;
	}

	csh_printf(csh, "saved %s  %dx%d 24bit BMP  %u bytes\r\n",
	           path, w, h, (unsigned)(54 + row_bytes * (uint32_t)h));
	return 0;
}

/* ---------------- 命令 ---------------- */

static void cam_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  camera              status\r\n");
	csh_printf(csh, "  camera show         capture 1 frame -> %dx%d RGB565 -> lcd\r\n", LCD_W, LCD_H);
	csh_printf(csh, "  camera save [name]  capture 1 frame -> BMP on SD (default 1:/cam.bmp)\r\n");
	csh_printf(csh, "  camera raw [name]   capture 1 frame -> raw YUYV on SD (default 1:/cam.yuv)\r\n");
	csh_printf(csh, "  camera dump         取一帧打印元数据+每行头 16 字节（查 DVP 是否写入）\r\n");
	csh_printf(csh, "  camera yuvfmt <fmt> sensor YUV 字节序 yuyv|uyvy|yyuv|vyuy|vuyy\r\n");
	csh_printf(csh, "  camera crop [0|1]   1=中心裁 360x360 1:1 转换（跳过 SDK 缩放）\r\n");
	csh_printf(csh, "  camera size <w> <h> set capture size (default %dx%d)\r\n", CAM_W_DEF, CAM_H_DEF);
	csh_printf(csh, "  camera mclk [mhz]   show / set MCLK (default 16; sensor PLL 按 16MHz 调)\r\n");
	csh_printf(csh, "  camera div [2|3|4|6] yuv_buf 模块时钟分频（越大 DVP 越慢，默认 3）\r\n");
	csh_printf(csh, "  camera pclk [0|1..31] sensor PCLK 分频（越大线上速率越低，默认 2）\r\n");
	csh_printf(csh, "  camera sync [v] [h] DVP vsync/hsync 极性 high|low（默认 high high）\r\n");
	csh_printf(csh, "  camera pins high|low|z 关掉摄像头后把 MCLK/PCLK/HSYNC/VSYNC/D0..D7 拉高/拉低/释放\r\n");
	csh_printf(csh, "  camera sccb [hz]    show / set dvp-path SCCB baud (default 100k)\r\n");
	csh_printf(csh, "  camera detect       run SDK bk_dvp_detect only (diagnose open fail)\r\n");
	csh_printf(csh, "  camera swap [0|1]   toggle RGB565 byte order for lcd output\r\n");
	csh_printf(csh, "  camera on|off       open / close dvp\r\n");
	csh_printf(csh, "pre: 'lcdq init' before show, 'sd list' before save\r\n");
}

static int yuv_div_to_n(mclk_div_t d)
{
	switch (d) {
	case YUV_MCLK_DIV_2: return 2;
	case YUV_MCLK_DIV_3: return 3;
	case YUV_MCLK_DIV_4: return 4;
	case YUV_MCLK_DIV_6: return 6;
	default:             return 0;
	}
}

/* camera div [2|3|4|6]：yuv_buf 模块时钟分频。
 * SDK 默认 DIV_3；改成 4/6 相当于给 DVP 输入通路减速，用来验证
 * 0 帧 / 花屏是不是采样时钟太快或排线信号完整性导致。 */
static int cam_div_cmd(chry_shell_t *csh, int argc, char **argv)
{
	mclk_div_t d;
	int n;

	if (argc < 3) {
		csh_printf(csh, "yuv_buf mclk_div = /%d (%s)\r\n",
		           yuv_div_to_n(s_yuv_div),
		           s_opened ? "applied" : "apply on next open");
		csh_printf(csh, "  usage: camera div <2|3|4|6>  (越大 DVP 模块时钟越慢，默认 3)\r\n");
		return 0;
	}

	n = atoi(argv[2]);
	switch (n) {
	case 2: d = YUV_MCLK_DIV_2; break;
	case 3: d = YUV_MCLK_DIV_3; break;
	case 4: d = YUV_MCLK_DIV_4; break;
	case 6: d = YUV_MCLK_DIV_6; break;
	default:
		csh_printf(csh, "invalid div %d (2|3|4|6)\r\n", n);
		return -1;
	}

	s_yuv_div = d;

	if (s_opened) {
		bk_err_t r = bk_yuv_buf_set_mclk_div(d);

		if (r != BK_OK) {
			csh_printf(csh, "set yuv_buf mclk_div failed: %d\r\n", r);
			return -1;
		}
	}

	csh_printf(csh, "yuv_buf mclk_div -> /%d (%s)\r\n", n,
	           s_opened ? "applied" : "apply on next open");
	return 0;
}

static bool cam_parse_sync(const char *s, sync_level_t *out)
{
	if (!strcmp(s, "high") || !strcmp(s, "1")) {
		*out = SYNC_HIGH_LEVEL;
		return true;
	}
	if (!strcmp(s, "low") || !strcmp(s, "0")) {
		*out = SYNC_LOW_LEVEL;
		return true;
	}
	return false;
}

/* camera sync [vsync] [hsync]：DVP 同步极性。
 *
 * yuv_buf 报 "sensor's yuyv data resoltion is not right"（收到 <1KB 数据）
 * 时，先怀疑 HREF/VSYNC 极性和 yuv_buf 预期不一致。这里四种组合逐个试：
 *   camera sync high high
 *   camera off ; camera show
 * 不用改代码重烧。 */
static int cam_sync_cmd(chry_shell_t *csh, int argc, char **argv)
{
	static const char *names[] = { "low", "high" };

	if (argc >= 3 && !cam_parse_sync(argv[2], &s_vsync)) {
		csh_printf(csh, "invalid vsync '%s' (high|low)\r\n", argv[2]);
		return -1;
	}
	if (argc >= 4 && !cam_parse_sync(argv[3], &s_hsync)) {
		csh_printf(csh, "invalid hsync '%s' (high|low)\r\n", argv[3]);
		return -1;
	}

	csh_printf(csh, "sync: vsync=%s hsync=%s%s\r\n",
	           names[s_vsync], names[s_hsync],
	           (argc >= 3) ? "  (apply: 'camera off' then 'camera show')" : "");
	if (argc < 3)
		csh_printf(csh, "  usage: camera sync <high|low> [high|low]\r\n");
	return 0;
}

/* camera pins high|low|z：虚焊排查。
 *
 * 把 DVP 的 11 根信号线（MCLK/PCLK/HSYNC/VSYNC/D0~D7）临时切成普通
 * 推挽输出，拉到一个固定电平，然后拿万用表在模组/排线端逐根量：
 *   - 量到对应电平 -> 这段连线是通的；
 *   - 量不到 / 悬空 -> 虚焊、断线或接错脚。
 * 注意：先把摄像头断电（`ov3660 off`），否则模组也在驱动这些脚
 * （DVP 数据线是 sensor 输出），两边对着推挽会互相较劲。
 * `z` 释放成输入；重新 `camera show` 会把引脚映射回 DVP 功能。 */
static int cam_pins_cmd(chry_shell_t *csh, int argc, char **argv)
{
	const char *arg = (argc >= 3) ? argv[2] : "high";
	bool release = false;
	bool high = true;
	int i;

	if (s_opened) {
		csh_printf(csh, "close camera first ('camera off'), then 'camera pins %s'\r\n", arg);
		return -1;
	}

	if (!strcmp(arg, "z") || !strcmp(arg, "release")) {
		release = true;
	} else if (!strcmp(arg, "low") || !strcmp(arg, "0")) {
		high = false;
	} else if (!strcmp(arg, "high") || !strcmp(arg, "1")) {
		high = true;
	} else {
		csh_printf(csh, "usage: camera pins high|low|z\r\n");
		return -1;
	}

	if (bsp_power_is_enabled(BSP_PWR_RAIL_CAM_1V5) ||
	    bsp_power_is_enabled(BSP_PWR_RAIL_CAM_2V8)) {
		csh_printf(csh, "WARN: camera 1V5/2V8 还开着，先 'ov3660 off' 断电再量\r\n");
	}

	csh_printf(csh, "camera pins -> %s\r\n",
	           release ? "input/hiz" : (high ? "high" : "low"));
	for (i = 0; i < CAM_PIN_CNT; i++) {
		gpio_id_t p = (gpio_id_t)s_cam_pins[i].pin;

		if (release) {
			bk_gpio_disable_output(p);
			bk_gpio_disable_pull(p);
			bk_gpio_enable_input(p);
		} else {
			/* 先 unmap 掉 DVP 复用，再按普通 GPIO 推挽输出 */
			gpio_dev_unmap(p);
			bk_gpio_disable_pull(p);
			bk_gpio_enable_input(p);
			if (high)
				bk_gpio_set_output_high(p);
			else
				bk_gpio_set_output_low(p);
			bk_gpio_enable_output(p);
		}

		csh_printf(csh, "  P%-2d %-5s %s\r\n", (int)p, s_cam_pins[i].name,
		           release ? "input" : (high ? "= 1" : "= 0"));
	}

	if (release)
		csh_printf(csh, "  note: 重新 'camera show' 会把引脚映射回 DVP\r\n");
	else
		csh_printf(csh, "  现在拿万用表逐根量到模组端，应为 %s\r\n",
		           high ? "高电平" : "低电平");
	return 0;
}

static void cam_status(chry_shell_t *csh)
{
	csh_printf(csh, "DVP camera (OV3660)\r\n");
	csh_printf(csh, "  sensor   : %s\r\n", bsp_sensor_ov3660.name);
	csh_printf(csh, "  capture  : %ux%u YUV422 @ %d fps, mclk=%dMHz\r\n",
	           (unsigned)s_w, (unsigned)s_h, 15, cam_mclk_to_mhz(s_mclk));
	csh_printf(csh, "  dvp clk  : yuv_buf /%d (camera div)\r\n",
	           yuv_div_to_n(s_yuv_div));
	csh_printf(csh, "  opened   : %s\r\n", s_opened ? "yes" : "no");
	csh_printf(csh, "  frames   : seq=%u drop=%u\r\n", (unsigned)s_seq, (unsigned)s_drop);
	csh_printf(csh, "  lcd      : %s (%dx%d)\r\n",
	           lcdq_is_ready() ? "inited" : "NOT inited, run 'lcdq init'", LCD_W, LCD_H);
	csh_printf(csh, "  power    : 1v5=%d 2v8=%d\r\n",
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_1V5),
	           bsp_power_get_pad(BSP_PWR_RAIL_CAM_2V8));
}

int camera_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	const char *sub;
	frame_buffer_t *f;
	bk_err_t ret;

	if (argc < 2) {
		cam_status(csh);
		return 0;
	}

	sub = argv[1];

	if (!strcmp(sub, "help")) {
		cam_usage(csh);
		return 0;
	}

	if (!strcmp(sub, "on")) {
		ret = cam_open(csh);
		return (ret == BK_OK) ? 0 : -1;
	}

	if (!strcmp(sub, "off")) {
		cam_close(csh);
		return 0;
	}

	if (!strcmp(sub, "size")) {
		if (argc < 4) {
			csh_printf(csh, "size = %ux%u\r\n", (unsigned)s_w, (unsigned)s_h);
			return 0;
		}
		if (s_opened) {
			csh_printf(csh, "close camera first ('camera off')\r\n");
			return -1;
		}
		s_w = (uint16_t)atoi(argv[2]);
		s_h = (uint16_t)atoi(argv[3]);
		csh_printf(csh, "capture size -> %ux%u\r\n", (unsigned)s_w, (unsigned)s_h);
		csh_printf(csh, "  note: %ux%u YUV = %u bytes (PSRAM heap 640KB shared with lcd)\r\n",
		           (unsigned)s_w, (unsigned)s_h,
		           (unsigned)s_w * s_h * 2);
		return 0;
	}

	/* camera mclk [15|16|20|24|30|32|40|48]：改后需 off -> show/save 重开 */
	if (!strcmp(sub, "mclk")) {
		if (argc < 3) {
			csh_printf(csh, "mclk = %d MHz\r\n", cam_mclk_to_mhz(s_mclk));
			csh_printf(csh, "  available: 15 16 20 24 30 32 40 48 (MHz)\r\n");
			csh_printf(csh, "  lower mclk -> lower PCLK -> better signal integrity\r\n");
			return 0;
		}

		{
			int mhz = atoi(argv[2]);
			mclk_freq_t m;

			switch (mhz) {
			case 15: m = MCLK_15M; break;
			case 16: m = MCLK_16M; break;
			case 20: m = MCLK_20M; break;
			case 24: m = MCLK_24M; break;
			case 30: m = MCLK_30M; break;
			case 32: m = MCLK_32M; break;
			case 40: m = MCLK_40M; break;
			case 48: m = MCLK_48M; break;
			default:
				csh_printf(csh, "unsupported mclk %d MHz\r\n", mhz);
				return -1;
			}
			s_mclk = m;
		}

		csh_printf(csh, "mclk -> %d MHz\r\n", cam_mclk_to_mhz(s_mclk));
		csh_printf(csh, "  apply: 'camera off' then 'camera show'/'camera save'\r\n");
		return 0;
	}

	/* camera pclk [div]：sensor PCLK 分频，直接降 DVP 线上数据速率 */
	if (!strcmp(sub, "pclk")) {
		if (argc < 3) {
			csh_printf(csh, "pclk divider = %u (0=auto, 640x480 default 2)\r\n",
			           s_pclk_div);
			csh_printf(csh, "  usage: camera pclk <0|1..31>  (越大 PCLK 越慢)\r\n");
			return 0;
		}
		{
			int n = atoi(argv[2]);

			if (n < 0 || n > 31) {
				csh_printf(csh, "invalid pclk divider %d (0..31)\r\n", n);
				return -1;
			}
			s_pclk_div = (uint8_t)n;
			bsp_ov3660_set_pclk_div(s_pclk_div);
			csh_printf(csh, "pclk divider -> %u (%s)\r\n", s_pclk_div,
			           s_opened ? "written, then 'camera off'+'show' to resync"
			                    : "apply on next open");
		}
		return 0;
	}

	/* camera div [2|3|4|6]：yuv_buf 模块时钟分频（DVP 减速） */
	if (!strcmp(sub, "div"))
		return cam_div_cmd(csh, argc, argv);

	/* camera sync [v] [h]：DVP 同步极性 */
	if (!strcmp(sub, "sync"))
		return cam_sync_cmd(csh, argc, argv);

	/* camera pins high|low|z：DVP 引脚拉固定电平，排查虚焊 */
	if (!strcmp(sub, "pins"))
		return cam_pins_cmd(csh, argc, argv);

	/* camera yuvfmt yuyv|vyuy|uyvy|yyuv|vyuy：sensor YUV422 字节序。
	 * 偏蓝/偏绿但轮廓正常时逐个试（OV3660 常见是 YUYV 或 UYVY）。 */
	if (!strcmp(sub, "yuvfmt")) {
		static const struct {
			const char *name;
			img_fmt fmt;
		} fmts[] = {
			{ "yuyv", PIXEL_MSB_YUYV },
			{ "vuyy", PIXEL_MSB_VUYY },
			{ "uyvy", PIXEL_MSB_UYVY },
			{ "yyuv", PIXEL_MSB_YYUV },
			{ "vyuy", PIXEL_MSB_VYUY },
		};

		if (argc >= 3) {
			bool found = false;

			for (unsigned i = 0; i < sizeof(fmts) / sizeof(fmts[0]); i++) {
				if (!strcmp(argv[2], fmts[i].name)) {
					s_yuv_fmt = fmts[i].fmt;
					found = true;
					break;
				}
			}
			if (!found) {
				csh_printf(csh, "invalid fmt '%s' (yuyv|uyvy|yyuv|vyuy|vuyy)\r\n",
				           argv[2]);
				return -1;
			}
		}

		csh_printf(csh, "yuv input fmt = %d (0=yuyv 1=vuyy 2=uyvy 3=yyuv 4=vyuy)\r\n",
		           (int)s_yuv_fmt);
		return 0;
	}

	/* camera crop 0|1：1=中心裁 360x360 1:1 转换（跳过 SDK 缩放器） */
	if (!strcmp(sub, "crop")) {
		if (argc >= 3)
			s_crop = (atoi(argv[2]) != 0);
		csh_printf(csh, "crop mode = %d (%s)\r\n", (int)s_crop,
		           s_crop ? "center 360x360 1:1, no scaler" : "scale to 360x360");
		return 0;
	}

	/* camera swap 0|1：SDK 转换输出的 RGB565 字节序切换（看屏上颜色是否错乱） */
	if (!strcmp(sub, "swap")) {
		if (argc < 3) {
			csh_printf(csh, "byte swap = %d\r\n", (int)s_swap);
			csh_printf(csh, "  0 = as-is (BMP path expects little-endian)\r\n");
			csh_printf(csh, "  1 = swap for lcd (if colors look wrong on screen)\r\n");
			return 0;
		}
		s_swap = (atoi(argv[2]) != 0);
		csh_printf(csh, "byte swap -> %d\r\n", (int)s_swap);
		return 0;
	}

	/* camera detect：只跑 SDK 的 bk_dvp_detect，把 open 失败拆成
	 * "detect 失败(I2C/MCLK/sensor)" 或 "detect OK 但 yuv_mode 失败" */
	if (!strcmp(sub, "detect")) {
		const dvp_sensor_config_t *found;
		bk_dvp_config_t cfg;
		bk_err_t r;

		/* detect 不走 bk_dvp_open：上电 + 设置 sensor 列表后直接调
		 * SDK 的 bk_dvp_detect()，把 open 失败定位到 detect vs yuv_mode */
		r = bsp_power_camera_on();
		if (r != BK_OK) {
			csh_printf(csh, "camera power on failed: %d\r\n", r);
			return -1;
		}

		{
			static const dvp_sensor_config_t *list[] = { &bsp_sensor_ov3660 };
			bk_dvp_set_devices_list(list, 1);
		}

		/* 同 cam_open：先拆掉我方 I2C 状态，避免 bk_i2c_init 重复初始化
		 * 破坏 P20/P21 复用；速率也同步给 sensor 驱动 */
		i2c_bus_deinit(0);
		bsp_ov3660_set_sccb_baud(s_i2c_hz);

		os_memset(&cfg, 0, sizeof(cfg));
		cfg.reset_pin = 0xFF;
		cfg.pwdn_pin  = 0xFF;
		cfg.io_config.data_width = SENSOR_BITS_WIDTH_8BIT;
		for (int i = 0; i < 8; i++)
			cfg.io_config.data_pin[i] = s_data_pins[i];
		cfg.io_config.vsync_pin = BOARD_CIS_VSYNC_PIN;
		cfg.io_config.hsync_pin = BOARD_CIS_HSYNC_PIN;
		cfg.io_config.xclk_pin  = BOARD_CIS_MCLK_PIN;
		cfg.io_config.pclk_pin  = BOARD_CIS_PCLK_PIN;
		cfg.clk_source = s_mclk;
		cfg.i2c_config.id        = BOARD_CAM_I2C_ID;
		cfg.i2c_config.scl_pin   = BOARD_CAM_I2C_SCL_PIN;
		cfg.i2c_config.sda_pin   = BOARD_CAM_I2C_SDA_PIN;
		cfg.i2c_config.baud_rate = s_i2c_hz;

		/* DVP 数据脚映射（同 cam_open）：CONFIG_GPIO_DEFAULT_SET_SUPPORT=y
		 * 时 bk_dvp_detect 里的 io_init 被 #ifndef 裁掉，必须自己映射，
		 * 否则 DVP 控制器收不到任何数据（实测 dvp:0[0 0KB 0Kbps]） */
		dvp_camera_io_init(&cfg.io_config);

		found = bk_dvp_detect(&cfg);
		if (found)
			csh_printf(csh, "detect OK: %s (step2 yuv_mode is the suspect)\r\n", found->name);
		else
			csh_printf(csh, "detect FAILED: check mclk(%dMHz)/sccb(%uHz)/wiring\r\n",
			           cam_mclk_to_mhz(s_mclk), (unsigned)s_i2c_hz);

		/* detect 内部 bk_i2c_init 过，强制重建一次保住后续 ov3660 命令 */
		i2c_bus_force_rebuild(0);
		return found ? 0 : -1;
	}

	/* camera sccb [hz]：DVP 通路 SCCB 波特率（与 ov3660 sccb 互相独立） */
	if (!strcmp(sub, "sccb")) {
		if (argc < 3) {
			csh_printf(csh, "dvp sccb baud = %u Hz\r\n", (unsigned)s_i2c_hz);
			csh_printf(csh, "  usage: camera sccb <hz>   (10k..400k, default 100k)\r\n");
			csh_printf(csh, "  note : ov3660 cmds use 'ov3660 sccb <hz>' instead\r\n");
			return 0;
		}

		{
			uint32_t hz = (uint32_t)atoi(argv[2]);

			if (hz < 10000 || hz > 400000) {
				csh_printf(csh, "invalid baud %u (10k..400k)\r\n", (unsigned)hz);
				return -1;
			}
			if (s_opened) {
				csh_printf(csh, "close camera first ('camera off')\r\n");
				return -1;
			}
			s_i2c_hz = hz;
			bsp_ov3660_set_sccb_baud(s_i2c_hz);
		}

		csh_printf(csh, "dvp sccb baud -> %u Hz (apply on next camera open)\r\n",
		           (unsigned)s_i2c_hz);
		return 0;
	}

	/* camera dump：取一帧，打印元数据 + 每行开头 16 字节，
	 * 用来判断 DVP 到底有没有把图像数据写进 frame（蓝屏/花纹时先看这个） */
	if (!strcmp(sub, "dump")) {
		uint32_t sum = 0;

		if (cam_open(csh) != BK_OK)
			return -1;

		f = cam_grab(csh, CAM_WAIT_MS);
		if (!f)
			return -1;

		csh_printf(csh, "frame=%p seq=%u %ux%u len=%u fmt=%d\r\n",
		           f->frame, (unsigned)f->sequence, (unsigned)f->width,
		           (unsigned)f->height, (unsigned)f->length, (int)f->fmt);
		for (int r = 0; r < 4; r++) {
			uint32_t row = (uint32_t)r * (f->height ? (f->height - 1) / 3 : 0);
			uint32_t base = row * f->width * 2;
			uint32_t mid = base + (f->width / 2) * 2;
			uint32_t end = base + (f->width - 8) * 2;

			csh_printf(csh, " row%-3u:", (unsigned)row);
			for (int i = 0; i < 8; i++)
				csh_printf(csh, " %02X", f->frame[base + i]);
			csh_printf(csh, " |");
			for (int i = 0; i < 8; i++)
				csh_printf(csh, " %02X", f->frame[mid + i]);
			csh_printf(csh, " |");
			for (int i = 0; i < 8; i++)
				csh_printf(csh, " %02X", f->frame[end + i]);
			csh_printf(csh, "\r\n");
		}
		for (uint32_t i = 0; i < f->length; i++)
			sum += f->frame[i];
		csh_printf(csh, " sum=0x%08X\r\n", (unsigned)sum);
		dvp_frame_queue_free(IMAGE_YUV, f);
		return 0;
	}

	if (!strcmp(sub, "show")) {
		if (!lcdq_is_ready()) {
			csh_printf(csh, "lcd not inited, run 'lcdq init' first\r\n");
			return -1;
		}
		if (cam_open(csh) != BK_OK)
			return -1;

		csh_printf(csh, "waiting frame ...\r\n");
		f = cam_grab(csh, CAM_WAIT_MS);
		if (!f)
			return -1;

		ret = cam_convert_to_lcd(f);
		dvp_frame_queue_free(IMAGE_YUV, f);
		if (ret != 0) {
			csh_printf(csh, "image_convert failed\r\n");
			return -1;
		}
		if (s_swap)
			cam_swap_bytes();

		ret = lcdq_display_rgb565(lcdq_frame_buf(), LCD_FRAME_BYTES);
		if (ret != 0) {
			csh_printf(csh, "lcd display failed: %d\r\n", ret);
			return -1;
		}
		csh_printf(csh, "shown (%ux%u -> %dx%d)\r\n",
		           (unsigned)s_w, (unsigned)s_h, LCD_W, LCD_H);
		return 0;
	}

	/* camera raw [name]：把一帧**原始 YUYV** 存到 SD（可用 PC 端
	 * ffplay -f rawvideo -pixel_format yuyv422 -video_size 640x480 查看）。
	 * 和 camera save(BMP) 配合：raw 正常、BMP 花 => 转换问题。 */
	if (!strcmp(sub, "raw")) {
		const char *path = (argc >= 3) ? argv[2] : "1:/cam.yuv";
		uint32_t len;
		FIL fp;
		FRESULT fr;
		UINT bw = 0;

		if (cam_open(csh) != BK_OK)
			return -1;

		csh_printf(csh, "waiting frame ...\r\n");
		f = cam_grab(csh, CAM_WAIT_MS);
		if (!f)
			return -1;

		len = f->length;
		fr = cam_sd_mount();
		if (fr == FR_OK) {
			fr = f_open(&fp, path, FA_WRITE | FA_CREATE_ALWAYS);
			if (fr == FR_OK) {
				fr = f_write(&fp, f->frame, len, &bw);
				f_close(&fp);
			}
			f_unmount(1, "1:", 0);
		}
		dvp_frame_queue_free(IMAGE_YUV, f);

		if (fr != FR_OK) {
			csh_printf(csh, "write raw %s failed: %d (mount with 'sd list')\r\n",
			           path, fr);
			return -1;
		}
		csh_printf(csh, "raw %ux%u YUYV %u bytes -> %s\r\n",
		           (unsigned)s_w, (unsigned)s_h, (unsigned)bw, path);
		return 0;
	}

	if (!strcmp(sub, "save")) {
		const char *path = (argc >= 3) ? argv[2] : "1:/cam.bmp";

		if (cam_open(csh) != BK_OK)
			return -1;

		csh_printf(csh, "waiting frame ...\r\n");
		f = cam_grab(csh, CAM_WAIT_MS);
		if (!f)
			return -1;

		ret = cam_convert_to_lcd(f);
		dvp_frame_queue_free(IMAGE_YUV, f);
		if (ret != 0) {
			csh_printf(csh, "image_convert failed\r\n");
			return -1;
		}
		/* 转换输出就是本机 RGB565（uint16），BMP 写入按这个值分解即可；
		 * 不要套用屏幕的 swap（那是给 QSPI 字节序用的）。 */

		return write_bmp(csh, path, (uint16_t *)lcdq_frame_buf(), LCD_W, LCD_H);
	}

	csh_printf(csh, "unknown subcmd '%s'\r\n", sub);
	cam_usage(csh);
	return -1;
}
