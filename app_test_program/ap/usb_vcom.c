// Copyright 2026
// USB 虚拟串口（CDC ACM）—— 基于 SDK 里的 CherryUSB 设备栈
//
// 硬件：BK7258 的 USB 是芯片自带的专用 USB 2.0 全速设备口，DP/DM 不走 GPIO
// 复用（见 bsp_board_cfg.h 里的说明），所以这一路没有需要配置的引脚；
// 板上也没给 USB VBUS 做 GPIO 控制，插上 USB 线即上电。
//
// 用到的 SDK 组件（都在 ap/components/bk_usb/CherryUSB 下）：
//   core/usbd_core.c              CherryUSB 设备核心      CONFIG_USB_DEVICE 时由 SDK 编译
//   port/beken_musb/*.c           Beken MUSBD 端口驱动    CONFIG_USB_DEVICE 时由 SDK 编译
//
// 注意：SDK 自带的 class/cdc/usbd_cdc.c 没有被 bk_usb 的 CMakeLists 编进来
// （它用的是 CherryUSB 上游的 "usbd_cdc.h" 头文件布局，在本 SDK 里找不到），
// 所以 CDC ACM 的类请求处理在下面自己实现，只用到 usb_cdc.h 里的宏和结构体。
//
// 一次完整的枚举流程：
//   usb_vcom_init()
//     -> usbd_desc_register()          挂描述符
//     -> usbd_add_interface() x2       通信接口 + 数据接口
//     -> usbd_add_endpoint() x2        bulk in / bulk out
//     -> bk_usb_open(USB_DEVICE_MODE)  上电、开时钟、usbd_initialize()
//     -> 主机 SET_CONFIGURATION         s_configured = 1，任务自动发欢迎串

#include <string.h>

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>
#include <components/usb_types.h>
#include <components/cherryusb/usbd_core.h>
#include <components/cherryusb/usbd_cdc.h>

#include "usb_vcom.h"

#define TAG "usb_vcom"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* bk_usb_open 声明在 <components/usb.h>，但那个头会把 UVC/UAC 的 API 一起
 * 拉进来，这里只用到它一个，直接声明省事。 */
extern bk_err_t bk_usb_open(uint32_t usb_mode);

/* ============================ 描述符 ============================ */

/* VID/PID：0x1209 是 pid.codes 分配的社区开源 VID，随便用不会和量产设备冲突。
 * CDC ACM 是标准类，Win10+ / Linux / macOS 都自带驱动，不需要 inf。 */
#define USBD_VID            0x1209
#define USBD_PID            0x0001
#define USBD_MAX_POWER      100         /* mA */
#define USBD_LANGID_STRING  0x0409      /* English (US) */

/* 端点地址：中断通知用 0x81，数据 IN 0x82，数据 OUT 0x02 */
#define CDC_INT_EP          0x81
#define CDC_OUT_EP          0x02
#define CDC_IN_EP           0x82

/* 全速（FS）bulk 端点最大包长固定 64 */
#define CDC_MPS             64

/* 设备描述符 + 配置描述符（9） + CDC ACM 功能描述符（CDC_ACM_DESCRIPTOR_LEN） */
#define USB_CONFIG_TOTAL_LEN    (9 + CDC_ACM_DESCRIPTOR_LEN)

#define USB_C(ch)           ((ch) & 0xFF), 0x00     /* 字符串描述符里的一个 UTF-16LE 字符 */

static const uint8_t cdc_acm_descriptor[] = {
	/* 设备描述符：bDeviceClass = CDC(0x02)，Interface Association 里再细化 */
	USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, USB_DEVICE_CLASS_CDC, 0x00, 0x00,
	                           USBD_VID, USBD_PID, 0x0100, 0x01),

	/* 配置描述符：2 个接口（通信 + 数据），总线供电 */
	USB_CONFIG_DESCRIPTOR_INIT(USB_CONFIG_TOTAL_LEN, 0x02, 0x01,
	                           USB_CONFIG_BUS_POWERED, USBD_MAX_POWER),

	/* IAD + 通信接口(0) + Header/CallMgmt/ACM/Union 功能描述符 +
	 * 中断通知端点 + 数据接口(1) + bulk out/in 端点 */
	CDC_ACM_DESCRIPTOR_INIT(0x00, CDC_INT_EP, CDC_OUT_EP, CDC_IN_EP, CDC_MPS, 0x00),

	/* string0：语言 ID */
	USB_LANGID_INIT(USBD_LANGID_STRING),

	/* string1：厂商 "Beken"，5 字符 -> 2 + 10 = 12 */
	0x0C, USB_DESCRIPTOR_TYPE_STRING,
	USB_C('B'), USB_C('e'), USB_C('k'), USB_C('e'), USB_C('n'),

	/* string2：产品名 "BK7258 CDC VCOM"，15 字符 -> 2 + 30 = 32 */
	0x20, USB_DESCRIPTOR_TYPE_STRING,
	USB_C('B'), USB_C('K'), USB_C('7'), USB_C('2'), USB_C('5'), USB_C('8'),
	USB_C(' '), USB_C('C'), USB_C('D'), USB_C('C'), USB_C(' '),
	USB_C('V'), USB_C('C'), USB_C('O'), USB_C('M'),

	/* string3：序列号 "0001"，4 字符 -> 2 + 8 = 10 */
	0x0A, USB_DESCRIPTOR_TYPE_STRING,
	USB_C('0'), USB_C('0'), USB_C('0'), USB_C('1'),

	/* 描述符表结束标记 */
	0x00
};

/* ============================ 状态 ============================ */

#define VCOM_RX_RING_SIZE       256
#define VCOM_TX_TIMEOUT_MS      500     /* 等一个 bulk 包发出去的超时 */
#define VCOM_TASK_STACK         3072
#define VCOM_TASK_PRIO          BEKEN_DEFAULT_WORKER_PRIORITY

static struct usbd_interface s_intf_comm;   /* 通信接口 intf0 */
static struct usbd_interface s_intf_data;   /* 数据接口 intf1 */

static uint8_t s_rx_buf[CDC_MPS] __attribute__((aligned(4)));

static uint8_t s_rx_ring[VCOM_RX_RING_SIZE];
static volatile uint32_t s_rx_wr;
static volatile uint32_t s_rx_rd;
static volatile uint32_t s_rx_total;
static volatile uint32_t s_tx_total;

static volatile uint8_t s_tx_busy;          /* IN 端点正忙（已挂包、等硬件发完） */
static volatile bool s_configured;          /* 主机已完成 SET_CONFIGURATION */
static volatile bool s_dtr;                 /* 主机应用打开了串口 */
static volatile bool s_rts;

static struct cdc_line_coding s_line_coding = {
	.dwDTERate = 115200,
	.bCharFormat = 0,       /* 1 stop bit */
	.bParityType = 0,       /* none */
	.bDataBits = 8,
};

static beken_thread_t s_task = NULL;
static beken_mutex_t s_tx_mutex = NULL;
static volatile bool s_running;
static bool s_usb_opened;       /* bk_usb_open(USB_DEVICE_MODE) 是否做过 */
static bool s_inited;

/* ============================ CDC ACM 类请求 ============================ */

/* 主机侧（cdc_acm / usbser）在打开串口、改波特率时会发这些 class 请求。
 * 虚拟串口不需要真的去配 UART，记录一下状态即可，但必须应答，
 * 否则主机会认为设备异常。 */
static int cdc_acm_class_request_handler(struct usb_setup_packet *setup,
                                         uint8_t **data, uint32_t *len)
{
	struct cdc_line_coding lc;
	uint8_t intf_num = (uint8_t)(setup->wIndex & 0xFF);

	switch (setup->bRequest) {
	case CDC_REQUEST_SET_LINE_CODING:
		/* 数据段由 usbd_core 先读到 req_data 再回调，这里 data 指向它 */
		memcpy(&lc, *data, sizeof(lc));
		s_line_coding = lc;
		LOGI("intf%u line coding: %u %u %u %u\r\n", intf_num,
		     (unsigned)lc.dwDTERate, lc.bDataBits, lc.bParityType, lc.bCharFormat);
		break;

	case CDC_REQUEST_GET_LINE_CODING:
		memcpy(*data, &s_line_coding, sizeof(struct cdc_line_coding));
		*len = sizeof(struct cdc_line_coding);
		break;

	case CDC_REQUEST_SET_CONTROL_LINE_STATE:
		s_dtr = (setup->wValue & SET_CONTROL_LINE_STATE_DTR) ? true : false;
		s_rts = (setup->wValue & SET_CONTROL_LINE_STATE_RTS) ? true : false;
		LOGI("intf%u control line: dtr=%d rts=%d\r\n", intf_num, s_dtr, s_rts);
		break;

	case CDC_REQUEST_SEND_BREAK:
		LOGW("intf%u send break\r\n", intf_num);
		break;

	default:
		LOGW("unhandled cdc request 0x%02x\r\n", setup->bRequest);
		return -1;
	}

	return 0;
}

/* 注册一个 CDC ACM 接口（通信接口和数据接口共用同一套类请求处理） */
static struct usbd_interface *cdc_acm_init_intf(struct usbd_interface *intf)
{
	intf->class_interface_handler = cdc_acm_class_request_handler;
	intf->class_endpoint_handler = NULL;
	intf->vendor_handler = NULL;
	intf->notify_handler = NULL;

	return intf;
}

/* ============================ 端点回调（中断上下文） ============================ */

static void cdc_in_ep_callback(uint8_t ep, uint32_t nbytes)
{
	(void)ep;

	s_tx_total += nbytes;
	s_tx_busy = 0;
}

static void cdc_out_ep_callback(uint8_t ep, uint32_t nbytes)
{
	(void)ep;

	for (uint32_t i = 0; i < nbytes; i++) {
		uint32_t next = (s_rx_wr + 1) % VCOM_RX_RING_SIZE;

		if (next != s_rx_rd) {          /* 满了丢新字节 */
			s_rx_ring[s_rx_wr] = s_rx_buf[i];
			s_rx_wr = next;
		}
	}
	s_rx_total += nbytes;

	/* 这个驱动的 OUT 端点是一次性的：收完一包必须重新挂读请求 */
	usbd_ep_start_read(CDC_OUT_EP, s_rx_buf, CDC_MPS);
}

static struct usbd_endpoint cdc_in_ep = {
	.ep_cb = cdc_in_ep_callback,
	.ep_addr = CDC_IN_EP,
};

static struct usbd_endpoint cdc_out_ep = {
	.ep_cb = cdc_out_ep_callback,
	.ep_addr = CDC_OUT_EP,
};

/* ============================ 事件通知 ============================ */

static void cdc_notify_handler(uint8_t event, void *arg)
{
	(void)arg;

	switch (event) {
	case USBD_EVENT_CONFIGURED:
		LOGI("configured, /dev/ttyACM ready\r\n");
		s_configured = true;
		/* 主机配好之后先把 OUT 端点挂起来，才能收数据 */
		usbd_ep_start_read(CDC_OUT_EP, s_rx_buf, CDC_MPS);
		break;

	case USBD_EVENT_RESET:
	case USBD_EVENT_DISCONNECTED:
		LOGI("reset/disconnect\r\n");
		s_configured = false;
		s_dtr = false;
		s_rts = false;
		s_tx_busy = 0;
		break;

	default:
		break;
	}
}

/* ============================ 收发 ============================ */

int usb_vcom_send(const void *data, uint32_t len)
{
	const uint8_t *p = (const uint8_t *)data;
	uint32_t sent = 0;
	int ret;

	if (!s_configured) {
		LOGE("not configured, drop %u bytes\r\n", (unsigned)len);
		return -1;
	}

	/* shell 任务和欢迎串任务可能同时发，串行化一下，避免两包数据交错 */
	if (s_tx_mutex)
		rtos_lock_mutex(&s_tx_mutex);

	while (sent < len) {
		uint32_t chunk = len - sent;
		uint32_t waited = 0;

		if (chunk > CDC_MPS)
			chunk = CDC_MPS;

		/* 等上一包发完（USB 中断里会清 s_tx_busy） */
		while (s_tx_busy && (waited < VCOM_TX_TIMEOUT_MS)) {
			rtos_delay_milliseconds(1);
			waited++;
		}
		if (s_tx_busy) {
			LOGE("tx timeout at %u/%u\r\n", (unsigned)sent, (unsigned)len);
			goto exit;
		}

		s_tx_busy = 1;
		ret = usbd_ep_start_write(CDC_IN_EP, p + sent, chunk);
		if (ret < 0) {
			s_tx_busy = 0;
			LOGE("ep write failed: %d\r\n", ret);
			goto exit;
		}
		sent += chunk;
	}

	/* 长度正好是 64 的整数倍时，主机不知道数据已经传完，需要补一个 ZLP */
	if (len && ((len % CDC_MPS) == 0)) {
		uint32_t waited = 0;

		while (s_tx_busy && (waited < VCOM_TX_TIMEOUT_MS)) {
			rtos_delay_milliseconds(1);
			waited++;
		}
		if (!s_tx_busy) {
			s_tx_busy = 1;
			if (usbd_ep_start_write(CDC_IN_EP, NULL, 0) < 0)
				s_tx_busy = 0;
		}
	}

exit:
	if (s_tx_mutex)
		rtos_unlock_mutex(&s_tx_mutex);

	return (sent > 0) ? (int)sent : -1;
}

uint32_t usb_vcom_read(uint8_t *buf, uint32_t len)
{
	uint32_t n = 0;

	while ((n < len) && (s_rx_rd != s_rx_wr)) {
		buf[n++] = s_rx_ring[s_rx_rd];
		s_rx_rd = (s_rx_rd + 1) % VCOM_RX_RING_SIZE;
	}

	return n;
}

bool usb_vcom_is_configured(void)
{
	return s_configured;
}

bool usb_vcom_is_dtr(void)
{
	return s_dtr;
}

uint32_t usb_vcom_rx_total(void)
{
	return s_rx_total;
}

uint32_t usb_vcom_tx_total(void)
{
	return s_tx_total;
}

void usb_vcom_get_line_coding(uint32_t *baud, uint8_t *data_bits,
                              uint8_t *parity, uint8_t *stop_bits)
{
	if (baud)
		*baud = s_line_coding.dwDTERate;
	if (data_bits)
		*data_bits = s_line_coding.bDataBits;
	if (parity)
		*parity = s_line_coding.bParityType;
	if (stop_bits)
		*stop_bits = s_line_coding.bCharFormat;
}

/* ============================ 欢迎串任务 ============================ */

/* 主机打开串口（SET_CONTROL_LINE_STATE 把 DTR 置位）之后自动发一次欢迎串。
 *
 * 为什么等 DTR 而不是等 SET_CONFIGURATION：
 *   枚举完成只说明主机认了设备，此时主机还没有向 bulk IN 端点发 IN 令牌，
 *   数据会一直压在设备 FIFO 里（s_tx_busy 也就一直是 1），后面的发送会卡住。
 *   主机上层一旦 open() 串口就会置 DTR，这时候发才是真正能送到对端的。
 *
 * 也不放在 USB 中断里做：发送要等端点空，中断上下文里不能阻塞。 */
static void usb_vcom_task(void *arg)
{
	bool greeted = false;
	uint32_t wait_ms = 0;

	(void)arg;

	while (s_running) {
		if (!s_configured) {
			greeted = false;
			wait_ms = 0;
		} else if (s_dtr) {
			if (!greeted) {
				/* 等主机把 bulk IN 的读请求挂上，DTR 和它不是一个时序 */
				rtos_delay_milliseconds(150);
				if (usb_vcom_send(USB_VCOM_GREETING, sizeof(USB_VCOM_GREETING) - 1) > 0)
					LOGI("greeting sent\r\n");
				greeted = true;
			}
		} else {
			/* 有的上位机只做枚举、不置 DTR，给个提示方便排查 */
			wait_ms += 100;
			if (wait_ms == 3000)
				LOGW("configured but DTR not set (open the port to receive data)\r\n");
		}

		rtos_delay_milliseconds(100);
	}

	LOGI("task exit\r\n");
	s_task = NULL;
	rtos_delete_thread(NULL);
}

/* ============================ 初始化 / 反初始化 ============================ */

bk_err_t usb_vcom_init(void)
{
	bk_err_t ret;

	if (s_inited)
		return BK_OK;

	/* 1. 描述符 + 接口 + 端点，顺序不能乱：
	 *    usbd_desc_register 会把接口序号计数清零，usbd_add_interface 依次分配
	 *    intf_num = 0 / 1，类请求靠这个序号路由。
	 *    usb init -> deinit -> init 这种重复初始化时，先把上次的接口链表清空，
	 *    否则同一个 usbd_interface 节点会被挂两次。 */
	usb_slist_init(&usbd_intf_head);
	usbd_desc_register(cdc_acm_descriptor);
	usbd_add_interface(cdc_acm_init_intf(&s_intf_comm));
	usbd_add_interface(cdc_acm_init_intf(&s_intf_data));
	usbd_add_endpoint(&cdc_in_ep);
	usbd_add_endpoint(&cdc_out_ep);

	/* 只在通信接口上挂事件回调，数据接口挂了会收到两份 */
	s_intf_comm.notify_handler = cdc_notify_handler;

	s_configured = false;
	s_dtr = false;
	s_tx_busy = 0;
	s_rx_wr = s_rx_rd = 0;
	s_rx_total = s_tx_total = 0;

	if (s_tx_mutex == NULL)
		rtos_init_mutex(&s_tx_mutex);

	/* 2. 打开 USB：上电、开时钟、做模拟 PHY 配置，再 usbd_initialize()
	 *    -> usb_dc_init() -> 挂 USB 中断。
	 *    第一次走 bk_usb_open()，它会额外投 USB 的电源/休眠票；
	 *    之后（usb deinit 后再 init）只重做设备侧，因为 bk_usb_close() 在本 SDK
	 *    里会连 host 一起反初始化，而 host 没初始化过的话会踩空指针。 */
	if (!s_usb_opened) {
		ret = bk_usb_open(USB_DEVICE_MODE);
		if (ret != BK_OK) {
			LOGE("bk_usb_open(device) failed: %d\r\n", ret);
			return ret;
		}
		s_usb_opened = true;
	} else {
		if (usbd_initialize() != 0) {
			LOGE("usbd_initialize failed\r\n");
			return BK_FAIL;
		}
	}

	s_inited = true;

	/* 3. 独立任务负责发欢迎串 */
	s_running = true;
	if (rtos_create_thread(&s_task, VCOM_TASK_PRIO, "usb_vcom",
	                       (beken_thread_function_t)usb_vcom_task,
	                       VCOM_TASK_STACK, NULL) != kNoErr) {
		s_running = false;
		LOGE("create task failed\r\n");
		usbd_deinitialize();
		s_inited = false;
		return BK_FAIL;
	}

	LOGI("usb cdc acm vcom started, wait for host enumeration\r\n");
	return BK_OK;
}

bk_err_t usb_vcom_deinit(void)
{
	if (!s_inited)
		return BK_OK;

	s_running = false;
	rtos_delay_milliseconds(200);       /* 让任务自己退出并释放栈 */

	s_configured = false;

	/* 只做设备侧下电：关 USB 中断、关 PHY/时钟。
	 *
	 * 这里不能用 bk_usb_close()：本工程 host 和 device 是同时编进来的
	 * （CONFIG_USB_HOST 还开着，bk_uvc 等组件依赖它），而 bk_usb_close() 里
	 * CONFIG_USB_HOST 分支会调 usbh_deinitialize() -> usbh_roothub_free_port1_hub()，
	 * host 没 initialize 过时 roothub 是 NULL，会直接踩空指针。 */
	usbd_deinitialize();

	s_inited = false;
	LOGI("usb vcom stopped (device side only)\r\n");
	return BK_OK;
}
