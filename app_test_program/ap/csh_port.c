// Copyright 2026
// CherrySH 移植层实现（UART1）
//
// 串口接收：关掉 UART 驱动的软件 fifo，注册自己的 RX 中断，把硬件 fifo
//           里的字节搬进环形缓冲；CherrySH 的 sget 从环形缓冲取数据。
// 串口发送：sput 直接调 bk_uart_write_bytes()（阻塞到写进 TX fifo）。
//
// 为什么不直接用 bk_uart_read_bytes()：
//   软件 fifo 模式下，没数据时它会等信号量并打 "recv data timeout" 警告，
//   shell 是高频轮询的，会把日志刷爆。这里沿用 SDK shell_uart 的做法。

#include "csh_port.h"

#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>
#include <driver/uart.h>
#include "bk_uart.h"            /* uart_read_byte_ex() */

#define TAG "csh_port"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define CSH_UART_ID         UART_ID_1
#define CSH_UART_BAUD       115200
#define CSH_TASK_STACK      8192
#define CSH_TASK_PRIO       BEKEN_DEFAULT_WORKER_PRIORITY
#define CSH_IDLE_MS         10
#define CSH_RX_RING_SIZE    512

static chry_shell_t s_csh;
static beken_thread_t s_csh_thread = NULL;
static bool s_running = false;

/* 静态缓冲：prompt / 历史 / 行编辑 */
static char s_prompt_buf[128];
static char s_history_buf[512];
static char s_line_buf[128];

/* 串口接收环形缓冲：中断写、shell 任务读 */
static uint8_t s_rx_ring[CSH_RX_RING_SIZE];
static volatile uint32_t s_rx_wr;
static volatile uint32_t s_rx_rd;

/* PATH 环境变量：命令默认在 /bin 下，这里把 /bin 和 /sbin 都列上 */
#define CSH_ENV_PATH "/sbin:/bin"
static const char s_env_path[] = CSH_ENV_PATH;

/*
 * 命令表/变量表。
 *
 * CherrySH 官方示例是用 CSH_CMD_EXPORT 把命令塞进链接脚本的 FSymTab 段，
 * 然后由 __fsymtab_start/__fsymtab_end 遍历。但这套机制依赖 "命令所在的 .o
 * 一定会被链接进最终镜像"：本 SDK 里 app 组件的源码是打成 libap.a 静态库的，
 * ld 不会为了链接脚本里的 KEEP(*(FSymTab)) 去抽取 .o，结果就是新加的命令
 * 悄悄不出现（help/shsize 也一样）。
 *
 * 所以这里改成显式注册表：表里引用了命令函数，命令所在的 .o 自然会被拉进来，
 * 不依赖链接顺序，也不用改 SDK 的 .ld。
 */
extern int help_cmd(int argc, char **argv);
extern int ldo_cmd(int argc, char **argv);
extern int board_cmd(int argc, char **argv);
extern int gpio_cmd(int argc, char **argv);
extern int i2c_cmd(int argc, char **argv);
extern int accel_cmd(int argc, char **argv);
extern int ov3660_cmd(int argc, char **argv);
extern int lcdq_cmd(int argc, char **argv);
extern int bat_cmd(int argc, char **argv);
extern int camera_cmd(int argc, char **argv);
extern int dmic_cmd(int argc, char **argv);
extern int sd_cmd(int argc, char **argv);
extern int music_cmd(int argc, char **argv);
extern int usb_cmd(int argc, char **argv);

static const chry_syscall_t s_cmd_table[] = {
	{ "/bin", "help",  help_cmd  },
	{ "/bin", "ldo",   ldo_cmd   },
	{ "/bin", "board", board_cmd },
	{ "/bin", "gpio",  gpio_cmd  },
	{ "/bin", "i2c",   i2c_cmd   },
	{ "/bin", "accel", accel_cmd },
	{ "/bin", "ov3660", ov3660_cmd },
	{ "/bin", "cam",   ov3660_cmd },  /* 别名，打起来短一点 */
	{ "/bin", "lcdq",  lcdq_cmd  },   /* QSPI 屏 ST77916 调试 */
	{ "/bin", "qspi",  lcdq_cmd  },   /* 别名 */
	{ "/bin", "lcd",   lcdq_cmd  },   /* 别名 */
	{ "/bin", "bat",   bat_cmd   },   /* 电池电压 ADC 测量 */
	{ "/bin", "adc",   bat_cmd   },   /* 别名 */
	{ "/bin", "camera", camera_cmd }, /* DVP 摄像头出图 */
	{ "/bin", "dvp",    camera_cmd }, /* 别名（cam 已被 ov3660 占用） */
	{ "/bin", "dmic",  dmic_cmd  },
	{ "/bin", "sd",    sd_cmd    },
	{ "/bin", "music", music_cmd },
	{ "/bin", "usb",   usb_cmd   },
};

static const chry_sysvar_t s_var_table[] = {
	{ "PATH", (void *)s_env_path, 0x80000000 | sizeof(CSH_ENV_PATH) },
};

chry_shell_t *csh_port_shell(void)
{
	return &s_csh;
}

/* ---------------- UART1 <-> CherrySH ---------------- */

static uint16_t csh_sput_cb(chry_readline_t *rl, const void *data, uint16_t size)
{
	(void)rl;

	if (!size)
		return 0;

	if (bk_uart_write_bytes(CSH_UART_ID, data, size) != BK_OK)
		return 0;

	return size;
}

static uint16_t csh_sget_cb(chry_readline_t *rl, void *data, uint16_t size)
{
	uint8_t *out = (uint8_t *)data;
	uint16_t n = 0;

	(void)rl;

	while ((n < size) && (s_rx_rd != s_rx_wr)) {
		out[n++] = s_rx_ring[s_rx_rd];
		s_rx_rd = (s_rx_rd + 1) % CSH_RX_RING_SIZE;
	}

	return n;
}

static void csh_uart_rx_isr(uart_id_t id, void *param)
{
	uint8_t ch;

	(void)param;

	while (uart_read_byte_ex(id, &ch) == 0) {
		uint32_t next = (s_rx_wr + 1) % CSH_RX_RING_SIZE;

		if (next != s_rx_rd) {          /* 满了丢新字节 */
			s_rx_ring[s_rx_wr] = ch;
			s_rx_wr = next;
		}
	}
}

static bk_err_t csh_uart_init(void)
{
	uart_config_t cfg;
	bk_err_t ret;

	os_memset(&cfg, 0, sizeof(cfg));
	cfg.baud_rate = CSH_UART_BAUD;
	cfg.data_bits = UART_DATA_8_BITS;
	cfg.parity = UART_PARITY_NONE;
	cfg.stop_bits = UART_STOP_BITS_1;
	cfg.flow_ctrl = UART_FLOWCTRL_DISABLE;
	cfg.src_clk = UART_SCLK_XTAL_26M;
	cfg.rx_dma_en = UART_DMA_DISABLE;
	cfg.tx_dma_en = UART_DMA_DISABLE;

	ret = bk_uart_init(CSH_UART_ID, &cfg);
	if (ret != BK_OK) {
		LOGE("uart%d init failed: %d\r\n", CSH_UART_ID, ret);
		return ret;
	}

	s_rx_wr = 0;
	s_rx_rd = 0;

	/* 关软件 fifo + 注册自己的 rx 中断 */
	ret = bk_uart_take_rx_isr(CSH_UART_ID, csh_uart_rx_isr, NULL);
	if (ret != BK_OK) {
		LOGE("take rx isr failed: %d\r\n", ret);
		return ret;
	}

	ret = bk_uart_enable_rx_interrupt(CSH_UART_ID);
	if (ret != BK_OK) {
		LOGE("enable rx int failed: %d\r\n", ret);
		return ret;
	}

	return BK_OK;
}

/* ---------------- shell 任务 ---------------- */

static void csh_task(void *arg)
{
	(void)arg;

	while (s_running) {
		int ret = chry_shell_task_repl(&s_csh);

		if (ret < 0) {
			LOGE("shell repl error\r\n");
			break;
		}

		/* NOBLOCK 模式下 1 = 这次没收到完整输入，让出 CPU */
		if (ret == 1)
			rtos_delay_milliseconds(CSH_IDLE_MS);
	}

	LOGE("shell task exit\r\n");
	s_csh_thread = NULL;
	rtos_delete_thread(NULL);
}

/* ---------------- 内建命令 help ---------------- */

int help_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];

	(void)argc;

	csh_printf(csh, "commands:\r\n");
	for (const chry_syscall_t *c = csh->cmd_tbl_beg; c < csh->cmd_tbl_end; c++)
		csh_printf(csh, "  %-8s %s\r\n", c->name, c->path);

	return 0;
}

bk_err_t csh_port_start(void)
{
	static const char banner[] =
		"\r\n"
		"BK7258 CherrySH on UART1 (GPIO0=TX GPIO1=RX, 115200 8N1)\r\n"
		"type 'help' for commands, 'ldo' for power rail test,\r\n"
		"'usb' for the usb cdc acm virtual com port\r\n\r\n";
	chry_shell_init_t init;
	bk_err_t ret;
	int iret;

	if (s_running)
		return BK_OK;

	ret = csh_uart_init();
	if (ret != BK_OK)
		return ret;

	/* 手写 banner：shell 起来之前，让用户知道串口是通的 */
	bk_uart_write_bytes(CSH_UART_ID, banner, sizeof(banner) - 1);

	os_memset(&init, 0, sizeof(init));
	init.sput = csh_sput_cb;
	init.sget = csh_sget_cb;

	init.command_table_beg = s_cmd_table;
	init.command_table_end = s_cmd_table + sizeof(s_cmd_table) / sizeof(s_cmd_table[0]);
	init.variable_table_beg = s_var_table;
	init.variable_table_end = s_var_table + sizeof(s_var_table) / sizeof(s_var_table[0]);

#if defined(CONFIG_CSH_PROMPTEDIT) && CONFIG_CSH_PROMPTEDIT
	init.prompt_buffer = s_prompt_buf;
	init.prompt_buffer_size = sizeof(s_prompt_buf);
#endif

#if defined(CONFIG_CSH_HISTORY) && CONFIG_CSH_HISTORY
	init.history_buffer = s_history_buf;
	init.history_buffer_size = sizeof(s_history_buf);
#endif

#if defined(CONFIG_CSH_LNBUFF_STATIC) && CONFIG_CSH_LNBUFF_STATIC
	init.line_buffer = s_line_buf;
	init.line_buffer_size = sizeof(s_line_buf);
#endif

	init.uid = 0;
	init.user[0] = "root";
	/* 这里必须是空字符串，不能给 NULL：
	 * chry_shell_init() 里会对 hash 直接 strnlen()，给 NULL 会读 0 地址
	 * （AP 的地址 0 没有映射）→ HardFault → 芯片复位，表现为反复打印 banner。
	 * 不调用 csh_login() 就不会走登录流程，空串等价于"没有密码"。 */
	init.hash[0] = "";
	init.host = "bk7258";
	init.user_data = NULL;

	iret = chry_shell_init(&s_csh, &init);
	if (iret != 0) {
		LOGE("chry_shell_init failed: %d\r\n", iret);
		return BK_FAIL;
	}

	s_running = true;

	iret = rtos_create_thread(&s_csh_thread, CSH_TASK_PRIO, "csh",
	                          (beken_thread_function_t)csh_task,
	                          CSH_TASK_STACK, NULL);
	if (iret != kNoErr) {
		s_running = false;
		LOGE("create shell task failed: %d\r\n", iret);
		return BK_FAIL;
	}

	LOGI("cherrysh ready on uart%d (%d 8N1)\r\n", CSH_UART_ID, CSH_UART_BAUD);
	return BK_OK;
}
