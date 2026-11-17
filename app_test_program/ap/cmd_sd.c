// Copyright 2026
// CherrySH 命令：sd —— SD 卡（SDIO 4 线）通讯测试
//
//   sd                  上电/初始化，打印卡信息，读 block0 校验引导扇区
//   sd off              反初始化
//   sd r <blk> [n]      读 n 个块（默认 1），hexdump 第一块前 64 字节
//   sd w <blk> [n]      往 n 个块写测试模式再读回比对（会覆盖原数据！）
//   sd speed [n]        顺序读 n 块测速（默认 64 = 32KB）
//   sd list [path]      挂载 FATFS 到 "0:"，列目录 / 查剩余空间
//
// 板上 SD 卡：SDIO 4 线，CLK=P14 CMD=P15 D0=P16 D1=P17 D2=P18 D3=P19，
// 卡座 CD 脚 SD_DET = P40（低有效）。
//
// 依赖的配置（本工程已开）：
//   CONFIG_SDCARD=y、CONFIG_SDIO_HOST=y、CONFIG_FATFS=y、CONFIG_FATFS_SDCARD=y
//   CONFIG_SDCARD_BUSWIDTH_4LINE=y  ← 卡侧 ACMD6 切 4 线
//   CONFIG_SDIO_4LINES_EN=y         ← 主机侧 4 线
// 本工程开了 CONFIG_GPIO_DEFAULT_SET_SUPPORT，SDIO 驱动不会自己映射引脚，
// 所以 P14~P19 的复用写在 ap/config/bk7258_ap/usr_gpio_cfg.h 的默认表里。

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_board_cfg.h"
#include "gpio_driver.h"
#include <driver/gpio.h>
#include <driver/sd_card.h>
#include <driver/sdio_host.h>
#include <driver/pwr_clk.h>   /* bk_pm_module_vote_ctrl_external_ldo() */
#include "gpio_ll.h"        /* gpio_ll_get_gpio_perial_mode() */
#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include "ff.h"

#define SD_BLK          512
#define SD_HEXDUMP_LEN  64
#define SD_SPEED_BLOCKS 64
#define SD_MAX_BLOCKS   512        /* 256KB，受 AP 堆限制 */

static FATFS s_fs;
static FILINFO s_fi;
static bool s_sd_inited;

/* 卡座 CD 脚：低有效，读之前先配成上拉输入 */
static int sd_card_present(void)
{
	int level;

	gpio_dev_unmap(BOARD_SD_DETECT_PIN);
	bk_gpio_disable_pull(BOARD_SD_DETECT_PIN);
	bk_gpio_pull_up(BOARD_SD_DETECT_PIN);
	bk_gpio_disable_output(BOARD_SD_DETECT_PIN);
	bk_gpio_enable_input(BOARD_SD_DETECT_PIN);
	rtos_delay_milliseconds(2);

	level = bk_gpio_get_input(BOARD_SD_DETECT_PIN) ? 1 : 0;
	return BOARD_SD_DETECT_ACTIVE_LOW ? !level : level;
}

static const char *sd_type_str(uint32_t type)
{
	return (type == 1) ? "SDHC/SDXC" : "SDSC";
}

static void sd_hexdump(chry_shell_t *csh, const uint8_t *buf, int len)
{
	int i;

	for (i = 0; i < len; i++) {
		if ((i % 16) == 0)
			csh_printf(csh, "  %04x:", i);
		csh_printf(csh, " %02x", buf[i]);
		if ((i % 16) == 15)
			csh_printf(csh, "\r\n");
	}
	if ((len % 16) != 0)
		csh_printf(csh, "\r\n");
}

static int sd_open(chry_shell_t *csh)
{
	sd_card_info_t info;
	uint32_t size;
	bk_err_t ret;

	if (s_sd_inited)
		return 0;

	/* 给卡座上电。等价于 disk_io 里 sdcard_ldo_power_enable(1)：
	 * 板上的 3V3 LDO 由 CONFIG_LDO3V3_CTRL_GPIO 控制，不打开的话卡是不通电的，
	 * 现象就是 CMD/DAT 被钳到 0（'gpio 14 1' 读回来 pad=0），卡也不应答。 */
	bk_pm_module_vote_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO,
	                                    CONFIG_LDO3V3_CTRL_GPIO,
	                                    GPIO_OUTPUT_STATE_HIGH);
	rtos_delay_milliseconds(5);

	ret = bk_sd_card_init();
	if (ret != BK_OK) {
		csh_printf(csh, "sd: init failed: %d (CMD 无响应)\r\n", ret);
		csh_printf(csh, "      check: 卡的 3V3 / P14~P19 接线 / CMD 上拉\r\n");
		/* 初始化失败时 SDIO 主机可能还开着（有时钟、有中断），关掉避免留后遗症 */
		bk_sdio_host_deinit();
		return -1;
	}
	s_sd_inited = true;

	os_memset(&info, 0, sizeof(info));
	bk_sd_card_get_card_info(&info);
	size = bk_sd_card_get_card_size();

	csh_printf(csh, "sd: %u blocks x 512B = %u MB, %s, ver %d.x, class %u, RCA 0x%04x\r\n",
	           size, (unsigned)((uint64_t)size * SD_BLK / (1024 * 1024)),
	           sd_type_str(info.card_type), (int)(info.card_version + 1),
	           info.class, info.relative_card_addr);
	return 0;
}

/* sd —— 初始化 + 卡信息 + 读 block0 */
static int sd_cmd_info(chry_shell_t *csh)
{
	uint8_t *buf;
	bk_err_t ret;
	int present = sd_card_present();

	csh_printf(csh, "sd: card detect P%d -> %s\r\n", (int)BOARD_SD_DETECT_PIN,
	           present ? "inserted" : "NOT inserted");

	if (sd_open(csh) != 0)
		return -1;

	buf = os_malloc(SD_BLK);
	if (buf == NULL) {
		csh_printf(csh, "sd: malloc failed\r\n");
		return -1;
	}

	ret = bk_sd_card_read_blocks(buf, 0, 1);
	bk_sd_card_rw_sync();
	if (ret != BK_OK) {
		csh_printf(csh, "sd: read block0 failed: %d\r\n", ret);
		os_free(buf);
		return -1;
	}

	sd_hexdump(csh, buf, 16);
	if ((buf[510] == 0x55) && (buf[511] == 0xAA))
		csh_printf(csh, "sd: block0 signature 0x55AA ok, partition type 0x%02x\r\n",
		           buf[0x1BE + 4]);
	else
		csh_printf(csh, "sd: block0 has no 0x55AA signature (exFAT/无分区表/全 0)\r\n");

	os_free(buf);
	return 0;
}

/* sd r <blk> [n] */
static int sd_cmd_read(chry_shell_t *csh, uint32_t blk, uint32_t n)
{
	uint8_t *buf;
	bk_err_t ret = BK_FAIL;

	if (sd_open(csh) != 0)
		return -1;

	buf = os_malloc((size_t)SD_BLK * n);
	if (buf == NULL) {
		csh_printf(csh, "sd: malloc %u bytes failed\r\n", (unsigned)(SD_BLK * n));
		return -1;
	}

	ret = bk_sd_card_read_blocks(buf, blk, n);
	bk_sd_card_rw_sync();
	if (ret != BK_OK)
		csh_printf(csh, "sd: read blk %u x%u failed: %d\r\n", blk, n, ret);
	else {
		csh_printf(csh, "sd: read blk %u x%u ok\r\n", blk, n);
		sd_hexdump(csh, buf, SD_HEXDUMP_LEN);
	}

	os_free(buf);
	return (ret == BK_OK) ? 0 : -1;
}

/* sd w <blk> [n] —— 写测试模式再读回比对 */
static int sd_cmd_write(chry_shell_t *csh, uint32_t blk, uint32_t n)
{
	uint8_t *wbuf = NULL, *rbuf = NULL;
	bk_err_t ret = BK_FAIL;
	uint32_t i;

	if (sd_open(csh) != 0)
		return -1;

	wbuf = os_malloc((size_t)SD_BLK * n);
	rbuf = os_malloc((size_t)SD_BLK * n);
	if ((wbuf == NULL) || (rbuf == NULL)) {
		csh_printf(csh, "sd: malloc failed\r\n");
		goto out;
	}

	for (i = 0; i < (uint32_t)SD_BLK * n; i++)
		wbuf[i] = (uint8_t)(i + blk);

	ret = bk_sd_card_write_blocks(wbuf, blk, n);
	bk_sd_card_rw_sync();
	if (ret != BK_OK) {
		csh_printf(csh, "sd: write blk %u x%u failed: %d\r\n", blk, n, ret);
		goto out;
	}

	os_memset(rbuf, 0, (size_t)SD_BLK * n);
	ret = bk_sd_card_read_blocks(rbuf, blk, n);
	bk_sd_card_rw_sync();
	if (ret != BK_OK) {
		csh_printf(csh, "sd: read back failed: %d\r\n", ret);
		goto out;
	}

	for (i = 0; i < (uint32_t)SD_BLK * n; i++) {
		if (rbuf[i] != wbuf[i]) {
			csh_printf(csh, "sd: MISMATCH at byte %u: wrote 0x%02x read 0x%02x\r\n",
			           i, wbuf[i], rbuf[i]);
			goto out;
		}
	}

	csh_printf(csh, "sd: write+read blk %u x%u ok, %u bytes verified\r\n",
	           blk, n, (unsigned)(SD_BLK * n));

out:
	if (wbuf)
		os_free(wbuf);
	if (rbuf)
		os_free(rbuf);
	return (ret == BK_OK) ? 0 : -1;
}

/* sd speed [n] */
static int sd_cmd_speed(chry_shell_t *csh, uint32_t n)
{
	uint8_t *buf;
	bk_err_t ret;
	uint32_t t0, ms;

	if (sd_open(csh) != 0)
		return -1;

	buf = os_malloc((size_t)SD_BLK * n);
	if (buf == NULL) {
		csh_printf(csh, "sd: malloc %u bytes failed\r\n", (unsigned)(SD_BLK * n));
		return -1;
	}

	t0 = rtos_get_time();
	ret = bk_sd_card_read_blocks(buf, 0, n);
	bk_sd_card_rw_sync();
	ms = rtos_get_time() - t0;

	if (ret != BK_OK)
		csh_printf(csh, "sd: speed read failed: %d\r\n", ret);
	else
		csh_printf(csh, "sd: read %u blocks (%u KB) in %u ms -> %u KB/s\r\n",
		           n, (unsigned)(n * SD_BLK / 1024), ms,
		           ms ? (unsigned)((uint64_t)n * SD_BLK / 1024 * 1000 / ms) : 0);

	os_free(buf);
	return (ret == BK_OK) ? 0 : -1;
}

/* sd list [path] */
static int sd_cmd_list(chry_shell_t *csh, const char *arg)
{
	char path[32];
	DIR dir;
	FRESULT fr;
	DWORD nclst = 0;
	FATFS *fs = NULL;
	int n = 0;

	if (sd_open(csh) != 0)
		return -1;

	/* 注意盘符：SD 卡是 DISK_NUMBER_SDIO_SD = 1（0 是 RAM 盘，没实现） */
	fr = f_mount(&s_fs, "1:", 1);
	if (fr != FR_OK) {
		csh_printf(csh, "sd: f_mount failed: %d（卡没初始化或不是 FAT 分区）\r\n", fr);
		return -1;
	}

	if ((arg == NULL) || (arg[0] == '\0'))
		os_strcpy(path, "1:/");
	else if (arg[1] == ':')
		os_strcpy(path, arg);
	else
		os_snprintf(path, sizeof(path), "1:/%s", arg);

	fr = f_opendir(&dir, path);
	if (fr != FR_OK) {
		csh_printf(csh, "sd: open dir %s failed: %d\r\n", path, fr);
		f_unmount(1, "1:", 0);
		return -1;
	}

	csh_printf(csh, "sd: %s\r\n", path);
	while (1) {
		fr = f_readdir(&dir, &s_fi);
		if ((fr != FR_OK) || (s_fi.fname[0] == 0))
			break;
		csh_printf(csh, "  %-8s %10u  %s\r\n",
		           (s_fi.fattrib & AM_DIR) ? "<DIR>" : "<FILE>",
		           (unsigned)s_fi.fsize, s_fi.fname);
		n++;
	}
	f_closedir(&dir);

	fr = f_getfree("1:", &nclst, &fs);
	if ((fr == FR_OK) && (fs != NULL))
		csh_printf(csh, "  %d item(s), free %u MB / %u MB\r\n", n,
		           (unsigned)((uint64_t)nclst * fs->csize * SD_BLK / (1024 * 1024)),
		           (unsigned)((uint64_t)(fs->n_fatent - 2) * fs->csize * SD_BLK / (1024 * 1024)));
	else
		csh_printf(csh, "  %d item(s)\r\n", n);

	f_unmount(1, "1:", 0);
	return 0;
}


/* 读 GPIO 的"第二功能模式"编号。
 *
 * 不能用 gpio_ll_get_gpio_perial_mode()：它内部用的是 gpio_ll.h 里
 * 文件级的 static 指针 gpio_system_gpio_func_mode，只有调过
 * gpio_ll_init() 的 .c 才有值；本文件没调过，那份副本是 NULL，
 * 解引用直接 Data Fault → 芯片复位。
 * 这里按 GPIO_LL_SYSTEM_REG_BASE 直接算地址读：
 * 每 8 个 pad 一个 32bit 寄存器，每个 pad 占 4bit。
 *   pad < 32 : gpio_sys_num[0~3] → 0x30+n
 *   pad >= 32: gpio_sys_num[4~6] → 0x80+(n-4)
 */
static uint32_t sd_gpio_func_mode(int pad)
{
	uint32_t base = GPIO_LL_SYSTEM_REG_BASE;
	uint32_t reg_off, shift;

	if (pad < 32) {
		reg_off = 0x30 + (pad / 8);
		shift = (pad % 8) * 4;
	} else {
		reg_off = 0x80 + ((pad - 32) / 8);
		shift = ((pad - 32) % 8) * 4;
	}

	return (REG_READ(base + reg_off * 4) >> shift) & 0xF;
}

/* sd dbg —— 打印 P14~P19 的实际复用状态，用来判断是"引脚没切"还是"时钟没开" */
static int sd_cmd_dbg(chry_shell_t *csh)
{
	static const gpio_id_t pins[] = { GPIO_14, GPIO_15, GPIO_16, GPIO_17, GPIO_18, GPIO_19 };
	static const char *names[] = { "SDIO_CLK", "SDIO_CMD", "SDIO_D0", "SDIO_D1", "SDIO_D2", "SDIO_D3" };
	gpio_hw_t *hw = (gpio_hw_t *)GPIO_LL_REG_BASE;
	int i, ok = 0;

	csh_printf(csh, "sd: gpio mux state (P14~P19 期望 func_en=1 mode=0)\r\n");
	csh_printf(csh, "  pin  func_en  mode  out_en  in_en  level\r\n");

	for (i = 0; i < 6; i++) {
		gpio_id_t p = pins[i];
		uint32_t mode = sd_gpio_func_mode(p);

		csh_printf(csh, "  P%-2d  %3d     %3d    %3d     %3d    %3d\r\n",
		           (int)p,
		           (int)hw->gpio_num[p].cfg.gpio_2_func_en,
		           (int)mode,
		           (int)hw->gpio_num[p].cfg.gpio_output_en,
		           (int)hw->gpio_num[p].cfg.gpio_input_en,
		           (int)hw->gpio_num[p].cfg.gpio_input);

		if ((hw->gpio_num[p].cfg.gpio_2_func_en == 1) && (mode == 0))
			ok++;
	}

	csh_printf(csh, "sd: %d/6 pins muxed to SDIO (mode 0 = %s..%s)\r\n",
	           ok, names[0], names[5]);

	if (ok == 0)
		csh_printf(csh, "  -> 引脚没切到 SDIO：先跑一次 'sd'（会触发初始化+映射）再看\r\n");
	else if (ok == 6)
		csh_printf(csh, "  -> 复用正确；P14 此时应有 ~100kHz 时钟，没有就是 SDIO 时钟没开\r\n");

	return 0;
}

int sd_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];

	if (csh == NULL)                     /* 防御：csh->rl 在结构体偏移 4，NULL 会 MemFault */
		return -1;

	if (argc < 2)
		return sd_cmd_info(csh);

	if (!strcmp(argv[1], "help")) {
		csh_printf(csh, "usage:\r\n");
		csh_printf(csh, "  sd                    init + card info + read block0\r\n");
		csh_printf(csh, "  sd off                deinit\r\n");
		csh_printf(csh, "  sd r <blk> [n]        read blocks, hexdump first 64B\r\n");
		csh_printf(csh, "  sd w <blk> [n]        write pattern + verify (OVERWRITES DATA!)\r\n");
		csh_printf(csh, "  sd speed [n]          sequential read speed (default 64 blocks)\r\n");
		csh_printf(csh, "  sd list [path]        mount FATFS (drive 1:) and list dir\r\n");
		csh_printf(csh, "  sd dbg                dump P14~P19 gpio mux state\r\n");
		return 0;
	}

	if (!strcmp(argv[1], "off")) {
		if (s_sd_inited) {
			bk_sd_card_deinit();
			s_sd_inited = false;
		}
		csh_printf(csh, "sd: deinit\r\n");
		return 0;
	}

	if (!strcmp(argv[1], "dbg"))
		return sd_cmd_dbg(csh);

	if (!strcmp(argv[1], "list"))
		return sd_cmd_list(csh, (argc > 2) ? argv[2] : NULL);

	if (!strcmp(argv[1], "speed")) {
		uint32_t n = SD_SPEED_BLOCKS;

		if (argc > 2) {
			n = (uint32_t)atoi(argv[2]);
			if (n == 0)
				n = SD_SPEED_BLOCKS;
			if (n > SD_MAX_BLOCKS)
				n = SD_MAX_BLOCKS;
		}
		return sd_cmd_speed(csh, n);
	}

	if (!strcmp(argv[1], "r") || !strcmp(argv[1], "w")) {
		uint32_t blk, n = 1;

		if (argc < 3) {
			csh_printf(csh, "usage: sd %s <blk> [n]\r\n", argv[1]);
			return -1;
		}

		blk = (uint32_t)strtoul(argv[2], NULL, 0);
		if (argc > 3) {
			n = (uint32_t)atoi(argv[3]);
			if (n == 0)
				n = 1;
			if (n > SD_MAX_BLOCKS)
				n = SD_MAX_BLOCKS;
		}

		if (argv[1][0] == 'r')
			return sd_cmd_read(csh, blk, n);

		csh_printf(csh, "sd: WARNING: writing blk %u x%u will destroy data there\r\n", blk, n);
		return sd_cmd_write(csh, blk, n);
	}

	csh_printf(csh, "sd: unknown subcommand '%s', try 'sd help'\r\n", argv[1]);
	return -1;
}
