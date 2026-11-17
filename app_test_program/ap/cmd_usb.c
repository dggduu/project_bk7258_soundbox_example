// Copyright 2026
// CherrySH 命令：usb —— USB 虚拟串口（CDC ACM）测试
//
//   usb                 显示枚举状态 / 收发统计 / 主机设置的行参数
//   usb init            启动 USB 设备模式（ap_main 已经默认启动，这里可重复调用）
//   usb deinit          关闭 USB，释放资源
//   usb hello           发一条 "hello from cherryUSB!"
//   usb send <text>     发任意文本（多个参数会用空格拼起来）
//   usb recv            把主机发来的数据以 hex + ascii 打出来
//
// 主机侧对应设备：
//   Linux    /dev/ttyACM0    （cat /dev/ttyACM0 能看到 hello）
//   Windows  COMx            （设备管理器里是 "USB 串行设备"）
//   macOS    /dev/tty.usbmodemXXXX
//
// 命令注册在 ap/csh_port.c 的 s_cmd_table[] 里。

#include <string.h>
#include <stdio.h>

#include "csh.h"
#include "csh_port.h"
#include "usb_vcom.h"

#define USB_CMD_TEXT_MAX    128

static void usb_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  usb                 show status\r\n");
	csh_printf(csh, "  usb init|deinit     start/stop usb device mode\r\n");
	csh_printf(csh, "  usb hello           send greeting\r\n");
	csh_printf(csh, "  usb send <text>     send text\r\n");
	csh_printf(csh, "  usb recv            dump received data\r\n");
}

static void usb_show_status(chry_shell_t *csh)
{
	uint32_t baud;
	uint8_t bits, parity, stop;

	usb_vcom_get_line_coding(&baud, &bits, &parity, &stop);

	csh_printf(csh, "usb cdc acm vcom (BK7258 dedicated usb port)\r\n");
	csh_printf(csh, "  configured : %s\r\n", usb_vcom_is_configured() ? "yes" : "no");
	csh_printf(csh, "  dtr        : %s\r\n", usb_vcom_is_dtr() ? "on" : "off");
	csh_printf(csh, "  line coding: %u %u%c%u\r\n", (unsigned)baud,
	           (unsigned)bits, (parity == 0) ? 'N' : ((parity == 1) ? 'O' : 'E'),
	           (unsigned)(stop + 1));
	csh_printf(csh, "  tx bytes   : %u\r\n", (unsigned)usb_vcom_tx_total());
	csh_printf(csh, "  rx bytes   : %u\r\n", (unsigned)usb_vcom_rx_total());
}

/* 打一行 hex dump：AAAA: xx xx ... |ascii| */
static void usb_dump_line(chry_shell_t *csh, uint32_t addr, const uint8_t *d, uint32_t len)
{
	char line[96];
	int pos = 0;

	pos += snprintf(&line[pos], sizeof(line) - pos, "%04x: ", (unsigned)addr);

	for (uint32_t i = 0; i < 16; i++) {
		if (i < len)
			pos += snprintf(&line[pos], sizeof(line) - pos, "%02x ", d[i]);
		else
			pos += snprintf(&line[pos], sizeof(line) - pos, "   ");
	}

	pos += snprintf(&line[pos], sizeof(line) - pos, " |");
	for (uint32_t i = 0; i < len; i++)
		pos += snprintf(&line[pos], sizeof(line) - pos, "%c",
		                ((d[i] >= 0x20) && (d[i] < 0x7f)) ? d[i] : '.');
	snprintf(&line[pos], sizeof(line) - pos, "|");

	csh_printf(csh, "%s\r\n", line);
}

static int usb_dump_rx(chry_shell_t *csh)
{
	uint8_t buf[64];
	uint8_t line[16];
	uint32_t line_len = 0;
	uint32_t addr = 0;
	uint32_t total = 0;
	uint32_t n;

	while ((n = usb_vcom_read(buf, sizeof(buf))) > 0) {
		for (uint32_t i = 0; i < n; i++) {
			line[line_len++] = buf[i];
			total++;
			if (line_len == sizeof(line)) {
				usb_dump_line(csh, addr, line, line_len);
				addr += line_len;
				line_len = 0;
			}
		}
	}

	if (line_len)
		usb_dump_line(csh, addr, line, line_len);

	if (total == 0)
		csh_printf(csh, "no received data\r\n");
	else
		csh_printf(csh, "%u byte(s)\r\n", (unsigned)total);

	return 0;
}

int usb_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];

	if (argc < 2) {
		usb_show_status(csh);
		return 0;
	}

	if (!strcmp(argv[1], "init")) {
		bk_err_t ret = usb_vcom_init();

		csh_printf(csh, "usb_vcom_init -> %d\r\n", ret);
		return (ret == BK_OK) ? 0 : -1;
	}

	if (!strcmp(argv[1], "deinit")) {
		bk_err_t ret = usb_vcom_deinit();

		csh_printf(csh, "usb_vcom_deinit -> %d\r\n", ret);
		return (ret == BK_OK) ? 0 : -1;
	}

	if (!strcmp(argv[1], "hello")) {
		int ret = usb_vcom_send(USB_VCOM_GREETING, sizeof(USB_VCOM_GREETING) - 1);

		csh_printf(csh, "send %u bytes -> %d\r\n",
		           (unsigned)(sizeof(USB_VCOM_GREETING) - 1), ret);
		return (ret > 0) ? 0 : -1;
	}

	if (!strcmp(argv[1], "send")) {
		char text[USB_CMD_TEXT_MAX];
		uint32_t len = 0;
		int ret;

		if (argc < 3) {
			usb_usage(csh);
			return -1;
		}

		for (int i = 2; (i < argc) && (len < sizeof(text) - 2); i++) {
			uint32_t alen = strlen(argv[i]);

			if (i > 2)
				text[len++] = ' ';
			if (alen > sizeof(text) - 2 - len)
				alen = sizeof(text) - 2 - len;
			memcpy(&text[len], argv[i], alen);
			len += alen;
		}
		text[len++] = '\r';
		text[len++] = '\n';

		ret = usb_vcom_send(text, len);
		csh_printf(csh, "send %u bytes -> %d\r\n", (unsigned)len, ret);
		return (ret > 0) ? 0 : -1;
	}

	if (!strcmp(argv[1], "recv")) {
		return usb_dump_rx(csh);
	}

	usb_usage(csh);
	return -1;
}
