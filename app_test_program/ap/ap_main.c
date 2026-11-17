// app_test_uart1
// UART1 上的 CherrySH 命令行 + 板上几路电源使能（LDO_EN）测试 + USB 虚拟串口
//
// 接线（BK7258 / app_bk_pi 板）：
//   UART1_TXD = GPIO0(P0)   -> USB-TTL 的 RX
//   UART1_RXD = GPIO1(P1)   -> USB-TTL 的 TX
//   GND 共地，115200 8N1
//   注意 GPIO0/1 和 SWDIO/SWCLK 复用，用这个 shell 时不能再接 SWD
//
// 日志口仍然是 UART0（GPIO11 TX / GPIO10 RX），AP 的 LOG 经 mailbox 由 CP 输出。
//
// USB：BK7258 板载 USB 口（芯片专用 DP/DM，不占 GPIO）枚举成 CDC ACM 串口，
//      主机侧是 /dev/ttyACM0（Linux）或 COMx（Windows），
//      主机一打开这个串口就会自动收到一条 "hello from cherryUSB!"。
//
// 常用命令（在 UART1 终端里）：
//   help
//   ldo                      列出 scr/2v8/1v5/mic/pa 五路电源使能
//   ldo 2v8 on               打开摄像头 2V8
//   ldo 2v8 off / toggle
//   ldo all on|off
//   board                    打印板级引脚和当前电源状态
//   usb                      打印 USB 虚拟串口状态
//   usb hello                再发一次 hello from cherryUSB!
//   usb send <text>          从 USB 发一段文本给主机
//   usb recv                 dump 主机发来的数据
//   ov3660 (或 cam)           OV3660 摄像头状态 / SCCB 寄存器调试
//   ov3660 probe             上电 + MCLK + 读 PID（起步就打这一条）
//   lcdq (或 qspi / lcd)      QSPI 屏 ST77916 360x360 调试
//   lcdq init                断电复位 + 上电 + 初始化（起步就打这一条）
//   lcdq bars                8 色条；lcdq fill 0xF800 纯红

#include "bk_private/bk_init.h"
#include <components/system.h>
#include <os/os.h>
#include <components/log.h>

#include "bsp_power.h"
#include "csh_port.h"
#include "usb_vcom.h"

#define TAG "app"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

int main(void)
{
	bk_err_t ret;

	bk_init();

	/* 板上所有电源使能脚先拉到关闭状态（bsp_power_init 里会解复用并配成输出） */
	ret = bsp_power_init();
	if (ret != BK_OK) {
		LOGE("power init failed: %d\r\n", ret);
		return -1;
	}

	LOGI("uart1 cherrysh + ldo test start\r\n");

	ret = csh_port_start();
	if (ret != BK_OK) {
		LOGE("cherrysh start failed: %d\r\n", ret);
		return -1;
	}

	/* USB 虚拟串口：bk_init() 里已经做过 bk_usb_driver_init()，
	 * 这里注册 CDC ACM 描述符并打开 USB 设备模式。失败不阻塞 shell。 */
	ret = usb_vcom_init();
	if (ret != BK_OK)
		LOGE("usb vcom init failed: %d\r\n", ret);

	/* shell 在独立任务里跑，main 保持存活即可 */
	while (1)
		rtos_delay_milliseconds(1000);

	return 0;
}
