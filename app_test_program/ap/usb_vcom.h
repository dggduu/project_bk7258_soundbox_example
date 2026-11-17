// Copyright 2026
// USB 虚拟串口（CDC ACM）—— 对外接口
//
// 用 SDK 里的 CherryUSB 设备栈把 BK7258 的 USB 口枚举成一个串口设备：
//   Linux    -> /dev/ttyACM0
//   Windows  -> COMx（Win10 及以上自带 usbser.sys，无需装驱动）
//   macOS    -> /dev/tty.usbmodemXXXX
//
// 枚举成功后主机侧会出现 /dev/ttyACM0（Linux）或 COMx（Windows）；
// 主机一打开这个串口（DTR 置位）设备就会自动发一条 "hello from cherryUSB!"，
// 主机发来的数据会进环形缓冲，用 shell 命令 `usb recv` 取出来。

#pragma once

#include <stdbool.h>
#include <stdint.h>
#include <common/bk_err.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 枚举成功后自动发出的字符串 */
#define USB_VCOM_GREETING   "hello from cherryUSB!\r\n"

/**
 * @brief 初始化 USB 设备（CDC ACM 虚拟串口）并打开 USB 设备模式。
 *
 * 必须在 bk_init() 之后调用（bk_init 里已经做了 bk_usb_driver_init）。
 *
 * @return BK_OK 成功
 */
bk_err_t usb_vcom_init(void);

/**
 * @brief 停掉 USB 设备侧（关 USB 中断、PHY、时钟），主机侧设备会消失。
 *
 * 停完还可以再 usb_vcom_init() 重新枚举。
 */
bk_err_t usb_vcom_deinit(void);

/**
 * @brief 通过虚拟串口向主机发送数据（阻塞，直到数据进了 USB 端点）。
 *
 * @param data 数据
 * @param len  长度
 * @return >=0 实际发送的字节数；<0 失败（没枚举完成 / 端点忙）
 */
int usb_vcom_send(const void *data, uint32_t len);

/**
 * @brief 从接收环形缓冲里取走主机发来的数据。
 *
 * @return 实际取到的字节数
 */
uint32_t usb_vcom_read(uint8_t *buf, uint32_t len);

/** @brief 主机是否已经完成配置（SET_CONFIGURATION） */
bool usb_vcom_is_configured(void);

/** @brief 主机应用是否打开了串口（DTR 置位） */
bool usb_vcom_is_dtr(void);

/** @brief 累计收到的字节数 */
uint32_t usb_vcom_rx_total(void);

/** @brief 累计发出的字节数 */
uint32_t usb_vcom_tx_total(void);

/** @brief 拿到主机设置的行参数（波特率等） */
void usb_vcom_get_line_coding(uint32_t *baud, uint8_t *data_bits,
                              uint8_t *parity, uint8_t *stop_bits);

#ifdef __cplusplus
}
#endif
