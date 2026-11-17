// Copyright 2026
// CherrySH 移植层：shell 跑在 UART1（GPIO0=TX / GPIO1=RX，115200 8N1）
//
// AP 的日志走 mailbox -> CP -> UART0（GPIO10/11），所以 UART1 可以独占给 shell。
// 注意：GPIO0/GPIO1 和 SWDIO/SWCLK 是复用脚，用 UART1 shell 时不能同时接 SWD。

#pragma once

#include <common/bk_err.h>
#include "csh.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 取 shell 实例，命令里用 csh_printf(csh_port_shell(), ...) 往 UART1 打印。
 */
chry_shell_t *csh_port_shell(void);

/**
 * @brief 初始化 UART1 + CherrySH，并创建 shell 任务。
 *
 * @return BK_OK 成功
 */
bk_err_t csh_port_start(void);

#ifdef __cplusplus
}
#endif
