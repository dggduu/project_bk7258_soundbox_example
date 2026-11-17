// Copyright 2026
// USB 模式：CherryUSB UAC1 声卡（48kHz / 16bit / 立体声）
//
// 电脑把本机识别为 USB Speaker，主机下发的 PCM 经环形缓冲送到 DAC。
// 采样率固定 48k，主机侧音量通过 UAC Feature Unit 映射到本地音量。

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void app_usb_start(void);
void app_usb_set_active(bool active);
void app_usb_poll(void);

/* USB 麦克风模式开关：只有打开时才会真正启动 DMIC 采集并送给主机。
 * 普通 USB 声卡模式下降音（主机能看到麦克风但收不到数据）。 */
void app_usb_set_mic_mode(bool on);

bool app_usb_is_streaming(void);
bool app_usb_is_connected(void);

#ifdef __cplusplus
}
#endif
