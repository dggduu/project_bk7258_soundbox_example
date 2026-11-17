#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* 收到 AVRCP DEFAULT_COVER_ART 的 image handle 后启动一次最小 BIP 拉取。 */
void bt_cover_art_on_handle(const char *handle, uint32_t len);

/* UI 侧取封面：104x104 RGB565，已按 LV_COLOR_16_SWAP 处理 */
bool bt_cover_art_copy_cover(uint16_t *dst, int w, int h);
uint32_t bt_cover_art_generation(void);

#ifdef __cplusplus
}
#endif
