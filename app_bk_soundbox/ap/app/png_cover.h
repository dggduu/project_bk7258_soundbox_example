/*
 * png_cover.h - 流式 PNG 封面解码（降采样直接输出 RGB565）
 *
 * 【为什么不用 libpng / LV_USE_PNG】
 * libpng + LVGL 的 png decoder 需要 ~80KB flash 和较多动态内存；
 * 封面的目标尺寸固定 104x104，我们只需要
 *   1) 解析 PNG chunk（IHDR/PLTE/IDAT/IEND）
 *   2) 用 zlib inflate 解压 IDAT（PNG 的 IDAT 就是 zlib 流）
 *   3) 逐行 unfilter + 边解边缩到目标尺寸
 * ~300 行就能覆盖所有常见封面（8bit 灰度/RGB/RGBA/调色板，非交错）。
 *
 * 【内存】峰值 = zlib 解压窗口 32KB + 两个源行缓冲（w*ch+1 字节 ×2，
 * 3000 宽 RGBA ≈ 24KB）+ 列映射表（w*2B ≈ 6KB）。全部 PSRAM，
 * 与图像尺寸基本无关 —— 3000x3000 和 500x500 用一样的内存。
 *
 * 【JPEG 超大图为什么没有对应方案】
 * SDK 的硬解（含 optimized 模式）和软解都要求 w*h*2 的整帧输出缓冲，
 * 3000x3000 要 17.5MB，PSRAM 放不下 —— 见 app_sd.c 的 skip 保护。
 * JPEG 大图请在 PC 端预处理（tools/shrink_covers.py）。
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief 解码 PNG（内存中的完整字节流）并降采样到 RGB565。
 *
 * @param data PNG 文件原始字节（来自 PSRAM 的 APIC/PICTURE 缓存）
 * @param len  字节长度
 * @param dst  输出 RGB565，尺寸 dw*dh
 * @param dw,dh 目标尺寸（如 104x104）
 * @return true 解码成功；false 格式不支持/数据损坏
 *
 * 支持：bit depth 8；色彩类型 0(灰度)/2(RGB)/3(调色板)/4(灰度+alpha)/6(RGBA)；
 *       无 Adam7 交错；多 IDAT 块。
 * 缩放：box 平均（缩小与放大同一套映射，均为单调 floor 映射，目标必被覆盖）。
 */
bool png_cover_decode(const uint8_t *data, uint32_t len,
                      uint16_t *dst, int dw, int dh);
