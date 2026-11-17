// Copyright 2026
// BK7258 board pin map for app_bk_pi (立创开源 BK7258 模组)
//
// 引脚来源：嘉立创EDA 专业版工程「基于 BK7258 模组的蓝牙小音箱」
//   工程 UUID : 9a5cc9de3ea54f6091ee08597e58ea11
//   原理图页  : Block / MCU / 电源 / 外设 / 音频 / 外部接口
// 通过 EasyEDA Pro API 读取网表（EasyEDA 2.0.0 netlist）得到，非猜测。
//
// 约定：BOARD_xxx_PIN 全部使用 gpio_id_t（GPIO_x），与模组丝印 Pxx 一一对应。

#pragma once

#include <driver/gpio.h>
#include <driver/adc_types.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. BK7258 模组 U1 (LC-OPEN-BK7258-MOD)
 * ========================================================================== */

/* --- 调试串口 / SWD --- */
#define BOARD_UART0_TX_PIN          GPIO_11     /* P11/TXD  -> UART0_TX */
#define BOARD_UART0_RX_PIN          GPIO_10     /* P10/RXD  -> UART0_RX */
#define BOARD_SWDIO_PIN             GPIO_0      /* P0       -> SWDIO  */
#define BOARD_SWCLK_PIN             GPIO_1      /* P1       -> SWCLK  */

/* --- 屏幕：QSPI1 组（硬件固定 GPIO2~GPIO7） --- */
#define BOARD_LCD_QSPI_CLK_PIN      GPIO_2      /* P2  LCD_CLK -> QSPI1_CLK  */
#define BOARD_LCD_QSPI_CS_PIN       GPIO_3      /* P3  LCD_CS  -> QSPI1_CSN  */
#define BOARD_LCD_QSPI_D0_PIN       GPIO_4      /* P4  LCD_D0  -> QSPI1_IO0  */
#define BOARD_LCD_QSPI_D1_PIN       GPIO_5      /* P5  LCD_D1  -> QSPI1_IO1  */
#define BOARD_LCD_QSPI_D2_PIN       GPIO_6      /* P6  LCD_D2  -> QSPI1_IO2  */
#define BOARD_LCD_QSPI_D3_PIN       GPIO_7      /* P7  LCD_D3  -> QSPI1_IO3  */

/* --- 屏幕：SPI 模式备选（同一组排针，P4=MOSI / P5=MISO） --- */
#define BOARD_LCD_SPI_SCK_PIN       GPIO_2      /* P2  LCD_CLK -> SPI1_SCK   */
#define BOARD_LCD_SPI_CS_PIN        GPIO_3      /* P3  LCD_CS  -> SPI1_CSN   */
#define BOARD_LCD_SPI_MOSI_PIN      GPIO_4      /* P4  LCD_D0  -> SPI1_MOSI  */
#define BOARD_LCD_SPI_MISO_PIN      GPIO_5      /* P5  LCD_D1  -> SPI1_MISO  */

/* --- 屏幕：分辨率 / 色深（沿用屏幕厂家驱动里的命名） --- */
#define BSP_LCD_H_RES               360
#define BSP_LCD_V_RES               360
#define BSP_LCD_BPP                 16      /* RGB565 */

/* --- 屏幕：背光 / 触摸 / 复位 --- */
#define BOARD_LCD_BACKLIGHT_PIN     GPIO_23     /* P23 LCD_BLC，直连屏背光，无 MOS */
#define BOARD_LCD_BACKLIGHT_PWM_CH  3           /* GPIO23 第二功能 PWM3（调光用） */
#define BOARD_TP_INT_PIN            GPIO_53     /* P53 TP_INT，触摸中断 */
#define BOARD_TP_I2C_ID             1           /* 触摸走 I2C1（与 SC7A20 共用） */
#define BOARD_TP_I2C_ADDR           0x15        /* CST816S 7bit 地址 */
#define BOARD_TP_I2C_SCL_PIN        GPIO_42     /* P42 I2C1_SCL（触摸 + SC7A20） */
#define BOARD_TP_I2C_SDA_PIN        GPIO_43     /* P43 I2C1_SDA */

/* 屏复位脚在排针 H2 上接 CEN（与模组 RST 同网络），软件不可单独控制，
 * 只能用 BOARD_PWR_SCR_EN 断电复位。见 bsp_screen.c 的复位时序。 */

/* --- 摄像头：DVP 8bit + SCCB(I2C0) + MCLK --- */
#define BOARD_CIS_MCLK_PIN          GPIO_27     /* P27 CIS_MCLK  */
#define BOARD_CIS_PCLK_PIN          GPIO_29     /* P29 CIS_PCLK  */
#define BOARD_CIS_HSYNC_PIN         GPIO_30     /* P30 CIS_HSYNC */
#define BOARD_CIS_VSYNC_PIN         GPIO_31     /* P31 CIS_VSYNC */
#define BOARD_CIS_D0_PIN            GPIO_32     /* P32 CIS_PSD0  */
#define BOARD_CIS_D1_PIN            GPIO_33     /* P33 CIS_PSD1  */
#define BOARD_CIS_D2_PIN            GPIO_34     /* P34 CIS_PSD2  */
#define BOARD_CIS_D3_PIN            GPIO_35     /* P35 CIS_PSD3  */
#define BOARD_CIS_D4_PIN            GPIO_36     /* P36 CIS_PSD4  */
#define BOARD_CIS_D5_PIN            GPIO_37     /* P37 CIS_PSD5  */
#define BOARD_CIS_D6_PIN            GPIO_38     /* P38 CIS_PSD6  */
#define BOARD_CIS_D7_PIN            GPIO_39     /* P39 CIS_PSD7  */
#define BOARD_CAM_I2C_ID            0           /* CAM_I2C_SCL=P20 / SDA=P21 -> I2C0 */
#define BOARD_CAM_I2C_SCL_PIN       GPIO_20
#define BOARD_CAM_I2C_SDA_PIN       GPIO_21

/* 摄像头 PWDN(FPC1 pin17) 经 R38 100k 下拉到 GND，固定为工作态，无 GPIO。
 * 摄像头 RESET(FPC1 pin19) 接 CEN，与模组复位同网络，软件不可控。 */

/* --- SD 卡：SDIO 4bit（硬件固定 GPIO14~GPIO19） --- */
#define BOARD_SDIO_CLK_PIN          GPIO_14     /* P14 SDIO_CLK */
#define BOARD_SDIO_CMD_PIN          GPIO_15     /* P15 SDIO_CMD */
#define BOARD_SDIO_D0_PIN           GPIO_16     /* P16 SDIO_D0  */
#define BOARD_SDIO_D1_PIN           GPIO_17     /* P17 SDIO_D1  */
#define BOARD_SDIO_D2_PIN           GPIO_18     /* P18 SDIO_D2  */
#define BOARD_SDIO_D3_PIN           GPIO_19     /* P19 SDIO_D3  */
#define BOARD_SD_DETECT_PIN         GPIO_40     /* P40 SD_DET，卡座 CD 脚，低有效 */
#define BOARD_SD_DETECT_ACTIVE_LOW  1

/* --- 按键：ADC 分压键盘，接 P28/ADC4 --- */
#define BOARD_KEY_ADC_PIN           GPIO_28     /* P28 ADC4 */
#define BOARD_KEY_ADC_CHAN          ADC_4
/* 分压网络：R29 100k 上拉到 3V3，各键经不同电阻下拉到 GND
 *   KEY1 = SW1 + R28 220k -> 3.3 * 220/320 = 2.26875V
 *   KEY2 = SW2 + R30 100k -> 3.3 * 100/200 = 1.65V
 *   KEY3 = SW3 + R33  47k -> 3.3 *  47/147 = 1.055V
 *   KEY4 = SW4 直连      -> 0V
 */
#define BOARD_KEY1_MV               2269
#define BOARD_KEY2_MV               1650
#define BOARD_KEY3_MV               1055
#define BOARD_KEY4_MV               0
#define BOARD_KEY_ADC_VREF_MV       3300
#define BOARD_KEY_ADC_SAMPLE_NUM    8

/* --- 电池检测：BAT_ADC = P22/ADC5，R21 220k / R24 110k = 1:3 分压 ---
 *
 * 【2026-11 硬件改版】原来是 R21/R24 双 220k（1:2），4.2V 满电 -> 2.1V，
 * 超过 ADC 约 2048mV 的量程上限，实测 raw 常驻 8190/8191（饱和），
 * 满电段完全读不出来。改成 R24=110k（1:3）后：
 *   4.2V -> 1400mV -> raw ≈ 5600，3.0V -> 1000mV -> raw ≈ 4000，
 *   全区间落在量程内且分辨率足够。
 * 若实际贴的是别的比例，只改 BOARD_BAT_ADC_DIV_NUM 即可（电压 = 分压点 × N）。 */
#define BOARD_BAT_ADC_PIN           GPIO_22     /* P22 BAT_ADC */
#define BOARD_BAT_ADC_CHAN          ADC_5
#define BOARD_BAT_ADC_DIV_NUM       3           /* (R21+R24)/R24 = (220+110)/110 = 3 */
#define BOARD_BAT_ADC_VREF_MV       3300
#define BOARD_BAT_ADC_EN_PIN        GPIO_49     /* P49 BAT_ADC_EN，PMOS Q2 栅极，低有效 */
#define BOARD_BAT_ADC_EN_ACTIVE_LOW 1

/* --- 电源使能 --- */
#define BOARD_PWR_SCR_EN_PIN        GPIO_51     /* P51 PWR_SCR_EN，PMOS Q5，低有效：屏 3V3 */
#define BOARD_PWR_SCR_EN_ACTIVE_LOW 1
#define BOARD_PWR_2V8_EN_PIN        GPIO_50     /* P50 PWR_2V8_EN，LDO U10 EN，高有效 */
#define BOARD_PWR_2V8_EN_ACTIVE_LOW 0
#define BOARD_PWR_1V5_EN_PIN        GPIO_26     /* P26 PWR_1V5_EN，LDO U9  EN，高有效 */
#define BOARD_PWR_1V5_EN_ACTIVE_LOW 0
#define BOARD_PWR_MIC_EN_PIN        GPIO_48     /* P48 PWR_MIC_EN，LDO U8  EN，高有效：MIC 1V8 */
#define BOARD_PWR_MIC_EN_ACTIVE_LOW 0
#define BOARD_PA_EN_PIN             GPIO_52     /* P52 PA_EN，NS4150B CTRL，高有效 */
#define BOARD_PA_EN_ACTIVE_LOW      0

/* --- 其它已引出但 BSP 不占用 --- */
#define BOARD_P12_PIN               GPIO_12     /* H6 */
#define BOARD_P24_PIN               GPIO_24     /* H6 */
#define BOARD_P25_PIN               GPIO_25     /* H6 */
#define BOARD_P41_PIN               GPIO_41     /* H7 */
#define BOARD_P44_PIN               GPIO_44     /* H7 */
#define BOARD_P45_PIN               GPIO_45     /* H7 */
#define BOARD_P46_PIN               GPIO_46     /* H7 */
#define BOARD_P47_PIN               GPIO_47     /* H7 */

/* --- 音频 --- */
#define BOARD_DMIC_CLK_PIN          GPIO_8      /* P8  DMIC_CLK */
#define BOARD_DMIC_DATA_PIN         GPIO_9      /* P9  DMIC_DAT */

/* --- USB：芯片专用 USB 2.0 口，不占用任何 GPIO ---
 *
 * BK7258 的 USB DP/DM 是芯片自带的专用引脚（原理图上就是 USB_DP / USB_DN），
 * 没有 GPIO 复用编号，所以这里没有 pin 需要配置，也不该去配。
 * 依据：ap/middleware/driver/include/bk_private/bk_gpio.h 里的
 * GPIO_USB_DP_PIN / GPIO_USB_DN_PIN 只对 BK7231U/BK7251/BK7271/BK7256 定义，
 * BK7258（CONFIG_SOC_BK7258）没有这两个宏。
 *
 * 板上也没给 USB 供电做 GPIO 控制：插上 USB 线即有 5V，PHY 由芯片内部 LDO 供。
 * 注意 SDK 默认配置里 CONFIG_USB_VBAT_CONTROL_GPIO_ID = 0x1C（GPIO28）是给
 * BK7258 EVB 的 VBUS 控制脚准备的；本板上 GPIO28 是 ADC4（按键分压网络），
 * 所以做 USB 设备（从机）时不要调 bk_usb_power_ops()，否则会把按键 ADC 驱动坏。
 */
#define BOARD_USB_PORT_IS_DEDICATED 1
#define BOARD_USB_VBUS_CTRL_PIN     0xFF        /* 0xFF = 板上没有这一路控制脚 */
#define BOARD_USB_FS_BULK_MPS       64          /* USB 2.0 全速，bulk 端点最大包长 */

/* ==========================================================================
 * 2. 板上器件 I2C 地址（7bit）
 * ========================================================================== */
#define BOARD_SC7A20_I2C_ADDR       0x18        /* SDO 悬空/内部下拉 -> 0x18，探测时兼容 0x19 */
#define BOARD_SC7A20_I2C_ADDR_ALT   0x19
#define BOARD_OV3660_I2C_ADDR       0x3C        /* SCCB 8bit 0x78 >> 1 */

/* ==========================================================================
 * 3. 上电时序参数（ms）
 * ========================================================================== */
#define BOARD_PWR_SETTLE_MS         20          /* LDO 使能到电压稳定 */
#define BOARD_CAM_PWR_SETTLE_MS     30          /* 摄像头 2V8/1V5 上电到可访问 SCCB */
#define BOARD_CAM_PWR_OFF_MS        200         /* 断电后等电容放完，保证 power cycle 有效 */
/* ST77916 手册要求上电后 ≥120ms 才能收初始化命令；30ms 会让整段 init
 * 被屏忽略，现象是花屏且之后 frame_display 也不更新（app_bk_pi 上实测）。 */
#define BOARD_SCR_PWR_SETTLE_MS     120         /* 屏电源上电到可发初始化命令 */
#define BOARD_SENSOR_RESET_MS       20          /* sensor 软复位后等待 */

#ifdef __cplusplus
}
#endif
