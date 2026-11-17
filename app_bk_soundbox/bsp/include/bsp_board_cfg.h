// Copyright 2026
// BK7258 三模音箱板级配置（app_bk_pi）
//
// 硬件：立创开源「基于 BK7258 模组的蓝牙小音箱」底板 + ILI9341 2.8 寸
// 240x320 单线 SPI 屏。引脚来自嘉立创 EDA 网表，屏的接法来自
// projects/app_test_screen 已验证的接线：
//
//   LCD  : SCLK=P2  CS=P3  MOSI=P4  DC=P5  RST=P6
//          背光=P23（GPIO 直驱，高亮），屏 3V3 使能=P51（PMOS，低有效）
//   KEY  : ADC 分压键盘 P28/ADC4，四档 2.269/1.650/1.055/0 V
//   SD   : SDIO 4line，CLK=P14 CMD=P15 D0~D3=P16~P19，CD=P40（低有效）
//          卡座 3V3 LDO 由 GPIO13 控制（CONFIG_LDO3V3_CTRL_GPIO=13）
//   AUDIO: 内部 DAC AUDLP/AUDN -> NS4150B 差分功放 -> 喇叭，PA_EN=P52
//   USB  : BK7258 专用 USB DP/DM，不占 GPIO
//
// 本工程不启用摄像头、触摸和 MIC（咪头硬件有问题，先排除）。

#pragma once

#include <driver/adc_types.h>
#include <driver/gpio.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 1. BK7258 模组 U1 (LC-OPEN-BK7258-MOD)
 * ========================================================================== */

/* --- 调试串口 / SWD --- */
#define BOARD_UART0_TX_PIN          GPIO_11     /* P11/TXD -> UART0_TX */
#define BOARD_UART0_RX_PIN          GPIO_10     /* P10/RXD -> UART0_RX */
#define BOARD_SWDIO_PIN             GPIO_0      /* P0      -> SWDIO  */
#define BOARD_SWCLK_PIN             GPIO_1      /* P1      -> SWCLK  */

/* --- 屏幕：ST77916 360x360 圆形 QSPI，4 线（P2~P7 排针组） ---
 *
 * 【2026-11 换屏】原 ILI9341 240x320 单线 SPI 已换成 BOE 1.8" 360x360
 * 圆形屏（ST77916，QSPI 4 线）。接线仍是同一组排针，但用法变了：
 *   P2=CLK  P3=CSN  P4=IO0  P5=IO1  P6=IO2  P7=IO3
 * 注意：ILI9341 方案里 P5=DC / P6=RST 是**软件 GPIO**，换屏后这两脚
 * 变成 QSPI 数据线，不能再当 GPIO 用；RST 也不再单独引出（本板 RST
 * 接在 CSN 上，所以 bk_lcd_qspi_init() 的 reset_pin 传 GPIO_NUM 跳过
 * 硬复位，与 app_test_uart1 的 LCDQ_RESET_PIN 一致）。
 *
 * 引脚映射**不要手工 gpio_dev_map**：lcd_qspi_driver.c 的 qspi 初始化
 * 自己会 map，usr_gpio_cfg.h 里保持 GPIO_DEV_INVALID 让它释放即可。 */
#define BOARD_LCD_QSPI_ID           1           /* QSPI_ID_1 */
#define BOARD_LCD_QSPI_CLK_PIN      GPIO_2
#define BOARD_LCD_QSPI_CS_PIN       GPIO_3
#define BOARD_LCD_QSPI_D0_PIN       GPIO_4
#define BOARD_LCD_QSPI_D1_PIN       GPIO_5
#define BOARD_LCD_QSPI_D2_PIN       GPIO_6
#define BOARD_LCD_QSPI_D3_PIN       GPIO_7
#define BOARD_LCD_USE_HW_RESET      0           /* RST 接 CSN，驱动跳过硬复位 */

/* --- 屏幕参数：360x360 圆形，RGB565 --- */
#define BSP_LCD_H_RES               360
#define BSP_LCD_V_RES               360
#define BSP_LCD_BPP                 16
#define BSP_LCD_IS_ROUND            1

/* 圆屏直径（用于 UI 安全区计算，等于 H_RES） */
#define BSP_LCD_DIAMETER            BSP_LCD_H_RES

/* 背光：P23 直连，无 MOS，高电平点亮 */
#define BOARD_LCD_BACKLIGHT_PIN     GPIO_23

/* --- SD 卡：SDIO 4bit（硬件固定 P14~P19） --- */
#define BOARD_SDIO_CLK_PIN          GPIO_14
#define BOARD_SDIO_CMD_PIN          GPIO_15
#define BOARD_SDIO_D0_PIN           GPIO_16
#define BOARD_SDIO_D1_PIN           GPIO_17
#define BOARD_SDIO_D2_PIN           GPIO_18
#define BOARD_SDIO_D3_PIN           GPIO_19
#define BOARD_SD_DETECT_PIN         GPIO_40     /* 卡座 CD，低有效 */
#define BOARD_SD_DETECT_ACTIVE_LOW  1

/* 卡座 3V3 LDO 的使能脚。SDK 的 disk_io 通过
 * bk_pm_module_vote_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO,
 * CONFIG_LDO3V3_CTRL_GPIO, HIGH) 打开；Kconfig 里的
 * CONFIG_LDO3V3_CTRL_GPIO 必须与这里一致（当前板是 GPIO13）。 */
#define BOARD_SD_LDO_CTRL_PIN       GPIO_13

/* --- 按键：ADC 分压键盘 P28/ADC4 --- */
#define BOARD_KEY_ADC_PIN           GPIO_28
#define BOARD_KEY_ADC_CHAN          ADC_4
/*   KEY1 = SW1 + R28 220k -> 3.3 * 220/320 = 2.269 V
 *   KEY2 = SW2 + R30 100k -> 3.3 * 100/200 = 1.650 V
 *   KEY3 = SW3 + R33  47k -> 3.3 *  47/147 = 1.055 V
 *   KEY4 = SW4 直连      -> 0 V
 *   无按键经 R29 100k 上拉到 3.3 V */
#define BOARD_KEY1_MV               2269
#define BOARD_KEY2_MV               1650
#define BOARD_KEY3_MV               1055
#define BOARD_KEY4_MV               0
#define BOARD_KEY_ADC_VREF_MV       3300
#define BOARD_KEY_ADC_SAMPLE_NUM    8

/* --- 电池检测：P22/ADC5，R21 220k / R24 110k 三分压 ---
 * 分压电阻常接，ADC 直接读 VBAT/3，软件乘 DIV_NUM 还原。
 * 原来的 PMOS 使能脚（P49，bat_adc_en）因电路问题已取消，不再占用 P49。
 *
 * 【2026-11 变更】R24 由 220k 改 110k，分压比 1:2 -> 1:3。
 * 原因：ADC 实际量程只有 ~2048mV，而满电 4.2V/2 = 2.1V 已经越界，
 * 原方案在满电时会饱和、读数恒定封顶。改 1:3 后：
 *   4.2V -> 1400mV（raw ≈ 5600）  余量充足
 *   3.7V -> 1233mV（raw ≈ 4933）
 *   3.0V -> 1000mV（raw ≈ 4000）  仍在有效区
 * 源阻抗 220k||110k = 73.3k，比 1:2 的 110k 更低，采样精度反而更好。 */
#define BOARD_BAT_ADC_PIN           GPIO_22
#define BOARD_BAT_ADC_CHAN          ADC_5
#define BOARD_BAT_ADC_DIV_NUM       3
#define BOARD_BAT_ADC_VREF_MV       3300

/* --- 电源使能 --- */
#define BOARD_PWR_SCR_EN_PIN        GPIO_51     /* 屏 3V3，PMOS，低有效 */
#define BOARD_PWR_SCR_EN_ACTIVE_LOW 1
#define BOARD_PWR_2V8_EN_PIN        GPIO_50     /* 摄像头 2V8，高有效（本工程不用） */
#define BOARD_PWR_2V8_EN_ACTIVE_LOW 0
#define BOARD_PWR_1V5_EN_PIN        GPIO_26     /* 摄像头 1V5，高有效（本工程不用） */
#define BOARD_PWR_1V5_EN_ACTIVE_LOW 0
#define BOARD_PWR_MIC_EN_PIN        GPIO_48     /* MIC 1V8，PMOS 栅极，低有效（原 LDO U8 高有效，已换 PMOS 省静态电流） */
#define BOARD_PWR_MIC_EN_ACTIVE_LOW 1
#define BOARD_PA_EN_PIN             GPIO_52     /* NS4150B CTRL，高有效 */
#define BOARD_PA_EN_ACTIVE_LOW      0

/* --- 其它已引出但本工程不占用 --- */
#define BOARD_P12_PIN               GPIO_12
#define BOARD_P24_PIN               GPIO_24
#define BOARD_P25_PIN               GPIO_25
#define BOARD_P41_PIN               GPIO_41
#define BOARD_P44_PIN               GPIO_44
#define BOARD_P45_PIN               GPIO_45
#define BOARD_P46_PIN               GPIO_46
#define BOARD_P47_PIN               GPIO_47

/* ==========================================================================
 * 2. USB（设备模式，UAC 声卡）
 * ========================================================================== */
/* DP/DM 为芯片专用引脚，无 GPIO 编号，也无需配置。板上没有 VBUS 控制脚，
 * 插线即上电；因此禁止调用 bk_usb_power_ops()（CONFIG_USB_VBAT_CONTROL_GPIO_ID
 * 在 config 里设为 0xFF）。 */
#define BOARD_USB_PORT_IS_DEDICATED 1
#define BOARD_USB_VBUS_CTRL_PIN     0xFF
#define BOARD_USB_FS_ISO_MPS        192         /* 48kHz/16bit/stereo，1ms 一包 */

/* ==========================================================================
 * 3. 上电时序 / 默认参数
 * ========================================================================== */
#define BOARD_PWR_SETTLE_MS         20
#define BOARD_CAM_PWR_SETTLE_MS     30
/* 屏 3V3 上电后到允许发第一条命令的等待时间。
 *
 * 【必须 ≥120ms】ST77916 要求在 VDD/VDDI 稳定后延时 120ms 才能接收
 * 初始化命令（数据手册的 power-on sequence），屏内部 LDO 与振荡器
 * 需要这段时间起振。原来给的 30ms 是按 ILI9341 的经验留的，换屏后
 * 明显不足 —— 症状是初始化命令整段被屏忽略：屏停在未配置状态，
 * GRAM 是随机值，看起来就是"花屏"；而 COLMOD 也停在默认的 18bit，
 * 又叠加一层"只填 2/3 GRAM"的错乱。
 *
 * 换屏前 app_test_uart1 手动敲 `lcdq init` 时屏早已上电数秒，所以
 * 这个坑在那边不会暴露，只有 app_bk_pi 这种 main 早期初始化的路径
 * 才会踩到。 */
#define BOARD_SCR_PWR_SETTLE_MS     120

#define BOARD_DEFAULT_VOLUME        60          /* 0~100 */
#define BOARD_DEFAULT_BACKLIGHT     100

/* ---------------- I2C ---------------- */
/* I2C0 固定 GPIO20/21（摄像头 SCCB，本工程不用）；
 * I2C1 走 GPIO42/43 复用组，板上挂 CST816S 触摸和 SC7A20 加速度计 */
#define BOARD_CAM_I2C_ID            0
#define BOARD_CAM_I2C_SCL_PIN       GPIO_20
#define BOARD_CAM_I2C_SDA_PIN       GPIO_21

#define BOARD_TP_I2C_ID             1
#define BOARD_TP_I2C_SCL_PIN        GPIO_42
#define BOARD_TP_I2C_SDA_PIN        GPIO_43

/* ---------------- 加速度计 SC7A20（摇一摇切歌） ---------------- */
#define BOARD_SC7A20_I2C_ADDR       0x18        /* SDO 下拉 -> 0x18，探测时兼容 0x19 */
#define BOARD_SC7A20_I2C_ADDR_ALT   0x19
#define BOARD_SC7A20_INT1_PIN       GPIO_55     /* SC_INT1（本工程用轮询，不接中断） */
#define BOARD_SC7A20_INT2_PIN       GPIO_54     /* SC_INT2 */

#ifdef __cplusplus
}
#endif
