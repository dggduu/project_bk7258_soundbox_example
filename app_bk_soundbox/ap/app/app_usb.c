// Copyright 2026
// CherryUSB UAC2 声卡设备实现（Speaker + Microphone 双向）
//
// 描述符布局：IAD + AudioControl(0) + AudioStreaming(1, alt0/alt1, Speaker)
//                            + AudioStreaming(2, alt0/alt1, Microphone)
// 端点：0x01 ISO OUT，48kHz/16bit/2ch -> 192 bytes/ms（主机 -> 喇叭）
//       0x82 ISO IN ，48kHz/16bit/2ch <- 192 bytes/ms（DMIC -> 主机）
//
// 播放：主机 -> ISO OUT -> 8KB 环形缓冲 -> app_audio -> DAC
// 录音：DMIC -> onboard_dual_dmic 组件 -> ISO IN -> 主机

#include "app_usb.h"
#include "app_audio.h"
#include "app_mode.h"
#include "app_state.h"
#include "bsp.h"

#include <components/cherryusb/usbd_audio.h>
#include <components/cherryusb/usbd_core.h>
#include <components/log.h>
#include <components/usb_types.h>
#include <os/mem.h>
#include <os/os.h>
#include <os/str.h>
#include <string.h>

#define TAG "app_usb"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

extern bk_err_t bk_usb_open(uint32_t usb_mode);

#define UAC_VID 0x1209
#define UAC_PID 0x7259
/* Windows 设备实例 ID 里带 bcdDevice（USB\VID_xxxx&PID_xxxx&REV_0100）。
 * 反复改描述符调试时，Windows 会复用旧实例的枚举/驱动状态，
 * 改了固件插上去仍然是旧的失败状态。把这里 +1（0x0100 -> 0x0101）
 * 就会生成一个全新的设备实例，强制重新走一遍驱动安装 —— 排查时最省事的开关。
 * 加麦克风属于描述符结构变化，Windows 一定会缓存旧的，所以这次一并 +1。 */
/* 【改任何描述符/类请求行为后都必须动这个号】
 *
 * Windows 会把 UAC 设备的**音量范围（RANGE）和特性描述符**跟着设备实例
 * 一起缓存到注册表，只要 bcdDevice 没变，它就认为"还是同一台设备"，
 * 直接复用上次读到的值 —— 哪怕固件里已经把 RANGE 从"MAX=0.39dB"
 * 改成"-37..0dB"，主机侧滑块依旧按旧范围工作（表现为"还是调不了"）。
 *
 * 所以只要动了 usbd_audio.c 里的 RANGE / SET_CUR / GET_CUR，或者
 * app_usb.c 里的 UAC 描述符，**必须把这个号 +1**，让主机当成新设备
 * 重新枚举。这是踩过一次的坑（UAC1 换 UAC2 时也是靠它才生效）。
 *
 * 历史：
 *   0x0100  UAC1
 *   0x0101  UAC2 + 麦克风
 *   0x0102  UAC2 音量范围/单位修正
 *   0x0103  UAC2 **bmaControls 改为双位编码 0x0F**（Mute+Volume 均读写）
 *   0x0104  只保留 master 音量（声道位 0x00）+ 量程放到 -127dB
 *   0x0105  UAC2 麦克风 Feature Unit 补上声道控制组（Windows 枚举修复）
 *   0x0106  UAC2 麦克风改立体声（2ch，FU 3 组，192B/ms）
 */
#define UAC_BCD_DEVICE 0x0106
#define UAC_LANGID 0x0409
#define UAC_EP_OUT 0x01
#define UAC_EP_IN 0x82
#define UAC_RATE 48000
#define UAC_CHANNELS 2

/* 接口编号：AC=0，Speaker AS=1，Microphone AS=2 */
#define UAC_AC_INTF 0
#define UAC_AS_INTF 1
#define UAC_MIC_INTF 2

/* 播放方向（立体声）每 1ms 的字节数：48k × 2ch × 2B = 192。
 * 录音方向同样按 2ch 算（见 UAC_MIC_PACKET），也是 192B/ms。 */
#define UAC_OUT_PACKET (UAC_RATE / 1000 * UAC_CHANNELS * 2)      /* 192 */
#define UAC_OUT_MPS UAC_OUT_PACKET

#define UAC_RING_SIZE 8192
#define UAC_TASK_STACK 4096
#define UAC_TASK_PRIO (BEKEN_DEFAULT_WORKER_PRIORITY - 1)

#define USB_C(ch) ((ch) & 0xFF), 0x00

/* UAC1 描述符形态（编译期二选一）：
 *
 *  0 = **官方例程形态（默认）**：bDeviceClass=0xEF/0x02/0x01，**配置里不带 IAD**。
 *      CherryUSB 官方 UAC1 例程（ref/CherryUSB/demo/audio_v1_mic_speaker_multichan_template.c）
 *      就是这个组合：设备描述符声明"复合设备"（让 Windows 加载 usbccgp），
 *      但功能分组不靠 IAD，而是靠 Windows 对 USB Audio 的内置规则 ——
 *      按 AC 头里的 baInterfaceNr[] 把 AS 接口关联到 AC 接口。
 *      这是被大量 UAC1 声卡（PCM2902 一类）验证过的路径。
 *
 *  1 = 带 IAD 形态：9(config)+8(IAD)+…，总长 118。
 *      IAD 的 bFunctionClass/SubClass/Protocol 镜像该功能第一个接口（AC）的
 *      01/01/00。规范上完全合规，但如果主机端的 usbccgp 分组逻辑与
 *      期望不一致，反而会把音频功能拆散 —— 留作对照实验用。
 */
#define UAC_DESC_WITH_IAD 0

/* ============================ 协议版本 ============================
 * 1 = UAC1，Windows 用 usbaudio.sys（老系统兼容性最好）
 * 2 = UAC2，Windows 用 usbaudio2.sys（Win10 1703+ / Win11 原生支持）
 *
 * UAC2 的意义：UAC1 在 Windows 上有一套独立的类驱动，历史上多次被系统更新
 * 改坏（如 2026-09 的 UAC1 回归，表现为 0xC00000E5/Code 10 无法启动）；
 * UAC2 走另一套驱动，已实测在本板 Win11 23H2 上工作正常。
 * ================================================================ */
#define UAC_PROTOCOL_VERSION 2

/* 实体 ID（两个方向各自独立一套，同一 AC 接口内不能重号） */
#define UAC_IT_ID 1        /* Speaker 方向：Input Terminal（USB Streaming） */
#define UAC_FU_ID 2        /* Speaker 方向：Feature Unit */
#define UAC_OT_ID 3        /* Speaker 方向：Output Terminal（Speaker） */
#define UAC_CLOCK_ID 4     /* Speaker 方向：Clock Source */

#define UAC_MIC_IT_ID 5    /* Mic 方向：Input Terminal（Microphone） */
#define UAC_MIC_FU_ID 6    /* Mic 方向：Feature Unit */
#define UAC_MIC_OT_ID 7    /* Mic 方向：Output Terminal（USB Streaming） */
#define UAC_MIC_CLOCK_ID 8 /* Mic 方向：Clock Source */

/* 字符串描述符表（两个版本共用）。
 * 必须紧跟在配置描述符后面：CherryUSB 的 usbd_desc_register() 不接受
 * 单独的 string_descriptors[] 表，它靠 bLength 逐段遍历整个数组。 */
#define UAC_STRING_DESCRIPTORS                                     \
    USB_LANGID_INIT(UAC_LANGID),                                   \
    /* string1: dggdoo（6 字符 -> 2 + 12 = 14 = 0x0e） */           \
    0x0e, USB_DESCRIPTOR_TYPE_STRING,                              \
    USB_C('d'), USB_C('g'), USB_C('g'), USB_C('d'), USB_C('o'), USB_C('o'), \
    /* string2: bk7258_soundbox（15 字符 -> 2 + 30 = 32 = 0x20） */  \
    0x20, USB_DESCRIPTOR_TYPE_STRING,                              \
    USB_C('b'), USB_C('k'), USB_C('7'), USB_C('2'), USB_C('5'), USB_C('8'), \
    USB_C('_'), USB_C('s'), USB_C('o'), USB_C('u'), USB_C('n'), USB_C('d'), \
    USB_C('b'), USB_C('o'), USB_C('x'),                            \
    /* string3: 0001（4 字符 -> 2 + 8 = 10 = 0x0a） */              \
    0x0a, USB_DESCRIPTOR_TYPE_STRING,                              \
    USB_C('0'), USB_C('0'), USB_C('0'), USB_C('1'),

#if UAC_PROTOCOL_VERSION == 2

/* ============================== UAC2 ============================== */
/* AC 头的 wTotalLength：AC 头 + 两个方向的 Clock/IT/FU/OT（不含 AC 接口描述符本身）
 *
 * `AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(ch)` = 6 + (ch+1)*4，ch 是**声道数**，
 * 必须和 FU 实际写的 bmaControls 组数一致（组数 = ch + 1）：
 *
 *   播放 FU：DBVAL(0x0F), DBVAL(0x00), DBVAL(0x00) -> 3 组 -> ch 传 2
 *   录音 FU：DBVAL(0x0F), DBVAL(0x00), DBVAL(0x00) -> 3 组 -> ch 传 2
 *
 * 麦克风 FU 也必须给满 bNrChannels + 1 组 bmaControls（立体声 3 组）：
 * 只写 master 一组（bLength=10）等于声明 bNrChannels=0，与 Input Terminal
 * 的声道数对不上。Linux 的 snd-usb-audio 能容错，但 Windows 的
 * usbaudio2.sys 会解析失败 —— 设备管理器里就是 "该设备无法启动 (Code 10)"。
 * 当前立体声逐段：9+8+17+18+12 +8+17+**18**+12 = 119。 */
#define UAC_AC_SIZ (AUDIO_V2_SIZEOF_AC_HEADER_DESC +                    \
                    AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +              \
                    AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +            \
                    AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(UAC_CHANNELS) +\
                    AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC +           \
                    AUDIO_V2_SIZEOF_AC_CLOCK_SOURCE_DESC +              \
                    AUDIO_V2_SIZEOF_AC_INPUT_TERMINAL_DESC +            \
                    AUDIO_V2_SIZEOF_AC_FEATURE_UNIT_DESC(2) +           \
                    AUDIO_V2_SIZEOF_AC_OUTPUT_TERMINAL_DESC)

/* 配置总长：9(config) + 26(IAD + AC 接口 + **AC 头**) + AC 内其余类描述符 + 两个 AS。
 * 注意 AC 头已经在 AUDIO_V2_AC_DESCRIPTOR_INIT_LEN 里算过一次，
 * 所以后面的类描述符要减掉它 —— 重复计算会让 wTotalLength 多 9 字节，
 * 主机按这个长度取配置描述符时会把字符串表的内容也捎出去，枚举直接失败。 */
#define UAC_CONFIG_LEN (9 + AUDIO_V2_AC_DESCRIPTOR_INIT_LEN +            \
                        (UAC_AC_SIZ - AUDIO_V2_SIZEOF_AC_HEADER_DESC) +  \
                        AUDIO_V2_AS_DESCRIPTOR_INIT_LEN +                \
                        AUDIO_V2_AS_DESCRIPTOR_INIT_LEN)

/* 麦克风按**立体声**上报（双 DMIC 左/右各一路，原样送主机）：
 * DMIC 组件本身是 stereo（bsp_mic_init 的 stereo=true），USB 侧直接
 * 收两路，不再做左声道抽取 —— 录音软件能拿到真正的双麦立体声。
 * 要回单声道就把这里改 1，并注意 UAC_AC_SIZ 里 FU 的声道数、
 * 以及 mic 任务里的兜底分支。 */
#define UAC_MIC_CHANNELS 2
#define UAC_MIC_BMCHCFG 0x00000003   /* AUDIO_CHANNEL_FL | FR：左右声道 */

/* 录音方向每 1ms 的字节数 —— **必须与上面这个声道数一致**。
 * 48k × 2ch × 2B = 192B/ms。改 UAC_MIC_CHANNELS 时自动跟着变。 */
#define UAC_MIC_PACKET (UAC_RATE / 1000 * UAC_MIC_CHANNELS * 2)  /* 192 */

/* IN 侧 MPS 留 4 字节余量：主机（尤其 Linux/Android）允许异步 IN 端点
 * 每帧比标称多几个字节做漂移补偿；包长写死等于标称值会让主机偶尔拒收。 */
#define UAC_IN_MPS (UAC_MIC_PACKET + 4)                          /* 196 */

static const uint8_t s_uac_descriptor[] = {
    /* 与官方 audio_v2_mic_speaker 例程一致：设备类 0x00，功能分组靠配置里的 IAD
     * （AUDIO_V2_AC_DESCRIPTOR_INIT 宏自带 IAD，字段是 01/00/20 —— 这正是
     * Windows usbaudio2.inf 匹配的 USB\Class_01&SubClass_00&Prot_20）。 */
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, 0x00, 0x00, 0x00, UAC_VID, UAC_PID,
                               UAC_BCD_DEVICE, 0x01),
    /* bNumInterfaces = 3（AC + 播放 AS + 录音 AS） */
    USB_CONFIG_DESCRIPTOR_INIT(UAC_CONFIG_LEN, 3, 1, USB_CONFIG_BUS_POWERED,
                               250),

    /* bCategory 用 UNDEF（与官方 audio_v2_mic_speaker 例程一致）。
     *
     * SDK 头里的 AUDIO_CATEGORY_HEADSET 实际是 0x04，而 UAC2 规范里
     * 0x04 = Microphone（0x03 才是 Headset）—— 按 HEADSET 写会让 Windows
     * 把整个功能当"麦克风"，对双向设备不友好。
     * bCategory 只是给主机做分类提示，UNDEF 不会丢功能：
     * 主机仍按 AC 里的 IT/OT + AS 接口枚举出播放和录音两路。
     * bInterfaceCount = 3 把三个接口归到同一个功能里（must 与 bNumInterfaces 一致）。 */
    AUDIO_V2_AC_DESCRIPTOR_INIT(UAC_AC_INTF, 3, UAC_AC_SIZ,
                                AUDIO_CATEGORY_UNDEF, 0x00, 0),

    /* ---- 播放方向：Clock -> USB IT -> FU -> Speaker OT ---- */
    AUDIO_V2_AC_CLOCK_SOURCE_DESCRIPTOR_INIT(UAC_CLOCK_ID, 0x03, 0x03),
    AUDIO_V2_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(UAC_IT_ID, AUDIO_TERMINAL_STREAMING,
                                               UAC_CLOCK_ID, UAC_CHANNELS,
                                               0x00000003, 0x0000),
    /* 【UAC2 的 bmaControls 是"每控制 2 bit"，不是每控制 1 bit】
     *
     * 这里踩过一个很深的坑，记下来免得再犯：UAC1 的 bmaControls 是
     * **每个控制占 1 bit**（bit0=Mute, bit1=Volume, bit2=AGC...），
     * 但 **UAC2 改成每个控制占 2 bit**，两位表示该控制的访问属性：
     *
     *     00 = 不支持      01 = 只读
     *     10 = 保留(非法)  11 = 读写
     *
     * 对应 CherryUSB 的宏（ref/CherryUSB/class/audio/usb_audio.h:248）：
     *     AUDIO_V2_CONTROL_MUTE   = (0x03 << 0) = 0x03
     *     AUDIO_V2_CONTROL_VOLUME = (0x03 << 2) = 0x0C
     *     BMCONTROL = MUTE | VOLUME = **0x0F**
     *
     * 之前按 UAC1 的思维写成 `0x03 / 0x02 / 0x02`，实际含义是：
     *   master 只声明了"Mute 可读写"（Volume 两位是 00 = 不支持）
     *   两个声道写成 0x02，bit[1:0]=10 是**保留的非法值**
     * 后果是 Linux 只建出 'PCM Playback Switch'（Mute），
     * 建不出 'PCM Playback Volume' —— 表现为**电脑端完全无法调音量**，
     * 而 `lsusb -v` 会明说 `bmaControls(1) ... ILLEGAL VALUE (0b10)`。
     *
     * 现在 master 写 0x0F（Mute + Volume 均可读写），与官方例程
     * `OUTPUT_CTRL = DBVAL(BMCONTROL), ...` 一致。
     *
     * 【声道位为什么是 0x00】本机只有一个喇叭、也只有一个音量。
     * 三组都报 Volume 时 Linux 会建出**两个** "PCM Playback Volume"
     * （一个 2 声道、一个 master），各桌面混音器各自缓存、各拖各的，
     * 于是"电脑上的值和设备上的值不一致"。只留 master 后主机只有
     * 一个滑块、一个数据源。声道位=0 表示该声道无独立控制，master
     * 覆盖全部声道，符合 UAC2 规范。 */
    AUDIO_V2_AC_FEATURE_UNIT_DESCRIPTOR_INIT(UAC_FU_ID, UAC_IT_ID,
                                             DBVAL(0x0F), DBVAL(0x00), DBVAL(0x00)),
    AUDIO_V2_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(UAC_OT_ID, AUDIO_OUTTERM_SPEAKER,
                                                UAC_FU_ID, UAC_CLOCK_ID, 0x0000),

    /* ---- 录音方向：Clock -> Mic IT -> FU -> USB Streaming OT ---- */
    AUDIO_V2_AC_CLOCK_SOURCE_DESCRIPTOR_INIT(UAC_MIC_CLOCK_ID, 0x03, 0x03),
    /* wTerminalType = 0x0201 通用麦克风 */
    AUDIO_V2_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(UAC_MIC_IT_ID, AUDIO_INTERM_MIC,
                                               UAC_MIC_CLOCK_ID, UAC_MIC_CHANNELS,
                                               UAC_MIC_BMCHCFG, 0x0000),
    /* 【必须给 bNrChannels+1 组 bmaControls】
     *
     * 立体声麦克风的 Feature Unit 是 master(0) + 左 + 右 三组，
     * 不能只写 master。之前只写一组（bLength=10），等于声明 bNrChannels=0，
     * 与 Input Terminal 的 2 声道对不上 —— Linux 的 snd-usb-audio 能容错，
     * 但 Windows 的 usbaudio2.sys 会解析失败，设备管理器里报
     * "该设备无法启动 (Code 10)"。官方 audio_v2_mic 例程也是给满
     * (bNrChannels+1) 组。
     * 声道 1/2 给 0x00（无独立控制），只有 master 有 Mute+Volume。 */
    AUDIO_V2_AC_FEATURE_UNIT_DESCRIPTOR_INIT(UAC_MIC_FU_ID, UAC_MIC_IT_ID,
                                             DBVAL(0x0F), DBVAL(0x00), DBVAL(0x00)),
    AUDIO_V2_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(UAC_MIC_OT_ID, AUDIO_TERMINAL_STREAMING,
                                                UAC_MIC_FU_ID, UAC_MIC_CLOCK_ID, 0x0000),

    /* ---- 播放 AS：bTerminalLink 指向播放 Input Terminal ---- */
    AUDIO_V2_AS_DESCRIPTOR_INIT(UAC_AS_INTF, UAC_IT_ID, UAC_CHANNELS, 0x00000003,
                                2, 16, UAC_EP_OUT, 0x09, UAC_OUT_MPS, 1),
    /* ---- 录音 AS：bTerminalLink 指向录音 Output Terminal ---- */
    AUDIO_V2_AS_DESCRIPTOR_INIT(UAC_MIC_INTF, UAC_MIC_OT_ID, UAC_MIC_CHANNELS,
                                UAC_MIC_BMCHCFG, 2, 16, UAC_EP_IN, 0x05,
                                UAC_IN_MPS, 1),

    UAC_STRING_DESCRIPTORS
    0x00
};

#else  /* UAC1 */

/* ============================== UAC1 ============================== */

#if UAC_DESC_WITH_IAD
/* 9(config) + 8(IAD) + 9(AC intf) + 9(AC hdr) + 12(IT) + 10(FU) + 9(OT) + 52(AS) */
#define UAC_CONFIG_LEN    118
#else
/* 9(config) + 9(AC intf) + 9(AC hdr) + 12(IT) + 10(FU) + 9(OT) + 52(AS) */
#define UAC_CONFIG_LEN    110
#endif
#define UAC_DEV_CLASS     0xEF
#define UAC_DEV_SUBCLASS  0x02
#define UAC_DEV_PROTO     0x01

/* UAC1 的麦克风方向未实现（本工程默认 UAC2）。保留最小可编译的描述符。 */
static const uint8_t s_uac_descriptor[] = {
    USB_DEVICE_DESCRIPTOR_INIT(USB_2_0, UAC_DEV_CLASS, UAC_DEV_SUBCLASS,
                               UAC_DEV_PROTO, UAC_VID, UAC_PID, UAC_BCD_DEVICE, 0x01),
    USB_CONFIG_DESCRIPTOR_INIT(UAC_CONFIG_LEN, 2, 1, USB_CONFIG_BUS_POWERED,
                               250),

#if UAC_DESC_WITH_IAD
    /* IAD：bFirstInterface=0, bInterfaceCount=2, Class=0x01(Audio),
     * SubClass=0x01(AUDIOCONTROL，镜像 AC 接口), Protocol=0x00, iFunction=0 */
    0x08, 0x0B, UAC_AC_INTF, 0x02, 0x01, 0x01, 0x00, 0x00,
#endif

    /* AudioControl 接口：Class=0x01 SubClass=0x01(AUDIOCONTROL) iInterface=2 */
    0x09, 0x04, UAC_AC_INTF, 0x00, 0x00, 0x01, 0x01, 0x00, 0x02,
    /* AC 头：bcdADC=0x0100, wTotalLength=40(9+12+10+9), 1 个采集, 接口 1 */
    0x09, 0x24, 0x01, 0x00, 0x01, 0x28, 0x00, 0x01, UAC_AS_INTF,

    AUDIO_AC_INPUT_TERMINAL_DESCRIPTOR_INIT(UAC_IT_ID, 0x0101, UAC_CHANNELS, 0x0003),
    /* master: mute + volume(0x03)，左右声道 volume；否则 Windows 只看到静音，
	 * 不会把主机音量滑块发下来。 */
	AUDIO_AC_FEATURE_UNIT_DESCRIPTOR_INIT(UAC_FU_ID, UAC_IT_ID, 1, 0x03, 0x02, 0x02),
    AUDIO_AC_OUTPUT_TERMINAL_DESCRIPTOR_INIT(UAC_OT_ID, 0x0301, UAC_FU_ID),
    AUDIO_AS_DESCRIPTOR_INIT(UAC_AS_INTF, UAC_OT_ID, UAC_CHANNELS, 2, 16,
                             UAC_EP_OUT, 0x09, UAC_OUT_MPS, 1,
                             0x80, 0xBB, 0x00), /* 48000 */

    UAC_STRING_DESCRIPTORS
    0x00
};

#endif /* UAC_PROTOCOL_VERSION */

static struct usbd_interface s_intf_ac;
static struct usbd_interface s_intf_as;
static struct usbd_interface s_intf_mic;
static struct audio_entity_info s_entity[] = {
#if UAC_PROTOCOL_VERSION == 2
    /* UAC2 的采样率由 Clock Source 的 SAM_FREQ 控制承载，
     * 类驱动按 entity_id 查 subtype，表里没有就会对请求回 STALL。
     * ep 字段是类驱动用来区分方向的唯一依据：主机对该实体的
     * SET/GET_CUR 会原样把 ep 传给回调（含 Clock 的采样率设置）。 */
    {AUDIO_CONTROL_CLOCK_SOURCE, UAC_CLOCK_ID, UAC_EP_OUT},
    {AUDIO_CONTROL_CLOCK_SOURCE, UAC_MIC_CLOCK_ID, UAC_EP_IN},
#endif
    {AUDIO_CONTROL_FEATURE_UNIT, UAC_FU_ID, UAC_EP_OUT},
    {AUDIO_CONTROL_FEATURE_UNIT, UAC_MIC_FU_ID, UAC_EP_IN},
};

#if UAC_PROTOCOL_VERSION == 2
#define UAC_AUDIO_VERSION 0x0200
#else
#define UAC_AUDIO_VERSION 0x0100
#endif

/* ---- 播放方向：主机 -> 设备 ---- */
static uint8_t s_pkt[2][UAC_OUT_MPS] __attribute__((aligned(4)));
static volatile uint8_t s_pkt_idx;
static uint8_t s_ring[UAC_RING_SIZE] __attribute__((aligned(4)));
static volatile uint32_t s_wr;
static volatile uint32_t s_rd;

/* ---- 录音方向：设备 -> 主机 ----
 *
 *   DMIC 20ms 一帧 --(环形缓冲)--> USB IN 1ms 一包 --> MUSB FIFO
 *
 * 采集与发送在两个完全不同的时间尺度上跑：DMIC 20ms 一块，USB 1ms 一包。
 * 环形缓冲负责把两者解耦，深度就是容忍调度抖动的额度（这里 42ms）。
 * 缓冲做成 2 的幂，取模用掩码。 */
#define UAC_MIC_RING_SIZE 16384  /* 约 85ms @48k/16bit/stereo（192B/ms），吸收调度抖动 */
static uint8_t s_mic_ring[UAC_MIC_RING_SIZE] __attribute__((aligned(4)));
static volatile uint32_t s_mic_wr;
static volatile uint32_t s_mic_rd;

/* 为什么不能跳过这层、直接把 s_mic_ring 里的地址丢给 usbd_ep_start_write()：
 *
 * Beken 的 MUSB 移植里 usbd_ep_start_write() 只是**登记**缓冲并写一次 FIFO，
 * 真正"取下一包"发生在 IN 中断里（usb_dc_beken_musb_fdrc.c:726 起）。
 * 如果登记的是 s_mic_ring 内部的地址，而发完后 ISR 立刻在同一地址上登记下一包，
 * 这块内存就会同时是"DMA 正在读的缓冲"和"下一次要写的目标" —— 语义上打架。
 *
 * 所以这里用一组独立的、带占用标记的包缓冲：
 *   - s_mic_pkt_busy[i] 为 true 表示这块已登记给 USB、还在飞；
 *   - 分配时只挑 false 的块，USB 完全回调后才置回 false。
 * 这样端点在飞期间那块内存绝不会被重写，也不会出现"同一块内存
 * 既是 DMA 读源、又是下次写入目标"的打架。
 * 4 块 × 1ms 足够：一块在飞、一块刚写完等着提交，还剩两块余量。
 *
 * 顺带一提，MUSB FIFO 入口 musb_write_packet() 对未对齐指针走逐字节写，
 * 所以这几块都强制 4 字节对齐。 */
#define UAC_MIC_SEATS 4
static uint8_t s_mic_pkt[UAC_MIC_SEATS][UAC_MIC_PACKET] __attribute__((aligned(4)));
static volatile bool s_mic_pkt_busy[UAC_MIC_SEATS];
static volatile uint8_t s_mic_pkt_hint;      /* 下一块优先尝试的下标 */
static volatile bool s_mic_tx_busy;          /* EP 上有一包在飞 */
static volatile bool s_mic_streaming;
/* 主机是否已经 SET_INTERFACE 打开录音流（不管我们有没有真的去采） */
static volatile bool s_mic_requested;
/* 当前是不是"USB 麦克风"模式：**只有这个模式**才允许真正启动 DMIC 采集。
 * 普通 USB 声卡模式下主机即使打开录音流，我们也只回静音，不占 DMIC。 */
static volatile bool s_mic_mode;

/* 诊断计数（只在任务上下文打印，不在 ISR 里 printf）：
 *   s_dbg_in_ok    IN 完成回调次数
 *   s_dbg_starve   取包时环形缓冲不够、补了静音的包数
 *   s_dbg_underrun 环形缓冲被读空（数据源没跟上）的次数
 *   s_dbg_cap      采集任务成功读到 DMIC 帧的次数
 * 定位"节点在但没声音"时先看这四个，能立刻区分
 * "根本没采到" / "采到了但没发出去" / "发出去了但主机不认"。 */
static volatile uint32_t s_dbg_in_ok;
static volatile uint32_t s_dbg_starve;
static volatile uint32_t s_dbg_underrun;
static volatile uint32_t s_dbg_cap;

static volatile bool s_configured;
static volatile bool s_streaming;
static volatile bool s_active;
static volatile bool s_muted;
static uint8_t s_saved_volume;
/* GET_CUR 在 USB 中断上下文被调用，不能拿 app_state 的互斥锁，
 * 所以这里保留一个“影子值”，由 app_usb_poll() 在任务上下文同步。 */
static volatile uint8_t s_volume_cache = 60;
/* 最后一次真正写给音频层的音量（0~100），用于 poll 里做收敛校正 */
static volatile uint8_t s_applied_volume = 0xFF;

/* ISR -> 任务 的延迟事件（USB 类回调在中断上下文，不能拿互斥锁/发队列） */
static volatile bool s_pending_publish;
static volatile bool s_pending_mode;
static volatile bool s_pending_mute;
static volatile bool s_pending_mute_val;
static volatile bool s_pending_volume;
static volatile uint8_t s_pending_volume_val;

static beken_thread_t s_drain_task;
static volatile bool s_drain_run;
static beken_thread_t s_mic_task;
static volatile bool s_mic_run;
static beken_thread_t s_init_task;

static void publish(void) {
  app_state_t *st;
  app_track_t *tr;

  app_state_lock();
  st = app_state_raw();
  tr = &st->usb;
  st->usb_connected = s_configured;
  st->usb_streaming = s_streaming;
  st->usb_mic_streaming = s_mic_streaming && s_mic_mode;
  /* UAC 是裸 PCM 流，没有容器格式 */
  os_strncpy(st->codec, s_streaming ? "PCM" : "", sizeof(st->codec) - 1);
  st->codec[sizeof(st->codec) - 1] = '\0';
  if (s_mic_mode) {
    os_strncpy(tr->title, "USB 麦克风", sizeof(tr->title) - 1);
    os_strncpy(tr->artist,
               st->usb_mic_streaming ? "电脑录音中 · 双 DMIC" : "等待电脑录音",
               sizeof(tr->artist) - 1);
  } else {
    os_strncpy(tr->title, "bk7258_soundbox", sizeof(tr->title) - 1);
    os_strncpy(tr->artist, "48kHz / 16bit / 立体声", sizeof(tr->artist) - 1);
  }
  tr->title[sizeof(tr->title) - 1] = '\0';
  tr->artist[sizeof(tr->artist) - 1] = '\0';
  tr->pos_ms = 0;
  tr->dur_ms = 0;
  tr->has_lyric = false;
  if (st->mode == APP_MODE_USB)
    st->play = (s_streaming && s_active) ? APP_PLAY_PLAYING : APP_PLAY_STOPPED;
  else if (st->mode == APP_MODE_USB_MIC)
    st->play = st->usb_mic_streaming ? APP_PLAY_PLAYING : APP_PLAY_STOPPED;
  app_state_unlock();
}

bool app_usb_is_streaming(void) { return s_streaming; }

bool app_usb_is_connected(void) { return s_configured; }

/* ============================ 环形缓冲 ============================ */

static void uac_ring_push(const uint8_t *data, uint32_t len) {
  uint32_t wr = s_wr;
  uint32_t rd = s_rd;
  uint32_t used = wr - rd;
  uint32_t space = UAC_RING_SIZE - used;

  /* 单生产者（USB ISR）/单消费者（drain 线程）：生产者绝不改 s_rd。
   * 溢出时直接丢这一包，避免和消费者的 s_rd 写竞争。 */
  if (len > space)
    return;

  for (uint32_t i = 0; i < len; i++)
    s_ring[(wr + i) & (UAC_RING_SIZE - 1)] = data[i];

  s_wr = wr + len;
}

static uint32_t uac_ring_pop(uint8_t *out, uint32_t max) {
  uint32_t rd = s_rd;
  uint32_t wr = s_wr;
  uint32_t n = wr - rd;

  if ((n == 0) || (n > UAC_RING_SIZE)) {
    /* 索引被异常重置时直接对齐，避免读出一大段旧数据 */
    if (n > UAC_RING_SIZE)
      s_rd = wr;
    return 0;
  }
  if (n > max)
    n = max;

  for (uint32_t i = 0; i < n; i++)
    out[i] = s_ring[(rd + i) & (UAC_RING_SIZE - 1)];

  s_rd = rd + n;
  return n;
}

static void uac_drain_task(void *arg) {
  static uint8_t tmp[960];

  (void)arg;

  while (s_drain_run) {
    uint32_t n = uac_ring_pop(tmp, sizeof(tmp));

    if (n == 0) {
      rtos_delay_milliseconds(4);
      continue;
    }

    if (s_active && app_audio_is_open()) {
      if (app_audio_write(tmp, n) != BK_OK)
        rtos_delay_milliseconds(2);
    }
  }

  s_drain_task = NULL;
  rtos_delete_thread(NULL);
}

static void uac_drain_start(void) {
  if (s_drain_task || s_drain_run)
    return;

  s_drain_run = true;
  if (rtos_create_thread(&s_drain_task, UAC_TASK_PRIO, "uac_drain",
                         (beken_thread_function_t)uac_drain_task,
                         UAC_TASK_STACK, NULL) != kNoErr) {
    s_drain_run = false;
    s_drain_task = NULL;
    LOGE("create uac drain task failed\r\n");
  }
}

static void uac_drain_stop(void) {
  if (!s_drain_run)
    return;

  s_drain_run = false;
  for (int i = 0; i < 20 && s_drain_task; i++)
    rtos_delay_milliseconds(5);
}

/* ============================ 录音方向 ============================ */

static void uac_mic_ring_reset(void) {
  s_mic_wr = 0;
  s_mic_rd = 0;
}

/* 单生产者（mic 采集任务）/ 单消费者（USB IN 完成中断）。
 *
 * 消费者会改 s_mic_rd，所以溢出时**绝不能**再动 s_mic_rd（会和 ISR
 * 的读游标打架，乱序后整个人声都成噪音）。这里的策略是：放不下就整块丢，
 * 只丢生产者自己这一侧的数据 —— 顶多丢 20ms，而且下一帧立刻恢复。
 * 42ms 的深度正常情况下不会走到这条路。 */
static void uac_mic_ring_push(const uint8_t *data, uint32_t len) {
  uint32_t wr = s_mic_wr;
  uint32_t rd = s_mic_rd;

  if ((len > UAC_MIC_RING_SIZE) || ((wr - rd) + len > UAC_MIC_RING_SIZE))
    return;

  for (uint32_t i = 0; i < len; i++)
    s_mic_ring[(wr + i) & (UAC_MIC_RING_SIZE - 1)] = data[i];

  s_mic_wr = wr + len;
}

/* 取 len 字节，不够就补静音（USB IN 端点必须每帧都有数据，绝不能少给）。
 * 只被 USB IN 完成中断调用，天然串行。 */
static void uac_mic_ring_pop(uint8_t *out, uint32_t len) {
  uint32_t rd = s_mic_rd;
  uint32_t wr = s_mic_wr;
  uint32_t n = wr - rd;

  if (n > UAC_MIC_RING_SIZE) {
    /* 索引被异常重置（重新开流没清干净），直接对齐到写游标 */
    s_mic_rd = wr;
    n = 0;
    s_dbg_underrun++;
  }
  if (n > len)
    n = len;

  for (uint32_t i = 0; i < n; i++)
    out[i] = s_mic_ring[(rd + i) & (UAC_MIC_RING_SIZE - 1)];

  if (n < len) {
    /* 数据源跟不上，这一包用静音顶上（端点必须每帧都给）。 */
    os_memset(out + n, 0, len - n);
    s_dbg_starve++;
  }

  s_mic_rd = rd + n;
}

/* 采样率转换占位：当前只支持 48k 立体声对齐，不做重采样。
 * DMIC 一帧 20ms 采 960 点/声道（48k），USB 每 ms 要 48 点/声道
 * （立体声 16bit = 192B），192*20 = 3840 = 960*2*2，正好整除，不需要 SRC。 */

/* 采集任务：只做"DMIC 一帧 -> 环形缓冲"这一件事。
 *
 * 发包完全不在这里 —— USB IN 每 1ms 要一包，任务调度抖几十 ms 根本跟不上，
 * 所以发送由 IN 完成中断**自己**驱动，见 uac_in_ep_callback()。
 * 这里只要保证环形缓冲不干涸即可，UAC_MIC_RING_SIZE 的深度就是为此留的。
 *
 * 缓冲放静态区：20ms@48k 双声道 int16 是 3.8KB，塞进 4KB 任务栈太冒险。
 *
 * 【曾经的 bug】这里原来把 int32_t 当采样点索引（`int32_t s_mic_frame[2*960]`），
 * 但 bsp_mic（onboard_dual_dmic_mic_stream）输出的是 **int16 交织**：
 *   c.adc_cfg.bits = 16、c.dmic_data_shift = 0、stereo 时 dual_dmic_sgl_out = 0
 * 内存布局是 L0 R0 L1 R1 …（每点 2 字节）。按 int32 索引等于每次跨 2 个
 * 立体声帧，取到的不是"第 i 点"而是"第 2i 帧"，高 16 位还混进了下一个
 * 声道 —— 结果既降采样又混叠，主机侧表现就是"有数据流但听不出人声/全是噪声"。
 * 现在按 uint8 缓冲 + int16 视图，立体声原样送给主机。 */
static uint8_t s_mic_frame_raw[20 * 48000 / 1000 * 2 * 2] __attribute__((aligned(4)));
/* 单麦兜底时才用：把一路复制成双声道（USB 描述符固定 2ch） */
static int16_t s_mic_stereo[960 * 2];

static void uac_mic_task(void *arg) {
  (void)arg;

  while (s_mic_run) {
    const int16_t *frame = (const int16_t *)s_mic_frame_raw;
    uint32_t got, ch, i;

    if (!s_mic_streaming) {
      rtos_delay_milliseconds(5);
      continue;
    }

    /* 【采集健康度，每约 2 秒一行】
     * 放在读之前，这样**采集不到数据时也能看到**（读超时会 continue）。
     * 判读：
     *   cap 不涨、filled<=0  -> 组件没在产数据（DMIC/DMA/时钟/供电/ADC 未启动）
     *   cap 不涨、filled>0   -> 端口里有数据，但读的长度和块大小对不上
     *   cap 在涨、主机没声音 -> 问题在发包侧（看 starve/underrun/in_ok） */
    {
      static uint32_t s_tick;

      if ((s_tick++ % 100) == 0) {
        LOGI("mic health: cap=%u starve=%u filled=%dB\r\n",
             (unsigned)s_dbg_cap, (unsigned)s_dbg_starve,
             bsp_mic_filled());
      }
    }

    /* bsp_mic_read() 只读一整个 s_block_bytes（20ms 一帧），且要求
     * len >= s_block_bytes。这里必须传真实帧长而不是 sizeof()，
     * 否则哪天 bsp_mic 的 frame_ms 改了，长度就对不上。
     *
     * 帧长为 0 说明 bsp_mic 还没 init（主机开流后 poll 才去拉 DMIC，
     * 中间有几十毫秒窗口）。此时**不能退出任务** —— 一退出就再也没人
     * 往环里塞数据，IN 中断会永远补静音，表现为"流在走但全零"，
     * 且从日志上看不出任何异常。这里只等一会儿再试。 */
    if (bsp_mic_frame_bytes() == 0) {
      rtos_delay_milliseconds(5);
      continue;
    }
    if (bsp_mic_frame_bytes() > sizeof(s_mic_frame_raw)) {
      /* 这个是真错配（缓冲开小了），每 1s 报一次就够了，然后继续等 */
      static uint32_t s_warn_tick;

      if ((s_warn_tick++ % 200) == 0)
        LOGE("mic frame %u > buf %u\r\n",
             (unsigned)bsp_mic_frame_bytes(), (unsigned)sizeof(s_mic_frame_raw));
      rtos_delay_milliseconds(5);
      continue;
    }

    if (bsp_mic_read(s_mic_frame_raw, sizeof(s_mic_frame_raw), 50) != BK_OK) {
      /* 数据没到：短等，不要空转 */
      rtos_delay_milliseconds(2);
      continue;
    }
    s_dbg_cap++;

    got = bsp_mic_frame_samples();   /* 每声道点数，48k/20ms = 960 */
    ch = bsp_mic_channels();
    if (ch == 0)
      ch = 1;

    if (ch >= 2) {
      /* 双 DMIC 原样立体声输出：L0 R0 L1 R1 …（int16 交织）。
       * 不做左声道抽取 —— 主机侧要的就是两路麦克风的立体声。 */
      uint32_t bytes = got * 2 * sizeof(int16_t);

      if (bytes > sizeof(s_mic_frame_raw))
        bytes = sizeof(s_mic_frame_raw);
      uac_mic_ring_push(s_mic_frame_raw, bytes);
    } else {
      /* 单麦兜底：复制成双声道（USB 描述符固定 2ch） */
      uint32_t max = (sizeof(s_mic_stereo) / sizeof(s_mic_stereo[0])) / 2;

      if (got > max)
        got = max;
      for (i = 0; i < got; i++) {
        s_mic_stereo[i * 2] = frame[i];
        s_mic_stereo[i * 2 + 1] = frame[i];
      }
      uac_mic_ring_push((const uint8_t *)s_mic_stereo, got * 2 * sizeof(int16_t));
    }
  }

  s_mic_task = NULL;
  rtos_delete_thread(NULL);
}

static void uac_mic_task_start(void) {
  if (s_mic_task || s_mic_run)
    return;

  s_mic_run = true;
  if (rtos_create_thread(&s_mic_task, UAC_TASK_PRIO, "uac_mic",
                         (beken_thread_function_t)uac_mic_task,
                         UAC_TASK_STACK, NULL) != kNoErr) {
    s_mic_run = false;
    s_mic_task = NULL;
    LOGE("create uac mic task failed\r\n");
  }
}

static void uac_mic_task_stop(void) {
  if (!s_mic_run)
    return;

  s_mic_run = false;
  for (int i = 0; i < 30 && s_mic_task; i++)
    rtos_delay_milliseconds(5);
}

/* ============================ UAC 类回调 ============================ */

static void uac_out_ep_callback(uint8_t ep, uint32_t nbytes) {
  uint8_t idx = s_pkt_idx;

  if (nbytes > 0 && nbytes <= UAC_OUT_MPS)
    uac_ring_push(s_pkt[idx], nbytes);

  s_pkt_idx = idx ^ 1;
  usbd_ep_start_read(ep, s_pkt[s_pkt_idx], UAC_OUT_MPS);
}

static struct usbd_endpoint s_ep_out = {
    .ep_cb = uac_out_ep_callback,
    .ep_addr = UAC_EP_OUT,
};

/* ---------------- 录音方向发包引擎 ----------------
 *
 * 时序约束：UAC2 的 ISO IN 是**每 1ms 一个事务**，主机在每帧开始就要数据，
 * 晚一帧就是一次欠载（听感上是咔哒/断续）。任何"任务轮询补包"的方案都
 * 撑不住 —— 应用主循环 app_usb_poll() 是 200ms 一轮，差了两个数量级。
 *
 * 所以发包直接借用 USB 自己的硬件节拍，不需要任何定时器：
 *
 *   本包发送完成 -> IN 中断 -> uac_in_ep_callback() -> 立刻登记下一包
 *
 * MUSB 在 IN 端点发完一包后会置 TXRDY 中断，这个中断本身就是 1ms 的节拍源。
 * 回调里马上把下一包塞进去，端点就永远不会空 —— 这是 USB 设备端音频的
 * 常规做法（主机那边是异步 IN，设备侧保持"永远有数据可发"即可）。
 *
 * 中断里做的事：从 16KB 环形缓冲搬 192B 到一块空闲包缓冲，再登记一次。
 * 192B 字节拷贝在 240MHz 上不到 2us，对 USB 中断完全可接受；换来的是
 * 不用起高优先级定时任务、不额外占栈、也不受 FreeRTOS 调度抖动影响。
 */

/* 当前正在飞的包缓冲下标；-1 表示没有包在飞 */
static volatile int8_t s_mic_inflight = -1;

/* 取一块空闲包缓冲并填入下一包数据。
 * 4 块都在飞理论上不可能（1ms 内发不完 4 包），真遇到就强行复用
 * hint 那块 —— 宁可冒一次重写风险，也不能让端点空掉。 */
static uint8_t *uac_mic_seat_alloc(uint8_t *out_idx) {
  uint8_t pick = UAC_MIC_SEATS;
  uint8_t *seat;

  for (uint8_t n = 0; n < UAC_MIC_SEATS; n++) {
    uint8_t i = (uint8_t)((s_mic_pkt_hint + n) % UAC_MIC_SEATS);

    if (!s_mic_pkt_busy[i]) {
      pick = i;
      break;
    }
  }
  if (pick >= UAC_MIC_SEATS)
    pick = s_mic_pkt_hint;

  s_mic_pkt_hint = (uint8_t)((pick + 1) % UAC_MIC_SEATS);
  s_mic_pkt_busy[pick] = true;

  seat = s_mic_pkt[pick];
  uac_mic_ring_pop(seat, UAC_MIC_PACKET);

  *out_idx = pick;
  return seat;
}

/* 往 IN 端点登记一包。调用者需保证端点当前空闲。 */
static void uac_mic_submit_next(void) {
  uint8_t idx;
  uint8_t *seat = uac_mic_seat_alloc(&idx);

  s_mic_inflight = (int8_t)idx;
  s_mic_tx_busy = true;

  if (usbd_ep_start_write(UAC_EP_IN, seat, UAC_MIC_PACKET) != 0) {
    /* 提交失败（端点没使能 / 上一包还没腾出硬件缓冲）：
     * 归还缓冲并清掉在飞标记，让下一次中断节拍重试，绝不能让 busy 卡死。 */
    s_mic_pkt_busy[idx] = false;
    s_mic_inflight = -1;
    s_mic_tx_busy = false;
  }
}

/* IN 端点发送"完全"完成回调（USB 中断上下文）。
 *
 * 注意这不是"包发完了"那么细，而是 usbd_ep_start_write() 登记的那一整段
 * 都进过 FIFO 了 —— Beken 端口在中断里按 MPS 分片，只有最后一片发完才
 * 回调（usb_dc_beken_musb_fdrc.c:726）。所以每包 UAC_MIC_PACKET(192)B
 * 只回调一次，nbytes 恒等于 UAC_MIC_PACKET。 */
static void uac_in_ep_callback(uint8_t ep, uint32_t nbytes) {
  int8_t idx = s_mic_inflight;

  (void)ep;
  (void)nbytes;

  s_dbg_in_ok++;

  /* 这一包已进硬件，缓冲可以还给采集侧了 */
  if (idx >= 0) {
    s_mic_pkt_busy[idx] = false;
    s_mic_inflight = -1;
  }
  s_mic_tx_busy = false;

  /* 立刻补下一包：这就是 1ms 节拍的全部来源 */
  if (s_mic_streaming)
    uac_mic_submit_next();
}

static struct usbd_endpoint s_ep_in = {
    .ep_cb = uac_in_ep_callback,
    .ep_addr = UAC_EP_IN,
};

/* 类驱动的 open/close 只带接口号，按接口号区分方向：
 *   UAC_AS_INTF(1)  = 播放 AS，主机开流 -> 开始收包
 *   UAC_MIC_INTF(2) = 录音 AS，主机开流 -> 开始向主机发包
 * AC 接口(0) 的 alt 恒为 0，不会被当成开流。 */
/* 【录音流启动：顺序不能变】
 *
 * 主机 SET_INTERFACE(alt=1) 之后，snd-usb-audio 就按 1ms 节拍开始收
 * ISO IN 包。端点一直空着，主机会判定端点停摆（arecord 报 overrun、
 * OBS 显示设备无输入）。而 IN 完成中断是后续所有包的**唯一**节拍源，
 * 第一包不提交 = 那个中断永远不会来 = 引擎死锁。
 *
 * 所以这里必须：清环 -> 清计数 -> **立刻提交第一包** -> 叫醒主循环去拉 DMIC。
 * 此时 DMIC 可能还没起来，ring 是空的，pop 会自动补静音顶着。 */
static void uac_mic_stream_start(void) {
  uac_mic_ring_reset();
  for (uint8_t i = 0; i < UAC_MIC_SEATS; i++)
    s_mic_pkt_busy[i] = false;
  s_dbg_in_ok = 0;
  s_dbg_starve = 0;
  s_dbg_underrun = 0;
  s_dbg_cap = 0;
  s_mic_pkt_hint = 0;
  s_mic_inflight = -1;
  s_mic_tx_busy = false;
  s_mic_streaming = true;
  s_pending_publish = true;

  uac_mic_submit_next();
  app_mode_poll_now();
}

/* 录音流停止：只清标志，让中断里的接力逻辑不再提交下一包 */
static void uac_mic_stream_stop(void) {
  if (!s_mic_streaming)
    return;

  s_mic_streaming = false;
  s_mic_inflight = -1;
  s_mic_tx_busy = false;
  s_pending_publish = true;
}

void usbd_audio_open(uint8_t intf) {
  if (intf == UAC_MIC_INTF) {
    /* 记录"主机想录音"，但**是否真的启动采集由模式决定**：
     * 只有 USB 麦克风模式才把 DMIC 接进来，普通 USB 声卡模式保持静音，
     * 避免和 DAC/其它模式抢音频子系统。 */
    s_mic_requested = true;

    if (!s_mic_mode) {
      LOGI("mic itf opened alt=%u, not in usb-mic mode -> stay silent\r\n",
           (unsigned)intf);
      return;
    }

    if (s_mic_streaming)
      return;

    LOGI("mic itf opened alt=%u, start capture\r\n", (unsigned)intf);
    uac_mic_stream_start();
    return;
  }

  if (intf != UAC_AS_INTF)
    return;

  if (s_streaming)
    return;

  /* 中断上下文：只置位，由 app_usb_poll() 在任务里处理 */
  s_pkt_idx = 0;
  s_streaming = true;
  s_pending_publish = true;
  s_pending_mode = true;

  usbd_ep_start_read(UAC_EP_OUT, s_pkt[0], UAC_OUT_MPS);
}

void usbd_audio_close(uint8_t intf) {
  if (intf == UAC_MIC_INTF) {
    s_mic_requested = false;
    uac_mic_stream_stop();
    return;
  }

  if (intf != UAC_AS_INTF)
    return;

  if (!s_streaming)
    return;

  /* 中断上下文：只置位，app_usb_poll() 会关音频 */
  s_streaming = false;
  s_pending_publish = true;
}

/* ==================== UAC 音量曲线（主机 % == 设备 VOL） ====================
 *
 * 类驱动交给这里的量是 UAC1 语义：0~255，其中 128 = 0dB（即 dB+128）。
 *
 * 【为什么不能按 dB 线性映射】
 * 桌面端（PulseAudio / PipeWire / WirePlumber，GNOME、KDE 的音量滑块都
 * 走它们）把 ALSA 的 dB 值换算成滑块百分比时，用的是**感知曲线**：
 *
 *      host_pct = 100 * 10^(dB / 60)          即 dB = 60 * log10(host_pct/100)
 *
 * 而本机 VOL 是线性的幅度百分比（bsp_audio_percent_to_gain 按比例给 DAC
 * 增益）。旧实现按 dB 线性映射（dB = (pct-100)*37/100），两边就对不上：
 *
 *      host 25%  ->  dB -36  ->  设备 VOL 3%      <-- 差 22 个点
 *      host 40%  ->  dB -24  ->  设备 VOL 35%
 *      host 60%  ->  dB -15  ->  设备 VOL 46%
 *
 * 这就是"电脑上显示的与设备上显示的不一致"。
 *
 * 【现在的做法】
 * 直接用上面那条感知曲线，量程放到 -127dB..0dB（1dB 步进，128 档，
 * 足够覆盖 1%..100%），并预生成两张表互查：
 *
 *      s_uac_db_of_pct[设备 VOL]  -> 要报给主机的 dB（GET_CUR）
 *      s_uac_pct_of_db[dB + 127]  -> 主机设来的 dB 对应的设备 VOL（SET_CUR）
 *
 * 于是主机滑块百分比 ≈ 设备 VOL，误差 ≤1%（1dB 量化所致，无法再小：
 * 0dB 附近 1dB 就相当于 4%）。
 *
 * 【注意】alsamixer / `amixer contents` 显示的是**原始 dB 档位**（0~127
 * 或 -127..0dB），不是百分比，和这里不冲突。
 *
 * 只有播放方向（ep == UAC_EP_OUT）的音量才映射到喇叭；麦克风 Feature
 * Unit 的 GET 一律回 0dB（主机显示满格），SET 忽略 —— 否则在系统
 * "录音设备"面板拖一下麦克风增益会把喇叭音量也改了。 */
#define UAC_VOL_DB_MIN  (-127)

/* 设备 VOL(%) -> 报给主机的 dB。表由 60*log10 反解生成，已校验
 * "主机设 X% -> 设备 X%" 往返一致（1dB 量化处最多差 1）。 */
static const int8_t s_uac_db_of_pct[101] = {
	-127, -120, -102,  -91,  -84,  -78,  -73,  -69,  -66,  -63,  -60,  -58,  -55,  -53,  -51,  -49,  -48,  -46,  -45,  -43,
	 -42,  -41,  -39,  -38,  -37,  -36,  -35,  -34,  -33,  -32,  -31,  -31,  -30,  -29,  -28,  -27,  -27,  -26,  -25,  -25,
	 -24,  -23,  -23,  -22,  -21,  -21,  -20,  -20,  -19,  -19,  -18,  -18,  -17,  -17,  -16,  -16,  -15,  -15,  -14,  -14,
	 -13,  -13,  -12,  -12,  -12,  -11,  -11,  -10,  -10,  -10,   -9,   -9,   -9,   -8,   -8,   -7,   -7,   -7,   -6,   -6,
	  -6,   -5,   -5,   -5,   -5,   -4,   -4,   -4,   -3,   -3,   -3,   -2,   -2,   -2,   -2,   -1,   -1,   -1,   -1,    0,
	   0,
};

/* 主机设来的 dB -> 设备 VOL(%)，下标 = dB - UAC_VOL_DB_MIN（0..127） */
static const int8_t s_uac_pct_of_db[128] = {
	   0,    0,    0,    0,    0,    0,    0,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    1,    2,    2,
	   2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    2,    3,    3,    3,    3,    3,    3,    3,    3,    3,
	   4,    4,    4,    4,    4,    4,    4,    5,    5,    5,    5,    5,    6,    6,    6,    6,    7,    7,    7,    7,
	   8,    8,    8,    9,    9,    9,   10,   10,   10,   11,   11,   12,   12,   13,   13,   14,   14,   15,   15,   16,
	  16,   17,   18,   18,   19,   20,   21,   22,   22,   23,   24,   25,   26,   27,   28,   29,   30,   32,   33,   34,
	  35,   37,   38,   40,   41,   43,   45,   46,   48,   50,   52,   54,   56,   58,   61,   63,   66,   68,   71,   74,
	  76,   79,   83,   86,   89,   93,   96,  100,
};

void usbd_audio_set_volume(uint8_t ep, uint8_t ch, int volume_db) {
  int db;
  int vol;

  (void)ch;

  if (ep != UAC_EP_OUT)
    return;

  db = volume_db - 128;              /* 还原成 dB（类驱动已折算成 dB+128） */
  if (db > 0)
    db = 0;
  if (db < UAC_VOL_DB_MIN)
    db = UAC_VOL_DB_MIN;

  vol = s_uac_pct_of_db[db - UAC_VOL_DB_MIN];

  LOGI("UAC vol: %d dB -> %d%%\r\n", db, vol);

  s_pending_volume_val = (uint8_t)vol;
  s_volume_cache = (uint8_t)vol;   /* 主机 GET_CUR 立即读到新值 */
  s_pending_volume = true;
}

int usbd_audio_get_volume(uint8_t ep, uint8_t ch) {
  uint8_t pct;

  (void)ch;

  /* 麦克风 FU：永远报 0dB，主机侧显示满格；本地不参与 */
  if (ep != UAC_EP_OUT)
    return 128;

  pct = s_volume_cache;
  if (pct > 100)
    pct = 100;

  return 128 + s_uac_db_of_pct[pct];   /* 与 SET_CUR 的映射严格互逆 */
}

void usbd_audio_set_mute(uint8_t ep, uint8_t ch, bool mute) {
  (void)ch;

  /* 麦克风的静音请求不落到喇叭上 */
  if (ep != UAC_EP_OUT)
    return;

  if (mute == s_muted)
    return;

  s_muted = mute;
  s_pending_mute_val = mute;
  s_pending_mute = true;
}

bool usbd_audio_get_mute(uint8_t ep, uint8_t ch) {
  (void)ch;

  if (ep != UAC_EP_OUT)
    return false;

  return s_muted;
}

/* 主机通过 Clock Source(SAM_FREQ) / 端点(SAMPLING_FREQ) 设置的采样率。
 * 播放与录音各自独立记录：主机在"录音设备"里选的采样率不该影响喇叭。 */
static volatile uint32_t s_stream_rate = UAC_RATE;
static volatile uint32_t s_mic_rate = UAC_RATE;

void usbd_audio_set_sampling_freq(uint8_t ep, uint32_t sampling_freq) {
  if (ep == UAC_EP_IN) {
    /* 录音侧只接受 48k：DMIC 通路每 20ms 固定出 960 点，做别的速率
     * 需要重采样，这里不支持。主机请求别的值时静默保持 48k，
     * 主机后续 GET_CUR 会看到 48000，自己调整。 */
    if (sampling_freq != s_mic_rate) {
      if (sampling_freq == UAC_RATE) {
        s_mic_rate = sampling_freq;
        LOGI("UAC mic rate -> %u Hz\r\n", (unsigned)sampling_freq);
      } else {
        LOGW("UAC mic rate %u not supported, keep %u\r\n",
             (unsigned)sampling_freq, (unsigned)s_mic_rate);
      }
    }
    return;
  }

  if ((sampling_freq >= 8000) && (sampling_freq <= 96000) &&
      (sampling_freq != s_stream_rate)) {
    LOGI("UAC rate -> %u Hz\r\n", (unsigned)sampling_freq);
    s_stream_rate = sampling_freq;
  }
}

uint32_t usbd_audio_get_sampling_freq(uint8_t ep) {
  if (ep == UAC_EP_IN)
    return s_mic_rate;
  return s_stream_rate;
}

/* UAC2 的 GET_RANGE(Clock Source) 响应：
 *   wNumSubranges(2B) + N × { dMin(4B), dMax(4B), dRes(4B) }
 * 只报 48k 单档 —— 与配置描述符里固定的包长严格一致，
 * 主机就不会选到别的速率（选了会导致包长不匹配、欠载/溢出变调）。
 * 注意：类驱动在 UAC2 分支里会直接解引用这个指针，**不能返回 NULL**。 */
void usbd_audio_get_sampling_freq_table(uint8_t ep,
                                        uint8_t **sampling_freq_table) {
  static uint8_t s_rate_range_out[] = {
      0x01, 0x00,                          /* wNumSubranges = 1 */
      0x80, 0xBB, 0x00, 0x00,              /* dMin = 48000 */
      0x80, 0xBB, 0x00, 0x00,              /* dMax = 48000 */
      0x01, 0x00, 0x00, 0x00,              /* dRes = 1 Hz */
  };
  static uint8_t s_rate_range_in[] = {
      0x01, 0x00,                          /* wNumSubranges = 1 */
      0x80, 0xBB, 0x00, 0x00,              /* dMin = 48000 */
      0x80, 0xBB, 0x00, 0x00,              /* dMax = 48000 */
      0x01, 0x00, 0x00, 0x00,              /* dRes = 1 Hz */
  };

  if (sampling_freq_table)
    *sampling_freq_table = (ep == UAC_EP_IN) ? s_rate_range_in : s_rate_range_out;
}

/* AC(0)/播放 AS(1)/录音 AS(2) 三个接口共用一个通知处理：
 *   CONFIGURED/RESET/DISCONNECTED 由类驱动广播给所有接口，直接置位；
 *   SET_INTERFACE 只认两个 AS 接口 —— alt=1 表示主机开流，alt=0 表示关流。
 * 注意类驱动自己也有一个 audio_notify_handler，注册时会被这里的赋值覆盖掉，
 * 所以 open/close 的分发要在这里自己做全。 */
static void uac_notify_handler(uint8_t event, void *arg) {
  switch (event) {
  case USBD_EVENT_CONFIGURED:
    s_configured = true;
    s_pending_publish = true;
    LOGI("usb configured\r\n");
    break;
  case USBD_EVENT_SET_INTERFACE: {
    struct usb_interface_descriptor *d = (struct usb_interface_descriptor *)arg;

    if (!d)
      break;
    if ((d->bInterfaceNumber != UAC_AS_INTF) &&
        (d->bInterfaceNumber != UAC_MIC_INTF))
      break;
    LOGI("usb set_itf %u alt=%u\r\n", (unsigned)d->bInterfaceNumber,
         (unsigned)d->bAlternateSetting);
    if (d->bAlternateSetting)
      usbd_audio_open(d->bInterfaceNumber);
    else
      usbd_audio_close(d->bInterfaceNumber);
    break;
  }
  case USBD_EVENT_RESET:
  case USBD_EVENT_DISCONNECTED:
    /* 这两条是定位"录音途中突然 EBADFD / 设备每几秒重连"的关键：
     * 主机侧看到 disconnect，说明设备端 D+ 上拉掉了 —— 要么 USB 栈被
     * 复位，要么整个 AP 重启了。看到这行就说明问题在设备侧，不是主机。 */
    LOGE("usb %s (in_ok=%u cap=%u starve=%u underrun=%u)\r\n",
         (event == USBD_EVENT_RESET) ? "RESET" : "DISCONNECTED",
         (unsigned)s_dbg_in_ok, (unsigned)s_dbg_cap,
         (unsigned)s_dbg_starve, (unsigned)s_dbg_underrun);
    s_configured = false;
    s_streaming = false;
    s_mic_streaming = false;
    s_mic_inflight = -1;
    s_mic_tx_busy = false;
    s_pending_publish = true;
    break;
  default:
    break;
  }
}

/* ============================ 初始化 / 模式接口 ============================
 */

static void usb_init_task(void *arg) {
  bk_err_t ret;

  (void)arg;

  usb_slist_init(&usbd_intf_head);
  usbd_desc_register(s_uac_descriptor);
  usbd_add_interface(usbd_audio_init_intf(
      &s_intf_ac, UAC_AUDIO_VERSION, s_entity, sizeof(s_entity) / sizeof(s_entity[0])));
  usbd_add_interface(usbd_audio_init_intf(
      &s_intf_as, UAC_AUDIO_VERSION, s_entity, sizeof(s_entity) / sizeof(s_entity[0])));
  usbd_add_interface(usbd_audio_init_intf(
      &s_intf_mic, UAC_AUDIO_VERSION, s_entity, sizeof(s_entity) / sizeof(s_entity[0])));
  usbd_add_endpoint(&s_ep_out);
  usbd_add_endpoint(&s_ep_in);

  /* 三个接口都用统一的通知处理：既有配置/复位事件，也有 SET_INTERFACE */
  s_intf_ac.notify_handler = uac_notify_handler;
  s_intf_as.notify_handler = uac_notify_handler;
  s_intf_mic.notify_handler = uac_notify_handler;

  ret = bk_usb_open(USB_DEVICE_MODE);
  if (ret != BK_OK) {
    /* bk_usb_open() 内部只在"驱动未初始化"或"已经打开过"时才会返回
     * 非 OK（正常情况下它无条件返回 BK_OK），所以这里一旦命中，
     * 基本就是重复调用或初始化顺序问题。 */
    LOGE("bk_usb_open failed: %d\r\n", ret);
    app_state_toast("USB 初始化失败");
    s_init_task = NULL;
    rtos_delete_thread(NULL);
    return;
  }

  /* 走到这里只代表"设备侧已经打开、描述符已注册"，
   * **不代表主机已经枚举成功** —— 那要等主机发 SET_CONFIGURATION，
   * 由 uac_notify_handler 打出 "usb configured"。
   * 排查"电脑没反应"时先确认有没有这一行。 */
  LOGI("uac registered (spk+mic), waiting host enumeration...\r\n");
  s_init_task = NULL;
  rtos_delete_thread(NULL);
}

void app_usb_start(void) {
  if (s_init_task)
    return;

  if (rtos_create_thread(&s_init_task, BEKEN_DEFAULT_WORKER_PRIORITY, "app_usb",
                         (beken_thread_function_t)usb_init_task, 4096,
                         NULL) != kNoErr) {
    s_init_task = NULL;
    LOGE("create usb init task failed\r\n");
  }
}

/* ==================== USB 麦克风模式 ====================
 *
 * 与普通 USB 声卡模式的区别：
 *   · 不打开 DAC 播放通路（s_active 保持 false）—— 这个模式只做麦克风
 *   · 允许把 DMIC 采集接进 UAC 的 IN 端点（s_mic_mode = true）
 *
 * 描述符不变：UAC 设备始终同时暴露扬声器 + 麦克风两个接口，
 * 切换模式只影响**本机是否真的去采**。这样不用动态改描述符，
 * 主机侧不需要重新枚举，切模式只是"麦克风开始/停止出数据"。 */
void app_usb_set_mic_mode(bool on) {
  if (s_mic_mode == on)
    return;

  s_mic_mode = on;

  if (on) {
    LOGI("enter usb mic mode\r\n");
    /* 主机可能在切到这个模式**之前**就打开了录音流（那时我们按
     * "非麦克风模式"忽略了 open），这里补启动，否则用户会看到
     * "录音设备选好了但一直是静音"。 */
    if (s_mic_requested && !s_mic_streaming)
      uac_mic_stream_start();
  } else {
    LOGI("leave usb mic mode\r\n");
    uac_mic_stream_stop();
    uac_mic_task_stop();
    bsp_mic_deinit();
  }

  publish();
}

void app_usb_set_active(bool active) {
  s_active = active;
  s_volume_cache = app_state_volume();

  if (active) {
    LOGI("enter usb mode\r\n");
    uac_drain_stop();
    s_wr = s_rd = 0;

    if (s_streaming) {
      if (!app_audio_is_open() &&
          (app_audio_open(s_stream_rate, UAC_CHANNELS) != BK_OK))
        LOGE("usb audio open failed\r\n");
      if (app_audio_is_open()) {
        if (s_muted)
          app_audio_set_volume(0);
        uac_drain_start();
      }
    }
    /* 麦克风不在这里启动：录音流由主机控制，见 app_usb_poll() 里的
     * s_mic_streaming 边沿检测 —— 主机可能在任何页面开始/结束录音。 */
  } else {
    LOGI("leave usb mode\r\n");
    uac_drain_stop();
    app_audio_close();
    s_applied_volume = 0xFF;   /* 音频层已关，下次开流要重新下发音量 */

    /* 离开 USB 模式时把麦克风也停掉：DMIC 与 DAC 同属音频子系统，
     * 留着空转会拖慢其它模式的播放。主机若仍在录音，回到 USB 模式时
     * poll 的边沿检测不会重新触发，所以只在真正离开时停。 */
    if (s_mic_streaming) {
      uac_mic_task_stop();
      bsp_mic_deinit();
    }
  }

  publish();
}

void app_usb_poll(void) {
  static bool s_was_streaming;
  static bool s_was_mic_streaming;
  uint8_t vol;

  /* 主机刚开流：先停 drain，再安全地清空环形缓冲 */
  if (s_streaming && !s_was_streaming) {
    if (s_active)
      uac_drain_stop();
    s_wr = s_rd = 0;
    if (s_active && app_audio_is_open())
      uac_drain_start();
  }
  s_was_streaming = s_streaming;

  /* 麦克风开/关流：无论当前是否停在 USB 页都要响应，
   * 否则主机在别的页面开始录音会一直发空包（听上去是静音）。 */
  if (s_mic_streaming != s_was_mic_streaming) {
    if (s_mic_streaming && s_mic_mode) {
      bsp_mic_cfg_t mc;

      bsp_mic_default_cfg(&mc);
      mc.samp_rate = UAC_RATE;
      mc.stereo = true;
      if (!bsp_mic_is_running() &&
          ((bsp_mic_init(&mc) != BK_OK) || (bsp_mic_start() != BK_OK)))
        LOGE("mic start failed\r\n");
      if (bsp_mic_is_running())
        uac_mic_task_start();
      LOGI("mic stream on, %uHz %uch 16bit, %uB/pkt\r\n",
           (unsigned)UAC_RATE, (unsigned)UAC_MIC_CHANNELS,
           (unsigned)UAC_MIC_PACKET);
    } else {
      uac_mic_task_stop();
      if (bsp_mic_is_running())
        bsp_mic_deinit();
      LOGI("mic stream off (in_ok=%u starve=%u underrun=%u cap=%u)\r\n",
           (unsigned)s_dbg_in_ok, (unsigned)s_dbg_starve,
           (unsigned)s_dbg_underrun, (unsigned)s_dbg_cap);
    }
    s_was_mic_streaming = s_mic_streaming;
  }

  /* 发包引擎的"点火"：收到主机 SET_INTERFACE(alt=1) 后如果端点还空着，
   * 就给它提交第一包占住位置。之后每一包都由 IN 完成中断接力提交，
   * 这个轮询只作为**恢复手段**（中断万一丢了不至于永久哑掉），
   * 正常情况下 s_mic_tx_busy 恒为 true，这里什么都不会做。
   *
   * 早期版本写成 "for (i<12) kick()" 来补齐漏掉的 1ms 包，那是错的：
   * 本函数由 200ms 轮询驱动，一次最多补 12ms，其余 188ms 全在欠载。
   * 现在真正的节拍在 uac_in_ep_callback() 里。 */
  if (s_mic_streaming && bsp_mic_is_running() && !s_mic_tx_busy)
    uac_mic_submit_next();

  /* 录音诊断：每 2s 打一行，只在开流时打。
   *
   * 排查"主机能看到设备但没声音"时，这一行就能定位断在哪一段：
   *   in_ok  ≈ 2000/2s  → USB 侧在正常发包
   *   cap    ≈ 100/2s   → DMIC 在正常出帧
   *   starve 持续增长     → 采集比 USB 消耗慢（任务被饿死）
   *   in_ok 不涨          → 端点没被主机使能 / 提交全失败
   *   cap 不涨            → DMIC 通路没起来（回到 bsp_mic 排查） */
  if (s_mic_streaming) {
    static uint32_t s_dbg_tick;

    if (++s_dbg_tick >= 10) {   /* poll 周期 200ms -> 2s */
      s_dbg_tick = 0;
      LOGI("uac mic: in_ok=%u cap=%u starve=%u underrun=%u ring=%u\r\n",
           (unsigned)s_dbg_in_ok, (unsigned)s_dbg_cap,
           (unsigned)s_dbg_starve, (unsigned)s_dbg_underrun,
           (unsigned)(s_mic_wr - s_mic_rd));
    }
  }

  /* 【插上电脑没反应时看这里】
   *
   * 设备侧打开（"uac registered"）和主机枚举成功是两件事。主机要发
   * SET_CONFIGURATION，才会走到 uac_notify_handler 的 CONFIGURED 分支
   * 打出 "usb configured"。如果插线后一直只有 registered 没有 configured，
   * 说明主机压根没枚举 —— 排查方向是 D+/D- 接线、USB 供电、晶振/时钟，
   * 而不是 UAC 描述符（描述符有问题时会先枚举成"未知设备"）。
   *
   * 每 5s 打一次，避免刷屏；一旦 connected 就不再打扰。 */
  if (!s_configured) {
    static uint32_t s_wait_tick;

    if (++s_wait_tick >= 25) {   /* poll 周期 200ms -> 5s */
      s_wait_tick = 0;
      LOGW("usb: no host enumeration yet (waiting for SET_CONFIGURATION)\r\n");
    }
  }

  /* 1. 处理 USB 中断里置位的事件 */
  if (s_pending_mode) {
    s_pending_mode = false;
    app_mode_request(APP_MODE_USB);
  }

  if (s_pending_mute) {
    s_pending_mute = false;
    if (s_pending_mute_val) {
      s_saved_volume = app_state_volume();
      if (s_active)
        app_audio_set_volume(0);
    } else if (s_active && !s_muted) {
      app_audio_set_volume(s_saved_volume);
    }
  }

  if (s_pending_volume) {
    s_pending_volume = false;
    s_volume_cache = s_pending_volume_val;
    /* app_state 是唯一的音量真值：主机设了就写进去，UI/其它模式都跟着变 */
    app_state_set_volume(s_pending_volume_val);
    if (app_audio_is_open() && !s_muted && (s_active || s_streaming)) {
      app_audio_set_volume(s_pending_volume_val);
      s_applied_volume = s_pending_volume_val;
    }
  } else {
    /* 没收到主机事件时（例如本机按键改过音量），把影子值对齐 app_state，
     * 保证主机 GET_CUR 读到的就是我们真正的音量 */
    vol = app_state_volume();
    if (vol != s_volume_cache)
      s_volume_cache = vol;
  }

  /* 收敛校正：USB 占用 DAC 期间，任何遗漏的事件都由这里补上。
   * 主机经常在音频流还没建立时就设过音量，开流后必须补一次。 */
  if (app_audio_is_open() && !s_muted && (s_active || s_streaming)) {
    uint8_t want = app_state_volume();

    if (want != s_applied_volume) {
      app_audio_set_volume(want);
      s_applied_volume = want;
      s_volume_cache = want;
      LOGI("UAC vol apply %u%% (active=%d stream=%d)\r\n", (unsigned)want,
           (int)s_active, (int)s_streaming);
    }
  }

  if (s_pending_publish) {
    s_pending_publish = false;
    publish();
  }

  /* 2. 主机开/关流后，音频通道跟着开/关 */
  if (s_streaming && s_active) {
    if (!app_audio_is_open()) {
      uac_drain_start();
      if (app_audio_open(s_stream_rate, UAC_CHANNELS) != BK_OK)
        LOGE("usb audio open failed\r\n");
      publish();
    }
  } else if (!s_streaming && s_active) {
    if (app_audio_is_open()) {
      uac_drain_stop();
      app_audio_close();
      publish();
    }
  }
}
