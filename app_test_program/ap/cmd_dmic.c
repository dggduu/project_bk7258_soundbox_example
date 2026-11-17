// Copyright 2026
// CherrySH 命令：dmic —— 数字麦克风（DMIC）采集 / 电平测试 / 录音到 SD 卡
//
//   dmic                   采 200ms，打印左右声道电平
//   dmic <ms>              指定采集时长（10..2000ms）
//   dmic raw               打印 16 个原始 FIFO 值（排查格式/数据通路）
//   dmic dbg               打印 DMIC 寄存器/引脚/电源自检（排查 no data）
//   dmic rec [sec] [file]  录音到 SD 卡 WAV（默认 5s，默认 1:/dmic.wav）
//   dmic adk rec [sec] [f] 同上，但走官方 onboard_dual_dmic_mic_stream 组件
//                          （默认 5s，默认 1:/adk.wav；受 dmic shift 影响）
//   dmic stop              停止录音（录音是后台任务，shell 不阻塞）
//   dmic status            查看录音状态
//
// 板上 DMIC：CLK=P8、DAT=P9（SDK 的 bk_aud_dmic_init() 里硬编码的就是
// GPIO_8/GPIO_9，正好对得上），供电走 P48 的 MIC 1V8 LDO。
//
// 数据格式：DMIC FIFO 每个 32bit word 含两个 16bit 半字。取哪一路写成
// 单声道 PCM 由 `dmic pick [h|l]` 运行时决定，**默认取低 16bit**：
// 实测高半字几乎恒定（≈-80）、低半字才有变化，早期按 bsp_mic.c 的
// left_is_high=true 取高半字，录出来是恒定直流。
//
// 录音通路（单帧 DMA 窗口 + ISR 快照）：
//   DMIC FIFO --DMA(REPEAT, 单帧)--> DTCM 缓冲 --ISR memcpy--> s_dma_snap
//             --录音任务--> FATFS --> SD 卡
// 起停顺序按官方例程：bk_dma_start() → bk_aud_dmic_start() → bk_aud_adc_start()；
// 停止反过来。ADC 必须用 `dmic cfg drain on` 排空 FIFO，否则约 0.5s 后
// 共享通路被拖死、CIC 输出冻结。

#include <string.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>

#include "csh.h"
#include "csh_port.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"
#include "bsp_audio.h"
#include "gpio_driver.h"
#include <driver/gpio.h>
#include <driver/aud_dmic.h>
#include <driver/aud_adc.h>
#include <driver/dma.h>
#include <driver/sd_card.h>
#include <driver/sdio_host.h>
#include <driver/pwr_clk.h>
#include "gpio_ll.h"        /* gpio_ll_get_gpio_perial_mode() */
#include <os/os.h>
#include <os/mem.h>
#include "ff.h"

/* soc 层的寄存器访问宏（aud_ll_get_audio_config_* 等）。
 * 这里直接引 aud_ll_macro_def.h 而不是 aud_hal.h：后者在
 * ap/middleware/soc/common/hal/include 下，那个目录只在 soc/common
 * 组件内部可见，app 组件拿不到它的 include 路径；而
 * aud_ll_macro_def.h 在 soc/bk7258_ap/hal 下，是自包含的（只依赖
 * <soc/soc.h>），app 侧能直接引。 */
#include <aud_ll_macro_def.h>
/* 时钟树的 getter（cksel_aud / dmic 分频位 / aud_cken / pwdaudpll）。
 * 和 aud_ll_macro_def.h 同在 soc/bk7258_ap/hal 下，app 侧可直接引。 */
#include <sys_ll.h>
/* sys_drv_aud_clock_en() / sys_drv_aud_select_clock()：显式打开 aud_cken、
 * 选 cksel_aud；bk_pm_module_power_state_get()：读 AUDP 电源域状态。 */
#include "sys_driver.h"

/* 官方 ADK 双麦流组件（`dmic adk`）。由
 * CONFIG_ADK_ONBOARD_DUAL_DMIC_MIC_STREAM 控制，未开启时不参与编译。 */
#if CONFIG_ADK_ONBOARD_DUAL_DMIC_MIC_STREAM
#include <components/bk_audio/audio_streams/onboard_dual_dmic_mic_stream.h>
#include <components/bk_audio/audio_pipeline/audio_element.h>
#include <components/bk_audio/audio_pipeline/audio_port.h>
#include <components/bk_audio/audio_pipeline/rb_port.h>
#endif
#include <modules/pm.h>
/* 采样率选择说明（已按实测修正）：
 *
 * 板上麦是 LMD4030T261-OAS02（LinkMems），单比特 PDM 输出，规格书要求
 *   - VDD 1.62~3.6V（本板 1.8V，OK）
 *   - 时钟频率 标准模式 1000~4800kHz，典型 2400kHz
 *   - 时钟 duty cycle 45%~55%
 *
 * SDK 里 DMIC CLK = 24576kHz / (dig_mic_div+1)：
 *   16000Hz → div=23 → CLK=1024kHz
 *   32000Hz → div=11 → CLK=2048kHz
 *   48000Hz → div=7  → CLK=3072kHz（手册里 3.072MHz 那个例子）
 *
 * **实测结论（2026-11-12，两轮 A/B 已完成）**：
 *   16000Hz、32000Hz **都正常出数**：
 *     16k: got 3200/3200, L rms=8824  R rms=6531
 *     32k: got 6400/6400, L rms=6461  R rms=4583
 *   早先"32k 出不来数"的锅不在采样率，而在 dmic_hw_init() 里那三个伪补丁
 *   （清 soft_reset 把 clk_ctrl 从 0x1 改成 0x0）。补丁一关，两个速率都能跑。
 *
 * 默认仍取 16000：与官方录音卡 ADC_FS 一致、语音记录够用、SD 带宽减半。
 * 需要更高保真时运行时切即可，不必重新烧固件：
 *   dmic rate 32000
 *
 * 手册写"PDM 8:1 CIC、PDM 3.072MHz 时 PCM 最高 384kHz"，但本 SDK 的
 * dmic_cic_sel 只有 16/32/64（aud_dmic_types.h: AUD_DMIC_CIC_SEL_16/32/64，
 * 0..3 映射 16/32/64/64），拿不到 8:1，所以实际可用上限是 48kHz
 * （3.072MHz / 64）。不要去追 96k/192k。
 */
/* 默认 32000，不是 16000。手册(LMD4030T261 Operation Ratings)要求：
 *
 *   Sleep Mode      : 0 ~ 10 kHz
 *   Low Power Mode  : 150 ~ 900 kHz（典型 768 kHz）
 *   Standard Mode   : 1300 ~ 4800 kHz
 *   Clock Duty      : 40 ~ 60 %
 *
 * SDK 的 PDM CLK = 24576/(dig_mic_div+1) kHz（CIC 固定 ÷64）：
 *   16000 → div 23 → **1024 kHz**  ← 落在 900~1300 的空档里，**出规格**！
 *   32000 → div 11 → 2048 kHz ✔ Standard
 *   48000 → div 7  → 3072 kHz ✔ Standard
 *
 * 16 kHz 这种"出规格"时钟能起振、短采样(200ms)也正常，但长时间跑麦克风可能
 * 失去锁相/停推 DATA —— 与"约 0.5s 后 DMIC 输出冻结成恒定值"的现象吻合
 * （此前所有冻结实测都在 16 kHz 下）。所以默认改用 32 kHz。
 * 需要 16 kHz 时用 `dmic rate 16000` 复现对照。 */
#define DMIC_SAMP_RATE      32000

/* ADC 预初始化的采样率：**固定 16000，不跟随 DMIC 采样率**。
 *
 * 原因：bk_aud_adc_set_samp_rate() 在 case 32000 分支里会置
 *   aud_hal_set_extend_cfg_adc_fracmod_manual(1) + adc_fracmod_value(CONST_DIV_32K)
 * 走 ADC 自己的小数分频通路（aud_adc_driver.c:234-238），16000 走常规分频。
 * ADC 在这条链路里只为"借电源"，没必要去碰小数分频，少一个变量。
 *
 * 注：`dmic ref 200 32000` 已证明即使 ADC 在 32kHz（走 fracmod 通路）
 * DMIC 也能正常出数，所以这里只是保守做法，不是硬性要求。 */
#define DMIC_ADC_PREINIT_RATE   16000
#define DMIC_MIN_MS         10
#define DMIC_MAX_MS         2000
#define DMIC_FIRST_WORDS    8

/* 取哪个 16bit 作为录音声道，运行时可用 `dmic pick [h|l]` 切换。
 *
 * 【2026-11-12 实测改默认】多次 raw 里高 16 位几乎恒定、低 16 位在变化：
 *   FFB0FF9C  FFAFFFB8  FFB0FF6E  FFAEFFE1
 *   高: -80   -81       -80      -82      ← 恒定
 *   低: -100  -72       -146     -31      ← 有变化
 * 早期按 bsp_mic.c 的说法把默认设成"高 16bit = 左"，导致录进 WAV 的是
 * 恒定直流。默认改为取**低 16 位**，保留 h 供单变量对照。
 * 旧宏 DMIC_LEFT_IS_HIGH 已废弃，不再使用。 */

/* ---------------- 录音参数 ---------------- */
#define DMIC_REC_FRAME_MS   20                      /* 每帧时长 */

/* 【已废弃，勿再使用】原来是 4 帧环形缓冲，靠 (s_dma_cnt-1)%4 猜槽位。
 * 实测证明 REPEAT 模式每轮会把 dst 指针重载回 start_addr，数据只落在槽 0，
 * 其余槽永远是 memset 的 0 —— 结果 WAV 里 peak 正常但 rms 只有 1/5。
 * 现在改成单帧窗口 + 固定读 buf[0]，这个宏已无引用，仅留作说明。 */
//#define DMIC_REC_BUF_FRAMES 4

#define DMIC_REC_DEF_SEC    5
#define DMIC_REC_MAX_SEC    120
#define DMIC_REC_TASK_STACK 6144
#define DMIC_REC_STOP_TO    3000                    /* 等任务退出上限 ms */
#define DMIC_REC_PATH_LEN   96

static int16_t w_low(uint32_t w)
{
	return (int16_t)(w & 0xFFFF);
}

static int16_t w_high(uint32_t w)
{
	return (int16_t)(w >> 16);
}

/* 默认取低 16 位（有变化那一路），`dmic pick h` 可切回高 16 位做对照 */
static bool s_pick_high = false;

static int16_t w_pick(uint32_t w)
{
	return s_pick_high ? w_high(w) : w_low(w);
}

/* 整数开方（不引 libm） */
static uint32_t isqrt64(uint64_t v)
{
	uint64_t x = v;
	uint64_t y = (x + 1) / 2;

	if (x == 0)
		return 0;

	while (y < x) {
		x = y;
		y = (x + v / x) / 2;
	}

	return (uint32_t)x;
}

/* 峰值 -> 0.1dBFS 整数（-267 就是 -26.7dBFS）。
 * 用「每翻一倍算 -6dB」的整数迭代，避免为一个日志引入 log10/浮点。
 * 6.00 vs 6.02 的误差在 60dB 跨度内 <0.2dB，判"有没有声"足够。 */
static int dmic_pk_to_db100(int32_t pk)
{
	int db = 0;

	if (pk <= 0)
		return -9990;

	while ((pk < 32768) && (db > -960)) {
		pk <<= 1;
		db -= 6;
	}

	return db;
}

/* 复用表里的模式索引：DMIC1_CLK / DMIC1_DAT 都排第 4 个（索引 3） */
#define DMIC_GPIO_FUNC_MODE  3

/* 三个"补丁"拆成独立位，可单独开关（`dmic patch [mask]`）。
 *
 * 早期把三者捆成一个 bool 一起开关，导致 A/B 结论被污染：三合一为 ON 时
 * 失败，就被判定成"三个都是伪缺陷"，但实际上坏的只有 soft_reset 那一步。
 *
 * 现在默认只开 PATCH_ADCKEN（保持 audio APB 时钟）：
 *   - aud_cken：如果 PDM 时钟被 PM 空闲门控，这一步就是解药；
 *   - soft_reset 写 0：已实测有害（0x1 才是可用态），默认关；
 *   - clk_gate 写 0：已实测无必要，默认关。 */
#define PATCH_ADCKEN    0x1u    /* sys_drv_aud_clock_en(1) */
#define PATCH_SOFTRST   0x2u    /* soft_reset 脉冲 1→0 */
#define PATCH_CLKGATE   0x4u    /* 把 clk_gate 清 0 */
static uint32_t s_patch_mask = PATCH_ADCKEN;

/* 是否在启动 DMIC 之后顺便把 ADC 也 start。
 * 实测：ADC 不 start 时 DMIC **完全不出数**（fifo=0x00A0F000 恒 empty），
 * ADC start 时能出数但约 0.5s 后冻结。所以它既是"必需"又"有害"，
 * 默认保持 ON（至少能出数），用 `dmic cfg adc off` 复现"完全没数"。 */
static bool s_adc_start_en = true;

/* 是否在音频期间投 CPU 频率票（PM_CPU_FRQ_480M）。
 * 照官方 onboard_mic_record.c:731 加的，但实测加上后 DMIC 完全不出数，
 * 默认 OFF。用 `dmic cfg freq on` 复现。 */
static bool s_freq_vote_en = false;

/* 是否给 ADC FIFO 配一个"只丢数据"的排空 DMA（`dmic cfg drain on|off`）。
 *
 * 背景：ADC 必须 start，DMIC 才出数（见 dmic_hw_start 的注释）；但 ADC FIFO
 * 没有任何人去读，会持续灌满。怀疑这就是"约 0.5s 后 DMIC 输出冻结成恒定值"
 * 的机理——官方录音卡给 AMIC 配了独立 DMA 排空（aud_ns_recoder.c:614/632
 * 的 amic_dma），我们只要 DMIC，所以用一个只丢数据的 DMA 等效替代。
 * 默认 OFF（保持当前基线），用 `dmic cfg drain on` 做 A/B。 */
static bool s_adc_drain_en = false;

/* 运行时采样率（`dmic rate [n]` 可改，免去每换一次速率就重烧固件）。
 * 16k / 32k / 48k 均已实测可用；默认沿用官方录音卡的 16000。 */
static uint32_t s_dmic_rate = DMIC_SAMP_RATE;

/* 运行时数字增益（`dmic gain [n]`）。0xFF = 不干预，用 SDK 默认 0x2d。
 *
 * 依据：AVDK 的 onboard_dual_dmic_mic_stream_set_digital_gain() 就是调用
 * bk_aud_adc_set_gain()，说明 audio_config0 里的 adc_set_gain 对 DMIC 通路
 * 同样生效（DMIC 的 CIC 输出走的是同一条数字增益级）。
 * adc_gain 取值 0x00~0x3f，默认 0x2d，最大 0x3f 还有约 +10dB 余量。
 *
 * **实测打脸（2026-11-12 18:32）**：0x2d -> 0x3f 名义上 +18dB（8 倍），
 * 但 DMIC 采到的底噪反而从 rms≈110 掉到 ≈80，说明 adc_set_gain 对
 * DMIC 数据基本不生效（那 2.8dB 是环境漂移）。所以 `dmic gain` 只保留
 * 给手写 ADC 路径对照用，**DMIC 要放大请用 `dmic shift`**。
 */
static uint8_t s_dmic_gain = 0xFF;

/* DMIC 专用数字增益：组件的 dmic_data_shift，在 L/R 选完之后对 int16
 * 直接移位（onboard_dual_dmic_data_shift，`.c:702`）。
 * 正数左移放大（+1 = +6dB），负数右移衰减，0 = 原始。
 * 这是纯软件运算，不依赖任何模拟增益寄存器，**一定生效**。 */
static int32_t s_dmic_shift = 0;

/* 实现在下面的 "自检" 区，这里先声明给 dmic_hw_start() 用 */
static uint32_t dmic_gpio_func_mode(int pad);
static void dmic_gpio_func_mode_set(int pad, uint32_t mode);

/* DMIC 初始化（不含 start）。
 *
 * 只保留一条**被实测证明有效**的关键动作：
 *
 * 【必须】audio 模块模拟总电源要先开
 *   aud_common_driver.c:374 那句 `//sys_hal_aud_aud_en(1);` 是**被注释掉的**，
 *   所以 bk_aud_driver_init() 不会开总电源。全 SDK 只有 ADC 路径会开
 *   （aud_adc_driver.c:102 sys_drv_aud_aud_en(1)）。
 *   → 照官方 onboard_dual_dmic_mic_stream.c:916/950 的做法，先
 *     bk_aud_adc_init() 借这条路径把总电源/ANA 拉起来，再 init DMIC。
 *   实测：这一步拿掉后 ADC FIFO 都读不到数，是真正的硬依赖。
 *
 * 【已证伪】soft_reset 清零 / clk_gate 修正 / 显式 aud_cken
 *   这三步曾被认为是"SDK 缺陷"，2026-11-12 的对照实验推翻了它：
 *   官方录音卡流程完全不做这三步（clk_ctrl 保持 0x1），`dmic ref`
 *   在 16kHz 下 got 3200/3200、左右声道 rms 8499/6605，数据完全正常；
 *   而打开这三步（clk_ctrl 被改成 0x0）时 fifo 恒 empty、一个 word 都读不到。
 *   → 默认关闭，代码保留在 s_patch_en 开关后面，供单变量复核。
 */
static bk_err_t dmic_hw_init(chry_shell_t *csh, aud_dmic_config_t *cfg)
{
	aud_adc_config_t adc_cfg = DEFAULT_AUD_ADC_CONFIG();
	bk_err_t ret;

	/* 0. 音频期间把 CPU 频率钉到 480MHz。
	 *
	 * 照抄官方 AMIC 采集的做法（bk_avdk_voice_record/.../onboard_mic_record.c:731
	 * 的 bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_480M)）。
	 * PM 在空闲后会降频，降频可能打断 APLL/audio 时钟链路；实测出现过
	 * "录音前 0.5s 正常、之后 DMIC 输出冻结成恒定值"的现象，PM 降频是
	 * 嫌疑之一。dmic_hw_stop() 里配对释放。 */
	if (s_freq_vote_en)
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_480M);

	/* 1. 先开 audio 模块总电源（ADC 路径内部会 sys_drv_aud_aud_en(1)）。
	 *    ADC 本身不用它的数据，只是借它把公共电源/时钟/ANA 拉起来。
	 *    mic 电源、gain 都按最小配置，避免干扰 DMIC。
	 *
	 *    注意采样率固定用 DMIC_ADC_PREINIT_RATE(16000)，不跟随 cfg->samp_rate：
	 *    ADC 在 32kHz 会走小数分频通路（见该宏的注释），而它在这里只为
	 *    "借电源"，没必要动共享时钟树。 */
	adc_cfg.adc_chl = AUD_ADC_CHL_L;
	adc_cfg.samp_rate = DMIC_ADC_PREINIT_RATE;
	adc_cfg.clk_src = cfg->clk_src;
	ret = bk_aud_adc_init(&adc_cfg);
	if (ret != BK_OK && ret != BK_FAIL) {
		/* BK_FAIL 一般表示 adc 已 init 过，不是致命错误 */
		csh_printf(csh, "dmic: adc pre-init failed: %d\r\n", ret);
		return ret;
	}

	/* 2. 配 DMIC 数字通路 */
	ret = bk_aud_dmic_init(cfg);
	if (ret != BK_OK) {
		csh_printf(csh, "dmic init failed: %d\r\n", ret);
		return ret;
	}

	/* 2.2 数字增益（`dmic gain [n]` 设置过才生效，否则保持 SDK 默认 0x2d）。
	 * 必须放在 bk_aud_adc_init() 之后：bk_aud_adc_set_gain() 内部带
	 * AUD_ADC_RETURN_ON_NOT_INIT 检查，且 ADC 初始化会把 gain 重置成
	 * adc_cfg.adc_gain（默认 0x2d）。 */
	if (s_dmic_gain <= 0x3f) {
		ret = bk_aud_adc_set_gain(s_dmic_gain);
		if ((csh != NULL) && (ret != BK_OK))
			csh_printf(csh, "dmic: set gain 0x%02X failed: %d\r\n",
			           (unsigned)s_dmic_gain, ret);
		ret = BK_OK;
	}

	/* 2.5 下面三步是历史排查中加的"补丁"，**默认关闭**（s_patch_en=false）。
	 *
	 * 关闭的实测依据（2026-11-12，同一块板同一颗麦，两轮 A/B）：
	 *
	 *   第一轮（确认硬件 OK）
	 *     `dmic ref`（官方流程，不做这三步、16kHz）
	 *         got 3200/3200 frames, fifo=0x00103C0C
	 *         L: rms=8499 peak=16384  R: rms=6605 peak=14285
	 *     `dmic`（打开这三步 + 32kHz）
	 *         fifo 恒 0x0080F000（empty），200 次轮询 changed=0
	 *
	 *   第二轮（把采样率单独摘出来）
	 *     `dmic ref 200 32000`（官方流程 + 32kHz，仍不做这三步）
	 *         got 6400/6400 frames, L: rms=6461  R: rms=4583
	 *     → 32kHz 也没问题，**唯一的差异就是这三步**。
	 *
	 * 由此定论：soft_reset / clk_gate / aud_cken 都不是 DMIC 出数的必要条件，
	 * 而把 `clk_control` 从官方可用态 0x1 改写成 0x0 反而让 FIFO 一个 word
	 * 都不进。寄存器名叫 soft_reset，实际更像某种 enable，不要去清它。
	 *
	 * 保留成开关是为了还能做单变量复核：
	 *   dmic patch on     # 打开三个补丁（会复现"没数"）
	 *   dmic patch off    # 关闭（默认）
	 */
	/* 三个补丁拆成独立位，默认只开 ADCKEN（见 PATCH_* 注释）。 */
	if (s_patch_mask & PATCH_ADCKEN)
		sys_drv_aud_clock_en(1);

	if (s_patch_mask & PATCH_SOFTRST) {
		/* 软复位脉冲：写 1 → 保持 1ms → 写 0（已实测有害，默认关） */
		aud_ll_set_clk_control_soft_reset(1);
		rtos_delay_milliseconds(1);
		aud_ll_set_clk_control_soft_reset(0);

		if (csh)
			csh_printf(csh, "  [patch] after clear: soft_reset=%u clk_ctrl=0x%08X\r\n",
			           (unsigned)aud_ll_get_clk_control_soft_reset(),
			           (unsigned)aud_ll_get_clk_control_value());
	}

	if (s_patch_mask & PATCH_CLKGATE) {
		/* clk_gate 语义反相（0=使能 apb clock gate），已实测无必要 */
		if (aud_ll_get_clk_control_clk_gate())
			aud_ll_set_clk_control_clk_gate(0);
	}

	/* 4. 到这里只完成"初始化 + 上电 + 引脚复用 + 时钟树"，**不启动采集**。
	 *
	 * 真正的 bk_aud_dmic_start() 交给 dmic_hw_start()（无 DMA 路径）或
	 * 录音任务（有 DMA 路径，必须排在 bk_dma_start() 之后）。
	 *
	 * 依据两个官方例程，DMIC 的启动都必须晚于 DMA：
	 *   bk_project_voice_record/main/aud_ns_recoder.c:626~633
	 *       bk_dma_start(dmic_dma_id);
	 *       bk_aud_dmic_start();
	 *       bk_aud_adc_start();
	 *   bk_avdk_smp/.../onboard_dual_dmic_mic_stream.c:476~487
	 *       bk_dma_start(dmic_dma_id);
	 *       bk_aud_dmic_start();
	 * 反过来（先 start DMIC 再 start DMA）在 REPEAT 双缓冲上会丢掉开局
	 * 若干个完成中断，表现为"录不到帧 / 一个中断都不来"。
	 */
	return BK_OK;
}

/* ---------------- ADC FIFO 排空 DMA（可选） ---------------- */

#define DMIC_ADC_DRAIN_WORDS  64
static dma_id_t s_adc_drain_dma = DMA_ID_MAX;
static uint32_t *s_adc_drain_buf;

static void dmic_adc_drain_stop(void)
{
	if (s_adc_drain_dma < DMA_ID_MAX) {
		bk_dma_stop(s_adc_drain_dma);
		bk_dma_deinit(s_adc_drain_dma);
		bk_dma_free(DMA_DEV_AUDIO_RX, s_adc_drain_dma);
		s_adc_drain_dma = DMA_ID_MAX;
	}

	if (s_adc_drain_buf != NULL) {
		os_free(s_adc_drain_buf);
		s_adc_drain_buf = NULL;
	}
}

/* 起一个 REPEAT DMA：源 = ADC FIFO（固定地址），目的 = 64 word 假缓冲，
 * 读出来就丢。ADC 继续正常出数（DMIC 需要的 line_enable 保持），但 FIFO
 * 不再堆积。 */
static bk_err_t dmic_adc_drain_start(bool force)
{
	dma_config_t cfg;
	uint32_t fifo = 0;
	bk_err_t ret;

	if ((!force && !s_adc_drain_en) || (s_adc_drain_dma < DMA_ID_MAX))
		return BK_OK;

	if (bk_aud_adc_get_fifo_addr(&fifo) != BK_OK)
		return BK_FAIL;

	if (s_adc_drain_buf == NULL) {
		s_adc_drain_buf = (uint32_t *)os_malloc(DMIC_ADC_DRAIN_WORDS * 4);
		if (s_adc_drain_buf == NULL)
			return BK_ERR_NO_MEM;
	}

	(void)bk_dma_driver_init();

	s_adc_drain_dma = bk_dma_alloc(DMA_DEV_AUDIO_RX);
	if ((s_adc_drain_dma < DMA_ID_0) || (s_adc_drain_dma >= DMA_ID_MAX)) {
		s_adc_drain_dma = DMA_ID_MAX;
		return BK_FAIL;
	}

	os_memset(&cfg, 0, sizeof(cfg));
	cfg.mode = DMA_WORK_MODE_REPEAT;
	cfg.chan_prio = 1;
	cfg.trans_type = DMA_TRANS_DEFAULT;

	cfg.src.dev = DMA_DEV_AUDIO_RX;
	cfg.src.width = DMA_DATA_WIDTH_32BITS;
	cfg.src.addr_inc_en = DMA_ADDR_INC_DISABLE;   /* FIFO 固定地址 */
	cfg.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
	cfg.src.start_addr = fifo;
	cfg.src.end_addr = fifo + 4;

	cfg.dst.dev = DMA_DEV_DTCM;
	cfg.dst.width = DMA_DATA_WIDTH_32BITS;
	cfg.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
	cfg.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
	cfg.dst.start_addr = (uint32_t)(uintptr_t)s_adc_drain_buf;
	cfg.dst.end_addr = (uint32_t)(uintptr_t)s_adc_drain_buf + DMIC_ADC_DRAIN_WORDS * 4;

	ret = bk_dma_init(s_adc_drain_dma, &cfg);
	if (ret != BK_OK)
		goto fail;

	bk_dma_set_transfer_len(s_adc_drain_dma, DMIC_ADC_DRAIN_WORDS * 4);
#if (CONFIG_SPE)
	bk_dma_set_dest_sec_attr(s_adc_drain_dma, DMA_ATTR_SEC);
	bk_dma_set_src_sec_attr(s_adc_drain_dma, DMA_ATTR_SEC);
#endif

	ret = bk_dma_start(s_adc_drain_dma);
	if (ret != BK_OK)
		goto fail;

	return BK_OK;

fail:
	dmic_adc_drain_stop();
	return BK_FAIL;
}

/* 初始化 + 立即启动（短采集用，无 DMA）。 */
static bk_err_t dmic_hw_start(chry_shell_t *csh, aud_dmic_config_t *cfg)
{
	bk_err_t ret = dmic_hw_init(csh, cfg);

	if (ret != BK_OK)
		return ret;

	ret = bk_aud_dmic_start();
	if (ret != BK_OK)
		csh_printf(csh, "dmic start failed: %d\r\n", ret);

	/* ADC start 是可开关的（`dmic cfg adc on|off`）：
	 *
	 *   实测（2026-11-12）：
	 *     - ADC 不 start → DMIC **完全不出数**（fifo 恒 0x00A0F000）；
	 *     - ADC start   → 能出数，但录音约 0.5s 后 DMIC 输出冻结成恒定值。
	 *
	 *   所以它既是"必需"又"有害"。默认保持 ON（至少能出数），用
	 *   `dmic cfg adc off` 复现"完全没数"，用 `dmic cfg adc on` 回到出数态。
	 *
	 *   怀疑的机理解释：bk_aud_adc_start() 除了 adc_enable 还会置
	 *   audio_config.line_enable(0x30[5])，DMIC 的 FIFO/PDM 通路可能需要
	 *   这一位；而 ADC FIFO 没人排空，约 0.5s 后被灌满，连带把 DMIC 的
	 *   CIC 输出拖停（官方录音卡是 ADC 另配 DMA 排空的，见
	 *   aud_ns_recoder.c:614/632 的 amic_dma)。
	 *   下一步验证：给 ADC 配一个只丢数据的排空 DMA，看冻结是否消失。
	 */
	if (s_adc_start_en) {
		ret = bk_aud_adc_start();
		if ((csh != NULL) && (ret != BK_OK))
			csh_printf(csh, "dmic: adc start failed: %d\r\n", ret);
		ret = BK_OK;

		/* ADC FIFO 排空（`dmic cfg drain on` 才生效），见函数注释 */
		if (dmic_adc_drain_start(false) != BK_OK) {
			if (csh != NULL)
				csh_printf(csh, "dmic: adc drain dma start failed\r\n");
		}
	}

	if (csh)
		csh_printf(csh, "  [trace] after start: soft_reset=%u clk_ctrl=0x%08X\r\n",
		           (unsigned)aud_ll_get_clk_control_soft_reset(),
		           (unsigned)aud_ll_get_clk_control_value());

	return ret;
}

/* 停止 + 释放 DMIC，并把 dmic_hw_start() 里借来开总电源的 ADC 一起释放。
 *
 * 之所以要配对释放：dmic_hw_start() 用 bk_aud_adc_init() 打开 audio 模块
 * 总电源（sys_drv_aud_aud_en(1)），这不属于 DMIC 的引用计数。
 * 只调 bk_aud_dmic_deinit() 的话 ADC 那边仍认为自己在占用，总电源不会关，
 * 下次再 init 还可能撞上 "aud adc is init already"。
 *
 * 顺序：先停 DMIC，再 deinit DMIC，最后 deinit ADC（ADC 最后关，因为它
 * 提供的是公共电源/时钟，要保证 DMIC 先干净退出）。 */
static void dmic_hw_stop(void)
{
	dmic_adc_drain_stop();

	/* 官方例程的停法是 ADC/DMIC 各自先 stop 再 deinit，这里照做：
	 * bk_aud_adc_stop() 只清 audio_config.adc_enable，不动公共电源；
	 * bk_aud_adc_deinit() 才会级联 bk_aud_driver_deinit() 把 AUDP 域
	 * 投票释放掉，所以 ADC 必须最后关。 */
	bk_aud_adc_stop();
	bk_aud_dmic_stop();
	bk_aud_dmic_deinit();
	bk_aud_adc_deinit();

	/* 配对释放 dmic_hw_init() 里投的 CPU 频率票 */
	if (s_freq_vote_en)
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_DEFAULT);
}

static void dmic_print_stat(chry_shell_t *csh, const char *name,
                            int64_t sum_sq, int32_t peak, int n)
{
	uint32_t rms = isqrt64((uint64_t)(sum_sq / (n > 0 ? n : 1)));
	uint32_t pct10 = (uint32_t)((uint64_t)rms * 1000 / 32768);

	csh_printf(csh, "  %s: rms=%5u (%u.%u%% FS)  peak=%5d\r\n",
	           name, rms, pct10 / 10, pct10 % 10, peak);
}

/* ---------------- 自检 ---------------- */

/* 读 GPIO 的 "第二功能模式" 编号。
 *
 * 不能用 gpio_ll_get_gpio_perial_mode()：它内部用的是 gpio_ll.h 里的
 * 文件级 static 指针 gpio_system_gpio_func_mode，只有调过 gpio_ll_init()
 * 的 .c 才有值。本文件没调过（那个副本是 NULL），解引用就是往 0 地址
 * 写/读 → Data Fault → 芯片复位（表现是打完几行日志又重印 banner）。
 *
 * 所以这里直接按寄存器算地址，语义和 ll 层一致：
 *   gpio_sys_num[SOC_GPIO_SYSTEM_GROUP_NUM=7]，每项一个 32bit 寄存器，
 *   每 8 个 pad 共用一个寄存器，每个 pad 占 4bit。
 *
 * 【坑】GPIO_LL_SYSTEM_REG_BASE 本身已经是 (SOC_SYS_REG_BASE + 0x30*4)，
 * 即 gpio_sys_num[0] 的地址。所以这里的偏移必须从 0 起算：
 *   pad  0~31 : gpio_sys_num[0..3] → base + (pad/8)*4
 *   pad 32~55 : gpio_sys_num[4..6] → base + (4 + (pad-32)/8)*4
 * 之前误写成 base + (0x30 + pad/8)*4，多偏移了 0x30*4 字节，
 * 读写全落在不相干的寄存器上，所以写 mode=3 后回读永远还是 0。
 */
static uint32_t dmic_gpio_func_mode(int pad)
{
	uint32_t base = GPIO_LL_SYSTEM_REG_BASE;
	uint32_t idx, shift;

	if (pad < 32) {
		idx = pad / 8;
		shift = (pad % 8) * 4;
	} else {
		idx = 4 + (pad - 32) / 8;
		shift = ((pad - 32) % 8) * 4;
	}

	return (REG_READ(base + idx * 4) >> shift) & 0xF;
}

/* 写 GPIO 的 "第二功能模式" 编号（读-改-写，不动同寄存器里别的 pad）。
 *
 * gpio_map.h 里 GPIO_8 那行：{BT_ACTIVE, PWM2, I2S1_DIN, DMIC1_CLK, ...}
 * → DMIC1_CLK 索引 3；GPIO_9 那行 DMIC1_DAT 同样索引 3。
 * 地址算法见上面 dmic_gpio_func_mode() 的注释。
 */
static void dmic_gpio_func_mode_set(int pad, uint32_t mode)
{
	uint32_t base = GPIO_LL_SYSTEM_REG_BASE;
	uint32_t idx, shift, addr, v;

	if (pad < 32) {
		idx = pad / 8;
		shift = (pad % 8) * 4;
	} else {
		idx = 4 + (pad - 32) / 8;
		shift = ((pad - 32) % 8) * 4;
	}

	addr = base + idx * 4;
	v = REG_READ(addr);
	v &= ~(0xFu << shift);
	v |= (mode & 0xFu) << shift;
	REG_WRITE(addr, v);
}

/* 把 DMIC 相关的寄存器/引脚/电源全打一遍。
 * 排查 "no data" 时看三件事：
 *   1) dmic_enable=1（不然数字通路根本没开）
 *   2) apll_sel=1 且 dig_mic_div/cic_sel 跟采样率对得上
 *      （32000 要求 apll_sel=1, div=11, cic=2；16000 则 div=23）
 *   3) P8/P9 的 func_en=1 且 mode 指向 DMIC
 */
static void dmic_dump_regs(chry_shell_t *csh)
{
	gpio_hw_t *hw = (gpio_hw_t *)GPIO_LL_REG_BASE;
	uint32_t st;
	int i;

	csh_printf(csh, "dmic regs: apll_sel=%u dmic_enable=%u cic_sel=%u dig_mic_div=%u\r\n",
	           (unsigned)aud_ll_get_audio_config_apll_sel(),
	           (unsigned)aud_ll_get_audio_config_dmic_enable(),
	           (unsigned)aud_ll_get_audio_config_dmic_cic_sel(),
	           (unsigned)aud_ll_get_audio_config_dig_mic_div());
	csh_printf(csh, "           wr_threshold=%u int_en=%u\r\n",
	           (unsigned)aud_ll_get_fifo_config_dmic_wr_threshold(),
	           (unsigned)aud_ll_get_fifo_config_dmic_int_en());

	/* AUD 模块的时钟控制寄存器（REG_0x02）。
	 *
	 * 【已修正的旧结论】这里原来写着"soft_reset 必须由软件清 0，一直保持 1
	 * 整个音频模块就停在复位态"。2026-11-12 的实测推翻了它：
	 *   - 官方录音卡流程全程不动这个寄存器（clk_ctrl 保持 0x1），`dmic ref`
	 *     16kHz 下 got 3200/3200，左右声道都正常；
	 *   - 主动改写成 0x0 之后 fifo 恒 empty，一个 word 都读不到。
	 * 所以 **0x1 才是可用态**，不要再去清它。判断 DMIC 通不通请看
	 * dig_mic_div / dmic_enable / fifo 是否真的进数，而不是看这一位。 */
	csh_printf(csh, "clk_ctrl=0x%08X soft_reset=%u clk_gate=%u (正常态即 0x1，勿清)\r\n",
	           (unsigned)aud_ll_get_clk_control_value(),
	           (unsigned)aud_ll_get_clk_control_soft_reset(),
	           (unsigned)aud_ll_get_clk_control_clk_gate());

	/* APLL 到底有没有起振？
	 *
	 * pwdaudpll 在 SOC_SYS_REG_BASE + (0x45<<2) 的 bit13，语义**反相**：
	 *   1 = power down（关）, 0 = 上电（开）
	 * sys_hal_apll_en(1) 内部就是写 sys_ll_set_ana_reg5_pwdaudpll(0)。
	 * 若这里读到 1，说明 APLL 被关掉了，dig_mic_div 配得再对也没有源时钟，
	 * P8 自然恒低 —— 这是 soft_reset 之后最可能的下一个卡点。
	 */
	{
		uint32_t pwdaudpll = sys_ll_get_ana_reg5_pwdaudpll();
		uint32_t ana5 = REG_READ(SOC_SYS_REG_BASE + (0x45 << 2));

		csh_printf(csh, "apll: ana_reg5=0x%08X pwdaudpll=%u (%s)\r\n",
		           (unsigned)ana5, (unsigned)pwdaudpll,
		           pwdaudpll ? "POWER DOWN -> APLL 没开！" : "on, APLL 已上电");
	}

	/* 时钟树上的三个关键开关，缺一个 DMIC 都拿不到源时钟：
	 *   cksel_aud    = 1  → 音频子系统时钟源选 APLL（0=XTAL）
	 *   reserved_13  = 1  → dmic 分频使能（sys_drv_dmic_clk_div_set(1)）
	 *                     名字叫 reserved，实际是 DMIC 时钟门
	 *   aud_cken     = 1  → 音频 APB 时钟使能
	 */
	csh_printf(csh, "clk tree: cksel_aud=%u dmic_div_en(res13)=%u aud_cken=%u\r\n",
	           (unsigned)sys_ll_get_cpu_clk_div_mode1_cksel_aud(),
	           (unsigned)sys_ll_get_cpu_clk_div_mode2_reserved_13_13(),
	           (unsigned)sys_ll_get_cpu_device_clk_enable_aud_cken());

	/* P8 = DMIC1_CLK, P9 = DMIC1_DAT。
	 * 复用表里 GPIO_8/GPIO_9 的第 4 项就是 DMIC1_CLK/DMIC1_DAT → mode=3。 */
	{
		uint32_t mc = dmic_gpio_func_mode(BOARD_DMIC_CLK_PIN);
		uint32_t md = dmic_gpio_func_mode(BOARD_DMIC_DATA_PIN);

		csh_printf(csh, "gpio: P%d(CLK) func_en=%u mode=%u %s\r\n",
		           (int)BOARD_DMIC_CLK_PIN,
		           (unsigned)hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_2_func_en,
		           (unsigned)mc,
		           (mc == DMIC_GPIO_FUNC_MODE) ? "ok" : "BAD(want 3)");
		csh_printf(csh, "      P%d(DAT) func_en=%u mode=%u %s\r\n",
		           (int)BOARD_DMIC_DATA_PIN,
		           (unsigned)hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_2_func_en,
		           (unsigned)md,
		           (md == DMIC_GPIO_FUNC_MODE) ? "ok" : "BAD(want 3)");
	}
	csh_printf(csh, "      cfg P%d=0x%08X P%d=0x%08X\r\n",
	           (int)BOARD_DMIC_CLK_PIN,
	           (unsigned)hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.v,
	           (int)BOARD_DMIC_DATA_PIN,
	           (unsigned)hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.v);

	/* 注意：bsp_power_get_pad() 虽然名字叫 get_pad，返回的其实是该使能脚
	 * 的输入电平（0/1），不是 pad 号。pad 号直接用板级宏打。 */
	csh_printf(csh, "power: MIC 1V8 rail (P%d) = %s, level=%d\r\n",
	           (int)BOARD_PWR_MIC_EN_PIN,
	           bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8) ? "ON" : "OFF",
	           (int)bsp_power_get_pad(BSP_PWR_RAIL_MIC_1V8));

	/* FIFO status 连续采 8 次，看有没有在动 */
	csh_printf(csh, "fifo status x8:");
	for (i = 0; i < 8; i++) {
		bk_aud_dmic_get_status(&st);
		csh_printf(csh, " %06X", (unsigned)(st & 0x00FFFFFF));
		rtos_delay_milliseconds(5);
	}
	csh_printf(csh, "\r\n");
	csh_printf(csh, "        bit20/21/22/23 = nearfull/nearempty/full/empty\r\n");

	/* 直接读 FIFO 端口：FIFO 空的时候读出来一般固定不变；
	 * 如果 8 次读到的值在变，说明硬件在出数，只是状态位/中断没配上 */
	{
		uint32_t w0 = 0, w1 = 0;

		bk_aud_dmic_get_fifo_data(&w0);
		rtos_delay_milliseconds(10);
		bk_aud_dmic_get_fifo_data(&w1);
		csh_printf(csh, "fifo port 0x%08X -> 0x%08X %s\r\n",
		           (unsigned)w0, (unsigned)w1,
		           (w0 != w1) ? "(changing, hw is producing data)" : "(stuck)");
	}

	/* FIFO 端口密集轮询：绕开 DMA / 中断，直接看 DMIC 有没有在生产数据。
	 *
	 * 为什么还要这一步：DMA 那条路上，只要 finish 中断不来，我们就无从
	 * 知道是「没数据」还是「有数据但中断没配好」。而直接连续读
	 * AUD_DMIC_FPORT（0x4780004c）可以完全绕开 DMA：
	 *   读到的值一直在变 → DMIC 在出数，问题在 DMA/中断侧
	 *   读到的值恒为 0    → DMIC 确实没在转换，问题在 DMIC/时钟/引脚侧
	 * 读之前先看 EMPTY 位；非空才读，避免把「空读」误当成数据。
	 */
	{
		uint32_t vals[8] = {0};
		int nread = 0, nchanged = 0, tries;
		uint32_t prev = 0;

		for (tries = 0; tries < 200 && nread < 8; tries++) {
			bk_aud_dmic_get_status(&st);
			if (st & AUD_DMIC_FIFO_EMPTY_MASK) {
				rtos_delay_milliseconds(1);
				continue;
			}
			bk_aud_dmic_get_fifo_data(&vals[nread]);
			if (nread > 0 && vals[nread] != prev)
				nchanged++;
			prev = vals[nread];
			nread++;
		}

		csh_printf(csh, "fifo poll: got %d words in %d tries, changed=%d\r\n",
		           nread, tries, nchanged);
		if (nread > 0) {
			int k;

			csh_printf(csh, "  data:");
			for (k = 0; k < nread; k++)
				csh_printf(csh, " %08X", (unsigned)vals[k]);
			csh_printf(csh, "\r\n");
		}
		if (nread == 0)
			csh_printf(csh, "  -> 200 次尝试全空：DMIC 根本没在转换数据\r\n");
		else if (nchanged == 0)
			csh_printf(csh, "  -> 有数但恒定不变：时钟可能在跑但数据线没信号\r\n");
		else
			csh_printf(csh, "  -> 数据在变化：DMIC 正常工作，问题在 DAC/DMA 侧\r\n");
	}

	/* P8(CLK) 上到底有没有时钟在翻转？这是判断"麦有没有被驱动"的直接证据。
	 *
	 * 要点三条：
	 *  1) gpio_input_monitor 是**使能位**，置 1 后 pad 实际电平才映射到
	 *     cfg.gpio_input（bit0）；读电平必须读 gpio_input，不能读 monitor 本身。
	 *  2) 还必须置 gpio_input_en=1，否则输入缓冲是关的，gpio_input 读到的是
	 *     恒定值（实测就是恒 2000/2000，纯假象）。
	 *  3) 采样期间临时把 gpio_2_func_en 关掉、input_en 打开，采完再恢复，
	 *     避免破坏 DMIC 自己的第二功能配置。
	 *
	 * DMIC CLK 是 2.048MHz（32000Hz 时 24576/(11+1) kHz），远快于 CPU 读
	 * 循环，所以只要时钟活着，多次采样里必然既有 0 也有 1。
	 */
	{
		uint32_t clk_pin = BOARD_DMIC_CLK_PIN;
		uint32_t dat_pin = BOARD_DMIC_DATA_PIN;
		uint32_t clk_save = hw->gpio_num[clk_pin].cfg.v;
		uint32_t dat_save = hw->gpio_num[dat_pin].cfg.v;
		int clk_hi = 0, dat_hi = 0, k;

		/* 【测量方法修正，很重要】
		 * 旧版本这里先把 gpio_2_func_en 清 0 再读 gpio_input，那不是
		 * "测引脚电平"，而是"把外设从引脚上摘下来测"：func_en=0 之后
		 * DMIC 已经不再驱动这根线，pad 悬空、无上下拉，读出来恒 0 是
		 * 必然结果，跟"时钟有没有输出"毫无关系。
		 * 之前所有"P8 恒低 → CLK 没出时钟"的结论都建立在这个无效测量上。
		 *
		 * 正确做法：**保持 func_en=1**，只把 GPIO 输出关掉、打开输入缓冲
		 * 和 input monitor，并关掉内部上下拉。此时 gpio_input 反映的是
		 * pad 上的真实电平（外设驱动值）。
		 */
		hw->gpio_num[clk_pin].cfg.gpio_output_en = 0;
		hw->gpio_num[clk_pin].cfg.gpio_input_en = 1;
		hw->gpio_num[clk_pin].cfg.gpio_input_monitor = 1;
		hw->gpio_num[clk_pin].cfg.gpio_pull_mode_en = 0;
		hw->gpio_num[dat_pin].cfg.gpio_output_en = 0;
		hw->gpio_num[dat_pin].cfg.gpio_input_en = 1;
		hw->gpio_num[dat_pin].cfg.gpio_input_monitor = 1;
		hw->gpio_num[dat_pin].cfg.gpio_pull_mode_en = 0;

		for (k = 0; k < 4000; k++) {
			if (hw->gpio_num[clk_pin].cfg.gpio_input)
				clk_hi++;
			if (hw->gpio_num[dat_pin].cfg.gpio_input)
				dat_hi++;
		}

		csh_printf(csh, "pad sample(keep func_en) x4000: P%d(CLK) hi=%d  P%d(DAT) hi=%d\r\n",
		           (int)clk_pin, clk_hi, (int)dat_pin, dat_hi);
		if (clk_hi == 0)
			csh_printf(csh, "      -> P%d 恒低：PDM 时钟确实没在翻转\r\n",
			           (int)clk_pin);
		else if (clk_hi == 4000)
			csh_printf(csh, "      -> P%d 恒高：时钟卡死在高电平\r\n",
			           (int)clk_pin);
		else
			csh_printf(csh, "      -> P%d 电平有变化（%d/4000 为高）：CLK 在翻转，PDM 时钟正常\r\n",
			           (int)clk_pin, clk_hi);

		hw->gpio_num[clk_pin].cfg.v = clk_save;
		hw->gpio_num[dat_pin].cfg.v = dat_save;
	}

	/* ---- 引脚外部连接探测（上拉/下拉法）----
	 *
	 * 【为什么不用"输出再回读"】
	 * gpio_struct.h:28 明确写 gpio_input 是 **R（只读）**，且
	 * gpio_input_monitor 的注释是 "view gpio input values"。
	 * 一旦 gpio_output_en=1，gpio_input 反映的是内部输出锁存值，
	 * 不是 pad 真实电平 —— 所以之前 set hi->read 1 / set lo->read 1
	 * 两个都读到 1 是**必然的**，跟焊接毫无关系，那个测试本身无效。
	 *
	 * 【正确做法】把 pad 切成纯输入（output_en=0），用内部上拉/下拉
	 * 各测一次电平：
	 *   上拉读到高、下拉读到低  → 引脚是"软"的，外部没强驱动
	 *                            （悬空 或 麦只做高阻输入）
	 *   两次读数都被拉向同一侧 → 外部有强驱动源
	 * 对 DMIC_CLK 这个**芯片输出**脚来说，正常情况应当读到"软"，
	 * 因为对外它是输出，输入侧本来就该是悬空/高阻。所以这个测试
	 * 主要用于确认 pad 没被焊到电源/地上短路。
	 */
	{
		uint32_t clk_pin = BOARD_DMIC_CLK_PIN;
		uint32_t save = hw->gpio_num[clk_pin].cfg.v;
		int pu_rd, pd_rd;

		/* 纯输入模式 */
		hw->gpio_num[clk_pin].cfg.gpio_2_func_en = 0;
		hw->gpio_num[clk_pin].cfg.gpio_output_en = 0;
		hw->gpio_num[clk_pin].cfg.gpio_input_en = 1;
		hw->gpio_num[clk_pin].cfg.gpio_input_monitor = 1;

		/* 内部上拉 */
		hw->gpio_num[clk_pin].cfg.gpio_pull_mode_en = 1;
		hw->gpio_num[clk_pin].cfg.gpio_pull_mode = 1;
		rtos_delay_milliseconds(2);
		pu_rd = hw->gpio_num[clk_pin].cfg.gpio_input ? 1 : 0;

		/* 内部下拉 */
		hw->gpio_num[clk_pin].cfg.gpio_pull_mode = 0;
		rtos_delay_milliseconds(2);
		pd_rd = hw->gpio_num[clk_pin].cfg.gpio_input ? 1 : 0;

		csh_printf(csh, "pad probe: P%d pullup->%d pulldown->%d\r\n",
		           (int)clk_pin, pu_rd, pd_rd);
		if (pu_rd == 1 && pd_rd == 0)
			csh_printf(csh, "      -> 引脚可被拉动，未见对电源/地短路\r\n");
		else if (pu_rd == 1 && pd_rd == 1)
			csh_printf(csh, "      -> 恒高，疑似被外部拉到 VCC\r\n");
		else
			csh_printf(csh, "      -> 恒低，疑似被外部拉到 GND\r\n");

		/* 恢复原配置 */
		hw->gpio_num[clk_pin].cfg.v = save;
	}
}

/* dmic dbg —— 上电后开 DMIC，打自检，不采数据
 * clk_src 传 AUD_CLK_APLL 走默认路径；传 AUD_CLK_XTAL 做时钟源对照实验。 */
static int dmic_dbg_ex(chry_shell_t *csh, aud_clk_t clk_src)
{
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
	bool mic_was_on;
	bk_err_t ret;

	mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	if (!mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	cfg.samp_rate = s_dmic_rate;
	/* 与官方 DEFAULT_AUD_DMIC_CONFIG() / 单麦录音卡 aud_ns_recoder.c:428
	 * 一致，用 AUD_DMIC_CHL_L（只有双麦 nch==2 才用 LR）。
	 * 注：aud_dmic_driver.c 里并没有读取 dmic_chl 这个字段，声道数实际
	 * 由硬件 SEL 引脚决定，这里保持官方默认、不做自创假设。 */
	cfg.dmic_chl = AUD_DMIC_CHL_L;
	cfg.clk_src = clk_src;

	csh_printf(csh, "dmic dbg: clk_src=%s\r\n",
	           (clk_src == AUD_CLK_APLL) ? "APLL" : "XTAL");

	ret = dmic_hw_start(csh, &cfg);
	if (ret != BK_OK) {
		/* init 失败：把可能已经开起来的 audio 总电源关掉 */
		bk_aud_adc_deinit();
		if (!mic_was_on)
			bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);
		csh_printf(csh, "dmic: hw start failed: %d\r\n", ret);
		return -1;
	}

	rtos_delay_milliseconds(20);   /* 等 CIC 滤波器建立 */
	dmic_dump_regs(csh);

	dmic_hw_stop();
	if (!mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);

	return 0;
}

static int dmic_dbg(chry_shell_t *csh)
{
	return dmic_dbg_ex(csh, AUD_CLK_APLL);
}

/* ---------------- 短采集（电平测试） ---------------- */

/* 采一次（指定时钟源）；APLL 没数再试 XTAL 的逻辑在 dmic_capture() 里 */
/* dmic mon [ms] —— 只启动 DMIC 并等待，不做任何采集。
 *
 * 全部走公开驱动 API（bk_aud_* / bsp_power_* / rtos_*），不直接读写
 * LL 寄存器或 GPIO 结构体 —— 与官方例程的用法保持一致。
 *
 * 用途：配合逻辑分析仪长捕获，确认启动期间 CLK/DAT 是否全程存活，
 * 且期间没有本工程自己的 FIFO 读取、GPIO 改动等动作干扰硬件。
 */
static int dmic_mon_cmd(chry_shell_t *csh, int ms)
{
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
	uint32_t status = 0;
	bool mic_was_on;
	bk_err_t ret;

	/* 参数单位是**毫秒**。以前设了 100ms 下限，`dmic mon 10` 会被悄悄
	 * 拉长成 100ms，日志和输入对不上。这里如实执行，只留 1ms 保护。
	 * 想观察"跑一会儿才坏"要用 `dmic mon 3000`（3 秒）。 */
	if (ms < 1)
		ms = 1;
	if (ms > 30000)
		ms = 30000;

	/* 与 dmic_capture_once() 保持一致：先开 MIC 1V8 rail 再启动。
	 * 之前 mon 漏了这一步，日志里 `mic_pwr=0` 就是这么来的。
	 * 板上麦实测由 3V3 直供、1V8 只是兜底，但两条路径不该有差异。 */
	mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	if (!mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	cfg.samp_rate = s_dmic_rate;
	/* 用官方默认值 AUD_DMIC_CHL_L（DEFAULT_AUD_DMIC_CONFIG 与单麦录音卡
	 * aud_ns_recoder.c:428 都是 L）。只有双麦才用 LR。 */
	cfg.clk_src = AUD_CLK_APLL;

	ret = dmic_hw_start(csh, &cfg);
	if (ret != BK_OK) {
		csh_printf(csh, "dmic mon: start failed: %d\r\n", ret);
		if (!mic_was_on)
			bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);
		return -1;
	}

	(void)bk_aud_dmic_get_status(&status);
	csh_printf(csh, "dmic mon: %uHz, %dms, mic_pwr=%d, fifo@start=0x%06X\r\n",
	           (unsigned)s_dmic_rate, ms,
	           (int)bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8),
	           (unsigned)(status & 0x00FFFFFF));

	rtos_delay_milliseconds((uint32_t)ms);

	(void)bk_aud_dmic_get_status(&status);
	csh_printf(csh, "dmic mon: fifo@end=0x%06X\r\n",
	           (unsigned)(status & 0x00FFFFFF));

	dmic_hw_stop();
	if (!mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);
	csh_printf(csh, "dmic mon: done\r\n");
	return 0;
}

#if CONFIG_ADK_ONBOARD_DUAL_DMIC_MIC_STREAM
/* 录音到 SD 的两个工具（定义在下面「录音到 SD 卡」一节）：
 * 组件路径复用它们写 WAV，避免再复制一份 FatFs 打开/挂载逻辑。 */
static int dmic_rec_sd_mount(chry_shell_t *csh);
static void wav_build_header(uint8_t hdr[44], uint32_t rate, uint32_t data_len, uint8_t nch);
/* 把用户给的文件名规范成 "1:/xxx.wav"（定义在「录音到 SD 卡」一节） */
static int dmic_rec_build_path(const char *arg, char *out, size_t outlen);

/* dmic adk [ms] [path] —— 直接走官方 onboard_dual_dmic_mic_stream 组件。
 * path 非 NULL 时把 PCM 落成 WAV 写到 SD 卡（`dmic adk rec <sec> <file>`）。
 *
 * 与手写 cmd_dmic 路径的区别：组件内部自带
 *   - aud_adc_dma_config()：给 ADC FIFO 配独立排空 DMA（等价于 dmic cfg drain on）
 *   - aud_dmic_dma_config()：官方多帧 ring buffer + DMA_DEV_AUD_DMIC + SEC 属性
 *   - bk_aud_adc_init/dmic_init/start 的完整启动顺序
 * 所以不需要 dmic cfg drain、dmic patch、单帧窗口、ISR 快照、w_pick 猜半字。
 *
 * dual_dmic_sgl_out=1 让组件内部把交错的 L/R 只保留 L（ptr[i]=ptr[2*i]），
 * 对应本板 BOARD_DMIC_LEFT_IS_LOW=1，避免自己判断高/低 16bit。
 *
 * stereo=1 时改成 dual_dmic_sgl_out=0：组件把 DMA 读到的 L/R 交织原样
 * 输出（每个 32bit word = 低16bit 左 + 高16bit 右），out_block_size 翻倍。
 * 用途：本板两颗麦物理位置不同，声学通道是否通畅只能靠 L/R 对比判断
 * （一颗被洗板水/助焊剂堵住时，单声道版本把 R 丢掉就看不出来了）。
 *
 * 数据从 audio_element 的 output port（ring buffer）读出，全部用公开 API。
 */
static int dmic_adk_cmd(chry_shell_t *csh, int ms, const char *path, bool stereo, bool no_pwr)
{
	onboard_dual_dmic_mic_stream_cfg_t cfg = DEFAULT_ONBOARD_DUAL_DMIC_STREAM_CONFIG();
	audio_element_handle_t el = NULL;
	audio_port_handle_t port = NULL;
	uint32_t frame_bytes, block_bytes, t_start, t_end, seg_end, t_warm, t_dump;
	uint32_t seg_got = 0;
	/* 以下统计量都按声道分开累计：[0]=L 或 mono，[1]=R（仅 stereo 有效）。
	 * 去直流后的能量统计：rms/pk/hf 全部基于 AC 分量算，
	 * 否则一个持续直流偏置就能把 rms 顶到与语音同量级。 */
	uint32_t seg_n[2] = {0}, seg_zc[2] = {0}, nsamp[2] = {0};
	/* flat = 去直流后 |y| <= 1 的样本占比。真麦克风（哪怕声孔被封死）
	 * 仍有自噪声，flat 通常 <10%；某一路如果是"数字死路"（没在驱动
	 * 数据线、或整个半字恒为常数），flat 会接近 100%。 */
	uint32_t flat[2] = {0};
	int64_t ac_sq[2] = {0}, hf_sq[2] = {0};
	int64_t seg_ac_sq[2] = {0}, seg_hf_sq[2] = {0};
	int64_t sum_dc[2] = {0}, seg_dc_sum[2] = {0};
	int32_t ac_pk[2] = {0}, seg_ac_pk[2] = {0};
	int32_t prev[2] = {0};
	int32_t dc_prev[2] = {0}, dc_y[2] = {0}, hf_prev[2] = {0}, hf_y[2] = {0};
	uint8_t dumped = 0;
	int16_t *buf = NULL;
	bool has_prev[2] = {false};
	/* 「到底有没有出声」的硬证据：按 100ms 窗统计每路峰值，只保留最响
	 * 的 3 个窗（含出现时刻）。5 秒里拍一次手就必然上榜，
	 * 不再依赖"我好像喊了"这种只能靠回忆的判断。 */
	uint32_t win_start = 0, win_pk[2] = {0};
	uint32_t burst_ms[2][3] = {{0}};
	int32_t burst_pk[2][3] = {{0}};
	int burst_cnt[2] = {0};
	int i, c, nch, ret = -1;
	bool mic_was_on;
	bool adc_started = false;
	/* 录音相关（path == NULL 时全部不启用） */
	static char s_adk_path[DMIC_REC_PATH_LEN];
	FIL fil;
	bool f_opened = false;
	bool write_ok = true;
	uint32_t data_bytes = 0;
	UINT bw = 0;

	nch = stereo ? 2 : 1;

	if (ms < 200)
		ms = 200;
	if (ms > (path ? 30000 : 10000))
		ms = path ? 30000 : 10000;

	if (path != NULL) {
		/* FatFs 要求卷前缀，用户习惯性打成 `/adk_lr.wav` 时要自动补成
		 * `1:/adk_lr.wav`，否则 f_open 返回 FR_INVALID_NAME，
		 * 表现就是一句 "open /adk_lr.wav failed"（2026-11-12 实测踩到）。 */
		if (dmic_rec_build_path(path, s_adk_path, sizeof(s_adk_path)) != 0) {
			/* 这里还没申请任何资源（电源/频率票/文件都没动），直接返回。 */
			csh_printf(csh, "dmic adk: 文件名非法或过长：%s\r\n", path);
			return -1;
		}
	}

	/* MIC 1V8（P48）供电。组件是通用件，不会碰板级电源，
	 * 必须自己开，否则 DMIC 没有供电，采集出来全是 0。
	 * `np`（no power）用于「断电反证」：不改变 LDO 现有状态直接采，
	 * 配合 `dmic pwr off` 就能判断数据到底是不是这颗麦产生的。 */
	mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	if (!no_pwr && !mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}
	if (no_pwr)
		csh_printf(csh, "dmic adk: 跳过上电，MIC 1V8 现在 = %s（断电反证模式）\r\n",
		           mic_was_on ? "ON" : "OFF");

	/* 音频期间把 CPU 频率钉到 480MHz，与手写路径一致（组件本身不投票）。 */
	if (s_freq_vote_en)
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_480M);

	/* 录音模式：先挂卡、开文件、写 44 字节占位头。
	 * 放在组件初始化之前做，避免 FatFs 首次挂载的几百毫秒把 DMA 时序打乱，
	 * 也让「卡没插」这种错误在开始采集之前就暴露。
	 * 失败时 el 还是 NULL，必须直接跳 out_power（out_deinit 会 deinit el）。 */
	if (path != NULL) {
		uint8_t hdr[44];

		if (dmic_rec_sd_mount(csh) != 0)
			goto out_power;

		if (f_open(&fil, s_adk_path, FA_CREATE_ALWAYS | FA_WRITE) != FR_OK) {
			csh_printf(csh, "dmic adk: open %s failed\r\n", s_adk_path);
			goto out_power;
		}
		f_opened = true;

		wav_build_header(hdr, s_dmic_rate, 0, (uint8_t)nch);
		if ((f_write(&fil, hdr, 44, &bw) != FR_OK) || (bw != 44)) {
			csh_printf(csh, "dmic adk: write wav header failed\r\n");
			goto out_power;
		}
	}

	/* 20ms 一帧、单声道 16bit；block_bytes = 一次从端口读出的块。
	 * 组件的 _read 在 dual_dmic 下恒定返回 frame_size*2（L/R 交织），
	 * stereo（sgl_out=0）时原样输出，mono（sgl_out=1）时组件折半。 */
	frame_bytes = (s_dmic_rate / 1000) * 20 * 2;
	block_bytes = frame_bytes * (uint32_t)nch;

	cfg.adc_cfg.sample_rate = s_dmic_rate;
	/* chl_num 只影响 ADC 那侧的初始化，DMIC 通路固定 AUD_ADC_CHL_LR；
	 * 保持 1 不变，避免动已经验证过的 ADC 预初始化路径。 */
	cfg.adc_cfg.chl_num = 1;
	cfg.adc_cfg.bits = 16;
	/* `dmic gain <v>` 设置过就用它；否则用组件默认 0dB（0x2d）。
	 * 范围 0x00~0x3f = -45dB ~ +18dB，测麦克风灵敏度时用 `dmic gain 0x3f`。 */
	cfg.adc_cfg.dig_gain = (s_dmic_gain <= 0x3f) ? (int32_t)s_dmic_gain : 0x2d;
	cfg.frame_size = frame_bytes;
	cfg.out_block_size = block_bytes;
	/* 录音时把输出 ring buffer 加倍：f_write 是同步的，SD 偶发几十毫秒
	 * 的写延迟需要靠缓冲吸收，否则 DMA 端会被拖出 overrun。 */
	cfg.out_block_num = path ? 8 : 4;
	cfg.multi_out_port_num = 0;
	cfg.task_stack = 4096;
	cfg.task_core = 1;
	cfg.task_prio = 3;
	cfg.ref_mode = 0;
	cfg.dual_dmic = 1;
	cfg.dual_dmic_sgl_out = stereo ? 0 : 1;
	/* 组件自带的移位是裸 `<<=`、无饱和，shift>=5 会按位回绕成失真。
	 * 这里置 0，改在下面的采样循环里做「带直流阻断 + 带饱和」的增益。 */
	cfg.dmic_data_shift = 0;

	el = onboard_dual_dmic_mic_stream_init(&cfg);
	if (el == NULL) {
		csh_printf(csh, "dmic adk: init failed（其它 dmic 命令是否还在占用 audio 模块？）\r\n");
		goto out_power;
	}

	port = audio_element_get_output_port(el);
	if (port == NULL) {
		/* 组件是 PORT_TYPE_RB 输出，且 cfg.write == NULL，所以
		 * audio_element_init() 不会自己建输出端口（只有 PORT_TYPE_CB
		 * 才在 init 里建）。必须像 player_service_example 那样，
		 * 自己 ringbuf_port_init() 一个 RB 端口再 set 上去，
		 * 否则 _process 里 audio_element_output() 因 el->out == NULL
		 * 直接把数据丢掉，get_output_port 永远返回 NULL。 */
		ringbuf_port_cfg_t rb_cfg = RINGBUF_PORT_CFG_DEFAULT();

		rb_cfg.ringbuf_size = block_bytes * cfg.out_block_num * 2;
		if (rb_cfg.ringbuf_size < RINGBUF_PORT_RINGBUF_SIZE)
			rb_cfg.ringbuf_size = RINGBUF_PORT_RINGBUF_SIZE;

		port = ringbuf_port_init(&rb_cfg);
		if (port == NULL) {
			csh_printf(csh, "dmic adk: ringbuf port init failed\r\n");
			goto out_deinit;
		}
		audio_element_set_output_port(el, port);
		csh_printf(csh, "dmic adk: 自建输出 ringbuffer %uB\r\n", (unsigned)rb_cfg.ringbuf_size);
	}

	buf = (int16_t *)os_malloc(block_bytes);
	if (buf == NULL) {
		csh_printf(csh, "dmic adk: no mem\r\n");
		goto out_deinit;
	}

	if (audio_element_run(el) != BK_OK) {
		csh_printf(csh, "dmic adk: element run failed\r\n");
		goto out_free;
	}

	/* run() 只创建元素任务，**不会**调 open()。
	 * 真正开始采集要走 resume()：AEL_MSG_CMD_RESUME → _open() →
	 * bk_dma_start(dmic) + bk_aud_dmic_start()。
	 * 官方 audio_pipeline_run() 就是 run() 之后紧跟 resume()
	 * （audio_pipeline.c:333 + :288）。漏掉这一步的症状：
	 * 元素任务在跑、但 DMA 从未启动，ring buffer 的 fill 恒为 0，
	 * _read 一直返回 0 → 输出端口一帧数据都没有（"没读到数据"），
	 * 且 stop 时因为 is_running==false 打出 "Element already stopped"。 */
	if (audio_element_resume(el, 0, 2000 / portTICK_RATE_MS) != BK_OK) {
		csh_printf(csh, "dmic adk: element resume failed（DMA/DMIC 启动失败？）\r\n");
		goto out_free;
	}

	/* 组件的 `dual_dmic && !ref_mode` 分支只启 DMIC，**不启 ADC**
	 * （`onboard_dual_dmic_mic_stream.c:458-488`）。
	 * 但本板实测：ADC 不 start → DMIC FIFO 恒 EMPTY（`fifo=0x00A0F000`），
	 * DMA 一次完成中断都等不到 → 组件任务永远卡在
	 * `rtos_wait_for_event_flags(EVENT_DUAL_DMIC_DMA_DONE_BIT, FOREVER)`，
	 * 表现为 `filled=0 / state=3` 且销毁时 `Element task destroy timeout`。
	 * 所以这里按本板结论补两步：ADC start + ADC FIFO 排空 DMA
	 * （不排空会在约 0.5s 后把 CIC 拖停）。 */
	adc_started = (bk_aud_adc_start() == BK_OK);
	if (!adc_started)
		csh_printf(csh, "dmic adk: adc start failed\r\n");
	if (dmic_adc_drain_start(true) != BK_OK)
		csh_printf(csh, "dmic adk: adc drain dma start failed\r\n");

	csh_printf(csh, "dmic adk: 官方组件 %uHz, %dms, block=%uB, %s, shift=x%d（已去直流）\r\n",
	           (unsigned)s_dmic_rate, ms, (unsigned)block_bytes,
	           stereo ? "stereo=L/R" : "sgl_out=L",
	           (int)((s_dmic_shift >= 0) ? (1 << s_dmic_shift) : 0));
	if (f_opened)
		csh_printf(csh, "dmic adk: 录音中 -> %s（%uHz %s 16bit）\r\n",
		           s_adk_path, (unsigned)s_dmic_rate, stereo ? "stereo" : "mono");

	/* 上电/建立瞬态：组件启动瞬间会出一段大信号（实测峰值打到 16384
	 * = 50%FS，衰减要 300~500ms），若计入统计会把总 rms 抬高一个量级，
	 * 看不出稳态底噪。第一个 500ms 窗口只排空端口、不累计统计。 */
	t_start = rtos_get_time();
	t_end = t_start + (uint32_t)ms;
	t_warm = t_start + 500;
	seg_end = t_start + 500;
	t_dump = t_start + 1200;
	win_start = t_warm;

	while (rtos_get_time() < t_end) {
		int filled = (int)audio_port_get_filled_size(port);

		if (filled >= (int)block_bytes) {
			int rd = audio_port_read(port, (char *)buf, block_bytes, 100);

			if (rd > 0) {
				int frames = rd / (2 * nch);
				bool warm = (rtos_get_time() >= t_warm);

				seg_got += (uint32_t)rd;

				/* 三步顺序不能变：
				 *   1) 一阶高通去直流（拐点 ≈40Hz@32k）；
				 *   2) 带饱和的增益（左移后夹到 int16）；
				 *   3) 统计。
				 * 先放大再去直流会把直流顶到饱和；先统计再放大则
				 * 统计值不等于落盘内容。
				 * stereo 时 buf 是 L/R 交织，滤波器状态和统计量按声道
				 * 各存一份，绝不能让 R 的样本污染 L 的历史值。 */
				for (i = 0; i < frames; i++) {
					for (c = 0; c < nch; c++) {
						int idx = i * nch + c;
						int32_t x = buf[idx];
						int32_t y, hf;

						/* y[n] = x[n] - x[n-1] + a*y[n-1]，a = 1 - 1/128。
						 * 只杀直流和超低频漂移，语音（>80Hz）不受影响。 */
						dc_y[c] = x - dc_prev[c] + dc_y[c] - (dc_y[c] >> 7);
						dc_prev[c] = x;
						y = dc_y[c];

						/* 再串一个 a=0.75 的一阶高通（拐点 ≈1.3kHz@32k），
						 * 只统计能量，用来判断「低频成分为主的语音」还是
						 * 「宽带噪声」：语音 hf 占比低，白噪 hf 占比接近 100%。 */
						hf_y[c] = y - hf_prev[c] + hf_y[c] - (hf_y[c] >> 2);
						hf_prev[c] = y;
						hf = hf_y[c];

						if (warm) {
							sum_dc[c] += x;
							ac_sq[c] += (int64_t)y * y;
							hf_sq[c] += (int64_t)hf * hf;
							seg_ac_sq[c] += (int64_t)y * y;
							seg_hf_sq[c] += (int64_t)hf * hf;
							seg_dc_sum[c] += x;
							if (y > ac_pk[c]) ac_pk[c] = y;
							if (y < -ac_pk[c]) ac_pk[c] = -y;
							if (y > seg_ac_pk[c]) seg_ac_pk[c] = y;
							if (y < -seg_ac_pk[c]) seg_ac_pk[c] = -y;
							/* 100ms 窗峰值（用于"有没有声事件"判定）。 */
							if (y > (int32_t)win_pk[c])
								win_pk[c] = (uint32_t)y;
							if (-y > (int32_t)win_pk[c])
								win_pk[c] = (uint32_t)(-y);
							if (has_prev[c] && ((y >= 0) != (prev[c] >= 0)))
								seg_zc[c]++;
							prev[c] = y;
							has_prev[c] = true;
							seg_n[c]++;
							nsamp[c]++;
							if ((y <= 1) && (y >= -1))
								flat[c]++;
						}

						/* 组件自带的 dmic_data_shift 是裸 `<<=`、无饱和
						 * （onboard_dual_dmic_mic_stream.c:589），shift>=5
						 * 就会按位回绕成失真。这里改用带饱和的移位，
						 * 任意档位都不会溢出。 */
						if (s_dmic_shift >= 0) {
							int32_t t = y << s_dmic_shift;

							if (t > 32767) t = 32767;
							else if (t < -32768) t = -32768;
							buf[idx] = (int16_t)t;
						} else {
							buf[idx] = (int16_t)(y >> (-s_dmic_shift));
						}
					}
				}

				/* 100ms 统计窗收口：把本窗峰值插入该路的「最响 3 名」。
				 * 只在 warm 之后累计，窗口起点与统计起点对齐。 */
				{
					uint32_t now = rtos_get_time();

					if (warm && (now - win_start >= 100)) {
						for (c = 0; c < nch; c++) {
							int k = 0, j;

							if (win_pk[c] == 0)
								continue;
							while ((k < burst_cnt[c]) &&
							       (win_pk[c] <= (uint32_t)burst_pk[c][k]))
								k++;
							if (k < 3) {
								int last = (burst_cnt[c] < 3) ? burst_cnt[c] : 2;

								for (j = last; j > k; j--) {
									burst_pk[c][j] = burst_pk[c][j - 1];
									burst_ms[c][j] = burst_ms[c][j - 1];
								}
								burst_pk[c][k] = (int32_t)win_pk[c];
								burst_ms[c][k] = now - t_start;
								if (burst_cnt[c] < 3)
									burst_cnt[c]++;
							}
							win_pk[c] = 0;
						}
						win_start = now;
					}
				}

				/* 落盘：跳过前 500ms 上电瞬态。否则每个文件开头都是
				 * 一声半量程（peak 16384）的「咔」，盖掉后面正常内容，
				 * 听起来就是「一声爆音 + 噪声」。 */
				if (f_opened && write_ok && warm) {
					if (f_write(&fil, buf, (UINT)rd, &bw) != FR_OK) {
						csh_printf(csh, "dmic adk: 写卡失败（卡满/拔卡？），"
						           "停止写入，已写的部分仍会保存\r\n");
						write_ok = false;
					} else {
						data_bytes += (uint32_t)bw;
					}
				}

				/* 抓一段原始波形（落盘后的值）。步长按帧长自适应：
				 * 32k/20ms 一帧 640 点，每 4 点取 1 → 128 点覆盖 16ms；
				 * 16k 只有 320 点，就逐点打，绝不越界。 */
				if (!dumped && rtos_get_time() >= t_dump) {
					int step = (frames >= 512) ? 4 : 1;
					int cnt = frames / step;
					int k;

					if (cnt > 128)
						cnt = 128;
					dumped = 1;
					csh_printf(csh, "dmic adk: 波形抽查（%d 点，步长 %d%s）\r\n",
					           cnt, step, stereo ? "，L/R 成对" : "");
					for (k = 0; k < cnt; k++) {
						if (stereo)
							csh_printf(csh, "%6d/%6d%s",
							           (int)buf[k * step * 2],
							           (int)buf[k * step * 2 + 1],
							           ((k % 8) == 7) ? "\r\n" : "");
						else
							csh_printf(csh, "%7d%s", (int)buf[k * step],
							           ((k % 16) == 15) ? "\r\n" : "");
					}
					if ((stereo ? (cnt % 8) : (cnt % 16)) != 0)
						csh_printf(csh, "\r\n");
				}
			}
		} else {
			rtos_delay_milliseconds(1);
		}

		/* 每 500ms 一行，时间按相对开始计；没有取到任何字节才叫
		 * 「无数据」。之前按瞬时 filled 判定，会在正常采集时
		 * （端口刚被读空）也打出「无数据 filled=0」，纯噪音。 */
		if (rtos_get_time() >= seg_end) {
			uint32_t rel = seg_end - 500 - t_start;

			if (seg_got == 0) {
				csh_printf(csh, "  [%5ums] 无数据 filled=%d state=%d\r\n",
				           (unsigned)rel, filled,
				           (int)audio_element_get_state(el));
			} else if (seg_n[0] == 0) {
				csh_printf(csh, "  [%5ums] 预热中（上电瞬态）got=%uB\r\n",
				           (unsigned)rel, (unsigned)seg_got);
			} else {
				uint32_t rms = isqrt64((uint64_t)(seg_ac_sq[0] / (int64_t)seg_n[0]));
				uint32_t hfr = isqrt64((uint64_t)(seg_hf_sq[0] / (int64_t)seg_n[0]));
				int32_t dc = (int32_t)(seg_dc_sum[0] / (int64_t)seg_n[0]);
				uint32_t zcr_s = (uint32_t)((uint64_t)seg_zc[0] * s_dmic_rate / seg_n[0]);
				uint32_t cr = rms ? (uint32_t)(seg_ac_pk[0] / (int32_t)rms) : 0;

				/* cr = peak/rms。平稳噪声 3~5，语音有音节峰谷 8 以上；
				 * hf = 1.3kHz 以上能量占比，白噪接近 100%，语声低得多。
				 * dc 是原始直流，用来确认去直流前扣掉了多少。
				 *
				 * stereo 时 L、R 并排打，判断两颗麦是否对称就看这两列
				 * 的 rms/pk —— 一颗声孔被堵死时它那列会明显偏小。 */
				if (stereo) {
					uint32_t n1 = seg_n[1] ? seg_n[1] : 1;
					uint32_t rms1 = isqrt64((uint64_t)(seg_ac_sq[1] / (int64_t)n1));
					uint32_t hfr1 = isqrt64((uint64_t)(seg_hf_sq[1] / (int64_t)n1));
					uint32_t cr1 = rms1 ? (uint32_t)(seg_ac_pk[1] / (int32_t)rms1) : 0;

					csh_printf(csh, "  [%5ums] L rms=%5u pk=%6d cr=%2u hf=%3u%%"
					           " | R rms=%5u pk=%6d cr=%2u hf=%3u%%\r\n",
					           (unsigned)rel,
					           (unsigned)rms, (int)seg_ac_pk[0], (unsigned)cr,
					           (unsigned)(rms ? (hfr * 100 / rms) : 0),
					           (unsigned)rms1, (int)seg_ac_pk[1], (unsigned)cr1,
					           (unsigned)(rms1 ? (hfr1 * 100 / rms1) : 0));
				} else {
					csh_printf(csh, "  [%5ums] rms=%5u pk=%6d cr=%2u hf=%3u%% dc=%5d zcr=%5u/s\r\n",
					           (unsigned)rel, (unsigned)rms, (int)seg_ac_pk[0],
					           (unsigned)cr,
					           (unsigned)(rms ? (hfr * 100 / rms) : 0),
					           (int)dc, (unsigned)zcr_s);
				}
			}
			seg_end += 500;
			seg_got = 0;
			for (c = 0; c < 2; c++) {
				seg_ac_sq[c] = 0;
				seg_hf_sq[c] = 0;
				seg_dc_sum[c] = 0;
				seg_n[c] = 0;
				seg_zc[c] = 0;
				seg_ac_pk[c] = 0;
				has_prev[c] = false;
			}
		}
	}

	if (nsamp[0] > 0) {
		uint32_t rms = isqrt64((uint64_t)(ac_sq[0] / (int64_t)nsamp[0]));
		uint32_t hfr = isqrt64((uint64_t)(hf_sq[0] / (int64_t)nsamp[0]));
		int32_t dc = (int32_t)(sum_dc[0] / (int64_t)nsamp[0]);
		uint32_t cr = rms ? (uint32_t)(ac_pk[0] / (int32_t)rms) : 0;
		uint32_t pct = (uint32_t)((int64_t)ac_pk[0] * 100 / 32768);

		if (stereo) {
			uint32_t n1 = nsamp[1] ? nsamp[1] : 1;
			uint32_t rms1 = isqrt64((uint64_t)(ac_sq[1] / (int64_t)n1));
			uint32_t hfr1 = isqrt64((uint64_t)(hf_sq[1] / (int64_t)n1));
			uint32_t cr1 = rms1 ? (uint32_t)(ac_pk[1] / (int32_t)rms1) : 0;
			uint32_t pct1 = (uint32_t)((int64_t)ac_pk[1] * 100 / 32768);
			uint32_t ratio = rms ? (uint32_t)((uint64_t)rms1 * 100 / rms) : 0;
			int32_t dc1 = (int32_t)(sum_dc[1] / (int64_t)n1);
			uint32_t fl0 = (uint32_t)((uint64_t)flat[0] * 100 / (nsamp[0] ? nsamp[0] : 1));
			uint32_t fl1 = (uint32_t)((uint64_t)flat[1] * 100 / n1);

			csh_printf(csh, "ADK L: rms=%u peak=%d (%u%%FS) crest=%u hf=%u%% flat=%u%% dc=%d\r\n",
			           (unsigned)rms, (int)ac_pk[0], (unsigned)pct,
			           (unsigned)cr, (unsigned)(rms ? (hfr * 100 / rms) : 0),
			           (unsigned)fl0, (int)dc);
			csh_printf(csh, "ADK R: rms=%u peak=%d (%u%%FS) crest=%u hf=%u%% flat=%u%% dc=%d\r\n",
			           (unsigned)rms1, (int)ac_pk[1], (unsigned)pct1,
			           (unsigned)cr1, (unsigned)(rms1 ? (hfr1 * 100 / rms1) : 0),
			           (unsigned)fl1, (int)dc1);
			csh_printf(csh, "  -> R/L 电平比 = %u%%（100%% 表示两颗麦一样响）\r\n",
			           (unsigned)ratio);
			/* 判「这一路是不是麦克风」：真 DMIC 是 PDM 调制器，只要供电和
			 * 时钟在就一直在抖动，去直流后仍有宽带成分，hf 应与另一路同量级。
			 * 若某路 hf 极低（<10%）且电平很小，说明它输出的是缓慢漂移的
			 * 常数而不是调制流 —— 该路根本没在驱动 DAT（DOUT 虚焊 / 未供电
			 * / 未起振），不是「安静」。flat 是同一判断的补充形态。 */
			{
				uint32_t hfp0 = rms ? (hfr * 100 / rms) : 0;
				uint32_t hfp1 = rms1 ? (hfr1 * 100 / rms1) : 0;
				bool dead_l = (fl0 >= 90) || ((hfp0 < 10) && (rms < 200));
				bool dead_r = (fl1 >= 90) || ((hfp1 < 10) && (rms1 < 200));

				if (dead_r && !dead_l)
					csh_printf(csh, "  -> R 路异常：电平只有 L 的 %u%%、hf 仅 %u%%、crest=%u，"
					           "不像调制流 → 查 U15(SEL=1V8) 的供电与 DOUT 焊接\r\n",
					           (unsigned)ratio, (unsigned)hfp1, (unsigned)cr1);
				else if (dead_l && !dead_r)
					csh_printf(csh, "  -> L 路异常：hf 仅 %u%%、crest=%u，不像调制流 → "
					           "查 U16(SEL=GND) 的供电与 DOUT 焊接\r\n",
					           (unsigned)hfp0, (unsigned)cr);
				else if (dead_l && dead_r)
					csh_printf(csh, "  -> 两路都不像调制流（hf 都极低），先查共用的"
					           "MIC 1V8 与 PDM 时钟\r\n");
				else if ((ratio >= 70) && (ratio <= 140))
					csh_printf(csh, "  -> 两路都在出数且电平相当，两颗麦都在工作\r\n");
				else
					csh_printf(csh, "  -> 两路都在出数但电平差到 %u%%：%s，"
					           "按声孔堵塞/振膜受损排查\r\n", (unsigned)ratio,
					           (ratio < 70) ? "R 偏小" : "L 偏小");
			}
			if ((cr >= 8) || (cr1 >= 8))
				csh_printf(csh, "  -> 至少一路 crest>=8，波形有峰谷结构（像语声/脉冲）\r\n");
			else
				csh_printf(csh, "  -> 两路都 crest<8，整段电平平稳（形态上不像语声/脉冲）\r\n");
		} else {
			csh_printf(csh, "ADK PCM: dc=%d  rms(AC)=%u  peak=%d (%u%%FS)  crest=%u  hf=%u%%\r\n",
			           (int)dc, (unsigned)rms, (int)ac_pk[0], (unsigned)pct,
			           (unsigned)cr, (unsigned)(rms ? (hfr * 100 / rms) : 0));
			if (cr >= 8)
				csh_printf(csh, "  -> crest=%u，波形有峰谷结构（像语声/脉冲）\r\n", (unsigned)cr);
			else
				csh_printf(csh, "  -> crest=%u，整段电平平稳（形态上不像语声/脉冲）\r\n", (unsigned)cr);
		}
		/* 峰值事件报告：把每路「最响的 3 个 100ms 窗」摊开。
		 * 这是唯一不依赖回忆的"刚才到底有没有出声"证据：
		 * 一次拍手/一次大喊必然上榜，噪声永远上不了榜。 */
		for (c = 0; c < nch; c++) {
			int k;

			csh_printf(csh, "  -> %s 最响 100ms 窗：",
			           stereo ? ((c == 0) ? "L" : "R") : "MIC");
			if (burst_cnt[c] == 0) {
				csh_printf(csh, "无\r\n");
				continue;
			}
			for (k = 0; k < burst_cnt[c]; k++) {
				int d100 = dmic_pk_to_db100(burst_pk[c][k]);

				csh_printf(csh, "%st=%ums pk=%d (%d.%ddBFS)",
				           k ? "，" : "", (unsigned)burst_ms[c][k],
				           (int)burst_pk[c][k],
				           d100 / 10, ((d100 < 0) ? -d100 : d100) % 10);
			}
			csh_printf(csh, "\r\n");
		}

		/* 判定门槛 800 LSB ≈ 2.4%FS ≈ -32dBFS。按常见 DMIC 灵敏度
		 * （-26dBFS @ 94dBSPL）折算约 88dBSPL 峰值——5cm 处拍手、
		 * 贴麦大声说话都会超过；而底噪峰值只有 ~330，不会误触发。 */
		{
			int32_t top = 0;
			int d100;

			for (c = 0; c < nch; c++)
				if ((burst_cnt[c] > 0) && (burst_pk[c][0] > top))
					top = burst_pk[c][0];
			d100 = dmic_pk_to_db100(top);
			if (top <= 0)
				csh_printf(csh, "  -> 没有任何有效峰值：这一路没在出数\r\n");
			else if (top < 800)
				csh_printf(csh, "  -> 全程最响 %d LSB (%d.%ddBFS)：%dms 内没有任何强瞬态。"
				           "要么采集时确实没出声，要么麦克风有自噪声但声孔/振膜不通\r\n",
				           (int)top, d100 / 10, ((d100 < 0) ? -d100 : d100) % 10,
				           ms);
			else
				csh_printf(csh, "  -> 全程最响 %d LSB (%d.%ddBFS)：捕获到声事件，"
				           "麦克风与采集链路是通的\r\n",
				           (int)top, d100 / 10, ((d100 < 0) ? -d100 : d100) % 10);
		}
		csh_printf(csh, "  （已跳过前 500ms 上电瞬态，且已去直流；与手写 dmic 路径对照）\r\n");
	} else {
		csh_printf(csh, "dmic adk: 没读到数据\r\n");
	}
	ret = 0;

	/* 组件在 input 超时后会自己走 on_cmd_finish（is_running 置 false），
	 * 此时再 stop 只会打一条 "Element already stopped" 的 ERROR 日志。
	 * 先看状态，避免这条假告警。 */
	if (audio_element_get_state(el) == AEL_STATE_RUNNING) {
		audio_element_stop(el);
		(void)audio_element_wait_for_stop_ms(el, 200);
	}

out_free:
	os_free(buf);
out_deinit:
	/* 先撤自己补的 ADC 排空/启动（引用 ADC FIFO 的 DMA 必须先停），
	 * 再让组件走它的 close/destroy（内部会 bk_aud_adc_deinit）。 */
	dmic_adc_drain_stop();
	if (adc_started)
		bk_aud_adc_stop();
	/* deinit 会调用组件的 destroy：停 DMA + deinit ADC/DMIC，释放硬件 */
	(void)audio_element_deinit(el);
out_power:
	/* 录音收尾：回填 RIFF/data 长度再关闭文件。
	 * data 长度用实际写入字节，而不是 ms 标称时长——中途没数据、
	 * 写卡失败、提前 Ctrl 打断时，头里的长度仍然和文件内容一致，
	 * 播放器不会报文件损坏。 */
	if (f_opened) {
		uint8_t hdr[44];
		uint32_t ms_total = 0;

		/* 实际时长 = 字节数 / (采样率 * 2B * 声道数)。用实际值而不是标称 ms，
		 * 这样中途没数据或写卡失败时，头部长度与内容仍然自洽。 */
		if (s_dmic_rate > 0)
			ms_total = (uint32_t)((uint64_t)data_bytes * 1000 /
			                      ((uint64_t)s_dmic_rate * 2 * (uint32_t)nch));

		wav_build_header(hdr, s_dmic_rate, data_bytes, (uint8_t)nch);
		if (f_lseek(&fil, 0) == FR_OK)
			(void)f_write(&fil, hdr, 44, &bw);
		(void)f_close(&fil);
		f_opened = false;

		csh_printf(csh, "dmic adk: WAV 已保存 %s  %u B / %u 帧 / %u.%us\r\n",
		           s_adk_path, (unsigned)data_bytes,
		           (unsigned)(data_bytes / (2 * nch)),
		           (unsigned)(ms_total / 1000), (unsigned)((ms_total % 1000) / 100));
	}

	if (s_freq_vote_en)
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_DEFAULT);
	if (!no_pwr && !mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);
	csh_printf(csh, "dmic adk: done\r\n");
	return ret;
}
#endif

static int dmic_capture_once(chry_shell_t *csh, int ms, aud_clk_t clk_src, int rate)
{
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
	uint32_t first[DMIC_FIRST_WORDS];
	uint32_t w = 0, st = 0, deadline;
	int total, got = 0, i, nfirst = 0;
	int64_t sum_l = 0, sum_r = 0;
	int32_t pk_l = 0, pk_r = 0;
	/* 分段统计：每 500ms 打一行 rms/peak/zcr。
	 * 200ms 的短采样永远落在"还没冻结"的那一段，发现不了"跑一会儿才坏"，
	 * 这里直接在轮询路径里分段，不依赖 SD 卡和 PC 侧脚本。 */
	uint32_t seg_samples, seg_n = 0;
	int64_t seg_sq = 0;
	int32_t seg_pk = 0;
	uint32_t seg_zc = 0;
	int32_t prev_s = 0;
	bool seg_has_prev = false;
	uint32_t seg_start_ms = 0;
	/* 分段里同时统计右声道（高16位）。只统计 w_pick 会掩盖"低16有声音、
	 * 高16是恒定直流"这种情况，那会表现成"全程冻结"的假象。 */
	int64_t seg_sq_r = 0;
	int32_t seg_pk_r = 0;
	uint32_t seg_zc_r = 0;
	int32_t prev_sr = 0;
	bool seg_has_prev_r = false;
	uint32_t seg_raw[4];
	uint32_t seg_raw_n = 0;
	/* 两个半字完全相同的 word 计数：若接近 100%，说明 FIFO 一个 32bit
	 * 槽里装的是**同一个采样的两份拷贝**（或两个相邻采样），此时按
	 * L/R 取声道、按 samp_rate 计算时长都会出错。 */
	uint32_t dup_n = 0;
	/* 全程统计：zcr 与直流。zcr 是区分"真声音"和"低频漂移/冻结"的关键：
	 * 语音至少几百 Hz，zcr 会到 1k/s 量级；直流漂移只有几 Hz ~ 几十 Hz；
	 * 冻结则是 0。只看 rms/peak 会被大直流偏置骗过去。 */
	uint32_t zc_total = 0;
	bool zc_has_prev = false;
	int32_t zc_prev = 0;
	int64_t sum_sel = 0;
	bool mic_was_on;
	bk_err_t ret = BK_OK;

	mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	if (!mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		csh_printf(csh, "dmic: MIC 1V8 rail on (P48)\r\n");
		rtos_delay_milliseconds(5);
	}

	/* rate <= 0 时退回全局设定；正常的调用方传 s_dmic_rate */
	cfg.samp_rate = (rate > 0) ? (uint32_t)rate : s_dmic_rate;
	cfg.dmic_chl = AUD_DMIC_CHL_L;
	cfg.clk_src = clk_src;

	ret = dmic_hw_start(csh, &cfg);
	if (ret != BK_OK)
		goto out_deinit;

	/* 等 CIC 滤波器出第一个数：最多 100ms */
	deadline = rtos_get_time() + 100;
	while (rtos_get_time() < deadline) {
		bk_aud_dmic_get_status(&st);
		if (!(st & AUD_DMIC_FIFO_EMPTY_MASK))
			break;
	}
	if (rtos_get_time() >= deadline) {
		csh_printf(csh, "dmic: no data (%s), fifo status=0x%08X\r\n",
		           (clk_src == AUD_CLK_APLL) ? "APLL" : "XTAL", (unsigned)st);
		csh_printf(csh, "      check: mic VDD / P8 CLK / P9 DAT, 或跑 'dmic diag'\r\n");
		ret = BK_FAIL;
		goto out_stop;
	}

	/* 用实际生效的采样率算总采样数，别用编译期宏 —— 否则
	 * `dmic 200 apll 32000` 会按 16k 算，只采到一半时长。 */
	total = (int)(cfg.samp_rate * (uint32_t)ms / 1000);
	seg_samples = cfg.samp_rate / 2;      /* 500ms 一段 */
	if (seg_samples == 0)
		seg_samples = 8000;
	deadline = rtos_get_time() + (uint32_t)ms + 300;

	while ((got < total) && (rtos_get_time() < deadline)) {
		bk_aud_dmic_get_status(&st);
		if (st & AUD_DMIC_FIFO_EMPTY_MASK) {
			rtos_delay_milliseconds(1);
			continue;
		}

		bk_aud_dmic_get_fifo_data(&w);
		if (nfirst < DMIC_FIRST_WORDS)
			first[nfirst++] = w;

		if (seg_raw_n < 4)
			seg_raw[seg_raw_n++] = w;

		if (w_low(w) == w_high(w))
			dup_n++;

		{
			int32_t l = w_low(w);
			int32_t r = w_high(w);
			int32_t sel = w_pick(w);

			sum_l += (int64_t)l * l;
			sum_r += (int64_t)r * r;
			if (l > pk_l) pk_l = l;
			if (l < -pk_l) pk_l = -l;
			if (r > pk_r) pk_r = r;
			if (r < -pk_r) pk_r = -r;

			/* 全程 zcr / 直流 */
			sum_sel += sel;
			if (zc_has_prev && ((sel >= 0) != (zc_prev >= 0)))
				zc_total++;
			zc_prev = sel;
			zc_has_prev = true;

			/* 分段(500ms)：rms/peak/zcr，zcr=0 且 rms 恒定 = CIC 冻结 */
			seg_sq += (int64_t)sel * sel;
			if (sel > seg_pk) seg_pk = sel;
			if (sel < -seg_pk) seg_pk = -sel;
			if (seg_has_prev && ((sel >= 0) != (prev_s >= 0)))
				seg_zc++;
			prev_s = sel;
			seg_has_prev = true;
			seg_n++;

			/* 右声道（高16位）单独累计，用于分辨"哪一路才是活的" */
			seg_sq_r += (int64_t)r * r;
			if (r > seg_pk_r) seg_pk_r = r;
			if (r < -seg_pk_r) seg_pk_r = -r;
			if (seg_has_prev_r && ((r >= 0) != (prev_sr >= 0)))
				seg_zc_r++;
			prev_sr = r;
			seg_has_prev_r = true;

			if (seg_n >= seg_samples) {
				uint32_t rms_l = isqrt64((uint64_t)(seg_sq / (int64_t)seg_n));
				uint32_t rms_r = isqrt64((uint64_t)(seg_sq_r / (int64_t)seg_n));
				const char *note = "";
				uint32_t q;

				if ((seg_zc == 0) && (seg_zc_r == 0))
					note = "  <- 双侧 zcr=0：CIC 输出恒定";
				else if ((seg_zc + seg_zc_r) * 2 < 60)
					note = "  <- zcr 极低：直流/漂移，不是声音";

				csh_printf(csh,
				           "  [%4ums] L rms=%5u zcr=%5u pk=%5d | R rms=%5u zcr=%5u pk=%5d%s\r\n",
				           (unsigned)seg_start_ms, (unsigned)rms_l,
				           (unsigned)(seg_zc * 2), (int)seg_pk,
				           (unsigned)rms_r, (unsigned)(seg_zc_r * 2), (int)seg_pk_r,
				           note);

				if (seg_raw_n > 0) {
					csh_printf(csh, "             raw:");
					for (q = 0; q < seg_raw_n; q++)
						csh_printf(csh, " %08X", (unsigned)seg_raw[q]);
					csh_printf(csh, "\r\n");
				}

				seg_start_ms += 500;
				seg_sq = 0;
				seg_n = 0;
				seg_pk = 0;
				seg_zc = 0;
				seg_has_prev = false;
				seg_sq_r = 0;
				seg_pk_r = 0;
				seg_zc_r = 0;
				seg_has_prev_r = false;
				seg_raw_n = 0;
			}
		}
		got++;
	}

	csh_printf(csh, "dmic: %uHz, %dms, %d/%d frames\r\n",
	           (unsigned)cfg.samp_rate, ms, got, total);
	if (nfirst > 0) {
		csh_printf(csh, "  raw:");
		for (i = 0; i < nfirst; i++)
			csh_printf(csh, " %08X", first[i]);
		csh_printf(csh, "\r\n");
	}

	if (got > 0) {
		uint32_t zcr = (uint32_t)((uint64_t)zc_total * 1000 / (uint64_t)(ms > 0 ? ms : 1));
		int32_t dc = (int32_t)(sum_sel / (got > 0 ? got : 1));

		dmic_print_stat(csh, "L", sum_l, pk_l, got);
		dmic_print_stat(csh, "R", sum_r, pk_r, got);
		csh_printf(csh, "  zcr=%u/s  dc=%d  （语音 zcr 应 >100/s；几十 = 直流/漂移；0 = 冻结）\r\n",
		           (unsigned)zcr, (int)dc);
		csh_printf(csh, "  halfword: low==high 占 %u/%d（接近 100%% = 一个 32bit 槽是同一采样的两份）\r\n",
		           (unsigned)dup_n, got);

		/* 采集结束时用公开 API 看一眼 FIFO 最终状态 */
		{
			uint32_t fs = 0;

			(void)bk_aud_dmic_get_status(&fs);
			csh_printf(csh, "  fifo@end=0x%06X  mic_pwr=%d\r\n",
			           (unsigned)(fs & 0x00FFFFFF),
			           (int)bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8));
		}

		if ((pk_l < 16) && (pk_r < 16))
			csh_printf(csh, "  -> 没信号：检查 MIC 供电/时钟，或麦克风没接\r\n");
		else if (zcr == 0)
			csh_printf(csh, "  -> 输出恒定：CIC 冻结（或麦克风没在驱动 DATA）\r\n");
		else if (zcr < 100)
			csh_printf(csh, "  -> 只有直流/低频漂移，不是声音：麦可能没驱动 DATA"
			                "（SEL 电平不合规 / 没供电 / 焊接）\r\n");
		else
			csh_printf(csh, "  -> 数据在变化且 zcr 合理，通路正常\r\n");
	}

out_stop:
	bk_aud_dmic_stop();
out_deinit:
	bk_aud_dmic_deinit();
	bk_aud_adc_deinit();   /* 配对释放 dmic_hw_start() 借来的总电源 */
	if (!mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);

	return ret == BK_OK ? 0 : -1;
}

/* 采集：APLL 没数据时自动换 XTAL(26MHz) 再试一次。
 *
 * 注：早期注释写"16k 的 1024kHz PDM 对 LMD4030T261 不合法，所以固定用 32k"，
 * 那是错的 —— 麦的标准模式下限是 1000kHz，1024kHz 在范围内，且 2026-11-12
 * 实测 16k / 32k 都能正常出数（见 DMIC_SAMP_RATE 注释）。速率由调用方给，
 * 默认取 `dmic rate` 的当前值。 */
static int dmic_capture(chry_shell_t *csh, int ms, aud_clk_t clk_src, int rate)
{
	int ret = dmic_capture_once(csh, ms, clk_src, rate);

	if ((ret != 0) && (clk_src == AUD_CLK_APLL)) {
		csh_printf(csh, "dmic: APLL 时钟源没数据，自动换 XTAL(26MHz) 再试一次\r\n");
		ret = dmic_capture_once(csh, ms, AUD_CLK_XTAL, rate);
	}

	return ret;
}

/* dmic raw —— dump 16 个原始 FIFO word */
static int dmic_dump_raw(chry_shell_t *csh)
{
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
	uint32_t w, st = 0, deadline;
	int got = 0;
	bool mic_was_on;
	bk_err_t ret;

	mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	if (!mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	cfg.samp_rate = s_dmic_rate;
	cfg.dmic_chl = AUD_DMIC_CHL_L;

	ret = dmic_hw_start(csh, &cfg);
	if (ret != BK_OK)
		goto out_power;

	rtos_delay_milliseconds(10);

	deadline = rtos_get_time() + 200;
	while ((got < 16) && (rtos_get_time() < deadline)) {
		bk_aud_dmic_get_status(&st);
		if (st & AUD_DMIC_FIFO_EMPTY_MASK) {
			rtos_delay_milliseconds(1);
			continue;
		}
		bk_aud_dmic_get_fifo_data(&w);
		csh_printf(csh, "  [%2d] 0x%08X  L=%6d R=%6d\r\n",
		           got, w, w_low(w), w_high(w));
		got++;
	}

	if (got == 0) {
		csh_printf(csh, "dmic: no data, fifo status=0x%08X\r\n", st);
		csh_printf(csh, "      run 'dmic dbg' for register/gpio self-check\r\n");
	}

	bk_aud_dmic_stop();
	bk_aud_dmic_deinit();
	bk_aud_adc_deinit();   /* 配对释放 dmic_hw_start() 借来的总电源 */
out_power:
	if (!mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);

	return (got > 0) ? 0 : -1;
}

/* ================= 录音到 SD 卡 ================= */

typedef enum {
	REC_IDLE = 0,
	REC_RUNNING,
} rec_state_t;

static FATFS s_fs;
static bool s_fs_mounted;

static volatile rec_state_t s_rec_state = REC_IDLE;
static volatile bool s_rec_stop;
static beken_thread_t s_rec_task = NULL;
static beken_semaphore_t s_rec_done_sem = NULL;

static uint32_t s_rec_rate = DMIC_SAMP_RATE;
static uint32_t s_rec_sec;
static char s_rec_path[DMIC_REC_PATH_LEN];

/* DMA 通道与双缓冲 */
static dma_id_t s_dma_id = DMA_ID_MAX;
static beken_semaphore_t s_dma_sem = NULL;
static uint32_t *s_dma_buf = NULL;
static uint32_t s_frame_samples;    /* 每帧采样数（32bit word 数） */
static uint32_t s_frame_bytes;
static volatile uint32_t s_dma_cnt;

/* 中断里把刚搬完的那一帧快照到这块内存，任务只读快照。
 *
 * 为什么需要：DMA 是 REPEAT 连续跑的，单帧窗口下它每 20ms 就回头覆盖同一块
 * 内存。任务在 `dmic` 的读路径上稍慢一点（信号量唤醒 + 调度）就会读到"半个
 * 新帧 + 半个旧帧"的撕裂数据 —— 实测 `dma raw` 第一个 word 出现过
 * 1FD44000（正常应该是 00000000/音频数据），就是被下一帧的开头盖掉了。
 *
 * 快照在 ISR 里做，一次 1280B(16k)/3840B(48k) 的 memcpy，几十微秒，代价很小，
 * 换来的是任务无论什么时候读都拿到完整的一帧。
 * 上限 960 word = 48kHz × 20ms，和 pcm[] 一致。 */
#define DMIC_REC_MAX_WORDS  960
static uint32_t s_dma_snap[DMIC_REC_MAX_WORDS];

static void w_le16(uint8_t *p, uint16_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
}

static void w_le32(uint8_t *p, uint32_t v)
{
	p[0] = (uint8_t)(v & 0xFF);
	p[1] = (uint8_t)((v >> 8) & 0xFF);
	p[2] = (uint8_t)((v >> 16) & 0xFF);
	p[3] = (uint8_t)((v >> 24) & 0xFF);
}

/* 拼 44 字节 PCM 16bit WAV 头。nch = 1 单声道 / 2 双声道（L、R 交织）。
 * data_len 先给 0，录完再用 f_lseek 回来填。 */
static void wav_build_header(uint8_t hdr[44], uint32_t rate, uint32_t data_len, uint8_t nch)
{
	uint32_t byte_rate = rate * 2 * nch;   /* nch * 16bit */

	memcpy(&hdr[0], "RIFF", 4);
	w_le32(&hdr[4], 36 + data_len);          /* RIFF chunk size */
	memcpy(&hdr[8], "WAVE", 4);

	memcpy(&hdr[12], "fmt ", 4);
	w_le32(&hdr[16], 16);                    /* fmt chunk size */
	w_le16(&hdr[20], 1);                     /* PCM */
	w_le16(&hdr[22], nch);                   /* mono / stereo */
	w_le32(&hdr[24], rate);
	w_le32(&hdr[28], byte_rate);
	w_le16(&hdr[32], (uint16_t)(2 * nch));   /* block align */
	w_le16(&hdr[34], 16);                    /* bits per sample */

	memcpy(&hdr[36], "data", 4);
	w_le32(&hdr[40], data_len);
}

static void dmic_rec_dma_isr(dma_id_t id)
{
	(void)id;

	/* 立刻把刚完成的那一帧拷走，避免任务读到被下一帧覆盖的撕裂数据。
	 * 这里只做 memcpy，不加锁：任务端只看 s_dma_cnt 是否推进。 */
	if ((s_dma_buf != NULL) && (s_frame_bytes <= sizeof(s_dma_snap))) {
		memcpy(s_dma_snap, s_dma_buf, s_frame_bytes);
	}

	s_dma_cnt++;
	if (s_dma_sem)
		rtos_set_semaphore(&s_dma_sem);
}

/* SD 卡上电 + 挂载，等价于 cmd_sd.c 里的 sd_open() */
static int dmic_rec_sd_mount(chry_shell_t *csh)
{
	FRESULT fr;
	bk_err_t ret;

	if (s_fs_mounted)
		return 0;

	bk_pm_module_vote_ctrl_external_ldo(GPIO_CTRL_LDO_MODULE_SDIO,
	                                    CONFIG_LDO3V3_CTRL_GPIO,
	                                    GPIO_OUTPUT_STATE_HIGH);
	rtos_delay_milliseconds(5);

	ret = bk_sd_card_init();
	if (ret != BK_OK) {
		csh_printf(csh, "dmic: sd init failed: %d (CMD 无响应)\r\n", ret);
		csh_printf(csh, "      check: 卡的 3V3 / P14~P19 接线\r\n");
		bk_sdio_host_deinit();
		return -1;
	}

	/* 盘符 1: 才是 SD 卡（0 是未实现的 RAM 盘） */
	fr = f_mount(&s_fs, "1:", 1);
	if (fr != FR_OK) {
		csh_printf(csh, "dmic: f_mount failed: %d（卡没插/不是 FAT 分区）\r\n", fr);
		return -1;
	}

	s_fs_mounted = true;
	return 0;
}

/* 配 DMA：DMIC FIFO -> 双缓冲，REPEAT 模式每帧一次完成中断。
 * 注意 dst 的 dev 填 DMA_DEV_DTCM 是 SDK 里的固定写法（bsp_mic.c 同样），
 * 但缓冲区本身是普通堆内存；如果遇到 DMA 收不到数，优先怀疑这里——
 * 换成静态数组（放 .bss/DTCM）通常就能好。 */
static void dmic_rec_dma_deconfig(void);

static bk_err_t dmic_rec_dma_config(void)
{
	dma_config_t dma_cfg;
	uint32_t fifo_addr = 0;
	bk_err_t ret;

	s_frame_samples = (s_rec_rate / 1000) * DMIC_REC_FRAME_MS;
	s_frame_bytes = s_frame_samples * 4;     /* 每个 word 4 字节 */

	/* 官方两个例程都显式做了 DMA 驱动初始化：
	 *   aud_ns_recoder.c:603   bk_dma_driver_init();
	 *   onboard_dual_dmic_mic_stream.c 里则是 #if 0 掉（依赖别处已 init）。
	 * 这里显式调一次：幂等。少了它 DMA 可能能搬数但不进完成中断，
	 * 表现正是"卡在等信号量"。 */
	(void)bk_dma_driver_init();

	s_dma_id = bk_dma_alloc(DMA_DEV_AUDIO);
	if ((s_dma_id < DMA_ID_0) || (s_dma_id >= DMA_ID_MAX)) {
		s_dma_id = DMA_ID_MAX;
		return BK_FAIL;
	}

	/* 只需要一帧的 DMA 目的缓冲（见下面 dst.end_addr 处的说明）。
	 * 16k 一帧 1280B，48k 一帧 3840B，比原来 4 帧省 3/4 内存。 */
	s_dma_buf = (uint32_t *)os_malloc(s_frame_bytes);
	if (s_dma_buf == NULL) {
		ret = BK_ERR_NO_MEM;
		goto fail;
	}
	os_memset(s_dma_buf, 0, s_frame_bytes);

	ret = bk_aud_dmic_get_fifo_addr(&fifo_addr);
	if (ret != BK_OK)
		goto fail;

	os_memset(&dma_cfg, 0, sizeof(dma_cfg));
	dma_cfg.mode = DMA_WORK_MODE_REPEAT;
	dma_cfg.chan_prio = 1;
	dma_cfg.trans_type = DMA_TRANS_DEFAULT;

	dma_cfg.src.dev = DMA_DEV_AUD_DMIC;
	dma_cfg.src.width = DMA_DATA_WIDTH_32BITS;
	/* 官方 aud_ns_recoder.c:381 对 FIFO 端口用的是 INC_DISABLE：
	 * 源是固定地址的 FIFO 端口，不该递增。窗口 [port, port+4) 只有一个地址，
	 * 两种写法在这套配置下等价，但保持与官方一致，避免后续改窗口时出错。 */
	dma_cfg.src.addr_inc_en = DMA_ADDR_INC_DISABLE;
	dma_cfg.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
	dma_cfg.src.start_addr = fifo_addr;
	dma_cfg.src.end_addr = fifo_addr + 4;

	dma_cfg.dst.dev = DMA_DEV_DTCM;
	dma_cfg.dst.width = DMA_DATA_WIDTH_32BITS;
	dma_cfg.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
	dma_cfg.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
	dma_cfg.dst.start_addr = (uint32_t)(uintptr_t)s_dma_buf;
	/* dst 窗口只放**一帧**，不用多帧环形。
	 *
	 * 原先写成 buf .. buf + 4 帧，然后按 (s_dma_cnt-1) % 4 去读对应槽。
	 * 实测证明这个假设不成立：REPEAT 模式每一轮结束会把 dst 指针**重载回
	 * start_addr**，于是数据始终落在槽 0，槽 1~3 一直是 os_memset 出来的 0。
	 * 而任务却按 0/1/2/3 轮流去读 —— 4 次里有 3 次读到全 0。
	 *
	 * 症状正好对上实测：WAV 里 peak 12196（偶尔读到真数据）但 rms 只有
	 * 1232（3.7% FS），而 `dmic 200` 轮询同一个麦 rms 是 8859（27% FS）。
	 *
	 * 改成单帧窗口后，无论硬件是"每轮重载到 start"还是"在窗口内轮转"，
	 * 数据都只会落在这唯一一帧里，读永远读得到。 */
	dma_cfg.dst.end_addr = (uint32_t)(uintptr_t)s_dma_buf + s_frame_bytes;

	ret = bk_dma_init(s_dma_id, &dma_cfg);
	if (ret != BK_OK)
		goto fail;

	bk_dma_set_transfer_len(s_dma_id, s_frame_bytes);
	bk_dma_register_isr(s_dma_id, NULL, dmic_rec_dma_isr);
	bk_dma_enable_finish_interrupt(s_dma_id);

#if (CONFIG_SPE)
	/* 关键：本项目 CONFIG_SPE=1 / CONFIG_TZ=y。
	 *
	 * bk_dma_driver_init() 在 SPE 下只注册 secure 中断向量
	 * （dma_driver.c:218-222）：
	 *     #if CONFIG_SPE
	 *         bk_int_isr_register(INT_SRC_GDMA, dma_isr, NULL);
	 *     #else
	 *         bk_int_isr_register(INT_SRC_DMA0_NSEC, dma_isr, NULL);
	 * 而 DMA 通道 src/dst 的安全属性默认是 non-secure，于是：
	 *   - 通道按 non-secure 读 audio FIFO，被 TZ 拦住，数据搬不动；
	 *   - 完成中断也落在没人注册的 NSEC 向量上，ISR 永不触发。
	 * 现象正好对上实测：fifo=0x00503C0C（bit22=FULL，数据在产生）
	 * 却一个完成中断都没有，录到 0 帧。
	 *
	 * 官方两处采集都有这一段，且就放在 enable_finish_interrupt 之后：
	 *   aud_ns_recoder.c:407-410
	 *   onboard_dual_dmic_mic_stream.c:371-374
	 *   bk_avdk_voice_record/.../onboard_mic_record.c:498-501
	 */
	bk_dma_set_dest_sec_attr(s_dma_id, DMA_ATTR_SEC);
	bk_dma_set_src_sec_attr(s_dma_id, DMA_ATTR_SEC);
#endif

	return BK_OK;

fail:
	dmic_rec_dma_deconfig();
	return ret;
}

static void dmic_rec_dma_deconfig(void)
{
	if (s_dma_id < DMA_ID_MAX) {
		bk_dma_stop(s_dma_id);
		bk_dma_deinit(s_dma_id);
		bk_dma_free(DMA_DEV_AUDIO, s_dma_id);
		s_dma_id = DMA_ID_MAX;
	}

	if (s_dma_buf) {
		os_free(s_dma_buf);
		s_dma_buf = NULL;
	}
}

/* 录音任务：等 DMA 帧 -> 32bit word 拆成 16bit PCM -> 写 SD */
static void dmic_rec_task(void *arg)
{
	chry_shell_t *csh = csh_port_shell();
	uint8_t hdr[44];
	FIL fil;
	/* 一帧最多 48000/1000*20 = 960 个采样。放静态区，别占任务栈
	 * （FIL + 960*2 字节一起压在 6K 栈上容易溢出） */
	static int16_t pcm[960];
	uint32_t total_frames, written_frames = 0, drop = 0;
	uint32_t last_cnt = 0, t0, elapsed;
	/* 写盘电平统计：判断"文件是空的"还是"有数据但播不出来" */
	int64_t sum_sq = 0;
	int64_t sum_v = 0;      /* 直流分量 */
	uint32_t nz = 0;        /* 非零采样个数 */
	/* zcr：区分"真声音"和"直流漂移"。只看 rms/peak 会被大直流偏置骗过去。 */
	uint32_t zc = 0;
	int32_t zc_prev = 0;
	bool zc_has_prev = false;
	int32_t peak = 0;
	uint32_t nsamp = 0;
	bool first_dumped = false;
	UINT bw;
	FRESULT fr;
	bk_err_t ret = BK_OK;
	bool opened = false;

	(void)arg;

	/* total_frames 的单位是**采样点**，不是毫秒、也不是 DMA 帧。
	 * 这里原来多除了一个 1000（s_rec_rate * s_rec_sec / 1000），结果
	 * 5s@16k 只写 80 个采样点 = 5ms，ffplay 里显示 Duration 00:00:00.01。
	 * 正确值：16000 * 5 = 80000 个采样点 = 160000 字节。 */
	total_frames = s_rec_rate * s_rec_sec;

	/* 1. 先写 44 字节占位头，后面回填 */
	wav_build_header(hdr, s_rec_rate, 0, 1);
	fr = f_open(&fil, s_rec_path, FA_CREATE_ALWAYS | FA_WRITE);
	if (fr != FR_OK) {
		csh_printf(csh, "dmic: open %s failed: %d\r\n", s_rec_path, fr);
		ret = BK_FAIL;
		goto out;
	}
	opened = true;

	if ((f_write(&fil, hdr, 44, &bw) != FR_OK) || (bw != 44)) {
		csh_printf(csh, "dmic: write header failed\r\n");
		ret = BK_FAIL;
		goto out;
	}

	csh_printf(csh, "dmic: recording %us -> %s (%uHz mono 16bit)\r\n",
	           (unsigned)s_rec_sec, s_rec_path, (unsigned)s_rec_rate);

	/* 打出 DMA 缓冲区信息：buf 应落在内部 SRAM（0x2xxxxxxx）。若显示
	 * 0x6xxxxxxx（PSRAM）也不一定有事——官方 audio_dma_mem_calloc 本身就是
	 * psram_malloc，DMA_DEV_DTCM 配 PSRAM 是允许的。 */
	csh_printf(csh, "      dma id=%d buf=%p frame=%uB (single-slot)\r\n",
	           (int)s_dma_id, (void *)s_dma_buf, (unsigned)s_frame_bytes);

	/* 2. 起顺序必须是 DMA → DMIC → ADC（对齐官方录音卡 aud_ns_recoder.c:626~633）。
	 *
	 * 之前这里写反了（先 dmic_start 再 dma_start）。DMIC 一旦使能就开始往
	 * FIFO 灌数，而 DMA 此时还没 armed，REPEAT 模式首帧完成中断容易丢，
	 * 表现出来就是"录满时长但一帧没有"。 */
	ret = bk_dma_start(s_dma_id);
	if (ret != BK_OK) {
		csh_printf(csh, "dmic: dma start failed: %d\r\n", ret);
		goto out;
	}

	ret = bk_aud_dmic_start();
	if (ret != BK_OK) {
		csh_printf(csh, "dmic: dmic start failed: %d\r\n", ret);
		bk_dma_stop(s_dma_id);
		goto out;
	}

	/* ADC start 与 pol 路径同开关（`dmic cfg adc on|off`）。
	 * 实测 ADC 不 start 时 DMIC 完全不出数，所以默认 ON；
	 * 但 ADC FIFO 没人排空，约 0.5s 后 DMIC 输出会被拖停（待验证/待修）。 */
	if (s_adc_start_en) {
		ret = bk_aud_adc_start();
		if (ret != BK_OK)
			csh_printf(csh, "dmic: adc start failed: %d\r\n", ret);
		ret = BK_OK;

		if (dmic_adc_drain_start(false) != BK_OK)
			csh_printf(csh, "dmic: adc drain dma start failed\r\n");
	}

	s_dma_cnt = 0;
	last_cnt = 0;
	t0 = rtos_get_time();

	/* 3. 边采边写 */
	{
	uint32_t wait_start = t0;
	uint32_t last_got = 0;
	int no_data_warned = 0;

	while (!s_rec_stop && (written_frames < total_frames)) {
		uint32_t idx, i, cnt;

		/* 硬超时：正常情况 5s 的录音 5s 内写满 total_frames 就退出；
		 * 若一帧都收不到（DMA 挂了），这里必须兜住，否则会一直挂到
		 * 用户手动 dmic stop（实测挂过 24s）。 */
		if ((rtos_get_time() - t0) > (uint32_t)(s_rec_sec * 1000 + 2000)) {
			csh_printf(csh, "dmic: 超时退出（%us 窗口内只收到 %u 帧）\r\n",
			           (unsigned)s_rec_sec, (unsigned)written_frames);
			break;
		}

		/* 等一帧（超时给 200ms，避免卡死后 stop 不响应） */
		if (rtos_get_semaphore(&s_dma_sem, 200) != kNoErr) {
			/* 连续 600ms 一个 DMA 完成中断都没有。
			 * 打印三组量，用来区分"通道没启动"/"传输卡住"/"中断丢了"：
			 *   dma_en   : bk_dma_get_enable_status，0 = 通道没在跑
			 *   remain   : 剩余传输长度，两次采样不变 = 完全没搬
			 *   fifo     : bit22=FULL 说明数据在产生但没人搬 */
			if ((rtos_get_time() - wait_start > 600) && (last_got == 0) &&
			    (no_data_warned < 3)) {
				uint32_t st = 0;
				uint32_t en = bk_dma_get_enable_status(s_dma_id);
				uint32_t rl = bk_dma_get_remain_len(s_dma_id);

				bk_aud_dmic_get_status(&st);
				csh_printf(csh, "dmic: no dma frame after 600ms, fifo=0x%08X "
				           "dma_en=%u remain=%u\r\n",
				           (unsigned)st, (unsigned)en, (unsigned)rl);
				if (en == 0)
					csh_printf(csh, "      -> DMA 通道没在跑：检查 bk_dma_start / 通道号\r\n");
				else if (st & AUD_DMIC_FIFO_FULL_MASK)
					csh_printf(csh, "      -> FIFO 已满但 DMA 没搬：怀疑 TZ/SEC 属性挡住总线访问\r\n");
				no_data_warned++;
			}
			continue;
		}

		/* 取一次快照：s_dma_cnt 会被 DMA 中断随时改，多次读会不一致 */
		cnt = s_dma_cnt;
		/* 单帧窗口 + ISR 快照：数据永远在 s_dma_snap[0..s_frame_samples)。
		 * 不再按 (cnt-1)%N 猜槽位 —— 那个假设已被实测证伪。 */
		idx = 0;

		/* 一次完成中断 = 一帧。两次读之间 cnt 跳了，说明中间有帧没来得及
		 * 取走（快照被下一次 ISR 覆盖），按帧记丢。 */
		if ((last_cnt != 0) && (cnt - last_cnt > 1))
			drop += (cnt - last_cnt) - 1;
		last_cnt = cnt;
		last_got = cnt;

		/* 定点体检：0.5s 和 1.0s 各打一次。上一版就是在这两个时间点之间
		 * 从"有声音"变成"恒定直流"的，打出来能直接看到是哪个 FIFO 先异常。 */
		if ((cnt == 25) || (cnt == 50)) {
			uint32_t df = 0, af = 0;

			bk_aud_dmic_get_status(&df);
			bk_aud_adc_get_status(&af);
			csh_printf(csh, "      [t=%ums] dmic_fifo=0x%08X adc_fifo=0x%08X "
			           "dma_en=%u remain=%u\r\n",
			           (unsigned)(cnt * DMIC_REC_FRAME_MS),
			           (unsigned)df, (unsigned)af,
			           (unsigned)bk_dma_get_enable_status(s_dma_id),
			           (unsigned)bk_dma_get_remain_len(s_dma_id));
		}

		/* 第一帧到手时，把 ISR 快照的头 8 个 word 打出来。
		 * 这是判断"DMA 到底搬到了什么"最直接的一手数据，和 `dmic` 轮询
		 * 打印的 raw 是同一格式，可以直接比：
		 *   两边都非 0        → 数据没问题，去查 WAV 头/播放端
		 *   这边全 0          → DMA 搬的是空数据（启动时序、TZ 权限）
		 *   这边有小值/削顶   → 声道选错或增益不对
		 */
		if (!first_dumped) {
			uint32_t k;

			first_dumped = true;
			csh_printf(csh, "      dma raw:");
			for (k = 0; k < 8; k++)
				csh_printf(csh, " %08X",
				           (unsigned)s_dma_snap[idx * s_frame_samples + k]);
			csh_printf(csh, "\r\n");
		}

		for (i = 0; i < s_frame_samples; i++) {
			int32_t v = w_pick(s_dma_snap[idx * s_frame_samples + i]);

			pcm[i] = (int16_t)v;
			sum_sq += (int64_t)v * v;
			sum_v += v;
			if (v != 0)
				nz++;
			if (v > peak)
				peak = v;
			if (v < -peak)
				peak = -v;
			if (zc_has_prev && ((v >= 0) != (zc_prev >= 0)))
				zc++;
			zc_prev = v;
			zc_has_prev = true;
		}
		nsamp += s_frame_samples;

		{
			uint32_t n = s_frame_samples;
			uint32_t bytes;

			if (written_frames + n > total_frames)
				n = total_frames - written_frames;
			bytes = n * 2;

			if ((f_write(&fil, pcm, bytes, &bw) != FR_OK) || (bw != bytes)) {
				csh_printf(csh, "dmic: write failed (卡满?)\r\n");
				ret = BK_FAIL;
				break;
			}
			written_frames += n;
		}
	}
	}   /* end of 采集块作用域 */

	/* 4. 停采集：DMA → DMIC → ADC（与官方相反顺序停） */
	bk_dma_stop(s_dma_id);
	bk_aud_dmic_stop();
	bk_aud_adc_stop();

	elapsed = rtos_get_time() - t0;

	/* 5. 回填长度并落盘 */
	{
		uint32_t data_len = written_frames * 2;

		wav_build_header(hdr, s_rec_rate, data_len, 1);
		f_lseek(&fil, 0);
		if ((f_write(&fil, hdr, 44, &bw) != FR_OK) || (bw != 44))
			csh_printf(csh, "dmic: header rewrite failed\r\n");
		f_sync(&fil);
	}

	/* 统计一律打：用户点 stop 提前结束时更需要看到到底录了多少 */
	csh_printf(csh, "dmic: %u frames / %u bytes, %ums%s\r\n",
	           (unsigned)written_frames, (unsigned)(written_frames * 2),
	           (unsigned)elapsed, s_rec_stop ? " (stopped)" : "");

	if (drop)
		csh_printf(csh, "      dropped %u frame(s)\r\n", (unsigned)drop);
	if (written_frames != total_frames)
		csh_printf(csh, "      expected %u frames, got %u\r\n",
		           (unsigned)total_frames, (unsigned)written_frames);
	if (written_frames == 0)
		csh_printf(csh, "      -> 一帧都没收到：DMIC 没出数据，先跑 'dmic dbg'\r\n");

	/* 写进 WAV 的实际电平。这条是"文件里到底有没有声音"的直接答案：
	 * 与 `dmic` 打印的 L/R 电平对比，就能判断是采集问题还是落盘/播放问题。 */
	if (nsamp > 0) {
		uint32_t zero_pm = (uint32_t)(((uint64_t)(nsamp - nz) * 1000) / nsamp);
		int32_t  dc = (int32_t)(sum_v / (int64_t)nsamp);

		{
			/* zcr = 过零次数 / 采样时长(秒)
			 *      = zc / (nsamp / s_rec_rate)
			 *      = zc * s_rec_rate / nsamp
			 *
			 * 【修正】旧式写成 zc*1000/(nsamp/rate+1)，把"秒数+1"当成了毫秒，
			 * 会把 53/s 放大成 43833/s（超过采样率上限都不自知），严重误导判读。
			 * 过零率上限就是采样率本身，超了一定是公式错。 */
			uint32_t zcr = (uint32_t)((uint64_t)zc *
			                          (uint64_t)(s_rec_rate > 0 ? s_rec_rate : 1) /
			                          (uint64_t)(nsamp > 0 ? nsamp : 1));

			dmic_print_stat(csh, "PCM", sum_sq, peak, (int)nsamp);
			csh_printf(csh, "      dc=%d  zero=%u.%u%%  nonzero=%u/%u\r\n",
			           (int)dc, (unsigned)(zero_pm / 10), (unsigned)(zero_pm % 10),
			           (unsigned)nz, (unsigned)nsamp);
			csh_printf(csh, "      zcr=%u/s  （语音 >100/s；几十 = 直流漂移；0 = 恒定）\r\n",
			           (unsigned)zcr);

			if ((zcr < 60) && (peak > 0))
				csh_printf(csh, "      -> 只有直流/低频漂移，不是声音：麦没在驱动 DATA\r\n");
		}

		if (peak == 0) {
			csh_printf(csh, "      -> 写进 WAV 的采样全是 0：DMA 搬的是空数据\r\n");
		} else if (zero_pm > 700) {
			/* rms 低但 peak 正常，且七成以上采样是 0 —— 典型的"只取到部分
			 * 数据"：DMA 窗口/快照时序没对上，不是麦或增益的问题。 */
			csh_printf(csh, "      -> 七成以上采样为 0：不是增益问题，是取数不完整\r\n");
		} else if (peak < 64) {
			csh_printf(csh, "      -> 电平极低，听不到是正常的：查 DMIC 增益/麦距离\r\n");
		} else if (dc > 3000 || dc < -3000) {
			csh_printf(csh, "      -> 直流偏置过大(%d)：低频轰鸣、听不清，需要高通\r\n", (int)dc);
		}
	}

out:
	if (opened) {
		f_close(&fil);
		if (ret == BK_OK)
			csh_printf(csh, "dmic: saved %s\r\n", s_rec_path);
	}

	/* 收尾：DMA -> DMIC/ADC -> 信号量。deinit 必须做，否则下次
	 * bk_aud_dmic_init() 会因为 "already init" 直接返回失败。
	 * dmic_hw_stop() 里已经带上了 ADC 的 stop/deinit（audio 总电源是
	 * dmic_hw_init() 借 bk_aud_adc_init() 打开的，必须配对释放）。 */
	dmic_rec_dma_deconfig();
	dmic_hw_stop();

	if (s_dma_sem) {
		rtos_deinit_semaphore(&s_dma_sem);
		s_dma_sem = NULL;
	}

	s_rec_state = REC_IDLE;
	s_rec_task = NULL;
	if (s_rec_done_sem)
		rtos_set_semaphore(&s_rec_done_sem);
	rtos_delete_thread(NULL);
}

/* 把用户给的文件名规范成 "1:/xxx.wav" */
static int dmic_rec_build_path(const char *arg, char *out, size_t outlen)
{
	int n;

	if ((arg == NULL) || (arg[0] == '\0'))
		return -1;

	if ((arg[0] == '1') && (arg[1] == ':'))
		n = snprintf(out, outlen, "%s", arg);
	else if (arg[0] == '/')
		n = snprintf(out, outlen, "1:%s", arg);
	else
		n = snprintf(out, outlen, "1:/%s", arg);

	if ((n < 0) || ((size_t)n >= outlen))
		return -1;

	/* 没写扩展名就补 .wav */
	if (strstr(out, ".wav") == NULL) {
		size_t l = strlen(out);

		if ((l + 4) >= outlen)
			return -1;
		strcat(out, ".wav");
	}

	return 0;
}

static int dmic_rec_start(chry_shell_t *csh, uint32_t sec, const char *file)
{
	if ((s_rec_state != REC_IDLE) || (s_rec_task != NULL)) {
		csh_printf(csh, "dmic: already recording, run 'dmic stop' first\r\n");
		return -1;
	}

	if (dmic_rec_build_path(file, s_rec_path, sizeof(s_rec_path)) != 0) {
		csh_printf(csh, "dmic: bad file name\r\n");
		return -1;
	}

	if (dmic_rec_sd_mount(csh) != 0)
		return -1;

	/* 供电：MIC 1V8（P48），录完不关（下次录音更快；要关用 'ldo mic off'） */
	if (!bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8)) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	/* DMIC 初始化 —— 必须走 dmic_hw_init()，不能直接裸调
	 * bk_aud_dmic_init()。
	 *
	 * 原因：全 SDK 只有 bk_aud_adc_init() 会调 sys_drv_aud_aud_en(1)
	 * （aud_adc_driver.c:102），bk_aud_driver_init() 里那句
	 * `sys_hal_aud_aud_en(1)` 是被注释掉的（aud_common_driver.c:374），
	 * 而 bk_aud_dmic_init() 内部只会走到 bk_aud_driver_init()。也就是说
	 * 不做 ADC 预初始化，audio 的模拟总开关根本没人打开，DMIC 数字通路
	 * 配得再对也不会有 PDM 时钟。
	 *
	 * 之前录音路径就是漏了这一步（只调了 bk_aud_dmic_init），所以
	 * `dmic dbg` 里寄存器全对、`dmic rec` 却一帧都收不到。
	 *
	 * 注意这里只 init 不 start：DMIC 的 start 必须排在 bk_dma_start() 之后，
	 * 由录音任务里做。DMA 配置也要在 init 之后，因为要取 FIFO 地址。
	 */
	{
		aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
		bk_err_t ret;

		cfg.samp_rate = s_rec_rate;
		cfg.dmic_chl = AUD_DMIC_CHL_L;
		cfg.clk_src = AUD_CLK_APLL;

		ret = dmic_hw_init(csh, &cfg);
		if (ret != BK_OK) {
			csh_printf(csh, "dmic: init failed: %d\r\n", ret);
			return -1;
		}
	}

	if (rtos_init_semaphore(&s_dma_sem, 1) != kNoErr) {
		csh_printf(csh, "dmic: create dma sem failed\r\n");
		dmic_hw_stop();
		return -1;
	}

	if (dmic_rec_dma_config() != BK_OK) {
		csh_printf(csh, "dmic: dma config failed\r\n");
		rtos_deinit_semaphore(&s_dma_sem);
		s_dma_sem = NULL;
		dmic_hw_stop();
		return -1;
	}

	if (s_rec_done_sem == NULL) {
		if (rtos_init_semaphore(&s_rec_done_sem, 1) != kNoErr) {
			csh_printf(csh, "dmic: create semaphore failed\r\n");
			dmic_rec_dma_deconfig();
			rtos_deinit_semaphore(&s_dma_sem);
			s_dma_sem = NULL;
			dmic_hw_stop();
			return -1;
		}
	} else {
		while (rtos_get_semaphore(&s_rec_done_sem, 0) == kNoErr)
			;
	}

	s_rec_sec = sec;
	s_rec_stop = false;
	s_rec_state = REC_RUNNING;

	if (rtos_create_thread(&s_rec_task, BEKEN_DEFAULT_WORKER_PRIORITY, "dmic_rec",
	                       (beken_thread_function_t)dmic_rec_task,
	                       DMIC_REC_TASK_STACK, NULL) != kNoErr) {
		s_rec_task = NULL;
		s_rec_state = REC_IDLE;
		dmic_rec_dma_deconfig();
		rtos_deinit_semaphore(&s_dma_sem);
		s_dma_sem = NULL;
		dmic_hw_stop();
		csh_printf(csh, "dmic: create task failed\r\n");
		return -1;
	}

	csh_printf(csh, "dmic: started, 'dmic stop' to finish early\r\n");
	return 0;
}

static int dmic_rec_stop(chry_shell_t *csh)
{
	uint32_t waited = 0;

	if ((s_rec_state == REC_IDLE) && (s_rec_task == NULL)) {
		csh_printf(csh, "dmic: not recording\r\n");
		return 0;
	}

	s_rec_stop = true;

	while ((s_rec_task != NULL) && (waited < DMIC_REC_STOP_TO)) {
		if (rtos_get_semaphore(&s_rec_done_sem, 50) == kNoErr)
			break;
		waited += 50;
	}

	if (s_rec_task != NULL) {
		csh_printf(csh, "dmic: stop timeout, task still running\r\n");
		return -1;
	}

	return 0;
}

static void dmic_rec_status(chry_shell_t *csh)
{
	if ((s_rec_state == REC_IDLE) && (s_rec_task == NULL)) {
		csh_printf(csh, "dmic: idle\r\n");
		csh_printf(csh, "      sd %s, MIC 1V8 %s\r\n",
		           s_fs_mounted ? "mounted" : "not mounted",
		           bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8) ? "ON" : "OFF");
		return;
	}

	csh_printf(csh, "dmic: recording %us -> %s\r\n",
	           (unsigned)s_rec_sec, s_rec_path);
	csh_printf(csh, "      %uHz mono 16bit, frame %ums, dma frames=%u\r\n",
	           (unsigned)s_rec_rate, DMIC_REC_FRAME_MS, (unsigned)s_dma_cnt);
}

/* dmic mode [n] —— 直接给 P8/P9 写复用 mode 并立刻回读。
 *
 * 用途：绕过 SDK 的 gpio_dev_map() 验证"到底哪个地址才是复用寄存器"。
 * 不带参数时只读；带参数时先写再读。
 * 同时把 gpio_sys_num[0..6] 七个寄存器原值全打出来，方便对照。 */
static int dmic_mode_cmd(chry_shell_t *csh, int argc, char **argv)
{
	uint32_t base = GPIO_LL_SYSTEM_REG_BASE;
	int i;
	int mode = -1;

	if (argc > 2)
		mode = atoi(argv[2]);

	csh_printf(csh, "dmic mode: GPIO_LL_SYSTEM_REG_BASE=0x%08X\r\n", (unsigned)base);

	if (mode >= 0) {
		dmic_gpio_func_mode_set(BOARD_DMIC_CLK_PIN, (uint32_t)mode);
		dmic_gpio_func_mode_set(BOARD_DMIC_DATA_PIN, (uint32_t)mode);
		csh_printf(csh, "  wrote mode=%d to P%d/P%d\r\n",
		           mode, (int)BOARD_DMIC_CLK_PIN, (int)BOARD_DMIC_DATA_PIN);
	}

	csh_printf(csh, "  read back: P%d=%u P%d=%u\r\n",
	           (int)BOARD_DMIC_CLK_PIN, (unsigned)dmic_gpio_func_mode(BOARD_DMIC_CLK_PIN),
	           (int)BOARD_DMIC_DATA_PIN, (unsigned)dmic_gpio_func_mode(BOARD_DMIC_DATA_PIN));

	csh_printf(csh, "  gpio_sys_num[0..6]:");
	for (i = 0; i < 7; i++)
		csh_printf(csh, " %08X", (unsigned)REG_READ(base + i * 4));
	csh_printf(csh, "\r\n");

	return 0;
}

/* dmic pin —— 硬件层面的引脚判定（必须在外设完全停掉后运行）
 *
 * 为什么需要这条命令：前面 `pad probe` 是在 DMIC 还在运行时测的，
 * 那时 P8 的第二功能没解除，音频外设可能仍在驱动这根线，
 * 读到 0 无法区分「外设驱动低」和「板子对地短路」。
 *
 * 本命令的做法是把一切都关掉之后，**只留内部上拉**，再看电平：
 *   1) 停 DMIC/ADC，释放复用（gpio_dev_unmap），让 pad 回归纯 GPIO；
 *   2) 配成输入 + 内部上拉，读电平；
 *   3) 配成输入 + 内部下拉，读电平。
 *
 * 判读：
 *   上拉读到 1 且下拉读到 0  → pad 跟得上，**没有对地短路**，
 *                              那么之前测到的恒低就是外设驱动的，属软件问题
 *   上拉/下拉都读到 0        → 有强下拉源，**疑似对地短路或焊锡桥连**
 *   上拉/下拉都读到 1        → 有强上拉源，疑似短到 VCC
 */
static int dmic_pin_cmd(chry_shell_t *csh)
{
	gpio_hw_t *hw = (gpio_hw_t *)GPIO_LL_REG_BASE;
	uint32_t pins[2];
	int i;

	pins[0] = BOARD_DMIC_CLK_PIN;
	pins[1] = BOARD_DMIC_DATA_PIN;

	/* 1. 彻底停掉音频外设，并解除复用 */
	dmic_hw_stop();
	gpio_dev_unmap(BOARD_DMIC_CLK_PIN);
	gpio_dev_unmap(BOARD_DMIC_DATA_PIN);
	rtos_delay_milliseconds(2);

	csh_printf(csh, "dmic pin: audio 已停、复用已释放，仅留 GPIO + 内部上下拉\r\n");

	for (i = 0; i < 2; i++) {
		uint32_t pad = pins[i];
		const char *name = (i == 0) ? "CLK" : "DAT";
		int pu, pd;

		/* 纯输入 */
		hw->gpio_num[pad].cfg.gpio_2_func_en = 0;
		hw->gpio_num[pad].cfg.gpio_output_en = 0;
		hw->gpio_num[pad].cfg.gpio_input_en = 1;
		hw->gpio_num[pad].cfg.gpio_input_monitor = 1;

		/* 内部上拉 */
		hw->gpio_num[pad].cfg.gpio_pull_mode_en = 1;
		hw->gpio_num[pad].cfg.gpio_pull_mode = 1;
		rtos_delay_milliseconds(5);
		pu = hw->gpio_num[pad].cfg.gpio_input ? 1 : 0;

		/* 内部下拉 */
		hw->gpio_num[pad].cfg.gpio_pull_mode = 0;
		rtos_delay_milliseconds(5);
		pd = hw->gpio_num[pad].cfg.gpio_input ? 1 : 0;

		csh_printf(csh, "  P%u(%s): pullup->%d pulldown->%d  ",
		           (unsigned)pad, name, pu, pd);
		if (pu == 1 && pd == 0)
			csh_printf(csh, "正常（可被拉动，无短路）\r\n");
		else if (pu == 0 && pd == 0)
			csh_printf(csh, "恒低 -> 被强下拉/对地短路\r\n");
		else
			csh_printf(csh, "恒高 -> 被强上拉/短到 VCC\r\n");

		/* 推挽输出测试：能输出高并读回 1，说明 pad 没被硬短路到 GND；
		 * 输出高读回还是 0，就是真的对地短（或外部有 <10 欧的强下拉）。 */
		{
			int oh, ol;

			hw->gpio_num[pad].cfg.gpio_pull_mode_en = 0;
			hw->gpio_num[pad].cfg.gpio_input_en = 1;
			hw->gpio_num[pad].cfg.gpio_input_monitor = 1;
			hw->gpio_num[pad].cfg.gpio_output_en = 1;

			hw->gpio_num[pad].cfg.gpio_output = 1;
			rtos_delay_milliseconds(2);
			oh = hw->gpio_num[pad].cfg.gpio_input ? 1 : 0;

			hw->gpio_num[pad].cfg.gpio_output = 0;
			rtos_delay_milliseconds(2);
			ol = hw->gpio_num[pad].cfg.gpio_input ? 1 : 0;

			hw->gpio_num[pad].cfg.gpio_output_en = 0;

			csh_printf(csh, "        推挽: 输出1读回=%d 输出0读回=%d  -> %s\r\n",
			           oh, ol,
			           (oh == 1 && ol == 0) ? "pad 正常，能驱动到两种电平" :
			           (oh == 0 && ol == 0) ? "输出高都被拉成 0：对地短路/极强下拉" :
			           (oh == 1 && ol == 1) ? "输出低都被拉成 1：对 VCC 短路/上拉" :
			                                  "异常");
		}

		/* 恢复成"无上下拉、输入"的干净状态，避免影响后续 */
		hw->gpio_num[pad].cfg.gpio_pull_mode_en = 0;
		hw->gpio_num[pad].cfg.gpio_input_en = 0;
		hw->gpio_num[pad].cfg.gpio_input_monitor = 0;
	}

	csh_printf(csh, "  说明：两个脚都正常 -> 之前的恒低是外设驱动，属软件问题；\r\n");
	csh_printf(csh, "        任一脚恒低   -> 用万用表量该脚对 GND 电阻，确认是否短路\r\n");

	return 0;
}

/* ---------------- 新增自检命令（诊断 DMIC 不出数的问题） ---------------- */

/* dmic pm —— 看 audio 电源域相关信息，并走一遍 SDK 的 PM 投票 / 时钟使能。
 *
 * 说明（之前这条命令的提示是错的，已改正）：
 *   bk_pm_module_power_state_get(PM_POWER_SUB_MODULE_NAME_AUDP_AUDIO) 读的是
 *   SYS 的 cpu_power_sleep_wakeup 位图（bit6），含义是「深睡时是否关掉 AUDP
 *   电源域」，**不是当前是否上电**。所以 bit=1 不代表模块现在是关的，
 *   往这个位图里写 0 也不会把模块打开。
 *
 *   真正让 CP 打开 audio 电源/时钟的是 bk_pm_module_vote_power_ctrl() 和
 *   bk_pm_clock_ctrl()（都是发邮箱给 CP，超时只会打 BK_LOGD）。 */
static int dmic_pm_cmd(chry_shell_t *csh)
{
	int32_t sleep_bit = bk_pm_module_power_state_get(PM_POWER_SUB_MODULE_NAME_AUDP_AUDIO);
	bk_err_t ret;

	csh_printf(csh, "dmic pm: PM sleep bitmap(AUDP, bit6)=%d（1 = 深睡时关该域，不是当前电源状态）\r\n",
	           (int)sleep_bit);

	/* 真正请求 CP 打开 audio 电源域 */
	ret = bk_pm_module_vote_power_ctrl(PM_POWER_SUB_MODULE_NAME_AUDP_AUDIO,
	                                   PM_POWER_MODULE_STATE_ON);
	csh_printf(csh, "  vote AUDP power ON -> %d（0=OK；邮箱超时这里看不到）\r\n", ret);

	/* audio APB 时钟 + 时钟源（APLL），这两个是 AP 侧 SYS 寄存器，能直接读回 */
	sys_drv_aud_clock_en(1);
	sys_drv_aud_select_clock(1);
	rtos_delay_milliseconds(2);

	csh_printf(csh, "  clk: cksel_aud=%u aud_cken=%u dmic_div_en=%u\r\n",
	           (unsigned)sys_ll_get_cpu_clk_div_mode1_cksel_aud(),
	           (unsigned)sys_ll_get_cpu_device_clk_enable_aud_cken(),
	           (unsigned)sys_ll_get_cpu_clk_div_mode2_reserved_13_13());
	csh_printf(csh, "  apll: ana_reg5=0x%08X pwdaudpll=%u (%s) ana_reg25=0x%08X\r\n",
	           (unsigned)REG_READ(SOC_SYS_REG_BASE + (0x45 << 2)),
	           (unsigned)sys_ll_get_ana_reg5_pwdaudpll(),
	           sys_ll_get_ana_reg5_pwdaudpll() ? "POWER DOWN!" : "on",
	           (unsigned)REG_READ(SOC_SYS_REG_BASE + (0x25 << 2)));
	return 0;
}

/* 单独跑 ADC 类探针时用：把 ADC + DMIC 按默认参数起一遍 */
static bk_err_t dmic_audio_setup(chry_shell_t *csh, aud_clk_t clk_src)
{
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();

	if (!bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8)) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	cfg.samp_rate = s_dmic_rate;
	cfg.dmic_chl = AUD_DMIC_CHL_L;
	cfg.clk_src = clk_src;

	return dmic_hw_start(csh, &cfg);
}

static void dmic_audio_teardown(void)
{
	bk_aud_dmic_stop();
	bk_aud_dmic_deinit();
	bk_aud_adc_deinit();
}

/* dmic adc —— 用 ADC 反证 audio 时钟：ADC 能读出 sample 就说明音频时钟在跑 */
static int dmic_adc_probe(chry_shell_t *csh)
{
	uint32_t st = 0, w = 0, got = 0;
	int i;

	if (bk_aud_adc_start() != BK_OK) {
		csh_printf(csh, "dmic adc: adc start failed\r\n");
		return -1;
	}

	rtos_delay_milliseconds(20);
	for (i = 0; i < 64; i++) {
		bk_aud_adc_get_status(&st);
		if (st & (1u << 14)) {          /* adcl fifo empty */
			if (got == 0)
				rtos_delay_milliseconds(1);
			continue;
		}
		if (bk_aud_adc_get_fifo_data(&w) != BK_OK)
			break;
		got++;
	}
	bk_aud_adc_stop();

	csh_printf(csh, "dmic adc: fifo=0x%08X got=%u -> audio clock %s\r\n",
	           (unsigned)st, (unsigned)got,
	           (got > 0) ? "在跑（DMIC 问题不在时钟树）" :
	                       "没在跑（APLL/audio 时钟树有问题）");
	return (got > 0) ? 0 : -1;
}

/* dmic digmicsel —— adc_config0[25] 的 dig_mic_sel 调试位：
 * 置 1 让数字麦数据走 ADC 通路，看 ADC FIFO 能不能读到 */
static int dmic_dig_mic_sel_cmd(chry_shell_t *csh)
{
	uint32_t st = 0, w = 0, got = 0, first = 0, last = 0;
	int i;

	csh_printf(csh, "dmic digmicsel: adc_config0.dig_mic_sel=1（数字麦数据走 ADC 通路）\r\n");
	aud_ll_set_adc_config0_dig_mic_sel(1);

	if (bk_aud_adc_start() == BK_OK) {
		rtos_delay_milliseconds(20);
		for (i = 0; i < 64; i++) {
			bk_aud_adc_get_status(&st);
			if (st & (1u << 14)) {
				if (got == 0)
					rtos_delay_milliseconds(1);
				continue;
			}
			if (bk_aud_adc_get_fifo_data(&w) != BK_OK)
				break;
			if (got == 0)
				first = w;
			last = w;
			got++;
		}
		bk_aud_adc_stop();
	} else {
		csh_printf(csh, "  adc start failed\r\n");
	}

	aud_ll_set_adc_config0_dig_mic_sel(0);

	csh_printf(csh, "  got=%u first=0x%08X last=0x%08X -> %s\r\n",
	           (unsigned)got, (unsigned)first, (unsigned)last,
	           (got == 0) ? "还是没数据" :
	           ((first != last) ? "有数据且在变化！DMIC 数据能走 ADC 通路" :
	                              "有数据但恒定"));
	return (got > 0) ? 0 : -1;
}

/* bit-bang 循环：拉高 -> 采样(高相位) -> 拉低 -> 采样(低相位)。
 * 两颗麦共用 DATA：SEL=VDD 的在时钟高相位驱动、SEL=GND 的在低相位驱动，
 * 所以两个相位分别采样就能分别看到两颗麦的 PDM 密度。
 *
 * 注意：这里**直接写/读 GPIO 寄存器**（而不是用 cfg.gpio_output 位域）。
 * 位域赋值会编译成"读-改-写"，每次要两次 AON 总线访问，实测只能跑到
 * ~350kHz，掉进麦的 Low Power 区间（150~900kHz），不是标准模式。
 * 直接读写把每次访问压到一次，才能进标准模式（1300~4800kHz）。 */
static void dmic_bitbang_loop(uint32_t cycles, uint32_t iters,
                              uint32_t *ones_hi, uint32_t *ones_lo)
{
	volatile uint32_t *clk_reg = (volatile uint32_t *)(GPIO_LL_REG_BASE +
	                                                   (uint32_t)BOARD_DMIC_CLK_PIN * 4);
	volatile uint32_t *dat_reg = (volatile uint32_t *)(GPIO_LL_REG_BASE +
	                                                   (uint32_t)BOARD_DMIC_DATA_PIN * 4);
	uint32_t base = *clk_reg;
	uint32_t v_hi = base | 0x2;        /* gpio_output = 1 */
	uint32_t v_lo = base & ~0x2u;
	volatile int d;
	uint32_t i;

	*ones_hi = 0;
	*ones_lo = 0;

	for (i = 0; i < cycles; i++) {
		*clk_reg = v_hi;
		for (d = 0; d < (int)iters; d++)
			;
		*ones_hi += (*dat_reg & 0x1u);
		for (d = 0; d < (int)iters; d++)
			;
		*clk_reg = v_lo;
		for (d = 0; d < (int)iters; d++)
			;
		*ones_lo += (*dat_reg & 0x1u);
		for (d = 0; d < (int)iters; d++)
			;
	}
}

/* dmic bitbang —— 完全绕开 SoC 的 DMIC 模块：用 GPIO 自己给麦打 1.3~4.8MHz
 * 的 PDM 时钟（频率自动校准并打印），DATA 打开内部下拉，高低相位分别统计
 * PDM 密度，用来判断"麦本身 + 供电 + 走线"是否正常。 */
static int dmic_bitbang_cmd(chry_shell_t *csh)
{
	gpio_hw_t *hw = (gpio_hw_t *)GPIO_LL_REG_BASE;
	uint32_t iters = 6;
	uint32_t f_khz = 0;
	uint32_t ones_hi = 0, ones_lo = 0;
	uint32_t t0, dt;
	bool mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	int k;

	/* 麦要有电才会驱动 DATA；板上麦实测是 3V3 直供，这里使能 1V8 轨只是兜底 */
	if (!mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	/* 先停掉 audio 外设、把 P8/P9 从 DMIC 复用里解出来当普通 GPIO */
	dmic_hw_stop();
	gpio_dev_unmap(BOARD_DMIC_CLK_PIN);
	gpio_dev_unmap(BOARD_DMIC_DATA_PIN);

	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_2_func_en = 0;
	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_input_en = 0;
	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_input_monitor = 0;
	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_output_en = 1;
	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_pull_mode_en = 0;
	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_output = 0;

	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_2_func_en = 0;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_output_en = 0;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_input_en = 1;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_input_monitor = 1;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_pull_mode_en = 1;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_pull_mode = 0;   /* 内部下拉 */

	/* 校准到麦克风标准模式要求的 1300~4800kHz。
	 * 注意 iters 允许降到 0：每相位只剩"一次寄存器写 + 一次寄存器读"，
	 * 这是软件打时钟的极限频率。之前限制 iters>=1，结果只能到 ~444kHz，
	 * 掉进麦的 Low Power 区间（150~900kHz），测出来的 0% 不能算数。 */
	for (k = 0; k < 6; k++) {
		uint32_t o1 = 0, o2 = 0;

		t0 = rtos_get_time();
		dmic_bitbang_loop(8000, iters, &o1, &o2);
		dt = rtos_get_time() - t0;

		if (dt == 0) {          /* 1ms 内跑完 8000 周期，太快 */
			iters *= 4;
			if (iters > 4096)
				iters = 4096;
			continue;
		}

		f_khz = 8000 / dt;
		if (f_khz < 1300) {
			if (iters == 0)
				break;      /* 已经是最快 */
			iters = (iters > 1) ? (iters / 2) : 0;
			continue;
		}
		if (f_khz > 4800) {
			iters = (iters < 1) ? 1 : (iters * 2);
			continue;
		}
		break;
	}

	dmic_bitbang_loop(4000, iters, &ones_hi, &ones_lo);

	csh_printf(csh, "dmic bitbang: iters=%u clk≈%u kHz, PDM 密度 高相位=%u%% 低相位=%u%%\r\n",
	           (unsigned)iters, (unsigned)f_khz,
	           (unsigned)(ones_hi * 100 / 4000), (unsigned)(ones_lo * 100 / 4000));
	if (f_khz < 1300) {
		csh_printf(csh, "  注意：clk %u kHz 低于手册 Standard 下限 1300kHz（落在 Low Power 区间），"
		                "本次结果只作参考\r\n", (unsigned)f_khz);
	}

	if ((ones_hi < 400) && (ones_lo < 400)) {
		csh_printf(csh, "  -> 两个相位都接近 0：麦没在驱动 DATA（供电/焊接/麦坏）\r\n");
	} else if ((ones_hi > 3600) && (ones_lo > 3600)) {
		csh_printf(csh, "  -> 两个相位都接近 100%：DATA 被拉死高（短路/接触问题）\r\n");
	} else {
		csh_printf(csh, "  -> DATA 上有 PDM 密度：麦是活的，问题在 SoC 的 DMIC 模块侧\r\n");
	}

	/* 恢复成普通输入，P8/P9 保持 GPIO；下次 dmic 命令会重新 map 回 DMIC */
	hw->gpio_num[BOARD_DMIC_CLK_PIN].cfg.gpio_output_en = 0;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_pull_mode_en = 0;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_input_en = 0;
	hw->gpio_num[BOARD_DMIC_DATA_PIN].cfg.gpio_input_monitor = 0;

	if (!mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);

	return 0;
}

/* dmic diag —— 一把梭：
 *   PM/时钟 → (APLL) AUD/GPIO/FIFO 自检 + ADC 时钟 + dig_mic_sel
 *   → (XTAL) ADC 时钟对照 → bitbang 硬件真值 */
static int dmic_diag_cmd(chry_shell_t *csh)
{
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
	bk_err_t ret;
	int adc_ap = -1, adc_xtal = -1;

	csh_printf(csh, "===== dmic diag =====\r\n");

	dmic_pm_cmd(csh);

	/* 麦供电（板上实测 3V3 直供，这里的 1V8 轨只是兜底） */
	if (!bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8)) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	/* ---------- 第一轮：APLL 时钟源 ---------- */
	cfg.samp_rate = s_dmic_rate;
	cfg.dmic_chl = AUD_DMIC_CHL_L;
	cfg.clk_src = AUD_CLK_APLL;

	ret = dmic_hw_start(csh, &cfg);
	if (ret != BK_OK) {
		csh_printf(csh, "dmic diag: hw start failed: %d\r\n", ret);
		return -1;
	}

	rtos_delay_milliseconds(20);
	dmic_dump_regs(csh);        /* AUD 寄存器 / APLL / 时钟树 / GPIO 复用 / pad / FIFO */
	adc_ap = dmic_adc_probe(csh);
	dmic_dig_mic_sel_cmd(csh);

	bk_aud_dmic_stop();
	bk_aud_dmic_deinit();
	bk_aud_adc_deinit();

	/* ---------- 第二轮：XTAL 26MHz 时钟源对照（APLL 起不来的话这里会有数） ---------- */
	csh_printf(csh, "-- XTAL(26MHz) 时钟源对照 --\r\n");
	if (dmic_audio_setup(csh, AUD_CLK_XTAL) == BK_OK) {
		rtos_delay_milliseconds(20);
		adc_xtal = dmic_adc_probe(csh);
		dmic_audio_teardown();
	} else {
		csh_printf(csh, "dmic diag: XTAL setup failed\r\n");
	}

	if ((adc_ap == 0) || (adc_xtal == 0)) {
		csh_printf(csh, "  -> audio 时钟是活的（APLL:%s XTAL:%s）\r\n",
		           (adc_ap == 0) ? "有数" : "没数",
		           (adc_xtal == 0) ? "有数" : "没数");
	} else {
		csh_printf(csh, "  -> APLL/XTAL 都没数：audio 模块整体没跑（电源/复位），不是 DMIC 单独问题\r\n");
	}

	/* 最后完全绕开 DMIC 模块，用 GPIO 打时钟测麦本身 */
	dmic_bitbang_cmd(csh);

	csh_printf(csh, "===== diag end =====\r\n");
	return 0;
}

/* dmic ref [ms] [rate] —— 1:1 复刻官方"录音卡"例程的 DMIC 打开流程，
 * 用来和本文件其它路径做 A/B 对照。
 *
 * 参照 bk_project_voice_record/main/aud_ns_recoder.c 的 DMIC 分支
 * （Dadc_ch > 0，第 592~633 行），它做的事只有这些：
 *
 *   adc.chl = AUD_ADC_CHL_L, adc.rate = ADC_FS(16000), clk = APLL
 *   bk_aud_adc_init(&adc)          ← 唯一会 sys_drv_aud_aud_en(1) 的路径
 *   dmic.rate = 16000, chl = AUD_DMIC_CHL_L, clk = APLL
 *   bk_aud_dmic_init(&dmic)        （内部 gpio_dev_map P8/P9 + 配 APLL/div）
 *   bk_dma_start() → bk_aud_dmic_start() → bk_aud_adc_start()
 *
 * 注意它**不做**这几件事，本命令也刻意不做（这正是它的对照价值）：
 *   - 不清 clk_control.soft_reset（保持 0x1）
 *   - 不改 clk_gate
 *   - 不额外调 sys_drv_aud_clock_en()
 *
 * 默认 rate 取当前 `dmic rate` 的值，保证和 `dmic` 同条件对比；
 * 想复刻官方原始配置就显式写 `dmic ref 200 16000`。
 *
 * 2026-11-12 实测（两轮）：
 *   dmic ref            → 3200/3200, L rms=8499 R rms=6605
 *   dmic ref 200 32000  → 6400/6400, L rms=6461 R rms=4583
 *   两个速率都正常 → 早期"32k 不行"的判断是错的，见 DMIC_SAMP_RATE 注释。
 */
static int dmic_ref_cmd(chry_shell_t *csh, int ms, int rate)
{
	aud_adc_config_t adc_cfg = DEFAULT_AUD_ADC_CONFIG();
	aud_dmic_config_t cfg = DEFAULT_AUD_DMIC_CONFIG();
	uint32_t w = 0, st = 0, deadline, first[8];
	int got = 0, total, nfirst = 0, i;
	int64_t sum_l = 0, sum_r = 0;
	int32_t pk_l = 0, pk_r = 0;
	bool mic_was_on;
	bk_err_t ret;

	if (rate <= 0)
		rate = 16000;
	if (ms < DMIC_MIN_MS)
		ms = DMIC_MIN_MS;
	if (ms > DMIC_MAX_MS)
		ms = DMIC_MAX_MS;

	mic_was_on = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
	if (!mic_was_on) {
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
		rtos_delay_milliseconds(5);
	}

	csh_printf(csh, "dmic ref: 官方录音卡流程 %dHz APLL, %dms\r\n", rate, ms);

	/* 1. ADC 先 init —— 全 SDK 只有这条路径打开 audio 模拟总开关 */
	adc_cfg.adc_chl = AUD_ADC_CHL_L;
	adc_cfg.samp_rate = (uint32_t)rate;
	adc_cfg.clk_src = AUD_CLK_APLL;
	ret = bk_aud_adc_init(&adc_cfg);
	if (ret != BK_OK)
		csh_printf(csh, "  adc init: %d（已 init 过则继续）\r\n", ret);

	/* 2. DMIC init */
	cfg.samp_rate = (uint32_t)rate;
	cfg.dmic_chl = AUD_DMIC_CHL_L;
	cfg.clk_src = AUD_CLK_APLL;
	ret = bk_aud_dmic_init(&cfg);
	if (ret != BK_OK) {
		csh_printf(csh, "  dmic init failed: %d\r\n", ret);
		goto out;
	}

	/* 注：官方这里还有一句 i2s_mclk_close()，它清的是 0x440100CC
	 * （gpio_config3，即 P24~P31 的复用）里 P28 的 mode。本板 P28 是
	 * ADC4 按键分压，不属于 I2S，不能照抄，跳过。 */

	/* 3. 起：dmic_start → adc_start（官方无 DMA 时的顺序） */
	ret = bk_aud_dmic_start();
	if (ret != BK_OK) {
		csh_printf(csh, "  dmic start failed: %d\r\n", ret);
		goto out;
	}
	(void)bk_aud_adc_start();

	rtos_delay_milliseconds(20);   /* 等 CIC 建立 */

	/* 4. 轮询 FIFO */
	total = rate * ms / 1000;
	deadline = rtos_get_time() + (uint32_t)ms + 300;
	while ((got < total) && (rtos_get_time() < deadline)) {
		bk_aud_dmic_get_status(&st);
		if (st & AUD_DMIC_FIFO_EMPTY_MASK) {
			rtos_delay_milliseconds(1);
			continue;
		}
		bk_aud_dmic_get_fifo_data(&w);
		if (nfirst < (int)(sizeof(first) / sizeof(first[0])))
			first[nfirst++] = w;

		{
			int32_t l = w_low(w);
			int32_t r = w_high(w);

			sum_l += (int64_t)l * l;
			sum_r += (int64_t)r * r;
			if (l > pk_l) pk_l = l;
			if (l < -pk_l) pk_l = -l;
			if (r > pk_r) pk_r = r;
			if (r < -pk_r) pk_r = -r;
		}
		got++;
	}

	csh_printf(csh, "  got %d/%d frames, fifo=0x%08X\r\n",
	           got, total, (unsigned)st);
	if (nfirst > 0) {
		csh_printf(csh, "  raw:");
		for (i = 0; i < nfirst; i++)
			csh_printf(csh, " %08X", (unsigned)first[i]);
		csh_printf(csh, "\r\n");
		dmic_print_stat(csh, "L", sum_l, pk_l, got);
		dmic_print_stat(csh, "R", sum_r, pk_r, got);
	}

	if (got == 0)
		csh_printf(csh, "  -> 官方流程也拿不到数据：与启动顺序/ADC/采样率无关，\r\n"
		                "     下一步查 P8 时钟、MIC 供电、P9 接线（配 'dmic bitbang' 对照）\r\n");
	else
		csh_printf(csh, "  -> 官方流程有数据：DMIC 通路 OK。若 'dmic' 同时没数，\r\n"
		                "     差异就出在 dmic_hw_init 的补丁/顺序上（用 'dmic patch on|off' 二分）\r\n");

out:
	bk_aud_adc_stop();
	bk_aud_dmic_stop();
	bk_aud_dmic_deinit();
	bk_aud_adc_deinit();
	if (!mic_was_on)
		bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);

	return (got > 0) ? 0 : -1;
}

/* dmic tone [hz] [ms] —— 用内部 DAC + 功放放一段正弦音。
 *
 * 这条不是 DMIC 的测试，而是用来判断"整个 audio 通路到底能不能工作"：
 * 听得到声音 → audio 电源域/时钟/APLL/DAC 都正常，那 DMIC 出不了数就是
 * DMIC 模块或麦克风侧的问题；听不到声音 → 整个 audio 模块没跑，
 * 先解决电源/时钟，再谈 DMIC。 */
static int dmic_tone_cmd(chry_shell_t *csh, int argc, char **argv)
{
	/* sin(2*pi*i/64) * 12000，64 点查表 */
	static const int16_t sine64[64] = {
	0, 1176, 2341, 3483, 4592, 5657, 6667, 7613,
	8485, 9276, 9978, 10583, 11087, 11483, 11769, 11942,
	12000, 11942, 11769, 11483, 11087, 10583, 9978, 9276,
	8485, 7613, 6667, 5657, 4592, 3483, 2341, 1176,
	0, -1176, -2341, -3483, -4592, -5657, -6667, -7613,
	-8485, -9276, -9978, -10583, -11087, -11483, -11769, -11942,
	-12000, -11942, -11769, -11483, -11087, -10583, -9978, -9276,
	-8485, -7613, -6667, -5657, -4592, -3483, -2341, -1176
	};
	bsp_audio_cfg_t cfg;
	int16_t buf[160];              /* 10ms @16kHz 单声道 */
	uint32_t freq = 1000;
	uint32_t dur_ms = 1500;
	uint32_t inc, phase = 0;
	uint32_t frames, i;
	bk_err_t ret;

	if (argc > 2)
		freq = (uint32_t)atoi(argv[2]);
	if (argc > 3)
		dur_ms = (uint32_t)atoi(argv[3]);
	if (freq < 100)
		freq = 100;
	if (freq > 4000)
		freq = 4000;
	if (dur_ms < 100)
		dur_ms = 100;
	if (dur_ms > 5000)
		dur_ms = 5000;

	bsp_audio_default_cfg(&cfg);
	cfg.samp_rate = 16000;
	cfg.channels = 1;
	cfg.volume = 70;
	cfg.frame_size = sizeof(buf);

	ret = bsp_audio_init(&cfg);
	if (ret != BK_OK) {
		csh_printf(csh, "dmic tone: audio init failed: %d\r\n", ret);
		return -1;
	}

	inc = (uint32_t)(((uint64_t)freq << 32) / 16000);
	frames = dur_ms / 10;
	csh_printf(csh, "dmic tone: %uHz %ums（应该听到持续蜂鸣；听不到说明 audio 通路没工作）\r\n",
	           (unsigned)freq, (unsigned)dur_ms);

	for (; frames > 0; frames--) {
		for (i = 0; i < sizeof(buf) / sizeof(buf[0]); i++) {
			buf[i] = sine64[(phase >> 26) & 0x3F];
			phase += inc;
		}
		if (bsp_audio_write(buf, sizeof(buf)) != BK_OK)
			break;
	}

	bsp_audio_stop();
	csh_printf(csh, "dmic tone: done\r\n");
	return 0;
}

/* ---------------- 命令入口 ---------------- */

static void dmic_usage(chry_shell_t *csh)
{
	csh_printf(csh, "usage:\r\n");
	csh_printf(csh, "  dmic [ms] [apll|xtal] [rate]  capture 10..%dms, print level (APLL 没数自动试 XTAL)\r\n", DMIC_MAX_MS);
	csh_printf(csh, "  dmic diag               一把梭自检：电源域/AUD/GPIO/FIFO/ADC时钟/dig_mic_sel/bitbang\r\n");
	csh_printf(csh, "  dmic ref [ms] [rate]    按官方录音卡流程打开 DMIC（默认 200ms / 16000Hz）\r\n");
	csh_printf(csh, "  dmic patch [on|off|m]   补丁位：0x1 aud_cken / 0x2 soft_reset / 0x4 clk_gate（默认 0x1）\r\n");
	csh_printf(csh, "  dmic cfg adc on|off     是否在 DMIC start 后也 start ADC（默认 on）\r\n");
	csh_printf(csh, "  dmic cfg freq on|off    是否投 PM_CPU_FRQ_480M 频率票（默认 off）\r\n");
	csh_printf(csh, "  dmic cfg drain on|off   ADC FIFO 是否用 DMA 排空（默认 off）\r\n");
	csh_printf(csh, "  dmic adk [ms]           ★ 用官方 onboard_dual_dmic_mic_stream 组件采集（默认 2000ms）\r\n");
	csh_printf(csh, "                          自带 ADC FIFO 排空 DMA / 官方 ring buffer，无需 dmic cfg drain\r\n");
	csh_printf(csh, "  dmic adk rec [sec] [f]  ★ 组件采集并录成 WAV 存 SD 卡（默认 5s，默认 1:/adk.wav）\r\n");
	csh_printf(csh, "                          录音同样受 dmic shift 影响，先用 shift 3（+18dB）再录\r\n");
	csh_printf(csh, "  dmic adk stereo [ms]    ★ 双声道（L/R 分开统计），查两颗麦是否对称\r\n");
	csh_printf(csh, "  dmic adk rec 5 /adk_lr.wav stereo   录 2 声道 WAV（可省略 1: 前缀）\r\n");
	csh_printf(csh, "                          flat%% 接近 100%% 说明该路没有有效 PDM 流（虚焊/未供电）\r\n");
	csh_printf(csh, "  dmic pwr [on|off]       MIC 1V8 开关；'dmic pwr off' + 'dmic adk stereo 3000 np'\r\n");
	csh_printf(csh, "                          = 断电反证：电平塌到 0 说明噪声来自这颗麦\r\n");
	csh_printf(csh, "  dmic mon [ms]           只启动 DMIC 并等待，不做采集（参数=毫秒，默认 3000）\r\n");
	csh_printf(csh, "                          看 0.5s 后是否冻结请用 'dmic mon 3000'（不是 10）\r\n");
	csh_printf(csh, "  dmic rate [n]           查看/设置采样率（16k/32k/48k，免重烧）\r\n");
	csh_printf(csh, "  dmic gain [n]           查看/设置 ADC 数字增益 0~0x3f（实测对 DMIC 基本无效）\r\n");
	csh_printf(csh, "  dmic shift [n]          ★ DMIC 专用增益，左移放大（+1=6dB），默认 0\r\n");
	csh_printf(csh, "  dmic pick [h|l]         选择录音取高/低 16bit（默认 l：低半字有变化）\r\n");
	csh_printf(csh, "  dmic pm                 PM sleep 位图 + 走一遍 PM 投票/时钟使能 + APLL 状态\r\n");
	csh_printf(csh, "  dmic adc [apll|xtal]    用 ADC 反证 audio 时钟是否在跑（默认 APLL）\r\n");
	csh_printf(csh, "  dmic digmicsel          adc_config0.dig_mic_sel 调试位试验\r\n");
	csh_printf(csh, "  dmic bitbang            GPIO 自己打 PDM 时钟，测麦的 PDM 密度（硬件真值）\r\n");
	csh_printf(csh, "  dmic tone [hz] [ms]     用 DAC+功放放一段正弦音，验证整个 audio 通路\r\n");
	csh_printf(csh, "  dmic raw                dump 16 raw fifo words\r\n");
	csh_printf(csh, "  dmic dbg                register/gpio/power self-check\r\n");
	csh_printf(csh, "  dmic xtal               same as dbg but force XTAL clock src\r\n");
	csh_printf(csh, "  dmic mode [n]           read/write P8/P9 pinmux mode\r\n");
	csh_printf(csh, "  dmic pin                stop audio then probe P8/P9 pull-up (hw check)\r\n");
	csh_printf(csh, "  dmic rec [sec] [file]   record to sd wav (default %ds, 1:/dmic.wav)\r\n",
	           DMIC_REC_DEF_SEC);
	csh_printf(csh, "  dmic stop               stop recording\r\n");
	csh_printf(csh, "  dmic status             show recording state\r\n");
}

int dmic_cmd(int argc, char **argv)
{
	chry_shell_t *csh = (void *)argv[argc + 1];
	int ms = 200;
	aud_clk_t clk = AUD_CLK_APLL;

	if (csh == NULL)
		return -1;

	if (argc > 1) {
		if (!strcmp(argv[1], "raw"))
			return dmic_dump_raw(csh);

		if (!strcmp(argv[1], "dbg"))
			return dmic_dbg(csh);

		if (!strcmp(argv[1], "xtal"))
			return dmic_dbg_ex(csh, AUD_CLK_XTAL);

		if (!strcmp(argv[1], "mode"))
			return dmic_mode_cmd(csh, argc, argv);

		if (!strcmp(argv[1], "pin"))
			return dmic_pin_cmd(csh);

		if (!strcmp(argv[1], "diag"))
			return dmic_diag_cmd(csh);

		if (!strcmp(argv[1], "ref")) {
			int rms = 200;
			int rate = (int)s_dmic_rate;   /* 与 dmic 用同一速率，便于同条件对比 */

			if (argc > 2)
				rms = atoi(argv[2]);
			if (argc > 3)
				rate = atoi(argv[3]);
			return dmic_ref_cmd(csh, rms, rate);
		}

		if (!strcmp(argv[1], "patch")) {
			if (argc > 2) {
				if (!strcmp(argv[2], "on"))
					s_patch_mask = PATCH_ADCKEN | PATCH_SOFTRST | PATCH_CLKGATE;
				else if (!strcmp(argv[2], "off"))
					s_patch_mask = 0;
				else
					s_patch_mask = (uint32_t)strtoul(argv[2], NULL, 0) & 0x7u;
			} else {
				s_patch_mask = (s_patch_mask == 0) ? 0x7u : 0u;
			}
			csh_printf(csh, "dmic patch: mask=0x%X (%s%s%s)\r\n", (unsigned)s_patch_mask,
			           (s_patch_mask & PATCH_ADCKEN)  ? "aud_cken " : "",
			           (s_patch_mask & PATCH_SOFTRST) ? "soft_reset " : "",
			           (s_patch_mask & PATCH_CLKGATE) ? "clk_gate" : "");
			csh_printf(csh, "      0x1=aud_cken 0x2=soft_reset 0x4=clk_gate（默认只开 0x1）\r\n");
			csh_printf(csh, "      已实测：soft_reset 清 0 会让 FIFO 恒 empty，别开 0x2\r\n");
			return 0;
		}

		if (!strcmp(argv[1], "cfg")) {
			if ((argc > 3) && !strcmp(argv[2], "adc"))
				s_adc_start_en = !strcmp(argv[3], "on");
			else if ((argc > 3) && !strcmp(argv[2], "freq"))
				s_freq_vote_en = !strcmp(argv[3], "on");
			else if ((argc > 3) && !strcmp(argv[2], "drain"))
				s_adc_drain_en = !strcmp(argv[3], "on");

			csh_printf(csh, "dmic cfg: adc_start=%s  freq_vote=%s  drain=%s  patch_mask=0x%X\r\n",
			           s_adc_start_en ? "on" : "off",
			           s_freq_vote_en ? "on" : "off",
			           s_adc_drain_en ? "on" : "off",
			           (unsigned)s_patch_mask);
			csh_printf(csh, "      adc off -> DMIC 完全不出数；freq on -> 实测也不出数\r\n");
			csh_printf(csh, "      drain on -> ADC FIFO 用 DMA 排空，验证'0.5s 冻结'是否消失\r\n");
			return 0;
		}

		if (!strcmp(argv[1], "mon")) {
			int mon_ms = 3000;

			if (argc > 2)
				mon_ms = atoi(argv[2]);
			return dmic_mon_cmd(csh, mon_ms);
		}

#if CONFIG_ADK_ONBOARD_DUAL_DMIC_MIC_STREAM
		if (!strcmp(argv[1], "adk")) {
			int adk_ms = 2000;
			int rec_sec = 5;
			const char *path = NULL;
			const char *file = NULL;
			bool adk_stereo = false;
			bool want_rec = false;
			bool adk_nopwr = false;
			int ai;

			/* 参数扫描，顺序随意：
			 *   dmic adk [ms]                     只打电平统计（单声道，只留 L）
			 *   dmic adk stereo [ms]              双声道，只打统计（L/R 并排）
			 *   dmic adk rec [sec] [file] [stereo] 采集并落 WAV 到 SD 卡
			 *   ... np / nopwr                    不碰 MIC 1V8（断电反证用）
			 * 例如 `dmic adk rec 5 /adk_lr.wav stereo` 和
			 * `dmic adk stereo rec 5` 等价。 */
			for (ai = 2; ai < argc; ai++) {
				const char *a = argv[ai];

				if (!strcmp(a, "stereo") || !strcmp(a, "lr") || !strcmp(a, "dual"))
					adk_stereo = true;
				else if (!strcmp(a, "rec"))
					want_rec = true;
				else if (!strcmp(a, "np") || !strcmp(a, "nopwr"))
					adk_nopwr = true;
				else if ((a[0] >= '0') && (a[0] <= '9')) {
					if (want_rec)
						rec_sec = atoi(a);
					else
						adk_ms = atoi(a);
				} else if (want_rec)
					file = a;
			}

			if (want_rec) {
				if (rec_sec < 1)
					rec_sec = 1;
				if (rec_sec > 30)
					rec_sec = 30;
				adk_ms = rec_sec * 1000;
				path = file ? file : (adk_stereo ? "1:/adk_lr.wav" : "1:/adk.wav");
			}

			return dmic_adk_cmd(csh, adk_ms, path, adk_stereo, adk_nopwr);
		}
#else
		if (!strcmp(argv[1], "adk")) {
			csh_printf(csh, "dmic adk: 需开 CONFIG_ADK_ONBOARD_DUAL_DMIC_MIC_STREAM\r\n");
			return -1;
		}
#endif

		if (!strcmp(argv[1], "pwr")) {
			/* MIC 1V8（P48 控制的那路 LDO）单独开关。
			 *
			 * 用途是「断电反证」：DMIC 的 DOUT 是推挽输出，麦一断电就变
			 * 高阻，P9 上只剩 P8 时钟的串扰 → CIC 收到恒定量 →
			 * 去直流后 rms≈0、flat≈100%。
			 * 所以 `dmic pwr off` + `dmic adk stereo 3000 np`：
			 *   电平塌到 ~0 → 之前那 80LSB 噪声确实是这颗麦产生的
			 *                  （麦活着但声学不通：声孔/振膜/背腔）；
			 *   电平一点不变 → 样本根本不是这颗麦给的，去查接收侧。
			 * 注意 np 只是"不自动上电"，不会自己关电。 */
			bool cur = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);

			if (argc > 2) {
				bool want = strcmp(argv[2], "off") != 0;

				bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, want);
				rtos_delay_milliseconds(20);
				cur = bsp_power_is_enabled(BSP_PWR_RAIL_MIC_1V8);
			}
			csh_printf(csh, "dmic pwr: MIC 1V8 (P%d) = %s\r\n",
			           (int)bsp_power_get_pad(BSP_PWR_RAIL_MIC_1V8),
			           cur ? "ON" : "OFF");
			return 0;
		}

		if (!strcmp(argv[1], "rate")) {
			if (argc > 2) {
				int r = atoi(argv[2]);

				/* 只接受 bk_aud_dmic_set_samp_rate() 支持的集合
				 * （aud_dmic_driver.c:105-108）。注意 **8000 不在里面**，
				 * 传 8000 会走 default 分支只打一条 ERROR 而不配任何分频。
				 *
				 * PDM CLK = 24576/(dig_mic_div+1) kHz，CIC 固定 ÷64：
				 *   16000 → div23 → 1024kHz ✘ 落在手册 900~1300 空档，出规格
				 *   32000 → div11 → 2048kHz ✔ 手册 Standard Mode
				 *   48000 → div7  → 3072kHz ✔ 手册 Standard Mode
				 * 11025/12000/22050/24000/44100 五档，SDK 里的 div 与 ÷64
				 * 对不上（24000 写 div=3 → 96k），不建议使用。 */
				switch (r) {
				case 16000:
				case 32000:
				case 48000:
					s_dmic_rate = (uint32_t)r;
					if ((s_rec_state == REC_IDLE) && (s_rec_task == NULL))
						s_rec_rate = r;   /* 顺手同步给录音，避免不一致 */
					break;
				case 11025: case 12000: case 22050: case 24000: case 44100:
					csh_printf(csh, "dmic rate: %d 在 SDK 里 div/÷64 对不上，"
					                "实际 PDM 会偏离，不建议用\r\n", r);
					return -1;
				default:
					csh_printf(csh, "dmic rate: %d 不支持（可用 16000 / 32000 / 48000）\r\n", r);
					return -1;
				}
			}
			{
				uint32_t div = (s_dmic_rate == 32000) ? 11 :
				               (s_dmic_rate == 48000) ? 7 : 23;
				uint32_t pdm = 24576u / (div + 1u);
				const char *verdict;

				if (pdm < 1300)
					verdict = "低于手册 Standard 下限 1300kHz（900~1300 是空档，出规格）";
				else if (pdm > 4800)
					verdict = "高于手册上限 4800kHz（出规格）";
				else
					verdict = "在手册 Standard Mode 1300~4800kHz 内 ✔";

				csh_printf(csh, "dmic rate: %u Hz  div=%u  PDM CLK = %u kHz\r\n",
				           (unsigned)s_dmic_rate, (unsigned)div, (unsigned)pdm);
				csh_printf(csh, "      %s\r\n", verdict);
			}
			csh_printf(csh, "      可用 16000 / 32000 / 48000；dmic 与 dmic rec 同步\r\n");
			csh_printf(csh, "      注：16000 -> 1024kHz 出规格，仅用于复现冻结对照\r\n");
			return 0;
		}

		if (!strcmp(argv[1], "shift")) {
			if (argc > 2) {
				int v = atoi(argv[2]);

				if ((v < -8) || (v > 10)) {
					csh_printf(csh, "dmic shift: %d 超范围（-8 ~ 10；+1 = x2）\r\n", v);
					return -1;
				}
				s_dmic_shift = v;
			}
			csh_printf(csh, "dmic shift: %d  （%d dB，每 1 位 = 6dB = 2 倍）\r\n",
			           (int)s_dmic_shift, (int)(s_dmic_shift * 6));
			csh_printf(csh, "      去直流后对 int16 移位，超出范围自动削顶（不会回绕）\r\n");
			csh_printf(csh, "      只作用于 `dmic adk`；手写 dmic 路径保持原始数据作对照\r\n");
			return 0;
		}

		if (!strcmp(argv[1], "gain")) {
			if (argc > 2) {
				long g = strtol(argv[2], NULL, 0);   /* 支持 0x3f / 63 */

				if ((g < 0) || (g > 0x3f)) {
					csh_printf(csh, "dmic gain: %ld 超范围（0 ~ 0x3f）\r\n", g);
					return -1;
				}
				s_dmic_gain = (uint8_t)g;
			}
			if (s_dmic_gain > 0x3f)
				csh_printf(csh, "dmic gain: 未设置（用 SDK 默认 0x2d）\r\n");
			else
				csh_printf(csh, "dmic gain: 0x%02X（下次 dmic / dmic rec 生效）\r\n",
				           (unsigned)s_dmic_gain);
			return 0;
		}

		if (!strcmp(argv[1], "pick")) {
			if (argc > 2) {
				if (!strcmp(argv[2], "h") || !strcmp(argv[2], "high"))
					s_pick_high = true;
				else if (!strcmp(argv[2], "l") || !strcmp(argv[2], "low"))
					s_pick_high = false;
				else {
					csh_printf(csh, "dmic pick: 用 h|high 或 l|low\r\n");
					return -1;
				}
			}
			csh_printf(csh, "dmic pick: 取 %s 16bit（%s）\r\n",
			           s_pick_high ? "高" : "低",
			           s_pick_high ? "旧默认，实测高半字恒定" : "默认，实测低半字有变化");
			csh_printf(csh, "      影响 dmic 的全程统计与 dmic rec 的 WAV 内容\r\n");
			return 0;
		}

		if (!strcmp(argv[1], "pm"))
			return dmic_pm_cmd(csh);


		if (!strcmp(argv[1], "adc") || !strcmp(argv[1], "digmicsel")) {
			aud_clk_t clk = AUD_CLK_APLL;
			int r;

			if ((argc > 2) && !strcmp(argv[2], "xtal"))
				clk = AUD_CLK_XTAL;

			r = dmic_audio_setup(csh, clk);
			if (r != BK_OK) {
				csh_printf(csh, "dmic: audio setup failed: %d\r\n", r);
				return r;
			}
			rtos_delay_milliseconds(20);

			r = !strcmp(argv[1], "adc") ? dmic_adc_probe(csh) :
			                              dmic_dig_mic_sel_cmd(csh);
			dmic_audio_teardown();
			return r;
		}

		if (!strcmp(argv[1], "bitbang"))
			return dmic_bitbang_cmd(csh);

		if (!strcmp(argv[1], "tone"))
			return dmic_tone_cmd(csh, argc, argv);

		if (!strcmp(argv[1], "rec")) {
			uint32_t sec = DMIC_REC_DEF_SEC;
			const char *file = "1:/dmic.wav";

			if (argc > 2) {
				/* 允许 dmic rec [秒] [文件名] 的两种简写：
				 *   dmic rec 5 /a.wav   → 5 秒 + 指定文件
				 *   dmic rec /a.wav     → 默认秒数 + 指定文件
				 * 之前统一用 atoi(argv[2]) 判秒，atoi("/a.wav")==0，
				 * 于是秒数保持默认、**文件名被静默忽略**，录到的是
				 * 1:/dmic.wav 而不是用户输入的 /a.wav。 */
				if ((argv[2][0] >= '0') && (argv[2][0] <= '9')) {
					int v = atoi(argv[2]);

					if (v > 0)
						sec = (uint32_t)v;
					if (argc > 3)
						file = argv[3];
				} else {
					file = argv[2];
				}
			}
			if (sec > DMIC_REC_MAX_SEC)
				sec = DMIC_REC_MAX_SEC;

			return dmic_rec_start(csh, sec, file);
		}

		if (!strcmp(argv[1], "stop"))
			return dmic_rec_stop(csh);

		if (!strcmp(argv[1], "status")) {
			dmic_rec_status(csh);
			return 0;
		}

		if (!strcmp(argv[1], "help")) {
			dmic_usage(csh);
			return 0;
		}

		ms = atoi(argv[1]);
		if (ms < DMIC_MIN_MS)
			ms = DMIC_MIN_MS;
		if (ms > DMIC_MAX_MS)
			ms = DMIC_MAX_MS;
	}

	if ((argc > 2) && !strcmp(argv[2], "xtal"))
		clk = AUD_CLK_XTAL;

	/* 可选第三参数临时指定采样率（不改 s_dmic_rate），方便一次测多档：
	 *   dmic 200 apll 32000 */
	{
		int rate = (int)s_dmic_rate;

		if ((argc > 3) && (atoi(argv[3]) > 0))
			rate = atoi(argv[3]);

		return dmic_capture(csh, ms, clk, rate);
	}
}
