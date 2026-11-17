// Copyright 2026
// USB 麦克风采集 BSP（DMIC -> PCM16）
//
// 硬件：U15 / U16 = MEMS 数字麦，CLK 共用 P8、DATA 共用 P9（单线双通道）
//   U16 L/R 接 GND -> 左声道，U15 L/R 接 1V8 -> 右声道
//   1V8 供电由 P48（BSP_PWR_RAIL_MIC_1V8，低有效 PMOS）控制
//
// 实现走 SDK 官方组件 onboard_dual_dmic_mic_stream：
//   该组件内部完成 bk_aud_adc_init()（全 SDK 唯一会拉 sys_drv_aud_aud_en 的
//   路径，DMIC 必须借它上电）、DMIC 初始化、DMA 双缓冲、以及把 32bit
//   打包字拆成左右两个 int16 —— 输出就是 UAC 要的 L/R 交织 PCM16。
//
// 关键配置（app_test_uart1 上逐项实测定案，勿随意改）：
//   dmic_data_shift = 0     组件里的移位是裸 `<<=`，无饱和，>=5 会按位回绕
//                           成破音。用户明确要求保持 0。
//   dual_dmic       = 1     本板是双麦单线
//   dual_dmic_sgl_out = 0   保留立体声（L/R 交织），UAC 侧按需取单声道
//   ref_mode        = 0     不启用 AEC 参考通路。置 1 会让组件自己
//                           start ADC 并在同一个 ring buffer 里混入参考数据
//   frame/block     = 20ms  与 USB 1ms 包的整数倍关系最好
//
// 启动顺序（顺序不能变，否则 DMA 一个完成中断都不来）：
//   audio_element_run() -> audio_element_resume()
//   -> bk_aud_adc_start() + ADC FIFO 排空 DMA
// 最后两步是组件在 dual_dmic && !ref_mode 分支里**没做**的事，
// 但本板实测 ADC 不 start 的话 DMIC FIFO 恒为 EMPTY。

#include "bsp_mic.h"
#include "bsp_power.h"
#include "bsp_board_cfg.h"

#include <components/bk_audio/audio_streams/onboard_dual_dmic_mic_stream.h>
#include <components/bk_audio/audio_pipeline/audio_element.h>
#include <components/bk_audio/audio_pipeline/audio_port.h>
#include <components/bk_audio/audio_pipeline/rb_port.h>
#include <driver/aud_adc.h>
#include <driver/dma.h>
#include <driver/pwr_clk.h>   /* bk_pm_module_vote_cpu_freq() */
#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#define TAG "bsp_mic"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* 20ms 一帧；双声道 16bit -> 每毫秒 4 字节 */
#define MIC_FRAME_MS          20
#define MIC_BYTES_PER_MS(ch)  (2 * (ch))

/* ADC FIFO 排空 DMA 的假缓冲长度（word 数），读出来丢掉 */
#define MIC_ADC_DRAIN_WORDS   64

static bool s_inited = false;
static bool s_running = false;

static audio_element_handle_t s_el = NULL;
static audio_port_handle_t s_port = NULL;

static uint32_t s_rate = 0;
static uint32_t s_ch = 0;               /* 声道数：1 或 2 */
/* 【两个量必须分开，别再合并】
 *
 *   s_ch_bytes    = **单声道**一帧字节数（48k/20ms -> 1920）→ 传给组件
 *   s_frame_bytes = 交织后的总量（stereo -> 3840）→ 我们自己读的单位
 *
 * 组件把 `cfg.frame_size` 当成**单声道**大小，内部再乘 2 得到 DMA 一次
 * 搬运量（onboard_dual_dmic_mic_stream.c:327
 *   frame_size = onboard_mic->frame_size * 2;
 *   bk_dma_set_transfer_len(mic_dma_id, frame_size);
 * ）。
 *
 * 曾经把交织总量（3840）直接传进去，组件又乘 2 变 7680 —— DMA 要凑
 * 40ms 数据才搬一次，ring buffer 里永远填不满一整块，症状是
 *     OB_MIC:W  mic_fill: 6088 < len: 7680
 *     uac mic: in_ok=1212 cap=0 starve=1214
 * 即主机侧在正常补静音发包，但采集侧 cap 恒 0。
 *
 * 所以：**frame_size 传单声道，读取按交织总量**。这是 2026-11-16 重新
 * 启用 USB 麦克风时最终定下的写法。 */
static bool s_freq_voted = false;    /* 是否已把 CPU 钉到 480MHz */
static uint32_t s_ch_bytes = 0;      /* 单声道一帧字节数（传给组件） */
static uint32_t s_frame_bytes = 0;   /* 交织总量（自己读的单位） */

static dma_id_t s_drain_dma = DMA_ID_MAX;
static uint32_t *s_drain_buf = NULL;

void bsp_mic_default_cfg(bsp_mic_cfg_t *cfg)
{
	if (!cfg)
		return;

	cfg->samp_rate = 48000;
	cfg->frame_ms = MIC_FRAME_MS;
	cfg->stereo = true;
}

uint32_t bsp_mic_frame_bytes(void)
{
	return s_frame_bytes;
}

uint32_t bsp_mic_frame_samples(void)
{
	return (s_ch ? (s_frame_bytes / 2 / s_ch) : 0);
}

uint32_t bsp_mic_sample_rate(void)
{
	return s_rate;
}

uint32_t bsp_mic_channels(void)
{
	return s_ch;
}

bool bsp_mic_is_running(void)
{
	return s_running;
}

/* ---------------- ADC FIFO 排空 DMA ----------------
 *
 * 为什么要单独起一个 DMA 去读 ADC FIFO 然后丢掉：
 * DMIC 模式下 ADC 通路只是被借来"打开音频模块电源/时钟"，但 FIFO 仍在
 * 正常出数。没人读的话 FIFO 灌满后会把 CIC 滤波器拖停 —— 实测约 0.5s
 * 后 DMIC 输出冻结成常量。所以在 FIFO 固定地址上挂一个 REPEAT DMA 空转，
 * 数据读出来直接丢弃，FIFO 永远不堆积。 */
static void mic_drain_stop(void)
{
	if (s_drain_dma < DMA_ID_MAX) {
		bk_dma_stop(s_drain_dma);
		bk_dma_deinit(s_drain_dma);
		bk_dma_free(DMA_DEV_AUDIO_RX, s_drain_dma);
		s_drain_dma = DMA_ID_MAX;
	}

	if (s_drain_buf) {
		os_free(s_drain_buf);
		s_drain_buf = NULL;
	}
}

static bk_err_t mic_drain_start(void)
{
	dma_config_t cfg;
	uint32_t fifo = 0;
	bk_err_t ret;

	if (s_drain_dma < DMA_ID_MAX)
		return BK_OK;

	if (bk_aud_adc_get_fifo_addr(&fifo) != BK_OK) {
		BSP_LOGE("get adc fifo addr failed\r\n");
		return BK_FAIL;
	}

	if (!s_drain_buf) {
		s_drain_buf = (uint32_t *)os_malloc(MIC_ADC_DRAIN_WORDS * 4);
		if (!s_drain_buf)
			return BK_ERR_NO_MEM;
	}

	(void)bk_dma_driver_init();

	s_drain_dma = bk_dma_alloc(DMA_DEV_AUDIO_RX);
	if ((s_drain_dma < DMA_ID_0) || (s_drain_dma >= DMA_ID_MAX)) {
		s_drain_dma = DMA_ID_MAX;
		BSP_LOGE("drain dma alloc failed\r\n");
		return BK_FAIL;
	}

	os_memset(&cfg, 0, sizeof(cfg));
	cfg.mode = DMA_WORK_MODE_REPEAT;
	cfg.chan_prio = 1;
	cfg.trans_type = DMA_TRANS_DEFAULT;

	cfg.src.dev = DMA_DEV_AUDIO_RX;
	cfg.src.width = DMA_DATA_WIDTH_32BITS;
	cfg.src.addr_inc_en = DMA_ADDR_INC_DISABLE;    /* FIFO 固定地址 */
	cfg.src.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
	cfg.src.start_addr = fifo;
	cfg.src.end_addr = fifo + 4;

	cfg.dst.dev = DMA_DEV_DTCM;
	cfg.dst.width = DMA_DATA_WIDTH_32BITS;
	cfg.dst.addr_inc_en = DMA_ADDR_INC_ENABLE;
	cfg.dst.addr_loop_en = DMA_ADDR_LOOP_ENABLE;
	cfg.dst.start_addr = (uint32_t)(uintptr_t)s_drain_buf;
	cfg.dst.end_addr = (uint32_t)(uintptr_t)s_drain_buf + MIC_ADC_DRAIN_WORDS * 4;

	ret = bk_dma_init(s_drain_dma, &cfg);
	if (ret != BK_OK)
		goto fail;

	bk_dma_set_transfer_len(s_drain_dma, MIC_ADC_DRAIN_WORDS * 4);

	/* TrustZone：本工程 CONFIG_SPE=1，漏掉这两句会因为 DMA 以 non-secure
	 * 身份访问外设 FIFO 被拦，症状是"通道配好了但 ISR 一次不来"。 */
#if (CONFIG_SPE)
	bk_dma_set_dest_sec_attr(s_drain_dma, DMA_ATTR_SEC);
	bk_dma_set_src_sec_attr(s_drain_dma, DMA_ATTR_SEC);
#endif

	ret = bk_dma_start(s_drain_dma);
	if (ret != BK_OK)
		goto fail;

	return BK_OK;

fail:
	BSP_LOGE("drain dma start failed: %d\r\n", ret);
	mic_drain_stop();
	return BK_FAIL;
}

/* ---------------- 对外接口 ---------------- */

bk_err_t bsp_mic_init(const bsp_mic_cfg_t *cfg)
{
	onboard_dual_dmic_mic_stream_cfg_t c = DEFAULT_ONBOARD_DUAL_DMIC_STREAM_CONFIG();
	bsp_mic_cfg_t mc;
	uint32_t frame_ms;
	bk_err_t ret;

	if (s_inited)
		return BK_OK;

	if (cfg)
		mc = *cfg;
	else
		bsp_mic_default_cfg(&mc);

	if (mc.samp_rate == 0)
		mc.samp_rate = 48000;
	/* 只支持 16k/32k/48k：PDM CLK = 24576/(div+1) kHz 且 CIC 固定 ÷64，
	 * 其它档位与 ÷64 不自洽，采出来是变调的。 */
	if ((mc.samp_rate != 16000) && (mc.samp_rate != 32000) &&
	    (mc.samp_rate != 48000)) {
		BSP_LOGE("unsupported rate %u, use 16000/32000/48000\r\n",
		         (unsigned)mc.samp_rate);
		return BK_ERR_PARAM;
	}
	frame_ms = mc.frame_ms ? mc.frame_ms : MIC_FRAME_MS;

	s_rate = mc.samp_rate;
	s_ch = mc.stereo ? 2 : 1;
	/* 单声道给组件，交织总量给自己（见文件头两个变量的说明） */
	s_ch_bytes = (s_rate / 1000) * frame_ms * MIC_BYTES_PER_MS(1);
	s_frame_bytes = s_ch_bytes * s_ch;

	/* 1. MIC 1V8 供电（P48，低有效 PMOS）。组件是通用件，不碰板级电源，
	 * 必须自己开，否则 DMIC 采出来全是 0。 */
	ret = bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, true);
	if (ret != BK_OK) {
		BSP_LOGE("mic power on failed: %d\r\n", ret);
		return ret;
	}
	rtos_delay_milliseconds(5);   /* 等 MEMS 麦上电稳定 */

	/* 2. 组件配置 */
	c.adc_cfg.sample_rate = s_rate;
	/* chl_num 只影响 ADC 那侧初始化；DMIC 通路固定 AUD_ADC_CHL_LR，
	 * 保持 1 不动（这条路径在 app_test_uart1 已验证）。 */
	c.adc_cfg.chl_num = 1;
	c.adc_cfg.bits = 16;
	c.adc_cfg.dig_gain = 0x2D;   /* 0dB */
	c.adc_cfg.ana_gain = 0x00;
	c.adc_cfg.mode = AUD_ADC_MODE_DIFFEN;
	c.adc_cfg.clk_src = AUD_CLK_APLL;

	/* frame_size 传**单声道**字节数：组件内部会 *2 得到 DMA 搬运量。
	 * out_block_size 是元素输出块的字节数，按交织总量给。 */
	c.frame_size = s_ch_bytes;
	c.out_block_size = s_frame_bytes;
	c.out_block_num = 4;
	c.multi_out_port_num = 0;
	c.task_stack = 4096;
	c.task_core = 1;
	c.task_prio = 3;
	c.ref_mode = 0;
	c.dual_dmic = 1;
	c.dual_dmic_sgl_out = mc.stereo ? 0 : 1;
	/* 保持 0：组件里的移位无饱和，>=5 会回绕成破音 */
	c.dmic_data_shift = 0;

	s_el = onboard_dual_dmic_mic_stream_init(&c);
	if (!s_el) {
		BSP_LOGE("dmic stream init failed\r\n");
		ret = BK_FAIL;
		goto fail_power;
	}

	/* 3. 组件是 PORT_TYPE_RB 输出且 cfg.write == NULL，audio_element_init()
	 * 不会自己建输出端口。必须自己 ringbuf_port_init() 一个再 set 上去，
	 * 否则 _process 里 audio_element_output() 因 out == NULL 把数据全丢，
	 * get_output_port() 永远返回 NULL。 */
	s_port = audio_element_get_output_port(s_el);
	if (!s_port) {
		ringbuf_port_cfg_t rb = RINGBUF_PORT_CFG_DEFAULT();

		rb.ringbuf_size = s_frame_bytes * c.out_block_num * 2;
		if (rb.ringbuf_size < RINGBUF_PORT_RINGBUF_SIZE)
			rb.ringbuf_size = RINGBUF_PORT_RINGBUF_SIZE;

		s_port = ringbuf_port_init(&rb);
		if (!s_port) {
			BSP_LOGE("ringbuf port init failed\r\n");
			ret = BK_FAIL;
			goto fail_el;
		}
		audio_element_set_output_port(s_el, s_port);
	}

	s_inited = true;
	BSP_LOGI("dmic ready: %uHz %s, block %uB (shift=0)\r\n",
	         (unsigned)s_rate, mc.stereo ? "stereo" : "mono",
	         (unsigned)s_frame_bytes);
	return BK_OK;

fail_el:
	audio_element_deinit(s_el);
	s_el = NULL;
fail_power:
	bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);
	return ret;
}

bk_err_t bsp_mic_start(void)
{
	bk_err_t ret;

	if (!s_inited)
		return BK_ERR_NOT_INIT;
	if (s_running)
		return BK_OK;

	/* run() 只创建元素任务，不会调 _open()；
	 * 真正启动 DMA/DMIC 的是 resume() -> AEL_MSG_CMD_RESUME -> _open()。
	 * 漏掉 resume 的症状：任务在跑但 ring buffer 的 fill 恒为 0。 */
	if (audio_element_run(s_el) != BK_OK) {
		BSP_LOGE("element run failed\r\n");
		return BK_FAIL;
	}

	if (audio_element_resume(s_el, 0, 2000 / portTICK_RATE_MS) != BK_OK) {
		BSP_LOGE("element resume failed\r\n");
		audio_element_stop(s_el);
		audio_element_deinit(s_el);
		s_el = NULL;
		s_port = NULL;
		return BK_FAIL;
	}

	/* 组件在 dual_dmic && !ref_mode 分支里只启 DMIC、不启 ADC。
	 * 本板实测 ADC 不 start 的话 DMIC FIFO 恒 EMPTY，一个完成中断都不来。 */
	ret = bk_aud_adc_start();
	if (ret != BK_OK) {
		BSP_LOGE("adc start failed: %d\r\n", ret);
		goto fail;
	}

	/* 不排空约 0.5s 后 CIC 会冻结 */
	if (mic_drain_start() != BK_OK) {
		BSP_LOGE("adc drain start failed\r\n");
		goto fail;
	}

	/* 【必须提频到 480MHz，组件自己不投票】
	 * app_test_uart1 的 `dmic adk rec` 里有一句同样目的的投票注释：
	 * "音频期间把 CPU 频率钉到 480MHz，与手写路径一致（组件本身不投票）"。
	 * DMIC 是 48kHz 的 DMA + 每 20ms 一帧拷贝，默认频率下任务调度跟不上，
	 * 表现为端口永远填不满、`bsp_mic_read` 恒超时（cap=0，主机只有静音）。
	 * USB 麦克风模式**不会**走 app_audio_open()，所以那里 240MHz 的票在这里
	 * 是拿不到的，必须自己投。 */
	if (!s_freq_voted) {
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_480M);
		s_freq_voted = true;
	}

	s_running = true;
	BSP_LOGI("mic capture started\r\n");

	return BK_OK;

fail:
	mic_drain_stop();
	audio_element_stop(s_el);
	audio_element_deinit(s_el);
	s_el = NULL;
	s_port = NULL;
	return BK_FAIL;
}

bk_err_t bsp_mic_stop(void)
{
	if (!s_inited || !s_running)
		return BK_OK;

	s_running = false;

	mic_drain_stop();
	bk_aud_adc_stop();

	/* 组件的 _close 会停 DMIC + 释放 DMA。元素 stop 之后必须重新
	 * init 才能再用，所以这里不动 s_el 本身，交给 deinit。 */
	if (s_el)
		audio_element_stop(s_el);

	if (s_freq_voted) {
		bk_pm_module_vote_cpu_freq(PM_DEV_ID_AUDIO, PM_CPU_FRQ_DEFAULT);
		s_freq_voted = false;
	}

	return BK_OK;
}

/* 输出端口当前可读字节数（诊断用：0 = 组件没在产数据）。 */
int bsp_mic_filled(void)
{
	if (!s_inited || !s_port)
		return -1;
	return (int)audio_port_get_filled_size(s_port);
}

bk_err_t bsp_mic_read(void *buf, uint32_t len, uint32_t timeout_ms)
{
	int filled;
	bk_err_t ret;

	if (!s_inited || !s_running || !s_port)
		return BK_ERR_NOT_INIT;
	if (!buf || len < s_frame_bytes)
		return BK_ERR_PARAM;

	filled = (int)audio_port_get_filled_size(s_port);
	if (filled < (int)s_frame_bytes)
		return BK_ERR_TIMEOUT;

	/* 【audio_port_read 返回的是"读到的字节数"，不是 BK_OK！】
	 *
	 * rb_port.c 的 _ringbuf_port_read 直接透传 rb_read() 的返回值
	 * （>=0 = 实际字节数，<0 = 错误）。之前这里写 `if (ret != BK_OK)`
	 * —— 读成功时返回 3840，3840 != 0 被当成超时丢掉；但此时 rb 的
	 * 读指针**已经前进**，数据被真实消费了。于是出现：
	 *     mic health: cap=0 starve=xxx filled=3840B   <- 刚产出一块
	 *     mic health: cap=0 starve=xxx filled=0B      <- 被读走了！
	 * 且 cap 永远为 0 —— 数据一直在被读走又被当作失败丢弃，
	 * 主机只能收到静音。
	 *
	 * 正确判法见 app_test_uart1 的 `dmic adk rec`（cmd_dmic.c:1293）：
	 *     int rd = audio_port_read(port, buf, block_bytes, 100);
	 *     if (rd > 0) ...
	 * 这里要求读满一整帧（交织 L/R 3840B）才算成功；不足整帧时数据
	 * 已被消费、无法回退，只能报错让上层放弃这一轮。 */
	ret = audio_port_read(s_port, (char *)buf, (int)s_frame_bytes,
	                      (TickType_t)(timeout_ms / portTICK_RATE_MS + 1));
	if (ret < (int)s_frame_bytes)
		return BK_ERR_TIMEOUT;

	return BK_OK;
}

uint32_t bsp_mic_drop_frames(void)
{
	return 0;
}

bk_err_t bsp_mic_deinit(void)
{
	if (!s_inited)
		return BK_OK;

	bsp_mic_stop();

	if (s_el) {
		audio_element_deinit(s_el);
		s_el = NULL;
	}
	s_port = NULL;

	bsp_power_enable(BSP_PWR_RAIL_MIC_1V8, false);

	s_inited = false;
	return BK_OK;
}
