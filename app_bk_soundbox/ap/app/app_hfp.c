// Copyright 2026
// 蓝牙免提通话（HFP HF）+ 软件 AEC —— 实现
//
// 为什么要自己写而不是直接用 SDK 的 projects/bluetooth/headset 示例：
//   示例的 hfp_hf_demo.c 里有一整套 AT 自测序列（连上就 query operator、
//   CNUM、BVRA、然后 dial "112" 拨号），而且音频走 audio_play/audio_record
//   两个 headset 工程专属的 ADK 子系统。直接搬进本工程会出现两个问题：
//   ① 每次手机连上都会自己拨 112；
//   ② 与本工程的 app_audio / bsp_mic（DMIC）完全对不上。
//   所以这里只借用它的**协议用法**（回调注册、voice_out_write、mSBC 编解码），
//   事件与音频通路全部按本工程的形态重写。
//
// 数据流（16kHz 宽带语音 + mSBC）：
//
//   下行：SCO --bk_bt_hf_client_data_cb--> [mSBC 解 2B 头+SBC 帧] --> PCM16
//         --> app_audio_write() 出声
//         --> aec ring（参考信号，同一份数据，供 AEC 抵消回声）
//
//   上行：DMIC 16k --> aec_algorithm（ref = 上面 ring）--> PCM16
//         --> [mSBC 编码 + 2B 头] --> bk_bt_hf_client_voice_out_write()
//
// AEC 是本文件的重点，见 aec_pipeline_start()。

#include "app_hfp.h"
#include "app_audio.h"
#include "app_mode.h"
#include "app_state.h"
#include "app_bt.h"
#include "bsp.h"

#include <string.h>
#include <os/os.h>
#include <os/mem.h>

#include <components/log.h>
#include <components/bluetooth/bk_dm_bluetooth.h>
#include <components/bluetooth/bk_dm_hfp.h>
#include <components/bluetooth/bk_dm_hfp_types.h>

/* AEC 算法元素（audio_pipeline） */
#include <components/bk_audio/audio_algorithms/aec_algorithm.h>
#include <components/bk_audio/audio_pipeline/audio_element.h>
#include <components/bk_audio/audio_pipeline/audio_port.h>
#include <components/bk_audio/audio_pipeline/rb_port.h>

/* mSBC 编解码 */
#include <driver/sbc.h>
#include <driver/sbc_types.h>
#include <modules/sbc_encoder.h>

#define TAG "app_hfp"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* ---------------- 语音参数 ----------------
 *
 * 本机与对端约定 16kHz（mSBC 宽带）。选 16k 而不是 8k 的原因：
 *   ① bsp_mic 的 DMIC 白名单是 16k/32k/48k，8k 不在其中（PDM CLK 与
 *      CIC ÷64 不自洽会变调），16k 可直接用；
 *   ② mSBC 本身就是 16k，正好一一对应，不需要重采样。
 *
 * 帧长：mSBC 每帧 120 个采样点（7.5ms），CVSD 每帧 60 点（7.5ms）。
 * AEC 元素固定按 20ms 一帧处理（SDK 写死 frame_size = fs/1000*2*20），
 * 所以中间用 ring 做 7.5ms <-> 20ms 的块大小转换。 */
#define HFP_RATE              16000
#define HFP_MSBC_SAMPLES      120               /* 每帧采样点数（7.5ms） */
#define HFP_CVSD_SAMPLES      60
#define HFP_MSBC_HDR_LEN      2                 /* mSBC 同步头 H2 格式 */

#define HFP_TASK_STACK        4096
#define HFP_TASK_PRIO         (BEKEN_DEFAULT_WORKER_PRIORITY - 1)

/* 下行播放环：蓝牙栈回调把解出来的 PCM 丢进来，通话任务取出去写 DAC。
 *
 * 为什么必须中转而不能在回调里直接 app_audio_write()：
 * audio_play_write_data() 在 DAC 缓冲满时会**阻塞等待**，而这个回调
 * 跑在蓝牙协议栈的任务上下文里 —— 一旦在那里阻塞几毫秒，整条 BT 链路
 * 的 ACL/SCO 收发都会被拖慢，表现为通话断续、甚至 AT 应答超时。
 * A2DP 那条路径也是同样的做法（独立 speaker 任务 + ring）。
 *
 * 深度 512ms @16k/mono：够吸收 DAC 启动/抖动，又不会让通话延迟过大
 * （通话场景 total 延迟超过 200ms 就会觉得像对讲机）。 */
#define HFP_DOWN_RING_BYTES   16384
static uint8_t s_down_ring[HFP_DOWN_RING_BYTES] __attribute__((aligned(4)));
static volatile uint32_t s_down_wr;
static volatile uint32_t s_down_rd;

/* 上行待发缓冲：DMIC 20ms 一帧，mSBC 7.5ms 一包，一帧要切 2~3 包 */
#define HFP_UP_BUF_BYTES      2048

#define HFP_AEC_FRAME_BYTES   (HFP_RATE / 1000 * 2 * 20)   /* 640 */

/* ---------------- 消息队列 ----------------
 * HFP 回调全部在蓝牙协议栈任务上下文（不是本文件的任务），
 * 那里不能阻塞、不能碰 audio/UI，所以一律转成消息。 */
typedef enum {
	HFP_MSG_CONN_STATE = 1,
	HFP_MSG_AUDIO_STATE,
	HFP_MSG_CALL_STATUS,
	HFP_MSG_CALL_SETUP,
	HFP_MSG_RING,
	HFP_MSG_CLIP,
	HFP_MSG_ANSWER,
	HFP_MSG_HANGUP,
} hfp_msg_type_t;

typedef struct {
	uint8_t type;
	uint8_t val;       /* 状态值 / 0/1 */
	uint16_t len;
	char text[32];     /* 来电号码 */
} hfp_msg_t;

static beken_queue_t s_que;
static beken_thread_t s_task;
static volatile bool s_run;
static volatile bool s_ready;

static volatile app_hfp_state_t s_state = APP_HFP_STATE_IDLE;

/* 对端地址（answer/hangup/voice_out 都要带） */
static uint8_t s_peer_bda[6];
static volatile bool s_peer_valid;

/* 当前协商出的 codec */
static uint8_t s_codec = CODEC_VOICE_CVSD;
static volatile bool s_sco_up;      /* SCO 音频链路已建立 */

/* ---------------- AEC / 音频通路状态 ---------------- */
static audio_element_handle_t s_aec;
static audio_port_handle_t s_aec_in;     /* source：DMIC 数据进来 */
static audio_port_handle_t s_aec_ref;    /* reference：下行 PCM 进来 */
static audio_port_handle_t s_aec_out;    /* AEC 输出：消完回声的麦克风 */
static bool s_aec_running;

/* DMIC 侧直接用 bsp_mic（16k 单声道） */
static bool s_mic_on;

/* 编解码上下文 */
static SbcEncoderContext s_enc;
static bool s_enc_inited;
static sbcdecodercontext_t s_dec;
static bool s_dec_inited;

/* 上行拼包缓冲 */
static uint8_t s_up[HFP_UP_BUF_BYTES] __attribute__((aligned(4)));
static uint32_t s_up_len;

/* mSBC 下行解码输出缓冲 */
static int16_t s_dec_pcm[HFP_MSBC_SAMPLES] __attribute__((aligned(4)));

/* AEC 一帧的输出缓冲 */
static int16_t s_aec_pcm[HFP_AEC_FRAME_BYTES / 2] __attribute__((aligned(4)));

/* ============================ 工具 ============================ */

static bool post(uint8_t type, uint8_t val)
{
	hfp_msg_t m;

	if (!s_run || !s_que)
		return false;

	os_memset(&m, 0, sizeof(m));
	m.type = type;
	m.val = val;

	return rtos_push_to_queue(&s_que, &m, BEKEN_NO_WAIT) == kNoErr;
}

static void set_state(app_hfp_state_t st)
{
	if (s_state == st)
		return;

	s_state = st;
	LOGI("state -> %d\r\n", (int)st);

	/* UI 只关心"有没有来电/通话"，走 app_state 便于主循环统一绘制 */
	switch (st) {
	case APP_HFP_STATE_INCOMING:
		app_state_set_call(true, true);
		break;
	case APP_HFP_STATE_ACTIVE:
		app_state_set_call(true, false);
		break;
	case APP_HFP_STATE_OUTGOING:
		app_state_set_call(true, false);
		break;
	default:
		app_state_set_call(false, false);
		break;
	}
}

/* ============================ AEC 通路 ============================ */

/* AEC 元素在软件模式下需要两个输入端口：
 *   in      —— 麦克风（本机说话 + 喇叭回声）
 *   multi_in[0] —— 参考信号（刚放出去的喇叭声）
 * 以及一个输出端口（消完回声的干净语音）。 */
static bk_err_t aec_pipeline_start(void)
{
	aec_algorithm_cfg_t cfg = DEFAULT_AEC_ALGORITHM_CONFIG();
	ringbuf_port_cfg_t rb = RINGBUF_PORT_CFG_DEFAULT();

	if (s_aec_running)
		return BK_OK;

	/* ---- 参数标定 ----
	 * fs 必须 16k（与 SCO 一致，省掉重采样）。
	 * delay_points：参考信号与麦克风信号之间的**声学延迟**，单位采样点，
	 *   16k 下 1 点 = 62.5us。喇叭到麦的直达声大约 1~3cm，
	 *   加上 DAC 缓冲与 DMIC 采集流水线，实测从几百点起调。
	 *   注意这只是初值，真机要按"回声残留 vs 自己说话被削"权衡：
	 *   给小了回声消不掉，给大了会把自己的人声也当回声削掉。
	 * ec_depth：回声越强给越大（1~50）。
	 * ns_level：噪声抑制档位 1~8，室内 2 够用。 */
	cfg.aec_cfg.mode = AEC_MODE_SOFTWARE;
	cfg.aec_cfg.fs = HFP_RATE;
	cfg.aec_cfg.delay_points = 100;
	cfg.aec_cfg.ec_depth = 30;
	cfg.aec_cfg.TxRxThr = AEC_ALGORITHM_TXRXTHR;
	cfg.aec_cfg.TxRxFlr = AEC_ALGORITHM_TXRXFLR;
	cfg.aec_cfg.ref_scale = AEC_ALGORITHM_REF_SCALE;
	cfg.aec_cfg.ns_level = AEC_ALGORITHM_NS_LEVEL;
	cfg.aec_cfg.ns_para = AEC_ALGORITHM_NS_PARA;
	cfg.out_block_num = AEC_ALGORITHM_OUT_BLOCK_NUM;
	cfg.task_stack = AEC_ALGORITHM_TASK_STACK;
	cfg.task_core = AEC_ALGORITHM_TASK_CORE;
	cfg.task_prio = AEC_ALGORITHM_TASK_PRIO;

	s_aec = aec_algorithm_init(&cfg);
	if (!s_aec) {
		LOGE("aec init failed\r\n");
		return BK_FAIL;
	}

	/* 三个端口都要自己建：AEC 元素的 in/out 都是 PORT_TYPE_RB 且
	 * read/write 为 NULL，audio_element_init() 不会代建。
	 * 参考端口是 multi_in，同理。 */
	rb.ringbuf_size = HFP_AEC_FRAME_BYTES * 4;
	if (rb.ringbuf_size < RINGBUF_PORT_RINGBUF_SIZE)
		rb.ringbuf_size = RINGBUF_PORT_RINGBUF_SIZE;

	s_aec_in = ringbuf_port_init(&rb);
	s_aec_ref = ringbuf_port_init(&rb);
	s_aec_out = ringbuf_port_init(&rb);
	if (!s_aec_in || !s_aec_ref || !s_aec_out) {
		LOGE("aec port init failed\r\n");
		goto fail;
	}

	audio_element_set_input_port(s_aec, s_aec_in);
	/* 注意参数顺序是 (el, port, index)，不是 (el, index, port) */
	audio_element_set_multi_input_port(s_aec, s_aec_ref, 0);
	audio_element_set_output_port(s_aec, s_aec_out);

	/* 顺序不能变：先 run 再 resume，否则任务不会启动 */
	audio_element_run(s_aec);
	audio_element_resume(s_aec, 0, 2000 / portTICK_RATE_MS);

	s_aec_running = true;
	LOGI("aec started (sw mode, fs=%u, delay=%u)\r\n", (unsigned)HFP_RATE,
	     (unsigned)cfg.aec_cfg.delay_points);
	return BK_OK;

fail:
	if (s_aec_in) { audio_port_deinit(s_aec_in); s_aec_in = NULL; }
	if (s_aec_ref) { audio_port_deinit(s_aec_ref); s_aec_ref = NULL; }
	if (s_aec_out) { audio_port_deinit(s_aec_out); s_aec_out = NULL; }
	if (s_aec) {
		audio_element_deinit(s_aec);
		s_aec = NULL;
	}
	return BK_FAIL;
}

static void aec_pipeline_stop(void)
{
	if (!s_aec_running)
		return;

	audio_element_stop(s_aec);
	audio_element_deinit(s_aec);
	s_aec = NULL;
	s_aec_running = false;

	if (s_aec_in) { audio_port_deinit(s_aec_in); s_aec_in = NULL; }
	if (s_aec_ref) { audio_port_deinit(s_aec_ref); s_aec_ref = NULL; }
	if (s_aec_out) { audio_port_deinit(s_aec_out); s_aec_out = NULL; }

	LOGI("aec stopped\r\n");
}

/* ============================ 通话音频通路 ============================ */

/* 下行一帧 PCM 的投递点（**蓝牙栈回调上下文，绝不能阻塞**）：
 * 只做一次 memcpy 进环形缓冲，真正的"喂 AEC + 写 DAC"由通话任务做。
 *
 * 顺序很重要 —— 任务里必须**先把参考写进 AEC，再写 DAC**。
 * 反过来的话，AEC 拿到参考时喇叭已经开始出声了，参考与回声在时间上
 * 错开，AEC 会把回声当成有效信号处理不掉（听感是"回声只小了一点"）。 */
static void speak_downlink(const int16_t *pcm, uint32_t samples)
{
	uint32_t len = samples * 2;
	uint32_t wr = s_down_wr;
	uint32_t rd = s_down_rd;

	/* 单生产者（BT 回调）/ 单消费者（通话任务）：生产者不动读游标。
	 * 放不下就丢这一帧 —— SCO 本身有重传，丢一帧的影响远小于卡住链路。 */
	if ((wr - rd) + len > HFP_DOWN_RING_BYTES)
		return;

	for (uint32_t i = 0; i < len; i++)
		s_down_ring[(wr + i) & (HFP_DOWN_RING_BYTES - 1)] = ((const uint8_t *)pcm)[i];

	s_down_wr = wr + len;
}

/* 通话任务里调用：把下行数据真正送到 AEC 参考端口和 DAC。
 * 一次搬 20ms（640B），与 AEC 帧长对齐。 */
static void pump_downlink(void)
{
	static int16_t frame[HFP_AEC_FRAME_BYTES / 2] __attribute__((aligned(4)));
	uint32_t rd = s_down_rd;
	uint32_t wr = s_down_wr;
	uint32_t avail = wr - rd;

	if (!s_sco_up || (avail < HFP_AEC_FRAME_BYTES))
		return;

	for (uint32_t i = 0; i < HFP_AEC_FRAME_BYTES; i++)
		((uint8_t *)frame)[i] = s_down_ring[(rd + i) & (HFP_DOWN_RING_BYTES - 1)];

	s_down_rd = rd + HFP_AEC_FRAME_BYTES;

	/* 先喂参考（AEC 需要它先于回声到达），再出声 */
	if (s_aec_running && s_aec_ref)
		audio_port_write(s_aec_ref, (char *)frame, HFP_AEC_FRAME_BYTES, 0);

	app_audio_write(frame, HFP_AEC_FRAME_BYTES);
}

/* 上行：从 AEC 输出端口取一帧，做 mSBC 编码后写入 SCO。
 *
 * AEC 固定 20ms 一帧（640B），而 mSBC 一包 7.5ms（120 点/240B），
 * 所以一帧要切成 2 个整包 + 1 个半包，跨帧拼。s_up 就是拼包缓冲。 */
static void pump_uplink(void)
{
	uint32_t frame_bytes = HFP_AEC_FRAME_BYTES;
	uint32_t pkt_bytes = (s_codec == CODEC_VOICE_MSBC) ?
	                     (HFP_MSBC_SAMPLES * 2) : (HFP_CVSD_SAMPLES * 2);
	bk_err_t ret;

	if (!s_aec_running || !s_aec_out || !s_sco_up)
		return;

	if ((uint32_t)audio_port_get_filled_size(s_aec_out) < frame_bytes)
		return;

	ret = audio_port_read(s_aec_out, (char *)s_aec_pcm, (int)frame_bytes, 0);
	if (ret != BK_OK)
		return;

	/* 拼进上行缓冲；满了就丢最旧的（宁可漏字也不能卡住 SCO） */
	if (s_up_len + frame_bytes > sizeof(s_up))
		s_up_len = 0;
	os_memcpy(s_up + s_up_len, s_aec_pcm, frame_bytes);
	s_up_len += frame_bytes;

	while (s_up_len >= pkt_bytes) {
		if (s_codec == CODEC_VOICE_MSBC) {
			int32_t produced = sbc_encoder_encode(&s_enc, (int16_t *)s_up);

			if (produced > 0) {
				/* stream[-2] / stream[-1] 是 SDK 编码器预留的
				 * 同步头位置，与官方示例一致 */
				bk_bt_hf_client_voice_out_write(s_peer_bda,
				                                (uint8_t *)&s_enc.stream[-2],
				                                (uint16_t)(produced + 2));
			}
		} else {
			bk_bt_hf_client_voice_out_write(s_peer_bda, s_up,
			                                (uint16_t)pkt_bytes);
		}

		s_up_len -= pkt_bytes;
		if (s_up_len)
			os_memmove(s_up, s_up + pkt_bytes, s_up_len);
	}
}

/* DMIC -> AEC 输入端口。bsp_mic 是 20ms 一帧，与 AEC 帧长天然对齐。 */
static void pump_mic_into_aec(void)
{
	static int16_t frame[HFP_AEC_FRAME_BYTES / 2] __attribute__((aligned(4)));

	if (!s_aec_running || !s_aec_in || !s_mic_on)
		return;

	/* 端口快满了就先不读，避免把 bsp_mic 的 ring 读出来又塞不进去 */
	if ((uint32_t)audio_port_get_filled_size(s_aec_in) + HFP_AEC_FRAME_BYTES >
	    (HFP_AEC_FRAME_BYTES * 4))
		return;

	if (bsp_mic_read(frame, sizeof(frame), 2) != BK_OK)
		return;

	audio_port_write(s_aec_in, (char *)frame, HFP_AEC_FRAME_BYTES, 0);
}

/* A2DP 与通话共用同一个 DAC。通话期间必须让 A2DP 停止推流：
 * 否则两边各自 app_audio_write()，一个 16k 单声道、一个 44.1k 立体声，
 * 会互相把对方的采样率顶掉（听感是两个声音交替卡顿）。 */
extern void bt_audio_sink_set_output_active(bool active);

/* 建立通话音频通路（SCO 已 up 时调用） */
static bk_err_t call_audio_start(void)
{
	bsp_mic_cfg_t mc;

	LOGI("call audio start, codec=%s\r\n",
	     (s_codec == CODEC_VOICE_MSBC) ? "mSBC" : "CVSD");

	/* 0. 让 A2DP 让出 DAC。放最前面：它置位后 A2DP 的 spk 任务下一轮
	 *    就会 app_audio_close()，我们随后的 app_audio_open() 才不会
	 *    和一个正在写 DAC 的线程抢句柄。 */
	bt_audio_sink_set_output_active(false);

	/* 1. DAC 切到 16k 单声道。app_audio_open 内部会先 stop 再 open，
	 *    所以不需要显式 close。 */
	if (app_audio_open(HFP_RATE, 1) != BK_OK) {
		LOGE("call audio open failed\r\n");
		bt_audio_sink_set_output_active(true);
		return BK_FAIL;
	}

	/* 2. 解码器 / 编码器 */
	if (s_codec == CODEC_VOICE_MSBC) {
		if (!s_enc_inited) {
			if (sbc_encoder_init(&s_enc, HFP_RATE, 1) < 0) {
				LOGE("msbc encoder init failed\r\n");
				goto fail_audio;
			}
			sbc_encoder_ctrl(&s_enc, SBC_ENCODER_CTRL_CMD_SET_MSBC_ENCODE_MODE,
			                 (uint32_t)NULL);
			s_enc_inited = true;
		}
		if (!s_dec_inited) {
			/* 两步都要：先 init，再显式打开 mSBC 支持。
			 * 只 init 不 support_msbc 的话，解 mSBC 帧会直接返回错误。 */
			if (bk_sbc_decoder_init(&s_dec) != BK_OK) {
				LOGE("msbc decoder init failed\r\n");
				goto fail_audio;
			}
			bk_sbc_decoder_support_msbc(true);
			s_dec_inited = true;
		}
	}

	/* 3. AEC（软件参考） */
	if (aec_pipeline_start() != BK_OK) {
		LOGE("aec unavailable, fall back to raw mic\r\n");
	}

	/* 4. DMIC：16k 单声道。bsp_mic 的白名单包含 16k。
	 *    stereo=false 让组件走 dual_dmic_sgl_out（双麦取一路）。 */
	bsp_mic_default_cfg(&mc);
	mc.samp_rate = HFP_RATE;
	mc.stereo = false;
	if (bsp_mic_init(&mc) == BK_OK && bsp_mic_start() == BK_OK) {
		s_mic_on = true;
	} else {
		LOGE("mic start failed\r\n");
	}

	s_up_len = 0;
	s_down_wr = 0;
	s_down_rd = 0;
	s_sco_up = true;
	return BK_OK;

fail_audio:
	app_audio_close();
	bt_audio_sink_set_output_active(true);
	return BK_FAIL;
}

static void call_audio_stop(void)
{
	s_sco_up = false;

	if (s_mic_on) {
		bsp_mic_deinit();
		s_mic_on = false;
	}

	aec_pipeline_stop();

	s_up_len = 0;
	app_audio_close();

	/* 通话结束，把 DAC 还给 A2DP。恢复后 A2DP 的 spk 任务会自己
	 * 重新 app_audio_open(44.1k/48k)，不需要这里再开。 */
	bt_audio_sink_set_output_active(true);

	LOGI("call audio stop\r\n");
}

/* ============================ 下行解码 ============================ */

/* SCO 下行数据（蓝牙协议栈任务上下文）：
 *   CVSD：60 点/包，直接是 PCM16
 *   mSBC：2 字节同步头 + 57 字节 SBC 帧，解出 120 点
 *
 * 这里只做解码 + 出声，不做队列中转 —— SCO 的节拍本身就是稳的
 * （7.5ms 一包），再套一层队列只会增加延迟。 */
static void downlink_feed(const uint8_t *data, uint16_t len)
{
	if (!s_sco_up || !data || !len)
		return;

	if (s_codec == CODEC_VOICE_MSBC) {
		const uint8_t *fb = data;
		bk_err_t ret;

		if (len <= HFP_MSBC_HDR_LEN)
			return;

		/* 跳过 mSBC 的 H2 同步头，与官方示例一致 */
		fb += HFP_MSBC_HDR_LEN;
		ret = bk_sbc_decoder_frame_decode(&s_dec, (uint8_t *)fb,
		                                  (uint32_t)(len - HFP_MSBC_HDR_LEN));
		if (ret != BK_OK) {
			/* 解码失败就补静音，绝不能直接丢 —— 丢了会越来越不同步 */
			os_memset(s_dec_pcm, 0, sizeof(s_dec_pcm));
			speak_downlink(s_dec_pcm, HFP_MSBC_SAMPLES);
			return;
		}

		/* 解码器输出是 int32_t pcm_sample[2][128]（每声道一维数组），
		 * 而且高位对齐，低位是 0；取左声道并右移 16 位转成 int16。
		 * 官方示例直接把 pcm_sample 的地址当 uint8* 往下传，那是把
		 * 高位字节当样本用（在它那条 audio_play 路径里恰好能对上它的
		 * 位深约定），这里走的是标准 PCM16 通路，必须自己取高位。 */
		for (uint32_t i = 0; i < HFP_MSBC_SAMPLES; i++)
			s_dec_pcm[i] = (int16_t)(s_dec.pcm_sample[0][i] >> 16);

		speak_downlink(s_dec_pcm, HFP_MSBC_SAMPLES);
	} else {
		/* CVSD 是 8k 窄带，但我们按 16k 开的 DAC。
		 * 直接按 16k 播会快一倍（音调上扬），必须上采样。
		 * 这里用最简单的样点保持（每个点重复两次）：CVSD 本身
		 * 只有 4kHz 带宽，保持法带来的镜像在 8k 以上，听感影响很小，
		 * 而线性插值要点表、要处理跨包边界，收益不明显。 */
		static int16_t up[HFP_CVSD_SAMPLES * 2 * 2] __attribute__((aligned(4)));
		uint32_t n = len / 2;

		if (n > HFP_CVSD_SAMPLES)
			n = HFP_CVSD_SAMPLES;
		for (uint32_t i = 0; i < n; i++) {
			int16_t s = (int16_t)(data[2 * i] | (data[2 * i + 1] << 8));

			up[2 * i] = s;
			up[2 * i + 1] = s;
		}
		speak_downlink(up, n * 2);
	}
}

/* HFP 数据回调（蓝牙栈上下文） */
static void hfp_voice_data_cb(const uint8_t *data, uint16_t len)
{
	downlink_feed(data, len);
}

/* ============================ HFP 事件回调 ============================ */

static void hfp_client_cb(bk_hf_client_cb_event_t event,
                          bk_hf_client_cb_param_t *param)
{
	if (!param)
		return;

	switch (event) {
	case BK_HF_CLIENT_CONNECTION_STATE_EVT:
		post(HFP_MSG_CONN_STATE,
		     (param->conn_state.state == BK_HF_CLIENT_CONNECTION_STATE_CONNECTED) ? 1 : 0);
		if (param->conn_state.state == BK_HF_CLIENT_CONNECTION_STATE_CONNECTED) {
			os_memcpy(s_peer_bda, param->remote_bda, 6);
			s_peer_valid = true;
		}
		break;

	case BK_HF_CLIENT_AUDIO_STATE_EVT:
		/* codec 在 AUDIO_CONNECTED 事件里才会确定下来 */
		if (param->audio_state.state == BK_HF_CLIENT_AUDIO_STATE_CONNECTED)
			s_codec = param->audio_state.codec_type;
		post(HFP_MSG_AUDIO_STATE,
		     (param->audio_state.state == BK_HF_CLIENT_AUDIO_STATE_CONNECTED) ? 1 : 0);
		break;

	case BK_HF_CLIENT_CIND_CALL_EVT:
		post(HFP_MSG_CALL_STATUS, (uint8_t)param->call.status);
		break;

	case BK_HF_CLIENT_CIND_CALL_SETUP_EVT:
		post(HFP_MSG_CALL_SETUP, (uint8_t)param->call_setup.status);
		break;

	case BK_HF_CLIENT_RING_IND_EVT:
		post(HFP_MSG_RING, 1);
		break;

	case BK_HF_CLIENT_CLIP_EVT:
		/* 来电号码：ISR 里不能动队列以外的东西，拷进消息体 */
		if (s_que) {
			hfp_msg_t m;

			os_memset(&m, 0, sizeof(m));
			m.type = HFP_MSG_CLIP;
			if (param->clip.number) {
				os_strncpy(m.text, param->clip.number, sizeof(m.text) - 1);
				m.len = (uint16_t)os_strlen(m.text);
			}
			rtos_push_to_queue(&s_que, &m, BEKEN_NO_WAIT);
		}
		break;

	case BK_HF_CLIENT_VOLUME_CONTROL_EVT:
		/* AG 下发的扬声器音量（0~15）。映射到本机音量，
		 * 让手机侧音量键能调音箱。 */
		if (param->volume_control.type == BK_HF_VOLUME_CONTROL_TARGET_SPK) {
			uint8_t vol = (uint8_t)(param->volume_control.volume * 100 / 15);

			app_state_set_volume(vol);
		}
		break;

	default:
		break;
	}
}

/* ============================ 通话任务 ============================ */

static void hfp_task(void *arg)
{
	hfp_msg_t m;

	(void)arg;

	while (s_run) {
		if (rtos_pop_from_queue(&s_que, &m, 10) != kNoErr) {
			/* 队列空也要跑音频泵：AEC 输入/输出是流式的，
			 * 不能等事件才动。10ms 一轮，对 20ms 的帧长足够。 */
			if (s_sco_up) {
				pump_downlink();
				pump_mic_into_aec();
				pump_uplink();
			}
			continue;
		}

		switch (m.type) {
		case HFP_MSG_CONN_STATE:
			if (!m.val) {
				LOGI("hfp disconnected\r\n");
				s_peer_valid = false;
				if (s_sco_up)
					call_audio_stop();
				set_state(APP_HFP_STATE_IDLE);
			} else {
				LOGI("hfp connected\r\n");
			}
			break;

		case HFP_MSG_AUDIO_STATE:
			if (m.val) {
				/* SCO 建立：通话真正开始，开 16k 音频通路 */
				if (!s_sco_up)
					call_audio_start();
				if (s_state != APP_HFP_STATE_ACTIVE)
					set_state(APP_HFP_STATE_ACTIVE);
			} else {
				if (s_sco_up)
					call_audio_stop();
				/* SCO 断了但可能还在通话（比如 AG 临时收音频），
				 * 真正的状态由 CALL_STATUS 决定，这里不强行清 */
			}
			break;

		case HFP_MSG_CALL_STATUS:
			/* 0 = 无通话，1 = 有通话（active 或 hold） */
			if (!m.val && (s_state == APP_HFP_STATE_ACTIVE))
				set_state(APP_HFP_STATE_IDLE);
			break;

		case HFP_MSG_CALL_SETUP:
			switch (m.val) {
			case BK_HF_CALL_SETUP_STATUS_INCOMING:
				LOGI("incoming call\r\n");
				set_state(APP_HFP_STATE_INCOMING);
				break;
			case BK_HF_CALL_SETUP_STATUS_OUTGOING_DIALING:
			case BK_HF_CALL_SETUP_STATUS_OUTGOING_ALERTING:
				set_state(APP_HFP_STATE_OUTGOING);
				break;
			case BK_HF_CALL_SETUP_STATUS_IDLE:
				/* 建立阶段回到 IDLE：可能是被拒/对方挂断。
				 * 若 READY 的 CALL_STATUS 说明还有通话，
				 * 就保持 ACTIVE 不动。 */
				if ((s_state == APP_HFP_STATE_INCOMING) ||
				    (s_state == APP_HFP_STATE_OUTGOING))
					set_state(APP_HFP_STATE_IDLE);
				break;
			default:
				break;
			}
			break;

		case HFP_MSG_RING:
			/* RING 只是提示振铃，不改变状态（状态靠 CALL_SETUP） */
			break;

		case HFP_MSG_CLIP:
			if (m.len)
				LOGI("caller: %s\r\n", m.text);
			break;

		case HFP_MSG_ANSWER:
			if (s_peer_valid) {
				LOGI("answer\r\n");
				bk_bt_hf_client_answer_call(s_peer_bda);
			}
			break;

		case HFP_MSG_HANGUP:
			if (s_peer_valid) {
				LOGI("hangup\r\n");
				bk_bt_hf_client_reject_call(s_peer_bda);
			}
			break;

		default:
			break;
		}

		/* 处理完事件的同一轮也泵一次音频，降低首包延迟 */
		if (s_sco_up) {
			pump_downlink();
			pump_mic_into_aec();
			pump_uplink();
		}
	}

	s_task = NULL;
	rtos_delete_thread(NULL);
}

/* ============================ 对外 API ============================ */

void app_hfp_init(void)
{
	bt_err_t ret;

	if (s_ready)
		return;

	if (rtos_init_queue(&s_que, "app_hfp", sizeof(hfp_msg_t), 12) != kNoErr) {
		LOGE("queue init failed\r\n");
		return;
	}

	s_run = true;
	if (rtos_create_thread(&s_task, HFP_TASK_PRIO, "app_hfp",
	                       (beken_thread_function_t)hfp_task,
	                       HFP_TASK_STACK, NULL) != kNoErr) {
		LOGE("task create failed\r\n");
		s_run = false;
		return;
	}

	ret = bk_bt_hf_client_register_callback(hfp_client_cb);
	if (ret) {
		LOGE("register cb failed: %d\r\n", ret);
		goto fail;
	}

	/* 1 = 支持 mSBC。手机侧能协商成功就会走 16k 宽带。 */
	ret = bk_bt_hf_client_init(1);
	if (ret) {
		LOGE("hf client init failed: %d\r\n", ret);
		goto fail;
	}

	ret = bk_bt_hf_client_register_data_callback(hfp_voice_data_cb);
	if (ret) {
		LOGE("register data cb failed: %d\r\n", ret);
		goto fail;
	}

	s_ready = true;
	LOGI("hfp ready (msbc supported)\r\n");
	return;

fail:
	s_run = false;
}

bool app_hfp_is_ready(void)
{
	return s_ready;
}

app_hfp_state_t app_hfp_state(void)
{
	return s_state;
}

bool app_hfp_is_ringing(void)
{
	return s_state == APP_HFP_STATE_INCOMING;
}

bool app_hfp_in_call(void)
{
	return (s_state == APP_HFP_STATE_ACTIVE) ||
	       (s_state == APP_HFP_STATE_OUTGOING);
}

void app_hfp_answer(void)
{
	if (s_state == APP_HFP_STATE_INCOMING)
		post(HFP_MSG_ANSWER, 1);
}

void app_hfp_hangup(void)
{
	if ((s_state == APP_HFP_STATE_ACTIVE) ||
	    (s_state == APP_HFP_STATE_OUTGOING) ||
	    (s_state == APP_HFP_STATE_INCOMING))
		post(HFP_MSG_HANGUP, 1);
}

bool app_hfp_shake(void)
{
	if (s_state == APP_HFP_STATE_INCOMING) {
		LOGI("shake -> answer\r\n");
		post(HFP_MSG_ANSWER, 1);
		return true;
	}

	if ((s_state == APP_HFP_STATE_ACTIVE) ||
	    (s_state == APP_HFP_STATE_OUTGOING)) {
		LOGI("shake -> hangup\r\n");
		post(HFP_MSG_HANGUP, 1);
		return true;
	}

	return false;
}
