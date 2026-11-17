// Copyright 2026
// 蓝牙免提通话（HFP HF）+ 软件 AEC
//
// 层次：
//   bt/hfp_hf_demo.c   官方 HFP HF 示例（AT 状态机、SCO 编解码、回调注册）
//   app_hfp.c          本文件：把官方示例的"回调/线程"改造成能接进本产品的形态
//     - 只保留通话相关的 AT 交互，去掉示例里那一长串自测序列
//         （query operator / CNUM / BVRA / dial 112 ... 那些是给实验室用的，
//          真机上会给每次连接都拨一遍 112，必须清掉）
//     - 音频通路走本工程的 app_audio + bsp_mic，而不是示例自带的
//         audio_play/audio_record（那两个是 headset 工程的独立 ADK 子系统）
//     - 扬声器参考信号回灌给 aec_algorithm，实现软件 AEC
//
// 数据流：
//   下行（远端说话）：SCO -> [mSBC 解码] -> 参考环 -> AEC ref 端口
//                                              -> DAC（同时出声）
//   上行（本机说话）  ：DMIC 16k -> AEC source 端口 -> [mSBC 编码] -> SCO
//
// 为什么 AEC 参考必须自己回灌：本板是双 DMIC 数字麦，没有"把喇叭输出
// 再采回 ADC 右声道"的模拟回路，AEC_MODE_HARDWARE 这条路走不通，
// 只能用 AEC_MODE_SOFTWARE，由软件把下行 PCM 按帧送进参考端口。

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
	APP_HFP_STATE_IDLE = 0,   /* 未连接 / 无通话 */
	APP_HFP_STATE_INCOMING,   /* 来电振铃 */
	APP_HFP_STATE_OUTGOING,   /* 去电（拨号中/对方振铃） */
	APP_HFP_STATE_ACTIVE,     /* 通话中 */
} app_hfp_state_t;

/* 注册 HFP 回调并初始化通话子系统。由 app_bt_init() 在蓝牙栈就绪后调用。
 * 内部会创建消息任务，不阻塞。 */
void app_hfp_init(void);

bool app_hfp_is_ready(void);

/* 当前通话状态（UI 用）。任何线程可调，只读一个 volatile。 */
app_hfp_state_t app_hfp_state(void);

/* 有来电正在振铃 */
bool app_hfp_is_ringing(void);

/* 正在通话（含去电已建立） */
bool app_hfp_in_call(void);

/* 接听 / 挂断。由摇一摇、按键、UI 按钮调用，内部投消息，不阻塞。
 * 无来电无通话时是安全的空操作。 */
void app_hfp_answer(void);
void app_hfp_hangup(void);

/**
 * @brief 摇一摇的统一入口：来电时接听，通话中挂断。
 *
 * @return true 表示这次摇动已经被通话消费掉了，调用方**不要**再去切歌；
 *         false 表示当前没有来电/通话，调用方按原逻辑处理（下一首等）。
 */
bool app_hfp_shake(void);

#ifdef __cplusplus
}
#endif
