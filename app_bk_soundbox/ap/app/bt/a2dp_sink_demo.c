#include <components/system.h>
#include <os/os.h>
#include <os/mem.h>
#include <os/str.h>
#include <string.h>
#include <stdint.h>
#include <stdio.h>

#include "bt_config.h"

#include "a2dp_sink_demo.h"

#include "components/bluetooth/bk_dm_bluetooth_types.h"
#include "components/bluetooth/bk_dm_bt_types.h"
#include "components/bluetooth/bk_dm_bt.h"
#include "components/bluetooth/bk_dm_a2dp_types.h"
#include "components/bluetooth/bk_dm_a2dp.h"
#include "components/bluetooth/bk_dm_gap_bt.h"

#include <driver/sbc_types.h>
//#include <driver/aud_types.h>
//#include <driver/aud.h>
#ifdef CONFIG_A2DP_AUDIO
//#include "aud_intf.h"
#endif
#include <driver/sbc.h>
#include "ring_buffer_node.h"
#include "components/bluetooth/bk_dm_avrcp.h"
#include "bluetooth_storage.h"
#if CONFIG_WIFI_COEX_SCHEME
#include "bk_coex_ext.h"
#endif
#include "bt_manager.h"

#include "bk_gpio.h"

#if CONFIG_AAC_DECODER
#include "modules/aacdec.h"
#include "bk_aac_decoder.h"
#endif

#include <driver/aud_dac.h>
#include <driver/aud_dac_types.h>

#include "app_audio.h"
#include "app_bt_link.h"

#include "mpeg4_latm_dec.h"

#define TAG "a2dp_sink"

#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)
#define LOGD(...) BK_LOGD(TAG, ##__VA_ARGS__)

#define CHECK_NULL(ptr) do {\
        if (ptr == NULL) {\
            LOGI("CHECK_NULL fail \n");\
            return;\
        }\
    } while(0)


#define CODEC_AUDIO_SBC                      0x00U
#define CODEC_AUDIO_AAC                      0x02U

#define A2DP_SBC_CHANNEL_MONO                        0x08U
#define A2DP_SBC_CHANNEL_DUAL                        0x04U
#define A2DP_SBC_CHANNEL_STEREO                      0x02U
#define A2DP_SBC_CHANNEL_JOINT_STEREO                0x01U


#define A2DP_CACHE_BUFFER_SIZE  ((CONFIG_A2DP_CACHE_FRAME_NUM + 2) * 1024)
#define A2DP_SBC_FRAME_BUFFER_MAX_SIZE  128   /**< A2DP SBC encoded frame maximum size */
#define A2DP_AAC_FRAME_BUFFER_MAX_SIZE  1024  /**< A2DP AAC encoded frame maximum size */

#define A2DP_SPEAKER_THREAD_PRI             BEKEN_DEFAULT_WORKER_PRIORITY-2
#define A2DP_SPEAKER_WRITE_SBC_FRAME_NUM    7
#define A2DP_SBC_MAX_FRAME_NUMS             0xF*5*1
#define A2DP_SBC_FRAME_HEADER_LEN           13

#if CONFIG_AAC_DECODER
#define A2DP_ACC_SINGLE_CHANNEL_MAX_FRAM_SIZE   768   //1024(samples)/48k/s(sample rate)*288k/s (max bitrate)/8
#define A2DP_AAC_MAX_FRAME_NUMS                 5     //21ms/frame*5
#endif

#define AVRCP_PASSTHROUTH_CMD_TIMEOUT                     2000

static int speaker_task_init();
static void speaker_task(void *arg);


enum
{
    BT_AUDIO_MSG_NULL = 0,
    BT_AUDIO_D2DP_START_MSG = 1,
    BT_AUDIO_D2DP_STOP_MSG = 2,
    BT_AUDIO_D2DP_DATA_IND_MSG = 3,
    BT_AUDIO_D2DP_SEND_DATA_2_SPK_MSG = 4,
    BT_AUDIO_WIFI_STATE_UPDATE_MSG = 5,
};


typedef struct
{
    uint8_t type;
    uint16_t len;
    char *data;
} bt_audio_sink_demo_msg_t;

typedef struct
{
    uint8_t wifi_state;
    uint8_t a2dp_state;
    uint8_t avrcp_state;
    beken2_timer_t avrcp_connect_tmr;
} bt_env_s;


static bk_a2dp_mcc_t bt_audio_a2dp_sink_codec = {0};


static beken_queue_t bt_audio_sink_demo_msg_que = NULL;
static beken_thread_t bt_audio_sink_demo_thread_handle = NULL;

#ifdef CONFIG_A2DP_AUDIO
static sbcdecodercontext_t bt_audio_sink_sbc_decoder;
#if CONFIG_AAC_DECODER
static aacdecodercontext_t bt_audio_sink_aac_decoder;
#endif

static RingBufferNodeContext s_a2dp_frame_nodes;
static uint8_t s_spk_is_started = 0;
static uint16_t frame_length = 0;
static uint8_t *p_cache_buff = NULL;

static beken_thread_t a2dp_speaker_thread_handle = NULL;
static beken_semaphore_t a2dp_speaker_sema = NULL;
static beken_semaphore_t a2dp_speaker_exit_sema = NULL;
//static beken_timer_t a2dp_speaker_tmr = {0};


#endif
static uint8_t s_a2dp_vol = DEFAULT_A2DP_VOLUME;//0~0x7f
/* 上一次回发给主机的音量值，用来避免同一值重复回发（见 SET_ABSOLUTE_VOLUME 处理） */
static uint8_t s_a2dp_vol_notified = 0xFF;
/* 音量回发去抖：收到 SET 立即改 DAC 增益；CHANGED 等值稳定 80ms 后再回。
 * 手机拖动时每个中间值都回 CHANGED，会让滞后的值追上滑块（音量延迟/被拉回）。 */
static uint8_t s_a2dp_vol_pending;
static uint32_t s_a2dp_vol_pending_ms;
static uint16_t s_tg_current_registered_noti;

static bt_env_s s_bt_env;
#if CONFIG_WIFI_COEX_SCHEME
static coex_to_bt_func_p_t s_coex_to_bt_func = {0};
#endif

static beken_semaphore_t s_bt_api_event_cb_sema = NULL;
static beken_semaphore_t s_bt_avrcp_event_cb_sema = NULL;

/* BT 模式是否允许出声。离开 BT 模式时置 false，speaker_task 会关闭
 * app_audio，让 SD/USB 模式独占 DAC。A2DP 数据仍会到达，但直接丢弃。 */
static volatile bool s_bt_output_active = true;
static volatile bool s_bt_audio_opened = false;

/* ==================== BT 音频停滞看门狗 ====================
 *
 * "放一段时间后卡住"的通用兜底：只要发现**主机还在送包**、但本机已经
 * 超过 BT_AUDIO_STALL_MS 没成功写进 DAC，就认定音频通路死了，
 * 关掉并重建（DAC 通道 + ring buffer），把"永久卡死"降级成"卡 3 秒自愈"。
 *
 * 为什么必须区分"有包/没包"：手机暂停时本来就没有包，不能当成故障。
 * 只有 s_a2dp_pkt_cnt 还在涨、而写一直不成功，才是通路问题。 */
#define BT_AUDIO_STALL_MS   3000
static volatile uint32_t s_last_write_ok_ms;
static volatile uint32_t s_a2dp_pkt_cnt;
static volatile uint32_t s_audio_restart_cnt;

/* 关闭音频通路并丢掉已缓存的帧。失败/停滞后的统一恢复动作，不碰 BT 链路。 */
static void bt_audio_pipeline_reset(bool *audio_open)
{
    app_audio_close();
    if (audio_open)
        *audio_open = false;
    s_bt_audio_opened = false;

    /* 丢掉过期帧，避免重建后先播一段旧数据 */
    while (ring_buffer_node_get_fill_nodes(&s_a2dp_frame_nodes))
        ring_buffer_node_take_read_node(&s_a2dp_frame_nodes);

    s_last_write_ok_ms = rtos_get_time();
    s_audio_restart_cnt++;
    LOGE("bt audio pipeline reset (#%u, pkt=%u)\n",
         (unsigned)s_audio_restart_cnt, (unsigned)s_a2dp_pkt_cnt);
}

void bt_audio_sink_set_output_active(bool active)
{
    s_bt_output_active = active;
    if (a2dp_speaker_sema)
        rtos_set_semaphore(&a2dp_speaker_sema);
}

bool bt_audio_sink_is_output_active(void)
{
    return s_bt_output_active;
}

bool bt_audio_sink_is_audio_opened(void)
{
    return s_bt_audio_opened;
}

/* 丢弃包计数：数据回调里累加，心跳日志要读，所以放文件作用域 */
static volatile uint32_t s_a2dp_drop_cnt;

void bt_audio_sink_stats(uint32_t *pkt, uint32_t *drop, uint32_t *resets)
{
    if (pkt)
        *pkt = s_a2dp_pkt_cnt;
    if (drop)
        *drop = s_a2dp_drop_cnt;
    if (resets)
        *resets = s_audio_restart_cnt;
}

/* 本函数收到的 gain 是 DAC 数字增益 0~0x3F（调用方传 s_a2dp_vol >> 1），
 * 映射到应用音量 0~100。 */
static bk_err_t bk_bt_dac_set_gain(uint8_t gain)
{
    uint8_t vol = (uint8_t)(((uint32_t)gain * 100 + 31) / 63);

    if (vol > 100)
        vol = 100;
    app_audio_set_volume(vol);
    return BK_OK;
}

void avrcp_connect_timer_hdl(void *param, unsigned int ulparam)
{
    rtos_deinit_oneshot_timer(&s_bt_env.avrcp_connect_tmr);
    if (0 == s_bt_env.avrcp_state)
    {
        bk_bt_avrcp_connect(bt_manager_get_connected_device());
    }
}

static void bk_bt_start_avrcp_connect(void)
{
    if (!rtos_is_oneshot_timer_init(&s_bt_env.avrcp_connect_tmr))
    {
        rtos_init_oneshot_timer(&s_bt_env.avrcp_connect_tmr, 300, (timer_2handler_t)avrcp_connect_timer_hdl, NULL, 0);
        rtos_start_oneshot_timer(&s_bt_env.avrcp_connect_tmr);
    }
}

static bk_err_t one_spk_frame_played_cmpl_handler(unsigned int size)
{
    bt_audio_sink_demo_msg_t demo_msg;
    int rc = -1;

    if (bt_audio_sink_demo_msg_que == NULL)
    {
        return BK_OK;
    }

    demo_msg.type = BT_AUDIO_D2DP_SEND_DATA_2_SPK_MSG;
    demo_msg.len = 0;

    rc = rtos_push_to_queue(&bt_audio_sink_demo_msg_que, &demo_msg, BEKEN_NO_WAIT);

    if (kNoErr != rc)
    {
        LOGE("%s, send queue failed\r\n", __func__);
    }
    return BK_OK;
}

void bt_audio_sink_demo_main(void *arg)
{
    uint8_t recon_addr[6] = {0};
    if ((bluetooth_storage_get_newest_linkkey_info(recon_addr,NULL)) < 0)
    {
		LOGI("%s can't find linkkey info\n", __func__);
        bt_manager_set_mode(BT_MNG_MODE_PAIRING);
    }
    else
    {
		LOGI("%s find addr\n", __func__);
        if(bluetooth_storage_find_volume_by_addr(recon_addr, &s_a2dp_vol) < 0)
        {
            s_a2dp_vol = DEFAULT_A2DP_VOLUME; //default vol
        }

        if(s_a2dp_vol == 0)
		{
		    s_a2dp_vol = DEFAULT_A2DP_VOLUME;
		}

        LOGI("initial volume %d %02x:%02x:%02x:%02x:%02x:%02x\r\n",s_a2dp_vol,
                        recon_addr[5],
                        recon_addr[4],
                        recon_addr[3],
                        recon_addr[2],
                        recon_addr[1],
                        recon_addr[0]
                        );

        bt_manager_start_reconnect(recon_addr, 1);
    }

    while (1)
    {
        bk_err_t err;
        bt_audio_sink_demo_msg_t msg;

        err = rtos_pop_from_queue(&bt_audio_sink_demo_msg_que, &msg, BEKEN_WAIT_FOREVER);

        if (kNoErr == err)
        {
            switch (msg.type)
            {
            case BT_AUDIO_D2DP_START_MSG:
            {
                LOGI("BT_AUDIO_D2DP_START_MSG \r\n");

#ifdef CONFIG_A2DP_AUDIO
                bk_a2dp_mcc_t *p_codec_info = (bk_a2dp_mcc_t *)msg.data;
                if (CODEC_AUDIO_SBC == p_codec_info->type)
                {
                    uint8_t chnl_mode = p_codec_info->cie.sbc_codec.channel_mode;
                    uint8_t chnls = p_codec_info->cie.sbc_codec.channels;
                    uint8_t subbands = p_codec_info->cie.sbc_codec.subbands;
                    uint8_t blocks = p_codec_info->cie.sbc_codec.block_len;
                    uint8_t bitpool = p_codec_info->cie.sbc_codec.bit_pool;
                    if(chnl_mode == A2DP_SBC_CHANNEL_MONO || chnl_mode == A2DP_SBC_CHANNEL_DUAL)
                    {
                         frame_length = 4 + ((4 * subbands * chnls)>>3) + ((blocks*chnls*bitpool+7)>>3);
                    }else if(chnl_mode == A2DP_SBC_CHANNEL_STEREO)
                    {
                        frame_length = 4 + ((4 * subbands * chnls)>>3) + ((blocks*bitpool+7)>>3);
                    }else //A2DP_SBC_CHANNEL_JOINT_STEREO
                    {
                        frame_length = 4 + ((4 * subbands * chnls)>>3) + ((subbands+blocks*bitpool+7)>>3);
                    }

                    //get_sbc_encoder_len(p_codec_info);

                    LOGI("cm:%d, c:%d, s:%d, b:%d, bi:%d, frame_length:%d \n",chnl_mode, chnls, subbands,blocks, bitpool, frame_length);
                    bk_sbc_decoder_init(&bt_audio_sink_sbc_decoder);
                    if(p_cache_buff)
                    {
                        LOGE("p_cache_buffer remalloc, error !!");
                    }else
                    {
                        p_cache_buff  = (uint8_t *)os_malloc((frame_length + 2) * A2DP_SBC_MAX_FRAME_NUMS); //max number of frames 4bit
                        if (!p_cache_buff)
                            p_cache_buff = (uint8_t *)psram_malloc((frame_length + 2) * A2DP_SBC_MAX_FRAME_NUMS);
                        if (!p_cache_buff)
                        {
                            LOGE("%s, malloc cache buf failed!!!\r\n", __func__);
                        }
                        else
                        {
                            ring_buffer_node_init(&s_a2dp_frame_nodes, p_cache_buff, frame_length + 2, A2DP_SBC_MAX_FRAME_NUMS);
                        }
                    }
                }
#if (CONFIG_AAC_DECODER)
                else if(CODEC_AUDIO_AAC == p_codec_info->type)
                {
                    uint8_t channle = p_codec_info->cie.aac_codec.channels;
                    uint32_t sample_rate = p_codec_info->cie.aac_codec.sample_rate;
                    int ret = bk_aac_decoder_init(&bt_audio_sink_aac_decoder, sample_rate, channle);
                    if(ret < 0)
                    {
                        LOGE("aac deocder init fail \n");
                        return;
                    }
                	frame_length = A2DP_ACC_SINGLE_CHANNEL_MAX_FRAM_SIZE*channle;
                	LOGI("ch:%d, frame_length:%d \n", channle, frame_length);
                    if(p_cache_buff)
                    {
                        LOGE("p_cache_buffer remalloc, error !!");
                    }else
                    {
                        p_cache_buff  = (uint8_t *)os_malloc(frame_length * A2DP_AAC_MAX_FRAME_NUMS); //max number of frames 4bit
                        if (!p_cache_buff)
                        {
                            LOGE("%s, malloc cache buf failed!!!\r\n", __func__);
                        }
                        else
                        {
                            ring_buffer_node_init(&s_a2dp_frame_nodes, p_cache_buff, frame_length, A2DP_AAC_MAX_FRAME_NUMS);
                        }
                    }
                }
#endif
                s_spk_is_started = 1;
                speaker_task_init();
#endif
                os_free(msg.data);
            }
            break;

            case BT_AUDIO_D2DP_DATA_IND_MSG:
            {
#ifdef CONFIG_A2DP_AUDIO

                uint8 *fb = (uint8_t *)msg.data;
                if(s_spk_is_started)
                {
                    if (CODEC_AUDIO_SBC == bt_audio_a2dp_sink_codec.type)
                    {
                        uint8_t payload_header = *fb++;
                        uint8_t frame_num = payload_header&0xF;

                        if(msg.len - 1 != frame_length*frame_num)
                        {
                            //LOGI("recv undef sbc, payload_header %d, payload_len: %d, frame_num:%d %d\n", payload_header, msg.len - 1, frame_num, frame_length);
                        }

                        if((msg.len - 1) % frame_num)
                        {
                            LOGE("%s frame len invalid\n", __func__);
                        }


                        for(uint8_t i = 0; i < frame_num; i++)
                        {
                            if (ring_buffer_node_get_free_nodes(&s_a2dp_frame_nodes))
                            {
                                uint16_t tmp_len = (msg.len - 1) / frame_num;

                                ring_buffer_node_write(&s_a2dp_frame_nodes, fb, tmp_len);
                                fb += tmp_len;
                            }
                            else
                            {
                                LOGI("A2DP frame nodes buffer(sbc) is full %d %d\n", i, frame_num);
                                //os_free(msg.data);
                                break;
                            }
                        }

                    }
#if CONFIG_AAC_DECODER
                    else if(CODEC_AUDIO_AAC == bt_audio_a2dp_sink_codec.type)
                    {
                        // LOGI("-> %d \n", msg.len);
                        uint8_t *inbuf = &fb[9];
                        uint32_t inlen = 0;
                        uint8_t  len   = 255;

                        do
                        {
                            inlen += len = *inbuf++;
                        }
                        while (len == 255);

                        {
                            if (ring_buffer_node_get_free_nodes(&s_a2dp_frame_nodes))
                            {
                                uint8_t *output = 0;
                                uint32_t output_len = 0;

                                if(mpeg4_latm_decode(fb, msg.len, &output, &output_len))
                                {
                                    LOGE("====%s latm decode err, discard it\n", __func__);
                                    os_free(msg.data);
                                    break;
                                }
                                else if(msg.len - (output - fb) != output_len ||
                                                inlen != output_len ||
                                                output != inbuf)
                                {
                                    LOGE("====%s len not match, total %d, type1 offset %d len %d, type2 offset %d len %d\n", __func__, msg.len, output - fb, inbuf - fb, output_len, inlen);
                                }

                                uint8_t *node = ring_buffer_node_get_write_node(&s_a2dp_frame_nodes);

                                os_memcpy(node, &inlen, sizeof(inlen));
                                os_memcpy(node + sizeof(inlen), inbuf, inlen);
                            }
                            else
                            {
                                LOGI("A2DP frame nodes buffer(aac) is full\n");
                                os_free(msg.data);
                                break;
                            }
                        }
                    }
#endif
                    else
                    {
                        LOGE("%s, Unsupported a2dp codec %d \r\n", __func__, bt_audio_a2dp_sink_codec.type);
                    }
                    if(a2dp_speaker_sema)
                    {
                        rtos_set_semaphore(&a2dp_speaker_sema);
                    }
                }
                os_free(msg.data);
#else
                os_free(msg.data);
#endif
            }
            break;

            case BT_AUDIO_D2DP_STOP_MSG:
            {
                LOGI("BT_AUDIO_D2DP_STOP_MSG \r\n");
#ifdef CONFIG_A2DP_AUDIO

                if(a2dp_speaker_thread_handle)
                {
                    s_spk_is_started = 0;
                    if (kNoErr != rtos_init_semaphore(&a2dp_speaker_exit_sema, 1))
                    {
                        LOGE("init sema fail, %d \n", __LINE__);
                    }
                    if(a2dp_speaker_sema)
                    {
                        rtos_set_semaphore(&a2dp_speaker_sema);
                    }
                    LOGI("%s wait thread end\n", __func__);
                    if (a2dp_speaker_exit_sema)
                    {
                        rtos_get_semaphore(&a2dp_speaker_exit_sema, BEKEN_WAIT_FOREVER);
                    }
                    LOGI("%s thread end !!!\n", __func__);
                    a2dp_speaker_thread_handle = NULL;
                    if (a2dp_speaker_exit_sema)
                    {
                        rtos_deinit_semaphore(&a2dp_speaker_exit_sema);
                        a2dp_speaker_exit_sema = NULL;
                    }
                }

#endif

            }
            break;

            case BT_AUDIO_D2DP_SEND_DATA_2_SPK_MSG:
            {

            }
            break;

            case BT_AUDIO_WIFI_STATE_UPDATE_MSG:
            {
                LOGI("BT_AUDIO_WIFI_STATE_UPDATE_MSG \r\n");
                uint8_t reconn_addr[8] = {0};
                bt_manager_get_reconnect_device(reconn_addr);
                if (0 == s_bt_env.wifi_state && BT_STATE_WAIT_FOR_RECONNECT == bt_manager_get_connect_state())
                {
                    bt_manager_start_reconnect(reconn_addr, 1);
                }

                if (s_bt_env.wifi_state && BT_STATE_RECONNECTING == bt_manager_get_connect_state())
                {
                   bk_bt_gap_create_conn_cancel(reconn_addr);
                }
            }
            break;

            default:
                break;
            }
        }
    }

    rtos_deinit_queue(&bt_audio_sink_demo_msg_que);
    bt_audio_sink_demo_msg_que = NULL;
    bt_audio_sink_demo_thread_handle = NULL;
    rtos_delete_thread(NULL);
}

int bt_audio_sink_demo_task_init(void)
{
    bk_err_t ret = BK_OK;

    if ((!bt_audio_sink_demo_thread_handle) && (!bt_audio_sink_demo_msg_que))
    {
        ret = rtos_init_queue(&bt_audio_sink_demo_msg_que,
                              "bt_audio_sink_demo_msg_que",
                              sizeof(bt_audio_sink_demo_msg_t),
                              BT_AUDIO_SINK_DEMO_MSG_COUNT);

        if (ret != kNoErr)
        {
            LOGE("bt_audio sink demo msg queue failed \r\n");
            return BK_FAIL;
        }

        ret = rtos_create_thread(&bt_audio_sink_demo_thread_handle,
                                 A2DP_SINK_DEMO_TASK_PRIORITY,
                                 "bt_audio_sink_demo",
                                 (beken_thread_function_t)bt_audio_sink_demo_main,
                                 4096,
                                 (beken_thread_arg_t)0);

        if (ret != kNoErr)
        {
            LOGE("bt_audio sink demo task fail \r\n");
            rtos_deinit_queue(&bt_audio_sink_demo_msg_que);
            bt_audio_sink_demo_msg_que = NULL;
            bt_audio_sink_demo_thread_handle = NULL;
        }

        return kNoErr;
    }
    else
    {
        return kInProgressErr;
    }
}

void bt_audio_sink_media_data_ind(const uint8_t *data, uint16_t data_len)
{
    bt_audio_sink_demo_msg_t demo_msg;
    int rc = -1;
    /* 数据流统计：定位"放一段时间后卡住"用。
     *   pkt   收到的包数（A2DP 每包约 1 个 SBC frame，~100 包/秒）
     *   drop  分配失败或入队失败被丢弃的包数
     * 只要 drop 开始持续增长，就说明是内存/队列耗尽，而不是 BT 链路问题。 */
    static uint32_t s_pkt;

    /* 看门狗用的"到达计数"：无条件先加，不管后面分配/入队是否成功。
     * 这样"有包到达但写不进 DAC"能被准确识别。 */
    s_a2dp_pkt_cnt++;

    os_memset(&demo_msg, 0x0, sizeof(bt_audio_sink_demo_msg_t));

    if (bt_audio_sink_demo_msg_que == NULL)
    {
        return;
    }

    /* 【必须用 PSRAM，不能用 os_malloc】
     *
     * 这个回调是 A2DP 的**数据流热路径**：每包一次 malloc + 消费端一次
     * free，1 秒上百次，连续几十分钟。os_malloc 走的是内部堆（总量只有
     * 一百多 KB，且被 LVGL 任务栈、各组件长期块切得七零八落），这种
     * 高频等长分配/释放会迅速把堆切成碎块 —— 表现为**刚开始播放正常，
     * 30 秒到几分钟后 os_malloc 开始失败**，丢包后音频彻底断掉，
     * 而 BT 链路本身还是连着的（所以看上去像"卡住"而不是"断开"）。
     *
     * PSRAM 池有 4MB 级别，同样频率的分配释放碎片风险低两个数量级。
     * 释放侧不用改：`os/mem.h` 里 `#define psram_free os_free`，
     * 统一的 free 能正确回收 PSRAM 指针。 */
    demo_msg.data = (char *) psram_malloc(data_len);

    if (demo_msg.data == NULL)
    {
        s_a2dp_drop_cnt++;
        if ((s_a2dp_drop_cnt & 0x3F) == 1)
        {
            LOGE("%s, alloc %u failed (drop=%u/%u, psram=%uKB)\n", __func__,
                 data_len, (unsigned)s_a2dp_drop_cnt, s_pkt,
                 (unsigned)(rtos_get_psram_free_heap_size() / 1024));
        }
        return;
    }

    os_memcpy(demo_msg.data, data, data_len);
    demo_msg.type = BT_AUDIO_D2DP_DATA_IND_MSG;
    demo_msg.len = data_len;

    rc = rtos_push_to_queue(&bt_audio_sink_demo_msg_que, &demo_msg, BEKEN_NO_WAIT);

    if (kNoErr != rc)
    {
        s_a2dp_drop_cnt++;
        if ((s_a2dp_drop_cnt & 0x3F) == 1)
        {
            LOGE("%s, queue full (drop=%u/%u)\n", __func__,
                 (unsigned)s_a2dp_drop_cnt, s_pkt);
        }

        if (demo_msg.data)
        {
            os_free(demo_msg.data);
        }
        return;
    }

    s_pkt++;
    if ((s_pkt % 500) == 0)
    {
        LOGI("a2dp stream: pkt=%u drop=%u heap=%uKB psram=%uKB\n",
             s_pkt, (unsigned)s_a2dp_drop_cnt,
             (unsigned)(rtos_get_free_heap_size() / 1024),
             (unsigned)(rtos_get_psram_free_heap_size() / 1024));
    }
}

void bt_audio_a2dp_sink_suspend_ind(void)
{
    bt_audio_sink_demo_msg_t demo_msg;
    int rc = -1;

    os_memset(&demo_msg, 0x0, sizeof(bt_audio_sink_demo_msg_t));

    if (bt_audio_sink_demo_msg_que == NULL)
    {
        return;
    }

    demo_msg.type = BT_AUDIO_D2DP_STOP_MSG;
    demo_msg.len = 0;

    rc = rtos_push_to_queue(&bt_audio_sink_demo_msg_que, &demo_msg, BEKEN_NO_WAIT);

    if (kNoErr != rc)
    {
        LOGE("%s, send queue failed\r\n", __func__);
    }
}

void bt_audio_a2dp_sink_start_ind(bk_a2dp_mcc_t *codec)
{
    bt_audio_sink_demo_msg_t demo_msg;
    int rc = -1;

    os_memset(&demo_msg, 0x0, sizeof(bt_audio_sink_demo_msg_t));

    if (bt_audio_sink_demo_msg_que == NULL)
    {
        return;
    }

    demo_msg.data = (char *) os_malloc(sizeof(bk_a2dp_mcc_t));

    if (demo_msg.data == NULL)
    {
        LOGE("%s, malloc failed\r\n", __func__);
        return;
    }

    os_memcpy(demo_msg.data, codec, sizeof(bk_a2dp_mcc_t));
    demo_msg.type = BT_AUDIO_D2DP_START_MSG;
    demo_msg.len = sizeof(bk_a2dp_mcc_t);

    rc = rtos_push_to_queue(&bt_audio_sink_demo_msg_que, &demo_msg, BEKEN_NO_WAIT);

    if (kNoErr != rc)
    {
        LOGE("%s, send queue failed\r\n", __func__);
    }
}

static bk_a2dp_audio_state_t s_audio_state = BK_A2DP_AUDIO_STATE_SUSPEND;

void bk_bt_app_a2dp_sink_cb(bk_a2dp_cb_event_t event, bk_a2dp_cb_param_t *p_param)
{
    LOGI("%s event: %d\r\n", __func__, event);

    bk_a2dp_cb_param_t *a2dp = (bk_a2dp_cb_param_t *)(p_param);

    switch (event)
    {
    case BK_A2DP_PROF_STATE_EVT:
    {
        LOGI("a2dp prof init action %d status %d reason %d\r\n",
                        p_param->a2dp_prof_stat.action, p_param->a2dp_prof_stat.status, p_param->a2dp_prof_stat.reason);

        if (!p_param->a2dp_prof_stat.status)
        {
            if (s_bt_api_event_cb_sema)
            {
                rtos_set_semaphore( &s_bt_api_event_cb_sema );
            }
        }
    }
    break;

    case BK_A2DP_CONNECTION_STATE_EVT:
    {
        uint8_t *bda = a2dp->conn_state.remote_bda;
        LOGI("A2DP connection state: %d, [%02x:%02x:%02x:%02x:%02x:%02x]\r\n",
                  a2dp->conn_state.state, bda[5], bda[4], bda[3], bda[2], bda[1], bda[0]);

        if (BK_A2DP_CONNECTION_STATE_DISCONNECTED == a2dp->conn_state.state)
        {
            s_bt_env.a2dp_state = 0;
            app_bt_on_a2dp_stop();
            if (BK_A2DP_AUDIO_STATE_STARTED == s_audio_state)
            {
                s_audio_state = BK_A2DP_AUDIO_STATE_SUSPEND;
                bt_audio_a2dp_sink_suspend_ind();
            }
        }
        else if (BK_A2DP_CONNECTION_STATE_CONNECTED == a2dp->conn_state.state)
        {
            bt_manager_set_connect_state(BT_STATE_PROFILE_CONNECTED);
            s_bt_env.a2dp_state = 1;
            app_bt_on_a2dp_connected();
            //bluetooth_storage_update_to_newest(bda);
            //bluetooth_storage_sync_to_flash();
            if (0 == s_bt_env.avrcp_state)
            {
                bk_bt_start_avrcp_connect();
            }
        }
    }
    break;

    case BK_A2DP_AUDIO_STATE_EVT:
    {
        LOGI("A2DP audio state: %d\r\n", a2dp->audio_state.state);

        if (BK_A2DP_AUDIO_STATE_STARTED == a2dp->audio_state.state)
        {
            s_audio_state = a2dp->audio_state.state;
            bt_audio_a2dp_sink_start_ind(&bt_audio_a2dp_sink_codec);
            if (CODEC_AUDIO_SBC == bt_audio_a2dp_sink_codec.type)
                app_bt_on_a2dp_start(bt_audio_a2dp_sink_codec.cie.sbc_codec.sample_rate,
                                     CONFIG_BOARD_AUDIO_CHANNLE_NUM);
#if (CONFIG_AAC_DECODER)
            else
                app_bt_on_a2dp_start(bt_audio_a2dp_sink_codec.cie.aac_codec.sample_rate,
                                     CONFIG_BOARD_AUDIO_CHANNLE_NUM);
#endif
        }
        else if ((BK_A2DP_AUDIO_STATE_SUSPEND == a2dp->audio_state.state) && (BK_A2DP_AUDIO_STATE_STARTED == s_audio_state))
        {
            s_audio_state = a2dp->audio_state.state;
            bt_audio_a2dp_sink_suspend_ind();
            app_bt_on_a2dp_stop();
        }
    }
    break;

    case BK_A2DP_AUDIO_CFG_EVT:
    {
        bt_audio_a2dp_sink_codec = a2dp->audio_cfg.mcc;
        LOGI("%s, codec_id %d \r\n", __func__, bt_audio_a2dp_sink_codec.type);
    }
    break;

    default:
        LOGW("Invalid A2DP event: %d\r\n", event);
        break;
    }
}

static void gap_event_cb(bk_gap_bt_cb_event_t event, bk_bt_gap_cb_param_t *param)
{
    switch (event)
    {
        case BK_BT_GAP_ACL_DISCONN_CMPL_STAT_EVT:
        break;

        case BK_BT_GAP_ACL_CONN_CMPL_STAT_EVT:
        break;

        case BK_BT_GAP_AUTH_CMPL_EVT:
        break;

        case BK_BT_GAP_LINK_KEY_NOTIF_EVT:
        {
            s_a2dp_vol = DEFAULT_A2DP_VOLUME;
            bluetooth_storage_save_volume(param->link_key_notif.bda, s_a2dp_vol);
        }
        break;

        case BK_BT_GAP_LINK_KEY_REQ_EVT:
        {
        }
        break;

        default:
            break;
    }

}

uint8_t avrcp_remote_bda[6] = {0};

static void bt_av_notify_evt_handler(uint8_t event_id, bk_avrcp_rn_param_t *event_parameter)
{
    switch (event_id)
    {
        /* when track status changed, this event comes */
        case BK_AVRCP_RN_PLAY_STATUS_CHANGE:
        {
            LOGI("Playback status changed: 0x%x\r\n", event_parameter->playback);
            app_bt_on_play_status(event_parameter->playback);
            bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_PLAY_STATUS_CHANGE, 0);
        }
        break;

        case BK_AVRCP_RN_TRACK_CHANGE:
        {
            uint64_t track = 0;
            os_memcpy(&track, event_parameter->elm_id, sizeof(event_parameter->elm_id));
            LOGI("track changed: %lld\n", track);
            app_bt_on_track_change();
            /* 车载歌词：手机把当前歌词行塞在 TITLE 里，TRACK_CHANGE 不一定
             * 每行都发，所以 app_bt 还会周期性主动查 TITLE。这里先把全量
             * 元数据取一次。 */
            bk_bt_avrcp_ct_send_get_elem_attribute_cmd(avrcp_remote_bda,
                (1 << BK_AVRCP_MEDIA_ATTR_ID_TITLE) |
                (1 << BK_AVRCP_MEDIA_ATTR_ID_ARTIST) |
                (1 << BK_AVRCP_MEDIA_ATTR_ID_ALBUM) |
                (1 << BK_AVRCP_MEDIA_ATTR_ID_PLAYING_TIME));
            bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_TRACK_CHANGE, 0);
        }
        break;

        case BK_AVRCP_RN_PLAY_POS_CHANGED:
        {
            app_bt_on_play_pos(event_parameter->play_pos);
            bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_PLAY_POS_CHANGED, 1);
        }
        break;

        case BK_AVRCP_RN_AVAILABLE_PLAYERS_CHANGE:
        {
            LOGI("avaliable player changed\n");
            bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_AVAILABLE_PLAYERS_CHANGE, 0);
        }
        break;

        /* others */
        default:
        LOGW("unhandled event: %d\r\n", event_id);
            break;
    }
}

static void bk_bt_app_avrcp_ct_cb(bk_avrcp_ct_cb_event_t event, bk_avrcp_ct_cb_param_t *param)
{
    LOGI("%s event: %d\r\n", __func__, event);

    bk_avrcp_ct_cb_param_t *avrcp = (bk_avrcp_ct_cb_param_t *)(param);

    switch (event)
    {
        case BK_AVRCP_CT_CONNECTION_STATE_EVT:
        {
            uint8_t *bda = avrcp->conn_state.remote_bda;
        LOGI("AVRCP CT connection state: %d, [%02x:%02x:%02x:%02x:%02x:%02x]\r\n",
                      avrcp->conn_state.connected, bda[5], bda[4], bda[3], bda[2], bda[1], bda[0]);

        s_bt_env.avrcp_state = avrcp->conn_state.connected;
            if (avrcp->conn_state.connected)
            {
                os_memcpy(avrcp_remote_bda, bda, 6);
                app_bt_on_avrcp_connected(true);
                bk_bt_avrcp_ct_send_get_rn_capabilities_cmd(bda);
            }
            else
            {
                os_memset(avrcp_remote_bda, 0x0, sizeof(avrcp_remote_bda));
                app_bt_on_avrcp_connected(false);
            }
        }
        break;

        case BK_AVRCP_CT_PASSTHROUGH_RSP_EVT:
        {
            struct avrcp_ct_psth_rsp_param *pm = (typeof(pm))param;

            LOGI("AVRCP psth rsp 0x%x op 0x%x release %d tl %d %02x:%02x:%02x:%02x:%02x:%02x\n",
                       pm->rsp_code,
                       pm->key_code,
                       pm->key_state,
                       pm->tl,
                       pm->remote_bda[5],
                       pm->remote_bda[4],
                       pm->remote_bda[3],
                       pm->remote_bda[2],
                       pm->remote_bda[1],
                       pm->remote_bda[0]);
            if(s_bt_avrcp_event_cb_sema
                && pm->key_state == BK_AVRCP_PT_CMD_STATE_PRESSED //press state set sem
                && (pm->key_code == BK_AVRCP_PT_CMD_PLAY
                ||  pm->key_code == BK_AVRCP_PT_CMD_PAUSE
                ||  pm->key_code == BK_AVRCP_PT_CMD_FORWARD
                ||  pm->key_code == BK_AVRCP_PT_CMD_BACKWARD
                ||  pm->key_code == BK_AVRCP_PT_CMD_VOL_DOWN
                ||  pm->key_code == BK_AVRCP_PT_CMD_VOL_UP) )
            {
                rtos_set_semaphore(&s_bt_avrcp_event_cb_sema);
            }
        }
        break;

        case BK_AVRCP_CT_GET_RN_CAPABILITIES_RSP_EVT:
        {
            LOGI("AVRCP peer supported notification events 0x%x %02x:%02x:%02x:%02x:%02x:%02x\n", avrcp->get_rn_caps_rsp.evt_set.bits,
                            avrcp->get_rn_caps_rsp.remote_bda[5],
                            avrcp->get_rn_caps_rsp.remote_bda[4],
                            avrcp->get_rn_caps_rsp.remote_bda[3],
                            avrcp->get_rn_caps_rsp.remote_bda[2],
                            avrcp->get_rn_caps_rsp.remote_bda[1],
                            avrcp->get_rn_caps_rsp.remote_bda[0]);

            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_PLAY_STATUS_CHANGE))
            {
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_PLAY_STATUS_CHANGE, 0);
            }

            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_TRACK_CHANGE))
            {
                //bk_bt_app_avrcp_ct_get_attr(BK_AVRCP_MEDIA_ATTR_ID_TITLE);
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_TRACK_CHANGE, 0);
            }

            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_PLAY_POS_CHANGED))
            {
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_PLAY_POS_CHANGED, 1);
            }
#if 0
            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_APP_SETTING_CHANGE))
            {
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_APP_SETTING_CHANGE, 0);
            }

            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_AVAILABLE_PLAYERS_CHANGE))
            {
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_AVAILABLE_PLAYERS_CHANGE, 0);
            }

            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_ADDRESSED_PLAYER_CHANGE))
            {
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_ADDRESSED_PLAYER_CHANGE, 0);
            }

            if (avrcp->get_rn_caps_rsp.evt_set.bits & (0x01 << BK_AVRCP_RN_UIDS_CHANGE))
            {
                bk_bt_avrcp_ct_send_register_notification_cmd(avrcp_remote_bda, BK_AVRCP_RN_UIDS_CHANGE, 0);
            }
#endif
        }
        break;

        case BK_AVRCP_CT_CHANGE_NOTIFY_EVT:
        {
            LOGI("AVRCP event notification: %d %02x:%02x:%02x:%02x:%02x:%02x\n", avrcp->change_ntf.event_id,
                            avrcp->change_ntf.remote_bda[5],
                            avrcp->change_ntf.remote_bda[4],
                            avrcp->change_ntf.remote_bda[3],
                            avrcp->change_ntf.remote_bda[2],
                            avrcp->change_ntf.remote_bda[1],
                            avrcp->change_ntf.remote_bda[0]);

            bt_av_notify_evt_handler(avrcp->change_ntf.event_id, &avrcp->change_ntf.event_parameter);
        }
        break;

        case BK_AVRCP_CT_GET_ELEM_ATTR_RSP_EVT:
        {
            struct avrcp_ct_elem_attr_rsp_param *pm = (typeof(pm))param;
            LOGI("%s get elem rsp status %d count %d %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, pm->status, pm->attr_count,
                            pm->remote_bda[5],
                            pm->remote_bda[4],
                            pm->remote_bda[3],
                            pm->remote_bda[2],
                            pm->remote_bda[1],
                            pm->remote_bda[0]);

            for (uint32_t i = 0; i < pm->attr_count; ++i)
            {
                LOGI("%s get elem attr 0x%x charset 0x%x len %d\n", __func__, pm->attr_array[i].attr_id, pm->attr_array[i].attr_text_charset, pm->attr_array[i].attr_length);
                app_bt_on_avrcp_attr(pm->attr_array[i].attr_id,
                                     (const char *)pm->attr_array[i].attr_text,
                                     pm->attr_array[i].attr_length,
                                     pm->attr_array[i].attr_text_charset);
            }
        }
        break;

        default:
            LOGW("Invalid AVRCP event: %d\r\n", event);
        break;
    }
}
/* 把当前 TG 音量以 VOLUME_CHANGE(CHANGED) 通知给对端。
 * 只有对端注册过这个通知时才发（s_tg_current_registered_noti）。 */
static void bt_avrcp_tg_notify_volume_changed(void)
{
    bk_avrcp_rn_param_t cmd;
    int ret;

    if (!(s_tg_current_registered_noti & (1 << BK_AVRCP_RN_VOLUME_CHANGE)))
        return;

    os_memset(&cmd, 0, sizeof(cmd));
    cmd.volume = s_a2dp_vol;

    ret = bk_bt_avrcp_tg_send_rn_rsp(avrcp_remote_bda, BK_AVRCP_RN_VOLUME_CHANGE,
                                     BK_AVRCP_RN_RSP_CHANGED, &cmd);
    if (ret)
        LOGE("%s send vol changed err %d\n", __func__, ret);
    else
        LOGI("vol changed notified: %d\n", s_a2dp_vol);
}

/* 由 app_bt_poll 周期调用（~200ms）：手机拖动音量停止 80ms 后回一个 CHANGED。
 * 拖动过程中每个中间值都回发会把链路和手机端滑块拖慢（音量响应滞后），
 * 完全不回又会让 Android 判定设置未生效而弹回滑块。 */
void bt_avrcp_volume_poll(void)
{
    if (!s_a2dp_vol_pending)
        return;
    if ((rtos_get_time() - s_a2dp_vol_pending_ms) < 80)
        return;

    s_a2dp_vol_pending = 0;
    if (s_a2dp_vol != s_a2dp_vol_notified) {
        s_a2dp_vol_notified = s_a2dp_vol;
        bt_avrcp_tg_notify_volume_changed();
    }
}

/* app_bt 在本机（按键/设置页/USB 主机）改了音量后调用，把新音量同步给对端：
 * 手机是 CT，Windows 也一样 —— 收到 CHANGED 后滑块才会跟着动。 */
void bt_avrcp_report_volume_percent(uint8_t percent)
{
    uint8_t vol;

    if (percent > 100)
        percent = 100;

    vol = (uint8_t)(((uint32_t)percent * 0x7f + 50) / 100);
    if (vol == s_a2dp_vol)
        return;

    s_a2dp_vol = vol;
    bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
    bt_avrcp_tg_notify_volume_changed();
}

static void avrcp_tg_cb(bk_avrcp_tg_cb_event_t event, bk_avrcp_tg_cb_param_t *param)
{
    int ret = 0;

    //LOGI("%s event: %d\n", __func__, event);

    switch (event)
    {
    case BK_AVRCP_TG_CONNECTION_STATE_EVT:
    {
        uint8_t status = param->conn_stat.connected;
        uint8_t *bda = param->conn_stat.remote_bda;
        LOGI("%s avrcp tg connection state: %d, [%02x:%02x:%02x:%02x:%02x:%02x]\n", __func__,
                  status, bda[5], bda[4], bda[3], bda[2], bda[1], bda[0]);

        if (!status)
        {
            s_tg_current_registered_noti = 0;
        }
    }
    break;

    case BK_AVRCP_TG_SET_ABSOLUTE_VOLUME_CMD_EVT:
    {
        /* 拖动时这条会高频触发：不要用 LOGI（115200 下一行 ~8ms，几十步
         * 就是几百 ms 的阻塞，音量和链路都被拖慢）。 */
        LOGD("%s recv abs vol %d\n", __func__, param->set_abs_vol.volume);

        s_a2dp_vol = param->set_abs_vol.volume;

        bk_bt_dac_set_gain(s_a2dp_vol >> 1);
        if (s_a2dp_vol)
        {
            //sys_hal_aud_dacmute_en(0); //TODO
        }
        else
        {
            //sys_hal_aud_dacmute_en(1); //TODO
        }
        bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);

        /* 【必须回发，但相同的值只发一次】
         *
         * 上一版把这次回发整个删掉了，结果**手机端直接失去音量控制** ——
         * AVRCP 的 CT（手机）发完 SetAbsoluteVolume 后要等 TG 的
         * VolumeChanged(CHANGED) 作为确认，收不到就认为设置没生效，
         * 把滑块弹回并停止后续下发。所以这条回发是**功能必需**，
         * 不能按"规范上 CT 自己知道值"的理解删掉 —— 各家主机实现差异很大，
         * Android 明确依赖这个确认。
         *
         * 但也不能无条件每次都回：手机快速连续拖动时，每次 SET 都回一个
         * CHANGED，链路延迟会让滞后值追上滑块（"音量被拉回"）。
         * 折中用"值变化才回发"：拖动过程中每个中间值仍然会回，
         * 但主机重复发同一个值（松手后的补偿包）不会产生额外流量。 */
        /* 立即生效（上面已改 DAC 增益），回发去抖：
         * 只记下"待回发"，由 bt_avrcp_volume_poll() 在值稳定 80ms 后回一次。
         * 拖动过程中每步都回 CHANGED 会让手机把滞后值当最终值，表现为
         * 音量延迟/被拉回；完全不发又会让 Android 弹回滑块并停止下发。 */
        s_a2dp_vol_pending = 1;
        s_a2dp_vol_pending_ms = rtos_get_time();
    }
    break;

    case BK_AVRCP_TG_REGISTER_NOTIFICATION_EVT:
    {
        LOGI("%s recv reg evt 0x%x param %d %02x:%02x:%02x:%02x:%02x:%02x\n", __func__, param->reg_ntf.event_id, param->reg_ntf.event_parameter,
                        param->reg_ntf.remote_bda[5],
                        param->reg_ntf.remote_bda[4],
                        param->reg_ntf.remote_bda[3],
                        param->reg_ntf.remote_bda[2],
                        param->reg_ntf.remote_bda[1],
                        param->reg_ntf.remote_bda[0]);

        s_tg_current_registered_noti |= (1 << param->reg_ntf.event_id);

        bk_avrcp_rn_param_t cmd;

        os_memset(&cmd, 0, sizeof(cmd));

        switch (param->reg_ntf.event_id)
        {
        case BK_AVRCP_RN_VOLUME_CHANGE:
            cmd.volume = s_a2dp_vol;
            break;

        default:
            LOGW("%s unknow reg event 0x%x\n", __func__, param->reg_ntf.event_id);
            goto error;
        }

        ret = bk_bt_avrcp_tg_send_rn_rsp(avrcp_remote_bda, param->reg_ntf.event_id, BK_AVRCP_RN_RSP_INTERIM, &cmd);

        if (ret)
        {
            LOGE("%s send rn rsp err %d\n", __func__, ret);
        }
    }
    break;

    default:
        LOGW("%s unknow event 0x%x\n", __func__, event);
        break;
    }

error:;
}

void bk_bt_app_avrcp_ct_play(void)
{
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_PLAY, BK_AVRCP_PT_CMD_STATE_PRESSED);
    if(s_bt_avrcp_event_cb_sema)
    {
        rtos_get_semaphore(&s_bt_avrcp_event_cb_sema, AVRCP_PASSTHROUTH_CMD_TIMEOUT);
    }
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_PLAY, BK_AVRCP_PT_CMD_STATE_RELEASED);
}

void bk_bt_app_avrcp_ct_pause(void)
{
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_PAUSE, BK_AVRCP_PT_CMD_STATE_PRESSED);
    if(s_bt_avrcp_event_cb_sema)
    {
        rtos_get_semaphore(&s_bt_avrcp_event_cb_sema, AVRCP_PASSTHROUTH_CMD_TIMEOUT);
    }
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_PAUSE, BK_AVRCP_PT_CMD_STATE_RELEASED);
}

void bk_bt_app_avrcp_ct_next(void)
{
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_FORWARD, BK_AVRCP_PT_CMD_STATE_PRESSED);
    if(s_bt_avrcp_event_cb_sema)
    {
        rtos_get_semaphore(&s_bt_avrcp_event_cb_sema, AVRCP_PASSTHROUTH_CMD_TIMEOUT);
    }
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_FORWARD, BK_AVRCP_PT_CMD_STATE_RELEASED);
}

void bk_bt_app_avrcp_ct_prev(void)
{
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_BACKWARD, BK_AVRCP_PT_CMD_STATE_PRESSED);
    if(s_bt_avrcp_event_cb_sema)
    {
        rtos_get_semaphore(&s_bt_avrcp_event_cb_sema, AVRCP_PASSTHROUTH_CMD_TIMEOUT);
    }
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_BACKWARD, BK_AVRCP_PT_CMD_STATE_RELEASED);
}

void bk_bt_app_avrcp_ct_rewind(uint32_t ms)
{
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_REWIND, BK_AVRCP_PT_CMD_STATE_PRESSED);

    if(ms)
    {
        rtos_delay_milliseconds(ms);
    }

    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_REWIND, BK_AVRCP_PT_CMD_STATE_RELEASED);
}

void bk_bt_app_avrcp_ct_fast_forward(uint32_t ms)
{
    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_FAST_FORWARD, BK_AVRCP_PT_CMD_STATE_PRESSED);

    if(ms)
    {
        rtos_delay_milliseconds(ms);
    }

    bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_FAST_FORWARD, BK_AVRCP_PT_CMD_STATE_RELEASED);
}

void bk_bt_app_avrcp_ct_vol_up(void)
{
    uint8_t idx = (s_a2dp_vol + 5) >> 3;
    uint8_t old = s_a2dp_vol;
    if (idx < 16)
    {
        idx += 1;
        s_a2dp_vol = (idx <= 0) ? 0 : (idx >= 16) ? 0x7f :((idx-1) * 8 + 9);
#ifdef CONFIG_A2DP_AUDIO
        bk_bt_dac_set_gain(s_a2dp_vol >> 1);
        if (s_a2dp_vol)
        {
            //sys_hal_aud_dacmute_en(0);//TODO
        }
        else
        {
            //sys_hal_aud_dacmute_en(1);//TODO
        }

        if (s_tg_current_registered_noti & (1 << BK_AVRCP_RN_VOLUME_CHANGE))
        {
            bk_avrcp_rn_param_t cmd;
            int ret = 0;

            os_memset(&cmd, 0, sizeof(cmd));

            cmd.volume = s_a2dp_vol;

            ret = bk_bt_avrcp_tg_send_rn_rsp(avrcp_remote_bda, BK_AVRCP_RN_VOLUME_CHANGE, BK_AVRCP_RN_RSP_CHANGED, &cmd);

            if (ret)
            {
                LOGE("%s send rn rsp err %d\n", __func__, ret);
            }

            bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
        }
        else if(0)
        {
            LOGE("%s peer not reg vol change, adjust local only !!!\n", __func__);
            bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
        }
        else
        {
            bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_VOL_UP, BK_AVRCP_PT_CMD_STATE_PRESSED);
            if(s_bt_avrcp_event_cb_sema)
            {
                rtos_get_semaphore(&s_bt_avrcp_event_cb_sema, AVRCP_PASSTHROUTH_CMD_TIMEOUT);
            }
            bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_VOL_UP, BK_AVRCP_PT_CMD_STATE_RELEASED);

            bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
        }
#endif
        LOGI("vol_up, vol: %d -> %d\r\n", old, s_a2dp_vol);
    }
}

void bk_bt_app_avrcp_ct_vol_down(void)
{
    uint8_t idx = (s_a2dp_vol + 5) >> 3;
    uint8_t old = s_a2dp_vol;

    if (idx > 0)
    {
        idx -= 1;
        s_a2dp_vol = (idx <= 0) ? 0 : (idx >= 16) ? 0x7f :((idx-1) * 8 + 9);
#ifdef CONFIG_A2DP_AUDIO
        bk_bt_dac_set_gain(s_a2dp_vol >> 1);
        if (s_a2dp_vol)
        {
            //sys_hal_aud_dacmute_en(0);//TODO
        }
        else
        {
            //sys_hal_aud_dacmute_en(1);//TODO
        }

        if (s_tg_current_registered_noti & (1 << BK_AVRCP_RN_VOLUME_CHANGE))
        {
            bk_avrcp_rn_param_t cmd;
            int ret = 0;

            os_memset(&cmd, 0, sizeof(cmd));

            cmd.volume = s_a2dp_vol;

            ret = bk_bt_avrcp_tg_send_rn_rsp(avrcp_remote_bda, BK_AVRCP_RN_VOLUME_CHANGE, BK_AVRCP_RN_RSP_CHANGED, &cmd);

            if (ret)
            {
                LOGE("%s send rn rsp err %d\n", __func__, ret);
            }

            bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
        }
        else if(0)
        {
            LOGE("%s peer not reg vol change, adjust local only !!!\n", __func__);
            bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
        }
        else
        {
            bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_VOL_DOWN, BK_AVRCP_PT_CMD_STATE_PRESSED);
            if(s_bt_avrcp_event_cb_sema)
            {
                rtos_get_semaphore(&s_bt_avrcp_event_cb_sema, AVRCP_PASSTHROUTH_CMD_TIMEOUT);
            }
            bk_bt_avrcp_ct_send_passthrough_cmd(avrcp_remote_bda, BK_AVRCP_PT_CMD_VOL_DOWN, BK_AVRCP_PT_CMD_STATE_RELEASED);

            bluetooth_storage_save_volume(bt_manager_get_connected_device(), s_a2dp_vol);
        }
#endif
        LOGI("vol_down, vol: %d -> %d\r\n", old, s_a2dp_vol);
    }
}

void bt_wifi_state_updated(void)
{
    bt_audio_sink_demo_msg_t demo_msg;
    int rc = -1;

    os_memset(&demo_msg, 0x0, sizeof(bt_audio_sink_demo_msg_t));

    if (bt_audio_sink_demo_msg_que == NULL)
    {
        return;
    }

    demo_msg.type = BT_AUDIO_WIFI_STATE_UPDATE_MSG;
    demo_msg.len = 0;

    rc = rtos_push_to_queue(&bt_audio_sink_demo_msg_que, &demo_msg, BEKEN_NO_WAIT);

    if (kNoErr != rc)
    {
        LOGE("%s, send queue failed\r\n", __func__);
    }
}

#if CONFIG_WIFI_COEX_SCHEME
void wifi_state_callback(uint8_t status_id, uint8_t status_info)
{
    if (COEX_WIFI_STAT_ID_SCANNING == status_id || COEX_WIFI_STAT_ID_CONNECTING == status_id)
    {
        if (status_info)
        {
            s_bt_env.wifi_state |= (1 << status_id);
        }
        else
        {
            s_bt_env.wifi_state &= ~(1 << status_id);
        }
        bt_wifi_state_updated();
    }
}

static void coex_regisiter_wifi_state(void)
{
    os_memset(&s_coex_to_bt_func, 0, sizeof(s_coex_to_bt_func));
    s_coex_to_bt_func.version = 0x01;
    s_coex_to_bt_func.inform_wifi_status = wifi_state_callback;

    coex_bt_if_init(&s_coex_to_bt_func);
}
#endif

int32_t bk_bt_app_avrcp_ct_get_attr(uint32_t attr)
{
    uint32_t media_attr_id_mask = (1 << BK_AVRCP_MEDIA_ATTR_ID_TITLE);

    if(attr)
    {
        media_attr_id_mask = (1 << attr);
    }

    return bk_bt_avrcp_ct_send_get_elem_attribute_cmd(avrcp_remote_bda, media_attr_id_mask);
}

/* 车载歌词轮询：部分手机把歌词写进 TITLE 后不会再发 TRACK_CHANGE，
 * 由 app_bt 每秒主动查一次 TITLE/ARTIST/ALBUM。 */
int32_t bt_avrcp_request_cover_art(void)
{
    if (0 == (avrcp_remote_bda[0] | avrcp_remote_bda[1] | avrcp_remote_bda[2] |
              avrcp_remote_bda[3] | avrcp_remote_bda[4] | avrcp_remote_bda[5]))
        return -1;

    int32_t ret = bk_bt_avrcp_ct_send_get_elem_attribute_cmd(avrcp_remote_bda,
        (1u << BK_AVRCP_MEDIA_ATTR_ID_DEFAULT_COVER_ART));
    LOGI("request cover art attr ret=%d\r\n", ret);
    return ret;
}

int32_t bt_avrcp_poll_attrs(void)
{
    if (0 == (avrcp_remote_bda[0] | avrcp_remote_bda[1] | avrcp_remote_bda[2] |
              avrcp_remote_bda[3] | avrcp_remote_bda[4] | avrcp_remote_bda[5]))
        return -1;

    return bk_bt_avrcp_ct_send_get_elem_attribute_cmd(avrcp_remote_bda,
        (1 << BK_AVRCP_MEDIA_ATTR_ID_TITLE) |
        (1 << BK_AVRCP_MEDIA_ATTR_ID_ARTIST) |
        (1 << BK_AVRCP_MEDIA_ATTR_ID_ALBUM) |
        (1 << BK_AVRCP_MEDIA_ATTR_ID_PLAYING_TIME));
}

static void bk_bt_a2dp_connect(uint8_t *remote_addr)
{
    if (s_bt_env.wifi_state)
    {
        bt_manager_set_connect_state(BT_STATE_WAIT_FOR_RECONNECT);
        return;
    }
    bk_bt_a2dp_sink_connect(remote_addr);
}

static void bk_bt_a2dp_stop_connect()
{
    if (rtos_is_oneshot_timer_init(&s_bt_env.avrcp_connect_tmr))
    {
        if (rtos_is_oneshot_timer_running(&s_bt_env.avrcp_connect_tmr))
        {
            rtos_stop_oneshot_timer(&s_bt_env.avrcp_connect_tmr);
        }
        rtos_deinit_oneshot_timer(&s_bt_env.avrcp_connect_tmr);
    }
}

static void bk_bt_a2dp_disconnect(uint8_t *remote_addr)
{
    bk_bt_a2dp_sink_disconnect(bt_manager_get_connected_device());
}

int a2dp_sink_demo_init(uint8_t aac_supported)
{
    int ret = 0;
    LOGI("%s\r\n", __func__);

    if (aac_supported)
    {
#if (!CONFIG_AAC_DECODER)
        LOGI("%s AAC is not supported!\n", __func__);
        return -1;
#endif
    }

    bk_avrcp_rn_evt_cap_mask_t tmp_cap = {0};
    bk_avrcp_rn_evt_cap_mask_t final_cap =
    {
        .bits = 0
        | (1 << BK_AVRCP_RN_VOLUME_CHANGE)
        ,
    };

    if (!s_bt_api_event_cb_sema)
    {
        ret = rtos_init_semaphore(&s_bt_api_event_cb_sema, 1);

        if (ret)
        {
            LOGE("%s sem init err %d\n", __func__, ret);
            return -1;
        }
    }
    if(!s_bt_avrcp_event_cb_sema)
    {
        ret = rtos_init_semaphore(&s_bt_avrcp_event_cb_sema, 1);

        if (ret)
        {
            LOGE("%s avrcp evt sem init err %d\n", __func__, ret);
            return -1;
        }
    }

#if CONFIG_WIFI_COEX_SCHEME
    coex_regisiter_wifi_state();
#endif

    btm_callback_s btm_cb =
    {
        .gap_cb = gap_event_cb,
        .start_connect_cb = bk_bt_a2dp_connect,
        .start_disconnect_cb = bk_bt_a2dp_disconnect,
        .stop_connect_cb = bk_bt_a2dp_stop_connect,
    };
    bt_manager_register_callback(&btm_cb);

    bt_audio_sink_demo_task_init();

    bk_bt_avrcp_ct_init();
    bk_bt_avrcp_ct_register_callback(bk_bt_app_avrcp_ct_cb);

    bk_bt_avrcp_tg_init();

    /* AVRCP 1.4 的绝对音量要求 SDP 的 SupportedFeatures 里带上 CAT_2
     * （Monitor/Amplifier）。缺这一位时 Windows 会认为本机没有音量通道，
     * 滑块只显示一个固定值，拖动也不会下发 SetAbsoluteVolume。
     * 必须在任何 AVRCP 连接之前设置。 */
    {
        uint16_t feat = 0;

        if (bk_bt_avrcp_tg_sdp_feature_operation(
                BK_AVRCP_SDP_FEATURE_API_METHOD_GET_CURRENT_ENABLE, &feat) == BK_ERR_BT_SUCCESS)
        {
            LOGI("%s tg sdp feat 0x%x\n", __func__, feat);
            if (!(feat & BK_AVRCP_SDP_FEATURE_TG_CAT_2))
            {
                uint16_t want = (uint16_t)(feat | BK_AVRCP_SDP_FEATURE_TG_CAT_2);

                if (bk_bt_avrcp_tg_sdp_feature_operation(
                        BK_AVRCP_SDP_FEATURE_API_METHOD_SET, &want) == BK_ERR_BT_SUCCESS)
                    LOGI("%s tg sdp feat -> 0x%x (CAT_2 on)\n", __func__, want);
                else
                    LOGE("%s set tg sdp feat failed\n", __func__);
            }
        }
    }

    bk_bt_avrcp_tg_get_rn_evt_cap(BK_AVRCP_RN_CAP_API_METHOD_ALLOWED, &tmp_cap);

    final_cap.bits &= tmp_cap.bits;

    LOGI("%s set rn cap 0x%x\n", __func__, final_cap.bits);

    ret = bk_bt_avrcp_tg_set_rn_evt_cap(&final_cap);

    if (ret)
    {
        LOGE("%s set rn cap err %d\n", __func__, ret);
        return -1;
    }

    bk_bt_avrcp_tg_register_callback(avrcp_tg_cb);

    ret = bk_bt_a2dp_register_callback(bk_bt_app_a2dp_sink_cb);
    if (ret)
    {
        LOGE("%s bk_bt_a2dp_register_callback err %d\n", __func__, ret);
        return -1;
    }

    ret = bk_bt_a2dp_sink_init(aac_supported);

    if (ret)
    {
        LOGI("%s a2dp sink init err %d\n", __func__, ret);
        return -1;
    }

    ret = rtos_get_semaphore(&s_bt_api_event_cb_sema, 6 * 1000);

    if (ret)
    {
        LOGI("%s get sem for a2dp sink init err\n", __func__);
        return -1;
    }

    ret = bk_bt_a2dp_sink_register_data_callback(&bt_audio_sink_media_data_ind);

    if (ret)
    {
        LOGE("%s bk_bt_a2dp_sink_register_data_callback err %d\n",__func__,  ret);
        return -1;
    }

    return 0;
}


static int speaker_task_init()
{
    bk_err_t ret = BK_OK;
    if (!a2dp_speaker_thread_handle)
    {
        ret = rtos_create_thread(&a2dp_speaker_thread_handle,
                                 A2DP_SPEAKER_THREAD_PRI,
                                 "bt_a2dp_speaker",
                                 (beken_thread_function_t)speaker_task,
                                 4096,
                                 (beken_thread_arg_t)0);
        if (ret != kNoErr)
        {
            LOGE("speaker task fail \r\n");
        }

        return kNoErr;
    }
    else
    {
        LOGE("%s task is exist !!!\n", __func__);
        return kInProgressErr;
    }
}

static void speaker_task(void *arg)
{
    bk_err_t ret = 0;
    uint32_t samp_rate;
    uint8_t channels = CONFIG_BOARD_AUDIO_CHANNLE_NUM;
    bool audio_open = false;

    (void)arg;

    if (CODEC_AUDIO_SBC == bt_audio_a2dp_sink_codec.type)
        samp_rate = bt_audio_a2dp_sink_codec.cie.sbc_codec.sample_rate;
#if (CONFIG_AAC_DECODER)
    else
        samp_rate = bt_audio_a2dp_sink_codec.cie.aac_codec.sample_rate;
#else
    else
        samp_rate = 44100;
#endif

    if (a2dp_speaker_sema == NULL)
    {
        if (kNoErr != rtos_init_semaphore(&a2dp_speaker_sema, 1))
        {
            LOGE("init sema fail, %d \n", __LINE__);
            goto end;
        }
    }

    LOGI("%s start, %uHz %uch active=%d\r\n", __func__,
         (unsigned)samp_rate, channels, s_bt_output_active);

    while (s_spk_is_started)
    {
        /* 【不能无限等信号量】手机暂停时不会有包，而一旦音频通路出问题，
         * 无限等会让下面的停滞看门狗永远没机会跑。1 秒醒一次即可：
         * 正常情况下包间隔只有几毫秒，超时分支几乎不会被触发。
         * （CONFIG_FREERTOS_TICK_RATE_HZ=1000，所以 1000 tick = 1 秒） */
        rtos_get_semaphore(&a2dp_speaker_sema, 1000);

        if (!s_spk_is_started)
            break;

        /* 非 BT 模式：丢弃已缓存的帧并关闭 DAC，让 SD/USB 使用 */
        if (!s_bt_output_active)
        {
            while (ring_buffer_node_get_fill_nodes(&s_a2dp_frame_nodes))
                ring_buffer_node_take_read_node(&s_a2dp_frame_nodes);

            if (audio_open)
            {
                app_audio_close();
                audio_open = false;
                s_bt_audio_opened = false;
            }
            continue;
        }

        /* 【停滞看门狗】主机还在送包，但已超过 3 秒没写成功 -> 通路死了。
         * 连续两轮都满足才动手，避免"暂停很久后刚恢复"被误判。 */
        if (audio_open) {
            static uint32_t s_pkt_seen;
            static uint8_t s_stall_ticks;
            uint32_t now = rtos_get_time();

            if (s_a2dp_pkt_cnt == s_pkt_seen) {
                s_stall_ticks = 0;                 /* 没数据：手机暂停，正常 */
            } else {
                s_pkt_seen = s_a2dp_pkt_cnt;       /* 有数据在流 */
                if ((now - s_last_write_ok_ms) > BT_AUDIO_STALL_MS) {
                    if (++s_stall_ticks >= 2) {
                        s_stall_ticks = 0;
                        LOGE("%s stalled %ums (pkt=%u, fill=%u)\r\n", __func__,
                             (unsigned)(now - s_last_write_ok_ms),
                             (unsigned)s_a2dp_pkt_cnt,
                             (unsigned)ring_buffer_node_get_fill_nodes(&s_a2dp_frame_nodes));
                        bt_audio_pipeline_reset(&audio_open);
                        continue;
                    }
                } else {
                    s_stall_ticks = 0;
                }
            }
        }

        if (!audio_open)
        {
            if (app_audio_open(samp_rate, channels) != BK_OK)
            {
                LOGE("%s open audio failed, retry\r\n", __func__);
                rtos_delay_milliseconds(50);
                continue;
            }
            audio_open = true;
            s_bt_audio_opened = true;
            s_last_write_ok_ms = rtos_get_time();
            bk_bt_dac_set_gain(s_a2dp_vol >> 1);
        }

        uint32_t frame_nodes = ring_buffer_node_get_fill_nodes(&s_a2dp_frame_nodes);
        while (frame_nodes && s_spk_is_started && s_bt_output_active)
        {
            int i = 0;
            uint8_t s_frame = (uint8_t)frame_nodes;
            for (; i < s_frame; i++)
            {
                uint8_t *inbuf = ring_buffer_node_peek_read_node(&s_a2dp_frame_nodes);
                uint16_t tmp_len = (uint16_t)(inbuf[1] << 8 | inbuf[0]);

                inbuf += 2;

                if (CODEC_AUDIO_SBC == bt_audio_a2dp_sink_codec.type)
                {
                    ret = bk_sbc_decoder_frame_decode(&bt_audio_sink_sbc_decoder, inbuf, tmp_len);
                    ring_buffer_node_take_read_node(&s_a2dp_frame_nodes);
                    if (ret < 0)
                    {
                        LOGE("sbc_decoder_decode error <%d> %d %d\n", ret, tmp_len, s_frame);
                        continue;
                    }
                    int16_t *dst = (int16_t *)bt_audio_sink_sbc_decoder.pcm_sample;
                    int16_t w_len = (int16_t)(bt_audio_sink_sbc_decoder.pcm_length *
                                              bt_audio_sink_sbc_decoder.channel_number * 2);

                    if (app_audio_write(dst, (uint32_t)w_len) != BK_OK)
                    {
                        /* 模式切换时音频会被关掉，这里直接退出本轮 */
                        if (!s_bt_output_active)
                            break;
                        LOGE("audio write failed (sbc len=%d)\n", w_len);
                        bt_audio_pipeline_reset(&audio_open);
                        break;
                    }

                    /* 写成功就刷新停滞看门狗的时基 */
                    s_last_write_ok_ms = rtos_get_time();
                }
#if (CONFIG_AAC_DECODER)
                else if (CODEC_AUDIO_AAC == bt_audio_a2dp_sink_codec.type)
                {
                    uint32_t len = 0;
                    memcpy(&len, inbuf, sizeof(len));

                    uint8_t *out_buf = 0;
                    uint32_t out_len = 0;
                    ret = bk_aac_decoder_decode(&bt_audio_sink_aac_decoder,
                                                (inbuf + sizeof(len)), len,
                                                &out_buf, &out_len);
                    ring_buffer_node_take_read_node(&s_a2dp_frame_nodes);
                    if (ret == 0)
                    {
                        int16_t *dst = (int16_t *)out_buf;
                        if (app_audio_write(dst, out_len) != BK_OK)
                        {
                            if (!s_bt_output_active)
                                break;
                            LOGE("audio write failed (aac len=%u)\n", out_len);
                            bt_audio_pipeline_reset(&audio_open);
                            break;
                        }

                        s_last_write_ok_ms = rtos_get_time();
                    }
                    else
                    {
                        LOGE("aac_decoder_decode error <%d>\n", ret);
                    }
                }
#endif
                else
                {
                    ring_buffer_node_take_read_node(&s_a2dp_frame_nodes);
                    LOGE("unsupported codec!!\n");
                }
            }
            frame_nodes -= (uint32_t)s_frame;
        }
    }

end:;
    LOGI("%s a2dp exit start!! \r\n", __func__);

    if (audio_open)
    {
        app_audio_close();
        audio_open = false;
    }
    s_bt_audio_opened = false;

    LOGI("%s a2dp end!!\r\n", __func__);
    if (a2dp_speaker_sema)
    {
        rtos_deinit_semaphore(&a2dp_speaker_sema);
        a2dp_speaker_sema = NULL;
    }
    if (CODEC_AUDIO_SBC == bt_audio_a2dp_sink_codec.type)
    {
        bk_sbc_decoder_deinit();
    }
#if (CONFIG_AAC_DECODER)
    bk_aac_decoder_deinit(&bt_audio_sink_aac_decoder);
#endif
    ring_buffer_node_clear(&s_a2dp_frame_nodes);
    ring_buffer_node_deinit(&s_a2dp_frame_nodes);
    if (p_cache_buff)
    {
        os_free(p_cache_buff);
        p_cache_buff = NULL;
    }
    if (s_spk_is_started)
    {
        s_spk_is_started = 0;
        LOGE("!! speaker tash exit error !!\n");
    }

    if (a2dp_speaker_exit_sema)
    {
        rtos_set_semaphore(&a2dp_speaker_exit_sema);
    }

    rtos_delete_thread(NULL);
}

int32_t wait_a2dp_speaker_task_end(void)
{
    while(a2dp_speaker_thread_handle)
    {
        rtos_delay_milliseconds(20);
    }

    return 0;
}
