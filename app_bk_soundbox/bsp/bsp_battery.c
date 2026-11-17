// Copyright 2026
// 电池电压检测实现
//
// 硬件：VBAT -- R21 220k --+-- R24 110k -- GND
//                          |
//                        P22 / ADC_5
//
// 分压比 1:3（2026-11 由 1:2 改），ADC 读到的就是 VBAT/3，软件乘 3 还原：
//     4.2V -> 分压点 1400mV
//     3.7V -> 分压点 1233mV
//     3.0V -> 分压点 1000mV
// 源阻抗 220k||110k ≈ 73.3k（1:2 时是 110k），采样精度更好。
//
// 【换算口径：用 SDK 的 mV，不要自己按 raw 硬套系数】
//
// 历史坑（务必看完，不然会再踩一次）：
//   早先 1:2 + P49 PMOS 那版硬件上测到"分压点 2045mV，SDK 报 3288mV"，
//   于是得出一条结论：SDK 把满码当 3.3V、实际量程只有 2.0V 左右，必须
//   按 raw 自己换算，并用 `bat cal` 反算出满码 1559mV。app_test_uart1 的
//   `bat` 命令里至今还是这个 1559（BAT_ADC_FS_MV）。
//
//   但换成 1:3、去掉 PMOS 后，真机数据推翻了它：
//     电池 4.1V -> P22 应为 4.1/3 = 1367mV，实测 raw = 3420；
//     而 SDK 对 raw 3420 算出来是 1374mV（误差 0.5%），
//     反算满码 = 1367 * 8191 / 3420 ≈ 3273mV —— 和 SDK 的 3.3V 口径一致。
//   也就是说：**这块板当前硬件下 SDK 的换算是对的**（它就是芯片出厂
//   标定，按键 P28 也一直用同一套，四个键的 2.269/1.650/1.055V 门限
//   能对上）。1559 是 PMOS 漏电/分压点测错时代的产物，用它会把 4.1V
//   电池算成 1.95V、显示成"无电池"。
//
//   所以本文件用 bsp_adc_read_raw_mv() 同时拿 raw 和 SDK 的 mV：
//     pin_mv = SDK 换算的分压点电压
//     vbat   = pin_mv * BOARD_BAT_ADC_DIV_NUM
//   raw 只用于判饱和 / 打日志（排查分压电阻和量程问题）。
//   **不要再用 raw * 1559 / 8191 这种硬编码系数。**
//
// 【PMOS 已取消】原来中间串了一颗 PMOS（bat_adc_en，P49 控制）用来在不
// 需要采样时切断分压网络省电，那颗管子电路有问题，已从方案里去掉：
// 现在分压电阻**常接**，ADC 直接测 P22 上的 VBAT/3。
// 代价是 220k+110k 常态漏电 ≈ 4.2V/330k ≈ 12.7uA，对 1000mAh 电池可忽略。
//
// 因此本文件不再有：使能脚驱动、极性自检、栅极电平回读。
// 唯一依赖是 bsp_adc_read_raw_mv()，它内部会把 P22 切到 GPIO_DEV_ADC5
// （P22 的默认第二功能是 LCD_R4，必须显式切走，见 bsp_adc_util.c）。

#include "bsp_battery.h"
#include "bsp_board_cfg.h"
#include "bsp_adc_util.h"

#include <os/os.h>
#include <components/log.h>

#define TAG "bsp_battery"
#define BSP_LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define BSP_LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

#define BAT_MV_FULL     4200
#define BAT_MV_EMPTY    3000

/* raw 码值满量程（只用来判饱和/打日志，换算走 SDK 的 mV） */
#define BAT_ADC_RAW_FS      8191
/* 判饱和：raw 超过这个值说明输入已进饱和区，读数不可信 */
#define BAT_ADC_RAW_SAT     8063

/* 分压点电压下限（VBAT/3）。
 *
 * 正常锂电 3.0~4.2V -> 分压点 1000~1400mV。
 * 低于 800mV（VBAT 2.4V）物理上不可能给运行中的系统供电：锂电保护板
 * 2.5V 左右就切断输出，PMU 欠压复位阈值也在 3V 上下。所以这个门限的
 * 语义是"低于它 = 没接电池 / 分压网络开路"：
 *   - 电池座空着、分压电阻还接着：P22 被 R24 拉到 0V 附近；
 *   - R21/R24 虚焊或缺失：读数同样接近 0。
 * 不放太高（比如 900mV）的原因：放歌时功放拉电流会让 VBAT 瞬间跌落，
 * 分压点跟着掉 1/3 幅度（约 30~50mV），电池接近空电时会被误判成
 * "没接电池"，状态栏在 0% 和 "--%" 之间跳。 */
#define BAT_ADC_VALID_MIN_MV   800

/* 分压点上限：1:3 分压后 VBAT 6.6V 才会到 2200mV，锂电不可能。
 * 真到了说明分压比不对（比如 R24 没贴、分压点直连 VBAT）或输入过压。 */
#define BAT_ADC_VALID_MAX_MV   2200

/* 日志限流：同样的状态 60s 内只报一次，避免 USB 供电调试时刷屏。 */
#define BAT_LOG_PERIOD_MS   60000

static bool s_bat_inited = false;
static uint8_t s_last_pct = 0xFF;
static bk_err_t s_last_err = BK_OK;
static uint32_t s_last_log_ms = 0;

bk_err_t bsp_battery_init(void)
{
	if (s_bat_inited)
		return BK_OK;

	s_bat_inited = true;
	BSP_LOGI("battery init: P%d/ADC%d, 1:%d divider, SDK mV conversion\r\n",
	         BOARD_BAT_ADC_PIN, BOARD_BAT_ADC_CHAN, BOARD_BAT_ADC_DIV_NUM);
	/* 现场核对用：拿万用表量 VBAT，对照这里的分压点期望值。
	 * 对不上先查分压电阻/接线，不要急着改换算（换算走芯片标定）。 */
	BSP_LOGI("  expect pin: 4.2V~%umV 3.7V~%umV 3.0V~%umV\r\n",
	         (unsigned)(4200u / BOARD_BAT_ADC_DIV_NUM),
	         (unsigned)(3700u / BOARD_BAT_ADC_DIV_NUM),
	         (unsigned)(3000u / BOARD_BAT_ADC_DIV_NUM));
	return BK_OK;
}

/* 错误日志限流：同样的错误 60s 内只报一次 */
static bool bat_log_err_allowed(bk_err_t err, uint32_t now)
{
	if ((err == s_last_err) && ((now - s_last_log_ms) < BAT_LOG_PERIOD_MS))
		return false;

	s_last_err = err;
	s_last_log_ms = now;
	return true;
}

bk_err_t bsp_battery_get_mv_avg(uint32_t sample_num, uint32_t *mv)
{
	bk_err_t ret;
	uint32_t raw = 0;
	uint32_t pin_mv = 0;
	uint32_t vbat_mv;
	uint32_t now;
	uint8_t pct;

	if (!mv || sample_num == 0)
		return BK_ERR_NULL_PARAM;

	if (!s_bat_inited)
		bsp_battery_init();

	/* 一次采集同时拿到 raw 和 SDK 换算的 mV（同一批采样）
	 * pin_mv 用的是芯片出厂标定的换算，和按键 ADC 完全同一口径。 */
	ret = bsp_adc_read_raw_mv(BOARD_BAT_ADC_CHAN, sample_num, &raw, &pin_mv);
	now = rtos_get_time();

	if (ret != BK_OK) {
		if (bat_log_err_allowed(ret, now))
			BSP_LOGE("adc read failed: %d\r\n", ret);
		return ret;
	}

	vbat_mv = pin_mv * BOARD_BAT_ADC_DIV_NUM;

	if (raw >= BAT_ADC_RAW_SAT) {
		if (bat_log_err_allowed(BK_ERR_PARAM, now))
			BSP_LOGE("saturated: raw=%u/%d pin=%umV, input over range "
			         "(divider wrong / over voltage)\r\n",
			         (unsigned)raw, BAT_ADC_RAW_FS, (unsigned)pin_mv);
		return BK_ERR_PARAM;
	}

	if (pin_mv > BAT_ADC_VALID_MAX_MV) {
		if (bat_log_err_allowed(BK_ERR_PARAM, now))
			BSP_LOGE("pin %umV > %dmV: divider is smaller than 1:%d "
			         "(check R21/R24)\r\n",
			         (unsigned)pin_mv, BAT_ADC_VALID_MAX_MV,
			         BOARD_BAT_ADC_DIV_NUM);
		return BK_ERR_PARAM;
	}

	if (pin_mv < BAT_ADC_VALID_MIN_MV) {
		if (bat_log_err_allowed(BK_ERR_NOT_FOUND, now))
			BSP_LOGE("absent: raw=%u pin=%umV < %dmV "
			         "(no battery / divider open / P22 not mapped)\r\n",
			         (unsigned)raw, (unsigned)pin_mv,
			         BAT_ADC_VALID_MIN_MV);
		return BK_ERR_NOT_FOUND;
	}

	s_last_err = BK_OK;

	pct = bsp_battery_mv_to_percent(vbat_mv);

	/* 正常读数只在百分比变化时打一行，外加 60s 一条心跳：
	 * 值不变却每 5s 刷一条会把串口日志冲掉。 */
	if ((pct != s_last_pct) || ((now - s_last_log_ms) >= BAT_LOG_PERIOD_MS)) {
		s_last_pct = pct;
		s_last_log_ms = now;
		BSP_LOGI("bat raw=%u/%d pin=%umV vbat=%umV pct=%u%%\r\n",
		         (unsigned)raw, BAT_ADC_RAW_FS, (unsigned)pin_mv,
		         (unsigned)vbat_mv, (unsigned)pct);
	}

	*mv = vbat_mv;
	return BK_OK;
}

bk_err_t bsp_battery_get_mv(uint32_t *mv)
{
	return bsp_battery_get_mv_avg(1, mv);
}

uint8_t bsp_battery_mv_to_percent(uint32_t mv)
{
	if (mv >= BAT_MV_FULL)
		return 100;
	if (mv <= BAT_MV_EMPTY)
		return 0;

	return (uint8_t)(((mv - BAT_MV_EMPTY) * 100) / (BAT_MV_FULL - BAT_MV_EMPTY));
}

bk_err_t bsp_battery_get_percent(uint8_t *percent)
{
	uint32_t mv = 0;
	bk_err_t ret;

	if (!percent)
		return BK_ERR_NULL_PARAM;

	ret = bsp_battery_get_mv_avg(8, &mv);
	if (ret != BK_OK)
		return ret;

	*percent = bsp_battery_mv_to_percent(mv);
	return BK_OK;
}

bk_err_t bsp_battery_power_down(void)
{
	/* 分压电阻常接在 VBAT 上，软件无法断开，保留空实现供上层统一调用。 */
	return BK_OK;
}
