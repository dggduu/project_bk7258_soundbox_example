// Copyright 2026
// BK7258 USB / BT / SD 三模音箱主入口
//
// 硬件：ILI9341 240x320 SPI 屏（无触摸）、ADC 四键、SDIO SD 卡、
//       DAC + NS4150B 功放、CherryUSB UAC 声卡、A2DP sink。
//
// 界面：LVGL 竖屏，封面在上、歌词与底部 Tab 导航栏在下。

#include "bk_private/bk_init.h"
#include <components/system.h>
#include <os/os.h>
#include <components/log.h>

#include "bsp.h"
#include "bsp_font.h"
#include "app_mode.h"
#include "app_config.h"
#include "app_ui.h"
#include "app_key.h"
#include "app_bt.h"
#include "media_service.h"

#define TAG "main"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

/* ==================== 启动心跳 + 阶段标记 ====================
 *
 * 现场出现过"日志停在电源轨初始化之后、后面什么都没有"的现象，
 * 于是无法判断到底是：
 *   (a) main 卡在某一个初始化步骤，
 *   (b) 日志输出通路本身挂了，
 *   (c) 芯片在反复复位（复位日志走 CP 的 UART2，我们看不到）。
 *
 * 这里起一个独立小线程每 2 秒打一行 `boot beat`，配合每一步的
 * `stage N` 标记，三种情况可以一眼分开：
 *   - 能连续看到 boot beat，但 stage 不再前进 -> main 卡住，日志正常
 *   - 连 boot beat 都没有                      -> 日志通路挂了或被复位
 *   - 每隔几秒重新出现 `=== boot start`         -> 在复位循环（看 reset 原因）
 *
 * 定位完成后可以把这个线程删掉（它只占 2KB 栈、2 秒一行日志）。 */
static volatile uint32_t s_boot_stage;

static void boot_beat_task(void *arg)
{
	(void)arg;

	for (;;) {
		rtos_delay_milliseconds(2000);
		LOGI("boot beat: stage=%u\r\n", (unsigned)s_boot_stage);
	}
}

int main(void)
{
	bk_err_t ret;

	bk_init();

	LOGI("=== boot start, reset_reason=0x%x ===\r\n",
	     (unsigned)bk_misc_get_reset_reason());

	{
		beken_thread_t th = NULL;

		if (rtos_create_thread(&th, BEKEN_DEFAULT_WORKER_PRIORITY,
		                       "bootbeat", boot_beat_task, 2048, 0) != kNoErr)
			LOGE("boot beat task create failed\r\n");
	}

	/* 与 SDK headset 例程一致：先初始化 media service，再做蓝牙协议栈早期初始化。
	 * 蓝牙的 local name / 可发现性在 main 里就设置好，不依赖后面的 UI 线程。 */
	if (media_service_init() != BK_OK)
		LOGE("media_service_init failed\r\n");
	app_bt_early_init();
	LOGI("stage 1: bt early ready\r\n");
	s_boot_stage = 1;

	/* 1. 板级基础：电源轨、控制脚 */
	bsp_init();
	bsp_dump_pinmap();
	LOGI("stage 2: bsp init done\r\n");
	s_boot_stage = 2;

	/* 2. 屏幕 + LVGL 显示后端 */
	ret = bsp_screen_init();
	if (ret != BK_OK) {
		LOGE("screen init failed: %d\r\n", ret);
		return -1;
	}
	LOGI("stage 3: screen ready\r\n");
	s_boot_stage = 3;

	ret = bsp_lvgl_init(app_ui_loop);
	if (ret != BK_OK) {
		LOGE("lvgl init failed: %d\r\n", ret);
		return -1;
	}
	LOGI("stage 4: lvgl backend ready\r\n");
	s_boot_stage = 4;

	/* 3. 应用状态机 / 三个模式（会挂 SD、起 USB/BT 初始化线程） */
	app_mode_init();
	LOGI("stage 5: app mode init done\r\n");
	s_boot_stage = 5;

	/* 3.5 字体来源：SD 卡 TTF 优先，失败回退内置字库。
	 * 必须在 SD 挂载之后、UI 创建之前。
	 * 再用 easyflash 里保存的"字体开关"覆盖（用户上次选的来源优先）。 */
	bsp_font_init();
	app_config_apply_font();
	LOGI("stage 6: font ready\r\n");
	s_boot_stage = 6;

	/* 4. LVGL 界面 + 输入 */
	app_ui_init();
	LOGI("stage 7: ui created\r\n");
	s_boot_stage = 7;

	bsp_lvgl_start();
	app_key_init();
	LOGI("stage 8: lvgl started, key ready\r\n");
	s_boot_stage = 8;

	/* 5. 默认进入蓝牙模式 */
	app_mode_start();
	LOGI("stage 9: mode started\r\n");
	s_boot_stage = 9;

	LOGI("tri-mode speaker started\r\n");

	while (1)
		rtos_delay_milliseconds(1000);

	return 0;
}
