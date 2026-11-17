// Copyright 2026
//
// 【BIP 封面功能已停用】
//
// 原实现走 AVRCP Cover Art（BIP / OBEX over L2CAP）：手机下发 7 字节
// image handle，本机再用 L2CAP 把整张图拉下来（缓冲 512KB PSRAM）解码成
// 104x104。原文件备份在
//   .workbuddy/backup/2026-11-16-bip/bt_cover_art.c.full
//
// 停用原因（用户要求，且现场没有能配合的手机，无法验证）：
//   1. 这条路要开一条 L2CAP 通道拉几百 KB，**和 A2DP 抢同一条蓝牙链路**。
//      拉图期间 A2DP 的数据回调被挤掉；一旦 OBEX 会话卡在中间某一步，
//      链路恢复不了，声音就再也回不来，而 A2DP 连接看起来还是好的
//      —— 正好就是"放一段时间后卡住"的形态。
//   2. 每首歌都要拉一次，内存/时序压力随播放时间累积，很难现场复现。
//   3. 没有可测试的环境，留着只有风险。
//
// 现在这里只保留接口壳子：
//   - `bt_cover_art_on_handle()` 忽略 image handle（不再注册任何 L2CAP 回调，
//     不注册就不会抢数据）
//   - `bt_cover_art_copy_cover()` 恒 false → UI 显示内置默认封面
//   - `bt_cover_art_generation()` 恒 0 → 封面永不"更新"，UI 不会因此重绘
//
// 恢复时：把备份文件覆盖回来即可，调用方（app_bt.c / app_ui.c）无需改动。

#include "bt_cover_art.h"

#include <components/log.h>

#define TAG "bt_cover"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)

void bt_cover_art_on_handle(const char *handle, uint32_t len)
{
	static bool warned;

	(void)handle;
	(void)len;

	/* 只提示一次，避免每首歌刷一行 */
	if (!warned) {
		warned = true;
		LOGI("cover art (BIP) disabled, image handle ignored\r\n");
	}
}

bool bt_cover_art_copy_cover(uint16_t *dst, int w, int h)
{
	(void)dst;
	(void)w;
	(void)h;
	return false;
}

uint32_t bt_cover_art_generation(void)
{
	return 0;
}
