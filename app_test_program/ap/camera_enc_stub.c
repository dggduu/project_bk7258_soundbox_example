// Copyright 2026
//
// YUV-only 工程的编码器接口空实现。
//
// 本工程没开 CONFIG_H264 / CONFIG_JPEGENC_HW，但官方 bk_camera 组件
// （bk_camera_dvp_ctlr.c）的 ops 表引用了 bk_dvp_suspend/resume，而它们在
// **运行时**才判断 img_format 分支，链接器会保留 bk_h264_encode_disable /
// bk_h264_global_soft_reset / bk_jpeg_enc_global_soft_reset 的调用，导致
// 链接失败。YUV 模式永远不会走到这几个分支，这里补空实现即可。

#include <common/bk_err.h>
#include <driver/h264.h>
#include <driver/jpeg_enc.h>

bk_err_t bk_h264_encode_disable(void)
{
	return BK_OK;
}

bk_err_t bk_h264_global_soft_reset(uint8_t enable)
{
	(void)enable;
	return BK_OK;
}

bk_err_t bk_jpeg_enc_global_soft_reset(uint8_t enable)
{
	(void)enable;
	return BK_OK;
}
