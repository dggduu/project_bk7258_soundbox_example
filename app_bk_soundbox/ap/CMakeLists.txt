set(incs
    .
    app
    app/bt
    app/assets
    app/zlib_inflate
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/include
    )

# 只编译本产品用到的 BSP，摄像头/触摸的旧文件保留在目录里但不参与编译
#
# 【2026-11 换屏】bsp_ili9341_panel.c 已停用（原 240x320 单线 SPI 屏），
# 改用 bsp_st77916_panel.c（360x360 QSPI 圆屏）。两者都保留在目录里，
# 只切编译列表，方便以后回退。
set(bsp_srcs
    ../bsp/bsp_init.c
    ../bsp/bsp_power.c
    ../bsp/bsp_adc_util.c
    ../bsp/bsp_key.c
    ../bsp/bsp_screen.c
    ../bsp/bsp_st77916_panel.c
    ../bsp/bsp_audio.c
    ../bsp/bsp_sd.c
    ../bsp/bsp_lvgl.c
    ../bsp/bsp_i2c.c
    ../bsp/bsp_sc7a20.c
    ../bsp/bsp_font.c
    ../bsp/bsp_battery.c
    ../bsp/bsp_mic.c
    )

set(app_srcs
    app/app_state.c
    app/app_config.c
    app/app_audio.c
    app/app_mode.c
    app/app_bt.c
    app/app_usb.c
    app/app_sd.c
    app/sd_player.c
    app/gsensor_sc7a20_glue.c
    app/media_meta.c
    app/png_cover.c
    app/zlib_inflate/inflate.c
    app/zlib_inflate/inftrees.c
    app/zlib_inflate/inffast.c
    app/zlib_inflate/zutil.c
    app/zlib_inflate/adler32.c
    app/zlib_inflate/crc32.c
    app/app_ui.c
    app/app_ui_theme.c
    app/app_key.c
    app/app_shake.c
    app/assets/font_sc12.c
    app/assets/font_sc16.c
    app/assets/img_cover_default.c
    )

# A2DP sink（移植自 SDK headset 示例）+ AVRCP 车载歌词 + HFP 免提通话
set(bt_srcs
    app/bt/bt_cover_art.c
    app/bt/a2dp_sink_demo.c
    app/bt/ring_buffer_node.c
    app/bt/mpeg4_latm_dec.c
    app/bt/bt_manager.c
    app/bt/bluetooth_storage.c
    )

set(srcs
    ap_main.c
    ${app_srcs}
    ${bt_srcs}
    ${bsp_srcs}
    )

set(priv_req
    media_service
    multimedia
    fatfs
    audio_play
    bk_audio
    bk_audio_player
    audio_pipeline
    bk_gsensor
    easy_flash
    lvgl
    bk_init
    lwip_intf_v2_1
    utf8
    bk_usb
    bk_jpeg_decoder
    )

armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_req}")

# 第三方/示例代码里的未使用变量等告警不当作错误
function(set_local_library_compile_option input_target)
    set(local_compile_option)
    foreach(w unused-but-set-variable unused-variable unused-function unused-label)
        list(APPEND local_compile_option -Wno-error=${w} -Wno-${w})
    endforeach()
    target_compile_options(${input_target} PRIVATE ${local_compile_option})
endfunction()

get_filename_component(config_component_name ${CMAKE_CURRENT_LIST_DIR} NAME)
armino_component_get_target(target "${config_component_name}")
if (TARGET ${target})
    set_local_library_compile_option(${target})
endif()
