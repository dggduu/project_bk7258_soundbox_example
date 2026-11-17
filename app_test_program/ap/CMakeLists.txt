set(incs
    .
    ${CMAKE_CURRENT_LIST_DIR}/../bsp/include
    ${CMAKE_CURRENT_LIST_DIR}/../components/CherrySH
    ${CMAKE_CURRENT_LIST_DIR}/../components/CherrySH/cherryrl
    )

# CherrySH 组件放在工程目录下（components/CherrySH），直接编进 app 组件，
# 避免改动 SDK 的组件扫描逻辑。
#
# 只编译 shell 内核 + readline；builtin/（help/shsize/login）不参与编译：
# 本 SDK 里 app 组件打成 libap.a，靠链接段自动注册的内建命令会被 ld 丢掉，
# 命令统一在 ap/csh_port.c 的 s_cmd_table[] 里显式注册（help 是自己实现的）。
file(GLOB cherry_srcs
    "${CMAKE_CURRENT_LIST_DIR}/../components/CherrySH/*.c"
    "${CMAKE_CURRENT_LIST_DIR}/../components/CherrySH/cherryrl/*.c"
    )
# cherryrl/example.c 是 readline 的独立 demo，自带 main()，必须排除
list(FILTER cherry_srcs EXCLUDE REGEX "/example\\.c$")

# 板级电源 BSP（LDO_EN）
file(GLOB bsp_srcs "${CMAKE_CURRENT_LIST_DIR}/../bsp/*.c")

set(srcs
    ap_main.c
    csh_port.c
    cmd_ldo.c
    cmd_i2c.c
    cmd_ov3660.c
    cmd_lcdq.c
    cmd_bat.c
    cmd_camera.c
    # 官方 dvp_example 的帧队列（ready/free 队列），配合 bk_camera 组件用，
    # 避免自己维护 s_frame 在 SMP 下的竞态/双重释放
    dvp_frame_list.c
    # YUV-only 工程：给 bk_dvp suspend/resume 引用的 h264/jpeg 编码器接口补空实现
    camera_enc_stub.c
    cmd_dmic.c
    cmd_sd.c
    cmd_music.c
    # USB 虚拟串口（CDC ACM），跑在 SDK 的 CherryUSB 设备栈上
    usb_vcom.c
    cmd_usb.c
    ${bsp_srcs}
    ${cherry_srcs}
    )

# USB 设备栈（usbd_core + MUSBD 端口驱动）由 SDK 的 bk_usb 组件在
# CONFIG_USB_DEVICE=y 时编译，整个 bk_usb 组件本来就在链接列表里，这里不额外
# 声明依赖（免得把 CherryUSB 的一堆 include 目录引进来）。
# bk_peripheral: dvp_sensor_devices.h (dvp_camera_i2c_* 声明)
# bk_camera: 官方 camera 控制层（bk_camera_dvp_ctlr_new/open）+ dvp 帧队列
set(priv_req  media_service multimedia audio_play bk_peripheral image_convert bk_camera)

if (CONFIG_USB_DEVICE)
    message(STATUS "app: usb cdc acm vcom enabled")
else()
    message(WARNING "app: CONFIG_USB_DEVICE 没有打开，usb 虚拟串口跑不起来"
                    "（在 ap/config/bk7258_ap/config 里打开后重新编译）")
endif()


armino_component_register(SRCS "${srcs}" INCLUDE_DIRS "${incs}" PRIV_REQUIRES "${priv_req}"
)
