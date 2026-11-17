# app_test_uart1: UART1 上的 CherrySH + LDO_EN 测试
cmake_minimum_required(VERSION 3.5)

# SDK 的 project.cmake 位置由 ARMINO_TOOLS_PATH 决定（= <sdk>/tools）。
#   - make 构建：build_main.mk 会导出 ARMINO_TOOLS_PATH，直接可用；
#   - 单独用 VSCode 的 CMake Tools 配置：没有这个变量，这里用 ARMINO_PATH 兜底。
# 注意不要用 $ENV{ARMINO_PATH}/tools 直接 include：AP 子构建里 ARMINO_PATH
# 会被设成 <sdk>/ap，拼出来就是 <sdk>/ap/tools/...，一定找不到。
if(NOT DEFINED ENV{ARMINO_TOOLS_PATH} OR "$ENV{ARMINO_TOOLS_PATH}" STREQUAL "")
    if(DEFINED ENV{ARMINO_PATH} AND EXISTS "$ENV{ARMINO_PATH}/tools/build_tools/cmake/project.cmake")
        set(ENV{ARMINO_TOOLS_PATH} "$ENV{ARMINO_PATH}/tools")
    endif()
endif()

# Linux 下工具链路径（来自 armino_env_setup.sh 安装位置）
if(NOT DEFINED ENV{COMPILER_TOOLCHAIN_PATH} OR "$ENV{COMPILER_TOOLCHAIN_PATH}" STREQUAL "")
    if(EXISTS "/opt/gcc-arm-none-eabi-10.3-2021.10/bin/arm-none-eabi-gcc")
        set(ENV{COMPILER_TOOLCHAIN_PATH} "/opt/gcc-arm-none-eabi-10.3-2021.10/bin")
    endif()
endif()

include($ENV{ARMINO_TOOLS_PATH}/build_tools/cmake/project.cmake)
project(app)
