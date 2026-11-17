# The following lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# SDK 通过环境变量 COMPILER_TOOLCHAIN_PATH 决定交叉编译器位置，
# 若外层没传（Windows 下 make -> python -> cmake 容易丢），这里补一个本机默认值。
# 注意：SDK 内部对该变量未加引号，路径不能含空格，因此优先使用无空格路径。
if(NOT DEFINED ENV{COMPILER_TOOLCHAIN_PATH} OR "$ENV{COMPILER_TOOLCHAIN_PATH}" STREQUAL "")
    if(EXISTS "E:/tools/arm_tools/gcc132/bin/arm-none-eabi-gcc.exe")
        set(ENV{COMPILER_TOOLCHAIN_PATH} "E:/tools/arm_tools/gcc132/bin")
    elseif(EXISTS "E:/tools/arm_tools/13.2 Rel1/bin/arm-none-eabi-gcc.exe")
        set(ENV{COMPILER_TOOLCHAIN_PATH} "E:/tools/arm_tools/13.2 Rel1/bin")
    endif()
endif()

include($ENV{ARMINO_TOOLS_PATH}/build_tools/cmake/project.cmake)
project(app)
