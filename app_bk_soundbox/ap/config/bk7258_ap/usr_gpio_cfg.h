// Copyright 2020-2021 Beken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#define GPIO_DEFAULT_DEV_CONFIG                                                                               \
  {                                                                                                           \
      /* ---------- 调试口 / SWD：保持原样，不要动 ---------- */                                              \
      {GPIO_0, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_I2C1_SCL, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_1, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_I2C1_SDA, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_10, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_UART0_RXD, GPIO_IO_DISABLE,                                \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_DISABLE},                                                \
      {GPIO_11, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_UART0_TXD, GPIO_IO_DISABLE,                                \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_DISABLE},                                                \
                                                                                                              \
      /* ---------- QSPI LCD：交给 QSPI 驱动，表里释放 ---------- */                                          \
      {GPIO_2, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_3, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_4, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_5, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_6, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_7, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- DMIC：交给 DMIC 驱动 ---------- */                                                        \
      {GPIO_8, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_9, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                   \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- SDIO 4bit：CLK/CMD/D0~D3 在默认表里直接复用给 SDIO ---------- */                          \
      {GPIO_14, GPIO_SECOND_FUNC_ENABLE, GPIO_DEV_SDIO_HOST_CLK, GPIO_IO_DISABLE,                             \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_15, GPIO_SECOND_FUNC_ENABLE, GPIO_DEV_SDIO_HOST_CMD, GPIO_IO_DISABLE,                             \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_16, GPIO_SECOND_FUNC_ENABLE, GPIO_DEV_SDIO_HOST_DATA0, GPIO_IO_DISABLE,                           \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_17, GPIO_SECOND_FUNC_ENABLE, GPIO_DEV_SDIO_HOST_DATA1, GPIO_IO_DISABLE,                           \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_18, GPIO_SECOND_FUNC_ENABLE, GPIO_DEV_SDIO_HOST_DATA2, GPIO_IO_DISABLE,                           \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_19, GPIO_SECOND_FUNC_ENABLE, GPIO_DEV_SDIO_HOST_DATA3, GPIO_IO_DISABLE,                           \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_3,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- SD_DET：输入 + 上拉，低有效检测 ---------- */                                             \
      {GPIO_40, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_INPUT_ENABLE,                                \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- I2C0（摄像头 SCCB）：交给 I2C 驱动，表里释放 ---------- */                                \
      {GPIO_20, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_2,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_21, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_2,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- I2C1（触摸）：交给 I2C 驱动 ---------- */                                                 \
      {GPIO_42, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_2,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_43, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_2,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- TP_INT：输入 + 上拉，交给触摸驱动配中断 ---------- */                                     \
      {GPIO_53, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_INPUT_ENABLE,                                \
       GPIO_PULL_UP_EN, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                            \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- ADC：输入，交给 ADC 驱动 ---------- */                                                    \
      {GPIO_22, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_INPUT_ENABLE,                                \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_28, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_INPUT_ENABLE,                                \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- 背光 PWM3：交给 PWM 驱动，表里先输出低（灭） ---------- */                                \
      {GPIO_23, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_DISABLE, GPIO_INT_DISABLE,                                               \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_1, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                                   \
                                                                                                              \
      /* ---------- 电源使能：必须在表里确定初始状态 ---------- */ /* PWR_1V5_EN:                             \
                                                                      高有效，初值先输出低（关） \
                                                                    */                                        \
      {GPIO_26, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_DISABLE, GPIO_INT_DISABLE,                                               \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_1, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE}, /* PWR_MIC_EN:                                                    \
                                               高有效，初值先输出低（关） */                     \
      {GPIO_48, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_DISABLE, GPIO_INT_DISABLE,                                               \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_1, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE}, /* BAT_ADC_EN: PMOS                                               \
                                               低有效，初值输出高（分压断开，省电）         \
                                             */                                                               \
      {GPIO_49, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_UP_EN, GPIO_INT_DISABLE,                                                 \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_1, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE}, /* PWR_2V8_EN:                                                    \
                                               高有效，初值先输出低（关） */                     \
      {GPIO_50, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_DISABLE, GPIO_INT_DISABLE,                                               \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_1, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE}, /* PWR_SCR_EN: PMOS                                               \
                                               低有效，初值输出高（屏 3V3 关）                   \
                                             */                                                               \
      {GPIO_51, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_UP_EN, GPIO_INT_DISABLE,                                                 \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_3, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE}, /* PA_EN:                                                         \
                                               高有效，初值输出低（功放关） */                  \
      {GPIO_52, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID,                                                   \
       GPIO_OUTPUT_ENABLE, GPIO_PULL_DISABLE, GPIO_INT_DISABLE,                                               \
       GPIO_INT_TYPE_LOW_LEVEL, GPIO_LOW_POWER_DISCARD_IO_STATUS,                                             \
       GPIO_DRIVER_CAPACITY_1, GPIO_INIT_ENABLE,                                                              \
       GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                                   \
                                                                                                              \
      /* ---------- 其余未用/备用 GPIO：保持高阻 ---------- */                                                \
      {GPIO_12, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_13, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_24, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_25, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_41, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_44, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_45, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_46, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_47, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- 摄像头 DVP：交给摄像头驱动，表里释放 ---------- */                                        \
      {GPIO_27, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_29, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_30, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_31, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_32, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_33, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_34, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_35, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_36, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_37, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_38, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_39, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_1,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
                                                                                                              \
      /* ---------- 未使用 ---------- */                                                                      \
      {GPIO_54, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
      {GPIO_55, GPIO_SECOND_FUNC_DISABLE, GPIO_DEV_INVALID, GPIO_IO_DISABLE,                                  \
       GPIO_PULL_DISABLE, GPIO_INT_DISABLE, GPIO_INT_TYPE_LOW_LEVEL,                                          \
       GPIO_LOW_POWER_DISCARD_IO_STATUS, GPIO_DRIVER_CAPACITY_0,                                              \
       GPIO_INIT_ENABLE, GPIO_TIME_SHARING_MULTIPLEX_ENABLE},                                                 \
  }

// Just for PWM_USE_DEFAULT_GPIO_MAP is disabled, define the GPIO_PWM_MAP_TABLE
// here
#if !CONFIG_PWM_USE_DEFAULT_GPIO_MAP
#define GPIO_PWM_MAP_TABLE                                                     \
  {                                                                            \
    {                                                                          \
        {GPIO_6, GPIO_DEV_PWM0},  {GPIO_7, GPIO_DEV_PWM1},                     \
        {GPIO_8, GPIO_DEV_PWM2},  {GPIO_9, GPIO_DEV_PWM3},                     \
        {GPIO_24, GPIO_DEV_PWM4}, {GPIO_25, GPIO_DEV_PWM5},                    \
    },                                                                         \
    {                                                                          \
      {GPIO_32, GPIO_DEV_PWM6}, {GPIO_33, GPIO_DEV_PWM7},                      \
          {GPIO_34, GPIO_DEV_PWM8}, {GPIO_35, GPIO_DEV_PWM9},                  \
          {GPIO_36, GPIO_DEV_PWM10}, {GPIO_37, GPIO_DEV_PWM11},                \
    }                                                                          \
  }
#endif

#ifdef __cplusplus
}
#endif
