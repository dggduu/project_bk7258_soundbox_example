// Copyright 2026
// CherrySH 命令：bat —— 电池电压 ADC 测量与校准（P22 / ADC5）
//
//   bat                  读 8 次取平均，打印 raw / 两种换算 / 电量百分比
//   bat <n>              指定采样次数（1..64）
//   bat raw [n]          只打印 raw 码值与两种换算（不做电池还原）
//   bat div              查看当前 vol_div 档位
//   bat div <0..6>       切换 vol_div 并立即读一次（找合适量程用）
//   bat cal <mv>         用当前读数 + 万用表实测值反算系数（校准辅助）
//   bat en / bat en on|off  只读 / 手动驱动 BAT_ADC_EN(P49)，诊断用
//   bat help
//
// 硬件（bsp_board_cfg.h）：
//   VBAT -- R21 220k --+-- R24 220k -- GND
//                      |
//                    P22 / ADC_5        （分压比 1:2）
//   P49 曾接一颗 PMOS 做采样使能，但该电路栅极电平拉不到位（关不严/开不开），
//   app_bk_pi 已从方案里去掉，分压电阻常接。所以本命令默认**不碰 P49**，
//   直接读 P22；`bat en on|off` 仅供诊断那颗管子用。
//
// 【2026-11 实测偏差与修正】
//   实测：万用表量分压点 2040mV，`bat` 报 3288mV，比例 1.6118。
//   3288 ≈ 2040 × 3.3/2.048，即 bk_adc_data_calculate() 按 3.3V 满量程算，
//   而当前配置（vol_div=NONE + saturate_mode=3）的实际输入量程约 2.048V。
//   因此本命令额外按 2048/3300 修正一次，并打印 raw 码值：
//     raw 接近 8191 = 已饱和，2.04V 已在量程边缘（满电 4.2V/2 = 2.1V 会超），
//     必须用 `bat div <n>` 增大衰减档位，否则电池满电段无法分辨。
//
// 三个必须遵守的坑（见 MEMORY.md）：
//   ① P22 的默认第二功能是 LCD_R4，必须显式 gpio_dev_map(GPIO_22,
//   GPIO_DEV_ADC5)；
//      不要用 bk_adc_chan_init_gpio()（走 IPC 且对已映射通道直接 return）。
//   ② P49 绝不开内部上拉 —— 会把 PMOS 栅极电平拉错；只用推挽输出。
//   ③ 每次读之前都要重新 map 引脚：屏幕/别的模块初始化会把 P22 占回去。
//
// 命令注册在 ap/csh_port.c 的 s_cmd_table[]。

#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "bsp_board_cfg.h"
#include "csh.h"
#include "csh_port.h"

#include "gpio_driver.h" /* gpio_dev_map() / gpio_dev_unmap() */
#include <driver/adc.h>
#include <driver/gpio.h>
#include <os/mem.h>
#include <os/os.h>

#define BAT_ADC_TIMEOUT_MS 200
#define BAT_SAMPLE_DEF 8
#define BAT_SAMPLE_MAX 64

/* 满电 / 空电（单节锂电），用于线性估算百分比 */
#define BAT_MV_FULL 4200
#define BAT_MV_EMPTY 3000

/* 分压后读数下限：3.0V 电池 -> 1500mV。低于 500mV 基本是没接电池
 * （悬空被拉到 0）或分压电阻未贴。 */
#define BAT_ADC_VALID_MIN_MV 500

/* 实测换算修正：SDK 按 3300mV 满量程算，实际量程 2048mV。
 * 换 vol_div 档位后量程会变，用 `bat cal <mv>` 反算新系数。 */
#define BAT_ADC_FS_MV 1559
#define BAT_ADC_CALC_FS_MV 3300

/* ADC 原始码满量程：实测是 **13bit（0..8191）**，不是 12bit。
 * 判据：输入 2045mV 时 raw=8190，8190/8191*2048 = 2047.7mV（误差 0.13%）。 */
#define BAT_ADC_RAW_FS 8191

static beken_mutex_t s_bat_mutex = NULL;
static adc_vol_div_t s_vol_div = ADC_VOL_DIV_NONE;

static void bat_usage(chry_shell_t *csh) {
  csh_printf(csh, "usage:\r\n");
  csh_printf(csh,
             "  bat                 read %d samples, raw + mV + percent\r\n",
             BAT_SAMPLE_DEF);
  csh_printf(csh, "  bat <n>             read n samples (1..%d)\r\n",
             BAT_SAMPLE_MAX);
  csh_printf(csh, "  bat raw [n]         raw code + both conversions\r\n");
  csh_printf(csh,
             "  bat div [0..6]      show / set ADC vol_div (attenuation)\r\n");
  csh_printf(
      csh, "  bat scan            try all 7 vol_div steps, print raw each\r\n");
  csh_printf(csh, "  bat cal <mv>        calibrate: given real mV at P%d\r\n",
             (int)BOARD_BAT_ADC_PIN);
  csh_printf(csh,
             "  bat en [on|off]     read / drive P%d (diagnostic only)\r\n",
             (int)BOARD_BAT_ADC_EN_PIN);
  csh_printf(csh, "hw: P%d/ADC%d, 1/%d divider, VREF %dmV\r\n",
             (int)BOARD_BAT_ADC_PIN, (int)BOARD_BAT_ADC_CHAN,
             BOARD_BAT_ADC_DIV_NUM, BOARD_BAT_ADC_VREF_MV);
}

/* 把 P22 切到 ADC 功能。gpio_dev_map() 内部会先关输出/输入/上下拉/中断
 * 再写 func_sel，所以不需要先 unmap。 */
static bool bat_pin_map(void) {
  return gpio_dev_map(BOARD_BAT_ADC_PIN, GPIO_DEV_ADC5) == BK_OK;
}

static bk_err_t bat_lock_init(void) {
  if (s_bat_mutex)
    return BK_OK;
  return rtos_init_mutex(&s_bat_mutex);
}

static void bat_config_fill(adc_config_t *cfg) {
  os_memset(cfg, 0, sizeof(*cfg));
  cfg->chan = BOARD_BAT_ADC_CHAN;
  cfg->adc_mode = ADC_CONTINUOUS_MODE;
  cfg->src_clk = ADC_SCLK_XTAL_26M;
  cfg->clk = 0x30A0C5;
  cfg->saturate_mode = ADC_SATURATE_MODE_3;
  cfg->steady_ctrl = 7;
  cfg->adc_filter = 0;
  cfg->sample_rate = 0;
  cfg->is_open = 0;
  cfg->vol_div = s_vol_div;
}

/**
 * 采样 n 次，输出平均 raw 码与 SDK 换算值（mV）。
 * 全部动作在 mutex 内，避免和别的 ADC 使用者抢 acquire。
 */
static bk_err_t bat_read(chry_shell_t *csh, uint32_t n, uint32_t *raw_out,
                         uint32_t *mv_out) {
  bk_err_t ret;
  adc_config_t cfg;
  uint32_t sum_raw = 0, sum_mv = 0, i;

  if (!raw_out || !mv_out || n == 0 || n > BAT_SAMPLE_MAX)
    return BK_ERR_PARAM;

  ret = bat_lock_init();
  if (ret != BK_OK)
    return ret;

  rtos_lock_mutex(&s_bat_mutex);

  ret = bk_adc_acquire();
  if (ret != BK_OK) {
    csh_printf(csh, "adc acquire failed: %d\r\n", ret);
    goto out_unlock;
  }

  /* 每次读都重新 map：P22 默认是 LCD_R4，屏幕初始化会把它占回去 */
  if (!bat_pin_map()) {
    csh_printf(csh, "gpio_dev_map(P%d, ADC%d) failed\r\n",
               (int)BOARD_BAT_ADC_PIN, (int)BOARD_BAT_ADC_CHAN);
    ret = BK_FAIL;
    goto out_release;
  }

  ret = bk_adc_init(BOARD_BAT_ADC_CHAN);
  if (ret != BK_OK) {
    csh_printf(csh, "adc init ch%d failed: %d\r\n", (int)BOARD_BAT_ADC_CHAN,
               ret);
    goto out_release;
  }

  bat_config_fill(&cfg);
  ret = bk_adc_set_config(&cfg);
  if (ret != BK_OK) {
    csh_printf(csh, "adc set_config failed: %d\r\n", ret);
    goto out_deinit;
  }

  bk_adc_enable_bypass_clalibration();

  ret = bk_adc_start();
  if (ret != BK_OK) {
    csh_printf(csh, "adc start failed: %d\r\n", ret);
    goto out_deinit;
  }

  for (i = 0; i < n; i++) {
    uint16_t raw = 0;

    ret = bk_adc_read(&raw, BAT_ADC_TIMEOUT_MS);
    if (ret != BK_OK) {
      csh_printf(csh, "adc read failed at sample %u: %d\r\n", (unsigned)i, ret);
      break;
    }
    sum_raw += raw;
    sum_mv += bk_adc_data_calculate(raw, BOARD_BAT_ADC_CHAN);
  }

  bk_adc_stop();

  if (i == 0) {
    ret = BK_FAIL;
    goto out_deinit;
  }

  *raw_out = sum_raw / i;
  *mv_out = sum_mv / i;
  ret = BK_OK;

out_deinit:
  bk_adc_deinit(BOARD_BAT_ADC_CHAN);
out_release:
  bk_adc_release();
out_unlock:
  rtos_unlock_mutex(&s_bat_mutex);
  return ret;
}

static uint8_t bat_mv_to_percent(uint32_t mv) {
  if (mv >= BAT_MV_FULL)
    return 100;
  if (mv <= BAT_MV_EMPTY)
    return 0;
  return (uint8_t)(((mv - BAT_MV_EMPTY) * 100) / (BAT_MV_FULL - BAT_MV_EMPTY));
}

/* P49 只做普通推挽输出；绝不开内部上拉（会把 PMOS 栅极电平带错）。 */
static void bat_en_pin_config(void) {
  gpio_dev_unmap(BOARD_BAT_ADC_EN_PIN);
  bk_gpio_disable_pull(BOARD_BAT_ADC_EN_PIN);
  bk_gpio_enable_output(BOARD_BAT_ADC_EN_PIN);
}

static void bat_en_write(bool on) {
  uint32_t level = on ? 1 : 0;

  if (BOARD_BAT_ADC_EN_ACTIVE_LOW)
    level = !level;

  if (level)
    bk_gpio_set_output_high(BOARD_BAT_ADC_EN_PIN);
  else
    bk_gpio_set_output_low(BOARD_BAT_ADC_EN_PIN);
}

static void bat_show_en(chry_shell_t *csh) {
  bat_en_pin_config();
  csh_printf(csh,
             "BAT_ADC_EN P%d: pad=%d (%s-active), expect %s to conduct\r\n",
             (int)BOARD_BAT_ADC_EN_PIN, bk_gpio_get_input(BOARD_BAT_ADC_EN_PIN),
             BOARD_BAT_ADC_EN_ACTIVE_LOW ? "low" : "high",
             BOARD_BAT_ADC_EN_ACTIVE_LOW ? "pad=0" : "pad=1");
  csh_printf(csh,
             "  note: PMOS gate circuit was flaky; divider is normally\r\n");
  csh_printf(csh, "        hard-wired, 'bat' does NOT touch P%d\r\n",
             (int)BOARD_BAT_ADC_EN_PIN);
}

static const char *bat_div_name(adc_vol_div_t d) {
  switch (d) {
  case ADC_VOL_DIV_NONE:
    return "NONE";
  case ADC_VOL_DIV_1:
    return "DIV_1";
  case ADC_VOL_DIV_2:
    return "DIV_2";
  case ADC_VOL_DIV_3:
    return "DIV_3";
  case ADC_VOL_DIV_4:
    return "DIV_4";
  case ADC_VOL_DIV_5:
    return "DIV_5";
  case ADC_VOL_DIV_7:
    return "DIV_7";
  default:
    return "?";
  }
}

/* vol_div 枚举值不是连续的 0..6（最后一个是 7），按索引表取 */
static bool bat_div_by_index(int idx, adc_vol_div_t *out) {
  static const adc_vol_div_t tbl[] = {
      ADC_VOL_DIV_NONE, ADC_VOL_DIV_1, ADC_VOL_DIV_2, ADC_VOL_DIV_3,
      ADC_VOL_DIV_4,    ADC_VOL_DIV_5, ADC_VOL_DIV_7,
  };

  if (idx < 0 || idx >= (int)(sizeof(tbl) / sizeof(tbl[0])))
    return false;

  *out = tbl[idx];
  return true;
}

int bat_cmd(int argc, char **argv) {
  chry_shell_t *csh = (void *)argv[argc + 1];
  const char *sub;
  uint32_t n = BAT_SAMPLE_DEF;
  uint32_t raw = 0, mv = 0;
  bool raw_only = false;
  bk_err_t ret;

  if (argc >= 2 && !strcmp(argv[1], "help")) {
    bat_usage(csh);
    return 0;
  }

  /* bat en [on|off] */
  if (argc >= 2 && !strcmp(argv[1], "en")) {
    if (argc < 3) {
      bat_show_en(csh);
      return 0;
    }
    if (!strcmp(argv[2], "on") || !strcmp(argv[2], "1")) {
      bat_en_pin_config();
      bat_en_write(true);
    } else if (!strcmp(argv[2], "off") || !strcmp(argv[2], "0")) {
      bat_en_pin_config();
      bat_en_write(false);
    } else {
      bat_usage(csh);
      return -1;
    }
    bat_show_en(csh);
    return 0;
  }

  /* bat scan —— 同一输入电压下遍历 7 个 vol_div 档位。
   * 用途：找出"raw 落在 1/4..3/4 量程"的那一档。raw 贴近 0 说明
   * 衰减过大（分辨率不够），贴近 8191 说明量程不够（会饱和）。 */
  if (argc >= 2 && !strcmp(argv[1], "scan")) {
    adc_vol_div_t saved = s_vol_div;
    int k;

    csh_printf(csh, "vol_div scan (P%d/ADC%d):\r\n", (int)BOARD_BAT_ADC_PIN,
               (int)BOARD_BAT_ADC_CHAN);
    csh_printf(csh, "  idx  div      raw/8191   sdk mV\r\n");

    for (k = 0; k < 7; k++) {
      adc_vol_div_t d;
      uint32_t r = 0, m = 0;

      if (!bat_div_by_index(k, &d))
        continue;

      s_vol_div = d;
      ret = bat_read(csh, 4, &r, &m);
      if (ret != BK_OK) {
        csh_printf(csh, "  %3d  %-7s  read failed (%d)\r\n", k, bat_div_name(d),
                   ret);
        continue;
      }

      csh_printf(csh, "  %3d  %-7s  %4u%s   %4u\r\n", k, bat_div_name(d),
                 (unsigned)r,
                 (r >= (BAT_ADC_RAW_FS - 128)) ? " SAT"
                                               : ((r <= 64) ? " low" : "    "),
                 (unsigned)m);
    }

    s_vol_div = saved;
    csh_printf(csh, "  (div restored to %s)\r\n", bat_div_name(s_vol_div));
    csh_printf(csh,
               "  pick the step where raw ~= 2000, then 'bat div <idx>'\r\n");
    return 0;
  }

  /* bat div [0..6] */
  if (argc >= 2 && !strcmp(argv[1], "div")) {
    if (argc < 3) {
      csh_printf(csh, "vol_div = %s (%d)\r\n", bat_div_name(s_vol_div),
                 (int)s_vol_div);
      csh_printf(csh, "  index 0 = least attenuation (smallest range)\r\n");
      return 0;
    }

    {
      adc_vol_div_t d;
      int idx = atoi(argv[2]);

      if (!bat_div_by_index(idx, &d)) {
        csh_printf(csh, "invalid div index %d (0..6)\r\n", idx);
        return -1;
      }
      s_vol_div = d;
    }

    csh_printf(csh, "vol_div -> %s, measure now:\r\n", bat_div_name(s_vol_div));

    ret = bat_read(csh, BAT_SAMPLE_DEF, &raw, &mv);
    if (ret != BK_OK) {
      csh_printf(csh, "read failed: %d\r\n", ret);
      return -1;
    }
    csh_printf(
        csh, "  raw=%u/%d  sdk=%u mV  (linear)=%u mV\r\n", (unsigned)raw,
        BAT_ADC_RAW_FS, (unsigned)mv,
        (unsigned)((uint32_t)((uint64_t)raw * BAT_ADC_FS_MV / BAT_ADC_RAW_FS)));
    if (raw >= (BAT_ADC_RAW_FS - 128))
      csh_printf(csh, "  WARN: near saturation, increase div index\r\n");
    return 0;
  }

  /* bat cal <real_mv> */
  if (argc >= 3 && !strcmp(argv[1], "cal")) {
    uint32_t real_mv = (uint32_t)atoi(argv[2]);

    if (real_mv == 0) {
      bat_usage(csh);
      return -1;
    }

    ret = bat_read(csh, BAT_SAMPLE_DEF, &raw, &mv);
    if (ret != BK_OK) {
      csh_printf(csh, "read failed: %d\r\n", ret);
      return -1;
    }

    csh_printf(csh, "raw=%u/%d  sdk=%u mV  real=%u mV (div=%s)\r\n",
               (unsigned)raw, BAT_ADC_RAW_FS, (unsigned)mv, (unsigned)real_mv,
               bat_div_name(s_vol_div));
    if (raw == 0 || mv == 0) {
      csh_printf(csh, "raw/mv = 0, cannot calibrate\r\n");
      return -1;
    }
    csh_printf(csh, "  scale     : real/sdk = %u/1000\r\n",
               (unsigned)((uint64_t)real_mv * 1000 / mv));
    csh_printf(csh, "  implied FS: %u mV (sdk assumes %d mV)\r\n",
               (unsigned)((uint64_t)real_mv * BAT_ADC_CALC_FS_MV / mv),
               BAT_ADC_CALC_FS_MV);
    csh_printf(csh, "  mv per raw: %u uV\r\n",
               (unsigned)((uint64_t)real_mv * 1000 / raw));
    if (raw >= (BAT_ADC_RAW_FS - 128))
      csh_printf(csh,
                 "  WARN: raw near full scale, this point is unreliable\r\n");
    return 0;
  }

  sub = (argc >= 2) ? argv[1] : NULL;

  /* bat raw [n] */
  if (sub && !strcmp(sub, "raw")) {
    raw_only = true;
    if (argc >= 3)
      n = (uint32_t)atoi(argv[2]);
  } else if (sub) {
    /* bat <n>：只接受纯十进制数字，避免 'bat j' 被 atoi() 变成 0
     * 然后报一个误导性的 "invalid sample count 0" */
    char *end = NULL;
    long v = strtol(sub, &end, 10);

    if (end == sub || (end && *end != '\0') || v <= 0) {
      csh_printf(csh, "unknown subcommand '%s'\r\n", sub);
      bat_usage(csh);
      return -1;
    }
    n = (uint32_t)v;
  }

  if (n == 0 || n > BAT_SAMPLE_MAX) {
    csh_printf(csh, "invalid sample count %u (1..%d)\r\n", (unsigned)n,
               BAT_SAMPLE_MAX);
    bat_usage(csh);
    return -1;
  }

  ret = bat_read(csh, n, &raw, &mv);
  if (ret != BK_OK) {
    csh_printf(csh, "read failed: %d\r\n", ret);
    return -1;
  }

  {
    /* 直接用 raw 线性换算：实测比 "sdk mV × 系数" 更准（0.1% 量级） */
    uint32_t adc_mv =
        (uint32_t)((uint64_t)raw * BAT_ADC_FS_MV / BAT_ADC_RAW_FS);

    csh_printf(csh, "P%d/ADC%d  %u samples  vol_div=%s\r\n",
               (int)BOARD_BAT_ADC_PIN, (int)BOARD_BAT_ADC_CHAN, (unsigned)n,
               bat_div_name(s_vol_div));
    csh_printf(csh, "  raw    : %u / %d%s\r\n", (unsigned)raw, BAT_ADC_RAW_FS,
               (raw >= (BAT_ADC_RAW_FS - 128)) ? "  (NEAR SATURATION)" : "");
    csh_printf(csh,
               "  adc    : %u mV (corrected, %dmV FS) | %u mV (sdk calc)\r\n",
               (unsigned)adc_mv, BAT_ADC_FS_MV, (unsigned)mv);

    if (raw_only) {
      csh_printf(csh,
                 "  hint   : raw near %d = divider ratio too small (1/%d).\r\n",
                 BAT_ADC_RAW_FS, BOARD_BAT_ADC_DIV_NUM);
      csh_printf(csh, "           vol_div steps do NOT change the analog front "
                      "end on this board\r\n");
      return 0;
    }

    if (adc_mv < BAT_ADC_VALID_MIN_MV) {
      csh_printf(
          csh, "  battery: -- mV  (too low: no battery / divider missing)\r\n");
      return 0;
    }

    csh_printf(csh, "  battery: %u mV  (~%u%%)\r\n",
               (unsigned)(adc_mv * BOARD_BAT_ADC_DIV_NUM),
               (unsigned)bat_mv_to_percent(adc_mv * BOARD_BAT_ADC_DIV_NUM));
    csh_printf(csh, "  range  : %d..%d mV = 0..100%%\r\n", BAT_MV_EMPTY,
               BAT_MV_FULL);
  }

  return 0;
}
