# 中文字库来源与许可

## 来源

`font_sc16.c` / `font_sc12.c` 不是 SDK 自带字体，也不是手工点阵，而是从
**Noto Sans CJK SC Regular** 生成的 LVGL v8 子集字体。

- 上游：<https://github.com/notofonts/noto-cjk>
- 本机使用的文件：`/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc`
  （TTC face index 2，即 Noto Sans CJK SC）
- 许可证：**SIL Open Font License 1.1**（见同目录 `OFL.txt`）
- 版权：Copyright 2014-2021 Adobe (<http://www.adobe.com/>)，
  with Reserved Font Name 'Source'；Noto 是 Google 的商标。

## 生成方式

1. 用 `fontTools` 从 TTC 中取出 SC face，另存为 TTF；
2. 字符集取 **GB2312 一级+二级汉字 + JIS X 0208 第 1 水准日文汉字 + 假名 + ASCII + 界面用标点/符号**；
3. 用 LVGL 官方 `lv_font_conv` 生成：

```bash
lv_font_conv --font NotoSansSC-Regular.ttf \
  --symbols "$(cat chars.txt)" \
  --size 16 --bpp 1 --format lvgl --lv-include lvgl.h \
  -o font_sc16.c --force-fast-kern-format

lv_font_conv --font NotoSansSC-Regular.ttf \
  --symbols "$(cat chars.txt)" \
  --size 12 --bpp 1 --format lvgl --lv-include lvgl.h \
  -o font_sc12.c --force-fast-kern-format
```

结果就是 `ap/app/assets/font_sc16.c` 和 `font_sc12.c`：16px / 12px、1bpp、
LVGL v8 `lv_font_t` 格式。1bpp + 约 8400 字是为了在 2.1MB AP 分区里同时
覆盖常用中文、日文汉字和假名，并保持歌词可读。

这不是完整 Unicode 字库；CJK 扩展 A/B、生僻繁体、韩文等仍可能缺字。
需要时把对应字符加入脚本里的字符集重新生成即可。

## 重新生成

仓库里提供了 `tools/gen_cjk_fonts.sh`，依赖：

- Python 3 + `fonttools`
- Node.js + `lv_font_conv`（可 `npm i -g lv_font_conv`）
- 系统安装 Noto Sans CJK（或用 `NOTO_CJK_TTC` 指定 TTC 路径）

```bash
cd projects/app_bk_pi
./tools/gen_cjk_fonts.sh
```

脚本会把结果写回 `ap/app/assets/`。如果替换成其他字体，请同时保留该字体的
许可证文件，并确认允许嵌入/再分发。

## 为什么不用 SDK 自带字体

SDK 的 `lv_font_simsun_16_cjk` 约 1MB 源码/较大二进制，全量打开会明显挤压
1.9MB 的 AP 分区；本工程只需要界面和歌词常用字，所以用子集字体。

## 注意

- 字库覆盖 GB2312 一级+二级、JIS X 0208 第 1 水准和假名；CJK 扩展区、
  生僻繁体、韩文等仍可能显示方框，需要时扩展字符集重新生成。
- `ap/app/app_ui.c` 里所有固定中文文案都已检查在子集内；动态歌词/ID3 文本
  超出子集时同样会缺字。
