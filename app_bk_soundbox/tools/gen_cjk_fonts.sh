#!/usr/bin/env bash
# 重新生成 ap/app/assets/font_sc16.c / font_sc12.c
#
# 依赖：
#   - Python 3 + fonttools（pip install fonttools）
#   - lv_font_conv（npm i -g lv_font_conv，或用 LV_FONT_CONV 指定路径）
#   - Noto Sans CJK（默认 /usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc）
#
# 用法：
#   cd projects/app_bk_pi
#   ./tools/gen_cjk_fonts.sh
#
# 可用环境变量：
#   NOTO_CJK_TTC=/path/to/NotoSansCJK-Regular.ttc
#   NOTO_CJK_FACE=2
#   LV_FONT_CONV=/path/to/lv_font_conv

set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
ASSETS="$ROOT/ap/app/assets"
TTC="${NOTO_CJK_TTC:-/usr/share/fonts/noto-cjk/NotoSansCJK-Regular.ttc}"
FACE="${NOTO_CJK_FACE:-2}"
LVFC="${LV_FONT_CONV:-lv_font_conv}"

command -v python3 >/dev/null || { echo "python3 not found"; exit 1; }
command -v "$LVFC" >/dev/null || {
	echo "lv_font_conv not found: $LVFC"
	echo "install: npm i -g lv_font_conv"
	exit 1
}
python3 -c 'import fontTools' 2>/dev/null || {
	echo "fonttools not found: pip install fonttools"
	exit 1
}
[ -f "$TTC" ] || { echo "Noto CJK TTC not found: $TTC"; exit 1; }

WORK="$(mktemp -d)"
trap 'rm -rf "$WORK"' EXIT

# GB2312 汉字区：一级 0xB0A1~0xD7F9（3755 字，常用），二级 0xD8A1~0xF7FE（3008 字）
# GB2312_LEVEL=1 只保留一级汉字：字库约省 40%（AP FLASH 已到 93.9%，紧张时用）。
# 代价：二级汉字（较生僻）显示为方框。
LEVEL="${GB2312_LEVEL:-2}"
case "$LEVEL" in
	1|2) ;;
	*) echo "GB2312_LEVEL must be 1 or 2 (got: $LEVEL)"; exit 1 ;;
esac

python3 - "$TTC" "$FACE" "$WORK/NotoSansSC-Regular.ttf" "$WORK/chars.txt" "$LEVEL" <<'PY'
import sys
from fontTools.ttLib import TTCollection

ttc, face, ttf_out, chars_out = sys.argv[1], int(sys.argv[2]), sys.argv[3], sys.argv[4]
level = int(sys.argv[5])
collection = TTCollection(ttc)
if face < 0 or face >= len(collection.fonts):
    raise SystemExit(f"face index {face} out of range (0..{len(collection.fonts)-1})")
collection.fonts[face].save(ttf_out)

chars = set()
# GB2312 一级（+ 二级，取决于 GB2312_LEVEL）
hi_end = 0xD8 if level == 1 else 0xF8
for hi in range(0xB0, hi_end):
    for lo in range(0xA1, 0xFF):
        try:
            chars.add(bytes([hi, lo]).decode("gb2312"))
        except UnicodeDecodeError:
            pass
# JIS X 0208 第 1 水准（EUC-JP 0xB0~0xCF），补日文汉字
for hi in range(0xB0, 0xD0):
    for lo in range(0xA1, 0xFF):
        try:
            chars.add(bytes([hi, lo]).decode("euc_jp"))
        except UnicodeDecodeError:
            pass
# 日文假名、日文标点、全角符号
for a, b in [(0x3000, 0x303F), (0x3040, 0x30FF),
             (0x31F0, 0x31FF), (0xFF01, 0xFF9F)]:
    for cp in range(a, b + 1):
        chars.add(chr(cp))
# ASCII + 界面/歌词常用符号
chars.update(chr(i) for i in range(0x20, 0x7F))
chars.update("·×÷—‘’“”…℃←↑→↓■□○●★☆♪♫、。《》【】！％（）＋，－：；？")
chars.discard("\n")
chars.discard("\r")
with open(chars_out, "w", encoding="utf-8", newline="") as f:
    f.write("".join(sorted(chars)))
print(f"chars: {len(chars)}")
PY

for size in 16 12; do
	out="$WORK/font_sc${size}.c"
	"$LVFC" --font "$WORK/NotoSansSC-Regular.ttf" \
		--symbols "$(cat "$WORK/chars.txt")" \
		--size "$size" --bpp 1 --format lvgl --lv-include lvgl.h \
		-o "$out" --force-fast-kern-format
	cp "$out" "$ASSETS/font_sc${size}.c"
	echo "wrote $ASSETS/font_sc${size}.c"
done

echo "done. Font license: ap/app/assets/OFL.txt, notice: FONT_NOTICE.md"
