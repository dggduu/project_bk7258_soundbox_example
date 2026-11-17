#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mkfont.py —— 给 app_bk_pi 做一份能塞进设备的字体子集。

设备端（bsp/bsp_font.c）会把整份字体读进 PSRAM，上限 4MB；直接放
Noto Sans CJK（18MB 的 .ttc）会超限并回退内置字库。这个脚本把大字体
裁成只含目标字符的子集，中日英同时保留，并丢掉 LVGL 用不到的
GSUB/GPOS（CJK 字体里这两张表常占 2MB）。

依赖：pip install fonttools

用法：
    python3 tools/mkfont.py 输入.ttf/.otf/.ttc -o font.ttf
    python3 tools/mkfont.py input.ttc --face 2 --preset cjk -o font.ttf
    python3 tools/mkfont.py input.ttf --preset ascii,kana,gb2312-1 --max-size 1500
    python3 tools/mkfont.py input.ttf --text "扩" --chars-file mychars.txt

然后把生成的 font.ttf 放到 SD 卡根目录，重启即可（日志看 font source:）。

实测（Noto Sans CJK SC face 2，drop GSUB/GPOS）：
    --preset cjk                    8325 字  ≈ 2080 KB   （与内置字库等价）
    --preset ascii,kana,gb2312-1    5486 字  ≈ 1326 KB   （不含二级汉字）

预设字符集（可逗号叠加）：
    ascii       0x20-0x7E 基本拉丁
    kana        平假名 + 片假名
    jiskanji    JIS X 0208 第 1 水准日文汉字
    gb2312-1    GB2312 一级汉字（3755）
    gb2312-2    GB2312 二级汉字（3008）
    symbols     全角标点/全角字母/常用符号
    cjk         = kana + jiskanji + gb2312-1 + gb2312-2 + symbols（默认）
    all         输入字体里的全部字符（不推荐，等于不做子集）

默认 --preset cjk，与固件内置字库的字符集对齐：换成 TTF 后不会比内置少字。
"""

import argparse
import os
import sys


def charset_ascii():
    return {chr(c) for c in range(0x20, 0x7F)}


def charset_kana():
    # 平假名/片假名 + ゝ々 之类
    return {chr(c) for c in range(0x3041, 0x3100)}


def charset_jis_kanji():
    # JIS X 0208 第 1 水准：EUC-JP 0xB0..0xCF 区（6355 字里的常用部分）
    out = set()
    for hi in range(0xB0, 0xD0):
        for lo in range(0xA1, 0xFF):
            try:
                out.add(bytes([hi, lo]).decode("euc_jp"))
            except UnicodeDecodeError:
                pass
    return out


def charset_gb2312(level):
    # GB2312 汉字区：一级 0xB0A1~0xD7F9，二级 0xD8A1~0xF7FE
    out = set()
    hi_end = 0xD8 if level == 1 else 0xF8
    for hi in range(0xB0, hi_end):
        for lo in range(0xA1, 0xFF):
            try:
                out.add(bytes([hi, lo]).decode("gb2312"))
            except UnicodeDecodeError:
                pass
    return out


def charset_symbols():
    # 全角标点、全角字母数字、常用符号（覆盖内置字库的界面符号）
    out = {chr(c) for c in range(0x3000, 0x3040)}   # CJK 符号
    out |= {chr(c) for c in range(0xFF01, 0xFF60)}  # 全角 ASCII
    out |= {chr(c) for c in range(0xFFE0, 0xFFE7)}  # 货币等
    out |= {chr(c) for c in range(0x2010, 0x2030)}  # 引号/破折号/省略号
    return out


def build_charset(names):
    cjk_all = {"kana", "jiskanji", "gb2312-1", "gb2312-2", "cjk", "symbols"}
    picked = set()

    for name in names:
        name = name.strip().lower()
        if not name:
            continue
        if name == "ascii":
            picked |= charset_ascii()
        elif name == "kana":
            picked |= charset_kana()
        elif name == "jiskanji":
            picked |= charset_jis_kanji()
        elif name == "gb2312-1":
            picked |= charset_gb2312(1)
        elif name == "gb2312-2":
            picked |= charset_gb2312(2)
        elif name == "symbols":
            picked |= charset_symbols()
        elif name == "cjk":
            for sub in ("kana", "jiskanji", "gb2312-1", "gb2312-2", "symbols"):
                picked |= build_charset([sub])
        elif name == "all":
            return None                 # None = 不过滤
        else:
            raise SystemExit(f"未知预设: {name}（可用: ascii,kana,jiskanji,gb2312-1,"
                             f"gb2312-2,symbols,cjk,all）")

    if picked and not (cjk_all & set(names)):
        picked |= charset_symbols()      # 至少给点标点
    return picked


def face_family_name(font):
    try:
        n1 = font["name"].getDebugName(1) or ""
        n4 = font["name"].getDebugName(4) or ""
        return n1 if n1 == n4 or not n4 else f"{n1} ({n4})"
    except Exception:
        return "?"


def load_face(path, face, list_only=False):
    """返回 (可写的 TTFont, 总 face 数, 用的下标)。
    .ttc 按 face 取一个；不指定 face 时优先挑简体中文面（SC/CN）。"""
    from fontTools.ttLib import TTCollection, TTFont

    with open(path, "rb") as f:
        tag = f.read(4)

    if tag == b"ttcf":
        coll = TTCollection(path)
        if list_only:
            for i, f in enumerate(coll.fonts):
                print(f"    face {i}: {face_family_name(f)}")
            raise SystemExit(0)

        if face is not None:
            if face < 0 or face >= len(coll.fonts):
                raise SystemExit(f"face {face} 越界（{path} 里有 {len(coll.fonts)} 个）")
            return coll.fonts[face], len(coll.fonts), face

        # 自动挑：简体中文面优先（Noto CJK 的 face 顺序 JP/KR/SC/TC/HK…，
        # 拿 face 0 会得到日文字形，"骨""直""黑"等写法不同）
        prefer = ("SC", "Simplified", "CN", "Chinese")
        for i, f in enumerate(coll.fonts):
            nm = face_family_name(f)
            if any(k in nm for k in prefer):
                return f, len(coll.fonts), i
        for i, f in enumerate(coll.fonts):
            if "cmap" in f:
                return f, len(coll.fonts), i
        return coll.fonts[0], len(coll.fonts), 0

    return TTFont(path), 1, 0


def main():
    ap = argparse.ArgumentParser(
        description="给 app_bk_pi 生成字体子集（SD 卡放到 1:/font.ttf）",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    ap.add_argument("input", help="输入字体 .ttf/.otf/.ttc")
    ap.add_argument("-o", "--output", default="font.ttf", help="输出子集 ttf（默认 font.ttf）")
    ap.add_argument("--face", type=int, default=None,
                    help=".ttc 的 face 下标；不给就自动挑简体中文面（SC/CN）")
    ap.add_argument("--list-faces", action="store_true",
                    help="只列出 .ttc 里的 face 名字后退出")
    ap.add_argument("--preset", default="cjk",
                    help="字符集预设，逗号分隔（默认 cjk）")
    ap.add_argument("--text", default="", help="追加的字符")
    ap.add_argument("--chars-file", help="追加字符的文本文件（UTF-8，每行或整段均可）")
    ap.add_argument("--max-size", type=float, default=3072.0,
                    help="输出上限 KB，超过则报错（默认 3072；设备硬上限 4096）")
    ap.add_argument("--keep-gsub", action="store_true",
                    help="保留 GSUB/GPOS 表（连字/字距；LVGL 不用它们，通常徒增体积）")
    ap.add_argument("--report", action="store_true", help="打印表级体积报告")
    args = ap.parse_args()

    if not os.path.isfile(args.input):
        raise SystemExit(f"找不到输入文件: {args.input}")

    try:
        from fontTools import subset
    except ImportError:
        raise SystemExit("缺少 fontTools，请先：pip install fonttools")

    font, nfaces, used_face = load_face(args.input, args.face, args.list_faces)
    print(f"[info] 输入: {args.input}（{nfaces} 个 face，用 face {used_face}"
          f": {face_family_name(font)}）")

    names = [p for p in args.preset.split(",") if p.strip()]
    want_all = any(n.strip().lower() == "all" for n in names)
    extra = 0

    if want_all:
        chars = None
    else:
        chars = build_charset(names)
        for ch in args.text:
            chars.add(ch)
            extra += 1
        if args.chars_file:
            with open(args.chars_file, encoding="utf-8") as f:
                for line in f:
                    for ch in line:
                        if ch.strip():
                            chars.add(ch)
                            extra += 1
        # 去掉换行/制表等不可见字符
        chars = {c for c in chars if c not in "\r\n\t"}
        print(f"[info] 字符集: {'+'.join(names)} + 自定义 {extra} = {len(chars)} 字")

    opts = subset.Options()
    opts.layout_features = ["*"]
    opts.name_IDs = ["*"]
    opts.notdef_outline = True
    opts.recalc_bounds = True
    opts.recalc_timestamp = False
    # 关键：CJK 字体的 GSUB/GPOS 加起来常有 2MB，LVGL 渲染是按 codepoint 直接取
    # 字形、不用 OpenType 特性，丢掉它们体积几乎砍半（实测 8325 字：4037KB→2080KB）。
    if not args.keep_gsub:
        opts.drop_tables += ["GSUB", "GPOS"]
    opts.drop_tables += ["DSIG", "FFTM"]

    subsetter = subset.Subsetter(opts)
    if chars is not None:
        subsetter.populate(text="".join(sorted(chars)))
    else:
        cmap = font.getBestCmap()
        subsetter.populate(unicodes=set(cmap.keys()))
    subsetter.subset(font)

    out = args.output
    font.save(out)

    size = os.path.getsize(out)
    print(f"[ok] 输出: {out}  {size / 1024:.0f} KB")
    if size > args.max_size * 1024:
        print(f"[warn] 超过 --max-size {args.max_size:.0f}KB！"
              f"设备上限 4096KB 仍会拒绝。建议："
              f"--preset ascii,kana,gb2312-1（去掉二级汉字）")
        sys.exit(2)

    if args.report:
        from fontTools.ttLib import TTFont
        f2 = TTFont(out)
        total = 0
        for tag, entry in sorted(f2.reader.tables.items(),
                                 key=lambda kv: -kv[1].length):
            print(f"    {tag:<6} {entry.length / 1024:8.1f} KB")
            total += entry.length
        print(f"    {'total':<6} {total / 1024:8.1f} KB")


if __name__ == "__main__":
    main()
