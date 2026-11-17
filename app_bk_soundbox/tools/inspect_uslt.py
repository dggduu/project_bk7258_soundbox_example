#!/usr/bin/env python3
"""Inspect ID3v2 USLT embedded lyrics of an MP3 file.

Reports the frame layout, text encoding, whether the payload is actually a
full LRC (with [mm:ss.xx] timestamps), and which in-line separator is used
for bilingual (original + translation) lines.

This mirrors the firmware logic in ap/app/media_meta.c and ap/app/app_ui.c:
  - lrc_text_has_time() / lrc_parse_text()
  - lyric_split_point()

Usage:
    python3 tools/inspect_uslt.py <file.mp3> [--show N]

Only ID3v2.3 / v2.4 are supported (same as the firmware parser).
"""

import argparse
import re
import struct
import sys

# Keep in sync with lyric_split_point() in app_ui.c.
PLAIN_SEPS = ["\n", "\r", "\\n", "\\r", " / ", " | ", "|", "\uff0f"]
UNICODE_SEPS = {
    "\u2009": "U+2009 THIN SPACE",
    "\u2002": "U+2002 EN SPACE",
    "\u2003": "U+2003 EM SPACE",
    "\u2006": "U+2006 SIX-PER-EM SPACE",
    "\u00a0": "U+00A0 NO-BREAK SPACE",
}
ENC_NAME = {
    0: "ISO-8859-1",
    1: "UTF-16 with BOM",
    2: "UTF-16BE",
    3: "UTF-8",
}
TS_RE = re.compile(r"\[(\d{1,3}):(\d{1,2})(?:\.(\d{1,3}))?\]")
META_RE = re.compile(r"^\[(ti:|ar:|al:|offset:)", re.IGNORECASE)


def syncsafe(b):
    return (b[0] << 21) | (b[1] << 14) | (b[2] << 7) | b[3]


def read_frames(path):
    with open(path, "rb") as f:
        hdr = f.read(10)
        if len(hdr) < 10 or hdr[:3] != b"ID3":
            raise ValueError("no ID3v2 header")
        version = hdr[3]
        tag = f.read(syncsafe(hdr[6:10]))
    if version < 3:
        raise ValueError("ID3v2.2 is not supported by the firmware parser")

    frames = []
    off = 0
    while off + 10 <= len(tag):
        fid = tag[off:off + 4]
        if not all(32 <= c <= 126 for c in fid):
            break
        if version >= 4:
            size = syncsafe(tag[off + 4:off + 8])
        else:
            size = struct.unpack(">I", tag[off + 4:off + 8])[0]
        if size == 0:
            break
        frames.append((fid.decode("ascii"), tag[off + 10:off + 10 + size]))
        off += 10 + size
    return version, frames


def decode_uslt(body):
    """USLT: enc(1) lang(3) desc(encoding-terminated) text."""
    if len(body) < 6:
        raise ValueError("USLT frame too short")
    enc = body[0]
    lang = body[1:4].decode("ascii", "replace")
    p = 4
    if enc in (1, 2):
        while p + 1 < len(body) and not (body[p] == 0 and body[p + 1] == 0):
            p += 2
        p = min(p + 2, len(body))
    else:
        while p < len(body) and body[p]:
            p += 1
        p = min(p + 1, len(body))

    data = body[p:]
    if enc == 0:
        text = data.decode("latin-1")
    elif enc == 1:
        if data[:2] == b"\xff\xfe":
            text = data[2:].decode("utf-16-le", "replace")
        elif data[:2] == b"\xfe\xff":
            text = data[2:].decode("utf-16-be", "replace")
        else:
            text = data.decode("utf-16-le", "replace")
    elif enc == 2:
        text = data.decode("utf-16-be", "replace")
    else:
        text = data.decode("utf-8", "replace")

    text = text.lstrip("\ufeff").rstrip("\x00")
    return ENC_NAME.get(enc, "unknown(%d)" % enc), lang, text


def parse_lrc(text):
    lines = []
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.startswith("[") and META_RE.match(line):
            continue
        rest = line
        times = []
        while len(times) < 8:
            m = TS_RE.match(rest)
            if not m:
                break
            mm = int(m.group(1))
            ss = int(m.group(2))
            xx = int(m.group(3) or 0)
            times.append(mm * 60000 + ss * 1000 + (xx * 10 if xx < 100 else xx))
            rest = rest[m.end():]
        body = rest.lstrip(" ")
        for t in times:
            if body:
                lines.append((t, body))
    lines.sort(key=lambda item: item[0])
    return lines


def split_line(s):
    for sep in PLAIN_SEPS:
        i = s.find(sep)
        if i >= 0:
            return repr(sep), s[:i].rstrip(), s[i + len(sep):].lstrip(" \t")
    for cp, name in UNICODE_SEPS.items():
        i = s.find(cp)
        if i >= 0:
            return name, s[:i].rstrip(), s[i + len(cp):].lstrip(" \t")
    return None, s, ""


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mp3", help="path to the MP3 file")
    ap.add_argument("--show", type=int, default=6,
                    help="number of parsed lines to preview (default 6)")
    args = ap.parse_args()

    try:
        version, frames = read_frames(args.mp3)
    except (OSError, ValueError) as e:
        print("error: %s" % e)
        return 1

    print("ID3v2.%d frames: %s" %
          (version, ", ".join("%s(%dB)" % (fid, len(body)) for fid, body in frames)))

    uslt = [body for fid, body in frames if fid in ("USLT", "SYLT")]
    if not uslt:
        print("no USLT/SYLT embedded lyrics")
        return 0
    if not any(fid == "USLT" for fid, _ in frames):
        print("only SYLT present (already time-stamped); nothing to inspect here")
        return 0

    body = [b for fid, b in frames if fid == "USLT"][0]
    enc, lang, text = decode_uslt(body)
    u8 = len(text.encode("utf-8"))
    print("USLT: %d bytes, encoding=%s, lang=%s" % (len(body), enc, lang))
    print("decoded: %d chars / %d UTF-8 bytes / %d physical lines" %
          (len(text), u8, len(text.splitlines())))
    if u8 > 1280:
        print("NOTE: exceeds the old 1280-byte stack buffer; the heap buffer fix is required")

    for cp, name in UNICODE_SEPS.items():
        n = sum(1 for line in text.splitlines() if cp in line)
        if n:
            print("bilingual separator %s: %d lines" % (name, n))

    lines = parse_lrc(text)
    if not lines:
        print("no LRC timestamps -> firmware falls back to even-spacing pseudo timeline")
        return 0
    print("LRC timestamps detected: %d valid lines, %d..%d ms" %
          (len(lines), lines[0][0], lines[-1][0]))

    paired = [(t, split_line(s)) for t, s in lines]
    n = sum(1 for _, (_, _, sub) in paired if sub)
    print("lines split into main + translation: %d / %d" % (n, len(lines)))
    print("")
    for t, (sep, main, sub) in paired[:max(0, args.show)]:
        print("[%6.2fs] main: %s" % (t / 1000.0, main))
        if sub:
            print("           sub : %s   (sep=%s)" % (sub, sep))
    return 0


if __name__ == "__main__":
    sys.exit(main())
