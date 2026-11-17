/*
 * png_cover.c - 流式 PNG 解码 + box 降采样（见 png_cover.h 的说明）
 *
 * 流程：解析 chunk -> IDAT 喂 zlib inflate -> 每解出一条原始扫描线
 * -> unfilter（引用上一行）-> 按"目标->源"box 映射累加 RGB
 * -> 该目标行覆盖的源行收齐后除以像素数，物化为 RGB565 写入 dst。
 *
 * 写入采用延迟策略（write_ptr）：
 *   - 缩小时每个目标行都覆盖到源行，顺序写满；
 *   - 放大时目标行数多于源行（跳行），用最近一次物化行回填空行；
 *   - 收尾把 write_ptr 之后补齐到 dh，保证无未初始化行。
 *
 * 峰值内存：32KB inflate 窗口 + 2 条源行（w*ch+1）+ 行映射表 h*2，
 * 3000x3000 RGBA ≈ 62KB PSRAM，与尺寸基本无关。
 */
#include "png_cover.h"

#include <string.h>
#include <stdlib.h>

#include <zlib.h>
#include <os/os.h>
#include <os/mem.h>
#include <components/log.h>

#define TAG "png_dec"
#define LOGI(...) BK_LOGI(TAG, ##__VA_ARGS__)
#define LOGW(...) BK_LOGW(TAG, ##__VA_ARGS__)
#define LOGE(...) BK_LOGE(TAG, ##__VA_ARGS__)

static const uint8_t PNG_SIG[8] = { 0x89, 'P', 'N', 'G', 0x0D, 0x0A, 0x1A, 0x0A };

static void *png_zalloc(void *opaque, unsigned items, unsigned size)
{
	(void)opaque;
	void *p = psram_malloc((uint32_t)items * size);

	if (!p)
		p = os_malloc((uint32_t)items * size);
	return p;
}

static void png_zfree(void *opaque, void *p)
{
	(void)opaque;
	os_free(p);   /* os/mem.h: psram_free == os_free */
}

static uint32_t rd32be(const uint8_t *p)
{
	return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) |
	       ((uint32_t)p[2] << 8) | p[3];
}

static int next_chunk(const uint8_t *data, uint32_t len, uint32_t *pos,
                      uint8_t type_out[5], const uint8_t **payload,
                      uint32_t *payload_len)
{
	uint32_t p = *pos;
	uint32_t clen;

	if (p + 8 > len)
		return 0;
	clen = rd32be(data + p);
	p += 4;
	if (p + 4 > len || clen > len - p - 4)
		return 0;
	memcpy(type_out, data + p, 4);
	type_out[4] = '\0';
	p += 4;
	*payload = data + p;
	*payload_len = clen;
	*pos = p + clen + 4;   /* 数据 + CRC（封面场景跳过校验） */
	return 1;
}

static inline int paeth(int a, int b, int c)
{
	int pa = abs(b - c);
	int pb = abs(a - c);
	int pc = abs(a + b - 2 * c);

	if (pa <= pb && pa <= pc)
		return a;
	if (pb <= pc)
		return b;
	return c;
}

static inline void pixel_rgb(const uint8_t *px, int idx, int ch, int ct,
                             const uint8_t *plte, uint32_t plte_len,
                             uint32_t *r, uint32_t *g, uint32_t *b)
{
	const uint8_t *p = px + (size_t)idx * ch;

	switch (ct) {
	case 0:
	case 4:
		*r = *g = *b = p[0];
		break;
	case 2:
		*r = p[0]; *g = p[1]; *b = p[2];
		break;
	case 3: {
		uint32_t i = p[0];

		if (i >= plte_len)
			i = 0;
		*r = plte[i * 3]; *g = plte[i * 3 + 1]; *b = plte[i * 3 + 2];
		break;
	}
	default:   /* 6 = RGBA；alpha 丢弃（封面按不透明处理） */
		*r = p[0]; *g = p[1]; *b = p[2];
		break;
	}
}

bool png_cover_decode(const uint8_t *data, uint32_t len,
                      uint16_t *dst, int dw, int dh)
{
	z_stream strm;
	uint8_t type[5];
	const uint8_t *payload = NULL;
	uint32_t payload_len = 0;
	uint32_t pos = 8;

	int w = 0, h = 0, ch = 0, ct = -1;
	const uint8_t *plte = NULL;
	uint32_t plte_len = 0;
	bool ihdr_ok = false, stream_end = false, inited = false;

	uint8_t *cur = NULL, *prev = NULL;
	uint16_t *rowmap = NULL;        /* src 行 -> dst 行 */
	int *sx0 = NULL, *sspan = NULL; /* dst 列 -> 源列起点/跨度 */
	uint32_t *acc = NULL;           /* dst 列 x RGB 累加器 */
	uint16_t *drow = NULL;          /* 最近物化的目标行 */

	int rows_done = 0;              /* 已解出的源行数 */
	int cur_dy = -1, rowlines = 0;  /* 当前累计的目标行 */
	int write_ptr = 0;              /* dst 已写到的行 */
	bool have_row = false, ok = false;

	if (!data || len < 8 || memcmp(data, PNG_SIG, 8) || !dst ||
	    dw <= 0 || dh <= 0)
		return false;

	memset(&strm, 0, sizeof(strm));
	strm.zalloc = png_zalloc;
	strm.zfree = png_zfree;
	if (inflateInit(&strm) != Z_OK)
		return false;
	inited = true;

	while (next_chunk(data, len, &pos, type, &payload, &payload_len)) {
		if (!ihdr_ok && memcmp(type, "IHDR", 4))
			goto fail;

		if (!memcmp(type, "IHDR", 4)) {
			int bd;

			if (payload_len != 13 || ihdr_ok)
				goto fail;
			w = (int)rd32be(payload);
			h = (int)rd32be(payload + 4);
			bd = payload[8];
			ct = payload[9];
			if (payload[10] != 0 || payload[11] != 0 ||
			    payload[12] != 0)     /* 压缩/滤波/交错法 */
				goto fail;
			if (w <= 0 || h <= 0 || w > 8192 || h > 8192 || bd != 8)
				goto fail;
			switch (ct) {
			case 0: case 3: ch = 1; break;
			case 4: ch = 2; break;
			case 2: ch = 3; break;
			case 6: ch = 4; break;
			default:
				goto fail;
			}
			ihdr_ok = true;

			cur = psram_malloc((uint32_t)w * ch + 1);
			prev = psram_malloc((uint32_t)w * ch + 1);
			rowmap = psram_malloc((uint32_t)h * sizeof(uint16_t));
			sx0 = os_malloc((uint32_t)dw * sizeof(int));
			sspan = os_malloc((uint32_t)dw * sizeof(int));
			acc = os_malloc((uint32_t)dw * 3 * sizeof(uint32_t));
			drow = os_malloc((uint32_t)dw * sizeof(uint16_t));
			if (!cur || !prev || !rowmap || !sx0 || !sspan ||
			    !acc || !drow)
				goto fail;

			/* 行映射（round）：h>dh 连续；h<dh 跳行由回填处理 */
			for (int y = 0; y < h; y++)
				rowmap[y] =
				  (uint16_t)(((int64_t)y * dh + dh / 2) / h);
			if (rowmap[h - 1] >= dh)
				rowmap[h - 1] = (uint16_t)(dh - 1);

			/* 列映射：dst 列覆盖 [sx0, sx0+span)，至少 1 列 */
			for (int dx = 0; dx < dw; dx++) {
				int s0 = (int)((int64_t)dx * w / dw);
				int s1 = (int)((int64_t)(dx + 1) * w / dw);

				if (s1 <= s0)
					s1 = s0 + 1;
				if (s0 >= w)
					s0 = w - 1;
				if (s1 > w)
					s1 = w;
				if (s1 <= s0)
					s1 = s0 + 1;
				sx0[dx] = s0;
				sspan[dx] = s1 - s0;
			}
			memset(prev, 0, (size_t)w * ch);
			continue;
		}

		if (!memcmp(type, "PLTE", 4)) {
			if (payload_len % 3 || payload_len > 256 * 3)
				goto fail;
			plte = payload;
			plte_len = payload_len / 3;
			continue;
		}

		if (!memcmp(type, "IEND", 4))
			break;

		if (memcmp(type, "IDAT", 4))
			continue;   /* 其它 chunk 跳过 */

		if (!ihdr_ok)
			goto fail;
		strm.next_in = (Bytef *)payload;
		strm.avail_in = payload_len;

		/* 尽量解出完整扫描线 */
		while (!stream_end && rows_done < h) {
			int need = w * ch + 1;
			int dy;
			uint8_t *px;
			int f, rowbytes;

			strm.next_out = cur;
			strm.avail_out = need;
			while (strm.avail_out) {
				int ret = inflate(&strm, Z_NO_FLUSH);

				if (ret == Z_STREAM_END) {
					stream_end = true;
					break;
				}
				if (ret != Z_OK) {
					/* 输入耗尽且无进展：等下一 IDAT */
					if (ret == Z_BUF_ERROR &&
					    strm.avail_in == 0)
						break;
					goto fail;
				}
			}
			if (strm.avail_out) {
				if (!stream_end)
					break;   /* 下一块 IDAT 续传 */
				memset(cur + (need - strm.avail_out), 0,
				       strm.avail_out);
			}

			/* ---- unfilter（引用上一行） ---- */
			f = cur[0];
			px = cur + 1;
			rowbytes = w * ch;
			for (int x = 0; x < rowbytes; x++) {
				int a = (x >= ch) ? px[x - ch] : 0;
				int b = prev[x];
				int c = (x >= ch) ? prev[x - ch] : 0;
				int v = px[x];

				switch (f) {
				case 1: v += a; break;
				case 2: v += b; break;
				case 3: v += (a + b) >> 1; break;
				case 4: v += paeth(a, b, c); break;
				default: break;   /* 0 = None */
				}
				px[x] = (uint8_t)v;
			}

			/* ---- 进入/继续目标行 ---- */
			dy = rowmap[rows_done];
			if (dy != cur_dy) {
				memset(acc, 0,
				       (uint32_t)dw * 3 * sizeof(uint32_t));
				rowlines = 0;
				cur_dy = dy;
			}

			/* ---- box 累加 ---- */
			for (int dx = 0; dx < dw; dx++) {
				int s = sx0[dx];
				uint32_t r = 0, g = 0, b = 0;

				for (int k = 0; k < sspan[dx]; k++, s++) {
					uint32_t rr, gg, bb;

					pixel_rgb(px, s, ch, ct, plte,
					          plte_len, &rr, &gg, &bb);
					r += rr; g += gg; b += bb;
				}
				acc[dx * 3] += r;
				acc[dx * 3 + 1] += g;
				acc[dx * 3 + 2] += b;
			}
			rowlines++;
			rows_done++;
			memcpy(prev, px, (size_t)rowbytes);

			/* ---- 该目标行覆盖的源行收齐 -> 物化 ---- */
			if (rows_done >= h || rowmap[rows_done] != dy) {
				for (int dx = 0; dx < dw; dx++) {
					uint32_t divv =
					  (uint32_t)rowlines *
					  (uint32_t)sspan[dx];
					uint32_t r = acc[dx * 3] / divv;
					uint32_t g = acc[dx * 3 + 1] / divv;
					uint32_t b = acc[dx * 3 + 2] / divv;

					if (r > 31) r = 31;
					if (g > 63) g = 63;
					if (b > 31) b = 31;
					drow[dx] =
					  (uint16_t)((r << 11) | (g << 5) | b);
				}
				/* 放大跳行：[write_ptr, dy) 用本行回填 */
				for (int r = write_ptr; r < dy; r++)
					memcpy(dst + (size_t)r * dw, drow,
					       (size_t)dw * 2);
				memcpy(dst + (size_t)dy * dw, drow,
				       (size_t)dw * 2);
				write_ptr = dy + 1;
				have_row = true;
				rowlines = 0;
				cur_dy = -1;
			}
		}
		if (rows_done >= h)
			break;
	}

	/* 收尾：尾部未覆盖的目标行用最后一行补齐 */
	if (have_row && write_ptr < dh) {
		for (int r = write_ptr; r < dh; r++)
			memcpy(dst + (size_t)r * dw, drow, (size_t)dw * 2);
		write_ptr = dh;
	}

	ok = (rows_done > 0);
	if (ok)
		LOGI("png %dx%d ch%d -> %dx%d done\r\n", w, h, ch, dw, dh);
	else
		LOGW("png: no rows decoded\r\n");
fail:
	if (inited)
		inflateEnd(&strm);
	os_free(cur);
	os_free(prev);
	os_free(rowmap);
	os_free(sx0);
	os_free(sspan);
	os_free(acc);
	os_free(drow);
	return ok;
}
