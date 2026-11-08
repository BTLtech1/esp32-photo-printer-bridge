// BTL Photo Printer Bridge - website text in the corner of prints. See btl_watermark.h.
// MIT License. Made for the BTL Photo Printer Bridge.
#include "btl_watermark.h"
#include "btl_wmfont.h"
#include <stdlib.h>
#include <string.h>
#include <math.h>
#ifdef ARDUINO
#include <esp_heap_caps.h>
static void* wAlloc(size_t n) { void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); return p ? p : malloc(n); }
#else
static void* wAlloc(size_t n) { return malloc(n); }
#endif

static int glyphIndex(char ch) { int c = (uint8_t)ch; if (c < BTL_FONT_FIRST || c > BTL_FONT_LAST) c = '?'; return c - BTL_FONT_FIRST; }

// Max filter with a round kernel: grows the letters' shape by r pixels (for the outline rings).
static void dilate(const uint8_t* in, uint8_t* out, int w, int h, int r) {
  for (int y = 0; y < h; y++)
    for (int x = 0; x < w; x++) {
      int m = 0;
      for (int dy = -r; dy <= r && m < 255; dy++) {
        int yy = y + dy; if (yy < 0 || yy >= h) continue;
        int span = (int)sqrtf((float)(r * r - dy * dy));
        const uint8_t* row = in + yy * w;
        for (int dx = -span; dx <= span; dx++) { int xx = x + dx; if (xx >= 0 && xx < w && row[xx] > m) m = row[xx]; }
      }
      out[y * w + x] = (uint8_t)m;
    }
}

bool BtlWatermark::prepare(const char* text, int imgW, int imgH, int o, float marginPct) {
  end();
  if (!text || !text[0] || imgW < 64 || imgH < 64) return false;
  iw = imgW; ih = imgH; orient = o;
  bool rot = o == 6 || o == 8;
  dW = rot ? ih : iw; dH = rot ? iw : ih;

  // 1. The text at the font's size (48 px) as a coverage mask, with room for the outline.
  const BtlFont& f = font_wm;
  const int ring1 = 3, ring2 = 6, pad = ring2 + 1;
  int textW = 0;
  for (const char* p = text; *p; p++) textW += f.glyphs[glyphIndex(*p)].adv;
  int mw = textW + 2 * pad, mh = f.lineH + 2 * pad;
  uint8_t* a = (uint8_t*)wAlloc((size_t)mw * mh * 3);
  if (!a) return false;
  uint8_t* b = a + mw * mh; uint8_t* g = b + mw * mh;
  memset(a, 0, (size_t)mw * mh);
  int x = pad;
  for (const char* p = text; *p; p++) {
    const BtlGlyph& gl = f.glyphs[glyphIndex(*p)];
    const uint8_t* bits = f.bits + gl.off;
    int bpr = (gl.w + 1) / 2;
    for (int j = 0; j < gl.h; j++)
      for (int i = 0; i < gl.w; i++) {
        int px = x + gl.x + i, py = pad + gl.y + j;
        if (px < 0 || px >= mw || py < 0 || py >= mh) continue;
        uint8_t v = bits[j * bpr + i / 2];
        int c = (i & 1) ? (v & 15) : (v >> 4);
        a[py * mw + px] = (uint8_t)(c * 17);
      }
    x += gl.adv;
  }
  // 2. Outline rings: black hugging the letters, gray around that.
  dilate(a, b, mw, mh, ring1);
  dilate(a, g, mw, mh, ring2);

  // 3. Scale to the print: the letters about 3.7% of the photo's short side tall.
  float s = (float)(dW < dH ? dW : dH) * 0.037f / 48.0f;
  tw = (int)lroundf(mw * s); th = (int)lroundf(mh * s);
  if (tw < 8 || th < 4 || tw > dW / 2) { free(a); return false; }
  col = (uint8_t*)wAlloc((size_t)tw * th);
  alp = (uint8_t*)wAlloc((size_t)tw * th);
  if (!col || !alp) { free(a); end(); return false; }
  for (int y = 0; y < th; y++) {
    float fy = (y + 0.5f) / s - 0.5f; int y0 = (int)floorf(fy); float wy = fy - y0;
    for (int xx = 0; xx < tw; xx++) {
      float fx = (xx + 0.5f) / s - 0.5f; int x0 = (int)floorf(fx); float wx = fx - x0;
      float A = 0, B = 0, G = 0;                               // bilinear sample of the three masks
      for (int k = 0; k < 4; k++) {
        int sx = x0 + (k & 1), sy = y0 + (k >> 1);
        if (sx < 0 || sx >= mw || sy < 0 || sy >= mh) continue;
        float w = ((k & 1) ? wx : 1 - wx) * ((k >> 1) ? wy : 1 - wy);
        A += w * a[sy * mw + sx]; B += w * b[sy * mw + sx]; G += w * g[sy * mw + sx];
      }
      A /= 255; B /= 255; G /= 255;
      // stack the layers: gray edge (50% opaque), then black, then white letters
      float c = 0, al = 0;
      float ga = G * 0.5f; c = 128 * ga; al = ga;
      c = c * (1 - B); al = B + al * (1 - B);
      c = 255 * A + c * (1 - A); al = A + al * (1 - A);
      col[y * tw + xx] = (uint8_t)lroundf(c);
      alp[y * tw + xx] = (uint8_t)lroundf(al * 255);
    }
  }
  free(a);

  // 4. Where it goes: bottom-right of the upright photo, with a margin.
  int m = (int)lroundf((dW < dH ? dW : dH) * marginPct / 100.0f);   // the outline's own padding keeps it off the very edge
  tx0 = dW - m - tw; ty0 = dH - m - th;
  // its box in stored coordinates (map the four corners back)
  sx0 = iw; sy0 = ih; sx1 = -1; sy1 = -1;
  for (int k = 0; k < 4; k++) {
    int dx = tx0 + ((k & 1) ? tw - 1 : 0), dy = ty0 + ((k >> 1) ? th - 1 : 0), sx, sy;
    switch (orient) {                                          // inverse of stored -> upright
      case 3:  sx = iw - 1 - dx; sy = ih - 1 - dy; break;
      case 6:  sx = dy;          sy = ih - 1 - dx; break;      // upright (dx, dy) = (ih-1-sy, sx)
      case 8:  sx = iw - 1 - dy; sy = dx;          break;      // upright (dx, dy) = (sy, iw-1-sx)
      default: sx = dx;          sy = dy;
    }
    if (sx < sx0) sx0 = sx; if (sx > sx1) sx1 = sx; if (sy < sy0) sy0 = sy; if (sy > sy1) sy1 = sy;
  }
  return true;
}

void BtlWatermark::apply(uint8_t* px, int y0, int rows) const {
  if (!col) return;
  int ya = y0 > sy0 ? y0 : sy0, yb = (y0 + rows - 1) < sy1 ? (y0 + rows - 1) : sy1;
  for (int sy = ya; sy <= yb; sy++) {
    uint8_t* row = px + (size_t)(sy - y0) * iw * 3;
    for (int sx = sx0; sx <= sx1; sx++) {
      int dx, dy;
      switch (orient) {
        case 3:  dx = iw - 1 - sx; dy = ih - 1 - sy; break;
        case 6:  dx = ih - 1 - sy; dy = sx;          break;
        case 8:  dx = sy;          dy = iw - 1 - sx; break;
        default: dx = sx;          dy = sy;
      }
      int tx = dx - tx0, ty = dy - ty0;
      if ((unsigned)tx >= (unsigned)tw || (unsigned)ty >= (unsigned)th) continue;
      int al = alp[ty * tw + tx];
      if (!al) continue;
      int c = col[ty * tw + tx], inv = 255 - al;
      uint8_t* p = row + sx * 3;
      p[0] = (uint8_t)(c + (p[0] * inv + 127) / 255);
      p[1] = (uint8_t)(c + (p[1] * inv + 127) / 255);
      p[2] = (uint8_t)(c + (p[2] * inv + 127) / 255);
    }
  }
}

void BtlWatermark::end() { free(col); free(alp); col = alp = nullptr; }
