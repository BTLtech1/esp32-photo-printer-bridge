// BTL Photo Printer Bridge - photos on thermal label and receipt printers. See btl_thermal.h.
// MIT License. Made for the BTL Photo Printer Bridge.
#include "btl_thermal.h"
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#ifdef ARDUINO
#include <esp_heap_caps.h>
static void* tAlloc(size_t n) {
  void* p = heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  return p ? p : malloc(n);
}
#else
static void* tAlloc(size_t n) { return malloc(n); }
#endif

bool BtlThermal::begin(int srcW, int srcH, int orient, const ThermalCfg& c) {
  end();
  cfg = c; sw = srcW; sh = srcH; y = 0;
  if (sw < 1 || sh < 1 || cfg.dotsW < 8) return false;

  // How the source stands upright (d = displayed), then whether it goes sideways on the label.
  // Each step is an affine map with -1/0/1 coefficients; compose them.
  int dW = sw, dH = sh;
  int a1 = 1, b1 = 0, c1 = 0, a2 = 0, b2 = 1, c2 = 0;                    // dx = a1 x + b1 y + c1, dy = a2 x + b2 y + c2
  switch (orient) {
    case 3: a1 = -1; c1 = sw - 1; b2 = -1; c2 = sh - 1; break;
    case 6: a1 = 0; b1 = -1; c1 = sh - 1; a2 = 1; b2 = 0; c2 = 0; dW = sh; dH = sw; break;
    case 8: a1 = 0; b1 = 1; c1 = 0; a2 = -1; b2 = 0; c2 = sw - 1; dW = sh; dH = sw; break;
  }
  bool side = dW > dH;                                                      // landscape: long side along the paper
  int mw = side ? dH : dW, mh = side ? dW : dH;                             // size on the label
  // label from displayed: side ? (lx = dH-1-dy, ly = dx) : (lx = dx, ly = dy)
  int p1, q1, r1, p2, q2, r2;                                              // lx = p1 dx + q1 dy + r1, ly = p2 dx + q2 dy + r2
  if (side) { p1 = 0; q1 = -1; r1 = dH - 1; p2 = 1; q2 = 0; r2 = 0; }
  else      { p1 = 1; q1 = 0; r1 = 0; p2 = 0; q2 = 1; r2 = 0; }

  bw = (cfg.dotsW + 7) & ~7; bpr = bw / 8;
  bh = cfg.lang == THERM_ESCPOS ? mh : cfg.dotsH;
  int ox = (bw - mw) / 2, oy = (bh - mh) / 2;
  ax = p1 * a1 + q1 * a2; bx = p1 * b1 + q1 * b2; cx = p1 * c1 + q1 * c2 + r1 + ox;
  ay = p2 * a1 + q2 * a2; by = p2 * b1 + q2 * b2; cy = p2 * c1 + q2 * c2 + r2 + oy;

  bits = (uint8_t*)tAlloc((size_t)bpr * bh);
  for (int i = 0; i < 3; i++) err[i] = (int32_t*)tAlloc((size_t)(sw + 4) * sizeof(int32_t));
  if (!bits || !err[0] || !err[1] || !err[2]) { end(); return false; }
  memset(bits, 0, (size_t)bpr * bh);
  for (int i = 0; i < 3; i++) memset(err[i], 0, (size_t)(sw + 4) * sizeof(int32_t));

  // Tone: thermal dots spread a little, so prints come out darker than the screen. Lift the
  // midtones, stretch the ends slightly, then the user's lighter/darker step.
  float g = 0.80f * powf(1.18f, (float)cfg.tone);
  for (int v = 0; v < 256; v++) {
    float x = (v - 8) / 239.0f; if (x < 0) x = 0; if (x > 1) x = 1;
    x = x - 0.35f * sinf(6.2831853f * x) / 6.2831853f;     // a little more contrast: dot patterns flatten midtones
    lut[v] = (uint8_t)lroundf(255.0f * powf(x, g));
  }
  return true;
}

void BtlThermal::toneRows(uint8_t* p, size_t pixels, bool bgr) {
  const int ri = bgr ? 2 : 0, bi = bgr ? 0 : 2;
  for (size_t i = 0; i < pixels; i++, p += 3) {
    uint8_t v = lut[(77 * p[ri] + 150 * p[1] + 29 * p[bi]) >> 8];
    p[0] = p[1] = p[2] = v;
  }
}

void BtlThermal::addRows(const uint8_t* px, int rows) {
  if (!bits) return;
  for (int r = 0; r < rows && y < sh; r++, y++) {
    const uint8_t* row = px + (size_t)r * sw * 3;
    int32_t* e0 = err[0] + 2; int32_t* e1 = err[1] + 2; int32_t* e2 = err[2] + 2;
    if (cfg.dither == THERM_DITHER_RETRO) {
      // Atkinson: passes on 6/8 of the error, which keeps highlights and shadows clean (the classic Mac look)
      for (int x = 0; x < sw; x++) {
        int v = row[x * 3] + (e0[x] >> 3);
        int o = v >= 128 ? 255 : 0, e = v - o;
        if (!o) dot(x, y);
        e0[x + 1] += e; e0[x + 2] += e; e1[x - 1] += e; e1[x] += e; e1[x + 1] += e; e2[x] += e;
      }
    } else {
      // Floyd-Steinberg, alternating direction each row (avoids diagonal "worms")
      bool rev = y & 1;
      for (int i = 0; i < sw; i++) {
        int x = rev ? sw - 1 - i : i, s = rev ? -1 : 1;
        int v = row[x * 3] + (e0[x] >> 4);
        int o = v >= 128 ? 255 : 0, e = v - o;
        if (!o) dot(x, y);
        e0[x + s] += e * 7; e1[x - s] += e * 3; e1[x] += e * 5; e1[x + s] += e;
      }
    }
    // rotate the error rows: next becomes current, the oldest is cleared for reuse
    int32_t* t = err[0]; err[0] = err[1]; err[1] = err[2]; err[2] = t;
    memset(err[2], 0, (size_t)(sw + 4) * sizeof(int32_t));
  }
}

size_t BtlThermal::jobSize() const {
  size_t n = (size_t)bpr * bh;
  switch (cfg.lang) {
    case THERM_ZPL:    return n * 2 + 128;
    case THERM_ESCPOS: return n + (size_t)(bh / 128 + 1) * 8 + 32;
    default:           return n + 160;
  }
}

size_t BtlThermal::build(uint8_t* out, size_t cap) const {
  if (!bits || cap < jobSize()) return 0;
  size_t n = (size_t)bpr * bh, o = 0;
  auto put = [&](const char* s) { size_t k = strlen(s); memcpy(out + o, s, k); o += k; };
  char h[160];
  if (cfg.lang == THERM_ZPL) {
    snprintf(h, sizeof(h), "^XA^PW%d^LL%d^FO0,0^GFA,%u,%u,%d,", bw, bh, (unsigned)n, (unsigned)n, bpr);
    put(h);
    static const char hex[] = "0123456789ABCDEF";
    for (size_t i = 0; i < n; i++) { out[o++] = hex[bits[i] >> 4]; out[o++] = hex[bits[i] & 15]; }
    put("^FS^XZ");
  } else if (cfg.lang == THERM_ESCPOS) {
    put("\x1B\x40");                                     // reset
    for (int y0 = 0; y0 < bh; y0 += 128) {               // raster image in bands (some printers limit the height)
      int rows = bh - y0 < 128 ? bh - y0 : 128;
      uint8_t g[8] = { 0x1D, 0x76, 0x30, 0x00, (uint8_t)(bpr & 255), (uint8_t)(bpr >> 8), (uint8_t)(rows & 255), (uint8_t)(rows >> 8) };
      memcpy(out + o, g, 8); o += 8;
      memcpy(out + o, bits + (size_t)y0 * bpr, (size_t)rows * bpr); o += (size_t)rows * bpr;
    }
    put("\x1B\x64\x04");                                 // feed 4 lines so the photo clears the tear bar
    put("\x1D\x56\x42\x00");                             // cut (ignored by printers without a cutter)
  } else {
    snprintf(h, sizeof(h), "SIZE %.1f mm,%.1f mm\r\nGAP %.1f mm,0 mm\r\nCLS\r\nBITMAP 0,0,%d,%d,0,",
             cfg.widthMm, cfg.heightMm, cfg.gapMm, bpr, bh);
    put(h);
    for (size_t i = 0; i < n; i++) out[o++] = (uint8_t)~bits[i];   // TSPL: 1 = no dot
    put("\r\nPRINT 1,1\r\n");
  }
  return o;
}

void BtlThermal::end() {
  free(bits); bits = nullptr;
  for (int i = 0; i < 3; i++) { free(err[i]); err[i] = nullptr; }
}
