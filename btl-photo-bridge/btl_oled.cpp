// BTL Photo Printer Bridge - small 128x64 OLED. See btl_oled.h.
// MIT License. Made for the BTL Photo Printer Bridge.
#include "btl_oled.h"
#include "btl_font.h"
#include <string.h>
#include <stdio.h>

// ------------------------------------------------------------------ drawing (host-testable)
static inline void px(uint8_t* fb, int x, int y, bool on) {
  if ((unsigned)x >= 128 || (unsigned)y >= 64) return;
  uint8_t& b = fb[(y >> 3) * 128 + x]; uint8_t m = 1 << (y & 7);
  if (on) b |= m; else b &= ~m;
}
static void fill(uint8_t* fb, int x0, int y0, int w, int h, bool on) {
  for (int y = y0; y < y0 + h; y++) for (int x = x0; x < x0 + w; x++) px(fb, x, y, on);
}
static int gIndex(char c) { int i = (uint8_t)c; if (i < BTL_FONT_FIRST || i > BTL_FONT_LAST) i = '?'; return i - BTL_FONT_FIRST; }
static int textW(const BtlFont& f, const char* s) { int w = 0; for (; *s; s++) w += f.glyphs[gIndex(*s)].adv; return w; }
// yTop = top of the font's ascent. Pixels with coverage >= half are lit (or cleared when on = false).
static void text(uint8_t* fb, const BtlFont& f, int x, int yTop, const char* s, bool on = true) {
  for (; *s; s++) {
    const BtlGlyph& g = f.glyphs[gIndex(*s)];
    const uint8_t* bits = f.bits + g.off; int bpr = (g.w + 1) / 2;
    for (int j = 0; j < g.h; j++)
      for (int i = 0; i < g.w; i++) {
        uint8_t v = bits[j * bpr + i / 2]; int c = (i & 1) ? (v & 15) : (v >> 4);
        if (c >= 8) px(fb, x + g.x + i, yTop + g.y + j, on);
      }
    x += g.adv;
  }
}
static void textC(uint8_t* fb, const BtlFont& f, int cx, int yTop, const char* s, bool on = true) {
  text(fb, f, cx - textW(f, s) / 2, yTop, s, on);
}
// Top of the digits sits this far below the top of the ascent (cap height ~ 0.73 of the font size).
static int digitTop(const BtlFont& f) { return f.glyphs[gIndex('8')].y; }

void oledRender(uint8_t* fb, const ScreenInfo& s, const char* note1, const char* note2, uint32_t ms) {
  memset(fb, 0, 1024);
  bool blink = (ms / 600) & 1;
  if (note1) {                                       // a short message, e.g. after a button press
    fill(fb, 0, 0, 128, 64, false);
    for (int x = 0; x < 128; x++) { px(fb, x, 0, true); px(fb, x, 63, true); }
    for (int y = 0; y < 64; y++) { px(fb, 0, y, true); px(fb, 127, y, true); }
    const BtlFont& big = textW(font_medium, note1) <= 120 ? font_medium : font_small;
    textC(fb, big, 64, note2 && note2[0] ? 8 : 18, note1);
    if (note2 && note2[0]) textC(fb, font_small, 64, 36, note2);
    return;
  }
  // status bar
  const char* st = "READY";
  switch (s.state) {
    case SCR_ST_OFFLINE: st = "NO PRINTER"; break;
    case SCR_ST_RECEIVING: st = "RECEIVING"; break;
    case SCR_ST_PRINTING: st = "PRINTING"; break;
    case SCR_ST_TEST: st = "TEST MODE"; break;
    case SCR_ST_STOPPED: st = s.why[0] ? s.why : "STOPPED"; break;
  }
  bool inv = s.state == SCR_ST_STOPPED;
  if (inv) fill(fb, 0, 0, 128, 15, true);
  char q[8] = "";
  if (s.queued) snprintf(q, sizeof(q), "+%u", (unsigned)s.queued);
  int qw = q[0] ? textW(font_small, q) + 4 : 0;
  if (qw && textW(font_small, st) > 126 - qw) { q[0] = 0; qw = 0; }   // the reason matters more than the count
  char line[32]; strncpy(line, st, sizeof(line) - 1); line[sizeof(line) - 1] = 0;
  while (line[0] && textW(font_small, line) > 126 - qw) line[strlen(line) - 1] = 0;
  text(fb, font_small, 1, -2, line, !inv);
  if (q[0]) text(fb, font_small, 127 - qw + 4, -2, q, !inv);
  if (!inv) for (int x = 0; x < 128; x++) px(fb, x, 15, true);
  // ink and paper
  text(fb, font_small, 2, 15, "INK");
  text(fb, font_small, 68, 15, "PAPER");
  char n[8];
  int ny = 37 - digitTop(font_large);               // digits from y = 37 down to ~62
  bool lowInk = s.inkLeft <= 5, lowPaper = s.paperLeft <= 2;
  snprintf(n, sizeof(n), "%d", (int)s.inkLeft);
  if (!(lowInk && blink)) text(fb, font_large, 2, ny, n);
  snprintf(n, sizeof(n), "%d", (int)s.paperLeft);
  if (!(lowPaper && blink)) text(fb, font_large, 68, ny, n);
  for (int y = 19; y < 64; y += 2) px(fb, 63, y, true);   // dotted divider
}

// ------------------------------------------------------------------ the display itself
#ifdef ARDUINO
#include <Arduino.h>
#include <Wire.h>

static uint8_t addr = 0;
static bool sh1106 = false;
static volatile uint32_t noteUntil = 0, lastWake = 0;
static char note1[32], note2[32];
static uint8_t fbuf[1024], sent[1024];
static portMUX_TYPE noteMux = portMUX_INITIALIZER_UNLOCKED;

static void cmd(uint8_t c) { Wire.beginTransmission(addr); Wire.write(0x00); Wire.write(c); Wire.endTransmission(); }
static void cmds(const uint8_t* c, size_t n) {
  Wire.beginTransmission(addr); Wire.write(0x00);
  for (size_t i = 0; i < n; i++) Wire.write(c[i]);
  Wire.endTransmission();
}
static void pushPage(int p, int x0, int x1) {      // send columns x0..x1 of page p
  uint8_t col = x0 + (sh1106 ? 2 : 0);
  const uint8_t a[] = { (uint8_t)(0xB0 | p), (uint8_t)(col & 15), (uint8_t)(0x10 | (col >> 4)) };
  cmds(a, 3);
  for (int x = x0; x <= x1; ) {
    Wire.beginTransmission(addr); Wire.write(0x40);
    for (int k = 0; k < 16 && x <= x1; k++, x++) Wire.write(fbuf[p * 128 + x]);
    Wire.endTransmission();
  }
}

static void oledTask(void*) {
  uint32_t dimmed = 0;
  for (;;) {
    ScreenInfo s; screenGetInfo(s);
    uint32_t now = millis();
    if (s.busy || s.state == SCR_ST_PRINTING || s.state == SCR_ST_STOPPED) lastWake = now;
    char n1[32], n2[32]; bool note;
    portENTER_CRITICAL(&noteMux);
    note = noteUntil && (int32_t)(noteUntil - now) > 0;
    if (note) { memcpy(n1, note1, 32); memcpy(n2, note2, 32); }
    portEXIT_CRITICAL(&noteMux);
    oledRender(fbuf, s, note ? n1 : nullptr, note ? n2 : nullptr, now);
    // send only the pages that changed
    for (int p = 0; p < 8; p++) {
      int x0 = -1, x1 = -1;
      for (int x = 0; x < 128; x++) if (fbuf[p * 128 + x] != sent[p * 128 + x]) { if (x0 < 0) x0 = x; x1 = x; }
      if (x0 >= 0) { pushPage(p, x0, x1); memcpy(sent + p * 128 + x0, fbuf + p * 128 + x0, x1 - x0 + 1); }
    }
    // dim after 10 idle minutes (OLEDs wear where the same picture stays lit)
    bool wantDim = now - lastWake > 600000;
    if (wantDim != (bool)dimmed) { cmd(0x81); cmd(wantDim ? 0x01 : 0xCF); dimmed = wantDim; }
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

static bool probe(uint8_t a) { Wire.beginTransmission(a); return Wire.endTransmission() == 0; }

bool oledBegin() {
  Wire.begin(OLED_PIN_SDA, OLED_PIN_SCL, 400000);
  if (probe(0x3C)) addr = 0x3C; else if (probe(0x3D)) addr = 0x3D; else { Wire.end(); return false; }
  // SH1106 (1.3") answers a status read with bit 3 clear on most modules; SSD1306 sets low bits 0x03/0x06.
  Wire.requestFrom((int)addr, 1);
  int st = Wire.available() ? Wire.read() : -1;
  sh1106 = st >= 0 && (st & 0x0F) == 0x08;
  static const uint8_t init[] = {
    0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0x8D, 0x14,   // off, clock, 64 rows, offset 0, start 0, charge pump on
    0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12, 0x81, 0xCF, 0xD9, 0xF1,   // page addressing, flip to match the board, contrast
    0xDB, 0x40, 0xA4, 0xA6, 0xAF };                               // VCOMH, show RAM, normal (not inverted), on
  cmds(init, sizeof(init));
  memset(sent, 0xFF, sizeof(sent));                   // force a full first draw
  lastWake = millis();
  xTaskCreatePinnedToCore(oledTask, "oled", 4096, nullptr, 1, nullptr, 0);
  return true;
}
bool oledPresent() { return addr != 0; }
void oledWake() { lastWake = millis(); }
void oledNote(const char* l1, const char* l2, uint32_t ms) {
  portENTER_CRITICAL(&noteMux);
  strncpy(note1, l1 ? l1 : "", 31); note1[31] = 0;
  strncpy(note2, l2 ? l2 : "", 31); note2[31] = 0;
  noteUntil = millis() + ms; if (!noteUntil) noteUntil = 1;
  portEXIT_CRITICAL(&noteMux);
  lastWake = millis();
}
#endif
