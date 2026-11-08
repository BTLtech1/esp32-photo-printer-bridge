// BTL Photo Printer Bridge - optional 2.8" touchscreen (ILI9341 + XPT2046). See btl_screen.h.
// MIT License. Made for the BTL Photo Printer Bridge.
#include "btl_screen.h"
#include "btl_font.h"
#include <SPI.h>
#include <Preferences.h>
#include <esp_heap_caps.h>
#include <driver/gpio.h>

#define W 320
#define H 240
#define SPI_HZ_LCD   40000000
#define SPI_HZ_READ   4000000
#define SPI_HZ_TOUCH  2000000
#define SLEEP_MS     (3 * 60 * 1000)     // backlight off after 3 minutes without touches or photos

// ---------------------------------------------------------------- colors (RGB565)
static constexpr uint16_t C(uint8_t r, uint8_t g, uint8_t b) { return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3); }
static const uint16_t BG = C(16, 16, 16), PANEL = C(32, 32, 32), LINE = C(60, 60, 60);
static const uint16_t TXT = C(235, 235, 235), MUT = C(150, 150, 150), DARK = C(20, 20, 20);
static const uint16_t AMBER = C(245, 180, 0), GREEN = C(40, 150, 85), BLUE = C(45, 108, 223);
static const uint16_t RED = C(200, 60, 50), PURPLE = C(130, 80, 200), BTN = C(50, 50, 50);

// ---------------------------------------------------------------- state
static SPIClass* spi = nullptr;
static uint16_t* fb = nullptr;                 // 320x240 frame, native RGB565, in PSRAM
static uint16_t lineBuf[W * 8];                 // byte-swapped chunk for SPI, internal RAM
static bool present = false;
static volatile bool wakeReq = false, calReq = false;
static bool blOn = true;
static uint32_t lastActive = 0;

// touch calibration: screen = (raw[axis] - r0) * span / (r1 - r0) + p0
struct Cal { uint8_t swap; int16_t x0, x1, y0, y1; };   // raw values at screen x=20/300 and y=20/220
static Cal cal = { 0, 0, 0, 0, 0 };
static bool calOk = false;

// thumbnail
#define TW 150
#define TH 100
static uint16_t* thPend = nullptr;              // being built (source orientation, up to TW x TW)
static uint16_t* thShow = nullptr;              // TW x TH, ready to show
static volatile uint32_t thSeq = 0;
static int thSrcW, thSrcH, thW, thH, thOrient;  // source size, unrotated thumb size
static float thScale = 0;
static int thLastRow = -1;

// ---------------------------------------------------------------- low level SPI
static inline void csLcd(bool on) { digitalWrite(SCR_PIN_CS, on ? LOW : HIGH); }
static inline void csTouch(bool on) { digitalWrite(SCR_PIN_TCS, on ? LOW : HIGH); }

static void lcdCmd(uint8_t c, const uint8_t* d = nullptr, int n = 0) {
  spi->beginTransaction(SPISettings(SPI_HZ_LCD, MSBFIRST, SPI_MODE0));
  csLcd(true);
  digitalWrite(SCR_PIN_DC, LOW); spi->transfer(c);
  digitalWrite(SCR_PIN_DC, HIGH);
  for (int i = 0; i < n; i++) spi->transfer(d[i]);
  csLcd(false);
  spi->endTransaction();
}

static uint32_t lcdReadId() {                    // ILI9341 "read ID4": dummy, 0x00, 0x93, 0x41
  spi->beginTransaction(SPISettings(SPI_HZ_READ, MSBFIRST, SPI_MODE0));
  csLcd(true);
  digitalWrite(SCR_PIN_DC, LOW); spi->transfer(0xD3);
  digitalWrite(SCR_PIN_DC, HIGH);
  uint32_t v = 0;
  for (int i = 0; i < 4; i++) v = (v << 8) | spi->transfer(0x00);
  csLcd(false);
  spi->endTransaction();
  return v & 0xFFFFFF;
}

static void lcdInit(uint8_t madctl) {
  digitalWrite(SCR_PIN_RST, HIGH); delay(5);
  digitalWrite(SCR_PIN_RST, LOW); delay(20);
  digitalWrite(SCR_PIN_RST, HIGH); delay(150);
  static const uint8_t seq[] = {
    0xEF, 3, 0x03, 0x80, 0x02,
    0xCF, 3, 0x00, 0xC1, 0x30,
    0xED, 4, 0x64, 0x03, 0x12, 0x81,
    0xE8, 3, 0x85, 0x00, 0x78,
    0xCB, 5, 0x39, 0x2C, 0x00, 0x34, 0x02,
    0xF7, 1, 0x20,
    0xEA, 2, 0x00, 0x00,
    0xC0, 1, 0x23,
    0xC1, 1, 0x10,
    0xC5, 2, 0x3E, 0x28,
    0xC7, 1, 0x86,
    0x37, 1, 0x00,
    0x3A, 1, 0x55,
    0xB1, 2, 0x00, 0x18,
    0xB6, 3, 0x08, 0x82, 0x27,
    0xF2, 1, 0x00,
    0x26, 1, 0x01,
    0xE0, 15, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00,
    0xE1, 15, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F,
    0x00
  };
  for (const uint8_t* p = seq; *p; ) { uint8_t c = *p++, n = *p++; lcdCmd(c, p, n); p += n; }
  lcdCmd(0x36, &madctl, 1);
  lcdCmd(0x11); delay(120);
  lcdCmd(0x29); delay(20);
}

// Sends the whole frame (about 60 ms at 40 MHz).
static void lcdPush() {
  uint8_t ca[4] = { 0, 0, (W - 1) >> 8, (W - 1) & 0xFF }, ra[4] = { 0, 0, (H - 1) >> 8, (H - 1) & 0xFF };
  lcdCmd(0x2A, ca, 4); lcdCmd(0x2B, ra, 4);
  spi->beginTransaction(SPISettings(SPI_HZ_LCD, MSBFIRST, SPI_MODE0));
  csLcd(true);
  digitalWrite(SCR_PIN_DC, LOW); spi->transfer(0x2C);
  digitalWrite(SCR_PIN_DC, HIGH);
  const int chunk = sizeof(lineBuf) / 2;
  for (int i = 0; i < W * H; i += chunk) {
    int n = min(chunk, W * H - i);
    for (int k = 0; k < n; k++) { uint16_t v = fb[i + k]; lineBuf[k] = (v >> 8) | (v << 8); }
    spi->writeBytes((const uint8_t*)lineBuf, n * 2);
  }
  csLcd(false);
  spi->endTransaction();
}

static void backlight(bool on) {
  if (on == blOn) return;
  blOn = on;
  digitalWrite(SCR_PIN_BL, on ? HIGH : LOW);
}

// ---------------------------------------------------------------- touch (XPT2046)
static uint16_t tRead(uint8_t cmd) {
  spi->transfer(cmd);
  uint16_t hi = spi->transfer(0), lo = spi->transfer(0);
  return ((hi << 8) | lo) >> 3;
}
static bool touchDown() { return digitalRead(SCR_PIN_TIRQ) == LOW; }

// Raw reading, median of 5. Returns false if the pen lifted during the read.
static bool touchRaw(int& rx, int& ry) {
  if (!touchDown()) return false;
  int xs[5], ys[5];
  spi->beginTransaction(SPISettings(SPI_HZ_TOUCH, MSBFIRST, SPI_MODE0));
  csTouch(true);
  tRead(0xD0);                                   // first reading after a switch is noisy
  for (int i = 0; i < 5; i++) { xs[i] = tRead(0xD0); ys[i] = tRead(0x90); }
  tRead(0x80); spi->transfer(0); spi->transfer(0); // power down, keep the pen interrupt on
  csTouch(false);
  spi->endTransaction();
  if (!touchDown()) return false;
  for (int i = 1; i < 5; i++) for (int k = i; k > 0 && xs[k] < xs[k - 1]; k--) { int t = xs[k]; xs[k] = xs[k - 1]; xs[k - 1] = t; }
  for (int i = 1; i < 5; i++) for (int k = i; k > 0 && ys[k] < ys[k - 1]; k--) { int t = ys[k]; ys[k] = ys[k - 1]; ys[k - 1] = t; }
  rx = xs[2]; ry = ys[2];
  return rx > 50 && rx < 4050 && ry > 50 && ry < 4050;
}

static bool touchXY(int& x, int& y) {
  int rx, ry;
  if (!touchRaw(rx, ry) || !calOk) return false;
  int a = cal.swap ? ry : rx, b = cal.swap ? rx : ry;
  x = 20 + (int)((long)(a - cal.x0) * 280 / (cal.x1 - cal.x0));
  y = 20 + (int)((long)(b - cal.y0) * 200 / (cal.y1 - cal.y0));
  x = constrain(x, 0, W - 1); y = constrain(y, 0, H - 1);
  return true;
}

// ---------------------------------------------------------------- drawing
static void fill(int x, int y, int w, int h, uint16_t c) {
  if (x < 0) { w += x; x = 0; } if (y < 0) { h += y; y = 0; }
  if (x + w > W) w = W - x; if (y + h > H) h = H - y;
  for (int j = 0; j < h; j++) { uint16_t* p = fb + (y + j) * W + x; for (int i = 0; i < w; i++) p[i] = c; }
}

static void rfill(int x, int y, int w, int h, int r, uint16_t c) {
  for (int j = 0; j < h; j++) {
    int in = 0;
    int dy = j < r ? r - j : (j >= h - r ? j - (h - r - 1) : 0);
    if (dy) { float d = r - sqrtf((float)(r * r) - (float)(dy - 0.5f) * (dy - 0.5f)); in = (int)(d + 0.5f); }
    fill(x + in, y + j, w - 2 * in, 1, c);
  }
}

static inline uint16_t blend(uint16_t bg, uint16_t fg, int a) {     // a: 0..15
  if (a >= 15) return fg; if (a <= 0) return bg;
  int r = (((bg >> 11) & 31) * (15 - a) + ((fg >> 11) & 31) * a) / 15;
  int g = (((bg >> 5) & 63) * (15 - a) + ((fg >> 5) & 63) * a) / 15;
  int b = ((bg & 31) * (15 - a) + (fg & 31) * a) / 15;
  return (r << 11) | (g << 5) | b;
}

static int textW(const BtlFont& f, const char* s) {
  int w = 0;
  for (; *s; s++) { int c = (uint8_t)*s; if (c < BTL_FONT_FIRST || c > BTL_FONT_LAST) c = '?'; w += f.glyphs[c - BTL_FONT_FIRST].adv; }
  return w;
}

// Draws text with its top (ascent line) at y. maxW clips with "..." when > 0.
static int text(const BtlFont& f, int x, int y, const char* s, uint16_t col, int maxW = 0) {
  char buf[64];
  if (maxW > 0 && textW(f, s) > maxW) {
    int n = 0, dots = textW(f, "...");
    int w = 0;
    for (; s[n] && n < 56; n++) {
      int c = (uint8_t)s[n]; if (c < BTL_FONT_FIRST || c > BTL_FONT_LAST) c = '?';
      if (w + f.glyphs[c - BTL_FONT_FIRST].adv + dots > maxW) break;
      w += f.glyphs[c - BTL_FONT_FIRST].adv;
    }
    memcpy(buf, s, n); strcpy(buf + n, "..."); s = buf;
  }
  for (; *s; s++) {
    int c = (uint8_t)*s; if (c < BTL_FONT_FIRST || c > BTL_FONT_LAST) c = '?';
    const BtlGlyph& g = f.glyphs[c - BTL_FONT_FIRST];
    const uint8_t* bits = f.bits + g.off;
    int bpr = (g.w + 1) / 2;
    for (int j = 0; j < g.h; j++) {
      int py = y + g.y + j; if (py < 0 || py >= H) continue;
      for (int i = 0; i < g.w; i++) {
        int px = x + g.x + i; if (px < 0 || px >= W) continue;
        uint8_t v = bits[j * bpr + i / 2];
        int a = (i & 1) ? (v & 15) : (v >> 4);
        if (a) { uint16_t* p = fb + py * W + px; *p = blend(*p, col, a); }
      }
    }
    x += g.adv;
  }
  return x;
}
static void textC(const BtlFont& f, int cx, int y, const char* s, uint16_t col) { text(f, cx - textW(f, s) / 2, y, s, col); }
static void textR(const BtlFont& f, int rx, int y, const char* s, uint16_t col) { text(f, rx - textW(f, s), y, s, col); }
// Vertically centers a line of text in a box of height h starting at y.
static int midY(const BtlFont& f, int y, int h) { return y + (h - f.ascent) / 2 - 1; }

// ---------------------------------------------------------------- buttons
struct Btn { int16_t x, y, w, h; uint8_t id; };
static Btn btns[16]; static int nBtn = 0;
static void button(int x, int y, int w, int h, uint8_t id, const char* label, uint16_t bg, uint16_t fg, const BtlFont& f = font_medium) {
  rfill(x, y, w, h, 8, bg);
  if (label[0] && !label[1]) {                     // single symbol (+, -): center its actual shape
    const BtlGlyph& g = f.glyphs[(uint8_t)label[0] - BTL_FONT_FIRST];
    text(f, x + (w - g.w) / 2 - g.x, y + (h - g.h) / 2 - g.y, label, fg);
  } else textC(f, x + w / 2, midY(f, y, h), label, fg);
  if (nBtn < 16) btns[nBtn++] = { (int16_t)x, (int16_t)y, (int16_t)w, (int16_t)h, id };
}
static int hit(int x, int y) {
  for (int i = 0; i < nBtn; i++) {
    const Btn& b = btns[i];
    if (x >= b.x - 4 && x < b.x + b.w + 4 && y >= b.y - 4 && y < b.y + b.h + 4) return b.id;
  }
  return 0;
}

// ---------------------------------------------------------------- pages
enum { PG_MAIN, PG_SET, PG_CAL };
enum { B_SETTINGS = 1, B_DONE, B_WARM_M, B_WARM_P, B_CON_M, B_CON_P, B_SAT_M, B_SAT_P, B_INK, B_PAPER, B_CAL, B_DEST_M, B_DEST_P, B_WM };
static int page = PG_MAIN;
static int confirmId = 0; static uint32_t confirmUntil = 0;

static const char* warmText(int v, char* b) {
  if (v == 0) return "Neutral";
  sprintf(b, v < 0 ? "Cooler %d" : "Warmer %d", v < 0 ? -v : v); return b;
}
static const char* stepText(int v, char* b) { if (!v) return "As shot"; sprintf(b, "+%d", v); return b; }
static const char* destText(int d) { return d == 2 ? "Label" : d == 3 ? "Both" : "SELPHY"; }

static void bar(int x, int y, int w, int left, int cap, bool warnLow) {
  rfill(x, y, w, 8, 4, LINE);
  if (cap > 0 && left > 0) {
    int fw = (int)((long)w * min(left, cap) / cap); if (fw < 8) fw = 8;
    rfill(x, y, fw, 8, 4, warnLow ? AMBER : GREEN);
  }
}

static void drawMain(const ScreenInfo& s) {
  fill(0, 0, W, H, BG);
  // header
  text(font_small, 10, midY(font_small, 0, 28), "BTL Photo Bridge", AMBER);
  const char* lk = s.link == 2 ? "USB" : s.link == 1 ? "Wi-Fi" : "No printer";
  uint16_t lc = s.link == 2 ? GREEN : s.link == 1 ? BLUE : RED;
  int lw = textW(font_small, lk);
  rfill(W - 10 - lw - 20, 7, 10, 10, 5, lc);
  textR(font_small, W - 10, midY(font_small, 0, 28), lk, TXT);
  // state banner
  const char* st; uint16_t sb, sf = TXT; char qb[24];
  switch (s.state) {
    case SCR_ST_READY:     st = "READY"; sb = GREEN; break;
    case SCR_ST_RECEIVING: st = "RECEIVING"; sb = BLUE; break;
    case SCR_ST_PRINTING:  st = "PRINTING"; sb = AMBER; sf = DARK; break;
    case SCR_ST_TEST:      st = "TEST MODE"; sb = PURPLE; break;
    case SCR_ST_STOPPED:   st = "CHECK PRINTER"; sb = RED; break;
    default:               st = "NO PRINTER"; sb = RED; break;
  }
  if (s.queued > 0 && s.state != SCR_ST_OFFLINE) { sprintf(qb, "%s +%d", st, s.queued); st = qb; }
  rfill(8, 30, W - 16, 46, 10, sb);
  textC(font_large, W / 2, midY(font_large, 30, 46), st, sf);
  // thumbnail
  int tx = 8, ty = 84;
  fill(tx - 1, ty - 1, TW + 2, TH + 2, LINE);
  if (thSeq && thShow) {
    for (int j = 0; j < TH; j++) memcpy(fb + (ty + j) * W + tx, thShow + j * TW, TW * 2);
  } else {
    fill(tx, ty, TW, TH, PANEL);
    textC(font_small, tx + TW / 2, ty + TH / 2 - 8, "No photo yet", MUT);
  }
  // ink and paper
  int px = 172, pw = W - px - 10; char b[24];
  text(font_small, px, 86, "Ink", MUT);
  sprintf(b, "%d left", s.inkLeft); textR(font_medium, px + pw, 82, b, s.inkLeft <= 5 ? AMBER : TXT);
  bar(px, 108, pw, s.inkLeft, s.inkCap, s.inkLeft <= 5);
  text(font_small, px, 132, "Paper", MUT);
  sprintf(b, "%d left", s.paperLeft); textR(font_medium, px + pw, 128, b, s.paperLeft <= 3 ? AMBER : TXT);
  bar(px, 154, pw, s.paperLeft, s.paperCap, s.paperLeft <= 3);
  // last photo
  if (s.lastName[0]) {
    char l[96]; snprintf(l, sizeof(l), "%s  %s", s.lastName, s.lastInfo);
    text(font_small, 8, 190, l, MUT, W - 16);
  }
  // settings button with a summary of the color settings
  nBtn = 0;
  char sum[64], w1[16], c1[16], s1[16];
  snprintf(sum, sizeof(sum), "%s  |  %s  |  C %s  |  Col %s", destText(s.dest), warmText(s.warm, w1), stepText(s.con, c1), stepText(s.sat, s1));
  rfill(8, 208, W - 16, 28, 8, BTN);
  text(font_small, 18, midY(font_small, 208, 28), sum, TXT, W - 60);
  textR(font_small, W - 18, midY(font_small, 208, 28), ">", AMBER);
  btns[nBtn++] = { 8, 208, W - 16, 28, B_SETTINGS };
}

static void stepper(int y, const char* label, const char* val, uint8_t idM, uint8_t idP) {
  text(font_medium, 10, midY(font_medium, y, 34), label, TXT);
  button(118, y, 44, 34, idM, "-", BTN, TXT, font_large);
  textC(font_medium, 118 + 44 + 51, midY(font_medium, y, 34), val, AMBER);
  button(W - 10 - 44, y, 44, 34, idP, "+", BTN, TXT, font_large);
}

static void drawSettings(const ScreenInfo& s) {
  fill(0, 0, W, H, BG);
  nBtn = 0;
  char b[16];
  stepper(4, "Print on", destText(s.dest), B_DEST_M, B_DEST_P);
  stepper(42, "Warmth", warmText(s.warm, b), B_WARM_M, B_WARM_P);
  stepper(80, "Contrast", stepText(s.con, b), B_CON_M, B_CON_P);
  stepper(118, "Color", stepText(s.sat, b), B_SAT_M, B_SAT_P);
  bool ci = confirmId == B_INK && millis() < confirmUntil, cp = confirmId == B_PAPER && millis() < confirmUntil;
  button(8, 160, 148, 34, B_INK, ci ? "Tap to confirm" : "New ink", ci ? AMBER : BTN, ci ? DARK : TXT, font_small);
  button(164, 160, 148, 34, B_PAPER, cp ? "Tap to confirm" : "Paper loaded", cp ? AMBER : BTN, cp ? DARK : TXT, font_small);
  button(8, 200, 92, 36, B_CAL, "Touch setup", PANEL, MUT, font_small);
  button(106, 200, 108, 36, B_WM, s.wm == 1 ? "Website: on" : s.wm == 0 ? "Website: off" : "No website", s.wm == 1 ? GREEN : BTN, s.wm == 2 ? MUT : TXT, font_small);
  button(220, 200, W - 228, 36, B_DONE, "Done", AMBER, DARK);
}

// Calibration: tap three crosshairs.
static const int calPts[3][2] = { { 20, 20 }, { 300, 20 }, { 20, 220 } };
static int calStep = 0, calRaw[3][2];
static void drawCal() {
  fill(0, 0, W, H, BG);
  nBtn = 0;
  textC(font_medium, W / 2, 90, "Touch setup", AMBER);
  textC(font_small, W / 2, 122, "Tap the center of the cross", TXT);
  textC(font_small, W / 2, 142, "with a fingernail or stylus.", MUT);
  int x = calPts[calStep][0], y = calPts[calStep][1];
  fill(x - 12, y - 1, 25, 3, TXT); fill(x - 1, y - 12, 3, 25, TXT);
}
static void calFinish() {
  int dxX = abs(calRaw[1][0] - calRaw[0][0]), dxY = abs(calRaw[1][1] - calRaw[0][1]);
  cal.swap = dxY > dxX ? 1 : 0;
  int ai = cal.swap ? 1 : 0, bi = cal.swap ? 0 : 1;
  cal.x0 = calRaw[0][ai]; cal.x1 = calRaw[1][ai];
  cal.y0 = (calRaw[0][bi] + calRaw[1][bi]) / 2; cal.y1 = calRaw[2][bi];
  calOk = abs(cal.x1 - cal.x0) > 300 && abs(cal.y1 - cal.y0) > 300;
  if (calOk) { Preferences p; p.begin("btlscr", false); p.putBytes("cal", &cal, sizeof(cal)); p.end(); }
}

// ---------------------------------------------------------------- task
static void setVal(int act, int v) { screenAction(act, v); }

static void onTap(int id, const ScreenInfo& s) {
  switch (id) {
    case B_SETTINGS: page = PG_SET; break;
    case B_DONE:     page = PG_MAIN; break;
    case B_WARM_M:   if (s.warm > -5) setVal(SCR_ACT_WARM, s.warm - 1); break;
    case B_WARM_P:   if (s.warm < 5) setVal(SCR_ACT_WARM, s.warm + 1); break;
    case B_CON_M:    if (s.con > 0) setVal(SCR_ACT_CON, s.con - 1); break;
    case B_CON_P:    if (s.con < 3) setVal(SCR_ACT_CON, s.con + 1); break;
    case B_SAT_M:    if (s.sat > 0) setVal(SCR_ACT_SAT, s.sat - 1); break;
    case B_SAT_P:    if (s.sat < 3) setVal(SCR_ACT_SAT, s.sat + 1); break;
    case B_INK: case B_PAPER:
      if (confirmId == id && millis() < confirmUntil) {
        setVal(id == B_INK ? SCR_ACT_NEW_INK : SCR_ACT_PAPER_LOADED, 0);
        confirmId = 0;
      } else { confirmId = id; confirmUntil = millis() + 3000; }
      break;
    case B_CAL: page = PG_CAL; calStep = 0; break;
    case B_WM:       if (s.wm != 2) setVal(SCR_ACT_WM, 0); break;
    case B_DEST_M:   setVal(SCR_ACT_DEST, s.dest <= 1 ? 3 : s.dest - 1); break;
    case B_DEST_P:   setVal(SCR_ACT_DEST, s.dest >= 3 ? 1 : s.dest + 1); break;
  }
}

static void screenTask(void*) {
  ScreenInfo cur = {}, last = {};
  uint32_t lastSeq = 0, lastDraw = 0;
  int lastPage = -1; bool dirty = true, wasDown = false, lastConfirm = false;
  lastActive = millis();
  if (!calOk) page = PG_CAL;
  while (true) {
    screenGetInfo(cur);
    bool changed = memcmp(&cur, &last, sizeof(cur)) != 0;
    bool newPhoto = thSeq != lastSeq;
    if (newPhoto || (changed && cur.state != last.state) || wakeReq) { lastActive = millis(); wakeReq = false; backlight(true); }
    if (calReq) { calReq = false; page = PG_CAL; calStep = 0; backlight(true); lastActive = millis(); }
    if (changed || newPhoto || page != lastPage) dirty = true;
    bool conf = confirmId && millis() < confirmUntil;
    if (conf != lastConfirm) { dirty = true; lastConfirm = conf; }

    // touch
    bool down = touchDown();
    if (down && !wasDown) {
      delay(15);                                         // let it settle
      if (!blOn) { backlight(true); lastActive = millis(); }
      else if (page == PG_CAL) {
        int rx, ry;
        if (touchRaw(rx, ry)) {
          calRaw[calStep][0] = rx; calRaw[calStep][1] = ry;
          if (++calStep == 3) { calFinish(); page = calOk ? PG_MAIN : PG_CAL; calStep = 0; }
          dirty = true;
        }
      } else {
        int x, y;
        if (touchXY(x, y)) { int id = hit(x, y); if (id) { onTap(id, cur); dirty = true; } }
      }
      lastActive = millis();
      wasDown = true;
    } else if (!down) wasDown = false;
    if (down) lastActive = millis();

    if (blOn && millis() - lastActive > SLEEP_MS && page != PG_CAL) { backlight(false); page = PG_MAIN; }

    if (dirty && millis() - lastDraw > (cur.busy && page == PG_MAIN ? 1000u : 120u)) {
      if (page == PG_MAIN) drawMain(cur); else if (page == PG_SET) drawSettings(cur); else drawCal();
      lcdPush();
      last = cur; lastSeq = thSeq; lastPage = page; dirty = false; lastDraw = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(down ? 10 : 40));
  }
}

// ---------------------------------------------------------------- public
bool screenPresent() { return present; }
void screenWake() { wakeReq = true; }
void screenRecalibrate() { calReq = true; }

bool screenBegin() {
  pinMode(SCR_PIN_CS, OUTPUT); digitalWrite(SCR_PIN_CS, HIGH);
  pinMode(SCR_PIN_TCS, OUTPUT); digitalWrite(SCR_PIN_TCS, HIGH);
  pinMode(SCR_PIN_DC, OUTPUT); pinMode(SCR_PIN_RST, OUTPUT);
  pinMode(SCR_PIN_BL, OUTPUT); digitalWrite(SCR_PIN_BL, LOW);
  pinMode(SCR_PIN_TIRQ, INPUT_PULLUP);
  pinMode(SCR_PIN_MISO, INPUT_PULLDOWN);              // no screen: reads come back as zeros
  spi = new SPIClass(FSPI);
  spi->begin(SCR_PIN_SCK, SCR_PIN_MISO, SCR_PIN_MOSI, -1);
  gpio_pullup_dis((gpio_num_t)SCR_PIN_MISO); gpio_pulldown_en((gpio_num_t)SCR_PIN_MISO);   // after begin(), which resets the pin
  lcdInit(0x28);                                      // landscape
  uint32_t id = lcdReadId();
  if ((id & 0xFFFF) != 0x9341) {                       // some clones don't answer the ID read: try the status read
    lcdCmd(0x00);
    spi->beginTransaction(SPISettings(SPI_HZ_READ, MSBFIRST, SPI_MODE0));
    csLcd(true); digitalWrite(SCR_PIN_DC, LOW); spi->transfer(0x0A); digitalWrite(SCR_PIN_DC, HIGH);
    spi->transfer(0); uint8_t pm = spi->transfer(0);   // power mode: 0x9C when on and awake
    csLcd(false); spi->endTransaction();
    // expect booster on, awake, normal mode, display on
    if (pm == 0xFF || (pm & 0x9C) != 0x9C) { spi->end(); delete spi; spi = nullptr; present = false; return false; }
  }
  fb = (uint16_t*)heap_caps_malloc(W * H * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  thPend = (uint16_t*)heap_caps_malloc(TW * TW * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  thShow = (uint16_t*)heap_caps_malloc(TW * TH * 2, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!fb || !thPend || !thShow) { present = false; return false; }
  Preferences p; p.begin("btlscr", true);
  calOk = p.getBytes("cal", &cal, sizeof(cal)) == sizeof(cal) && abs(cal.x1 - cal.x0) > 300 && abs(cal.y1 - cal.y0) > 300;
  p.end();
  fill(0, 0, W, H, BG); lcdPush();
  backlight(false); blOn = false; backlight(true);
  present = true;
  xTaskCreatePinnedToCore(screenTask, "screen", 6144, nullptr, 1, nullptr, 0);
  return true;
}

// ---------------------------------------------------------------- thumbnail
void screenThumbBegin(int w, int h, int orient) {
  thLastRow = -1;
  if (!present || w < 1 || h < 1) { thScale = 0; return; }
  bool rot = orient == 6 || orient == 8;
  int dw = rot ? h : w, dh = rot ? w : h;               // as shown
  thScale = min((float)TW / dw, (float)TH / dh);
  thSrcW = w; thSrcH = h; thOrient = orient;
  thW = max(1, min(TW, (int)lroundf(w * thScale))); thH = max(1, min(TW, (int)lroundf(h * thScale)));
}

void screenThumbRows(const uint8_t* px, int y0, int rows, bool bgr) {
  if (thScale <= 0 || !thPend) return;
  int ri = bgr ? 2 : 0, bi = bgr ? 0 : 2;
  for (int ty = 0; ty < thH; ty++) {
    int sy = (int)((ty + 0.5f) * thSrcH / thH);
    if (sy < y0 || sy >= y0 + rows) continue;
    const uint8_t* row = px + (size_t)(sy - y0) * thSrcW * 3;
    uint16_t* out = thPend + ty * thW;
    for (int tx = 0; tx < thW; tx++) {
      int sx = (int)((tx + 0.5f) * thSrcW / thW);
      const uint8_t* p = row + sx * 3;
      out[tx] = C(p[ri], p[1], p[bi]);
    }
  }
}

void screenThumbEnd() {
  if (thScale <= 0 || !thShow) return;
  bool rot = thOrient == 6 || thOrient == 8;
  int dw = rot ? thH : thW, dh = rot ? thW : thH;
  int ox = (TW - dw) / 2, oy = (TH - dh) / 2;
  for (int i = 0; i < TW * TH; i++) thShow[i] = PANEL;
  for (int y = 0; y < dh; y++)
    for (int x = 0; x < dw; x++) {
      int sx, sy;                                       // source pixel for shown pixel (x, y)
      switch (thOrient) {
        case 3:  sx = thW - 1 - x; sy = thH - 1 - y; break;
        case 6:  sx = y;           sy = thH - 1 - x; break;   // rotate 90 clockwise
        case 8:  sx = thW - 1 - y; sy = x;           break;   // rotate 90 counter-clockwise
        default: sx = x;           sy = y;
      }
      thShow[(oy + y) * TW + ox + x] = thPend[sy * thW + sx];
    }
  thScale = 0;
  thSeq++;
  wakeReq = true;
}
