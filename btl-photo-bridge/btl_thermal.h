// BTL Photo Printer Bridge - photos on thermal label and receipt printers.
//
// Takes the resized photo row by row, turns it into black dots (dithering: grays become dot
// patterns), places it on the label (turned sideways if that fits better) and wraps it in the
// printer's language: TSPL or ZPL for label printers, ESC/POS for receipt printers.
//
// MIT License. Made for the BTL Photo Printer Bridge.
#pragma once
#include <stdint.h>
#include <stddef.h>

enum { THERM_TSPL = 0, THERM_ZPL = 1, THERM_ESCPOS = 2 };
enum { THERM_DITHER_SMOOTH = 0, THERM_DITHER_RETRO = 1 };

struct ThermalCfg {
  uint8_t  lang = THERM_TSPL;
  uint8_t  dither = THERM_DITHER_SMOOTH;
  int8_t   tone = 0;            // -3 lighter .. +3 darker
  uint16_t dotsW = 816;         // across the print head (a multiple of 8)
  uint16_t dotsH = 1218;        // along the paper (labels; receipts follow the photo)
  float    widthMm = 101.6f, heightMm = 152.4f, gapMm = 3.0f;   // for TSPL's SIZE / GAP
};

class BtlThermal {
 public:
  // srcW x srcH = the rows that will be fed in; orient = how they need turning to stand upright
  // (EXIF: 1 as is, 3 = 180, 6 = 90 clockwise, 8 = 90 counter-clockwise).
  bool begin(int srcW, int srcH, int orient, const ThermalCfg& c);
  // Turns pixels into the toned gray the printer will get, in place (R = G = B), so a preview
  // made from them shows what prints. Call before addRows.
  void toneRows(uint8_t* px, size_t pixels, bool bgr);
  // Dithers 'rows' toned rows (3 bytes per pixel, srcW pixels each) onto the label.
  void addRows(const uint8_t* px, int rows);
  // Size of the finished printer job, and building it into 'out'. Returns bytes written (0 = no room).
  size_t jobSize() const;
  size_t build(uint8_t* out, size_t cap) const;
  void end();
  int bitmapW() const { return bw; }
  int bitmapH() const { return bh; }

 private:
  ThermalCfg cfg;
  int sw = 0, sh = 0, y = 0;
  int bw = 0, bh = 0, bpr = 0;          // label bitmap, 1 = black
  uint8_t* bits = nullptr;
  int32_t* err[3] = { nullptr, nullptr, nullptr };   // error rows (current and the next two), x offset by 2
  uint8_t lut[256];
  // source (x, y) -> label (lx, ly):  lx = ax*x + bx*y + cx,  ly = ay*x + by*y + cy
  int ax = 1, bx = 0, cx = 0, ay = 0, by = 1, cy = 0;
  inline void dot(int x, int yy) {
    int lx = ax * x + bx * yy + cx, ly = ay * x + by * yy + cy;
    if ((unsigned)lx < (unsigned)bw && (unsigned)ly < (unsigned)bh) bits[ly * bpr + (lx >> 3)] |= 0x80 >> (lx & 7);
  }
};
