// BTL Photo Printer Bridge - your website (or any short text) in the corner of every print.
//
// White letters with a black outline and a soft gray edge, so it reads on light and dark photos.
// It's drawn into the rows as they stream past the encoder (or the label ditherer), at the
// bottom-right corner of the photo as you see it, whichever way the camera stored it.
//
// MIT License. Made for the BTL Photo Printer Bridge.
#pragma once
#include <stdint.h>
#include <stddef.h>

class BtlWatermark {
 public:
  // imgW x imgH: the rows that will be fed (as stored); orient: EXIF orientation of those rows.
  // Returns false (and draws nothing) if the text is empty or memory is short.
  // marginPct: gap from the edges as % of the short side (0 = right in the corner).
  bool prepare(const char* text, int imgW, int imgH, int orient, float marginPct = 0);
  // Draws onto 'rows' rows of packed 3-byte pixels starting at image row y0. The colors are
  // neutral (white, black, gray), so RGB vs BGR order doesn't matter.
  void apply(uint8_t* px, int y0, int rows) const;
  void end();
  ~BtlWatermark() { end(); }

 private:
  int iw = 0, ih = 0, orient = 1;
  int tw = 0, th = 0;                 // tile size (as seen upright)
  int dW = 0, dH = 0;                 // image size as seen upright
  int tx0 = 0, ty0 = 0;               // tile's top-left corner in the upright image
  int sx0 = 0, sy0 = 0, sx1 = 0, sy1 = 0;   // the tile's bounding box in stored coordinates
  uint8_t* col = nullptr;             // premultiplied gray level, per tile pixel
  uint8_t* alp = nullptr;             // coverage, per tile pixel
};
