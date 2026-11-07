// BTL Photo Printer Bridge - a small streaming baseline JPEG encoder.
//
// Takes RGB rows in bands (16 rows at a time) and emits the JPEG bytes as it goes, so the
// printer can start reading the file while the rest is still being made. 4:2:0 subsampling,
// standard Huffman tables, quality 1-100 scaled like libjpeg.
//
// MIT License. Made for the BTL Photo Printer Bridge.
#pragma once
#include <stdint.h>
#include <stddef.h>

class BtlJpegEnc {
 public:
  typedef void (*OutFn)(void* ctx, const uint8_t* data, size_t len);

  // Starts a new image. Writes the headers through 'out'. Returns false on bad arguments.
  bool begin(int width, int height, int quality, OutFn out, void* ctx);
  // Adds 'rows' rows of packed RGB (3 bytes per pixel, width pixels each). Rows are taken in
  // groups of 16 (the last group may be shorter); any left over waits for the next call.
  void addRows(const uint8_t* rgb, int rows);
  // Finishes the image (pads the last band, writes the trailer). Returns the total bytes written.
  size_t end();

 private:
  int w = 0, h = 0, rowsDone = 0;
  OutFn out = nullptr; void* ctx = nullptr;
  size_t total = 0;
  // output bit buffer
  uint8_t obuf[4096]; size_t olen = 0;
  uint32_t bitBuf = 0; int bitCnt = 0;
  // tables
  uint8_t qY[64], qC[64];              // quantization tables in zigzag order (as written to the file)
  int32_t qrY[64], qrC[64];            // reciprocals for the quantizer, natural order
  uint16_t ehufY_dc[12], ehufC_dc[12]; uint8_t ehufY_dcL[12], ehufC_dcL[12];
  uint16_t ehufY_ac[256], ehufC_ac[256]; uint8_t ehufY_acL[256], ehufC_acL[256];
  int dcY = 0, dcCb = 0, dcCr = 0;
  // band assembly (16 rows of RGB)
  uint8_t* band = nullptr; int bandRows = 0;
  int16_t blk[64];

  void flush();
  void putByte(uint8_t b) { obuf[olen++] = b; if (olen == sizeof(obuf)) flush(); }
  void putBits(uint32_t code, int len);
  void putMarker(uint8_t m) { putByte(0xFF); putByte(m); }
  void put16(uint16_t v) { putByte(v >> 8); putByte(v & 0xFF); }
  void writeHeaders();
  void encodeBand(const uint8_t* rgb, int rows);
  void encodeBlock(const int16_t* in, const int32_t* qr, int& dcPrev,
                   const uint16_t* dcCode, const uint8_t* dcLen, const uint16_t* acCode, const uint8_t* acLen);
  static void fdct(int16_t* data);
  static void buildHuff(const uint8_t* bits, const uint8_t* vals, uint16_t* code, uint8_t* len, int n);
};
