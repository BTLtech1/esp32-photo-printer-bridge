// BTL Photo Printer Bridge - streaming baseline JPEG encoder (see btl_jpeg_enc.h).
#include "btl_jpeg_enc.h"
#include <string.h>
#include <stdlib.h>

// ---- standard tables (ITU T.81 Annex K)
static const uint8_t kZigzag[64] = {
  0,1,8,16,9,2,3,10,17,24,32,25,18,11,4,5,12,19,26,33,40,48,41,34,27,20,13,6,7,14,21,28,
  35,42,49,56,57,50,43,36,29,22,15,23,30,37,44,51,58,59,52,45,38,31,39,46,53,60,61,54,47,55,62,63 };
static const uint8_t kQLum[64] = {
  16,11,10,16,24,40,51,61, 12,12,14,19,26,58,60,55, 14,13,16,24,40,57,69,56, 14,17,22,29,51,87,80,62,
  18,22,37,56,68,109,103,77, 24,35,55,64,81,104,113,92, 49,64,78,87,103,121,120,101, 72,92,95,98,112,100,103,99 };
static const uint8_t kQChr[64] = {
  17,18,24,47,99,99,99,99, 18,21,26,66,99,99,99,99, 24,26,56,99,99,99,99,99, 47,66,99,99,99,99,99,99,
  99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99, 99,99,99,99,99,99,99,99 };
static const uint8_t kDcLumBits[16] = {0,1,5,1,1,1,1,1,1,0,0,0,0,0,0,0};
static const uint8_t kDcLumVals[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t kDcChrBits[16] = {0,3,1,1,1,1,1,1,1,1,1,0,0,0,0,0};
static const uint8_t kDcChrVals[12] = {0,1,2,3,4,5,6,7,8,9,10,11};
static const uint8_t kAcLumBits[16] = {0,2,1,3,3,2,4,3,5,5,4,4,0,0,1,0x7d};
static const uint8_t kAcLumVals[162] = {
  0x01,0x02,0x03,0x00,0x04,0x11,0x05,0x12,0x21,0x31,0x41,0x06,0x13,0x51,0x61,0x07,0x22,0x71,0x14,0x32,0x81,0x91,0xa1,0x08,
  0x23,0x42,0xb1,0xc1,0x15,0x52,0xd1,0xf0,0x24,0x33,0x62,0x72,0x82,0x09,0x0a,0x16,0x17,0x18,0x19,0x1a,0x25,0x26,0x27,0x28,
  0x29,0x2a,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,0x59,
  0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x83,0x84,0x85,0x86,0x87,0x88,0x89,
  0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,0xb5,0xb6,
  0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xe1,0xe2,
  0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa };
static const uint8_t kAcChrBits[16] = {0,2,1,2,4,4,3,4,7,5,4,4,0,1,2,0x77};
static const uint8_t kAcChrVals[162] = {
  0x00,0x01,0x02,0x03,0x11,0x04,0x05,0x21,0x31,0x06,0x12,0x41,0x51,0x07,0x61,0x71,0x13,0x22,0x32,0x81,0x08,0x14,0x42,0x91,
  0xa1,0xb1,0xc1,0x09,0x23,0x33,0x52,0xf0,0x15,0x62,0x72,0xd1,0x0a,0x16,0x24,0x34,0xe1,0x25,0xf1,0x17,0x18,0x19,0x1a,0x26,
  0x27,0x28,0x29,0x2a,0x35,0x36,0x37,0x38,0x39,0x3a,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x53,0x54,0x55,0x56,0x57,0x58,
  0x59,0x5a,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x82,0x83,0x84,0x85,0x86,0x87,
  0x88,0x89,0x8a,0x92,0x93,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xb2,0xb3,0xb4,
  0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,
  0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa };

void BtlJpegEnc::buildHuff(const uint8_t* bits, const uint8_t* vals, uint16_t* code, uint8_t* len, int n) {
  memset(len, 0, n);
  uint16_t c = 0; int k = 0;
  for (int l = 1; l <= 16; l++) {
    for (int i = 0; i < bits[l - 1]; i++) { code[vals[k]] = c++; len[vals[k]] = l; k++; }
    c <<= 1;
  }
}

bool BtlJpegEnc::begin(int width, int height, int quality, OutFn outFn, void* c) {
  if (width < 1 || height < 1 || width > 16384 || height > 16384 || !outFn) return false;
  w = width; h = height; rowsDone = 0; out = outFn; ctx = c; total = 0; olen = 0; bitBuf = 0; bitCnt = 0;
  dcY = dcCb = dcCr = 0;
  free(band); band = (uint8_t*)malloc((size_t)w * 16 * 3); bandRows = 0;
  if (!band) return false;
  if (quality < 1) quality = 1; if (quality > 100) quality = 100;
  int scale = quality < 50 ? 5000 / quality : 200 - quality * 2;
  for (int i = 0; i < 64; i++) {
    int y = (kQLum[i] * scale + 50) / 100, cc = (kQChr[i] * scale + 50) / 100;
    if (y < 1) y = 1; if (y > 255) y = 255; if (cc < 1) cc = 1; if (cc > 255) cc = 255;
    qrY[i] = (1 << 16) / (y * 8);                       // the DCT below is scaled by 8
    qrC[i] = (1 << 16) / (cc * 8);
  }
  for (int z = 0; z < 64; z++) {                        // the file stores the tables in zigzag order
    int i = kZigzag[z];
    int y = (kQLum[i] * scale + 50) / 100, cc = (kQChr[i] * scale + 50) / 100;
    if (y < 1) y = 1; if (y > 255) y = 255; if (cc < 1) cc = 1; if (cc > 255) cc = 255;
    qY[z] = (uint8_t)y; qC[z] = (uint8_t)cc;
  }
  buildHuff(kDcLumBits, kDcLumVals, ehufY_dc, ehufY_dcL, 12);
  buildHuff(kDcChrBits, kDcChrVals, ehufC_dc, ehufC_dcL, 12);
  buildHuff(kAcLumBits, kAcLumVals, ehufY_ac, ehufY_acL, 256);
  buildHuff(kAcChrBits, kAcChrVals, ehufC_ac, ehufC_acL, 256);
  writeHeaders();
  return true;
}

void BtlJpegEnc::flush() {
  if (olen) { out(ctx, obuf, olen); total += olen; olen = 0; }
}

void BtlJpegEnc::putBits(uint32_t code, int len) {
  bitBuf = (bitBuf << len) | (code & ((1u << len) - 1));
  bitCnt += len;
  while (bitCnt >= 8) {
    uint8_t b = (uint8_t)(bitBuf >> (bitCnt - 8));
    putByte(b);
    if (b == 0xFF) putByte(0x00);                       // byte stuffing
    bitCnt -= 8;
  }
}

void BtlJpegEnc::writeHeaders() {
  putMarker(0xD8);                                      // SOI
  putMarker(0xE0); put16(16);                           // APP0 / JFIF
  putByte('J'); putByte('F'); putByte('I'); putByte('F'); putByte(0);
  putByte(1); putByte(1); putByte(1); put16(300); put16(300);   // version 1.1, dots per inch, 300 x 300
  putByte(0); putByte(0);
  putMarker(0xDB); put16(2 + 65 * 2);                   // DQT: two 8-bit tables
  putByte(0x00); for (int i = 0; i < 64; i++) putByte(qY[i]);
  putByte(0x01); for (int i = 0; i < 64; i++) putByte(qC[i]);
  putMarker(0xC0); put16(17);                           // SOF0 baseline, 3 components, 4:2:0
  putByte(8); put16((uint16_t)h); put16((uint16_t)w); putByte(3);
  putByte(1); putByte(0x22); putByte(0);
  putByte(2); putByte(0x11); putByte(1);
  putByte(3); putByte(0x11); putByte(1);
  const uint8_t* bits[4] = { kDcLumBits, kAcLumBits, kDcChrBits, kAcChrBits };
  const uint8_t* vals[4] = { kDcLumVals, kAcLumVals, kDcChrVals, kAcChrVals };
  const uint8_t ids[4] = { 0x00, 0x10, 0x01, 0x11 };
  const int cnt[4] = { 12, 162, 12, 162 };
  for (int t = 0; t < 4; t++) {
    putMarker(0xC4); put16(2 + 1 + 16 + cnt[t]);
    putByte(ids[t]);
    for (int i = 0; i < 16; i++) putByte(bits[t][i]);
    for (int i = 0; i < cnt[t]; i++) putByte(vals[t][i]);
  }
  putMarker(0xDA); put16(12);                           // SOS
  putByte(3); putByte(1); putByte(0x00); putByte(2); putByte(0x11); putByte(3); putByte(0x11);
  putByte(0); putByte(63); putByte(0);
}

// Forward DCT, integer, accurate (the "slow integer" method from the IJG library). Output is
// scaled up by 8 compared to the true DCT.
#define CONST_BITS 13
#define PASS1_BITS 2
#define FIX_0_298631336 2446
#define FIX_0_390180644 3196
#define FIX_0_541196100 4433
#define FIX_0_765366865 6270
#define FIX_0_899976223 7373
#define FIX_1_175875602 9633
#define FIX_1_501321110 12299
#define FIX_1_847759065 15137
#define FIX_1_961570560 16069
#define FIX_2_053119869 16819
#define FIX_2_562915447 20995
#define FIX_3_072711026 25172
#define DESCALE(x, n) (((x) + (1 << ((n) - 1))) >> (n))

__attribute__((optimize("O2")))
void BtlJpegEnc::fdct(int16_t* data) {
  int32_t tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, tmp6, tmp7, tmp10, tmp11, tmp12, tmp13, z1, z2, z3, z4, z5;
  int16_t* d = data;
  for (int r = 0; r < 8; r++, d += 8) {                 // rows
    tmp0 = d[0] + d[7]; tmp7 = d[0] - d[7]; tmp1 = d[1] + d[6]; tmp6 = d[1] - d[6];
    tmp2 = d[2] + d[5]; tmp5 = d[2] - d[5]; tmp3 = d[3] + d[4]; tmp4 = d[3] - d[4];
    tmp10 = tmp0 + tmp3; tmp13 = tmp0 - tmp3; tmp11 = tmp1 + tmp2; tmp12 = tmp1 - tmp2;
    d[0] = (int16_t)((tmp10 + tmp11) << PASS1_BITS);
    d[4] = (int16_t)((tmp10 - tmp11) << PASS1_BITS);
    z1 = (tmp12 + tmp13) * FIX_0_541196100;
    d[2] = (int16_t)DESCALE(z1 + tmp13 * FIX_0_765366865, CONST_BITS - PASS1_BITS);
    d[6] = (int16_t)DESCALE(z1 + tmp12 * (-FIX_1_847759065), CONST_BITS - PASS1_BITS);
    z1 = tmp4 + tmp7; z2 = tmp5 + tmp6; z3 = tmp4 + tmp6; z4 = tmp5 + tmp7;
    z5 = (z3 + z4) * FIX_1_175875602;
    tmp4 *= FIX_0_298631336; tmp5 *= FIX_2_053119869; tmp6 *= FIX_3_072711026; tmp7 *= FIX_1_501321110;
    z1 *= -FIX_0_899976223; z2 *= -FIX_2_562915447; z3 *= -FIX_1_961570560; z4 *= -FIX_0_390180644;
    z3 += z5; z4 += z5;
    d[7] = (int16_t)DESCALE(tmp4 + z1 + z3, CONST_BITS - PASS1_BITS);
    d[5] = (int16_t)DESCALE(tmp5 + z2 + z4, CONST_BITS - PASS1_BITS);
    d[3] = (int16_t)DESCALE(tmp6 + z2 + z3, CONST_BITS - PASS1_BITS);
    d[1] = (int16_t)DESCALE(tmp7 + z1 + z4, CONST_BITS - PASS1_BITS);
  }
  d = data;
  for (int c = 0; c < 8; c++, d++) {                    // columns
    tmp0 = d[0] + d[56]; tmp7 = d[0] - d[56]; tmp1 = d[8] + d[48]; tmp6 = d[8] - d[48];
    tmp2 = d[16] + d[40]; tmp5 = d[16] - d[40]; tmp3 = d[24] + d[32]; tmp4 = d[24] - d[32];
    tmp10 = tmp0 + tmp3; tmp13 = tmp0 - tmp3; tmp11 = tmp1 + tmp2; tmp12 = tmp1 - tmp2;
    d[0] = (int16_t)DESCALE(tmp10 + tmp11, PASS1_BITS);
    d[32] = (int16_t)DESCALE(tmp10 - tmp11, PASS1_BITS);
    z1 = (tmp12 + tmp13) * FIX_0_541196100;
    d[16] = (int16_t)DESCALE(z1 + tmp13 * FIX_0_765366865, CONST_BITS + PASS1_BITS);
    d[48] = (int16_t)DESCALE(z1 + tmp12 * (-FIX_1_847759065), CONST_BITS + PASS1_BITS);
    z1 = tmp4 + tmp7; z2 = tmp5 + tmp6; z3 = tmp4 + tmp6; z4 = tmp5 + tmp7;
    z5 = (z3 + z4) * FIX_1_175875602;
    tmp4 *= FIX_0_298631336; tmp5 *= FIX_2_053119869; tmp6 *= FIX_3_072711026; tmp7 *= FIX_1_501321110;
    z1 *= -FIX_0_899976223; z2 *= -FIX_2_562915447; z3 *= -FIX_1_961570560; z4 *= -FIX_0_390180644;
    z3 += z5; z4 += z5;
    d[56] = (int16_t)DESCALE(tmp4 + z1 + z3, CONST_BITS + PASS1_BITS);
    d[40] = (int16_t)DESCALE(tmp5 + z2 + z4, CONST_BITS + PASS1_BITS);
    d[24] = (int16_t)DESCALE(tmp6 + z2 + z3, CONST_BITS + PASS1_BITS);
    d[8] = (int16_t)DESCALE(tmp7 + z1 + z4, CONST_BITS + PASS1_BITS);
  }
}

static inline int nbits(int v) { int n = 0; while (v) { n++; v >>= 1; } return n; }

__attribute__((optimize("O2")))
void BtlJpegEnc::encodeBlock(const int16_t* in, const int32_t* qr, int& dcPrev,
                             const uint16_t* dcCode, const uint8_t* dcLen, const uint16_t* acCode, const uint8_t* acLen) {
  int16_t coef[64];
  memcpy(coef, in, sizeof(coef));
  fdct(coef);
  int q[64];
  for (int i = 0; i < 64; i++) {                        // quantize (natural order)
    int32_t c = coef[i];
    q[i] = c >= 0 ? (int)((c * qr[i] + 32768) >> 16) : -(int)(((-c) * qr[i] + 32768) >> 16);
  }
  int diff = q[0] - dcPrev; dcPrev = q[0];
  int cat = nbits(diff < 0 ? -diff : diff);
  putBits(dcCode[cat], dcLen[cat]);
  if (cat) putBits((uint32_t)(diff < 0 ? diff - 1 : diff), cat);
  int run = 0;
  for (int z = 1; z < 64; z++) {
    int v = q[kZigzag[z]];
    if (v == 0) { run++; continue; }
    while (run > 15) { putBits(acCode[0xF0], acLen[0xF0]); run -= 16; }
    int ac = nbits(v < 0 ? -v : v);
    int sym = (run << 4) | ac;
    putBits(acCode[sym], acLen[sym]);
    putBits((uint32_t)(v < 0 ? v - 1 : v), ac);
    run = 0;
  }
  if (run) putBits(acCode[0], acLen[0]);               // EOB
}

__attribute__((optimize("O2")))
void BtlJpegEnc::encodeBand(const uint8_t* rgb, int rows) {
  // one band = one row of 16x16 macroblocks: 4 Y blocks + Cb + Cr each
  int16_t Y[4][64], Cb[64], Cr[64];
  int cbSum[64], crSum[64];
  for (int mx = 0; mx < w; mx += 16) {
    memset(cbSum, 0, sizeof(cbSum)); memset(crSum, 0, sizeof(crSum));
    for (int y = 0; y < 16; y++) {
      int sy = y < rows ? y : rows - 1;                 // past the bottom: repeat the last row
      const uint8_t* row = rgb + (size_t)sy * w * 3;
      for (int x = 0; x < 16; x++) {
        int sx = mx + x; if (sx >= w) sx = w - 1;       // past the right edge: repeat the last column
        const uint8_t* p = row + (size_t)sx * 3;
        int r = p[0], g = p[1], b = p[2];
        int yy = (19595 * r + 38470 * g + 7471 * b + 32768) >> 16;
        int cb = ((-11059 * r - 21709 * g + 32768 * b + 32768) >> 16);
        int cr = ((32768 * r - 27439 * g - 5329 * b + 32768) >> 16);
        Y[(y >> 3) * 2 + (x >> 3)][(y & 7) * 8 + (x & 7)] = (int16_t)(yy - 128);
        int ci = (y >> 1) * 8 + (x >> 1);
        cbSum[ci] += cb; crSum[ci] += cr;
      }
    }
    for (int i = 0; i < 64; i++) { Cb[i] = (int16_t)((cbSum[i] + 2) >> 2); Cr[i] = (int16_t)((crSum[i] + 2) >> 2); }
    for (int k = 0; k < 4; k++) encodeBlock(Y[k], qrY, dcY, ehufY_dc, ehufY_dcL, ehufY_ac, ehufY_acL);
    encodeBlock(Cb, qrC, dcCb, ehufC_dc, ehufC_dcL, ehufC_ac, ehufC_acL);
    encodeBlock(Cr, qrC, dcCr, ehufC_dc, ehufC_dcL, ehufC_ac, ehufC_acL);
  }
}

void BtlJpegEnc::addRows(const uint8_t* rgb, int rows) {
  while (rows > 0 && rowsDone < h) {
    if (bandRows == 0 && rows >= 16) {                  // a whole band straight from the caller
      encodeBand(rgb, 16);
      rgb += (size_t)16 * w * 3; rows -= 16; rowsDone += 16;
      continue;
    }
    int take = 16 - bandRows; if (take > rows) take = rows;
    memcpy(band + (size_t)bandRows * w * 3, rgb, (size_t)take * w * 3);
    bandRows += take; rgb += (size_t)take * w * 3; rows -= take;
    if (bandRows == 16) { encodeBand(band, 16); rowsDone += 16; bandRows = 0; }
  }
}

size_t BtlJpegEnc::end() {
  if (bandRows > 0 && rowsDone < h) { encodeBand(band, bandRows); rowsDone += bandRows; bandRows = 0; }
  while (bitCnt > 0) putBits(1, 1);                     // pad with 1-bits to a byte boundary
  putMarker(0xD9);                                      // EOI
  flush();
  free(band); band = nullptr;
  return total;
}
