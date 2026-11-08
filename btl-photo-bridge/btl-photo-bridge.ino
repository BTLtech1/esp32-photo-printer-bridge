/*
  BTL Photo Printer Bridge  v0.8.3   (ESP32-S3)
  Camera  --Wi-Fi/FTP-->  ESP32-S3  --USB cable or Wi-Fi (IPP)-->  photo printer (tested: Canon SELPHY CP1500)

  - Runs a small FTP server. Set the camera to "Transfer with SET" and press SET in playback to print.
  - Print modes (on the bridge's page): every JPEG received, or only photos rated N stars or more.
  - Each photo is shrunk to the printer's native print size (1872x1248 dots for a 4x6 with bleed) and
    turned upright before printing. Printers like the SELPHY take data in slowly, so smaller files print much sooner.
  - Photos wait in a queue (PSRAM) while the printer is busy.
  - Printer connection: USB cable when the printer is plugged into the board's "USB" port
    ("IPP over USB"), otherwise Wi-Fi. Board powered and flashed through its "UART" port.
  - Bridge page: http://btl-photo.local  (or the IP shown in Serial Monitor / on your router)

  Easiest install: https://btltech1.github.io/esp32-photo-printer-bridge (Chrome or Edge, no Arduino needed).

  Building it yourself in Arduino IDE (plug the board's UART port into the computer):
    Boards Manager: esp32 by Espressif, version 3.1.1 (newer 3.3.x versions don't connect USB printers yet)
    Library: copy libraries/esp_new_jpeg (from the repository) into your sketchbook's libraries folder
    Board: ESP32S3 Dev Module   |   USB CDC On Boot: Disabled   |   USB Mode: Hardware CDC and JTAG
    PSRAM: OPI PSRAM (N16R8 boards)
  (With USB CDC On Boot: Enabled it still works, but over Wi-Fi only.)

  Wi-Fi: the bridge runs its own network ("BTL-Photo") that the camera connects to directly:
  one Wi-Fi hop, no router needed, address always 192.168.4.1. It can also join your home Wi-Fi
  (set from the web installer, or from its page at http://192.168.4.1) for the printer over Wi-Fi
  and for reaching its page from your normal network.

  MIT License. Made by BTL. JPEG: Espressif's esp_new_jpeg (precompiled, see libraries/), TJpgDec (C)ChaN as fallback.
*/

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <esp_wifi.h>
#include <esp_heap_caps.h>
#include <stdarg.h>
#include <vector>
#include <deque>
#include "img_converters.h"   // JPEG encoder from the ESP32 core
#include "btl_tjpgd.h"        // JPEG decoder (TJpgDec): fallback
#include "esp_jpeg_dec.h"     // Espressif's fast JPEG decoder (libraries/esp_new_jpeg): the normal path
#include "btl_jpeg_enc.h"     // our streaming JPEG encoder: bytes go out while the photo is still being made
#include "usb/usb_host.h"     // USB host (printer on the board's USB port)
#include "freertos/stream_buffer.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/usb_serial_jtag_reg.h"
#include "improv_serial.h"     // Wi-Fi setup from the web installer
#include "btl_screen.h"       // optional 2.8" touchscreen

#define FW_VERSION    "0.9.0"
#define HOSTNAME      "btl-photo"
#define FTP_PORT      21
#define FTP_PASV_PORT 50021

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #define SRV_ACCEPT(s) (s).accept()
#else
  #define SRV_ACCEPT(s) (s).available()
#endif

// Optional hard-coded Wi-Fi. Leave empty to use the saved network or the setup page.
const char* WIFI_SSID = "";
const char* WIFI_PASS = "";

WebServer   server(80);
WiFiServer  ftpCtl(FTP_PORT);
Preferences prefs;

SemaphoreHandle_t printerMtx;   // one conversation with the printer at a time
SemaphoreHandle_t dataMtx;      // protects queue, log, status strings

// ---- settings
int    printMode = 0;           // 0 = print every JPEG received, 1 = only rated >= minRating
int    minRating = 3;
bool   shrinkOn  = true;     // shrink photos to print size before sending
bool   fillOn    = true;     // crop photos to the paper's 3:2 shape instead of leaving white strips
bool   scaleFill = true;     // borderless: ask for the paper with zero margins (media-col)
bool   dryRun    = false;    // test mode: process photos but don't print them
volatile bool saveScaleFill = false;
int    cassCap   = 54;       // prints per ink cassette (RP-108 = 2 x 54; KP-108IN = 3 x 36)
volatile int cassUsed = 0;   // prints since the last "New cassette"
volatile bool saveCass = false;
volatile bool camBusy = false;            // a photo is coming in from the camera
volatile bool saveColor = false;          // color settings changed from the screen
volatile int  scrLogAct = 0;              // counter reset from the screen, logged from loop()
char lastPhotoName[40] = "", lastPhotoInfo[40] = "";
uint32_t storStartMs = 0;
#define notePhoto(nm, fmt, ...) do { strlcpy(lastPhotoName, (nm), sizeof(lastPhotoName)); \
  snprintf(lastPhotoInfo, sizeof(lastPhotoInfo), fmt, ##__VA_ARGS__); } while (0)
// The SELPHY doesn't report how much ink is left, so the bridge counts its own prints.
int    paperCap  = 18;       // sheets loaded in the paper tray (the CP1500 tray holds 18)
volatile int paperUsed = 0;
#define countPrint() do { if (!dryRun) { cassUsed++; paperUsed++; saveCass = true; } } while (0)
#define paperLeft() (paperCap - paperUsed < 0 ? 0 : paperCap - paperUsed)
#define printsLeft() (cassCap - cassUsed < 0 ? 0 : cassCap - cassUsed)
int    jpegQ     = 85;       // JPEG quality of the shrunk photo
int    speedMode = 0;        // 0 = Best (1872x1248, the printer's native size), 1 = Fast (two thirds of that)
int    colWarm   = 0;        // -5 (cooler) .. +5 (warmer): the SELPHY's prints lean yellow, so cooler helps
int    colCon    = 0;        // 0..3 contrast
int    colSat    = 0;        // 0..3 color (saturation)
static uint8_t colLut[3][256];          // per-channel curve: warmth + contrast
static int     colSatK = 256;           // saturation factor, 256 = unchanged
static volatile bool colOn = false;

bool   useUsb    = true;     // print over USB when the printer is plugged in
String ftpUser   = "btl";
String ftpPass   = "print";
bool   apOn   = true;                 // the bridge's own Wi-Fi network for the camera
String apSsid = "BTL-Photo", apPass = "printphoto";
static const IPAddress AP_IP(192, 168, 4, 1);

// ---- printer
IPAddress printerIP;
uint16_t  printerPort  = 631;
String    printerPath  = "ipp/print";
String    printerName  = "";
volatile bool printerFound = false;
int       printerState = -1;
String    printerReasons = "";
volatile uint32_t lastSubmitMs = 0;
static bool curUsb = false;     // the current printer conversation is over USB
uint32_t  reqId = 1;

struct Job { uint8_t* data; size_t len; String name; int rating; uint8_t tries; };

// State for shrinking one photo to print size (see "shrink to print size" below).
struct ShrinkJob {
  // input
  const uint8_t* hb = nullptr;   // bytes already received (at least the full JPEG header)
  size_t hn = 0;
  Client* d = nullptr;           // rest of the photo comes from here (nullptr = hb is the whole file)
  bool eof = false;
  int orient = 1;
  uint8_t quality = 85;
  bool fast = false;             // Fast mode: two thirds of the native size
  // result
  int result = 0;                // 1 shrunk, 0 not shrunk (original still usable), -1 failed mid-way
  uint8_t* out = nullptr;
  size_t outLen = 0;
  String info, why;
  // working state
  size_t pos = 0, sockBytes = 0;
  bool err = false;
  int iw = 0, ih = 0, s = 0, sw = 0, sh = 0, tw = 0, th = 0, fw = 0, fh = 0, W = 0, base = 0;
  bool fill = true;              // fill the paper: crop the edges that don't fit 3:2 (otherwise fit inside, leaving white)
  size_t rxBytes = 0;            // bytes taken from the camera link
  uint32_t rxFirstMs = 0, rxLastMs = 0, waitMs = 0;
  int cx0 = 0, cy0 = 0, cw = 0, ch = 0;   // the part of the (scaled) source that goes onto the print
  float kScale = 1.0f;           // output size / source size
  uint32_t tDecode = 0;
  // Area-weighted (box) resampling: each source pixel covers part of one or two output pixels.
  // ox0/oy0 = first output pixel it lands in, wx0/wy0 = how much of it lands there (out of tw/th).
  uint16_t *ox0 = nullptr, *wx0 = nullptr, *oy0 = nullptr, *wy0 = nullptr;
  uint32_t* sums = nullptr;      // a few output rows being accumulated (fast internal RAM)
  uint32_t* hrow = nullptr;      // one source row, already shrunk horizontally
  uint8_t* band = nullptr;       // one row of decoded JPEG blocks (up to 16 source rows)
  int bandH = 16, bandTop = -1;
  uint8_t* frame = nullptr;
  void* pool = nullptr;
  size_t outCap = 0;
  bool outFail = false;
  TaskHandle_t waiter = nullptr;
  // fast path (Espressif codec): whole file in memory, output rows go band by band into the encoder
  uint8_t* whole = nullptr;      // the JPEG received into PSRAM (owned; handed back to the caller if not shrunk)
  size_t wholeCap = 0;
  uint8_t* bandOut = nullptr;    // one encoder block of finished output rows (RGB)
  int bandRows = 0;
  size_t bandBytes = 0;
  void* enc = nullptr;
  bool encFail = false;
  uint32_t tRecv = 0, tDec = 0, tEnc = 0, tDecOnly = 0;
  bool encTwoCores = false;
  bool ownSums = false;          // accumulators allocated for this job (else the shared ones reserved at boot)
  // Band pipeline: the resampler fills one band while a helper task encodes the other (on the other core)
  // and hands the encoded bytes to the sink as they come (the printer, while it's still resizing).
  uint8_t* bands[2] = { nullptr, nullptr };
  SemaphoreHandle_t bandFull[2] = { nullptr, nullptr }, bandFree[2] = { nullptr, nullptr };
  volatile int bandCount = 0, bandsTotal = 0;
  volatile bool encExited = false;
  TaskHandle_t encTaskH = nullptr;
  void (*sink)(void* ctx, const uint8_t* data, size_t len) = nullptr;   // where encoded bytes go as they're made
  void* sinkCtx = nullptr;
  volatile size_t sinkSent = 0;  // bytes of 'out' already handed to the sink
  volatile bool sinkFail = false, sendExited = false;
  SemaphoreHandle_t outReady = nullptr;   // the encoder signals the sender when more bytes exist
  uint32_t tFirstSend = 0, tEncDone = 0, tSendDone = 0, sendWrites = 0;   // timeline of the stream to the printer
};
class CamData;                 // camera data link, defined below (declared here for Arduino's auto-prototypes)
std::deque<Job>     jobs;
std::deque<String>  logLines;
std::vector<String> printedNames;

// =============================================================== small helpers
static void blog(const char* fmt, ...) {
  char buf[320];
  va_list ap; va_start(ap, fmt); vsnprintf(buf, sizeof(buf), fmt, ap); va_end(ap);
  uint32_t s = millis() / 1000;
  char ts[16];
  snprintf(ts, sizeof(ts), "%02lu:%02lu:%02lu  ", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60), (unsigned long)(s % 60));
  String line = String(ts) + buf;
  Serial.println(line);
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  logLines.push_front(line);
  while (logLines.size() > 40) logLines.pop_back();
  xSemaphoreGive(dataMtx);
}

// Copies the ESP32's own error messages (for example USB setup failures) into the activity log.
static vprintf_like_t prevLog = nullptr;
static int logHook(const char* fmt, va_list ap) {
  char b[200];
  va_list ap2; va_copy(ap2, ap);
  vsnprintf(b, sizeof(b), fmt, ap2);
  va_end(ap2);
  int r = prevLog ? prevLog(fmt, ap) : 0;
  if (b[0] != 'E' && b[0] != 'W' && !(b[0] == 0x1B && (strstr(b, "E (") || strstr(b, "W (")))) return r;   // only errors and warnings
  String line;                                    // drop colour codes and line breaks
  for (const char* c = b; *c; c++) {
    if (*c == 0x1B) { while (*c && *c != 'm') c++; if (!*c) break; continue; }
    if (*c == '\r' || *c == '\n') continue;
    line += *c;
  }
  if (line.length() && dataMtx && xSemaphoreTake(dataMtx, 0) == pdTRUE) {
    uint32_t s = millis() / 1000;
    char ts[16];
    snprintf(ts, sizeof(ts), "%02lu:%02lu:%02lu  ", (unsigned long)(s / 3600), (unsigned long)(s / 60 % 60), (unsigned long)(s % 60));
    logLines.push_front(String(ts) + "ESP32: " + line);
    while (logLines.size() > 40) logLines.pop_back();
    xSemaphoreGive(dataMtx);
  }
  return r;
}

static String stars(int r) {
  if (r < 0) return "no rating";
  return String(r) + "\xE2\x98\x85";   // N★
}

static String fmtSize(size_t n) {
  char b[24];
  if (n >= 1024 * 1024) snprintf(b, sizeof(b), "%.1f MB", n / 1048576.0);
  else snprintf(b, sizeof(b), "%u KB", (unsigned)(n / 1024));
  return String(b);
}

static String esc(const String& s) {
  String o; o.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '<') o += "&lt;"; else if (ch == '>') o += "&gt;"; else if (ch == '&') o += "&amp;";
    else if (ch == '"') o += "&quot;"; else if (ch == '\'') o += "&#39;"; else o += ch;
  }
  return o;
}

static String jsonEsc(const String& s) {
  String o; o.reserve(s.length() + 8);
  for (size_t i = 0; i < s.length(); i++) {
    char ch = s[i];
    if (ch == '"') o += "\\\""; else if (ch == '\\') o += "\\\\";
    else if (ch == '\n') o += "\\n"; else if ((uint8_t)ch < 0x20) o += ' '; else o += ch;
  }
  return o;
}

static void* bigAlloc(size_t n) {
  void* p = psramFound() ? heap_caps_malloc(n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : nullptr;
  if (!p) p = malloc(n);
  return p;
}
static void* bigRealloc(void* p, size_t n) {
  void* q = psramFound() ? heap_caps_realloc(p, n, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : nullptr;
  if (!q) q = realloc(p, n);
  return q;
}
static void* fastAlloc(size_t n) {          // internal RAM if there's room, else PSRAM
  void* p = heap_caps_malloc(n, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  return p ? p : bigAlloc(n);
}
static size_t spoolLimit() {
  if (psramFound()) {
    size_t m = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
    return m > 512 * 1024 ? m - 512 * 1024 : 0;
  }
  size_t m = ESP.getMaxAllocHeap();
  return m > 96 * 1024 ? min(m - 96 * 1024, (size_t)160 * 1024) : 0;
}

static size_t queueLen() {
  xSemaphoreTake(dataMtx, portMAX_DELAY); size_t n = jobs.size(); xSemaphoreGive(dataMtx); return n;
}
static void addPrinted(const String& n) {
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  printedNames.push_back(n);
  if (printedNames.size() > 500) printedNames.erase(printedNames.begin());
  xSemaphoreGive(dataMtx);
}
static bool wasPrinted(const String& n) {
  bool f = false;
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  for (auto& s : printedNames) if (s == n) { f = true; break; }
  xSemaphoreGive(dataMtx);
  return f;
}

// =============================================================== IPP encoding
static void put8(std::vector<uint8_t>& b, uint8_t v)   { b.push_back(v); }
static void put16(std::vector<uint8_t>& b, uint16_t v) { b.push_back(v >> 8); b.push_back(v & 0xFF); }
static void put32(std::vector<uint8_t>& b, uint32_t v) { put16(b, v >> 16); put16(b, v & 0xFFFF); }
static void putBytes(std::vector<uint8_t>& b, const char* s, size_t n) { for (size_t i = 0; i < n; i++) b.push_back((uint8_t)s[i]); }

static void putAttr(std::vector<uint8_t>& b, uint8_t tag, const char* name, const String& val) {
  put8(b, tag);
  put16(b, strlen(name)); putBytes(b, name, strlen(name));
  put16(b, val.length()); putBytes(b, val.c_str(), val.length());
}
static void putAttrMore(std::vector<uint8_t>& b, uint8_t tag, const String& val) {   // additional value
  put8(b, tag); put16(b, 0);
  put16(b, val.length()); putBytes(b, val.c_str(), val.length());
}

static String printerUri() {
  if (curUsb) return "ipp://localhost/ipp/print";
  return "ipp://" + printerIP.toString() + ":" + String(printerPort) + "/" + printerPath;
}

static void ippHeader(std::vector<uint8_t>& b, uint16_t op) {
  put8(b, 0x01); put8(b, 0x01);           // IPP/1.1
  put16(b, op);
  put32(b, reqId++);
  put8(b, 0x01);
  putAttr(b, 0x47, "attributes-charset", "utf-8");
  putAttr(b, 0x48, "attributes-natural-language", "en");
  putAttr(b, 0x45, "printer-uri", printerUri());
}

// Collection members (IPP "memberAttrName" + value, no attribute name)
static void putMember(std::vector<uint8_t>& b, const char* member) {
  put8(b, 0x4A); put16(b, 0); put16(b, strlen(member)); for (const char* c = member; *c; c++) put8(b, (uint8_t)*c);
}
static void putMemberInt(std::vector<uint8_t>& b, const char* member, int32_t v) {
  putMember(b, member); put8(b, 0x21); put16(b, 0); put16(b, 4); put32(b, (uint32_t)v);
}
static void putMemberKeyword(std::vector<uint8_t>& b, const char* member, const char* v) {
  putMember(b, member); put8(b, 0x44); put16(b, 0); put16(b, strlen(v)); for (const char* c = v; *c; c++) put8(b, (uint8_t)*c);
}
static void beginCollection(std::vector<uint8_t>& b, const char* name) {   // name may be "" for a nested one
  put8(b, 0x34); put16(b, strlen(name)); for (const char* c = name; *c; c++) put8(b, (uint8_t)*c); put16(b, 0);
}
static void endCollection(std::vector<uint8_t>& b) { put8(b, 0x37); put16(b, 0); put16(b, 0); }

// The SELPHY's default paper setting is the postcard WITH margins (3.7 mm top/bottom, 2.5 mm sides);
// borderless is the same paper asked for with all margins at zero, as a "media-col" collection.
static void buildPrintJob(std::vector<uint8_t>& h, const String& name, bool withScaling = true) {
  ippHeader(h, 0x0002);                   // Print-Job
  putAttr(h, 0x42, "requesting-user-name", "btl-photo");
  putAttr(h, 0x42, "job-name", name.length() ? name : String("photo.jpg"));
  putAttr(h, 0x49, "document-format", "image/jpeg");
  if (withScaling && scaleFill) {
    put8(h, 0x02);                        // job attributes group
    beginCollection(h, "media-col");
    putMember(h, "media-size");
    beginCollection(h, "");
    putMemberInt(h, "x-dimension", 10000);   // 100 x 148 mm, in hundredths of a millimetre
    putMemberInt(h, "y-dimension", 14800);
    endCollection(h);
    putMemberInt(h, "media-top-margin", 0);
    putMemberInt(h, "media-bottom-margin", 0);
    putMemberInt(h, "media-left-margin", 0);
    putMemberInt(h, "media-right-margin", 0);
    putMemberKeyword(h, "media-type", "photographic");
    putMemberKeyword(h, "media-source", "photo");
    endCollection(h);
  }
  put8(h, 0x03);
}

static String ippStatusName(uint16_t s) {
  switch (s) {
    case 0x0000: return "ok";
    case 0x0001: return "ok (some settings substituted)";
    case 0x0002: return "ok (conflicting settings)";
    case 0x0400: return "bad request";
    case 0x040A: return "file format not supported";
    case 0x040B: return "settings not supported";
    case 0x0500: return "printer internal error";
    case 0x0506: return "printer not accepting jobs";
    case 0x0507: return "printer busy";
    case 0xFFFD: return "camera stopped sending";
    case 0xFFFE: return "could not reach printer";
    case 0xFFFF: return "no valid answer from printer";
  }
  char b[10]; snprintf(b, sizeof(b), "0x%04X", s); return String(b);
}

// =============================================================== IPP response parsing
static uint32_t rd32(const uint8_t* p) {
  return ((uint32_t)p[0] << 24) | ((uint32_t)p[1] << 16) | ((uint32_t)p[2] << 8) | p[3];
}

// Turns an IPP response into "name = value" lines (collection members as "name.member = value").
// Returns the IPP status code.
static uint16_t parseIpp(const std::vector<uint8_t>& d, String* out) {
  if (d.size() < 8) return 0xFFFF;
  uint16_t status = (d[2] << 8) | d[3];
  size_t p = 8, n = d.size();
  String path[6]; int depth = 0;                        // collection nesting
  String member;                                        // pending member name inside a collection
  String lastTop;                                       // name of the attribute currently being listed (for extra values)
  while (p < n) {
    uint8_t tag = d[p++];
    if (tag == 0x03) break;
    if (tag < 0x10) continue;
    if (p + 2 > n) break;
    uint16_t nl = (d[p] << 8) | d[p + 1]; p += 2;
    if (p + nl > n) break;
    String name; for (uint16_t i = 0; i < nl; i++) name += (char)d[p + i];
    p += nl;
    if (p + 2 > n) break;
    uint16_t vl = (d[p] << 8) | d[p + 1]; p += 2;
    if (p + vl > n) break;
    const uint8_t* v = &d[p]; p += vl;
    if (!out) continue;
    if (tag == 0x4A) { member = ""; for (uint16_t i = 0; i < vl; i++) member += (char)v[i]; continue; }   // memberAttrName
    String full = name.length() ? name : member;
    if (depth > 0 && depth < 6) full = path[depth - 1] + "." + full;
    if (tag == 0x34) { if (depth < 6) path[depth] = full; depth++; member = ""; continue; }   // begCollection
    if (tag == 0x37) { if (depth > 0) depth--; member = ""; continue; }                      // endCollection
    String s;
    if ((tag == 0x21 || tag == 0x23) && vl == 4) s = String((int32_t)rd32(v));
    else if (tag == 0x22 && vl == 1) s = v[0] ? "true" : "false";
    else if (tag == 0x32 && vl == 9) s = String((int32_t)rd32(v)) + "x" + String((int32_t)rd32(v + 4)) + (v[8] == 3 ? " dpi" : " dpcm");
    else if (tag == 0x33 && vl == 8) s = String((int32_t)rd32(v)) + "-" + String((int32_t)rd32(v + 4));
    else if (tag >= 0x40) { uint16_t m = vl > 160 ? 160 : vl; for (uint16_t i = 0; i < m; i++) s += (char)v[i]; }
    else if (tag == 0x13) s = "no-value";
    else { s = "(tag 0x" + String(tag, HEX) + ", " + String(vl) + " bytes)"; }
    if (name.length() || depth > 0) { *out += "\n" + full + " = " + s; lastTop = full; }
    else *out += ", " + s;                                // another value of the same attribute
    if (depth > 0) member = "";
  }
  return status;
}

static String attrVal(const String& txt, const char* name) {
  String key = "\n" + String(name) + " = ";
  int i = txt.indexOf(key);
  if (i < 0) return "";
  i += key.length();
  int e = txt.indexOf('\n', i);
  return e < 0 ? txt.substring(i) : txt.substring(i, e);
}

// =============================================================== network I/O helpers
static int readByteT(Client& c, uint32_t timeoutMs) {
  uint32_t t = millis();
  while (!c.available()) {
    if (!c.connected()) return -1;
    if (millis() - t > timeoutMs) return -1;
    delay(1);
  }
  return c.read();
}

static bool readLineT(Client& c, String& line, uint32_t timeoutMs) {
  line = "";
  while (true) {
    int ch = readByteT(c, timeoutMs);
    if (ch < 0) return line.length() > 0;
    if (ch == '\n') return true;
    if (ch != '\r') line += (char)ch;
    if (line.length() > 1024) return true;
  }
}

// Reads up to len bytes. Returns >0 bytes read, 0 at end of stream, -1 on timeout.
static int readSome(Client& d, uint8_t* buf, size_t len, uint32_t timeoutMs) {
  uint32_t t = millis();
  while (true) {
    int av = d.available();
    if (av > 0) {
      int n = d.read(buf, min((size_t)av, len));
      if (n > 0) return n;
    } else if (!d.connected()) {
      if (d.available() <= 0) return 0;
      continue;
    }
    if (millis() - t > timeoutMs) return -1;
    delay(1);
  }
}

static bool writeAll(Client& c, const uint8_t* p, size_t n) {
  uint32_t t = millis();
  while (n) {
    size_t w = c.write(p, n > 8192 ? 8192 : n);
    if (w == 0) {
      if (!c.connected() || millis() - t > 20000) return false;
      delay(2); continue;
    }
    p += w; n -= w; t = millis();
  }
  return true;
}

static void sendChunk(Client& c, const uint8_t* data, size_t len) {
  if (!len) return;
  char hdr[12]; int hl = snprintf(hdr, sizeof(hdr), "%X\r\n", (unsigned)len);
  if (len <= 16384) {                                   // one write: fewer round trips with the printer
    uint8_t* b = (uint8_t*)malloc(hl + len + 2);
    if (b) {
      memcpy(b, hdr, hl); memcpy(b + hl, data, len); b[hl + len] = '\r'; b[hl + len + 1] = '\n';
      writeAll(c, b, hl + len + 2); free(b); return;
    }
  }
  writeAll(c, (const uint8_t*)hdr, hl); writeAll(c, data, len); writeAll(c, (const uint8_t*)"\r\n", 2);
}

// timing of the last print job, for the log
uint32_t lastSendMs = 0, lastAnswerMs = 0;

static void sendPostHeaders(Client& c, bool chunked, size_t len) {
  String h;
  if (curUsb)
    h = "POST /ipp/print HTTP/1.1\r\n"
        "Host: localhost\r\n"
        "Content-Type: application/ipp\r\n";
  else
    h = "POST /" + printerPath + " HTTP/1.1\r\n"
        "Host: " + printerIP.toString() + ":" + String(printerPort) + "\r\n"
        "Content-Type: application/ipp\r\n"
        "Connection: close\r\n";
  h += chunked ? "Transfer-Encoding: chunked\r\n" : ("Content-Length: " + String(len) + "\r\n");
  h += "\r\n";
  c.print(h);
}

static int readHttpResponse(Client& c, std::vector<uint8_t>& body) {
  String line;
  // Normally the answer starts with "HTTP/1.1 200 OK". Over USB the SELPHY may send the bare IPP
  // answer instead (it starts with the IPP version byte, 1 or 2): then read it until it goes quiet.
  int first = readByteT(c, 90000);
  if (first < 0) return -1;
  if (first == 0x01 || first == 0x02) {
    body.push_back((uint8_t)first);
    int ch;
    while ((ch = readByteT(c, 400)) >= 0) body.push_back((uint8_t)ch);
    return body.size() >= 8 ? 200 : -1;
  }
  if (first != '\n') { if (first != '\r') line += (char)first; String rest; if (!readLineT(c, rest, 10000) && !line.length()) return -1; line += rest; }
  int code = -1;
  int sp = line.indexOf(' ');
  if (sp > 0) code = line.substring(sp + 1, sp + 4).toInt();
  bool chunked = false; long clen = -1;
  while (readLineT(c, line, 10000)) {
    if (line.length() == 0) break;
    String l = line; l.toLowerCase();
    if (l.startsWith("transfer-encoding:") && l.indexOf("chunked") > 0) chunked = true;
    if (l.startsWith("content-length:")) clen = l.substring(15).toInt();
  }
  if (code == 100) return readHttpResponse(c, body);
  if (chunked) {
    while (true) {
      if (!readLineT(c, line, 10000)) break;
      line.trim();
      if (line.length() == 0) continue;
      long sz = strtol(line.c_str(), nullptr, 16);
      if (sz <= 0) { readLineT(c, line, 2000); break; }
      for (long i = 0; i < sz; i++) { int ch = readByteT(c, 10000); if (ch < 0) return code; body.push_back((uint8_t)ch); }
      readLineT(c, line, 2000);
    }
  } else if (clen >= 0) {
    for (long i = 0; i < clen; i++) { int ch = readByteT(c, 10000); if (ch < 0) break; body.push_back((uint8_t)ch); }
  } else {
    int ch; while ((ch = readByteT(c, 3000)) >= 0) body.push_back((uint8_t)ch);
  }
  return code;
}

// =============================================================== USB printer (IPP over USB)
// The SELPHY can also take print jobs over its USB-C port. Printers that support "IPP over USB"
// (USB class 7, subclass 1, protocol 4) accept exactly the same IPP requests as over Wi-Fi: the
// HTTP request goes out on a bulk OUT endpoint and the answer comes back on a bulk IN endpoint.
// Same USB host setup as the BTL label printer bridge: the printer plugs into the board's port
// labeled USB, and the board is powered and flashed through its UART port.
#define USB_OUT_BYTES 16384
#define USB_IN_BYTES  4096                  // must be a multiple of the endpoint packet size
#define USB_RX_BYTES  32768

static usb_host_client_handle_t uClient = nullptr;
static usb_device_handle_t uDev = nullptr;
static volatile uint8_t uNewAddr = 0;
static volatile bool uGone = false;
static volatile bool uReady = false;        // IPP-over-USB interface claimed, reader running
static volatile bool uOutStuck = false;
static uint8_t uIntf = 0xFF, uAlt = 0, uEpOut = 0, uEpIn = 0;
static uint16_t uMpsOut = 64, uMpsIn = 64;
// counters shown on the diagnostics page
static volatile uint32_t dOutXfers = 0, dOutBytes = 0, dOutErr = 0, dInXfers = 0, dInBytes = 0, dInErr = 0;
static volatile int dOutLastSt = -1, dInLastSt = -1;
static char dLastReply[96] = "";
static usb_transfer_t *uOut = nullptr, *uIn = nullptr, *uCtrl = nullptr;
static SemaphoreHandle_t uOutDone = nullptr;
static volatile bool uOutOk = false, uInBusy = false, uCtrlDone = false, uCtrlOk = false;
static StreamBufferHandle_t uRx = nullptr;
static portMUX_TYPE uMux = portMUX_INITIALIZER_UNLOCKED;
static String usbName = "", usbState = "not started";

static void setUsbState(const String& s) {
  xSemaphoreTake(dataMtx, portMAX_DELAY); usbState = s; xSemaphoreGive(dataMtx);
}
static String getUsbState() {
  xSemaphoreTake(dataMtx, portMAX_DELAY); String s = usbState; xSemaphoreGive(dataMtx); return s;
}

static void uInCb(usb_transfer_t* t);

// Keep one read request waiting on the printer's IN endpoint whenever there's room for its answer.
static void uInSubmit() {
  if (!uReady || uInBusy) return;
  if (xStreamBufferSpacesAvailable(uRx) < USB_IN_BYTES) return;   // read() resubmits once there's room
  bool go = false;
  portENTER_CRITICAL(&uMux);
  if (!uInBusy) { uInBusy = true; go = true; }
  portEXIT_CRITICAL(&uMux);
  if (!go) return;
  uIn->device_handle = uDev;
  uIn->bEndpointAddress = uEpIn;
  uIn->num_bytes = USB_IN_BYTES;
  uIn->callback = uInCb;
  uIn->context = nullptr;
  uIn->timeout_ms = 0;
  if (usb_host_transfer_submit(uIn) != ESP_OK) uInBusy = false;
}

static void uInCb(usb_transfer_t* t) {
  dInLastSt = (int)t->status;
  if (t->status == USB_TRANSFER_STATUS_COMPLETED) {
    dInXfers++;
    if (t->actual_num_bytes > 0) {
      dInBytes += t->actual_num_bytes;
      if (!dLastReply[0]) {                          // keep the start of the first reply, as text
        int n = min((int)t->actual_num_bytes, (int)sizeof(dLastReply) - 1), k = 0;
        for (int i = 0; i < n; i++) { char ch = (char)t->data_buffer[i]; dLastReply[k++] = (ch >= 32 && ch < 127) ? ch : '.'; }
        dLastReply[k] = 0;
      }
      xStreamBufferSend(uRx, t->data_buffer, t->actual_num_bytes, 0);
    }
  } else dInErr++;
  bool ok = t->status == USB_TRANSFER_STATUS_COMPLETED;
  uInBusy = false;
  if (ok) uInSubmit();
  else if (t->status != USB_TRANSFER_STATUS_NO_DEVICE && t->status != USB_TRANSFER_STATUS_CANCELED)
    blog("USB: read from printer failed (status %d)", (int)t->status);
}

static void uOutCb(usb_transfer_t* t) {
  uOutOk = (t->status == USB_TRANSFER_STATUS_COMPLETED);
  dOutLastSt = (int)t->status;
  if (uOutOk) { dOutXfers++; dOutBytes += t->actual_num_bytes; } else dOutErr++;
  xSemaphoreGive(uOutDone);
}

static void uCtrlCb(usb_transfer_t* t) {
  uCtrlOk = (t->status == USB_TRANSFER_STATUS_COMPLETED);
  uCtrlDone = true;
}

// Looks like a network connection to the printing code, but runs over the USB cable.
class UsbIppClient : public Client {
public:
  using Print::write;
  bool begin() {                                      // start of a new request
    if (!uReady || uOutStuck) return false;
    uint8_t junk[256];                                // throw away anything left over
    while (xStreamBufferReceive(uRx, junk, sizeof(junk), 0) > 0) {}
    dLastReply[0] = 0;
    uInSubmit();
    return true;
  }
  int connect(IPAddress, uint16_t) override { return begin() ? 1 : 0; }
  int connect(const char*, uint16_t) override { return begin() ? 1 : 0; }
  size_t write(uint8_t b) override { return write(&b, 1); }
  size_t write(const uint8_t* p, size_t n) override {
    size_t done = 0;
    while (done < n && uReady && !uOutStuck) {
      size_t k = min(n - done, (size_t)USB_OUT_BYTES);
      memcpy(uOut->data_buffer, p + done, k);
      uOut->device_handle = uDev;
      uOut->bEndpointAddress = uEpOut;
      uOut->num_bytes = k;
      // If the data ends exactly on a packet boundary, add an empty "end" packet, or the
      // printer may keep waiting for more.
      uOut->flags = (uMpsOut && (k % uMpsOut) == 0) ? USB_TRANSFER_FLAG_ZERO_PACK : 0;
      uOut->callback = uOutCb;
      uOut->context = nullptr;
      uOut->timeout_ms = 0;
      xSemaphoreTake(uOutDone, 0);
      if (usb_host_transfer_submit(uOut) != ESP_OK) break;
      if (xSemaphoreTake(uOutDone, pdMS_TO_TICKS(20000)) != pdTRUE) {
        uOutStuck = true;
        blog("USB: printer stopped taking data - unplug and replug it");
        break;
      }
      if (!uOutOk) { blog("USB: sending to printer failed (status %d)", dOutLastSt); break; }
      done += k;
    }
    return done;
  }
  int available() override {
    size_t n = xStreamBufferBytesAvailable(uRx);
    if (!n) uInSubmit();
    return (int)n;
  }
  int read() override {
    uint8_t b;
    if (xStreamBufferReceive(uRx, &b, 1, 0) != 1) return -1;
    if (!uInBusy) uInSubmit();
    return b;
  }
  int read(uint8_t* buf, size_t len) override {
    size_t n = xStreamBufferReceive(uRx, buf, len, 0);
    if (!uInBusy) uInSubmit();
    return n ? (int)n : -1;
  }
  int peek() override { return -1; }
  void flush() override {}
  void stop() override {}                             // the USB "connection" stays open
  uint8_t connected() override { return (uReady && !uOutStuck) ? 1 : 0; }
  operator bool() override { return connected(); }
};
UsbIppClient usbIpp;

static bool usbIppReady() { return uReady && !uOutStuck && useUsb; }

static void uClientCb(const usb_host_client_event_msg_t* msg, void* arg) {
  if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) uNewAddr = msg->new_dev.address;
  else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) uGone = true;
}

// Waits for USB events in this task (the one that delivers transfer callbacks).
static void uPump(volatile bool* flag, bool want, uint32_t ms) {
  uint32_t t = millis();
  while (*flag != want && millis() - t < ms) usb_host_client_handle_events(uClient, pdMS_TO_TICKS(10));
}

static String usbStr(const usb_str_desc_t* d) {
  String o; if (!d) return o;
  int n = (d->bLength - 2) / 2;
  for (int i = 0; i < n; i++) { uint16_t c = d->wData[i]; if (c >= 32 && c < 127) o += (char)c; }
  o.trim(); return o;
}

static void usbClose(const char* why) {
  bool wasReady = uReady;
  uReady = false;
  if (uDev) {
    if (uInBusy) {                                     // cancel the waiting read request
      uPump(&uInBusy, false, 300);
      if (uInBusy) {
        usb_host_endpoint_halt(uDev, uEpIn);
        usb_host_endpoint_flush(uDev, uEpIn);
        uPump(&uInBusy, false, 1000);
        usb_host_endpoint_clear(uDev, uEpIn);
      }
    }
    if (uIntf != 0xFF) usb_host_interface_release(uClient, uDev, uIntf);
    usb_host_device_close(uClient, uDev);
  }
  uDev = nullptr; uIntf = 0xFF; uOutStuck = false;
  xSemaphoreTake(dataMtx, portMAX_DELAY); usbName = ""; xSemaphoreGive(dataMtx);
  setUsbState("waiting for the printer on the USB port");
  if (wasReady || why[0]) blog("USB: printer %s", why);
}

static void usbOpen(uint8_t addr) {
  esp_err_t oe = usb_host_device_open(uClient, addr, &uDev);
  if (oe != ESP_OK) { uDev = nullptr; blog("USB: couldn't open the device at address %d (error 0x%X)", addr, (unsigned)oe); return; }
  const usb_device_desc_t* dd = nullptr;
  usb_host_get_device_descriptor(uDev, &dd);
  usb_device_info_t info;
  String prod;
  if (usb_host_device_info(uDev, &info) == ESP_OK) {
    prod = usbStr(info.str_desc_product);
    if (!prod.length()) prod = usbStr(info.str_desc_manufacturer);
  }
  char idb[16]; snprintf(idb, sizeof(idb), "%04X:%04X", dd ? dd->idVendor : 0, dd ? dd->idProduct : 0);
  const usb_config_desc_t* cfg = nullptr;
  esp_err_t ce = usb_host_get_active_config_descriptor(uDev, &cfg);
  if (ce != ESP_OK || !cfg) { blog("USB: %s - couldn't read its configuration (error 0x%X)", idb, (unsigned)ce); usbClose(""); return; }

  // Walk the descriptors: note every interface (for the log) and find IPP over USB (7/1/4).
  const uint8_t* p = (const uint8_t*)cfg;
  const uint8_t* end = p + cfg->wTotalLength;
  String list;
  int ci = -1, ca = 0, cc = -1, cs = -1, cp = -1, cin = 0, cout = 0, cmi = 64, cmo = 64;
  int bi = -1, ba = 0, bin = 0, bout = 0, bmi = 64, bmo = 64;
  auto closeIntf = [&]() {
    if (ci >= 0 && cc == 7 && cs == 1 && cp == 4 && cin && cout && bi < 0) { bi = ci; ba = ca; bin = cin; bout = cout; bmi = cmi; bmo = cmo; }
  };
  while (p < end && p[0] >= 2) {
    if (p[1] == 0x04 && p[0] >= 9) {
      closeIntf();
      ci = p[2]; ca = p[3]; cc = p[5]; cs = p[6]; cp = p[7]; cin = cout = 0;
      char b[24]; snprintf(b, sizeof(b), " %d.%d=%d/%d/%d", ci, ca, cc, cs, cp);
      if (list.length() < 120) list += b;
    } else if (p[1] == 0x05 && p[0] >= 7 && ci >= 0) {
      uint8_t ep = p[2], attr = p[3];
      int mps = (p[4] | (p[5] << 8)) & 0x7FF;
      if ((attr & 0x03) == 0x02) { if (ep & 0x80) { if (!cin) { cin = ep; cmi = mps; } } else { if (!cout) { cout = ep; cmo = mps; } } }
    }
    p += p[0];
  }
  closeIntf();
  blog("USB: found %s (%s), interfaces:%s", prod.length() ? prod.c_str() : "a device", idb, list.c_str());

  if (bi < 0) {
    blog("USB: this device doesn't offer IPP over USB (needs an interface 7/1/4), so it can't be used over USB");
    setUsbState(String("connected, but no IPP over USB (") + idb + ")");
    if (uDev) usb_host_device_close(uClient, uDev);
    uDev = nullptr;
    return;
  }
  if (usb_host_interface_claim(uClient, uDev, bi, ba) != ESP_OK) { blog("USB: couldn't claim interface %d", bi); usbClose(""); return; }
  uIntf = bi; uAlt = ba; uEpIn = bin; uEpOut = bout;
  uMpsIn = bmi ? bmi : 64; uMpsOut = bmo ? bmo : 64;
  if (ba != 0) {                                       // switch the printer to the IPP-over-USB mode
    USB_SETUP_PACKET_INIT_SET_INTERFACE((usb_setup_packet_t*)uCtrl->data_buffer, bi, ba);
    uCtrl->num_bytes = sizeof(usb_setup_packet_t);
    uCtrl->device_handle = uDev;
    uCtrl->bEndpointAddress = 0;
    uCtrl->callback = uCtrlCb;
    uCtrl->context = nullptr;
    uCtrlDone = false;
    if (usb_host_transfer_submit_control(uClient, uCtrl) == ESP_OK) uPump(&uCtrlDone, true, 1000);
    if (!uCtrlDone || !uCtrlOk) blog("USB: printer didn't confirm the IPP mode switch (continuing anyway)");
  }
  xSemaphoreTake(dataMtx, portMAX_DELAY); usbName = prod.length() ? prod : String(idb); xSemaphoreGive(dataMtx);
  uOutStuck = false;
  uReady = true;
  uInSubmit();
  setUsbState("ready");
  blog("USB: IPP over USB ready on interface %d.%d (out 0x%02X/%d, in 0x%02X/%d) - prints will go over the cable", bi, ba, bout, uMpsOut, bin, uMpsIn);
}

// ---- USB port diagnostics: read the USB controller's own registers.
#define BTL_USB_OTG_BASE 0x60080000u                   // USB OTG controller (USB_DWC)
#define BTL_HPRT   (BTL_USB_OTG_BASE + 0x440)          // host port control/status
#define BTL_GINTSTS (BTL_USB_OTG_BASE + 0x014)
#define BTL_GUSBCFG (BTL_USB_OTG_BASE + 0x00C)
static volatile bool usbHostStarted = false;

static String hprtText(uint32_t h) {
  bool conn = h & 1, pwr = h & (1u << 12), ena = h & (1u << 2);
  int ln = (h >> 10) & 3, spd = (h >> 17) & 3;
  String o = String("port power ") + (pwr ? "ON" : "OFF");
  o += String(", device ") + (conn ? "SENSED" : "not sensed");
  o += String(", D+ ") + ((ln & 1) ? "high" : "low") + ", D- " + ((ln & 2) ? "high" : "low");
  if (conn) o += String(", ") + (ena ? "enabled" : "not enabled") + ", speed " + (spd == 1 ? "full" : spd == 2 ? "low" : spd == 0 ? "high" : "?");
  return o;
}

static String usbDiagText() {
  String o = "BTL Photo Printer Bridge v" FW_VERSION " USB diagnostics\n\n";
#ifdef ARDUINO_USB_MODE
  o += "Build: USB Mode = " + String(ARDUINO_USB_MODE ? "Hardware CDC and JTAG" : "USB-OTG (TinyUSB)");
#else
  o += "Build: USB Mode = (not set)";
#endif
#ifdef ARDUINO_USB_CDC_ON_BOOT
  o += ", USB CDC On Boot = " + String(ARDUINO_USB_CDC_ON_BOOT ? "Enabled" : "Disabled") + "\n";
#else
  o += ", USB CDC On Boot = (not set)\n";
#endif
  o += "USB state: " + getUsbState() + "\n";
  if (usbHostStarted) {
    usb_host_lib_info_t li;
    if (usb_host_lib_info(&li) == ESP_OK) o += "USB library: " + String(li.num_devices) + " device(s), " + String(li.num_clients) + " client(s)\n";
  } else o += "USB library: not started\n";
  uint32_t h = REG_READ(BTL_HPRT);
  char b[64];
  snprintf(b, sizeof(b), "HPRT = 0x%08lX: ", (unsigned long)h);
  o += String(b) + hprtText(h) + "\n";
  char c2[200];
  snprintf(c2, sizeof(c2), "IPP channel: interface %d, out 0x%02X (%d B packets), in 0x%02X (%d B packets), %s\n",
           uIntf == 0xFF ? -1 : uIntf, uEpOut, uMpsOut, uEpIn, uMpsIn, uReady ? "open" : "closed");
  o += c2;
  snprintf(c2, sizeof(c2), "Sent: %lu transfers, %lu bytes, %lu errors (last status %d)%s\n",
           (unsigned long)dOutXfers, (unsigned long)dOutBytes, (unsigned long)dOutErr, dOutLastSt, uOutStuck ? ", STUCK" : "");
  o += c2;
  snprintf(c2, sizeof(c2), "Received: %lu transfers, %lu bytes, %lu errors (last status %d), read waiting: %s\n",
           (unsigned long)dInXfers, (unsigned long)dInBytes, (unsigned long)dInErr, dInLastSt, uInBusy ? "yes" : "no");
  o += c2;
  o += String("Start of last reply: ") + (dLastReply[0] ? dLastReply : "(none)") + "\n";
  uint32_t gi = REG_READ(BTL_GINTSTS), gu = REG_READ(BTL_GUSBCFG);
  o += String("Controller mode: ") + ((gi & 1) ? "HOST" : "device") + ((gu & (1u << 29)) ? " (forced host)" : "") + "\n";
  uint32_t rc = REG_READ(RTC_CNTL_USB_CONF_REG);
  o += String("PHY routing: sw_hw_usb_phy_sel=") + ((rc & RTC_CNTL_SW_HW_USB_PHY_SEL) ? "1" : "0") +
       " sw_usb_phy_sel=" + ((rc & RTC_CNTL_SW_USB_PHY_SEL) ? "1" : "0") + "\n";
  uint32_t cj = REG_READ(USB_SERIAL_JTAG_CONF0_REG);
  o += String("USB-Serial-JTAG: pad_enable=") + ((cj & USB_SERIAL_JTAG_USB_PAD_ENABLE) ? "1" : "0") +
       " phy_sel=" + ((cj & USB_SERIAL_JTAG_PHY_SEL) ? "1" : "0") +
       " pull_override=" + ((cj & USB_SERIAL_JTAG_PAD_PULL_OVERRIDE) ? "1" : "0") +
       " dp_pullup=" + ((cj & USB_SERIAL_JTAG_DP_PULLUP) ? "1" : "0") + "\n";
  return o;
}

static void usbLibTask(void*) {
  usb_host_config_t hc = {};
  hc.skip_phy_setup = false;
  hc.intr_flags = ESP_INTR_FLAG_LEVEL1;
  esp_err_t ie = usb_host_install(&hc);
  if (ie != ESP_OK) { setUsbState("USB host failed to start (error " + String((int)ie) + ")"); blog("USB: host failed to start (error %d)", (int)ie); vTaskDelete(nullptr); }
  usbHostStarted = true;
  for (;;) {
    uint32_t f = 0;
    usb_host_lib_handle_events(portMAX_DELAY, &f);
    if (f & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
  }
}

static void usbClientTask(void*) {
  usb_host_client_config_t cc = {};
  cc.is_synchronous = false;
  cc.max_num_event_msg = 5;
  cc.async.client_event_callback = uClientCb;
  cc.async.callback_arg = nullptr;
  while (usb_host_client_register(&cc, &uClient) != ESP_OK) vTaskDelay(pdMS_TO_TICKS(100));
  if (usb_host_transfer_alloc(USB_OUT_BYTES, 0, &uOut) != ESP_OK ||
      usb_host_transfer_alloc(USB_IN_BYTES, 0, &uIn) != ESP_OK ||
      usb_host_transfer_alloc(64, 0, &uCtrl) != ESP_OK) {
    setUsbState("not enough memory for USB"); vTaskDelete(nullptr);
  }
  setUsbState("waiting for the printer on the USB port");
  blog("USB: host started - %s", hprtText(REG_READ(BTL_HPRT)).c_str());
  uint32_t lastH = 0xFFFFFFFF, lastChk = 0, lastScan = 0;
  uint8_t tried[8]; int triedN = 0;          // devices already looked at (so a non-printer isn't reopened)
  const uint32_t HMASK = 1u | (1u << 12) | (1u << 2) | (3u << 17);   // connect, power, enabled, speed (not data-line flicker)
  for (;;) {
    usb_host_client_handle_events(uClient, pdMS_TO_TICKS(20));
    if (millis() - lastChk > 300) {                    // log every change on the USB port
      lastChk = millis();
      uint32_t h = REG_READ(BTL_HPRT) & HMASK;
      if (h != lastH) { lastH = h; blog("USB port: %s", hprtText(h).c_str()); }
    }
    if (uNewAddr) {
      uint8_t a = uNewAddr; uNewAddr = 0;
      if (!uDev) {
        bool seen = false;
        for (int i = 0; i < triedN; i++) if (tried[i] == a) seen = true;
        if (!seen && triedN < 8) tried[triedN++] = a;
        blog("USB: new device at address %d", a);
        usbOpen(a);
      }
    }
    if (uGone) { uGone = false; usbClose("unplugged"); }
    // Don't rely only on the "new device" message: every second, ask which devices are connected.
    if (millis() - lastScan > 1000) {
      lastScan = millis();
      uint8_t addrs[8]; int n = 0;
      if (usb_host_device_addr_list_fill(sizeof(addrs), addrs, &n) == ESP_OK) {
        int keep = 0;                            // forget devices that are no longer there
        for (int i = 0; i < triedN; i++) {
          bool present = false;
          for (int k = 0; k < n; k++) if (addrs[k] == tried[i]) present = true;
          if (present) tried[keep++] = tried[i];
        }
        triedN = keep;
        if (!uDev) {
          for (int k = 0; k < n && !uDev; k++) {
            bool seen = false;
            for (int i = 0; i < triedN; i++) if (tried[i] == addrs[k]) seen = true;
            if (seen) continue;
            if (triedN < 8) tried[triedN++] = addrs[k];
            blog("USB: device connected at address %d - opening it", addrs[k]);
            usbOpen(addrs[k]);
          }
        }
      }
    }
  }
}

static void usbStart() {
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
  setUsbState("off: set Tools > USB CDC On Boot > Disabled and upload through the UART port");
#else
  uOutDone = xSemaphoreCreateBinary();
  uRx = xStreamBufferCreate(USB_RX_BYTES, 1);
  // Stacks in external RAM: these tasks never write to flash, and internal RAM is precious.
  xTaskCreatePinnedToCoreWithCaps(usbLibTask, "usblib", 4096, nullptr, 5, nullptr, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  xTaskCreatePinnedToCoreWithCaps(usbClientTask, "usbcli", 6144, nullptr, 5, nullptr, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
#endif
}

// =============================================================== printer communication
// (all of these are called with printerMtx held)

static bool printerAvailable() { return usbIppReady() || printerFound; }

// Opens a conversation with the printer: over USB if it's plugged in, else over Wi-Fi.
static Client* printerConnect(WiFiClient& w, uint32_t timeoutMs) {
  if (usbIppReady()) { curUsb = true; return usbIpp.begin() ? (Client*)&usbIpp : nullptr; }
  curUsb = false;
  if (!printerFound) return nullptr;
  if (!w.connect(printerIP, printerPort, timeoutMs)) return nullptr;
  w.setNoDelay(true);
  return &w;
}

static void setPrinterStatus(int st, const String& reasons) {
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  printerState = st; printerReasons = reasons;
  xSemaphoreGive(dataMtx);
}

static int queryPrinterStatus() {
  static int fails = 0;
  if (!printerAvailable()) return -1;
  WiFiClient w;
  Client* c = printerConnect(w, 4000);
  if (!c) {
    setPrinterStatus(-1, curUsb ? "USB not responding" : "printer not reachable");
    if (!curUsb && ++fails >= 3) { printerFound = false; fails = 0; blog("Lost the printer - searching again"); }
    return -1;
  }
  if (!curUsb) fails = 0;
  std::vector<uint8_t> req;
  ippHeader(req, 0x000B);                                   // Get-Printer-Attributes
  putAttr(req, 0x44, "requested-attributes", "printer-state");
  putAttrMore(req, 0x44, "printer-state-reasons");
  put8(req, 0x03);
  sendPostHeaders(*c, false, req.size());
  writeAll(*c, req.data(), req.size());
  std::vector<uint8_t> body;
  int code = readHttpResponse(*c, body);
  c->stop();
  if (code != 200) { setPrinterStatus(-1, curUsb ? "no answer over USB" : "no answer"); return -1; }
  String txt;
  parseIpp(body, &txt);
  int ps = attrVal(txt, "printer-state").toInt();
  setPrinterStatus(ps, attrVal(txt, "printer-state-reasons"));
  return ps;
}

static bool printerReady() {
  if (!printerAvailable()) return false;
  if (lastSubmitMs && millis() - lastSubmitMs < 8000) return false;   // let the last job get going
  return queryPrinterStatus() == 3;                                     // 3 = idle
}

static uint16_t sendBufferJob(const Job& j, bool withScaling = true) {
  WiFiClient w;
  Client* c = printerConnect(w, 5000);
  if (!c) return 0xFFFE;
  Client& pc = *c;
  uint32_t t0 = millis();
  std::vector<uint8_t> h;
  buildPrintJob(h, j.name, withScaling);
  sendPostHeaders(pc, false, h.size() + j.len);
  writeAll(pc, h.data(), h.size());
  writeAll(pc, j.data, j.len);
  uint32_t t1 = millis();
  std::vector<uint8_t> body;
  int code = readHttpResponse(pc, body);
  pc.stop();
  lastSendMs = t1 - t0; lastAnswerMs = millis() - t1;
  if (code != 200) return 0xFFFF;
  uint16_t st = parseIpp(body, nullptr);
  if (st >= 0x0400 && st < 0x0500 && withScaling && scaleFill) {   // printer didn't like print-scaling: send it plain
    blog("Printer refused the borderless paper setting (%s); turning that option off", ippStatusName(st).c_str());
    scaleFill = false; saveScaleFill = true;            // saved from loop(): this task's stack lives in external RAM
    return sendBufferJob(j, false);
  }
  if (st < 0x0100) lastSubmitMs = millis();
  return st;
}

// Streams a photo straight from the camera's data connection into a Print-Job.
// 'pre' holds bytes already read from the camera.
static uint16_t streamJob(Client& d, const uint8_t* pre, size_t preLen, bool eof, const String& name, size_t& total) {
  // Collect the camera's data into big blocks before forwarding: lots of tiny network
  // packets make the printer's connection crawl.
  const size_t BLK = 64 * 1024;
  static uint8_t* buf = nullptr;
  if (!buf) buf = (uint8_t*)bigAlloc(BLK);
  if (!buf) return 0xFFFE;
  total = preLen;
  WiFiClient w;
  Client* c = printerConnect(w, 5000);
  if (!c) return 0xFFFE;
  Client& pc = *c;
  uint32_t t0 = millis();
  sendPostHeaders(pc, true, 0);
  std::vector<uint8_t> h;
  buildPrintJob(h, name);
  sendChunk(pc, h.data(), h.size());
  for (size_t o = 0; o < preLen; o += BLK) sendChunk(pc, pre + o, min(BLK, preLen - o));
  size_t fill = 0;
  while (!eof) {
    int n = readSome(d, buf + fill, BLK - fill, 30000);
    if (n < 0) { pc.stop(); return 0xFFFD; }
    if (n == 0) break;
    fill += n; total += n;
    if (fill == BLK) { sendChunk(pc, buf, fill); fill = 0; }
  }
  if (fill) sendChunk(pc, buf, fill);
  writeAll(pc, (const uint8_t*)"0\r\n\r\n", 5);
  uint32_t t1 = millis();
  std::vector<uint8_t> body;
  int code = readHttpResponse(pc, body);
  pc.stop();
  lastSendMs = t1 - t0; lastAnswerMs = millis() - t1;
  if (code != 200) return 0xFFFF;
  uint16_t st = parseIpp(body, nullptr);
  if (st < 0x0100) lastSubmitMs = millis();
  return st;
}

static bool discoverPrinter() {
  int n = MDNS.queryService("ipp", "tcp");
  int pick = -1;
  for (int i = 0; i < n; i++) {
    String ty = MDNS.txt(i, "ty"), host = MDNS.hostname(i);
    if (ty.indexOf("SELPHY") >= 0 || host.indexOf("SELPHY") >= 0) { pick = i; break; }   // prefer a SELPHY if there are several
    if (pick < 0) pick = i;
  }
  if (pick < 0) return false;
  IPAddress ip;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ip = MDNS.address(pick);
#else
  ip = MDNS.IP(pick);
#endif
  String rp = MDNS.txt(pick, "rp");
  String ty = MDNS.txt(pick, "ty");
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  printerIP = ip;
  printerPort = MDNS.port(pick);
  if (rp.length()) printerPath = rp;
  printerName = ty.length() ? ty : MDNS.hostname(pick);
  xSemaphoreGive(dataMtx);
  printerFound = true;
  return true;
}

// =============================================================== print queue task
static bool retryable(uint16_t st) { return st == 0x0506 || st == 0x0507 || st == 0xFFFE; }

static void enqueueJob(uint8_t* data, size_t len, const String& name, int rating) {
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  jobs.push_back({ data, len, name, rating, 0 });
  xSemaphoreGive(dataMtx);
}

static void printTask(void*) {
  uint32_t lastPoll = 0, lastDiscover = 0;
  for (;;) {
    if (!printerAvailable()) {
      if (lastDiscover == 0 || millis() - lastDiscover > 15000) {
        lastDiscover = millis();
        if (discoverPrinter()) blog("Found printer: %s (%s)", printerName.c_str(), printerIP.toString().c_str());
      }
      vTaskDelay(pdMS_TO_TICKS(500));
      continue;
    }
    if (queueLen() > 0) {
      if (xSemaphoreTake(printerMtx, pdMS_TO_TICKS(500)) == pdTRUE) {
        if (printerReady()) {
          xSemaphoreTake(dataMtx, portMAX_DELAY);
          Job j = jobs.front();
          xSemaphoreGive(dataMtx);
          uint16_t st = sendBufferJob(j);
          bool done = true;
          if (st < 0x0100) {
            blog("Printing %s (%s) - sent over %s in %.1f s, printer took %.1f s to accept", j.name.c_str(), fmtSize(j.len).c_str(), curUsb ? "USB" : "Wi-Fi", lastSendMs / 1000.0, lastAnswerMs / 1000.0);
            countPrint();
            notePhoto(j.name.c_str(), "from queue, %s", curUsb ? "USB" : "Wi-Fi");
            if (printMode == 1) addPrinted(j.name);
          } else if (retryable(st) && j.tries < 5) {
            done = false;
            blog("Printer not ready for %s (%s) - will retry", j.name.c_str(), ippStatusName(st).c_str());
          } else {
            blog("Could not print %s: %s", j.name.c_str(), ippStatusName(st).c_str());
          }
          xSemaphoreTake(dataMtx, portMAX_DELAY);
          if (done) { free(jobs.front().data); jobs.pop_front(); }
          else jobs.front().tries++;
          xSemaphoreGive(dataMtx);
        }
        xSemaphoreGive(printerMtx);
      }
      vTaskDelay(pdMS_TO_TICKS(2000));
    } else {
      if (millis() - lastPoll > 10000) {
        if (xSemaphoreTake(printerMtx, pdMS_TO_TICKS(50)) == pdTRUE) { queryPrinterStatus(); xSemaphoreGive(printerMtx); }
        lastPoll = millis();
      }
      vTaskDelay(pdMS_TO_TICKS(300));
    }
  }
}

// =============================================================== JPEG metadata (rating, orientation, size)
// Returns -2 if not a JPEG, -1 if more data is needed, else the offset just past the start-of-scan
// header (everything the decoder needs before the compressed image data).
static long jpegHeaderEnd(const uint8_t* b, size_t n) {
  if (n < 2) return -1;
  if (b[0] != 0xFF || b[1] != 0xD8) return -2;
  size_t p = 2;
  while (true) {
    if (p + 4 > n) return -1;
    if (b[p] != 0xFF) return -2;
    uint8_t m = b[p + 1];
    if (m == 0xFF) { p++; continue; }
    if ((m >= 0xD0 && m <= 0xD7) || m == 0x01) { p += 2; continue; }
    size_t L = (b[p + 2] << 8) | b[p + 3];
    if (m == 0xDA) return (p + 2 + L <= n) ? (long)(p + 2 + L) : -1;
    p += 2 + L;
  }
}

// Image size from the frame header. baseline = false for progressive JPEGs (not supported by the decoder).
static bool jpegSof(const uint8_t* b, size_t n, int& w, int& h, bool& baseline) {
  size_t p = 2;
  while (p + 4 <= n) {
    if (b[p] != 0xFF) return false;
    uint8_t m = b[p + 1];
    if (m == 0xFF) { p++; continue; }
    if ((m >= 0xD0 && m <= 0xD7) || m == 0x01) { p += 2; continue; }
    size_t L = (b[p + 2] << 8) | b[p + 3];
    if (m >= 0xC0 && m <= 0xCF && m != 0xC4 && m != 0xC8 && m != 0xCC) {
      if (p + 9 > n) return false;
      h = (b[p + 5] << 8) | b[p + 6];
      w = (b[p + 7] << 8) | b[p + 8];
      baseline = (m == 0xC0 || m == 0xC1);
      return true;
    }
    if (m == 0xDA) return false;
    p += 2 + L;
  }
  return false;
}

// EXIF IFD0: tag 0x4746 = Rating, 0x0112 = Orientation
static int exifRating(const uint8_t* t, size_t len, int* orient) {
  if (len < 8) return -1;
  bool le = t[0] == 'I';
  auto u16 = [&](size_t o) -> uint32_t {
    if (o + 2 > len) return 0;
    return le ? (t[o] | (t[o + 1] << 8)) : ((t[o] << 8) | t[o + 1]);
  };
  auto u32 = [&](size_t o) -> uint32_t {
    if (o + 4 > len) return 0;
    return le ? (t[o] | (t[o + 1] << 8) | (t[o + 2] << 16) | ((uint32_t)t[o + 3] << 24))
              : (((uint32_t)t[o] << 24) | (t[o + 1] << 16) | (t[o + 2] << 8) | t[o + 3]);
  };
  size_t ifd = u32(4);
  if (ifd + 2 > len) return -1;
  uint32_t cnt = u16(ifd);
  int rating = -1;
  for (uint32_t i = 0; i < cnt; i++) {
    size_t e = ifd + 2 + i * 12;
    if (e + 12 > len) break;
    uint32_t tag = u16(e);
    if (tag == 0x4746) rating = (int)u16(e + 8);
    else if (tag == 0x0112 && orient) *orient = (int)u16(e + 8);
  }
  return rating;
}

static int xmpRating(const char* s, size_t len) {          // <xmp:Rating>3</xmp:Rating> or xmp:Rating="3"
  const char* key = "xmp:Rating";
  const size_t kl = 10;
  for (size_t i = 0; i + kl < len; i++) {
    if (memcmp(s + i, key, kl) != 0) continue;
    size_t j = i + kl;
    for (int k = 0; k < 4 && j < len; k++, j++) {
      char ch = s[j];
      if (ch == '=' || ch == '"' || ch == '>' || ch == '\'' || ch == ' ') continue;
      break;
    }
    if (j < len) {
      if (s[j] == '-') return 0;
      if (s[j] >= '0' && s[j] <= '5') return s[j] - '0';
    }
  }
  return -1;
}

static int findMeta(const uint8_t* b, size_t n, int* orient) {
  int rating = -1;
  size_t p = 2;
  while (p + 4 <= n && b[p] == 0xFF) {
    uint8_t m = b[p + 1];
    if (m == 0xFF) { p++; continue; }
    if (!((m >= 0xE0 && m <= 0xEF) || m == 0xFE)) break;
    size_t L = (b[p + 2] << 8) | b[p + 3];
    size_t segStart = p + 4;
    size_t segLen = L >= 2 ? L - 2 : 0;
    if (segStart + segLen > n) segLen = n > segStart ? n - segStart : 0;
    const uint8_t* s = b + segStart;
    if (m == 0xE1 && segLen >= 14 && memcmp(s, "Exif\0\0", 6) == 0) {
      int r = exifRating(s + 6, segLen - 6, orient);
      if (r >= 0) rating = r;
    } else if (m == 0xE1 && segLen > 29 && memcmp(s, "http://ns.adobe.com/xap/1.0/", 28) == 0) {
      int r = xmpRating((const char*)s, segLen);
      if (r >= 0) rating = r;
    }
    p += 2 + L;
  }
  return rating;
}

// =============================================================== camera data link
// The camera's photo comes in over the FTP data connection. The Wi-Fi stack's normal sockets only
// offer the camera a 5.7 KB receive window, and the camera's power saving makes every round trip slow,
// so the transfer crawled at 80-200 KB/s. This connection uses the stack's raw TCP interface instead,
// advertises a 32 KB window, acknowledges every segment at once, and parks the data in a 1 MB buffer
// in PSRAM for the decoder to read at its own pace.
#include "lwip/tcp.h"
#include "lwip/tcpip.h"
#include "lwip/priv/tcp_priv.h"

#define CAM_WND   32768
#define CAM_RING  (1024 * 1024)

class CamData : public Client {
 public:
  void listen(uint16_t port) {
    ringMem = (uint8_t*)heap_caps_malloc(CAM_RING, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ring = ringMem ? xStreamBufferCreateStatic(CAM_RING, 1, ringMem, &ringCtl) : nullptr;
    if (!ring) { blog("Camera link: no memory for the receive buffer"); return; }
    LOCK_TCPIP_CORE();
    tcp_pcb* l = tcp_new();
    if (l && tcp_bind(l, IP_ANY_TYPE, port) == ERR_OK) {
      l = tcp_listen_with_backlog(l, 2);
      tcp_arg(l, this);
      tcp_accept(l, onAccept);
      lst = l;
    } else if (l) tcp_abort(l);
    UNLOCK_TCPIP_CORE();
    if (!lst) blog("Camera link: couldn't listen on port %u", port);
    else blog("Camera data link listening on port %u (window %u KB)", port, CAM_WND / 1024);
  }

  // Waits for the camera to connect to the data port.
  bool open(uint32_t timeoutMs) {
    uint32_t t = millis();
    while (!accepted) {
      if (millis() - t > timeoutMs) { blog("Camera data link: the camera didn't connect to port %u (seen %lu connections so far)", FTP_PASV_PORT, (unsigned long)accepts); return false; }
      delay(2);
    }
    return true;
  }

  int available() override { return ring ? (int)xStreamBufferBytesAvailable(ring) : 0; }
  int read(uint8_t* buf, size_t len) override {
    if (!ring) return 0;
    int n = (int)xStreamBufferReceive(ring, buf, len, 0);
    if (n > 0 && clamped) reopenWindow();
    return n;
  }
  int read() override { uint8_t b; return read(&b, 1) == 1 ? b : -1; }
  int peek() override { return -1; }
  uint8_t connected() override { return (pcb != nullptr && !closed) || available() > 0; }
  operator bool() override { return connected(); }
  void flush() override {}
  size_t write(uint8_t) override { return 0; }
  size_t write(const uint8_t*, size_t) override { return 0; }
  int connect(IPAddress, uint16_t) override { return 0; }
  int connect(const char*, uint16_t) override { return 0; }

  // Closes a connection politely. lwIP sends a reset instead of a close when the receive window
  // isn't at its default (it takes that as "the application didn't read everything"), so put the
  // default back first.
  static void closePcb(tcp_pcb* p) {
    tcp_arg(p, nullptr); tcp_recv(p, nullptr); tcp_err(p, nullptr);
    p->rcv_wnd = TCP_WND_MAX(p);
    p->rcv_ann_wnd = TCP_WND_MAX(p);
    if (tcp_close(p) != ERR_OK) tcp_abort(p);
  }

  // Closes the data connection and empties the buffer.
  void stop() override {
    LOCK_TCPIP_CORE();
    if (pcb) { closePcb(pcb); pcb = nullptr; }
    UNLOCK_TCPIP_CORE();
    accepted = false; closed = false; clamped = false;
    if (ring) xStreamBufferReset(ring);
  }

 private:
  tcp_pcb* lst = nullptr;
  tcp_pcb* volatile pcb = nullptr;
  StreamBufferHandle_t ring = nullptr;
  StaticStreamBuffer_t ringCtl;
  uint8_t* ringMem = nullptr;
  volatile bool accepted = false, closed = false, clamped = false;
  volatile uint32_t accepts = 0, inBytes = 0;

  size_t windowNow() { size_t f = ring ? xStreamBufferSpacesAvailable(ring) : 0; return f < CAM_WND ? f : CAM_WND; }

  // Tell the camera how much it may send. Called with the stack locked (from its thread or ours).
  void setWindow(tcp_pcb* p) {
    size_t w = windowNow();
    clamped = w < CAM_WND;
    p->rcv_wnd = (tcpwnd_size_t)w;
    p->rcv_ann_wnd = (tcpwnd_size_t)w;
    p->rcv_ann_right_edge = p->rcv_nxt + p->rcv_ann_wnd;
    tcp_set_flags(p, TF_ACK_NOW);
  }

  void reopenWindow() {                                   // the decoder freed space: let the camera continue
    if (windowNow() < CAM_WND) return;
    LOCK_TCPIP_CORE();
    if (pcb && !closed) { setWindow(pcb); tcp_output(pcb); }
    UNLOCK_TCPIP_CORE();
  }

  static err_t onAccept(void* arg, tcp_pcb* np, err_t err) {
    CamData* self = (CamData*)arg;
    if (err != ERR_OK || !np) { blog("Camera data link: accept error %d", (int)err); return ERR_VAL; }
    self->accepts++;
    if (self->pcb) { blog("Camera data link: refused a second connection"); tcp_abort(np); return ERR_ABRT; }   // one at a time
    self->inBytes = 0;
    if (self->ring) xStreamBufferReset(self->ring);
    self->closed = false; self->clamped = false;
    tcp_arg(np, self);
    tcp_recv(np, onRecv);
    tcp_err(np, onErr);
    tcp_nagle_disable(np);
    self->setWindow(np);
    self->pcb = np;
    self->accepted = true;
    return ERR_OK;
  }

  static err_t onRecv(void* arg, tcp_pcb* p, pbuf* pb, err_t err) {
    CamData* self = (CamData*)arg;
    if (!pb) {                                            // the camera closed its side: the file is complete
      self->closed = true;
      if (self->inBytes < 1024) blog("Camera data link: camera closed after %lu bytes", (unsigned long)self->inBytes);
      self->pcb = nullptr;
      closePcb(p);
      return ERR_OK;
    }
    if (err != ERR_OK) { pbuf_free(pb); return ERR_OK; }
    if (xStreamBufferSpacesAvailable(self->ring) < pb->tot_len) return ERR_MEM;   // keep it, lwIP offers it again
    for (pbuf* q = pb; q; q = q->next) xStreamBufferSend(self->ring, q->payload, q->len, 0);
    self->inBytes += pb->tot_len;
    pbuf_free(pb);
    self->setWindow(p);
    return ERR_OK;
  }

  static void onErr(void* arg, err_t e) {                 // connection reset or dropped; the pcb is gone already
    CamData* self = (CamData*)arg;
    if (!self) return;
    self->pcb = nullptr;
    self->closed = true;
    blog("Camera data link: connection lost (%d)", (int)e);
  }
};
CamData camData;

// =============================================================== FTP server (for the camera)
static void ftpSend(WiFiClient& c, const String& s) { c.print(s + "\r\n"); }

static size_t drain(Client& d) {
  static uint8_t tmp[4096];
  size_t t = 0;
  while (true) { int n = readSome(d, tmp, sizeof(tmp), 30000); if (n <= 0) break; t += n; }
  return t;
}

// =============================================================== shrink to print size
// The SELPHY only reads photo data at roughly 45 KB/s, so the fewer bytes it gets, the sooner it
// prints. A SELPHY 4x6 (postcard) print is 1872x1248 dots: 300 dpi plus the bleed it prints past the
// paper's edges for borderless prints. Sending exactly that size means the printer neither scales
// nor leaves a margin.
// The photo is decoded as it arrives, averaged down to that size, turned upright using the EXIF
// orientation, and re-encoded.
#define PRINT_LONG  1872
#define PRINT_SHORT 1248


static size_t shrinkIn(JDEC* jd, uint8_t* buf, size_t nd) {
  ShrinkJob* j = (ShrinkJob*)jd->device;
  size_t got = 0;
  uint8_t tmp[512];
  while (got < nd) {
    if (j->pos < j->hn) {
      size_t k = min(nd - got, j->hn - j->pos);
      if (buf) memcpy(buf + got, j->hb + j->pos, k);
      j->pos += k; got += k;
      continue;
    }
    if (j->eof || !j->d) break;
    uint8_t* dst = buf ? buf + got : tmp;
    size_t want = buf ? nd - got : min(nd - got, sizeof(tmp));
    uint32_t w0 = millis();
    int n = readSome(*j->d, dst, want, 30000);
    j->waitMs += millis() - w0;
    if (n <= 0) { if (n < 0) j->err = true; j->eof = true; break; }
    got += n; j->sockBytes += n;
  }
  return got;
}

__attribute__((optimize("O2")))
static void shrinkFinalizeRow(ShrinkJob* j, int oy) {
  uint32_t* row = j->sums + (size_t)(oy % j->W) * j->tw * 3;
  const uint32_t D = (uint32_t)j->cw * j->ch;          // total weight of every output pixel
  const uint32_t half = D / 2;
  const uint64_t inv = ((uint64_t)1 << 40) / D;         // divide by multiplying: (v * inv) >> 40
  #define SHRINK_DIV(v) ((uint8_t)(((uint64_t)((v) + half) * inv) >> 40))
  for (int ox = 0; ox < j->tw; ox++) {
    const uint32_t* sm = row + ox * 3;
    int dx, dy;
    switch (j->bandOut ? 1 : j->orient) {                            // fast path: no rotation
      case 3:  dx = j->tw - 1 - ox; dy = j->th - 1 - oy; break;   // upside down
      case 6:  dx = j->th - 1 - oy; dy = ox;               break;   // rotate 90 clockwise
      case 8:  dx = oy;             dy = j->tw - 1 - ox;   break;   // rotate 90 counter-clockwise
      default: dx = ox;             dy = oy;
    }
    if (j->bandOut) {                                                  // fast path: R, G, B into the band
      uint8_t* f = j->bandOut + ((size_t)(oy % j->bandRows) * j->fw + dx) * 3;
      f[0] = SHRINK_DIV(sm[0]);
      f[1] = SHRINK_DIV(sm[1]);
      f[2] = SHRINK_DIV(sm[2]);
    } else {
      uint8_t* f = j->frame + ((size_t)dy * j->fw + dx) * 3;          // old encoder wants B, G, R
      f[0] = SHRINK_DIV(sm[2]);
      f[1] = SHRINK_DIV(sm[1]);
      f[2] = SHRINK_DIV(sm[0]);
    }
  }
  #undef SHRINK_DIV
  memset(row, 0, (size_t)j->tw * 3 * sizeof(uint32_t));
  if (j->bandOut && !j->encFail && ((oy + 1) % j->bandRows == 0 || oy == j->th - 1)) {
    if (oy == j->th - 1) {                                             // last band may be partial: repeat the last row
      int have = oy % j->bandRows + 1;
      for (int r = have; r < j->bandRows; r++)
        memcpy(j->bandOut + (size_t)r * j->fw * 3, j->bandOut + (size_t)(have - 1) * j->fw * 3, (size_t)j->fw * 3);
    }
    int b = j->bandCount++;
    xSemaphoreGive(j->bandFull[b & 1]);                                // encoder task: this band is ready
    if (b + 1 < j->bandsTotal) {                                       // move on to the other band once it's free
      xSemaphoreTake(j->bandFree[(b + 1) & 1], portMAX_DELAY);
      j->bandOut = j->bands[(b + 1) & 1];
    }
  }
}

// The encoder's output lands in the job's output buffer; the sender streams it from there.
static void encOut(void* ctx, const uint8_t* d, size_t n) {
  ShrinkJob* j = (ShrinkJob*)ctx;
  if (j->encFail) return;
  if (j->outLen + n > j->outCap) {
    size_t nc = j->outCap * 2; while (nc < j->outLen + n) nc *= 2;
    uint8_t* nb = (uint8_t*)bigRealloc(j->out, nc);
    if (!nb) { j->encFail = true; return; }
    j->out = nb; j->outCap = nc;
  }
  memcpy(j->out + j->outLen, d, n);
  j->outLen += n;
}

// Encoder task: takes bands as the resampler finishes them, encodes each, and the bytes become
// available to the sender right away.
// Builds the color curves from the settings. Runs when settings change; cheap.
static void buildColor() {
  // Warmth bends the red and blue midtones (a gamma per channel) so white stays white and black stays black.
  float gam[3] = { 1.0f - 0.025f * colWarm, 1.0f, 1.0f + 0.04f * colWarm };   // R, G, B; cooler = blue up, red down
  float a = 0.15f * colCon;                                                  // S-curve strength (stays monotonic)
  for (int c = 0; c < 3; c++)
    for (int v = 0; v < 256; v++) {
      float x = v / 255.0f;
      x = x - a * sinf(6.2831853f * x) / 6.2831853f;                         // contrast
      if (x > 0.0f) x = powf(x, gam[c]);                                     // warmth
      int o = (int)lroundf(x * 255.0f);
      colLut[c][v] = (uint8_t)(o < 0 ? 0 : o > 255 ? 255 : o);
    }
  colSatK = 256 + 26 * colSat;                                               // +10% per step
  colOn = colWarm || colCon || colSat;
}

// Applies the color settings to packed pixels in place (R,G,B order, or B,G,R when bgr).
static void colorAdjust(uint8_t* p, size_t pixels, bool bgr) {
  if (!colOn) return;
  const int ri = bgr ? 2 : 0, bi = bgr ? 0 : 2;
  const int k = colSatK;
  for (size_t i = 0; i < pixels; i++, p += 3) {
    int r = colLut[0][p[ri]], g = colLut[1][p[1]], b = colLut[2][p[bi]];
    if (k != 256) {
      int y = (77 * r + 150 * g + 29 * b) >> 8;
      r = y + (((r - y) * k) >> 8); g = y + (((g - y) * k) >> 8); b = y + (((b - y) * k) >> 8);
      r = r < 0 ? 0 : r > 255 ? 255 : r; g = g < 0 ? 0 : g > 255 ? 255 : g; b = b < 0 ? 0 : b > 255 ? 255 : b;
    }
    p[ri] = r; p[1] = g; p[bi] = b;
  }
}

static void shrinkEncTask(void* arg) {
  ShrinkJob* j = (ShrinkJob*)arg;
  BtlJpegEnc* enc = (BtlJpegEnc*)j->enc;
  for (int b = 0; b < j->bandsTotal; b++) {
    xSemaphoreTake(j->bandFull[b & 1], portMAX_DELAY);
    if (!j->encFail) {
      uint32_t t = millis();
      int rows = j->th - b * j->bandRows; if (rows > j->bandRows) rows = j->bandRows;
      colorAdjust(j->bands[b & 1], (size_t)rows * j->tw, false);
      screenThumbRows(j->bands[b & 1], b * j->bandRows, rows, false);
      enc->addRows(j->bands[b & 1], rows);
      if (b + 1 == j->bandsTotal) screenThumbEnd();
      if (b + 1 == j->bandsTotal) enc->end();
      j->tEnc += millis() - t;
      if (j->outReady) xSemaphoreGive(j->outReady);
    }
    xSemaphoreGive(j->bandFree[b & 1]);
  }
  j->tEncDone = millis();
  j->encExited = true;
  if (j->outReady) xSemaphoreGive(j->outReady);
  vTaskDelete(nullptr);
}

// Sender task: feeds the printer from the output buffer at the printer's own pace, so neither the
// resampler nor the encoder ever waits for it.
static void shrinkSendTask(void* arg) {
  ShrinkJob* j = (ShrinkJob*)arg;
  while (true) {
    xSemaphoreTake(j->outReady, pdMS_TO_TICKS(50));
    size_t have = j->outLen;
    bool last = j->encExited;
    if (have > j->sinkSent && !j->sinkFail && !j->encFail && (last || have - j->sinkSent >= 8 * 1024)) {
      if (!j->tFirstSend) j->tFirstSend = millis();
      j->sink(j->sinkCtx, j->out + j->sinkSent, have - j->sinkSent);
      j->sinkSent = have; j->sendWrites++;
    }
    if (last && (j->sinkSent >= j->outLen || j->sinkFail || j->encFail)) break;
  }
  j->tSendDone = millis();
  j->sendExited = true;
  vTaskDelete(nullptr);
}

static void shrinkFinalizeUntil(ShrinkJob* j, int limit) {
  if (limit > j->th) limit = j->th;
  while (j->base < limit) shrinkFinalizeRow(j, j->base++);
}

// One decoded source row -> shrink horizontally, add into the output rows it covers, and finish
// any output rows that can't receive anything more. Works row by row, in order, so memory access
// stays sequential (the averaging buffers live in fast internal RAM).
__attribute__((optimize("O2")))
static void shrinkSourceRow(ShrinkJob* j, int y, const uint8_t* px) {
  if (j->oy0[y] == 0xFFFF) return;                      // outside the crop
  const int n3 = j->tw * 3;
  uint32_t* const h = j->hrow;
  memset(h, 0, (size_t)n3 * sizeof(uint32_t));
  const uint16_t* ox0 = j->ox0 + j->cx0;
  const uint16_t* wx0 = j->wx0 + j->cx0;
  const uint32_t tw = j->tw;
  px += (size_t)j->cx0 * 3;
  for (int x = 0; x < j->cw; x++, px += 3) {           // only the cropped part of the row
    uint32_t wa = wx0[x];
    uint32_t* a = h + ox0[x] * 3;
    uint32_t r = px[0], g = px[1], b = px[2];
    a[0] += r * wa; a[1] += g * wa; a[2] += b * wa;
    uint32_t wb = tw - wa;
    if (wb) { a[3] += r * wb; a[4] += g * wb; a[5] += b * wb; }
  }
  int oy = j->oy0[y];
  shrinkFinalizeUntil(j, oy);                           // rows above this one are complete
  uint32_t wa = j->wy0[y], wb = j->th - wa;
  uint32_t* ra = j->sums + (size_t)(oy % j->W) * n3;
  if (wb) {
    uint32_t* rb = j->sums + (size_t)((oy + 1) % j->W) * n3;
    for (int i = 0; i < n3; i++) { uint32_t v = h[i]; ra[i] += v * wa; rb[i] += v * wb; }
  } else {
    for (int i = 0; i < n3; i++) ra[i] += h[i] * wa;
  }
}

static void shrinkFlushBand(ShrinkJob* j) {
  if (j->bandTop < 0) return;
  for (int k = 0; k < j->bandH && j->bandTop + k < j->sh; k++)
    shrinkSourceRow(j, j->bandTop + k, j->band + (size_t)k * j->sw * 3);
  j->bandTop = -1;
}

// Decoder output: one block of pixels. Collect a full row of blocks, then process it.
static int shrinkOut(JDEC* jd, void* bmp, JRECT* r) {
  ShrinkJob* j = (ShrinkJob*)jd->device;
  if ((int)r->top != j->bandTop) {
    shrinkFlushBand(j);
    j->bandTop = r->top;
    vTaskDelay(1);                                      // let the web page and printer tasks run
  }
  int w = r->right - r->left + 1;
  int cw = min(w, j->sw - (int)r->left);                // ignore anything past the scaled width
  if (cw <= 0) return 1;
  const uint8_t* src = (const uint8_t*)bmp;
  for (int y = r->top; y <= r->bottom; y++) {
    int k = y - r->top;
    if (k >= j->bandH) break;
    memcpy(j->band + ((size_t)k * j->sw + r->left) * 3, src + (size_t)k * w * 3, (size_t)cw * 3);
  }
  return 1;
}

static size_t shrinkJpgOut(void* arg, size_t index, const void* data, size_t len) {
  ShrinkJob* j = (ShrinkJob*)arg;
  if (j->outFail) return len;
  if (j->outLen + len > j->outCap) {
    size_t nc = j->outCap ? j->outCap * 2 : 512 * 1024;
    while (nc < j->outLen + len) nc *= 2;
    uint8_t* nb = (uint8_t*)bigRealloc(j->out, nc);
    if (!nb) { j->outFail = true; return len; }
    j->out = nb; j->outCap = nc;
  }
  memcpy(j->out + j->outLen, data, len);
  j->outLen += len;
  return len;
}

// Pulls the rest of the photo into PSRAM so the fast decoder can work on the whole file.
// Leaves room for the decoder, the encoder and the print queue; a file that doesn't fit is
// passed on to the streaming path with whatever was received so far.
static bool shrinkReceiveWhole(ShrinkJob* j) {
  if (!j->d || j->eof) return true;                     // already complete in memory
  const size_t KEEP = 2560 * 1024;                      // leave room for the decoder, encoder and queue
  size_t room = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  if (room < KEEP + 512 * 1024) return false;
  size_t cap = min(max(j->hn * 2, (size_t)(1536 * 1024)), room - KEEP);   // grows if the file is bigger
  uint8_t* buf = (uint8_t*)heap_caps_malloc(cap, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!buf) return false;
  memcpy(buf, j->hb, j->hn);
  size_t n = j->hn;
  bool full = false;
  uint32_t t0 = millis();
  while (true) {
    if (n >= cap) {                                     // bigger than expected: grow while there's room
      size_t more = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
      size_t nc = cap + min((size_t)(1024 * 1024), more > KEEP + cap ? more - KEEP - cap : (size_t)0);
      uint8_t* nb = nc > cap ? (uint8_t*)heap_caps_realloc(buf, nc, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) : nullptr;
      if (!nb) { full = true; break; }
      buf = nb; cap = nc;
    }
    int got = readSome(*j->d, buf + n, cap - n, 30000);
    if (got < 0) { j->err = true; j->eof = true; break; }
    if (got == 0) { j->eof = true; break; }
    if (!j->rxFirstMs) j->rxFirstMs = millis();
    n += got; j->rxBytes += got; j->rxLastMs = millis();
  }
  j->tRecv = millis() - t0;
  j->whole = buf; j->wholeCap = cap; j->hb = buf; j->hn = n; j->pos = 0;
  j->sockBytes = 0;                                     // hn now covers everything received so far
  if (full) return false;                               // too big for the fast path: streaming path continues from the socket
  return !j->err;
}

static void shrinkFreeWhole(ShrinkJob* j) { free(j->whole); j->whole = nullptr; }
static void shrinkFreeBands(ShrinkJob* j) {
  for (int i = 0; i < 2; i++) {
    free(j->bands[i]); j->bands[i] = nullptr;
    if (j->bandFull[i]) { vSemaphoreDelete(j->bandFull[i]); j->bandFull[i] = nullptr; }
    if (j->bandFree[i]) { vSemaphoreDelete(j->bandFree[i]); j->bandFree[i] = nullptr; }
  }
}

static bool shrinkPrepareResampler(ShrinkJob* j);       // below
static void shrinkFreeWork(ShrinkJob* j);

// Fast path: Espressif's codec decodes the file (in memory) a block row at a time straight into the
// box resampler; finished output rows are encoded band by band. No rotation: the printer turns the
// sheet to fit, so a photo stored sideways prints the same either way.
// Returns false (with nothing consumed) when the standard path should be used instead.
static bool shrinkFast(ShrinkJob* j, uint32_t t0) {
  if (j->iw % 8 || j->ih % 8) return false;
  j->s = 0; j->sw = j->iw; j->sh = j->ih;
  int cwF = min(j->iw, max(j->tw, (int)lroundf(j->tw / j->kScale)));
  int chF = min(j->ih, max(j->th, (int)lroundf(j->th / j->kScale)));
  j->cw = cwF; j->ch = chF; j->cx0 = (j->sw - j->cw) / 2; j->cy0 = (j->sh - j->ch) / 2;
  j->fw = j->tw; j->fh = j->th;

  // The resampler's accumulators go first so they get internal RAM (they're hit for every pixel);
  // the codec's own buffers come after and may spill into external RAM instead.
  if (!shrinkPrepareResampler(j)) { shrinkFreeWork(j); blog("Fast resizer: not enough memory for the resampler"); return false; }
  jpeg_dec_config_t dc = {};
  dc.output_type = JPEG_PIXEL_FORMAT_RGB888; dc.rotate = JPEG_ROTATE_0D; dc.block_enable = true;
  jpeg_dec_handle_t dec = nullptr;
  if (jpeg_dec_open(&dc, &dec) != JPEG_ERR_OK || !dec) { shrinkFreeWork(j); blog("Fast decoder: couldn't start"); return false; }
  jpeg_dec_io_t io = {};
  io.inbuf = (uint8_t*)j->hb; io.inbuf_len = (int)j->hn; io.inbuf_remain = (int)j->hn;
  jpeg_dec_header_info_t hi = {};
  int blkLen = 0, count = 0;
  if (jpeg_dec_parse_header(dec, &io, &hi) != JPEG_ERR_OK || hi.width != j->iw || hi.height != j->ih ||
      jpeg_dec_get_outbuf_len(dec, &blkLen) != JPEG_ERR_OK || blkLen < j->iw * 3 ||
      jpeg_dec_get_process_count(dec, &count) != JPEG_ERR_OK || count < 1) {
    blog("Fast decoder: can't read this file (reports %ux%u, expected %dx%d)", hi.width, hi.height, j->iw, j->ih);
    jpeg_dec_close(dec); shrinkFreeWork(j); return false;
  }
  uint8_t* blk = (uint8_t*)heap_caps_aligned_alloc(16, blkLen, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (!blk) { blog("Fast decoder: no memory for a block (%d bytes)", blkLen); jpeg_dec_close(dec); shrinkFreeWork(j); return false; }
  io.outbuf = blk;

  screenThumbBegin(j->tw, j->th, j->orient);
  BtlJpegEnc* enc = new BtlJpegEnc();
  j->outCap = 512 * 1024;
  j->out = (uint8_t*)bigAlloc(j->outCap);
  if (!enc || !j->out || !enc->begin(j->fw, j->fh, j->quality, encOut, j)) {
    blog("Encoder: couldn't start for %dx%d", j->fw, j->fh);
    delete enc; free(j->out); j->out = nullptr; j->outCap = 0; free(blk); jpeg_dec_close(dec); shrinkFreeWork(j); return false;
  }
  j->bandRows = 16; j->bandBytes = (size_t)j->fw * 16 * 3;
  int bs = (int)j->bandBytes;
  j->bandsTotal = (j->th + j->bandRows - 1) / j->bandRows;
  j->bandCount = 0; j->encFail = false; j->encExited = false; j->sinkSent = 0; j->sinkFail = false;
  for (int i = 0; i < 2; i++) {
    j->bands[i] = (uint8_t*)heap_caps_aligned_alloc(16, bs, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    j->bandFull[i] = xSemaphoreCreateBinary();
    j->bandFree[i] = xSemaphoreCreateBinary();
  }
  j->bandOut = j->bands[0];
  if (j->bandFree[1]) xSemaphoreGive(j->bandFree[1]);  // band 0 is in use from the start; band 1 is free
  j->enc = enc;
  bool encStarted = false, sendStarted = false;
  j->sendExited = true;
  if (j->bands[0] && j->bands[1] && j->bandFull[0] && j->bandFull[1] && j->bandFree[0] && j->bandFree[1] && j->out) {
    if (j->sink) {                                        // sender first, so it's there when the first band is encoded
      j->outReady = xSemaphoreCreateBinary();
      j->sendExited = false;
      sendStarted = j->outReady && xTaskCreatePinnedToCoreWithCaps(shrinkSendTask, "send", 6144, j, 1, nullptr, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS;
      if (!sendStarted) { j->sendExited = true; }
    }
    // encoder on the other core; a stack in internal RAM if there's room, else external
    if (!j->sink || sendStarted) {
      encStarted = xTaskCreatePinnedToCore(shrinkEncTask, "encode", 10240, j, 2, &j->encTaskH, 0) == pdPASS;
      if (!encStarted) encStarted = xTaskCreatePinnedToCoreWithCaps(shrinkEncTask, "encode", 10240, j, 2, &j->encTaskH, 0, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT) == pdPASS;
    }
  }
  if (!encStarted) {
    blog("Fast resizer: not enough memory (largest free PSRAM block %u KB, internal %u KB)", (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) / 1024),
         (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
    if (sendStarted) { j->encExited = true; j->encFail = true; xSemaphoreGive(j->outReady); while (!j->sendExited) delay(1); }
    if (j->outReady) { vSemaphoreDelete(j->outReady); j->outReady = nullptr; }
    shrinkFreeWork(j); shrinkFreeBands(j); free(j->out); j->out = nullptr; j->outCap = 0; j->enc = nullptr; j->bandOut = nullptr;
    delete enc; free(blk); jpeg_dec_close(dec); return false;
  }
  j->encTwoCores = true;

  uint32_t t1 = millis();
  int y = 0;
  jpeg_error_t r = JPEG_ERR_OK;
  for (int b = 0; b < count; b++) {
    uint32_t td = millis();
    r = jpeg_dec_process(dec, &io);
    j->tDecOnly += millis() - td;
    if (r != JPEG_ERR_OK) break;
    int rows = io.out_size / (j->iw * 3);
    for (int k = 0; k < rows && y < j->ih; k++, y++) shrinkSourceRow(j, y, blk + (size_t)k * j->iw * 3);
    if (j->encFail) break;
    if ((b & 3) == 3) vTaskDelay(1);                    // let the web page and printer tasks run now and then
  }
  bool ok = r == JPEG_ERR_OK && y == j->ih && !j->encFail;
  if (ok) shrinkFinalizeUntil(j, j->th);
  uint32_t tResDone = millis();
  // let the encoder finish (or, after a failure, give it the bands it's still waiting for so it exits)
  while (j->bandCount < j->bandsTotal) { j->encFail = true; xSemaphoreGive(j->bandFull[(j->bandCount++) & 1]); }
  while (!j->encExited) delay(1);
  while (!j->sendExited) delay(1);                      // the sender finishes handing the last bytes to the printer
  if (j->outReady) { vSemaphoreDelete(j->outReady); j->outReady = nullptr; }
  ok = ok && !j->encFail && !j->sinkFail && j->outLen > 0;
  j->tDec = tResDone - t1;                              // decode+resize wall time (encoding and sending ran alongside)
  delete enc; j->enc = nullptr;
  jpeg_dec_close(dec);
  free(blk);
  shrinkFreeWork(j);
  shrinkFreeBands(j); j->bandOut = nullptr;
  if (!ok) {
    free(j->out); j->out = nullptr; j->outLen = 0; j->outCap = 0; j->encFail = false; j->base = 0;
    blog("Fast decoder couldn't handle this photo (error %d at row %d) - using the slow one", (int)r, y);
    return false;
  }
  size_t inBytes = j->hn;
  char b[320];
  bool cropped = j->cw < j->sw || j->ch < j->sh;
  char st[120] = "";
  if (j->sink && j->tFirstSend)
    snprintf(st, sizeof(st), "; printer: first chunk at %.1f s, %lu writes, took the last %.1f s after encoding ended",
             (j->tFirstSend - t0) / 1000.0, (unsigned long)j->sendWrites, (int32_t)(j->tSendDone - j->tEncDone) / 1000.0);
  snprintf(b, sizeof(b), "shrunk %dx%d %s -> %dx%d %s%s in %.1f s (camera %.1f s at %u KB/s, decode %.1f s + resize %.1f s, encoded alongside in %.1f s%s)",
           j->iw, j->ih, fmtSize(inBytes).c_str(), j->fw, j->fh, fmtSize(j->outLen).c_str(), cropped ? " (edges trimmed to fill the paper)" : "",
           (millis() - t0) / 1000.0, j->tRecv / 1000.0, j->tRecv ? (unsigned)(j->rxBytes / j->tRecv) : 0u, j->tDecOnly / 1000.0,
           (j->tDec - j->tDecOnly) / 1000.0, j->tEnc / 1000.0, st);
  j->info = b;
  j->result = 1;
  return true;
}

static void shrinkFreeWork(ShrinkJob* j) {
  free(j->ox0); free(j->wx0); free(j->oy0); free(j->wy0); free(j->band); free(j->pool);
  if (j->ownSums) { free(j->sums); free(j->hrow); }
  j->ox0 = j->wx0 = j->oy0 = j->wy0 = nullptr; j->sums = j->hrow = nullptr; j->band = nullptr; j->pool = nullptr;
}

// Mapping tables and accumulators for the box resampler (needs sw, sh, tw, th and the crop set).
// The accumulators are hit for every pixel and must sit in internal RAM, which is scarce and
// fragmented once Wi-Fi and USB are busy. So they're reserved once at boot and reused for every photo.
#define RES_W 2                                         // output rows open at once: a source row touches at most two
static uint32_t* resSums = nullptr;                     // RES_W x PRINT_LONG x 3
static uint32_t* resHrow = nullptr;                     // PRINT_LONG x 3
static void reserveResampler() {
  resSums = (uint32_t*)heap_caps_malloc((size_t)RES_W * PRINT_LONG * 3 * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  resHrow = (uint32_t*)heap_caps_malloc((size_t)PRINT_LONG * 3 * sizeof(uint32_t), MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (!resSums || !resHrow) { free(resSums); free(resHrow); resSums = resHrow = nullptr; }
}

static bool shrinkPrepareResampler(ShrinkJob* j) {
  j->W = RES_W;
  j->base = 0;
  j->ox0   = (uint16_t*)fastAlloc((size_t)j->sw * 2);  // small tables: keep them in fast internal RAM
  j->wx0   = (uint16_t*)fastAlloc((size_t)j->sw * 2);
  j->oy0   = (uint16_t*)fastAlloc((size_t)j->sh * 2);
  j->wy0   = (uint16_t*)fastAlloc((size_t)j->sh * 2);
  size_t sumBytes = (size_t)j->W * j->tw * 3 * sizeof(uint32_t), rowBytes = (size_t)j->tw * 3 * sizeof(uint32_t);
  j->ownSums = !(resSums && resHrow && j->tw <= PRINT_LONG);
  if (!j->ownSums) { j->sums = resSums; j->hrow = resHrow; }
  else {
    j->sums = (uint32_t*)heap_caps_malloc(sumBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    j->hrow = (uint32_t*)heap_caps_malloc(rowBytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!j->sums || !j->hrow) {
      blog("Resizer: only %u KB of internal RAM free (largest block %u KB), using slow external RAM for the accumulators",
           (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), (unsigned)(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL) / 1024));
      if (!j->sums) j->sums = (uint32_t*)bigAlloc(sumBytes);
      if (!j->hrow) j->hrow = (uint32_t*)bigAlloc(rowBytes);
    }
  }
  if (!j->ox0 || !j->wx0 || !j->oy0 || !j->wy0 || !j->sums || !j->hrow) return false;
  memset(j->sums, 0, sumBytes);
  // Within the crop, source pixel x spans [x*tw, (x+1)*tw) and output pixel o spans [o*cw, (o+1)*cw)
  // on a common scale. Source pixels outside the crop are skipped (0xFFFF).
  for (int x = 0; x < j->sw; x++) {
    if (x < j->cx0 || x >= j->cx0 + j->cw) { j->ox0[x] = 0xFFFF; j->wx0[x] = 0; continue; }
    uint32_t a = (uint32_t)(x - j->cx0) * j->tw, o = a / j->cw, edge = (o + 1) * (uint32_t)j->cw;
    j->ox0[x] = o;
    j->wx0[x] = (a + j->tw <= edge) ? j->tw : (edge - a);
  }
  for (int y = 0; y < j->sh; y++) {
    if (y < j->cy0 || y >= j->cy0 + j->ch) { j->oy0[y] = 0xFFFF; j->wy0[y] = 0; continue; }
    uint32_t a = (uint32_t)(y - j->cy0) * j->th, o = a / j->ch, edge = (o + 1) * (uint32_t)j->ch;
    j->oy0[y] = o;
    j->wy0[y] = (a + j->th <= edge) ? j->th : (edge - a);
  }
  return true;
}

static void shrinkRun(ShrinkJob* j) {
  j->result = 0;
  uint32_t t0 = millis();
  bool baseline = false;
  if (!jpegSof(j->hb, j->hn, j->iw, j->ih, baseline) || j->iw < 1 || j->ih < 1) { j->why = "couldn't read the image size"; return; }
  if (!baseline) { j->why = "progressive JPEG"; return; }

  int L = max(j->iw, j->ih), S = min(j->iw, j->ih);
  int pl = j->fast ? PRINT_LONG * 2 / 3 : PRINT_LONG;      // Fast: 1248x832 (200 dpi on 4x6)
  int ps = j->fast ? PRINT_SHORT * 2 / 3 : PRINT_SHORT;
  // fit: whole photo on the paper (white strips if it isn't 3:2); fill: cover the paper, trim the rest
  float k = j->fill ? max((float)pl / L, (float)ps / S) : min((float)pl / L, (float)ps / S);
  if (k >= 0.999f) { j->why = "already print size"; return; }
  int tL = j->fill ? pl : max(1, (int)lroundf(L * k)), tS = j->fill ? ps : max(1, (int)lroundf(S * k));
  if (j->iw >= j->ih) { j->tw = tL; j->th = tS; } else { j->tw = tS; j->th = tL; }
  j->kScale = k;

  // Fast path first: the whole file in memory, Espressif's codec. Falls back to the streaming path below.
  if (j->iw % 8 == 0 && j->ih % 8 == 0 && shrinkReceiveWhole(j)) {
    if (shrinkFast(j, t0)) { shrinkFreeWhole(j); return; }
  }

  int cwF = min(j->iw, max(j->tw, (int)lroundf(j->tw / k)));   // source area that ends up on the print
  int chF = min(j->ih, max(j->th, (int)lroundf(j->th / k)));
  j->s = 0;                                             // let the decoder do the first halvings for free
  while (j->s < 3 && (cwF >> (j->s + 1)) >= j->tw && (chF >> (j->s + 1)) >= j->th) j->s++;
  j->sw = j->iw >> j->s; j->sh = j->ih >> j->s;
  j->cw = min(j->sw, max(j->tw, cwF >> j->s)); j->ch = min(j->sh, max(j->th, chF >> j->s));
  j->cx0 = (j->sw - j->cw) / 2; j->cy0 = (j->sh - j->ch) / 2;
  bool rot = j->orient == 6 || j->orient == 8;
  j->fw = rot ? j->th : j->tw; j->fh = rot ? j->tw : j->th;
  j->bandH = 16 >> j->s; if (j->bandH < 1) j->bandH = 1;

  size_t frameBytes = (size_t)j->fw * j->fh * 3;
  if (heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM) < frameBytes + 256 * 1024) { j->why = "not enough memory"; return; }
  j->frame = (uint8_t*)heap_caps_malloc(frameBytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  j->band  = (uint8_t*)bigAlloc((size_t)j->bandH * j->sw * 3);
  const size_t POOL = 32 * 1024;
  j->pool  = malloc(POOL);
  if (!j->pool) j->pool = bigAlloc(POOL);
  if (!j->frame || !j->band || !j->pool || !shrinkPrepareResampler(j)) {
    shrinkFreeWork(j); free(j->frame); j->frame = nullptr;
    j->why = "not enough memory"; return;
  }

  JDEC jd;
  JRESULT rc = jd_prepare(&jd, shrinkIn, j->pool, POOL, j);
  if (rc != JDR_OK || jd.width != j->iw || jd.height != j->ih) {
    shrinkFreeWork(j); free(j->frame); j->frame = nullptr;
    j->why = "decoder could not read it (" + String((int)rc) + ")";
    j->result = (j->sockBytes && !j->whole) ? -1 : 0;   // with the file in memory the original is still usable
    return;
  }
  int mcuRows = (jd.msy * 8) >> j->s;                     // block row height: 8 or 16 rows, scaled
  if (mcuRows < 1) mcuRows = 1;
  if (mcuRows > j->bandH) {                               // never: blocks are at most 16 rows tall
    shrinkFreeWork(j); free(j->frame); j->frame = nullptr; shrinkFreeWhole(j);
    j->why = "unexpected block size"; j->result = -1; return;
  }
  j->bandH = mcuRows;
  rc = jd_decomp(&jd, shrinkOut, j->s);
  if (rc != JDR_OK) {
    shrinkFreeWork(j); free(j->frame); j->frame = nullptr; shrinkFreeWhole(j);
    j->why = j->err ? String("camera stopped sending") : ("decode error " + String((int)rc));
    j->result = -1;
    return;
  }
  shrinkFlushBand(j);
  shrinkFinalizeUntil(j, j->th);
  shrinkFreeWork(j);
  uint32_t t1 = millis();
  if (j->d && !j->eof) { j->sockBytes += drain(*j->d); j->eof = true; }   // e.g. Canon's embedded preview

  colorAdjust(j->frame, (size_t)j->fw * j->fh, true);
  screenThumbBegin(j->fw, j->fh, 1); screenThumbRows(j->frame, 0, j->fh, true); screenThumbEnd();
  bool ok = fmt2jpg_cb(j->frame, frameBytes, j->fw, j->fh, PIXFORMAT_RGB888, j->quality, shrinkJpgOut, j);
  free(j->frame); j->frame = nullptr;
  if (!ok || j->outFail || !j->outLen) {
    free(j->out); j->out = nullptr; j->outLen = 0;
    shrinkFreeWhole(j);
    j->why = "encoder failed";
    j->result = -1;
    return;
  }
  uint32_t t2 = millis();
  size_t inBytes = j->hn + j->sockBytes;
  shrinkFreeWhole(j);
  char b[200];
  bool cropped = j->cw < j->sw || j->ch < j->sh;
  if (j->rxBytes)
    snprintf(b, sizeof(b), "shrunk %dx%d %s -> %dx%d %s%s in %.1f s (camera sent %s in %.1f s, decode+resize %.1f s of which %.1f s waiting for the camera, encode %.1f s)",
             j->iw, j->ih, fmtSize(inBytes).c_str(), j->fw, j->fh, fmtSize(j->outLen).c_str(), cropped ? " (edges trimmed to fill the paper)" : "",
             (t2 - t0) / 1000.0, fmtSize(j->rxBytes).c_str(), (j->rxLastMs - j->rxFirstMs) / 1000.0, (t1 - t0) / 1000.0, j->waitMs / 1000.0, (t2 - t1) / 1000.0);
  else
    snprintf(b, sizeof(b), "shrunk %dx%d %s -> %dx%d %s%s in %.1f s (receive+decode %.1f s, encode %.1f s)",
             j->iw, j->ih, fmtSize(inBytes).c_str(), j->fw, j->fh, fmtSize(j->outLen).c_str(), cropped ? " (edges trimmed to fill the paper)" : "",
             (t2 - t0) / 1000.0, (t1 - t0) / 1000.0, (t2 - t1) / 1000.0);
  j->info = b;
  j->result = 1;
}

// The decoder and encoder need a lot of stack, so they run in their own short-lived task.
static void shrinkTaskFn(void* arg) {
  ShrinkJob* j = (ShrinkJob*)arg;
  shrinkRun(j);
  xTaskNotifyGive(j->waiter);
  vTaskDelete(nullptr);
}

static SemaphoreHandle_t shrinkMtx = nullptr;
static void shrinkBlocking(ShrinkJob* j) {
  if (!shrinkMtx) shrinkMtx = xSemaphoreCreateMutex();
  xSemaphoreTake(shrinkMtx, portMAX_DELAY);              // one photo at a time (shared accumulators)
  j->waiter = xTaskGetCurrentTaskHandle();
  if (xTaskCreatePinnedToCore(shrinkTaskFn, "shrink", 32768, j, 1, nullptr, 1) != pdPASS) {
    j->result = 0; j->why = "no memory to start the resizer"; xSemaphoreGive(shrinkMtx); return;
  }
  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  xSemaphoreGive(shrinkMtx);
}

// Live printing: the print job is opened on the printer before the photo is processed, and the
// encoded bytes are streamed into it (HTTP chunked) as they're produced.
static Client* liveClient = nullptr;
#define LIVE_CHUNK (48 * 1024)
static uint8_t* liveBuf = nullptr;                       // one chunk (size line + data + line end) in a single write:
                                                         // every separate write costs a round trip with the printer
static void liveSink(void* ctx, const uint8_t* d, size_t n) {
  ShrinkJob* j = (ShrinkJob*)ctx;
  if (!liveClient || j->sinkFail) return;
  while (n && !j->sinkFail) {
    size_t k = n > LIVE_CHUNK ? LIVE_CHUNK : n;
    if (liveBuf) {
      int hl = snprintf((char*)liveBuf, 16, "%X\r\n", (unsigned)k);
      memcpy(liveBuf + hl, d, k);
      liveBuf[hl + k] = '\r'; liveBuf[hl + k + 1] = '\n';
      if (!writeAll(*liveClient, liveBuf, hl + k + 2)) j->sinkFail = true;
    } else {
      char hdr[12]; int hl = snprintf(hdr, sizeof(hdr), "%X\r\n", (unsigned)k);
      if (!writeAll(*liveClient, (const uint8_t*)hdr, hl) || !writeAll(*liveClient, d, k) || !writeAll(*liveClient, (const uint8_t*)"\r\n", 2))
        j->sinkFail = true;
    }
    d += k; n -= k;
  }
}

// Shrinks a complete JPEG held in memory. On success replaces data/len with the small version.
static bool shrinkBuffer(uint8_t*& data, size_t& len, String& info) {
  if (!shrinkOn || !psramFound()) { info = "shrinking is off"; return false; }
  ShrinkJob sj;
  sj.hb = data; sj.hn = len; sj.d = nullptr; sj.eof = true; sj.quality = jpegQ; sj.fast = (speedMode == 1); sj.fill = fillOn;
  findMeta(data, len, &sj.orient);
  shrinkBlocking(&sj);
  if (sj.result != 1) { info = sj.why; return false; }
  free(data);
  data = sj.out; len = sj.outLen; info = sj.info;
  return true;
}

static void flushPendingData() { camData.stop(); }

static bool openData(CamData& d, bool active, IPAddress ip, uint16_t port) {
  if (active) return false;                             // only passive mode (the camera connects to us)
  return d.open(10000);
}

static String finishStream(uint16_t st, const String& name, size_t total) {
  if (st < 0x0100) {
    blog("Printing %s (%s) - sent over %s in %.1f s, printer took %.1f s to accept", name.c_str(), fmtSize(total).c_str(), curUsb ? "USB" : "Wi-Fi", lastSendMs / 1000.0, lastAnswerMs / 1000.0);
    countPrint();
    notePhoto(name.c_str(), "%.1f s, %s", (millis() - storStartMs) / 1000.0, curUsb ? "USB" : "Wi-Fi");
    if (printMode == 1) addPrinted(name);
    return "226 Transfer complete - printing";
  }
  blog("Could not print %s: %s", name.c_str(), ippStatusName(st).c_str());
  return "451 Printer error: " + ippStatusName(st);
}

// Receives one file from the camera and decides what to do with it. Returns the FTP reply.
struct CamBusyGuard { CamBusyGuard() { camBusy = true; storStartMs = millis(); screenWake(); } ~CamBusyGuard() { camBusy = false; } };
static String ftpStor(Client& d, const String& path) {
  CamBusyGuard busy;
  String name = path;
  int sl = name.lastIndexOf('/');
  if (sl >= 0) name = name.substring(sl + 1);
  String lower = name; lower.toLowerCase();
  bool isJpg = lower.endsWith(".jpg") || lower.endsWith(".jpeg");

  if (!isJpg) {
    drain(d);
    blog("Skipped %s - not a JPEG (set the camera to send JPEG only)", name.c_str());
    return "226 Received (not printed: not a JPEG)";
  }

  // 1. Read the start of the file: metadata (star rating, orientation) and the image header.
  size_t hcap = psramFound() ? 256 * 1024 : 64 * 1024;
  uint8_t* hb = (uint8_t*)bigAlloc(hcap);
  if (!hb) { drain(d); blog("Out of memory receiving %s", name.c_str()); return "451 Out of memory"; }
  size_t hn = 0; bool eof = false;
  while (hn < hcap) {
    int n = readSome(d, hb + hn, hcap - hn, 30000);
    if (n < 0) { free(hb); blog("Camera stopped sending %s", name.c_str()); return "426 Timed out"; }
    if (n == 0) { eof = true; break; }
    hn += n;
    if (jpegHeaderEnd(hb, hn) != -1) break;
  }
  int orient = 1;
  int rating = findMeta(hb, hn, &orient);

  // 2. Rating filter.
  if (printMode == 1) {
    if (rating < minRating) {
      if (!eof) drain(d);
      free(hb);
      blog("Skipped %s (%s) - below the %d\xE2\x98\x85 minimum", name.c_str(), stars(rating).c_str(), minRating);
      return "226 Received (not printed: rating)";
    }
    if (wasPrinted(name)) {
      if (!eof) drain(d);
      free(hb);
      blog("Skipped %s - already printed", name.c_str());
      return "226 Received (already printed)";
    }
  }

  // 3. Shrink to print size (much faster to print). If the printer is free, the print job is opened
  //    first and the result streams straight into it; otherwise the result waits in the queue.
  if (shrinkOn && psramFound()) {
    ShrinkJob sj;
    sj.hb = hb; sj.hn = hn; sj.d = &d; sj.eof = eof; sj.orient = orient; sj.quality = jpegQ; sj.fast = (speedMode == 1); sj.fill = fillOn;
    WiFiClient w;
    bool live = false;
    uint32_t tLive = 0;
    if (!dryRun && queueLen() == 0 && xSemaphoreTake(printerMtx, pdMS_TO_TICKS(3000)) == pdTRUE) {
      if (printerReady()) {
        liveClient = printerConnect(w, 5000);
        if (liveClient) {
          tLive = millis();
          std::vector<uint8_t> h;
          buildPrintJob(h, name);
          sendPostHeaders(*liveClient, true, 0);
          sendChunk(*liveClient, h.data(), h.size());
          sj.sink = liveSink; sj.sinkCtx = &sj;
          live = true;
        }
      }
      if (!live) xSemaphoreGive(printerMtx);
    }
    shrinkBlocking(&sj);
    if (sj.whole) {                                     // the resizer pulled the photo into memory: keep using that copy
      free(hb); hb = sj.whole; hn = sj.hn; hcap = sj.wholeCap; eof = sj.eof; sj.whole = nullptr;
    }
    if (live) {
      uint16_t st = 0xFFFE;
      if (sj.result == 1 && !sj.sinkFail) {
        if (sj.outLen > sj.sinkSent) liveSink(&sj, sj.out + sj.sinkSent, sj.outLen - sj.sinkSent);   // the rest (e.g. fallback path)
        writeAll(*liveClient, (const uint8_t*)"0\r\n\r\n", 5);
        uint32_t t1 = millis();
        std::vector<uint8_t> body;
        int code = readHttpResponse(*liveClient, body);
        lastSendMs = t1 - tLive; lastAnswerMs = millis() - t1;
        st = code == 200 ? parseIpp(body, nullptr) : 0xFFFF;
        if (st < 0x0100) lastSubmitMs = millis();
      } else {
        writeAll(*liveClient, (const uint8_t*)"0\r\n\r\n", 5);   // end the job cleanly; the printer will reject the short file
        std::vector<uint8_t> body; readHttpResponse(*liveClient, body);
      }
      liveClient->stop(); liveClient = nullptr;
      xSemaphoreGive(printerMtx);
      free(hb);
      if (sj.result == 1 && !sj.sinkFail) {
        free(sj.out);
        if (st < 0x0100) {
          blog("Printed live: %s (%s) - %s; printer took %.1f s to accept", name.c_str(), stars(rating).c_str(), sj.info.c_str(), lastAnswerMs / 1000.0);
          countPrint();
          notePhoto(name.c_str(), "%.1f s, %s", (millis() - storStartMs) / 1000.0, curUsb ? "USB" : "Wi-Fi");
          if (printMode == 1) addPrinted(name);
          return "226 Transfer complete - printing";
        }
        blog("Could not print %s: %s", name.c_str(), ippStatusName(st).c_str());
        return "451 Printer error: " + ippStatusName(st);
      }
      free(sj.out);
      if (!sj.eof) drain(d);
      blog("Could not process %s: %s - send it again", name.c_str(), sj.result == 1 ? "printer stopped taking data" : sj.why.c_str());
      return "451 Could not process the photo";
    }
    if (sj.result == 1) {
      free(hb);
      if (dryRun) {
        free(sj.out);
        blog("Test mode, not printed: %s (%s) - %s", name.c_str(), stars(rating).c_str(), sj.info.c_str());
        notePhoto(name.c_str(), "test, %.1f s", (millis() - storStartMs) / 1000.0);
        return "226 Transfer complete (test mode)";
      }
      enqueueJob(sj.out, sj.outLen, name, rating);
      blog("Received %s (%s) - %s", name.c_str(), stars(rating).c_str(), sj.info.c_str());
      return "226 Transfer complete";
    }
    if (sj.result == -1) {
      free(hb);
      if (!sj.eof) drain(d);
      blog("Could not shrink %s: %s - send it again", name.c_str(), sj.why.c_str());
      return "451 Could not process the photo";
    }
    blog("%s: sending the original (%s)", name.c_str(), sj.why.c_str());
  }

  if (dryRun) {
    if (!eof) drain(d);
    free(hb);
    blog("Test mode, not printed: %s (%s)", name.c_str(), stars(rating).c_str());
    return "226 Transfer complete (test mode)";
  }

  // 4. Printer free and nothing waiting: stream straight through.
  if (queueLen() == 0 && xSemaphoreTake(printerMtx, pdMS_TO_TICKS(3000)) == pdTRUE) {
    if (printerReady()) {
      blog("Received %s (%s) - sending to printer", name.c_str(), stars(rating).c_str());
      size_t total = 0;
      uint16_t st = streamJob(d, hb, hn, eof, name, total);
      xSemaphoreGive(printerMtx);
      free(hb);
      return finishStream(st, name, total);
    }
    xSemaphoreGive(printerMtx);
  }

  // 5. Printer busy: hold the photo in memory and queue it.
  size_t limit = spoolLimit();
  size_t cap = hcap, sn = hn;
  uint8_t* sb = hb;
  bool overflow = false;
  while (!eof) {
    if (cap - sn < 8192) {
      size_t nc = cap + cap / 2 + 65536;
      if (nc > limit) nc = limit;
      if (nc < sn + 8192) { overflow = true; break; }
      uint8_t* nb = (uint8_t*)bigRealloc(sb, nc);
      if (!nb) { overflow = true; break; }
      sb = nb; cap = nc;
    }
    int n = readSome(d, sb + sn, cap - sn, 30000);
    if (n < 0) { free(sb); blog("Camera stopped sending %s", name.c_str()); return "426 Timed out"; }
    if (n == 0) { eof = true; break; }
    sn += n;
  }
  if (!overflow) {
    enqueueJob(sb, sn, name, rating);
    blog("Received %s (%s, %s) - queued, printer busy", name.c_str(), stars(rating).c_str(), fmtSize(sn).c_str());
    return "226 Transfer complete - queued";
  }

  // 6. Too big to hold: pause the camera's transfer until the printer is free, then stream.
  blog("%s is too big to hold - waiting for the printer", name.c_str());
  uint32_t t0 = millis();
  while (millis() - t0 < 240000) {
    if (queueLen() == 0 && xSemaphoreTake(printerMtx, pdMS_TO_TICKS(1000)) == pdTRUE) {
      if (printerReady()) {
        size_t total = 0;
        uint16_t st = streamJob(d, sb, sn, false, name, total);
        xSemaphoreGive(printerMtx);
        free(sb);
        return finishStream(st, name, total);
      }
      xSemaphoreGive(printerMtx);
    }
    vTaskDelay(pdMS_TO_TICKS(2000));
  }
  free(sb);
  drain(d);
  blog("Gave up on %s - printer stayed busy. Send it again.", name.c_str());
  return "451 Printer busy, try again";
}

static void ftpSession(WiFiClient& c) {
  c.setNoDelay(true);
  blog("Camera connected (%s)", c.remoteIP().toString().c_str());
  ftpSend(c, "220 BTL Photo Printer Bridge ready");
  uint32_t tStart = millis();          // timing of what the camera does before it sends (for the log)
  String trace;
  bool authed = false, active = false;
  String user, cwd = "/", line;
  IPAddress aIP; uint16_t aPort = 0;

  while (c.connected()) {
    if (!readLineT(c, line, 900000)) break;
    line.trim();
    if (!line.length()) continue;
    int sp = line.indexOf(' ');
    String cmd = sp > 0 ? line.substring(0, sp) : line;
    String arg = sp > 0 ? line.substring(sp + 1) : "";
    cmd.toUpperCase();
    if (trace.length() < 200) { trace += cmd; trace += '@'; trace += String(millis() - tStart); trace += ' '; }

    if (cmd == "USER") { user = arg; ftpSend(c, "331 Password please"); continue; }
    if (cmd == "PASS") {
      if (user == ftpUser && arg == ftpPass) { authed = true; ftpSend(c, "230 Logged in"); }
      else { ftpSend(c, "530 Login incorrect"); blog("Camera login rejected (user '%s') - check the FTP login on this page", user.c_str()); }
      continue;
    }
    if (cmd == "QUIT") { ftpSend(c, "221 Bye"); break; }
    if (cmd == "SYST") { ftpSend(c, "215 UNIX Type: L8"); continue; }
    if (cmd == "FEAT") { c.print("211-Features:\r\n PASV\r\n EPSV\r\n UTF8\r\n SIZE\r\n211 End\r\n"); continue; }
    if (cmd == "OPTS" || cmd == "CLNT" || cmd == "NOOP") { ftpSend(c, "200 OK"); continue; }
    if (cmd == "AUTH" || cmd == "PBSZ" || cmd == "PROT") { ftpSend(c, "502 Use plain FTP"); continue; }
    if (!authed) { ftpSend(c, "530 Please log in"); continue; }

    if (cmd == "TYPE" || cmd == "MODE" || cmd == "STRU") ftpSend(c, "200 OK");
    else if (cmd == "PWD" || cmd == "XPWD") ftpSend(c, "257 \"" + cwd + "\"");
    else if (cmd == "CWD" || cmd == "XCWD") {
      cwd = arg.startsWith("/") ? arg : (cwd.endsWith("/") ? cwd + arg : cwd + "/" + arg);
      ftpSend(c, "250 OK");
    }
    else if (cmd == "CDUP" || cmd == "XCUP") { int p = cwd.lastIndexOf('/'); cwd = p > 0 ? cwd.substring(0, p) : "/"; ftpSend(c, "250 OK"); }
    else if (cmd == "MKD" || cmd == "XMKD") ftpSend(c, "257 \"" + arg + "\" created");
    else if (cmd == "RMD" || cmd == "XRMD" || cmd == "DELE" || cmd == "RNTO") ftpSend(c, "250 OK");
    else if (cmd == "RNFR") ftpSend(c, "350 OK");
    else if (cmd == "SIZE" || cmd == "MDTM") ftpSend(c, "550 No such file");
    else if (cmd == "ALLO") ftpSend(c, "202 OK");
    else if (cmd == "REST") ftpSend(c, arg.toInt() == 0 ? "350 OK" : "502 Resume not supported");
    else if (cmd == "PASV") {
      active = false; flushPendingData();
      IPAddress ip = c.localIP();                 // the bridge's address on whichever network the camera came in on
      char b[80];
      snprintf(b, sizeof(b), "227 Entering Passive Mode (%u,%u,%u,%u,%u,%u)", ip[0], ip[1], ip[2], ip[3], FTP_PASV_PORT >> 8, FTP_PASV_PORT & 0xFF);
      ftpSend(c, b);
    }
    else if (cmd == "EPSV") { active = false; flushPendingData(); ftpSend(c, "229 Entering Extended Passive Mode (|||" + String(FTP_PASV_PORT) + "|)"); }
    else if (cmd == "PORT") {
      int v0, v1, v2, v3, v4, v5;
      if (sscanf(arg.c_str(), "%d,%d,%d,%d,%d,%d", &v0, &v1, &v2, &v3, &v4, &v5) == 6) {
        aIP = IPAddress(v0, v1, v2, v3); aPort = v4 * 256 + v5; active = true; ftpSend(c, "200 OK");
      } else ftpSend(c, "501 Bad PORT");
    }
    else if (cmd == "EPRT") {
      char dl = arg.length() ? arg[0] : '|';
      int a = arg.indexOf(dl, 1), b = arg.indexOf(dl, a + 1), e = arg.indexOf(dl, b + 1);
      if (a > 0 && b > a && e > b) { aIP.fromString(arg.substring(a + 1, b)); aPort = arg.substring(b + 1, e).toInt(); active = true; ftpSend(c, "200 OK"); }
      else ftpSend(c, "501 Bad EPRT");
    }
    else if (cmd == "LIST" || cmd == "NLST" || cmd == "MLSD") {
      CamData& d = camData;
      if (!openData(d, active, aIP, aPort)) ftpSend(c, active ? "425 Use passive mode" : "425 No data connection");
      else { ftpSend(c, "150 Listing"); d.stop(); ftpSend(c, "226 Done"); }
    }
    else if (cmd == "STOR" || cmd == "APPE" || cmd == "STOU") {
      CamData& d = camData;
      if (!openData(d, active, aIP, aPort)) { ftpSend(c, active ? "425 Use passive mode" : "425 No data connection"); continue; }
      blog("Camera timing (ms): %sdata@%lu", trace.c_str(), (unsigned long)(millis() - tStart));
      ftpSend(c, "150 Ready to receive");
      String r = ftpStor(d, arg.length() ? arg : String("photo.jpg"));
      trace = ""; tStart = millis();
      d.stop();
      ftpSend(c, r);
    }
    else if (cmd == "STAT") ftpSend(c, "211 BTL Photo Printer Bridge");
    else ftpSend(c, "502 Not implemented");
  }
  if (trace.length()) blog("Camera disconnected without sending; it did: %s", trace.c_str());
  else blog("Camera disconnected");
}

static void ftpTask(void*) {
  for (;;) {
    WiFiClient c = SRV_ACCEPT(ftpCtl);
    if (c) { ftpSession(c); c.stop(); }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

// =============================================================== web page
static void redirectHome() { server.sendHeader("Location", "/"); server.send(303); }

static const char PAGE_STYLE[] =
  "<style>:root{--bg:#111;--card:#1b1b1b;--fg:#eee;--mut:#9a9a9a;--acc:#f5b301}"
  "body{font-family:system-ui,sans-serif;max-width:760px;margin:0 auto;padding:16px;background:var(--bg);color:var(--fg)}"
  "h1{font-size:1.35em;margin:.2em 0}h2{font-size:1.05em;margin:.1em 0 .6em;color:var(--acc)}"
  ".card{background:var(--card);border-radius:10px;padding:12px 14px;margin:12px 0}"
  "pre{background:#0b0b0b;padding:10px;white-space:pre-wrap;word-break:break-word;font-size:.85em;max-height:360px;overflow:auto;margin:0}"
  "button{padding:8px 14px;font-size:1em;border-radius:6px;border:0;background:var(--acc);color:#111;font-weight:600;margin-top:6px}"
  "input,select{padding:6px;margin:3px 0;font-size:1em;max-width:100%}label{display:block;margin:7px 0}"
  "table{border-collapse:collapse}td{padding:3px 12px 3px 0;vertical-align:top}.mut{color:var(--mut)}.warn{color:#ff7a59}"
  "</style>";

static void handleRoot() {
  bool sta = WiFi.status() == WL_CONNECTED;
  String ip = sta ? WiFi.localIP().toString() : String("not connected");
  String h; h.reserve(10000);
  h += "<!doctype html><html><head><meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
       "<title>BTL Photo Printer Bridge</title>";
  h += PAGE_STYLE;
  h += "</head><body><h1>BTL Photo Printer Bridge</h1><div class=mut>v" FW_VERSION;
  if (apOn) h += " &middot; camera network <b>" + esc(apSsid) + "</b>: " + AP_IP.toString();
  h += " &middot; home Wi-Fi: " + ip + "</div>";

  if (!psramFound())
    h += "<div class='card warn'>PSRAM is off, so photos can't wait in a queue while the printer is busy. "
         "In Arduino IDE set Tools &rarr; PSRAM &rarr; OPI PSRAM and upload again.</div>";

  h += "<div class=card><h2>Status</h2><table>"
       "<tr><td>Printer</td><td id=pr>&hellip;</td></tr>"
       "<tr><td>USB port</td><td id=pu>&hellip;</td></tr>"
       "<tr><td>State</td><td id=ps>&hellip;</td></tr>"
       "<tr><td>Waiting to print</td><td id=pq>0</td></tr>"
       "<tr><td>Ink cassette</td><td><span id=pl>&hellip;</span>"
       "<form method=post action=/cassette style='display:inline;margin-left:8px'>"
       "of <input name=cap type=number min=1 max=200 style='width:4.5em' value=" + String(cassCap) + "> "
       "<button name=new value=1 style='padding:4px 10px'>New cassette</button> <button style='padding:4px 10px'>Save size</button></form></td></tr>"
       "<tr><td>Paper in tray</td><td><span id=pp>&hellip;</span>"
       "<form method=post action=/paper style='display:inline;margin-left:8px'>"
       "loaded <input name=cap type=number min=1 max=40 style='width:4.5em' value=" + String(paperCap) + "> "
       "<button style='padding:4px 10px'>Paper loaded</button></form></td></tr>"
       "<tr><td>Mode</td><td>";
  if (printMode == 1) h += "Only photos rated " + String(minRating) + "&#9733; or more";
  else h += "Every photo the camera sends";
  h += "</td></tr></table>"
       "<form method=post action=/rediscover><button>Search for printer again</button></form>"
       "<form method=post action=/usbcheck><button>Check the printer over USB</button></form>"
       "<form method=post action=/usbreset><button>Restart the USB port</button></form>"
       "<div class=mut style='margin-top:6px'><a style='color:inherit' href=/usbdiag>USB diagnostics</a> &middot; <a style='color:inherit' href=/printerinfo>Printer details</a></div>";
  if (screenPresent()) h += "<form method=post action=/screencal style='margin-top:8px'><button>Redo screen touch setup</button></form>";
  h += "</div>";

  h += "<div class=card><h2>Activity</h2><pre id=log>&hellip;</pre></div>";

  h += "<div class=card><h2>Print settings</h2><form method=post action=/settings>"
       "<label><input type=radio name=mode value=all";
  if (printMode == 0) h += " checked";
  h += "> Print every photo the camera sends <span class=mut>(use with Transfer with SET)</span></label>"
       "<label><input type=radio name=mode value=rating";
  if (printMode == 1) h += " checked";
  h += "> Only print photos rated <select name=minr>";
  for (int i = 1; i <= 5; i++) h += "<option value=" + String(i) + (i == minRating ? " selected>" : ">") + String(i) + "&#9733;</option>";
  h += "</select> or more</label>"
       "<label><input type=checkbox name=shrink";
  if (shrinkOn) h += " checked";
  h += "> Shrink photos to print size first <span class=mut>(prints much sooner; still the printer's full 300 dpi)</span></label>"
       "<label><input type=checkbox name=fill";
  if (fillOn) h += " checked";
  h += "> Fill the paper <span class=mut>(photos that aren't 3:2 get their edges trimmed instead of white strips)</span></label>"
       "<label><input type=checkbox name=pscale";
  if (scaleFill) h += " checked";
  h += "> Borderless <span class=mut>(asks for the postcard paper with zero margins; off = the printer's own margins, about 3.7 mm top and bottom)</span></label>"
       "<label><input type=checkbox name=dry";
  if (dryRun) h += " checked";
  h += "> <b>Test mode</b>: process photos but don't print them <span class=mut>(timing shows in the log, no paper used)</span></label>";
  h += "<label>Warmth <select name=cwarm>";
  for (int v = -5; v <= 5; v++)
    h += "<option value=" + String(v) + (v == colWarm ? " selected>" : ">") +
         (v == 0 ? String("Neutral (as shot)") : v < 0 ? "Cooler " + String(-v) : "Warmer " + String(v)) + "</option>";
  h += "</select> <span class=mut>(prints look yellow? pick Cooler)</span></label>";
  h += "<label>Contrast <select name=ccon>";
  for (int v = 0; v <= 3; v++) h += "<option value=" + String(v) + (v == colCon ? " selected>" : ">") + (v ? "+" + String(v) : String("As shot")) + "</option>";
  h += "</select></label><label>Color <select name=csat>";
  for (int v = 0; v <= 3; v++) h += "<option value=" + String(v) + (v == colSat ? " selected>" : ">") + (v ? "+" + String(v) : String("As shot")) + "</option>";
  h += "</select> <span class=mut>(flat skin tones: try Contrast +1 and Color +1)</span></label>";
  h += "<label>Quality sent to the printer <select name=q>";
  const int qs[3] = { 75, 85, 95 };
  const char* qn[3] = { "Fast", "Balanced", "Best" };
  for (int i = 0; i < 3; i++) h += "<option value=" + String(qs[i]) + (qs[i] == jpegQ ? " selected>" : ">") + qn[i] + "</option>";
  h += "</select></label>"
       "<label>Print detail <select name=speed>";
  h += String("<option value=0") + (speedMode == 0 ? " selected" : "") + ">Best: 1872x1248 (the printer's native size, 300 dpi)</option>";
  h += String("<option value=1") + (speedMode == 1 ? " selected" : "") + ">Fast: 1248x832 (200 dpi): the printer has to scale it, so not actually faster</option>";
  h += "</select></label>"
       "<label><input type=checkbox name=usb";
  if (useUsb) h += " checked";
  h += "> Print over the USB cable when the printer is plugged in <span class=mut>(otherwise Wi-Fi)</span></label>"
       "<label>FTP user name <input name=fuser value='" + esc(ftpUser) + "'></label>"
       "<label>FTP password <input name=fpass value='" + esc(ftpPass) + "'></label>"
       "<button>Save</button></form>"
       "<form method=post action=/clearhist><button>Forget which photos were printed</button></form>"
       "<div class=mut style='margin-top:6px'>In star-rating mode each photo prints once; this lets them print again.</div></div>";

  h += "<div class=card><h2>Camera setup (Canon FTP transfer)</h2><table>";
  if (apOn)
    h += "<tr><td>Wi-Fi network</td><td><b>" + esc(apSsid) + "</b>, password <b>" + esc(apPass) + "</b> <span class=mut>(the bridge's own network: fastest, works anywhere)</span>"
         "<br><span class=mut>or your home Wi-Fi</span></td></tr>"
         "<tr><td>Server address</td><td><b>" + AP_IP.toString() + "</b> on " + esc(apSsid) + "<span class=mut>, or " + ip + " on your home Wi-Fi</span></td></tr>";
  else
    h += "<tr><td>Server address</td><td><b>" + ip + "</b></td></tr>";
  h += "<tr><td>FTP mode</td><td>FTP <span class=mut>(not FTPS or SFTP)</span></td></tr>"
       "<tr><td>Port</td><td>21</td></tr>"
       "<tr><td>Passive mode</td><td>Enable</td></tr>"
       "<tr><td>Proxy</td><td>Disable</td></tr>"
       "<tr><td>Login</td><td>user <b>" + esc(ftpUser) + "</b>, password <b>" + esc(ftpPass) + "</b></td></tr>"
       "<tr><td>Target folder</td><td>Root folder</td></tr>"
       "<tr><td>FTP transfer settings</td><td>Automatic transfer: Disable<br>Transfer with SET: Enable<br>RAW+JPEG: send JPEG only</td></tr>"
       "</table></div>";

  h += "<div class=card><h2>Test print</h2><form method=post action=/testprint enctype=multipart/form-data "
       "onsubmit=\"var f=this.f.files[0],m=" + String((unsigned long)testPrintMax()) + ",o=document.getElementById('tpm');"
       "if(!f){o.textContent='Choose a JPEG first.';return false}"
       "if(f.size>m){o.textContent='That file is '+(f.size/1048576).toFixed(1)+' MB; this box takes up to '+(m/1048576).toFixed(1)+' MB. Full-size camera photos: send them from the camera with SET instead.';return false}"
       "o.textContent='Sending and resizing... the page comes back when it is queued.';\">"
       "<input type=file name=f accept='image/jpeg'> <button>Print this JPEG</button></form>"
       "<div id=tpm class=mut style='margin-top:6px'>Up to " + fmtSize(testPrintMax()) + ". Full-size camera photos are bigger than that: send those from the camera.</div></div>";

  h += "<div class=card><details><summary>Camera network (the bridge's own Wi-Fi)</summary><form method=post action=/apsettings>"
       "<label><input type=checkbox name=apon";
  if (apOn) h += " checked";
  h += "> Run the bridge's own Wi-Fi network for the camera</label>"
       "<label>Network name <input name=apssid value='" + esc(apSsid) + "' maxlength=31></label>"
       "<label>Password <input name=appass value='" + esc(apPass) + "' minlength=8 maxlength=63></label>"
       "<button>Save</button><div class=mut>At least 8 characters. The camera's address on this network is always " + AP_IP.toString() + ".</div></form></details></div>";

  h += "<div class=card><details><summary>Home Wi-Fi (optional: printer over Wi-Fi, page from your normal network)</summary><form method=post action=/savewifi>"
       "<label>Wi-Fi name <input name=ssid></label><label>Password <input name=pass type=password></label>"
       "<button>Save and restart</button><div class=mut>2.4 GHz networks only.</div></form></details></div>";

  h += "<script>async function u(){try{const s=await(await fetch('/status')).json();"
       "pr.textContent=s.printer;ps.textContent=s.pstate+(s.reasons&&s.reasons!='none'?' ('+s.reasons+')':'');"
       "pu.textContent=s.usb;pq.textContent=s.queue;pl.textContent=s.left+' prints left';pl.style.color=s.left<=5?'#f5b400':'';pp.textContent=s.paper+' sheets left';pp.style.color=s.paper<=3?'#f5b400':'';log.textContent=s.log.join('\\n')||'Nothing yet.';}catch(e){}}"
       "u();setInterval(u,3000);</script></body></html>";
  server.send(200, "text/html", h);
}

// Asks the printer what it is over the USB cable and writes the answer to the activity log.
static void handleUsbCheck() {
  if (!uReady) { blog("USB check: no printer on the USB port (%s)", getUsbState().c_str()); redirectHome(); return; }
  if (xSemaphoreTake(printerMtx, pdMS_TO_TICKS(5000)) != pdTRUE) { blog("USB check: printer busy, try again"); redirectHome(); return; }
  bool was = useUsb; useUsb = true;
  WiFiClient w;
  Client* c = printerConnect(w, 4000);
  if (!c) { blog("USB check: couldn't start"); }
  else {
    std::vector<uint8_t> req;
    ippHeader(req, 0x000B);
    putAttr(req, 0x44, "requested-attributes", "printer-make-and-model");
    putAttrMore(req, 0x44, "printer-state");
    putAttrMore(req, 0x44, "document-format-supported");
    put8(req, 0x03);
    uint32_t t0 = millis();
    sendPostHeaders(*c, false, req.size());
    writeAll(*c, req.data(), req.size());
    std::vector<uint8_t> body;
    int code = readHttpResponse(*c, body);
    String txt;
    uint16_t st = code == 200 ? parseIpp(body, &txt) : 0xFFFF;
    if (code == 200 && st < 0x0100)
      blog("USB check OK in %lu ms: %s, state %s, formats: %s", (unsigned long)(millis() - t0),
           attrVal(txt, "printer-make-and-model").c_str(), attrVal(txt, "printer-state").c_str(),
           attrVal(txt, "document-format-supported").c_str());
    else
      blog("USB check failed after %lu ms: HTTP %d, IPP %s (sent %lu bytes, received %lu bytes)", (unsigned long)(millis() - t0),
           code, ippStatusName(st).c_str(), (unsigned long)dOutBytes, (unsigned long)dInBytes);
  }
  useUsb = was;
  xSemaphoreGive(printerMtx);
  redirectHome();
}

// Asks the printer what it can do (paper, scaling, resolution) and shows the answer as text,
// or (raw = true) hands over the printer's reply bytes as a file for inspection elsewhere.
static void handlePrinterInfoRaw(bool raw);
static void handlePrinterInfo() { handlePrinterInfoRaw(false); }
static void handlePrinterInfoRaw(bool raw) {
  if (!printerAvailable()) { server.send(200, "text/plain", "No printer yet."); return; }
  if (xSemaphoreTake(printerMtx, pdMS_TO_TICKS(8000)) != pdTRUE) { server.send(200, "text/plain", "Printer busy, try again in a few seconds."); return; }
  WiFiClient w;
  Client* c = printerConnect(w, 5000);
  String o = "BTL Photo Printer Bridge v" FW_VERSION " - printer details (" + String(curUsb ? "USB cable" : "Wi-Fi") + ")\n";
  if (!c) o += "Couldn't reach the printer.\n";
  else {
    std::vector<uint8_t> req;
    ippHeader(req, 0x000B);
    putAttr(req, 0x44, "requested-attributes", "printer-make-and-model");
    const char* want[] = { "printer-state", "printer-state-reasons", "ipp-versions-supported", "document-format-supported",
      "media-default", "media-ready", "media-supported", "media-type-supported", "media-source-supported",
      "print-scaling-default", "print-scaling-supported", "printer-resolution-default", "printer-resolution-supported",
      "print-quality-default", "print-quality-supported", "print-color-mode-supported", "media-bottom-margin-supported",
      "media-top-margin-supported", "media-left-margin-supported", "media-right-margin-supported", "job-creation-attributes-supported",
      "media-col-default", "media-col-database", "media-size-supported", "marker-names", "marker-levels", "marker-types",
      "printer-alert", "printer-alert-description", "printer-supply", "printer-supply-description", "printer-input-tray", "printer-output-tray" };
    for (auto a : want) putAttrMore(req, 0x44, a);
    put8(req, 0x03);
    uint32_t t0 = millis();
    sendPostHeaders(*c, false, req.size());
    writeAll(*c, req.data(), req.size());
    std::vector<uint8_t> body;
    int code = readHttpResponse(*c, body);
    c->stop();
    if (raw) {
      xSemaphoreGive(printerMtx);
      server.sendHeader("Content-Disposition", "attachment; filename=\"printer-attributes.bin\"");
      server.send_P(200, "application/octet-stream", (const char*)body.data(), body.size());
      return;
    }
    String txt;
    uint16_t st = code == 200 ? parseIpp(body, &txt) : 0xFFFF;
    o += "HTTP " + String(code) + ", IPP " + ippStatusName(st) + ", " + String(millis() - t0) + " ms, " + String(body.size()) + " bytes\n" + txt + "\n";
    o += "\nRaw reply as a file: /printerraw\n";
  }
  xSemaphoreGive(printerMtx);
  server.send(200, "text/plain", o);
}

// ---- touchscreen hooks (btl_screen.cpp calls these from its own task)
void screenGetInfo(ScreenInfo& si) {
  memset(&si, 0, sizeof(si));
  si.link = usbIppReady() ? 2 : printerFound ? 1 : 0;
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  si.queued = (uint8_t)min((size_t)99, jobs.size());
  strlcpy(si.lastName, lastPhotoName, sizeof(si.lastName));
  strlcpy(si.lastInfo, lastPhotoInfo, sizeof(si.lastInfo));
  xSemaphoreGive(dataMtx);
  if (dryRun) si.state = SCR_ST_TEST;
  else if (liveClient || printerState == 4) si.state = SCR_ST_PRINTING;
  else if (camBusy) si.state = SCR_ST_RECEIVING;
  else if (!si.link) si.state = SCR_ST_OFFLINE;
  else if (printerState == 5) si.state = SCR_ST_STOPPED;
  else si.state = SCR_ST_READY;
  si.inkLeft = printsLeft(); si.inkCap = cassCap; si.paperLeft = paperLeft(); si.paperCap = paperCap;
  si.warm = colWarm; si.con = colCon; si.sat = colSat;
  si.busy = camBusy;
}

void screenAction(int act, int v) {
  switch (act) {
    case SCR_ACT_WARM: colWarm = constrain(v, -5, 5); saveColor = true; break;
    case SCR_ACT_CON:  colCon = constrain(v, 0, 3); saveColor = true; break;
    case SCR_ACT_SAT:  colSat = constrain(v, 0, 3); saveColor = true; break;
    case SCR_ACT_NEW_INK: cassUsed = 0; saveCass = true; scrLogAct = act; break;
    case SCR_ACT_PAPER_LOADED: paperUsed = 0; saveCass = true; scrLogAct = act; break;
  }
}

static void handleStatus() {
  String j = "{";
  xSemaphoreTake(dataMtx, portMAX_DELAY);
  String pst = dryRun ? "TEST MODE - nothing will print" : printerState == 3 ? "ready" : printerState == 4 ? "printing" : printerState == 5 ? "stopped" : "unknown";
  String pn = usbIppReady() ? usbName + " (USB cable)"
            : printerFound ? printerName + " (Wi-Fi, " + printerIP.toString() + ")" : String("searching...");
  String us = usbState;
  if (uReady && !useUsb) us += " (turned off in settings, using Wi-Fi)";
  j += "\"printer\":\"" + jsonEsc(pn) + "\",\"usb\":\"" + jsonEsc(us) + "\",\"pstate\":\"" + pst + "\",\"reasons\":\"" + jsonEsc(printerReasons) +
       "\",\"queue\":" + String(jobs.size()) + ",\"left\":" + String(printsLeft()) + ",\"cap\":" + String(cassCap) + ",\"paper\":" + String(paperLeft()) + ",\"log\":[";
  for (size_t i = 0; i < logLines.size(); i++) { if (i) j += ","; j += "\"" + jsonEsc(logLines[i]) + "\""; }
  xSemaphoreGive(dataMtx);
  j += "]}";
  server.send(200, "application/json", j);
}

static void handlePaper() {
  int c = server.arg("cap").toInt();
  if (c >= 1 && c <= 40) { paperCap = c; prefs.putInt("pcap", paperCap); }
  paperUsed = 0; prefs.putInt("pused", 0);
  blog("Paper loaded: %d sheets", paperCap);
  redirectHome();
}

static void handleCassette() {
  int c = server.arg("cap").toInt();
  if (c >= 1 && c <= 200) { cassCap = c; prefs.putInt("ccap", cassCap); }
  if (server.hasArg("new")) { cassUsed = 0; prefs.putInt("cused", 0); blog("New ink cassette: %d prints", cassCap); }
  else blog("Ink cassette size set to %d prints (%d left)", cassCap, printsLeft());
  redirectHome();
}

static void handleSettings() {
  printMode = server.arg("mode") == "rating" ? 1 : 0;
  int r = server.arg("minr").toInt();
  if (r >= 1 && r <= 5) minRating = r;
  if (server.arg("fuser").length()) ftpUser = server.arg("fuser");
  if (server.arg("fpass").length()) ftpPass = server.arg("fpass");
  shrinkOn = server.hasArg("shrink");
  fillOn = server.hasArg("fill"); prefs.putBool("fill", fillOn);
  scaleFill = server.hasArg("pscale"); prefs.putBool("pscale", scaleFill);
  dryRun = server.hasArg("dry"); prefs.putBool("dry", dryRun);
  useUsb = server.hasArg("usb");
  speedMode = server.arg("speed").toInt() == 1 ? 1 : 0;
  prefs.putBool("usb", useUsb); prefs.putInt("speed", speedMode);
  colWarm = constrain(server.arg("cwarm").toInt(), -5, 5);
  colCon  = constrain(server.arg("ccon").toInt(), 0, 3);
  colSat  = constrain(server.arg("csat").toInt(), 0, 3);
  prefs.putInt("cwarm", colWarm); prefs.putInt("ccon", colCon); prefs.putInt("csat", colSat);
  buildColor();
  int q = server.arg("q").toInt();
  if (q >= 50 && q <= 100) jpegQ = q;
  prefs.putBool("shrink", shrinkOn); prefs.putInt("q", jpegQ);
  prefs.putInt("mode", printMode); prefs.putInt("minr", minRating);
  prefs.putString("fuser", ftpUser); prefs.putString("fpass", ftpPass);
  blog("Settings saved: %s", printMode == 1 ? ("only " + stars(minRating) + " and up").c_str() : "print every photo");
  redirectHome();
}

// test print upload -> queue
uint8_t* upBuf = nullptr; size_t upLen = 0, upCap = 0; bool upFail = false; String upName;
// Biggest file the test-print box can take: it has to fit in memory whole, with room left to resize it.
static size_t testPrintMax() { size_t m = spoolLimit(); return m > 2560 * 1024 ? m - 2560 * 1024 : 0; }
static void handleTestUpload() {
  HTTPUpload& up = server.upload();
  if (up.status == UPLOAD_FILE_START) {
    if (upBuf) { free(upBuf); upBuf = nullptr; }
    upLen = upCap = 0; upFail = false; upName = up.filename;
  } else if (up.status == UPLOAD_FILE_WRITE) {
    if (upFail) return;
    if (upLen + up.currentSize > upCap) {
      size_t nc = upCap + 1024 * 1024;                   // grow 1 MB at a time (doubling ran out of room at ~4 MB)
      while (nc < upLen + up.currentSize) nc += 1024 * 1024;
      uint8_t* nb = (nc <= testPrintMax() + upCap) ? (uint8_t*)bigRealloc(upBuf, nc) : nullptr;
      if (!nb) { upFail = true; return; }
      upBuf = nb; upCap = nc;
    }
    memcpy(upBuf + upLen, up.buf, up.currentSize);
    upLen += up.currentSize;
  } else if (up.status == UPLOAD_FILE_END) {
    if (upFail || !upLen) {
      if (upBuf) free(upBuf);
      upBuf = nullptr; upLen = 0;
      blog("Test print: that file is too big for the bridge's memory (it takes up to %s here). Send it from the camera instead - that path streams.", fmtSize(testPrintMax()).c_str());
    }
  } else if (up.status == UPLOAD_FILE_ABORTED) {
    if (upBuf) free(upBuf);
    upBuf = nullptr; upLen = 0;
  }
}

static void handleTestPrintDone() {
  if (upBuf && upLen) {
    String info;
    bool shrunk = shrinkBuffer(upBuf, upLen, info);
    String nm = upName.length() ? upName : String("test.jpg");
    enqueueJob(upBuf, upLen, nm, -1);
    blog("Test print queued: %s (%s)", nm.c_str(), shrunk ? info.c_str() : fmtSize(upLen).c_str());
    upBuf = nullptr; upLen = 0;
  }
  redirectHome();
}

// Returns the photo exactly as it would be sent to the printer, without printing it.
static void handlePreviewDone() {
  if (!upBuf || !upLen) { server.send(400, "text/plain", "No JPEG received"); return; }
  String info;
  bool shrunk = shrinkBuffer(upBuf, upLen, info);
  server.sendHeader("X-Shrink", shrunk ? info : ("not shrunk: " + info));
  server.send_P(200, "image/jpeg", (const char*)upBuf, upLen);
  free(upBuf); upBuf = nullptr; upLen = 0;
}

static void startAccessPoint() {
  if (!apOn) { WiFi.softAPdisconnect(true); return; }
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255, 255, 255, 0));
  WiFi.softAP(apSsid.c_str(), apPass.length() >= 8 ? apPass.c_str() : nullptr, 1, 0, 4);
}

static void handleApSettings() {
  bool on = server.hasArg("apon");
  String ssid = server.arg("apssid"), pass = server.arg("appass");
  if (!ssid.length() || ssid.length() > 31) { server.send(400, "text/plain", "Network name: 1-31 characters"); return; }
  if (pass.length() && pass.length() < 8) { server.send(400, "text/plain", "Password: at least 8 characters (or empty for an open network)"); return; }
  apOn = on; apSsid = ssid; apPass = pass;
  prefs.putBool("apon", apOn); prefs.putString("apssid", apSsid); prefs.putString("appass", apPass);
  startAccessPoint();
  blog(apOn ? "Camera network: %s, address %s" : "Camera network turned off", apSsid.c_str(), AP_IP.toString().c_str());
  redirectHome();
}

static void handleSaveWifi() {
  String s = server.arg("ssid"), p = server.arg("pass");
  if (!s.length()) { server.send(400, "text/plain", "Wi-Fi name is required"); return; }
  prefs.putString("ssid", s); prefs.putString("pass", p);
  server.send(200, "text/html",
    "<meta charset=utf-8><meta name=viewport content='width=device-width,initial-scale=1'>"
    "<body style='font-family:sans-serif;background:#111;color:#eee;padding:16px'><h2>Saved - restarting</h2>"
    "<p>Open <b>http://btl-photo.local</b> from your home Wi-Fi in about a minute (or http://192.168.4.1 from the camera network).</p>");
  delay(1500);
  ESP.restart();
}

// =============================================================== Wi-Fi
ImprovSerial improv(Serial);
uint32_t restartAt = 0;                         // set when new Wi-Fi was saved; restart into normal mode
char improvName[80] = "BTL Photo Printer Bridge";

static void improvShowAddress() {               // the installer shows this as the device name
  snprintf(improvName, sizeof(improvName), "BTL Photo Printer Bridge at http://%s", WiFi.localIP().toString().c_str());
  improv.setDeviceName(improvName);
}

// Called by the web installer with the Wi-Fi name and password typed on the install page.
bool improvConnect(const char *ssid, const char *password) {
  WiFi.disconnect();
  WiFi.begin(ssid, password);
  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) delay(250);
  if (WiFi.status() != WL_CONNECTED) return false;
  prefs.putString("ssid", ssid); prefs.putString("pass", password);
  improvShowAddress();
  blog("Wi-Fi set from the installer: %s, %s", ssid, WiFi.localIP().toString().c_str());
  restartAt = millis() + 4000;                  // after the installer shows "connected"
  return true;
}

static void serviceImprov() {
  improv.handle();
  if (restartAt && (int32_t)(millis() - restartAt) > 0) ESP.restart();
}

// Joins the home Wi-Fi if one is saved. The camera network (access point) is already up by then.
static bool connectWifi() {
  String s = prefs.getString("ssid", ""), p = prefs.getString("pass", "");
  if (s.length()) {
    WiFi.begin(s.c_str(), p.c_str());
  } else if (strlen(WIFI_SSID)) {
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  } else {
    wifi_config_t conf;                         // network saved by an earlier sketch
    if (esp_wifi_get_config(WIFI_IF_STA, &conf) == ESP_OK && conf.sta.ssid[0]) {
      char ss[33] = {0}, pp[65] = {0};
      memcpy(ss, conf.sta.ssid, 32); memcpy(pp, conf.sta.password, 64);
      WiFi.begin(ss, pp);
    } else {
      bootMark("no saved Wi-Fi");
      return false;
    }
  }
  bootMark("connecting to saved Wi-Fi");
  Serial.println("Connecting to Wi-Fi");          // no dots: they'd hide the installer's replies
  uint32_t t = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t < 20000) { serviceImprov(); delay(10); }
  return WiFi.status() == WL_CONNECTED;
}

// =============================================================== setup / loop
static void bootMark(const char* what) {         // progress markers in Serial Monitor, to see where a boot stops
  Serial.printf("[boot %lu ms] %s  (heap %u)\n", (unsigned long)millis(), what, (unsigned)ESP.getFreeHeap());
  Serial.flush();
}

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\nBTL Photo Printer Bridge v" FW_VERSION);
  Serial.printf("[boot] %s rev %d, flash %u MB, PSRAM %u MB, reset reason %d\n", ESP.getChipModel(), ESP.getChipRevision(),
                (unsigned)(ESP.getFlashChipSize() >> 20), (unsigned)(ESP.getPsramSize() >> 20), (int)esp_reset_reason());
  Serial.flush();
  printerMtx = xSemaphoreCreateMutex();
  dataMtx = xSemaphoreCreateMutex();

  prefs.begin("btlphoto", false);
  printMode = prefs.getInt("mode", 0);
  minRating = prefs.getInt("minr", 3);
  shrinkOn = prefs.getBool("shrink", true);
  fillOn = prefs.getBool("fill", true);
  apOn = prefs.getBool("apon", true);
  apSsid = prefs.getString("apssid", "BTL-Photo");
  apPass = prefs.getString("appass", "printphoto");
  scaleFill = prefs.getBool("pscale", true);
  dryRun = prefs.getBool("dry", false);
  jpegQ = prefs.getInt("q", 85);
  useUsb = prefs.getBool("usb", true);
  speedMode = prefs.getInt("speed", 0);
  cassCap = prefs.getInt("ccap", 54); cassUsed = prefs.getInt("cused", 0); paperCap = prefs.getInt("pcap", 18); paperUsed = prefs.getInt("pused", 0);
  colWarm = prefs.getInt("cwarm", 0); colCon = prefs.getInt("ccon", 0); colSat = prefs.getInt("csat", 0);
  buildColor();
  ftpUser = prefs.getString("fuser", "btl");
  ftpPass = prefs.getString("fpass", "print");

  if (psramFound()) reserveResampler();            // while internal RAM is still clean
  liveBuf = (uint8_t*)bigAlloc(LIVE_CHUNK + 32);
  bootMark("settings loaded");
  improv.begin("BTL Photo Printer Bridge", FW_VERSION, "ESP32-S3", improvName, improvConnect);
  prevLog = esp_log_set_vprintf(logHook);
  esp_log_level_set("*", ESP_LOG_INFO);           // chatty until Wi-Fi is up, so boot problems show in Serial Monitor
  usbStart();                                     // before Wi-Fi, same order as the label bridge
  bootMark("USB host starting");
  WiFi.setHostname(HOSTNAME);
  WiFi.mode(apOn ? WIFI_AP_STA : WIFI_STA);
  WiFi.setSleep(false);
  bootMark("Wi-Fi started");
  startAccessPoint();
  if (apOn) bootMark("camera network up");
  bool wifiOk = connectWifi();
  esp_log_level_set("*", ESP_LOG_ERROR);
  bootMark(wifiOk ? "home Wi-Fi connected" : "no home Wi-Fi (keeps trying in the background)");
  if (wifiOk) improvShowAddress();

  if (screenBegin()) blog("Screen: 2.8\" touchscreen found");
  else blog("Screen: none connected");
  if (apOn) blog("Camera network %s: address %s", apSsid.c_str(), AP_IP.toString().c_str());
  if (wifiOk) blog("Bridge ready at http://%s  (PSRAM %s, %u KB internal RAM free, resizer buffers %s)", WiFi.localIP().toString().c_str(), psramFound() ? "on" : "OFF",
                   (unsigned)(heap_caps_get_free_size(MALLOC_CAP_INTERNAL) / 1024), resSums ? "reserved" : "NOT reserved");
  else blog("Home Wi-Fi not connected - page at http://%s on the camera network  (PSRAM %s)", AP_IP.toString().c_str(), psramFound() ? "on" : "OFF");
  MDNS.begin(HOSTNAME);
  MDNS.addService("http", "tcp", 80);
  ftpCtl.begin();
  camData.listen(FTP_PASV_PORT);

  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/settings", HTTP_POST, handleSettings);
  server.on("/cassette", HTTP_POST, handleCassette);
  server.on("/paper", HTTP_POST, handlePaper);
  server.on("/screencal", HTTP_POST, []() { screenRecalibrate(); blog("Screen: touch setup started - tap the crosses on the screen"); redirectHome(); });
  server.on("/rediscover", HTTP_POST, []() { printerFound = false; redirectHome(); });
  server.on("/usbcheck", HTTP_POST, handleUsbCheck);
  server.on("/usbdiag", HTTP_GET, []() { server.send(200, "text/plain", usbDiagText()); });
  server.on("/printerinfo", HTTP_GET, handlePrinterInfo);
  server.on("/printerraw", HTTP_GET, []() { handlePrinterInfoRaw(true); });
  server.on("/usbreset", HTTP_POST, []() {
    if (usbHostStarted) {
      blog("USB: switching the port off and on");
      usb_host_lib_set_root_port_power(false);
      delay(800);
      usb_host_lib_set_root_port_power(true);
    }
    redirectHome();
  });
  server.on("/clearhist", HTTP_POST, []() {
    xSemaphoreTake(dataMtx, portMAX_DELAY); printedNames.clear(); xSemaphoreGive(dataMtx);
    blog("Printed-photo history cleared"); redirectHome();
  });
  server.on("/testprint", HTTP_POST, handleTestPrintDone, handleTestUpload);
  server.on("/preview", HTTP_POST, handlePreviewDone, handleTestUpload);
  server.on("/savewifi", HTTP_POST, handleSaveWifi);
  server.on("/apsettings", HTTP_POST, handleApSettings);
  server.begin();

  xTaskCreatePinnedToCoreWithCaps(printTask, "print", 12288, nullptr, 1, nullptr, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  xTaskCreatePinnedToCoreWithCaps(ftpTask, "ftp", 16384, nullptr, 1, nullptr, 1, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
}

void loop() {
  serviceImprov();
  server.handleClient();
  if (saveScaleFill) { saveScaleFill = false; prefs.putBool("pscale", scaleFill); }
  if (saveColor) { saveColor = false; buildColor(); prefs.putInt("cwarm", colWarm); prefs.putInt("ccon", colCon); prefs.putInt("csat", colSat); }
  if (scrLogAct) { int a = scrLogAct; scrLogAct = 0;
    if (a == SCR_ACT_NEW_INK) blog("New ink cassette (from the screen): %d prints", cassCap);
    else blog("Paper loaded (from the screen): %d sheets", paperCap); }
  if (saveCass) { saveCass = false; prefs.putInt("cused", cassUsed); prefs.putInt("pused", paperUsed);
    if (paperLeft() == 3) blog("Paper tray: 3 sheets left");
    if (paperLeft() == 0) blog("Paper tray: count says it's empty - refill and press Paper loaded");
    if (printsLeft() == 5) blog("Ink cassette: 5 prints left - have a new pack ready");
    if (printsLeft() == 0) blog("Ink cassette: count says it's used up - put in a new pack and press New cassette"); }
  static bool wasUp = false;                      // home Wi-Fi can come and go; note it in the log
  bool up = WiFi.status() == WL_CONNECTED;
  if (up != wasUp) {
    wasUp = up;
    if (up) { improvShowAddress(); blog("Home Wi-Fi connected: http://%s", WiFi.localIP().toString().c_str()); }
    else blog("Home Wi-Fi lost - reconnecting in the background (camera network keeps working)");
  }
  delay(2);
}
