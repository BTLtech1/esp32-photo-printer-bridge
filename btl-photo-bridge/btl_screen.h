// BTL Photo Printer Bridge - optional 2.8" touchscreen (ILI9341 + XPT2046, SPI).
//
// Shows what the bridge is doing (state, the photo being printed, ink and paper left) and lets
// you change the color settings and reset the counters by touch. Works the same with no screen
// connected: if the screen doesn't answer at startup, this module stays out of the way.
//
// Wiring (screen pin -> ESP32-S3 GPIO):
//   VCC -> 3V3, GND -> GND, CS -> 10, RESET -> 14, DC -> 9, SDI(MOSI) -> 11, SCK -> 12,
//   LED -> 21, SDO(MISO) -> 13, T_CLK -> 12, T_CS -> 8, T_DIN -> 11, T_DO -> 13, T_IRQ -> 7
//
// MIT License. Made for the BTL Photo Printer Bridge.
#pragma once
#include <Arduino.h>

#define SCR_PIN_SCK   12
#define SCR_PIN_MOSI  11
#define SCR_PIN_MISO  13
#define SCR_PIN_CS    10
#define SCR_PIN_DC     9
#define SCR_PIN_RST   14
#define SCR_PIN_BL    21
#define SCR_PIN_TCS    8
#define SCR_PIN_TIRQ   7

// What the screen shows. Filled in by the sketch (screenGetInfo) a few times a second.
struct ScreenInfo {
  uint8_t link;           // 0 = no printer, 1 = Wi-Fi, 2 = USB
  uint8_t state;          // see SCR_ST_*
  uint8_t queued;         // photos waiting
  char    lastName[40];   // last photo printed (or processed in test mode)
  char    lastInfo[40];   // e.g. "3.4 s"
  int16_t inkLeft, inkCap, paperLeft, paperCap;
  int8_t  warm, con, sat; // color settings (-5..5, 0..3, 0..3)
  uint8_t busy;           // a photo is being processed: redraw less often so printing isn't slowed
  uint8_t dest;           // where unrated photos print: 1 = SELPHY, 2 = label, 3 = both
  uint8_t wm;             // corner text: 0 off, 1 on, 2 no text set
  char    why[24];        // why the printer stopped ("OUT OF PAPER"), empty if unknown
};
enum { SCR_ST_OFFLINE = 0, SCR_ST_READY, SCR_ST_RECEIVING, SCR_ST_PRINTING, SCR_ST_TEST, SCR_ST_STOPPED };

// Things the screen asks the sketch to do.
enum { SCR_ACT_WARM = 1, SCR_ACT_CON, SCR_ACT_SAT, SCR_ACT_NEW_INK, SCR_ACT_PAPER_LOADED, SCR_ACT_DEST, SCR_ACT_WM };

// Implemented by the sketch.
void screenGetInfo(ScreenInfo& s);
void screenAction(int action, int value);

// Starts the screen task. Returns true if a screen answered.
bool screenBegin();
bool screenPresent();
// Wakes the backlight (e.g. a photo arrived).
void screenWake();
// Shows the touch calibration on the screen next time it draws.
void screenRecalibrate();

// Thumbnail of the photo being made, fed from the resizer's output rows.
// orient: EXIF orientation of the rows (1 = as is, 3 = 180, 6 = needs 90 clockwise, 8 = needs 90 counter-clockwise).
void screenThumbBegin(int w, int h, int orient);
void screenThumbRows(const uint8_t* px, int y0, int rows, bool bgr);
void screenThumbEnd();
