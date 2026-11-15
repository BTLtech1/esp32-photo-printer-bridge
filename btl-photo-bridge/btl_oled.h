// BTL Photo Printer Bridge - small 128x64 OLED (SSD1306 0.96" or SH1106 1.3", I2C).
//
// Shows what matters in the field: prints left in the ink cassette, sheets left in the tray, and
// one status line (READY, PRINTING, OUT OF PAPER...). Uses the same ScreenInfo as the touchscreen.
// Wire it to IO4 = SDA and IO5 = SCL.
//
// MIT License. Made for the BTL Photo Printer Bridge.
#pragma once
#include <stdint.h>
#include "btl_screen.h"

#define OLED_PIN_SDA 4
#define OLED_PIN_SCL 5

// Finds the OLED (address 0x3C or 0x3D) and starts its task. Returns false if none answers.
bool oledBegin();
bool oledPresent();
// Shows a short message over the normal page for ms milliseconds (e.g. "New ink: 54").
void oledNote(const char* line1, const char* line2, uint32_t ms);
// Wakes the display if it dimmed.
void oledWake();

// Drawing, kept separate so it can be tested on a computer: renders the page into a 128x64
// 1-bit buffer (8 rows per byte, the OLED's own layout). note1/note2 may be null.
void oledRender(uint8_t* fb, const ScreenInfo& s, const char* note1, const char* note2, uint32_t ms);
