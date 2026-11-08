# BTL Photo Printer Bridge

Print straight from your camera with a ~$10 ESP32-S3 board. Review a shot, press **SET**, and it comes out of the printer.

- **Camera to printer, no phone, computer or router.** The bridge runs its own Wi-Fi network; the camera joins it and sends each photo over Canon's built-in FTP transfer. Works at home or in the field.
- **USB cable or Wi-Fi to the printer.** Over USB ("IPP over USB") the print starts sooner; over Wi-Fi there's nothing to plug in.
- **Smart sizing.** Each photo is shrunk to the printer's native print size (1872 x 1248 dots for a borderless 4x6) before it's sent, so big camera files print fast and the printer has nothing left to scale. Photos that aren't 3:2 are trimmed to fill the paper (or kept whole with white strips, your choice).
- **Two print modes.** Print every photo you send, or only photos you've rated in camera (for example 3 stars and up).
- **Print queue.** Send several photos in a row; they wait their turn while the printer is busy.
- **Status page.** See the printer, the activity log and the exact camera settings at `http://btl-photo.local`.

Tested with a **Canon EOS R6 Mark II** and a **Canon SELPHY CP1500**. Other cameras with FTP transfer and other printers that support IPP (AirPrint/Mopria) should work too; more printers are planned.

---

## What you need

| Part | Notes |
|---|---|
| **ESP32-S3** board with two USB-C ports | "ESP32-S3 N16R8" (16 MB flash, 8 MB PSRAM). PSRAM is needed for the print queue. |
| A camera with FTP transfer | Canon EOS R6 Mark II, R5, R3 and others. |
| A photo printer with IPP | Canon SELPHY CP1500 (Wi-Fi and USB). |
| For USB printing | A USB-C to USB-A (OTG) adapter and a good USB-A to USB-C cable. See "Printing over USB". |
| Optional touchscreen | 2.8" ILI9341 SPI TFT with XPT2046 touch, 240x320 (the common red board). See "Touchscreen". |

## Install (no Arduino needed)

Open **https://btltech1.github.io/esp32-photo-printer-bridge** in Chrome or Edge, plug the board's **UART** port into your computer and click **Install**.

The bridge starts its own Wi-Fi network, **BTL-Photo** (password `printphoto`). Its page is at **http://192.168.4.1** on that network, and that's also the address the camera sends to. Nothing else is required for printing over the USB cable.

Right after installing, the page also offers to join your home Wi-Fi (2.4 GHz). That's optional: it lets the bridge reach a printer on your Wi-Fi and lets you open its page (**http://btl-photo.local**) from your normal network. Once connected, the installer's dialog shows the bridge's home address next to its name.

If the page sits on "Wrapping up" for more than a minute (some USB-serial chips don't let the browser reopen the port straight after flashing), the firmware is already installed: reload the page, click **Install** again, pick the port, and choose **Connect to Wi-Fi**. The bridge also creates a Wi-Fi network called **BTL-Photo-Setup**: join it and go to `http://192.168.4.1` to enter your Wi-Fi there instead.

## Camera setup (Canon)

In the camera's network menu, create an **FTP transfer** connection. For the Wi-Fi network pick **BTL-Photo** (password `printphoto`); your home Wi-Fi also works if the bridge is on it.

| Setting | Value |
|---|---|
| FTP mode | FTP (not FTPS or SFTP) |
| Address | **192.168.4.1** on BTL-Photo (or the bridge's home address, shown on its page) |
| Port | 21 |
| Passive mode | Enable |
| Proxy | Disable |
| Login | user `btl`, password `print` (change them on the bridge's page) |
| Target folder | Root folder |

Then in **FTP transfer settings**: Automatic transfer **Disable**, Transfer with SET **Enable**, and send **JPEG** only. Press **Playback**, pick a photo, press **SET**.

On the BTL-Photo network the address never changes. If you use your home Wi-Fi instead, reserve the bridge's IP address in your router.

## Printing over USB

Over Wi-Fi the SELPHY reads photo data slowly. Plugging it into the bridge by USB starts prints sooner.

**Wiring:** board port **USB** &rarr; USB-C to USB-A adapter &rarr; USB-A to USB-C cable &rarr; printer. Power the board from its **UART/COM** port.

Three things have to be right:

1. **5 V on the board's USB port.** Bridge the two small **OTG** solder pads on the back of the board. After that, never plug that port into a computer or charger.
2. **The right cable.** The SELPHY only turns on its USB connection when the cable tells it a computer is attached. That signal is a small resistor inside the USB-C plug of a proper USB-A to USB-C cable, and cheap cables often leave it out. Test the cable on a computer first: the printer should show up in Device Manager.
3. **Not a USB-C to USB-C cable.** The SELPHY's port can act as a host itself (for flash drives and cameras), so with C-to-C both sides try to be in charge.

The bridge's page shows "SELPHY CP1500 (USB cable)" when it's connected, and **Check the printer over USB** asks the printer for its status without printing. **USB diagnostics** shows what the board sees on its USB port.

## Touchscreen (optional)

A 2.8" touchscreen on the bridge shows what it's doing without opening the web page:

- **READY / RECEIVING / PRINTING / TEST MODE / NO PRINTER**, plus how many photos are waiting
- a preview of the photo being printed, with the color settings applied
- **ink and paper left** (the bridge counts prints, because the SELPHY doesn't report them)
- **Settings:** warmth, contrast and color by touch, plus **New ink** and **Paper loaded** (tap twice to confirm)

The screen turns off after 3 minutes with nothing happening and wakes on a touch or a new photo. With no screen connected, the bridge works exactly the same.

**Wiring** (screen pin &rarr; ESP32-S3 GPIO). The touch pins share the screen's SPI wires.

| Screen | GPIO | | Screen | GPIO |
|---|---|---|---|---|
| VCC | 3V3 | | SDO (MISO) | 13 |
| GND | GND | | LED | 21 |
| CS | 10 | | T_CLK | 12 |
| RESET | 14 | | T_CS | 8 |
| DC | 9 | | T_DIN | 11 |
| SDI (MOSI) | 11 | | T_DO | 13 |
| SCK | 12 | | T_IRQ | 7 |

The first time it starts, the screen asks you to tap three crosses so it knows where your touches land. To redo that later, use **Redo screen touch setup** on the bridge's page or **Touch setup** in the screen's settings.

## Build it yourself (Arduino IDE)

1. **Boards Manager:** install **esp32 by Espressif**, version **3.1.1**. The newer 3.3.x versions don't finish connecting USB devices yet.
2. Copy the folder `libraries/esp_new_jpeg` from this repository into your sketchbook's `libraries` folder (usually `Documents/Arduino/libraries`). It's Espressif's fast JPEG codec, precompiled for the ESP32-S3.
3. Open `btl-photo-bridge/btl-photo-bridge.ino`.
4. **Tools:** Board **ESP32S3 Dev Module**, USB CDC On Boot **Disabled**, USB Mode **Hardware CDC and JTAG**, PSRAM **OPI PSRAM**.
5. Plug in the board's **UART** port and upload.

## Troubleshooting

| Problem | Fix |
|---|---|
| Camera says it can't connect | On BTL-Photo the address is 192.168.4.1. On home Wi-Fi, check the address on the camera matches the bridge's page. |
| Camera's SET option is missing | The camera isn't connected to the bridge. Turn the camera off and on. |
| Photo is "Skipped - not a JPEG" | Set the camera to send JPEG only (RAW files can't be printed). |
| Printer not found over Wi-Fi | Connect the printer to the same Wi-Fi (on the SELPHY: Wi-Fi settings, Connection Settings, Other, Via Wireless Router). |
| USB row says "waiting for the printer" | See "Printing over USB": 5 V pads, the cable, and no C-to-C. |

## Credits

Made by [BTL](https://btlstore.com). I designed and tested this; the code was written with help from AI (Claude).

JPEG decoding and encoding use Espressif's [esp_new_jpeg](https://github.com/espressif/esp-adf-libs/tree/master/esp_new_jpeg) (precompiled, ESPRESSIF MIT License), with [TJpgDec](http://elm-chan.org/fsw/tjpgd/) by ChaN as the fallback for photos it can't handle.

The touchscreen uses the [Inter](https://rsms.me/inter/) typeface by Rasmus Andersson (SIL Open Font License 1.1), converted with `tools/make_font.py`.

## License

MIT. See `LICENSE`.
