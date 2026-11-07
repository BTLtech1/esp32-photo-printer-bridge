# BTL Photo Printer Bridge

Print straight from your camera with a ~$10 ESP32-S3 board. Review a shot, press **SET**, and it comes out of the printer.

- **Camera to printer, no phone or computer.** The camera sends the photo over Wi-Fi (Canon's built-in FTP transfer) and the bridge prints it.
- **USB cable or Wi-Fi to the printer.** Over USB ("IPP over USB") the print starts sooner; over Wi-Fi there's nothing to plug in.
- **Smart sizing.** Each photo is shrunk to the printer's real resolution (1800 x 1200 on a 4x6 at 300 dpi) and turned upright before it's sent, so big camera files print fast. **Fast** mode (1200 x 800) starts even sooner. Photos that aren't 3:2 are trimmed to fill the paper (or kept whole with white strips, your choice).
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

## Install (no Arduino needed)

Open **https://btltech1.github.io/esp32-photo-printer-bridge** in Chrome or Edge, plug the board's **UART** port into your computer and click **Install**.

Right after installing, the page asks for your Wi-Fi (2.4 GHz, the same network as the printer). Once it's connected, **Visit Device** opens the bridge's page, which shows its IP address for the camera. Later you can also use **http://btl-photo.local**.

If the page sits on "Wrapping up" for more than a minute (some USB-serial chips don't let the browser reopen the port straight after flashing), the firmware is already installed: reload the page, click **Install** again, pick the port, and choose **Connect to Wi-Fi**. The bridge also creates a Wi-Fi network called **BTL-Photo-Setup**: join it and go to `http://192.168.4.1` to enter your Wi-Fi there instead.

## Camera setup (Canon)

In the camera's network menu, create an **FTP transfer** connection over your Wi-Fi:

| Setting | Value |
|---|---|
| FTP mode | FTP (not FTPS or SFTP) |
| Address | the bridge's IP address (shown on its page) |
| Port | 21 |
| Passive mode | Enable |
| Proxy | Disable |
| Login | user `btl`, password `print` (change them on the bridge's page) |
| Target folder | Root folder |

Then in **FTP transfer settings**: Automatic transfer **Disable**, Transfer with SET **Enable**, and send **JPEG** only. Press **Playback**, pick a photo, press **SET**.

Reserve the bridge's IP address in your router so the camera always finds it.

## Printing over USB

Over Wi-Fi the SELPHY reads photo data slowly. Plugging it into the bridge by USB starts prints sooner.

**Wiring:** board port **USB** &rarr; USB-C to USB-A adapter &rarr; USB-A to USB-C cable &rarr; printer. Power the board from its **UART/COM** port.

Three things have to be right:

1. **5 V on the board's USB port.** Bridge the two small **OTG** solder pads on the back of the board. After that, never plug that port into a computer or charger.
2. **The right cable.** The SELPHY only turns on its USB connection when the cable tells it a computer is attached. That signal is a small resistor inside the USB-C plug of a proper USB-A to USB-C cable, and cheap cables often leave it out. Test the cable on a computer first: the printer should show up in Device Manager.
3. **Not a USB-C to USB-C cable.** The SELPHY's port can act as a host itself (for flash drives and cameras), so with C-to-C both sides try to be in charge.

The bridge's page shows "SELPHY CP1500 (USB cable)" when it's connected, and **Check the printer over USB** asks the printer for its status without printing. **USB diagnostics** shows what the board sees on its USB port.

## Build it yourself (Arduino IDE)

1. **Boards Manager:** install **esp32 by Espressif**, version **3.1.1**. The newer 3.3.x versions don't finish connecting USB devices yet.
2. Open `btl-photo-bridge/btl-photo-bridge.ino`.
3. **Tools:** Board **ESP32S3 Dev Module**, USB CDC On Boot **Disabled**, USB Mode **Hardware CDC and JTAG**, PSRAM **OPI PSRAM**.
4. Plug in the board's **UART** port and upload.

## Troubleshooting

| Problem | Fix |
|---|---|
| Camera says it can't connect | Check the address on the camera matches the bridge's page, and that both are on the same Wi-Fi. |
| Camera's SET option is missing | The camera isn't connected to the bridge. Turn the camera off and on. |
| Photo is "Skipped - not a JPEG" | Set the camera to send JPEG only (RAW files can't be printed). |
| Printer not found over Wi-Fi | Connect the printer to the same Wi-Fi (on the SELPHY: Wi-Fi settings, Connection Settings, Other, Via Wireless Router). |
| USB row says "waiting for the printer" | See "Printing over USB": 5 V pads, the cable, and no C-to-C. |

## Credits

Made by [BTL](https://btlstore.com). I designed and tested this; the code was written with help from AI (Claude).

JPEG decoding uses [TJpgDec](http://elm-chan.org/fsw/tjpgd/) by ChaN.

## License

MIT. See `LICENSE`.
