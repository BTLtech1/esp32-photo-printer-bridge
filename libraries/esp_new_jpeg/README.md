# esp_new_jpeg (precompiled, ESP32-S3)

Espressif's optimized JPEG decoder and encoder, unmodified, from
[esp-adf-libs](https://github.com/espressif/esp-adf-libs/tree/master/esp_new_jpeg), version 1.0.2.
It uses the ESP32-S3's vector instructions and decodes about ten times faster than a plain C decoder.

License: ESPRESSIF MIT License (see `LICENSE`): free to use on Espressif chips.

**Arduino IDE:** copy this folder into your sketchbook's `libraries` folder (for example
`Documents/Arduino/libraries/esp_new_jpeg`) so the bridge sketch can find `esp_jpeg_dec.h`.
