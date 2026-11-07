/*----------------------------------------------*/
/* TJpgDec configuration for BTL Selphy Bridge  */
/* (TJpgDec R0.03 (C)ChaN, 2021 - see btl_tjpgd.c) */
/*----------------------------------------------*/
#define JD_SZBUF        4096   /* stream input buffer size */
#define JD_FORMAT       0      /* 0: RGB888 output */
#define JD_USE_SCALE    1      /* allow 1/2, 1/4, 1/8 decoding */
#define JD_TBLCLIP      1      /* table saturation (faster) */
#define JD_FASTDECODE   2      /* fastest huffman decoding (32-bit MCU) */
#define JD_DEFAULT_HUFFMAN 0

/* Own names, so they never clash with the copies in the ESP32 ROM or Espressif's libraries. */
#define jd_prepare btl_jd_prepare
#define jd_decomp  btl_jd_decomp
