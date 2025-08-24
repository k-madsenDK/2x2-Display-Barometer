/* ============================================================================

  SBComputer 2x2 Barometer/Clock for RP2040 Pico (dual-core)
  Target: Tachyon device variant with 4x ST7789 240x240 TFT displays

  What it does
  - Shows humidity, temperature (with compensation), pressure (hPa), and a clock.
  - Clock has monthly background BMPs loaded from SD.
  - Analog clock hands (optional), time/date/day overlays.
  - Web UI to view status, set backlight, and manage calibration (incl. auto-cal).
  - Calibration values persist to /calib.cfg on SD (Core1 owns SD).

  Architecture
  - Core1 (UI owner): Owns all TFTs and the SD card. Draws UI, loads/saves assets & calibration.
  - Core0 (IO/logic): BME280, RTC (DS3231) + NTP, WiFi + Web UI, and auto-calibration.

  Important
  - The CLOCK background is always mirrored in a static 240x240 RGB565 RAM buffer.
    All overlay background sampling and rectangle restores come from that RAM copy.

  Hardware (see README.md for full table)
  - MCU: Raspberry Pi Pico (RP2040), Earle Philhower Arduino-Pico core
  - Sensor: BME280 @ I2C0 (SDA=GP20, SCL=GP21)
  - Displays: 4x ST7789 240x240 on SPI0/SPI1 (Core1 dynamically remaps SPI0 as needed)
  - SD card: SPI0 default pins (SCK=18, MOSI=19, MISO=16, CS=17), 25 MHz
  - Backlight: PWM on GP14 (0..255)
  - Note: TEMP_DC (GP16) shares with SD MISO; Core1 remaps SPI0 safely.

  SD Card layout (see README.md)
  - /wifi.cfg     → SSID/PASS (optional)
  - /calib.cfg    → HEATK, BLCOMP (persisted/updated by firmware)
  - /img/*.bmp    → Month backgrounds + category art (24-bit BMP, up to 240x240)

  Build & dependencies
  - Board support: Arduino-Pico (Earle Philhower) for RP2040
  - Libs: Adafruit_GFX, Adafruit_ST7789, Adafruit_BME280, RTClib, WiFi(+Udp), NTPClient, SD, Ticker
  - Memory headroom (from your last compile):
      Sketch ~604 kB of 2049 kB
      Globals ~192 kB of 262 kB

  Quick configuration points (search for these):
  - USE_ANALOG_CLOCK: 0=text only, 1=analog only, 2=both (time is drawn above hands)
  - TFT_ROT: Display rotation (all four tiles use the same rotation)
  - PRESS_OFFSET: Site elevation correction for hPa

  Notes for makers
  - Strings on-screen are currently in Danish (e.g., “Fugtighed”, “Temp”, “Soendag”).
    See README.md “Change display text to English” for line references.
  - This sketch keeps comments in English for general readability, with minimal
    functional changes, so your device runs exactly as before.

  License
  - Add a LICENSE file to your repo (e.g., MIT). This file contains no license text.

============================================================================ */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <SD.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <NTPClient.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_BME280.h>
#include <RTClib.h>
#include <math.h>   // roundf, cosf, sinf, expf, fminf, fmaxf, fabsf
#include <Ticker.h>
#include <stdlib.h>  // strtod
#include <string.h>
#include "ClkBgInfo.h"
#include "Globals.h"

// RP2040 Pico SDK headers (Earle core)
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "pico/util/queue.h"

// Embedded RGB565 image for TEMP display (defines TEMP240_W, TEMP240_H, uint16_t temp240[])
#include "temp240.h"
#include "Cyw43Power.h"

// ===== Diagnostics (enable for bring-up, disable for silence) =====
#define DIAG_SD_PING_MS   0
#define DIAG_CYCLE_HPA    0

// Boot/setup diagnostics to locate where it locks
#define BOOT_DIAG 1
#if BOOT_DIAG
  #define DBG(msg) do { Serial.println(msg); Serial.flush(); } while(0)
#else
  #define DBG(msg) do {} while(0)
#endif

// ===== Clock mode selection =====
// 0 = text clock only (DOW + TIME + DATE)
// 1 = analog clock only (DOW + DATE, no TIME line)
// 2 = both analog and time text (TIME is drawn on top of hands)
#define USE_ANALOG_CLOCK 1

// =================== Pins and constants ===================
// All tiles use the same rotation. Adjust per physical mounting.
#define TFT_ROT 3

// HUMIDITY TFT (SPI1)
#define HUM_CS   13
#define HUM_DC   12
#define HUM_RST  -1
#define HUM_SCK  10
#define HUM_MOSI 11

// TEMP TFT (SPI0 default pins: 18/19), note: TEMP_DC=16 shares with SD MISO
#define TEMP_CS   28
#define TEMP_DC   16
#define TEMP_RST  -1
#define TEMP_SCK  18
#define TEMP_MOSI 19

// HPA TFT (SPI0 → pins 2/3 via dynamic mapping)
#define HPA_CS   5
#define HPA_DC   4
#define HPA_RST  -1
#define HPA_SCK  2
#define HPA_MOSI 3

// CLOCK TFT (SPI0 → pins 6/7 via dynamic mapping)
#define CLK_CS   9
#define CLK_DC   8
#define CLK_RST  -1
#define CLK_SCK  6
#define CLK_MOSI 7

// Extra CS (kept high)
#define TFT4_CS  28

// Backlight PWM (GP14), 0..255
#define BACKLIGHT 14

// SD on SPI0 (default pins)
#define SD_SCK  18
#define SD_MOSI 19
#define SD_MISO 16
#define SD_CS   17
#define SD_HZ   25000000U   // 25 MHz

// I2C0 for BME280 and DS3231
#define I2C_SDA 20
#define I2C_SCL 21
#define BME_ADDR 0x76

// Colors (RGB565)
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x001F
#define C_CYAN    0x07FF
#define C_YELLOW  0xFFE0
#define C_ORANGE  0xFD20

// Text size used on tiles
#define TXT_SIZE 3

// On-screen label/value placement (HUM/TEMP/HPA tiles)
#define F_LABEL_X  35
#define F_LABEL_Y  80
#define F_VALUE_X  60
#define F_VALUE_Y  120

#define T_LABEL_X  85
#define T_LABEL_Y  80
#define T_VALUE_X  75
#define T_VALUE_Y  120

#define P_LABEL_Y  60
#define P_VALUE_Y  100
#define P_STATE_Y  200

// Clock overlay Y positions (top/middle/bottom)
#define C_DOW_Y    6
#define C_TIME_Y   160
#define C_DATE_Y   212

// WiFi indicator (a tiny square on CLOCK)
#define IND_SIZE    12
#define IND_MARGIN   3
#define IND_Y        IND_MARGIN
#define IND_WIFI_X  (240 - IND_MARGIN - IND_SIZE)

// Analog clock hub size
#define center_size 12

// Pressure offset (site elevation etc.), tweak to your location
#define PRESS_OFFSET 7.0f

// Global text color (legacy) – not needed when using adaptive text; kept for compatibility
static uint16_t g_clkTextFgColor = C_BLACK;

// ===== Forward declarations / small helpers =====
struct DrawCmd;
struct Rect { int x, y, w, h; };
static inline Rect makeRect(int x, int y, int w, int h){ Rect r{ x,y,w,h }; return r; }
static inline void q_push(const DrawCmd& c);

// Text helpers
static inline int textPixelWidth(const char* s, int textSize);
static void drawCentered(Adafruit_ST7789* tft, int y, const char* txt, uint16_t fg, uint16_t bg, int size);

// Web handlers (Core0)
void handleIndex();
void handleStatusJson();
void handleBacklight();
void handleCalib();
void handleCalibJson();
void handleAutoCalibJson();
void handleAutoCalibStart();
void handleAutoCalibCancel();
void handleAutoCalibApply();

// Sensor/compensation helpers
static float estimateAmbientTemp(float t_meas);
static float correctedRH(float t_meas_C, float rh_meas_pct, float t_amb_C);

// Asset selectors
static const char* categoryBmp(int cat);
static const char* categoryText(int cat);
static const char* monthBmp(int month1_12);

// =================== Global instances / shared ===================

SPIClassRP2040& SPI0 = SPI;   // SD + HPA + CLOCK + TEMP (via dynamic remap)

// Core1-owned devices (TFTs)
static Adafruit_ST7789* tftHumid = nullptr; // SPI1
static Adafruit_ST7789* tftTemp  = nullptr; // SPI0
static Adafruit_ST7789* tftHpa   = nullptr; // SPI0
static Adafruit_ST7789* tftClk   = nullptr; // SPI0
static volatile bool sd_ready = false;      // Core1 sets when SD init OK

// Shared WiFi cfg between cores (Core1 reads SD early; Core0 connects)
volatile bool g_wifi_cfg_ready   = false;
volatile bool g_wifi_cfg_missing = false;
char g_wifi_ssid[64] = {0};
char g_wifi_pass[64] = {0};

// Core sync flags
volatile bool g_queue_ready = false;  // queue initialized by Core0
volatile bool g_core1_ready = false;  // Core1 finished HW init

// Backlight & calibration values (shared between cores)
uint8_t g_backlight   = 255;
float   g_tempHeatK   = 2.8f;
float   backlightcomp = 3.7f / 255.0f;   // shown in UI as “°C @255”; internal is “°C / PWM”

ClkBgInfo g_clkBgInfo; // Clock background header cache for SD-fallback restore

// =================== Cross-core draw command queue ===================

static queue_t g_q;
static inline void q_push(const DrawCmd& c) { queue_try_add(&g_q, &c); }

enum CmdType : uint8_t {
  CMD_NOP = 0,
  CMD_HUM_TEXT,
  CMD_TEMP_TEXT,
  CMD_HPA_TEXT,
  CMD_HPA_BG_CAT,     // ivalue: 0..4
  CMD_CLK_BG_MONTH,   // ivalue: 1..12
  CMD_CLK_TEXT_DOW,
  CMD_CLK_TEXT_TIME,
  CMD_CLK_TEXT_DATE,
  CMD_CLK_IND_WIFI,   // ivalue: RGB565 color
  CMD_SAVE_CALIB,     // Core1 saves /calib.cfg (Core1 owns SD)
  CMD_CLK_ANALOG_HMS  // ivalue: (hh<<16)|(mm<<8)|ss
};
struct DrawCmd {
  uint8_t  type;
  int32_t  ivalue;
  char     payload[24];
};

// =================== Core1 helpers: SPI routing and IO ===================

static inline void force_all_tft_cs_high() {
  pinMode(HUM_CS, OUTPUT); digitalWrite(HUM_CS, HIGH);
  pinMode(HPA_CS, OUTPUT); digitalWrite(HPA_CS, HIGH);
  pinMode(CLK_CS, OUTPUT); digitalWrite(CLK_CS, HIGH);
  pinMode(TFT4_CS, OUTPUT); digitalWrite(TFT4_CS, HIGH);
  pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);
}

// SPI0 remappers: SD vs. each SPI0-based TFT
static inline void mapSPI0_to_SD() {
  pinMode(TEMP_DC, INPUT);   // free GP16 for SD MISO
  SPI0.setSCK(SD_SCK);
  SPI0.setTX(SD_MOSI);
  SPI0.setRX(SD_MISO);
  SPI0.begin();
  digitalWrite(SD_CS, HIGH);
}
static inline void mapSPI0_to_TEMP() { SPI0.setSCK(TEMP_SCK); SPI0.setTX(TEMP_MOSI); SPI0.begin(); pinMode(TEMP_DC, OUTPUT); }
static inline void mapSPI0_to_HPA()  { SPI0.setSCK(HPA_SCK);  SPI0.setTX(HPA_MOSI);  SPI0.begin(); digitalWrite(HPA_CS, HIGH); }
static inline void mapSPI0_to_CLK()  { SPI0.setSCK(CLK_SCK);  SPI0.setTX(CLK_MOSI);  SPI0.begin(); digitalWrite(CLK_CS, HIGH); }

// TEMP access critical section (because TEMP_DC shares with SD MISO)
static inline void beginTempAccess() { digitalWrite(SD_CS, HIGH); digitalWrite(HPA_CS, HIGH); digitalWrite(CLK_CS, HIGH); SPI0.end(); mapSPI0_to_TEMP(); }
static inline void endTempAccess()   { mapSPI0_to_SD(); }

// Small helpers for SD reads and TFT writes (Core1)
static uint16_t packRGB565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static uint32_t rd32(File& f) { uint8_t b[4]; if (f.read(b,4)!=4) return 0xFFFFFFFF; return (uint32_t)b[0] | ((uint32_t)b[1]<<8) | ((uint32_t)b[2]<<16) | ((uint32_t)b[3]<<24); }
static uint16_t rd16(File& f){ uint8_t b[2]; if (f.read(b,2)!=2) return 0xFFFF; return (uint16_t)b[0] | ((uint16_t)b[1]<<8); }
static bool readChunks(File& f, uint8_t* dst, int len) {
  const int CHUNK = 256; int off = 0; uint32_t t0 = millis();
  while (off < len) {
    if (millis() - t0 > 8000) return false;
    int n = len - off; if (n > CHUNK) n = CHUNK;
    int got = f.read(dst + off, n);
    if (got <= 0) return false;
    off += got; delay(0);
  }
  return true;
}
static void writeRowHW(Adafruit_ST7789* tft, int16_t y, uint16_t* rgb565, int16_t w) {
  tft->startWrite();
  tft->setAddrWindow(0, y, w, 1);
  tft->writePixels(rgb565, (uint32_t)w, true);
  tft->endWrite();
}
// Line buffers for BMP → TFT
static uint8_t  s_rowBuf[720];   // 240 * 3 bytes
static uint16_t s_line565[240];  // 240 * 2 bytes

// ===== Asset selectors (category + month) =====
static const char* categoryBmp(int cat) {
  switch (cat) {
    case 0: return "/img/thunder240.bmp";
    case 1: return "/img/regn240.bmp";
    case 2: return "/img/skyer240.bmp";
    case 3: return "/img/letteskyer240.bmp";
    default:return "/img/sol240.bmp";
  }
}
static const char* categoryText(int cat) {
  switch (cat) {
    case 0: return " STORM ";
    case 1: return " REGN ";
    case 2: return " USTADIGT ";
    case 3: return " FINT ";
    default:return " MEGET FINT ";
  }
}
static const char* monthBmp(int month1_12) {
  static const char* arr[12] = {
    "/img/0240.bmp","/img/1240.bmp","/img/2240.bmp","/img/3240.bmp",
    "/img/4240.bmp","/img/5240.bmp","/img/6240.bmp","/img/7240.bmp",
    "/img/8240.bmp","/img/9240.bmp","/img/10240.bmp","/img/11240.bmp"
  };
  if (month1_12 < 1 || month1_12 > 12) return "/img/0240.bmp";
  return arr[month1_12 - 1];
}

// ===== Simple 24-bit BMP blitters (expect up to 240x240) =====
// These draw directly to displays; CLOCK also has a buffered path (see below).

static bool drawBMP24_240x240_SPI1(Adafruit_ST7789* tft, const char* path) {
  if (!sd_ready) return false;
  mapSPI0_to_SD();
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.print("Open fail: "); Serial.println(path); return false; }
  uint8_t magic[2]; if (f.read(magic,2)!=2 || magic[0]!='B' || magic[1]!='M'){ f.close(); return false; }
  (void)rd32(f); (void)rd16(f); (void)rd16(f);
  uint32_t dataOffset = rd32(f);
  uint32_t dibSize = rd32(f);
  int32_t  bmpW    = (int32_t)rd32(f);
  int32_t  bmpH    = (int32_t)rd32(f);
  uint16_t planes  = rd16(f);
  uint16_t bpp     = rd16(f);
  uint32_t comp    = rd32(f);
  (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f);
  if (dibSize < 40 || planes != 1 || comp != 0 || bpp != 24) { f.close(); return false; }
  bool topDown = bmpH < 0;
  int32_t h = min(240, (int)(topDown ? -bmpH : bmpH));
  int32_t w = min(240, (int)bmpW);
  uint32_t rowSize = ((uint32_t)bpp * (uint32_t)bmpW + 31) / 32 * 4;
  if (!f.seek(dataOffset)) { f.close(); return false; }
  int32_t destY = topDown ? 0 : (h - 1);
  int32_t step  = topDown ? 1 : -1;
  for (int32_t i = 0; i < h; ++i) {
    if (!readChunks(f, s_rowBuf, (int)rowSize)) { f.close(); return false; }
    for (int32_t x = 0; x < w; ++x) {
      uint8_t b = s_rowBuf[x*3+0], g = s_rowBuf[x*3+1], r = s_rowBuf[x*3+2];
      s_line565[x] = packRGB565(r,g,b);
    }
    writeRowHW(tft, (int16_t)destY, s_line565, (int16_t)w);
    destY += step; delay(0);
  }
  f.close();
  return true;
}
static bool drawBMP24_240x240_HPA_SPI0(Adafruit_ST7789* tft, const char* path) {
  if (!sd_ready) return false;
  mapSPI0_to_SD();
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.print("Open fail: "); Serial.println(path); return false; }
  uint8_t magic[2]; if (f.read(magic,2)!=2 || magic[0]!='B' || magic[1]!='M'){ f.close(); return false; }
  (void)rd32(f); (void)rd16(f); (void)rd16(f);
  uint32_t dataOffset = rd32(f);
  uint32_t dibSize = rd32(f);
  int32_t  bmpW    = (int32_t)rd32(f);
  int32_t  bmpH    = (int32_t)rd32(f);
  uint16_t planes  = rd16(f);
  uint16_t bpp     = rd16(f);
  uint32_t comp    = rd32(f);
  (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f);
  if (dibSize < 40 || planes != 1 || comp != 0 || bpp != 24) { f.close(); return false; }
  bool topDown = bmpH < 0;
  int32_t h = min(240, (int)(topDown ? -bmpH : bmpH));
  int32_t w = min(240, (int)bmpW);
  uint32_t rowSize = ((uint32_t)bpp * (uint32_t)bmpW + 31) / 32 * 4;
  if (!f.seek(dataOffset)) { f.close(); return false; }
  int32_t destY = topDown ? 0 : (h - 1);
  int32_t step  = topDown ? 1 : -1;
  for (int32_t i = 0; i < h; ++i) {
    mapSPI0_to_SD();
    if (!readChunks(f, s_rowBuf, (int)rowSize)) { f.close(); return false; }
    for (int32_t x = 0; x < w; ++x) {
      uint8_t b = s_rowBuf[x*3+0], g = s_rowBuf[x*3+1], r = s_rowBuf[x*3+2];
      s_line565[x] = packRGB565(r,g,b);
    }
    mapSPI0_to_HPA();
    writeRowHW(tft, (int16_t)destY, s_line565, (int16_t)w);
    destY += step; delay(0);
  }
  f.close();
  mapSPI0_to_SD();
  return true;
}
static bool drawBMP24_240x240_CLK_SPI0(Adafruit_ST7789* tft, const char* path) {
  if (!sd_ready) return false;
  mapSPI0_to_SD();
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.print("Open fail: "); Serial.println(path); return false; }
  uint8_t magic[2]; if (f.read(magic,2)!=2 || magic[0]!='B' || magic[1]!='M'){ f.close(); return false; }
  (void)rd32(f); (void)rd16(f); (void)rd16(f);
  uint32_t dataOffset = rd32(f);
  uint32_t dibSize = rd32(f);
  int32_t  bmpW    = (int32_t)rd32(f);
  int32_t  bmpH    = (int32_t)rd32(f);
  uint16_t planes  = rd16(f);
  uint16_t bpp     = rd16(f);
  uint32_t comp    = rd32(f);
  (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f);
  if (dibSize < 40 || planes != 1 || comp != 0 || bpp != 24) { f.close(); return false; }
  bool topDown = bmpH < 0;
  int32_t h = min(240, (int)(topDown ? -bmpH : bmpH));
  int32_t w = min(240, (int)bmpW);
  uint32_t rowSize = ((uint32_t)bpp * (uint32_t)bmpW + 31) / 32 * 4;
  if (!f.seek(dataOffset)) { f.close(); return false; }
  int32_t destY = topDown ? 0 : (h - 1);
  int32_t step  = topDown ? 1 : -1;
  for (int32_t i = 0; i < h; ++i) {
    mapSPI0_to_SD();
    if (!readChunks(f, s_rowBuf, (int)rowSize)) { f.close(); return false; }
    for (int32_t x = 0; x < w; ++x) {
      uint8_t b = s_rowBuf[x*3+0], g = s_rowBuf[x*3+1], r = s_rowBuf[x*3+2];
      s_line565[x] = packRGB565(r,g,b);
    }
    mapSPI0_to_CLK();
    writeRowHW(tft, (int16_t)destY, s_line565, (int16_t)w);
    destY += step; delay(0);
  }
  f.close();
  mapSPI0_to_SD();
  return true;
}

// ===== CLOCK background buffer (RAM) and SD-fallback =====
// Important: Buffer holds pixels in screen coordinates (rotation handled by GFX during draw)
static uint16_t s_clkBg[240 * 240];   // 240x240 RGB565 (~115.2 kB)
static bool     s_clkBgValid = false;

// Luma helper (RGB565 → approx luma)
static inline uint8_t rgb565_luma(uint16_t c) {
  uint8_t r = ((c >> 11) & 0x1F); r = (r << 3) | (r >> 2);
  uint8_t g = ((c >> 5)  & 0x3F); g = (g << 2) | (g >> 4);
  uint8_t b = ( c        & 0x1F); b = (b << 3) | (b >> 2);
  return (uint8_t)((77u*r + 150u*g + 29u*b) >> 8);
}

// Average luma from RAM background over a rect (x,y,w,h in screen coordinates)
static uint8_t avgLumaFromBgRect(int x, int y, int w, int h) {
  if (!s_clkBgValid) return 255;
  // Clip
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= 240 || y >= 240 || w <= 0 || h <= 0) return 255;
  if (x + w > 240) w = 240 - x;
  if (y + h > 240) h = 240 - y;

  uint32_t sum = 0, n = 0;
  for (int sy = y; sy < y + h; ++sy) {
    const uint16_t* row = s_clkBg + sy*240 + x;
    for (int sx = 0; sx < w; ++sx) {
      sum += rgb565_luma(row[sx]);
      ++n;
    }
  }
  if (!n) return 255;
  return (uint8_t)(sum / n);
}

static inline uint16_t rgb888_to_rgb565(uint8_t r, uint8_t g, uint8_t b) {
  return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}
static inline void rgb565_to_rgb888(uint16_t c, uint8_t& r, uint8_t& g, uint8_t& b) {
  r = (uint8_t)((c >> 11) & 0x1F); r = (uint8_t)((r << 3) | (r >> 2));
  g = (uint8_t)((c >> 5)  & 0x3F); g = (uint8_t)((g << 2) | (g >> 4));
  b = (uint8_t)( c        & 0x1F); b = (uint8_t)((b << 3) | (b >> 2));
}
static inline uint16_t invertRGB565(uint16_t c) {
  uint8_t r,g,b; rgb565_to_rgb888(c,r,g,b);
  return rgb888_to_rgb565((uint8_t)(255 - r), (uint8_t)(255 - g), (uint8_t)(255 - b));
}
static uint16_t computeAvgColorFromBuffer(const uint16_t* buf, int w=240, int h=240) {
  if (!buf || w<=0 || h<=0) return C_BLACK;
  uint64_t rs=0, gs=0, bs=0; uint32_t n=(uint32_t)w*(uint32_t)h;
  for (int y=0;y<h;++y) {
    const uint16_t* row = buf + y*240;
    for (int x=0;x<w;++x) {
      uint8_t r,g,b; rgb565_to_rgb888(row[x], r,g,b);
      rs+=r; gs+=g; bs+=b;
    }
  }
  uint8_t ar=(uint8_t)(rs/n), ag=(uint8_t)(gs/n), ab=(uint8_t)(bs/n);
  return rgb888_to_rgb565(ar,ag,ab);
}

// Average color from SD image (fallback when RAM not valid)
static uint16_t computeAvgColorFromBmp24(const char* path) {
  if (!sd_ready) return C_BLACK;
  mapSPI0_to_SD();
  File f = SD.open(path, FILE_READ);
  if (!f) return C_BLACK;
  uint8_t magic[2]; if (f.read(magic,2)!=2 || magic[0]!='B' || magic[1]!='M'){ f.close(); return C_BLACK; }
  (void)rd32(f); (void)rd16(f); (void)rd16(f);
  uint32_t dataOffset = rd32(f);
  uint32_t dibSize    = rd32(f);
  int32_t  bmpW       = (int32_t)rd32(f);
  int32_t  bmpH       = (int32_t)rd32(f);
  uint16_t planes     = rd16(f);
  uint16_t bpp        = rd16(f);
  uint32_t comp       = rd32(f);
  (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f);
  if (dibSize < 40 || planes!=1 || comp!=0 || bpp!=24) { f.close(); return C_BLACK; }

  bool topDown = bmpH < 0;
  int32_t h = min(240, (int)(topDown ? -bmpH : bmpH));
  int32_t w = min(240, (int)bmpW);
  uint32_t rowSize = ((uint32_t)bpp * (uint32_t)bmpW + 31) / 32 * 4;
  if (!f.seek(dataOffset)) { f.close(); return C_BLACK; }

  uint64_t rs=0, gs=0, bs=0; uint32_t n=(uint32_t)w*(uint32_t)h;
  for (int32_t i=0;i<h;++i) {
    if (!readChunks(f, s_rowBuf, (int)rowSize)) { f.close(); return C_BLACK; }
    for (int32_t x=0; x<w; ++x) {
      uint8_t b = s_rowBuf[x*3+0], g = s_rowBuf[x*3+1], r = s_rowBuf[x*3+2];
      rs+=r; gs+=g; bs+=b;
    }
  }
  f.close();
  uint8_t ar=(uint8_t)(rs/n), ag=(uint8_t)(gs/n), ab=(uint8_t)(bs/n);
  return rgb888_to_rgb565(ar,ag,ab);
}

// Read BMP header (for SD-fallback restore)
static bool readBmpHeader(const char* path, ClkBgInfo& out) {
  if (!sd_ready) return false;
  mapSPI0_to_SD();
  File f = SD.open(path, FILE_READ);
  if (!f) return false;

  uint8_t magic[2];
  if (f.read(magic,2)!=2 || magic[0]!='B' || magic[1]!='M') { f.close(); return false; }
  (void)rd32(f); (void)rd16(f); (void)rd16(f);
  uint32_t dataOffset = rd32(f);
  uint32_t dibSize    = rd32(f);
  int32_t  bmpW       = (int32_t)rd32(f);
  int32_t  bmpH       = (int32_t)rd32(f);
  uint16_t planes     = rd16(f);
  uint16_t bpp        = rd16(f);
  uint32_t comp       = rd32(f);
  (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f);
  f.close();

  if (dibSize < 40 || planes != 1 || comp != 0 || bpp != 24) return false;
  out.valid    = true;
  out.topDown  = (bmpH < 0);
  out.bmpW     = bmpW;
  out.bmpH     = bmpH;
  out.drawW    = min(240, (int)bmpW);
  out.drawH    = min(240, (int)(out.topDown ? -bmpH : bmpH));
  out.rowSize  = ((uint32_t)bpp * (uint32_t)bmpW + 31) / 32 * 4;
  out.dataOffset = dataOffset;
  strncpy(out.path, path, sizeof(out.path)-1);
  return true;
}

// Load full 24-bit BMP into RGB565 buffer (240x240) as rotation 0 (“as stored”)
// We then draw the buffer with the display’s rotation applied (GFX handles it).
static bool loadBMP24_240x240_toBuffer(const char* path, uint16_t* dst) {
  if (!sd_ready || !dst) return false;
  mapSPI0_to_SD();
  File f = SD.open(path, FILE_READ);
  if (!f) { Serial.print("Open fail: "); Serial.println(path); return false; }

  uint8_t magic[2];
  if (f.read(magic,2)!=2 || magic[0]!='B' || magic[1]!='M'){ f.close(); return false; }
  (void)rd32(f); (void)rd16(f); (void)rd16(f);
  uint32_t dataOffset = rd32(f);
  uint32_t dibSize    = rd32(f);
  int32_t  bmpW       = (int32_t)rd32(f);
  int32_t  bmpH       = (int32_t)rd32(f);
  uint16_t planes     = rd16(f);
  uint16_t bpp        = rd16(f);
  uint32_t comp       = rd32(f);
  (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f); (void)rd32(f);

  if (dibSize < 40 || planes != 1 || comp != 0 || bpp != 24) { f.close(); return false; }

  const bool topDown = (bmpH < 0);
  const int32_t srcH = min(240, (int)(topDown ? -bmpH : bmpH));
  const int32_t srcW = min(240, (int)bmpW);
  const uint32_t rowSize = ((uint32_t)bpp * (uint32_t)bmpW + 31) / 32 * 4;

  if (!f.seek(dataOffset)) { f.close(); return false; }

  // Clear the buffer outside the image area
  for (int i = 0; i < 240*240; ++i) dst[i] = C_BLACK;

  int32_t uy    = topDown ? 0 : (srcH - 1);
  int32_t stepY = topDown ? 1 : -1;

  // Copy as rotation 0 (line by line)
  for (int32_t row = 0; row < srcH; ++row) {
    if (!readChunks(f, s_rowBuf, (int)rowSize)) { f.close(); return false; }
    for (int32_t ux = 0; ux < srcW; ++ux) {
      uint8_t b = s_rowBuf[ux*3 + 0];
      uint8_t g = s_rowBuf[ux*3 + 1];
      uint8_t r = s_rowBuf[ux*3 + 2];
      dst[uy*240 + ux] = packRGB565(r,g,b);
    }
    uy += stepY;
    delay(0);
  }

  f.close();
  return true;
}

// Draw full buffer to CLOCK TFT (GFX handles display rotation)
static void drawBufferToClk(Adafruit_ST7789* tft, const uint16_t* buf) {
  if (!tft || !buf) return;
  mapSPI0_to_CLK();
  tft->drawRGBBitmap(0, 0, const_cast<uint16_t*>(buf), 240, 240);
}

// Restore a rectangle from RAM background (fast path) in screen coordinates
static inline void restoreRectFromBg(Adafruit_ST7789* tft, int x, int y, int w, int h) {
  if (!tft || !s_clkBgValid) return;

  // Clip to 240x240
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= 240 || y >= 240 || w <= 0 || h <= 0) return;
  if (x + w > 240) w = 240 - x;
  if (y + h > 240) h = 240 - y;

  mapSPI0_to_CLK();
  for (int sy = y; sy < y + h; ++sy) {
    // Copy one scanline from the RAM buffer to the TFT
    memcpy(s_line565, s_clkBg + sy*240 + x, (size_t)w * sizeof(uint16_t));
    tft->startWrite();
    tft->setAddrWindow(x, sy, w, 1);
    tft->writePixels(s_line565, (uint32_t)w, true);
    tft->endWrite();
  }
}

// Restore rectangle directly from SD BMP (fallback when RAM buffer missing)
static void restoreRectFromSD_Clk(int x, int y, int w, int h) {
  if (!sd_ready || !tftClk || !g_clkBgInfo.valid) return;

  // Clip to 240x240
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= 240 || y >= 240 || w <= 0 || h <= 0) return;
  if (x + w > 240) w = 240 - x;
  if (y + h > 240) h = 240 - y;

  mapSPI0_to_SD();
  File f = SD.open(g_clkBgInfo.path, FILE_READ);
  if (!f) return;

  for (int yy = 0; yy < h; ++yy) {
    int bmpY = g_clkBgInfo.topDown ? (y + yy) : (g_clkBgInfo.drawH - 1 - (y + yy));
    if (bmpY < 0 || bmpY >= g_clkBgInfo.drawH) continue;

    uint32_t off = g_clkBgInfo.dataOffset + (uint32_t)bmpY * g_clkBgInfo.rowSize + (uint32_t)x * 3;
    if (!f.seek(off)) break;

    int toRead = w * 3;
    if (!readChunks(f, s_rowBuf, toRead)) break;

    for (int xx = 0; xx < w; ++xx) {
      uint8_t b = s_rowBuf[xx*3 + 0];
      uint8_t g = s_rowBuf[xx*3 + 1];
      uint8_t r = s_rowBuf[xx*3 + 2];
      s_line565[xx] = packRGB565(r,g,b);
    }

    mapSPI0_to_CLK();
    tftClk->startWrite();
    tftClk->setAddrWindow(x, y + yy, w, 1);
    tftClk->writePixels(s_line565, (uint32_t)w, true);
    tftClk->endWrite();
  }

  f.close();
}

// Unified restore: prefer RAM copy; otherwise fall back to SD
static inline void restoreRectFromClkBackground(Adafruit_ST7789* tft, int x, int y, int w, int h) {
  if (s_clkBgValid) restoreRectFromBg(tft, x, y, w, h);
  else              restoreRectFromSD_Clk(x, y, w, h);
}

/* ======== Adaptive color helpers for text/hands ======== */

static uint16_t avgColorFromBgRect(int x, int y, int w, int h) {
  if (!s_clkBgValid) return C_WHITE;
  // Clip
  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= 240 || y >= 240 || w <= 0 || h <= 0) return C_WHITE;
  if (x + w > 240) w = 240 - x;
  if (y + h > 240) h = 240 - y;

  uint64_t rs=0, gs=0, bs=0; uint32_t n=0;
  for (int sy = y; sy < y + h; ++sy) {
    const uint16_t* row = s_clkBg + sy*240 + x;
    for (int sx = 0; sx < w; ++sx) {
      uint8_t r,g,b; rgb565_to_rgb888(row[sx], r,g,b);
      rs += r; gs += g; bs += b; ++n;
    }
  }
  if (!n) return C_WHITE;
  uint8_t ar=(uint8_t)(rs/n), ag=(uint8_t)(gs/n), ab=(uint8_t)(bs/n);
  return rgb888_to_rgb565(ar,ag,ab);
}

static bool avgRectFromSD(int x, int y, int w, int h, uint8_t* outLuma, uint16_t* outRGB565) {
  if (!sd_ready || !g_clkBgInfo.valid) return false;

  if (x < 0) { w += x; x = 0; }
  if (y < 0) { h += y; y = 0; }
  if (x >= g_clkBgInfo.drawW || y >= g_clkBgInfo.drawH || w <= 0 || h <= 0) return false;
  if (x + w > g_clkBgInfo.drawW) w = g_clkBgInfo.drawW - x;
  if (y + h > g_clkBgInfo.drawH) h = g_clkBgInfo.drawH - y;

  mapSPI0_to_SD();
  File f = SD.open(g_clkBgInfo.path, FILE_READ);
  if (!f) return false;

  uint64_t rs=0, gs=0, bs=0; uint32_t n=0;
  for (int yy = 0; yy < h; ++yy) {
    int bmpY = g_clkBgInfo.topDown ? (y + yy) : (g_clkBgInfo.drawH - 1 - (y + yy));
    uint32_t off = g_clkBgInfo.dataOffset + (uint32_t)bmpY * g_clkBgInfo.rowSize + (uint32_t)x * 3;
    if (!f.seek(off)) { f.close(); return false; }
    int toRead = w * 3;
    if (!readChunks(f, s_rowBuf, toRead)) { f.close(); return false; }
    for (int xx=0; xx<w; ++xx) {
      uint8_t B = s_rowBuf[xx*3+0], G = s_rowBuf[xx*3+1], R = s_rowBuf[xx*3+2];
      rs += R; gs += G; bs += B; ++n;
    }
  }
  f.close();
  if (!n) return false;

  uint8_t ar=(uint8_t)(rs/n), ag=(uint8_t)(gs/n), ab=(uint8_t)(bs/n);
  if (outRGB565) *outRGB565 = rgb888_to_rgb565(ar,ag,ab);
  if (outLuma)   *outLuma   = (uint8_t)((77u*ar + 150u*ag + 29u*ab) >> 8);
  return true;
}

// Choose black/white text based on average luma
static uint8_t avgLumaFromRectAny(int x, int y, int w, int h) {
  uint8_t L = 255;
  if (s_clkBgValid) {
    L = avgLumaFromBgRect(x,y,w,h);
  } else {
    if (!avgRectFromSD(x,y,w,h, &L, nullptr)) L = 255;
  }
  return L;
}
static uint16_t avgColorFromRectAny(int x, int y, int w, int h) {
  if (s_clkBgValid) return avgColorFromBgRect(x,y,w,h);
  uint16_t c = C_WHITE;
  (void)avgRectFromSD(x,y,w,h, nullptr, &c);
  return c;
}
static inline uint16_t chooseBWFromLuma(uint8_t L) { return (L >= 128) ? C_BLACK : C_WHITE; }

// ===== Simple text helpers =====
static inline int textPixelHeight(int textSize) { return 8 * textSize; }
static inline int textPixelWidth(const char* s, int textSize) { return (int)strlen(s) * 6 * textSize; }

static void drawCentered(Adafruit_ST7789* tft, int y, const char* txt, uint16_t fg, uint16_t bg, int size) {
  int w = textPixelWidth(txt, size);
  int x = (240 - w) / 2; if (x < 0) x = 0;
  tft->setTextWrap(false);
  tft->setTextSize(size);
  tft->setTextColor(fg, bg);
  tft->setCursor(x, y);
  tft->print(txt);
}

// Transparent text helper (not used for DOW/DATE, used for TIME to preserve photo pixels)
static void drawCenteredTransparentOnClk(int y, const char* txt, uint16_t fg=C_BLACK, int size=TXT_SIZE) {
  if (!tftClk) return;
  int w = textPixelWidth(txt, size);
  int h = textPixelHeight(size);
  int x = (240 - w) / 2; if (x < 0) x = 0;

  // Restore exact pixels from background, then print transparent text
  restoreRectFromClkBackground(tftClk, x, y, w, h);

  mapSPI0_to_CLK();
  tftClk->setTextWrap(false);
  tftClk->setTextSize(size);
  tftClk->setTextColor(fg);  // transparent background
  tftClk->setCursor(x, y);
  tftClk->print(txt);
}

// Opaque centered text with adaptive bg/fg (used for DOW/DATE by default)
static void drawCenteredAdaptiveOnClk(int y, const char* txt, int size=TXT_SIZE) {
  if (!tftClk) return;
  int w = textPixelWidth(txt, size);
  int h = textPixelHeight(size);
  int x = (240 - w) / 2; if (x < 0) x = 0;

  uint16_t bg = avgColorFromRectAny(x, y, w, h);
  uint8_t  L  = avgLumaFromRectAny(x, y, w, h);
  uint16_t fg = chooseBWFromLuma(L);

  mapSPI0_to_CLK();
  tftClk->fillRect(x, y, w, h, bg);
  tftClk->setTextWrap(false);
  tftClk->setTextSize(size);
  tftClk->setTextColor(fg, bg);
  tftClk->setCursor(x, y);
  tftClk->print(txt);
}

// =================== WiFi config from SD (Core1 reads early) ===================
#define WIFI_CFG_FILE "/wifi.cfg"
String wifi_ssid = "";
String wifi_pass = "";

// Core1 reads WiFi creds early (before Core0 queue init)
static bool core1ReadWifiCfg() {
  mapSPI0_to_SD();
  if (!SD.exists("/wifi.cfg")) { Serial.println("[WiFi] core1: wifi.cfg not found"); g_wifi_cfg_missing = true; return false; }
  File f = SD.open("/wifi.cfg", FILE_READ);
  if (!f) { Serial.println("[WiFi] core1: cannot open wifi.cfg"); g_wifi_cfg_missing = true; return false; }
  String ssid, pass;
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (line.startsWith("SSID=")) ssid = line.substring(5);
    else if (line.startsWith("PASS=")) pass = line.substring(5);
  }
  f.close();
  if (ssid.length()==0 || pass.length()==0) { Serial.println("[WiFi] core1: wifi.cfg missing SSID or PASS"); g_wifi_cfg_missing = true; return false; }
  strncpy(g_wifi_ssid, ssid.c_str(), sizeof(g_wifi_ssid)-1);
  strncpy(g_wifi_pass, pass.c_str(), sizeof(g_wifi_pass)-1);
  g_wifi_cfg_ready = true;
  Serial.print("[WiFi] core1 loaded SSID="); Serial.println(g_wifi_ssid);
  return true;
}

// =================== Calibration to/from SD (/calib.cfg) ===================
static bool core1ReadCalibCfg() {
  if (!sd_ready) return false;
  mapSPI0_to_SD();
  if (!SD.exists("/calib.cfg")) { Serial.println("[CALIB] calib.cfg not found (using defaults)"); return false; }
  File f = SD.open("/calib.cfg", FILE_READ);
  if (!f) { Serial.println("[CALIB] cannot open calib.cfg"); return false; }
  float hk = g_tempHeatK, bc = backlightcomp;
  while (f.available()) {
    String line = f.readStringUntil('\n'); line.trim();
    if (line.startsWith("HEATK="))   hk = line.substring(6).toFloat();
    else if (line.startsWith("BLCOMP=")) bc = line.substring(7).toFloat();
  }
  f.close();
  if (!isfinite(hk) || hk < 0.0f || hk > 10.0f) hk = g_tempHeatK;
  if (!isfinite(bc) || bc < 0.0f || bc > 0.1f)  bc = backlightcomp;
  g_tempHeatK   = hk;
  backlightcomp = bc;
  Serial.print("[CALIB] Loaded HEATK="); Serial.print(g_tempHeatK);
  Serial.print(" BLCOMP="); Serial.println(backlightcomp, 6);
  return true;
}
static bool core1SaveCalibCfg() {
  if (!sd_ready) return false;
  mapSPI0_to_SD();
  SD.remove("/calib.cfg");
  File f = SD.open("/calib.cfg", FILE_WRITE);
  if (!f) { Serial.println("[CALIB] open for write failed"); return false; }
  f.print("HEATK=");  f.println(String(g_tempHeatK, 3));
  f.print("BLCOMP="); f.println(String(backlightcomp, 6));
  f.close();
  Serial.print("[CALIB] Saved HEATK="); Serial.print(g_tempHeatK, 3);
  Serial.print(" BLCOMP="); Serial.println(backlightcomp, 6);
  return true;
}

// =================== WiFi/NTP maintenance (Core0) ===================

static const char* WIFI_HOSTNAME = "Barometer";
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;
static const uint32_t WIFI_LOOP_TIMEOUT_MS    = 8000;
static const uint32_t WIFI_RETRY_INTERVAL_MS  = 60000;
static uint32_t g_lastWiFiAttemptMs = 0;

WiFiUDP keepAliveUdp;
static unsigned long lastKeepAliveMs = 0;
static const unsigned long keepAliveIntervalMs = 30000;

static void wifiKeepAlive() {
  if (WiFi.status() != WL_CONNECTED) return;
  unsigned long now = millis();
  if (now - lastKeepAliveMs < keepAliveIntervalMs) return;
  lastKeepAliveMs = now;
  IPAddress gw = WiFi.gatewayIP();
  if (gw == IPAddress(0,0,0,0)) return;
  if (keepAliveUdp.begin(0)) {
    uint8_t b = 0;
    keepAliveUdp.beginPacket(gw, 53);
    keepAliveUdp.write(&b, 1);
    keepAliveUdp.endPacket();
    keepAliveUdp.stop();
  }
}
static const char* decodeStatus(uint8_t st) {
  switch (st) {
    case WL_IDLE_STATUS:    return "IDLE";
    case WL_NO_SSID_AVAIL:  return "NO_SSID";
    case WL_SCAN_COMPLETED: return "SCAN_DONE";
    case WL_CONNECTED:      return "CONNECTED";
    case WL_CONNECT_FAILED: return "CONNECT_FAILED";
    case WL_CONNECTION_LOST:return "CONNECTION_LOST";
    case WL_DISCONNECTED:   return "DISCONNECTED";
    default:                return "UNKNOWN";
  }
}
static bool wifiWaitFor(uint32_t timeoutMs) {
  uint32_t t0 = millis(); uint8_t last = 255;
  while ((millis() - t0) < timeoutMs) {
    uint8_t st = WiFi.status();
    if (st != last) { last = st; Serial.print("[WiFi] status="); Serial.println(decodeStatus(st)); }
    if (st == WL_CONNECTED) {
      Serial.print("[WiFi] Connected, IP="); Serial.println(WiFi.localIP());
      int8_t crssi; if (Cyw43Power::getRSSI(crssi)) { Serial.print("[WiFi] RSSI="); Serial.println(crssi); }
      return true;
    }
    delay(200);
  }
  return false;
}
static void wifiRadioReset() {
  WiFi.disconnect(true); delay(150);
  WiFi.mode(WIFI_OFF);   delay(150);
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(WIFI_HOSTNAME);
  delay(100);
}
static bool wifiConnectWithTimeout(uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;
  if (!g_wifi_cfg_ready) return false;
  Serial.println("[WiFi] Connecting...");
  (void)Cyw43Power::setPowerMode(Cyw43Power::NoSave);
  wifiRadioReset();
  WiFi.begin(g_wifi_ssid, g_wifi_pass);
  if (wifiWaitFor(timeoutMs)) return true;
  Serial.println("[WiFi] Connect timeout/failed, continue without WiFi");
  WiFi.disconnect(true);
  return false;
}
static void wifiMaintainInLoop() {
  static uint8_t  last = 255;
  static uint32_t discSince = 0;
  static uint8_t  lastShownWiFi = 255;

  uint8_t st = WiFi.status();
  if (st != lastShownWiFi && g_queue_ready) {
    DrawCmd c{}; c.type = CMD_CLK_IND_WIFI; c.ivalue = (st == WL_CONNECTED) ? C_GREEN : C_RED; q_push(c);
    lastShownWiFi = st;
  }

  if (st == WL_CONNECTED) { last = st; discSince = 0; wifiKeepAlive(); return; }
  if (last == WL_CONNECTED && st != WL_CONNECTED) {
    if (discSince == 0) discSince = millis();
    if (millis() - discSince < 2000) return;
  }
  last = st;

  uint32_t now = millis();
  if (now - g_lastWiFiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    g_lastWiFiAttemptMs = now;
    (void)wifiConnectWithTimeout(WIFI_LOOP_TIMEOUT_MS);
  }
}

// =================== RTC/DS3231/NTP (UTC internal; local for display) ===================

static RTC_DS3231 g_ds;
static bool g_ds_ok = false;

WiFiUDP g_ntpUdp;
NTPClient g_ntp(g_ntpUdp, "dk.pool.ntp.org", 0 /*UTC*/, 60000);
static bool g_ntp_inited = false;
static uint32_t g_lastNtpSyncMs = 0;
static const uint32_t NTP_SYNC_INTERVAL_MS = 12UL * 60UL * 60UL * 1000UL; // 12 hours

// Date helpers for EU DST
static inline bool isLeap(int y){ return ( (y%4==0) && ((y%100!=0) || (y%400==0)) ); }
static int daysInMonth(int y,int m){
  static const int mdays[12]={31,28,31,30,31,30,31,31,30,31,30,31};
  int d = mdays[m-1]; if(m==2 && isLeap(y)) d = 29; return d;
}
static int sakamotoDow(int y, int m, int d){
  static int t[] = {0,3,2,5,0,3,5,1,4,6,2,4};
  if(m < 3) y -= 1;
  return (y + y/4 - y/100 + y/400 + t[m-1] + d) % 7;
}
static int lastSundayOfMonth(int y,int m){
  int last = daysInMonth(y,m);
  int dow = sakamotoDow(y,m,last); // 0=Sun
  return last - dow;
}
static bool isEU_DST_UTC(int y,int m,int d,int hh){
  if(m < 3 || m > 10) return false;
  if(m > 3 && m < 10) return true;
  if(m == 3){
    int L = lastSundayOfMonth(y,3);
    if(d > L) return true;
    if(d < L) return false;
    return hh >= 1;
  }
  int L = lastSundayOfMonth(y,10);
  if(d < L) return true;
  if(d > L) return false;
  return hh < 1;
}
static void utcToLocalEU(const datetime_t& utc, datetime_t* outLocal){
  int y=utc.year, m=utc.month, d=utc.day, hh=utc.hour, mm=utc.min, ss=utc.sec;
  int offsHours = isEU_DST_UTC(y,m,d,hh) ? 2 : 1;
  int newH = hh + offsHours;
  int newD = d, newM = m, newY = y;
  while(newH >= 24){
    newH -= 24; newD++;
    if(newD > daysInMonth(newY,newM)){ newD = 1; newM++; if(newM > 12){ newM=1; newY++; } }
  }
  int dow = sakamotoDow(newY,newM,newD);
  outLocal->year=newY; outLocal->month=newM; outLocal->day=newD;
  outLocal->hour=newH; outLocal->min=mm; outLocal->sec=ss; outLocal->dotw=dow;
}
static void rtcSetFromEpochUTC(time_t epoch) {
  struct tm tmv; gmtime_r(&epoch, &tmv);
  datetime_t dt;
  dt.year  = tmv.tm_year + 1900;
  dt.month = tmv.tm_mon + 1;
  dt.day   = tmv.tm_mday;
  dt.dotw  = tmv.tm_wday; // 0=Sun
  dt.hour  = tmv.tm_hour;
  dt.min   = tmv.tm_min;
  dt.sec   = tmv.tm_sec;
  rtc_init(); rtc_set_datetime(&dt);
  Serial.println("[RTC] RP2040 set (UTC)");
}
static bool ds3231ReadToRp2040UTC() {
  if (!g_ds_ok) return false;
  DateTime dt = g_ds.now();  // UTC
  datetime_t hw;
  hw.year  = dt.year(); hw.month = dt.month(); hw.day   = dt.day();
  hw.dotw  = dt.dayOfTheWeek(); hw.hour  = dt.hour(); hw.min   = dt.minute(); hw.sec   = dt.second();
  rtc_init(); rtc_set_datetime(&hw);
  Serial.println("[RTC] RP2040 set from DS3231 (UTC)");
  return true;
}
static void ds3231SetFromEpochUTC(time_t epoch) {
  if (!g_ds_ok) return; g_ds.adjust(DateTime((uint32_t)epoch));
  Serial.println("[RTC] DS3231 set (UTC)");
}
static void ntpSyncMaintain() {
  if (millis() - g_lastNtpSyncMs < NTP_SYNC_INTERVAL_MS) return;
  if (WiFi.status() != WL_CONNECTED) return;
  if (!g_ntp_inited) { g_ntp.begin(); g_ntp_inited = true; }
  if (g_ntp.update()) {
    time_t epoch = g_ntp.getEpochTime(); // UTC
    rtcSetFromEpochUTC(epoch);
    ds3231SetFromEpochUTC(epoch);
    g_lastNtpSyncMs = millis();
    Serial.println("[NTP] Sync OK (UTC, twice/day)");
  } else {
    Serial.println("[NTP] update() failed");
    g_lastNtpSyncMs = millis() - (NTP_SYNC_INTERVAL_MS - 5UL*60UL*1000UL);
  }
}

// =================== Core1: setup/loop (TFT+SD owner) ===================

static inline void setBacklight(uint8_t duty) { g_backlight = duty; analogWrite(BACKLIGHT, duty); }

static void drawHumidityLabel(Adafruit_ST7789* tft) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(F_LABEL_X, F_LABEL_Y); tft->print(" Fugtighed "); }
static void drawHumidityValue(Adafruit_ST7789* tft, const char* txt) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(F_VALUE_X, F_VALUE_Y); tft->print(" "); tft->print(txt); tft->print(" "); }
static void drawTempLabel(Adafruit_ST7789* tft) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(T_LABEL_X, T_LABEL_Y); tft->print(" Temp "); }
static void drawTempValue(Adafruit_ST7789* tft, const char* txt) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(T_VALUE_X, T_VALUE_Y); tft->print(" "); tft->print(txt); tft->print(" "); }
static void drawHpaLabel(Adafruit_ST7789* tft) { drawCentered(tft, P_LABEL_Y, " Hpa ", C_BLACK, C_WHITE, TXT_SIZE); }
static void drawHpaValue(Adafruit_ST7789* tft, const char* txt) { char buf[24]; snprintf(buf, sizeof(buf), " %s ", txt); drawCentered(tft, P_VALUE_Y, buf, C_BLACK, C_WHITE, TXT_SIZE); }
static void drawHpaState(Adafruit_ST7789* tft, const char* txt) { drawCentered(tft, P_STATE_Y, txt, C_BLACK, C_WHITE, TXT_SIZE); }

// WiFi indicator on CLOCK tile
static uint16_t s_wifi_col = 0;
static void drawWifiDot(uint16_t color) { if (!tftClk) return; tftClk->fillRect(IND_WIFI_X, IND_Y, IND_SIZE, IND_SIZE, color); }
static void redrawWifiDot() { if (s_wifi_col) drawWifiDot(s_wifi_col); }

// Latest overlay strings for redraws
static char s_dowText[24]  = "";
static char s_timeText[24] = "";
static char s_dateText[24] = "";

// Rectangles for DOW/TIME/DATE
static Rect s_dowRect  {0,0,0,0};
static Rect s_timeRect {0,0,0,0};
static Rect s_dateRect {0,0,0,0};

// Redraw flags if hands cross areas
static volatile bool s_needDateRedraw = false;
static volatile bool s_needTimeRedraw = false;

// Compute centered rect
static inline void computeCenteredRectForText(int y, const char* txt, int size, int& x, int& w, int& h) {
  w = textPixelWidth(txt, size);
  h = textPixelHeight(size);
  x = (240 - w) / 2; if (x < 0) x = 0;
}

// Rect intersection
static inline bool rectIntersects(const Rect& a, const Rect& b) {
  return (a.w > 0 && a.h > 0 && b.w > 0 && b.h > 0) &&
         (a.x < b.x + b.w) && (a.x + a.w > b.x) &&
         (a.y < b.y + b.h) && (a.y + a.h > b.y);
}

// CLOCK overlays
static void drawClockDOW(Adafruit_ST7789* /*tft*/, const char* txt) {
  int x,w,h; computeCenteredRectForText(C_DOW_Y, txt, TXT_SIZE, x, w, h);
  s_dowRect = makeRect(x, C_DOW_Y, w, h);
  drawCenteredAdaptiveOnClk(C_DOW_Y, txt, TXT_SIZE);
}

// TIME: restore original pixels first, then print transparent text for perfect photo matching
static void drawClockTime(Adafruit_ST7789* /*tft*/, const char* txt) {
  int x,w,h; computeCenteredRectForText(C_TIME_Y, txt, TXT_SIZE, x, w, h);
  s_timeRect = makeRect(x, C_TIME_Y, w, h);

  // Restore original pixels from RAM (or SD-fallback)
  restoreRectFromClkBackground(tftClk, x, C_TIME_Y, w, h);

  // Choose text color from luma of restored area
  uint8_t  L  = avgLumaFromRectAny(x, C_TIME_Y, w, h);
  uint16_t fg = chooseBWFromLuma(L);

  // Transparent text on top of original photo
  mapSPI0_to_CLK();
  tftClk->setTextWrap(false);
  tftClk->setTextSize(TXT_SIZE);
  tftClk->setTextColor(fg);  // transparent bg
  tftClk->setCursor(x, C_TIME_Y);
  tftClk->print(txt);
}

static void drawClockDate(Adafruit_ST7789* /*tft*/, const char* txt) {
  int x,w,h; computeCenteredRectForText(C_DATE_Y, txt, TXT_SIZE, x, w, h);
  s_dateRect = makeRect(x, C_DATE_Y, w, h);
  drawCenteredAdaptiveOnClk(C_DATE_Y, txt, TXT_SIZE);
}

// SD.begin retry helper (helps stabilize bring-up)
static bool sdInitWithRetry(uint8_t tries = 3, uint32_t hz = SD_HZ) {
  for (uint8_t i = 0; i < tries; ++i) {
    force_all_tft_cs_high();
    SPI0.end();
    delay(5);
    mapSPI0_to_SD();
    DBG(String("[SD] begin try #") + String((int)(i+1)));
    if (SD.begin(SD_CS, hz, SPI0)) {
      if (i) { Serial.print("[SD] Init OK after retries: "); Serial.println((int)i); }
      return true;
    }
    Serial.println("[SD] begin failed"); Serial.flush();
    delay(200);
  }
  return false;
}

void setup1() {
  DBG("[Core1] setup1 start, waiting for queue...");
  uint32_t t_wait = millis();
  while (!g_queue_ready) {
    if (millis() - t_wait > 5000) { DBG("[Core1] still waiting for queue..."); t_wait = millis(); }
    delay(1);
  }

  pinMode(BACKLIGHT, OUTPUT);
  analogWriteFreq(20000);
  analogWriteRange(255);
  setBacklight(g_backlight);
  DBG("[Core1] Backlight configured");

  force_all_tft_cs_high();
  DBG("[Core1] All CS high");

  // HUM on SPI1
  SPI1.setSCK(HUM_SCK);
  SPI1.setTX(HUM_MOSI);
  SPI1.begin();
  DBG("[Core1] SPI1 begin");

  tftHumid = new Adafruit_ST7789(&SPI1, HUM_CS, HUM_DC, HUM_RST);
  tftHumid->setSPISpeed(48000000);
  tftHumid->init(240, 240);
  tftHumid->setRotation(TFT_ROT);
  tftHumid->fillScreen(C_WHITE);
  tftHumid->drawRect(0, 0, 239, 239, C_RED);
  drawHumidityLabel(tftHumid);
  DBG("[Core1] HUM display OK");

  // TEMP on SPI0 (with TEMP_DC/SD sharing protection)
  beginTempAccess();
  {
    tftTemp = new Adafruit_ST7789(&SPI0, TEMP_CS, TEMP_DC, TEMP_RST);
    tftTemp->setSPISpeed(48000000);
    tftTemp->init(240, 240);
    tftTemp->setRotation(TFT_ROT);
    tftTemp->fillScreen(C_WHITE);
    tftTemp->drawRGBBitmap(0, 0, temp240, TEMP240_W, TEMP240_H);
    tftTemp->drawRect(0, 0, 239, 239, C_RED);
    drawTempLabel(tftTemp);
  }
  endTempAccess();
  DBG("[Core1] TEMP display OK");

  // HPA on SPI0 (remap)
  mapSPI0_to_HPA();
  tftHpa = new Adafruit_ST7789(&SPI0, HPA_CS, HPA_DC, HPA_RST);
  tftHpa->setSPISpeed(48000000);
  tftHpa->init(240, 240);
  tftHpa->setRotation(TFT_ROT);
  tftHpa->fillScreen(C_WHITE);
  tftHpa->drawRect(0, 0, 239, 239, C_RED);
  drawHpaLabel(tftHpa);
  DBG("[Core1] HPA display OK");

  // CLOCK on SPI0 (remap)
  mapSPI0_to_CLK();
  tftClk = new Adafruit_ST7789(&SPI0, CLK_CS, CLK_DC, CLK_RST);
  tftClk->setSPISpeed(48000000);
  tftClk->init(240, 240);
  tftClk->setRotation(TFT_ROT);
  tftClk->fillScreen(C_WHITE);
  tftClk->drawRect(0, 0, 239, 239, C_RED);
  DBG("[Core1] CLK display OK");

  // Background buffer is static; mark invalid until first load
  s_clkBgValid = false;
  DBG("[CLK] RAM background buffer reserved (static)");

  // Init SD + read configs (wifi + calib)
  if (sdInitWithRetry(3, SD_HZ)) {
    sd_ready = true;
    Serial.print("SD init OK @"); Serial.print(SD_HZ/1000000); Serial.println("MHz");
    (void)core1ReadWifiCfg();
    (void)core1ReadCalibCfg();
  } else {
    sd_ready = false;
    Serial.println("SD.begin FAILED after retries");
    g_wifi_cfg_missing = true;
  }

  // Optional HUM image
  if (sd_ready) {
    (void)drawBMP24_240x240_SPI1(tftHumid, "/img/fugtighed240.bmp");
    tftHumid->drawRect(0, 0, 239, 239, C_RED);
    drawHumidityLabel(tftHumid);
  }

  DBG("[Core1] setup1 done");
  g_core1_ready = true;
}

// =================== Analog clock renderer (Core1) ===================

class AnalogClock {
public:
  void attach(Adafruit_ST7789* disp) { tft = disp; }

  void drawMarks() {
    if (!tft) return;
    mapSPI0_to_CLK();
    for (int i = 0; i < 12; ++i) {
      float ang_deg = i * 30.0f;
      float th = (ang_deg - 90.0f) * 3.14159265f / 180.0f;
      int x0 = cx + (int)roundf(105.0f * cosf(th));
      int y0 = cy + (int)roundf(105.0f * sinf(th));
      int x1 = cx + (int)roundf(120.0f * cosf(th));
      int y1 = cy + (int)roundf(120.0f * sinf(th));
      tft->drawLine(x0,y0,x1,y1,C_BLACK);
    }
    for (int i = 0; i < 4; ++i) {
      float ang_deg = i * 90.0f;
      float th = (ang_deg - 90.0f) * 3.14159265f / 180.0f;
      int x0 = cx + (int)roundf(100.0f * cosf(th));
      int y0 = cy + (int)roundf(100.0f * sinf(th));
      int x1 = cx + (int)roundf(120.0f * cosf(th));
      int y1 = cy + (int)roundf(120.0f * sinf(th));
      for (int o=-1;o<=1;++o) tft->drawLine(x0+o,y0,x1+o,y1,C_BLACK);
      for (int o=-1;o<=1;++o) tft->drawLine(x0,y0+o,x1,y1+o,C_BLACK);
    }
  }

  void update(int hh, int mm, int ss) {
    if (!tft) return;

    // Mark areas for possible redraw if hands cross text rects
    if (rectIntersects(bboxHour, s_dateRect) || rectIntersects(bboxMin, s_dateRect) || rectIntersects(bboxSec, s_dateRect))
      s_needDateRedraw = true;
    if (rectIntersects(bboxHour, s_timeRect) || rectIntersects(bboxMin, s_timeRect) || rectIntersects(bboxSec, s_timeRect))
      s_needTimeRedraw = true;

    // Restore previous hands
    restoreRect(bboxHour);
    restoreRect(bboxMin);
    restoreRect(bboxSec);

    // Redraw marks (keeps dial crisp)
    drawMarks();

    // Predict boxes for luma/contrast
    Rect rSec  = predictHandBBox(ss * 6.0f, 95, 18, 4);
    Rect rMin  = predictHandBBox(mm * 6.0f, 95, 28, 8);
    float hourDeg = (hh % 12) * 30.0f + (mm / 60.0f) * 30.0f;
    Rect rHour = predictHandBBox(hourDeg, 70, 26, 10);

    // Pick black/white for best contrast against background
    uint16_t cSec  = pickBWForRect(rSec);
    uint16_t cMin  = pickBWForRect(rMin);
    uint16_t cHour = pickBWForRect(rHour);

    // Draw hands
    bboxSec  = drawTaperedHand(ss * 6.0f, 90, 18, 4,  cSec);
    bboxMin  = drawTaperedHand(mm * 6.0f, 90, 28, 8,  cMin);
    bboxHour = drawTaperedHand(hourDeg,    70, 26, 10, cHour);

    // Check intersections again after drawing
    if (rectIntersects(bboxHour, s_dateRect) || rectIntersects(bboxMin, s_dateRect) || rectIntersects(bboxSec, s_dateRect))
      s_needDateRedraw = true;
    if (rectIntersects(bboxHour, s_timeRect) || rectIntersects(bboxMin, s_timeRect) || rectIntersects(bboxSec, s_timeRect))
      s_needTimeRedraw = true;

    // Draw hub (contrast picked from local luma)
    Rect hub = makeRect(cx-5, cy-5, 10, 10);
    uint16_t cHub = pickBWForRect(hub);
    mapSPI0_to_CLK();
    tft->fillCircle(cx, cy, center_size, cHub);
  }

  void reset() { bboxHour = {0,0,0,0}; bboxMin = {0,0,0,0}; bboxSec = {0,0,0,0}; }

private:
  Adafruit_ST7789* tft = nullptr;
  const int cx = 120, cy = 120;

  Rect bboxHour{0,0,0,0}, bboxMin{0,0,0,0}, bboxSec{0,0,0,0};

  static inline Rect makeBBoxFromTri(int16_t x1,int16_t y1,int16_t x2,int16_t y2,int16_t x3,int16_t y3) {
    int minx = min(x1, min(x2,x3));
    int miny = min(y1, min(y2,y3));
    int maxx = max(x1, max(x2,x3));
    int maxy = max(y1, max(y2,y3));
    return makeRect(minx, miny, maxx-minx+1, maxy-miny+1);
  }

  Rect drawTaperedHand(float deg, int r_tip, int r_base, int base_w, uint16_t color) {
    float th = (deg - 90.0f) * 3.14159265f / 180.0f;

    int16_t xt = cx + (int16_t)roundf(r_tip  * cosf(th));
    int16_t yt = cy + (int16_t)roundf(r_tip  * sinf(th));

    int16_t xb = cx + (int16_t)roundf(r_base * cosf(th));
    int16_t yb = cy + (int16_t)roundf(r_base * sinf(th));

    float px = -sinf(th), py = cosf(th);
    float halfW = base_w * 0.5f;

    int16_t x1 = (int16_t)roundf(xb + halfW * px);
    int16_t y1 = (int16_t)roundf(yb + halfW * py);
    int16_t x2 = (int16_t)roundf(xb - halfW * px);
    int16_t y2 = (int16_t)roundf(yb - halfW * py);

    mapSPI0_to_CLK();
    tft->fillTriangle(x1,y1, x2,y2, xt,yt, color);

    return makeBBoxFromTri(x1,y1,x2,y2,xt,yt);
  }

  Rect predictHandBBox(float deg, int r_tip, int r_base, int base_w) const {
    float th = (deg - 90.0f) * 3.14159265f / 180.0f;
    int16_t xt = cx + (int16_t)roundf(r_tip  * cosf(th));
    int16_t yt = cy + (int16_t)roundf(r_tip  * sinf(th));
    int16_t xb = cx + (int16_t)roundf(r_base * cosf(th));
    int16_t yb = cy + (int16_t)roundf(r_base * sinf(th));
    float px = -sinf(th), py = cosf(th);
    float halfW = base_w * 0.5f;
    int16_t x1 = (int16_t)roundf(xb + halfW * px);
    int16_t y1 = (int16_t)roundf(yb + halfW * py);
    int16_t x2 = (int16_t)roundf(xb - halfW * px);
    int16_t y2 = (int16_t)roundf(yb - halfW * py);
    return makeBBoxFromTri(x1,y1,x2,y2,xt,yt);
  }

  static uint16_t pickBWForRect(Rect r) {
    uint8_t L = avgLumaFromRectAny(r.x, r.y, r.w, r.h);
    return chooseBWFromLuma(L);
  }

  void restoreRect(Rect r) {
    if (r.w <= 0 || r.h <= 0) return;
    restoreRectFromClkBackground(tft, r.x, r.y, r.w, r.h);
  }
};
static AnalogClock g_analog;

void loop1() {
  if (!g_queue_ready) { delay(1); return; }

  DrawCmd cmd;
  if (queue_try_remove(&g_q, &cmd)) {
    switch (cmd.type) {
      case CMD_HUM_TEXT:
        drawHumidityValue(tftHumid, cmd.payload);
        break;

      case CMD_TEMP_TEXT:
        beginTempAccess();
        drawTempValue(tftTemp, cmd.payload);
        endTempAccess();
        break;

      case CMD_HPA_TEXT:
        drawHpaValue(tftHpa, cmd.payload);
        break;

      case CMD_HPA_BG_CAT:
        if (sd_ready) {
          (void)drawBMP24_240x240_HPA_SPI0(tftHpa, categoryBmp(cmd.ivalue));
          tftHpa->drawRect(0, 0, 239, 239, C_RED);
          drawHpaLabel(tftHpa);
          drawHpaState(tftHpa, categoryText(cmd.ivalue));
        }
        break;

      case CMD_CLK_TEXT_DOW: {
        if (strncmp(s_dowText, cmd.payload, sizeof(s_dowText)) != 0) {
          strncpy(s_dowText, cmd.payload, sizeof(s_dowText));
          s_dowText[sizeof(s_dowText)-1] = 0;
          drawClockDOW(tftClk, s_dowText);
        }
      } break;
      
      case CMD_CLK_ANALOG_HMS: {
        int hh = (cmd.ivalue >> 16) & 0xFF;
        int mm = (cmd.ivalue >> 8)  & 0xFF;
        int ss = (cmd.ivalue)       & 0xFF;

        g_analog.attach(tftClk);
        g_analog.update(hh, mm, ss);

#if (USE_ANALOG_CLOCK == 2)
        // Both: keep TIME above hands
        if (s_timeText[0]) {
          drawClockTime(tftClk, s_timeText);
          s_needTimeRedraw = false;
        }
#elif (USE_ANALOG_CLOCK == 1)
        // Analog-only: if TIME rect existed, restore it once
        if (s_timeRect.w > 0 && s_timeRect.h > 0) {
          restoreRectFromClkBackground(tftClk, s_timeRect.x, s_timeRect.y, s_timeRect.w, s_timeRect.h);
          s_timeRect = {0,0,0,0};
          s_timeText[0] = 0;
        }
#endif
        if (s_needDateRedraw && s_dateText[0]) {
          drawClockDate(tftClk, s_dateText);
          s_needDateRedraw = false;
        }
        redrawWifiDot();
      } break;

      case CMD_CLK_TEXT_TIME: {
#if (USE_ANALOG_CLOCK == 0) || (USE_ANALOG_CLOCK == 2)
        if (strncmp(s_timeText, cmd.payload, sizeof(s_timeText)) != 0) {
          strncpy(s_timeText, cmd.payload, sizeof(s_timeText));
          s_timeText[sizeof(s_timeText)-1] = 0;
          drawClockTime(tftClk, s_timeText);
        }
#else
        // Analog-only mode: ignore TIME updates
#endif
      } break;

      case CMD_CLK_TEXT_DATE: {
        if (strncmp(s_dateText, cmd.payload, sizeof(s_dateText)) != 0) {
          strncpy(s_dateText, cmd.payload, sizeof(s_dateText));
          s_dateText[sizeof(s_dateText)-1] = 0;
          drawClockDate(tftClk, s_dateText);
        }
      } break;

      case CMD_CLK_BG_MONTH: {
        if (!sd_ready) break;
        const char* p = monthBmp(cmd.ivalue);
        DBG(String("[CLK] bg=") + p);

        // Load BMP → RAM (rotation 0) and draw from RAM (fast)
        bool ok = loadBMP24_240x240_toBuffer(p, s_clkBg);
        s_clkBgValid = ok;

        if (ok) {
          drawBufferToClk(tftClk, s_clkBg);
        } else {
          // Emergency fallback: draw direct from SD
          (void)drawBMP24_240x240_CLK_SPI0(tftClk, p);
        }

        // Cache header for SD-fallback restore/sampling
        (void)readBmpHeader(p, g_clkBgInfo);

        // Red frame
        tftClk->drawRect(0, 0, 239, 239, C_RED);

#if (USE_ANALOG_CLOCK >= 1)
        g_analog.attach(tftClk);
        g_analog.reset();
        g_analog.drawMarks();
#endif

        // Overlays after background
        if (s_dowText[0])  drawClockDOW(tftClk,  s_dowText);

#if (USE_ANALOG_CLOCK == 0) || (USE_ANALOG_CLOCK == 2)
        if (s_timeText[0]) drawClockTime(tftClk, s_timeText);
#else
        s_timeRect = {0,0,0,0};
#endif

        if (s_dateText[0]) drawClockDate(tftClk, s_dateText);

        // WiFi indicator last
        redrawWifiDot();
      } break;

      case CMD_CLK_IND_WIFI:
        s_wifi_col = (uint16_t)cmd.ivalue;
        drawWifiDot(s_wifi_col);
        break;

      case CMD_SAVE_CALIB:
        (void)core1SaveCalibCfg();
        break;

      default:
        break;
    }
  } else {
    delay(1);
  }
}

// =================== Core0: sensors/RTC/web/auto-cal ===================

static Adafruit_BME280 bme;
static bool bme_ok = false;
#include <WebServer.h>
WebServer* server = new WebServer(80);

// Last values for web status (shown on /)
static float g_lastTemp = 0;
static float g_lastHpa = 0;
static float g_lastHumidity = 0;
static float g_cpuTemp = 0;
static String g_lastTime = "--:--:--";

// Backlight control from Core0 (shared PWM)
static inline void setBacklightCore0(uint8_t duty) { g_backlight = duty; analogWrite(BACKLIGHT, duty); }

// Locale-safe float parser (accepts comma or dot decimal separators)
static inline float parseFloatLocale(const String& in, bool* ok) {
  String s = in;
  s.trim();
  s.replace(',', '.');
  const char* c = s.c_str();
  char* endp = nullptr;
  double v = strtod(c, &endp);
  bool good = (endp && endp != c);
  if (ok) *ok = good;
  return (float)v;
}

// ===== Web: index page (status and backlight) =====
void handleIndex() {
  String html = "<!doctype html><html lang='da'><head><meta charset='utf-8'><title>Barometer</title><style>body{ text-align:center; font-family:Arial,sans-serif;} form{display:inline-block;margin-top:16px;} hr{margin:24px auto;width:80%;} a.btn{display:inline-block;margin-top:12px;padding:8px 12px;border:1px solid #444;border-radius:6px;text-decoration:none;color:#000;background:#f0f0f0}</style></head><body>";
  html += "<h2>SBComputer 2x2 pico Barometer</h2>";
  html += "<b>Time:</b> " + g_lastTime + "<br>";
  html += "<b>Temperature:</b> " + String(g_lastTemp, 1) + " &deg;C<br>";
  html += "<b>Pressure:</b> " + String(g_lastHpa, 2) + " hPa<br>";
  html += "<b>Humidity:</b> " + String(g_lastHumidity, 1) + " %<br>";
  html += "<b>CPU temp:</b> " + String(g_cpuTemp, 1) + " &deg;C<br>";
  html += "<hr>";
  html += "<form action='/backlight' method='get'>";
  html += "Backlight:<br><input type='range' min='0' max='255' value='" + String(g_backlight) + "' name='bl' oninput='this.nextElementSibling.value = this.value'>";
  html += "<output style='display:inline-block; min-width:40px;'>" + String(g_backlight) + "</output><br>";
  html += "<input type='submit' value='Update'>";
  html += "</form>";
  html += "<br><a class='btn' href='/calib'>Calibration</a>";
  html += "</body></html>";
  server->send(200, "text/html; charset=utf-8", html);
}

void handleStatusJson() {
  String json = "{";
  json += "\"time\":\"" + g_lastTime + "\",";
  json += "\"temperature\":" + String(g_lastTemp, 1) + ",";
  json += "\"pressure\":" + String(g_lastHpa, 2) + ",";
  json += "\"humidity\":" + String(g_lastHumidity, 1) + ",";
  json += "\"cpu_temp\":" + String(g_cpuTemp, 1);
  json += "}";
  server->send(200, "application/json; charset=utf-8", json);
}

void handleBacklight() {
  if (server->hasArg("bl")) {
    int val = server->arg("bl").toInt();
    val = constrain(val, 0, 255);
    setBacklightCore0((uint8_t)val);
  }
  server->sendHeader("Location", "/");
  server->send(302, "text/plain", "");
}

// ===== Calibration JSON (for tools) =====
void handleCalibJson() {
  String json = "{";
  json += "\"heatk\":" + String(g_tempHeatK, 3) + ",";
  json += "\"bl255\":" + String(backlightcomp * 255.0f, 3) + ",";
  json += "\"blcomp\":" + String(backlightcomp, 6);
  json += "}";
  server->send(200, "application/json; charset=utf-8", json);
}

// ===== Auto-calibration (Core0): low-CPU state machine, Ticker-based =====
enum AutoCalState : uint8_t { AC_IDLE = 0, AC_BL0 = 1, AC_BL255 = 2, AC_DONE = 3 };
struct AutoCalCtx {
  AutoCalState state = AC_IDLE;
  bool  running = false;
  bool  dryRun = true;
  bool  haveResults = false;
  bool  appliedOnDone = false;
  uint8_t prevBacklight = 255;

  uint32_t bootMs = 0;
  uint32_t stateStartMs = 0;
  const uint32_t minHoldMs   = 10UL * 60UL * 1000UL;
  const uint32_t stableMaxMs = 20UL * 60UL * 1000UL;

  volatile bool tickFlag = false;
  uint32_t tickIntervalMs = 10000;

  float refTempC = NAN;
  float t1_meas  = NAN;
  float t2_meas  = NAN;

  float computedHeatK = NAN;
  float computedBl255 = NAN;

  float  lastRounded01 = NAN;
  uint8_t stableCount  = 0;
} g_ac;
Ticker g_acTicker;

static inline bool acAllowedNow() {
  uint32_t sinceBoot = millis() - g_ac.bootMs;
  return sinceBoot >= (30UL * 60UL * 1000UL);
}
static inline float round01(float x) { return roundf(x * 10.0f) / 10.0f; }
static float readBmeRawTemp() {
  if (!bme_ok) return NAN;
  bme.takeForcedMeasurement();
  return bme.readTemperature();
}
static void acTickerISR() { g_ac.tickFlag = true; }

static bool autoCalibStart(float refTempC, bool dryRun, char* msg, size_t nmsg) {
  if (g_ac.running) { snprintf(msg, nmsg, "Auto-calibration already running"); return false; }
  if (!isfinite(refTempC)) { snprintf(msg, nmsg, "Invalid reference temperature"); return false; }
  g_ac.refTempC = refTempC;
  if (!acAllowedNow()) {
    int32_t msLeft = (int32_t)((30UL * 60UL * 1000UL) - (millis() - g_ac.bootMs));
    if (msLeft < 0) msLeft = 0; int minLeft = msLeft / 60000;
    snprintf(msg, nmsg, "Too early: wait ~%d min", minLeft); return false;
  }
  if (!bme_ok) { snprintf(msg, nmsg, "BME280 not ready"); return false; }

  g_ac.prevBacklight = g_backlight;
  g_ac.state = AC_BL0;
  g_ac.running = true;
  g_ac.dryRun = dryRun;
  g_ac.haveResults = false;
  g_ac.appliedOnDone = false;
  g_ac.stateStartMs = millis();
  g_ac.lastRounded01 = NAN;
  g_ac.stableCount = 0;
  g_ac.t1_meas = NAN; g_ac.t2_meas = NAN;
  g_ac.computedHeatK = NAN; g_ac.computedBl255 = NAN;

  setBacklightCore0(0);
  snprintf(msg, nmsg, "Auto-calibration started (BL=0, %s)", dryRun ? "dry-run" : "will save on completion");
  return true;
}
static void autoCalibCancel(char* msg, size_t nmsg) {
  setBacklightCore0(g_ac.prevBacklight);
  g_ac.state = AC_IDLE; g_ac.running = false;
  g_ac.lastRounded01 = NAN; g_ac.stableCount = 0;
  snprintf(msg, nmsg, "Auto-calibration canceled");
}
static void autoCalibFinalizeIfNeeded() {
  float heatK = g_ac.t1_meas - g_ac.refTempC;
  if (!isfinite(heatK)) heatK = 0.0f;
  heatK = fminf(fmaxf(heatK, 0.0f), 10.0f);

  float bl255 = g_ac.t2_meas - g_ac.t1_meas;
  if (!isfinite(bl255)) bl255 = 0.0f;
  bl255 = fminf(fmaxf(bl255, 0.0f), 10.0f);

  g_ac.computedHeatK = heatK;
  g_ac.computedBl255 = bl255;
  g_ac.haveResults   = true;

  if (!g_ac.dryRun) {
    g_tempHeatK   = heatK;
    backlightcomp = fminf(fmaxf(bl255 / 255.0f, 0.0f), 0.1f);
    DrawCmd c{}; c.type = CMD_SAVE_CALIB; q_push(c);
    g_ac.appliedOnDone = true;
  }

  setBacklightCore0(g_ac.prevBacklight);
  g_ac.state = AC_DONE; g_ac.running = false;
}
static void autoCalibStep() {
  if (!g_ac.running) return;

  float t = readBmeRawTemp();
  if (!isfinite(t)) return;

  float r01 = round01(t);
  if (!isfinite(g_ac.lastRounded01) || fabsf(r01 - g_ac.lastRounded01) > 0.049f) g_ac.stableCount = 0;
  else if (g_ac.stableCount < 255) g_ac.stableCount++;
  g_ac.lastRounded01 = r01;

  uint32_t elapsed = millis() - g_ac.stateStartMs;
  bool timeExceeded = (elapsed >= g_ac.stableMaxMs);
  bool minHoldOk    = (elapsed >= g_ac.minHoldMs);
  bool stableNow    = (g_ac.stableCount >= 3);

  if (g_ac.state == AC_BL0) {
    if ((minHoldOk && stableNow) || timeExceeded) {
      g_ac.t1_meas = t;
      g_ac.state = AC_BL255;
      g_ac.stateStartMs = millis();
      g_ac.lastRounded01 = NAN; g_ac.stableCount = 0;
      setBacklightCore0(255);
    }
    return;
  }
  if (g_ac.state == AC_BL255) {
    if ((minHoldOk && stableNow) || timeExceeded) {
      g_ac.t2_meas = t;
      autoCalibFinalizeIfNeeded();
    }
  }
}
static bool autoCalibApply(bool alsoSave, char* msg, size_t nmsg) {
  if (!g_ac.haveResults || !isfinite(g_ac.computedHeatK) || !isfinite(g_ac.computedBl255)) {
    snprintf(msg, nmsg, "No results to apply");
    return false;
  }
  g_tempHeatK   = fminf(fmaxf(g_ac.computedHeatK, 0.0f), 10.0f);
  backlightcomp = fminf(fmaxf(g_ac.computedBl255 / 255.0f, 0.0f), 0.1f);
  if (alsoSave) { DrawCmd c{}; c.type = CMD_SAVE_CALIB; q_push(c); snprintf(msg, nmsg, "Applied and saved: HEATK=%.3f, BL255=%.3f", g_tempHeatK, g_ac.computedBl255); }
  else { snprintf(msg, nmsg, "Applied: HEATK=%.3f, BL255=%.3f (not saved)"); }
  return true;
}

// ===== Web: auto-cal JSON + API =====
void handleAutoCalibJson() {
  uint32_t sinceBoot = millis() - g_ac.bootMs;
  int32_t msLeft = (int32_t)((30UL * 60UL * 1000UL) - sinceBoot); if (msLeft < 0) msLeft = 0;
  int minLeft = msLeft / 60000;

  String json = "{";
  json += "\"running\":" + String(g_ac.running ? "true" : "false") + ",";
  json += "\"state\":" + String((int)g_ac.state) + ",";
  json += "\"dry_run\":" + String(g_ac.dryRun ? "true" : "false") + ",";
  json += "\"allowed_now\":" + String(acAllowedNow() ? "true" : "false") + ",";
  json += "\"minutes_until_allowed\":" + String(minLeft) + ",";
  json += "\"ref_temp\":" + String(isfinite(g_ac.refTempC) ? g_ac.refTempC : NAN, 1) + ",";
  json += "\"t1_meas_bl0\":" + String(isfinite(g_ac.t1_meas) ? g_ac.t1_meas : NAN, 2) + ",";
  json += "\"t2_meas_bl255\":" + String(isfinite(g_ac.t2_meas) ? g_ac.t2_meas : NAN, 2) + ",";
  json += "\"computed_heatk\":" + String(isfinite(g_ac.computedHeatK) ? g_ac.computedHeatK : NAN, 3) + ",";
  json += "\"computed_bl255\":" + String(isfinite(g_ac.computedBl255) ? g_ac.computedBl255 : NAN, 3) + ",";
  json += "\"have_results\":" + String(g_ac.haveResults ? "true" : "false") + ",";
  json += "\"applied_on_done\":" + String(g_ac.appliedOnDone ? "true" : "false") + ",";
  json += "\"current_heatk\":" + String(g_tempHeatK, 3) + ",";
  json += "\"current_bl255\":" + String(backlightcomp * 255.0f, 3) + ",";
  json += "\"stable_count\":" + String(g_ac.stableCount);
  json += "}";
  server->send(200, "application/json; charset=utf-8", json);
}
void handleAutoCalibStart() {
  float ref = server->hasArg("ref") ? server->arg("ref").toFloat() : NAN;
  bool dry = server->hasArg("dry") ? (server->arg("dry").toInt() != 0) : false;
  char m[128]; bool ok = autoCalibStart(ref, dry, m, sizeof(m));
  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"message\":\"" + String(m) + "\"";
  json += "}";
  server->send(ok ? 200 : 400, "application/json; charset=utf-8", json);
}
void handleAutoCalibCancel() {
  char m[96]; autoCalibCancel(m, sizeof(m));
  String json = "{\"ok\":true,\"message\":\"" + String(m) + "\"}";
  server->send(200, "application/json; charset=utf-8", json);
}
void handleAutoCalibApply() {
  bool save = server->hasArg("save") ? (server->arg("save").toInt()!=0) : true;
  char m[128]; bool ok = autoCalibApply(save, m, sizeof(m));
  String json = "{";
  json += "\"ok\":" + String(ok ? "true" : "false") + ",";
  json += "\"message\":\"" + String(m) + "\"";
  json += "}";
  server->send(ok ? 200 : 400, "application/json; charset=utf-8", json);
}

// ===== Web: calibration page (manual + auto) =====
void handleCalib() {
  String msg = "";

  bool haveHeatK  = server->hasArg("heatk");
  bool haveBl255  = server->hasArg("bl255");
  bool haveBlComp = server->hasArg("blcomp");

  if (haveHeatK || haveBl255 || haveBlComp) {
    float hk = g_tempHeatK;
    float bc = backlightcomp;

    if (haveHeatK) {
      bool ok = false;
      float v = parseFloatLocale(server->arg("heatk"), &ok);
      if (ok && isfinite(v)) hk = fminf(fmaxf(v, 0.0f), 10.0f);
    }
    if (haveBl255) {
      bool ok = false;
      float v = parseFloatLocale(server->arg("bl255"), &ok);
      if (ok && isfinite(v)) bc = fminf(fmaxf(v / 255.0f, 0.0f), 0.1f);
    } else if (haveBlComp) {
      bool ok = false;
      float v = parseFloatLocale(server->arg("blcomp"), &ok);
      if (ok && isfinite(v)) bc = fminf(fmaxf(v, 0.0f), 0.1f);
    }

    g_tempHeatK   = hk;
    backlightcomp = bc;

    DrawCmd c{}; c.type = CMD_SAVE_CALIB; q_push(c);
    msg += "<div style='color:green;margin:8px 0;'>Saved new calibration values</div>";
  }

  String html = "<!doctype html><html lang='da'><head><meta charset='utf-8'><title>Calibration</title><style>"
                "body{font-family:Arial,sans-serif;margin:16px}"
                "label{display:block;margin:8px 0 4px}"
                "input[type=number]{width:220px;padding:6px}"
                ".hint{font-size:12px;color:#444}"
                ".btn{margin-top:12px;padding:8px 12px}"
                "a{margin-left:12px}"
                "fieldset{margin-top:16px;padding:12px;border:1px solid #ccc;border-radius:6px}"
                "legend{padding:0 6px;color:#333}"
                "</style></head><body>";

  html += "<h2>Calibration</h2>";
  if (msg.length()) html += msg;

  html += "<fieldset><legend>Manual</legend>";
  html += "<form action='/calib' method='get'>";
  html += "<label>Fixed heat offset (HEATK), &deg;C</label>";
  html += "<input type='number' name='heatk' step='0.001' min='0' max='10' value='" + String(g_tempHeatK, 3) + "'>";
  html += "<div class='hint'>Typically 2&ndash;4 &deg;C depending on your build.</div>";

  html += "<label>Backlight-induced rise (&deg;C at PWM=255)</label>";
  html += "<input type='number' name='bl255' step='0.001' min='0' max='10' value='" + String(backlightcomp * 255.0f, 3) + "'>";
  html += "<div class='hint'>Shown as &deg;C at 255; internally stored as &deg;C per PWM-count (" + String(backlightcomp, 6) + ").</div>";

  html += "<br><input class='btn' type='submit' value='Save'>";
  html += "</form>";
  html += "</fieldset>";

  uint32_t sinceBoot = millis() - g_ac.bootMs;
  int32_t msLeft = (int32_t)((30UL * 60UL * 1000UL) - sinceBoot); if (msLeft < 0) msLeft = 0;
  int minLeft = msLeft / 60000;

  html += "<fieldset><legend>Auto-calibration</legend>";
  html += "<form action='/calib' method='get'>";
  html += "<label>Reference temperature (&deg;C)</label>";
  html += "<input type='number' name='ref' step='0.1' min='-40' max='85' value='" + String(isfinite(g_ac.refTempC)?g_ac.refTempC:NAN, 1) + "'>";
  html += "<div class='hint'>Requires a stable environment. Can be started only 30 minutes after boot. Each phase holds at least 10 minutes (max 20) and ends when the reading is stable at 0.1 &deg;C resolution.</div>";
  html += "<label><input type='checkbox' name='dry' value='1' " + String(g_ac.dryRun ? "checked" : "") + "> Dry-run (compute; don’t auto-save)</label>";

  if (!g_ac.running) {
    if (acAllowedNow()) {
      html += "<br><button class='btn' type='submit' name='startac' value='1'>Start Auto-calibration</button>";
    } else {
      html += "<div class='hint' style='color:#b00;'>Can start in ~" + String(minLeft) + " min.</div>";
      html += "<br><button class='btn' type='submit' name='startac' value='1' disabled>Start Auto-calibration</button>";
    }
  } else {
    html += "<div class='hint'>Running... state=" + String((int)g_ac.state) + ", stable in a row: " + String(g_ac.stableCount) + "</div>";
    html += "<br><button class='btn' type='submit' name='cancelac' value='1'>Cancel</button>";
  }
  html += "</form>";

  html += "<div class='hint'>Measured BL=0: " + String(isfinite(g_ac.t1_meas)?g_ac.t1_meas:NAN, 2) + " &deg;C, "
                       "BL=255: " + String(isfinite(g_ac.t2_meas)?g_ac.t2_meas:NAN, 2) + " &deg;C</div>";

  if (g_ac.haveResults) {
    html += "<div class='hint'>Suggested: HEATK=" + String(g_ac.computedHeatK,3) + " &deg;C, "
                         "Backlight rise=" + String(g_ac.computedBl255,3) + " &deg;C at 255</div>";
    html += "<form action='/calib' method='get' style='margin-top:8px;'>";
    html += "<input type='hidden' name='applyac' value='1'>";
    html += "<input type='hidden' name='save' value='1'>";
    html += "<button class='btn' type='submit'>Apply and save results</button>";
    html += "</form>";
  }

  html += "<div class='hint'>Current: HEATK=" + String(g_tempHeatK,3) + " &deg;C, "
                       "Backlight rise=" + String(backlightcomp*255.0f,3) + " &deg;C at 255</div>";
  html += "<div class='hint'>API: <a href='/auto_calib.json'>/auto_calib.json</a>, <a href='/calib.json'>/calib.json</a></div>";
  html += "</fieldset>";

  html += "<br><a href='/' class='btn' style='border:1px solid #444;text-decoration:none;'>Back</a>";
  html += "</body></html>";

  server->send(200, "text/html; charset=utf-8", html);
}

// Register web routes
void setupWebHandlers() {
  server->on("/", handleIndex);
  server->on("/index.htm", handleIndex);
  server->on("/statusjson.htm", handleStatusJson);
  server->on("/backlight", handleBacklight);
  server->on("/calib", handleCalib);
  server->on("/calib.json", handleCalibJson);

  server->on("/auto_calib.json", handleAutoCalibJson);
  server->on("/auto_calib_start", handleAutoCalibStart);
  server->on("/auto_calib_cancel", handleAutoCalibCancel);
  server->on("/auto_calib_apply", handleAutoCalibApply);

  server->begin();
}

// ===== Temperature compensation model =====
// ambient = measured − (HEATK + backlightcomp × backlightPWM)
static float estimateAmbientTemp(float t_meas) {
  float dT = g_tempHeatK;
  float bT = backlightcomp * g_backlight;
  return t_meas - (dT + bT);
}

// Humidity correction (Magnus), expressed at ambient temperature
static float correctedRH(float t_meas_C, float rh_meas_pct, float t_amb_C) {
  if (!isfinite(t_meas_C) || !isfinite(rh_meas_pct) || !isfinite(t_amb_C)) return rh_meas_pct;
  rh_meas_pct = fminf(fmaxf(rh_meas_pct, 0.f), 100.f);
  float es_meas = 6.112f * expf((17.62f * t_meas_C) / (243.12f + t_meas_C));
  float es_amb  = 6.112f * expf((17.62f * t_amb_C ) / (243.12f + t_amb_C ));
  float abs_h   = (rh_meas_pct/100.0f) * es_meas;
  float rh_new  = (abs_h / es_amb) * 100.0f;
  return fminf(fmaxf(rh_new, 0.f), 100.f);
}

// Misc formatting helpers
static void fmtPressureHpa(float p_hpa, char* out, size_t n) { if (!isfinite(p_hpa)) { snprintf(out, n, "----.--"); return; } snprintf(out, n, "%.2f", p_hpa); }
static int pressureCategory(float p_hpa) {
  if (!isfinite(p_hpa)) return 2;
  int vi = (int)floorf(p_hpa);
  if (vi < 950)   return 0;
  if (vi < 983)   return 1;
  if (vi < 1018)  return 2;
  if (vi < 1050)  return 3;
  return 4;
}
static const char* dowText(int dow_0_sun) {
  switch (dow_0_sun) {
    case 0: return " Soendag ";
    case 1: return " Mandag  ";
    case 2: return " Tirsdag ";
    case 3: return " Onsdag  ";
    case 4: return " Torsdag ";
    case 5: return " Fredag  ";
    default:return " Loerdag ";
  }
}
static void fmtTime(int hh, int mm, int ss, char* out, size_t n) { snprintf(out, n, "%02d:%02d:%02d", hh, mm, ss); }
static void fmtDate(int yyyy, int mm, int dd, char* out, size_t n) { snprintf(out, n, "%02d/%02d/%04d", dd, mm, yyyy); }

// Fallback: set RP2040 RTC from compile-time (UTC)
static int monthFromStr(const char* mon) {
  if      (!strncmp(mon, "Jan", 3)) return 1;
  else if (!strncmp(mon, "Feb", 3)) return 2;
  else if (!strncmp(mon, "Mar", 3)) return 3;
  else if (!strncmp(mon, "Apr", 3)) return 4;
  else if (!strncmp(mon, "May", 3)) return 5;
  else if (!strncmp(mon, "Jun", 3)) return 6;
  else if (!strncmp(mon, "Jul", 3)) return 7;
  else if (!strncmp(mon, "Aug", 3)) return 8;
  else if (!strncmp(mon, "Sep", 3)) return 9;
  else if (!strncmp(mon, "Oct", 3)) return 10;
  else if (!strncmp(mon, "Nov", 3)) return 11;
  else return 12;
}
static void setRtcFromCompileTimeUTC() {
  datetime_t dt = {0};
  const char* d = __DATE__;
  const char* t = __TIME__;
  char monStr[4]; monStr[0]=d[0]; monStr[1]=d[1]; monStr[2]=d[2]; monStr[3]=0;
  dt.month = monthFromStr(monStr);
  dt.day   = (d[4]==' ' ? d[5]-'0' : (d[4]-'0')*10 + (d[5]-'0'));
  dt.year  = (d[7]-'0')*1000 + (d[8]-'0')*100 + (d[9]-'0')*10 + (d[10]-'0');
  dt.hour = (t[0]-'0')*10 + (t[1]-'0');
  dt.min  = (t[3]-'0')*10 + (t[4]-'0');
  dt.sec  = (t[6]-'0')*10 + (t[7]-'0');
  dt.dotw = 0;
  rtc_init(); rtc_set_datetime(&dt);
}
static void readRtc(datetime_t* out) { rtc_get_datetime(out); }

// =================== Core0: setup/loop ===================

void setup() {
  Serial.begin(115200);
  delay(200);
  DBG("[Core0] setup start");

  // Cross-core queue
  queue_init(&g_q, sizeof(DrawCmd), 64);
  g_queue_ready = true;
  DBG("[Core0] queue ready");

  // I2C bring-up
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin(); delay(10);
  DBG("[Core0] I2C begin");

  // BME280
  bme_ok = bme.begin(BME_ADDR, &Wire);
  Serial.println(bme_ok ? "BME280 OK" : "BME280 NOT FOUND");

  // DS3231
  g_ds_ok = g_ds.begin();
  if (g_ds_ok) {
    if (g_ds.lostPower()) { Serial.println("[RTC] DS3231 lost power; waiting for NTP (UTC)"); setRtcFromCompileTimeUTC(); }
    else { (void)ds3231ReadToRp2040UTC(); }
  } else {
    Serial.println("[RTC] DS3231 not found");
    setRtcFromCompileTimeUTC();
  }

  // Wait a bit for Core1 (not blocking boot)
  uint32_t t0 = millis();
  while (!g_core1_ready && millis() - t0 < 4000) { delay(1); }
  if (!g_core1_ready) DBG("[Core0] Core1 not ready after 4s (continuing)");

  // Initial visuals
  DrawCmd c{};
  c = DrawCmd{}; c.type = CMD_HPA_BG_CAT; c.ivalue = 2; q_push(c);
  datetime_t rt_utc; readRtc(&rt_utc);
  datetime_t rt_local; utcToLocalEU(rt_utc, &rt_local);
  c = DrawCmd{}; c.type = CMD_CLK_BG_MONTH; c.ivalue = rt_local.month; q_push(c);

  // First sensor read (if available)
  char hTxt[24] = "--.-%";
  char pTxt[24] = "----.--";
  char tTxt[24] = "--.-C";
  if (bme_ok) {
    bme.setSampling(
      Adafruit_BME280::MODE_FORCED,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_OFF,
      Adafruit_BME280::STANDBY_MS_1000
    );
    bme.takeForcedMeasurement();
    float t_meas = bme.readTemperature();
    float rh_meas = bme.readHumidity();
    float p_hpa   = (bme.readPressure() / 100.0f) + PRESS_OFFSET;

    float t_amb   = estimateAmbientTemp(t_meas);
    float rh_corr = correctedRH(t_meas, rh_meas, t_amb);

    snprintf(hTxt, sizeof(hTxt), "%.1f%%", rh_corr);
    fmtPressureHpa(p_hpa, pTxt, sizeof(pTxt));
    snprintf(tTxt, sizeof(tTxt), "%.1fC", t_amb);
  }
  c = DrawCmd{}; c.type = CMD_HUM_TEXT;  strncpy(c.payload, hTxt, sizeof(hTxt)); q_push(c);
  c = DrawCmd{}; c.type = CMD_HPA_TEXT;  strncpy(c.payload, pTxt, sizeof(pTxt)); q_push(c);
  c = DrawCmd{}; c.type = CMD_TEMP_TEXT; strncpy(c.payload, tTxt, sizeof(tTxt)); q_push(c);

  // WiFi/NTP schedule
  g_lastWiFiAttemptMs = millis() - WIFI_RETRY_INTERVAL_MS;
  g_lastNtpSyncMs     = millis() - NTP_SYNC_INTERVAL_MS;

  // Initial WiFi indicator color
  uint16_t wifiCol = (WiFi.status() == WL_CONNECTED) ? C_GREEN : C_RED;
  c = DrawCmd{}; c.type = CMD_CLK_IND_WIFI; c.ivalue = wifiCol; q_push(c);

  // Web server routes
  setupWebHandlers();
  DBG("[Core0] web started");

  // Auto-cal ticker
  g_ac.bootMs = millis();
  g_acTicker.attach_ms(g_ac.tickIntervalMs, acTickerISR);
  DBG("[Core0] setup done");
}

void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick < 200) return;
  lastTick = now;

  if (g_ac.tickFlag) { g_ac.tickFlag = false; autoCalibStep(); }

  wifiMaintainInLoop();
  ntpSyncMaintain();

  // Sensor refresh ~1 Hz
  static uint32_t lastSens = 0;
  static char lastHumTxt[24] = "";
  static char lastHpaTxt[24] = "";
  static char lastTempTxt[24] = "";
  static int  lastCat = -1;

  if (millis() - lastSens >= 1000) {
    lastSens = millis();

    float t_amb = NAN, rh_corr = NAN, p_hpa = NAN;

    if (bme_ok) {
      bme.takeForcedMeasurement();
      float t_meas = bme.readTemperature();
      float rh_meas = bme.readHumidity();
      p_hpa = (bme.readPressure() / 100.0f) + PRESS_OFFSET;

      t_amb          = estimateAmbientTemp(t_meas);
      g_lastTemp     = t_amb;
      rh_corr        = correctedRH(t_meas, rh_meas, t_amb);
      g_lastHumidity = rh_corr;
      g_lastHpa      = p_hpa;
    }
    g_cpuTemp = analogReadTemp();

    // HUM
    char hTxt[24]; snprintf(hTxt, sizeof(hTxt), "%.1f%%", rh_corr);
    if (strcmp(hTxt, lastHumTxt) != 0) {
      DrawCmd c{}; c.type = CMD_HUM_TEXT; strncpy(c.payload, hTxt, sizeof(c.payload)); q_push(c);
      strncpy(lastHumTxt, hTxt, sizeof(lastHumTxt));
    }

    // HPA
    char pTxt[24]; fmtPressureHpa(p_hpa, pTxt, sizeof(pTxt));
    if (strcmp(pTxt, lastHpaTxt) != 0) {
      DrawCmd c{}; c.type = CMD_HPA_TEXT; strncpy(c.payload, pTxt, sizeof(c.payload)); q_push(c);
      strncpy(lastHpaTxt, pTxt, sizeof(lastHpaTxt));
    }

    // TEMP
    char tTxt[24]; snprintf(tTxt, sizeof(tTxt), "%.1fC", t_amb);
    if (strcmp(tTxt, lastTempTxt) != 0) {
      DrawCmd c{}; c.type = CMD_TEMP_TEXT; strncpy(c.payload, tTxt, sizeof(tTxt)); q_push(c);
      strncpy(lastTempTxt, tTxt, sizeof(lastTempTxt));
    }

    // HPA category background
    int cat = pressureCategory(p_hpa);
    if (cat != lastCat) {
      DrawCmd c{}; c.type = CMD_HPA_BG_CAT; c.ivalue = cat; q_push(c);
      lastCat = cat;
    }
  }

  // Time updates once per second
  datetime_t rt_utc; readRtc(&rt_utc);
  datetime_t rt; utcToLocalEU(rt_utc, &rt);
  static int lastSecShown = -1;
  if (rt.sec != lastSecShown) {
    lastSecShown = rt.sec;

    // Month background may change at month rollover
    static int lastMonthSent = -1;
    if (rt.month != lastMonthSent) {
      lastMonthSent = rt.month;
      DrawCmd c{}; c.type = CMD_CLK_BG_MONTH; c.ivalue = rt.month; q_push(c);
    }

    int dow = sakamotoDow(rt.year, rt.month, rt.day);
    char timeBuf[24], dateBuf[24];
    fmtTime(rt.hour, rt.min, rt.sec, timeBuf, sizeof(timeBuf));
    fmtDate(rt.year, rt.month, rt.day, dateBuf, sizeof(dateBuf));
    const char* dowTxt = dowText(dow);

    DrawCmd c{};
    c = DrawCmd{}; c.type = CMD_CLK_TEXT_DOW;  strncpy(c.payload, dowTxt,  sizeof(c.payload)); q_push(c);
#if (USE_ANALOG_CLOCK == 0) || (USE_ANALOG_CLOCK == 2)
    c = DrawCmd{}; c.type = CMD_CLK_TEXT_TIME; strncpy(c.payload, timeBuf, sizeof(c.payload)); q_push(c);
#endif
    c = DrawCmd{}; c.type = CMD_CLK_TEXT_DATE; strncpy(c.payload, dateBuf, sizeof(c.payload)); q_push(c);

#if (USE_ANALOG_CLOCK >= 1)
    c = DrawCmd{}; c.type = CMD_CLK_ANALOG_HMS;
    c.ivalue = ((rt.hour & 0xFF) << 16) | ((rt.min & 0xFF) << 8) | (rt.sec & 0xFF);
    q_push(c);
#endif

    // Update web UI friendly time
    g_lastTime = String(rt.day) + "/" + String(rt.month) + "/" + String(rt.year) + " " +
                 String(rt.hour) + ":" + String(rt.min) + ":" + String(rt.sec);

    // Handle any web requests
    server->handleClient();
    delay(1);
  }

#if DIAG_CYCLE_HPA
  // Useful for demo/testing: cycle clock backgrounds
  static uint32_t lastClkBgTick = 0; static int diagMonth = 1;
  if (millis() - lastClkBgTick > 8000) {
    lastClkBgTick = millis();
    DrawCmd c{}; c.type = CMD_CLK_BG_MONTH; c.ivalue = diagMonth; q_push(c);
    diagMonth += 1; if (diagMonth > 12) diagMonth = 1;
  }
#endif
}
