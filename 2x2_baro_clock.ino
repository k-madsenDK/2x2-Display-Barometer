/*
  SBComputer 2x2 Barometer/Clock for RP2040 Pico (dual-core)
  Target device: "Pico 2x2" 2x2 unit with 4x ST7789 240x240 displays

  What this sketch does
  - Runs a dual-core firmware:
    - Core1 owns and drives: 4x ST7789 TFTs and SD card (SPI0/SPI1), plus draws status elements.
    - Core0 owns: BME280 sensor logic, RTC/DS3231 + NTP timekeeping, WiFi, web server, and the auto-calibration state machine.
  - Shows humidity, temperature (corrected for self-heating and backlight), pressure, and a clock with month backgrounds.
  - Provides a simple web UI to view status and adjust:
    - Backlight (0..255)
    - Temperature calibration parameters:
      - HEATK: fixed heat offset (°C at backlight=0).
      - Backlight temperature rise at 255 (shown as °C at 255; stored internally as °C per PWM-count).
  - Saves calibration to /calib.cfg on SD and restores on boot.
  - Supports an interrupt-like, low-CPU auto-calibration procedure from Core0 (Ticker):
    - Enforces a 30-minute wait after boot and requires a user-provided reference temperature.
    - State 1: backlight = 0. Hold at least 10 minutes (max 20) until readings stabilize (1 decimal stable).
    - State 2: backlight = 255. Same timing/stability criteria.
    - Results:
      - HEATK = max(0, temp_at_BL0 − reference_temp).
      - Backlight rise at 255 = max(0, temp_at_BL255 − temp_at_BL0).
      - Internally stores backlightcomp = (backlight rise at 255)/255.
    - Dry-run mode lets you preview computed values (and optionally apply+save afterward).

  Web endpoints
  - /                      HTML status page + backlight control + link to calibration page
  - /statusjson.htm        JSON with current readings
  - /backlight?bl=0..255   Set raw backlight PWM
  - /calib                 Calibration page (manual and auto-cal controls)
  - /calib.json            JSON: heatk, bl255, blcomp
  - /auto_calib.json       JSON: auto-calibration state and results
  - /auto_calib_start?ref=<C>&dry=1|0  Start auto-cal (requires 30 min since boot and a valid ref temp)
  - /auto_calib_cancel     Cancel auto-cal
  - /auto_calib_apply?save=1|0  Apply last computed results (and optionally save)

  Build notes
  - Arduino-Pico (Earle Philhower) core for RP2040.
  - Libraries: Adafruit_GFX, Adafruit_ST7789, Adafruit_BME280, RTClib, WiFi (CYW43), NTPClient, SD, Ticker.
  - GPIO summary (see constants and comments below). SPI0 is remapped between SD and 3 TFTs; SPI1 used for HUM display.
  - The BME280 compensation model:
    ambient = measured - (HEATK + backlightcomp * backlightPWM)
      HEATK is the fixed self-heating at BL=0 (°C).
      backlightcomp is the per-count rise (°C/count), shown in UI as °C at 255.

  Maker-friendly tips
  - If you change wiring, only adjust the pin defines at the top and SPI mapping helpers.
  - If you don’t have SD images, you’ll still get text on white backgrounds.
  - Use /calib to store HEATK and backlight rise; the device will remember values across reboots.
  - Auto-cal is slow-by-design to reduce CPU load and let the sensor reach equilibrium.
*/

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
#include <math.h>   // expf/fminf/fmaxf
#include <Ticker.h> // lightweight periodic ISR-like ticks on Core0
#include <stdlib.h>  // strtod

// RP2040 Pico SDK headers (Earle core)
#include "hardware/rtc.h"
#include "pico/util/datetime.h"
#include "pico/util/queue.h"

// Embedded RGB565 image for TEMP display (defines TEMP240_W, TEMP240_H, uint16_t temp240[])
#include "temp240.h"
#include "Cyw43Power.h"

// ===== Diagnostic toggles at build-time =====
#define DIAG_SD_PING_MS   0
#define DIAG_CYCLE_HPA    0

// =================== Pins and constants ===================
#define TFT_ROT 3

// HUMIDITY TFT (SPI1)
#define HUM_CS   13
#define HUM_DC   12
#define HUM_RST  -1
#define HUM_SCK  10
#define HUM_MOSI 11

// TEMP TFT (SPI0 default pins: 18/19), conflict: DC=16 is shared with SD MISO
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

// Backlight PWM (GP14)
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

// Colors
#define C_BLACK   0x0000
#define C_WHITE   0xFFFF
#define C_RED     0xF800
#define C_GREEN   0x07E0
#define C_BLUE    0x001F
#define C_CYAN    0x07FF
#define C_YELLOW  0xFFE0
#define C_ORANGE  0xFD20

// Text size
#define TXT_SIZE 3

// On-screen label/value placement (matching earlier MicroPython layout)
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

#define C_DOW_Y    0
#define C_TIME_Y   160
#define C_DATE_Y   200

// Single status indicator (WiFi) — 12x12 square top-right of clock display
#define IND_SIZE    12
#define IND_MARGIN   3
#define IND_Y        IND_MARGIN
#define IND_WIFI_X  (240 - IND_MARGIN - IND_SIZE)

// Pressure offset (site elevation etc.)
#define PRESS_OFFSET 7.0f

// Backlight PWM (0..255)
static uint8_t g_backlight = 255;

// Temperature compensation model parameters
// HEATK: fixed heat offset at backlight=0 (°C)
// backlightcomp: temperature rise per PWM-count (°C/count), shown in UI as °C at 255
#define temp_diag 0
static float g_tempHeatK   = 2.8f;
static float backlightcomp = 3.7f / 255.0f;

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

// Core sync
volatile bool g_queue_ready = false;  // queue initialized by Core0
volatile bool g_core1_ready = false;  // Core1 finished HW init

// Is SPI0 temporarily mapped to TEMP?
static volatile bool g_spi0_temp_busy = false;

// =================== Cross-core draw command queue ===================
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
  CMD_SAVE_CALIB      // Core1 saves /calib.cfg (Core1 owns SD)
};
struct DrawCmd {
  uint8_t  type;
  int32_t  ivalue;
  char     payload[24];
};
static queue_t g_q;
static inline void q_push(const DrawCmd& c) { queue_try_add(&g_q, &c); }

// =================== Core1 helpers: SPI routing and IO ===================
// Keep all chip-selects inactive
static inline void force_all_tft_cs_high() {
  pinMode(HUM_CS, OUTPUT); digitalWrite(HUM_CS, HIGH);
  pinMode(HPA_CS, OUTPUT); digitalWrite(HPA_CS, HIGH);
  pinMode(CLK_CS, OUTPUT); digitalWrite(CLK_CS, HIGH);
  pinMode(TFT4_CS, OUTPUT); digitalWrite(TFT4_CS, HIGH);
  pinMode(SD_CS,  OUTPUT); digitalWrite(SD_CS,  HIGH);
}

// IMPORTANT: TEMP_DC (GP16) doubles as SD MISO. We remap SPI0 on the fly.
static inline void mapSPI0_to_SD() {
  pinMode(TEMP_DC, INPUT);   // free GP16 for SD MISO
  SPI0.setSCK(SD_SCK);
  SPI0.setTX(SD_MOSI);
  SPI0.setRX(SD_MISO);
  SPI0.begin();
  digitalWrite(SD_CS, HIGH);
}
static inline void mapSPI0_to_TEMP() {
  SPI0.setSCK(TEMP_SCK);
  SPI0.setTX(TEMP_MOSI);
  SPI0.begin();
  pinMode(TEMP_DC, OUTPUT);
}
static inline void mapSPI0_to_HPA() {
  SPI0.setSCK(HPA_SCK);
  SPI0.setTX(HPA_MOSI);
  SPI0.begin();
  digitalWrite(HPA_CS, HIGH);
}
static inline void mapSPI0_to_CLK() {
  SPI0.setSCK(CLK_SCK);
  SPI0.setTX(CLK_MOSI);
  SPI0.begin();
  digitalWrite(CLK_CS, HIGH);
}

// TEMP access critical section (SPI0 remap)
static inline void beginTempAccess() {
  digitalWrite(SD_CS, HIGH);
  digitalWrite(HPA_CS, HIGH);
  digitalWrite(CLK_CS, HIGH);
  g_spi0_temp_busy = true;
  SPI0.end();
  mapSPI0_to_TEMP();
}
static inline void endTempAccess() {
  mapSPI0_to_SD();
  g_spi0_temp_busy = false;
}

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

// Simple 24-bit BMP blitters (expect up to 240x240)
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

// Simple text helpers (Core1)
static inline int textPixelWidth(const char* s, int textSize) { return (int)strlen(s) * 6 * textSize; }
static void drawCentered(Adafruit_ST7789* tft, int y, const char* txt, uint16_t fg=C_BLACK, uint16_t bg=C_WHITE, int size=TXT_SIZE) {
  int w = textPixelWidth(txt, size);
  int x = (240 - w) / 2; if (x < 0) x = 0;
  tft->setTextWrap(false);
  tft->setTextSize(size);
  tft->setTextColor(fg, bg);
  tft->setCursor(x, y);
  tft->print(txt);
}
static void drawHumidityLabel(Adafruit_ST7789* tft) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(F_LABEL_X, F_LABEL_Y); tft->print(" Fugtighed "); }
static void drawHumidityValue(Adafruit_ST7789* tft, const char* txt) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(F_VALUE_X, F_VALUE_Y); tft->print(" "); tft->print(txt); tft->print(" "); }
static void drawTempLabel(Adafruit_ST7789* tft) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(T_LABEL_X, T_LABEL_Y); tft->print(" Temp "); }
static void drawTempValue(Adafruit_ST7789* tft, const char* txt) { tft->setTextWrap(false); tft->setTextSize(TXT_SIZE); tft->setTextColor(C_BLACK, C_WHITE); tft->setCursor(T_VALUE_X, T_VALUE_Y); tft->print(" "); tft->print(txt); tft->print(" "); }
static void drawHpaLabel(Adafruit_ST7789* tft) { drawCentered(tft, P_LABEL_Y, " Hpa "); }
static void drawHpaValue(Adafruit_ST7789* tft, const char* txt) { char buf[24]; snprintf(buf, sizeof(buf), " %s ", txt); drawCentered(tft, P_VALUE_Y, buf); }
static void drawHpaState(Adafruit_ST7789* tft, const char* txt) { drawCentered(tft, P_STATE_Y, txt); }
static char s_lastHpaText[24] = "";

// Single WiFi indicator tile on the clock display
static uint16_t s_wifi_col = 0;
static void drawWifiDot(uint16_t color) { if (!tftClk) return; tftClk->fillRect(IND_WIFI_X, IND_Y, IND_SIZE, IND_SIZE, color); }
static void redrawWifiDot() { if (s_wifi_col) drawWifiDot(s_wifi_col); }

// Assets selectors
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

// =================== WiFi config from SD (Core1 reads early) ===================
#define WIFI_CFG_FILE "/wifi.cfg"
String wifi_ssid = "";
String wifi_pass = "";

// Core1 reads WiFi creds early (before Core0 queue init) to avoid SPI0 remap conflicts
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
  if (!g_wifi_cfg_ready) return false;  // Core1 hasn't loaded creds yet (or missing)
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
  // Update simple indicator on state changes
  if (st != lastShownWiFi && g_queue_ready) {
    DrawCmd c{}; c.type = CMD_CLK_IND_WIFI; c.ivalue = (st == WL_CONNECTED) ? C_GREEN : C_RED; q_push(c);
    lastShownWiFi = st;
  }

  if (st == WL_CONNECTED) { last = st; discSince = 0; wifiKeepAlive(); return; }
  if (last == WL_CONNECTED && st != WL_CONNECTED) {
    if (discSince == 0) discSince = millis();
    if (millis() - discSince < 2000) return; // short grace period
  }
  last = st;

  uint32_t now = millis();
  if (now - g_lastWiFiAttemptMs >= WIFI_RETRY_INTERVAL_MS) {
    g_lastWiFiAttemptMs = now;
    (void)wifiConnectWithTimeout(WIFI_LOOP_TIMEOUT_MS);
  }
}

// =================== RTC/DS3231/NTP (UTC internal, local for display) ===================
static RTC_DS3231 g_ds;
static bool g_ds_ok = false;

WiFiUDP g_ntpUdp;
NTPClient g_ntp(g_ntpUdp, "dk.pool.ntp.org", 0 /*UTC*/, 60000);
static bool g_ntp_inited = false;
static uint32_t g_lastNtpSyncMs = 0;
static const uint32_t NTP_SYNC_INTERVAL_MS = 12UL * 60UL * 60UL * 1000UL; // 12 hours

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

void setup1() {
  // Wait until Core0 has initialized the queue
  while (!g_queue_ready) delay(1);

  // Backlight PWM on GP14 (fast, silent)
  pinMode(BACKLIGHT, OUTPUT);
  analogWriteFreq(20000);
  analogWriteRange(255);
  setBacklight(g_backlight);

  force_all_tft_cs_high();

  // SPI1 for HUM display
  SPI1.setSCK(HUM_SCK);
  SPI1.setTX(HUM_MOSI);
  SPI1.begin();

  // HUMIDITY display (SPI1)
  tftHumid = new Adafruit_ST7789(&SPI1, HUM_CS, HUM_DC, HUM_RST);
  tftHumid->setSPISpeed(48000000);
  tftHumid->init(240, 240);
  tftHumid->setRotation(TFT_ROT);
  tftHumid->fillScreen(C_WHITE);
  tftHumid->drawRect(0, 0, 239, 239, C_RED);
  drawHumidityLabel(tftHumid);

  // TEMP display (SPI0, remapped)
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

  // HPA display (SPI0 remap)
  mapSPI0_to_HPA();
  tftHpa = new Adafruit_ST7789(&SPI0, HPA_CS, HPA_DC, HPA_RST);
  tftHpa->setSPISpeed(48000000);
  tftHpa->init(240, 240);
  tftHpa->setRotation(TFT_ROT);
  tftHpa->fillScreen(C_WHITE);
  tftHpa->drawRect(0, 0, 239, 239, C_RED);
  drawHpaLabel(tftHpa);

  // CLOCK display (SPI0 remap)
  mapSPI0_to_CLK();
  tftClk = new Adafruit_ST7789(&SPI0, CLK_CS, CLK_DC, CLK_RST);
  tftClk->setSPISpeed(48000000);
  tftClk->init(240, 240);
  tftClk->setRotation(TFT_ROT);
  tftClk->fillScreen(C_WHITE);
  tftClk->drawRect(0, 0, 239, 239, C_RED);

  // SD init + read WiFi + read calibration (Core1 does early SD work)
  mapSPI0_to_SD();
  if (SD.begin(SD_CS, SD_HZ, SPI0)) {
    sd_ready = true;
    Serial.print("SD init OK @"); Serial.print(SD_HZ/1000000); Serial.println("MHz");
    (void)core1ReadWifiCfg();
    (void)core1ReadCalibCfg();
  } else {
    sd_ready = false;
    Serial.println("SD.begin FAILED");
    g_wifi_cfg_missing = true; // can't read file
  }

  // Optional HUM background image
  if (sd_ready) {
    (void)drawBMP24_240x240_SPI1(tftHumid, "/img/fugtighed240.bmp");
    tftHumid->drawRect(0, 0, 239, 239, C_RED);
    drawHumidityLabel(tftHumid);
  }

  g_core1_ready = true;
}

static void drawClockDOW(Adafruit_ST7789* tft, const char* txt)  { drawCentered(tft, C_DOW_Y,  txt, C_BLACK, C_WHITE, TXT_SIZE); }
static void drawClockTime(Adafruit_ST7789* tft, const char* txt) { drawCentered(tft, C_TIME_Y, txt, C_BLACK, C_WHITE, TXT_SIZE); }
static void drawClockDate(Adafruit_ST7789* tft, const char* txt) { drawCentered(tft, C_DATE_Y, txt, C_BLACK, C_WHITE, TXT_SIZE); }

void loop1() {
  if (!g_queue_ready) { delay(1); return; }
  DrawCmd cmd;
  if (queue_try_remove(&g_q, &cmd)) {
    switch (cmd.type) {
      case CMD_HUM_TEXT:  drawHumidityValue(tftHumid, cmd.payload); break;
      case CMD_TEMP_TEXT: beginTempAccess(); drawTempValue(tftTemp, cmd.payload); endTempAccess(); break;
      case CMD_HPA_TEXT:
        drawHpaValue(tftHpa, cmd.payload);
        // Gem sidst viste værdi, så den kan gen-tegnes efter en baggrundsopdatering
        strncpy(s_lastHpaText, cmd.payload, sizeof(s_lastHpaText));
        break;
      case CMD_HPA_BG_CAT:
        if (sd_ready) {
          (void)drawBMP24_240x240_HPA_SPI0(tftHpa, categoryBmp(cmd.ivalue));
          tftHpa->drawRect(0, 0, 239, 239, C_RED);
          drawHpaLabel(tftHpa);
          drawHpaState(tftHpa, categoryText(cmd.ivalue));
          // GEN-TEGN måleværdien oven på den nye baggrund
          if (s_lastHpaText[0] != '\0') {
            drawHpaValue(tftHpa, s_lastHpaText);
          }
        }
        break;
      case CMD_CLK_BG_MONTH:
        if (sd_ready) {
          (void)drawBMP24_240x240_CLK_SPI0(tftClk, monthBmp(cmd.ivalue));
          tftClk->drawRect(0, 0, 239, 239, C_RED);
          redrawWifiDot(); // draw indicator above background
        }
        break;
      case CMD_CLK_TEXT_DOW:  drawClockDOW(tftClk, cmd.payload);  break;
      case CMD_CLK_TEXT_TIME: drawClockTime(tftClk, cmd.payload); break;
      case CMD_CLK_TEXT_DATE: drawClockDate(tftClk, cmd.payload); break;
      case CMD_CLK_IND_WIFI:  s_wifi_col = (uint16_t)cmd.ivalue; drawWifiDot(s_wifi_col); break;
      case CMD_SAVE_CALIB:    (void)core1SaveCalibCfg(); break;
      default: break;
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

// Last values for web status
static float g_lastTemp = 0;
static float g_lastHpa = 0;
static float g_lastHumidity = 0;
static float g_cpuTemp = 0;
static String g_lastTime = "--:--:--";

// Backlight control from Core0 (shared PWM)
static inline void setBacklightCore0(uint8_t duty) { g_backlight = duty; analogWrite(BACKLIGHT, duty); }

// Locale-sikker float-parser: accepterer både "3.4" og "3,4"
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

// ===== Calibration helpers and web =====
void handleCalibJson() {
  String json = "{";
  json += "\"heatk\":" + String(g_tempHeatK, 3) + ",";
  json += "\"bl255\":" + String(backlightcomp * 255.0f, 3) + ",";
  json += "\"blcomp\":" + String(backlightcomp, 6);
  json += "}";
  server->send(200, "application/json; charset=utf-8", json);
}

// ===== Auto-calibration (Core0): low-CPU state machine via Ticker =====
// States: IDLE → BL=0 → BL=255 → DONE
enum AutoCalState : uint8_t { AC_IDLE = 0, AC_BL0 = 1, AC_BL255 = 2, AC_DONE = 3 };
struct AutoCalCtx {
  AutoCalState state = AC_IDLE;
  bool  running = false;
  bool  dryRun = true;         // if true: compute only; don't auto-apply+save on completion
  bool  haveResults = false;   // computed values available
  bool  appliedOnDone = false; // if not dry-run, we applied on completion
  uint8_t prevBacklight = 255;

  // Timing: at least 10 minutes per state, up to 20 minutes max
  uint32_t bootMs = 0;               // set in setup()
  uint32_t stateStartMs = 0;
  const uint32_t minHoldMs   = 10UL * 60UL * 1000UL;
  const uint32_t stableMaxMs = 20UL * 60UL * 1000UL;

  // Periodic sampling flag set by Ticker (every 10s)
  volatile bool tickFlag = false;
  uint32_t tickIntervalMs = 10000;

  // Reference and measurements (raw BME temps; no compensation)
  float refTempC = NAN;
  float t1_meas  = NAN;  // BL=0 stable/timeout
  float t2_meas  = NAN;  // BL=255 stable/timeout

  // Computed results
  float computedHeatK = NAN;
  float computedBl255 = NAN;

  // Stability detection (1 decimal stable for 3 consecutive samples)
  float  lastRounded01 = NAN;
  uint8_t stableCount  = 0;
} g_ac;
Ticker g_acTicker;

static inline bool acAllowedNow() {
  uint32_t sinceBoot = millis() - g_ac.bootMs;
  return sinceBoot >= (30UL * 60UL * 1000UL); // 30 min
}

static inline float round01(float x) { return roundf(x * 10.0f) / 10.0f; }
static float readBmeRawTemp() {
  if (!bme_ok) return NAN;
  bme.takeForcedMeasurement();
  return bme.readTemperature();
}
static void acTickerISR() { g_ac.tickFlag = true; }

// Start/Cancel/Apply
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

  setBacklightCore0(0); // enter BL=0 phase
  snprintf(msg, nmsg, "Auto-calibration started (BL=0, %s)", dryRun ? "dry-run" : "will save on completion");
  return true;
}
static void autoCalibCancel(char* msg, size_t nmsg) {
  setBacklightCore0(g_ac.prevBacklight); // restore BL
  g_ac.state = AC_IDLE; g_ac.running = false;
  g_ac.lastRounded01 = NAN; g_ac.stableCount = 0;
  snprintf(msg, nmsg, "Auto-calibration canceled");
}
static void autoCalibFinalizeIfNeeded() {
  // Compute HEATK and BL rise at 255 with sanity bounds
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

  setBacklightCore0(g_ac.prevBacklight); // restore BL
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
  bool stableNow    = (g_ac.stableCount >= 3); // 3 consecutive equal 0.1°C readings

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
  else { snprintf(msg, nmsg, "Applied: HEATK=%.3f, BL255=%.3f (not saved)", g_tempHeatK, g_ac.computedBl255); }
  return true;
}

// ===== Web: auto-cal JSON + no-GUI API =====
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
  // Default = not dry-run; only dry if dry=1 sendes
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
// ===== Web: calibration page (manual + auto) =====
void handleCalib() {
  String msg = "";

  // Manual save (HEATK og BL-kompensation) – accepterer komma eller punktum
  bool haveHeatK  = server->hasArg("heatk");
  bool haveBl255  = server->hasArg("bl255");
  bool haveBlComp = server->hasArg("blcomp"); // bagudkompatibilitet, °C per PWM-count

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
      float v = parseFloatLocale(server->arg("bl255"), &ok); // °C ved 255
      if (ok && isfinite(v)) bc = fminf(fmaxf(v / 255.0f, 0.0f), 0.1f);
    } else if (haveBlComp) {
      bool ok = false;
      float v = parseFloatLocale(server->arg("blcomp"), &ok); // °C pr. PWM-count
      if (ok && isfinite(v)) bc = fminf(fmaxf(v, 0.0f), 0.1f);
    }

    g_tempHeatK   = hk;
    backlightcomp = bc;

    // Gem på SD via Core1
    DrawCmd c{}; c.type = CMD_SAVE_CALIB; q_push(c);
    msg += "<div style='color:green;margin:8px 0;'>Saved new calibration values</div>";
  }

  // Auto-cal start/cancel/apply fra GUI
  if (server->hasArg("startac")) {
    float ref = server->hasArg("ref") ? server->arg("ref").toFloat() : NAN;
    bool dry = server->hasArg("dry") ? (server->arg("dry").toInt()!=0) : false; // default = ikke dry-run
    char m[128];
    if (autoCalibStart(ref, dry, m, sizeof(m))) msg += String("<div style='color:green;margin:8px 0;'>") + m + "</div>";
    else msg += String("<div style='color:#b00;margin:8px 0;'>") + m + "</div>";
  } else if (server->hasArg("cancelac")) {
    char m[96]; autoCalibCancel(m, sizeof(m));
    msg += String("<div style='color:#444;margin:8px 0;'>") + m + "</div>";
  } else if (server->hasArg("applyac")) {
    bool save = server->hasArg("save") ? (server->arg("save").toInt()!=0) : true;
    char m[128];
    if (autoCalibApply(save, m, sizeof(m))) msg += String("<div style='color:green;margin:8px 0;'>") + m + "</div>";
    else msg += String("<div style='color:#b00;margin:8px 0;'>") + m + "</div>";
  }

  // Render side (uændret nedenfor)
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

  html += "<label>Backlight-induced temperature rise (&deg;C at PWM=255)</label>";
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
  html += "<label><input type='checkbox' name='dry' value='1' " + String(g_ac.dryRun ? "checked" : "") + "> Dry-run (compute; don&rsquo;t auto-save)</label>";

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

  // Auto-cal no-GUI API
  server->on("/auto_calib.json", handleAutoCalibJson);
  server->on("/auto_calib_start", handleAutoCalibStart);
  server->on("/auto_calib_cancel", handleAutoCalibCancel);
  server->on("/auto_calib_apply", handleAutoCalibApply);

  server->begin();
}

// ===== Temperature compensation model =====
// ambient = measured - (HEATK + backlightcomp * backlightPWM)
// This is a simple, robust model for maker builds with some internal heating.
static float estimateAmbientTemp(float t_meas) {
  float dT = g_tempHeatK;                 // fixed offset (°C)
  float bT = backlightcomp * g_backlight; // backlight-induced rise (°C)
#if temp_diag
  Serial.print("raw="); Serial.print(t_meas);
  Serial.print(" comp="); Serial.print(dT+bT);
  Serial.print(" amb="); Serial.println(t_meas - (dT+bT));
#endif
  return t_meas - (dT + bT);
}

// Humidity correction to the ambient temperature (Magnus formula)
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
  delay(120);
  Serial.println("Pico SBComputer 2x2 Display dual-core: core1=SD+TFT, core0=Sensor/RTC");

  // Initialize cross-core queue first
  queue_init(&g_q, sizeof(DrawCmd), 64);
  g_queue_ready = true;

  // I2C + sensors/RTC
  Wire.setSDA(I2C_SDA);
  Wire.setSCL(I2C_SCL);
  Wire.begin(); delay(10);

  bme_ok = bme.begin(BME_ADDR, &Wire);
  Serial.println(bme_ok ? "BME280 OK" : "BME280 NOT FOUND");

  // DS3231 (UTC)
  g_ds_ok = g_ds.begin();
  if (g_ds_ok) {
    if (g_ds.lostPower()) { Serial.println("[RTC] DS3231 lost power; waiting for NTP (UTC)"); setRtcFromCompileTimeUTC(); }
    else { (void)ds3231ReadToRp2040UTC(); }
  } else {
    Serial.println("[RTC] DS3231 not found");
    setRtcFromCompileTimeUTC();
  }

  // Wait a bit for Core1 to finish
  uint32_t t0 = millis();
  while (!g_core1_ready && millis() - t0 < 4000) { delay(1); }

  // Initial screens
  DrawCmd c{};
  c = DrawCmd{}; c.type = CMD_HPA_BG_CAT; c.ivalue = 2; q_push(c);
  datetime_t rt_utc; readRtc(&rt_utc);
  datetime_t rt_local; utcToLocalEU(rt_utc, &rt_local);
  c = DrawCmd{}; c.type = CMD_CLK_BG_MONTH; c.ivalue = rt_local.month; q_push(c);

  // Initial sensor read in forced mode
  char hTxt[24] = "--.-%";
  char pTxt[24] = "----.--";
  char tTxt[24] = "--.-C";
  if (bme_ok) {
    bme.setSampling(
      Adafruit_BME280::MODE_FORCED,
      Adafruit_BME280::SAMPLING_X1,   // temp
      Adafruit_BME280::SAMPLING_X1,   // press
      Adafruit_BME280::SAMPLING_X1,   // hum
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

  // Initial time strings
  char timeBuf[24], dateBuf[24];
  fmtTime(rt_local.hour, rt_local.min, rt_local.sec, timeBuf, sizeof(timeBuf));
  fmtDate(rt_local.year, rt_local.month, rt_local.day, dateBuf, sizeof(dateBuf));
  int dow = sakamotoDow(rt_local.year, rt_local.month, rt_local.day);
  c = DrawCmd{}; c.type = CMD_CLK_TEXT_DOW;  strncpy(c.payload, dowText(dow), sizeof(c.payload)); q_push(c);
  c = DrawCmd{}; c.type = CMD_CLK_TEXT_TIME; strncpy(c.payload, timeBuf, sizeof(c.payload)); q_push(c);
  c = DrawCmd{}; c.type = CMD_CLK_TEXT_DATE; strncpy(c.payload, dateBuf, sizeof(c.payload)); q_push(c);

  // Kick WiFi/NTP maintenance
  g_lastWiFiAttemptMs = millis() - WIFI_RETRY_INTERVAL_MS;
  g_lastNtpSyncMs     = millis() - NTP_SYNC_INTERVAL_MS;

  // Init WiFi indicator (single tile)
  uint16_t wifiCol = (WiFi.status() == WL_CONNECTED) ? C_GREEN : C_RED;
  c = DrawCmd{}; c.type = CMD_CLK_IND_WIFI; c.ivalue = wifiCol; q_push(c);

  // Start web
  setupWebHandlers();

  // Auto-cal init (Core0)
  g_ac.bootMs = millis();
  g_acTicker.attach_ms(g_ac.tickIntervalMs, acTickerISR); // sets tickFlag every 10s
}

void loop() {
  static uint32_t lastTick = 0;
  uint32_t now = millis();
  if (now - lastTick < 200) return; // 5 Hz pacing to keep things smooth
  lastTick = now;

  // Auto-cal: advance state machine on tick
  if (g_ac.tickFlag) { g_ac.tickFlag = false; autoCalibStep(); }

  // WiFi/NTP housekeeping
  wifiMaintainInLoop();
  ntpSyncMaintain();

  // Sensor update ~1 Hz
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
      float t_meas = bme.readTemperature();  // raw (can be warmer than ambient)
      float rh_meas = bme.readHumidity();
      p_hpa = (bme.readPressure() / 100.0f) + PRESS_OFFSET;

      t_amb   = estimateAmbientTemp(t_meas);
      g_lastTemp = t_amb;
      rh_corr = correctedRH(t_meas, rh_meas, t_amb);
      g_lastHumidity = rh_corr;
      g_lastHpa = p_hpa;
    }
    g_cpuTemp = analogReadTemp();

    // Push changed values to displays (Core1) only if text changed
    char hTxt[24]; snprintf(hTxt, sizeof(hTxt), "%.1f%%", rh_corr);
    if (strcmp(hTxt, lastHumTxt) != 0) {
      DrawCmd c{}; c.type = CMD_HUM_TEXT; strncpy(c.payload, hTxt, sizeof(c.payload)); q_push(c);
      strncpy(lastHumTxt, hTxt, sizeof(lastHumTxt));
    }
    char pTxt[24]; fmtPressureHpa(p_hpa, pTxt, sizeof(pTxt));
    if (strcmp(pTxt, lastHpaTxt) != 0) {
      DrawCmd c{}; c.type = CMD_HPA_TEXT; strncpy(c.payload, pTxt, sizeof(c.payload)); q_push(c);
      strncpy(lastHpaTxt, pTxt, sizeof(lastHpaTxt));
    }
    char tTxt[24]; snprintf(tTxt, sizeof(tTxt), "%.1fC", t_amb);
    if (strcmp(tTxt, lastTempTxt) != 0) {
      DrawCmd c{}; c.type = CMD_TEMP_TEXT; strncpy(c.payload, tTxt, sizeof(tTxt)); q_push(c);
      strncpy(lastTempTxt, tTxt, sizeof(lastTempTxt));
    }

    int cat = pressureCategory(p_hpa);
    if (cat != lastCat) {
      DrawCmd c{}; c.type = CMD_HPA_BG_CAT; c.ivalue = cat; q_push(c);
      lastCat = cat;
    }
  }

  // Update clock once per second (local time)
  datetime_t rt_utc; readRtc(&rt_utc);
  datetime_t rt; utcToLocalEU(rt_utc, &rt);
  static int lastSecShown = -1;
  if (rt.sec != lastSecShown) {
    lastSecShown = rt.sec;

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

    g_lastTime = String(dateBuf) + " " + String(timeBuf);

    DrawCmd c{};
    c = DrawCmd{}; c.type = CMD_CLK_TEXT_DOW;  strncpy(c.payload, dowTxt,  sizeof(c.payload)); q_push(c);
    c = DrawCmd{}; c.type = CMD_CLK_TEXT_TIME; strncpy(c.payload, timeBuf, sizeof(c.payload)); q_push(c);
    c = DrawCmd{}; c.type = CMD_CLK_TEXT_DATE; strncpy(c.payload, dateBuf, sizeof(c.payload)); q_push(c);

    // Service web server frequently; once per second is enough here
    server->handleClient();
    delay(1);
  }

#if DIAG_CYCLE_HPA
  // Optional: cycle month backgrounds for testing
  static uint32_t lastClkBgTick = 0; static int diagMonth = 1;
  if (millis() - lastClkBgTick > 8000) {
    lastClkBgTick = millis();
    DrawCmd c{}; c.type = CMD_CLK_BG_MONTH; c.ivalue = diagMonth; q_push(c);
    diagMonth += 1; if (diagMonth > 12) diagMonth = 1;
  }
#endif
}
