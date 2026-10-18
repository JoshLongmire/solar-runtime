/**
 * ESP32-S3-LCD-2 — BMP280/BME280 + battery HUD (large V, runtime est, sparkline, footer).
 *
 * Hardware (Waveshare ESP32-S3-LCD-2):
 *   - Sensors I2C: SDA=GPIO48, SCL=GPIO47 (same header as onboard IMU bus).
 *     BMP280 (chip 0x58) or BME280 (0x60) at 0x76 or 0x77. CS → 3.3V for I2C.
 *     BH1750 lux (ADDR→GND = 0x23, or ADDR→3.3V = 0x5C). Optional; OK if absent.
 *     CCS811 eCO2/TVOC (0x5A ADDR low, or 0x5B). WAK→GND. Optional; OK if absent.
 *   - GPIO15 → GND: click = slideshow; long-press = HUD. Wakes display.
 *   - GPIO18 → GND: click = cycle on-screen graphs (Vbat/temp/dV/net/eCO2/TVOC/lux).
 *   - QMI8658 IMU: backlight off after no motion (IDLE_OFF_S). A tap/grab or GPIO15/18 wakes;
 *     slow window/panel sway is ignored. WiFi stays up WIFI_SLEEP_AFTER_DISPLAY_OFF_S after
 *     the screen blanks, then radio off.
 *   - Battery: 1S cell on the Waveshare MX1.25 / JST battery header. GPIO5 is onboard
 *     BAT_ADC (200k/100k, not on pin headers). USB can stay plugged in for flashing;
 *     sense still reads the JST cell. Vbat = Vadc * 3. Logs to /LOG/env.csv on SD.
 *   - Solar: DFRobot Solar Power Manager-class board (~900 mA charge) + 5 V / 1 A panel.
 *     No INA — charge current is estimated from pack dV/dt and capped at the charger rating.
 *
 * WiFi: auto-joins Nomad Box; camcfg NVS SSID/pass override if set (camera sketch web UI).
 * Time: NTP after STA connects (12-hour AM/PM). Temperature: Fahrenheit (USE_FAHRENHEIT).
 * SD: FAT32 microSD (Waveshare recommends <=32 GB). Logs to /LOG/env.csv (append, 1 Hz)
 *     and /LOG/charge.csv when pack voltage is rising (sun/charge inferred, no INA).
 *     Rising pack voltage also holds WiFi on so the graphs page stays reachable.
 * Pictures: JPEG in /PICS on SD (GPIO15 slideshow). Web: live stats, log download, pic download.
 * OTA: solarmanager.local:3232 — USB: pio run -t upload ; Wi-Fi: pio run -e esp32s3_lcd2_ota -t upload
 *
 * Libraries: GFX, Adafruit BMP280, claws/BH1750, Adafruit CCS811, OneButton, JPEGDEC, WebServer, ESPmDNS, ArduinoOTA.
 * Install JPEGDEC once via Library Manager (do not also bundle under sketch/src/).
 */

#include "ESP32_S3_LCD2_Board.h"
#include "JpegSd.h"

#include <stdio.h>
#include <math.h>
#include <time.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_BME280.h>
#include <Adafruit_CCS811.h>
#include <BH1750.h>
#include <Arduino_GFX_Library.h>
#include <OneButton.h>
#include "FastIMU.h"

#ifndef BLACK
#define BLACK RGB565_BLACK
#define WHITE RGB565_WHITE
#define CYAN RGB565_CYAN
#define YELLOW RGB565_YELLOW
#define GREEN RGB565_GREEN
#define RED RGB565_RED
#define ORANGE RGB565_ORANGE
#define DARKGREY RGB565_DARKGREY
#define NAVY RGB565_NAVY
#endif

#define I2C_SDA 48
#define I2C_SCL 47

#define PIN_USER_BTN 15
#define PIN_GRAPH_BTN 18
#define IMU_ADDRESS 0x6B
#define IMU_WAKE_ACCEL_G 0.45f
#define IMU_WAKE_GYRO_DPS 90.0f
#define IMU_WAKE_HITS 3
#define IMU_GYRO_EMA_ALPHA 0.04f
#define IMU_ACCEL_EMA_ALPHA 0.03f

// Onboard BAT_ADC (GPIO5, not on headers): VBAT -- 200k -- GPIO5 -- 100k -- GND.
#define PIN_ADC_DIVIDER 5
#define DIVIDER_GAIN 3.0f
#define BAT_V_MIN 3.0f
#define BAT_V_MAX 4.2f
#define BAT_V_HUD_EMA_ALPHA 0.35f
#define BAT_CAPACITY_MAH 6000
#define BAT_AVG_LOAD_MA 160
// DFRobot Solar 1V1-class charger (~900 mA) fed by a 5 V / 1 A panel.
#define SOLAR_CHARGER_MAX_MA 900
#define SOLAR_PANEL_RATED_MA 1000
#define BAT_SPARK_N 80
#define BAT_GRAPH_V_MIN 3.40f
#define BAT_GRAPH_V_MAX 4.25f
#define TEMP_HIST_N 24
#define TEMP_HIST_MS (5u * 60u * 1000u)
#define CHARGE_SAMP_N 8
#define CHARGE_SAMP_MS (15u * 1000u)
#define CHARGE_ENTER_MV_MIN 1.5f
#define CHARGE_ENTER_MV_ABS 3.0f
#define CHARGE_FALL_MV_MIN (-2.5f)
#define CHARGE_EXIT_HITS 3
#define CHARGE_SAMPLE_LOG_MS (5u * 60u * 1000u)
#define DV_HIST_N 48
#define AQ_HIST_N 80
#define LUX_SUN_MIN 1200.0f
#define LUX_DARK_MAX 80.0f

#define EXAMPLE_PIN_NUM_LCD_SCLK 39
#define EXAMPLE_PIN_NUM_LCD_MOSI 38
#define EXAMPLE_PIN_NUM_LCD_MISO 40
#define EXAMPLE_PIN_NUM_LCD_DC 42
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS 45
#define EXAMPLE_PIN_NUM_LCD_BL 1

#define EXAMPLE_LCD_ROTATION 1
#define EXAMPLE_LCD_H_RES 240
#define EXAMPLE_LCD_V_RES 320

#define USE_FAHRENHEIT 1
#define UI_REFRESH_MS 1000u
#define IDLE_OFF_S 30u
#define WIFI_SLEEP_AFTER_DISPLAY_OFF_S 600u
#define BL_PCT 85u
#define LOG_INTERVAL_MS UI_REFRESH_MS
static const char *kLogPath = "/LOG/env.csv";
static const char *kChargePath = "/LOG/charge.csv";
static const char *kPicsDir = "/PICS";
#define MAX_PIC_FILES 32
#define PIC_NAME_MAX 40

#ifdef EXAMPLE_PIN_NUM_LCD_BL
#define BL_LEDC_FREQ 5000
#define BL_LEDC_BITS 10
#endif

#define FW_VERSION "1.7.2"
#define FW_BUILD (__DATE__ " " __TIME__)

static const char *kPrefsNs = "camcfg";
static const char *kPrefsBootNs = "envclk";
static const char *kTzDefault = "EST5EDT,M3.2.0/2,M11.1.0/2";
static const char *kStaSsidDefault = "Nomad Box";
static const char *kStaPassDefault = "";
static const char *kMdnsDefault = "solarmanager";
static const char *kOtaPassword = "solarmanager";
static const char *kSiteName = "Solar Manager";

Arduino_DataBus *bus = new Arduino_ESP32SPI(
    EXAMPLE_PIN_NUM_LCD_DC, EXAMPLE_PIN_NUM_LCD_CS,
    EXAMPLE_PIN_NUM_LCD_SCLK, EXAMPLE_PIN_NUM_LCD_MOSI, EXAMPLE_PIN_NUM_LCD_MISO);

Arduino_GFX *gfx = new Arduino_ST7789(
    bus, EXAMPLE_PIN_NUM_LCD_RST, EXAMPLE_LCD_ROTATION, true,
    EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);

Adafruit_BMP280 bmp(&Wire);
Adafruit_BME280 bme;
BH1750 bh1750;
Adafruit_CCS811 ccs;
QMI8658 IMU;
calData calib = {0};
GyroData gyroData;
AccelData accelData;
static float g_gyroEma = 0.0f;
static float g_accelEma = 1.0f;
static uint8_t g_imuWakeHits = 0;
OneButton userButton(PIN_USER_BTN, true);
OneButton graphButton(PIN_GRAPH_BTN, true);

enum GraphView {
  GRAPH_VBAT = 0,
  GRAPH_TEMP,
  GRAPH_DV,
  GRAPH_NET,
  GRAPH_ECO2,
  GRAPH_TVOC,
  GRAPH_LUX,
  GRAPH_COUNT
};
static GraphView g_graph = GRAPH_VBAT;
static bool g_force_hud = false;

static bool g_baro_ok = false;
static bool g_bme_humidity = false;  // true if BME280 (has RH); BMP280 has temp/pressure only
static float g_humidity_pct = 50.0f;
static bool g_lux_ok = false;
static float g_lux = 0.0f;
static bool g_ccs_ok = false;
static uint16_t g_eco2 = 0;
static uint16_t g_tvoc = 0;
static bool g_ccs_data_ok = false;
static bool g_imu_ok = false;
static bool g_sd_ok = false;
static bool g_ntp_configured = false;
static uint32_t g_last_log_ms = 0;
static bool g_wifi_has_ssid = false;
static bool g_wifi_sleeping = false;
static bool g_ota_busy = false;
static bool g_display_off = false;
static uint32_t g_display_off_since_ms = 0;
static uint32_t g_last_activity_ms = 0;
static uint32_t g_last_ui_ms = 0;
static uint32_t g_last_sd_retry_ms = 0;
static uint64_t g_sd_free_mb = 0;
static float g_spark[BAT_SPARK_N];
static uint8_t g_sparkHead = 0;
static uint8_t g_sparkCount = 0;
static float g_tempHist[TEMP_HIST_N];
static uint8_t g_tempHistHead = 0;
static uint8_t g_tempHistCount = 0;
static uint32_t g_lastTempHistMs = 0;
static float g_tempHistSum = 0.0f;
static uint16_t g_tempHistSamples = 0;
static float g_battVema = 0.0f;
static bool g_battVemaInit = false;
static float g_hud_vbat = 0.0f;
static int g_hud_pct = 0;
static float g_hud_tempF = 0.0f;
static bool g_hud_tempOk = false;
static float g_hud_hpa = 0.0f;
static bool g_hud_pressOk = false;
static uint32_t g_boot_count = 0;
static uint32_t g_poweroff_count = 0;
static uint32_t g_fault_count = 0;
static const char *g_reset_tag = "rst";
static bool g_fw_just_updated = false;
static bool g_update_was_ota = false;
static float g_chgSamp[CHARGE_SAMP_N];
static uint32_t g_chgSampMs[CHARGE_SAMP_N];
static uint8_t g_chgCount = 0;
static uint8_t g_chgHead = 0;
static uint32_t g_lastChgSampMs = 0;
static bool g_charging = false;
static uint8_t g_chargeExitHits = 0;
static float g_dv_mv_min = 0.0f;
static float g_dv_mv_abs = 0.0f;
static uint32_t g_charge_start_ms = 0;
static uint32_t g_charge_today_s = 0;
static int g_charge_today_yday = -1;
static uint32_t g_lastChargeLogMs = 0;
static const char *g_sun_label = "idle";
static float g_dvHist[DV_HIST_N];
static uint8_t g_dvHistHead = 0;
static uint8_t g_dvHistCount = 0;
static float g_netHist[DV_HIST_N];
static uint8_t g_netHistHead = 0;
static uint8_t g_netHistCount = 0;
static float g_eco2Hist[AQ_HIST_N];
static uint8_t g_eco2HistHead = 0;
static uint8_t g_eco2HistCount = 0;
static float g_tvocHist[AQ_HIST_N];
static uint8_t g_tvocHistHead = 0;
static uint8_t g_tvocHistCount = 0;
static float g_luxHist[AQ_HIST_N];
static uint8_t g_luxHistHead = 0;
static uint8_t g_luxHistCount = 0;

enum ViewMode { VIEW_HUD, VIEW_IMAGE };
static ViewMode g_view_mode = VIEW_HUD;
static bool g_sta_http_mdns_started = false;
static bool g_pic_list_dirty = true;
static int g_pic_count = 0;
static int g_pic_index = 0;
static char g_pic_names[MAX_PIC_FILES][PIC_NAME_MAX];
static SemaphoreHandle_t sdMutex = nullptr;
static WebServer server(80);
static uint8_t s_sdGetChunk[2048];

static void sdLock(void) {
  esp32_s3_lcd2_spi_bus_release(EXAMPLE_PIN_NUM_LCD_CS);
  if (sdMutex) {
    xSemaphoreTake(sdMutex, portMAX_DELAY);
  }
}

static void sdUnlock(void) {
  if (sdMutex) {
    xSemaphoreGive(sdMutex);
  }
}

static void applyBacklightPwm(void) {
#ifdef EXAMPLE_PIN_NUM_LCD_BL
  uint32_t pct = g_display_off ? 0u : BL_PCT;
  if (pct > 100) {
    pct = 100;
  }
  uint32_t maxd = (1u << BL_LEDC_BITS) - 1u;
  ledcWrite(EXAMPLE_PIN_NUM_LCD_BL, (maxd * pct) / 100u);
#endif
}

static void backlightHardwareInit(void) {
#ifdef EXAMPLE_PIN_NUM_LCD_BL
  ledcAttach(EXAMPLE_PIN_NUM_LCD_BL, BL_LEDC_FREQ, BL_LEDC_BITS);
  applyBacklightPwm();
#endif
}

static void noteUserActivity(void) {
  g_last_activity_ms = millis();
  if (g_display_off) {
    g_display_off = false;
    applyBacklightPwm();
  }
}

static void updateIdleBacklight(void) {
#ifdef EXAMPLE_PIN_NUM_LCD_BL
  if (IDLE_OFF_S == 0 || g_ota_busy) {
    return;
  }
  uint32_t now = millis();
  bool shouldOff = (uint32_t)(now - g_last_activity_ms) > (uint32_t)IDLE_OFF_S * 1000u;
  if (shouldOff != g_display_off) {
    g_display_off = shouldOff;
    applyBacklightPwm();
    if (g_display_off) {
      g_display_off_since_ms = now;
      Serial.println("Display off — WiFi oasis 10 min");
    }
  }
#endif
}

static void pollImuMotion(void) {
  if (!g_imu_ok) {
    return;
  }
  IMU.update();
  IMU.getGyro(&gyroData);
  IMU.getAccel(&accelData);

  float gmag = sqrtf(gyroData.gyroX * gyroData.gyroX + gyroData.gyroY * gyroData.gyroY +
                     gyroData.gyroZ * gyroData.gyroZ);
  if (g_gyroEma <= 0.01f) {
    g_gyroEma = gmag;
  } else {
    g_gyroEma += (gmag - g_gyroEma) * IMU_GYRO_EMA_ALPHA;
  }
  float gyroJerk = gmag - g_gyroEma;
  if (gyroJerk < 0.0f) {
    gyroJerk = 0.0f;
  }

  float amag = sqrtf(accelData.accelX * accelData.accelX + accelData.accelY * accelData.accelY +
                     accelData.accelZ * accelData.accelZ);
  g_accelEma += (amag - g_accelEma) * IMU_ACCEL_EMA_ALPHA;
  float accelJerk = fabsf(amag - g_accelEma);

  bool bump = (accelJerk >= IMU_WAKE_ACCEL_G) || (gyroJerk >= IMU_WAKE_GYRO_DPS);
  if (bump) {
    if (g_imuWakeHits < 255) {
      g_imuWakeHits++;
    }
  } else {
    g_imuWakeHits = 0;
  }
  if (g_imuWakeHits >= IMU_WAKE_HITS) {
    g_imuWakeHits = 0;
    noteUserActivity();
  }
}

// 12-hour clock with AM/PM, e.g. "2:45:30 PM"
static void formatTime12h(const struct tm *tm, char *buf, size_t buflen) {
  if (!tm || !buf || buflen == 0) {
    return;
  }
  int h = tm->tm_hour;
  const char *ampm = "AM";
  if (h >= 12) {
    ampm = "PM";
    if (h > 12) {
      h -= 12;
    }
  } else if (h == 0) {
    h = 12;
  }
  snprintf(buf, buflen, "%d:%02d:%02d %s", h, tm->tm_min, tm->tm_sec, ampm);
}

static void formatTime12hShort(const struct tm *tm, char *buf, size_t buflen) {
  if (!tm || !buf || buflen == 0) {
    return;
  }
  int h = tm->tm_hour;
  const char *ampm = "AM";
  if (h >= 12) {
    ampm = "PM";
    if (h > 12) {
      h -= 12;
    }
  } else if (h == 0) {
    h = 12;
  }
  snprintf(buf, buflen, "%d:%02d %s", h, tm->tm_min, ampm);
}

static float updateBattEma(float vSample) {
  if (!g_battVemaInit) {
    g_battVema = vSample;
    g_battVemaInit = true;
  } else {
    g_battVema += (vSample - g_battVema) * BAT_V_HUD_EMA_ALPHA;
  }
  return g_battVema;
}

static void storeHudStats(float tempF, bool tempOk, float pressureHpa, bool pressOk, float vbat, int pct) {
  g_hud_tempF = tempF;
  g_hud_tempOk = tempOk;
  g_hud_hpa = pressureHpa;
  g_hud_pressOk = pressOk;
  g_hud_vbat = vbat;
  g_hud_pct = pct;
}

static bool getLocalTimeValid(struct tm *tmOut) {
  if (!tmOut) {
    return false;
  }
  time_t now = time(nullptr);
  if (now < 1700000000) {
    return false;
  }
  localtime_r(&now, tmOut);
  return true;
}

static void maybeStartNtpSync(void) {
  if (g_ntp_configured) {
    return;
  }
  if (WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) {
    return;
  }
  configTzTime(kTzDefault, "pool.ntp.org", "time.nist.gov");
  g_ntp_configured = true;
  Serial.println("NTP: config started");
}

static String sanitizeMdnsHost(const String &in) {
  String o;
  for (size_t i = 0; i < in.length() && o.length() < 24; i++) {
    char c = in[i];
    if (c >= 'A' && c <= 'Z') {
      c = (char)(c - 'A' + 'a');
    }
    if ((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-') {
      o += c;
    }
  }
  while (o.length() > 0 && o[0] == '-') {
    o.remove(0, 1);
  }
  if (o.length() == 0) {
    o = kMdnsDefault;
  }
  return o;
}

static bool readSdFreeMbUnlocked(uint64_t *freeMbOut) {
  if (!freeMbOut) {
    return false;
  }
  uint64_t total = SD.totalBytes();
  uint64_t used = SD.usedBytes();
  if (total == 0 || total <= used) {
    return false;
  }
  *freeMbOut = (total - used) / (1024ull * 1024ull);
  return true;
}

static bool tryMountSd(void) {
  SD.end();
  delay(30);
  if (!esp32_s3_lcd2_sd_begin_tries(EXAMPLE_PIN_NUM_LCD_CS)) {
    return false;
  }
  sdLock();
  ensureSdDataDirs();
  sdUnlock();
  g_pic_list_dirty = true;
  uint64_t freeMb = 0;
  sdLock();
  if (readSdFreeMbUnlocked(&freeMb)) {
    g_sd_free_mb = freeMb;
  }
  sdUnlock();
  return true;
}

static void staWifiBeginFromPrefs(void) {
  g_sta_http_mdns_started = false;
  ArduinoOTA.end();
  server.stop();
  MDNS.end();

  String ssid = kStaSsidDefault;
  String pass = kStaPassDefault;
  String host = kMdnsDefault;
  Preferences q;
  if (q.begin(kPrefsNs, true)) {
    String nvsSsid = q.getString("sta_ssid", "");
    String nvsPass = q.getString("sta_pass", "");
    q.end();
    if (nvsSsid.length() > 0) {
      ssid = nvsSsid;
      pass = nvsPass;
    }
  }

  g_wifi_has_ssid = true;
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(host.c_str());
  WiFi.begin(ssid.c_str(), pass.c_str());
  Serial.printf("STA WiFi: joining '%s' (http://%s.local/ when up)\n", ssid.c_str(), host.c_str());
}

static void wifiRadioSleep(void) {
  if (g_wifi_sleeping) {
    return;
  }
  g_sta_http_mdns_started = false;
  ArduinoOTA.end();
  server.stop();
  MDNS.end();
  WiFi.disconnect(true);
  WiFi.mode(WIFI_OFF);
  g_ntp_configured = false;
  g_wifi_sleeping = true;
  Serial.println("WiFi: radio off (save power)");
}

static void wifiRadioWake(void) {
  if (!g_wifi_sleeping) {
    return;
  }
  g_wifi_sleeping = false;
  staWifiBeginFromPrefs();
}

static void updateWifiPowerSave(void) {
  if (!g_wifi_has_ssid || g_ota_busy) {
    return;
  }
  if (!g_display_off || g_charging) {
    if (g_charging) {
      g_display_off_since_ms = millis();
    }
    wifiRadioWake();
    return;
  }
  if (g_wifi_sleeping || WIFI_SLEEP_AFTER_DISPLAY_OFF_S == 0) {
    return;
  }
  if ((uint32_t)(millis() - g_display_off_since_ms) >= WIFI_SLEEP_AFTER_DISPLAY_OFF_S * 1000u) {
    wifiRadioSleep();
  }
}

/** Create /LOG and /PICS on SD if missing (caller may hold sdMutex). */
static bool ensureSdDataDirs(void) {
  if (!g_sd_ok) {
    return false;
  }
  bool ok = true;
  if (!SD.exists("/LOG")) {
    ok = SD.mkdir("/LOG") && ok;
  }
  if (!SD.exists(kPicsDir)) {
    ok = SD.mkdir(kPicsDir) && ok;
  }
  return ok;
}

static float readDividerVoltageV(void) {
  uint32_t sum_mv = 0;
  const int n = 16;
  for (int i = 0; i < n; i++) {
    sum_mv += analogReadMilliVolts(PIN_ADC_DIVIDER);
  }
  float v_adc = (sum_mv / (float)n) / 1000.0f;
  return v_adc * DIVIDER_GAIN;
}

static int batteryPctFromV(float v) {
  static const float kV[] = {3.00f, 3.30f, 3.50f, 3.60f, 3.70f, 3.80f, 3.90f, 4.00f, 4.10f, 4.20f};
  static const float kP[] = {0.0f, 2.0f, 9.0f, 22.0f, 40.0f, 58.0f, 75.0f, 88.0f, 96.0f, 100.0f};
  const int n = (int)(sizeof(kV) / sizeof(kV[0]));
  if (v <= kV[0]) {
    return 0;
  }
  if (v >= kV[n - 1]) {
    return 100;
  }
  for (int i = 1; i < n; i++) {
    if (v <= kV[i]) {
      float t = (v - kV[i - 1]) / (kV[i] - kV[i - 1]);
      return (int)(kP[i - 1] + t * (kP[i] - kP[i - 1]) + 0.5f);
    }
  }
  return 100;
}

static void sparkPush(float v) {
  g_spark[g_sparkHead] = v;
  g_sparkHead = (uint8_t)((g_sparkHead + 1u) % BAT_SPARK_N);
  if (g_sparkCount < BAT_SPARK_N) {
    g_sparkCount++;
  }
}

static float sparkSample(int i) {
  if (g_sparkCount < BAT_SPARK_N) {
    return g_spark[i];
  }
  return g_spark[(g_sparkHead + i) % BAT_SPARK_N];
}

static void tempHistPush(float t) {
  g_tempHist[g_tempHistHead] = t;
  g_tempHistHead = (uint8_t)((g_tempHistHead + 1u) % TEMP_HIST_N);
  if (g_tempHistCount < TEMP_HIST_N) {
    g_tempHistCount++;
  }
}

static float tempHistSample(int i) {
  if (g_tempHistCount < TEMP_HIST_N) {
    return g_tempHist[i];
  }
  return g_tempHist[(g_tempHistHead + i) % TEMP_HIST_N];
}

static void dvHistPush(float dv) {
  g_dvHist[g_dvHistHead] = dv;
  g_dvHistHead = (uint8_t)((g_dvHistHead + 1u) % DV_HIST_N);
  if (g_dvHistCount < DV_HIST_N) {
    g_dvHistCount++;
  }
  const float spanMv = (BAT_V_MAX - BAT_V_MIN) * 1000.0f;
  const float mahPerMv = (spanMv > 1.0f) ? ((float)BAT_CAPACITY_MAH / spanMv) : 5.0f;
  const float netMa = dv * mahPerMv * 60.0f;
  g_netHist[g_netHistHead] = netMa;
  g_netHistHead = (uint8_t)((g_netHistHead + 1u) % DV_HIST_N);
  if (g_netHistCount < DV_HIST_N) {
    g_netHistCount++;
  }
}

static float dvHistSample(int i) {
  if (g_dvHistCount < DV_HIST_N) {
    return g_dvHist[i];
  }
  return g_dvHist[(g_dvHistHead + i) % DV_HIST_N];
}

static float netHistSample(int i) {
  if (g_netHistCount < DV_HIST_N) {
    return g_netHist[i];
  }
  return g_netHist[(g_netHistHead + i) % DV_HIST_N];
}

static void aqHistPush(float *buf, uint8_t *head, uint8_t *count, uint8_t nMax, float v) {
  buf[*head] = v;
  *head = (uint8_t)((*head + 1u) % nMax);
  if (*count < nMax) {
    (*count)++;
  }
}

static float aqHistSample(const float *buf, uint8_t head, uint8_t count, uint8_t nMax, int i) {
  if (count < nMax) {
    return buf[i];
  }
  return buf[(head + i) % nMax];
}

static float eco2HistSample(int i) {
  return aqHistSample(g_eco2Hist, g_eco2HistHead, g_eco2HistCount, AQ_HIST_N, i);
}

static float tvocHistSample(int i) {
  return aqHistSample(g_tvocHist, g_tvocHistHead, g_tvocHistCount, AQ_HIST_N, i);
}

static float luxHistSample(int i) {
  return aqHistSample(g_luxHist, g_luxHistHead, g_luxHistCount, AQ_HIST_N, i);
}

static void histRange(int count, float (*sampleAt)(int), float pad, float minSpan, float *ymin, float *ymax) {
  float lo = 0.0f;
  float hi = minSpan;
  if (count > 0) {
    lo = sampleAt(0);
    hi = lo;
    for (int i = 1; i < count; i++) {
      float v = sampleAt(i);
      if (v < lo) {
        lo = v;
      }
      if (v > hi) {
        hi = v;
      }
    }
    lo -= pad;
    hi += pad;
  }
  if (hi - lo < minSpan) {
    float mid = (hi + lo) * 0.5f;
    lo = mid - minSpan * 0.5f;
    hi = mid + minSpan * 0.5f;
  }
  *ymin = lo;
  *ymax = hi;
}

static bool graphViewAvailable(GraphView v) {
  switch (v) {
    case GRAPH_ECO2:
    case GRAPH_TVOC:
      return g_ccs_ok;
    case GRAPH_LUX:
      return g_lux_ok;
    default:
      return true;
  }
}

static void cycleGraphView(void) {
  for (int n = 0; n < (int)GRAPH_COUNT; n++) {
    g_graph = (GraphView)(((int)g_graph + 1) % (int)GRAPH_COUNT);
    if (graphViewAvailable(g_graph)) {
      return;
    }
  }
  g_graph = GRAPH_VBAT;
}

static const char *graphViewLabel(GraphView v) {
  switch (v) {
    case GRAPH_TEMP:
      return "Temp5m";
    case GRAPH_DV:
      return "dV mV/m";
    case GRAPH_NET:
      return "Net mA";
    case GRAPH_ECO2:
      return "eCO2";
    case GRAPH_TVOC:
      return "TVOC";
    case GRAPH_LUX:
      return "Lux";
    case GRAPH_VBAT:
    default:
      return "Vbat80s";
  }
}

static void tempHistMaybePush(float tDisp, bool ok) {
  if (!ok) {
    return;
  }
  uint32_t now = millis();
  if (g_tempHistCount == 0) {
    tempHistPush(tDisp);
    g_lastTempHistMs = now;
    g_tempHistSum = 0.0f;
    g_tempHistSamples = 0;
    return;
  }
  g_tempHistSum += tDisp;
  g_tempHistSamples++;
  if ((uint32_t)(now - g_lastTempHistMs) < TEMP_HIST_MS) {
    return;
  }
  float avg = (g_tempHistSamples > 0) ? (g_tempHistSum / (float)g_tempHistSamples) : tDisp;
  tempHistPush(avg);
  g_tempHistSum = 0.0f;
  g_tempHistSamples = 0;
  g_lastTempHistMs = now;
}

static void drawSparkline(int gx, int gy, int gw, int gh, int count, float (*sampleAt)(int), float ymin,
                          float ymax, uint16_t color) {
  gfx->drawRect(gx, gy, gw, gh, DARKGREY);
  if (count < 2 || ymax <= ymin) {
    return;
  }
  int prevx = gx + 1;
  int prevy = gy + gh - 2;
  for (int i = 0; i < count; i++) {
    float v = sampleAt(i);
    if (v < ymin) {
      v = ymin;
    }
    if (v > ymax) {
      v = ymax;
    }
    int x = gx + 1 + (int)((float)i * (float)(gw - 3) / (float)(count - 1));
    int y = gy + gh - 2 - (int)((v - ymin) / (ymax - ymin) * (float)(gh - 3));
    if (i > 0) {
      gfx->drawLine(prevx, prevy, x, y, color);
    }
    prevx = x;
    prevy = y;
  }
}

static void formatRuntime(char *out, size_t len, int pct) {
  if (!out || len == 0) {
    return;
  }
  float mah = (float)BAT_CAPACITY_MAH * ((float)pct / 100.0f);
  float hours = mah / (float)BAT_AVG_LOAD_MA;
  if (hours < 0.0f) {
    hours = 0.0f;
  }
  int h = (int)hours;
  int m = (int)((hours - (float)h) * 60.0f + 0.5f);
  if (m >= 60) {
    h++;
    m = 0;
  }
  snprintf(out, len, "est %dh %02dm", h, m);
}

static void formatUptime(char *out, size_t len) {
  if (!out || len == 0) {
    return;
  }
  uint32_t sec = millis() / 1000u;
  uint32_t days = sec / 86400u;
  sec %= 86400u;
  uint32_t hours = sec / 3600u;
  sec %= 3600u;
  uint32_t mins = sec / 60u;
  if (days > 0) {
    snprintf(out, len, "up %lud %luh", (unsigned long)days, (unsigned long)hours);
  } else if (hours > 0) {
    snprintf(out, len, "up %luh %02lum", (unsigned long)hours, (unsigned long)mins);
  } else {
    snprintf(out, len, "up %lum", (unsigned long)mins);
  }
}

static const char *resetReasonTag(esp_reset_reason_t rr) {
  switch (rr) {
    case ESP_RST_POWERON:
      return "power";
    case ESP_RST_SW:
      return "reboot";
    case ESP_RST_PANIC:
      return "crash";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:
      return "wdt";
    case ESP_RST_BROWNOUT:
      return "brownout";
    case ESP_RST_USB:
      return "usb";
    default:
      return "other";
  }
}

static void recordBootStats(void) {
  const esp_reset_reason_t rr = esp_reset_reason();
  g_reset_tag = resetReasonTag(rr);

  Preferences p;
  if (!p.begin(kPrefsBootNs, false)) {
    return;
  }
  g_boot_count = p.getUInt("boots", 0) + 1u;
  p.putUInt("boots", g_boot_count);

  if (rr == ESP_RST_POWERON) {
    g_poweroff_count = p.getUInt("pwr", 0) + 1u;
    p.putUInt("pwr", g_poweroff_count);
  } else {
    g_poweroff_count = p.getUInt("pwr", 0);
  }

  if (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT ||
      rr == ESP_RST_BROWNOUT) {
    g_fault_count = p.getUInt("faults", 0) + 1u;
    p.putUInt("faults", g_fault_count);
  } else {
    g_fault_count = p.getUInt("faults", 0);
  }

  const bool otaPend = p.getBool("ota_pend", false);
  const String lastFw = p.getString("fw", "");
  if (lastFw != FW_VERSION) {
    g_fw_just_updated = true;
    g_update_was_ota = otaPend || (rr == ESP_RST_SW);
    p.putString("fw", FW_VERSION);
  }
  if (otaPend) {
    g_fw_just_updated = true;
    g_update_was_ota = true;
    p.putBool("ota_pend", false);
  }
  p.end();
}

static void markOtaPending(bool pending) {
  Preferences p;
  if (!p.begin(kPrefsBootNs, false)) {
    return;
  }
  p.putBool("ota_pend", pending);
  p.end();
}

/** Rough pack current from dV/dt (no INA). Linear over BAT_V_MIN..BAT_V_MAX. */
static void estimatePowerFlow(float *netMaOut, float *solarMaOut, float *loadMaOut, float *mahPerMinOut) {
  const float spanMv = (BAT_V_MAX - BAT_V_MIN) * 1000.0f;
  const float mahPerMv = (spanMv > 1.0f) ? ((float)BAT_CAPACITY_MAH / spanMv) : 5.0f;
  const float mahPerMin = g_dv_mv_min * mahPerMv;
  float netMa = mahPerMin * 60.0f;
  float loadMa = (float)BAT_AVG_LOAD_MA;
  float solarMa = 0.0f;
  if (g_charging) {
    // Rising: solar covers load + pack gain. Flat CV hold: solar ≈ load (dV~0).
    solarMa = netMa + loadMa;
    if (solarMa < loadMa * 0.5f) {
      solarMa = loadMa;
    }
    // Hard ceiling: DFRobot ~900 mA charger (panel is 5 V / 1 A upstream of the regulator).
    if (solarMa > (float)SOLAR_CHARGER_MAX_MA) {
      solarMa = (float)SOLAR_CHARGER_MAX_MA;
    }
    if (netMa > (float)SOLAR_CHARGER_MAX_MA) {
      netMa = (float)SOLAR_CHARGER_MAX_MA;
    }
  } else if (netMa < -5.0f) {
    loadMa = -netMa;
  }
  if (netMaOut) {
    *netMaOut = netMa;
  }
  if (solarMaOut) {
    *solarMaOut = solarMa;
  }
  if (loadMaOut) {
    *loadMaOut = loadMa;
  }
  if (mahPerMinOut) {
    *mahPerMinOut = mahPerMin;
  }
}

static void refreshSunLabel(void) {
  const bool bright = g_lux_ok && g_lux >= LUX_SUN_MIN;
  if (g_charging) {
    if (g_dv_mv_min >= CHARGE_ENTER_MV_MIN || g_dv_mv_abs >= CHARGE_ENTER_MV_ABS) {
      g_sun_label = "rising";
    } else if (g_dv_mv_min <= CHARGE_FALL_MV_MIN) {
      g_sun_label = "falling";
    } else if (bright) {
      g_sun_label = "bright";
    } else {
      g_sun_label = "holding";
    }
  } else if (g_dv_mv_min <= CHARGE_FALL_MV_MIN) {
    g_sun_label = "draining";
  } else {
    g_sun_label = "idle";
  }
}

static uint16_t battHudColor(int pct) {
  if (pct >= 40) {
    return GREEN;
  }
  if (pct >= 15) {
    return YELLOW;
  }
  return RED;
}

static void drawWifiIcon(int x, int y, bool connected) {
  const uint16_t color = connected ? GREEN : RED;
  gfx->fillRect(x + 0, y + 6, 3, 4, color);
  gfx->fillRect(x + 5, y + 3, 3, 7, color);
  gfx->fillRect(x + 10, y + 0, 3, 10, color);
  if (!connected) {
    gfx->drawLine(x, y + 11, x + 13, y - 1, color);
    gfx->drawLine(x, y + 12, x + 13, y, color);
  }
}

static void appendEnvLog(float tempF, bool tempOk, float pressureHpa, bool pressOk, const struct tm *tmValid,
                         bool timeOk, float vbat) {
  if (!g_sd_ok) {
    return;
  }
  sdLock();
  if (!ensureSdDataDirs()) {
    sdUnlock();
    return;
  }
  const bool newFile = !SD.exists(kLogPath);
  File f = SD.open(kLogPath, FILE_APPEND);
  if (!f) {
    sdUnlock();
    Serial.println("SD log open failed");
    return;
  }
  if (newFile) {
    f.println("unix,time12,temp_f,pressure_hpa,humidity_pct,vbat,lux,eco2_ppm,tvoc_ppb");
  }

  char line[220];
  const float luxOut = g_lux_ok ? g_lux : -999.0f;
  const int eco2Out = g_ccs_data_ok ? (int)g_eco2 : -999;
  const int tvocOut = g_ccs_data_ok ? (int)g_tvoc : -999;
  const float rhOut = g_bme_humidity ? g_humidity_pct : -999.0f;
  if (timeOk && tmValid) {
    char t12[20];
    formatTime12h(tmValid, t12, sizeof(t12));
    struct tm tcopy = *tmValid;
    time_t unixTs = mktime(&tcopy);
    snprintf(line, sizeof(line), "%ld,%s,%.1f,%.1f,%.1f,%.3f,%.1f,%d,%d\n", (long)unixTs, t12,
             tempOk ? tempF : -999.0f, pressOk ? pressureHpa : -999.0f, rhOut, vbat, luxOut, eco2Out,
             tvocOut);
  } else {
    snprintf(line, sizeof(line), ",,%.1f,%.1f,%.1f,%.3f,%.1f,%d,%d\n", tempOk ? tempF : -999.0f,
             pressOk ? pressureHpa : -999.0f, rhOut, vbat, luxOut, eco2Out, tvocOut);
  }
  f.print(line);
  f.close();
  sdUnlock();
}

static void appendChargeLog(const char *event, float vbat) {
  if (!g_sd_ok || !event) {
    return;
  }
  sdLock();
  if (!ensureSdDataDirs()) {
    sdUnlock();
    return;
  }
  const bool newFile = !SD.exists(kChargePath);
  File f = SD.open(kChargePath, FILE_APPEND);
  if (!f) {
    sdUnlock();
    return;
  }
  if (newFile) {
    f.println("unix,time12,event,vbat,dv_mv_min,sun_min_today,lux");
  }
  char line[180];
  char t12[20] = "";
  long unixTs = 0;
  struct tm t;
  if (getLocalTimeValid(&t)) {
    formatTime12h(&t, t12, sizeof(t12));
    struct tm tcopy = t;
    unixTs = (long)mktime(&tcopy);
  }
  const float luxOut = g_lux_ok ? g_lux : -999.0f;
  snprintf(line, sizeof(line), "%ld,%s,%s,%.3f,%.1f,%lu,%.1f\n", unixTs, t12, event, vbat, g_dv_mv_min,
           (unsigned long)(g_charge_today_s / 60u), luxOut);
  f.print(line);
  f.close();
  sdUnlock();
}

static void maybeResetChargeToday(void) {
  struct tm t;
  if (!getLocalTimeValid(&t)) {
    return;
  }
  if (g_charge_today_yday != t.tm_yday) {
    g_charge_today_yday = t.tm_yday;
    g_charge_today_s = 0;
  }
}

static void updateChargeDetect(float vbat) {
  const uint32_t now = millis();
  maybeResetChargeToday();

  static uint32_t lastAccMs = 0;
  if (lastAccMs == 0) {
    lastAccMs = now;
  }
  uint32_t dtAcc = (uint32_t)(now - lastAccMs);
  lastAccMs = now;
  if (g_charging) {
    g_charge_today_s += dtAcc / 1000u;
  }

  bool tookSample = false;
  if (g_chgCount == 0 || (uint32_t)(now - g_lastChgSampMs) >= CHARGE_SAMP_MS) {
    g_chgSamp[g_chgHead] = vbat;
    g_chgSampMs[g_chgHead] = now;
    g_chgHead = (uint8_t)((g_chgHead + 1u) % CHARGE_SAMP_N);
    if (g_chgCount < CHARGE_SAMP_N) {
      g_chgCount++;
    }
    g_lastChgSampMs = now;
    tookSample = true;
  }

  if (g_chgCount >= 2) {
    uint8_t oldest = (g_chgCount < CHARGE_SAMP_N) ? 0 : g_chgHead;
    uint8_t newest = (uint8_t)((g_chgHead + CHARGE_SAMP_N - 1u) % CHARGE_SAMP_N);
    float dtMin = (float)(g_chgSampMs[newest] - g_chgSampMs[oldest]) / 60000.0f;
    if (dtMin < 0.2f) {
      dtMin = 0.2f;
    }
    g_dv_mv_abs = (g_chgSamp[newest] - g_chgSamp[oldest]) * 1000.0f;
    g_dv_mv_min = g_dv_mv_abs / dtMin;
    if (tookSample) {
      dvHistPush(g_dv_mv_min);
    }
  }

  // Enter only on rising pack V (daytime shade can still be >1200 lux with little panel current).
  // Once charging, bright lux + flat V (CV hold) stays latched until fall or dark.
  const bool rising =
      (g_chgCount >= 3) && (g_dv_mv_min >= CHARGE_ENTER_MV_MIN || g_dv_mv_abs >= CHARGE_ENTER_MV_ABS);
  const bool falling = (g_chgCount >= 3) && (g_dv_mv_min <= CHARGE_FALL_MV_MIN);
  const bool dark = g_lux_ok && g_lux <= LUX_DARK_MAX;

  bool next = g_charging;
  if (g_charging) {
    const bool shouldExit = falling || (dark && !rising);
    if (shouldExit) {
      g_chargeExitHits++;
      if (g_chargeExitHits >= CHARGE_EXIT_HITS) {
        next = false;
      }
    } else {
      g_chargeExitHits = 0;
      next = true;
    }
  } else if (rising) {
    g_chargeExitHits = 0;
    next = true;
  }

  if (next != g_charging) {
    g_charging = next;
    if (g_charging) {
      g_charge_start_ms = now;
      g_lastChargeLogMs = now;
      appendChargeLog("start", vbat);
      Serial.printf("SUN: start  V=%.3f  dV=%+.1f mV/min (%+.1f mV)\n", vbat, g_dv_mv_min, g_dv_mv_abs);
    } else {
      appendChargeLog("stop", vbat);
      Serial.printf("SUN: stop  V=%.3f  dV=%+.1f mV/min  today=%lum\n", vbat, g_dv_mv_min,
                    (unsigned long)(g_charge_today_s / 60u));
    }
  } else if (g_charging && (uint32_t)(now - g_lastChargeLogMs) >= CHARGE_SAMPLE_LOG_MS) {
    g_lastChargeLogMs = now;
    appendChargeLog("sample", vbat);
  }
  refreshSunLabel();
}

static void i2cBusBegin(void) {
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(50);
}

static void i2cScanBus(void) {
  Serial.println("I2C scan (SDA=48 SCL=47):");
  int found = 0;
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("  0x%02X", addr);
      if (addr == 0x6B) {
        Serial.print(" (onboard QMI8658 IMU)");
      }
      if (addr == 0x76 || addr == 0x77) {
        uint8_t chipId = 0;
        Wire.beginTransmission(addr);
        Wire.write(0xD0);
        if (Wire.endTransmission() == 0 && Wire.requestFrom(addr, (uint8_t)1) == 1) {
          chipId = Wire.read();
        }
        if (chipId == 0x58) {
          Serial.print(" BMP280");
        } else if (chipId == 0x60) {
          Serial.print(" BME280 (use BME library)");
        } else {
          Serial.printf(" chipID=0x%02X", chipId);
        }
      }
      if (addr == 0x23 || addr == 0x5C) {
        Serial.print(" (BH1750?)");
      }
      if (addr == 0x5A || addr == 0x5B) {
        Serial.print(" (CCS811?)");
      }
      Serial.println();
      found++;
    }
  }
  if (found == 0) {
    Serial.println("  (no devices — check SDA/SCL wiring and 3.3V)");
  }
}

static bool initBmp280(void) {
  i2cBusBegin();
  i2cScanBus();

  const uint8_t addrs[] = {0x76, 0x77};
  const uint32_t clocks[] = {100000, 400000};
  g_bme_humidity = false;
  g_humidity_pct = 50.0f;

  // Prefer BME280 when present — RH feeds CCS811 eCO2/TVOC compensation.
  for (uint32_t clk : clocks) {
    Wire.setClock(clk);
    for (uint8_t addr : addrs) {
      if (bme.begin(addr, &Wire)) {
        bme.setSampling(Adafruit_BME280::MODE_FORCED, Adafruit_BME280::SAMPLING_X2,
                        Adafruit_BME280::SAMPLING_X16, Adafruit_BME280::SAMPLING_X1,
                        Adafruit_BME280::FILTER_OFF, Adafruit_BME280::STANDBY_MS_500);
        bme.takeForcedMeasurement();
        delay(12);
        float t = bme.readTemperature();
        float pPa = bme.readPressure();
        g_humidity_pct = bme.readHumidity();
        g_bme_humidity = true;
        Serial.printf("BME280 OK 0x%02X @ %lu Hz  T=%.2f C  P=%.1f hPa  RH=%.1f%%\n", addr,
                      (unsigned long)clk, t, pPa / 100.0f, g_humidity_pct);
        Wire.setClock(400000);
        return true;
      }
    }
  }

  for (uint32_t clk : clocks) {
    Wire.setClock(clk);
    for (uint8_t addr : addrs) {
      if (bmp.begin(addr)) {
        bmp.setSampling(Adafruit_BMP280::MODE_FORCED, Adafruit_BMP280::SAMPLING_X2,
                        Adafruit_BMP280::SAMPLING_X16, Adafruit_BMP280::FILTER_OFF,
                        Adafruit_BMP280::STANDBY_MS_500);
        bmp.takeForcedMeasurement();
        delay(12);
        float t = bmp.readTemperature();
        float pPa = bmp.readPressure();
        Serial.printf("BMP280 OK 0x%02X @ %lu Hz  T=%.2f C  P=%.1f hPa (RH assumed 50%% for CCS811)\n",
                      addr, (unsigned long)clk, t, pPa / 100.0f);
        Wire.setClock(400000);
        return true;
      }
    }
  }
  Serial.println("BMP/BME280 init failed — see scan (expect 0x76/0x77)");
  return false;
}

static bool probeBh1750Addr(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static bool initBh1750(void) {
  // ADDR pin: GND → 0x23, 3V3 → 0x5C. Safe no-op if the sensor is not soldered yet.
  Wire.setClock(100000);
  const uint8_t addrs[] = {0x23, 0x5C};
  for (uint8_t addr : addrs) {
    if (!probeBh1750Addr(addr)) {
      continue;
    }
    if (bh1750.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, addr)) {
      delay(180);
      float lux = bh1750.readLightLevel();
      if (!isnan(lux) && lux >= 0.0f) {
        Serial.printf("BH1750 OK @0x%02X  lux=%.1f\n", addr, lux);
        Wire.setClock(400000);
        return true;
      }
    }
  }
  Serial.println("BH1750 not found (optional — solder later on SDA48/SCL47)");
  Wire.setClock(400000);
  return false;
}

static void updateLuxReading(void) {
  if (!g_lux_ok) {
    return;
  }
  float lux = bh1750.readLightLevel();
  if (lux < 0.0f || isnan(lux)) {
    return;
  }
  g_lux = lux;
  aqHistPush(g_luxHist, &g_luxHistHead, &g_luxHistCount, AQ_HIST_N, g_lux);
}

static bool initCcs811(void) {
  // ADDR low → 0x5A, high → 0x5B. Many modules need WAK tied to GND.
  Wire.setClock(100000);
  const uint8_t addrs[] = {0x5A, 0x5B};
  for (uint8_t addr : addrs) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() != 0) {
      continue;
    }
    if (ccs.begin(addr)) {
      delay(50);
      Serial.printf("CCS811 OK @0x%02X (eCO2/TVOC)\n", addr);
      Wire.setClock(400000);
      return true;
    }
  }
  Serial.println("CCS811 not found (optional — solder later on SDA48/SCL47, WAK→GND)");
  Wire.setClock(400000);
  return false;
}

static void updateCcs811Reading(float tempC, bool tempOk) {
  if (!g_ccs_ok) {
    return;
  }
  // CCS811 env compensation: temperature from BMP/BME; humidity from BME when present,
  // otherwise 50% RH (sensor datasheet default — BMP280 has no humidity).
  if (tempOk) {
    float rh = g_humidity_pct;
    if (rh < 0.0f) {
      rh = 0.0f;
    }
    if (rh > 100.0f) {
      rh = 100.0f;
    }
    ccs.setEnvironmentalData(rh, tempC);
  }
  if (!ccs.available()) {
    return;
  }
  if (ccs.readData() != 0) {
    return;
  }
  g_eco2 = ccs.geteCO2();
  g_tvoc = ccs.getTVOC();
  g_ccs_data_ok = (g_eco2 > 0);
  if (g_ccs_data_ok) {
    aqHistPush(g_eco2Hist, &g_eco2HistHead, &g_eco2HistCount, AQ_HIST_N, (float)g_eco2);
    aqHistPush(g_tvocHist, &g_tvocHistHead, &g_tvocHistCount, AQ_HIST_N, (float)g_tvoc);
  }
}

static bool readBmp280(float *tempC, float *pressureHpa) {
  if (!g_baro_ok || !tempC) {
    return false;
  }
  if (g_bme_humidity) {
    bme.takeForcedMeasurement();
    delay(12);
    *tempC = bme.readTemperature();
    if (pressureHpa) {
      *pressureHpa = bme.readPressure() / 100.0f;
    }
    float rh = bme.readHumidity();
    if (!isnan(rh) && rh >= 0.0f && rh <= 100.0f) {
      g_humidity_pct = rh;
    }
  } else {
    bmp.takeForcedMeasurement();
    delay(12);
    *tempC = bmp.readTemperature();
    if (pressureHpa) {
      *pressureHpa = bmp.readPressure() / 100.0f;
    }
  }
  return !isnan(*tempC);
}

static void drawHud(float tempC, bool tempOk, float pressureHpa, bool pressOk, const struct tm *tmValid,
                    bool timeOk, bool wifiConnected, bool sdOk, float battV, int battPct) {
  const int w = gfx->width();
  const int h = gfx->height();
  const int m = 6;
  const int footerH = 36;
  const uint16_t vcol = battHudColor(battPct);
  char buf[28];
  char runBuf[20];

  gfx->fillScreen(BLACK);
  gfx->setTextWrap(false);

  formatRuntime(runBuf, sizeof(runBuf), battPct);
  uint16_t heroCol = vcol;
  char subBuf[24] = "";
  switch (g_graph) {
    case GRAPH_TEMP:
#if USE_FAHRENHEIT
      if (tempOk) {
        snprintf(buf, sizeof(buf), "%.1fF", tempC * 9.0f / 5.0f + 32.0f);
      } else {
        snprintf(buf, sizeof(buf), "-- F");
      }
#else
      if (tempOk) {
        snprintf(buf, sizeof(buf), "%.1fC", tempC);
      } else {
        snprintf(buf, sizeof(buf), "-- C");
      }
#endif
      heroCol = tempOk ? CYAN : YELLOW;
      snprintf(subBuf, sizeof(subBuf), "5min avg");
      break;
    case GRAPH_DV:
      snprintf(buf, sizeof(buf), "%+.0f", g_dv_mv_min);
      heroCol = (g_dv_mv_min >= 0.0f) ? GREEN : RED;
      snprintf(subBuf, sizeof(subBuf), "mV/min");
      break;
    case GRAPH_NET: {
      float netMa = 0, solarMa = 0, loadMa = 0, mahMin = 0;
      estimatePowerFlow(&netMa, &solarMa, &loadMa, &mahMin);
      snprintf(buf, sizeof(buf), "%+.0f", netMa);
      heroCol = (netMa >= 0.0f) ? GREEN : RED;
      snprintf(subBuf, sizeof(subBuf), "mA net");
      break;
    }
    case GRAPH_ECO2:
      if (g_ccs_data_ok) {
        const char *word = (g_eco2 < 800) ? "good" : (g_eco2 < 1500) ? "ok" : "poor";
        snprintf(buf, sizeof(buf), "%u", (unsigned)g_eco2);
        heroCol = (g_eco2 < 800) ? GREEN : (g_eco2 < 1500) ? YELLOW : RED;
        snprintf(subBuf, sizeof(subBuf), "%s  ppm", word);
      } else {
        snprintf(buf, sizeof(buf), "----");
        heroCol = YELLOW;
        snprintf(subBuf, sizeof(subBuf), "ppm eCO2");
      }
      break;
    case GRAPH_TVOC:
      if (g_ccs_data_ok) {
        const char *word = (g_tvoc < 220) ? "good" : (g_tvoc < 660) ? "ok" : "poor";
        snprintf(buf, sizeof(buf), "%u", (unsigned)g_tvoc);
        heroCol = (g_tvoc < 220) ? GREEN : (g_tvoc < 660) ? YELLOW : RED;
        snprintf(subBuf, sizeof(subBuf), "%s  ppb", word);
      } else {
        snprintf(buf, sizeof(buf), "----");
        heroCol = YELLOW;
        snprintf(subBuf, sizeof(subBuf), "ppb TVOC");
      }
      break;
    case GRAPH_LUX:
      snprintf(buf, sizeof(buf), "%.0f", g_lux);
      heroCol = YELLOW;
      snprintf(subBuf, sizeof(subBuf), "lux");
      break;
    case GRAPH_VBAT:
    default:
      snprintf(buf, sizeof(buf), "%.2fV", battV);
      heroCol = vcol;
      snprintf(subBuf, sizeof(subBuf), "%d%% %s", battPct, runBuf);
      break;
  }
  gfx->setTextSize(4);
  gfx->setTextColor(heroCol, BLACK);
  gfx->setCursor(m, 8);
  gfx->print(buf);
  if (g_charging) {
    gfx->setTextSize(2);
    gfx->setTextColor(YELLOW, BLACK);
    gfx->setCursor(m + 128, 12);
    gfx->print(g_sun_label);
  }
  gfx->setTextSize(2);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(m, 46);
  gfx->print(subBuf);

  if (timeOk && tmValid) {
    formatTime12hShort(tmValid, buf, sizeof(buf));
    gfx->setTextColor(GREEN, BLACK);
  } else if (!g_wifi_has_ssid) {
    snprintf(buf, sizeof(buf), "WiFi?");
    gfx->setTextColor(YELLOW, BLACK);
  } else if (!wifiConnected) {
    snprintf(buf, sizeof(buf), "WiFi...");
    gfx->setTextColor(YELLOW, BLACK);
  } else {
    snprintf(buf, sizeof(buf), "NTP...");
    gfx->setTextColor(YELLOW, BLACK);
  }
  gfx->setTextSize(2);
  {
    const int timeW = (int)strlen(buf) * 12;
    const int timeX = w - m - timeW;
    drawWifiIcon(timeX - 18, 18, wifiConnected);
    gfx->setCursor(timeX, 16);
    gfx->print(buf);
  }

  formatUptime(buf, sizeof(buf));
  gfx->setTextSize(1);
  gfx->setTextColor(WHITE, BLACK);
  gfx->setCursor(w - m - (int)strlen(buf) * 6, 34);
  gfx->print(buf);

  if (g_fw_just_updated) {
    snprintf(buf, sizeof(buf), "v%s %s", FW_VERSION, g_update_was_ota ? "OTA" : "new");
    gfx->setTextColor(GREEN, BLACK);
  } else if (g_fault_count > 0 && (strcmp(g_reset_tag, "crash") == 0 || strcmp(g_reset_tag, "wdt") == 0 ||
                                   strcmp(g_reset_tag, "brownout") == 0)) {
    snprintf(buf, sizeof(buf), "v%s %s", FW_VERSION, g_reset_tag);
    gfx->setTextColor(YELLOW, BLACK);
  } else {
    snprintf(buf, sizeof(buf), "v%s %luoff", FW_VERSION, (unsigned long)g_poweroff_count);
    gfx->setTextColor(DARKGREY, BLACK);
  }
  gfx->setCursor(w - m - (int)strlen(buf) * 6, 44);
  gfx->print(buf);

  const int barY = 68;
  const int barH = 10;
  const int barW = w - 2 * m;
  gfx->drawRect(m, barY, barW, barH, DARKGREY);
  int fillW = (barW - 2) * battPct / 100;
  if (fillW < 0) {
    fillW = 0;
  }
  if (fillW > barW - 2) {
    fillW = barW - 2;
  }
  if (fillW > 0) {
    gfx->fillRect(m + 1, barY + 1, fillW, barH - 2, vcol);
  }

  gfx->setTextSize(1);
  gfx->setTextColor(DARKGREY, BLACK);
  gfx->setCursor(m, 82);
  snprintf(buf, sizeof(buf), "%s  IO18=next", graphViewLabel(g_graph));
  gfx->print(buf);

  const int gx = m;
  const int gy = 94;
  const int gw = w - 2 * m;
  const int gh = h - gy - footerH - 6;
  float ymin = 0.0f;
  float ymax = 1.0f;
  switch (g_graph) {
    case GRAPH_TEMP:
      histRange(g_tempHistCount, tempHistSample, 1.0f, 2.0f, &ymin, &ymax);
      drawSparkline(gx, gy, gw, gh, g_tempHistCount, tempHistSample, ymin, ymax, ORANGE);
      break;
    case GRAPH_DV:
      histRange(g_dvHistCount, dvHistSample, 0.5f, 2.0f, &ymin, &ymax);
      if (ymin > 0.0f) {
        ymin = 0.0f;
      }
      if (ymax < 0.0f) {
        ymax = 0.0f;
      }
      drawSparkline(gx, gy, gw, gh, g_dvHistCount, dvHistSample, ymin, ymax, GREEN);
      break;
    case GRAPH_NET:
      histRange(g_netHistCount, netHistSample, 10.0f, 40.0f, &ymin, &ymax);
      if (ymin > 0.0f) {
        ymin = 0.0f;
      }
      if (ymax < 0.0f) {
        ymax = 0.0f;
      }
      drawSparkline(gx, gy, gw, gh, g_netHistCount, netHistSample, ymin, ymax, GREEN);
      break;
    case GRAPH_ECO2:
      histRange(g_eco2HistCount, eco2HistSample, 50.0f, 200.0f, &ymin, &ymax);
      if (ymin < 400.0f) {
        ymin = 400.0f;
      }
      drawSparkline(gx, gy, gw, gh, g_eco2HistCount, eco2HistSample, ymin, ymax, GREEN);
      break;
    case GRAPH_TVOC:
      histRange(g_tvocHistCount, tvocHistSample, 10.0f, 50.0f, &ymin, &ymax);
      if (ymin < 0.0f) {
        ymin = 0.0f;
      }
      drawSparkline(gx, gy, gw, gh, g_tvocHistCount, tvocHistSample, ymin, ymax, ORANGE);
      break;
    case GRAPH_LUX:
      histRange(g_luxHistCount, luxHistSample, 20.0f, 100.0f, &ymin, &ymax);
      if (ymin < 0.0f) {
        ymin = 0.0f;
      }
      drawSparkline(gx, gy, gw, gh, g_luxHistCount, luxHistSample, ymin, ymax, YELLOW);
      break;
    case GRAPH_VBAT:
    default:
      drawSparkline(gx, gy, gw, gh, g_sparkCount, sparkSample, BAT_GRAPH_V_MIN, BAT_GRAPH_V_MAX, CYAN);
      {
        int y37 = gy + gh - 1 -
                  (int)((3.70f - BAT_GRAPH_V_MIN) / (BAT_GRAPH_V_MAX - BAT_GRAPH_V_MIN) * (float)(gh - 2));
        if (y37 > gy && y37 < gy + gh - 1) {
          gfx->drawFastHLine(gx + 1, y37, gw - 2, DARKGREY);
        }
      }
      break;
  }

  const int fy = h - footerH;
  gfx->drawFastHLine(0, fy, w, DARKGREY);
  const int footY = fy + 10;
  gfx->setTextSize(2);

  if (g_graph == GRAPH_VBAT) {
#if USE_FAHRENHEIT
    if (tempOk) {
      snprintf(buf, sizeof(buf), "%.1fF", tempC * 9.0f / 5.0f + 32.0f);
      gfx->setTextColor(CYAN, BLACK);
    } else {
      snprintf(buf, sizeof(buf), "BMP--");
      gfx->setTextColor(YELLOW, BLACK);
    }
#else
    if (tempOk) {
      snprintf(buf, sizeof(buf), "%.1fC", tempC);
      gfx->setTextColor(CYAN, BLACK);
    } else {
      snprintf(buf, sizeof(buf), "BMP--");
      gfx->setTextColor(YELLOW, BLACK);
    }
#endif
  } else if (g_ccs_data_ok && (g_graph == GRAPH_TEMP || g_graph == GRAPH_DV || g_graph == GRAPH_NET ||
                               g_graph == GRAPH_LUX)) {
    snprintf(buf, sizeof(buf), "%uppm", (unsigned)g_eco2);
    gfx->setTextColor(GREEN, BLACK);
  } else {
    snprintf(buf, sizeof(buf), "%.2fV", battV);
    gfx->setTextColor(vcol, BLACK);
  }
  gfx->setCursor(m, footY);
  gfx->print(buf);

  char sdBuf[16];
  if (sdOk) {
    snprintf(sdBuf, sizeof(sdBuf), "SD %lluM", (unsigned long long)g_sd_free_mb);
  } else {
    snprintf(sdBuf, sizeof(sdBuf), "SD --");
  }
  if (pressOk) {
    snprintf(buf, sizeof(buf), "%.0fhPa", pressureHpa);
    gfx->setTextColor(WHITE, BLACK);
  } else {
    snprintf(buf, sizeof(buf), "hPa--");
    gfx->setTextColor(YELLOW, BLACK);
  }
  const int sdW = (int)strlen(sdBuf) * 12;
  const int pW = (int)strlen(buf) * 12;
  gfx->setCursor(w - m - sdW - 8 - pW, footY);
  gfx->print(buf);
  gfx->setTextColor(sdOk ? GREEN : YELLOW, BLACK);
  gfx->setCursor(w - m - sdW, footY);
  gfx->print(sdBuf);
}

static bool nameEndsWithJpg(const String &name) {
  if (name.length() < 5) {
    return false;
  }
  String tail = name.substring(name.length() - 4);
  tail.toLowerCase();
  return tail == ".jpg";
}

static bool isSafePicJpgName(const String &name) {
  if (name.length() == 0 || name.length() >= PIC_NAME_MAX) {
    return false;
  }
  if (name.indexOf('/') >= 0 || name.indexOf('\\') >= 0) {
    return false;
  }
  if (!nameEndsWithJpg(name)) {
    return false;
  }
  for (size_t i = 0; i < name.length(); i++) {
    char c = name[i];
    bool ok = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_' ||
              c == '-' || c == '.';
    if (!ok) {
      return false;
    }
  }
  return true;
}

static void refreshPicList(void) {
  g_pic_count = 0;
  if (!g_sd_ok) {
    g_pic_list_dirty = false;
    return;
  }
  sdLock();
  ensureSdDataDirs();
  File dir = SD.open(kPicsDir);
  if (!dir) {
    sdUnlock();
    g_pic_list_dirty = false;
    return;
  }
  while (g_pic_count < MAX_PIC_FILES) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }
    String name = entry.name();
    int slash = name.lastIndexOf('/');
    if (slash >= 0) {
      name = name.substring((size_t)slash + 1);
    }
    entry.close();
    if (isSafePicJpgName(name)) {
      name.toCharArray(g_pic_names[g_pic_count], PIC_NAME_MAX);
      g_pic_count++;
    }
  }
  dir.close();
  sdUnlock();
  g_pic_list_dirty = false;
  Serial.printf("PICS: %d JPEG(s) in %s\n", g_pic_count, kPicsDir);
}

static int jpegDrawCallback(JPEGDRAW *pDraw) {
  gfx->draw16bitBeRGBBitmap(pDraw->x, pDraw->y, pDraw->pPixels, pDraw->iWidth, pDraw->iHeight);
  return 1;
}

static void drawCurrentPic(void) {
  if (!g_sd_ok || g_pic_count <= 0 || g_pic_index < 0 || g_pic_index >= g_pic_count) {
    return;
  }
  char path[64];
  snprintf(path, sizeof(path), "%s/%s", kPicsDir, g_pic_names[g_pic_index]);

  gfx->fillScreen(BLACK);
  sdLock();
  unsigned long t0 = millis();
  jpegDrawSdPath(path, jpegDrawCallback, true, 0, 0, gfx->width(), gfx->height());
  sdUnlock();
  Serial.printf("JPEG draw %s (%lu ms)\n", path, millis() - t0);

  gfx->setTextColor(WHITE, BLACK);
  gfx->setTextSize(1);
  gfx->setCursor(2, gfx->height() - 10);
  gfx->printf("%d/%d %s", g_pic_index + 1, g_pic_count, g_pic_names[g_pic_index]);
}

static String htmlEscape(const String &in) {
  String o;
  o.reserve(in.length() + 8);
  for (size_t i = 0; i < in.length(); i++) {
    char c = in[i];
    switch (c) {
      case '&':
        o += "&amp;";
        break;
      case '<':
        o += "&lt;";
        break;
      case '>':
        o += "&gt;";
        break;
      case '"':
        o += "&quot;";
        break;
      default:
        o += c;
        break;
    }
  }
  return o;
}

static String buildPicTableHtml(void) {
  if (!g_sd_ok) {
    return "<tr><td colspan=\"3\">SD not mounted</td></tr>";
  }
  sdLock();
  if (!SD.exists(kPicsDir)) {
    sdUnlock();
    return "<tr><td colspan=\"3\">No /PICS folder yet</td></tr>";
  }
  File dir = SD.open(kPicsDir);
  if (!dir) {
    sdUnlock();
    return "<tr><td colspan=\"3\">Cannot open /PICS</td></tr>";
  }

  String out;
  int rows = 0;
  while (rows < 20) {
    File entry = dir.openNextFile();
    if (!entry) {
      break;
    }
    String name = entry.name();
    int slash = name.lastIndexOf('/');
    if (slash >= 0) {
      name = name.substring((size_t)slash + 1);
    }
    if (isSafePicJpgName(name)) {
      uint64_t sz = (uint64_t)entry.size();
      const String esc = htmlEscape(name);
      out += "<tr><td>" + esc + "</td><td>" + String((unsigned long long)sz) + "</td><td>";
      out += "<a href=\"/pics/get?name=" + esc + "\">Download</a> ";
      out += "<form method=\"POST\" action=\"/pics/delete\" style=\"display:inline\" onsubmit=\"return confirm('Delete?');\">";
      out += "<input type=\"hidden\" name=\"name\" value=\"" + esc + "\">";
      out += "<button type=\"submit\">Delete</button></form></td></tr>";
      rows++;
    }
    entry.close();
  }
  dir.close();
  sdUnlock();
  if (rows == 0) {
    return "<tr><td colspan=\"3\">No .jpg in /PICS</td></tr>";
  }
  return out;
}

static const char kPicsHtml[] PROGMEM = R"EOS(
<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Solar Manager</title>
<style>
:root{--bg:#0d1117;--card:#161b22;--line:#30363d;--text:#e6edf3;--muted:#8b949e;--accent:#3fb950;--cyan:#58a6ff;--warn:#d29922}
*{box-sizing:border-box}
body{font-family:Segoe UI,system-ui,sans-serif;margin:0;background:var(--bg);color:var(--text);line-height:1.4}
.wrap{max-width:1100px;margin:0 auto;padding:1.25rem 1.5rem 2.5rem}
header{display:flex;flex-wrap:wrap;align-items:baseline;justify-content:space-between;gap:.75rem;margin-bottom:1.25rem;border-bottom:1px solid var(--line);padding-bottom:.85rem}
header h1{margin:0;font-size:1.65rem;font-weight:650;letter-spacing:-.02em}
header .ver{color:var(--accent);font-size:.95rem;font-weight:600}
.flow{display:grid;grid-template-columns:repeat(4,1fr);gap:.75rem;margin-bottom:1.25rem}
.flow .card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:1rem 1.1rem}
.flow span{display:block;font-size:.72rem;text-transform:uppercase;letter-spacing:.04em;color:var(--muted);margin-bottom:.35rem}
.flow b{font-size:1.45rem;font-weight:650}
.flow .sub{font-size:.8rem;color:var(--muted);margin-top:.25rem}
.pos{color:var(--accent)}.neg{color:#f85149}
.aq-good{color:var(--accent)}.aq-ok{color:var(--warn)}.aq-poor{color:#f85149}
select.rate{width:100%;margin-top:.15rem;background:#0d1117;color:var(--text);border:1px solid var(--line);border-radius:6px;padding:.4rem .45rem;font:inherit;font-weight:650}
.stats{display:grid;grid-template-columns:repeat(4,1fr);gap:.65rem;margin-bottom:1.25rem}
.stats>div{background:var(--card);border:1px solid var(--line);border-radius:8px;padding:.7rem .8rem}
.stats span{display:block;font-size:.7rem;color:var(--muted);margin-bottom:.2rem}
.stats b{font-size:1.05rem;font-weight:600}
.graphs{display:grid;grid-template-columns:1fr 1fr;gap:.85rem;margin-bottom:1.5rem}
.gcard{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:.75rem .85rem .55rem}
.gmeta{display:flex;align-items:baseline;justify-content:space-between;gap:.5rem;margin-bottom:.35rem}
.gmeta .label{font-size:.72rem;text-transform:uppercase;letter-spacing:.04em;color:var(--muted)}
.gmeta .val{font-size:1.15rem;font-weight:650;color:var(--cyan)}
.gmeta .hint{font-size:.75rem;color:var(--muted);text-align:right}
.plot{position:relative}
.gcard canvas{width:100%;height:176px;background:#0a0e14;border-radius:6px;display:block;touch-action:pan-y}
.tip{position:absolute;display:none;pointer-events:none;z-index:2;background:rgba(13,17,23,.92);border:1px solid var(--line);border-radius:6px;padding:.28rem .5rem;font-size:.75rem;white-space:nowrap;color:var(--text)}
.main{display:grid;grid-template-columns:minmax(0,1fr) 280px;gap:1.1rem;align-items:start}
.legend{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:1rem 1.1rem;position:sticky;top:1rem}
.legend h2{margin:0 0 .6rem;font-size:1rem}
.legend dl{margin:0;font-size:.82rem}
.legend dt{color:var(--cyan);font-weight:600;margin-top:.55rem}
.legend dd{margin:.15rem 0 0;color:var(--muted)}
.legend-toggle{display:none}
.group h2{margin:0 0 .55rem}
.logs a{display:inline-block;margin:0 .6rem .5rem 0;padding:.45rem .75rem;background:#1c2128;border:1px solid var(--line);border-radius:6px;text-decoration:none}
h2{font-size:1.1rem;margin:1.35rem 0 .55rem;font-weight:600}
.note{font-size:.85rem;color:var(--muted);margin:.5rem 0 1rem}
table{width:100%;border-collapse:collapse;font-size:.9rem;background:var(--card);border-radius:8px;overflow:hidden}
th,td{border:1px solid var(--line);padding:8px 10px} th{background:#1c2128;text-align:left}
a{color:var(--cyan)} button{margin:2px;cursor:pointer}
@media (max-width:900px){
.wrap{padding:1rem .85rem 5.5rem}
.flow,.stats{grid-template-columns:1fr 1fr}
.graphs{grid-template-columns:1fr}
.flow b{font-size:1.2rem}
.gcard canvas{height:190px}
.main{grid-template-columns:1fr}
.legend{
  position:fixed;left:.5rem;right:.5rem;bottom:.45rem;top:auto;z-index:30;
  max-height:42vh;overflow:auto;
  box-shadow:0 -10px 28px rgba(0,0,0,.55);
}
.legend:not(.open){max-height:none;overflow:hidden;padding:.55rem .8rem}
.legend:not(.open) .legend-body{display:none}
.legend-toggle{
  display:flex;width:100%;align-items:center;justify-content:space-between;
  background:transparent;color:var(--text);border:0;font:inherit;font-weight:650;
  padding:.15rem 0;cursor:pointer;
}
}
</style></head><body>
<div class="wrap">
<header>
<h1>Solar Manager</h1>
<span class="ver">v%VER%</span>
</header>
<div class="main">
<div>
<section class="group">
<h2>Power</h2>
<section class="flow">
<div class="card"><span>Solar in (est)</span><b id="s_in" class="pos">%IN%</b><div class="sub">DFRobot ≤900 mA · 5V/1A panel</div></div>
<div class="card"><span>Load out (est)</span><b id="s_out" class="neg">%OUT%</b><div class="sub">sensors + board</div></div>
<div class="card"><span>Net to pack</span><b id="s_net">%NET%</b><div class="sub" id="s_mah">%MAH%</div></div>
<div class="card"><span>V trend</span><b id="s_dv">%DV%</b><div class="sub">sun today <span id="s_sunt">%SUNT%</span></div></div>
</section>
<section class="stats">
<div><span>Sunlight</span><b id="s_sun">%SUN%</b></div>
<div><span>Battery</span><b id="s_vbat">%VBAT%</b></div>
<div><span>Charge</span><b id="s_pct">%PCT%</b></div>
<div><span>Runtime est</span><b id="s_est">%EST%</b></div>
</section>
</section>
<section class="group">
<h2>Air</h2>
<section class="stats">
<div><span>Temperature</span><b id="s_temp">%TEMP%</b></div>
<div><span>Pressure</span><b id="s_hpa">%HPA%</b></div>
<div><span>Humidity</span><b id="s_rh">--</b></div>
<div><span>Light</span><b id="s_lux">%LUX%</b></div>
<div><span>eCO2</span><b id="s_eco2">%ECO2%</b></div>
<div><span>TVOC</span><b id="s_tvoc">%TVOC%</b></div>
</section>
</section>
<section class="group">
<h2>Graphs</h2>
<section class="graphs">
<div class="gcard"><div class="gmeta"><span class="label">Battery · 80s</span><b class="val" id="gv_vbat">%VBAT%</b><span class="hint" id="gv_pct">%PCT%</span></div><div class="plot"><canvas id="g_vbat"></canvas><div class="tip"></div></div></div>
<div class="gcard"><div class="gmeta"><span class="label">Temp · 5 min</span><b class="val" id="gv_temp">%TEMP%</b><span class="hint">BMP/BME</span></div><div class="plot"><canvas id="g_temp"></canvas><div class="tip"></div></div></div>
<div class="gcard"><div class="gmeta"><span class="label">Pack dV</span><b class="val" id="gv_dv">%DV%</b><span class="hint" id="gv_flow">mV/min</span></div><div class="plot"><canvas id="g_dv"></canvas><div class="tip"></div></div></div>
<div class="gcard"><div class="gmeta"><span class="label">Net current</span><b class="val" id="gv_net">%NET%</b><span class="hint">est mA</span></div><div class="plot"><canvas id="g_net"></canvas><div class="tip"></div></div></div>
<div class="gcard"><div class="gmeta"><span class="label">Light · 80s</span><b class="val" id="gv_lux">%LUX%</b><span class="hint">BH1750</span></div><div class="plot"><canvas id="g_lux"></canvas><div class="tip"></div></div></div>
<div class="gcard"><div class="gmeta"><span class="label">eCO2 · 80s</span><b class="val" id="gv_eco2">%ECO2%</b><span class="hint">CCS811</span></div><div class="plot"><canvas id="g_eco2"></canvas><div class="tip"></div></div></div>
<div class="gcard"><div class="gmeta"><span class="label">TVOC · 80s</span><b class="val" id="gv_tvoc">%TVOC%</b><span class="hint">CCS811</span></div><div class="plot"><canvas id="g_tvoc"></canvas><div class="tip"></div></div></div>
</section>
</section>
<section class="group">
<h2>System</h2>
<section class="stats">
<div><span>Time</span><b id="s_time">%TIME%</b></div>
<div><span>Uptime</span><b id="s_up">%UP%</b></div>
<div><span>WiFi</span><b id="s_wifi">%WIFI%</b></div>
<div><span>Signal</span><b id="s_rssi">--</b></div>
<div><span>Screen</span><b id="s_scr">--</b></div>
<div><span>Radio</span><b id="s_radio">--</b></div>
<div><span>SD free</span><b id="s_sd">%SD%</b></div>
<div><span>Last start</span><b id="s_rst">%RST%</b></div>
<div><span>Boots</span><b id="s_boots">--</b></div>
<div><span>Power-offs</span><b id="s_poff">--</b></div>
<div><span>Faults</span><b id="s_fault">--</b></div>
<div><span>Live refresh</span>
<select id="s_rate" class="rate">
<option value="1000">every 1 s</option>
<option value="2000">every 2 s</option>
<option value="5000" selected>every 5 s</option>
<option value="10000">every 10 s</option>
<option value="30000">every 30 s</option>
<option value="60000">every 1 min</option>
</select>
<b id="s_age" style="display:block;margin-top:.35rem;font-size:.8rem;font-weight:500;color:var(--muted)">board samples every 1 s</b>
</div>
</section>
</section>
<h2>Download logs</h2>
<p class="logs">
<a href="/logs/env.csv">env.csv</a>
<a href="/logs/charge.csv">charge.csv</a>
</p>
<p class="note">CSV on SD under /LOG (temp, pressure, humidity, V, lux, eCO2, TVOC). New env.csv picks up humidity; delete the old file on the card if the header is missing that column. GPIO18 cycles HUD graphs. GPIO15 slideshow.</p>
<h2>SD Pictures (/PICS)</h2>
<table><thead><tr><th>File</th><th>Bytes</th><th></th></tr></thead>
<tbody>%PIC_TABLE%</tbody></table>
</div>
<aside class="legend open" id="legend">
<button type="button" class="legend-toggle" id="legendToggle">Legend <span id="legendHint">hide</span></button>
<div class="legend-body">
<h2>Legend</h2>
<dl>
<dt>Vbat</dt><dd>1S pack volts at the Waveshare JST sense pin (ADC×3).</dd>
<dt>dV (mV/min)</dt><dd>How fast pack voltage is changing. + rising / − falling.</dd>
<dt>Hardware</dt><dd>DFRobot Solar 1V1-class (~900 mA charge) + 5 V / 1 A panel → 1S pack. Est. only — no INA.</dd>
<dt>Net mA</dt><dd>≈ dV_mV/min × (6000 mAh ÷ 1200 mV) × 60. Capped at 900 mA while charging.</dd>
<dt>mAh/min</dt><dd>Same idea per minute of charge into (or out of) the pack.</dd>
<dt>Solar in</dt><dd>When charging: net + assumed board/sensor load (~160 mA), max 900 mA (charger), panel rated 1000 mA.</dd>
<dt>Load out</dt><dd>Assumed draw while charging; from falling V when draining.</dd>
<dt>Light (lux)</dt><dd>BH1750. Bright lux helps confirm sun even if V is flat (CV hold).</dd>
<dt>Humidity</dt><dd>BME280 only. BMP280 has no RH; CCS811 then assumes 50%.</dd>
<dt>eCO2</dt><dd>CCS811 ppm. Good under 800, OK under 1500, poor above that. Outdoor air is about 400.</dd>
<dt>TVOC</dt><dd>CCS811 ppb. Good under 220, OK under 660, poor above that. Needs ~20 min warm-up. Compensated with temperature (and humidity on a BME280).</dd>
<dt>Shade / sun</dt><dd>Charge starts only on rising pack V. Lux never starts a charge (avoids bright-shade false positives); dark lux helps exit.</dd>
<dt>Sun states</dt><dd>rising · holding · bright · draining · idle</dd>
<dt>Live refresh</dt><dd>How often this page asks the board for new numbers. The board itself samples about once a second. Faster than 1 s repeats the same sample. The choice is saved in this browser.</dd>
</dl>
<p class="note" style="margin-top:1rem">OTA: <code>pio run -e esp32s3_lcd2_ota -t upload</code></p>
</div>
</aside>
</div>
</div>
<script>
const fmtMa=v=>Math.round(v)+' mA';
const aqClass=w=>w==='good'?'aq-good':w==='ok'?'aq-ok':'aq-poor';
const eco2Word=v=>v<800?'good':v<1500?'ok':'poor';
const tvocWord=v=>v<220?'good':v<660?'ok':'poor';
const setAq=(id,text,word)=>{
  const el=document.getElementById(id);
  el.textContent=text;
  el.classList.remove('aq-good','aq-ok','aq-poor');
  if(word)el.classList.add(aqClass(word));
};
const paint=j=>{
  document.getElementById('s_time').textContent=j.time;
  document.getElementById('s_vbat').textContent=j.vbat.toFixed(2)+' V';
  document.getElementById('s_pct').textContent=j.pct+'%';
  document.getElementById('s_est').textContent=j.est;
  document.getElementById('s_temp').textContent=j.temp_ok?j.temp_f.toFixed(1)+' F':'--';
  document.getElementById('s_hpa').textContent=j.press_ok?Math.round(j.hpa)+' hPa':'--';
  document.getElementById('s_lux').textContent=j.lux_ok?Math.round(j.lux)+' lx':'--';
  document.getElementById('s_rh').textContent=j.rh_ok?Math.round(j.rh)+' %':'--';
  setAq('s_eco2', j.ccs_ok?Math.round(j.eco2)+' ppm · '+eco2Word(j.eco2):'--', j.ccs_ok?eco2Word(j.eco2):'');
  setAq('s_tvoc', j.ccs_ok?Math.round(j.tvoc)+' ppb · '+tvocWord(j.tvoc):'--', j.ccs_ok?tvocWord(j.tvoc):'');
  document.getElementById('s_wifi').textContent=j.wifi?(j.ip||'up'):'down';
  document.getElementById('s_rssi').textContent=j.wifi?(j.rssi+' dBm'):'--';
  document.getElementById('s_scr').textContent=j.screen?'on':'off';
  document.getElementById('s_radio').textContent=j.wifi_sleep?'asleep':'awake';
  document.getElementById('s_sd').textContent=j.sd?(j.sd_mb+' MB'):'--';
  document.getElementById('s_up').textContent=j.uptime;
  document.getElementById('s_rst').textContent=j.reset;
  document.getElementById('s_boots').textContent=j.boots;
  document.getElementById('s_poff').textContent=j.poweroffs;
  document.getElementById('s_fault').textContent=j.faults;
  document.getElementById('s_sun').textContent=j.sun+(j.charging?' · charging':'');
  const dv=(j.dv_mv_min>=0?'+':'')+j.dv_mv_min.toFixed(1)+' mV/min';
  document.getElementById('s_dv').textContent=dv;
  document.getElementById('s_sunt').textContent=j.sun_min+' min';
  document.getElementById('s_in').textContent=fmtMa(j.solar_ma);
  document.getElementById('s_out').textContent=fmtMa(j.load_ma);
  const net=document.getElementById('s_net');
  net.textContent=(j.net_ma>=0?'+':'')+fmtMa(j.net_ma);
  net.className=j.net_ma>=0?'pos':'neg';
  document.getElementById('s_mah').textContent=(j.mah_min>=0?'+':'')+j.mah_min.toFixed(2)+' mAh/min';
  document.getElementById('gv_vbat').textContent=j.vbat.toFixed(2)+' V';
  document.getElementById('gv_pct').textContent=j.pct+'% · '+j.est;
  document.getElementById('gv_temp').textContent=j.temp_ok?j.temp_f.toFixed(1)+' F':'--';
  document.getElementById('gv_dv').textContent=dv;
  document.getElementById('gv_flow').textContent='in '+fmtMa(j.solar_ma)+' · out '+fmtMa(j.load_ma);
  document.getElementById('gv_net').textContent=(j.net_ma>=0?'+':'')+fmtMa(j.net_ma);
  document.getElementById('gv_lux').textContent=j.lux_ok?Math.round(j.lux)+' lx':'--';
  setAq('gv_eco2', j.ccs_ok?Math.round(j.eco2)+' · '+eco2Word(j.eco2):'--', j.ccs_ok?eco2Word(j.eco2):'');
  setAq('gv_tvoc', j.ccs_ok?Math.round(j.tvoc)+' · '+tvocWord(j.tvoc):'--', j.ccs_ok?tvocWord(j.tvoc):'');
};
const legend=document.getElementById('legend');
const legendHint=document.getElementById('legendHint');
document.getElementById('legendToggle').onclick=()=>{
  legend.classList.toggle('open');
  legendHint.textContent=legend.classList.contains('open')?'hide':'show';
};
const charts={};
const ageText=s=>{
  s=Math.max(0,Math.round(s));
  if(s<90)return s+'s ago';
  const m=Math.round(s/60);
  if(m<90)return m+' min ago';
  return (m/60).toFixed(1)+' h ago';
};
const fmtAxis=(v,d,sign)=>{
  const n=d?v.toFixed(d):String(Math.round(v));
  if(sign&&v>0)return '+'+n;
  return n;
};
const renderChart=(id)=>{
  const s=charts[id]; if(!s)return;
  const c=s.c, ctx=c.getContext('2d');
  const pad=s.pad, w=s.w, h=s.h, opt=s.opt;
  ctx.setTransform(s.dpr,0,0,s.dpr,0,0);
  ctx.clearRect(0,0,w,h);
  ctx.fillStyle='#0a0e14'; ctx.fillRect(0,0,w,h);
  ctx.font='11px Segoe UI,sans-serif';
  const tip=c.parentElement.querySelector('.tip');
  if(!s.ok){
    ctx.fillStyle='#8b949e';
    ctx.fillText('waiting for samples', pad.l, h/2);
    if(tip)tip.style.display='none';
    return;
  }
  const lo=s.lo, hi=s.hi, arr=s.arr, n=arr.length;
  const xOf=i=>pad.l+(n<2?0:(i/(n-1))*(w-pad.l-pad.r));
  const yOf=v=>pad.t+(1-(v-lo)/(hi-lo))*(h-pad.t-pad.b);
  ctx.strokeStyle='#21262d'; ctx.lineWidth=1; ctx.fillStyle='#8b949e';
  for(let g=0;g<4;g++){
    const v=lo+(hi-lo)*(g/3);
    const y=yOf(v);
    ctx.beginPath(); ctx.moveTo(pad.l,y); ctx.lineTo(w-pad.r,y); ctx.stroke();
    ctx.fillText(fmtAxis(v,opt.digits,opt.zero), 4, y+4);
  }
  if(opt.zero&&lo<0&&hi>0){
    ctx.strokeStyle='#484f58';
    ctx.beginPath(); ctx.moveTo(pad.l,yOf(0)); ctx.lineTo(w-pad.r,yOf(0)); ctx.stroke();
  }
  ctx.beginPath();
  for(let i=0;i<n;i++){
    const x=xOf(i), y=yOf(arr[i]);
    if(i===0)ctx.moveTo(x,y); else ctx.lineTo(x,y);
  }
  ctx.strokeStyle=opt.color; ctx.lineWidth=2; ctx.stroke();
  ctx.lineTo(xOf(n-1), h-pad.b); ctx.lineTo(xOf(0), h-pad.b); ctx.closePath();
  ctx.globalAlpha=0.16; ctx.fillStyle=opt.color; ctx.fill(); ctx.globalAlpha=1;
  const span=(n-1)*(opt.step||1);
  ctx.fillStyle='#8b949e';
  ctx.fillText(ageText(span), pad.l, h-6);
  const nowL=ctx.measureText('now').width;
  ctx.fillText('now', w-pad.r-nowL, h-6);
  const unit=opt.unit||'';
  const uw=ctx.measureText(unit).width;
  ctx.fillText(unit, (pad.l+w-pad.r-uw)/2, h-6);
  if(tip)tip.style.display='none';
  if(s.hover<pad.l||s.hover>w-pad.r)return;
  let idx=Math.round((s.hover-pad.l)/Math.max(1,w-pad.l-pad.r)*(n-1));
  if(idx<0)idx=0; if(idx>n-1)idx=n-1;
  const hx=xOf(idx), hy=yOf(arr[idx]);
  ctx.strokeStyle='#e6edf3'; ctx.globalAlpha=0.45;
  ctx.beginPath(); ctx.moveTo(hx,pad.t); ctx.lineTo(hx,h-pad.b); ctx.stroke();
  ctx.globalAlpha=1; ctx.fillStyle=opt.color;
  ctx.beginPath(); ctx.arc(hx,hy,3.5,0,6.3); ctx.fill();
  if(tip){
    const ago=(n-1-idx)*(opt.step||1);
    tip.textContent=fmtAxis(arr[idx],opt.digits,opt.zero)+' '+unit+' · '+(ago<1?'now':ageText(ago));
    tip.style.display='block';
    let left=hx+10; if(left+tip.offsetWidth>w-4) left=hx-tip.offsetWidth-10;
    tip.style.left=Math.max(4,left)+'px';
    tip.style.top=Math.max(4,hy-28)+'px';
  }
}
const drawChart=(id, arr, opt)=>{
  const c=document.getElementById(id); if(!c)return;
  const w=c.clientWidth||300, h=c.clientHeight||176;
  const dpr=Math.min(window.devicePixelRatio||1,2);
  if(c.width!==Math.floor(w*dpr)||c.height!==Math.floor(h*dpr)){
    c.width=Math.floor(w*dpr); c.height=Math.floor(h*dpr);
  }
  const prev=charts[id]?charts[id].hover:-1;
  let lo=0,hi=1,ok=arr&&arr.length>=2;
  if(ok){
    lo=arr[0]; hi=lo;
    for(let i=1;i<arr.length;i++){if(arr[i]<lo)lo=arr[i]; if(arr[i]>hi)hi=arr[i];}
    if(opt.zero){if(lo>0)lo=0; if(hi<0)hi=0;}
    const minSpan=opt.minSpan||0.05;
    if(hi-lo<minSpan){const mid=(hi+lo)/2; lo=mid-minSpan/2; hi=mid+minSpan/2;}
    const py=(hi-lo)*0.08; lo-=py; hi+=py;
    if(opt.floor0&&lo<0)lo=0;
  }
  charts[id]={c,w,h,dpr,arr:arr||[],opt,lo,hi,ok,hover:prev,pad:{l:48,r:8,t:8,b:20}};
  if(!c._bound){
    c._bound=1;
    const point=ev=>{
      const st=charts[id]; if(!st)return;
      const p=ev.touches?ev.touches[0]:ev;
      if(!p){st.hover=-1; renderChart(id); return;}
      st.hover=p.clientX-c.getBoundingClientRect().left;
      renderChart(id);
    };
    c.addEventListener('mousemove', point);
    c.addEventListener('mouseleave', ()=>{if(charts[id]){charts[id].hover=-1; renderChart(id);}});
    c.addEventListener('touchstart', point, {passive:true});
    c.addEventListener('touchmove', point, {passive:true});
    c.addEventListener('touchend', ()=>{if(charts[id]){charts[id].hover=-1; renderChart(id);}});
  }
  renderChart(id);
}
const tick=()=>{
  fetch('/status').then(r=>r.json()).then(j=>{lastFetch=Date.now();paint(j);ageLine();}).catch(()=>{});
  fetch('/graphs').then(r=>r.json()).then(g=>{
    drawChart('g_vbat', g.vbat, {unit:'V', digits:2, color:'#58a6ff', step:1, minSpan:0.02});
    drawChart('g_temp', g.temp, {unit:'F', digits:1, color:'#d29922', step:300, minSpan:1});
    drawChart('g_dv', g.dv, {unit:'mV/min', digits:1, color:'#3fb950', step:15, zero:true, minSpan:1});
    drawChart('g_net', g.net, {unit:'mA', digits:0, color:'#a371f7', step:15, zero:true, minSpan:20});
    drawChart('g_lux', g.lux, {unit:'lx', digits:0, color:'#e3b341', step:1, minSpan:20, floor0:true});
    drawChart('g_eco2', g.eco2, {unit:'ppm', digits:0, color:'#3fb950', step:1, minSpan:40, floor0:true});
    drawChart('g_tvoc', g.tvoc, {unit:'ppb', digits:0, color:'#f0883e', step:1, minSpan:10, floor0:true});
  }).catch(()=>{});
};
let pollMs=5000;
let pollTimer=0;
let lastFetch=0;
const rateEl=document.getElementById('s_rate');
try{
  const saved=localStorage.getItem('sm-poll');
  if(saved&&rateEl.querySelector('[value="'+saved+'"]')){pollMs=Number(saved);rateEl.value=saved;}
}catch(e){}
const ageLine=()=>{
  const el=document.getElementById('s_age');
  if(!el)return;
  if(!lastFetch){el.textContent='board samples every 1 s';return;}
  const s=Math.round((Date.now()-lastFetch)/1000);
  el.textContent=(s<1?'updated just now':'updated '+s+'s ago')+' · board every 1 s';
};
const arm=()=>{
  if(pollTimer)clearInterval(pollTimer);
  pollTimer=setInterval(tick,pollMs);
};
rateEl.onchange=()=>{
  pollMs=Number(rateEl.value)||5000;
  try{localStorage.setItem('sm-poll',String(pollMs));}catch(e){}
  tick();
  arm();
};
tick();
arm();
setInterval(ageLine,1000);
</script>
</body></html>
)EOS";

static void fillStatusStrings(String &timeS, String &vbatS, String &pctS, String &estS, String &tempS,
                              String &hpaS, String &wifiS, String &sdS) {
  char buf[28];
  struct tm t;
  if (getLocalTimeValid(&t)) {
    formatTime12h(&t, buf, sizeof(buf));
    timeS = buf;
  } else if (WiFi.status() != WL_CONNECTED) {
    timeS = "WiFi...";
  } else {
    timeS = "NTP...";
  }
  snprintf(buf, sizeof(buf), "%.2f V", g_hud_vbat);
  vbatS = buf;
  snprintf(buf, sizeof(buf), "%d%%", g_hud_pct);
  pctS = buf;
  formatRuntime(buf, sizeof(buf), g_hud_pct);
  estS = buf;
  if (g_hud_tempOk) {
    snprintf(buf, sizeof(buf), "%.1f F", g_hud_tempF);
    tempS = buf;
  } else {
    tempS = "--";
  }
  if (g_hud_pressOk) {
    snprintf(buf, sizeof(buf), "%.0f hPa", g_hud_hpa);
    hpaS = buf;
  } else {
    hpaS = "--";
  }
  if (WiFi.status() == WL_CONNECTED) {
    wifiS = WiFi.localIP().toString();
  } else {
    wifiS = "down";
  }
  if (g_sd_ok) {
    snprintf(buf, sizeof(buf), "%llu MB", (unsigned long long)g_sd_free_mb);
    sdS = buf;
  } else {
    sdS = "--";
  }
}

static void handleRoot(void) {
  String page = FPSTR(kPicsHtml);
  String timeS, vbatS, pctS, estS, tempS, hpaS, wifiS, sdS;
  fillStatusStrings(timeS, vbatS, pctS, estS, tempS, hpaS, wifiS, sdS);
  char upBuf[20];
  char rstBuf[32];
  formatUptime(upBuf, sizeof(upBuf));
  if (g_fw_just_updated && g_update_was_ota) {
    snprintf(rstBuf, sizeof(rstBuf), "OTA ok (%s)", g_reset_tag);
  } else if (g_fw_just_updated) {
    snprintf(rstBuf, sizeof(rstBuf), "new fw (%s)", g_reset_tag);
  } else {
    snprintf(rstBuf, sizeof(rstBuf), "%s", g_reset_tag);
  }
  page.replace("%TIME%", timeS);
  page.replace("%VBAT%", vbatS);
  page.replace("%PCT%", pctS);
  page.replace("%EST%", estS);
  page.replace("%TEMP%", tempS);
  page.replace("%HPA%", hpaS);
  page.replace("%WIFI%", wifiS);
  page.replace("%SD%", sdS);
  page.replace("%VER%", String(FW_VERSION));
  page.replace("%UP%", String(upBuf));
  page.replace("%RST%", String(rstBuf));
  page.replace("%SUN%", String(g_sun_label) + (g_charging ? " · charging" : ""));
  if (g_lux_ok) {
    char luxBuf[20];
    snprintf(luxBuf, sizeof(luxBuf), "%.0f lx", g_lux);
    page.replace("%LUX%", String(luxBuf));
  } else {
    page.replace("%LUX%", String("--"));
  }
  if (g_ccs_data_ok) {
    const char *ew = (g_eco2 < 800) ? "good" : (g_eco2 < 1500) ? "ok" : "poor";
    const char *tw = (g_tvoc < 220) ? "good" : (g_tvoc < 660) ? "ok" : "poor";
    page.replace("%ECO2%", String((unsigned)g_eco2) + " ppm · " + ew);
    page.replace("%TVOC%", String((unsigned)g_tvoc) + " ppb · " + tw);
  } else {
    page.replace("%ECO2%", String("--"));
    page.replace("%TVOC%", String("--"));
  }
  {
    float netMa = 0, solarMa = 0, loadMa = 0, mahMin = 0;
    estimatePowerFlow(&netMa, &solarMa, &loadMa, &mahMin);
    char buf[28];
    snprintf(buf, sizeof(buf), "%+.1f mV/min", g_dv_mv_min);
    page.replace("%DV%", String(buf));
    snprintf(buf, sizeof(buf), "%d mA", (int)(solarMa + 0.5f));
    page.replace("%IN%", String(buf));
    snprintf(buf, sizeof(buf), "%d mA", (int)(loadMa + 0.5f));
    page.replace("%OUT%", String(buf));
    snprintf(buf, sizeof(buf), "%+.0f mA", netMa);
    page.replace("%NET%", String(buf));
    snprintf(buf, sizeof(buf), "%+.2f mAh/min", mahMin);
    page.replace("%MAH%", String(buf));
  }
  page.replace("%SUNT%", String((unsigned long)(g_charge_today_s / 60u)) + " min");
  page.replace("%PIC_TABLE%", buildPicTableHtml());
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", page);
}

static void handleStatus(void) {
  char runBuf[20];
  formatRuntime(runBuf, sizeof(runBuf), g_hud_pct);
  char timeBuf[20] = "NTP...";
  struct tm t;
  if (getLocalTimeValid(&t)) {
    formatTime12h(&t, timeBuf, sizeof(timeBuf));
  } else if (WiFi.status() != WL_CONNECTED) {
    snprintf(timeBuf, sizeof(timeBuf), "WiFi...");
  }
  char upBuf[20];
  formatUptime(upBuf, sizeof(upBuf));
  char rstBuf[32];
  if (g_fw_just_updated && g_update_was_ota) {
    snprintf(rstBuf, sizeof(rstBuf), "OTA ok (%s)", g_reset_tag);
  } else if (g_fw_just_updated) {
    snprintf(rstBuf, sizeof(rstBuf), "new fw (%s)", g_reset_tag);
  } else {
    snprintf(rstBuf, sizeof(rstBuf), "%s", g_reset_tag);
  }
  float netMa = 0, solarMa = 0, loadMa = 0, mahMin = 0;
  estimatePowerFlow(&netMa, &solarMa, &loadMa, &mahMin);
  char json[1200];
  const int rssi = (WiFi.status() == WL_CONNECTED) ? WiFi.RSSI() : 0;
  snprintf(json, sizeof(json),
           "{\"vbat\":%.3f,\"pct\":%d,\"est\":\"%s\",\"temp_f\":%.1f,\"temp_ok\":%s,"
           "\"hpa\":%.1f,\"press_ok\":%s,\"rh\":%.1f,\"rh_ok\":%s,\"time\":\"%s\",\"wifi\":%s,\"ip\":\"%s\","
           "\"rssi\":%d,\"screen\":%s,\"wifi_sleep\":%s,"
           "\"sd\":%s,\"sd_mb\":%llu,\"ver\":\"%s\",\"uptime\":\"%s\",\"reset\":\"%s\","
           "\"boots\":%lu,\"poweroffs\":%lu,\"faults\":%lu,\"ota_ok\":%s,"
           "\"charging\":%s,\"sun\":\"%s\",\"dv_mv_min\":%.1f,\"sun_min\":%lu,"
           "\"solar_ma\":%.0f,\"load_ma\":%.0f,\"net_ma\":%.0f,\"mah_min\":%.2f,"
           "\"lux\":%.1f,\"lux_ok\":%s,\"eco2\":%u,\"tvoc\":%u,\"ccs_ok\":%s}",
           g_hud_vbat, g_hud_pct, runBuf, g_hud_tempF, g_hud_tempOk ? "true" : "false", g_hud_hpa,
           g_hud_pressOk ? "true" : "false", g_humidity_pct, g_bme_humidity ? "true" : "false", timeBuf,
           WiFi.status() == WL_CONNECTED ? "true" : "false", WiFi.localIP().toString().c_str(), rssi,
           g_display_off ? "false" : "true", g_wifi_sleeping ? "true" : "false", g_sd_ok ? "true" : "false",
           (unsigned long long)g_sd_free_mb, FW_VERSION, upBuf, rstBuf, (unsigned long)g_boot_count,
           (unsigned long)g_poweroff_count, (unsigned long)g_fault_count,
           (g_fw_just_updated && g_update_was_ota) ? "true" : "false", g_charging ? "true" : "false",
           g_sun_label, g_dv_mv_min, (unsigned long)(g_charge_today_s / 60u), solarMa, loadMa, netMa,
           mahMin, g_lux, g_lux_ok ? "true" : "false", (unsigned)g_eco2, (unsigned)g_tvoc,
           g_ccs_data_ok ? "true" : "false");
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

static void appendJsonArray(String &out, int count, float (*sampleAt)(int), int decimals) {
  out += '[';
  for (int i = 0; i < count; i++) {
    if (i) {
      out += ',';
    }
    char b[16];
    dtostrf(sampleAt(i), 0, decimals, b);
    out += b;
  }
  out += ']';
}

static void handleGraphs(void) {
  String out;
  out.reserve(2800);
  out += "{\"vbat\":";
  appendJsonArray(out, g_sparkCount, sparkSample, 3);
  out += ",\"temp\":";
  appendJsonArray(out, g_tempHistCount, tempHistSample, 1);
  out += ",\"dv\":";
  appendJsonArray(out, g_dvHistCount, dvHistSample, 1);
  out += ",\"net\":";
  appendJsonArray(out, g_netHistCount, netHistSample, 0);
  out += ",\"lux\":";
  appendJsonArray(out, g_luxHistCount, luxHistSample, 0);
  out += ",\"eco2\":";
  appendJsonArray(out, g_eco2HistCount, eco2HistSample, 0);
  out += ",\"tvoc\":";
  appendJsonArray(out, g_tvocHistCount, tvocHistSample, 0);
  out += '}';
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", out);
}

static void handlePicGet(void) {
  if (!g_sd_ok) {
    server.send(404, "text/plain", "SD not mounted");
    return;
  }
  String name = server.arg("name");
  if (!isSafePicJpgName(name)) {
    server.send(400, "text/plain", "bad name");
    return;
  }
  String path = String(kPicsDir) + "/" + name;
  sdLock();
  if (!SD.exists(path)) {
    sdUnlock();
    server.send(404, "text/plain", "not found");
    return;
  }
  File file = SD.open(path, FILE_READ);
  if (!file) {
    sdUnlock();
    server.send(404, "text/plain", "open failed");
    return;
  }
  const size_t flen = file.size();
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.setContentLength(flen);
  server.send(200, "image/jpeg", "");
  size_t left = flen;
  while (left > 0 && file.available()) {
    const size_t chunk = (left < sizeof(s_sdGetChunk)) ? left : sizeof(s_sdGetChunk);
    const size_t n = file.read(s_sdGetChunk, chunk);
    if (n == 0) {
      break;
    }
    server.sendContent((const char *)s_sdGetChunk, n);
    left -= n;
    yield();
  }
  file.close();
  sdUnlock();
}

static void handlePicDelete(void) {
  if (!g_sd_ok) {
    server.send(403, "text/plain", "SD not mounted");
    return;
  }
  if (!server.hasArg("name")) {
    server.send(400, "text/plain", "missing name");
    return;
  }
  String name = server.arg("name");
  if (!isSafePicJpgName(name)) {
    server.send(400, "text/plain", "bad name");
    return;
  }
  String path = String(kPicsDir) + "/" + name;
  sdLock();
  if (!SD.exists(path)) {
    sdUnlock();
    server.send(404, "text/plain", "not found");
    return;
  }
  if (!SD.remove(path)) {
    sdUnlock();
    server.send(500, "text/plain", "delete failed");
    return;
  }
  sdUnlock();
  g_pic_list_dirty = true;
  if (g_view_mode == VIEW_IMAGE) {
    refreshPicList();
    if (g_pic_count == 0) {
      g_view_mode = VIEW_HUD;
    } else if (g_pic_index >= g_pic_count) {
      g_pic_index = 0;
    }
  }
  server.sendHeader("Location", "/");
  server.send(303);
}

static void streamSdTextFile(const char *path, const char *downloadName) {
  if (!g_sd_ok) {
    server.send(404, "text/plain", "SD not mounted");
    return;
  }
  sdLock();
  if (!SD.exists(path)) {
    sdUnlock();
    server.send(404, "text/plain", "log not found yet");
    return;
  }
  File file = SD.open(path, FILE_READ);
  if (!file) {
    sdUnlock();
    server.send(404, "text/plain", "open failed");
    return;
  }
  const size_t flen = file.size();
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Disposition", String("attachment; filename=\"") + downloadName + "\"");
  server.sendHeader("Connection", "close");
  server.setContentLength(flen);
  server.send(200, "text/csv", "");
  size_t left = flen;
  while (left > 0 && file.available()) {
    const size_t chunk = (left < sizeof(s_sdGetChunk)) ? left : sizeof(s_sdGetChunk);
    const size_t n = file.read(s_sdGetChunk, chunk);
    if (n == 0) {
      break;
    }
    server.sendContent((const char *)s_sdGetChunk, n);
    left -= n;
    yield();
  }
  file.close();
  sdUnlock();
}

static void handleLogEnv(void) {
  streamSdTextFile(kLogPath, "env.csv");
}

static void handleLogCharge(void) {
  streamSdTextFile(kChargePath, "charge.csv");
}

static void registerHttpHandlers(void) {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/status", HTTP_GET, handleStatus);
  server.on("/graphs", HTTP_GET, handleGraphs);
  server.on("/logs/env.csv", HTTP_GET, handleLogEnv);
  server.on("/logs/charge.csv", HTTP_GET, handleLogCharge);
  server.on("/pics/get", HTTP_GET, handlePicGet);
  server.on("/pics/delete", HTTP_POST, handlePicDelete);
}

static void onOtaStart(void) {
  g_ota_busy = true;
  noteUserActivity();
  markOtaPending(true);
  Serial.println("OTA: start");
}

static void onOtaEnd(void) {
  g_ota_busy = false;
  Serial.println("OTA: done");
}

static void onOtaError(ota_error_t err) {
  g_ota_busy = false;
  markOtaPending(false);
  Serial.printf("OTA: error %u\n", (unsigned)err);
}

static void serviceStaHttpAndMdns(void) {
  if (!g_wifi_has_ssid || WiFi.getMode() != WIFI_STA || WiFi.status() != WL_CONNECTED) {
    return;
  }
  maybeStartNtpSync();

  if (!g_sta_http_mdns_started) {
    server.stop();
    registerHttpHandlers();
    if (MDNS.begin(kMdnsDefault)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("mDNS: http://%s.local/\n", kMdnsDefault);
    }
    ArduinoOTA.setHostname(kMdnsDefault);
    ArduinoOTA.setPassword(kOtaPassword);
    ArduinoOTA.onStart(onOtaStart);
    ArduinoOTA.onEnd(onOtaEnd);
    ArduinoOTA.onError(onOtaError);
    ArduinoOTA.begin();
    server.begin();
    g_sta_http_mdns_started = true;
    Serial.print("HTTP UI: http://");
    Serial.println(WiFi.localIP());
    Serial.printf("OTA: http://%s.local:3232  pio run -e esp32s3_lcd2_ota -t upload\n", kMdnsDefault);
  }
  server.handleClient();
  ArduinoOTA.handle();
}

static void onUserButtonClick(void) {
  noteUserActivity();
  if (!g_sd_ok) {
    return;
  }
  if (g_pic_list_dirty) {
    refreshPicList();
  }
  if (g_view_mode == VIEW_HUD) {
    if (g_pic_count == 0) {
      return;
    }
    g_view_mode = VIEW_IMAGE;
    g_pic_index = 0;
    drawCurrentPic();
    return;
  }
  g_pic_index = (g_pic_index + 1) % g_pic_count;
  drawCurrentPic();
}

static void onUserButtonLongPress(void) {
  noteUserActivity();
  if (g_view_mode == VIEW_IMAGE) {
    g_view_mode = VIEW_HUD;
    gfx->fillScreen(BLACK);
    g_force_hud = true;
  }
}

static void onGraphButtonClick(void) {
  noteUserActivity();
  if (g_view_mode != VIEW_HUD) {
    return;
  }
  cycleGraphView();
  g_force_hud = true;
}

void setup(void) {
  Serial.begin(115200);
  delay(200);
  analogReadResolution(12);
  analogSetPinAttenuation(PIN_ADC_DIVIDER, ADC_11db);
  g_last_activity_ms = millis();
  recordBootStats();
  Serial.printf("FW %s (%s)  reset=%s  boots=%lu  power-offs=%lu  faults=%lu%s\n", FW_VERSION, FW_BUILD,
                g_reset_tag, (unsigned long)g_boot_count, (unsigned long)g_poweroff_count,
                (unsigned long)g_fault_count, g_fw_just_updated ? (g_update_was_ota ? "  OTA ok" : "  new fw") : "");

#ifdef GFX_EXTRA_PRE_INIT
  GFX_EXTRA_PRE_INIT();
#endif

  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  gfx->fillScreen(BLACK);
  backlightHardwareInit();

  sdMutex = xSemaphoreCreateMutex();

  delay(100);
  g_sd_ok = tryMountSd();
  if (g_sd_ok) {
    uint8_t cardType = CARD_NONE;
    sdLock();
    cardType = SD.cardType();
    sdUnlock();
    Serial.printf("SD OK: type=%s  size=%llu MB  free~%llu MB (FAT32, <=32 GB)\n",
                  esp32_s3_lcd2_sd_card_type_name(cardType),
                  (unsigned long long)(SD.cardSize() / (1024 * 1024)),
                  (unsigned long long)g_sd_free_mb);
    Serial.println("SD dirs: /LOG /PICS (created if missing)");
    refreshPicList();
  } else {
    Serial.println("SD mount failed — check FAT32 card, full insert, gfx->begin() before SD");
  }
  g_last_sd_retry_ms = millis();

  // Init sensors after LCD/SD: SPI.begin for SD can disturb I2C until Wire is started again.
  g_baro_ok = initBmp280();
  g_lux_ok = initBh1750();
  if (g_lux_ok) {
    updateLuxReading();
  }
  g_ccs_ok = initCcs811();
  if (g_ccs_ok) {
    updateCcs811Reading(25.0f, false);
  }
  {
    int err = IMU.init(calib, IMU_ADDRESS);
    g_imu_ok = (err == 0);
    if (!g_imu_ok) {
      Serial.printf("IMU init failed: %d\n", err);
    } else {
      Serial.println("QMI8658 OK — display off until tap/grab (sway ignored)");
    }
  }

  staWifiBeginFromPrefs();

  pinMode(PIN_USER_BTN, INPUT_PULLUP);
  pinMode(PIN_GRAPH_BTN, INPUT_PULLUP);
  userButton.attachClick(onUserButtonClick);
  userButton.attachLongPressStart(onUserButtonLongPress);
  userButton.setClickMs(400);
  userButton.setDebounceMs(40);
  userButton.setPressMs(800);
  graphButton.attachClick(onGraphButtonClick);
  graphButton.setClickMs(400);
  graphButton.setDebounceMs(40);

  Serial.printf("%s ready (http://%s.local/  GPIO15: pic / GPIO18: graphs)\n", kSiteName, kMdnsDefault);
}

void loop(void) {
  userButton.tick();
  graphButton.tick();
  pollImuMotion();
  updateIdleBacklight();
  updateWifiPowerSave();

  if (!g_wifi_sleeping && g_wifi_has_ssid && WiFi.getMode() == WIFI_STA &&
      WiFi.status() != WL_CONNECTED) {
    static uint32_t lastRetry = 0;
    uint32_t now = millis();
    if ((uint32_t)(now - lastRetry) > 15000u) {
      lastRetry = now;
      WiFi.reconnect();
    }
  }

  serviceStaHttpAndMdns();

  uint32_t now = millis();

  if (!g_sd_ok && (uint32_t)(now - g_last_sd_retry_ms) > 10000u) {
    g_last_sd_retry_ms = now;
    if (tryMountSd()) {
      g_sd_ok = true;
      Serial.println("SD mount OK (retry)");
      refreshPicList();
    }
  }

  if (g_view_mode == VIEW_IMAGE) {
    if (g_sd_ok && (uint32_t)(now - g_last_log_ms) >= LOG_INTERVAL_MS) {
      g_last_log_ms = now;
      float tempC = 0.0f;
      bool tempOk = false;
      float pressureHpa = 0.0f;
      bool pressOk = false;
      if (g_baro_ok && readBmp280(&tempC, &pressureHpa)) {
        tempOk = true;
        if (!isnan(pressureHpa) && pressureHpa > 0.0f) {
          pressOk = true;
        }
      }
      struct tm t;
      bool timeOk = getLocalTimeValid(&t);
#if USE_FAHRENHEIT
      float tempF = tempOk ? (tempC * 9.0f / 5.0f + 32.0f) : 0.0f;
#else
      float tempF = tempC;
#endif
      updateLuxReading();
      updateCcs811Reading(tempC, tempOk);
      float vSample = readDividerVoltageV();
      float battVema = updateBattEma(vSample);
      int battPct = batteryPctFromV(battVema);
      storeHudStats(tempF, tempOk, pressureHpa, pressOk, battVema, battPct);
      updateChargeDetect(battVema);
      tempHistMaybePush(tempF, tempOk);
      appendEnvLog(tempF, tempOk, pressureHpa, pressOk, timeOk ? &t : nullptr, timeOk, vSample);
    }
    delay(10);
    return;
  }

  if (g_ota_busy) {
    delay(5);
    return;
  }

  if (!g_force_hud && (uint32_t)(now - g_last_ui_ms) < UI_REFRESH_MS) {
    delay(10);
    return;
  }
  g_force_hud = false;
  g_last_ui_ms = now;

  float tempC = 0.0f;
  bool tempOk = false;
  float pressureHpa = 0.0f;
  bool pressOk = false;
  if (g_baro_ok) {
    if (readBmp280(&tempC, &pressureHpa)) {
      tempOk = true;
      if (!isnan(pressureHpa) && pressureHpa > 0.0f) {
        pressOk = true;
      }
    }
  }

  struct tm t;
  bool timeOk = getLocalTimeValid(&t);
  bool wifiConnected = (WiFi.status() == WL_CONNECTED);

#if USE_FAHRENHEIT
  float tempF = tempOk ? (tempC * 9.0f / 5.0f + 32.0f) : 0.0f;
#else
  float tempF = tempC;
#endif

  updateLuxReading();
  updateCcs811Reading(tempC, tempOk);
  float vSample = readDividerVoltageV();
  float battVema = updateBattEma(vSample);
  int battPct = batteryPctFromV(battVema);
  storeHudStats(tempF, tempOk, pressureHpa, pressOk, battVema, battPct);
  updateChargeDetect(battVema);
  sparkPush(battVema);
  tempHistMaybePush(tempF, tempOk);

  if (!g_display_off && !g_ota_busy) {
    drawHud(tempC, tempOk, pressureHpa, pressOk, timeOk ? &t : nullptr, timeOk, wifiConnected, g_sd_ok, battVema,
            battPct);
  }

  if (g_sd_ok && (uint32_t)(now - g_last_log_ms) >= LOG_INTERVAL_MS) {
    g_last_log_ms = now;
    appendEnvLog(tempF, tempOk, pressureHpa, pressOk, timeOk ? &t : nullptr, timeOk, vSample);
    sdLock();
    readSdFreeMbUnlocked(&g_sd_free_mb);
    sdUnlock();
  }

  char timeStr[20] = "--:--:--";
  if (timeOk) {
    formatTime12h(&t, timeStr, sizeof(timeStr));
  }
  Serial.printf("T=%s %.1fF  P=%s %.1f hPa  Vbat=%.3f (%d%%) dV=%+.1fmV/m", tempOk ? "OK" : "--", tempF,
                pressOk ? "OK" : "--", pressureHpa, battVema, battPct, g_dv_mv_min);
  if (g_lux_ok) {
    Serial.printf(" lux=%.0f", g_lux);
  }
  if (g_ccs_data_ok) {
    Serial.printf(" eCO2=%u TVOC=%u", (unsigned)g_eco2, (unsigned)g_tvoc);
  }
  Serial.printf(" sun=%s  time=%s  WiFi=%s  SD=%s\n", g_sun_label, timeStr,
                g_wifi_sleeping ? "sleep" : (wifiConnected ? "up" : "down"), g_sd_ok ? "OK" : "--");

  delay(5);
}
