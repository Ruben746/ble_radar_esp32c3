// SPDX-License-Identifier: MIT
/*
 * ============================================================================
 *  ble_radar_esp32c3.ino
 *  Radar BLE autonome — ESP32-C3 SuperMini
 *  Cible : core Arduino-ESP32 3.3.7 (ESP-IDF 5.5), pile BLE intégrée (NimBLE)
 *  Fichier unique : firmware + interface Web (HTML/CSS/JS) + API JSON
 * ============================================================================
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ESPmDNS.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "host/ble_hs.h"
#include <time.h>
#include <cmath>
#include <stdarg.h>
#include <esp_random.h>
#include "mbedtls/sha256.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR < 3)
#error "Ce firmware requiert le core Arduino-ESP32 3.x (cible : 3.3.7)"
#endif
#if !defined(CONFIG_IDF_TARGET_ESP32C3)
#warning "Firmware concu pour ESP32-C3 (SuperMini)"
#endif

// ============================================================================
//  CONFIGURATION DE COMPILATION
// ============================================================================
#define DEBUG_MODE true
#define LED_PIN 8            // ESP32-C3 SuperMini: blue onboard LED; -1 disables it
#define LED_ON_LEVEL LOW     // Active-low LED: LOW = on, HIGH = off
#define LED_OFF_LEVEL HIGH
#define LED_PULSE_MS 80UL
#define USE_MDNS 0   // mettre 0 pour gagner ~6 Ko (accès par IP uniquement)
// Mot de passe du réseau ESP-C3-XXXX (8 à 63 caractères). "" = mot de passe aléatoire généré au 1er démarrage
#define AP_DEFAULT_PASS "ChangeMe123!"
static_assert(sizeof(AP_DEFAULT_PASS) == 1 || (sizeof(AP_DEFAULT_PASS) >= 9 && sizeof(AP_DEFAULT_PASS) <= 64),
              "AP_DEFAULT_PASS doit faire 8 a 63 caracteres (ou etre vide)");
#define FW_VERSION "1.0.2"
#define CORE_TARGET "Arduino-ESP32 3.3.7"
#define CONFIG_VERSION 1

#define MAX_WATCHED 20
#define MAX_VISIBLE 50
#define MAX_KNOWN_MACS 200
#define MAX_EVENTS 30
#define MAX_SESSIONS 4
#define MAX_LOGIN_SLOTS 8

#define BLE_SCAN_SECONDS 5
#define BLE_SCAN_INTERVAL_MS 100
#define BLE_SCAN_WINDOW_MS 60
#define WATCH_Q_SIZE 16
#define GEN_Q_SIZE 32

#define WATCH_PENDING_MS 20000UL
#define WATCH_CONFIRM_WINDOW_MS 15000UL
#define FILTER_RESET_GAP_MS 30000UL
#define PROX_CONFIRM_SAMPLES 3
#define WATCH_SAMPLE_MS 250UL
#define VISIBLE_TTL_MS 180000UL

#define SESSION_IDLE_MS (30UL * 60UL * 1000UL)
#define SESSION_MAX_MS (24UL * 3600UL * 1000UL)
#define LOGIN_MAX_FAILS 5
#define PIN_HASH_ROUNDS 4096

#define WIFI_CONNECT_TIMEOUT_MS 20000UL
#define WIFI_AP_FALLBACK_MS 120000UL

#define KNOWN_SAVE_MIN_INTERVAL_MS (5UL * 60UL * 1000UL)
#define KNOWN_SAVE_BATCH 10

#define CALIB_DURATION_MS 12000UL
#define CALIB_MAX_SAMPLES 64

#define TG_QUEUE_LEN 4
#define TG_TEXT_LEN 320
#define TG_ALERT 0
#define TG_TEST 1
#define TG_SETUP_TEST 2

#define WATCH_MAGIC 0xB1E5
#define OBS_NAME 0x01
#define OBS_MFG 0x02
#define OBS_SVC 0x04

#define DEFAULT_TZ "CET-1CEST,M3.5.0,M10.5.0/3"

#if DEBUG_MODE
#define DBG(tag, fmt, ...) Serial.printf("[" tag "] " fmt "\n", ##__VA_ARGS__)
#else
#define DBG(tag, fmt, ...) do { } while (0)
#endif

// ============================================================================
//  STRUCTURES
// ============================================================================
enum WatchState : uint8_t { WS_UNKNOWN = 0, WS_PRESENT, WS_NEAR, WS_ABSENT_PENDING, WS_ABSENT };
const char* const WATCH_STATE_NAMES[] = { "UNKNOWN", "PRESENT", "NEAR", "ABSENT_PENDING", "ABSENT" };

enum WifiMgrState : uint8_t { WM_IDLE = 0, WM_CONNECTING, WM_CONNECTED, WM_WAIT_RETRY };

// Configuration persistante globale
struct RadarSettings {
  uint16_t version;
  char mdnsName[32];
  uint8_t absenceResetMin;      // 1..60
  float emaAlpha;               // 0.05..1.0
  float pathLoss;               // 1.5..4.5
  int8_t rssi1mDefault;         // -100..-20
  uint8_t appearModeDefault;    // 0 = RAPIDE, 1 = CONFIRMÉ
  uint8_t appearConfDefault;    // 1..10
  uint16_t cooldownDefault;     // 0..3600 s
  uint8_t disappearMinDefault;  // 1..60
  uint8_t unknownAlert;         // 0/1
  uint8_t unknownConf;          // 2..20
  int8_t unknownMinRssi;        // -100..-30
  uint16_t unknownWindowSec;    // 5..600
  uint8_t ignoreRandom;         // 0/1
  uint8_t unknownMaxPerHour;    // 1..60
  uint8_t txLow;                // 0/1
  char tz[48];
};

// Configuration persistante par appareil surveillé
struct WatchedConfig {
  uint16_t magic;
  uint8_t mac[6];
  char name[33];
  uint8_t active;
  uint8_t calibrated;
  int8_t rssi1m;
  float pathLoss;          // 0 = réglage global
  uint8_t alertAppear;
  uint8_t appearMode;      // 0 = RAPIDE, 1 = CONFIRMÉ
  uint8_t confirmations;   // 1..10
  uint8_t alertProx;
  float proxThreshold;     // m
  float proxHyst;          // m
  uint8_t alertDisappear;
  uint8_t disappearMin;    // 1..60
  uint8_t absenceResetMin; // 0 = réglage global
  uint16_t cooldownSec;    // 0..3600
};

// Données runtime par appareil surveillé (non persistées)
struct WatchedRuntime {
  WatchState state;
  bool everSeen;
  bool appearArmed;
  bool hasFilt;
  int8_t rawRssi;
  float filtRssi;
  uint32_t firstSeenMs;
  uint32_t lastSeenMs;
  uint32_t lastFiltMs;
  uint32_t obsCount;
  uint8_t confirmCount;
  uint32_t confirmLastMs;
  uint8_t nearCnt;
  uint8_t farCnt;
  bool hasProxAlert;
  uint32_t lastProxAlertMs;
  bool hasDisAlert;
  uint32_t lastDisAlertMs;
  char bleName[33];
};

struct WatchedDevice {
  bool used;
  WatchedConfig cfg;
  WatchedRuntime rt;
};

struct DetectedDevice {
  bool used;
  uint8_t mac[6];
  uint8_t addrType;
  char name[33];
  int8_t rawRssi;
  float filtRssi;
  bool hasFilt;
  uint32_t firstSeenMs;
  uint32_t lastSeenMs;
  uint32_t obsCount;
  uint16_t mfgId;
  bool hasMfg;
  uint8_t appleType;   // 1er octet du payload Apple (0 = aucun)
  uint16_t svc16;      // 1er UUID de service 16 bits (0 = aucun)
  char svc[37];
  int8_t watchIdx;
  bool unkChecked;
  bool known;
  bool unkDone;
  uint8_t unkCount;
  uint32_t unkFirstMs;
  uint32_t unkLastMs;
};

struct KnownMac {
  uint8_t mac[6];
  uint8_t flags;  // bit0 utilisé, bit1 notifiée, bit2 random
};

struct EventLogEntry {
  uint32_t id;
  uint32_t ms;
  char text[100];
};

struct CalibrationState {
  bool running;
  bool done;
  bool ok;
  int8_t idx;
  uint32_t startMs;
  uint8_t n;
  int8_t samples[CALIB_MAX_SAMPLES];
  int8_t result;
  char msg[96];
};

struct Session {
  bool used;
  char token[33];
  uint32_t createdMs;
  uint32_t lastMs;
};

struct LoginSlot {
  bool used;
  uint32_t ip;
  uint8_t fails;
  uint8_t lockouts;
  bool locked;
  uint32_t lockStartMs;
  uint32_t lockDurMs;
  uint32_t lastMs;
};

struct Observation {
  uint8_t mac[6];
  uint8_t addrType;
  int8_t rssi;
  uint8_t flags;
  uint16_t mfgId;
  uint8_t appleType;
  uint16_t svc16;
  uint32_t ms;
  char name[33];
  char svc[37];
};

struct ObsRing {
  Observation* buf;
  uint16_t size;
  uint16_t head;
  uint16_t tail;
  uint32_t dropped;
};

struct TgMsg {
  uint8_t kind;
  uint32_t id;
  char label[40];
  char text[TG_TEXT_LEN];
};

struct TgResult {
  uint8_t kind;
  uint32_t id;
  bool ok;
  int16_t code;
  char label[40];
  char err[72];
};

#include <esp_timer.h>

// ============================================================================
//  VARIABLES GLOBALES
// ============================================================================
WebServer server(80);
Preferences prefs;
BLEScan* g_scan = nullptr;

RadarSettings g_set;
WatchedDevice g_watch[MAX_WATCHED];
DetectedDevice g_vis[MAX_VISIBLE];
KnownMac g_known[MAX_KNOWN_MACS];
uint16_t g_knownHead = 0;
bool g_knownDirty = false;
uint16_t g_knownPending = 0;
uint32_t g_knownLastSaveMs = 0;

EventLogEntry g_events[MAX_EVENTS];
uint8_t g_eventHead = 0;
uint8_t g_eventCount = 0;
uint32_t g_eventSeq = 0;

CalibrationState g_cal;
Session g_sessions[MAX_SESSIONS];
LoginSlot g_login[MAX_LOGIN_SLOTS];

char g_wifiSsid[33] = "";
char g_wifiPass[65] = "";
char g_apSsid[20] = "";
char g_apPass[64] = "";
char g_tgToken[80] = "";
char g_tgChat[40] = "";
char g_setupTgToken[80] = "";
char g_setupTgChat[40] = "";
uint8_t g_pinSalt[16];
uint8_t g_pinHash[32];
bool g_pinSet = false;
bool g_configured = false;

portMUX_TYPE g_qMux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE g_credMux = portMUX_INITIALIZER_UNLOCKED;
Observation g_watchQBuf[WATCH_Q_SIZE];
Observation g_genQBuf[GEN_Q_SIZE];
ObsRing g_watchQ = { g_watchQBuf, WATCH_Q_SIZE, 0, 0, 0 };
ObsRing g_genQ = { g_genQBuf, GEN_Q_SIZE, 0, 0, 0 };
uint8_t g_watchMacs[MAX_WATCHED][6];
uint8_t g_watchMacCount = 0;

volatile uint32_t g_bleCycles = 0;
volatile uint32_t g_bleLastCycleMs = 0;
volatile bool g_bleStarted = false;
volatile uint32_t g_loopAliveMs = 0;

QueueHandle_t g_tgQ = nullptr;
QueueHandle_t g_tgResQ = nullptr;
uint32_t g_tgNextId = 1;
uint32_t g_tgSent = 0;
uint32_t g_tgFailed = 0;
uint32_t g_tgDropped = 0;
int8_t g_tgLast = -1;
uint32_t g_tgTestId = 0;
uint8_t g_tgTestState = 0;
char g_tgTestErr[72] = "";
uint8_t g_setupTestState = 0;
uint32_t g_setupTestStartMs = 0;
uint32_t g_setupTestId = 0;
char g_setupTestErr[72] = "";
TgMsg g_tgTmp;
TgMsg g_tgDrop;

bool g_unkHourInit = false;
uint32_t g_unkHourStartMs = 0;
uint8_t g_unkHourCount = 0;

WifiMgrState g_wm = WM_IDLE;
uint32_t g_wmSinceMs = 0;
uint32_t g_wmRetryDelay = 2000;
uint32_t g_wmDisconnectedSinceMs = 0;
bool g_apActive = false;
bool g_mdnsStarted = false;

bool g_rebootPending = false;
bool g_factoryPending = false;
uint32_t g_rebootReqMs = 0;

bool g_ledOn = false;
uint32_t g_ledOnMs = 0;

// ============================================================================
//  SORTIE JSON EN FLUX (pas de String géante, pas de fragmentation)
// ============================================================================
class JsonStream {
 public:
  void begin(int code) {
    len = 0; depth = 0; first[0] = true; afterKey = false;
    server.sendHeader("Cache-Control", "no-store");
    server.sendHeader("X-Content-Type-Options", "nosniff");
    server.setContentLength(CONTENT_LENGTH_UNKNOWN);
    server.send(code, "application/json; charset=utf-8", String());
  }
  void end() { flush(); server.sendContent("", 0); }
  void objOpen() { sep(); put('{'); push(); }
  void objClose() { put('}'); pop(); }
  void arrOpen() { sep(); put('['); push(); }
  void arrClose() { put(']'); pop(); }
  void key(const char* k) { sep(); esc(k); put(':'); afterKey = true; }
  void vStr(const char* s) { sep(); esc(s ? s : ""); }
  void vInt(long v) { sep(); char b[16]; snprintf(b, sizeof(b), "%ld", v); raw(b); }
  void vFloat(float v, int dec) {
    sep();
    if (!std::isfinite(v)) { raw("null"); return; }
    char b[24]; snprintf(b, sizeof(b), "%.*f", dec, (double)v); raw(b);
  }
  void vBool(bool v) { sep(); raw(v ? "true" : "false"); }
  void kStr(const char* k, const char* v) { key(k); vStr(v); }
  void kInt(const char* k, long v) { key(k); vInt(v); }
  void kFloat(const char* k, float v, int dec) { key(k); vFloat(v, dec); }
  void kBool(const char* k, bool v) { key(k); vBool(v); }

 private:
  char buf[768];
  size_t len = 0;
  uint8_t depth = 0;
  bool first[8];
  bool afterKey = false;
  void put(char c) { if (len >= sizeof(buf)) flush(); buf[len++] = c; }
  void raw(const char* s) { while (*s) put(*s++); }
  void flush() { if (len) { server.sendContent(buf, len); len = 0; } }
  void push() { if (depth < 7) depth++; first[depth] = true; }
  void pop() { if (depth) depth--; }
  void sep() {
    if (afterKey) { afterKey = false; return; }
    if (depth > 0) { if (!first[depth]) put(','); first[depth] = false; }
  }
  void esc(const char* s) {
    put('"');
    for (; *s; s++) {
      uint8_t c = (uint8_t)*s;
      switch (c) {
        case '"': raw("\\\""); break;
        case '\\': raw("\\\\"); break;
        case '\n': raw("\\n"); break;
        case '\r': raw("\\r"); break;
        case '\t': raw("\\t"); break;
        case '<': raw("\\u003c"); break;
        case '>': raw("\\u003e"); break;
        case '&': raw("\\u0026"); break;
        default:
          if (c < 0x20) { char b[8]; snprintf(b, sizeof(b), "\\u%04x", c); raw(b); }
          else put((char)c);
      }
    }
    put('"');
  }
};
JsonStream js;

// ============================================================================
//  UTILITAIRES
// ============================================================================
uint32_t agoMs(uint32_t t) { return (uint32_t)(millis() - t); }
bool elapsedMs(uint32_t t, uint32_t iv) { return (uint32_t)(millis() - t) >= iv; }
long agoL(uint32_t t) { uint32_t a = agoMs(t); if (a > 2000000000UL) a = 2000000000UL; return (long)a; }

int hexVal(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseMac(const char* s, uint8_t* out) {
  if (!s || strlen(s) != 17) return false;
  for (int i = 0; i < 6; i++) {
    int hi = hexVal(s[i * 3]);
    int lo = hexVal(s[i * 3 + 1]);
    if (hi < 0 || lo < 0) return false;
    if (i < 5 && s[i * 3 + 2] != ':') return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

void formatMac(const uint8_t* m, char* out) {
  snprintf(out, 18, "%02X:%02X:%02X:%02X:%02X:%02X", m[0], m[1], m[2], m[3], m[4], m[5]);
}

bool macIsZero(const uint8_t* m) {
  for (int i = 0; i < 6; i++) if (m[i]) return false;
  return true;
}

// Copie en ne gardant que de l'UTF-8 valide et imprimable (noms BLE malformés)
void sanitizeUtf8(const char* in, size_t inLen, char* out, size_t outSize, bool trim) {
  if (!out || outSize == 0) return;
  size_t o = 0, i = 0;
  if (in) {
    while (i < inLen && in[i]) {
      uint8_t c = (uint8_t)in[i];
      size_t seq = 0;
      if (c >= 0x20 && c < 0x7F) seq = 1;
      else if (c >= 0xC2 && c <= 0xDF) seq = 2;
      else if (c >= 0xE0 && c <= 0xEF) seq = 3;
      else if (c >= 0xF0 && c <= 0xF4) seq = 4;
      if (seq == 0 || i + seq > inLen) { i++; continue; }
      bool ok = true;
      for (size_t k = 1; k < seq; k++) {
        if ((((uint8_t)in[i + k]) & 0xC0) != 0x80) { ok = false; break; }
      }
      if (!ok) { i++; continue; }
      if (o + seq >= outSize) break;
      memcpy(out + o, in + i, seq);
      o += seq; i += seq;
    }
  }
  out[o] = 0;
  if (trim) {
    while (o > 0 && out[o - 1] == ' ') out[--o] = 0;
    size_t s = 0;
    while (out[s] == ' ') s++;
    if (s) memmove(out, out + s, o - s + 1);
  }
}

bool isPin4(const char* s) {
  if (!s || strlen(s) != 4) return false;
  for (int i = 0; i < 4; i++) if (s[i] < '0' || s[i] > '9') return false;
  return true;
}

bool validHostname(const char* s) {
  size_t n = strlen(s);
  if (n < 1 || n > 31) return false;
  if (s[0] == '-' || s[n - 1] == '-') return false;
  for (size_t i = 0; i < n; i++) {
    char c = s[i];
    if (!((c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-')) return false;
  }
  return true;
}

bool validTz(const char* s) {
  size_t n = strlen(s);
  if (n < 1 || n > 47) return false;
  for (size_t i = 0; i < n; i++) {
    char c = s[i];
    bool ok = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
              c == ',' || c == '.' || c == '/' || c == ':' || c == '+' || c == '-' || c == '<' || c == '>';
    if (!ok) return false;
  }
  return true;
}

bool validWifiPass(const char* s) {
  size_t n = strlen(s);
  if (n < 8 || n > 63) return false;
  for (size_t i = 0; i < n; i++) if ((uint8_t)s[i] < 0x20 || (uint8_t)s[i] > 0x7E) return false;
  return true;
}

bool validTgToken(const char* t) {
  size_t n = strlen(t);
  if (n < 30 || n > 79) return false;
  const char* c = strchr(t, ':');
  if (!c) return false;
  size_t idLen = (size_t)(c - t);
  if (idLen < 5 || idLen > 15) return false;
  for (const char* p = t; p < c; p++) if (*p < '0' || *p > '9') return false;
  if (strlen(c + 1) < 20) return false;
  for (const char* p = c + 1; *p; p++) {
    char x = *p;
    bool ok = (x >= 'A' && x <= 'Z') || (x >= 'a' && x <= 'z') || (x >= '0' && x <= '9') || x == '_' || x == '-';
    if (!ok) return false;
  }
  return true;
}

bool validChatId(const char* s) {
  size_t n = strlen(s);
  if (n == 0 || n > 32) return false;
  if (s[0] == '@') {
    if (n < 5) return false;
    for (size_t i = 1; i < n; i++) {
      char x = s[i];
      bool ok = (x >= 'A' && x <= 'Z') || (x >= 'a' && x <= 'z') || (x >= '0' && x <= '9') || x == '_';
      if (!ok) return false;
    }
    return true;
  }
  const char* p = s;
  if (*p == '-') p++;
  if (!*p) return false;
  for (; *p; p++) if (*p < '0' || *p > '9') return false;
  return true;
}

const char* addrTypeName(uint8_t t, const uint8_t* mac) {
  if (t == 0) return "Public";
  if (t == 1) {
    uint8_t top = mac[0] >> 6;
    if (top == 3) return "Random statique";
    if (top == 1) return "Random privée";
    if (top == 0) return "Random non résolvable";
    return "Random";
  }
  if (t == 2 || t == 3) return "Random (RPA)";
  return "Inconnu";
}

const char* addrTypeShort(uint8_t t) { return t == 0 ? "Public" : "Random"; }

bool timeValid() { return time(nullptr) > 1704067200; }

// Heure locale d'un instant millis() ; sinon temps depuis démarrage
void fmtClockAtMs(uint32_t ms, char* out, size_t n) {
  if (timeValid()) {
    time_t t = time(nullptr) - (time_t)(agoMs(ms) / 1000UL);
    struct tm tmv;
    localtime_r(&t, &tmv);
    strftime(out, n, "%H:%M:%S", &tmv);
  } else {
    uint32_t s = ms / 1000UL;
    snprintf(out, n, "T+%02lu:%02lu:%02lu", (unsigned long)(s / 3600UL), (unsigned long)((s / 60UL) % 60UL), (unsigned long)(s % 60UL));
  }
}

float estimateDistance(float rssi, int rssi1m, float n) {
  if (n < 1.0f) n = 2.0f;
  float d = powf(10.0f, ((float)rssi1m - rssi) / (10.0f * n));
  if (!std::isfinite(d)) d = 999.0f;
  if (d < 0.01f) d = 0.01f;
  if (d > 999.0f) d = 999.0f;
  return d;
}

void fmtDist(float d, char* out, size_t n) {
  if (d < 0) snprintf(out, n, "—");
  else if (d < 10.0f) snprintf(out, n, "≈ %.1f m", (double)d);
  else if (d < 100.0f) snprintf(out, n, "≈ %.0f m", (double)d);
  else snprintf(out, n, "> 99 m");
}

void logEvent(const char* fmt, ...) {
  EventLogEntry& e = g_events[g_eventHead];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(e.text, sizeof(e.text), fmt, ap);
  va_end(ap);
  e.id = ++g_eventSeq;
  e.ms = millis();
  g_eventHead = (uint8_t)((g_eventHead + 1) % MAX_EVENTS);
  if (g_eventCount < MAX_EVENTS) g_eventCount++;
  DBG("EVENT", "%s", e.text);
}

void ledPulse() {
#if LED_PIN >= 0
  digitalWrite(LED_PIN, LED_ON_LEVEL);
  g_ledOn = true;
  g_ledOnMs = millis();
#endif
}

void ledLoop() {
#if LED_PIN >= 0
  if (g_ledOn && elapsedMs(g_ledOnMs, LED_PULSE_MS)) {
    digitalWrite(LED_PIN, LED_OFF_LEVEL);
    g_ledOn = false;
  }
#endif
}

// ============================================================================
//  PERSISTANCE NVS
// ============================================================================
void setDefaultSettings(RadarSettings& s) {
  memset(&s, 0, sizeof(s));
  s.version = CONFIG_VERSION;
  strlcpy(s.mdnsName, "bleradar", sizeof(s.mdnsName));
  s.absenceResetMin = 5;
  s.emaAlpha = 0.25f;
  s.pathLoss = 2.0f;
  s.rssi1mDefault = -59;
  s.appearModeDefault = 0;
  s.appearConfDefault = 2;
  s.cooldownDefault = 60;
  s.disappearMinDefault = 5;
  s.unknownAlert = 0;
  s.unknownConf = 3;
  s.unknownMinRssi = -80;
  s.unknownWindowSec = 30;
  s.ignoreRandom = 1;
  s.unknownMaxPerHour = 10;
  s.txLow = 1;
  strlcpy(s.tz, DEFAULT_TZ, sizeof(s.tz));
}

void validateSettings(RadarSettings& s) {
  s.mdnsName[sizeof(s.mdnsName) - 1] = 0;
  for (char* p = s.mdnsName; *p; p++) if (*p >= 'A' && *p <= 'Z') *p = (char)(*p - 'A' + 'a');
  if (!validHostname(s.mdnsName)) strlcpy(s.mdnsName, "bleradar", sizeof(s.mdnsName));
  if (s.absenceResetMin < 1 || s.absenceResetMin > 60) s.absenceResetMin = 5;
  if (!std::isfinite(s.emaAlpha) || s.emaAlpha < 0.05f || s.emaAlpha > 1.0f) s.emaAlpha = 0.25f;
  if (!std::isfinite(s.pathLoss) || s.pathLoss < 1.5f || s.pathLoss > 4.5f) s.pathLoss = 2.0f;
  if (s.rssi1mDefault < -100 || s.rssi1mDefault > -20) s.rssi1mDefault = -59;
  if (s.appearModeDefault > 1) s.appearModeDefault = 0;
  if (s.appearConfDefault < 1 || s.appearConfDefault > 10) s.appearConfDefault = 2;
  if (s.cooldownDefault > 3600) s.cooldownDefault = 60;
  if (s.disappearMinDefault < 1 || s.disappearMinDefault > 60) s.disappearMinDefault = 5;
  s.unknownAlert = s.unknownAlert ? 1 : 0;
  if (s.unknownConf < 2 || s.unknownConf > 20) s.unknownConf = 3;
  if (s.unknownMinRssi < -100 || s.unknownMinRssi > -30) s.unknownMinRssi = -80;
  if (s.unknownWindowSec < 5 || s.unknownWindowSec > 600) s.unknownWindowSec = 30;
  s.ignoreRandom = s.ignoreRandom ? 1 : 0;
  if (s.unknownMaxPerHour < 1 || s.unknownMaxPerHour > 60) s.unknownMaxPerHour = 10;
  s.txLow = s.txLow ? 1 : 0;
  s.tz[sizeof(s.tz) - 1] = 0;
  if (!validTz(s.tz)) strlcpy(s.tz, DEFAULT_TZ, sizeof(s.tz));
  s.version = CONFIG_VERSION;
}

void saveSettings() {
  g_set.version = CONFIG_VERSION;
  prefs.putBytes("cfg", &g_set, sizeof(g_set));
  DBG("NVS", "paramètres enregistrés");
}

bool commitSettings(const RadarSettings& ns) {
  if (memcmp(&ns, &g_set, sizeof(ns)) == 0) return false;
  g_set = ns;
  saveSettings();
  return true;
}

bool validateWatch(WatchedConfig& c) {
  if (c.magic != WATCH_MAGIC || macIsZero(c.mac)) return false;
  c.name[sizeof(c.name) - 1] = 0;
  char tmp[33];
  sanitizeUtf8(c.name, strlen(c.name), tmp, sizeof(tmp), true);
  strlcpy(c.name, tmp[0] ? tmp : "Appareil", sizeof(c.name));
  c.active = c.active ? 1 : 0;
  c.calibrated = c.calibrated ? 1 : 0;
  if (c.rssi1m < -100 || c.rssi1m > -20) { c.rssi1m = -59; c.calibrated = 0; }
  if (!std::isfinite(c.pathLoss) || (c.pathLoss != 0.0f && (c.pathLoss < 1.5f || c.pathLoss > 4.5f))) c.pathLoss = 0.0f;
  c.alertAppear = c.alertAppear ? 1 : 0;
  c.appearMode = c.appearMode ? 1 : 0;
  if (c.confirmations < 1 || c.confirmations > 10) c.confirmations = 2;
  c.alertProx = c.alertProx ? 1 : 0;
  if (!std::isfinite(c.proxThreshold) || c.proxThreshold < 0.3f || c.proxThreshold > 30.0f) c.proxThreshold = 3.0f;
  if (!std::isfinite(c.proxHyst) || c.proxHyst < 0.2f || c.proxHyst > 20.0f) c.proxHyst = 1.0f;
  c.alertDisappear = c.alertDisappear ? 1 : 0;
  if (c.disappearMin < 1 || c.disappearMin > 60) c.disappearMin = 5;
  if (c.absenceResetMin > 60) c.absenceResetMin = 0;
  if (c.cooldownSec > 3600) c.cooldownSec = 60;
  return true;
}

void watchKey(int i, char* key) { snprintf(key, 6, "w%02d", i); }

void saveWatch(int i) {
  char key[6];
  watchKey(i, key);
  prefs.putBytes(key, &g_watch[i].cfg, sizeof(WatchedConfig));
  DBG("NVS", "appareil %d enregistré (%s)", i, g_watch[i].cfg.name);
}

void removeWatchKey(int i) {
  char key[6];
  watchKey(i, key);
  if (prefs.isKey(key)) prefs.remove(key);
  DBG("NVS", "appareil %d supprimé", i);
}

bool loadStr(const char* key, char* out, size_t n) {
  out[0] = 0;
  if (!prefs.isKey(key)) return false;
  size_t l = prefs.getString(key, out, n);
  out[n - 1] = 0;
  if (l == 0) out[0] = 0;
  return l > 0;
}

void initWatchRuntime(WatchedRuntime& r) {
  memset(&r, 0, sizeof(r));
  r.state = WS_UNKNOWN;
  r.appearArmed = true;
}

int findWatchIdx(const uint8_t* mac) {
  for (int i = 0; i < MAX_WATCHED; i++) {
    if (g_watch[i].used && memcmp(g_watch[i].cfg.mac, mac, 6) == 0) return i;
  }
  return -1;
}

void rebuildWatchMacCache() {
  portENTER_CRITICAL(&g_qMux);
  uint8_t n = 0;
  for (int i = 0; i < MAX_WATCHED; i++) {
    if (g_watch[i].used) { memcpy(g_watchMacs[n], g_watch[i].cfg.mac, 6); n++; }
  }
  g_watchMacCount = n;
  portEXIT_CRITICAL(&g_qMux);
  for (int i = 0; i < MAX_VISIBLE; i++) {
    if (g_vis[i].used) g_vis[i].watchIdx = (int8_t)findWatchIdx(g_vis[i].mac);
  }
}

int knownFind(const uint8_t* mac) {
  for (int i = 0; i < MAX_KNOWN_MACS; i++) {
    if ((g_known[i].flags & 1) && memcmp(g_known[i].mac, mac, 6) == 0) return i;
  }
  return -1;
}

int knownCount() {
  int n = 0;
  for (int i = 0; i < MAX_KNOWN_MACS; i++) if (g_known[i].flags & 1) n++;
  return n;
}

void knownAdd(const uint8_t* mac, bool isRandom, bool notified) {
  int idx = knownFind(mac);
  if (idx >= 0) {
    if (notified && !(g_known[idx].flags & 2)) { g_known[idx].flags |= 2; g_knownDirty = true; g_knownPending++; }
    return;
  }
  KnownMac& k = g_known[g_knownHead];
  memcpy(k.mac, mac, 6);
  k.flags = (uint8_t)(1 | (notified ? 2 : 0) | (isRandom ? 4 : 0));
  g_knownHead = (uint16_t)((g_knownHead + 1) % MAX_KNOWN_MACS);
  g_knownDirty = true;
  g_knownPending++;
}

void knownSave(bool force) {
  if (!g_knownDirty) return;
  if (!force && g_knownPending < KNOWN_SAVE_BATCH && !elapsedMs(g_knownLastSaveMs, KNOWN_SAVE_MIN_INTERVAL_MS)) return;
  prefs.putBytes("kmac", g_known, sizeof(g_known));
  prefs.putUShort("khead", g_knownHead);
  g_knownDirty = false;
  g_knownPending = 0;
  g_knownLastSaveMs = millis();
  DBG("NVS", "cache MAC enregistré (%d entrées)", knownCount());
}

void loadAll() {
  uint16_t ver = prefs.isKey("ver") ? prefs.getUShort("ver", 0) : 0;
  bool verOk = (ver == CONFIG_VERSION);
  setDefaultSettings(g_set);

  if (!verOk) {
    if (ver != 0) DBG("NVS", "version %u incompatible : paramètres, appareils et cache réinitialisés", (unsigned)ver);
    if (prefs.isKey("cfg")) prefs.remove("cfg");
    for (int i = 0; i < MAX_WATCHED; i++) removeWatchKey(i);
    if (prefs.isKey("kmac")) prefs.remove("kmac");
    if (prefs.isKey("khead")) prefs.remove("khead");
    saveSettings();
    prefs.putUShort("ver", CONFIG_VERSION);
  } else {
    bool ok = false;
    if (prefs.isKey("cfg") && prefs.getBytesLength("cfg") == sizeof(RadarSettings)) {
      RadarSettings tmp;
      prefs.getBytes("cfg", &tmp, sizeof(tmp));
      if (tmp.version == CONFIG_VERSION) { validateSettings(tmp); g_set = tmp; ok = true; }
    }
    if (!ok) { DBG("NVS", "paramètres invalides : valeurs par défaut"); saveSettings(); }

    for (int i = 0; i < MAX_WATCHED; i++) {
      char key[6];
      watchKey(i, key);
      if (!prefs.isKey(key)) continue;
      WatchedConfig c;
      bool good = false;
      if (prefs.getBytesLength(key) == sizeof(WatchedConfig)) {
        prefs.getBytes(key, &c, sizeof(c));
        good = validateWatch(c) && findWatchIdx(c.mac) < 0;
      }
      if (good) {
        g_watch[i].used = true;
        g_watch[i].cfg = c;
        initWatchRuntime(g_watch[i].rt);
      } else {
        DBG("NVS", "entrée %s invalide supprimée", key);
        prefs.remove(key);
      }
    }

    if (prefs.isKey("kmac") && prefs.getBytesLength("kmac") == sizeof(g_known)) {
      prefs.getBytes("kmac", g_known, sizeof(g_known));
      g_knownHead = (uint16_t)(prefs.getUShort("khead", 0) % MAX_KNOWN_MACS);
      for (int i = 0; i < MAX_KNOWN_MACS; i++) {
        if ((g_known[i].flags & 1) && macIsZero(g_known[i].mac)) g_known[i].flags = 0;
        g_known[i].flags &= 0x07;
      }
    }
  }

  loadStr("wssid", g_wifiSsid, sizeof(g_wifiSsid));
  loadStr("wpass", g_wifiPass, sizeof(g_wifiPass));
  loadStr("tgtok", g_tgToken, sizeof(g_tgToken));
  loadStr("tgchat", g_tgChat, sizeof(g_tgChat));
  if (g_tgToken[0] && !validTgToken(g_tgToken)) g_tgToken[0] = 0;
  if (g_tgChat[0] && !validChatId(g_tgChat)) g_tgChat[0] = 0;

  g_pinSet = prefs.isKey("psalt") && prefs.isKey("phash") &&
             prefs.getBytesLength("psalt") == 16 && prefs.getBytesLength("phash") == 32;
  if (g_pinSet) {
    prefs.getBytes("psalt", g_pinSalt, 16);
    prefs.getBytes("phash", g_pinHash, 32);
  }

  bool setupFlag = prefs.isKey("setup") && prefs.getBool("setup", false);
  g_configured = setupFlag && g_wifiSsid[0] && g_pinSet;
  if (setupFlag && !g_configured) DBG("NVS", "configuration incomplète : retour au mode configuration");
  g_knownLastSaveMs = millis();

  int n = 0;
  for (int i = 0; i < MAX_WATCHED; i++) if (g_watch[i].used) n++;
  DBG("NVS", "chargé : configuré=%d, appareils=%d, MAC connues=%d", g_configured ? 1 : 0, n, knownCount());
}

// ============================================================================
//  SÉCURITÉ : PIN, SESSIONS, ANTI-BRUTEFORCE
// ============================================================================
void randomHex(char* out, size_t bytes) {
  static const char hx[] = "0123456789abcdef";
  for (size_t i = 0; i < bytes; i++) {
    uint8_t b = (uint8_t)(esp_random() & 0xFF);
    out[i * 2] = hx[b >> 4];
    out[i * 2 + 1] = hx[b & 15];
  }
  out[bytes * 2] = 0;
}

void genPassword(char* out, size_t len) {
  static const char cs[] = "abcdefghjkmnpqrstuvwxyzABCDEFGHJKLMNPQRSTUVWXYZ23456789";
  for (size_t i = 0; i < len; i++) out[i] = cs[esp_random() % (sizeof(cs) - 1)];
  out[len] = 0;
}

void buildApSsid() {
  uint64_t em = ESP.getEfuseMac();
  snprintf(g_apSsid, sizeof(g_apSsid), "ESP-C3-%02X%02X",
           (unsigned)((em >> 32) & 0xFF), (unsigned)((em >> 40) & 0xFF));
}

void pinDerive(const char* pin, const uint8_t* salt, uint8_t* out) {
  uint8_t buf[64];
  memcpy(buf, salt, 16);
  memcpy(buf + 16, pin, 4);
  mbedtls_sha256(buf, 20, out, 0);
  for (int i = 0; i < PIN_HASH_ROUNDS; i++) {
    memcpy(buf, out, 32);
    memcpy(buf + 32, salt, 16);
    mbedtls_sha256(buf, 48, out, 0);
  }
  memset(buf, 0, sizeof(buf));
}

bool pinVerify(const char* pin) {
  if (!g_pinSet || !isPin4(pin)) return false;
  uint8_t h[32];
  pinDerive(pin, g_pinSalt, h);
  uint8_t diff = 0;
  for (int i = 0; i < 32; i++) diff |= (uint8_t)(h[i] ^ g_pinHash[i]);
  return diff == 0;
}

void pinSetNew(const char* pin) {
  for (int i = 0; i < 16; i += 4) {
    uint32_t r = esp_random();
    memcpy(g_pinSalt + i, &r, 4);
  }
  pinDerive(pin, g_pinSalt, g_pinHash);
  g_pinSet = true;
  prefs.putBytes("psalt", g_pinSalt, 16);
  prefs.putBytes("phash", g_pinHash, 32);
  DBG("SECURITY", "PIN enregistré (hash salé)");
}

bool ctEq(const char* a, const char* b, size_t n) {
  uint8_t d = 0;
  for (size_t i = 0; i < n; i++) d |= (uint8_t)(a[i] ^ b[i]);
  return d == 0;
}

uint32_t clientIp() {
  IPAddress a = server.client().remoteIP();
  return ((uint32_t)a[0] << 24) | ((uint32_t)a[1] << 16) | ((uint32_t)a[2] << 8) | (uint32_t)a[3];
}

LoginSlot* loginSlot(uint32_t ip) {
  LoginSlot* freeS = nullptr;
  LoginSlot* oldest = nullptr;
  uint32_t oa = 0;
  for (int i = 0; i < MAX_LOGIN_SLOTS; i++) {
    LoginSlot& s = g_login[i];
    if (s.used && s.ip == ip) return &s;
    if (!s.used) { if (!freeS) freeS = &s; continue; }
    uint32_t a = agoMs(s.lastMs);
    if (!oldest || a > oa) { oldest = &s; oa = a; }
  }
  LoginSlot* s = freeS ? freeS : oldest;
  memset(s, 0, sizeof(*s));
  s->used = true;
  s->ip = ip;
  s->lastMs = millis();
  return s;
}

uint32_t loginLockRemaining(LoginSlot* s) {
  if (!s->locked) return 0;
  uint32_t a = agoMs(s->lockStartMs);
  if (a >= s->lockDurMs) { s->locked = false; return 0; }
  return s->lockDurMs - a;
}

void loginFail(LoginSlot* s) {
  s->lastMs = millis();
  s->fails++;
  DBG("SECURITY", "PIN incorrect (%u/%d)", s->fails, LOGIN_MAX_FAILS);
  if (s->fails >= LOGIN_MAX_FAILS) {
    s->fails = 0;
    uint8_t k = s->lockouts < 4 ? s->lockouts : 4;
    uint32_t dur = 60000UL << k;
    if (dur > 900000UL) dur = 900000UL;
    s->lockDurMs = dur;
    s->lockStartMs = millis();
    s->locked = true;
    if (s->lockouts < 250) s->lockouts++;
    logEvent("Sécurité : trop d'essais PIN, blocage %lu s", (unsigned long)(dur / 1000UL));
  }
}

void loginSuccess(LoginSlot* s) {
  s->fails = 0;
  s->lockouts = 0;
  s->locked = false;
  s->lastMs = millis();
}

bool readSessionCookie(char* out, size_t n) {
  if (!server.hasHeader("Cookie")) return false;
  String c = server.header("Cookie");
  int idx = 0;
  int len = (int)c.length();
  while (idx < len) {
    int semi = c.indexOf(';', idx);
    if (semi < 0) semi = len;
    String part = c.substring(idx, semi);
    part.trim();
    if (part.startsWith("sid=")) {
      String v = part.substring(4);
      if (v.length() == 32 && n > 32) { strlcpy(out, v.c_str(), n); return true; }
    }
    idx = semi + 1;
  }
  return false;
}

Session* sessionFind(const char* token) {
  for (int i = 0; i < MAX_SESSIONS; i++) {
    Session& s = g_sessions[i];
    if (!s.used) continue;
    if (!ctEq(token, s.token, 32)) continue;
    if (agoMs(s.lastMs) > SESSION_IDLE_MS || agoMs(s.createdMs) > SESSION_MAX_MS) { s.used = false; return nullptr; }
    return &s;
  }
  return nullptr;
}

Session* sessionCreate() {
  Session* s = nullptr;
  for (int i = 0; i < MAX_SESSIONS; i++) if (!g_sessions[i].used) { s = &g_sessions[i]; break; }
  if (!s) {
    uint32_t oa = 0;
    for (int i = 0; i < MAX_SESSIONS; i++) {
      uint32_t a = agoMs(g_sessions[i].lastMs);
      if (!s || a > oa) { s = &g_sessions[i]; oa = a; }
    }
  }
  s->used = true;
  randomHex(s->token, 16);
  s->createdMs = millis();
  s->lastMs = s->createdMs;
  return s;
}

void sessionsExpire() {
  for (int i = 0; i < MAX_SESSIONS; i++) {
    Session& s = g_sessions[i];
    if (s.used && (agoMs(s.lastMs) > SESSION_IDLE_MS || agoMs(s.createdMs) > SESSION_MAX_MS)) s.used = false;
  }
}

Session* currentSession() {
  char tok[40];
  if (!readSessionCookie(tok, sizeof(tok))) return nullptr;
  return sessionFind(tok);
}

bool authOk() {
  Session* s = currentSession();
  if (!s) return false;
  s->lastMs = millis();
  return true;
}

// ============================================================================
//  TELEGRAM
// ============================================================================
size_t appendStr(char* out, size_t pos, size_t cap, const char* s) {
  while (*s && pos + 1 < cap) out[pos++] = *s++;
  out[pos] = 0;
  return pos;
}

size_t urlEncodeAppend(char* out, size_t pos, size_t cap, const char* s) {
  static const char hx[] = "0123456789ABCDEF";
  for (; *s; s++) {
    uint8_t c = (uint8_t)*s;
    bool safe = (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') ||
                c == '-' || c == '_' || c == '.' || c == '~';
    if (safe) {
      if (pos + 1 >= cap) break;
      out[pos++] = (char)c;
    } else {
      if (pos + 3 >= cap) break;
      out[pos++] = '%';
      out[pos++] = hx[c >> 4];
      out[pos++] = hx[c & 15];
    }
  }
  out[pos] = 0;
  return pos;
}

bool tgConfigured() { return g_tgToken[0] && g_tgChat[0]; }

uint32_t tgEnqueue(uint8_t kind, const char* label, const char* text) {
  if (!g_tgQ) return 0;
  memset(&g_tgTmp, 0, sizeof(g_tgTmp));
  g_tgTmp.kind = kind;
  g_tgTmp.id = g_tgNextId++;
  if (g_tgNextId == 0) g_tgNextId = 1;
  sanitizeUtf8(label, strlen(label), g_tgTmp.label, sizeof(g_tgTmp.label), true);
  strlcpy(g_tgTmp.text, text, sizeof(g_tgTmp.text));
  if (xQueueSend(g_tgQ, &g_tgTmp, 0) != pdTRUE) {
    if (xQueueReceive(g_tgQ, &g_tgDrop, 0) == pdTRUE) {
      g_tgDropped++;
      logEvent("Telegram : file pleine, plus ancien message abandonné");
    }
    if (xQueueSend(g_tgQ, &g_tgTmp, 0) != pdTRUE) { logEvent("Telegram : file indisponible"); return 0; }
  }
  DBG("TELEGRAM", "en file : %s", g_tgTmp.label);
  return g_tgTmp.id;
}

// Exécuté UNIQUEMENT dans la tâche Telegram
int tgHttpSend(const char* token, const char* chat, const char* text, char* err, size_t errN) {
  if (!token[0] || !chat[0]) { strlcpy(err, "Telegram non configuré", errN); return -100; }
  uint32_t fh = ESP.getFreeHeap();
  uint32_t mb = ESP.getMaxAllocHeap();
  if (fh < 30000 || mb < 18000) {
    snprintf(err, errN, "Mémoire insuffisante pour TLS (libre %lu o, bloc max %lu o)", (unsigned long)fh, (unsigned long)mb);
    return -101;
  }
    IPAddress tgIp;
  if (!WiFi.hostByName("api.telegram.org", tgIp)) {
    strlcpy(err, "DNS : api.telegram.org introuvable", errN);
    return -103;
  }
  static char body[TG_TEXT_LEN * 3 + 200];
  size_t p = 0;
  body[0] = 0;
  p = appendStr(body, p, sizeof(body), "chat_id=");
  p = urlEncodeAppend(body, p, sizeof(body), chat);
  p = appendStr(body, p, sizeof(body), "&disable_web_page_preview=true&text=");
  p = urlEncodeAppend(body, p, sizeof(body), text);

  char url[140];
  snprintf(url, sizeof(url), "https://api.telegram.org/bot%s/sendMessage", token);

  WiFiClientSecure client;
  client.setInsecure();
  client.setHandshakeTimeout(12);
  HTTPClient http;
  http.setConnectTimeout(7000);
  http.setTimeout(9000);
  http.setReuse(false);
  if (!http.begin(client, url)) { strlcpy(err, "Initialisation HTTPS impossible", errN); return -102; }
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  int code = http.POST((uint8_t*)body, p);
  if (code <= 0) {
    snprintf(err, errN, "%s", HTTPClient::errorToString(code).c_str());
    char tls[80] = "";
    client.lastError(tls, sizeof(tls));
    DBG("TELEGRAM", "détail : %s | TLS=%s | heap=%lu bloc max=%lu",
        err, tls, (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
  } else if (code != 200) {
    String resp = http.getString();
    int k = resp.indexOf("\"description\":\"");
    if (k >= 0) {
      k += 15;
      int e = resp.indexOf('"', k);
      if (e < 0) e = resp.length();
      String dsc = resp.substring(k, e);
      snprintf(err, errN, "HTTP %d : %s", code, dsc.c_str());
    } else {
      snprintf(err, errN, "HTTP %d", code);
    }
  }
  http.end();
  return code;
}

void telegramTask(void* arg) {
  (void)arg;
  static TgMsg m;
  static TgResult res;
  char token[80];
  char chat[40];
  char err[72];
  for (;;) {
    if (xQueueReceive(g_tgQ, &m, pdMS_TO_TICKS(1000)) != pdTRUE) continue;
    memset(&res, 0, sizeof(res));
    res.kind = m.kind;
    res.id = m.id;
    strlcpy(res.label, m.label, sizeof(res.label));
    uint32_t start = millis();
    uint8_t attempts = 0;
    for (;;) {
      if (WiFi.status() != WL_CONNECTED) {
        uint32_t maxWait = (m.kind == TG_ALERT) ? 600000UL : 25000UL;
        if (agoMs(start) > maxWait) { res.ok = false; res.code = -1; strlcpy(res.err, "Wi-Fi indisponible", sizeof(res.err)); break; }
        vTaskDelay(pdMS_TO_TICKS(2000));
        continue;
      }
      portENTER_CRITICAL(&g_credMux);
      if (m.kind == TG_SETUP_TEST) {
        strlcpy(token, g_setupTgToken, sizeof(token));
        strlcpy(chat, g_setupTgChat, sizeof(chat));
      } else {
        strlcpy(token, g_tgToken, sizeof(token));
        strlcpy(chat, g_tgChat, sizeof(chat));
      }
      portEXIT_CRITICAL(&g_credMux);
      err[0] = 0;
      int code = tgHttpSend(token, chat, m.text, err, sizeof(err));
      attempts++;
      if (code == 200) { res.ok = true; res.code = 200; break; }
      res.code = (int16_t)code;
      strlcpy(res.err, err, sizeof(res.err));
      bool permanent = (code == 400 || code == 401 || code == 403 || code == 404 || code == -100);
      if (permanent || attempts >= 3) { res.ok = false; break; }
      DBG("TELEGRAM", "échec (%s), nouvel essai %u/3", err, attempts + 1);
      vTaskDelay(pdMS_TO_TICKS(code == 429 ? 8000UL : 2500UL * attempts));
    }
    memset(token, 0, sizeof(token));
    DBG("TELEGRAM", "%s : %s", res.ok ? "sent" : "failed", res.label);
    xQueueSend(g_tgResQ, &res, pdMS_TO_TICKS(200));
    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

void drainTelegramResults() {
  if (!g_tgResQ) return;
  TgResult r;
  while (xQueueReceive(g_tgResQ, &r, 0) == pdTRUE) {
    if (r.ok) { g_tgSent++; g_tgLast = 1; logEvent("Telegram envoyé : %s", r.label); }
    else { g_tgFailed++; g_tgLast = 0; logEvent("Telegram échec (%s) : %s", r.label, r.err); }
    if (r.kind == TG_TEST && r.id == g_tgTestId) {
      g_tgTestState = r.ok ? 2 : 3;
      strlcpy(g_tgTestErr, r.ok ? "" : r.err, sizeof(g_tgTestErr));
    }
    if (r.kind == TG_SETUP_TEST && r.id == g_setupTestId) {
      g_setupTestState = r.ok ? 3 : 4;
      strlcpy(g_setupTestErr, r.ok ? "" : r.err, sizeof(g_setupTestErr));
    }
  }
}

// ============================================================================
//  ALERTES
// ============================================================================
int watchR1(const WatchedDevice& w) { return w.cfg.calibrated ? w.cfg.rssi1m : g_set.rssi1mDefault; }
float watchN(const WatchedDevice& w) { return w.cfg.pathLoss >= 1.5f ? w.cfg.pathLoss : g_set.pathLoss; }
float watchDistance(const WatchedDevice& w) {
  if (!w.rt.hasFilt) return -1.0f;
  return estimateDistance(w.rt.filtRssi, watchR1(w), watchN(w));
}

bool sendAlert(const char* type, const char* name, const char* txt) {
  if (!tgConfigured()) {
    logEvent("Alerte %s non envoyée : Telegram non configuré", type);
    return false;
  }
  char label[40];
  snprintf(label, sizeof(label), "%s %s", type, name);
  return tgEnqueue(TG_ALERT, label, txt) != 0;
}

void alertAppearance(int i) {
  WatchedDevice& w = g_watch[i];
  char mac[18], clk[20], dist[20], txt[TG_TEXT_LEN];
  formatMac(w.cfg.mac, mac);
  fmtClockAtMs(w.rt.lastSeenMs, clk, sizeof(clk));
  fmtDist(watchDistance(w), dist, sizeof(dist));
  snprintf(txt, sizeof(txt),
           "🟢 Appareil détecté\n\n%s\n\nMAC : %s\nRSSI : %d dBm\nDistance estimée : %s\nHeure : %s",
           w.cfg.name, mac, (int)w.rt.rawRssi, dist, clk);
  if (sendAlert("apparition", w.cfg.name, txt)) DBG("TELEGRAM", "appearance alert queued");
}

void alertProximity(int i, float d) {
  WatchedDevice& w = g_watch[i];
  char clk[20], dist[20], txt[TG_TEXT_LEN];
  fmtClockAtMs(w.rt.lastSeenMs, clk, sizeof(clk));
  fmtDist(d, dist, sizeof(dist));
  snprintf(txt, sizeof(txt),
           "📍 Appareil à proximité\n\n%s\n\nDistance estimée : %s\nSeuil : %.1f m\nRSSI : %d dBm\nHeure : %s",
           w.cfg.name, dist, (double)w.cfg.proxThreshold, (int)lroundf(w.rt.filtRssi), clk);
  if (sendAlert("proximité", w.cfg.name, txt)) DBG("TELEGRAM", "proximity alert queued");
}

void alertDisappearance(int i) {
  WatchedDevice& w = g_watch[i];
  char mac[18], clk[20], txt[TG_TEXT_LEN];
  formatMac(w.cfg.mac, mac);
  fmtClockAtMs(w.rt.lastSeenMs, clk, sizeof(clk));
  unsigned long mins = (unsigned long)(agoMs(w.rt.lastSeenMs) / 60000UL);
  snprintf(txt, sizeof(txt),
           "🔴 Appareil non détecté\n\n%s\nMAC : %s\nDernière détection : %s\nAbsent depuis : %lu min",
           w.cfg.name, mac, clk, mins);
  if (sendAlert("disparition", w.cfg.name, txt)) DBG("TELEGRAM", "disappearance alert queued");
}

bool unknownRateOk() {
  if (!g_unkHourInit || elapsedMs(g_unkHourStartMs, 3600000UL)) {
    g_unkHourInit = true;
    g_unkHourStartMs = millis();
    g_unkHourCount = 0;
  }
  if (g_unkHourCount >= g_set.unknownMaxPerHour) return false;
  g_unkHourCount++;
  return true;
}

bool alertUnknown(const DetectedDevice& d) {
  if (!tgConfigured()) return false;
  if (!unknownRateOk()) {
    logEvent("Alerte nouvelle MAC limitée (%u/h)", (unsigned)g_set.unknownMaxPerHour);
    return false;
  }
  char mac[18], clk[20], dist[20], txt[TG_TEXT_LEN], label[40];
  formatMac(d.mac, mac);
  fmtClockAtMs(d.lastSeenMs, clk, sizeof(clk));
  fmtDist(estimateDistance(d.filtRssi, g_set.rssi1mDefault, g_set.pathLoss), dist, sizeof(dist));
  snprintf(txt, sizeof(txt),
           "⚠️ Nouvelle adresse BLE détectée\n\nNom : %s\nMAC : %s\nType : %s\nRSSI : %d dBm\nDistance estimée : %s\nHeure : %s",
           d.name[0] ? d.name : "Inconnu", mac, addrTypeShort(d.addrType), (int)d.rawRssi, dist, clk);
  snprintf(label, sizeof(label), "nouvelle MAC %s", mac);
  return tgEnqueue(TG_ALERT, label, txt) != 0;
}

// ============================================================================
//  BLE : FILES D'OBSERVATIONS, CALLBACK, TÂCHE DE SCAN
// ============================================================================
bool isWatchedMacFast(const uint8_t* mac) {
  bool found = false;
  portENTER_CRITICAL(&g_qMux);
  for (uint8_t i = 0; i < g_watchMacCount; i++) {
    if (memcmp(g_watchMacs[i], mac, 6) == 0) { found = true; break; }
  }
  portEXIT_CRITICAL(&g_qMux);
  return found;
}

bool obsPush(ObsRing& q, const Observation& o) {
  bool ok;
  portENTER_CRITICAL(&g_qMux);
  uint16_t next = (uint16_t)((q.head + 1) % q.size);
  if (next == q.tail) { q.dropped++; ok = false; }
  else { q.buf[q.head] = o; q.head = next; ok = true; }
  portEXIT_CRITICAL(&g_qMux);
  return ok;
}

bool obsPop(ObsRing& q, Observation& o) {
  bool ok = false;
  portENTER_CRITICAL(&g_qMux);
  if (q.tail != q.head) {
    o = q.buf[q.tail];
    q.tail = (uint16_t)((q.tail + 1) % q.size);
    ok = true;
  }
  portEXIT_CRITICAL(&g_qMux);
  return ok;
}

uint32_t obsDropped() {
  portENTER_CRITICAL(&g_qMux);
  uint32_t d = g_watchQ.dropped + g_genQ.dropped;
  portEXIT_CRITICAL(&g_qMux);
  return d;
}

// ----------------------------------------------------------------------------
//  Scan BLE direct via l'API GAP NimBLE (sans BLEScan) :
//  aucune allocation par paquet, aucune structure partagée entre tâches
// ----------------------------------------------------------------------------
volatile bool g_bleScanning = false;
volatile uint32_t g_bleLastAdvMs = 0;
volatile uint32_t g_bleStartReqMs = 0;

// Décodage des structures AD : [longueur][type][données]
void parseAdvData(const uint8_t* p, size_t len, Observation& o) {
  size_t i = 0;
  while (i + 1 < len) {
    uint8_t l = p[i];
    if (l == 0 || i + 1 + l > len) break;
    uint8_t t = p[i + 1];
    const uint8_t* d = p + i + 2;
    size_t dl = (size_t)l - 1;
    if ((t == 0x09 || (t == 0x08 && !(o.flags & OBS_NAME))) && dl > 0) {
      sanitizeUtf8((const char*)d, dl, o.name, sizeof(o.name), true);
      if (o.name[0]) o.flags |= OBS_NAME;
    } else if (t == 0xFF && dl >= 2 && !(o.flags & OBS_MFG)) {
      o.mfgId = (uint16_t)(d[0] | (d[1] << 8));
      if (o.mfgId == 0x004C && dl >= 3) o.appleType = d[2];  // 1er octet du message Apple
      o.flags |= OBS_MFG;
    } else if (!(o.flags & OBS_SVC)) {
      if ((t == 0x02 || t == 0x03) && dl >= 2) {
        o.svc16 = (uint16_t)(d[0] | (d[1] << 8));
        snprintf(o.svc, sizeof(o.svc), "%04X", (unsigned)o.svc16);
        o.flags |= OBS_SVC;
      } else if ((t == 0x04 || t == 0x05) && dl >= 4) {
        uint32_t u = (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
        snprintf(o.svc, sizeof(o.svc), "%08lX", (unsigned long)u);
        o.flags |= OBS_SVC;
      } else if ((t == 0x06 || t == 0x07) && dl >= 16) {
        static const char hx[] = "0123456789abcdef";
        int k = 0;
        for (int b = 15; b >= 0; b--) {
          o.svc[k++] = hx[d[b] >> 4];
          o.svc[k++] = hx[d[b] & 15];
          if (b == 12 || b == 10 || b == 8 || b == 6) o.svc[k++] = '-';
        }
        o.svc[k] = 0;
        o.flags |= OBS_SVC;
      }
    }
    i += 1 + (size_t)l;
  }
}

// Exécuté dans la tâche hôte NimBLE : court, sans allocation
void handleAdvReport(const ble_addr_t& a, int8_t rssi, const uint8_t* data, uint8_t len) {
  Observation o;
  memset(&o, 0, sizeof(o));
  for (int k = 0; k < 6; k++) o.mac[k] = a.val[5 - k];  // NimBLE stocke l'adresse octet de poids faible en premier
  o.addrType = a.type;
  o.rssi = rssi;
  o.ms = millis();
  if (data && len) parseAdvData(data, len, o);
  g_bleLastAdvMs = o.ms;
  if (isWatchedMacFast(o.mac)) obsPush(g_watchQ, o);
  else obsPush(g_genQ, o);
}

int radarGapEvent(struct ble_gap_event* ev, void* arg) {
  (void)arg;
  switch (ev->type) {
    case BLE_GAP_EVENT_DISC:
      handleAdvReport(ev->disc.addr, ev->disc.rssi, ev->disc.data, ev->disc.length_data);
      break;
#if MYNEWT_VAL(BLE_EXT_ADV)
    case BLE_GAP_EVENT_EXT_DISC:
      handleAdvReport(ev->ext_disc.addr, ev->ext_disc.rssi, ev->ext_disc.data, ev->ext_disc.length_data);
      break;
#endif
    case BLE_GAP_EVENT_DISC_COMPLETE:
      g_bleScanning = false;
      break;
    default:
      break;
  }
  return 0;
}

void bleRequestScan() {
  g_bleStartReqMs = millis();
  g_bleLastAdvMs = millis();
  if (!ble_hs_synced()) {
    DBG("BLE", "pile BLE pas encore prête");
    return;
  }
  uint8_t own = BLE_OWN_ADDR_PUBLIC;
  if (ble_hs_id_infer_auto(0, &own) != 0) own = BLE_OWN_ADDR_PUBLIC;
  struct ble_gap_disc_params dp;
  memset(&dp, 0, sizeof(dp));
  dp.itvl = (uint16_t)(BLE_SCAN_INTERVAL_MS * 16 / 10);  // unités de 0,625 ms
  dp.window = (uint16_t)(BLE_SCAN_WINDOW_MS * 16 / 10);
  dp.filter_policy = 0;
  dp.limited = 0;
  dp.passive = 0;             // scan actif (noms en réponse de scan)
  dp.filter_duplicates = 0;   // tous les paquets
  int rc = ble_gap_disc(own, BLE_HS_FOREVER, &dp, radarGapEvent, nullptr);
  if (rc == 0 || rc == BLE_HS_EALREADY) {
    g_bleScanning = true;
    ledPulse();  // Scan accepted: immediate activity flash, even without advertisements.
  }
  else DBG("BLE", "ble_gap_disc : erreur %d", rc);
}

// Supervision du scan continu (appelée par loop() chaque seconde) : relance si arrêté + battement de cœur
void bleSupervise() {
  static bool wasScanning = false;
  static bool firstReq = false;
  if (!firstReq) { firstReq = true; bleRequestScan(); return; }
  bool scanning = g_bleScanning;
  if (scanning != wasScanning) {
    DBG("BLE", "%s", scanning ? "scan continu actif" : "scan arrêté");
    wasScanning = scanning;
  }
  if (scanning) {
    if (agoMs(g_bleLastAdvMs) > 120000UL) {
      DBG("BLE", "aucun paquet depuis 2 min : relance du scan");
      ble_gap_disc_cancel();
      g_bleScanning = false;
      g_bleLastAdvMs = millis();
    } else {
      g_bleCycles = g_bleCycles + 1;
      g_bleLastCycleMs = millis();
      ledPulse();  // Continuous scan heartbeat, once per supervision tick (~1 second).
    }
  } else if (agoMs(g_bleStartReqMs) > 3000UL) {
    bleRequestScan();
  }
}

// ============================================================================
//  LOGIQUE DE DÉTECTION
// ============================================================================
void setWatchState(int i, WatchState ns) {
  WatchedDevice& w = g_watch[i];
  if (w.rt.state == ns) return;
  DBG("STATE", "%s %s -> %s", w.cfg.name, WATCH_STATE_NAMES[w.rt.state], WATCH_STATE_NAMES[ns]);
  w.rt.state = ns;
}

void calibrationFeed(int i, int8_t rssi) {
  if (g_cal.running && g_cal.idx == i && rssi > -127 && g_cal.n < CALIB_MAX_SAMPLES) {
    g_cal.samples[g_cal.n++] = rssi;
  }
}

void proximityEval(int i, float d) {
  WatchedDevice& w = g_watch[i];
  WatchedRuntime& r = w.rt;
  if (d < 0) return;
  float thr = w.cfg.proxThreshold;
  float outThr = thr + w.cfg.proxHyst;
  if (r.state == WS_PRESENT) {
    if (d < thr) {
      r.nearCnt++;
      if (r.nearCnt >= PROX_CONFIRM_SAMPLES) {
        r.nearCnt = 0;
        r.farCnt = 0;
        setWatchState(i, WS_NEAR);
        DBG("DISTANCE", "%s entre dans la zone (%.2f m < %.1f m)", w.cfg.name, (double)d, (double)thr);
        logEvent("%s : entrée dans zone %.1f m", w.cfg.name, (double)thr);
        if (w.cfg.active && w.cfg.alertProx) {
          if (!r.hasProxAlert || elapsedMs(r.lastProxAlertMs, (uint32_t)w.cfg.cooldownSec * 1000UL)) {
            alertProximity(i, d);
            r.hasProxAlert = true;
            r.lastProxAlertMs = millis();
          } else {
            logEvent("%s : alerte proximité ignorée (cooldown)", w.cfg.name);
          }
        }
      }
    } else {
      r.nearCnt = 0;
    }
  } else if (r.state == WS_NEAR) {
    if (d > outThr) {
      r.farCnt++;
      if (r.farCnt >= PROX_CONFIRM_SAMPLES) {
        r.farCnt = 0;
        r.nearCnt = 0;
        setWatchState(i, WS_PRESENT);
        DBG("DISTANCE", "%s sort de la zone (%.2f m > %.1f m)", w.cfg.name, (double)d, (double)outThr);
        logEvent("%s : sortie de zone", w.cfg.name);
      }
    } else {
      r.farCnt = 0;
    }
  }
}

// Chemin critique : advertisement -> MAC -> watchlist -> état -> notification
void watchObserve(int i, const Observation& o) {
  WatchedDevice& w = g_watch[i];
  WatchedRuntime& r = w.rt;
  bool gap = !r.hasFilt || (uint32_t)(o.ms - r.lastSeenMs) > FILTER_RESET_GAP_MS;
  bool sample = gap || (uint32_t)(o.ms - r.lastFiltMs) >= WATCH_SAMPLE_MS;
  if (!r.everSeen) r.firstSeenMs = o.ms;
  r.everSeen = true;
  r.lastSeenMs = o.ms;
  r.obsCount++;
  if (o.flags & OBS_NAME) strlcpy(r.bleName, o.name, sizeof(r.bleName));
  if (!sample) return;  // au plus 1 échantillon RSSI toutes les 250 ms (le 1er paquet passe toujours)

  if (o.rssi > -127) {
    r.rawRssi = o.rssi;
    r.filtRssi = gap ? (float)o.rssi : g_set.emaAlpha * (float)o.rssi + (1.0f - g_set.emaAlpha) * r.filtRssi;
    r.hasFilt = true;
  }
  r.lastFiltMs = o.ms;
  calibrationFeed(i, o.rssi);
  float d = watchDistance(w);
  char mac[18];
  formatMac(w.cfg.mac, mac);
  DBG("WATCH", "%s %s RSSI=%d filtered=%.0f dist=%.2fm", w.cfg.name, mac, (int)r.rawRssi, (double)r.filtRssi, (double)d);

  if (r.state == WS_PRESENT || r.state == WS_NEAR) { proximityEval(i, d); return; }

  if (!r.appearArmed) {
    setWatchState(i, WS_PRESENT);
    r.nearCnt = 0; r.farCnt = 0; r.confirmCount = 0;
    proximityEval(i, d);
    return;
  }

  if (r.confirmCount == 0 || (uint32_t)(o.ms - r.confirmLastMs) > WATCH_CONFIRM_WINDOW_MS) r.confirmCount = 0;
  if (r.confirmCount < 255) r.confirmCount++;
  r.confirmLastMs = o.ms;
  uint8_t need = w.cfg.appearMode ? (w.cfg.confirmations ? w.cfg.confirmations : 1) : 1;
  if (r.confirmCount < need) {
    DBG("WATCH", "%s confirmation %u/%u", w.cfg.name, (unsigned)r.confirmCount, (unsigned)need);
    return;
  }
  r.confirmCount = 0;
  r.appearArmed = false;
  r.nearCnt = 0; r.farCnt = 0;
  setWatchState(i, WS_PRESENT);
  logEvent("%s détecté", w.cfg.name);
  if (w.cfg.active && w.cfg.alertAppear) alertAppearance(i);
  proximityEval(i, d);
}

DetectedDevice* visUpsert(const Observation& o, int wi) {
  DetectedDevice* d = nullptr;
  DetectedDevice* freeSlot = nullptr;
  DetectedDevice* oldest = nullptr;
  uint32_t oldestAge = 0;
  for (int i = 0; i < MAX_VISIBLE; i++) {
    DetectedDevice& v = g_vis[i];
    if (!v.used) { if (!freeSlot) freeSlot = &v; continue; }
    if (memcmp(v.mac, o.mac, 6) == 0) { d = &v; break; }
    uint32_t a = agoMs(v.lastSeenMs);
    if (!oldest || a > oldestAge) { oldest = &v; oldestAge = a; }
  }
  if (!d) {
    d = freeSlot ? freeSlot : oldest;
    if (!d) return nullptr;
    memset(d, 0, sizeof(*d));
    d->used = true;
    memcpy(d->mac, o.mac, 6);
    d->firstSeenMs = o.ms;
    d->lastSeenMs = o.ms;
  }
  d->addrType = o.addrType;
  d->watchIdx = (int8_t)wi;
  if (o.rssi > -127) {
    bool reset = !d->hasFilt || (uint32_t)(o.ms - d->lastSeenMs) > FILTER_RESET_GAP_MS;
    d->rawRssi = o.rssi;
    d->filtRssi = reset ? (float)o.rssi : g_set.emaAlpha * (float)o.rssi + (1.0f - g_set.emaAlpha) * d->filtRssi;
    d->hasFilt = true;
  }
  d->lastSeenMs = o.ms;
  d->obsCount++;
  if (o.flags & OBS_NAME) strlcpy(d->name, o.name, sizeof(d->name));
  if (o.flags & OBS_MFG) { d->mfgId = o.mfgId; d->hasMfg = true; }
  if (o.appleType) d->appleType = o.appleType;
  if (o.svc16) d->svc16 = o.svc16;
  if (o.flags & OBS_SVC) strlcpy(d->svc, o.svc, sizeof(d->svc));
  return d;
}

void unknownEvaluate(DetectedDevice& d) {
  if (d.unkDone || d.known) return;
  if (!d.unkChecked) {
    d.unkChecked = true;
    if (knownFind(d.mac) >= 0) { d.known = true; return; }
  }
  bool isRandom = d.addrType != 0;
  if (isRandom && g_set.ignoreRandom) return;
  if (!d.hasFilt || d.filtRssi <= (float)g_set.unknownMinRssi) return;
  uint32_t now = millis();
  if (d.unkCount > 0 && (uint32_t)(now - d.unkLastMs) < 1000UL) return;  // 1 observation comptée par seconde
  d.unkLastMs = now;
  if (d.unkCount == 0 || (uint32_t)(now - d.unkFirstMs) > (uint32_t)g_set.unknownWindowSec * 1000UL) {
    d.unkCount = 0;
    d.unkFirstMs = now;
  }
  if (d.unkCount < 255) d.unkCount++;
  if (d.unkCount < g_set.unknownConf) return;
  d.unkDone = true;
  bool notified = false;
  if (g_set.unknownAlert) notified = alertUnknown(d);
  knownAdd(d.mac, isRandom, notified);
  char mac[18];
  formatMac(d.mac, mac);
  if (g_set.unknownAlert) logEvent("Nouvelle MAC détectée %s", mac);
  else DBG("BLE", "nouvelle MAC mémorisée %s", mac);
}

void processObservation(const Observation& o) {
  int wi = findWatchIdx(o.mac);
  if (wi >= 0) {
    watchObserve(wi, o);
    visUpsert(o, wi);
    return;
  }
  DetectedDevice* d = visUpsert(o, -1);
  if (d) unknownEvaluate(*d);
}

void drainWatchQueue() {
  Observation o;
  while (obsPop(g_watchQ, o)) processObservation(o);
}

void processQueues() {
  drainWatchQueue();
  Observation o;
  for (int n = 0; n < 24; n++) {
    if (!obsPop(g_genQ, o)) break;
    processObservation(o);
    if ((n & 7) == 7) drainWatchQueue();
  }
  drainWatchQueue();
}

void watchTick() {
  for (int i = 0; i < MAX_WATCHED; i++) {
    WatchedDevice& w = g_watch[i];
    if (!w.used) continue;
    WatchedRuntime& r = w.rt;
    if (r.confirmCount && elapsedMs(r.confirmLastMs, WATCH_CONFIRM_WINDOW_MS)) r.confirmCount = 0;
    if (!r.everSeen) continue;
    uint32_t a = agoMs(r.lastSeenMs);
    if ((r.state == WS_PRESENT || r.state == WS_NEAR) && a >= WATCH_PENDING_MS) {
      setWatchState(i, WS_ABSENT_PENDING);
      r.nearCnt = 0; r.farCnt = 0;
    }
    if (r.state == WS_ABSENT_PENDING && a >= (uint32_t)w.cfg.disappearMin * 60000UL) {
      setWatchState(i, WS_ABSENT);
      logEvent("%s absent", w.cfg.name);
      if (w.cfg.active && w.cfg.alertDisappear) {
        if (!r.hasDisAlert || elapsedMs(r.lastDisAlertMs, (uint32_t)w.cfg.cooldownSec * 1000UL)) {
          alertDisappearance(i);
          r.hasDisAlert = true;
          r.lastDisAlertMs = millis();
        } else {
          logEvent("%s : alerte disparition ignorée (cooldown)", w.cfg.name);
        }
      }
    }
    uint8_t resetMin = w.cfg.absenceResetMin ? w.cfg.absenceResetMin : g_set.absenceResetMin;
    if (!r.appearArmed && r.state != WS_PRESENT && r.state != WS_NEAR && a >= (uint32_t)resetMin * 60000UL) {
      r.appearArmed = true;
      DBG("STATE", "%s réarmé pour une nouvelle apparition", w.cfg.name);
    }
  }
}

void calibrationFinish() {
  g_cal.running = false;
  g_cal.done = true;
  g_cal.msg[0] = 0;
  uint8_t n = g_cal.n;
  if (n < 5) {
    g_cal.ok = false;
    snprintf(g_cal.msg, sizeof(g_cal.msg), "Échantillons insuffisants (%u). Vérifiez que l'appareil émet et réessayez.", (unsigned)n);
    logEvent("Calibration échouée (%u échantillons)", (unsigned)n);
    return;
  }
  int8_t s[CALIB_MAX_SAMPLES];
  memcpy(s, g_cal.samples, n);
  for (uint8_t a = 1; a < n; a++) {
    int8_t v = s[a];
    int b = a - 1;
    while (b >= 0 && s[b] > v) { s[b + 1] = s[b]; b--; }
    s[b + 1] = v;
  }
  uint8_t trim = n / 5;
  long sum = 0;
  uint8_t cnt = 0;
  for (uint8_t k = trim; k < n - trim; k++) { sum += s[k]; cnt++; }
  long res = lroundf((float)sum / (float)cnt);
  if (res < -100) res = -100;
  if (res > -20) res = -20;
  g_cal.result = (int8_t)res;
  g_cal.ok = true;
  int spread = s[n - trim - 1] - s[trim];
  if (spread > 15) snprintf(g_cal.msg, sizeof(g_cal.msg), "Mesure instable (écart %d dB) : résultat indicatif.", spread);
  DBG("DISTANCE", "calibration : %d dBm à 1 m (%u échantillons, écart %d dB)", (int)g_cal.result, (unsigned)n, spread);
  logEvent("Calibration : %d dBm à 1 m", (int)g_cal.result);
}

void calibrationTick() {
  if (g_cal.running && elapsedMs(g_cal.startMs, CALIB_DURATION_MS)) calibrationFinish();
}

void visExpire() {
  for (int i = 0; i < MAX_VISIBLE; i++) {
    if (g_vis[i].used && agoMs(g_vis[i].lastSeenMs) > VISIBLE_TTL_MS) g_vis[i].used = false;
  }
}

// ============================================================================
//  WI-FI, SOFTAP, MDNS, NTP
// ============================================================================
void applyTxPower() {
  WiFi.setTxPower(g_set.txLow ? WIFI_POWER_8_5dBm : WIFI_POWER_19_5dBm);
}

void startMdns() {
  #if !USE_MDNS
  return;
#endif
  if (g_mdnsStarted) { MDNS.end(); g_mdnsStarted = false; }
  if (MDNS.begin(g_set.mdnsName)) {
    MDNS.addService("http", "tcp", 80);
    g_mdnsStarted = true;
    DBG("MDNS", "http://%s.local", g_set.mdnsName);
  } else {
    DBG("MDNS", "échec du démarrage mDNS");
  }
}

void startSoftAP() {
  if (g_apActive) return;
  WiFi.mode(g_configured ? WIFI_AP_STA : WIFI_AP);
  bool ok = WiFi.softAP(g_apSsid, g_apPass, 1, 0, 4);
  applyTxPower();
  g_apActive = ok;
  DBG("AP", "%s %s IP=%s", ok ? "actif" : "échec", g_apSsid, WiFi.softAPIP().toString().c_str());
  if (ok && g_configured) {
    DBG("AP", "mot de passe secours : %s", g_apPass);
    logEvent("Réseau de secours %s actif", g_apSsid);
  }
}

void stopSoftAP() {
  if (!g_apActive) return;
  WiFi.softAPdisconnect(true);
  g_apActive = false;
  DBG("AP", "désactivé");
  logEvent("Réseau de secours désactivé");
}

void beginSta() {
  if (!g_wifiSsid[0]) return;
  WiFi.disconnect(false, false);
  if (!(WiFi.getMode() & WIFI_MODE_STA)) WiFi.mode(g_apActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.begin(g_wifiSsid, g_wifiPass[0] ? g_wifiPass : nullptr);
  applyTxPower();
  g_wm = WM_CONNECTING;
  g_wmSinceMs = millis();
  DBG("WIFI", "connexion à %s", g_wifiSsid);
}

void onStaConnected() {
  g_wm = WM_CONNECTED;
  g_wmSinceMs = millis();
  g_wmRetryDelay = 2000;
  String ip = WiFi.localIP().toString();
  DBG("WIFI", "connecté IP=%s RSSI=%d", ip.c_str(), (int)WiFi.RSSI());
  logEvent("Wi-Fi connecté (%s)", ip.c_str());
  if (g_apActive) stopSoftAP();
  startMdns();
  configTzTime(g_set.tz, "pool.ntp.org", "time.cloudflare.com", "time.google.com");
}

uint32_t nextRetry(uint32_t d) {
  d = (d < 5000UL) ? 5000UL : d * 2UL;
  uint32_t cap = g_apActive ? 120000UL : 60000UL;
  return d > cap ? cap : d;
}

void wifiForceReconnect() {
  if (!g_configured) return;
  WiFi.disconnect(false, false);
  if (g_wm == WM_CONNECTED) g_wmDisconnectedSinceMs = millis();
  g_wm = WM_WAIT_RETRY;
  g_wmSinceMs = millis();
  g_wmRetryDelay = 300;
  DBG("WIFI", "reconnexion demandée");
}

void setupTestLoop() {
  if (!g_apActive) startSoftAP();
  if (g_setupTestState == 1) {
    if (WiFi.status() == WL_CONNECTED) {
      g_setupTestId = tgEnqueue(TG_SETUP_TEST, "test configuration",
                                "✅ ESP-C3 opérationnel\n\nLes notifications Telegram fonctionnent.");
      if (g_setupTestId) g_setupTestState = 2;
      else { g_setupTestState = 4; strlcpy(g_setupTestErr, "File Telegram indisponible", sizeof(g_setupTestErr)); }
    } else if (elapsedMs(g_setupTestStartMs, 25000UL)) {
      g_setupTestState = 4;
      strlcpy(g_setupTestErr, "Connexion au Wi-Fi impossible (SSID ou mot de passe ?)", sizeof(g_setupTestErr));
      WiFi.disconnect(false, false);
    }
  } else if (g_setupTestState == 2 && elapsedMs(g_setupTestStartMs, 80000UL)) {
    g_setupTestState = 4;
    strlcpy(g_setupTestErr, "Délai dépassé", sizeof(g_setupTestErr));
  }
}

void wifiLoop() {
  if (!g_configured) { setupTestLoop(); return; }
  bool conn = WiFi.status() == WL_CONNECTED;
  switch (g_wm) {
    case WM_IDLE:
      beginSta();
      break;
    case WM_CONNECTING:
      if (conn) onStaConnected();
      else if (elapsedMs(g_wmSinceMs, WIFI_CONNECT_TIMEOUT_MS)) {
        DBG("WIFI", "échec de connexion, nouvel essai dans %lu s", (unsigned long)(nextRetry(g_wmRetryDelay) / 1000UL));
        WiFi.disconnect(false, false);
        g_wm = WM_WAIT_RETRY;
        g_wmSinceMs = millis();
        g_wmRetryDelay = nextRetry(g_wmRetryDelay);
      }
      break;
    case WM_CONNECTED:
      if (!conn) {
        DBG("WIFI", "connexion perdue");
        logEvent("Wi-Fi perdu");
        g_wm = WM_WAIT_RETRY;
        g_wmSinceMs = millis();
        g_wmRetryDelay = 2000;
        g_wmDisconnectedSinceMs = millis();
      }
      break;
    case WM_WAIT_RETRY:
      if (conn) { onStaConnected(); break; }
      if (elapsedMs(g_wmSinceMs, g_wmRetryDelay)) {
        // Ne pas perturber un utilisateur connecté au réseau de secours (max 10 min)
        if (g_apActive && WiFi.softAPgetStationNum() > 0 && !elapsedMs(g_wmSinceMs, 600000UL)) break;
        beginSta();
      }
      break;
  }
  if (!conn && !g_apActive && g_wm != WM_CONNECTED && elapsedMs(g_wmDisconnectedSinceMs, WIFI_AP_FALLBACK_MS)) {
    startSoftAP();
  }
}

// ============================================================================
//  INTERFACE WEB : CSS
// ============================================================================
const char CSS_TXT[] PROGMEM = R"RAWCSS(
:root{--bg:#0b0f14;--card:#131a22;--card2:#18212b;--line:#243040;--tx:#e6edf3;--mut:#8b98a5;--acc:#3fb6ff;--ok:#3ecf8e;--warn:#f5a524;--bad:#ff5d5d;--r:14px}
*{box-sizing:border-box}
html,body{margin:0;background:var(--bg);color:var(--tx);font:15px/1.4 -apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;-webkit-text-size-adjust:100%}
body{padding:env(safe-area-inset-top) env(safe-area-inset-right) env(safe-area-inset-bottom) env(safe-area-inset-left)}
.wrap{max-width:1100px;margin:0 auto;padding:12px}
header{display:flex;align-items:center;justify-content:space-between;padding:6px 2px 12px}
.brand b{display:block;font-size:17px;letter-spacing:.5px}.brand span{color:var(--mut);font-size:12px}
.btn{-webkit-appearance:none;appearance:none;border:1px solid var(--line);background:var(--card2);color:var(--tx);padding:10px 14px;border-radius:10px;font-weight:600;font-size:14px;cursor:pointer;min-height:40px}
.btn.p{background:var(--acc);border-color:var(--acc);color:#001522}
.btn.d{border-color:#5a2a2a;color:var(--bad)}
.btn.s{padding:6px 10px;min-height:32px;font-size:12px}
.btn:disabled{opacity:.5}
.cards{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-bottom:12px}
@media(min-width:760px){.cards{grid-template-columns:repeat(6,1fr)}}
.card{background:var(--card);border:1px solid var(--line);border-radius:var(--r);padding:10px;min-width:0}
.card small{color:var(--mut);font-size:11px;text-transform:uppercase;letter-spacing:.6px}
.card div{font-weight:700;font-size:14px;margin-top:3px;white-space:nowrap;overflow:hidden;text-overflow:ellipsis}
.tabs{display:flex;gap:6px;overflow-x:auto;margin-bottom:12px;position:sticky;top:0;background:var(--bg);padding:6px 0;z-index:5}
.tab{flex:1;text-align:center;padding:10px 8px;border-radius:10px;background:var(--card);border:1px solid var(--line);color:var(--mut);font-weight:700;font-size:13px;letter-spacing:.5px;cursor:pointer;white-space:nowrap;-webkit-user-select:none;user-select:none}
.tab.on{color:var(--tx);border-color:var(--acc)}
.panel{display:none}.panel.on{display:block}
.box{background:var(--card);border:1px solid var(--line);border-radius:var(--r);padding:12px;margin-bottom:12px}
.tw{overflow-x:auto;-webkit-overflow-scrolling:touch}
table{width:100%;border-collapse:collapse;font-size:13px}
th,td{padding:8px 6px;border-bottom:1px solid var(--line);text-align:left;white-space:nowrap}
th{color:var(--mut);font-weight:600;font-size:11px;text-transform:uppercase}
.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;font-size:12px}
.tag{display:inline-block;padding:2px 7px;border-radius:20px;font-size:11px;font-weight:700;border:1px solid var(--line)}
.t-ok{color:var(--ok);border-color:#1f5a41}.t-warn{color:var(--warn);border-color:#6b4a12}.t-bad{color:var(--bad);border-color:#5a2a2a}.t-mut{color:var(--mut)}.t-acc{color:var(--acc);border-color:#1b4f6e}
.note{color:var(--mut);font-size:12px;margin:8px 0 0}
label{display:block;font-size:12px;color:var(--mut);margin:10px 0 4px}
input,select{width:100%;background:#0e141b;color:var(--tx);border:1px solid var(--line);border-radius:10px;padding:10px;font-size:16px}
input[type=checkbox]{width:22px;height:22px;flex:none;padding:0;margin:0}
.row{display:flex;gap:8px;flex-wrap:wrap;align-items:center}
.row>*{flex:1;min-width:140px}
.chk{display:flex;align-items:center;gap:10px;margin:12px 0}
.chk span{font-size:14px;color:var(--tx)}
h3{margin:0 0 6px;font-size:13px;letter-spacing:.6px;text-transform:uppercase;color:var(--acc)}
.msg{margin-top:8px;font-size:13px;min-height:18px}
.ok{color:var(--ok)}.err{color:var(--bad)}
.modal{position:fixed;left:0;top:0;right:0;bottom:0;background:rgba(0,0,0,.6);display:none;align-items:flex-end;justify-content:center;z-index:50;padding:10px}
.modal.on{display:flex}
@media(min-width:600px){.modal{align-items:center}}
.sheet{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:16px;width:100%;max-width:460px;max-height:90vh;overflow-y:auto;-webkit-overflow-scrolling:touch}
.dev{display:grid;grid-template-columns:repeat(2,1fr);gap:8px 12px;font-size:13px;margin:8px 0}
@media(min-width:760px){.dev{grid-template-columns:repeat(4,1fr)}}
.dev small{display:block;color:var(--mut);font-size:11px;text-transform:uppercase}
.acts{display:flex;gap:6px;flex-wrap:wrap;margin-top:10px}
.log div{padding:6px 0;border-bottom:1px solid var(--line);font-size:13px}
.log span{color:var(--mut);margin-right:8px;font-family:ui-monospace,Menlo,monospace;font-size:12px}
canvas{width:100%;height:130px;display:block}
.center{max-width:360px;margin:12vh auto 0;text-align:center}
.pin{letter-spacing:14px;text-align:center;font-size:26px;padding:12px}
hr{border:0;border-top:1px solid var(--line);margin:14px 0}
)RAWCSS";

// ============================================================================
//  INTERFACE WEB : PAGE DE CONNEXION
// ============================================================================
const char LOGIN_HTML[] PROGMEM = R"RAWHTML(<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0b0f14"><title>ESP-C3</title><link rel="stylesheet" href="/s.css"></head><body><div class="wrap"><div class="center box">
<h2 style="margin:6px 0 2px">ESP-C3</h2><p class="note" style="margin-bottom:14px">Accès sécurisé</p>
<label for="p">PIN</label><input id="p" class="pin" type="password" inputmode="numeric" pattern="[0-9]*" maxlength="4" autocomplete="current-password" placeholder="••••">
<button id="b" class="btn p" style="width:100%;margin-top:14px">CONNEXION</button><div id="m" class="msg err"></div></div></div>
<script>
const p=document.getElementById('p'),b=document.getElementById('b'),m=document.getElementById('m');
p.addEventListener('input',()=>{p.value=p.value.replace(/\D/g,'').slice(0,4)});
async function go(){m.textContent='';if(!/^\d{4}$/.test(p.value)){m.textContent='Le PIN doit contenir exactement 4 chiffres.';return}b.disabled=true;
try{const r=await fetch('/api/login',{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Req':'1'},body:'pin='+encodeURIComponent(p.value)});const j=await r.json();if(j.ok){location.replace('/');return}m.textContent=j.err||'Erreur';p.value=''}catch(e){m.textContent='Connexion impossible'}b.disabled=false}
b.onclick=go;p.addEventListener('keydown',e=>{if(e.key==='Enter')go()});p.focus();
</script></body></html>)RAWHTML";

// ============================================================================
//  INTERFACE WEB : ASSISTANT DE CONFIGURATION INITIALE
// ============================================================================
const char SETUP_HTML[] PROGMEM = R"RAWHTML(<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0b0f14"><title>ESP-C3</title><link rel="stylesheet" href="/s.css"></head><body><div class="wrap" style="max-width:480px">
<header><div class="brand"><b>ESP-C3</b><span>Configuration initiale</span></div></header>
<div class="box"><h3>Wi-Fi</h3><label>SSID du Wi-Fi</label><input id="ss" maxlength="32" autocapitalize="off" autocorrect="off" spellcheck="false">
<label>Mot de passe Wi-Fi</label><input id="sp" type="password" maxlength="63"></div>
<div class="box"><h3>PIN Web</h3><label>PIN (4 chiffres)</label><input id="p1" class="pin" type="password" inputmode="numeric" pattern="[0-9]*" maxlength="4" autocomplete="new-password">
<label>Confirmation du PIN</label><input id="p2" class="pin" type="password" inputmode="numeric" pattern="[0-9]*" maxlength="4" autocomplete="new-password"></div>
<div class="box"><h3>Telegram</h3><label>Telegram Bot Token</label><input id="tt" autocapitalize="off" autocorrect="off" autocomplete="off" spellcheck="false">
<label>Telegram Chat ID</label><input id="tc" autocapitalize="off" autocorrect="off" spellcheck="false">
<div class="acts"><button class="btn" id="bt">TESTER TELEGRAM</button></div><div class="msg" id="mt"></div>
<p class="note">Le test connecte temporairement l’ESP à votre Wi-Fi : votre téléphone peut être brièvement déconnecté de ce réseau. Laissez Telegram vide pour le configurer plus tard.</p></div>
<button class="btn p" id="bs" style="width:100%">ENREGISTRER ET REDÉMARRER</button><div class="msg" id="ms"></div>
<p class="note">Utilisez la surveillance de présence uniquement pour vos propres appareils ou avec l’autorisation des personnes concernées.</p></div>
<script>
const $=i=>document.getElementById(i);
function post(u,d){return fetch(u,{method:'POST',headers:{'Content-Type':'application/x-www-form-urlencoded','X-Req':'1'},body:new URLSearchParams(d).toString()}).then(r=>r.json())}
function m(el,t,ok){el.textContent=t;el.className='msg '+(ok?'ok':'err')}
['p1','p2'].forEach(i=>$(i).addEventListener('input',e=>{e.target.value=e.target.value.replace(/\D/g,'').slice(0,4)}));
$('bt').onclick=async()=>{const mt=$('mt'),bt=$('bt');m(mt,'Connexion au Wi-Fi puis envoi…',true);bt.disabled=true;
try{const r=await post('/api/setup/test',{ssid:$('ss').value,pass:$('sp').value,token:$('tt').value.trim(),chat:$('tc').value.trim()});
if(!r.ok){m(mt,r.err||'Erreur',false);bt.disabled=false;return}
const t0=Date.now();const poll=async()=>{let s=null;try{s=await fetch('/api/setup/test',{cache:'no-store'}).then(x=>x.json())}catch(e){}
if(s&&s.state===3){m(mt,'✅ Message Telegram envoyé.',true);bt.disabled=false;return}
if(s&&s.state===4){m(mt,'Échec : '+s.err,false);bt.disabled=false;return}
if(Date.now()-t0>90000){m(mt,'Délai dépassé (reconnectez-vous au réseau ESP-C3 si nécessaire).',false);bt.disabled=false;return}
setTimeout(poll,1500)};setTimeout(poll,1500)}catch(e){m(mt,'Erreur réseau',false);bt.disabled=false}};
$('bs').onclick=async()=>{const ms=$('ms'),bs=$('bs');
if(!$('ss').value){m(ms,'SSID requis.',false);return}
if(!/^\d{4}$/.test($('p1').value)){m(ms,'Le PIN doit contenir exactement 4 chiffres.',false);return}
if($('p1').value!==$('p2').value){m(ms,'Les PIN ne correspondent pas.',false);return}
bs.disabled=true;try{const r=await post('/api/setup/save',{ssid:$('ss').value,pass:$('sp').value,pin:$('p1').value,pin2:$('p2').value,token:$('tt').value.trim(),chat:$('tc').value.trim()});
if(r.ok){m(ms,'Configuration enregistrée. Redémarrage… Rejoignez votre Wi-Fi puis ouvrez l’adresse IP indiquée dans le moniteur série ou votre routeur (mDNS optionnel).',true)}else{m(ms,r.err||'Erreur',false);bs.disabled=false}}catch(e){m(ms,'Erreur réseau',false);bs.disabled=false}};
</script></body></html>)RAWHTML";

// ============================================================================
//  INTERFACE WEB : APPLICATION
// ============================================================================
const char APP_HTML[] PROGMEM = R"RAWHTML(<!doctype html><html lang="fr"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"><meta name="theme-color" content="#0b0f14"><meta name="apple-mobile-web-app-capable" content="yes"><meta name="apple-mobile-web-app-status-bar-style" content="black-translucent"><title>ESP-C3</title><link rel="stylesheet" href="/s.css"></head><body><div class="wrap">
<header><div class="brand"><b>ESP-C3</b><span>Radar BLE</span></div><button class="btn s" id="bOut">DÉCONNEXION</button></header>
<div class="cards">
<div class="card"><small>Wi-Fi</small><div id="cWifi">—</div></div>
<div class="card"><small>BLE</small><div id="cBle">—</div></div>
<div class="card"><small>Visibles</small><div id="cVis">—</div></div>
<div class="card"><small>Apple / Android</small><div id="cKind">—</div></div>
<div class="card"><small>Trackers</small><div id="cTrack">—</div></div>
<div class="card"><small>Surveillés</small><div id="cWat">—</div></div>
<div class="card"><small>Telegram</small><div id="cTg">—</div></div>
<div class="card"><small>Uptime</small><div id="cUp">—</div></div>
</div>
<nav class="tabs"><div class="tab on" data-t="radar">RADAR</div><div class="tab" data-t="dev">APPAREILS</div><div class="tab" data-t="log">JOURNAL</div><div class="tab" data-t="set">PARAMÈTRES</div></nav>

<section id="p-radar" class="panel on">
<div class="box"><canvas id="cv"></canvas><p class="note">Position indicative basée sur le signal — aucune direction réelle n’est mesurée.</p></div>
<div class="box">
<div class="row"><input id="flt" placeholder="Filtrer (nom ou MAC)" autocapitalize="off" autocorrect="off"><select id="sort"><option value="rssi">Tri : RSSI</option><option value="name">Tri : Nom</option><option value="mac">Tri : MAC</option><option value="age">Tri : Dernière détection</option></select></div>
<div class="tw"><table><thead><tr><th>Nom</th><th>MAC</th><th>Type</th><th>RSSI</th><th>RSSI filtré</th><th>Distance estimée</th><th>Dernière détection</th><th>Statut</th><th>Action</th></tr></thead><tbody id="rt"></tbody></table></div>
<p class="note" id="rEmpty">Aucun appareil détecté pour le moment…</p>
<p class="note">Distance estimée à partir du RSSI. Cette valeur peut fortement varier selon l’environnement, les obstacles et l’appareil.</p>
<p class="note">Certains appareils BLE utilisent des adresses privées ou aléatoires. Leur adresse peut changer périodiquement.</p>
</div></section>

<section id="p-dev" class="panel">
<div class="box"><div class="row"><div><h3>Appareils surveillés</h3><span class="note" id="wcount"></span></div><button class="btn p" id="addMan" style="flex:none">AJOUTER MANUELLEMENT</button></div></div>
<div id="wl"></div><p class="note" id="wlEmpty">Aucun appareil enregistré. Ajoutez-en depuis l’onglet RADAR ou manuellement.</p>
<p class="note">Certains appareils Bluetooth LE utilisent des adresses privées qui changent périodiquement. Dans ce cas, la surveillance par MAC peut cesser de reconnaître l’appareil.</p>
<p class="note">Utilisez la surveillance de présence uniquement pour vos propres appareils ou avec l’autorisation des personnes concernées.</p>
</section>

<section id="p-log" class="panel"><div class="box log" id="lg"></div><p class="note">Journal conservé en mémoire vive uniquement (50 événements, effacé au redémarrage).</p></section>

<section id="p-set" class="panel">
<div class="box"><h3>Réseau</h3>
<div class="dev"><div><small>Wi-Fi</small><div id="sNet">—</div></div><div><small>Adresse IP</small><div id="sIp" class="mono">—</div></div><div><small>Réseau de secours</small><div id="sAp">—</div></div><div><small>Mot de passe secours</small><div id="sApP" class="mono">—</div></div></div>
<label>Nom mDNS (nom.local)</label><input id="sMdns" maxlength="31" autocapitalize="off" autocorrect="off" spellcheck="false">
<label>SSID Wi-Fi</label><input id="sSsid" maxlength="32" autocapitalize="off" autocorrect="off" spellcheck="false">
<label>Mot de passe Wi-Fi (vide = inchangé)</label><input id="sPass" type="password" maxlength="63" autocomplete="new-password">
<label for="sApPass">Nouveau mot de passe du réseau de secours (vide = inchangé)</label><input id="sApPass" type="password" minlength="8" maxlength="63" autocomplete="new-password" autocapitalize="off" autocorrect="off" spellcheck="false">
<label for="sApPass2">Confirmer le mot de passe du réseau de secours</label><input id="sApPass2" type="password" minlength="8" maxlength="63" autocomplete="new-password" autocapitalize="off" autocorrect="off" spellcheck="false">
<p class="note">8 à 63 caractères ASCII imprimables. Ce mot de passe protège le réseau ESP-C3-XXXX. Sa modification redémarre l’appareil ; reconnectez-vous avec le nouveau mot de passe si vous utilisez ce réseau.</p>
<label class="chk"><input type="checkbox" id="sTx"><span>Puissance Wi-Fi réduite (recommandé sur SuperMini)</span></label>
<div class="acts"><button class="btn p" id="bNet">ENREGISTRER</button><button class="btn" id="bRec">RECONNECTER LE WI-FI</button></div><div class="msg" id="mNet"></div></div>

<div class="box"><h3>Sécurité</h3>
<label>PIN actuel</label><input id="pOld" type="password" inputmode="numeric" pattern="[0-9]*" maxlength="4" autocomplete="current-password">
<label>Nouveau PIN</label><input id="pNew" type="password" inputmode="numeric" pattern="[0-9]*" maxlength="4" autocomplete="new-password">
<label>Confirmation</label><input id="pNew2" type="password" inputmode="numeric" pattern="[0-9]*" maxlength="4" autocomplete="new-password">
<div class="acts"><button class="btn p" id="bPin">CHANGER LE PIN</button></div><div class="msg" id="mPin"></div></div>

<div class="box"><h3>Détection</h3>
<label>Réarmement apparition après absence (min, 1–60)</label><input id="dAbs" type="number" inputmode="numeric" min="1" max="60">
<label>Alpha EMA du RSSI (0.05–1.00)</label><input id="dAlpha" type="number" inputmode="decimal" step="0.05" min="0.05" max="1">
<label>Mode apparition par défaut</label><select id="dMode"><option value="0">Rapide</option><option value="1">Confirmé</option></select>
<label>Confirmations par défaut (1–10)</label><input id="dConf" type="number" inputmode="numeric" min="1" max="10">
<label>Délai de disparition par défaut (min, 1–60)</label><input id="dDis" type="number" inputmode="numeric" min="1" max="60">
<label>Cooldown par défaut (s, 0–3600)</label><input id="dCd" type="number" inputmode="numeric" min="0" max="3600">
<div class="acts"><button class="btn p" id="bDet">ENREGISTRER</button></div><div class="msg" id="mDet"></div>
<p class="note">Le réarmement s’applique à tous les appareils (sauf surcharge individuelle). Les autres valeurs par défaut s’appliquent aux appareils ajoutés ensuite.</p></div>

<div class="box"><h3>Nouvelles MAC</h3>
<label class="chk"><input type="checkbox" id="uOn"><span>ALERTE NOUVELLE MAC INCONNUE</span></label>
<label>Observations nécessaires (2–20)</label><input id="uConf" type="number" inputmode="numeric" min="2" max="20">
<label>RSSI filtré minimum (dBm, -100 à -30)</label><input id="uRssi" type="number" inputmode="numeric" min="-100" max="-30">
<label>Fenêtre des observations (s, 5–600)</label><input id="uWin" type="number" inputmode="numeric" min="5" max="600">
<label class="chk"><input type="checkbox" id="uRnd"><span>Ignorer les adresses Random/Private pour les alertes inconnues</span></label>
<label>Maximum d’alertes par heure (1–60)</label><input id="uMax" type="number" inputmode="numeric" min="1" max="60">
<div class="acts"><button class="btn p" id="bUnk">ENREGISTRER</button></div><div class="msg" id="mUnk"></div>
<p class="note">Certains appareils BLE utilisent des adresses privées ou aléatoires. Leur adresse peut changer périodiquement.</p>
<p class="note">MAC mémorisées : <span id="uKnown">—</span></p></div>

<div class="box"><h3>Telegram</h3><div id="tState" class="note"></div>
<label>Bot Token (vide = inchangé)</label><input id="tTok" type="password" autocomplete="off" maxlength="79" autocapitalize="off" autocorrect="off" spellcheck="false">
<label>Chat ID</label><input id="tChat" maxlength="32" autocapitalize="off" autocorrect="off" spellcheck="false">
<div class="acts"><button class="btn p" id="bTg">ENREGISTRER</button><button class="btn" id="bTgTest">TESTER TELEGRAM</button><button class="btn d" id="bTgClr">EFFACER</button></div><div class="msg" id="mTg"></div></div>

<div class="box"><h3>Distance</h3>
<label>Path-loss exponent n (1.5–4.5)</label><input id="xN" type="number" inputmode="decimal" step="0.1" min="1.5" max="4.5">
<label>RSSI générique à 1 m (dBm, appareils non calibrés)</label><input id="xR1" type="number" inputmode="numeric" min="-100" max="-20">
<div class="acts"><button class="btn p" id="bDist">ENREGISTRER</button></div><div class="msg" id="mDist"></div>
<p class="note">Distance estimée à partir du RSSI. Cette valeur peut fortement varier selon l’environnement, les obstacles et l’appareil.</p></div>

<div class="box"><h3>Système</h3>
<label>Fuseau horaire</label><select id="zSel"><option value="CET-1CEST,M3.5.0,M10.5.0/3">Europe/Paris</option><option value="GMT0BST,M3.5.0/1,M10.5.0">Europe/London</option><option value="UTC0">UTC</option><option value="EST5EDT,M3.2.0,M11.1.0">America/New_York</option><option value="">Personnalisé (POSIX)</option></select>
<label>Chaîne TZ POSIX</label><input id="zTz" maxlength="47" autocapitalize="off" autocorrect="off" spellcheck="false">
<p class="note" id="sysInfo"></p>
<div class="acts"><button class="btn p" id="bSys">ENREGISTRER</button><button class="btn" id="bReboot">REDÉMARRER</button></div><div class="msg" id="mSys"></div>
<hr><h3 style="color:var(--bad)">Réinitialisation usine</h3><p class="note">Efface Wi-Fi, PIN, Telegram, appareils, cache MAC et paramètres, puis redémarre en mode configuration.</p>
<div class="acts"><button class="btn d" id="bFactory">RÉINITIALISATION USINE</button></div><div class="msg" id="mFac"></div></div>
</section>
</div>
<div class="modal" id="mdl"><div class="sheet" id="sh"></div></div>
<script>
'use strict';
const $=i=>document.getElementById(i);
function h(t,a,...c){const e=document.createElement(t);if(a)for(const k in a){const v=a[k];if(v===false||v==null)continue;if(k==='class')e.className=v;else if(k==='text')e.textContent=v;else if(k.slice(0,2)==='on')e.addEventListener(k.slice(2),v);else e.setAttribute(k,v===true?'':String(v))}for(const x of c){if(x==null||x===false)continue;e.append(typeof x==='string'?document.createTextNode(x):x)}return e}
function btn(t,f,c){return h('button',{class:'btn'+(c?' '+c:''),text:t,onclick:f})}
function inp(t,v,a){return h('input',Object.assign({type:t,value:v==null?'':String(v)},a||{}))}
function chk(v){const e=h('input',{type:'checkbox'});e.checked=!!v;return e}
function sel(o,v){const s=h('select',null,...o.map(x=>h('option',{value:x[0],text:x[1]})));s.value=String(v);return s}
function lab(t,el){return h('div',null,h('label',{text:t}),el)}
function ck(t,el){return h('label',{class:'chk'},el,h('span',{text:t}))}
function b01(e){return e.checked?'1':'0'}
function msg(el,t,ok){el.textContent=t||'';el.className='msg '+(ok?'ok':'err')}
function stat(l,v){return h('div',null,h('small',{text:l}),h('div',{text:v}))}
async function api(p,d){const o={method:d?'POST':'GET',headers:{},cache:'no-store',credentials:'same-origin'};if(d){o.headers['Content-Type']='application/x-www-form-urlencoded';o.headers['X-Req']='1';o.body=new URLSearchParams(d).toString()}
let r;try{r=await fetch(p,o)}catch(e){return{ok:false,err:'Connexion perdue'}}if(r.status===401){location.replace('/');return{ok:false,err:'Session expirée'}}
try{return await r.json()}catch(e){return{ok:false,err:'Réponse invalide ('+r.status+')'}}}
function fmtAge(ms){if(ms==null||ms<0)return'—';if(ms<10000)return(ms/1000).toFixed(1)+' s';if(ms<60000)return Math.round(ms/1000)+' s';if(ms<3600000)return Math.floor(ms/60000)+' min';return Math.floor(ms/3600000)+' h'}
function fmtDist(d){if(d==null||d<0)return'—';if(d<10)return'≈ '+d.toFixed(1)+' m';if(d<100)return'≈ '+Math.round(d)+' m';return'> 99 m'}
function fmtUp(s){const p=n=>String(n).padStart(2,'0');const d=Math.floor(s/86400);s%=86400;return(d?d+' j ':'')+p(Math.floor(s/3600))+':'+p(Math.floor(s%3600/60))+':'+p(s%60)}
const ST={UNKNOWN:['Jamais vu','t-mut'],PRESENT:['Présent','t-ok'],NEAR:['À proximité','t-acc'],ABSENT_PENDING:['Absence en cours','t-warn'],ABSENT:['Absent','t-bad']};
let tab='radar';const busy={};
async function guard(k,f){if(busy[k])return;busy[k]=1;try{await f()}catch(e){}finally{busy[k]=0}}
function showTab(t){tab=t;document.querySelectorAll('.tab').forEach(b=>b.classList.toggle('on',b.dataset.t===t));document.querySelectorAll('.panel').forEach(p=>p.classList.toggle('on',p.id==='p-'+t));
if(t==='radar')guard('r',pollRadar);if(t==='dev')guard('w',pollWatch);if(t==='log')guard('l',pollLog);if(t==='set')loadSettings()}
document.querySelectorAll('.tab').forEach(b=>b.addEventListener('click',()=>showTab(b.dataset.t)));

async function pollStatus(){const s=await api('/api/status');if(!s.ok)return;
$('cWifi').textContent=s.wifi.c?('Connecté · '+s.wifi.rssi+' dBm'):(s.wifi.ap?'Secours actif':'Déconnecté');
$('cBle').textContent=s.ble.ok?'Scan actif':'Arrêté';if(s.kinds){$('cKind').textContent=s.kinds.apple+' / '+s.kinds.android;$('cTrack').textContent=s.kinds.track+(s.kinds.win?' · '+s.kinds.win+' Win':'')}$('cWat').textContent=s.pres+' / '+s.wat;
$('cTg').textContent=!s.tg.conf?'Non configuré':((s.tg.last===1?'OK':s.tg.last===0?'Erreur':'Prêt')+(s.tg.q?' ('+s.tg.q+')':''));
$('cUp').textContent=fmtUp(s.up)}

const rows=new Map();let devs=[];
async function pollRadar(){const j=await api('/api/devices');if(!j.ok)return;devs=j.list;renderRadar();drawRadar()}
function mkRow(){const tr=h('tr');const t=[];for(let i=0;i<8;i++){const td=h('td');t.push(td);tr.append(td)}t[1].className='mono';const tag=h('span',{class:'tag'});t[7].append(tag);const act=h('td');tr.append(act);return{tr,t,tag,act,d:null,w:null}}
function updRow(r,d){r.d=d;const t=r.t;t[0].textContent=d.n||d.kind||'Inconnu';t[0].className=d.n?'':(d.kind?'t-acc':'t-mut');t[1].textContent=d.m;t[2].textContent=d.t;t[3].textContent=d.r+' dBm';t[4].textContent=Math.round(d.f)+' dBm';t[5].textContent=fmtDist(d.d);t[6].textContent=fmtAge(d.a);
const st=d.a<10000?['VISIBLE','t-ok']:d.a<60000?['RÉCENT','t-warn']:['ANCIEN','t-mut'];r.tag.textContent=st[0];r.tag.className='tag '+st[1];
const w=d.w>=0;if(r.w!==w){r.w=w;r.act.replaceChildren(w?h('span',{class:'tag t-acc',text:'SURVEILLÉ'}):btn('AJOUTER',()=>openAdd(r.d),'s'))}}
function renderRadar(){const tb=$('rt');const q=$('flt').value.trim().toLowerCase();const s=$('sort').value;
const L=devs.filter(d=>!q||(d.n||'').toLowerCase().includes(q)||d.m.toLowerCase().includes(q));
const C={name:(a,b)=>(a.n||'\uffff').localeCompare(b.n||'\uffff'),mac:(a,b)=>a.m.localeCompare(b.m),age:(a,b)=>a.a-b.a,rssi:(a,b)=>b.f-a.f};L.sort(C[s]||C.rssi);
const seen=new Set();for(const d of L){seen.add(d.m);let r=rows.get(d.m);if(!r){r=mkRow();rows.set(d.m,r)}updRow(r,d);tb.appendChild(r.tr)}
for(const [k,r] of rows)if(!seen.has(k)){r.tr.remove();rows.delete(k)}$('rEmpty').style.display=L.length?'none':''}
function drawRadar(){const c=$('cv');const dpr=window.devicePixelRatio||1;const W=c.clientWidth,H=c.clientHeight;if(!W||!H)return;
if(c.width!==Math.round(W*dpr)||c.height!==Math.round(H*dpr)){c.width=Math.round(W*dpr);c.height=Math.round(H*dpr)}
const g=c.getContext('2d');g.setTransform(dpr,0,0,dpr,0,0);g.clearRect(0,0,W,H);const x0=44,x1=W-12,mid=(H-18)/2;
const xr=v=>x0+(Math.min(-30,Math.max(-100,v))+30)/(-70)*(x1-x0);g.font='10px -apple-system,sans-serif';g.lineWidth=1;
for(const v of[-40,-60,-80,-100]){const x=xr(v);g.strokeStyle='#243040';g.beginPath();g.moveTo(x,4);g.lineTo(x,H-18);g.stroke();g.fillStyle='#8b98a5';g.fillText(v+' dBm',Math.min(x-18,W-44),H-4)}
g.fillStyle='#3fb6ff';g.beginPath();g.arc(16,mid,7,0,6.283);g.fill();g.fillStyle='#8b98a5';g.fillText('ESP',5,mid+20);
for(const d of devs){if(d.a>15000)continue;let hs=0;for(let i=0;i<d.m.length;i++)hs=(hs*31+d.m.charCodeAt(i))>>>0;const y=8+(hs%1000)/1000*(H-32);
g.fillStyle=d.w>=0?'#3fb6ff':'rgba(230,237,243,.55)';g.beginPath();g.arc(xr(d.f),y,d.w>=0?5:3,0,6.283);g.fill()}}
window.addEventListener('resize',()=>{if(tab==='radar')drawRadar()});
$('flt').addEventListener('input',renderRadar);$('sort').addEventListener('change',renderRadar);

let calPoll=null;
function modal(...c){$('sh').replaceChildren(...c);$('mdl').classList.add('on')}
function closeModal(){$('mdl').classList.remove('on');if(calPoll){clearInterval(calPoll);calPoll=null}}
$('mdl').addEventListener('click',e=>{if(e.target===$('mdl'))closeModal()});

function openAdd(d){const name=inp('text','',{maxlength:32,placeholder:'Ex. Téléphone Ruben'});const m=h('div',{class:'msg'});
const save=async()=>{const n=name.value.trim();if(!n){msg(m,'Nom personnalisé requis.',false);return}const r=await api('/api/watchlist/add',{mac:d.m,name:n});if(r.ok){closeModal();showTab('dev')}else msg(m,r.err,false)};
modal(h('h3',{text:'Ajouter un appareil'}),lab('Adresse MAC',h('div',{class:'mono',text:d.m})),lab('Nom BLE détecté',h('div',{text:d.n||'Inconnu'})),
h('div',{class:'dev'},stat('Type',d.t),stat('Observations',String(d.c)),stat('Première détection','il y a '+fmtAge(d.fs)),stat('Fabricant',d.mf?'0x'+d.mf:'—')),
d.sv?h('div',{class:'note mono',text:'Service : '+d.sv}):null,lab('Nom personnalisé',name),h('div',{class:'acts'},btn('ENREGISTRER',save,'p'),btn('ANNULER',closeModal)),m);
name.addEventListener('keydown',e=>{if(e.key==='Enter')save()});setTimeout(()=>name.focus(),60)}

function openManual(){const name=inp('text','',{maxlength:32,placeholder:'Ex. Balise voiture'});const mac=inp('text','',{maxlength:17,placeholder:'AA:BB:CC:DD:EE:FF',autocapitalize:'characters',autocorrect:'off',spellcheck:'false',class:'mono'});const m=h('div',{class:'msg'});
mac.addEventListener('input',()=>{mac.value=mac.value.toUpperCase().replace(/[^0-9A-F:]/g,'')});
modal(h('h3',{text:'Ajouter manuellement'}),lab('Nom personnalisé',name),lab('Adresse MAC',mac),h('div',{class:'acts'},btn('ENREGISTRER',async()=>{
const n=name.value.trim(),a=mac.value.trim().toUpperCase();if(!n||!a){msg(m,'Tous les champs sont requis.',false);return}
if(!/^([0-9A-F]{2}:){5}[0-9A-F]{2}$/.test(a)){msg(m,'Format MAC invalide (XX:XX:XX:XX:XX:XX).',false);return}
const r=await api('/api/watchlist/add',{mac:a,name:n});if(r.ok){closeModal();guard('w',pollWatch)}else msg(m,r.err,false)},'p'),btn('ANNULER',closeModal)),m)}
$('addMan').onclick=openManual;

const cards=new Map(),wdata=new Map();
async function pollWatch(){const j=await api('/api/watchlist');if(!j.ok)return;wdata.clear();for(const w of j.list)wdata.set(w.i,w);renderWatch(j.list)}
function renderWatch(list){const box=$('wl');$('wcount').textContent=list.length+' / 20';$('wlEmpty').style.display=list.length?'none':'';const seen=new Set();
for(const w of list){seen.add(w.i);const sig=JSON.stringify([w.name,w.mac,w.act,w.cal,w.r1,w.r1e,w.aa,w.am,w.ac,w.ap,w.pt,w.ph,w.ad,w.dm,w.ar,w.are,w.cd,w.pl]);
let c=cards.get(w.i);if(!c||c.sig!==sig){const n=mkCard(w);n.sig=sig;if(c)c.el.replaceWith(n.el);else box.append(n.el);cards.set(w.i,n);c=n}updCard(c,w)}
for(const [k,c] of cards)if(!seen.has(k)){c.el.remove();cards.delete(k)}}
function mkCard(w){const f={};const fld=(l,k,cl)=>{const v=h('div',{text:'—',class:cl||false});f[k]=v;return h('div',null,h('small',{text:l}),v)};const stTag=h('span',{class:'tag'});
const el=h('div',{class:'box'},h('div',{class:'row'},h('h3',{text:w.name,style:'color:var(--tx);text-transform:none;font-size:16px;letter-spacing:0'}),h('div',{style:'flex:none;min-width:0'},stTag)),
h('div',{class:'dev'},fld('MAC','mac','mono'),fld('État','st'),fld('RSSI','r'),fld('RSSI filtré','f'),fld('Distance estimée','d'),stat('Calibration',w.cal?(w.r1+' dBm à 1 m'):'NON CALIBRÉ ('+w.r1e+' dBm)'),fld('Dernière détection','a'),fld('Nom BLE détecté','ble'),
stat('Alerte apparition',w.aa?'ON':'OFF'),stat('Mode',w.am?'Confirmé ('+w.ac+')':'Rapide'),stat('Alerte proximité',w.ap?'ON':'OFF'),stat('Distance',w.pt.toFixed(1)+' m (sortie '+(w.pt+w.ph).toFixed(1)+' m)'),
stat('Alerte disparition',w.ad?'ON':'OFF'),stat('Absence',w.dm+' min'),stat('Réarmement',w.are+' min'+(w.ar?'':' (global)')),stat('Notifications',w.act?'Actives':'Désactivées')),
h('div',{class:'acts'},btn('MODIFIER',()=>openEdit(w.i)),btn('CALIBRER',()=>openCal(w.i)),btn(w.act?'DÉSACTIVER':'ACTIVER',()=>toggleAct(w.i,w.act)),btn('SUPPRIMER',()=>delWatch(w.i,w.name),'d')));
f.stTag=stTag;f.mac.textContent=w.mac;return{el,f,sig:''}}
function updCard(c,w){const f=c.f;const st=ST[w.st]||[w.st,'t-mut'];f.stTag.textContent=st[0];f.stTag.className='tag '+st[1];f.st.textContent=st[0];
f.r.textContent=w.a<0?'—':w.r+' dBm';f.f.textContent=w.hf?Math.round(w.f)+' dBm':'—';f.d.textContent=w.hf?fmtDist(w.d):'—';f.a.textContent=w.a<0?'Jamais (depuis le démarrage)':'il y a '+fmtAge(w.a);f.ble.textContent=w.ble||'Inconnu'}
function openEdit(i){const w=wdata.get(i);if(!w)return;const m=h('div',{class:'msg'});
const name=inp('text',w.name,{maxlength:32}),act=chk(w.act),aa=chk(w.aa),am=sel([['0','Rapide'],['1','Confirmé']],w.am),ac=inp('number',w.ac,{min:1,max:10,inputmode:'numeric'}),
ap=chk(w.ap),pt=inp('number',w.pt,{min:0.3,max:30,step:0.1,inputmode:'decimal'}),ph=inp('number',w.ph,{min:0.2,max:20,step:0.1,inputmode:'decimal'}),ad=chk(w.ad),
dm=inp('number',w.dm,{min:1,max:60,inputmode:'numeric'}),ar=inp('number',w.ar,{min:0,max:60,inputmode:'numeric'}),cd=inp('number',w.cd,{min:0,max:3600,inputmode:'numeric'}),pl=inp('number',w.pl,{min:0,max:4.5,step:0.1,inputmode:'decimal'});
modal(h('h3',{text:'Modifier'}),h('div',{class:'note mono',text:w.mac}),lab('Nom personnalisé',name),ck('Notifications actives',act),
ck('Alerter lorsque cet appareil apparaît',aa),lab('Mode apparition',am),lab('Confirmations (mode confirmé, 1–10)',ac),
ck('Alerter lorsque la distance estimée devient inférieure au seuil',ap),lab('Seuil de proximité (m)',pt),lab('Hystérésis (m) — sortie = seuil + hystérésis',ph),
ck('Alerter lorsque cet appareil n’est plus détecté',ad),lab('Délai avant disparition (min, 1–60)',dm),lab('Réarmement apparition (min, 0 = réglage global)',ar),
lab('Cooldown proximité / disparition (s, 0–3600)',cd),lab('Path-loss exponent (0 = réglage global)',pl),
h('div',{class:'acts'},btn('ENREGISTRER',async()=>{const n=name.value.trim();if(!n){msg(m,'Nom requis.',false);return}
const r=await api('/api/watchlist/update',{i,name:n,active:b01(act),alertAppear:b01(aa),appearMode:am.value,confirmations:ac.value,alertProx:b01(ap),proxThreshold:pt.value,proxHyst:ph.value,alertDisappear:b01(ad),disappearMin:dm.value,absenceMin:ar.value,cooldown:cd.value,pathLoss:pl.value});
if(r.ok){closeModal();guard('w',pollWatch)}else msg(m,r.err,false)},'p'),btn('ANNULER',closeModal)),m)}
function openCal(i){const w=wdata.get(i);if(!w)return;const info=h('div',{class:'msg'});const fill=h('div',{style:'height:100%;width:0;background:var(--acc)'});
const bar=h('div',{style:'height:8px;background:var(--line);border-radius:4px;overflow:hidden;margin-top:10px'},fill);const res=h('div',{style:'font-size:18px;font-weight:700;margin-top:12px'});
const save=btn('ENREGISTRER',async()=>{const r=await api('/api/calibration/save',{i});if(r.ok){closeModal();guard('w',pollWatch)}else msg(info,r.err,false)},'p');save.style.display='none';
const start=btn('DÉMARRER',async()=>{start.disabled=true;save.style.display='none';res.textContent='';fill.style.width='0';const r=await api('/api/calibration/start',{i});if(!r.ok){msg(info,r.err,false);start.disabled=false;return}
msg(info,'Mesure en cours…',true);if(calPoll)clearInterval(calPoll);calPoll=setInterval(async()=>{const s=await api('/api/calibration/status');if(!s.ok)return;fill.style.width=Math.min(100,s.el/s.dur*100)+'%';
if(s.run)msg(info,'Échantillons : '+s.n,true);if(s.done){clearInterval(calPoll);calPoll=null;start.disabled=false;start.textContent='RECOMMENCER';
if(s.okc){res.textContent='RSSI de référence à 1 m : '+s.res+' dBm';save.style.display='';msg(info,s.err||('Échantillons : '+s.n),!s.err)}else msg(info,s.err||'Échec',false)}},500)},'p');
modal(h('h3',{text:'Calibrer à 1 m'}),h('div',{text:w.name}),h('p',{class:'note',text:'Placez l’appareil à environ 1 mètre de l’ESP32.'}),h('p',{class:'note',text:'Mesure pendant 12 secondes. Gardez l’appareil immobile, sans obstacle.'}),
bar,res,info,h('div',{class:'acts'},start,save,btn('FERMER',closeModal),w.cal?btn('SUPPRIMER LA CALIBRATION',async()=>{const r=await api('/api/watchlist/update',{i,uncal:1});if(r.ok){closeModal();guard('w',pollWatch)}},'d'):null))}
async function toggleAct(i,a){const r=await api('/api/watchlist/update',{i,active:a?0:1});if(r.ok)guard('w',pollWatch);else alert(r.err)}
async function delWatch(i,n){if(!confirm('Supprimer « '+n+' » de la surveillance ?'))return;const r=await api('/api/watchlist/delete',{i});if(r.ok)guard('w',pollWatch);else alert(r.err)}

async function pollLog(){const j=await api('/api/events');if(!j.ok)return;$('lg').replaceChildren(...(j.list.length?j.list.map(e=>h('div',null,h('span',{text:e.t}),e.x)):[h('div',{class:'note',text:'Aucun événement.'})]))}

async function loadSettings(){const s=await api('/api/settings');if(!s.ok)return;
$('sNet').textContent=s.sta?('Connecté à '+s.ssid):('Non connecté'+(s.ssid?' ('+s.ssid+')':''));$('sIp').textContent=s.ip||'—';
$('sAp').textContent=s.apSsid+(s.ap?' (actif)':' (inactif)');$('sApP').textContent=s.apPass;$('sMdns').value=s.mdns;$('sSsid').value=s.ssid;$('sPass').value='';$('sApPass').value='';$('sApPass2').value='';$('sTx').checked=!!s.tx;
$('dAbs').value=s.abs;$('dAlpha').value=s.alpha;$('dMode').value=String(s.mode);$('dConf').value=s.conf;$('dDis').value=s.dis;$('dCd').value=s.cd;
$('uOn').checked=!!s.uOn;$('uConf').value=s.uConf;$('uRssi').value=s.uRssi;$('uWin').value=s.uWin;$('uRnd').checked=!!s.uRnd;$('uMax').value=s.uMax;$('uKnown').textContent=s.known+' / 200';
$('tState').textContent=s.tgConf?'Telegram configuré (token masqué).':'Telegram non configuré.';$('tTok').value='';$('tChat').value=s.tgChat||'';
$('xN').value=s.n;$('xR1').value=s.r1;$('zTz').value=s.tz;const zs=$('zSel');zs.value=[...zs.options].some(o=>o.value===s.tz)?s.tz:'';
$('sysInfo').textContent='Firmware '+s.fw+' · '+s.core+' · Heap '+Math.round(s.heap/1024)+' Ko (min '+Math.round(s.minHeap/1024)+' Ko) · Heure '+s.time}
async function save(section,data,mid){data.section=section;const r=await api('/api/settings',data);msg($(mid),r.ok?(r.reboot?'Enregistré. Redémarrage… Reconnectez-vous avec le nouveau mot de passe si vous utilisez le réseau de secours.':'Enregistré.'):(r.err||'Erreur'),r.ok);if(r.ok&&!r.reboot)loadSettings();return r}
$('bNet').onclick=async()=>{const a=$('sApPass').value,a2=$('sApPass2').value;if(a&&!/^[\x20-\x7E]{8,63}$/.test(a)){msg($('mNet'),'Le mot de passe secours doit contenir 8 à 63 caractères ASCII imprimables.',false);return}if(a!==a2){msg($('mNet'),'La confirmation du mot de passe secours ne correspond pas.',false);return}if(!confirm(a?'Appliquer les paramètres et redémarrer ? Notez le nouveau mot de passe du réseau de secours.':'Appliquer les paramètres réseau ? La connexion peut être interrompue.'))return;const b=$('bNet');b.disabled=true;try{const r=await save('network',{mdns:$('sMdns').value.trim().toLowerCase(),ssid:$('sSsid').value,pass:$('sPass').value,apPass:a,apPass2:a2,tx:b01($('sTx'))},'mNet');if(r.ok){$('sApPass').value='';$('sApPass2').value=''}}finally{b.disabled=false}};
$('bRec').onclick=async()=>{const r=await api('/api/wifi/reconnect',{});msg($('mNet'),r.ok?'Reconnexion lancée.':r.err,r.ok)};
['pOld','pNew','pNew2'].forEach(i=>$(i).addEventListener('input',e=>{e.target.value=e.target.value.replace(/\D/g,'').slice(0,4)}));
$('bPin').onclick=async()=>{const o=$('pOld').value,n=$('pNew').value,n2=$('pNew2').value;if(!/^\d{4}$/.test(n)){msg($('mPin'),'Le nouveau PIN doit contenir exactement 4 chiffres.',false);return}
if(n!==n2){msg($('mPin'),'La confirmation ne correspond pas.',false);return}const r=await api('/api/settings',{section:'security',old:o,new:n,new2:n2});msg($('mPin'),r.ok?'PIN modifié.':r.err,r.ok);['pOld','pNew','pNew2'].forEach(i=>$(i).value='')};
$('bDet').onclick=()=>save('detection',{abs:$('dAbs').value,alpha:$('dAlpha').value,mode:$('dMode').value,conf:$('dConf').value,dis:$('dDis').value,cd:$('dCd').value},'mDet');
$('bUnk').onclick=()=>save('unknown',{uOn:b01($('uOn')),uConf:$('uConf').value,uRssi:$('uRssi').value,uWin:$('uWin').value,uRnd:b01($('uRnd')),uMax:$('uMax').value},'mUnk');
$('bTg').onclick=()=>save('telegram',{token:$('tTok').value.trim(),chat:$('tChat').value.trim()},'mTg');
$('bTgClr').onclick=()=>{if(confirm('Effacer la configuration Telegram ?'))save('telegram',{clear:1},'mTg')};
$('bTgTest').onclick=async()=>{const m=$('mTg');msg(m,'Envoi en cours…',true);const r=await api('/api/telegram/test',{});if(!r.ok){msg(m,r.err,false);return}const t0=Date.now();
const it=setInterval(async()=>{const s=await api('/api/telegram/test');if(s.state===2){clearInterval(it);msg(m,'✅ Message de test envoyé.',true)}else if(s.state===3){clearInterval(it);msg(m,'Échec : '+s.err,false)}else if(Date.now()-t0>60000){clearInterval(it);msg(m,'Délai dépassé.',false)}},1000)};
$('bDist').onclick=()=>save('distance',{n:$('xN').value,r1:$('xR1').value},'mDist');
$('zSel').onchange=()=>{if($('zSel').value)$('zTz').value=$('zSel').value};
$('bSys').onclick=()=>save('system',{tz:$('zTz').value.trim()},'mSys');
$('bReboot').onclick=async()=>{if(!confirm('Redémarrer l’ESP ?'))return;const r=await api('/api/reboot',{});msg($('mSys'),r.ok?'Redémarrage… rechargez la page dans 20 s.':r.err,r.ok)};
$('bFactory').onclick=async()=>{if(!confirm('RÉINITIALISATION USINE : Wi-Fi, PIN, Telegram, appareils, cache MAC et paramètres seront effacés. Continuer ?'))return;
const t=prompt('Seconde confirmation : tapez RESET');if(t!=='RESET'){msg($('mFac'),'Annulé.',false);return}const r=await api('/api/factory-reset',{confirm:'RESET'});
msg($('mFac'),r.ok?'Effacement et redémarrage en mode configuration (réseau ESP-C3-XXXX)…':r.err,r.ok)};
$('bOut').onclick=async()=>{await api('/api/logout',{});location.replace('/')};

setInterval(()=>{if(!document.hidden&&tab==='radar')guard('r',pollRadar)},750);
setInterval(()=>{if(!document.hidden&&tab==='dev')guard('w',pollWatch)},1000);
setInterval(()=>{if(!document.hidden&&tab==='log')guard('l',pollLog)},3000);
setInterval(()=>{if(!document.hidden)guard('s',pollStatus)},2000);
guard('s',pollStatus);guard('r',pollRadar);
</script></body></html>)RAWHTML";

// ============================================================================
//  HTTP : AIDES
// ============================================================================
void addPageHeaders() {
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("X-Frame-Options", "DENY");
  server.sendHeader("Referrer-Policy", "no-referrer");
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Content-Security-Policy",
                    "default-src 'self'; script-src 'self' 'unsafe-inline'; style-src 'self' 'unsafe-inline'; "
                    "img-src 'self' data:; connect-src 'self'; frame-ancestors 'none'; base-uri 'none'; form-action 'self'");
}

void sendErr(int code, const char* m) {
  js.begin(code);
  js.objOpen();
  js.kBool("ok", false);
  js.kStr("err", m);
  js.objClose();
  js.end();
}

void sendOk() {
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.objClose();
  js.end();
}

bool csrfOk() { return server.header("X-Req") == "1"; }

bool requireAuth() {
  if (!g_configured) { sendErr(403, "Configuration initiale requise"); return false; }
  if (!authOk()) { sendErr(401, "Session expirée"); return false; }
  return true;
}

bool requireAuthPost() {
  if (!csrfOk()) { sendErr(403, "Requête refusée"); return false; }
  return requireAuth();
}

bool argLong(const char* key, long mn, long mx, long& out) {
  if (!server.hasArg(key)) return false;
  String v = server.arg(key);
  v.trim();
  if (v.length() == 0 || v.length() > 11) return false;
  char* end = nullptr;
  long x = strtol(v.c_str(), &end, 10);
  if (!end || *end != 0) return false;
  if (x < mn || x > mx) return false;
  out = x;
  return true;
}

bool argFloat(const char* key, float mn, float mx, float& out) {
  if (!server.hasArg(key)) return false;
  String v = server.arg(key);
  v.trim();
  if (v.length() == 0 || v.length() > 12) return false;
  char* end = nullptr;
  float x = strtof(v.c_str(), &end);
  if (!end || *end != 0 || !std::isfinite(x)) return false;
  if (x < mn || x > mx) return false;
  out = x;
  return true;
}

bool argBool(const char* key, bool& out) {
  if (!server.hasArg(key)) return false;
  String v = server.arg(key);
  v.trim();
  v.toLowerCase();
  if (v == "1" || v == "true" || v == "on") { out = true; return true; }
  if (v == "0" || v == "false" || v == "off") { out = false; return true; }
  return false;
}

bool argText(const char* key, char* out, size_t outSize, size_t minLen, bool trim) {
  out[0] = 0;
  if (!server.hasArg(key)) return false;
  String v = server.arg(key);
  if (v.length() >= outSize) return false;
  sanitizeUtf8(v.c_str(), v.length(), out, outSize, trim);
  return strlen(out) >= minLen;
}

#define ARG_INT(key, mn, mx, dst) do { if (server.hasArg(key)) { long _v; if (!argLong(key, mn, mx, _v)) { sendErr(400, "Valeur invalide : " key); return; } dst = _v; } } while (0)
#define ARG_FLOAT(key, mn, mx, dst) do { if (server.hasArg(key)) { float _f; if (!argFloat(key, mn, mx, _f)) { sendErr(400, "Valeur invalide : " key); return; } dst = _f; } } while (0)
#define ARG_BOOL(key, dst) do { if (server.hasArg(key)) { bool _b; if (!argBool(key, _b)) { sendErr(400, "Valeur invalide : " key); return; } dst = _b ? 1 : 0; } } while (0)

void requestReboot(bool factory) {
  g_rebootPending = true;
  g_factoryPending = factory;
  g_rebootReqMs = millis();
}

// ============================================================================
//  HTTP : PAGES
// ============================================================================
void handleRoot() {
  addPageHeaders();
  if (!g_configured) { server.send_P(200, "text/html; charset=utf-8", SETUP_HTML); return; }
  if (authOk()) server.send_P(200, "text/html; charset=utf-8", APP_HTML);
  else server.send_P(200, "text/html; charset=utf-8", LOGIN_HTML);
}

void handleCss() {
  server.sendHeader("Cache-Control", "max-age=600");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.send_P(200, "text/css; charset=utf-8", CSS_TXT);
}

void handleNotFound() {
  if (server.uri().startsWith("/api/")) { sendErr(404, "Route inconnue"); return; }
  server.sendHeader("Location", "/");
  server.send(302, "text/plain", "");
}

// ============================================================================
//  HTTP : AUTHENTIFICATION
// ============================================================================
void apiLogin() {
  if (!g_configured || !g_pinSet) { sendErr(403, "Appareil non configuré"); return; }
  if (!csrfOk()) { sendErr(403, "Requête refusée"); return; }
  LoginSlot* s = loginSlot(clientIp());
  uint32_t wait = loginLockRemaining(s);
  if (wait) {
    char m[96];
    snprintf(m, sizeof(m), "Trop de tentatives. Réessayez dans %lu s.", (unsigned long)((wait + 999UL) / 1000UL));
    sendErr(429, m);
    return;
  }
  String pin = server.arg("pin");
  if (!isPin4(pin.c_str()) || !pinVerify(pin.c_str())) {
    loginFail(s);
    wait = loginLockRemaining(s);
    if (wait) {
      char m[96];
      snprintf(m, sizeof(m), "PIN incorrect. Accès bloqué %lu s.", (unsigned long)((wait + 999UL) / 1000UL));
      sendErr(429, m);
    } else {
      sendErr(403, "PIN incorrect");
    }
    return;
  }
  loginSuccess(s);
  Session* se = sessionCreate();
  char cookie[120];
  snprintf(cookie, sizeof(cookie), "sid=%s; Path=/; HttpOnly; SameSite=Strict; Max-Age=86400", se->token);
  server.sendHeader("Set-Cookie", cookie);
  DBG("SECURITY", "connexion réussie");
  logEvent("Connexion à l'interface");
  sendOk();
}

void apiLogout() {
  if (!csrfOk()) { sendErr(403, "Requête refusée"); return; }
  Session* s = currentSession();
  if (s) s->used = false;
  server.sendHeader("Set-Cookie", "sid=; Path=/; HttpOnly; SameSite=Strict; Max-Age=0");
  sendOk();
}

// ============================================================================
//  HTTP : ÉTAT, RADAR, WATCHLIST
// ============================================================================
void apiStatus() {
  if (!requireAuth()) return;
  int wat = 0, pres = 0, vis = 0;
  for (int i = 0; i < MAX_WATCHED; i++) {
    if (!g_watch[i].used) continue;
    wat++;
    if (g_watch[i].rt.state == WS_PRESENT || g_watch[i].rt.state == WS_NEAR) pres++;
  }
  for (int i = 0; i < MAX_VISIBLE; i++) if (g_vis[i].used) vis++;
  bool conn = WiFi.status() == WL_CONNECTED;
  String ip = conn ? WiFi.localIP().toString() : String("");
  char clk[20];
  fmtClockAtMs(millis(), clk, sizeof(clk));
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kInt("up", (long)(esp_timer_get_time() / 1000000LL));
  js.kInt("heap", (long)ESP.getFreeHeap());
  js.kInt("minHeap", (long)ESP.getMinFreeHeap());
  js.kStr("time", clk);
  js.kBool("tv", timeValid());
  js.key("wifi"); js.objOpen();
  js.kBool("c", conn);
  js.kStr("ssid", g_wifiSsid);
  js.kStr("ip", ip.c_str());
  js.kInt("rssi", conn ? (long)WiFi.RSSI() : 0L);
  js.kBool("ap", g_apActive);
  js.kStr("apSsid", g_apSsid);
  js.objClose();
  js.key("ble"); js.objOpen();
  js.kBool("ok", agoMs(g_bleLastCycleMs) < 10000UL);
  js.kInt("cyc", (long)g_bleCycles);
  js.kInt("drop", (long)obsDropped());
  js.objClose();
  js.kInt("vis", vis);
  js.kInt("wat", wat);
  js.kInt("pres", pres);
  int nApple = 0, nAndroid = 0, nWin = 0, nTrack = 0;
  for (int i = 0; i < MAX_VISIBLE; i++) {
    if (!g_vis[i].used || agoMs(g_vis[i].lastSeenMs) > 60000UL) continue;
    if (g_vis[i].mfgId == 0x004C && g_vis[i].hasMfg) {
      if (g_vis[i].appleType == 0x12 || g_vis[i].appleType == 0x16) nTrack++;
      else nApple++;
    } else if ((g_vis[i].svc16 == 0xFEAA || g_vis[i].svc16 == 0xFE2C || g_vis[i].svc16 == 0xFD5A) ||
               (g_vis[i].hasMfg && g_vis[i].mfgId == 0x0075)) nAndroid++;
    else if (g_vis[i].hasMfg && g_vis[i].mfgId == 0x0006) nWin++;
    else if (g_vis[i].svc16 == 0xFEED) nTrack++;
  }
  js.key("kinds"); js.objOpen();
  js.kInt("apple", nApple);
  js.kInt("android", nAndroid);
  js.kInt("win", nWin);
  js.kInt("track", nTrack);
  js.objClose();
  js.key("tg"); js.objOpen();
  js.kBool("conf", tgConfigured());
  js.kInt("sent", (long)g_tgSent);
  js.kInt("fail", (long)g_tgFailed);
  js.kInt("q", g_tgQ ? (long)uxQueueMessagesWaiting(g_tgQ) : 0L);
  js.kInt("last", g_tgLast);
  js.objClose();
  js.objClose();
  js.end();
}

// Étiquette de type déduite du Manufacturer Data / Service UUID (identifie le TYPE, pas l'appareil)
const char* deviceKind(const DetectedDevice& d) {
  if (d.hasMfg) {
    switch (d.mfgId) {
      case 0x004C:  // Apple : le 1er octet du message donne le sous-type
        switch (d.appleType) {
          case 0x02: return "Apple – iBeacon";
          case 0x05: return "Apple – AirDrop";
          case 0x06: return "Apple – HomeKit";
          case 0x07: return "Apple – AirPods";
          case 0x08: return "Apple – \"Dis Siri\"";
          case 0x09: return "Apple – AirPlay";
          case 0x0A: return "Apple – AirPrint";
          case 0x0B: return "Apple – Watch (appairage)";
          case 0x0C: return "Apple – Handoff";
          case 0x0D: return "Apple – Continuité (WiFi)";
          case 0x0E: return "Apple – Hotspot";
          case 0x0F: return "Apple – Continuité (réseau)";
          case 0x10: return "Apple – iPhone/iPad (Nearby)";
          case 0x12: return "Apple – FindMy / AirTag";
          case 0x16: return "Apple – AirTag (séparé)";
          default:   return "Apple";
        }
      case 0x0006: return "Microsoft – Windows";
      case 0x0075: return "Samsung";
      case 0x00E0: return "Google";
      case 0x0087: return "Garmin";
      case 0x0157: return "Huawei";
      case 0x038F: return "Xiaomi";
      case 0x05A7: return "Sonos";
      case 0x0499: return "Ruuvi (capteur)";
    }
  }
  switch (d.svc16) {
    case 0xFEAA: return "Google – Eddystone";
    case 0xFE2C: return "Google – Fast Pair";
    case 0xFD5A: return "Google – Android";
    case 0xFEED: return "Tile (tracker)";
    case 0xFE9F: return "Google";
    case 0x181A: return "Capteur environnemental";
    case 0x180F: return "Batterie (BLE)";
    case 0xFE95: return "Xiaomi (capteur)";
  }
  if (d.hasMfg) {
    static char buf[24];
    snprintf(buf, sizeof(buf), "Fabricant 0x%04X", d.mfgId);
    return buf;
  }
  return "";
}

void apiDevices() {
  if (!requireAuth()) return;
  char mac[18], mf[8];
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.key("list");
  js.arrOpen();
  for (int i = 0; i < MAX_VISIBLE; i++) {
    DetectedDevice& d = g_vis[i];
    if (!d.used) continue;
    formatMac(d.mac, mac);
    float dist = -1.0f;
    if (d.hasFilt) {
      if (d.watchIdx >= 0 && g_watch[d.watchIdx].used) dist = estimateDistance(d.filtRssi, watchR1(g_watch[d.watchIdx]), watchN(g_watch[d.watchIdx]));
      else dist = estimateDistance(d.filtRssi, g_set.rssi1mDefault, g_set.pathLoss);
    }
    js.objOpen();
    js.kStr("n", d.name);
    js.kStr("m", mac);
    js.kStr("t", addrTypeName(d.addrType, d.mac));
    js.kInt("r", d.rawRssi);
    js.kFloat("f", d.hasFilt ? d.filtRssi : (float)d.rawRssi, 1);
    js.kFloat("d", dist, 2);
    js.kInt("a", agoL(d.lastSeenMs));
    js.kInt("fs", agoL(d.firstSeenMs));
    js.kInt("w", d.watchIdx);
    js.kInt("c", (long)d.obsCount);
    if (d.hasMfg) snprintf(mf, sizeof(mf), "%04X", d.mfgId); else mf[0] = 0;
    js.kStr("mf", mf);
    js.kStr("kind", deviceKind(d));
    js.kStr("sv", d.svc);
    js.objClose();
  }
  js.arrClose();
  js.objClose();
  js.end();
}

void writeWatchJson(int i) {
  WatchedDevice& w = g_watch[i];
  WatchedRuntime& r = w.rt;
  char mac[18];
  formatMac(w.cfg.mac, mac);
  js.objOpen();
  js.kInt("i", i);
  js.kStr("name", w.cfg.name);
  js.kStr("mac", mac);
  js.kStr("ble", r.bleName);
  js.kInt("r", r.rawRssi);
  js.kFloat("f", r.hasFilt ? r.filtRssi : 0.0f, 1);
  js.kBool("hf", r.hasFilt);
  js.kFloat("d", watchDistance(w), 2);
  js.kInt("a", r.everSeen ? agoL(r.lastSeenMs) : -1L);
  js.kStr("st", WATCH_STATE_NAMES[r.state]);
  js.kInt("act", w.cfg.active);
  js.kInt("cal", w.cfg.calibrated);
  js.kInt("r1", w.cfg.rssi1m);
  js.kInt("r1e", watchR1(w));
  js.kFloat("pl", w.cfg.pathLoss, 1);
  js.kFloat("ple", watchN(w), 1);
  js.kInt("aa", w.cfg.alertAppear);
  js.kInt("am", w.cfg.appearMode);
  js.kInt("ac", w.cfg.confirmations);
  js.kInt("ap", w.cfg.alertProx);
  js.kFloat("pt", w.cfg.proxThreshold, 1);
  js.kFloat("ph", w.cfg.proxHyst, 1);
  js.kInt("ad", w.cfg.alertDisappear);
  js.kInt("dm", w.cfg.disappearMin);
  js.kInt("ar", w.cfg.absenceResetMin);
  js.kInt("are", w.cfg.absenceResetMin ? w.cfg.absenceResetMin : g_set.absenceResetMin);
  js.kInt("cd", w.cfg.cooldownSec);
  js.kInt("obs", (long)r.obsCount);
  js.kBool("armed", r.appearArmed);
  js.objClose();
}

void apiWatchlist() {
  if (!requireAuth()) return;
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kInt("max", MAX_WATCHED);
  js.key("list");
  js.arrOpen();
  for (int i = 0; i < MAX_WATCHED; i++) if (g_watch[i].used) writeWatchJson(i);
  js.arrClose();
  js.objClose();
  js.end();
}

void apiWatchAdd() {
  if (!requireAuthPost()) return;
  String ms = server.arg("mac");
  ms.trim();
  ms.toUpperCase();
  if (ms.length() == 0) { sendErr(400, "Adresse MAC requise"); return; }
  uint8_t mac[6];
  if (!parseMac(ms.c_str(), mac) || macIsZero(mac)) { sendErr(400, "Format MAC invalide (XX:XX:XX:XX:XX:XX)"); return; }
  char nm[33];
  if (!argText("name", nm, sizeof(nm), 1, true)) { sendErr(400, "Nom personnalisé requis (1 à 32 caractères)"); return; }
  if (findWatchIdx(mac) >= 0) { sendErr(409, "Cette MAC est déjà enregistrée"); return; }
  int slot = -1;
  for (int i = 0; i < MAX_WATCHED; i++) if (!g_watch[i].used) { slot = i; break; }
  if (slot < 0) { sendErr(409, "Maximum de 20 appareils atteint"); return; }

  WatchedDevice& w = g_watch[slot];
  memset(&w, 0, sizeof(w));
  w.used = true;
  WatchedConfig& c = w.cfg;
  c.magic = WATCH_MAGIC;
  memcpy(c.mac, mac, 6);
  strlcpy(c.name, nm, sizeof(c.name));
  c.active = 1;
  c.calibrated = 0;
  c.rssi1m = g_set.rssi1mDefault;
  c.pathLoss = 0.0f;
  c.alertAppear = 1;
  c.appearMode = g_set.appearModeDefault;
  c.confirmations = g_set.appearConfDefault;
  c.alertProx = 1;
  c.proxThreshold = 3.0f;
  c.proxHyst = 1.0f;
  c.alertDisappear = 1;
  c.disappearMin = g_set.disappearMinDefault;
  c.absenceResetMin = 0;
  c.cooldownSec = g_set.cooldownDefault;
  initWatchRuntime(w.rt);
  saveWatch(slot);
  rebuildWatchMacCache();
  logEvent("Appareil ajouté : %s", nm);
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kInt("i", slot);
  js.objClose();
  js.end();
}

void apiWatchUpdate() {
  if (!requireAuthPost()) return;
  long idx;
  if (!argLong("i", 0, MAX_WATCHED - 1, idx) || !g_watch[idx].used) { sendErr(400, "Appareil introuvable"); return; }
  WatchedDevice& w = g_watch[idx];
  WatchedConfig c = w.cfg;
  if (server.hasArg("name")) {
    char nm[33];
    if (!argText("name", nm, sizeof(nm), 1, true)) { sendErr(400, "Nom invalide (1 à 32 caractères)"); return; }
    strlcpy(c.name, nm, sizeof(c.name));
  }
  ARG_BOOL("active", c.active);
  ARG_BOOL("alertAppear", c.alertAppear);
  ARG_INT("appearMode", 0, 1, c.appearMode);
  ARG_INT("confirmations", 1, 10, c.confirmations);
  ARG_BOOL("alertProx", c.alertProx);
  ARG_FLOAT("proxThreshold", 0.3f, 30.0f, c.proxThreshold);
  ARG_FLOAT("proxHyst", 0.2f, 20.0f, c.proxHyst);
  ARG_BOOL("alertDisappear", c.alertDisappear);
  ARG_INT("disappearMin", 1, 60, c.disappearMin);
  ARG_INT("absenceMin", 0, 60, c.absenceResetMin);
  ARG_INT("cooldown", 0, 3600, c.cooldownSec);
  ARG_FLOAT("pathLoss", 0.0f, 4.5f, c.pathLoss);
  if (c.pathLoss > 0.0f && c.pathLoss < 1.5f) { sendErr(400, "Path-loss : 0 (global) ou 1.5 à 4.5"); return; }
  if (server.hasArg("uncal") && server.arg("uncal") == "1") { c.calibrated = 0; c.rssi1m = g_set.rssi1mDefault; }
  bool activeChanged = c.active != w.cfg.active;
  bool changed = memcmp(&c, &w.cfg, sizeof(c)) != 0;
  w.cfg = c;
  if (changed) {
    saveWatch((int)idx);
    if (activeChanged) logEvent("%s : notifications %s", c.name, c.active ? "activées" : "désactivées");
    else logEvent("%s : paramètres modifiés", c.name);
  }
  sendOk();
}

void apiWatchDelete() {
  if (!requireAuthPost()) return;
  long idx;
  if (!argLong("i", 0, MAX_WATCHED - 1, idx) || !g_watch[idx].used) { sendErr(400, "Appareil introuvable"); return; }
  char nm[33];
  strlcpy(nm, g_watch[idx].cfg.name, sizeof(nm));
  if (g_cal.running && g_cal.idx == idx) g_cal.running = false;
  if (g_cal.idx == idx) g_cal.done = false;
  g_watch[idx].used = false;
  removeWatchKey((int)idx);
  rebuildWatchMacCache();
  logEvent("Appareil supprimé : %s", nm);
  sendOk();
}

// ============================================================================
//  HTTP : CALIBRATION
// ============================================================================
void apiCalStart() {
  if (!requireAuthPost()) return;
  long idx;
  if (!argLong("i", 0, MAX_WATCHED - 1, idx) || !g_watch[idx].used) { sendErr(400, "Appareil introuvable"); return; }
  memset(&g_cal, 0, sizeof(g_cal));
  g_cal.running = true;
  g_cal.idx = (int8_t)idx;
  g_cal.startMs = millis();
  DBG("DISTANCE", "calibration démarrée : %s", g_watch[idx].cfg.name);
  logEvent("Calibration démarrée : %s", g_watch[idx].cfg.name);
  sendOk();
}

void apiCalStatus() {
  if (!requireAuth()) return;
  uint32_t el = g_cal.running ? agoMs(g_cal.startMs) : (g_cal.done ? CALIB_DURATION_MS : 0);
  if (el > CALIB_DURATION_MS) el = CALIB_DURATION_MS;
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kBool("run", g_cal.running);
  js.kBool("done", g_cal.done);
  js.kBool("okc", g_cal.ok);
  js.kInt("i", g_cal.idx);
  js.kInt("el", (long)el);
  js.kInt("dur", (long)CALIB_DURATION_MS);
  js.kInt("n", g_cal.n);
  js.kInt("res", g_cal.result);
  js.kStr("err", g_cal.msg);
  js.objClose();
  js.end();
}

void apiCalSave() {
  if (!requireAuthPost()) return;
  long idx;
  if (!argLong("i", 0, MAX_WATCHED - 1, idx) || !g_watch[idx].used) { sendErr(400, "Appareil introuvable"); return; }
  if (!g_cal.done || !g_cal.ok || g_cal.idx != idx) { sendErr(400, "Aucune calibration valide pour cet appareil"); return; }
  WatchedDevice& w = g_watch[idx];
  w.cfg.calibrated = 1;
  w.cfg.rssi1m = g_cal.result;
  saveWatch((int)idx);
  g_cal.done = false;
  DBG("DISTANCE", "%s calibré : %d dBm à 1 m", w.cfg.name, (int)w.cfg.rssi1m);
  logEvent("%s calibré : %d dBm à 1 m", w.cfg.name, (int)w.cfg.rssi1m);
  sendOk();
}

// ============================================================================
//  HTTP : JOURNAL
// ============================================================================
void apiEvents() {
  if (!requireAuth()) return;
  char clk[20];
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.key("list");
  js.arrOpen();
  for (int k = 0; k < g_eventCount; k++) {
    int idx = ((int)g_eventHead - 1 - k + MAX_EVENTS * 2) % MAX_EVENTS;
    EventLogEntry& e = g_events[idx];
    fmtClockAtMs(e.ms, clk, sizeof(clk));
    js.objOpen();
    js.kInt("id", (long)e.id);
    js.kStr("t", clk);
    js.kStr("x", e.text);
    js.objClose();
  }
  js.arrClose();
  js.objClose();
  js.end();
}

// ============================================================================
//  HTTP : PARAMÈTRES
// ============================================================================
void apiSettingsGet() {
  if (!requireAuth()) return;
  bool conn = WiFi.status() == WL_CONNECTED;
  String ip = conn ? WiFi.localIP().toString() : (g_apActive ? WiFi.softAPIP().toString() + " (secours)" : String(""));
  char clk[20];
  fmtClockAtMs(millis(), clk, sizeof(clk));
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kStr("mdns", g_set.mdnsName);
  js.kStr("ssid", g_wifiSsid);
  js.kStr("ip", ip.c_str());
  js.kBool("sta", conn);
  js.kBool("ap", g_apActive);
  js.kStr("apSsid", g_apSsid);
  js.kStr("apPass", g_apPass);
  js.kInt("tx", g_set.txLow);
  js.kInt("abs", g_set.absenceResetMin);
  js.kFloat("alpha", g_set.emaAlpha, 2);
  js.kInt("mode", g_set.appearModeDefault);
  js.kInt("conf", g_set.appearConfDefault);
  js.kInt("dis", g_set.disappearMinDefault);
  js.kInt("cd", g_set.cooldownDefault);
  js.kInt("uOn", g_set.unknownAlert);
  js.kInt("uConf", g_set.unknownConf);
  js.kInt("uRssi", g_set.unknownMinRssi);
  js.kInt("uWin", g_set.unknownWindowSec);
  js.kInt("uRnd", g_set.ignoreRandom);
  js.kInt("uMax", g_set.unknownMaxPerHour);
  js.kInt("known", knownCount());
  js.kBool("tgConf", tgConfigured());
  js.kStr("tgChat", g_tgChat);
  js.kFloat("n", g_set.pathLoss, 1);
  js.kInt("r1", g_set.rssi1mDefault);
  js.kStr("tz", g_set.tz);
  js.kStr("time", clk);
  js.kStr("fw", FW_VERSION);
  js.kStr("core", CORE_TARGET);
  js.kInt("heap", (long)ESP.getFreeHeap());
  js.kInt("minHeap", (long)ESP.getMinFreeHeap());
  js.objClose();
  js.end();
}

void settingsNetwork() {
  RadarSettings ns = g_set;
  char ssid[33];
  char pass[65];
  bool wifiChanged = false;
  if (server.hasArg("mdns")) {
    String m = server.arg("mdns");
    m.trim();
    m.toLowerCase();
    if (m.length() > 31 || !validHostname(m.c_str())) { sendErr(400, "Nom mDNS invalide (a-z, 0-9, tiret)"); return; }
    strlcpy(ns.mdnsName, m.c_str(), sizeof(ns.mdnsName));
  }
  ARG_BOOL("tx", ns.txLow);
  strlcpy(ssid, g_wifiSsid, sizeof(ssid));
  if (server.hasArg("ssid")) {
    if (!argText("ssid", ssid, sizeof(ssid), 1, false)) { sendErr(400, "SSID invalide (1 à 32 caractères)"); return; }
    if (strcmp(ssid, g_wifiSsid) != 0) wifiChanged = true;
  }
  strlcpy(pass, g_wifiPass, sizeof(pass));
  String p = server.arg("pass");
  if (p.length()) {
    if (!validWifiPass(p.c_str())) { sendErr(400, "Mot de passe Wi-Fi invalide (8 à 63 caractères)"); return; }
    strlcpy(pass, p.c_str(), sizeof(pass));
    wifiChanged = true;
  }
  bool mdnsChanged = strcmp(ns.mdnsName, g_set.mdnsName) != 0;
  bool txChanged = ns.txLow != g_set.txLow;
  String ap = server.arg("apPass");
  String ap2 = server.arg("apPass2");
  if (ap != ap2) { sendErr(400, "La confirmation du mot de passe secours ne correspond pas."); return; }
  if (ap.length() && (ap.length() != strlen(ap.c_str()) || !validWifiPass(ap.c_str()))) {
    sendErr(400, "Mot de passe secours invalide (8 à 63 caractères ASCII imprimables)"); return;
  }
  bool apChanged = ap.length() && strcmp(ap.c_str(), g_apPass) != 0;
  // Validate all fields before writing. Persist before acknowledging the change.
  if (apChanged && prefs.putString("appass", ap.c_str()) != ap.length()) {
    sendErr(500, "Impossible d’enregistrer le mot de passe secours. Réessayez."); return;
  }
  commitSettings(ns);
  if (wifiChanged) {
    strlcpy(g_wifiSsid, ssid, sizeof(g_wifiSsid));
    strlcpy(g_wifiPass, pass, sizeof(g_wifiPass));
    prefs.putString("wssid", g_wifiSsid);
    prefs.putString("wpass", g_wifiPass);
    DBG("NVS", "identifiants Wi-Fi enregistrés");
    logEvent("Paramètres Wi-Fi modifiés");
  }
  if (txChanged) applyTxPower();
  if (mdnsChanged) {
    WiFi.setHostname(g_set.mdnsName);
    if (WiFi.status() == WL_CONNECTED) startMdns();
    logEvent("Nom mDNS : %s.local", g_set.mdnsName);
  }
  if (apChanged) {
    strlcpy(g_apPass, ap.c_str(), sizeof(g_apPass));
    logEvent("Mot de passe du réseau de secours modifié");
  }
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kBool("reboot", apChanged);
  js.objClose();
  js.end();
  // Let the HTTP response reach the browser before disconnecting the AP.
  if (apChanged) requestReboot(false);
  else if (wifiChanged) wifiForceReconnect();
}

void settingsSecurity() {
  LoginSlot* s = loginSlot(clientIp());
  uint32_t wait = loginLockRemaining(s);
  if (wait) {
    char m[96];
    snprintf(m, sizeof(m), "Trop de tentatives. Réessayez dans %lu s.", (unsigned long)((wait + 999UL) / 1000UL));
    sendErr(429, m);
    return;
  }
  String o = server.arg("old");
  String n = server.arg("new");
  String n2 = server.arg("new2");
  if (!isPin4(o.c_str()) || !pinVerify(o.c_str())) { loginFail(s); sendErr(403, "PIN actuel incorrect"); return; }
  loginSuccess(s);
  if (!isPin4(n.c_str())) { sendErr(400, "Le nouveau PIN doit contenir exactement 4 chiffres."); return; }
  if (n != n2) { sendErr(400, "La confirmation ne correspond pas."); return; }
  pinSetNew(n.c_str());
  Session* cur = currentSession();
  for (int i = 0; i < MAX_SESSIONS; i++) if (&g_sessions[i] != cur) g_sessions[i].used = false;
  logEvent("PIN modifié");
  sendOk();
}

void settingsTelegram() {
  if (server.hasArg("clear") && server.arg("clear") == "1") {
    portENTER_CRITICAL(&g_credMux);
    g_tgToken[0] = 0;
    g_tgChat[0] = 0;
    portEXIT_CRITICAL(&g_credMux);
    prefs.putString("tgtok", "");
    prefs.putString("tgchat", "");
    logEvent("Configuration Telegram effacée");
    sendOk();
    return;
  }
  char tok[80];
  char chat[40];
  String t = server.arg("token");
  t.trim();
  String c = server.arg("chat");
  c.trim();
  if (t.length()) {
    if (t.length() >= sizeof(tok) || !validTgToken(t.c_str())) { sendErr(400, "Bot Token invalide"); return; }
    strlcpy(tok, t.c_str(), sizeof(tok));
  } else {
    strlcpy(tok, g_tgToken, sizeof(tok));
  }
  if (c.length() >= sizeof(chat) || (c.length() && !validChatId(c.c_str()))) { sendErr(400, "Chat ID invalide"); return; }
  strlcpy(chat, c.c_str(), sizeof(chat));
  if (tok[0] && !chat[0]) { sendErr(400, "Chat ID requis"); return; }
  portENTER_CRITICAL(&g_credMux);
  strlcpy(g_tgToken, tok, sizeof(g_tgToken));
  strlcpy(g_tgChat, chat, sizeof(g_tgChat));
  portEXIT_CRITICAL(&g_credMux);
  prefs.putString("tgtok", g_tgToken);
  prefs.putString("tgchat", g_tgChat);
  memset(tok, 0, sizeof(tok));
  DBG("NVS", "Telegram enregistré");
  logEvent("Paramètres Telegram modifiés");
  sendOk();
}

void apiSettingsPost() {
  if (!requireAuthPost()) return;
  String sec = server.arg("section");
  if (sec == "network") { settingsNetwork(); return; }
  if (sec == "security") { settingsSecurity(); return; }
  if (sec == "telegram") { settingsTelegram(); return; }
  RadarSettings ns = g_set;
  if (sec == "detection") {
    ARG_INT("abs", 1, 60, ns.absenceResetMin);
    ARG_FLOAT("alpha", 0.05f, 1.0f, ns.emaAlpha);
    ARG_INT("mode", 0, 1, ns.appearModeDefault);
    ARG_INT("conf", 1, 10, ns.appearConfDefault);
    ARG_INT("dis", 1, 60, ns.disappearMinDefault);
    ARG_INT("cd", 0, 3600, ns.cooldownDefault);
  } else if (sec == "unknown") {
    ARG_BOOL("uOn", ns.unknownAlert);
    ARG_INT("uConf", 2, 20, ns.unknownConf);
    ARG_INT("uRssi", -100, -30, ns.unknownMinRssi);
    ARG_INT("uWin", 5, 600, ns.unknownWindowSec);
    ARG_BOOL("uRnd", ns.ignoreRandom);
    ARG_INT("uMax", 1, 60, ns.unknownMaxPerHour);
    for (int i = 0; i < MAX_VISIBLE; i++) if (g_vis[i].used && !g_vis[i].unkDone) g_vis[i].unkCount = 0;
  } else if (sec == "distance") {
    ARG_FLOAT("n", 1.5f, 4.5f, ns.pathLoss);
    ARG_INT("r1", -100, -20, ns.rssi1mDefault);
  } else if (sec == "system") {
    char tz[48];
    if (!argText("tz", tz, sizeof(tz), 1, true) || !validTz(tz)) { sendErr(400, "Fuseau horaire invalide"); return; }
    strlcpy(ns.tz, tz, sizeof(ns.tz));
    setenv("TZ", ns.tz, 1);
    tzset();
    if (WiFi.status() == WL_CONNECTED) configTzTime(ns.tz, "pool.ntp.org", "time.cloudflare.com", "time.google.com");
  } else {
    sendErr(400, "Section inconnue");
    return;
  }
  if (commitSettings(ns)) logEvent("Paramètres modifiés (%s)", sec.c_str());
  sendOk();
}

// ============================================================================
//  HTTP : TELEGRAM, SYSTÈME
// ============================================================================
void apiTelegramTest() {
  if (!requireAuthPost()) return;
  if (!tgConfigured()) { sendErr(400, "Telegram non configuré"); return; }
  uint32_t id = tgEnqueue(TG_TEST, "test", "✅ ESP-C3 opérationnel\n\nLes notifications Telegram fonctionnent.");
  if (!id) { sendErr(503, "File Telegram indisponible"); return; }
  g_tgTestId = id;
  g_tgTestState = 1;
  g_tgTestErr[0] = 0;
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kInt("id", (long)id);
  js.objClose();
  js.end();
}

void apiTelegramTestStatus() {
  if (!requireAuth()) return;
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kInt("state", g_tgTestState);
  js.kStr("err", g_tgTestErr);
  js.objClose();
  js.end();
}

void apiReboot() {
  if (!requireAuthPost()) return;
  logEvent("Redémarrage demandé");
  sendOk();
  requestReboot(false);
}

void apiWifiReconnect() {
  if (!requireAuthPost()) return;
  sendOk();
  wifiForceReconnect();
}

void apiFactoryReset() {
  if (!requireAuthPost()) return;
  if (server.arg("confirm") != "RESET") { sendErr(400, "Confirmation manquante"); return; }
  DBG("NVS", "réinitialisation usine demandée");
  sendOk();
  requestReboot(true);
}

// ============================================================================
//  HTTP : ASSISTANT INITIAL (uniquement si non configuré)
// ============================================================================
void apiSetupSave() {
  if (g_configured) { sendErr(403, "Appareil déjà configuré"); return; }
  if (!csrfOk()) { sendErr(403, "Requête refusée"); return; }
  char ssid[33];
  if (!argText("ssid", ssid, sizeof(ssid), 1, false)) { sendErr(400, "SSID requis (1 à 32 caractères)"); return; }
  String p = server.arg("pass");
  if (p.length() && !validWifiPass(p.c_str())) { sendErr(400, "Mot de passe Wi-Fi invalide (8 à 63 caractères)"); return; }
  String pin = server.arg("pin");
  String pin2 = server.arg("pin2");
  if (!isPin4(pin.c_str())) { sendErr(400, "Le PIN doit contenir exactement 4 chiffres."); return; }
  if (pin != pin2) { sendErr(400, "Les PIN ne correspondent pas."); return; }
  String t = server.arg("token");
  t.trim();
  String c = server.arg("chat");
  c.trim();
  if (t.length() || c.length()) {
    if (t.length() >= sizeof(g_tgToken) || !validTgToken(t.c_str())) { sendErr(400, "Bot Token invalide"); return; }
    if (c.length() >= sizeof(g_tgChat) || !validChatId(c.c_str())) { sendErr(400, "Chat ID invalide"); return; }
  }
  strlcpy(g_wifiSsid, ssid, sizeof(g_wifiSsid));
  strlcpy(g_wifiPass, p.c_str(), sizeof(g_wifiPass));
  prefs.putString("wssid", g_wifiSsid);
  prefs.putString("wpass", g_wifiPass);
  pinSetNew(pin.c_str());
  portENTER_CRITICAL(&g_credMux);
  strlcpy(g_tgToken, t.c_str(), sizeof(g_tgToken));
  strlcpy(g_tgChat, c.c_str(), sizeof(g_tgChat));
  portEXIT_CRITICAL(&g_credMux);
  prefs.putString("tgtok", g_tgToken);
  prefs.putString("tgchat", g_tgChat);
  prefs.putBool("setup", true);
  DBG("NVS", "configuration initiale enregistrée");
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kStr("mdns", g_set.mdnsName);
  js.objClose();
  js.end();
  requestReboot(false);
}

void apiSetupTest() {
  if (g_configured) { sendErr(403, "Appareil déjà configuré"); return; }
  if (!csrfOk()) { sendErr(403, "Requête refusée"); return; }
  if (g_setupTestState == 1 || g_setupTestState == 2) { sendErr(409, "Test déjà en cours"); return; }
  char ssid[33];
  if (!argText("ssid", ssid, sizeof(ssid), 1, false)) { sendErr(400, "SSID requis"); return; }
  String p = server.arg("pass");
  if (p.length() && !validWifiPass(p.c_str())) { sendErr(400, "Mot de passe Wi-Fi invalide (8 à 63 caractères)"); return; }
  String t = server.arg("token");
  t.trim();
  String c = server.arg("chat");
  c.trim();
  if (t.length() >= sizeof(g_setupTgToken) || !validTgToken(t.c_str())) { sendErr(400, "Bot Token invalide"); return; }
  if (c.length() >= sizeof(g_setupTgChat) || !validChatId(c.c_str())) { sendErr(400, "Chat ID invalide"); return; }
  portENTER_CRITICAL(&g_credMux);
  strlcpy(g_setupTgToken, t.c_str(), sizeof(g_setupTgToken));
  strlcpy(g_setupTgChat, c.c_str(), sizeof(g_setupTgChat));
  portEXIT_CRITICAL(&g_credMux);
  g_setupTestState = 1;
  g_setupTestStartMs = millis();
  g_setupTestErr[0] = 0;
  sendOk();
  WiFi.mode(WIFI_AP_STA);
  WiFi.disconnect(false, false);
  WiFi.begin(ssid, p.length() ? p.c_str() : nullptr);
  applyTxPower();
  DBG("WIFI", "test assistant : connexion à %s", ssid);
}

void apiSetupTestStatus() {
  if (g_configured) { sendErr(403, "Appareil déjà configuré"); return; }
  js.begin(200);
  js.objOpen();
  js.kBool("ok", true);
  js.kInt("state", g_setupTestState);
  js.kStr("err", g_setupTestErr);
  js.objClose();
  js.end();
}

void setupRoutes() {
  const char* hdrs[] = { "Cookie", "X-Req" };
  server.collectHeaders(hdrs, 2);
  server.on("/", HTTP_GET, handleRoot);
  server.on("/s.css", HTTP_GET, handleCss);
  server.on("/favicon.ico", HTTP_GET, []() { server.send(204); });

  server.on("/api/login", HTTP_POST, apiLogin);
  server.on("/api/logout", HTTP_POST, apiLogout);
  server.on("/api/status", HTTP_GET, apiStatus);
  server.on("/api/devices", HTTP_GET, apiDevices);
  server.on("/api/watchlist", HTTP_GET, apiWatchlist);
  server.on("/api/watchlist/add", HTTP_POST, apiWatchAdd);
  server.on("/api/watchlist/update", HTTP_POST, apiWatchUpdate);
  server.on("/api/watchlist/delete", HTTP_POST, apiWatchDelete);
  server.on("/api/calibration/start", HTTP_POST, apiCalStart);
  server.on("/api/calibration/status", HTTP_GET, apiCalStatus);
  server.on("/api/calibration/save", HTTP_POST, apiCalSave);
  server.on("/api/events", HTTP_GET, apiEvents);
  server.on("/api/settings", HTTP_GET, apiSettingsGet);
  server.on("/api/settings", HTTP_POST, apiSettingsPost);
  server.on("/api/telegram/test", HTTP_POST, apiTelegramTest);
  server.on("/api/telegram/test", HTTP_GET, apiTelegramTestStatus);
  server.on("/api/reboot", HTTP_POST, apiReboot);
  server.on("/api/wifi/reconnect", HTTP_POST, apiWifiReconnect);
  server.on("/api/factory-reset", HTTP_POST, apiFactoryReset);

  server.on("/api/setup/save", HTTP_POST, apiSetupSave);
  server.on("/api/setup/test", HTTP_POST, apiSetupTest);
  server.on("/api/setup/test", HTTP_GET, apiSetupTestStatus);

  server.onNotFound(handleNotFound);
}

// ============================================================================
//  SUPERVISION 24/7
// ============================================================================
void supervisorTask(void* arg) {
  (void)arg;
  bool low = false;
  uint32_t lowSince = 0;
  for (;;) {
    vTaskDelay(pdMS_TO_TICKS(5000));
    if (agoMs(g_loopAliveMs) > 30000UL) {
      Serial.println("[BOOT] boucle principale bloquée -> redémarrage");
      delay(50);
      ESP.restart();
    }
    if (g_bleStarted && agoMs(g_bleLastCycleMs) > 60000UL) {
      Serial.println("[BLE] scan bloqué -> redémarrage");
      delay(50);
      ESP.restart();
    }
    uint32_t fh = ESP.getFreeHeap();
    if (fh < 14000UL) {
      if (!low) { low = true; lowSince = millis(); }
      else if (agoMs(lowSince) > 60000UL) {
        Serial.printf("[BOOT] mémoire critique (%lu octets) -> redémarrage\n", (unsigned long)fh);
        delay(50);
        ESP.restart();
      }
    } else {
      low = false;
    }
  }
}

void doReboot() {
  if (g_factoryPending) {
    DBG("NVS", "effacement complet");
    prefs.clear();
    prefs.end();
    WiFi.disconnect(true, true);
  } else {
    knownSave(true);
    prefs.end();
  }
  DBG("BOOT", "redémarrage");
  delay(200);
  ESP.restart();
}

// ============================================================================
//  SETUP
// ============================================================================
void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && (uint32_t)(millis() - t0) < 1500UL) delay(10);
  delay(100);
  DBG("BOOT", "ESP-C3 radar BLE v%s (%s) — %s", FW_VERSION, CORE_TARGET, ESP.getChipModel());

#if LED_PIN >= 0
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF_LEVEL);
#endif

  memset(&g_cal, 0, sizeof(g_cal));
  g_cal.idx = -1;

  // 1-3. NVS + validation
  prefs.begin("bleradar", false);
  loadAll();

  // 4. BLE : pile NimBLE initialisée par BLEDevice, scan piloté directement via l'API GAP NimBLE
  BLEDevice::init("");
  rebuildWatchMacCache();
  DBG("BLE", "scan actif initialisé (fenêtre %d ms / intervalle %d ms)", BLE_SCAN_WINDOW_MS, BLE_SCAN_INTERVAL_MS);

  // Identité AP (RNG matériel alimenté par la radio active)
  buildApSsid();
  // A password saved in the interface takes precedence over the compile-time default.
  if (!loadStr("appass", g_apPass, sizeof(g_apPass)) || !validWifiPass(g_apPass)) {
    if (strlen(AP_DEFAULT_PASS) >= 8) strlcpy(g_apPass, AP_DEFAULT_PASS, sizeof(g_apPass));
    else genPassword(g_apPass, 12);
    prefs.putString("appass", g_apPass);
    DBG("NVS", "mot de passe du réseau de configuration initialisé");
  }

  g_tgQ = xQueueCreate(TG_QUEUE_LEN, sizeof(TgMsg));
  g_tgResQ = xQueueCreate(8, sizeof(TgResult));

  g_bleLastCycleMs = millis();
  g_bleStarted = true;
  xTaskCreate(telegramTask, "telegram", 8192, nullptr, 1, nullptr);

  setenv("TZ", g_set.tz, 1);
  tzset();

  // 5-6. Wi-Fi / SoftAP (modem-sleep conservé : obligatoire en coexistence BLE)
  WiFi.persistent(false);
  WiFi.setAutoReconnect(false);
  WiFi.setHostname(g_set.mdnsName);
  g_wmDisconnectedSinceMs = millis();
  if (g_configured) {
    WiFi.mode(WIFI_STA);
    beginSta();
  } else {
    startSoftAP();
    Serial.printf("\n[AP] ===== CONFIGURATION INITIALE =====\n");
    Serial.printf("[AP] Réseau Wi-Fi : %s\n", g_apSsid);
    Serial.printf("[AP] Mot de passe : %s\n", g_apPass);
    Serial.printf("[AP] Portail      : http://192.168.4.1\n");
    Serial.printf("[AP] ==================================\n\n");
  }

  // 7. Serveur Web (8. mDNS et 9. NTP démarrent à la connexion STA)
  setupRoutes();
  server.begin();
  DBG("WEB", "serveur HTTP démarré sur le port 80");

  g_loopAliveMs = millis();
  xTaskCreate(supervisorTask, "superv", 3072, nullptr, 1, nullptr);
  logEvent("Démarrage (%s)", g_configured ? "configuré" : "configuration initiale");
}

// ============================================================================
//  LOOP
// ============================================================================
void loop() {
  static uint32_t lastTick = 0;
  static uint32_t lastWifi = 0;
  static uint32_t lastSlow = 0;

  g_loopAliveMs = millis();

  processQueues();
  if (elapsedMs(lastTick, 250)) { lastTick = millis(); watchTick(); calibrationTick(); }

  server.handleClient();
  processQueues();

  if (elapsedMs(lastWifi, 250)) { lastWifi = millis(); wifiLoop(); }
  static uint32_t lastBle = 0;
  if (elapsedMs(lastBle, 1000)) { lastBle = millis(); bleSupervise(); }
  drainTelegramResults();

  if (elapsedMs(lastSlow, 5000)) {
    lastSlow = millis();
    visExpire();
    sessionsExpire();
    knownSave(false);
    static uint8_t heapLogDiv = 0;
    if (++heapLogDiv >= 12) {
      heapLogDiv = 0;
      DBG("BOOT", "heap libre=%lu min=%lu bloc max=%lu",
          (unsigned long)ESP.getFreeHeap(), (unsigned long)ESP.getMinFreeHeap(), (unsigned long)ESP.getMaxAllocHeap());
    }
  }

  ledLoop();

  if (g_rebootPending && elapsedMs(g_rebootReqMs, 1500)) doReboot();

  delay(2);
}
