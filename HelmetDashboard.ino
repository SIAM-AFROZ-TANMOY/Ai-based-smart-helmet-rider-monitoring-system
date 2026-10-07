/* ==========================================================================
 *  Project Helmet — Dashboard + Accident Alert + Rider Check
 *  Board: ESP32 DevKit V1 (30-pin, WROOM-32)
 *
 *  Joins your Wi-Fi and serves a live dashboard:
 *    http://<ip>/  or  http://helmet.local/
 *
 *    GET /data               JSON telemetry
 *    GET /api/zero-yaw       set current heading as yaw = 0
 *    GET /api/calibrate      re-measure gyro bias (hold helmet STILL)
 *    GET /api/reset-peak     clear peak-impact reading
 *    GET /api/beep           sound the buzzer
 *    GET /api/check-rider    ESP32-CAM photo -> VLM: helmet worn? drowsy?
 *    GET /api/cancel         "I'm OK" during the accident countdown
 *    GET /api/clear-alert    clear a sent accident alert (back to ARMED)
 *    GET /api/disarm         lock accident detection again
 *    GET /api/test-telegram  send a Telegram test message
 *
 *  SAFETY FLOW
 *    LOCKED  --scan authorised RFID tag------------------------> ARMED
 *    ARMED   --impact > IMPACT_G, or helmet tipped > TILT_DEG
 *              for TILT_HOLD_MS----------------------------------> PENDING
 *    PENDING --no cancel within CANCEL_WINDOW_MS-----------------> ALERTED
 *              (Telegram message with GPS link is sent)
 *    PENDING --touch sensor / tag / "I'm OK" button--------------> ARMED
 *    ALERTED --tag / "Clear alert" button------------------------> ARMED
 *    ARMED   --tag again-----------------------------------------> LOCKED
 *
 *  RIDER CHECK
 *    Dashboard button -> this ESP32 calls http://<cam>/check -> the ESP32-CAM
 *    takes a photo, sends it to the VLM agent (DigitalOcean) and returns
 *    {helmet, drowsy, eyes, ...}. The snapshot is shown straight from the
 *    camera (http://<cam>/last.jpg), so your browser must be on the same LAN.
 *
 *  Network calls (Telegram, camera) run in FreeRTOS tasks on core 0, so the
 *  IMU, GPS and dashboard keep running while a request takes several seconds.
 *
 *  ATTITUDE NOTE
 *    roll + pitch -> absolute (gravity-referenced complementary filter).
 *    yaw          -> relative and drifting (no magnetometer).
 *
 *  Libraries: MFRC522, Adafruit SSD1306 + GFX, Adafruit MPU6050 +
 *             Unified Sensor, TinyGPSPlus, ArduinoJson (v7).
 *             WiFi / WebServer / ESPmDNS / HTTPClient / WiFiClientSecure
 *             are part of the ESP32 core.
 * ========================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>
#include <WiFiClientSecure.h>
#include <HTTPClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <MFRC522.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

// ==========================  FILL THESE IN  ==============================
// On the road the helmet will not see your home Wi-Fi: use a phone hotspot
// (2.4 GHz) here, or accident alerts cannot reach Telegram.
const char* WIFI_SSID = "Room_401A-2.4G";
const char* WIFI_PASS = "HLiNaKRbCsFr@gr01";
const char* HOSTNAME  = "helmet";            // -> http://helmet.local

// Telegram bot (DUMMY values — get a real token from @BotFather, and your
// chat id from @userinfobot or https://api.telegram.org/bot<token>/getUpdates)
const char* TG_BOT_TOKEN = "7412589630:AAH_DUMMY-TOKEN_replace_me_x9Kq2Lm8Zt";
const char* TG_CHAT_ID   = "123456789";

// ESP32-CAM on the same network (DUMMY IP — use the IP the cam prints on
// its serial monitor; a DHCP reservation in your router keeps it fixed).
const char* CAM_BASE_URL = "http://192.168.1.60";
const char* CAM_TOKEN    = "helmet-cam-7f3a9c";   // must match HelmetCam.ino

// RFID tags that may arm/disarm. The serial monitor prints "CARD xx:xx:.."
// for every scan. Set ALLOW_ANY_TAG = false once you've listed yours.
const bool  ALLOW_ANY_TAG = true;
const char* AUTH_UIDS[]   = { "DE:AD:BE:EF", "12:34:56:78" };
// =========================================================================

// ---------------- Pin map (matches HelmetSelfTest) ----------------
#define I2C_SDA       21
#define I2C_SCL       22

#define RFID_SS        5
#define RFID_RST      27
#define RFID_SCK      18
#define RFID_MOSI     23
#define RFID_MISO     19

#define GPS_RX        16
#define GPS_TX        17
#define GPS_BAUD    9600

#define GAS_PIN       32
#define TOUCH_PIN     33
#define LDR_PIN       34
#define BUZZER_PIN    25

// ---------------- Config ----------------
#define OLED_W       128
#define OLED_H        64
#define MPU_ADDR    0x68

#define BUZZER_ACTIVE_LOW  false
#define LDR_DARK_IS_LOW    false     // this board's D0 goes HIGH in darkness
#define TOUCH_ACTIVE_HIGH  true

const uint16_t GAS_ALARM_RAW = 400;    // baseline measured ~120; tune to taste
const float    COMP_ALPHA    = 0.98f;  // gyro trust in the complementary filter
const uint32_t IMU_MS        = 10;     // 100 Hz attitude update
const uint32_t OLED_MS       = 500;
const uint32_t RFID_POLL_MS  = 150;

// ---------------- Accident detection ----------------
const float    IMPACT_G         = 4.0f;   // |a| spike that counts as a crash
const float    TILT_DEG         = 60.0f;  // tipped this far from how it was
const uint32_t TILT_HOLD_MS     = 4000;   //   worn at arming, for this long
const uint32_t CANCEL_WINDOW_MS = 15000;  // rider's chance to say "I'm OK"
const uint32_t ARM_DEBOUNCE_MS  = 3000;   // ignore repeat taps of the tag

// ---------------- Rider check / Telegram behaviour ----------------
const bool     TG_BOOT_MSG      = true;   // "helmet online" message at boot
const bool     TG_ON_RIDER_WARN = true;   // Telegram if no helmet / drowsy
const uint32_t AUTO_CHECK_MS    = 0;      // e.g. 60000 = check every minute
                                          // while ARMED; 0 = manual only

// ---------------- Objects ----------------
WebServer          server(80);
MFRC522            rfid(RFID_SS, RFID_RST);
Adafruit_SSD1306   oled(OLED_W, OLED_H, &Wire, -1);
Adafruit_MPU6050   mpu;
TinyGPSPlus        gps;

// ---------------- Types (must stay above the first function) ----------------
enum MpuMode   { MPU_NONE, MPU_ADAFRUIT, MPU_RAW };
enum SafeState { SAFE_LOCKED, SAFE_ARMED, SAFE_PENDING, SAFE_ALERTED };

struct Motion { float ax, ay, az, gx, gy, gz, tempC; bool valid; };

struct TgMsg { bool critical; char text[700]; };

struct RiderResult {
  uint32_t seq;          // increments on every finished check
  bool     valid;        // VLM answered and was parsed
  bool     face;         // a rider was visible
  bool     helmet;
  bool     drowsy;
  char     eyes[16];
  float    conf;
  char     notes[160];
  char     err[80];
  uint32_t atMs, tookMs;
};

// ---------------- State ----------------
MpuMode mpuMode   = MPU_NONE;
uint8_t mpuWhoAmI = 0xFF;

bool     oledOK = false, rfidOK = false;
uint8_t  oledAddr = 0, rfidVersion = 0;

float roll = 0, pitch = 0, yaw = 0;      // degrees
float yawOffset = 0;
float gBias[3] = {0, 0, 0};              // rad/s, removed before integration
float peakG = 0;                         // peak |a| in g
float lastMagG = 1.0f;                   // latest |a| in g
float lpA[3]  = {0, 0, 9.8f};            // low-passed accel (gravity estimate)
float refG[3] = {0, 0, 1};               // unit gravity vector at arming
uint32_t lastImu = 0, lastOled = 0, lastRfidPoll = 0, lastWifiTry = 0;
uint32_t lastAutoCheck = 0;
bool     lastTouch = false;
uint32_t cardCount = 0, lastCardMs = 0;
String   lastUid = "none";

// Buzzer: single beep + simple pattern player
uint32_t beepUntil = 0;
uint8_t  patLeft = 0;
uint16_t patOn = 0, patOff = 0;
uint32_t patNext = 0;
bool     patState = false;

// Safety
SafeState safeState = SAFE_LOCKED;
String    armedUid = "";
uint32_t  pendingSince = 0, tiltSince = 0, lastArmToggle = 0, alertMs = 0;
float     triggerG = 0;
char      triggerWhy[24] = "";
uint32_t  accidentCount = 0;

// Telegram
QueueHandle_t     tgQueue = nullptr;
volatile uint32_t tgSent = 0, tgFailed = 0;
char              tgLastErr[64] = "none";

// Rider check
RiderResult   rider = {};
volatile bool riderBusy = false;
TaskHandle_t  riderTaskH = nullptr;
portMUX_TYPE  riderMux = portMUX_INITIALIZER_UNLOCKED;

// ==========================================================================
//  Small helpers
// ==========================================================================
void buzzer(bool on) { digitalWrite(BUZZER_PIN, BUZZER_ACTIVE_LOW ? !on : on); }

void beep(uint16_t ms) {
  patLeft = 0;
  buzzer(true);
  beepUntil = millis() + ms;
}

void beepPattern(uint8_t count, uint16_t onMs, uint16_t offMs) {
  beepUntil = 0;
  patLeft = count; patOn = onMs; patOff = offMs;
  patNext = millis(); patState = false;
}

void serviceBeep() {
  uint32_t now = millis();
  if (beepUntil && (int32_t)(now - beepUntil) >= 0) { buzzer(false); beepUntil = 0; }
  if (patLeft && (int32_t)(now - patNext) >= 0) {
    if (!patState) { buzzer(true);  patState = true;  patNext = now + patOn; }
    else           { buzzer(false); patState = false; patLeft--; patNext = now + patOff; }
  }
}

bool i2cPresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

bool i2cWrite8(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg); Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool i2cRead(uint8_t addr, uint8_t reg, uint8_t* buf, uint8_t len) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)len, (int)true) != len) return false;
  for (uint8_t i = 0; i < len; i++) buf[i] = Wire.read();
  return true;
}

bool ldrDark() {
  bool d = digitalRead(LDR_PIN);
  return LDR_DARK_IS_LOW ? !d : d;
}

bool touched() {
  bool t = digitalRead(TOUCH_PIN);
  return TOUCH_ACTIVE_HIGH ? t : !t;
}

// Escape text for embedding inside a JSON string.
void jsonEsc(const char* in, char* out, size_t cap) {
  size_t o = 0;
  for (; *in && o + 7 < cap; in++) {
    unsigned char c = (unsigned char)*in;
    if (c == '"' || c == '\\') { out[o++] = '\\'; out[o++] = c; }
    else if (c < 0x20)         { o += snprintf(out + o, cap - o, "\\u%04x", c); }
    else                       { out[o++] = c; }
  }
  out[o] = 0;
}

void utcString(char* out, size_t cap) {
  if (gps.date.isValid() && gps.time.isValid()) {
    snprintf(out, cap, "%04u-%02u-%02u %02u:%02u:%02u",
             gps.date.year(), gps.date.month(), gps.date.day(),
             gps.time.hour(), gps.time.minute(), gps.time.second());
  } else {
    strlcpy(out, "--", cap);
  }
}

// Google Maps link (or a clear "no fix" line) for Telegram messages.
void locationText(char* out, size_t cap) {
  if (gps.location.isValid()) {
    snprintf(out, cap,
             "https://maps.google.com/?q=%.6f,%.6f\n(sats %lu, HDOP %.1f, fix age %lu s)",
             gps.location.lat(), gps.location.lng(),
             (unsigned long)gps.satellites.value(),
             gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
             (unsigned long)(gps.location.age() / 1000));
  } else {
    strlcpy(out, "NO GPS FIX - location unknown", cap);
  }
}

const char* safeName(SafeState s) {
  switch (s) {
    case SAFE_LOCKED:  return "LOCKED";
    case SAFE_ARMED:   return "ARMED";
    case SAFE_PENDING: return "PENDING";
    case SAFE_ALERTED: return "ALERTED";
  }
  return "?";
}

uint32_t countdownS() {
  if (safeState != SAFE_PENDING) return 0;
  uint32_t el = millis() - pendingSince;
  return el >= CANCEL_WINDOW_MS ? 0 : (CANCEL_WINDOW_MS - el + 999) / 1000;
}

// ==========================================================================
//  IMU — works with genuine MPU6050 and with MPU6500/9250 clones
//  Range raised to +-16 g: at +-2 g every crash would clip at 2 g and the
//  impact threshold could never be reached.
// ==========================================================================
static const float ACC_LSB = 2048.0f;    // +-16 g
static const float GYR_LSB = 65.5f;      // +-500 dps

Motion readMotionRaw() {
  Motion m; m.valid = false;
  uint8_t b[14];
  if (!i2cRead(MPU_ADDR, 0x3B, b, 14)) return m;

  int16_t rax = (int16_t)((b[0] << 8) | b[1]);
  int16_t ray = (int16_t)((b[2] << 8) | b[3]);
  int16_t raz = (int16_t)((b[4] << 8) | b[5]);
  int16_t rt  = (int16_t)((b[6] << 8) | b[7]);
  if (rax == 0 && ray == 0 && raz == 0) return m;

  m.ax = rax / ACC_LSB * 9.80665f;
  m.ay = ray / ACC_LSB * 9.80665f;
  m.az = raz / ACC_LSB * 9.80665f;
  m.tempC = rt / 340.0f + 36.53f;
  m.gx = (int16_t)((b[8]  << 8) | b[9])  / GYR_LSB * DEG_TO_RAD;
  m.gy = (int16_t)((b[10] << 8) | b[11]) / GYR_LSB * DEG_TO_RAD;
  m.gz = (int16_t)((b[12] << 8) | b[13]) / GYR_LSB * DEG_TO_RAD;
  m.valid = true;
  return m;
}

Motion readMotion() {
  if (mpuMode == MPU_RAW) return readMotionRaw();
  Motion m; m.valid = false;
  if (mpuMode != MPU_ADAFRUIT) return m;
  sensors_event_t a, g, tmp;
  mpu.getEvent(&a, &g, &tmp);
  m.ax = a.acceleration.x; m.ay = a.acceleration.y; m.az = a.acceleration.z;
  m.gx = g.gyro.x;         m.gy = g.gyro.y;         m.gz = g.gyro.z;
  m.tempC = tmp.temperature;
  m.valid = true;
  return m;
}

void initMPU() {
  mpuMode = MPU_NONE;
  if (!i2cRead(MPU_ADDR, 0x75, &mpuWhoAmI, 1)) mpuWhoAmI = 0xFF;

  if (mpu.begin(MPU_ADDR, &Wire)) {
    mpuMode = MPU_ADAFRUIT;
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  } else if (i2cPresent(MPU_ADDR)) {
    i2cWrite8(MPU_ADDR, 0x6B, 0x80); delay(100);   // reset
    i2cWrite8(MPU_ADDR, 0x6B, 0x00); delay(100);   // wake
    i2cWrite8(MPU_ADDR, 0x1A, 0x03);               // DLPF ~44 Hz
    i2cWrite8(MPU_ADDR, 0x1B, 0x08);               // gyro  +-500 dps
    i2cWrite8(MPU_ADDR, 0x1C, 0x18);               // accel +-16 g
    delay(50);
    if (readMotionRaw().valid) mpuMode = MPU_RAW;
  }
}

void calibrateGyro() {
  if (mpuMode == MPU_NONE) return;
  Serial.print(F("Calibrating gyro — hold still"));
  double sx = 0, sy = 0, sz = 0;
  const int N = 300;
  int got = 0;
  for (int i = 0; i < N; i++) {
    Motion m = readMotion();
    if (m.valid) { sx += m.gx; sy += m.gy; sz += m.gz; got++; }
    if (i % 60 == 0) Serial.print('.');
    delay(5);
  }
  if (got) {
    gBias[0] = sx / got; gBias[1] = sy / got; gBias[2] = sz / got;
  }
  Serial.printf(" done  bias[%.4f %.4f %.4f] rad/s\n",
                gBias[0], gBias[1], gBias[2]);
  Motion m = readMotion();
  if (m.valid) {
    roll  = atan2(m.ay, m.az) * RAD_TO_DEG;
    pitch = atan2(-m.ax, sqrt(m.ay * m.ay + m.az * m.az)) * RAD_TO_DEG;
    lpA[0] = m.ax; lpA[1] = m.ay; lpA[2] = m.az;
  }
  yaw = 0; yawOffset = 0;
}

void updateAttitude() {
  static uint32_t lastUs = 0;
  Motion m = readMotion();
  if (!m.valid) return;

  uint32_t now = micros();
  float dt = lastUs ? (now - lastUs) / 1000000.0f : 0.01f;
  lastUs = now;
  if (dt <= 0 || dt > 0.5f) return;

  float gx = (m.gx - gBias[0]) * RAD_TO_DEG;
  float gy = (m.gy - gBias[1]) * RAD_TO_DEG;
  float gz = (m.gz - gBias[2]) * RAD_TO_DEG;

  float rollAcc  = atan2(m.ay, m.az) * RAD_TO_DEG;
  float pitchAcc = atan2(-m.ax, sqrt(m.ay * m.ay + m.az * m.az)) * RAD_TO_DEG;

  roll  = COMP_ALPHA * (roll  + gx * dt) + (1 - COMP_ALPHA) * rollAcc;
  pitch = COMP_ALPHA * (pitch + gy * dt) + (1 - COMP_ALPHA) * pitchAcc;
  yaw  += gz * dt;

  while (yaw >= 360) yaw -= 360;
  while (yaw <    0) yaw += 360;

  // ~0.2 s low-pass of the accelerometer = where gravity points right now.
  const float LP = 0.05f;
  lpA[0] += LP * (m.ax - lpA[0]);
  lpA[1] += LP * (m.ay - lpA[1]);
  lpA[2] += LP * (m.az - lpA[2]);

  lastMagG = sqrt(m.ax * m.ax + m.ay * m.ay + m.az * m.az) / 9.80665f;
  if (lastMagG > peakG) peakG = lastMagG;
}

float yawDisplay() {
  float y = yaw - yawOffset;
  while (y >= 360) y -= 360;
  while (y <    0) y += 360;
  return y;
}

// Angle between gravity now and gravity when the helmet was armed. Works no
// matter how the MPU is mounted inside the shell.
float tiltFromRef() {
  float n = sqrt(lpA[0] * lpA[0] + lpA[1] * lpA[1] + lpA[2] * lpA[2]);
  if (n < 1.0f) return 0;
  float d = (lpA[0] * refG[0] + lpA[1] * refG[1] + lpA[2] * refG[2]) / n;
  if (d >  1) d =  1;
  if (d < -1) d = -1;
  return acosf(d) * RAD_TO_DEG;
}

// ==========================================================================
//  Telegram (runs in its own task; loop only queues messages)
// ==========================================================================
bool tgQueueMessage(const char* text, bool critical) {
  if (!tgQueue) return false;
  TgMsg msg;
  msg.critical = critical;
  strlcpy(msg.text, text, sizeof(msg.text));
  bool ok = xQueueSend(tgQueue, &msg, 0) == pdTRUE;
  if (!ok) Serial.println(F("Telegram queue full — message dropped"));
  return ok;
}

bool tgSendNow(const char* text) {
  WiFiClientSecure client;
  client.setInsecure();       // skips certificate check; fine for a hobby build
  HTTPClient http;
  http.setConnectTimeout(10000);
  http.setTimeout(15000);

  String url = String("https://api.telegram.org/bot") + TG_BOT_TOKEN + "/sendMessage";
  if (!http.begin(client, url)) {
    strlcpy(tgLastErr, "http.begin failed", sizeof(tgLastErr));
    return false;
  }
  http.addHeader("Content-Type", "application/json");

  JsonDocument doc;
  doc["chat_id"] = TG_CHAT_ID;
  doc["text"]    = text;
  String body;
  serializeJson(doc, body);

  int code = http.POST(body);
  String resp = code > 0 ? http.getString() : "";
  http.end();

  if (code == 200 && resp.indexOf("\"ok\":true") >= 0) {
    strlcpy(tgLastErr, "none", sizeof(tgLastErr));
    return true;
  }
  if (code == 401)      strlcpy(tgLastErr, "401 bad bot token", sizeof(tgLastErr));
  else if (code == 400) strlcpy(tgLastErr, "400 bad chat id?", sizeof(tgLastErr));
  else snprintf(tgLastErr, sizeof(tgLastErr), "HTTP %d", code);
  return false;
}

void telegramTask(void*) {
  static TgMsg msg;
  for (;;) {
    if (xQueueReceive(tgQueue, &msg, portMAX_DELAY) != pdTRUE) continue;
    // Accident alerts keep retrying for ~10 minutes (e.g. hotspot coming back
    // in range); everything else gets 3 tries.
    int tries = msg.critical ? 60 : 3;
    bool ok = false;
    for (int i = 0; i < tries && !ok; i++) {
      if (WiFi.status() != WL_CONNECTED) {
        strlcpy(tgLastErr, "waiting for Wi-Fi", sizeof(tgLastErr));
        vTaskDelay(pdMS_TO_TICKS(10000));
        continue;
      }
      ok = tgSendNow(msg.text);
      if (!ok) vTaskDelay(pdMS_TO_TICKS(msg.critical ? 10000 : 2000));
    }
    if (ok) tgSent++; else tgFailed++;
    Serial.printf("Telegram %s (%s)\n", ok ? "sent" : "FAILED", tgLastErr);
  }
}

// ==========================================================================
//  Rider check (runs in its own task; triggered with xTaskNotifyGive)
// ==========================================================================
void requestRiderCheck() {
  if (riderTaskH && !riderBusy) xTaskNotifyGive(riderTaskH);
}

void riderTask(void*) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    riderBusy = true;
    RiderResult r = {};
    uint32_t t0 = millis();

    if (WiFi.status() != WL_CONNECTED) {
      strlcpy(r.err, "Wi-Fi offline", sizeof(r.err));
    } else {
      HTTPClient http;
      http.setConnectTimeout(5000);
      http.setTimeout(45000);            // camera waits for the VLM
      String url = String(CAM_BASE_URL) + "/check?token=" + CAM_TOKEN +
                   "&flash=" + (ldrDark() ? "1" : "0");
      if (http.begin(url)) {
        int code = http.GET();
        if (code == 200) {
          String body = http.getString();
          JsonDocument d;
          if (deserializeJson(d, body) == DeserializationError::Ok) {
            if (d["ok"] | false) {
              r.valid  = true;
              r.face   = d["face"]   | true;
              r.helmet = d["helmet"] | false;
              r.drowsy = d["drowsy"] | false;
              r.conf   = d["confidence"] | 0.0f;
              strlcpy(r.eyes,  d["eyes"]  | "unknown", sizeof(r.eyes));
              strlcpy(r.notes, d["notes"] | "",        sizeof(r.notes));
            } else {
              strlcpy(r.err, d["error"] | "camera reported a failure", sizeof(r.err));
            }
          } else {
            strlcpy(r.err, "camera sent invalid JSON", sizeof(r.err));
          }
        } else if (code < 0) {
          snprintf(r.err, sizeof(r.err), "camera unreachable at %s", CAM_BASE_URL);
        } else {
          snprintf(r.err, sizeof(r.err), "camera HTTP %d", code);
        }
        http.end();
      } else {
        strlcpy(r.err, "bad CAM_BASE_URL", sizeof(r.err));
      }
    }

    r.tookMs = millis() - t0;
    r.atMs   = millis();
    portENTER_CRITICAL(&riderMux);
    r.seq = rider.seq + 1;
    rider = r;
    portEXIT_CRITICAL(&riderMux);
    riderBusy = false;

    if (r.valid) {
      Serial.printf("RIDER helmet=%d drowsy=%d eyes=%s conf=%.2f (%lu ms)\n",
                    r.helmet, r.drowsy, r.eyes, r.conf, (unsigned long)r.tookMs);
    } else {
      Serial.printf("RIDER check failed: %s\n", r.err);
    }

    if (r.valid && r.face && TG_ON_RIDER_WARN && (r.drowsy || !r.helmet)) {
      char loc[200], msg[700];
      locationText(loc, sizeof(loc));
      snprintf(msg, sizeof(msg),
        "⚠️ Rider check warning — Project Helmet\n\n"
        "Helmet: %s\nDrowsy: %s (eyes: %s)\nConfidence: %d%%\nNotes: %s\n"
        "Rider tag: %s\n\nLocation:\n%s",
        r.helmet ? "worn" : "NOT WORN",
        r.drowsy ? "YES" : "no", r.eyes, (int)(r.conf * 100), r.notes,
        armedUid.length() ? armedUid.c_str() : "not armed", loc);
      tgQueueMessage(msg, false);
    }
  }
}

// ==========================================================================
//  Safety state machine
// ==========================================================================
bool isAuthorized(const String& uid) {
  if (ALLOW_ANY_TAG) return true;
  for (const char* a : AUTH_UIDS) if (uid.equalsIgnoreCase(a)) return true;
  return false;
}

void armSafety(const String& uid) {
  float n = sqrt(lpA[0] * lpA[0] + lpA[1] * lpA[1] + lpA[2] * lpA[2]);
  if (n > 5.0f) { refG[0] = lpA[0] / n; refG[1] = lpA[1] / n; refG[2] = lpA[2] / n; }
  else          { refG[0] = 0; refG[1] = 0; refG[2] = 1; }
  armedUid  = uid;
  safeState = SAFE_ARMED;
  tiltSince = 0;
  peakG     = 0;
  beepPattern(2, 80, 80);
  Serial.printf("SAFETY armed by %s\n", uid.c_str());
}

void disarmSafety(const char* by) {
  safeState = SAFE_LOCKED;
  armedUid  = "";
  tiltSince = 0;
  beepPattern(1, 300, 0);
  Serial.printf("SAFETY locked (%s)\n", by);
}

void startPending(const char* why, float g) {
  safeState    = SAFE_PENDING;
  pendingSince = millis();
  triggerG     = g;
  strlcpy(triggerWhy, why, sizeof(triggerWhy));
  Serial.printf("SAFETY possible accident: %s %.2f g — %lu s to cancel\n",
                why, g, (unsigned long)(CANCEL_WINDOW_MS / 1000));
  requestRiderCheck();          // grab a rider snapshot for the dashboard
}

void cancelPending(const char* by) {
  if (safeState != SAFE_PENDING) return;
  safeState = SAFE_ARMED;
  tiltSince = 0;
  peakG     = 0;
  beepPattern(3, 50, 60);
  Serial.printf("SAFETY cancelled by %s\n", by);
}

void fireAccident() {
  safeState = SAFE_ALERTED;
  alertMs   = millis();
  accidentCount++;

  char loc[200], utc[24], msg[700];
  locationText(loc, sizeof(loc));
  utcString(utc, sizeof(utc));
  snprintf(msg, sizeof(msg),
    "🚨 ACCIDENT DETECTED — Project Helmet\n\n"
    "Trigger: %s (%.1f g)\n"
    "Rider tag: %s\n"
    "No response within %lu s.\n\n"
    "Location:\n%s\n\n"
    "Speed: %.1f km/h\nTilt now: %.0f°\nUTC: %s",
    triggerWhy, triggerG, armedUid.c_str(),
    (unsigned long)(CANCEL_WINDOW_MS / 1000), loc,
    gps.speed.isValid() ? gps.speed.kmph() : 0.0,
    tiltFromRef(), utc);
  tgQueueMessage(msg, true);
  Serial.println(F("SAFETY ACCIDENT ALERT queued for Telegram"));
}

void clearAlert(const char* by) {
  if (safeState != SAFE_ALERTED) return;
  safeState = SAFE_ARMED;
  tiltSince = 0;
  peakG     = 0;
  beepPattern(2, 150, 100);
  char msg[160];
  snprintf(msg, sizeof(msg), "✅ Accident alert cleared (%s). Rider tag: %s",
           by, armedUid.c_str());
  tgQueueMessage(msg, false);
  Serial.printf("SAFETY alert cleared by %s\n", by);
}

void handleTag(const String& uid) {
  if (!isAuthorized(uid)) {
    Serial.printf("CARD %s not authorised\n", uid.c_str());
    beepPattern(1, 700, 0);
    return;
  }
  if (millis() - lastArmToggle < ARM_DEBOUNCE_MS) return;
  lastArmToggle = millis();

  switch (safeState) {
    case SAFE_LOCKED:  armSafety(uid);            break;
    case SAFE_ARMED:   disarmSafety("tag");       break;
    case SAFE_PENDING: cancelPending("tag");      break;
    case SAFE_ALERTED: clearAlert("tag scanned"); break;
  }
}

void updateSafety() {
  uint32_t now = millis();
  switch (safeState) {
    case SAFE_ARMED: {
      if (lastMagG > IMPACT_G) { startPending("impact", lastMagG); break; }
      if (tiltFromRef() > TILT_DEG) {
        if (!tiltSince) tiltSince = now;
        else if (now - tiltSince >= TILT_HOLD_MS) startPending("tipped over", peakG);
      } else {
        tiltSince = 0;
      }
      break;
    }
    case SAFE_PENDING:
      if (now - pendingSince >= CANCEL_WINDOW_MS) fireAccident();
      break;
    default:
      break;
  }
}

// Countdown: fast beeps. After the alert: a slow locator chirp.
void serviceAlarm() {
  static uint32_t last = 0;
  uint32_t now = millis();
  if (safeState == SAFE_PENDING && now - last >= 500)  { last = now; beep(250); }
  if (safeState == SAFE_ALERTED && now - last >= 3000) { last = now; beep(150); }
}

// ==========================================================================
//  RFID / GPS
// ==========================================================================
void initRFID() {
  rfid.PCD_Init();
  delay(50);
  rfidVersion = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  rfidOK = (rfidVersion != 0x00 && rfidVersion != 0xFF);
}

void pollRFID() {
  if (!rfidOK || millis() - lastRfidPoll < RFID_POLL_MS) return;
  lastRfidPoll = millis();
  if (!rfid.PICC_IsNewCardPresent()) return;
  if (!rfid.PICC_ReadCardSerial())   return;

  String uid;
  for (byte i = 0; i < rfid.uid.size; i++) {
    if (rfid.uid.uidByte[i] < 0x10) uid += '0';
    uid += String(rfid.uid.uidByte[i], HEX);
    if (i + 1 < rfid.uid.size) uid += ':';
  }
  uid.toUpperCase();
  lastUid = uid;
  cardCount++;
  lastCardMs = millis();
  Serial.printf("CARD %s\n", uid.c_str());
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  handleTag(uid);
}

void feedGPS() { while (Serial2.available()) gps.encode(Serial2.read()); }

// ==========================================================================
//  Dashboard page
// ==========================================================================
const char INDEX_HTML[] PROGMEM = R"HTML(<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Project Helmet</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<style>
:root{--bg:#0e1116;--card:#171c24;--line:#242c38;--tx:#e6edf3;--dim:#8b96a5;
      --ok:#3fb950;--warn:#d29922;--bad:#f85149;--acc:#58a6ff}
*{box-sizing:border-box}
body{margin:0;padding:16px;background:var(--bg);color:var(--tx);
     font:14px/1.5 ui-sans-serif,system-ui,-apple-system,"Segoe UI",sans-serif}
h1{font-size:18px;margin:0 0 2px}
header{display:flex;justify-content:space-between;align-items:center;
       flex-wrap:wrap;gap:8px;margin-bottom:14px}
.sub{color:var(--dim);font-size:12px}
.dot{display:inline-block;width:8px;height:8px;border-radius:50%;
     background:var(--bad);margin-right:6px}
.dot.on{background:var(--ok)}
.grid{display:grid;gap:12px;grid-template-columns:repeat(auto-fit,minmax(250px,1fr))}
.card{background:var(--card);border:1px solid var(--line);border-radius:10px;padding:14px}
.card h2{font-size:12px;color:var(--dim);margin:0 0 10px;font-weight:600}
.row{display:flex;justify-content:space-between;gap:10px;padding:3px 0}
.row span:first-child{color:var(--dim);flex:none}
.mono{font-variant-numeric:tabular-nums;font-family:ui-monospace,Consolas,monospace}
.big{font-size:26px;font-weight:600;line-height:1.1}
.trip{display:flex;gap:10px;text-align:center;margin-bottom:10px}
.trip>div{flex:1;background:#0e1319;border:1px solid var(--line);
          border-radius:8px;padding:8px 4px}
.trip small{display:block;color:var(--dim);font-size:11px}
.viz{display:flex;gap:12px;align-items:center;justify-content:center;margin:6px 0 12px}
.horizon{position:relative;width:132px;height:132px;border-radius:50%;
         overflow:hidden;border:2px solid var(--line);flex:none;background:#000}
.sky{position:absolute;left:-60%;top:-60%;width:220%;height:220%;
     background:linear-gradient(#2d7dd2 0 50%,#7a5c3e 50% 100%);
     transform-origin:50% 50%}
.cross{position:absolute;inset:0;pointer-events:none}
.cross:before,.cross:after{content:"";position:absolute;background:#ffcc33}
.cross:before{left:26%;right:26%;top:50%;height:2px;margin-top:-1px}
.cross:after{left:50%;top:44%;height:12px;width:2px;margin-left:-1px}
.compass{position:relative;width:132px;height:132px;border-radius:50%;
         border:2px solid var(--line);flex:none;background:#0e1319}
.needle{position:absolute;left:50%;top:50%;width:3px;height:52px;
        background:linear-gradient(var(--bad) 0 50%,#5b6672 50% 100%);
        transform-origin:50% 100%;margin:-52px 0 0 -1.5px;border-radius:2px}
.compass b{position:absolute;left:50%;transform:translateX(-50%);top:5px;
           font-size:10px;color:var(--dim)}
.pill{font-size:11px;padding:2px 8px;border-radius:99px;border:1px solid var(--line)}
.pill.ok{color:var(--ok);border-color:#1d4429}.pill.bad{color:var(--bad);border-color:#5c2023}
.pill.warn{color:var(--warn);border-color:#5a4415}
.note{color:var(--warn);font-size:11px;margin-top:8px;line-height:1.4}
button{background:#21262d;color:var(--tx);border:1px solid var(--line);
       border-radius:7px;padding:7px 12px;font-size:12px;cursor:pointer}
button:hover{border-color:var(--acc);color:var(--acc)}
button:focus-visible{outline:2px solid var(--acc);outline-offset:2px}
button:disabled{opacity:.5;cursor:wait}
button.danger{border-color:#5c2023;color:var(--bad)}
button.primary{border-color:#1f4a7a;color:var(--acc);font-size:14px;padding:9px 16px}
.btns{display:flex;gap:8px;flex-wrap:wrap;margin-top:12px}
a{color:var(--acc)}
/* --- accident banner --- */
#banner{display:none;margin-bottom:12px;padding:14px 16px;border-radius:10px;
        font-weight:600;background:#3d1214;border:1px solid var(--bad);color:#ffd7d5}
#banner.pend{animation:blink 1s steps(2) infinite}
@keyframes blink{50%{background:#6b1a1d}}
@media (prefers-reduced-motion:reduce){#banner.pend{animation:none}}
/* --- rider check --- */
#rimg{display:none;width:100%;border-radius:8px;border:1px solid var(--line);
      background:#000;margin:4px 0 10px}
#rnotes{text-align:right}
/* --- live map --- */
#map{height:380px;border-radius:8px;border:1px solid var(--line);background:#0e1319}
#mapmsg{height:380px;display:flex;align-items:center;justify-content:center;
        text-align:center;padding:20px;color:var(--dim);font-size:13px;
        border:1px dashed var(--line);border-radius:8px}
.leaflet-container{background:#0e1319}
.leaflet-tile{filter:brightness(.72) contrast(1.05) saturate(.85)}
.leaflet-control-attribution{background:rgba(14,17,22,.8)!important;color:#8b96a5!important}
.leaflet-control-attribution a{color:#58a6ff!important}
.hmark{width:0;height:0}
.hmark i{position:absolute;left:-9px;top:-9px;width:18px;height:18px;
         border-radius:50%;background:var(--acc);border:2px solid #fff;
         box-shadow:0 0 0 4px rgba(88,166,255,.25)}
.hmark b{position:absolute;left:-7px;top:-24px;width:14px;height:14px;
         border-left:7px solid transparent;border-right:7px solid transparent;
         border-bottom:14px solid var(--acc);display:none;transform-origin:7px 24px}
.hmark.moving b{display:block}
.mapbar{display:flex;gap:8px;flex-wrap:wrap;align-items:center;margin-bottom:10px}
.mapbar label{font-size:12px;color:var(--dim);display:flex;align-items:center;gap:5px}
</style></head><body>

<header>
  <div><h1>Project Helmet</h1>
  <div class="sub" id="sub">connecting…</div></div>
  <div><span class="dot" id="dot"></span><span class="sub" id="live">offline</span></div>
</header>

<div id="banner" role="alert"></div>

<div class="grid">

  <div class="card"><h2>Accident alert</h2>
    <div class="row"><span>State</span><span id="sstate" class="pill">--</span></div>
    <div class="row"><span>Armed by tag</span><span class="mono" id="suid">--</span></div>
    <div class="row"><span>Tilt from worn position</span><span class="mono" id="stilt">--</span></div>
    <div class="row"><span>Last trigger</span><span class="mono" id="strig">--</span></div>
    <div class="row"><span>Accident alerts sent</span><span class="mono" id="scount">--</span></div>
    <div class="row"><span>Telegram</span><span class="mono" id="stg">--</span></div>
    <div class="note" id="shint">Scan an authorised RFID tag to turn on accident detection.</div>
    <div class="btns">
      <button class="danger" onclick="api('cancel')">I'm OK, cancel alert</button>
      <button onclick="api('clear-alert')">Clear alert</button>
      <button onclick="api('disarm')">Turn off detection</button>
      <button onclick="api('test-telegram')">Send test message</button>
    </div>
  </div>

  <div class="card"><h2>Rider check</h2>
    <button class="primary" id="rbtn" onclick="checkRider()">Check rider</button>
    <div class="sub" id="rstat" style="margin:8px 0">Takes a photo with the helmet camera and asks the vision model whether the rider is wearing a helmet and looks drowsy.</div>
    <img id="rimg" alt="Latest rider photo from the helmet camera">
    <div class="row"><span>Helmet</span><span id="rhelm" class="pill">--</span></div>
    <div class="row"><span>Drowsiness</span><span id="rdrow" class="pill">--</span></div>
    <div class="row"><span>Eyes</span><span class="mono" id="reyes">--</span></div>
    <div class="row"><span>Confidence</span><span class="mono" id="rconf">--</span></div>
    <div class="row"><span>Notes</span><span id="rnotes">--</span></div>
  </div>

  <div class="card" style="grid-column:1/-1">
    <h2>Attitude</h2>
    <div class="trip">
      <div><small>Roll</small><div class="big mono" id="roll">--</div></div>
      <div><small>Pitch</small><div class="big mono" id="pitch">--</div></div>
      <div><small>Yaw</small><div class="big mono" id="yaw">--</div></div>
    </div>
    <div class="viz">
      <div class="horizon"><div class="sky" id="sky"></div><div class="cross"></div></div>
      <div class="compass"><b>N</b><div class="needle" id="needle"></div></div>
    </div>
    <div class="row"><span>GPS course (true heading)</span><span class="mono" id="course">--</span></div>
    <div class="row"><span>Peak impact</span><span class="mono" id="peak">--</span></div>
    <div class="note">Roll and pitch are absolute (referenced to gravity).
      Yaw is integrated gyro only, so it drifts a few degrees per minute.
      GPS course is the only true heading, and only while moving.</div>
    <div class="btns">
      <button onclick="api('zero-yaw')">Zero yaw</button>
      <button onclick="api('calibrate')">Calibrate gyro (hold still)</button>
      <button onclick="api('reset-peak')">Reset peak</button>
      <button onclick="api('beep')">Beep</button>
    </div>
  </div>

  <div class="card"><h2>Motion raw</h2>
    <div class="row"><span>accel X</span><span class="mono" id="ax">--</span></div>
    <div class="row"><span>accel Y</span><span class="mono" id="ay">--</span></div>
    <div class="row"><span>accel Z</span><span class="mono" id="az">--</span></div>
    <div class="row"><span>|a|</span><span class="mono" id="mag">--</span></div>
    <div class="row"><span>gyro X/Y/Z</span><span class="mono" id="gyr">--</span></div>
    <div class="row"><span>die temp</span><span class="mono" id="temp">--</span></div>
  </div>

  <div class="card"><h2>Environment</h2>
    <div class="row"><span>Gas</span><span class="mono" id="gas">--</span></div>
    <div class="row"><span>Gas status</span><span id="gasp" class="pill">--</span></div>
    <div class="row"><span>Light</span><span id="ldr" class="pill">--</span></div>
    <div class="row"><span>Touch</span><span id="touch" class="pill">--</span></div>
  </div>

  <div class="card" style="grid-column:1/-1">
    <h2>Live map</h2>
    <div class="mapbar">
      <button onclick="recenter()">Recenter</button>
      <button onclick="clearTrail()">Clear trail</button>
      <label><input type="checkbox" id="follow" checked> Follow helmet</label>
      <span class="sub" id="mapinfo" style="margin-left:auto">waiting for fix…</span>
    </div>
    <div id="map"></div>
    <div id="mapmsg" style="display:none"></div>
  </div>

  <div class="card"><h2>Position</h2>
    <div class="row"><span>Fix</span><span id="fix" class="pill">--</span></div>
    <div class="row"><span>Latitude</span><span class="mono" id="lat">--</span></div>
    <div class="row"><span>Longitude</span><span class="mono" id="lng">--</span></div>
    <div class="row"><span>Satellites</span><span class="mono" id="sats">--</span></div>
    <div class="row"><span>Speed</span><span class="mono" id="kph">--</span></div>
    <div class="row"><span>Altitude</span><span class="mono" id="alt">--</span></div>
    <div class="row"><span>UTC</span><span class="mono" id="utc">--</span></div>
    <div class="row"><span>Map</span><span id="maplink">--</span></div>
  </div>

  <div class="card"><h2>Rider tag</h2>
    <div class="row"><span>Last UID</span><span class="mono" id="uid">--</span></div>
    <div class="row"><span>Scans</span><span class="mono" id="cards">--</span></div>
    <div class="row"><span>Last seen</span><span class="mono" id="ago">--</span></div>
  </div>

  <div class="card"><h2>System</h2>
    <div class="row"><span>IP</span><span class="mono" id="ip">--</span></div>
    <div class="row"><span>Wi-Fi RSSI</span><span class="mono" id="rssi">--</span></div>
    <div class="row"><span>Uptime</span><span class="mono" id="up">--</span></div>
    <div class="row"><span>Free heap</span><span class="mono" id="heap">--</span></div>
    <div class="row"><span>IMU mode</span><span class="mono" id="imumode">--</span></div>
    <div class="row"><span>Camera</span><span class="mono" id="camurl">--</span></div>
  </div>

</div>

<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<script>
const $=id=>document.getElementById(id);
let fails=0;

/* ---------------- Live map ----------------
   Leaflet and the map tiles are fetched by YOUR BROWSER, not by the ESP32. */
let map=null, marker=null, trail=null, accCircle=null;
let trailPts=[], lastPt=null, lastRender=null, totalM=0;
const MAX_TRAIL=600;

function mapUnavailable(msg){
  $('map').style.display='none';
  const m=$('mapmsg'); m.style.display='flex'; m.innerHTML=msg;
}

function initMap(lat,lng){
  if(typeof L==='undefined'){
    mapUnavailable('Map library could not load.<br>'+
      'Leaflet and the tiles come from the internet via your browser, '+
      'so this device needs internet access. Coordinates below still work.');
    return false;
  }
  map=L.map('map',{zoomControl:true,attributionControl:true}).setView([lat,lng],17);
  L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png',{
    maxZoom:19, attribution:'&copy; OpenStreetMap contributors'
  }).addTo(map);
  trail=L.polyline([],{color:'#58a6ff',weight:3,opacity:.75}).addTo(map);
  accCircle=L.circle([lat,lng],{radius:10,color:'#58a6ff',weight:1,
                                fillOpacity:.08}).addTo(map);
  marker=L.marker([lat,lng],{icon:L.divIcon({className:'hmark',
          html:'<b></b><i></i>',iconSize:[0,0]})}).addTo(map);
  map.on('dragstart',()=>{ $('follow').checked=false; });
  return true;
}

function metres(a,b){
  const R=6371000, t=Math.PI/180;
  const dLa=(b[0]-a[0])*t, dLo=(b[1]-a[1])*t;
  const h=Math.sin(dLa/2)**2 +
          Math.cos(a[0]*t)*Math.cos(b[0]*t)*Math.sin(dLo/2)**2;
  return 2*R*Math.asin(Math.sqrt(h));
}

function updateMap(g){
  if(!g.fix) return;
  const pt=[g.lat,g.lng];
  if(!map && !initMap(g.lat,g.lng)) return;
  if(lastRender && lastRender[0]===pt[0] && lastRender[1]===pt[1]) return;
  lastRender=pt;

  marker.setLatLng(pt);
  accCircle.setLatLng(pt).setRadius(Math.max(3,(g.hdop||1)*5));

  const el=marker.getElement();
  if(el){
    const moving=g.kph>2;
    el.classList.toggle('moving',moving);
    const b=el.querySelector('b');
    if(b) b.style.transform=moving?'rotate('+g.course+'deg)':'';
  }

  if(!lastPt || metres(lastPt,pt)>2.5){
    if(lastPt) totalM+=metres(lastPt,pt);
    trailPts.push(pt);
    if(trailPts.length>MAX_TRAIL) trailPts.shift();
    trail.setLatLngs(trailPts);
    lastPt=pt;
  }
  if($('follow').checked) map.panTo(pt,{animate:true,duration:.4});

  $('mapinfo').textContent=g.sats+' sats, HDOP '+(g.hdop||0).toFixed(1)+', trail '+
    (totalM<1000?Math.round(totalM)+' m':(totalM/1000).toFixed(2)+' km');
}

function recenter(){
  if(map && lastPt){ $('follow').checked=true; map.setView(lastPt,17); }
}
function clearTrail(){
  trailPts=[]; totalM=0; lastPt=null; lastRender=null;
  if(trail) trail.setLatLngs([]);
}

function pill(el,text,cls){el.textContent=text;el.className='pill '+(cls||'');}

function api(p){
  return fetch('/api/'+p).then(r=>r.text()).then(t=>{ $('sub').textContent=t; });
}

function fmt(v,d){return (v===null||v===undefined)?'--':Number(v).toFixed(d);}

/* ---------------- Accident alert ---------------- */
const SN={LOCKED:['off, scan tag','warn'],ARMED:['watching','ok'],
          PENDING:['possible accident','bad'],ALERTED:['alert sent','bad']};
const HINT={
  LOCKED:'Scan an authorised RFID tag to turn on accident detection.',
  ARMED:'Watching for hard impacts and for the helmet lying tipped over. Scan the tag again to turn detection off.',
  PENDING:'Countdown running. Cancel if the rider is fine.',
  ALERTED:'Alert sent to Telegram. Clear it once the rider is safe.'};

function updateSafety(s){
  const st=SN[s.state]||[s.state,''];
  pill($('sstate'),st[0],st[1]);
  $('suid').textContent=s.uid||'--';
  $('stilt').textContent=s.state==='LOCKED'?'--':fmt(s.tilt,0)+'°';
  $('strig').textContent=s.why?s.why+', '+fmt(s.trigG,1)+' g':'none yet';
  $('scount').textContent=s.accidents;
  $('stg').textContent=s.tgSent+' sent, '+s.tgFailed+' failed'+
    (s.tgQueued?', '+s.tgQueued+' queued':'')+
    (s.tgErr&&s.tgErr!=='none'?' ('+s.tgErr+')':'');
  $('shint').textContent=HINT[s.state]||'';

  const b=$('banner');
  if(s.state==='PENDING'){
    b.style.display='block'; b.className='pend';
    b.textContent='Possible accident ('+s.why+', '+fmt(s.trigG,1)+' g). '+
      'Telegram alert goes out in '+s.countdown+' s. Touch the helmet sensor, '+
      'scan the tag, or press "I\'m OK, cancel alert" to stop it.';
  }else if(s.state==='ALERTED'){
    b.style.display='block'; b.className='';
    b.textContent='Accident alert sent to Telegram with the last GPS position. '+
      'Scan the rider tag or press "Clear alert" once the rider is safe.';
  }else{
    b.style.display='none';
  }
}

/* ---------------- Rider check ---------------- */
let riderSeq=-1;
function checkRider(){
  $('rbtn').disabled=true;
  $('rstat').textContent='Starting check…';
  api('check-rider');
}

function updateRider(r){
  $('rbtn').disabled=r.busy;
  if(r.busy)        $('rstat').textContent='Checking: taking a photo and asking the vision model. This can take 5–20 s.';
  else if(!r.seq)   {}
  else if(r.valid)  $('rstat').textContent='Checked '+Math.round(r.agoMs/1000)+' s ago, took '+(r.tookMs/1000).toFixed(1)+' s.';
  else              $('rstat').textContent='Check failed: '+r.err+'. Make sure the camera is powered and CAM_BASE_URL is its IP.';

  if(r.seq!==riderSeq){
    riderSeq=r.seq;
    if(r.seq && r.valid){
      const im=$('rimg');
      im.onload=()=>{ im.style.display='block'; };
      im.onerror=()=>{ im.style.display='none';
        $('rstat').textContent+=' Photo not reachable from this browser ('+r.cam+').'; };
      im.src=r.cam+'/last.jpg?t='+Date.now();
    }
  }
  if(r.valid){
    if(!r.face){
      pill($('rhelm'),'no rider in view','warn');
      pill($('rdrow'),'no rider in view','warn');
    }else{
      pill($('rhelm'),r.helmet?'wearing helmet':'no helmet',r.helmet?'ok':'bad');
      pill($('rdrow'),r.drowsy?'drowsy':'alert',r.drowsy?'bad':'ok');
    }
    $('reyes').textContent=r.eyes;
    $('rconf').textContent=Math.round(r.conf*100)+'%';
    $('rnotes').textContent=r.notes||'--';
  }
}

async function tick(){
  try{
    const res=await fetch('/data',{cache:'no-store'});
    const d=await res.json();
    fails=0; $('dot').classList.add('on'); $('live').textContent='live';

    updateSafety(d.safety);
    updateRider(d.rider);

    $('roll').textContent  = fmt(d.att.roll,1)+'°';
    $('pitch').textContent = fmt(d.att.pitch,1)+'°';
    $('yaw').textContent   = fmt(d.att.yaw,1)+'°';
    $('sky').style.transform =
      'rotate('+(-d.att.roll)+'deg) translateY('+(d.att.pitch*1.6)+'px)';
    $('needle').style.transform = 'rotate('+d.att.yaw+'deg)';
    $('peak').textContent = fmt(d.imu.peak,2)+' g';

    $('ax').textContent=fmt(d.imu.ax,2)+' m/s²';
    $('ay').textContent=fmt(d.imu.ay,2)+' m/s²';
    $('az').textContent=fmt(d.imu.az,2)+' m/s²';
    $('mag').textContent=fmt(d.imu.mag,2)+' g';
    $('gyr').textContent=fmt(d.imu.gx,1)+' / '+fmt(d.imu.gy,1)+' / '+fmt(d.imu.gz,1)+' °/s';
    $('temp').textContent=fmt(d.imu.temp,1)+' °C';
    $('imumode').textContent=d.imu.mode;

    $('gas').textContent=d.env.gasRaw+' raw / '+d.env.gasMv+' mV';
    pill($('gasp'),d.env.gasAlarm?'ALARM':'normal',d.env.gasAlarm?'bad':'ok');
    pill($('ldr'),d.env.dark?'dark':'light',d.env.dark?'warn':'ok');
    pill($('touch'),d.env.touch?'touched':'idle',d.env.touch?'ok':'');

    pill($('fix'),d.gps.fix?'locked':'searching',d.gps.fix?'ok':'warn');
    $('lat').textContent=d.gps.fix?fmt(d.gps.lat,6):'--';
    $('lng').textContent=d.gps.fix?fmt(d.gps.lng,6):'--';
    $('sats').textContent=d.gps.sats;
    $('kph').textContent=fmt(d.gps.kph,1)+' km/h';
    $('alt').textContent=fmt(d.gps.altM,1)+' m';
    $('utc').textContent=d.gps.utc;
    $('course').textContent=(d.gps.fix&&d.gps.kph>2)
        ? fmt(d.gps.course,1)+'°' : 'needs motion';
    updateMap(d.gps);
    $('maplink').innerHTML = d.gps.fix
      ? '<a target="_blank" rel="noopener" href="https://www.openstreetmap.org/?mlat='
        +d.gps.lat+'&mlon='+d.gps.lng+'#map=18/'+d.gps.lat+'/'+d.gps.lng+'">open</a>'
      : '--';

    $('uid').textContent=d.rfid.uid;
    $('cards').textContent=d.rfid.count;
    $('ago').textContent=d.rfid.count?Math.round(d.rfid.agoMs/1000)+' s ago':'never';

    $('ip').textContent=d.sys.ip;
    $('rssi').textContent=d.sys.rssi+' dBm';
    $('heap').textContent=Math.round(d.sys.heap/1024)+' KB';
    $('camurl').textContent=d.rider.cam;
    const s=Math.floor(d.sys.upMs/1000);
    $('up').textContent=Math.floor(s/3600)+'h '+Math.floor(s%3600/60)+'m '+(s%60)+'s';
  }catch(e){
    if(++fails>2){$('dot').classList.remove('on');$('live').textContent='offline';}
  }
}
setInterval(tick,250); tick();
</script></body></html>)HTML";

// ==========================================================================
//  HTTP handlers
// ==========================================================================
void handleRoot() { server.send_P(200, "text/html", INDEX_HTML); }

void handleData() {
  static char b[2800];
  Motion m = readMotion();
  float mag = m.valid
      ? sqrt(m.ax * m.ax + m.ay * m.ay + m.az * m.az) / 9.80665f : 0;

  char utc[24];
  utcString(utc, sizeof(utc));

  const char* mode = mpuMode == MPU_ADAFRUIT ? "library"
                   : mpuMode == MPU_RAW      ? "direct" : "none";

  RiderResult r;
  portENTER_CRITICAL(&riderMux);
  r = rider;
  portEXIT_CRITICAL(&riderMux);
  char notesE[340], errE[180], eyesE[40];
  jsonEsc(r.notes, notesE, sizeof(notesE));
  jsonEsc(r.err,   errE,   sizeof(errE));
  jsonEsc(r.eyes,  eyesE,  sizeof(eyesE));

  size_t n = 0;
  n += snprintf(b + n, sizeof(b) - n,
      "{\"att\":{\"roll\":%.2f,\"pitch\":%.2f,\"yaw\":%.2f},",
      roll, pitch, yawDisplay());
  n += snprintf(b + n, sizeof(b) - n,
      "\"imu\":{\"ok\":%s,\"mode\":\"%s\",\"whoami\":%u,"
      "\"ax\":%.2f,\"ay\":%.2f,\"az\":%.2f,"
      "\"gx\":%.2f,\"gy\":%.2f,\"gz\":%.2f,"
      "\"mag\":%.3f,\"peak\":%.2f,\"temp\":%.1f},",
      m.valid ? "true" : "false", mode, mpuWhoAmI,
      m.ax, m.ay, m.az,
      (m.gx - gBias[0]) * RAD_TO_DEG,
      (m.gy - gBias[1]) * RAD_TO_DEG,
      (m.gz - gBias[2]) * RAD_TO_DEG,
      mag, peakG, m.tempC);
  int gasRaw = analogRead(GAS_PIN);
  n += snprintf(b + n, sizeof(b) - n,
      "\"env\":{\"gasRaw\":%d,\"gasMv\":%lu,\"gasAlarm\":%s,"
      "\"dark\":%s,\"touch\":%s},",
      gasRaw, (unsigned long)analogReadMilliVolts(GAS_PIN),
      gasRaw > GAS_ALARM_RAW ? "true" : "false",
      ldrDark() ? "true" : "false",
      touched()  ? "true" : "false");
  n += snprintf(b + n, sizeof(b) - n,
      "\"safety\":{\"state\":\"%s\",\"uid\":\"%s\",\"tilt\":%.1f,"
      "\"countdown\":%lu,\"why\":\"%s\",\"trigG\":%.2f,\"accidents\":%lu,"
      "\"tgSent\":%lu,\"tgFailed\":%lu,\"tgQueued\":%u,\"tgErr\":\"%s\"},",
      safeName(safeState), armedUid.c_str(), tiltFromRef(),
      (unsigned long)countdownS(), triggerWhy, triggerG,
      (unsigned long)accidentCount,
      (unsigned long)tgSent, (unsigned long)tgFailed,
      tgQueue ? (unsigned)uxQueueMessagesWaiting(tgQueue) : 0u, tgLastErr);
  n += snprintf(b + n, sizeof(b) - n,
      "\"rider\":{\"busy\":%s,\"seq\":%lu,\"valid\":%s,\"face\":%s,"
      "\"helmet\":%s,\"drowsy\":%s,\"eyes\":\"%s\",\"conf\":%.2f,"
      "\"notes\":\"%s\",\"err\":\"%s\",\"agoMs\":%lu,\"tookMs\":%lu,"
      "\"cam\":\"%s\"},",
      riderBusy ? "true" : "false", (unsigned long)r.seq,
      r.valid ? "true" : "false", r.face ? "true" : "false",
      r.helmet ? "true" : "false", r.drowsy ? "true" : "false",
      eyesE, r.conf, notesE, errE,
      (unsigned long)(r.atMs ? millis() - r.atMs : 0),
      (unsigned long)r.tookMs, CAM_BASE_URL);
  n += snprintf(b + n, sizeof(b) - n,
      "\"gps\":{\"fix\":%s,\"lat\":%.6f,\"lng\":%.6f,\"sats\":%lu,"
      "\"hdop\":%.1f,\"altM\":%.1f,\"kph\":%.1f,\"course\":%.1f,"
      "\"chars\":%lu,\"utc\":\"%s\"},",
      gps.location.isValid() ? "true" : "false",
      gps.location.isValid() ? gps.location.lat() : 0.0,
      gps.location.isValid() ? gps.location.lng() : 0.0,
      (unsigned long)gps.satellites.value(),
      gps.hdop.isValid() ? gps.hdop.hdop() : 0.0,
      gps.altitude.isValid() ? gps.altitude.meters() : 0.0,
      gps.speed.isValid() ? gps.speed.kmph() : 0.0,
      gps.course.isValid() ? gps.course.deg() : 0.0,
      (unsigned long)gps.charsProcessed(), utc);
  n += snprintf(b + n, sizeof(b) - n,
      "\"rfid\":{\"ok\":%s,\"uid\":\"%s\",\"count\":%lu,\"agoMs\":%lu},",
      rfidOK ? "true" : "false", lastUid.c_str(),
      (unsigned long)cardCount,
      (unsigned long)(lastCardMs ? millis() - lastCardMs : 0));
  n += snprintf(b + n, sizeof(b) - n,
      "\"sys\":{\"ip\":\"%s\",\"rssi\":%d,\"upMs\":%lu,\"heap\":%lu,"
      "\"oled\":%s}}",
      WiFi.localIP().toString().c_str(), WiFi.RSSI(),
      (unsigned long)millis(), (unsigned long)ESP.getFreeHeap(),
      oledOK ? "true" : "false");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", b);
}

void setupRoutes() {
  server.on("/", handleRoot);
  server.on("/data", handleData);

  server.on("/api/zero-yaw", []() {
    yawOffset = yaw;
    beep(60);
    server.send(200, "text/plain", "Yaw zeroed");
  });
  server.on("/api/calibrate", []() {
    calibrateGyro();                 // blocks ~1.5 s; helmet must be still
    beep(150);
    server.send(200, "text/plain", "Gyro calibrated");
  });
  server.on("/api/reset-peak", []() {
    peakG = 0;
    server.send(200, "text/plain", "Peak cleared");
  });
  server.on("/api/beep", []() {
    beep(200);
    server.send(200, "text/plain", "Beeped");
  });

  server.on("/api/check-rider", []() {
    if (riderBusy) { server.send(200, "text/plain", "Rider check already running"); return; }
    requestRiderCheck();
    server.send(200, "text/plain", "Rider check started");
  });
  server.on("/api/cancel", []() {
    if (safeState == SAFE_PENDING) {
      cancelPending("dashboard");
      server.send(200, "text/plain", "Alert cancelled, still watching");
    } else {
      server.send(200, "text/plain", "No countdown running");
    }
  });
  server.on("/api/clear-alert", []() {
    if (safeState == SAFE_ALERTED) {
      clearAlert("from dashboard");
      server.send(200, "text/plain", "Alert cleared, still watching");
    } else {
      server.send(200, "text/plain", "No alert to clear");
    }
  });
  server.on("/api/disarm", []() {
    if (safeState == SAFE_LOCKED) { server.send(200, "text/plain", "Detection is already off"); return; }
    disarmSafety("dashboard");
    server.send(200, "text/plain", "Detection turned off");
  });
  server.on("/api/test-telegram", []() {
    char loc[200], msg[400];
    locationText(loc, sizeof(loc));
    snprintf(msg, sizeof(msg),
             "🧪 Test message from Project Helmet\nDashboard: http://%s/\nLocation:\n%s",
             WiFi.localIP().toString().c_str(), loc);
    bool q = tgQueueMessage(msg, false);
    server.send(200, "text/plain", q ? "Test message queued" : "Telegram queue full");
  });

  server.onNotFound([]() { server.send(404, "text/plain", "not found"); });
}

// ==========================================================================
//  OLED
// ==========================================================================
void initOLED() {
  oledAddr = 0;
  if      (i2cPresent(0x3C)) oledAddr = 0x3C;
  else if (i2cPresent(0x3D)) oledAddr = 0x3D;
  if (!oledAddr) { oledOK = false; return; }
  oledOK = oled.begin(SSD1306_SWITCHCAPVCC, oledAddr);
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.setTextSize(1);
  oled.display();
}

void oledMessage(const char* l1, const char* l2) {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setTextSize(1);
  oled.setCursor(0, 0);
  oled.println(l1);
  if (l2) oled.println(l2);
  oled.display();
}

void drawOLED() {
  if (!oledOK) return;
  oled.clearDisplay();
  oled.setCursor(0, 0);

  if (safeState == SAFE_PENDING) {
    oled.setTextSize(2);
    oled.println(F("ACCIDENT?"));
    oled.setTextSize(3);
    oled.printf(" %2lus\n", (unsigned long)countdownS());
    oled.setTextSize(1);
    oled.println(F("touch/tag = I'm OK"));
    oled.display();
    return;
  }
  if (safeState == SAFE_ALERTED) {
    oled.setTextSize(2);
    oled.println(F("ALERT"));
    oled.println(F("SENT"));
    oled.setTextSize(1);
    oled.printf("tg sent %lu fail %lu\n", (unsigned long)tgSent, (unsigned long)tgFailed);
    oled.println(F("scan tag to clear"));
    oled.display();
    return;
  }

  oled.setTextSize(1);
  if (WiFi.status() == WL_CONNECTED) oled.println(WiFi.localIP().toString());
  else                               oled.println(F("WiFi: offline"));
  oled.printf("R%6.1f P%6.1f\n", roll, pitch);
  oled.printf("Y%6.1f  %ddBm\n", yawDisplay(), WiFi.RSSI());
  oled.printf("gas %4d %s\n", analogRead(GAS_PIN), ldrDark() ? "DARK" : "LIT");
  if (gps.location.isValid()) {
    oled.printf("sat %lu  %.4f\n", (unsigned long)gps.satellites.value(),
                gps.location.lat());
  } else {
    oled.printf("gps searching %lu\n", (unsigned long)gps.satellites.value());
  }
  oled.printf("tag %s\n", lastUid.c_str());
  oled.printf("crash det: %s\n", safeState == SAFE_ARMED ? "ON" : "off");
  if (riderBusy) oled.print(F("rider check..."));
  oled.display();
}

// ==========================================================================
//  Wi-Fi
// ==========================================================================
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.setHostname(HOSTNAME);
  WiFi.setSleep(false);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  Serial.printf("Joining \"%s\"", WIFI_SSID);
  oledMessage("WiFi connecting", WIFI_SSID);

  uint32_t t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 20000) {
    delay(300);
    Serial.print('.');
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("Connected. IP %s  RSSI %d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    if (MDNS.begin(HOSTNAME)) {
      MDNS.addService("http", "tcp", 80);
      Serial.printf("Also at http://%s.local/\n", HOSTNAME);
    }
    oledMessage("WiFi OK", WiFi.localIP().toString().c_str());
  } else {
    Serial.println(F("Wi-Fi FAILED — check SSID/password. Retrying in loop."));
    Serial.println(F("Note: ESP32 is 2.4 GHz only; a 5 GHz-only SSID cannot work."));
    oledMessage("WiFi FAILED", "check credentials");
  }
}

// ==========================================================================
//  Setup / loop
// ==========================================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println(F("\n\n=== Project Helmet — dashboard + accident alert ==="));

  pinMode(BUZZER_PIN, OUTPUT);
  buzzer(false);
  pinMode(LDR_PIN,   INPUT);
  pinMode(TOUCH_PIN, INPUT);

  analogReadResolution(12);
  analogSetPinAttenuation(GAS_PIN, ADC_11db);   // GPIO32 = ADC1, OK with Wi-Fi

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  SPI.begin(RFID_SCK, RFID_MISO, RFID_MOSI, RFID_SS);
  Serial2.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  delay(150);

  initOLED();
  Serial.printf("OLED  %s (0x%02X)\n", oledOK ? "ok" : "FAIL", oledAddr);
  initMPU();
  Serial.printf("MPU   %s  WHO_AM_I 0x%02X  mode %s\n",
                mpuMode != MPU_NONE ? "ok" : "FAIL", mpuWhoAmI,
                mpuMode == MPU_ADAFRUIT ? "library" :
                mpuMode == MPU_RAW      ? "direct"  : "none");
  initRFID();
  Serial.printf("RC522 %s (0x%02X)\n", rfidOK ? "ok" : "FAIL", rfidVersion);

  oledMessage("Calibrating gyro", "hold still...");
  calibrateGyro();

  // Background workers on core 0 (the Wi-Fi core); loop() stays on core 1.
  tgQueue = xQueueCreate(4, sizeof(TgMsg));
  xTaskCreatePinnedToCore(telegramTask, "telegram", 12288, nullptr, 1, nullptr, 0);
  xTaskCreatePinnedToCore(riderTask,    "rider",     8192, nullptr, 1, &riderTaskH, 0);

  connectWiFi();
  setupRoutes();
  server.begin();
  Serial.println(F("HTTP server up on port 80"));
  Serial.println(F("Scan an RFID tag to turn on accident detection."));

  if (TG_BOOT_MSG && WiFi.status() == WL_CONNECTED) {
    char msg[200];
    snprintf(msg, sizeof(msg),
             "🪖 Project Helmet online\nDashboard: http://%s/\n"
             "Accident detection is off until the rider tag is scanned.",
             WiFi.localIP().toString().c_str());
    tgQueueMessage(msg, false);
  }
  beep(120);

  lastImu = lastOled = millis();
}

void loop() {
  feedGPS();                    // constantly, or NMEA bytes are dropped
  server.handleClient();
  serviceBeep();
  pollRFID();

  // Touch: cancels the accident countdown; otherwise a short beep.
  bool t = touched();
  if (t && !lastTouch) {
    if (safeState == SAFE_PENDING) cancelPending("touch sensor");
    else                           beep(60);
  }
  lastTouch = t;

  if (millis() - lastImu >= IMU_MS) {
    lastImu = millis();
    updateAttitude();
    updateSafety();
  }
  serviceAlarm();

  if (AUTO_CHECK_MS && safeState == SAFE_ARMED && !riderBusy &&
      millis() - lastAutoCheck >= AUTO_CHECK_MS) {
    lastAutoCheck = millis();
    requestRiderCheck();
  }

  if (millis() - lastOled >= OLED_MS) {
    lastOled = millis();
    drawOLED();
  }

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 10000) {
    lastWifiTry = millis();
    Serial.println(F("Wi-Fi lost — reconnecting"));
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASS);
  }
}
