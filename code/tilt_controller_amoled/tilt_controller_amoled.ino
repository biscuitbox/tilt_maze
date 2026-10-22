/*
 * ================================================================
 *  원격 기울기 미로 — 조종기 (Waveshare ESP32-S3-Touch-AMOLED-1.75)
 * ================================================================
 *  보드에 들어 있는 6축 센서(QMI8658)로 기울기를 재서 ESP-NOW로 미로에 보냅니다.
 *  원형 화면에는 미로판이 기울어질 방향을 '구슬'로 보여 주고, 타이머·배터리를 표시합니다.
 *  미로 쪽 코드(tilt_maze_receiver)는 그대로 씁니다 — 패킷 형식이 같습니다.
 *
 *  [버튼]  BOOT 버튼 하나로 조작 (PWR 버튼은 전원용: 길게 누르면 꺼짐/켜짐)
 *    짧게            : 타이머  시작 → 멈춤 → 0으로
 *    1초 이상 누르기  : 지금 자세를 수평(0°)으로 저장
 *    4초 이상 누르기  : 방향 맞추기 (처음 한 번, 또는 손잡이에 보드를 다시 끼웠을 때)
 *        ① 손잡이를 내 쪽으로 하고 수평으로 들기  ② 미로 쪽(앞)으로 숙이기
 *        ③ 화면의 화살표가 미로 쪽을 가리킬 때까지 짧게 눌러 돌리고, 길게 눌러 저장
 *    → 보드를 손잡이에 어떤 각도로 끼웠든, 센서 방향을 자동으로 알아냅니다.
 *
 *  [아두이노 IDE 설정]  보드: ESP32S3 Dev Module
 *     Flash Size: 16MB  /  PSRAM: OPI PSRAM  /  Partition: 16M Flash (3MB APP/9.9MB FATFS)
 *     USB CDC On Boot: Enabled  (시리얼 모니터를 USB로 보려면)
 *  [라이브러리]  웨이브쉐어 예제에 들어 있는 버전을 libraries 폴더에 복사
 *     GFX Library for Arduino 1.6.4 / SensorLib 0.3.1 / XPowersLib 0.2.6
 *     + U8g2 (라이브러리 매니저에서 설치 — 한글 글꼴용)
 *  ESP32 Arduino 코어 3.x (3.1.0 권장, 3.3.0에서 컴파일 확인)
 * ================================================================
 */
#define U8G2_USE_LARGE_FONTS            // 한글 글꼴(갈무리 픽셀체) 사용
#include <Arduino.h>
#include <U8g2lib.h>                    // GFX 라이브러리의 한글(u8g2) 글꼴 기능을 켜는 데 필요
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include "Arduino_GFX_Library.h"
#include "SensorQMI8658.hpp"
#define XPOWERS_CHIP_AXP2101
#include "XPowersLib.h"

// ---------------- 설정 (미로 쪽과 같게) ----------------
#define GROUP_ID      1     // 같은 교실에서 여러 세트를 쓸 때 세트마다 다르게 (1~255)
#define WIFI_CHANNEL  1

// 화면의 '미리보기 구슬'용 — 미로 쪽 코드의 같은 이름 값과 맞추면 화면과 실제 판이 일치
const float HAND_GAIN = 0.35f, MAX_TILT = 8.0f, DEADZONE = 1.5f;

const float FILTER_ALPHA = 0.97f;          // 상보 필터 (자이로 비중)
const uint32_t SEND_INTERVAL_MS = 20;      // 50 Hz 송신
const uint32_t DIM_AFTER_MS = 60000;       // 1분 동안 가만히 있으면 화면 어둡게
const uint8_t BRIGHT_ON = 190, BRIGHT_DIM = 25;

// ---------------- 보드 핀 (웨이브쉐어 회로도 기준) ----------------
#define LCD_SDIO0 4
#define LCD_SDIO1 5
#define LCD_SDIO2 6
#define LCD_SDIO3 7
#define LCD_SCLK  38
#define LCD_CS    12
#define LCD_RESET 39
#define LCD_W     466
#define LCD_H     466
#define I2C_SDA   15
#define I2C_SCL   14
#define PIN_BOOT  0

// ---------------- 패킷 (미로 쪽과 동일해야 함) ----------------
typedef struct __attribute__((packed)) {
  uint8_t  magic;        // 'M'
  uint8_t  group;
  uint16_t seq;
  int16_t  tiltRight;    // 오른쪽이 내려간 각도 (0.01°)
  int16_t  tiltFront;    // 앞쪽이 내려간 각도 (0.01°)
  uint8_t  flags;
} TiltPacket;
static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---------------- 장치 ----------------
Arduino_DataBus *bus = new Arduino_ESP32QSPI(LCD_CS, LCD_SCLK, LCD_SDIO0, LCD_SDIO1, LCD_SDIO2, LCD_SDIO3);
Arduino_CO5300 *panel = new Arduino_CO5300(bus, LCD_RESET, 0, LCD_W, LCD_H, 6, 0, 0, 0);
Arduino_Canvas *gfx = new Arduino_Canvas(LCD_W, LCD_H, panel);   // PSRAM 프레임버퍼 → 깜빡임 없음
SensorQMI8658 imu;
XPowersPMU pmu;
Preferences prefs;
bool pmuOk = false;

// ---------------- 방향 보정값 (NVS 저장) ----------------
float fx = 1, fy = 0;      // 센서 XY 평면에서 '앞' 방향 단위벡터
int   ns = 1;              // 화면이 위를 볼 때 센서 Z 부호
int   uiRot = 0;           // 화면 회전 (0~3, 90° 단위)
float zeroF = 0, zeroR = 0;
float gbx = 0, gby = 0, gbz = 0;   // 자이로 바이어스 (°/s)

// ---------------- 공유 상태 (센서 코어 → 화면 코어) ----------------
enum Mode : uint8_t { M_RUN, M_CAL_LEVEL, M_CAL_TILT, M_CAL_ROT };
volatile Mode mode = M_RUN;
volatile float shR = 0, shF = 0;          // 0점 보정된 손 기울기 (°)
volatile float holdSec = 0;               // 버튼 누르고 있는 시간
volatile float calProgress = 0;           // 보정 단계 진행률 0~1
volatile int   batPct = -1;
volatile bool  charging = false;
volatile uint8_t wantBright = BRIGHT_ON;
volatile uint32_t timerStartMs = 0, timerAccumMs = 0;
volatile uint8_t timerState = 0;          // 0 정지(0초) 1 진행 2 멈춤
volatile uint32_t sentCount = 0;

float front = 0, right = 0;               // 필터 결과 (보정 전, 앞/오른쪽이 내려가면 +)

// ================================================================
//  센서 → 조종기 좌표 (앞/오른쪽)
// ================================================================
void accToAngles(float ax, float ay, float az, float &fr, float &ri) {
  float n = sqrtf(ax * ax + ay * ay + az * az);
  if (n < 0.2f) return;
  ax /= n; ay /= n; az /= n;
  float rx = fy * ns, ry = -fx * ns;                    // 오른쪽 = 앞 × 위
  float up = az * ns;
  fr = atan2f(-(ax * fx + ay * fy), up) * RAD_TO_DEG;   // 앞이 내려가면 +
  ri = atan2f(-(ax * rx + ay * ry), up) * RAD_TO_DEG;   // 오른쪽이 내려가면 +
}

bool readImu(float &ax, float &ay, float &az, float &gx, float &gy, float &gz) {
  if (!imu.getDataReady()) return false;
  if (!imu.getAccelerometer(ax, ay, az)) return false;
  if (!imu.getGyroscope(gx, gy, gz)) return false;
  return true;
}

// 일정 시간 평균 (가속도 단위벡터, 자이로)
void averageImu(uint32_t ms, float &ax, float &ay, float &az, float *gb = nullptr) {
  double sx = 0, sy = 0, sz = 0, gxs = 0, gys = 0, gzs = 0; int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < ms) {
    float a1, a2, a3, g1, g2, g3;
    if (readImu(a1, a2, a3, g1, g2, g3)) {
      sx += a1; sy += a2; sz += a3; gxs += g1; gys += g2; gzs += g3; n++;
      calProgress = (millis() - t0) / (float)ms;
    }
    delay(2);
  }
  if (!n) return;
  ax = sx / n; ay = sy / n; az = sz / n;
  if (gb) { gb[0] = gxs / n; gb[1] = gys / n; gb[2] = gzs / n; }
}

void saveCal() {
  prefs.putFloat("fx", fx); prefs.putFloat("fy", fy); prefs.putInt("ns", ns);
  prefs.putInt("rot", uiRot); prefs.putFloat("zf", zeroF); prefs.putFloat("zr", zeroR);
  prefs.putBool("ok", true);
}

// 방향 맞추기 마법사 (센서 코어에서 실행, 그동안 미로에는 수평 신호를 보냄)
void sendPacket(float r, float f);
bool buttonDown() { return digitalRead(PIN_BOOT) == LOW; }

void waitRelease() { while (buttonDown()) { sendPacket(0, 0); delay(10); } holdSec = 0; }

void calibrationWizard() {
  waitRelease();
  // ① 수평
  mode = M_CAL_LEVEL;
  float ax0 = 0, ay0 = 0, az0 = 1, gb[3];
  uint32_t t0 = millis();
  while (millis() - t0 < 1500) { sendPacket(0, 0); delay(10); calProgress = (millis() - t0) / 3000.0f; }
  averageImu(1500, ax0, ay0, az0, gb);
  gbx = gb[0]; gby = gb[1]; gbz = gb[2];
  ns = (az0 >= 0) ? 1 : -1;
  // ② 앞으로 숙이기: 수평 대비 15° 넘게 기운 상태가 0.8초 유지되면 기록
  mode = M_CAL_TILT; calProgress = 0;
  float n0 = sqrtf(ax0 * ax0 + ay0 * ay0 + az0 * az0);
  uint32_t okSince = 0;
  float ax1 = 0, ay1 = 0, az1 = 0;
  while (true) {
    sendPacket(0, 0);
    float a1, a2, a3, g1, g2, g3;
    if (readImu(a1, a2, a3, g1, g2, g3)) {
      float n1 = sqrtf(a1 * a1 + a2 * a2 + a3 * a3);
      float dot = (a1 * ax0 + a2 * ay0 + a3 * az0) / (n1 * n0);
      float ang = acosf(constrain(dot, -1.0f, 1.0f)) * RAD_TO_DEG;
      if (ang > 15 && ang < 60) {
        if (!okSince) { okSince = millis(); ax1 = ay1 = az1 = 0; }
        ax1 = 0.9f * ax1 + 0.1f * a1 / n1; ay1 = 0.9f * ay1 + 0.1f * a2 / n1; az1 = 0.9f * az1 + 0.1f * a3 / n1;
        calProgress = min(1.0f, (millis() - okSince) / 800.0f);
        if (millis() - okSince > 800) break;
      } else { okSince = 0; calProgress = 0; }
    }
    if (buttonDown()) { waitRelease(); mode = M_RUN; return; }   // 버튼 누르면 취소
    delay(5);
  }
  // 앞 방향 = 숙였을 때 '위' 벡터가 움직인 반대쪽
  float dx = -(ax1 - ax0 / n0), dy = -(ay1 - ay0 / n0);
  float dn = sqrtf(dx * dx + dy * dy);
  if (dn > 0.05f) { fx = dx / dn; fy = dy / dn; }
  // 0점: 수평 자세 기준
  float fr0 = 0, ri0 = 0;
  accToAngles(ax0, ay0, az0, fr0, ri0);
  zeroF = fr0; zeroR = ri0; front = fr0; right = ri0;
  // ③ 화면 방향
  mode = M_CAL_ROT;
  while (buttonDown()) delay(10);
  while (true) {
    sendPacket(0, 0);
    if (buttonDown()) {
      uint32_t p = millis();
      while (buttonDown()) { holdSec = (millis() - p) / 1000.0f; sendPacket(0, 0); delay(10); }
      float h = (millis() - p) / 1000.0f; holdSec = 0;
      if (h < 0.6f) uiRot = (uiRot + 1) % 4;
      else if (h >= 1.0f) break;
    }
    delay(10);
  }
  saveCal();
  Serial.printf("방향 저장: 앞=(%.2f,%.2f) z부호=%d 화면회전=%d\n", fx, fy, ns, uiRot);
  mode = M_RUN;
}

// ================================================================
//  ESP-NOW
// ================================================================
void sendPacket(float r, float f) {
  static uint16_t seq = 0;
  static uint32_t last = 0;
  if (millis() - last < SEND_INTERVAL_MS) return;
  last = millis();
  TiltPacket p;
  p.magic = 'M'; p.group = GROUP_ID; p.seq = seq++;
  p.tiltRight = (int16_t)constrain(r * 100.0f, -9000.0f, 9000.0f);
  p.tiltFront = (int16_t)constrain(f * 100.0f, -9000.0f, 9000.0f);
  p.flags = buttonDown() ? 1 : 0;
  esp_now_send(BROADCAST, (uint8_t *)&p, sizeof(p));
  sentCount = sentCount + 1;
}

// ================================================================
//  화면 (코어 0에서 25fps)
// ================================================================
const uint16_t C_BG = RGB565(0, 0, 0), C_GRID = RGB565(45, 50, 58), C_GRID2 = RGB565(28, 31, 36);
const uint16_t C_TXT = RGB565(235, 235, 230), C_DIM = RGB565(130, 135, 140);
const uint16_t C_ACC = RGB565(240, 140, 60), C_OK = RGB565(80, 200, 140), C_WARN = RGB565(240, 90, 70);

float shapeTilt(float hand) {
  if (fabsf(hand) < DEADZONE) return 0;
  return constrain((hand - copysignf(DEADZONE, hand)) * HAND_GAIN, -MAX_TILT, MAX_TILT);
}

void textC(const char *s, int y, uint8_t size, uint16_t col) {   // 가운데 정렬
  int16_t x1, y1; uint16_t w, h;
  gfx->setTextSize(size);
  gfx->getTextBounds(s, 0, 0, &x1, &y1, &w, &h);
  gfx->setTextColor(col);
  gfx->setCursor((LCD_W - (int)w) / 2 - x1, y);
  gfx->print(s);
}

void drawArrowUp(int cx, int tipY, int w, int h, uint16_t col) {
  gfx->fillTriangle(cx, tipY, cx - w / 2, tipY + h, cx + w / 2, tipY + h, col);
}

void drawHoldRing() {
  float h = holdSec;
  if (h <= 0.05f) return;
  // 1초까지: 수평 저장 구간(주황), 4초까지: 방향 맞추기 구간(초록)
  float a1 = min(h, 1.0f) * 360.0f;
  gfx->fillArc(233, 233, 230, 222, 270, 270 + a1, C_ACC);
  if (h > 1.0f) {
    float a2 = min((h - 1.0f) / 3.0f, 1.0f) * 360.0f;
    gfx->fillArc(233, 233, 220, 213, 270, 270 + a2, C_OK);
  }
}

void drawRun() {
  const int cx = 233, cy = 233, R = 180;
  gfx->drawCircle(cx, cy, R, C_GRID);
  gfx->drawCircle(cx, cy, R / 2, C_GRID2);
  gfx->drawFastHLine(cx - R, cy, 2 * R, C_GRID2);
  gfx->drawFastVLine(cx, cy - R, 2 * R, C_GRID2);
  gfx->drawCircle(cx, cy, 10, C_GRID);
  drawArrowUp(cx, 18, 26, 18, C_ACC);                       // 미로 쪽 표시

  float bx = shapeTilt(shR) / MAX_TILT, by = shapeTilt(shF) / MAX_TILT;
  float len = sqrtf(bx * bx + by * by);
  if (len > 1) { bx /= len; by /= len; }
  int px = cx + (int)(bx * (R - 26)), py = cy - (int)(by * (R - 26));   // 앞(미로 쪽)이 화면 위

  // 타이머
  uint32_t ms = timerAccumMs + (timerState == 1 ? millis() - timerStartMs : 0);
  char buf[32];
  snprintf(buf, sizeof(buf), "%02lu:%02lu.%lu", (unsigned long)(ms / 60000), (unsigned long)(ms / 1000 % 60), (unsigned long)(ms / 100 % 10));
  textC(buf, 300, 5, timerState == 1 ? C_TXT : (timerState == 2 ? C_ACC : C_DIM));
  snprintf(buf, sizeof(buf), "X %+.1f  Y %+.1f", shapeTilt(shR), shapeTilt(shF));
  textC(buf, 352, 2, C_DIM);

  // 구슬
  gfx->fillCircle(px, py, 24, RGB565(170, 175, 182));
  gfx->fillCircle(px - 7, py - 8, 9, RGB565(235, 238, 242));
  if (len >= 0.999f) gfx->drawCircle(px, py, 27, C_WARN);   // 최대 기울기

  // 배터리
  if (batPct >= 0) {
    snprintf(buf, sizeof(buf), "%s%d%%", charging ? "+" : "", batPct);
    textC(buf, 436, 2, batPct <= 15 && !charging ? C_WARN : C_DIM);
  }
}

void drawCal() {
  float p = calProgress;
  if (mode == M_CAL_LEVEL) {
    textC("방향 맞추기 1/3", 120, 3, C_ACC);
    textC("손잡이를 내 쪽으로", 200, 3, C_TXT);
    textC("수평으로 들고", 240, 3, C_TXT);
    textC("기다리세요", 280, 3, C_TXT);
  } else if (mode == M_CAL_TILT) {
    textC("방향 맞추기 2/3", 120, 3, C_ACC);
    textC("미로 쪽(앞)으로", 200, 3, C_TXT);
    textC("천천히 숙이세요", 240, 3, C_TXT);
    textC("버튼: 취소", 330, 2, C_DIM);
  } else {
    textC("방향 맞추기 3/3", 120, 3, C_ACC);
    drawArrowUp(233, 160, 70, 56, C_OK);
    textC("화살표가 미로 쪽을", 250, 3, C_TXT);
    textC("가리키게 맞추세요", 290, 3, C_TXT);
    textC("짧게: 돌리기  길게: 저장", 350, 2, C_DIM);
    p = 0;
  }
  if (p > 0) gfx->fillArc(233, 233, 230, 220, 270, 270 + p * 360, C_OK);
}

void uiTask(void *) {
  uint8_t curBright = BRIGHT_ON;
  int curRot = -1;
  for (;;) {
    uint32_t t0 = millis();
    if (curRot != uiRot) { curRot = uiRot; gfx->setRotation(curRot); }
    gfx->fillScreen(C_BG);
    if (mode == M_RUN) drawRun(); else drawCal();
    drawHoldRing();
    gfx->flush();
    if (wantBright != curBright) { curBright = wantBright; panel->setBrightness(curBright); }
    int32_t wait = 40 - (int32_t)(millis() - t0);
    vTaskDelay(pdMS_TO_TICKS(wait > 1 ? wait : 1));
  }
}

// ================================================================
//  설정
// ================================================================
void setup() {
  Serial.begin(115200);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  if (!gfx->begin()) Serial.println("화면 초기화 실패 (PSRAM: OPI PSRAM 설정 확인)");
  panel->setBrightness(BRIGHT_ON);
  gfx->setFont(u8g2_font_quan7_h_cjk);
  gfx->setUTF8Print(true);
  gfx->fillScreen(C_BG);
  gfx->flush();

  if (!imu.begin(Wire, QMI8658_L_SLAVE_ADDRESS, I2C_SDA, I2C_SCL)) {
    Serial.println("QMI8658 센서를 찾지 못했습니다");
    gfx->setTextSize(3); gfx->setTextColor(C_WARN); gfx->setCursor(90, 230); gfx->print("센서 오류"); gfx->flush();
    while (true) delay(1000);
  }
  imu.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
  imu.configGyroscope(SensorQMI8658::GYR_RANGE_256DPS, SensorQMI8658::GYR_ODR_896_8Hz, SensorQMI8658::LPF_MODE_3);
  imu.enableAccelerometer();
  imu.enableGyroscope();

  pmuOk = pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA, I2C_SCL);
  if (pmuOk) { pmu.enableBattDetection(); pmu.enableBattVoltageMeasure(); }

  prefs.begin("tiltctl2", false);
  bool calOk = prefs.getBool("ok", false);
  fx = prefs.getFloat("fx", 1); fy = prefs.getFloat("fy", 0); ns = prefs.getInt("ns", 1);
  uiRot = prefs.getInt("rot", 0); zeroF = prefs.getFloat("zf", 0); zeroR = prefs.getFloat("zr", 0);

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW 초기화 실패"); ESP.restart(); }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = WIFI_CHANNEL; peer.encrypt = false; peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  xTaskCreatePinnedToCore(uiTask, "ui", 8192, nullptr, 1, nullptr, 0);

  if (!calOk) {
    calibrationWizard();                         // 처음 켰을 때는 방향 맞추기부터
  } else {
    mode = M_CAL_LEVEL;                          // 자이로 바이어스만 다시 잼 (1.5초, 가만히)
    float a1, a2, a3, gb[3];
    averageImu(1500, a1, a2, a3, gb);
    gbx = gb[0]; gby = gb[1]; gbz = gb[2];
    accToAngles(a1, a2, a3, front, right);
    mode = M_RUN;
  }
  Serial.printf("조종기 준비 완료 (그룹 %d, 채널 %d)\n", GROUP_ID, WIFI_CHANNEL);
}

// ================================================================
//  반복 (코어 1): 센서 → 필터 → 송신, 버튼, 배터리
// ================================================================
void loop() {
  static uint32_t lastUs = micros(), pressMs = 0, lastBat = 0, lastActive = 0, lastPrint = 0;
  static bool wasDown = false;
  static float lastR = 0, lastF = 0;
  static bool webMode = false;               // 웹 시뮬레이터가 'W'를 보내면 켜짐
  static uint32_t lastWeb = 0;
  while (Serial.available()) if (Serial.read() == 'W') webMode = true;

  float ax, ay, az, gx, gy, gz;
  if (readImu(ax, ay, az, gx, gy, gz)) {
    uint32_t now = micros();
    float dt = constrain((now - lastUs) * 1e-6f, 0.0005f, 0.05f);
    lastUs = now;
    gx -= gbx; gy -= gby;
    float rx = fy * ns, ry = -fx * ns;
    float frontRate = -(gx * rx + gy * ry);     // 앞이 내려가는 각속도
    float rightRate = gx * fx + gy * fy;        // 오른쪽이 내려가는 각속도
    float fa = front, ra = right;
    accToAngles(ax, ay, az, fa, ra);
    front = FILTER_ALPHA * (front + frontRate * dt) + (1 - FILTER_ALPHA) * fa;
    right = FILTER_ALPHA * (right + rightRate * dt) + (1 - FILTER_ALPHA) * ra;
    shF = front - zeroF;
    shR = right - zeroR;
  }

  // 버튼
  bool down = buttonDown();
  if (down && !wasDown) pressMs = millis();
  if (down) {
    holdSec = (millis() - pressMs) / 1000.0f;
    if (holdSec >= 4.0f) { holdSec = 0; calibrationWizard(); wasDown = false; return; }
  }
  if (!down && wasDown) {
    float h = (millis() - pressMs) / 1000.0f;
    holdSec = 0;
    if (h < 0.6f) {                                // 타이머
      if (timerState == 0) { timerStartMs = millis(); timerAccumMs = 0; timerState = 1; }
      else if (timerState == 1) { timerAccumMs += millis() - timerStartMs; timerState = 2; }
      else { timerAccumMs = 0; timerState = 0; }
    } else if (h >= 1.0f) {                        // 수평 저장
      zeroF = front; zeroR = right;
      prefs.putFloat("zf", zeroF); prefs.putFloat("zr", zeroR);
      Serial.printf("수평 저장: 앞 %.1f° 오른쪽 %.1f°\n", zeroF, zeroR);
    }
    lastActive = millis();
  }
  wasDown = down;

  sendPacket(shR, shF);

  // 화면 밝기: 1분 동안 움직임이 없으면 어둡게
  if (fabsf(shR - lastR) > 2 || fabsf(shF - lastF) > 2 || down) { lastActive = millis(); lastR = shR; lastF = shF; }
  wantBright = (millis() - lastActive > DIM_AFTER_MS && timerState != 1) ? BRIGHT_DIM : BRIGHT_ON;

  // 배터리 (5초마다, I2C는 이 코어에서만 사용)
  if (pmuOk && millis() - lastBat > 5000) {
    lastBat = millis();
    batPct = pmu.isBatteryConnect() ? pmu.getBatteryPercent() : -1;
    charging = pmu.isCharging();
  }

  if (webMode) {
    if (millis() - lastWeb >= 20) { lastWeb = millis(); Serial.printf("TILT,%.2f,%.2f\n", shR, shF); }
  } else if (millis() - lastPrint > 250) {
    lastPrint = millis();
    Serial.printf("오른쪽 %+6.1f°  앞쪽 %+6.1f°  (송신 %lu)\n", shR, shF, (unsigned long)sentCount);
  }
  delay(1);
}
