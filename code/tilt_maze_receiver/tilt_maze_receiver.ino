/*
 * ================================================================
 *  원격 기울기 미로 — 미로 본체 (ESP32 + 서보 2개)   v2 : 떨림 방지 / 부드러운 움직임
 * ================================================================
 *  조종기에서 받은 기울기를 서보 2개로 미로판에 재현합니다.
 *
 *  [떨림을 잡는 4단계]  조종기 값 → ①튀는 값 거르기 → ②적응형 필터(One Euro)
 *                       → ③PID 추종 제어 + 속도·가속도 제한 → ④정지 구간 출력 고정 → 서보
 *    ① 무선 잡음 등으로 한 번에 말이 안 되게 튄 값은 버립니다.
 *    ② 손이 느릴 때는 강하게 걸러 손떨림·센서잡음을 없애고, 빠를 때는 약하게 걸러 늦지 않게 합니다.
 *    ③ 미로판 각도를 '가상의 판'으로 계산합니다. 목표와의 차이(P), 판의 속도(D)로 가속도를 정하고
 *       최대 속도·가속도를 넘지 않게 해서, 조종기를 확 꺾어도 판은 매끄러운 S자 곡선으로 따라갑니다.
 *       I(적분)는 실제 판 각도를 재는 센서가 있을 때만 의미가 있어 기본값 0입니다.
 *    ④ 판이 거의 멈춘 상태에서 아주 작은 변화(기본 0.2°)는 서보에 보내지 않습니다.
 *       서보는 내부 불감대 근처의 미세한 명령에서 덜덜 떠는데(헌팅), 이걸 막습니다.
 *    + 서보 PWM 분해능 16비트(약 0.03°) — 계단처럼 끊기는 움직임 제거
 *
 *  [배선]
 *    서보 X (미로판 +X 쪽을 밀어 올림) 신호(주황/노랑) ── GPIO 18
 *    서보 Y (미로판 +Y 쪽을 밀어 올림) 신호(주황/노랑) ── GPIO 19
 *    서보 빨강(+) ── 5V 외부 전원(+)   (5V 3A 이상 어댑터 권장)
 *    서보 갈색(-) ── 5V 외부 전원(-) ── ESP32 GND   ★ GND 공통 필수
 *    ESP32 5V(VIN) ── 5V 외부 전원(+)  (또는 USB 따로)
 *    외부 전원 +/- 사이에 1000µF 전해 커패시터 (서보 바로 옆에 둘수록 좋음)
 *
 *  [처음 조립할 때]
 *    1) 서보 혼을 끼우기 '전에' 이 코드를 올리고 전원을 켭니다 → 서보가 중앙(90°)으로 갑니다.
 *    2) 그 상태에서 혼을 '수평(바깥쪽을 향하게)'으로 끼웁니다.
 *    3) 시리얼 모니터(115200)에서 미세조정:
 *         q / a : 서보 X 중앙 +0.5 / -0.5°   w / s : 서보 Y 중앙 +0.5 / -0.5°
 *         x     : 서보 X 회전방향 반전       y     : 서보 Y 회전방향 반전
 *         h     : 수평 고정 모드 켬/끔 (조종기 무시)
 *         t     : 동작 테스트 (각 축 기울이기)
 *         g     : 그래프 모드 켬/끔 — 시리얼 플로터에서 원래값/필터값/판 명령을 비교
 *         p     : 현재 설정 출력             (설정은 자동 저장)
 *    스마트폰 웹앱(tilt_phone.html)은 블루투스로 이 보드에 바로 연결됩니다 (USE_BLE 1, 이름 TiltMaze-그룹번호).
 *    웹 시뮬레이터(tilt_maze_sim.html)를 USB로 연결하면 실제 미로판 각도가 화면에 그대로 재현됩니다.
 *    (서보 없이 ESP32만 PC에 꽂아도 무선 조종기 → PC 중계기로 쓸 수 있음)
 *
 *  보드: ESP32 Dev Module   /  ESP32 Arduino 코어 2.x, 3.x 모두 지원
 *  추가 라이브러리 필요 없음 (서보 PWM 은 LEDC 로 직접 생성)
 * ================================================================
 */
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_mac.h>
#include <Preferences.h>

// ---------------- 스마트폰 블루투스 조종 (웹앱 tilt_phone.html) ----------------
// 1 = 켬. 블루투스 라이브러리가 커서 보드 설정의 Partition Scheme 을 "Huge APP (3MB No OTA)"로 바꿔야 합니다.
// 무선 조종기(ESP-NOW)와 함께 쓸 수 있고, 마지막으로 신호를 보낸 쪽을 따릅니다.
#define USE_BLE 1
#if USE_BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>
#define BLE_SERVICE_UUID "7f1a0001-6d3c-4b2a-9e5f-1b2c3d4e5f60"   // 웹앱과 같아야 함
#define BLE_TILT_UUID    "7f1a0002-6d3c-4b2a-9e5f-1b2c3d4e5f60"   // 폰 → 미로: int16 오른쪽, int16 앞쪽 (0.01°), uint8 플래그
#define BLE_STATUS_UUID  "7f1a0003-6d3c-4b2a-9e5f-1b2c3d4e5f60"   // 미로 → 폰: int16 판X, int16 판Y (0.01°), uint8 입력원
#endif

// ---------------- 설정 (조종기와 같게) ----------------
#define GROUP_ID      1
#define WIFI_CHANNEL  1

// ---------------- 핀 ----------------
#define PIN_SERVO_X   18
#define PIN_SERVO_Y   19
#define PIN_LED       2

// ---------------- 조작감 ----------------
float HAND_GAIN = 0.35f;   // 조종기 1° → 미로판 0.35°  (손을 25° 기울이면 판 약 8.8°)
float MAX_TILT  = 8.0f;    // 미로판 최대 기울기 (°)  — 10° 넘기지 않기
float DEADZONE  = 1.5f;    // 조종기 ±1.5° 이내는 수평으로 간주

// ---------------- 부드러움 (떨림이 남으면 이 값들을 조절) ----------------
// ② 적응형 필터
float FILTER_MIN_CUTOFF = 2.0f;   // Hz. 낮출수록 가만히 있을 때 더 매끄러움(대신 살짝 늦음)
float FILTER_BETA       = 0.03f;  // 클수록 빠르게 움직일 때 덜 늦음
// ③ PID 추종 제어 (가상의 판:  가속도 = KP·오차 − KD·속도 + KI·∫오차)
//    추천 조합      KP    KD    MAX_SPEED  MAX_ACCEL   손을 휙 꺾었을 때 90% 따라가는 시간
//    아주 부드럽게   70   16.7     60        450         약 0.45초
//    기본           150   24.5     80        700         약 0.33초
//    빠릿하게       200   28.3     90        900         약 0.29초  (서보가 버거우면 떨림이 다시 생길 수 있음)
float PID_KP    = 150.0f;  // 따라가는 세기 [1/s²]. 클수록 빠릿함
float PID_KD    = 24.5f;   // 감쇠. 2·√KP 이면 넘침(오버슈트) 없이 딱 멈춤
float PID_KI    = 0.0f;    // 판 각도 센서가 없으면 0 유지
float MAX_SPEED = 80.0f;   // 미로판 최대 각속도 [°/s]  (서보 기준 약 230°/s — MG996R 한계 안쪽)
float MAX_ACCEL = 700.0f;  // 미로판 최대 각가속도 [°/s²]. 작을수록 출발·정지가 부드러움
// ④ 정지 구간 출력 고정
float HOLD_BAND = 0.15f;   // 판이 멈춘 뒤 이보다 작은 변화는 서보에 보내지 않음 [°] (서보 기준 약 0.65°)
const float HOLD_SPEED = 0.3f;          // '멈춤' 기준 속도 [°/s]
const float HOLD_AFTER = 0.3f;          // 이 시간 동안 멈춰 있으면 출력 고정 시작 [s]
// ① 튀는 값
const float SPIKE_DEG = 40.0f;          // 한 패킷(20ms) 사이에 이보다 크게 바뀌면 의심
const uint32_t LINK_TIMEOUT_MS = 500;   // 이 시간 동안 신호 없으면 수평으로 복귀
const float CONTROL_HZ = 200.0f;        // 제어 주기

// 조종기 '오른쪽' = 미로판 +X,  '앞쪽' = 미로판 +Y  (조종하는 사람이 -Y 쪽에 서서 +Y를 봄)
// 구슬이 반대로 가면 아래 값을 -1 로
const int MAP_RIGHT_DIR = 1;
const int MAP_FRONT_DIR = 1;

// ---------------- 기구 치수 (3D 모델과 동일) ----------------
const float R_ATTACH = 80.0f;   // 회전중심 ~ 링크 연결점 거리 (mm)
const float HORN_R   = 20.0f;   // 서보 축 ~ 혼에 링크를 단 구멍 거리 (mm)  ← 실제 구멍 위치로 수정

// ---------------- 패킷 (조종기와 동일) ----------------
typedef struct __attribute__((packed)) {
  uint8_t  magic;
  uint8_t  group;
  uint16_t seq;
  int16_t  tiltRight;
  int16_t  tiltFront;
  uint8_t  flags;
} TiltPacket;

Preferences prefs;
float centerX = 90, centerY = 90;     // 서보 중앙 (°) — 미로판 수평이 되는 값
int   dirX = 1, dirY = 1;             // 서보 회전방향
bool  holdLevel = false;
bool  plotMode = false;
bool  webMode = false;     // 웹 시뮬레이터 연결 시 (PC가 'W'를 보내면 켜짐)

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
volatile float rxRight = 0, rxFront = 0;
volatile uint32_t lastRxMs = 0;
volatile uint32_t rxCount = 0;
volatile uint8_t lastSrc = 0;            // 1: 무선 조종기(ESP-NOW)  2: 스마트폰(BLE)

// ================================================================
//  ② One Euro 필터 — 느릴 땐 강하게, 빠를 땐 약하게 거르는 적응형 저역통과 필터
// ================================================================
struct OneEuro {
  float x = 0, dx = 0;
  bool init = false;
  static float alpha(float cutoff, float dt) {
    float tau = 1.0f / (2.0f * PI * cutoff);
    return 1.0f / (1.0f + tau / dt);
  }
  float update(float v, float dt) {
    if (!init) { x = v; dx = 0; init = true; return x; }
    float d = (v - x) / dt;
    dx += alpha(1.0f, dt) * (d - dx);                       // 속도 추정도 살짝 거름
    float cutoff = FILTER_MIN_CUTOFF + FILTER_BETA * fabsf(dx);
    x += alpha(cutoff, dt) * (v - x);
    return x;
  }
  void reset(float v) { x = v; dx = 0; init = true; }
};

// ================================================================
//  ③ PID 추종 제어기 + 속도·가속도 제한  (축마다 하나)
//     '가상의 판' 각도 pos 가 목표를 따라가며, 그 pos 를 서보에 보냅니다.
// ================================================================
struct TiltAxis {
  float pos = 0, vel = 0, integ = 0;
  float out = 0;            // 실제로 서보에 보낸 값 (④ 적용 후)
  bool  holding = false;    // 출력 고정 중?
  float still = 0;          // 멈춰 있던 시간
  void step(float target, float dt) {
    float err = target - pos;
    if (PID_KI > 0) integ = constrain(integ + err * dt, -2.0f, 2.0f);   // 적분 누적 제한(와인드업 방지)
    float acc = PID_KP * err - PID_KD * vel + PID_KI * integ;
    acc = constrain(acc, -MAX_ACCEL, MAX_ACCEL);
    vel = constrain(vel + acc * dt, -MAX_SPEED, MAX_SPEED);
    pos += vel * dt;
    if (fabsf(pos) > MAX_TILT) { pos = copysignf(MAX_TILT, pos); vel = 0; }
    // ④ 정지 구간 출력 고정: 0.3초 이상 멈춰 있으면 출력을 고정하고,
    //    목표가 HOLD_BAND 넘게 벗어나면 다시 매끄럽게 따라가기 시작
    if (holding) {
      if (fabsf(pos - out) > HOLD_BAND) { holding = false; out = pos; still = 0; }
    } else {
      out = pos;
      still = (fabsf(vel) < HOLD_SPEED) ? still + dt : 0;
      if (still > HOLD_AFTER) holding = true;
    }
  }
};

OneEuro filtR, filtF;
TiltAxis axX, axY;
float tgtX = 0, tgtY = 0;          // 판 목표 각도 (°)  +X: +X쪽이 내려감,  +Y: +Y쪽이 내려감
float rawR = 0, rawF = 0, fR = 0, fF = 0;

// ---------------- 서보 PWM (16비트) ----------------
const int PWM_FREQ = 50, PWM_BITS = 16;
#if ESP_ARDUINO_VERSION_MAJOR >= 3
void servoAttach(int pin, int ch) { (void)ch; ledcAttach(pin, PWM_FREQ, PWM_BITS); }
void servoWriteDuty(int pin, int ch, uint32_t d) { (void)ch; ledcWrite(pin, d); }
#else
void servoAttach(int pin, int ch) { ledcSetup(ch, PWM_FREQ, PWM_BITS); ledcAttachPin(pin, ch); }
void servoWriteDuty(int pin, int ch, uint32_t d) { (void)pin; ledcWrite(ch, d); }
#endif

void servoWriteDeg(int pin, int ch, float deg) {
  deg = constrain(deg, 0.0f, 180.0f);
  float us = 500.0f + deg * 2000.0f / 180.0f;               // 0°=500µs, 180°=2500µs
  servoWriteDuty(pin, ch, (uint32_t)(us * ((1 << PWM_BITS) - 1) / 20000.0f + 0.5f));
}

// 미로판 기울기(°) → 혼 각도(°)   (링크 연결점이 내려가려면 혼이 아래로)
float hornFromTilt(float tiltDeg) {
  float s = R_ATTACH * sinf(tiltDeg * DEG_TO_RAD) / HORN_R;
  s = constrain(s, -0.95f, 0.95f);
  return -asinf(s) * RAD_TO_DEG;
}

void applyServos() {
  servoWriteDeg(PIN_SERVO_X, 0, centerX + dirX * hornFromTilt(axX.out));
  servoWriteDeg(PIN_SERVO_Y, 1, centerY + dirY * hornFromTilt(axY.out));
}

// ---------------- ESP-NOW 수신 ----------------
void handlePacket(const uint8_t *data, int len) {
  if (len != sizeof(TiltPacket)) return;
  TiltPacket p;
  memcpy(&p, data, sizeof(p));
  if (p.magic != 'M' || p.group != GROUP_ID) return;
  portENTER_CRITICAL(&mux);
  rxRight = p.tiltRight / 100.0f;
  rxFront = p.tiltFront / 100.0f;
  lastRxMs = millis();
  rxCount = rxCount + 1;
  lastSrc = 1;
  portEXIT_CRITICAL(&mux);
}

#if USE_BLE
// ---------------- BLE 수신 (스마트폰 웹앱) ----------------
BLECharacteristic *bleStatus = nullptr;
volatile bool bleConnected = false;

class TiltWriteCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override {
    size_t n = c->getLength();
    if (n < 4) return;
    const uint8_t *d = c->getData();
    int16_t r = (int16_t)(d[0] | (d[1] << 8));
    int16_t f = (int16_t)(d[2] | (d[3] << 8));
    bool pause = (n >= 5) && (d[4] & 1);      // 웹앱의 '일시정지' → 수평
    portENTER_CRITICAL(&mux);
    rxRight = pause ? 0 : r / 100.0f;
    rxFront = pause ? 0 : f / 100.0f;
    lastRxMs = millis();
    rxCount = rxCount + 1;
    lastSrc = 2;
    portEXIT_CRITICAL(&mux);
  }
};
volatile int bleEvent = 0;            // 1 연결됨, 2 끊김 → loop에서 시리얼로 알림
volatile uint8_t bleDiscReason = 0;
volatile uint32_t bleConnAt = 0, bleDiscCount = 0;
class ServerCB : public BLEServerCallbacks {
  void onConnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    bleConnected = true; bleConnAt = millis(); bleEvent = 1;
    // 연결 간격 15~30 ms, 응답 없이 버티는 시간 6초.
    // ESP32는 무선(ESP-NOW)과 블루투스가 안테나 하나를 나눠 쓰므로, 버티는 시간이 짧으면 잠깐 끊겨도 연결이 끊어집니다.
    s->updateConnParams(param->connect.remote_bda, 12, 24, 0, 600);
  }
  void onDisconnect(BLEServer *s, esp_ble_gatts_cb_param_t *param) override {
    bleConnected = false; bleDiscReason = param->disconnect.reason;
    bleDiscCount = bleDiscCount + 1; bleEvent = 2;
    BLEDevice::startAdvertising();      // 다시 연결 가능하게
  }
};
void bleReportEvents() {               // loop에서 호출
  int e = bleEvent;
  if (!e) return;
  bleEvent = 0;
  if (e == 1) { Serial.println("[블루투스] 폰 연결됨"); return; }
  uint8_t r = bleDiscReason;
  const char *why = r == 0x08 ? "신호 끊김(시간 초과) — 거리·장애물·전원 잡음·무선 혼잡"
                  : r == 0x13 ? "폰 쪽에서 끊음 — 화면 꺼짐, 다른 앱으로 전환, 연결 끊기 버튼"
                  : r == 0x16 ? "미로 쪽에서 끊음"
                  : r == 0x3E ? "연결 맺기 실패 — 다시 시도"
                  : r == 0x22 ? "응답 시간 초과(LMP)"
                  : "기타";
  Serial.printf("[블루투스] 끊김 #%lu  이유 0x%02X: %s  (연결 유지 %.1f초)\n",
                (unsigned long)bleDiscCount, r, why, (millis() - bleConnAt) / 1000.0f);
}

void bleBegin() {
  char name[20];
  snprintf(name, sizeof(name), "TiltMaze-%d", GROUP_ID);   // 폰에서 고를 이름 (그룹마다 다름)
  BLEDevice::init(name);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCB());
  BLEService *svc = srv->createService(BLE_SERVICE_UUID);
  BLECharacteristic *tilt = svc->createCharacteristic(BLE_TILT_UUID,
      BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  tilt->setCallbacks(new TiltWriteCB());
  bleStatus = svc->createCharacteristic(BLE_STATUS_UUID,
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  bleStatus->addDescriptor(new BLE2902());
  svc->start();
  // 광고 패킷은 31바이트까지라서 이름과 128비트 UUID를 나눠 싣습니다.
  //  광고: 플래그 + 이름 전체  /  스캔 응답: 서비스 UUID  → 폰이 이름으로도, UUID로도 찾을 수 있음
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  BLEAdvertisementData advData, scanData;
  advData.setFlags(ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT);
  advData.setName(name);
  scanData.setCompleteServices(BLEUUID(BLE_SERVICE_UUID));
  adv->setAdvertisementData(advData);
  adv->setScanResponseData(scanData);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.printf("블루투스 이름: %s (스마트폰 웹앱에서 이 이름을 고르세요)\n", name);
}

void bleNotifyStatus(float bx, float by) {
  static uint32_t last = 0;
  if (!bleConnected || !bleStatus || millis() - last < 100) return;   // 10Hz
  last = millis();
  int16_t x = (int16_t)(bx * 100), y = (int16_t)(by * 100);
  uint8_t d[5] = {(uint8_t)(x & 0xFF), (uint8_t)(x >> 8), (uint8_t)(y & 0xFF), (uint8_t)(y >> 8), lastSrc};
  bleStatus->setValue(d, 5);
  bleStatus->notify();
}
#endif

#if ESP_ARDUINO_VERSION_MAJOR >= 3
void onRecv(const esp_now_recv_info_t *info, const uint8_t *data, int len) { (void)info; handlePacket(data, len); }
#else
void onRecv(const uint8_t *mac, const uint8_t *data, int len) { (void)mac; handlePacket(data, len); }
#endif

// ---------------- 설정 저장 ----------------
void saveSettings() {
  prefs.putFloat("cx", centerX); prefs.putFloat("cy", centerY);
  prefs.putInt("dx", dirX);      prefs.putInt("dy", dirY);
}
void printSettings() {
  Serial.printf("[설정] 중앙 X=%.1f° Y=%.1f°  방향 X=%+d Y=%+d  수평고정=%s  게인=%.2f 최대=%.1f°\n",
                centerX, centerY, dirX, dirY, holdLevel ? "ON" : "OFF", HAND_GAIN, MAX_TILT);
  Serial.printf("[부드러움] 필터 %.1fHz/%.3f  KP %.0f KD %.1f KI %.1f  최대속도 %.0f°/s 최대가속 %.0f°/s²  고정 %.2f°\n",
                FILTER_MIN_CUTOFF, FILTER_BETA, PID_KP, PID_KD, PID_KI, MAX_SPEED, MAX_ACCEL, HOLD_BAND);
}

float shape(float hand) {   // 데드존 + 게인 + 제한 (경계에서 끊김 없이 이어지게)
  if (fabsf(hand) < DEADZONE) return 0;
  float v = (hand - copysignf(DEADZONE, hand)) * HAND_GAIN;
  return constrain(v, -MAX_TILT, MAX_TILT);
}

void runFor(float tx, float ty, uint32_t ms) {   // 테스트용: 목표를 주고 제어기를 돌림
  const float dt = 1.0f / CONTROL_HZ;
  for (uint32_t t = 0; t < ms; t += (uint32_t)(dt * 1000)) {
    axX.step(tx, dt); axY.step(ty, dt);
    applyServos();
    delayMicroseconds((uint32_t)(dt * 1e6f));
  }
}

void testSweep() {
  Serial.println("테스트: +X쪽 → -X쪽 → +Y쪽 → -Y쪽 순서로 내려가야 정상");
  float seqs[][2] = {{1, 0}, {-1, 0}, {0, 0}, {0, 1}, {0, -1}, {0, 0}};
  for (auto &s : seqs) {
    runFor(s[0] * MAX_TILT, s[1] * MAX_TILT, 900);
    Serial.printf("  판 X %+5.1f°  Y %+5.1f°\n", axX.out, axY.out);
  }
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'W') { webMode = true; continue; }   // 웹 시뮬레이터: TILT 줄을 50Hz로 보냄
    switch (c) {
      case 'q': centerX += 0.5f; break;
      case 'a': centerX -= 0.5f; break;
      case 'w': centerY += 0.5f; break;
      case 's': centerY -= 0.5f; break;
      case 'x': dirX = -dirX; break;
      case 'y': dirY = -dirY; break;
      case 'h': holdLevel = !holdLevel; break;
      case 't': testSweep(); break;
      case 'g': plotMode = !plotMode; break;
      case 'p': break;
      default: continue;
    }
    centerX = constrain(centerX, 45.0f, 135.0f);
    centerY = constrain(centerY, 45.0f, 135.0f);
    saveSettings();
    if (!plotMode) printSettings();
  }
}

// ---------------- 설정 ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  {
    esp_reset_reason_t rr = esp_reset_reason();   // 왜 다시 켜졌는지: 전원 부족이면 커패시터·어댑터 점검
    if (rr == ESP_RST_BROWNOUT)
      Serial.println("!! 전원 전압이 떨어져 재부팅됐습니다 (brownout). 서보 전원·커패시터·어댑터를 점검하세요.");
    else if (rr == ESP_RST_PANIC || rr == ESP_RST_INT_WDT || rr == ESP_RST_TASK_WDT || rr == ESP_RST_WDT)
      Serial.printf("!! 프로그램 오류로 재부팅됐습니다 (이유 %d).\n", (int)rr);
  }
  pinMode(PIN_LED, OUTPUT);

  prefs.begin("tiltmaze", false);
  centerX = prefs.getFloat("cx", 90);  centerY = prefs.getFloat("cy", 90);
  dirX = prefs.getInt("dx", 1);        dirY = prefs.getInt("dy", 1);

  servoAttach(PIN_SERVO_X, 0);
  servoAttach(PIN_SERVO_Y, 1);
  applyServos();                     // 바로 수평 자세

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패");
    ESP.restart();
  }
  esp_now_register_recv_cb(onRecv);
#if USE_BLE
  bleBegin();
#endif

  uint8_t mac[6];
  esp_read_mac(mac, ESP_MAC_WIFI_STA);   // WiFi.macAddress()는 코어 3.x에서 00:00:..으로 나올 때가 있음
  Serial.printf("미로 본체 준비 완료 (그룹 %d, 채널 %d)  MAC %02X:%02X:%02X:%02X:%02X:%02X\n",
                GROUP_ID, WIFI_CHANNEL, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
#if USE_BLE
  Serial.printf("블루투스: 켜짐 (이름 TiltMaze-%d)  [펌웨어 2026-10-20b]\n", GROUP_ID);
#else
  Serial.println("블루투스: 꺼짐 (USE_BLE 0) — 스마트폰 웹앱으로는 연결되지 않습니다  [펌웨어 2026-10-20b]");
#endif
  Serial.println("명령: q/a w/s(중앙) x/y(방향) h(수평 고정) t(테스트) g(그래프) p(설정)");
  printSettings();
}

// ---------------- 반복 ----------------
void loop() {
  static uint32_t lastTickUs = micros(), lastPktMs = 0, lastPrint = 0, lastCount = 0, seenCount = 0;
  static int spikeStrikes = 0;
  static float accR = 0, accF = 0;        // 마지막으로 받아들인 원래값
  handleSerial();

  const uint32_t periodUs = (uint32_t)(1e6f / CONTROL_HZ);
  uint32_t nowUs = micros();
  if (nowUs - lastTickUs < periodUs) return;
  float dt = (nowUs - lastTickUs) * 1e-6f;
  lastTickUs = nowUs;
  if (dt > 0.05f) dt = 0.05f;

  float r, f; uint32_t t, cnt;
  portENTER_CRITICAL(&mux);
  r = rxRight; f = rxFront; t = lastRxMs; cnt = rxCount;
  portEXIT_CRITICAL(&mux);
  bool linked = (t != 0) && (millis() - t < LINK_TIMEOUT_MS);

  // 새 패킷이 왔을 때만 ①② 처리 (패킷 간격을 dt 로 사용)
  if (cnt != seenCount) {
    seenCount = cnt;
    float pdt = lastPktMs ? constrain((t - lastPktMs) * 1e-3f, 0.005f, 0.1f) : 0.02f;
    lastPktMs = t;
    rawR = MAP_RIGHT_DIR * r;
    rawF = MAP_FRONT_DIR * f;
    // ① 튀는 값: 직전에 받아들인 값과 SPIKE_DEG 넘게 차이 나면 2번까지는 버림 (3번 연속이면 진짜 움직임)
    bool spike = filtR.init && (fabsf(rawR - accR) > SPIKE_DEG || fabsf(rawF - accF) > SPIKE_DEG);
    if (spike && spikeStrikes < 2) {
      spikeStrikes++;
    } else {
      spikeStrikes = 0;
      accR = rawR; accF = rawF;
      fR = filtR.update(rawR, pdt);
      fF = filtF.update(rawF, pdt);
    }
  }

  if (linked && !holdLevel) { tgtX = shape(fR); tgtY = shape(fF); }
  else {
    tgtX = 0; tgtY = 0;
    filtR.reset(0); filtF.reset(0); fR = fF = 0; accR = accF = 0;   // 다시 연결될 때 옛 값에서 튀지 않게
  }

  axX.step(tgtX, dt);
  axY.step(tgtY, dt);
  applyServos();

  digitalWrite(PIN_LED, linked ? ((cnt / 5) % 2) : ((millis() / 500) % 2));
#if USE_BLE
  bleReportEvents();
  bleNotifyStatus(axX.out, axY.out);
#endif

  if (webMode) {                          // 웹 시뮬레이터: TILT,손R,손F,판X,판Y
    if (millis() - lastPrint >= 20) {
      lastPrint = millis();
      Serial.printf("TILT,%.2f,%.2f,%.2f,%.2f\n", linked ? fR : 0.0f, linked ? fF : 0.0f, axX.out, axY.out);
    }
  } else if (plotMode) {                  // 아두이노 IDE 시리얼 플로터용
    if (millis() - lastPrint >= 20) {
      lastPrint = millis();
      Serial.printf("raw:%.2f,filtered:%.2f,target:%.2f,board:%.2f\n",
                    rawR * HAND_GAIN, fR * HAND_GAIN, tgtX, axX.out);
    }
  } else if (millis() - lastPrint > 500) {
    lastPrint = millis();
    Serial.printf("%s  조종기 R%+6.1f F%+6.1f → 판 X%+5.1f° Y%+5.1f°  (%lu pkt/s)\n",
                  linked ? (lastSrc == 2 ? "폰(BLE)" : "조종기") : "대기", r, f, axX.out, axY.out,
                  (unsigned long)((cnt - lastCount) * 2));
    lastCount = cnt;
  }
}
