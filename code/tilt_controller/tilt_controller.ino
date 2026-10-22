/*
 * ================================================================
 *  원격 기울기 미로 — 조종기 (ESP32 + MPU6050)
 * ================================================================
 *  손에 든 조종기를 기울이면 그 각도를 ESP-NOW로 미로 쪽 ESP32에 보냅니다.
 *
 *  [배선]  MPU6050(GY-521)    ESP32
 *          VCC  ───────────── 3V3
 *          GND  ───────────── GND
 *          SCL  ───────────── GPIO 22
 *          SDA  ───────────── GPIO 21
 *          (AD0 미연결 → 주소 0x68)
 *          푸시버튼 한쪽 ───── GPIO 4,  다른 쪽 ───── GND
 *
 *  [설치] GY-521 보드의 X 화살표가 조종기 '앞'(미로 쪽)을 향하게,
 *         칩이 위를 보게 케이스 바닥 틀에 붙입니다.
 *
 *  [사용] 1) 전원을 켜고 2초 동안 조종기를 가만히 둡니다(자이로 보정).
 *         2) 조종기를 편하게 '수평'이라고 느끼는 자세로 잡고 버튼을 짧게 누르면
 *            그 자세가 0°로 저장됩니다(전원을 꺼도 유지).
 *
 *  보드: ESP32 Dev Module   /  ESP32 Arduino 코어 2.x, 3.x 모두 지원
 *  추가 라이브러리 필요 없음
 * ================================================================
 */
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>

// ---------------- 설정 (미로 쪽과 같게) ----------------
#define GROUP_ID      1     // 같은 교실에서 여러 세트를 쓸 때 세트마다 다르게 (1~255)
#define WIFI_CHANNEL  1     // 1~13, 미로 쪽과 같아야 함

// ---------------- 핀 ----------------
#define PIN_SDA   21
#define PIN_SCL   22
#define PIN_BTN   4
#define PIN_LED   2         // 보드 내장 LED (없는 보드는 무시)

// ---------------- 센서 방향 보정 ----------------
// 조종기를 오른쪽으로 기울였는데 미로가 앞뒤로 움직이면 SWAP_AXES 를 true 로,
// 방향이 반대면 해당 INVERT 를 바꾸세요. (미로 쪽 코드에서도 바꿀 수 있음)
const bool SWAP_AXES    = false;
const bool INVERT_RIGHT = false;
const bool INVERT_FRONT = false;

const float FILTER_ALPHA = 0.96f;   // 상보 필터: 클수록 자이로 비중↑(부드러움), 작을수록 가속도 비중↑
const uint32_t SEND_INTERVAL_MS = 20;   // 50 Hz 송신

// ---------------- 패킷 (미로 쪽과 동일해야 함) ----------------
typedef struct __attribute__((packed)) {
  uint8_t  magic;        // 'M'
  uint8_t  group;        // GROUP_ID
  uint16_t seq;          // 순번
  int16_t  tiltRight;    // 오른쪽이 내려간 각도 (0.01°)
  int16_t  tiltFront;    // 앞쪽이 내려간 각도 (0.01°)
  uint8_t  flags;        // bit0: 버튼 눌림
} TiltPacket;

static const uint8_t BROADCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static uint8_t mpuAddr = 0x68;
Preferences prefs;

float gyroBiasX = 0, gyroBiasY = 0;
float roll = 0, pitch = 0;        // 필터 결과 (deg)
float zeroRoll = 0, zeroPitch = 0;
uint16_t seq = 0;

// ---------------- MPU6050 ----------------
bool i2cWrite(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool mpuRead(float &ax, float &ay, float &az, float &gx, float &gy) {
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)mpuAddr, 14) != 14) return false;
  uint8_t b[14];
  for (int i = 0; i < 14; i++) b[i] = Wire.read();
  int16_t rax = (b[0] << 8) | b[1];
  int16_t ray = (b[2] << 8) | b[3];
  int16_t raz = (b[4] << 8) | b[5];
  int16_t rgx = (b[8] << 8) | b[9];
  int16_t rgy = (b[10] << 8) | b[11];
  ax = rax / 16384.0f;  ay = ray / 16384.0f;  az = raz / 16384.0f;   // ±2g
  gx = rgx / 131.0f;    gy = rgy / 131.0f;                           // ±250°/s
  return true;
}

bool mpuBegin() {
  for (uint8_t a : {0x68, 0x69}) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) { mpuAddr = a; break; }
    if (a == 0x69) return false;
  }
  i2cWrite(0x6B, 0x00);   // 절전 해제
  delay(50);
  i2cWrite(0x1A, 0x03);   // 디지털 저역필터 44Hz
  i2cWrite(0x1B, 0x00);   // 자이로 ±250°/s
  i2cWrite(0x1C, 0x00);   // 가속도 ±2g
  Wire.beginTransmission(mpuAddr);
  Wire.write(0x75);
  Wire.endTransmission(false);
  Wire.requestFrom((int)mpuAddr, 1);
  Serial.printf("MPU 주소 0x%02X, WHO_AM_I=0x%02X (0x68 정품 / 0x70·0x72·0x98 호환칩도 동작)\n",
                mpuAddr, Wire.available() ? Wire.read() : 0);
  return true;
}

void accAngles(float ax, float ay, float az, float &r, float &p) {
  r = atan2f(ay, az) * RAD_TO_DEG;                          // 오른쪽 내려가면 +
  p = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;   // 앞쪽 내려가면 +
}

void calibrateGyro() {
  Serial.println("자이로 보정 중... 조종기를 움직이지 마세요");
  float sx = 0, sy = 0, sr = 0, sp = 0;
  const int N = 400;
  for (int i = 0; i < N; i++) {
    float ax, ay, az, gx, gy;
    if (mpuRead(ax, ay, az, gx, gy)) {
      sx += gx; sy += gy;
      float r, p; accAngles(ax, ay, az, r, p);
      sr += r; sp += p;
    }
    digitalWrite(PIN_LED, (i / 25) % 2);
    delay(5);
  }
  gyroBiasX = sx / N;  gyroBiasY = sy / N;
  roll = sr / N;       pitch = sp / N;
  Serial.printf("자이로 바이어스 X=%.2f Y=%.2f °/s\n", gyroBiasX, gyroBiasY);
}

// ---------------- 설정 ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  pinMode(PIN_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);

  Wire.begin(PIN_SDA, PIN_SCL, 400000);
  if (!mpuBegin()) {
    Serial.println("MPU6050을 찾지 못했습니다. SDA/SCL/전원 배선을 확인하세요.");
    while (true) { digitalWrite(PIN_LED, !digitalRead(PIN_LED)); delay(100); }
  }

  prefs.begin("tiltctl", false);
  zeroRoll  = prefs.getFloat("zr", 0);
  zeroPitch = prefs.getFloat("zp", 0);

  calibrateGyro();

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW 초기화 실패");
    ESP.restart();
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BROADCAST, 6);
  peer.channel = WIFI_CHANNEL;
  peer.encrypt = false;
  peer.ifidx = WIFI_IF_STA;
  esp_now_add_peer(&peer);

  Serial.printf("조종기 준비 완료  (그룹 %d, 채널 %d)\n", GROUP_ID, WIFI_CHANNEL);
  Serial.println("버튼 짧게: 현재 자세를 수평(0°)으로 저장");
}

// ---------------- 반복 ----------------
void loop() {
  static uint32_t lastUs = micros(), lastSend = 0, lastPrint = 0, btnDown = 0;
  static bool btnPrev = false;
  static bool webMode = false;               // 웹 시뮬레이터가 'W'를 보내면 켜짐
  while (Serial.available()) if (Serial.read() == 'W') webMode = true;

  float ax, ay, az, gx, gy;
  if (!mpuRead(ax, ay, az, gx, gy)) { delay(5); return; }

  uint32_t now = micros();
  float dt = (now - lastUs) * 1e-6f;
  lastUs = now;
  if (dt > 0.1f) dt = 0.1f;

  float rAcc, pAcc;
  accAngles(ax, ay, az, rAcc, pAcc);
  roll  = FILTER_ALPHA * (roll  + (gx - gyroBiasX) * dt) + (1 - FILTER_ALPHA) * rAcc;
  pitch = FILTER_ALPHA * (pitch + (gy - gyroBiasY) * dt) + (1 - FILTER_ALPHA) * pAcc;

  // 버튼: 짧게 누르면 수평 저장
  bool btn = digitalRead(PIN_BTN) == LOW;
  if (btn && !btnPrev) btnDown = millis();
  if (!btn && btnPrev && millis() - btnDown > 40) {
    zeroRoll = roll;  zeroPitch = pitch;
    prefs.putFloat("zr", zeroRoll);
    prefs.putFloat("zp", zeroPitch);
    Serial.printf("수평 저장: roll %.1f°, pitch %.1f°\n", zeroRoll, zeroPitch);
    for (int i = 0; i < 6; i++) { digitalWrite(PIN_LED, i % 2); delay(60); }
  }
  btnPrev = btn;

  if (millis() - lastSend >= SEND_INTERVAL_MS) {
    lastSend = millis();
    float right = roll - zeroRoll;
    float front = pitch - zeroPitch;
    if (SWAP_AXES) { float t = right; right = front; front = t; }
    if (INVERT_RIGHT) right = -right;
    if (INVERT_FRONT) front = -front;

    TiltPacket pkt;
    pkt.magic = 'M';
    pkt.group = GROUP_ID;
    pkt.seq = seq++;
    pkt.tiltRight = (int16_t)constrain(right * 100.0f, -9000.0f, 9000.0f);
    pkt.tiltFront = (int16_t)constrain(front * 100.0f, -9000.0f, 9000.0f);
    pkt.flags = btn ? 1 : 0;
    esp_now_send(BROADCAST, (uint8_t *)&pkt, sizeof(pkt));
    digitalWrite(PIN_LED, (seq / 10) % 2);   // 송신 중 깜빡임

    if (webMode) {
      Serial.printf("TILT,%.2f,%.2f\n", right, front);          // 웹 시뮬레이터용 (50Hz)
    } else if (millis() - lastPrint > 200) {
      lastPrint = millis();
      Serial.printf("오른쪽 %+6.1f°   앞쪽 %+6.1f°\n", right, front);
    }
  }
  delay(2);
}
