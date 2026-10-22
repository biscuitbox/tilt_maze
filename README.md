# 원격 기울기 미로 (ESP32 · 3D 프린팅)

손에 든 조종기나 **스마트폰을 기울이면**, 떨어져 있는 300 × 300 mm 미로판이 같은 방향으로 기울어집니다. 구슬을 출발점에서 도착 홈까지 굴려 보내는 메이커 작품입니다.

| 바로 가기 | 주소 |
|---|---|
| 📱 스마트폰으로 조종하기 (미로의 QR 안내판이 여는 곳) | https://biscuitbox.github.io/tilt_maze/ |
| 📘 만드는 방법 · 3D 시뮬레이터 · 배선도 · 조립 순서 | https://biscuitbox.github.io/tilt_maze/guide/ |
| ⬇ 전체 파일 한 번에 받기 | [tilt-maze-all.zip](tilt-maze-all.zip) |

## 어떻게 움직이나요

조종기(ESP32 + MPU6050)가 기울기를 읽어 무선(ESP-NOW)으로 보내면, 미로 쪽 ESP32가 서보 2개로 미로판을 앞뒤·좌우로 기울입니다. 스마트폰은 웹 블루투스로 미로 ESP32에 바로 연결됩니다. 서보 떨림을 줄이려고 필터와 PID 제어로 판을 부드럽게 움직입니다.

## 준비물

**전자 부품**: ESP32 DevKit 2개, MPU6050(GY-521) 1개, MG996R 서보 2개, 5V 3A 어댑터 + DC잭, 전해 커패시터 220~1000 µF, 푸시버튼 1개, 보조배터리, 점퍼선

**기구 부품**: M3 볼트·너트(16·20·10 mm), M4 × 35 볼트 2개, 케이블타이 3개, 고무발 4개, Ø16 구슬 — 볼링크 같은 RC 부품은 필요 없습니다. 자세한 수량은 [만드는 방법](https://biscuitbox.github.io/tilt_maze/guide/)의 준비물 표를 보세요.

## 폴더

| 폴더 | 내용 |
|---|---|
| [`stl/`](stl) | 3D 출력 파일. 뱀부랩 H2S(340 × 320 mm) 기준, PLA, 서포트 없음. 미로판은 300 mm 통짜 |
| [`code/`](code) | 아두이노 코드 — `tilt_maze_receiver`(미로), `tilt_controller`(MPU6050 조종기), `tilt_controller_amoled`(원형 화면 조종기) |
| [`cad/`](cad) | 파이썬 설계 원본(manifold3d). 치수나 미로 모양을 바꿔 STL을 다시 만들 수 있음 |
| [`guide/`](guide) | 만드는 방법 페이지 |
| `index.html` | 스마트폰 조종 페이지 |
| `firestore_tilt_maze.rules` | 스마트폰 기록 순위용 파이어베이스 보안 규칙 |

## 처음 만들 때 주의

- 미로 쪽 코드를 올릴 때 아두이노 IDE의 **도구 → Partition Scheme → Huge APP**을 고르세요 (블루투스 때문에 코드가 큽니다).
- 미로 ESP32는 USB(보조배터리)로 따로 켜고, 서보만 5V 어댑터로 켜면 재부팅·블루투스 끊김이 거의 없습니다. GND는 서로 연결합니다.
- 서보 혼을 끼우기 **전에** 코드를 올려 서보를 90°로 세운 뒤 혼을 수평으로 끼웁니다.

자세한 배선도, 링크 연결 그림, 문제 해결은 [만드는 방법](https://biscuitbox.github.io/tilt_maze/guide/) 페이지에 있습니다.
