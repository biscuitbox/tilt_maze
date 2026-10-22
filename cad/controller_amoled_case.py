"""
조종기 손잡이 케이스 — Waveshare ESP32-S3-Touch-AMOLED-1.75 (기판만 있는 기본형) + 리튬 배터리
python3 controller_amoled_case.py [출력폴더]

구성: 원형 프레임(보드 가장자리를 턱으로 받침) + 손잡이(배터리 칸) / 위에서 누르는 베젤 / 배터리 뚜껑 / 맞춤 확인 링
좌표: 보드 중심이 원점, 화면이 +Z, 손잡이는 -Y(내 쪽), 미로는 +Y(앞), 버튼 창은 +X(오른쪽)

★ 보드 치수는 제조사 수치가 공개돼 있지 않아 추정값입니다. 캘리퍼스로 재서 아래 두 값을 고친 뒤
  먼저 '14_맞춤확인링'(5분)을 뽑아 끼워 보고, 맞으면 나머지를 출력하세요.
"""
import math, os, sys
import manifold3d as mf
import tilt_maze_cad as T          # 같은 폴더의 도우미 함수(box, cyl, save …) 재사용

M = mf.Manifold
box, cyl_z, cyl_x, union, save = T.box, T.cyl_z, T.cyl_x, T.union, T.save

# ---------------- ★ 실측해서 고칠 값 ----------------
BOARD_D = 48.0       # 보드(화면 유리 또는 기판 중 큰 쪽)의 지름
STACK_FRONT = 3.8    # 기판 뒷면 ~ 화면 유리 윗면 두께 (뒷면 부품 제외)

# ---------------- 설계값 ----------------
CLR = 0.5            # 지름 여유
D_IN = BOARD_D + CLR
R_IN = D_IN / 2
WALL = 6.0
R_OUT = R_IN + WALL
LEDGE_W, LEDGE_T = 1.4, 2.0
Z_GLASS = -0.2                         # 유리 윗면 (프레임 윗면보다 0.2 낮게 → 베젤이 살짝 누름)
Z_LEDGE = Z_GLASS - STACK_FRONT        # 턱 윗면 = 기판 뒷면
Z_BOT = Z_LEDGE - LEDGE_T              # 프레임 아랫면
WIN_DEG = 75                           # 오른쪽 버튼·USB 창 각도
SCREW_ANG = (70, 180, 290)
R_SCREW = (R_IN + R_OUT) / 2

# 손잡이 / 배터리 (최대 45 x 27 x 11 mm 리튬폴리머, 예: 802540 500mAh)
H_LEN, H_W, H_H = 100.0, 34.0, 18.0
Y0 = -(R_OUT - 4)                      # 손잡이 시작 (프레임 벽과 겹침)
Y1 = -(R_OUT + H_LEN)
BAY_W, BAY_L, BAY_H = 27.5, 48.0, 12.0
BAY_Y1, BAY_Y0 = -42.0, -42.0 - BAY_L  # 배터리 칸 앞/뒤 끝
LID_T = 1.6
RECESS_Y1, RECESS_Y0 = BAY_Y1 + 6.5, BAY_Y0 - 6.5
LID_SCREW_Y = (RECESS_Y1 - 3.0, RECESS_Y0 + 3.0)

def sector(r, z0, z1, a0, a1, seg=64):
    pts = [(0, 0)] + [(r * math.cos(math.radians(a0 + (a1 - a0) * i / seg)),
                       r * math.sin(math.radians(a0 + (a1 - a0) * i / seg))) for i in range(seg + 1)]
    return M.extrude(mf.CrossSection([pts]), z1 - z0).translate([0, 0, z0])

def frame_ring():
    ring = cyl_z(0, 0, Z_BOT, 0, 2 * R_OUT, 128) - cyl_z(0, 0, Z_BOT - 1, 1, 2 * (R_IN - LEDGE_W), 128)
    ring = ring - cyl_z(0, 0, Z_LEDGE, 1, D_IN, 128)                  # 보드 자리
    ring = ring - sector(R_OUT + 2, Z_LEDGE, 1, -WIN_DEG / 2, WIN_DEG / 2)   # 버튼·USB 창
    for a in SCREW_ANG:
        x, y = R_SCREW * math.cos(math.radians(a)), R_SCREW * math.sin(math.radians(a))
        ring = ring - cyl_z(x, y, -5.0, 1, T.M3_PILOT, 24)
    return ring

def rounded_bar(x0, x1, y0, y1, z0, z1, r_v=6.0, r_b=5.0):
    """윗면 평평, 세로 모서리·아래 모서리 둥근 막대"""
    parts = []
    for x in (x0 + r_v, x1 - r_v):
        for y in (y0 + r_v, y1 - r_v):
            parts.append(cyl_z(x, y, z1 - 2, z1, 2 * r_v, 48))
            parts.append(M.sphere(r_b, 32).translate([x + math.copysign(r_v - r_b, x - (x0 + x1) / 2),
                                                       y + math.copysign(r_v - r_b, y - (y0 + y1) / 2), z0 + r_b]))
    return M.batch_hull(parts)

def housing():
    h = rounded_bar(-H_W / 2, H_W / 2, Y1, Y0, -H_H, 0)
    h = h - cyl_z(0, 0, Z_BOT - 30, 2, 2 * R_IN, 128)                 # 보드 영역 침범 금지
    body = union([frame_ring(), h])
    # 배터리 칸 + 뚜껑 자리
    body = body - box(-BAY_W / 2, BAY_W / 2, BAY_Y0, BAY_Y1, -H_H - 1, -H_H + LID_T + BAY_H)
    body = body - box(-BAY_W / 2 - 1.75, BAY_W / 2 + 1.75, RECESS_Y0, RECESS_Y1, -H_H - 1, -H_H + LID_T)
    for y in LID_SCREW_Y:
        body = body - cyl_z(0, y, -H_H - 1, -H_H + LID_T + 8, T.M3_PILOT, 24)
    # 배선 통로: 배터리 칸 앞 → 보드 뒷면 아래 빈 공간
    body = body - box(-4, 4, BAY_Y1 - 2, Y0 + 8, -H_H + LID_T + 1, Z_BOT - 0.6)
    # 줄 고리 구멍
    body = body - cyl_x(-H_W, H_W, Y1 + 9, -H_H / 2, 4.2, 24)
    return body

def bezel():
    b = cyl_z(0, 0, 0, 1.6, 2 * R_OUT, 128) - cyl_z(0, 0, -1, 3, BOARD_D - 2.4, 128)
    b = b + (cyl_z(0, 0, -0.4, 0.01, BOARD_D - 0.6, 128) - cyl_z(0, 0, -1, 1, BOARD_D - 2.4, 128))  # 누름 턱
    for a in SCREW_ANG:
        x, y = R_SCREW * math.cos(math.radians(a)), R_SCREW * math.sin(math.radians(a))
        b = b - cyl_z(x, y, -1, 3, T.M3_CLEAR, 24)
    return b

def battery_lid():
    w, l = BAY_W + 3.5 - 0.3, (RECESS_Y1 - RECESS_Y0) - 0.3
    lid = box(-w / 2, w / 2, -l / 2, l / 2, 0, LID_T)
    yc = (RECESS_Y1 + RECESS_Y0) / 2
    for y in LID_SCREW_Y:
        lid = lid - cyl_z(0, y - yc, -1, 3, T.M3_CLEAR, 24) - cyl_z(0, y - yc, -1, 0.9, 6.0, 24)
    return lid

def fit_ring():
    return frame_ring() ^ cyl_z(0, 0, Z_BOT - 1, 1, 2 * R_OUT + 2, 128)

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "stl")
    os.makedirs(out, exist_ok=True)
    print(f"보드 Ø{BOARD_D} / 두께 {STACK_FRONT} 기준")
    save(housing().rotate([180, 0, 0]), "11_조종기_손잡이", out)          # 윗면이 베드
    save(bezel().rotate([180, 0, 0]), "12_조종기_베젤", out)              # 앞면이 베드
    save(battery_lid(), "13_배터리_뚜껑", out)
    save(fit_ring().rotate([180, 0, 0]), "14_맞춤확인링", out)
