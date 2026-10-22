"""
원격 기울기 미로 (300 x 300 mm) — 3D 프린팅 부품 생성 스크립트
manifold3d + trimesh 로 STL 생성.   python3 tilt_maze_cad.py

좌표계: 원점 = 바닥판 중심의 책상면, Z 위쪽. 단위 mm.
 - 미로판 중심 위 유니버설 조인트(십자축) 2축 회전
 - 서보 X : (80, 0) 지점을 프린팅 링크로 밀어 Y축 둘레 기울기
 - 서보 Y : (0, 80) 지점을 밀어 X축 둘레 기울기
 - 링크 위쪽은 작은 2핀 관절(링크블록), 아래쪽은 혼에 M3 볼트 — 볼링크/볼스터드 불필요
프린터: 뱀부랩 H2S (출력 340 x 320 x 340 mm) — 미로판·하부프레임 통짜 출력.
"""
import math, random, os, sys
import numpy as np
import manifold3d as mf
import trimesh

M = mf.Manifold
SEG = 48

# ----------------------------------------------------------------- 파라미터
BOARD = 300.0          # 미로판 한 변
PITCH = 29.7           # 미로 칸 간격  (10 x 10 칸)
NCELL = 10
WALL_T = 3.0           # 벽 두께
WALL_H = 14.0          # 벽 높이 (구슬 Ø16 기준)
POST_D = 6.4           # 격자 기둥 지름 (나사 고정 기둥 겸용)
FLOOR_T = 4.0          # 미로판 바닥 두께
MAZE_SEED = 7          # 미로 모양 (숫자를 바꾸면 다른 미로)

Z_LOW = 81.5           # 아래 회전축(Y방향 볼트) 높이  -> 서보 X 가 만드는 기울기
Z_UP = Z_LOW + 17.0    # 위 회전축(X방향 볼트) 높이   -> 서보 Y 가 만드는 기울기
# 링크 아래핀(링크↔블록)은 회전축 위에 놓여야 다른 축 기울기에 영향이 없음
ATT_Z = {"X": Z_UP, "Y": Z_LOW}  # 서보X 링크는 위축 높이, 서보Y 링크는 아래축 높이
LINK_K = 7.0                    # 링크블록: 윗핀(탭) ~ 아랫핀(링크) 간격
HUB_T = 6.0
Z_HUB0 = Z_UP + 15.0            # 허브/이음판 아랫면 113.5
Z_FLOOR = Z_HUB0 + HUB_T        # 미로판 바닥 아랫면 119.5

R_ATTACH = 80.0        # 회전 중심 ~ 링크 연결점 거리
SERVO_Z = 30.0         # 서보 축 높이
HORN_R = 20.0          # 서보 축 ~ 혼 볼트 구멍 거리
SERVO_SHAFT_X = R_ATTACH - HORN_R   # 60

BED = (340.0, 320.0, 340.0)   # 뱀부랩 H2S 출력 크기 (X, Y, Z)

BASE = 180.0
BASE_T = 6.0

M3_PILOT = 2.6         # M3 나사가 직접 파고드는 구멍
M3_CLEAR = 3.4
M4_CLEAR = 4.4

# MG996R (클론마다 ±0.5mm 차이 → 실측 후 수정)
SV_L, SV_W = 40.7, 19.7
SV_SHAFT_OFF = 10.2    # 몸체 끝 ~ 축 중심
SV_HOLE_DX, SV_HOLE_DY = 49.5, 10.0
SV_FLANGE_Z = 27.0     # 몸체 바닥 ~ 귀(플랜지) 아랫면
SV_HORN_FACE = 44.0    # 몸체 바닥 ~ 혼 바깥면

# ----------------------------------------------------------------- 도우미
def box(x0, x1, y0, y1, z0, z1):
    return M.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])

def cyl_z(x, y, z0, z1, d, seg=SEG):
    return M.cylinder(z1 - z0, d / 2, d / 2, seg).translate([x, y, z0])

def cyl_x(x0, x1, y, z, d, seg=SEG):
    return M.cylinder(x1 - x0, d / 2, d / 2, seg).rotate([0, 90, 0]).translate([x0, y, z])

def cyl_y(y0, y1, x, z, d, seg=SEG):
    return M.cylinder(y1 - y0, d / 2, d / 2, seg).rotate([-90, 0, 0]).translate([x, y0, z])

def union(parts):
    parts = [p for p in parts if p is not None]
    return M.batch_boolean(parts, mf.OpType.Add) if len(parts) > 1 else parts[0]

def rot_z90(m, k=1):
    return m.rotate([0, 0, 90 * k])

def to_trimesh(m):
    mesh = m.to_mesh()
    return trimesh.Trimesh(vertices=np.asarray(mesh.vert_properties)[:, :3],
                           faces=np.asarray(mesh.tri_verts), process=False)

def save(m, name, outdir):
    b = m.bounding_box()
    m = m.translate([-(b[0] + b[3]) / 2, -(b[1] + b[4]) / 2, -b[2]])     # 베드 중앙, 바닥 z=0
    bb = m.bounding_box()
    dx, dy, dz = bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2]
    fits = dz <= BED[2] and ((dx <= BED[0] and dy <= BED[1]) or (dx <= BED[1] and dy <= BED[0]))
    assert fits, f"{name}: 프린터 출력 크기 초과 ({dx:.0f}x{dy:.0f}x{dz:.0f})"
    t = to_trimesh(m)
    t.export(os.path.join(outdir, name + ".stl"))
    b = m.bounding_box()
    print(f"  {name:28s} {b[3]-b[0]:6.1f} x {b[4]-b[1]:6.1f} x {b[5]-b[2]:6.1f} mm  "
          f"부피 {m.volume()/1000:6.1f} cm3")

def grid(i):
    return -BOARD / 2 + WALL_T / 2 + PITCH * i      # 격자선 좌표

# ----------------------------------------------------------------- 미로 생성
def make_maze(seed):
    """깊이 우선 탐색 완전 미로. 가장 긴 해답 경로가 나오는 시드를 고름."""
    rnd = random.Random(seed)
    n = NCELL
    # h[i][j]: 칸 (i,j)와 (i,j+1) 사이 벽 ;  v[i][j]: (i,j)와 (i+1,j) 사이 벽
    h = [[True] * (n - 1) for _ in range(n)]
    v = [[True] * n for _ in range(n - 1)]
    seen = [[False] * n for _ in range(n)]
    stack = [(0, 0)]
    seen[0][0] = True
    while stack:
        i, j = stack[-1]
        nb = [(i + di, j + dj) for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1))
              if 0 <= i + di < n and 0 <= j + dj < n and not seen[i + di][j + dj]]
        if not nb:
            stack.pop(); continue
        a, b = rnd.choice(nb)
        if a != i: v[min(a, i)][j] = False
        else:      h[i][min(b, j)] = False
        seen[a][b] = True
        stack.append((a, b))
    return h, v

def solve(h, v):
    n = NCELL
    from collections import deque
    prev = {(0, 0): None}; q = deque([(0, 0)])
    while q:
        i, j = q.popleft()
        for di, dj in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            a, b = i + di, j + dj
            if not (0 <= a < n and 0 <= b < n) or (a, b) in prev: continue
            if di and v[min(a, i)][j]: continue
            if dj and h[i][min(b, j)]: continue
            prev[(a, b)] = (i, j); q.append((a, b))
    path, c = [], (n - 1, n - 1)
    while c: path.append(c); c = prev[c]
    return path[::-1]

# ----------------------------------------------------------------- 미로판(타일)
def maze_board(h, v):
    """300x300 전체 미로판 (출력 전 4조각으로 자름)"""
    z0, z1 = Z_FLOOR, Z_FLOOR + FLOOR_T
    zt = z1 + WALL_H
    parts = [box(-BOARD/2, BOARD/2, -BOARD/2, BOARD/2, z0, z1)]
    n = NCELL
    def seg_x(i0, i1, j):  # x 방향 벽 (격자선 y=grid(j))
        parts.append(box(grid(i0) - WALL_T/2, grid(i1) + WALL_T/2,
                         grid(j) - WALL_T/2, grid(j) + WALL_T/2, z1 - 0.01, zt))
    def seg_y(i, j0, j1):
        parts.append(box(grid(i) - WALL_T/2, grid(i) + WALL_T/2,
                         grid(j0) - WALL_T/2, grid(j1) + WALL_T/2, z1 - 0.01, zt))
    # 테두리
    seg_x(0, n, 0); seg_x(0, n, n); seg_y(0, 0, n); seg_y(n, 0, n)
    # 내부 벽
    for i in range(n):
        for j in range(n - 1):
            if h[i][j]: seg_x(i, i + 1, j + 1)
    for i in range(n - 1):
        for j in range(n):
            if v[i][j]: seg_y(i + 1, j, j + 1)
    # 격자 기둥
    for i in range(n + 1):
        for j in range(n + 1):
            parts.append(cyl_z(grid(i), grid(j), z1 - 0.01, zt, POST_D, 32))
    board = union(parts) ^ box(-BOARD/2, BOARD/2, -BOARD/2, BOARD/2, z0 - 1, zt + 1)   # 정확히 300 x 300

    # 출발: 고리 홈 / 도착: 구슬이 멈추는 오목 홈 + 고리
    cs = lambda k: grid(k) + PITCH / 2
    sx, sy = cs(0), cs(0)
    gx, gy = cs(n - 1), cs(n - 1)
    ring = cyl_z(0, 0, z1 - 0.8, z1 + 1, 20, 64) - cyl_z(0, 0, z1 - 1, z1 + 2, 16, 64)
    rs = (9**2 + 2**2) / (2 * 2)          # 지름 18, 깊이 2 오목
    dimple = M.sphere(rs, 96).translate([gx, gy, z1 - 2 + rs]) ^ cyl_z(gx, gy, z1 - 3, z1 + 0.01, 18.2, 96)
    board = board - ring.translate([sx, sy, 0]) - dimple - ring.scale([1.25, 1.25, 1]).translate([gx, gy, 0])

    # 나사 구멍 (아래에서 M3x16 → 이음판 6 + 10mm 체결)
    for (x, y) in screw_points():
        board = board - cyl_z(x, y, z0 - 1, z0 + 12, M3_PILOT, 24)
    return board

def screw_points():
    g1, g2, g3, g4 = grid(6), grid(7), grid(8), grid(9)   # 29.7, 59.4, 89.1, 118.8
    pts = []
    for sx in (-1, 1):
        for sy in (-1, 1):
            pts += [(sx * g1, sy * g1), (sx * g2, sy * g2)]          # 허브
    for k in range(4):                                               # 이음판 4개
        for (x, y) in [(g3, g1), (g3, -g1), (g4, g1), (g4, -g1)]:
            c, s = round(math.cos(k * math.pi / 2)), round(math.sin(k * math.pi / 2))
            pts.append((c * x - s * y, s * x + c * y))
    return pts

def split_tiles(board):
    tiles = []
    BIG = 400
    for sx in (-1, 1):
        for sy in (-1, 1):
            q = box(min(0, sx * BIG), max(0, sx * BIG), min(0, sy * BIG), max(0, sy * BIG), -10, 300)
            tiles.append(((sx, sy), board ^ q))
    return tiles

# ----------------------------------------------------------------- 허브(중앙판 + 위 포크)
HUB_HALF = 65.0
def hub():
    z0, z1 = Z_HUB0, Z_HUB0 + HUB_T
    p = box(-HUB_HALF, HUB_HALF, -HUB_HALF, HUB_HALF, z0, z1)
    # 무게 줄이기 구멍
    for sx in (-1, 1):
        for sy in (-1, 1):
            p = p - cyl_z(sx * 44.5, sy * 15, z0 - 1, z1 + 1, 18) - cyl_z(sx * 15, sy * 44.5, z0 - 1, z1 + 1, 18)
    # 포크 (X방향 볼트, 십자블록 양옆)
    prongs = []
    for sx in (-1, 1):
        x0, x1 = (8.5, 14.5) if sx > 0 else (-14.5, -8.5)
        prongs.append(box(x0, x1, -8, 8, Z_UP, z0 + 0.5))
        prongs.append(cyl_x(x0, x1, 0, Z_UP, 16))
        prongs.append(box(x0, x1 , -11, 11, z0 - 4, z0 + 0.5))   # 뿌리 보강
    p = p + union(prongs)
    p = p - cyl_x(-30, 30, 0, Z_UP, M4_CLEAR)
    for (x, y) in screw_points():
        if abs(x) < HUB_HALF and abs(y) < HUB_HALF:
            p = p - cyl_z(x, y, z0 - 1, z1 + 1, M3_CLEAR, 24)
            p = p - cyl_z(x, y, z0 - 1, z0 + 2.5, 6.4, 24)   # 나사머리 자리
    return p

# ----------------------------------------------------------------- 이음판 (+X 방향 기준)
def seam_plate(drive=None, x0=HUB_HALF + 1):
    """drive: None(일반) / "X" / "Y" — +X 방향 기준으로 모델링, Y용은 90° 돌려 장착"""
    z0, z1 = Z_HUB0, Z_HUB0 + HUB_T
    p = box(x0, 146, -36, 36, z0, z1)
    p = p - cyl_z(103.95, 0, z0 - 1, z1 + 1, 20)   # 가벼움 구멍
    if drive:
        zp = ATT_Z[drive] + LINK_K                # 탭 핀 높이
        R = R_ATTACH
        tab = box(R - 5, R + 5, -15, -10, zp, z0 + 0.5) + cyl_y(-15, -10, R, zp, 10, 40)
        tab = tab + box(R - 2, R + 2, -22, -15, zp + 4.2, z0 + 0.5)    # 뒤쪽 리브 (탭 너트 위로 띄움)
        tab = tab - cyl_y(-30, 0, R, zp, M3_CLEAR, 24)
        p = p + tab
    for (x, y) in screw_points():
        if x > HUB_HALF and abs(y) < 40:
            p = p - cyl_z(x, y, z0 - 1, z1 + 1, M3_CLEAR, 24)
            p = p - cyl_z(x, y, z0 - 1, z0 + 2.5, 6.4, 24)
    return p

# ----------------------------------------------------------------- 프린팅 링크 (서보X 기준 좌표)
def link_block(which):
    """탭에 Y핀(위), 링크에 X핀(아래)으로 물리는 작은 2핀 관절"""
    Z, R = ATT_Z[which], R_ATTACH
    zp = Z + LINK_K
    p1 = box(R - 8.4, R + 5, -9.6, -5.6, Z - 4.5, zp) + cyl_y(-9.6, -5.6, R, zp, 9, 40)
    lug = box(R - 8.4, R - 4.4, -9.6, 0, Z - 4.5, Z + 4.5) + cyl_x(R - 8.4, R - 4.4, 0, Z, 9, 40)
    b = p1 + lug
    b = b - cyl_y(-20, 0, R, zp, M3_CLEAR, 24) - cyl_x(R - 20, R, 0, Z, M3_CLEAR, 24)
    return b

def link_rod(which):
    """혼(아래, Y핀) ~ 링크블록(위, X핀).  8x8 막대"""
    Z, R, zb = ATT_Z[which], R_ATTACH, SERVO_Z
    r = box(R - 4, R + 4, -4, 4, zb, Z) + cyl_x(R - 4, R + 4, 0, Z, 8, 40)
    r = r + cyl_y(-5.6, 4, R, zb, 8, 40)                     # 아랫눈 (혼 쪽으로 1.6 돌출 = 와셔 역할)
    r = r - cyl_x(R - 10, R + 10, 0, Z, M3_CLEAR, 24) - cyl_y(-10, 10, R, zb, M3_CLEAR, 24)
    return r

# ----------------------------------------------------------------- 하부프레임 (허브 + 팔 4개 일체, H2S용)
def under_frame():
    arms = [rot_z90(seam_plate(kind, x0=HUB_HALF - 3), k) for k, kind in enumerate(["X", "Y", None, None])]
    return union([hub()] + arms)

# ----------------------------------------------------------------- 십자블록
def spider():
    s = box(-8, 8, -8, 8, Z_LOW - 6.5, Z_UP + 6.5)
    s = s - cyl_y(-20, 20, 0, Z_LOW, M4_CLEAR) - cyl_x(-20, 20, 0, Z_UP, M4_CLEAR)
    return s

# ----------------------------------------------------------------- 기둥(아래 포크)
PILLAR_TOP = Z_LOW - 12
def pillar():
    p = cyl_z(0, 0, BASE_T, BASE_T + 6, 50, 96)                  # 플랜지
    p = p + cyl_z(0, 0, BASE_T, PILLAR_TOP, 32, 96)
    for sy in (-1, 1):
        y0, y1 = (8.5, 14.5) if sy > 0 else (-14.5, -8.5)
        p = p + box(-8, 8, y0, y1, PILLAR_TOP - 1, Z_LOW) + cyl_y(y0, y1, 0, Z_LOW, 16)
    p = p - cyl_y(-30, 30, 0, Z_LOW, M4_CLEAR)
    p = p - cyl_z(0, 0, BASE_T - 1, PILLAR_TOP - 15, 16)          # 속 비움
    for k in range(4):
        a = math.radians(45 + 90 * k)
        x, y = 19.5 * math.cos(a), 19.5 * math.sin(a)
        p = p - cyl_z(x, y, BASE_T - 1, BASE_T + 7, M3_CLEAR, 24)
        p = p - cyl_z(x, y, BASE_T + 4, BASE_T + 40, 6.6, 24)     # 나사머리/드라이버 자리
    return p

def pillar_holes():
    out = []
    for k in range(4):
        a = math.radians(45 + 90 * k)
        out.append((19.5 * math.cos(a), 19.5 * math.sin(a)))
    return out

# ----------------------------------------------------------------- 서보 브래킷 (서보 X 위치 기준)
PL_Y0, PL_Y1 = -29.0, -23.0     # 세움판 (앞면 y=-23 = 서보 귀 아랫면)
SV_Y_BOTTOM = PL_Y1 - SV_FLANGE_Z           # -50  서보 몸체 바닥(뒤쪽 끝)
SV_BODY_X0 = SERVO_SHAFT_X - (SV_L - SV_SHAFT_OFF)    # 29.5
SV_BODY_X1 = SERVO_SHAFT_X + SV_SHAFT_OFF             # 70.2
SV_BODY_XC = (SV_BODY_X0 + SV_BODY_X1) / 2
BR_X0, BR_X1 = 21.0, 84.0
FOOT_Y0, FOOT_Y1 = -52.0, -15.0
FOOT_T = 5.0

def servo_bracket():
    zt = SERVO_Z + SV_W / 2 + 8
    b = box(BR_X0, BR_X1, PL_Y0, PL_Y1, BASE_T, zt)                         # 세움판
    b = b + box(BR_X0, BR_X1, FOOT_Y0, FOOT_Y1, BASE_T, BASE_T + FOOT_T)    # 발판
    # 서보 받침(뒤쪽) — 몸체 아랫면을 받쳐 줌
    b = b + box(SV_BODY_X0 + 2, SV_BODY_X1 - 2, FOOT_Y0, PL_Y0 + 0.1, BASE_T, SERVO_Z - SV_W / 2 - 0.6)
    # 바깥쪽 끝 보강 리브
    b = b + box(BR_X1 - 4, BR_X1, FOOT_Y0, PL_Y0 + 0.1, BASE_T, BASE_T + 18)
    # 서보 창
    b = b - box(SV_BODY_X0 - 0.5, SV_BODY_X1 + 0.5, PL_Y0 - 1, PL_Y1 + 1,
                SERVO_Z - SV_W / 2 - 0.5, SERVO_Z + SV_W / 2 + 0.5)
    # 서보 귀 고정 구멍
    for dx in (-SV_HOLE_DX / 2, SV_HOLE_DX / 2):
        for dz in (-SV_HOLE_DY / 2, SV_HOLE_DY / 2):
            b = b - cyl_y(PL_Y0 - 1, PL_Y1 + 1, SV_BODY_XC + dx, SERVO_Z + dz, M3_CLEAR, 24)
    for (x, y) in bracket_holes():
        b = b - cyl_z(x, y, BASE_T - 1, BASE_T + FOOT_T + 1, M3_CLEAR, 24)
    return b

def bracket_holes():
    return [(BR_X0 + 4.5, -19.0), (BR_X1 - 8.5, -19.0), (BR_X0 + 4.5, -47.0), (BR_X1 - 8.5, -47.0)]

def servo_dummy(horn_deg=0.0):
    """미리보기·간섭검사용 MG996R 모형 (출력 X)"""
    yb = SV_Y_BOTTOM
    body = box(SV_BODY_X0, SV_BODY_X1, yb, yb + 36.5, SERVO_Z - SV_W / 2, SERVO_Z + SV_W / 2)
    ears = box(SV_BODY_XC - 27, SV_BODY_XC + 27, PL_Y1, PL_Y1 + 2.5, SERVO_Z - SV_W / 2, SERVO_Z + SV_W / 2)
    shaft = cyl_y(yb + 36.5, yb + 41, SERVO_SHAFT_X, SERVO_Z, 6)
    horn = (cyl_y(yb + 41, yb + 44, 0, 0, 12) +
            box(0, HORN_R + 4, yb + 41, yb + 44, -3.5, 3.5))
    horn = horn.rotate([0, -horn_deg, 0]).translate([SERVO_SHAFT_X, 0, SERVO_Z])
    return body + ears + shaft + horn

# ----------------------------------------------------------------- 바닥판
def base_plate():
    b = box(-BASE / 2, BASE / 2, -BASE / 2, BASE / 2, 0, BASE_T)
    holes = list(pillar_holes())
    for k in (0, 1):   # 서보 X, 서보 Y(90° 회전)
        for (x, y) in bracket_holes():
            c, s = (1, 0) if k == 0 else (0, 1)
            holes.append((c * x - s * y, s * x + c * y))
    for (x, y) in holes:
        b = b - cyl_z(x, y, -1, BASE_T + 1, M3_PILOT, 24)
    # 전자부품 자리(-X 쪽) 케이블타이 구멍 & 고무발 자리
    for y in (-60, -30, 0, 30, 60):
        for x in (-80, -30):
            b = b - box(x - 1.5, x + 1.5, y - 4, y + 4, -1, BASE_T + 1)
    for sx in (-1, 1):
        for sy in (-1, 1):
            b = b - cyl_z(sx * 78, sy * 78, -1, 1.2, 12.5, 48)     # 고무발(Ø12) 자리
    # 테두리 모서리 둥글게 대신 모따기
    return b

# ----------------------------------------------------------------- 조종기 케이스
CT_L, CT_W, CT_H, CT_WALL = 96.0, 60.0, 30.0, 2.4
def controller_case():
    L, W, H, t = CT_L, CT_W, CT_H, CT_WALL
    c = box(-L/2, L/2, -W/2, W/2, 0, H) - box(-L/2 + t, L/2 - t, -W/2 + t, W/2 - t, 2.0, H + 1)
    # 모서리 나사 기둥
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * (L/2 - t - 3.2), sy * (W/2 - t - 3.2)
            c = c + cyl_z(x, y, 0, H, 7, 32) - cyl_z(x, y, 4, H + 1, M3_PILOT, 24)
    # MPU6050(GY-521 21x16) 정렬 틀 — 케이스 중심, 화살표 = 앞(+X)
    fx0, fx1, fy0, fy1 = -10.8 + 18, 10.8 + 18, -8.3, 8.3
    c = c + (box(fx0 - 1.6, fx1 + 1.6, fy0 - 1.6, fy1 + 1.6, 2, 4.5) - box(fx0, fx1, fy0, fy1, 1.9, 5))
    # 바닥 앞쪽 화살표 홈(방향 표시)
    arrow = M.extrude(mf.CrossSection([[(0, -7), (10, 0), (0, 7)]]), 0.8).translate([34, 0, -0.01])
    c = c - arrow
    # USB 케이블 구멍 (뒤쪽 벽) — ESP32 USB 높이에 맞춰 필요시 줄로 다듬기
    c = c - box(-L/2 - 1, -L/2 + t + 1, -7, 7, 6, 16)
    return c

def controller_lid():
    L, W, t = CT_L, CT_W, CT_WALL
    lid = box(-L/2, L/2, -W/2, W/2, 0, 2.4)
    lid = lid + (box(-L/2 + t + 0.3, L/2 - t - 0.3, -W/2 + t + 0.3, W/2 - t - 0.3, 2.4, 4.4)
                 - box(-L/2 + t + 1.9, L/2 - t - 1.9, -W/2 + t + 1.9, W/2 - t - 1.9, 2, 5))
    for sx in (-1, 1):
        for sy in (-1, 1):
            x, y = sx * (L/2 - t - 3.2), sy * (W/2 - t - 3.2)
            lid = lid - cyl_z(x, y, -1, 6, M3_CLEAR, 24) - cyl_z(x, y, 2.4, 6, 8.2, 32)
            lid = lid - cyl_z(x, y, -1, 1.2, 6.2, 24)
    lid = lid - cyl_z(-26, 0, -1, 6, 7.2, 32)       # 7mm 패널 푸시버튼 (수평 다시 맞추기)
    return lid

# ----------------------------------------------------------------- 조립체 / 운동
def rot_about(m, axis, deg, center):
    cx, cy, cz = center
    r = [0, 0, 0]; r["xyz".index(axis)] = deg
    return m.translate([-cx, -cy, -cz]).rotate(r).translate([cx, cy, cz])

def board_pose(m, a_deg, b_deg):
    """a: 아래축(Y) 회전 = 서보X,  b: 위축(X) 회전 = 서보Y"""
    m = rot_about(m, "x", b_deg, (0, 0, Z_UP))
    return rot_about(m, "y", a_deg, (0, 0, Z_LOW))

def spider_pose(m, a_deg):
    return rot_about(m, "y", a_deg, (0, 0, Z_LOW))

def _rx(t):
    c, s_ = math.cos(t), math.sin(t); return np.array([[1, 0, 0], [0, c, -s_], [0, s_, c]])
def _ry(t):
    c, s_ = math.cos(t), math.sin(t); return np.array([[c, 0, s_], [0, 1, 0], [-s_, 0, c]])
def _rz(t):
    c, s_ = math.cos(t), math.sin(t); return np.array([[c, -s_, 0], [s_, c, 0], [0, 0, 1]])
def _T(R=np.eye(3), t=(0, 0, 0)):
    M = np.eye(4); M[:3, :3] = R; M[:3, 3] = t; return M
def board_matrix(a_deg, b_deg):
    a, b = math.radians(a_deg), math.radians(b_deg)
    Mx = _T(t=(0, 0, Z_UP)) @ _T(_rx(b)) @ _T(t=(0, 0, -Z_UP))
    My = _T(t=(0, 0, Z_LOW)) @ _T(_ry(a)) @ _T(t=(0, 0, -Z_LOW))
    return My @ Mx

def link_chain(a_deg, b_deg, which):
    """링크 기구를 정확히 닫음 → (혼 각도°, 링크 행렬, 블록 행렬, 아래핀 어긋남°)
    미지수: 혼 각 f, 블록 흔들림 beta(탭 핀 둘레), 링크 꺾임 delta(블록 X핀 둘레)"""
    from scipy.optimize import least_squares
    Rs = _T(_rz(0 if which == "X" else math.pi / 2))
    Mb = np.linalg.inv(Rs) @ board_matrix(a_deg, b_deg) @ Rs
    Z, R = ATT_Z[which], R_ATTACH
    pinP, K0 = np.array([R, 0, Z + LINK_K]), np.array([R, 0, Z])
    def mats(beta, delta):
        Mk = Mb @ _T(t=pinP) @ _T(_ry(beta)) @ _T(t=-pinP)
        Mr = Mk @ _T(t=K0) @ _T(_rx(delta)) @ _T(t=-K0)
        return Mk, Mr
    def tip(f):
        return np.array([SERVO_SHAFT_X + HORN_R * math.cos(f), 0, SERVO_Z + HORN_R * math.sin(f)])
    def res(x):
        f, beta, delta = x
        _, Mr = mats(beta, delta)
        return (Mr @ np.array([R, 0, SERVO_Z, 1.0]))[:3] - tip(f)
    sol = least_squares(res, [0.0, 0.0, 0.0], xtol=1e-12, ftol=1e-12)
    f, beta, delta = sol.x
    Mk, Mr = mats(beta, delta)
    yax = Mr[:3, :3] @ np.array([0, 1.0, 0])                     # 링크 아랫핀 방향 vs 혼 축(Y)
    mis = math.degrees(math.acos(min(1.0, abs(yax[1]))))
    return math.degrees(f), Rs @ Mr, Rs @ Mk, mis

def apply(m, M):
    return m.transform(M[:3, :])

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "stl")
    os.makedirs(out, exist_ok=True)
    # 해답 경로가 긴 미로를 고름
    best = max(range(MAZE_SEED, MAZE_SEED + 40), key=lambda s: len(solve(*make_maze(s))))
    h, v = make_maze(best)
    print(f"미로 시드 {best}, 최단 경로 {len(solve(h, v))}칸")
    board = maze_board(h, v)
    save(board, "01_미로판_300x300", out)                             # 통짜, 바닥면이 베드
    save(under_frame().rotate([180, 0, 0]), "02_하부프레임", out)      # 뒤집어서(회전) 윗면이 베드, 포크·탭이 위
    save(spider(), "03_십자블록", out)
    save(pillar(), "04_기둥_아래포크", out)
    save(servo_bracket(), "05_서보브래킷_x2", out)
    save(base_plate(), "06_바닥판", out)
    save(controller_case(), "07_조종기_케이스", out)
    save(controller_lid(), "08_조종기_뚜껑", out)                      # 바깥면이 베드, 턱이 위
    # 링크: 옆으로 눕혀(x면이 베드) 출력 / 링크블록: 넓은 판이 베드 (X·Y 공용 모양)
    save(link_rod("X").rotate([0, -90, 0]), "09a_링크_X_(긴것)", out)
    save(link_rod("Y").rotate([0, -90, 0]), "09b_링크_Y_(짧은것)", out)
    save(link_block("X").rotate([90, 0, 0]), "10_링크블록_x2", out)
