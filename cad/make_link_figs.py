"""링크 연결부 설명 그림용 부품 GLB (서보 X 쪽, 판 수평 상태, 실제 설계 형상 + 볼트·너트)
python3 make_link_figs.py <out.glb>"""
import sys, math, numpy as np, trimesh, manifold3d as mf
import tilt_maze_cad as T

M = mf.Manifold
Z = T.ATT_Z["X"]              # 98.5  링크 윗핀(링크블록 ↔ 링크) 높이
ZP = Z + T.LINK_K             # 105.5 탭 핀(탭 ↔ 링크블록) 높이
R = T.R_ATTACH                # 80

def bolt(L):
    """머리 받침면이 원점, 나사부가 -Z 방향 (M3 육각렌치 볼트)"""
    head = M.cylinder(3.0, 2.75, 2.75, 48) - M.cylinder(2.2, 1.3, 1.3, 6).translate([0, 0, 1.0])
    shank = M.cylinder(L, 1.5, 1.5, 32).translate([0, 0, -L])
    return head + shank

def nut(nylon):
    """두께 4: 쇠 부분 2.5 + 나일론 링 1.5. 받침면이 원점, +Z 쪽으로 쌓임"""
    if nylon:
        return M.cylinder(1.5, 3.1, 3.1, 6).translate([0, 0, 2.5]) - M.cylinder(5, 1.5, 1.5, 24).translate([0, 0, -1])
    return M.cylinder(2.5, 3.175, 3.175, 6) - M.cylinder(5, 1.5, 1.5, 24).translate([0, 0, -1])

def place(m, axis, origin):
    """로컬 +Z → axis 방향으로 돌려 origin에 놓기"""
    a = np.array(axis, float); a /= np.linalg.norm(a)
    z = np.array([0, 0, 1.0])
    v = np.cross(z, a); c = float(z @ a)
    if np.linalg.norm(v) < 1e-9:
        Rm = np.eye(3) if c > 0 else np.diag([1, -1, -1])
    else:
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        Rm = np.eye(3) + K + K @ K * (1 / (1 + c))
    return m.transform(np.hstack([Rm, np.array(origin, float).reshape(3, 1)]))

parts = {}
# 하부프레임 중 +X 팔 끝(탭 주변)만 잘라서
frame = T.under_frame() ^ T.box(55, 105, -45, 25, ZP - 12, T.Z_HUB0 + T.HUB_T + 1)
parts["frame"] = (frame, [240, 140, 50])
parts["block"] = (T.link_block("X"), [200, 70, 160])
parts["rod"] = (T.link_rod("X"), [250, 200, 60])
servo = T.servo_dummy(0)
yb = T.SV_Y_BOTTOM
horn = (T.cyl_y(yb + 41, yb + 44, T.SERVO_SHAFT_X, T.SERVO_Z, 12) +
        T.box(T.SERVO_SHAFT_X, T.SERVO_SHAFT_X + T.HORN_R + 4, yb + 41, yb + 44, T.SERVO_Z - 3.5, T.SERVO_Z + 3.5)) \
       - T.cyl_y(yb + 40, yb + 45, R, T.SERVO_Z, 3.2)
parts["servo"] = (servo - T.box(-500, 500, yb + 41, 500, -500, 500), [45, 50, 62])
parts["horn"] = (horn, [245, 245, 245])
parts["bracket"] = (T.servo_bracket(), [60, 120, 220])

STEEL, NYLON = [150, 155, 162], [235, 235, 225]
# ① 탭 ↔ 링크블록 : M3×16, 머리는 링크블록 쪽(+Y), 너트는 탭 바깥(-Y)
parts["bolt1"] = (place(bolt(16), [0, 1, 0], [R, -5.6, ZP]), STEEL)
parts["nut1"] = (place(nut(False), [0, -1, 0], [R, -15.0, ZP]), STEEL)
parts["nyl1"] = (place(nut(True), [0, -1, 0], [R, -15.0, ZP]), NYLON)
# ② 링크블록 귀 ↔ 링크 윗구멍 : M3×20, 머리는 귀 바깥(-X), 너트는 링크 너머(+X)
parts["bolt2"] = (place(bolt(20), [-1, 0, 0], [R - 8.4, 0, Z]), STEEL)
parts["nut2"] = (place(nut(False), [1, 0, 0], [R + 4.0, 0, Z]), STEEL)
parts["nyl2"] = (place(nut(True), [1, 0, 0], [R + 4.0, 0, Z]), NYLON)
# ③ 링크 아랫구멍 ↔ 서보 혼 : M3×20, 머리는 링크 쪽(+Y), 너트는 혼 뒤(-Y)
parts["bolt3"] = (place(bolt(20), [0, 1, 0], [R, 4.0, T.SERVO_Z]), STEEL)
parts["nut3"] = (place(nut(False), [0, -1, 0], [R, yb + 41, T.SERVO_Z]), STEEL)
parts["nyl3"] = (place(nut(True), [0, -1, 0], [R, yb + 41, T.SERVO_Z]), NYLON)

# 볼트가 부품 몸통을 뚫고 지나가지 않는지(구멍 안으로만 지나가는지) 확인
solid = {"frame": frame, "block": parts["block"][0], "rod": parts["rod"][0], "horn": horn, "servo": parts["servo"][0]}
for b in ("bolt1", "nut1", "bolt2", "nut2", "bolt3", "nut3"):
    hits = {k: round((parts[b][0] ^ s).volume(), 2) for k, s in solid.items()}
    hits = {k: v for k, v in hits.items() if v > 0.05}
    print(f"{b}: 부품과 겹침 {hits if hits else '없음'}")

scene = trimesh.Scene()
for name, (m, col) in parts.items():
    tm = T.to_trimesh(m)
    tm.visual = trimesh.visual.ColorVisuals(tm, face_colors=np.tile(col + [255], (len(tm.faces), 1)))
    scene.add_geometry(tm, node_name=name, geom_name=name)
scene.export(sys.argv[1])
print("saved", sys.argv[1])
