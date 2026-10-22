"""출력용 STL을 '실제로 뒤집어 놓듯' 되돌려서 조립 설계 위치의 부품과 완전히 겹치는지 확인
(거울 반전·방향 오류·부품 혼동을 잡아냄)   python3 verify_stl.py"""
import os, numpy as np, trimesh, manifold3d as mf
import tilt_maze_cad as T
import controller_amoled_case as C

STL = os.path.join(os.path.dirname(__file__), "..", "stl")
h, v = T.make_maze(12)
# 파일 → (조립 위치의 설계 형상, 출력 방향으로 돌릴 때 쓴 회전[deg])
checks = {
    "01_미로판_300x300": (T.maze_board(h, v), (0, 0, 0)),
    "02_하부프레임": (T.under_frame(), (180, 0, 0)),
    "03_십자블록": (T.spider(), (0, 0, 0)),
    "04_기둥_아래포크": (T.pillar(), (0, 0, 0)),
    "05_서보브래킷_x2": (T.servo_bracket(), (0, 0, 0)),
    "06_바닥판": (T.base_plate(), (0, 0, 0)),
    "07_조종기_케이스": (T.controller_case(), (0, 0, 0)),
    "08_조종기_뚜껑": (T.controller_lid(), (0, 0, 0)),
    "09a_링크_X_(긴것)": (T.link_rod("X"), (0, -90, 0)),
    "09b_링크_Y_(짧은것)": (T.link_rod("Y"), (0, -90, 0)),
    "10_링크블록_x2": (T.link_block("X"), (90, 0, 0)),
    "11_조종기_손잡이": (C.housing(), (180, 0, 0)),
    "12_조종기_베젤": (C.bezel(), (180, 0, 0)),
    "13_배터리_뚜껑": (C.battery_lid(), (0, 0, 0)),
    "14_맞춤확인링": (C.fit_ring(), (180, 0, 0)),
}

def to_manifold(tm):
    mesh = mf.Mesh(vert_properties=np.asarray(tm.vertices, dtype=np.float32), tri_verts=np.asarray(tm.faces, dtype=np.uint32))
    return mf.Manifold(mesh)

bad = 0
for name, (design, rot) in checks.items():
    tm = trimesh.load(os.path.join(STL, name + ".stl"))
    tm.merge_vertices()
    m = to_manifold(tm)
    # 출력 방향 회전을 되돌림 (회전만 — 거울 반전은 되돌릴 수 없으므로 여기서 드러남)
    rx, ry, rz = rot
    if rx or ry or rz:
        R = trimesh.transformations.euler_matrix(*np.radians([rx, ry, rz]), axes="sxyz")
        m = m.transform(np.linalg.inv(R)[:3, :])
    a, b = m.bounding_box(), design.bounding_box()
    m = m.translate([b[0] - a[0], b[1] - a[1], b[2] - a[2]])
    diff = (m - design).volume() + (design - m).volume()
    ok = diff < 0.002 * design.volume() + 1
    bad += not ok
    print(f"{'일치' if ok else '불일치'}  {name:26s} 차이 {diff:8.1f} mm³ / 부피 {design.volume():9.0f} mm³")
print("모두 일치" if not bad else f"{bad}개 불일치")
