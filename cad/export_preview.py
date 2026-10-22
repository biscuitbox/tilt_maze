"""가이드 페이지 3D 미리보기용 GLB (조립 위치, Z-up, mm)"""
import sys, numpy as np, trimesh
import tilt_maze_cad as T

h, v = T.make_maze(12)
board = T.maze_board(h, v)
parts = {}
parts["tile_board"] = (board, [236, 232, 222])                 # 01 미로판 (출력물과 같은 통짜)
parts["plate_frame"] = (T.under_frame(), [240, 140, 50])        # 02 하부프레임 (출력물과 같은 일체형)
parts["spider"] = (T.spider(), [120, 120, 130])
parts["pillar"] = (T.pillar(), [150, 155, 165])
parts["base"] = (T.base_plate(), [70, 75, 85])
parts["bracket_x"] = (T.servo_bracket(), [60, 120, 220])
parts["bracket_y"] = (T.rot_z90(T.servo_bracket()), [60, 120, 220])

def servo_body():
    s = T.servo_dummy(0)
    horn_only = T.servo_dummy(0) - (T.box(-500, 500, -500, T.SV_Y_BOTTOM + 41, -500, 500))
    return s - horn_only

def horn_local():
    yb = T.SV_Y_BOTTOM
    return (T.cyl_y(yb + 41, yb + 44, 0, 0, 12) + T.box(0, T.HORN_R + 4, yb + 41, yb + 44, -3.5, 3.5)
            - T.cyl_y(yb + 40, yb + 45, T.HORN_R, 0, 3))

parts["servo_x"] = (servo_body(), [35, 40, 55])
parts["servo_y"] = (T.rot_z90(servo_body()), [35, 40, 55])
parts["horn"] = (horn_local(), [250, 250, 250])          # 축 기준 로컬 (JS에서 배치)
for w in "XY":   # 서보X 좌표계 기준 로컬 (JS에서 기구 풀어 배치)
    parts[f"rod_{w}"] = (T.link_rod(w), [250, 200, 60])
    parts[f"blk_{w}"] = (T.link_block(w), [200, 70, 160])
parts["ctrl_case"] = (T.controller_case(), [90, 190, 140])
parts["ctrl_lid"] = (T.controller_lid().mirror([0, 0, 1]).translate([0, 0, T.CT_H + 2.4]), [90, 190, 140])

scene = trimesh.Scene()
for name, (m, col) in parts.items():
    tm = T.to_trimesh(m)
    tm.visual = trimesh.visual.ColorVisuals(tm, face_colors=np.tile(col + [255], (len(tm.faces), 1)))
    scene.add_geometry(tm, node_name=name, geom_name=name)
out = sys.argv[1]
scene.export(out)
import os; print(out, os.path.getsize(out) // 1024, "KB")

