"""스마트폰 조종 안내판: 'QR을 찍으면 스마트폰으로 조종이 가능합니다.' + QR (https://biscuitbox.github.io/tilt_maze/)
 - 16a 안내판: 150 x 60 x 3 mm 판 + 0.8 mm 솟은 글자·QR (판은 흰색, 글자·QR은 검은색으로 출력)
 - 16b 안내판 받침: 바닥판 -X 가장자리에 케이블타이 1개로 고정 (바닥판의 기존 구멍 (-80,30)(-80,60) 사용).
   받침의 홈에 안내판을 위에서 끼우면 바닥판 바깥쪽(-X)을 보고 섭니다.
python3 qr_plaque.py [출력폴더]"""
import os, sys, segno
from fontTools.ttLib import TTCollection
from fontTools.pens.basePen import BasePen
import manifold3d as mf
import tilt_maze_cad as T

M, box, union = T.M, T.box, T.union
URL = "https://biscuitbox.github.io/tilt_maze/"
LINES = ["QR을 찍으면", "스마트폰으로", "조종이 가능합니다."]
FONT = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"

PW, PH, PT = 150.0, 60.0, 3.0      # 안내판 가로·세로·두께
BLANK = 12.0                        # 아래쪽 받침 홈에 들어가는 빈 띠
MARK = 0.8                          # 글자·QR 높이 (0.2 mm 층 4장)
MOD = 1.25                          # QR 한 칸 (mm)
QUIET = 4                           # QR 둘레 빈칸 수
FS = 10.0                           # 글자 크기 (em, mm)

# ---------------- 글자 → 다각형 ----------------
class FlatPen(BasePen):
    def __init__(self, gs, steps=8):
        super().__init__(gs); self.contours, self.cur, self.steps = [], [], steps
    def _moveTo(self, p): self.cur = [p]
    def _lineTo(self, p): self.cur.append(p)
    def _curveToOne(self, p1, p2, p3):
        p0 = self.cur[-1]
        for i in range(1, self.steps + 1):
            t = i / self.steps; u = 1 - t
            self.cur.append((u**3 * p0[0] + 3 * u * u * t * p1[0] + 3 * u * t * t * p2[0] + t**3 * p3[0],
                             u**3 * p0[1] + 3 * u * u * t * p1[1] + 3 * u * t * t * p2[1] + t**3 * p3[1]))
    def _qCurveToOne(self, p1, p2):
        p0 = self.cur[-1]
        for i in range(1, self.steps + 1):
            t = i / self.steps; u = 1 - t
            self.cur.append((u * u * p0[0] + 2 * u * t * p1[0] + t * t * p2[0], u * u * p0[1] + 2 * u * t * p1[1] + t * t * p2[1]))
    def _closePath(self):
        if len(self.cur) > 2: self.contours.append(self.cur)
        self.cur = []
    _endPath = _closePath

def load_font():
    for f in TTCollection(FONT).fonts:
        if "KR" in f["name"].getDebugName(1): return f
    return TTCollection(FONT).fonts[0]

def text_shape(font, s, size):
    """한 줄 글자 → CrossSection (왼쪽 아래 기준선 0), 너비 반환"""
    gs, cmap, upm = font.getGlyphSet(), font.getBestCmap(), font["head"].unitsPerEm
    k = size / upm; x = 0.0; polys = []
    for ch in s:
        g = cmap.get(ord(ch))
        if g is None: x += size * 0.5; continue
        pen = FlatPen(gs); gs[g].draw(pen)
        for c in pen.contours:
            polys.append([(x + px * k, py * k) for (px, py) in c])
        x += gs[g].width * k
    cs = mf.CrossSection(polys, mf.FillRule.NonZero) if polys else mf.CrossSection()
    return cs, x

def qr_marks(x0, y0):
    q = segno.make(URL, error="m", micro=False)
    rows = [list(r) for r in q.matrix]            # 위 → 아래
    n = len(rows); parts = []
    for r, row in enumerate(rows):
        c = 0
        while c < n:
            if row[c]:
                c0 = c
                while c < n and row[c]: c += 1
                yy = y0 + (n - 1 - r) * MOD
                parts.append(box(x0 + c0 * MOD - 0.01, x0 + c * MOD + 0.01, yy - 0.01, yy + MOD + 0.01, PT - 0.01, PT + MARK))
            else:
                c += 1
    return union(parts), n

def plaque():
    """판 위(z=PT)에 글자·QR이 솟음. 출력 그대로 (판이 베드, 글자가 위)"""
    plate = box(0, PW, 0, PH, 0, PT)
    # 모서리 둥글게 (반지름 3)
    for (cx, cy) in ((0, 0), (PW, 0), (0, PH), (PW, PH)):
        sx, sy = (1 if cx == 0 else -1), (1 if cy == 0 else -1)
        corner = box(min(cx, cx + 3 * sx), max(cx, cx + 3 * sx), min(cy, cy + 3 * sy), max(cy, cy + 3 * sy), -1, PT + 1)
        plate = plate - (corner - T.cyl_z(cx + 3 * sx, cy + 3 * sy, -2, PT + 2, 6, 32))
    # QR (오른쪽, 빈칸 포함 정사각형이 글자 영역 높이에 맞음)
    n = len(segno.make(URL, error="m", micro=False).matrix)
    side = (n + 2 * QUIET) * MOD
    qx = PW - 4 - side + QUIET * MOD
    qy = BLANK + (PH - BLANK - side) / 2 + QUIET * MOD
    qr, _ = qr_marks(qx, qy)
    # 글자 (왼쪽 3줄)
    font = load_font()
    area_w = PW - 4 - side - 6
    lines = []
    for s in LINES:
        cs, w = text_shape(font, s, FS)
        lines.append((cs, w))
    wmax = max(w for _, w in lines)
    scale = min(1.0, area_w / wmax)
    lh = FS * 1.32 * scale
    block = lh * (len(lines) - 1) + FS * 0.88 * scale
    top = BLANK + (PH - BLANK + block) / 2 - FS * 0.88 * scale
    marks = []
    for i, (cs, w) in enumerate(lines):
        y = top - i * lh
        t = M.extrude(cs.scale([scale, scale]).translate([6, y]), MARK + 0.01).translate([0, 0, PT - 0.01])
        marks.append(t)
    text = union(marks)
    # 아래 빈 띠 표시선(끼우는 깊이)
    return plate, qr + text, side, scale

# ---------------- 받침 (바닥판 좌표) ----------------
Z0 = T.BASE_T
SLOT_X0, SLOT_X1 = -93.6, -90.2          # 안내판이 서는 홈 (두께 3 + 여유 0.4), 바닥판 가장자리 바로 바깥
Y0, Y1 = 18.0, 72.0
FLOOR = 3.0
TIES = [(-80, 30), (-80, 60)]

def stand():
    zf = Z0 + FLOOR
    foot = box(-88, -66, Y0, Y1, Z0, zf)
    for (x, y) in TIES:
        foot = foot - box(x - 1.9, x + 1.9, y - 4.4, y + 4.4, Z0 - 1, zf + 1)
    floor = box(SLOT_X0 - 2.4, -88, Y0, Y1, Z0, zf)                      # 홈 바닥 (바닥판 밖으로 나감)
    back = box(SLOT_X1, SLOT_X1 + 3.6, Y0, Y1, Z0, zf + 30)              # 안쪽 벽 (안내판 뒤를 받침)
    front = box(SLOT_X0 - 2.4, SLOT_X0, Y0, Y1, Z0, zf + BLANK - 1)       # 바깥 턱 (빈 띠만 가림)
    gus = [box(SLOT_X1 + 3.6, -70, y - 1.5, y + 1.5, zf, zf + 1) for y in (Y0 + 1.5, Y1 - 1.5)]
    # 안쪽 벽 보강 (삼각 리브 대신 계단)
    for k, h in enumerate((22, 14, 7)):
        gus += [box(SLOT_X1 + 3.6, SLOT_X1 + 3.6 + 4 * (k + 1), y - 1.5, y + 1.5, zf, zf + h) for y in (Y0 + 1.5, Y1 - 1.5)]
    return union([foot, floor, back, front] + gus)

def plaque_in_place(plate_and_marks):
    """안내판을 받침 홈에 끼운 위치로 (판 앞면이 -X를 봄)"""
    m = plate_and_marks
    # 판 좌표: x=가로, y=세로, z=두께(앞면 z=PT) → 바닥판 좌표: 가로=+Y 방향(바깥에서 볼 때 왼→오), 세로=+Z, 앞면=-X
    # 바깥(-X)에서 보면 오른쪽이 -Y 이므로 가로 x → -Y
    import numpy as np
    R = np.array([[0, 0, -1], [-1, 0, 0], [0, 1, 0]], float)          # (x,y,z) → (-z, -x, y)
    yc = (Y0 + Y1) / 2
    t = np.array([SLOT_X1 - 0.2, yc + PW / 2, Z0 + FLOOR], float)
    return m.transform(np.hstack([R, t.reshape(3, 1)]))

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "stl")
    plate, marks, side, sc = plaque()
    print(f"QR 한 변 {side:.1f} mm (빈칸 포함), 글자 배율 {sc:.2f}")
    T.save(plate + marks, "16a_QR안내판", out)
    # 두 색 출력용(AMS): 같은 위치 기준으로 판·글자를 따로
    import trimesh, numpy as np
    for m, name in ((plate, "16a_QR안내판_판(흰색)"), (marks, "16a_QR안내판_글자QR(검은색)")):
        T.to_trimesh(m).export(os.path.join(out, name + ".stl"))
    sc3 = trimesh.Scene()
    for m, name, col in ((plate, "plate_white", [245, 245, 240, 255]), (marks, "marks_black", [20, 20, 20, 255])):
        tm = T.to_trimesh(m); tm.visual = trimesh.visual.ColorVisuals(tm, face_colors=np.tile(col, (len(tm.faces), 1)))
        sc3.add_geometry(tm, node_name=name, geom_name=name)
    sc3.export(os.path.join(out, "16a_QR안내판_두색.3mf"))
    T.save(stand(), "16b_QR안내판_받침", out)
