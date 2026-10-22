"""기울기 극한에서 부품 간섭 + 서보 혼 각도 확인 (프린팅 링크 버전)"""
import itertools
import tilt_maze_cad as T

h, v = T.make_maze(12)
plates = [T.seam_plate("X"), T.rot_z90(T.seam_plate("Y")),
          T.rot_z90(T.seam_plate(None), 2), T.rot_z90(T.seam_plate(None), 3)]
board_grp = T.union([T.maze_board(h, v), T.under_frame()])
spider, pillar, base = T.spider(), T.pillar(), T.base_plate()
brk = {"X": T.servo_bracket(), "Y": T.rot_z90(T.servo_bracket())}
rod = {w: T.link_rod(w) for w in "XY"}
blk = {w: T.link_block(w) for w in "XY"}

def vol(a, b):
    return (a ^ b).volume()

ANG = [-10, -5, 0, 5, 10]
worst = 0
for a, b in itertools.product(ANG, ANG):
    Mb = T.board_matrix(a, b)
    bg = T.apply(board_grp, Mb)
    sp = T.spider_pose(spider, a)
    parts, horns, misal = {}, {}, {}
    for w in "XY":
        f, Mr, Mk, mis = T.link_chain(a, b, w)
        horns[w] = f; misal[w] = mis
        sv = T.servo_dummy(f)
        parts["서보" + w] = sv if w == "X" else T.rot_z90(sv)
        parts["링크" + w] = T.apply(rod[w] if w == "X" else rod[w], Mr)
        parts["블록" + w] = T.apply(blk[w], Mk)
    static = {"기둥": pillar, "바닥": base, "브래킷X": brk["X"], "브래킷Y": brk["Y"],
              "서보X": parts["서보X"], "서보Y": parts["서보Y"]}
    checks = {f"판-{n}": vol(bg, s) for n, s in static.items()}
    checks["판-십자"] = vol(bg, sp)
    checks["십자-기둥"] = vol(sp, pillar)
    for w in "XY":
        for n, s in static.items():
            checks[f"링크{w}-{n}"] = vol(parts["링크" + w], s)
            checks[f"블록{w}-{n}"] = vol(parts["블록" + w], s)
        checks[f"링크{w}-판"] = vol(parts["링크" + w], bg)
        checks[f"블록{w}-판"] = vol(parts["블록" + w], bg)
        checks[f"링크{w}-블록{w}"] = vol(parts["링크" + w], parts["블록" + w])
    checks["링크X-링크Y"] = vol(parts["링크X"], parts["링크Y"])
    bad = {k: round(x, 2) for k, x in checks.items() if x > 0.05}
    worst = max(worst, abs(horns["X"]), abs(horns["Y"]))
    worst_mis = max(globals().get("worst_mis", 0), misal["X"], misal["Y"])
    print(f"a={a:+3d} b={b:+3d}  혼 X {horns['X']:+6.1f}°  Y {horns['Y']:+6.1f}°  아랫핀 어긋남 {max(misal.values()):.1f}°  간섭: {bad if bad else '없음'}")
print("최대 혼 각도", round(worst, 1), " 최대 아랫핀 어긋남", round(worst_mis, 2), "°")
