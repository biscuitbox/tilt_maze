"""시뮬레이터용 난이도별 미로 + 구멍 함정 데이터 → ../preview/levels.json
어려움(10x10)은 실제 출력 미로판과 같은 미로 (maze.json).
함정 구멍 [x, y, r] (mm, 판 중심 기준):
  ① 지나침 함정: 경로가 꺾이는 칸에서 '곧장 가면' 들어가는 막다른 칸 — 너무 빨리 오면 빠짐
  ② 벽쪽 함정: 통로가 넓은 판(쉬움·보통)에서 경로 칸의 벽 쪽 — 반대편으로 붙어 지나가야 함
  ③ 막다른 길 함정: 경로 밖 칸"""
import json, os, math, random
import tilt_maze_cad as T
d = os.path.dirname(os.path.abspath(__file__))
real = json.load(open(os.path.join(d, '..', 'preview', 'maze.json')))
BALL_R = 8
#          key       이름     칸  시드  구멍 수  구멍 반지름  벽쪽 함정 최소 통로(구슬 중심이 다닐 폭)
LEVELS = [('easy',   '쉬움',   6, 101, 4, 11.0, 12.0),
          ('normal', '보통',   8, 201, 6, 9.5, 7.0),
          ('hard',   '어려움', 10, None, 8, 9.5, None),
          ('expert', '고수',  12, 301, 11, 9.0, None)]
DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))

def is_open(m, a, b):
    (i, j), (k, l) = a, b
    if not (0 <= k < m['n'] and 0 <= l < m['n']): return False
    if i != k: return not m['v'][min(i, k)][j]
    return not m['h'][i][min(j, l)]

def make_holes(m, count, rh, lane, seed):
    n, P, wall = m['n'], m['pitch'], m['wall']
    g = lambda k: -m['board'] / 2 + wall / 2 + P * k + P / 2      # 칸 중심
    half = (P - wall) / 2                                          # 칸 안쪽 반폭
    a = half - BALL_R                                              # 구슬 중심이 다닐 수 있는 반폭
    path = [tuple(c) for c in m['solution']]
    on_path = {c: k for k, c in enumerate(path)}
    banned = set(path[:2] + path[-2:])                             # 출발·도착 근처는 비움
    cands = []
    # ① 지나침 함정
    run = 0
    for k in range(1, len(path) - 1):
        d1 = (path[k][0] - path[k - 1][0], path[k][1] - path[k - 1][1])
        d2 = (path[k + 1][0] - path[k][0], path[k + 1][1] - path[k][1])
        run = run + 1 if k > 1 and d1 == (path[k - 1][0] - path[k - 2][0], path[k - 1][1] - path[k - 2][1]) else 1
        if d1 == d2: continue
        c = (path[k][0] + d1[0], path[k][1] + d1[1])
        if c in on_path or c in banned or not is_open(m, path[k], c): continue
        # 꺾이는 칸 쪽으로 붙인 구멍: 구슬이 그 칸에 1/3쯤 들어오면 빠짐
        off = max(0.0, min(half - rh - 0.6, P * 0.12))
        cands.append((10 + run, 'over', c, (g(c[0]) - d1[0] * off, g(c[1]) - d1[1] * off)))
    # ② 벽쪽 함정 (경로 칸, 닫힌 벽 쪽에 붙임 → 남은 통로 폭 확인)
    if lane:
        for k in range(2, len(path) - 2):
            c = path[k]
            closed = [dd for dd in DIRS if not is_open(m, c, (c[0] + dd[0], c[1] + dd[1]))]
            opens = [dd for dd in DIRS if dd not in closed]
            off = half - rh - 0.6
            if len(closed) == 2 and closed[0][0] != closed[1][0] * -1 or len(closed) == 2 and closed[0][1] != -closed[1][1]:
                pass
            for dd in closed:
                # 그 벽 쪽에 붙인 구멍. 구슬 중심이 지나갈 통로 = 반대쪽 끝(-a) ~ 구멍 가장자리(off - rh)
                free = a + (off - rh)
                # 꺾이는 칸이면 구멍이 들어오는 길을 막지 않는지: 열린 쪽 방향과 수직인 벽만
                if any(o == (-dd[0], -dd[1]) for o in opens) and len(opens) == 2 and opens[0][0] == -opens[1][0] and opens[0][1] == -opens[1][1]:
                    pass
                if free >= lane:
                    cands.append((5 + free / 10, 'side', c, (g(c[0]) + dd[0] * off, g(c[1]) + dd[1] * off)))
    # ③ 막다른 길·경로 밖 칸
    rnd = random.Random(seed)
    for i in range(n):
        for j in range(n):
            c = (i, j)
            if c in on_path or c in banned: continue
            deg = sum(is_open(m, c, (i + dd[0], j + dd[1])) for dd in DIRS)
            beside = any(is_open(m, c, (i + dd[0], j + dd[1])) and (i + dd[0], j + dd[1]) in on_path for dd in DIRS)
            # 경로에서 바로 갈라지는 입구 칸이 가장 효과적인 함정 (옆길로 새면 빠짐)
            cands.append((2 + (1.5 if beside else 0) + (0.5 if deg == 1 else 0) + rnd.random(), 'dead', c, (g(i), g(j))))
    cands.sort(key=lambda t: -t[0])
    holes, used = [], set()
    want = {'over': math.ceil(count * 0.5), 'side': math.ceil(count * 0.35) if lane else 0}
    have = {'over': 0, 'side': 0, 'dead': 0}
    def take(kind_ok):
        for sc, kind, c, (x, y) in cands:
            if len(holes) >= count: return
            if not kind_ok(kind) or c in used: continue
            if any(max(abs(c[0] - u[0]), abs(c[1] - u[1])) <= 1 for u in used): continue   # 이웃 칸끼리는 띄움
            if kind in want and have[kind] >= want[kind]: continue
            used.add(c); have[kind] += 1
            holes.append([round(x, 2), round(y, 2), rh, kind])
    take(lambda k: k == 'over'); take(lambda k: k == 'side'); take(lambda k: True)
    if len(holes) < count:      # 띄우기 조건을 풀고 채움
        for sc, kind, c, (x, y) in cands:
            if len(holes) >= count: break
            if c in used: continue
            used.add(c); holes.append([round(x, 2), round(y, 2), rh, kind])
    return holes

out = []
for key, name, n, seed0, count, rh, lane in LEVELS:
    if seed0 is None:
        m = dict(real)
    else:
        T.NCELL = n
        best = max(range(seed0, seed0 + 60), key=lambda s: len(T.solve(*T.make_maze(s))))
        h, v = T.make_maze(best)
        sol = T.solve(h, v)
        m = {'n': n, 'pitch': round((T.BOARD - T.WALL_T) / n, 4), 'wall': T.WALL_T, 'post': T.POST_D, 'board': T.BOARD,
             'h': [[int(x) for x in r] for r in h], 'v': [[int(x) for x in r] for r in v], 'solution': [list(c) for c in sol]}
    m['key'] = key; m['name'] = name
    m['holes'] = make_holes(m, count, rh, lane, (seed0 or 7) + 5)
    kinds = {}
    for hh in m['holes']: kinds[hh[3]] = kinds.get(hh[3], 0) + 1
    out.append(m)
    print(f"{name} {n}x{n}  통로 {m['pitch'] - m['wall']:.1f} mm  경로 {len(m['solution'])}칸  구멍 {len(m['holes'])}개 Ø{2*rh:.0f} {kinds}")
json.dump(out, open(os.path.join(d, '..', 'preview', 'levels.json'), 'w'), separators=(',', ':'))
