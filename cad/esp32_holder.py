"""미로 본체 ESP32 DevKit 홀더 (핀이 아래로 난 보드 + 점퍼선 직결용)
 - 바닥판 -X 쪽 전자부품 자리에 케이블타이 2개로 고정 (바닥판의 기존 구멍 (-80,-60)(-80,-30)(-30,-60)(-30,-30) 사용)
 - 보드를 가운데 받침 위 28 mm 높이에 올려, 아래로 난 핀에 점퍼선을 꽂을 공간을 둠
 - USB는 -Y 쪽(바닥판 가장자리)으로, 양 끝 턱이 보드 모서리를 눌러 잡음. 안테나 쪽 벽이 휘면서 딸깍 끼워짐
python3 esp32_holder.py [출력폴더]   →  15a_ESP32홀더_38핀.stl, 15b_ESP32홀더_30핀.stl"""
import os, sys
import tilt_maze_cad as T

box, union = T.box, T.union
Z0 = T.BASE_T              # 바닥판 윗면
CX = -55.0                 # 보드 가운데 x (바닥판 좌표)
USB_Y = -84.0              # 보드 USB 쪽 끝 y
FOOT_T = 3.0
RISE = 28.0                # 바닥판 윗면 → 보드 아랫면
PCB_T = 1.6
IN_W = 29.0                # 보드 폭 여유 (25.5~28.4 mm 보드)
WALL = 2.2                 # 끝 벽 두께
LIP = 1.2                  # 보드 위로 걸리는 턱 길이 (보드 끝 1.2 mm만 눌러 핀 끝에 닿지 않게)
TIE = [(-80, -60), (-80, -30), (-30, -60), (-30, -30)]   # 바닥판 케이블타이 구멍 (3 x 8 mm)

VARIANTS = {"38핀": 54.4, "30핀": 51.5}   # 보드 길이 (mm) — 내 보드를 재서 다르면 여기 숫자를 바꾸고 다시 실행

def holder(pcb_len):
    L = pcb_len + 0.6                         # 길이 여유
    y0, y1 = USB_Y, USB_Y + L                 # 보드가 놓이는 구간
    zb = Z0 + RISE                            # 보드 아랫면
    zt = zb + PCB_T + 0.4                     # 턱 아랫면
    x0, x1 = CX - IN_W / 2, CX + IN_W / 2
    parts = []
    # 발판 (케이블타이 구멍 4개)
    foot = box(-86, -25, y0 - WALL - 1, y1 + WALL + 1, Z0, Z0 + FOOT_T)
    for (x, y) in TIE:
        foot = foot - box(x - 1.9, x + 1.9, y - 4.4, y + 4.4, Z0 - 1, Z0 + FOOT_T + 1)
    # 발판 가볍게: 타이 지나가는 줄과 받침 사이를 비움
    foot = foot - box(-74, x0 - 1, y0 + 6, y1 - 6, Z0 - 1, Z0 + FOOT_T + 1)
    foot = foot - box(x1 + 1, -36, y0 + 6, y1 - 6, Z0 - 1, Z0 + FOOT_T + 1)
    parts.append(foot)
    # 가운데 받침 (핀 줄 사이, 폭 12) + 아래로 선이 지나갈 창
    rib = box(CX - 6, CX + 6, y0 + 3, y1 - 3, Z0, zb)
    rib = rib - box(CX - 7, CX + 7, y0 + 10, y1 - 10, Z0 + FOOT_T + 4, zb - 5)
    parts.append(rib)
    # 받침 위 보드 닿는 면 양옆 작은 턱(보드가 옆으로 미끄러지지 않게 — 핀 줄 안쪽)
    # USB 쪽 벽: 가운데 USB 플러그 자리를 크게 비움
    usb_wall = box(x0 - WALL, x1 + WALL, y0 - WALL, y0, Z0, zt + 2.2)
    usb_wall = usb_wall - box(CX - 8, CX + 8, y0 - WALL - 1, y0 + 1, zb - 4, zt + 5)
    parts.append(usb_wall)
    parts.append(box(x0 - WALL, CX - 8, y0 - 0.01, y0 + LIP, zt, zt + 2.2))       # 턱
    parts.append(box(CX + 8, x1 + WALL, y0 - 0.01, y0 + LIP, zt, zt + 2.2))
    # 안테나 쪽 벽: 얇아서 살짝 휨, 턱 위쪽은 비스듬히(끼울 때 밀려남)
    ant = box(x0 - WALL, x1 + WALL, y1, y1 + WALL, Z0 + FOOT_T + 6, zt + 2.2)
    ant = ant + box(x0 - WALL, x1 + WALL, y1, y1 + WALL + 1.5, Z0, Z0 + FOOT_T + 6)   # 뿌리는 두껍게
    ant = ant - box(CX - 10, CX + 10, y1 - 1, y1 + WALL + 2, zb - 1, zt + 5)        # ESP32 모듈(안테나)이 끝으로 튀어나와도 닿지 않게
    parts.append(ant)
    lip = T.M.extrude(T.mf.CrossSection([[(0, 0), (LIP, 0), (0, 2.2)]]), x1 - x0 + 2 * WALL)   # 단면: y-z 삼각형
    lip = lip.rotate([90, 0, 90]).translate([x0 - WALL, 0, 0])                 # x 방향으로 늘이고
    lip = lip.mirror([0, 1, 0]).translate([0, y1 + 0.01, zt])                  # 벽에서 보드 쪽(-y)으로
    lip = lip - box(CX - 10, CX + 10, y1 - 5, y1 + 5, zt - 1, zt + 5)           # 양쪽 모서리만 남김
    parts.append(lip)
    # 벽과 발판 이음 보강
    for xx in (x0 - WALL, x1):
        parts.append(box(xx, xx + WALL, y0 - WALL, y0 + 4, Z0, zb - 4))
    return union(parts)

def esp32_dummy(pcb_len):
    """보기용 보드 + 아래로 난 핀 + 점퍼선 소켓 (간섭 확인용)"""
    zb = Z0 + RISE
    y0, y1 = USB_Y + 0.3, USB_Y + 0.3 + pcb_len
    b = box(CX - 14, CX + 14, y0, y1, zb, zb + PCB_T)
    b = b + box(CX - 9, CX + 9, y1 - 25, y1 + 1.5, zb + PCB_T, zb + PCB_T + 3.2)  # 모듈 (안테나가 보드 끝보다 1.5 mm 튀어나온 경우)
    b = b + box(CX - 4, CX + 4, y0 - 1, y0 + 5, zb + PCB_T, zb + PCB_T + 3)      # USB
    for sx in (-1, 1):
        x = CX + sx * 12.7
        b = b + box(x - 1.27, x + 1.27, y0 + 4, y1 - 8, zb - 2.5, zb)            # 핀 헤더
        b = b + box(x - 0.5, x + 0.5, y0 + 4, y1 - 8, zb + PCB_T, zb + PCB_T + 1.2)   # 위로 나온 핀 끝·납땜
        b = b + box(x - 1.27, x + 1.27, y0 + 4, y1 - 8, zb - 17, zb - 2.5)       # 점퍼선 소켓
    return b

if __name__ == "__main__":
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.path.dirname(__file__), "..", "stl")
    for tag, (name, Lp) in zip("ab", VARIANTS.items()):
        T.save(holder(Lp), f"15{tag}_ESP32홀더_{name}", out)
