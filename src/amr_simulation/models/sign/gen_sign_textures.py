#!/usr/bin/env python3
"""
물류센터 표지판 텍스처 생성기 (구역·도크·충전·주의·비상구 판, 명세 4.6 "표지판" 인식 대상).

    python3 gen_sign_textures.py            # 같은 디렉토리의 materials/textures/ 에 다시 쓴다
    python3 gen_sign_textures.py <out_dir>

생성물(PNG)을 커밋하므로 빌드 시 실행할 필요는 없다. 개발 컨테이너의 OpenCV(cv2) 가 필요하다.
문구는 영문 대문자(Hershey 글꼴), 그림 문자(번개·경고 삼각형·달리는 사람·화살표)는 다각형으로 그린다.
판의 실제 크기(가로 x 세로 m)는 SIGNS 의 size 이고, 텍스처는 가로 512 px 기준으로 같은 비율이다.
월드 생성기(worlds/gen_warehouse_world.py SIGN_POSES)가 이름으로 텍스처를 붙인다 (판 법선 +x 가 보는 쪽).
"""
import os
import sys

import cv2
import numpy as np

# 색 (BGR)
BLUE = (160, 80, 20)
ORANGE = (20, 110, 225)
GREEN = (60, 140, 20)
YELLOW = (20, 205, 245)
LIGHT_BLUE = (200, 160, 60)
WHITE = (255, 255, 255)
BLACK = (20, 20, 20)
FONT = cv2.FONT_HERSHEY_DUPLEX

# 이름: (크기 [가로 m, 세로 m], 배경, 글자색, 문구 줄 [(문자열, 줄 높이 비율)], 그림 문자)
SIGNS = {
    "dock_1": ((0.90, 0.45), BLUE, WHITE, [("DOCK 1", 0.42), ("INBOUND", 0.20)], None),
    "dock_2": ((0.90, 0.45), BLUE, WHITE, [("DOCK 2", 0.42), ("INBOUND", 0.20)], None),
    "dock_a": ((0.90, 0.45), ORANGE, WHITE, [("DOCK A", 0.42), ("OUTBOUND", 0.20)], None),
    "dock_b": ((0.90, 0.45), ORANGE, WHITE, [("DOCK B", 0.42), ("OUTBOUND", 0.20)], None),
    "zone_inbound": ((1.60, 0.40), BLUE, WHITE, [("INBOUND ZONE", 0.45)], "arrow_down"),
    "zone_outbound": ((1.60, 0.40), ORANGE, WHITE, [("OUTBOUND ZONE", 0.45)], "arrow_down"),
    "charging": ((1.60, 0.50), GREEN, WHITE, [("CHARGING", 0.36), ("ZONE", 0.24)], "bolt"),
    "c1": ((0.30, 0.20), GREEN, WHITE, [("C1", 0.60)], None),
    "c2": ((0.30, 0.20), GREEN, WHITE, [("C2", 0.60)], None),
    "c3": ((0.30, 0.20), GREEN, WHITE, [("C3", 0.60)], None),
    "waiting": ((1.60, 0.50), LIGHT_BLUE, WHITE, [("AMR WAITING", 0.34), ("AREA", 0.24)], None),
    "zone_a": ((0.80, 0.60), WHITE, BLUE, [("A", 0.54), ("ZONE", 0.16)], None),
    "zone_b": ((0.80, 0.60), WHITE, BLUE, [("B", 0.54), ("ZONE", 0.16)], None),
    "zone_c": ((0.80, 0.60), WHITE, BLUE, [("C", 0.54), ("ZONE", 0.16)], None),
    "caution_forklift": ((0.60, 0.60), YELLOW, BLACK, [("CAUTION", 0.14), ("FORKLIFTS", 0.11)],
                         "warning"),
    "caution_narrow": ((0.60, 0.60), YELLOW, BLACK, [("CAUTION", 0.14), ("NARROW AISLE", 0.09)],
                       "warning"),
    "exit": ((0.80, 0.30), GREEN, WHITE, [("EXIT", 0.55)], "runner"),
}
WIDTH_PX = 512


def _text_block(img, lines, color, x0, x1, y0, y1):
    """문구 줄을 [x0, x1] x [y0, y1] 에 세로로 쌓아 가운데 정렬한다 (줄 높이 = 비율 × 판 높이)."""
    h = img.shape[0]
    heights = [int(r * h) for _, r in lines]
    gap = max(4, (y1 - y0 - sum(heights)) // (len(lines) + 1))
    y = y0 + gap
    for (text, _), lh in zip(lines, heights):
        scale = cv2.getFontScaleFromHeight(FONT, lh, max(1, lh // 9))
        thick = max(2, lh // 9)
        (tw, th), _ = cv2.getTextSize(text, FONT, scale, thick)
        if tw > (x1 - x0) * 0.94:                      # 가로가 넘치면 줄인다
            scale *= (x1 - x0) * 0.94 / tw
            (tw, th), _ = cv2.getTextSize(text, FONT, scale, thick)
        org = (x0 + (x1 - x0 - tw) // 2, y + th)
        cv2.putText(img, text, org, FONT, scale, color, thick, cv2.LINE_AA)
        y += lh + gap


def _pictogram(img, kind, fg, x0, x1, y0, y1):
    w, h = x1 - x0, y1 - y0
    cx, cy = (x0 + x1) // 2, (y0 + y1) // 2
    if kind == "bolt":        # 번개 (노랑)
        s = min(w, h) * 0.45
        pts = np.array([(0.15, -1.0), (-0.45, 0.10), (-0.02, 0.10), (-0.20, 1.0), (0.50, -0.20),
                        (0.06, -0.20), (0.30, -1.0)]) * s + (cx, cy)
        cv2.fillPoly(img, [pts.astype(np.int32)], YELLOW, cv2.LINE_AA)
    elif kind == "warning":   # 경고 삼각형 + 느낌표
        s = min(w, h) * 0.48
        tri = np.array([(0, -1.0), (0.95, 0.72), (-0.95, 0.72)]) * s + (cx, cy)
        cv2.fillPoly(img, [tri.astype(np.int32)], BLACK, cv2.LINE_AA)
        inner = np.array([(0, -0.62), (0.63, 0.52), (-0.63, 0.52)]) * s + (cx, cy)
        cv2.fillPoly(img, [inner.astype(np.int32)], YELLOW, cv2.LINE_AA)
        cv2.rectangle(img, (int(cx - 0.07 * s), int(cy - 0.35 * s)),
                      (int(cx + 0.07 * s), int(cy + 0.15 * s)), BLACK, -1)
        cv2.circle(img, (cx, int(cy + 0.33 * s)), int(0.08 * s), BLACK, -1, cv2.LINE_AA)
    elif kind == "runner":    # 달리는 사람 + 문 (ISO 7010 E001 느낌)
        s = min(w, h) * 0.42
        cv2.rectangle(img, (int(cx + 0.25 * s), int(cy - s)), (int(cx + 1.0 * s), int(cy + s)),
                      fg, 3)
        head = (int(cx - 0.05 * s), int(cy - 0.78 * s))
        cv2.circle(img, head, int(0.17 * s), fg, -1, cv2.LINE_AA)
        lw = max(3, int(0.16 * s))
        limbs = [((-0.15, -0.55), (-0.40, 0.10)), ((-0.40, 0.10), (-0.05, 0.45)),
                 ((-0.05, 0.45), (-0.10, 0.95)), ((-0.40, 0.10), (-0.80, 0.55)),
                 ((-0.20, -0.45), (0.15, -0.20)), ((-0.20, -0.45), (-0.65, -0.30))]
        for a, b in limbs:
            p = (int(cx + a[0] * s), int(cy + a[1] * s))
            q = (int(cx + b[0] * s), int(cy + b[1] * s))
            cv2.line(img, p, q, fg, lw, cv2.LINE_AA)
    elif kind == "arrow_down":
        s = min(w, h) * 0.40
        pts = np.array([(-0.35, -1.0), (0.35, -1.0), (0.35, 0.0), (0.8, 0.0), (0.0, 1.0),
                        (-0.8, 0.0), (-0.35, 0.0)]) * s + (cx, cy)
        cv2.fillPoly(img, [pts.astype(np.int32)], (255, 255, 255), cv2.LINE_AA)


def render(name):
    (wm, hm), bg, fg, lines, picto = SIGNS[name]
    w = WIDTH_PX
    h = int(round(w * hm / wm))
    img = np.full((h, w, 3), bg, np.uint8)
    border = max(4, h // 28)
    cv2.rectangle(img, (border, border), (w - border - 1, h - border - 1), fg, max(2, border // 2))
    if picto is None:
        _text_block(img, lines, fg, 2 * border, w - 2 * border, border, h - border)
    elif picto == "warning":       # 위 그림 + 아래 글
        _pictogram(img, picto, fg, 2 * border, w - 2 * border, border, int(h * 0.62))
        _text_block(img, lines, fg, 2 * border, w - 2 * border, int(h * 0.60), h - border)
    else:                          # 왼쪽 그림 + 오른쪽 글
        split = 2 * border + int(h * 0.9)
        _pictogram(img, picto, fg, 2 * border, split, border, h - border)
        _text_block(img, lines, fg, split, w - 2 * border, border, h - border)
    return img


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "materials", "textures")
    os.makedirs(out, exist_ok=True)
    for name in SIGNS:
        path = os.path.join(out, f"sign_{name}.png")
        cv2.imwrite(path, render(name), [cv2.IMWRITE_PNG_COMPRESSION, 9])
        print(f"{path}: {os.path.getsize(path)} bytes")


if __name__ == "__main__":
    main()
