#!/usr/bin/env python3
"""
ArUco 마커 텍스처 생성기 (DICT_4X4_50, id 0~41).

    python3 gen_aruco_textures.py            # 같은 디렉토리의 materials/textures/ 에 다시 쓴다
    python3 gen_aruco_textures.py <out_dir>

생성물(PNG)을 커밋하므로 빌드 시 실행할 필요는 없다. 개발 컨테이너의 OpenCV(cv2.aruco, ArucoDetector)가 필요하다.
마커 120 px(6 셀 x 20 px, 테두리 1 셀 포함) + 흰 여백 40 px = 200 x 200 px, 8 bit 흑백, 파일당 약 0.3 KB.
판(0.30 m)에 매핑되면 마커 흑백 영역 한 변 = 0.18 m. 흰 여백(quiet zone)이 있어야 벽/스테이션 앞에서도 검출된다.
id 배정: 0~3 = Dock-1/2/A/B, 4~6 = 충전 스테이션 C1~C3, 7~9 = 예비,
10~41 = 기둥 8개 × 네 면 위치 표지 (통로가 6 m 주기라 LiDAR 만으로는 구별 못 하는 순간 이동의
전역 기준점 — worlds/gen_warehouse_world.py 의 PILLAR_MARKER_ID0 / pillar_markers)
(worlds/gen_warehouse_world.py 의 DOCKS/CHARGERS).
"""
import os
import sys

import cv2
import numpy as np

CELL, MARGIN, N_IDS = 20, 40, 42


def main():
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "materials", "textures")
    os.makedirs(out, exist_ok=True)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    det = cv2.aruco.ArucoDetector(d, cv2.aruco.DetectorParameters())
    for mid in range(N_IDS):
        m = cv2.aruco.generateImageMarker(d, mid, 6 * CELL, borderBits=1)
        img = np.full((6 * CELL + 2 * MARGIN,) * 2, 255, np.uint8)
        img[MARGIN:MARGIN + 6 * CELL, MARGIN:MARGIN + 6 * CELL] = m
        path = os.path.join(out, f"aruco_4x4_50_{mid}.png")
        cv2.imwrite(path, img, [cv2.IMWRITE_PNG_COMPRESSION, 9])
        # 자기 검출로 id 확인
        _, ids, _ = det.detectMarkers(cv2.imread(path))
        assert ids is not None and int(ids.flatten()[0]) == mid, (mid, ids)
        print(f"{path}: {os.path.getsize(path)} bytes, self-detect id {mid} ok")


if __name__ == "__main__":
    main()
