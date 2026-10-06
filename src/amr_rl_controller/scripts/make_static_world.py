#!/usr/bin/env python3
"""
움직이는 장애물을 뺀 창고 월드 사본을 만든다 (보너스 1 A2 Gazebo 스모크 — 정적 맵).

    make_static_world.py <warehouse.sdf> <출력 디렉터리>    → <출력>/warehouse.sdf

사람 actor 전부(<actor>…</actor>)와 움직이는 물리 모델 include 두 개(forklift_main, shuttle_amr)를
지운다. 정적 구조물(벽·랙·기둥·표지·마커)은 그대로이고 월드 이름 'warehouse' 도 그대로다 — 스폰이
월드 이름으로 create 를 부르고 amr_simulation warehouse.launch.py 는 파일 이름(확장자 제외)을 월드
이름으로 쓴다. 원본은 고치지 않는다. 지운 개수를 출력하고, 예상(actor 6, 모델 2)과 다르면 실패한다.
"""

import os
import re
import sys

MOVING_MODELS = ('forklift_main', 'shuttle_amr')
EXPECT_ACTORS = 6


def strip_moving(sdf):
    """SDF 문자열 → (정적 사본, 지운 actor 수, 지운 모델 이름 목록)."""
    out, n_actor = re.subn(r'\n[ \t]*<actor name="[^"]*">.*?</actor>', '', sdf, flags=re.S)
    removed = []
    for name in MOVING_MODELS:
        pat = (r'\n[ \t]*<include>(?:(?!</include>).)*?<name>' + re.escape(name)
               + r'</name>.*?</include>')
        out, n = re.subn(pat, '', out, flags=re.S)
        if n == 1:
            removed.append(name)
    return out, n_actor, removed


def main():
    """사본을 쓴다."""
    if len(sys.argv) != 3:
        sys.exit('사용: make_static_world.py <warehouse.sdf> <출력 디렉터리>')
    src, out_dir = sys.argv[1], sys.argv[2]
    with open(src, encoding='utf-8') as f:
        sdf = f.read()
    out, n_actor, removed = strip_moving(sdf)
    if n_actor != EXPECT_ACTORS or sorted(removed) != sorted(MOVING_MODELS):
        sys.exit(f'예상과 다르다: actor {n_actor}, 모델 {removed}')
    if '<actor' in out or any(f'<name>{m}</name>' in out for m in MOVING_MODELS):
        sys.exit('지운 뒤에도 움직이는 장애물이 남았다')
    os.makedirs(out_dir, exist_ok=True)
    dst = os.path.join(out_dir, 'warehouse.sdf')
    with open(dst, 'w', encoding='utf-8') as f:
        f.write(out)
    print(f'{dst}: actor {n_actor} 개, 모델 {removed} 를 뺐다 ({len(sdf)} → {len(out)} 바이트)')


if __name__ == '__main__':
    main()
