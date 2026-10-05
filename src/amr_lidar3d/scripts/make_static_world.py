#!/usr/bin/env python3
"""
동적 행위자를 뺀 월드 사본 (센서 오차 측정용 — 정답 면이 없는 움직이는 물체를 없앤다).

    make_static_world.py <원본 warehouse.sdf> <출력 디렉터리>   → <출력 디렉터리>/<같은 파일 이름>

<actor> 전부와 --exclude 모델(기본 forklift, shuttle_amr)의 <include> 를 지운다. 파일 이름(= <world name>)은
그대로 둔다 — warehouse.launch.py 가 절대 경로를 받고 파일 이름으로 월드 이름을 정한다.
그 모델을 움직이던 월드 플러그인(TriggeredPublisher 등)은 남는다 — 대상이 없어 아무 일도 하지 않는다.
"""

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


def strip_world(src: str, exclude) -> ET.ElementTree:
    tree = ET.parse(src)
    world = tree.getroot().find('world')
    removed = []
    for actor in world.findall('actor'):
        world.remove(actor)
        removed.append(actor.get('name'))
    for inc in world.findall('include'):
        uri = inc.find('uri').text.strip().replace('model://', '')
        if uri in exclude:
            world.remove(inc)
            name = inc.find('name')
            removed.append(name.text if name is not None else uri)
    tree.removed = removed
    return tree


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('src')
    ap.add_argument('out_dir')
    ap.add_argument('--exclude', action='append', default=['forklift', 'shuttle_amr'])
    ns = ap.parse_args(argv)
    tree = strip_world(ns.src, set(ns.exclude))
    out = Path(ns.out_dir) / Path(ns.src).name
    out.parent.mkdir(parents=True, exist_ok=True)
    tree.write(out, encoding='utf-8', xml_declaration=True)
    print(f'{out}: 제거 {len(tree.removed)} — {", ".join(tree.removed)}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
