#!/usr/bin/env python3
"""
합성 창고 지도(PGM + YAML)와 무작위 출발지-목적지 쌍 파일을 만든다.

    ros2 run amr_navigation make_warehouse_map.py --out /tmp/wh/warehouse [--pairs 50 --seed 42]

출력: <out>.pgm, <out>.yaml (map_server 형식), <out>_pairs.csv (sx,sy,syaw,gx,gy,gyaw).
"""
import argparse
import csv
import sys

from amr_navigation import warehouse_map


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.strip().splitlines()[0])
    ap.add_argument('--out', required=True, help='출력 경로 접두어 (확장자 제외)')
    ap.add_argument('--resolution', type=float, default=0.05)
    ap.add_argument('--pairs', type=int, default=50)
    ap.add_argument('--seed', type=int, default=42)
    ap.add_argument('--min-clearance', type=float, default=0.45)
    ap.add_argument('--min-separation', type=float, default=5.0)
    args = ap.parse_args(argv)
    grid = warehouse_map.build_warehouse(args.resolution)
    yml = warehouse_map.write_map(grid, args.out)
    pairs = warehouse_map.sample_pairs(grid, args.pairs, args.seed, args.min_clearance,
                                       args.min_separation)
    with open(args.out + '_pairs.csv', 'w', newline='', encoding='utf-8') as f:
        w = csv.writer(f)
        w.writerow(['sx', 'sy', 'syaw', 'gx', 'gy', 'gyaw'])
        w.writerows(pairs)
    print(f'map {yml} ({grid.width}x{grid.height} @ {grid.resolution} m), '
          f'{len(pairs)} pairs -> {args.out}_pairs.csv')
    return 0


if __name__ == '__main__':
    sys.exit(main())
