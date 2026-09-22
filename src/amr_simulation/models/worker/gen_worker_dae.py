#!/usr/bin/env python3
"""
작업자(actor) 스킨용 COLLADA 메시 생성기 — 사람 비율의 매끈한 저폴리 인체 + 형광 안전조끼.

    python3 gen_worker_dae.py                # 같은 디렉토리의 meshes/worker.dae 를 다시 쓴다
    python3 gen_worker_dae.py out.dae

Gazebo Fortress 의 actor 는 스켈레톤이 있는 스킨 메시가 필수라서, 외부 대용량 메시(walk.dae ~MB, 이미지/Fuel 캐시에
없음) 대신 텍스트 DAE 를 직접 만든다 (단일 관절 + 정지 애니메이션 → 몸통이 미끄러지듯 이동, 자세는 걷는 중간 자세로 고정).
생성물(worker.dae)을 커밋하므로 빌드 시 실행할 필요는 없다. 의존성: 표준 라이브러리만.

형상 (결정, 신장 1.75 m 성인 비율 — 머리 1 : 신장 7.6)
  타원체 머리·안전모, 원기둥+반구(캡슐) 팔다리, 타원 단면 몸통. 정점 법선을 매끈하게 줘서 음영이 사람처럼 나온다.
  이전 박스 6개 모델은 사전학습 YOLOv8n/s(COCO)가 사람으로 전혀 검출하지 못했다 (리뷰 실측).
색 (결정)
  안전조끼 = 형광 황록색(EN ISO 20471 형광 황색 계열) + 은색 반사띠. 골판지 상자(갈색·황갈색), 랙 기둥(주황),
  기둥 안전띠·지게차(노랑)와 색상각이 30도 이상 떨어져 색만으로도 화물과 구분된다. 약한 자체발광으로 형광을 흉내 낸다.
  작업복 남색, 안전화 검정, 안전모 흰색, 피부.
좌표: +x = 진행 방향(얼굴), +z = 위, 원점 = 두 발 사이 바닥.
"""
import math
import os
import sys

# ---- 재질 (diffuse RGB, emission RGB) -------------------------------------------
MATERIALS = {
    "skin": ((0.80, 0.60, 0.48), (0.0, 0.0, 0.0)),
    "hair": ((0.12, 0.09, 0.07), (0.0, 0.0, 0.0)),
    "hardhat": ((0.93, 0.93, 0.90), (0.0, 0.0, 0.0)),
    "vest": ((0.72, 1.00, 0.10), (0.10, 0.14, 0.0)),
    "stripe": ((0.78, 0.78, 0.80), (0.05, 0.05, 0.05)),
    "shirt": ((0.12, 0.17, 0.34), (0.0, 0.0, 0.0)),
    "trousers": ((0.14, 0.15, 0.22), (0.0, 0.0, 0.0)),
    "shoes": ((0.05, 0.05, 0.05), (0.0, 0.0, 0.0)),
}

N_SEG = 12          # 원주 분할
N_RING = 4          # 반구/타원체 위도 분할 (파일 크기와 매끈함의 절충)
# 주의: ign-common ColladaLoader 는 VERTEX/NORMAL 입력이 같은 offset 을 공유하면 메시를 그리지 못한다 (실측: 보이지 않음)
#       → offset 0/1 로 인덱스를 두 번 쓴다.


def _norm(v):
    n = math.sqrt(sum(c * c for c in v)) or 1.0
    return tuple(c / n for c in v)


def _cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def _frame(axis):
    """단위벡터 w(axis 방향)와 수직인 u, v."""
    w = _norm(axis)
    ref = (0.0, 0.0, 1.0) if abs(w[2]) < 0.9 else (1.0, 0.0, 0.0)
    u = _norm(_cross(ref, w))
    v = _cross(w, u)
    return u, v, w


def ellipsoid(c, r, zcut=None):
    """중심 c, 반지름 (rx, ry, rz). zcut 이 있으면 c.z + zcut 아래를 자른 윗부분(안전모)."""
    verts, norms, tris = [], [], []
    lat0 = -math.pi / 2 if zcut is None else math.asin(max(-1.0, min(1.0, zcut / r[2])))
    rings = N_RING * 2
    for i in range(rings + 1):
        lat = lat0 + (math.pi / 2 - lat0) * i / rings
        for j in range(N_SEG + 1):
            lon = 2 * math.pi * j / N_SEG
            d = (math.cos(lat) * math.cos(lon), math.cos(lat) * math.sin(lon), math.sin(lat))
            verts.append((c[0] + r[0] * d[0], c[1] + r[1] * d[1], c[2] + r[2] * d[2]))
            norms.append(_norm((d[0] / r[0], d[1] / r[1], d[2] / r[2])))
    for i in range(rings):
        for j in range(N_SEG):
            a = i * (N_SEG + 1) + j
            b = a + N_SEG + 1
            tris += [(a, a + 1, b + 1), (a, b + 1, b)]
    if zcut is not None:           # 아래 뚜껑
        ctr = len(verts)
        verts.append((c[0], c[1], c[2] + zcut))
        norms.append((0.0, 0.0, -1.0))
        for j in range(N_SEG):
            tris.append((ctr, j + 1, j))
    return verts, norms, tris


def capsule(p0, p1, r0, r1):
    """p0→p1 축의 원뿔대 + 양 끝 반구 (팔다리)."""
    u, v, w = _frame(tuple(b - a for a, b in zip(p0, p1)))
    verts, norms, tris = [], [], []
    # 링 목록: (중심, 반지름, 축방향 성분 각) — p0 반구, 원통, p1 반구
    rings = []
    for i in range(N_RING, 0, -1):
        ang = -math.pi / 2 * i / N_RING
        rings.append((p0, r0, ang))
    rings.append((p0, r0, 0.0))
    rings.append((p1, r1, 0.0))
    for i in range(1, N_RING + 1):
        ang = math.pi / 2 * i / N_RING
        rings.append((p1, r1, ang))
    for ctr, r, ang in rings:
        for j in range(N_SEG + 1):
            lon = 2 * math.pi * j / N_SEG
            radial = tuple(math.cos(lon) * a + math.sin(lon) * b for a, b in zip(u, v))
            n = tuple(math.cos(ang) * rc + math.sin(ang) * wc for rc, wc in zip(radial, w))
            verts.append(tuple(cc + r * nc for cc, nc in zip(ctr, n)))
            norms.append(_norm(n))
    for i in range(len(rings) - 1):
        for j in range(N_SEG):
            a = i * (N_SEG + 1) + j
            b = a + N_SEG + 1
            tris += [(a, b, b + 1), (a, b + 1, a + 1)]
    return verts, norms, tris


def elliptic_tube(sections, closed_top=True, closed_bottom=True):
    """타원 단면 관(몸통/조끼/반사띠) — z 가 증가하는 (z, cx, rx, ry) 단면 목록으로 만든다."""
    verts, norms, tris = [], [], []
    for k, (z, cx, rx, ry) in enumerate(sections):
        dz = (sections[min(k + 1, len(sections) - 1)][0] - sections[max(k - 1, 0)][0]) or 1.0
        drx = (sections[min(k + 1, len(sections) - 1)][2] - sections[max(k - 1, 0)][2]) / dz
        for j in range(N_SEG + 1):
            lon = 2 * math.pi * j / N_SEG
            c, s = math.cos(lon), math.sin(lon)
            verts.append((cx + rx * c, ry * s, z))
            norms.append(_norm((c / rx, s / ry, -drx)))
    for k in range(len(sections) - 1):
        for j in range(N_SEG):
            a = k * (N_SEG + 1) + j
            b = a + N_SEG + 1
            tris += [(a, a + 1, b + 1), (a, b + 1, b)]
    for cap, k, nz in ((closed_bottom, 0, -1.0), (closed_top, len(sections) - 1, 1.0)):
        if not cap:
            continue
        z, cx = sections[k][0], sections[k][1]
        ctr = len(verts)
        verts.append((cx, 0.0, z))
        norms.append((0.0, 0.0, nz))
        base = k * (N_SEG + 1)
        for j in range(N_SEG):
            v = len(verts)
            verts += [verts[base + j], verts[base + j + 1]]
            norms += [(0.0, 0.0, nz), (0.0, 0.0, nz)]
            tris.append((ctr, v, v + 1) if nz > 0 else (ctr, v + 1, v))
    return verts, norms, tris


def _limb_pose(root, length_a, length_b, swing, bend, splay):
    """관절 체인 좌표: root 에서 swing(앞뒤, rad) 으로 length_a, 다시 bend 만큼 더 굽혀 length_b."""
    a = (root[0] + length_a * math.sin(swing), root[1] + splay,
         root[2] - length_a * math.cos(swing))
    ang = swing + bend
    b = (a[0] + length_b * math.sin(ang), a[1] + splay * 0.3, a[2] - length_b * math.cos(ang))
    return a, b


def build_parts():
    """(재질, (verts, norms, tris)) 목록 — 걷는 중간 자세(보폭 ±14도)."""
    parts = []
    stride = math.radians(14)
    # 다리: 엉덩이 관절 z 0.92, 허벅지 0.43, 정강이 0.42 → 발목 z ≈ 0.08
    for side, sw in ((1, stride), (-1, -stride)):
        hip = (0.0, 0.095 * side, 0.92)
        knee, ankle = _limb_pose(hip, 0.43, 0.42, sw, -0.10 if sw > 0 else 0.20, 0.005 * side)
        parts.append(("trousers", capsule(hip, knee, 0.078, 0.060)))
        shin_end = (ankle[0], ankle[1], ankle[2] + 0.02)
        parts.append(("trousers", capsule(knee, shin_end, 0.058, 0.045)))
        # 안전화: 발목 아래 앞으로 긴 타원체
        parts.append(("shoes", ellipsoid((ankle[0] + 0.05, ankle[1], max(0.045, ankle[2] - 0.03)),
                                         (0.13, 0.055, 0.05))))
    # 몸통 (작업복): 골반 ~ 어깨. 단면 (z, x중심, 반폭 x(앞뒤), 반폭 y(좌우))
    torso = [(0.86, 0.0, 0.105, 0.165), (0.98, 0.0, 0.100, 0.155), (1.12, 0.005, 0.110, 0.165),
             (1.28, 0.01, 0.120, 0.185), (1.40, 0.0, 0.105, 0.190), (1.46, 0.0, 0.075, 0.120)]
    parts.append(("shirt", elliptic_tube(torso)))
    # 형광 조끼: 몸통보다 1.2 cm 두껍게, 허리 ~ 어깨
    vest = [(0.96, 0.0, 0.114, 0.169), (1.12, 0.005, 0.124, 0.179), (1.28, 0.01, 0.134, 0.199),
            (1.40, 0.0, 0.118, 0.202), (1.45, 0.0, 0.090, 0.150)]
    parts.append(("vest", elliptic_tube(vest, closed_bottom=False)))
    # 반사띠 2줄 (가슴·허리)
    for zc, cx, rx, ry in ((1.26, 0.01, 0.137, 0.201), (1.04, 0.002, 0.121, 0.176)):
        band = [(zc - 0.025, cx, rx, ry), (zc + 0.025, cx, rx, ry)]
        parts.append(("stripe", elliptic_tube(band, closed_top=False, closed_bottom=False)))
    # 팔: 어깨 z 1.42, 위팔 0.29 (소매), 아래팔 0.26 (소매 끝 ~ 손목 피부), 손
    for side, sw in ((1, -stride * 1.1), (-1, stride * 1.1)):
        sh = (0.0, 0.215 * side, 1.415)
        elbow, wrist = _limb_pose(sh, 0.29, 0.26, sw, 0.25, 0.02 * side)
        parts.append(("shirt", ellipsoid(sh, (0.060, 0.055, 0.050))))
        parts.append(("shirt", capsule(sh, elbow, 0.052, 0.044)))
        cuff = tuple(e + 0.6 * (w - e) for e, w in zip(elbow, wrist))
        parts.append(("shirt", capsule(elbow, cuff, 0.044, 0.040)))
        parts.append(("skin", capsule(cuff, wrist, 0.035, 0.032)))
        forearm = _norm(tuple(b - a for a, b in zip(elbow, wrist)))
        hand = tuple(w + 0.07 * d for w, d in zip(wrist, forearm))
        parts.append(("skin", ellipsoid(hand, (0.035, 0.028, 0.06))))
    # 목, 머리, 머리카락(뒤통수), 안전모(+챙)
    parts.append(("skin", capsule((0.0, 0.0, 1.45), (0.01, 0.0, 1.55), 0.052, 0.050)))
    parts.append(("skin", ellipsoid((0.015, 0.0, 1.645), (0.100, 0.082, 0.118))))
    parts.append(("hair", ellipsoid((-0.012, 0.0, 1.665), (0.092, 0.084, 0.100))))
    parts.append(("hardhat", ellipsoid((0.01, 0.0, 1.690), (0.118, 0.100, 0.100), zcut=0.0)))
    parts.append(("hardhat", ellipsoid((0.05, 0.0, 1.692), (0.145, 0.108, 0.012))))
    return parts


def build(parts):
    """재질별 삼각형 묶음을 가진 단일 스킨 메시 COLLADA 문자열."""
    verts, norms = [], []
    groups = {m: [] for m in MATERIALS}
    for mat, (v, n, t) in parts:
        off = len(verts)
        verts += v
        norms += n
        groups[mat] += [(a + off, b + off, c + off) for a, b, c in t]
    nv = len(verts)
    pos = " ".join(f"{c:.3f}" for p in verts for c in p)            # 1 mm
    nrm = " ".join(f"{c:.2f}" for p in norms for c in p)
    ident = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
    xyz = "".join(f'<param name="{a}" type="float"/>' for a in "XYZ")
    effects = "".join(
        f"""    <effect id="{m}-effect">
      <profile_COMMON><technique sid="common"><lambert>
        <emission><color sid="emission">{e[0]} {e[1]} {e[2]} 1</color></emission>
        <diffuse><color sid="diffuse">{d[0]} {d[1]} {d[2]} 1</color></diffuse>
      </lambert></technique></profile_COMMON>
    </effect>
""" for m, (d, e) in MATERIALS.items())
    materials = "".join(
        f'    <material id="{m}" name="{m}"><instance_effect url="#{m}-effect"/></material>\n'
        for m in MATERIALS)
    triangles = "".join(
        f"""        <triangles material="{m}" count="{len(t)}">
          <input semantic="VERTEX" source="#worker-vertices" offset="0"/>
          <input semantic="NORMAL" source="#worker-normals" offset="1"/>
          <p>{" ".join(f"{i} {i}" for tri in t for i in tri)}</p>
        </triangles>
""" for m, t in groups.items() if t)
    binds = "".join(f'            <instance_material symbol="{m}" target="#{m}"/>\n'
                    for m, t in groups.items() if t)
    vcount = " ".join("1" for _ in range(nv))
    vw = " ".join("0 0" for _ in range(nv))
    return f"""<?xml version="1.0" encoding="utf-8"?>
<COLLADA xmlns="http://www.collada.org/2005/11/COLLADASchema" version="1.4.1">
  <asset>
    <contributor><authoring_tool>gen_worker_dae.py</authoring_tool></contributor>
    <unit name="meter" meter="1"/>
    <up_axis>Z_UP</up_axis>
  </asset>
  <library_effects>
{effects}  </library_effects>
  <library_materials>
{materials}  </library_materials>
  <library_geometries>
    <geometry id="worker-mesh" name="worker">
      <mesh>
        <source id="worker-positions">
          <float_array id="worker-positions-array" count="{nv * 3}">{pos}</float_array>
          <technique_common><accessor source="#worker-positions-array" count="{nv}" stride="3">
            {xyz}
          </accessor></technique_common>
        </source>
        <source id="worker-normals">
          <float_array id="worker-normals-array" count="{nv * 3}">{nrm}</float_array>
          <technique_common><accessor source="#worker-normals-array" count="{nv}" stride="3">
            {xyz}
          </accessor></technique_common>
        </source>
        <vertices id="worker-vertices">
          <input semantic="POSITION" source="#worker-positions"/>
        </vertices>
{triangles}      </mesh>
    </geometry>
  </library_geometries>
  <library_controllers>
    <controller id="worker-skin" name="Armature">
      <skin source="#worker-mesh">
        <bind_shape_matrix>{ident}</bind_shape_matrix>
        <source id="worker-skin-joints">
          <Name_array id="worker-skin-joints-array" count="1">root</Name_array>
          <technique_common><accessor source="#worker-skin-joints-array" count="1" stride="1">
            <param name="JOINT" type="name"/></accessor></technique_common>
        </source>
        <source id="worker-skin-bind_poses">
          <float_array id="worker-skin-bind_poses-array" count="16">{ident}</float_array>
          <technique_common><accessor source="#worker-skin-bind_poses-array" count="1" stride="16">
            <param name="TRANSFORM" type="float4x4"/></accessor></technique_common>
        </source>
        <source id="worker-skin-weights">
          <float_array id="worker-skin-weights-array" count="1">1</float_array>
          <technique_common><accessor source="#worker-skin-weights-array" count="1" stride="1">
            <param name="WEIGHT" type="float"/></accessor></technique_common>
        </source>
        <joints>
          <input semantic="JOINT" source="#worker-skin-joints"/>
          <input semantic="INV_BIND_MATRIX" source="#worker-skin-bind_poses"/>
        </joints>
        <vertex_weights count="{nv}">
          <input semantic="JOINT" source="#worker-skin-joints" offset="0"/>
          <input semantic="WEIGHT" source="#worker-skin-weights" offset="1"/>
          <vcount>{vcount}</vcount>
          <v>{vw}</v>
        </vertex_weights>
      </skin>
    </controller>
  </library_controllers>
  <library_animations>
    <animation id="root-anim" name="root-anim">
      <source id="root-anim-input">
        <float_array id="root-anim-input-array" count="2">0 1</float_array>
        <technique_common><accessor source="#root-anim-input-array" count="2" stride="1">
          <param name="TIME" type="float"/></accessor></technique_common>
      </source>
      <source id="root-anim-output">
        <float_array id="root-anim-output-array" count="32">{ident} {ident}</float_array>
        <technique_common><accessor source="#root-anim-output-array" count="2" stride="16">
          <param name="TRANSFORM" type="float4x4"/></accessor></technique_common>
      </source>
      <source id="root-anim-interp">
        <Name_array id="root-anim-interp-array" count="2">LINEAR LINEAR</Name_array>
        <technique_common><accessor source="#root-anim-interp-array" count="2" stride="1">
          <param name="INTERPOLATION" type="name"/></accessor></technique_common>
      </source>
      <sampler id="root-anim-sampler">
        <input semantic="INPUT" source="#root-anim-input"/>
        <input semantic="OUTPUT" source="#root-anim-output"/>
        <input semantic="INTERPOLATION" source="#root-anim-interp"/>
      </sampler>
      <channel source="#root-anim-sampler" target="root/transform"/>
    </animation>
  </library_animations>
  <library_visual_scenes>
    <visual_scene id="Scene" name="Scene">
      <node id="Armature" name="Armature" type="NODE">
        <matrix sid="transform">{ident}</matrix>
        <node id="root" name="root" sid="root" type="JOINT">
          <matrix sid="transform">{ident}</matrix>
        </node>
      </node>
      <node id="worker" name="worker" type="NODE">
        <matrix sid="transform">{ident}</matrix>
        <instance_controller url="#worker-skin">
          <skeleton>#root</skeleton>
          <bind_material><technique_common>
{binds}          </technique_common></bind_material>
        </instance_controller>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
"""


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = sys.argv[1] if len(sys.argv) > 1 else os.path.join(here, "meshes", "worker.dae")
    parts = build_parts()
    text = build(parts)
    with open(out, "w") as f:
        f.write(text)
    zs = [p[2] for _, (v, _, _) in parts for p in v]
    ys = [p[1] for _, (v, _, _) in parts for p in v]
    print(f"wrote {out}: {len(text) // 1024} KB, height {max(zs):.3f} m, "
          f"width {max(ys) - min(ys):.3f} m")


if __name__ == "__main__":
    main()
