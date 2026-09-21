#!/usr/bin/env python3
"""
작업자(actor) 스킨용 COLLADA 메시 생성기 — 박스 조합 인체 + 단일 관절 스켈레톤 + 정지 애니메이션.

Gazebo Fortress 의 actor 는 스켈레톤이 있는 스킨 메시가 필수라서, 외부 대용량 메시(walk.dae ~MB) 대신
수 KB 짜리 텍스트 DAE 를 직접 생성한다.  사용: python3 gen_worker_dae.py out.dae
"""
import sys


def box(cx, cy, cz, sx, sy, sz):
    """중심(cx,cy,cz), 크기(sx,sy,sz) 박스 → (정점, 삼각형, 법선) 리스트."""
    hx, hy, hz = sx / 2, sy / 2, sz / 2
    faces = [  # (법선, 4개 코너)
        ((0, 0, 1), [(-hx, -hy, hz), (hx, -hy, hz), (hx, hy, hz), (-hx, hy, hz)]),
        ((0, 0, -1), [(-hx, -hy, -hz), (-hx, hy, -hz), (hx, hy, -hz), (hx, -hy, -hz)]),
        ((1, 0, 0), [(hx, -hy, -hz), (hx, hy, -hz), (hx, hy, hz), (hx, -hy, hz)]),
        ((-1, 0, 0), [(-hx, -hy, -hz), (-hx, -hy, hz), (-hx, hy, hz), (-hx, hy, -hz)]),
        ((0, 1, 0), [(-hx, hy, -hz), (-hx, hy, hz), (hx, hy, hz), (hx, hy, -hz)]),
        ((0, -1, 0), [(-hx, -hy, -hz), (hx, -hy, -hz), (hx, -hy, hz), (-hx, -hy, hz)]),
    ]
    verts, tris, norms = [], [], []
    for n, corners in faces:
        base = len(verts)
        for (x, y, z) in corners:
            verts.append((cx + x, cy + y, cz + z))
            norms.append(n)
        tris += [(base, base + 1, base + 2), (base, base + 2, base + 3)]
    return verts, tris, norms


def build(parts, color):
    verts, tris, norms = [], [], []
    for p in parts:
        v, t, n = box(*p)
        off = len(verts)
        verts += v
        norms += n
        tris += [(a + off, b + off, c + off) for a, b, c in t]
    nv, nt = len(verts), len(tris)
    pos = " ".join(f"{c:.4f}" for v in verts for c in v)
    nrm = " ".join(f"{c:.1f}" for n in norms for c in n)
    p = " ".join(f"{i} {i}" for t in tris for i in t)
    ident = "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1"
    xyz = "".join(f'<param name="{a}" type="float"/>' for a in "XYZ")
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
    <effect id="mat-effect">
      <profile_COMMON><technique sid="common"><lambert>
        <diffuse><color sid="diffuse">{color} 1</color></diffuse>
      </lambert></technique></profile_COMMON>
    </effect>
  </library_effects>
  <library_materials>
    <material id="mat" name="mat"><instance_effect url="#mat-effect"/></material>
  </library_materials>
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
        <triangles material="mat" count="{nt}">
          <input semantic="VERTEX" source="#worker-vertices" offset="0"/>
          <input semantic="NORMAL" source="#worker-normals" offset="1"/>
          <p>{p}</p>
        </triangles>
      </mesh>
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
            <instance_material symbol="mat" target="#mat"/>
          </technique_common></bind_material>
        </instance_controller>
      </node>
    </visual_scene>
  </library_visual_scenes>
  <scene><instance_visual_scene url="#Scene"/></scene>
</COLLADA>
"""


# 인체 근사: 머리/몸통/팔/다리 (전체 높이 약 1.75 m, 폭 0.55 m), x 축이 진행 방향
WORKER = [
    (0.0, 0.0, 1.62, 0.20, 0.20, 0.24),    # 머리
    (0.0, 0.0, 1.20, 0.24, 0.42, 0.60),    # 몸통
    (0.0, 0.29, 1.15, 0.11, 0.11, 0.62),   # 왼팔
    (0.0, -0.29, 1.15, 0.11, 0.11, 0.62),  # 오른팔
    (0.0, 0.11, 0.45, 0.20, 0.18, 0.90),   # 왼다리
    (0.0, -0.11, 0.45, 0.20, 0.18, 0.90),  # 오른다리
]

if __name__ == "__main__":
    out = sys.argv[1]
    color = sys.argv[2] if len(sys.argv) > 2 else "0.95 0.55 0.10"
    with open(out, "w") as f:
        f.write(build(WORKER, color))
    print(out)
