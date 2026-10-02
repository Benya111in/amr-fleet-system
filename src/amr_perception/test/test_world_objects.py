"""world_objects 테스트: 실제 창고 월드 파싱(인라인 표지판 판 25 개 — 리뷰 회귀), 모델 외곽, 깊이 검증 라벨."""

import math
import os

from amr_perception import world_objects as wo
from amr_perception.synthetic import WorldObject
import numpy as np
import pytest

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def _sim_dir() -> str:
    """amr_simulation 소스(워크스페이스) 또는 설치 share."""
    src = os.path.join(os.path.dirname(PKG), 'amr_simulation')
    if os.path.isdir(os.path.join(src, 'worlds')):
        return src
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory('amr_simulation')


def _world():
    sim = _sim_dir()
    with open(os.path.join(sim, 'worlds', 'warehouse.sdf'), 'r', encoding='utf-8') as f:
        return f.read(), [os.path.join(sim, 'models')]


# ------------------------------------------------------------------ 실제 월드


def test_real_world_labels_inline_signs_and_boxes():
    sdf, dirs = _world()
    objs, actors = wo.parse_world(sdf, dirs)
    by_cls = {}
    for o in objs:
        by_cls.setdefault(o.class_name, []).append(o)
    # 회귀: 이전 parse_world 는 <include> 만 읽어 인라인 판 표지판 25 개를 하나도 라벨하지 않았다
    assert len(by_cls['sign']) == 25
    assert len(by_cls['box']) == 108            # 소 50 + 중 29 + 대 29 (랙 위 + 도크 바닥)
    assert len(actors) == 6
    sign = next(o for o in by_cls['sign'] if o.name.startswith('sign_zone_a_east'))
    assert np.allclose(sign.center, [19.012, 9.0, 1.7])
    assert np.allclose(sign.size, [0.02, 0.8, 0.6])
    assert sign.yaw == pytest.approx(0.0)
    west = next(o for o in by_cls['sign'] if o.name.startswith('sign_zone_a_west'))
    assert abs(abs(west.yaw) - math.pi) < 1e-3       # 서쪽 판은 −x 를 본다
    big = next(o for o in by_cls['box'] if o.name == 'dock_a_box_large')
    assert np.allclose(big.size, [0.6, 0.5, 0.4]) and big.center[2] == pytest.approx(0.2)
    # 마커 판(dock_*_marker)·문은 표지판이 아니다
    assert not any('marker' in o.name or 'door' in o.name for o in objs)


def test_real_world_dynamic_templates():
    sdf, dirs = _world()
    t = wo.dynamic_templates(sdf, dirs, {'forklift_main': 'forklift', 'shuttle_amr': 'amr'})
    fk = t['forklift_main']
    # 차체 뒤 −1.2 ~ 포크 끝 +2.0, 바퀴 바깥 ±0.64, 바닥 0 ~ 마스트 2.2
    # (amr_simulation config/dynamic_obstacles.yaml 발자국과 같다)
    assert np.allclose(fk.offset, [0.4, 0.0, 1.1], atol=1e-6)
    assert np.allclose(fk.size, [3.2, 1.28, 2.2], atol=1e-6)
    sh = t['shuttle_amr']
    assert sh.class_name == 'amr' and sh.size[0] == pytest.approx(0.6, abs=0.02)
    box = fk.at(10.0, 15.0, math.pi / 2)
    assert np.allclose(box.center, [10.0, 15.4, 1.1]) and box.yaw == pytest.approx(math.pi / 2)


# ------------------------------------------------------------------ 모델 외곽·인라인 판

MODEL = """<sdf version="1.8"><model name="m">
<link name="a"><pose>1 0 0 0 0 0</pose>
  <visual name="b"><pose>0 0 0.5 0 0 0</pose>
    <geometry><box><size>2 1 1</size></box></geometry></visual>
  <visual name="w"><pose>0 0.6 0.2 1.5708 0 0</pose>
    <geometry><cylinder><radius>0.2</radius><length>0.1</length></cylinder></geometry></visual>
  <visual name="mesh"><geometry><mesh><uri>x.dae</uri></mesh></geometry></visual>
</link></model></sdf>"""


def test_model_extent_rotated_cylinder():
    ctr, size = wo.model_extent(MODEL)
    # 박스 x 0..2, y −0.5..0.5, z 0..1; 굴린 원기둥 y 0.55..0.65, z 0..0.4
    assert np.allclose(ctr - size / 2, [0.0, -0.5, 0.0], atol=1e-4)
    assert np.allclose(ctr + size / 2, [2.0, 0.65, 1.0], atol=1e-4)
    assert wo.model_extent('<sdf><model name="e"><link name="l"/></model></sdf>') is None


INLINE = """<sdf version="1.8"><world name="w">
<model name="sign_x"><static>true</static><pose>5 0 0 0 0 1.5708</pose>
  <link name="link"><visual name="plate"><pose>0.1 0 1.6 0 0 0</pose>
  <geometry><box><size>0.02 0.6 0.4</size></box></geometry></visual></link></model>
<model name="door_x"><pose>0 0 0 0 0 0</pose><link name="l"><visual name="v">
  <geometry><box><size>1 1 1</size></box></geometry></visual></link></model>
</world></sdf>"""


def test_inline_plate_pose_composition():
    objs, _ = wo.parse_world(INLINE)
    assert len(objs) == 1
    o = objs[0]
    assert o.class_name == 'sign' and o.name == 'sign_x/plate'
    assert np.allclose(o.center, [5.0, 0.1, 1.6], atol=1e-4)     # 판 오프셋 x 0.1 이 yaw 90° 로 +y
    assert o.yaw == pytest.approx(math.pi / 2, abs=1e-4)
    assert wo.parse_world(INLINE, inline_rules={})[0] == []


# ------------------------------------------------------------------ 깊이 검증 라벨

K = wo.Intrinsics(337.21, 337.21, 320.0, 240.0, 640, 480)
BASE_TO_OPTICAL = wo.homogeneous(np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]),
                                 [0.29, 0.0, 0.25])   # 광학 z = 전방, x = 오른쪽, y = 아래 (sensors.yaml)


def _render_depth(objs, r_cw, t_cw, k=K, floor=True, far=np.inf):
    """시험용 광선 추적 깊이 이미지 (광학 Z): 박스들 + 바닥 z = 0."""
    uu, vv = np.meshgrid(np.arange(k.width) + 0.5, np.arange(k.height) + 0.5)
    d_c = np.stack([(uu - k.cx) / k.fx, (vv - k.cy) / k.fy, np.ones_like(uu)], axis=-1)
    d_w = d_c @ r_cw                                    # r_cwᵀ · d
    cam = -r_cw.T @ t_cw
    depth = np.full(uu.shape, np.inf)
    for o in objs:
        c, s = math.cos(o.yaw), math.sin(o.yaw)
        r_wo = np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
        half = 0.5 * np.asarray(o.size)
        d_o = d_w @ r_wo
        o_o = r_wo.T @ (cam - o.center)
        with np.errstate(divide='ignore', invalid='ignore'):
            t1, t2 = (-half - o_o) / d_o, (half - o_o) / d_o
        t_in = np.nanmax(np.minimum(t1, t2), axis=-1)
        t_out = np.nanmin(np.maximum(t1, t2), axis=-1)
        hit = (t_out >= t_in) & (t_in > 0.05)
        depth = np.where(hit & (t_in < depth), t_in, depth)
    if floor:
        with np.errstate(divide='ignore', invalid='ignore'):
            t = -cam[2] / d_w[..., 2]
        depth = np.where((t > 0.05) & (t < depth), t, depth)
    depth[depth > far] = np.inf
    return depth


def _cam(x=0.0, y=0.0, yaw=0.0):
    return wo.camera_from_base(x, y, yaw, BASE_TO_OPTICAL)


def test_camera_from_base_projects_forward_point_to_center():
    r_cw, t_cw = _cam(1.0, 2.0, math.pi / 2)
    p = r_cw @ np.array([1.0, 2.0 + 0.29 + 3.0, 0.25]) + t_cw
    assert np.allclose(p, [0.0, 0.0, 3.0], atol=1e-9)


def test_unoccluded_box_on_floor_gets_tight_projection():
    box = WorldObject('box', np.array([3.0, 0.2, 0.15]), np.array([0.5, 0.4, 0.3]), 0.3, 'b')
    r_cw, t_cw = _cam()
    depth = _render_depth([box], r_cw, t_cw)
    lab = wo.label_object(box, r_cw, t_cw, K, depth)
    assert lab.kept and lab.visible == pytest.approx(1.0) and lab.truncated == 0.0
    full = lab.full_bbox
    # 윗면·옆면이 보이는 박스 → 보이는 bbox 가 8 꼭짓점 투영과 거의 같다. 바닥 점은 들어가지 않는다 (아래 변)
    assert abs(lab.bbox[0] - full[0]) <= 2.0 and abs(lab.bbox[2] - full[2]) <= 2.0
    assert abs(lab.bbox[1] - full[1]) <= 2.0
    assert lab.bbox[3] <= full[3] + 1.0
    assert lab.distance == pytest.approx(math.hypot(3.0 - 0.29, 0.2, 0.1), abs=1e-6)


def test_occluder_halves_visibility_and_bbox_covers_visible_part():
    target = WorldObject('box', np.array([4.0, 0.0, 0.5]), np.array([0.4, 1.0, 1.0]), 0.0, 't')
    # 목표 왼쪽 절반(+y) 앞 1.5 m 에 기둥
    pole = WorldObject('pillar', np.array([2.5, 0.15, 0.6]), np.array([0.2, 0.3, 1.2]), 0.0, 'p')
    r_cw, t_cw = _cam()
    depth = _render_depth([target, pole], r_cw, t_cw)
    free = wo.label_object(target, r_cw, t_cw, K, _render_depth([target], r_cw, t_cw))
    lab = wo.label_object(target, r_cw, t_cw, K, depth)
    assert 0.3 < lab.visible < 0.8 and lab.kept
    assert lab.bbox[0] > free.bbox[0] + 20          # 왼쪽(이미지 −u, 월드 +y)이 가려져 bbox 가 오른쪽으로 줄었다
    assert lab.bbox[2] == pytest.approx(free.bbox[2], abs=2.0)


def test_fully_hidden_and_far_objects_rejected():
    target = WorldObject('person', np.array([6.0, 0.0, 0.95]), np.array([0.7, 0.7, 1.9]), 0.0, 'w')
    wall = WorldObject('wall', np.array([3.0, 0.0, 2.0]), np.array([0.2, 6.0, 4.0]), 0.0, 'x')
    r_cw, t_cw = _cam()
    lab = wo.label_object(target, r_cw, t_cw, K, _render_depth([target, wall], r_cw, t_cw))
    assert not lab.kept and lab.reason == 'pixels' and lab.visible == 0.0
    # 깊이 범위 밖(무효 = inf) → 검증 불가 → 라벨 없음
    lab = wo.label_object(target, r_cw, t_cw, K, _render_depth([target], r_cw, t_cw, far=5.0))
    assert not lab.kept and lab.reason == 'pixels'
    assert wo.label_object(target, r_cw, t_cw, K, np.full((480, 640), np.inf),
                           wo.LabelParams(max_range=3.0)) is None
    # 카메라 뒤
    behind = WorldObject('box', np.array([-3.0, 0.0, 0.2]), np.array([0.4, 0.4, 0.4]), 0.0, 'b')
    assert wo.label_object(behind, r_cw, t_cw, K, np.full((480, 640), np.inf)) is None


def test_thin_person_gets_silhouette_bbox_not_cuboid():
    # 사람 몸통(폭 0.3)만 있는데 라벨 박스는 0.7 폭 — 보이는 픽셀 외접 사각형은 몸통 폭이어야 한다
    body = WorldObject('body', np.array([4.0, 0.0, 0.9]), np.array([0.25, 0.3, 1.8]), 0.0, 'b')
    person = WorldObject('person', np.array([4.0, 0.0, 0.95]), np.array([0.7, 0.7, 1.9]), 0.0, 'p')
    r_cw, t_cw = _cam()
    lab = wo.label_object(person, r_cw, t_cw, K, _render_depth([body], r_cw, t_cw))
    width_px = lab.bbox[2] - lab.bbox[0]
    assert lab.kept and width_px == pytest.approx(0.3 * K.fx / (4.0 - 0.125 - 0.29), abs=4.0)
    assert (lab.full_bbox[2] - lab.full_bbox[0]) > 1.8 * width_px


def test_truncation_rules():
    r_cw, t_cw = _cam()
    # 이미지 오른쪽 가장자리에 반쯤 걸친 박스 → 잘린 채 라벨, bbox 는 이미지 안
    edge = WorldObject('box', np.array([3.0, -2.3, 0.3]), np.array([0.6, 0.6, 0.6]), 0.0, 'e')
    lab = wo.label_object(edge, r_cw, t_cw, K, _render_depth([edge], r_cw, t_cw))
    assert 0.2 < lab.truncated < 0.75 and lab.kept and lab.bbox[2] <= 640.0
    # 투영의 80 % 이상이 밖 + 보이는 부분이 작다 → 버림
    out = WorldObject('box', np.array([3.0, -2.9, 0.1]), np.array([0.6, 0.6, 0.2]), 0.0, 'o')
    lab = wo.label_object(out, r_cw, t_cw, K, _render_depth([out], r_cw, t_cw))
    assert lab is not None and lab.truncated > 0.75
    assert not lab.kept and lab.reason == 'truncated'
    # 바로 앞 사람 다리(투영의 대부분이 이미지 밖)라도 보이는 부분이 크면 라벨
    near = WorldObject('person', np.array([0.8, 0.0, 0.95]), np.array([0.5, 0.5, 1.9]), 0.0, 'n')
    lab = wo.label_object(near, r_cw, t_cw, K, _render_depth([near], r_cw, t_cw))
    assert lab.truncated > 0.75 and lab.kept


def test_unsupported_sign_plate_keeps_bottom_edge():
    plate = WorldObject('sign', np.array([5.0, 0.0, 1.0]), np.array([0.02, 0.8, 0.6]), math.pi,
                        's')
    r_cw, t_cw = _cam()
    depth = _render_depth([plate], r_cw, t_cw)
    labs = wo.label_objects([plate], r_cw, t_cw, K, depth)
    assert len(labs) == 1 and labs[0].kept
    assert labs[0].bbox[3] == pytest.approx(labs[0].full_bbox[3], abs=2.0)
    # 받침면 규칙을 적용하면(supported) 밑 3 cm 가 빠져 아래 변이 올라간다
    sup = wo.label_object(plate, r_cw, t_cw, K, depth, supported=True)
    assert sup.bbox[3] < labs[0].bbox[3] - 1.0


def test_iou_and_match_detections():
    assert wo.iou((0, 0, 10, 10), (0, 0, 10, 10)) == pytest.approx(1.0)
    assert wo.iou((0, 0, 10, 10), (5, 0, 15, 10)) == pytest.approx(1 / 3)
    assert wo.iou((0, 0, 0, 0), (0, 0, 0, 0)) == 0.0
    gts = [('person', (0, 0, 10, 20), True), ('box', (50, 50, 60, 60), True),
           ('box', (100, 100, 104, 104), False)]
    dets = [('person', (1, 0, 10, 20), 0.9), ('person', (0, 1, 10, 20), 0.8),
            ('box', (100, 100, 104, 105), 0.7), ('sign', (50, 50, 60, 60), 0.6)]
    gm, dm = wo.match_detections(gts, dets)
    assert gm == [0, -1, -1]            # 두 번째 person 은 중복(FP), box 필수 GT 는 못 맞힘(sign 은 다른 클래스)
    assert dm == [0, -1, 2, -1]         # 작은 box 검출은 무시 GT 에 붙는다 (FP 아님)
