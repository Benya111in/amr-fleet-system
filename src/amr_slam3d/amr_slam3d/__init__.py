"""
보너스 2 (3D LiDAR SLAM) 매핑 행 — rclpy 비의존 순수 모듈.

- route   : 정답(GT) 되먹임 경로 추종 (파일럿 bag 기록용 주행 — 경로가 추정기와 독립)
- se2     : 평면 자세 합성·각 정규화·쿼터니언 yaw
- grid    : SLAM 격자(자기 map 프레임) → 월드 정렬 격자, map_server PGM/YAML 쓰기, 점유 일치율
- bagstat : bag 메시지 스탬프 → 토픽별 개수·주기·간격 통계
- tftree  : 관측한 TF 간선 → 부모 하나 규칙·순환 검사
- rows    : 매핑 행 정의 (입력 토픽·재생 토픽·프레임·판정 패턴)
"""
