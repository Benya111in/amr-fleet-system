"""
빌드/테스트 파이프라인이 동작하는지 확인하는 임시 테스트.

A*/DWA 구현을 시작하면 이 파일을 실제 알고리즘 테스트로 교체한다.
명세 10장: 주요 모듈 테스트 커버리지 70% 이상.
"""


def test_pipeline_alive():
    """파이프라인(colcon test / pytest) 동작 확인."""
    assert True


def test_numpy_available():
    """수치 연산 의존성이 컨테이너에 있는지 확인."""
    import numpy as np

    assert np.isclose(np.linalg.norm([3.0, 4.0]), 5.0)
