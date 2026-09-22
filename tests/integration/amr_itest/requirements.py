"""
시나리오 요구사항 검사: 패키지·실행 파일·런치 파일·파이썬 모듈·GPU 존재 (ROS 설치 트리 조회).

시나리오는 launch 를 띄우기 전에 필요한 것을 모두 확인하고, 없으면 무엇이 없는지 정확한
사유로 건너뛴다 (패키지가 머지되면 자동으로 켜진다).
"""

from dataclasses import dataclass
import importlib.util
import os
from pathlib import Path
from typing import Iterable, List, Optional

from ament_index_python.packages import (get_package_prefix, get_package_share_directory,
                                         PackageNotFoundError)


def share_dir(package: str) -> Optional[Path]:
    """패키지 share 디렉토리 (설치 안 됐으면 None)."""
    try:
        return Path(get_package_share_directory(package))
    except (PackageNotFoundError, ValueError):
        return None


def has_package(package: str) -> bool:
    return share_dir(package) is not None


def executable_path(package: str, executable: str) -> Optional[Path]:
    """lib/<package>/<executable> 경로 (없거나 실행 불가면 None)."""
    try:
        prefix = Path(get_package_prefix(package))
    except (PackageNotFoundError, ValueError):
        return None
    path = prefix / 'lib' / package / executable
    return path if path.is_file() and os.access(path, os.X_OK) else None


def has_executable(package: str, executable: str) -> bool:
    return executable_path(package, executable) is not None


def launch_file(package: str, name: str) -> Optional[Path]:
    """share/<package>/launch/<name> 경로 (없으면 None)."""
    share = share_dir(package)
    if share is None:
        return None
    path = share / 'launch' / name
    return path if path.is_file() else None


def config_file(package: str, name: str) -> Optional[Path]:
    """share/<package>/config/<name> 경로 (없으면 None)."""
    share = share_dir(package)
    if share is None:
        return None
    path = share / 'config' / name
    return path if path.is_file() else None


def has_python_module(name: str) -> bool:
    try:
        return importlib.util.find_spec(name) is not None
    except (ImportError, ValueError):
        return False


def gpu_device_present() -> bool:
    """헤드리스 렌더링 센서용 GPU 장치 노드 (/dev/nvidia0 또는 /dev/dri/renderD*)."""
    if Path('/dev/nvidia0').exists():
        return True
    dri = Path('/dev/dri')
    return dri.is_dir() and any(p.name.startswith('renderD') for p in dri.iterdir())


@dataclass(frozen=True)
class Requirement:
    """
    요구사항 하나.

    kind: package | executable | launch | module | file | gpu
    why:  무엇 때문에 필요한지 (건너뛰기 사유에 그대로 나온다)
    """

    kind: str
    package: str = ''
    name: str = ''
    why: str = ''

    def missing(self) -> Optional[str]:
        """충족되면 None, 아니면 사람이 읽을 사유."""
        ok = True
        what = ''
        if self.kind == 'package':
            ok, what = has_package(self.package), f'package {self.package}'
        elif self.kind == 'executable':
            ok = has_executable(self.package, self.name)
            what = f'executable {self.package}/{self.name}'
        elif self.kind == 'launch':
            ok = launch_file(self.package, self.name) is not None
            what = f'launch file {self.package}/launch/{self.name}'
        elif self.kind == 'module':
            ok, what = has_python_module(self.name), f'python module {self.name}'
        elif self.kind == 'file':
            ok, what = Path(self.name).is_file(), f'file {self.name}'
        elif self.kind == 'gpu':
            ok, what = gpu_device_present(), 'GPU device (/dev/nvidia0 or /dev/dri/renderD*)'
        else:
            raise ValueError(f'unknown requirement kind {self.kind!r}')
        if ok:
            return None
        return f'{what} not found' + (f' ({self.why})' if self.why else '')


def package(pkg: str, why: str = '') -> Requirement:
    return Requirement('package', pkg, '', why)


def executable(pkg: str, exe: str, why: str = '') -> Requirement:
    return Requirement('executable', pkg, exe, why)


def launch(pkg: str, name: str, why: str = '') -> Requirement:
    return Requirement('launch', pkg, name, why)


def module(name: str, why: str = '') -> Requirement:
    return Requirement('module', '', name, why)


def file(path: Path, why: str = '') -> Requirement:
    return Requirement('file', '', str(path), why)


def gpu(why: str = 'Gazebo headless rendering sensors') -> Requirement:
    return Requirement('gpu', why=why)


def missing(reqs: Iterable[Requirement]) -> List[str]:
    """충족되지 않은 요구사항의 사유 목록."""
    return [m for m in (r.missing() for r in reqs) if m]
