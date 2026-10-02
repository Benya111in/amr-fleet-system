"""web/app.js 헤드리스 단위 검사 (test/js/app_unit.js) — node 가 있을 때만 (이미지에는 Node.js 가 없다)."""

import os
import shutil
import subprocess

import pytest

NODE = shutil.which('node')
HERE = os.path.dirname(os.path.abspath(__file__))
APP_JS = os.path.join(HERE, '..', 'web', 'app.js')

pytestmark = pytest.mark.skipif(NODE is None, reason='node 없음 — 호스트에서 node test/js/app_unit.js')


def test_app_js_headless_unit():
    result = subprocess.run([NODE, os.path.join(HERE, 'js', 'app_unit.js'), APP_JS],
                            capture_output=True, text=True, timeout=120)
    assert result.returncode == 0, result.stdout + result.stderr
    assert 'JS UNIT PASS' in result.stdout
