"""world_gate: ign CLI 출력 해석, 월드 대기·일시 정지·스폰 확인·재개 흐름 (가짜 CLI)."""

from amr_bringup import world_gate as wg
import pytest

SERVICES = '/gazebo/worlds\n/world/warehouse/control\n/world/warehouse/create\n'
MODELS = ('\nRequesting state for world [warehouse]...\n\nAvailable models:\n'
          '    - ground_plane\n    - amr_02\n    - amr_01\n')
STATS_PAUSED = ('real_time {\n  sec: 4\n  nsec: 5\n}\nsim_time {\n  sec: 1\n  nsec: 250000000\n}\n'
                'paused: true\niterations: 250\n')
STATS_RUNNING = 'sim_time {\n  nsec: 300000000\n}\niterations: 300\n'


class FakeCli:
    """명령 앞부분 → 출력 목록 (차례로, 마지막 값 반복). 호출 기록을 남긴다."""

    def __init__(self, table):
        self.table = {k: list(v) for k, v in table.items()}
        self.calls = []
        self.now = 0.0

    def __call__(self, cmd, timeout):
        self.calls.append(list(cmd))
        self.now += 1.0
        for key, outputs in self.table.items():
            if tuple(cmd[:len(key)]) == key:
                return outputs.pop(0) if len(outputs) > 1 else outputs[0]
        return ''

    def gate(self):
        return wg.WorldGate('warehouse', runner=self, clock=lambda: self.now,
                            sleep=lambda s: setattr(self, 'now', self.now + s),
                            log=lambda msg: self.calls.append(['log', msg]))

    def logs(self):
        return [c[1] for c in self.calls if c[0] == 'log']


def test_parsers():
    assert wg.service_listed(SERVICES, '/world/warehouse/create')
    assert not wg.service_listed(SERVICES, '/world/other/create')
    assert not wg.service_listed('/world/warehouse/create_x\n', '/world/warehouse/create')
    assert wg.parse_model_list(MODELS) == ['ground_plane', 'amr_02', 'amr_01']
    assert wg.parse_model_list('') == []
    assert wg.reply_ok('data: true\n') and not wg.reply_ok('data: false\n')
    assert not wg.reply_ok('Service call timed out\n')
    assert wg.parse_sim_time(STATS_PAUSED) == pytest.approx(1.25)
    assert wg.parse_sim_time(STATS_RUNNING) == pytest.approx(0.3)
    assert wg.parse_sim_time('iterations: 3\n') is None
    assert wg.parse_paused(STATS_PAUSED) is True
    assert wg.parse_paused(STATS_RUNNING) is False       # 기본값 false 는 텍스트에서 빠진다
    assert wg.parse_paused('') is None


def test_hold_pauses_as_soon_as_world_is_up():
    """월드가 뜨기 전부터 pause 요청을 반복하고, 멈춘 뒤 create 서비스를 확인한다."""
    cli = FakeCli({('ign', 'service', '-s'): ['', 'Service call timed out\n', 'data: true\n'],
                   ('ign', 'service', '-l'): [SERVICES],
                   ('ign', 'topic'): [STATS_PAUSED]})
    assert cli.gate().hold(pause=True, timeout=60.0) == wg.EXIT_OK
    calls = [c for c in cli.calls if c[0] == 'ign']
    control = [c for c in calls if c[:3] == ['ign', 'service', '-s']]
    assert len(control) == 3
    assert control[-1] == ['ign', 'service', '-s', '/world/warehouse/control',
                           '--reqtype', 'ignition.msgs.WorldControl',
                           '--reptype', 'ignition.msgs.Boolean',
                           '--timeout', '3000', '--req', 'pause: true']
    assert calls.index(control[-1]) < calls.index(['ign', 'service', '-l'])   # 멈춘 뒤 목록 확인
    assert any('paused=True' in m and 'sim t=1.25' in m for m in cli.logs())


def test_hold_without_pause_never_calls_control():
    cli = FakeCli({('ign', 'service', '-l'): [SERVICES]})
    assert cli.gate().hold(pause=False, timeout=60.0) == wg.EXIT_OK
    assert not any(c[:3] == ['ign', 'service', '-s'] for c in cli.calls)


@pytest.mark.parametrize('pause', [True, False])
def test_hold_times_out_without_world(pause):
    cli = FakeCli({('ign', 'service', '-l'): ['']})
    assert cli.gate().hold(pause=pause, timeout=5.0) == wg.EXIT_NO_WORLD
    assert any('ERROR' in m for m in cli.logs())


def test_hold_reports_pause_failure_but_world_up():
    cli = FakeCli({('ign', 'service', '-l'): [SERVICES],
                   ('ign', 'service', '-s'): ['Service call timed out\n']})
    assert cli.gate().hold(pause=True, timeout=5.0) == wg.EXIT_CONTROL
    assert sum(c[:3] == ['ign', 'service', '-s'] for c in cli.calls) >= 2    # 시간 안에서 재시도


def test_set_paused_retries():
    cli = FakeCli({('ign', 'service', '-s'): ['']})
    assert not cli.gate().set_paused(False, attempts=3)
    assert sum(c[:3] == ['ign', 'service', '-s'] for c in cli.calls) == 3


def test_release_waits_for_all_models_then_unpauses():
    partial = MODELS.replace('    - amr_01\n', '')
    cli = FakeCli({('ign', 'model'): [partial, partial, MODELS],
                   ('ign', 'service', '-s'): ['data: true\n'],
                   ('ign', 'topic'): [STATS_RUNNING]})
    assert cli.gate().release(['amr_01', 'amr_02'], unpause=True, timeout=60.0) == wg.EXIT_OK
    assert sum(c[:2] == ['ign', 'model'] for c in cli.calls) == 3
    control = [c for c in cli.calls if c[:3] == ['ign', 'service', '-s']]
    assert control[-1][-1] == 'pause: false'


def test_release_missing_model_still_unpauses():
    cli = FakeCli({('ign', 'model'): [MODELS], ('ign', 'service', '-s'): ['data: true\n']})
    code = cli.gate().release(['amr_01', 'amr_03'], unpause=True, timeout=3.0)
    assert code == wg.EXIT_MISSING
    assert any('amr_03' in m and 'ERROR' in m for m in cli.logs())
    assert [c for c in cli.calls if c[:3] == ['ign', 'service', '-s']][-1][-1] == 'pause: false'


def test_release_unpause_failure():
    cli = FakeCli({('ign', 'model'): [MODELS], ('ign', 'service', '-s'): ['']})
    assert cli.gate().release(['amr_01'], unpause=True, timeout=3.0) == wg.EXIT_CONTROL


def test_run_cli_handles_missing_binary_and_output():
    assert wg.run_cli(['/nonexistent/amr_bringup_cli'], 1.0) == ''
    assert wg.run_cli(['echo', 'data: true'], 5.0).strip() == 'data: true'


def test_main_dispatches(monkeypatch):
    seen = {}

    class Gate:
        def __init__(self, world):
            seen['world'] = world

        def hold(self, pause, timeout):
            seen['hold'] = (pause, timeout)
            return 0

        def release(self, names, unpause, timeout):
            seen['release'] = (names, unpause, timeout)
            return 2

    monkeypatch.setattr(wg, 'WorldGate', Gate)
    assert wg.main(['hold', '--world', 'w', '--pause']) == 0
    assert seen['hold'] == (True, 600.0) and seen['world'] == 'w'
    argv = ['release', '--world', 'w', '--models', 'amr_01, amr_02,', '--timeout', '9']
    assert wg.main(argv) == 2
    assert seen['release'] == (['amr_01', 'amr_02'], False, 9.0)
