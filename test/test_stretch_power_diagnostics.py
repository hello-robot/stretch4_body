"""stretch_power_diagnostics: the report, --json, --from-log, --at and the no-server path.

Statuses are real decoder output built by test_power_health. Live mode runs against a
fake status reader; nothing here opens a device or needs a server.
"""
import json
import zipfile
from pathlib import Path

import pytest

from stretch4_body.tools import stretch_power_diagnostics as cli
from stretch4_body.subsystem.power_health import CHECK_IDS, CHECK_TITLES
from test_power_health import evaluate, status

FIXTURE = Path(__file__).parent / 'fixtures' / 'power_status_log.json'
T0 = 1781000000.0

# Plausible bench values for the fields the health fixture leaves at fixture bytes.
REALISTIC = {
    'uptime_ms': 5400000, 'battery_soh': 98, 'charger_current_ma': 0,
    'voltage_cpu_mv': 19650, 'voltage_5v0_mv': 5040, 'voltage_36v0_mv': 35400,
    'voltage_12v0_mv': 12020, 'voltage_20v0_aux_mv': 19800, 'current_cpu_ma': 2100,
    'pimu_temp_centi_c': 3650, 'loop_time_max_us': 410, 'charger_enable_command': 0,
    'charger_expected_active': 0, 'adapter_voltage_present': 0, 'esp_adapter_connected': 0,
}

BOOT = {'sequence': 9, 'event_id': 1, 'event_name': 'boot', 'power_state': 0,
        'power_state_name': 'STATE_ACTIVE', 'timestamp_ms': 1200, 'data0': 1, 'data1': 2,
        'estimated_wall_time_s': T0 - 5398.8}
LENGTH_ERROR = {'sequence': 10, 'event_id': 45, 'event_name': 'esp_rx_length_error',
                'power_state': 0, 'power_state_name': 'STATE_ACTIVE', 'timestamp_ms': 5401500,
                'data0': 20, 'data1': 0x0709, 'estimated_wall_time_s': T0 + 1.5}


def build_samples(with_health=False):
    """Six logged samples, 0.5 s apart. From the fourth on, the ESP length-error counter
    climbs 20 per sample, so a replay over the window sees a rate that faults."""
    samples = []
    for i in range(6):
        errors = 0 if i < 3 else 20 * (i - 2)
        st = status(meta={'sample_seq': i + 1},
                    log={'events': [BOOT, LENGTH_ERROR] if i >= 3 else [BOOT],
                         'cursor_sequence': 10 if i >= 3 else 9},
                    esp_rx_length_errors=errors, **REALISTIC)
        if with_health:
            st['power_health'] = evaluate(st)
        samples.append({'timestamp': T0 + 0.5 * i, 'status_id': i, 'power_periph': st,
                        'pimu': st, 'lift': {'pos': 0.5}})
    return samples


def write_log(path, samples):
    path.write_text(json.dumps(samples, default=str))
    return path


def args(*argv):
    return cli.build_parser().parse_args(list(argv))


def test_committed_fixture_matches_the_builder():
    """The fixture file is build_samples() dumped; regenerate it if the decoder changes."""
    assert json.loads(FIXTURE.read_text()) == json.loads(json.dumps(build_samples(), default=str))


# --- rendering --------------------------------------------------------------------------------

def test_report_has_lamp_every_check_values_and_events():
    st = status(**REALISTIC, log={'events': [BOOT, LENGTH_ERROR]})
    text = cli.render_report(st, evaluate(st), 'live', events=10, now_wall=T0, robot='4056')
    assert text.startswith('Stretch power diagnostics, 4056, live,')
    assert 'PIMU firmware hello-pimu2.v0.1.50p14 (p14), diagnostics schema 2' in text
    assert 'EOA output         on, 0.35 A' in text
    assert 'Overall [ OK  ]' in text
    for check_id in CHECK_IDS:
        assert CHECK_TITLES[check_id] in text
    assert 'Battery (BMS)      27.20 V, 1.50 A, SoC 80 %' in text
    assert 'source adc (0), agreement agree (0)' in text
    assert 'last end none_since_boot (255)' in text
    assert 'esp_rx_length_error (45): length error count 20, id 0x07, length 9' in text
    assert 'boot (1): reset cause power_on' in text
    assert '\033[' not in text
    for dash in ('\u2013', '\u2014', '\u2500', '\u2502'):
        assert dash not in text


def test_report_lamps_follow_levels_and_color_is_optional():
    st = status(**REALISTIC, adapter_presence_source=1)
    health = evaluate(st)
    text = cli.render_report(st, health, 'live', now_wall=T0)
    assert 'Overall [WARN ]' in text
    assert '[WARN ] Adapter source' in text
    colored = cli.render_report(st, health, 'live', now_wall=T0, color=True)
    assert '\033[33m[WARN ]' in colored


def test_events_option_limits_and_zero_hides():
    st = status(**REALISTIC, log={'events': [BOOT, LENGTH_ERROR]})
    one = cli.render_report(st, evaluate(st), 'live', events=1, now_wall=T0)
    assert 'Last 1 power events (of 2 held' in one and '#9' not in one
    none = cli.render_report(st, evaluate(st), 'live', events=0, now_wall=T0)
    assert 'power events' not in none


def test_no_sample_yet_renders_without_key_values():
    st = status(meta={'available': False, 'sample_seq': 0, 'sample_age_s': None})
    st['power_diagnostics'] = {'_valid': False}
    text = cli.render_report(st, evaluate(st), 'live', now_wall=T0)
    assert 'diagnostics schema none' in text
    assert 'Key values' not in text


def test_schema1_report_says_unknown_not_warn():
    st = status(schema=1, **{k: v for k, v in REALISTIC.items()})
    health = evaluate(st)
    text = cli.render_report(st, health, 'live', now_wall=T0)
    assert health['level'] == 'ok'
    assert '[  ?  ] ADC banks' in text
    assert 'not reported by this firmware' in text
    assert 'NUC shutdown wait  idle' not in text


def test_json_report_is_complete_and_serializable():
    st = status(**REALISTIC, log={'events': [LENGTH_ERROR]})
    report = cli.report_dict(st, evaluate(st), 'live', events=5)
    back = json.loads(json.dumps(report))
    assert back['health']['level'] == 'ok'
    assert [c['id'] for c in back['health']['checks']] == list(CHECK_IDS)
    assert back['events'][0]['detail'] == 'length error count 20, id 0x07, length 9'
    assert back['key_values']['ADC'].startswith('stale mask 0x0')


# --- reading status logs ----------------------------------------------------------------------

def test_load_log_keeps_only_power_and_timestamps():
    samples, notes = cli.load_log(str(FIXTURE))
    assert notes == []
    assert len(samples) == 6
    assert set(samples[0]) == {'timestamp', 'power_periph'}
    assert samples[0]['timestamp'] == T0


def test_load_log_accepts_the_pimu_alias_only(tmp_path):
    samples = build_samples()
    for s in samples:
        del s['power_periph']
    loaded, _ = cli.load_log(str(write_log(tmp_path / 'status_a.json', samples)))
    assert len(loaded) == 6


def test_truncated_log_keeps_samples_before_the_damage(tmp_path):
    text = FIXTURE.read_text()
    cut = tmp_path / 'status_cut.json'
    cut.write_text(text[:int(len(text) * 0.6)])
    samples, notes = cli.load_log(str(cut))
    assert 0 < len(samples) < 6
    assert notes == ['status_cut.json is damaged: skipped 1 damaged spot, kept %d samples'
                     % len(samples)]


def test_spliced_log_skips_the_damaged_sample_and_goes_on(tmp_path):
    """Real status logs hold splices like 'fals{' partway through."""
    text = FIXTURE.read_text()
    second = text.index('{"timestamp"', 10)
    bad = tmp_path / 'status_splice.json'
    bad.write_text(text[:second] + 'fals' + text[second:])
    samples, notes = cli.load_log(str(bad))
    assert len(samples) == 5
    assert [s['timestamp'] for s in samples] == [T0] + [T0 + 0.5 * i for i in range(2, 6)]
    assert 'damaged' in notes[0]


def test_log_that_starts_mid_sample(tmp_path):
    """Some real log files start inside a sample, with no opening bracket."""
    text = FIXTURE.read_text()
    bad = tmp_path / 'status_mid.json'
    bad.write_text(text[200:])
    samples, notes = cli.load_log(str(bad))
    assert len(samples) == 5
    assert samples[0]['timestamp'] == T0 + 0.5


def test_empty_log_and_export_zip(tmp_path):
    empty = tmp_path / 'status_empty.json'
    empty.write_text('')
    assert cli.load_log(str(empty)) == ([], ['status_empty.json is empty'])
    zpath = tmp_path / 'export.zip'
    with zipfile.ZipFile(zpath, 'w') as zf:
        zf.writestr('status_1.json', json.dumps(build_samples()[:3], default=str))
        zf.writestr('status_2.json', json.dumps(build_samples()[3:], default=str))
        zf.writestr('notes.txt', 'ignored')
    samples, notes = cli.load_log(str(zpath))
    assert len(samples) == 6 and notes == []


@pytest.mark.parametrize('spec, expected', [
    (None, 5), ('+0', 0), ('+1.2', 2), ('-0.5', 4), ('-100', 0), (str(T0 + 1.0), 2),
])
def test_at_selects_the_sample_at_or_before(spec, expected):
    samples, _ = cli.load_log(str(FIXTURE))
    at = cli.parse_at(spec, samples) if spec else None
    assert cli.select_sample(samples, at) == expected


def test_at_accepts_local_clock_time():
    samples, _ = cli.load_log(str(FIXTURE))
    import datetime
    clock = datetime.datetime.fromtimestamp(T0 + 1.0).strftime('%H:%M:%S')
    assert cli.select_sample(samples, cli.parse_at(clock, samples)) == 2


def test_replay_recomputes_rates_when_the_log_has_no_health():
    samples, _ = cli.load_log(str(FIXTURE))
    health, how = cli.health_for_log_sample(samples, 5)
    assert how == 'recomputed from the logged diagnostics'
    frame = {c['id']: c for c in health['checks']}['esp_frame_errors']
    assert frame['level'] == 'fault'
    early, _ = cli.health_for_log_sample(samples, 2)
    assert early['level'] == 'ok'


def test_logged_health_is_used_as_is(tmp_path):
    samples = build_samples(with_health=True)
    samples[-1]['power_periph']['power_health']['summary'] = 'from the server'
    loaded, _ = cli.load_log(str(write_log(tmp_path / 'status_h.json', samples)))
    health, how = cli.health_for_log_sample(loaded, 5)
    assert how == 'as logged by the server'
    assert health['summary'] == 'from the server'


def test_from_log_main_prints_the_chosen_sample(capsys):
    assert cli.main(['--from-log', str(FIXTURE), '--at', '+1', '--no-color']) == 0
    out = capsys.readouterr().out
    assert 'from power_status_log.json, sample 3 of 6, health recomputed' in out
    assert 'Overall [ OK  ]' in out


def test_from_log_json(capsys):
    assert cli.main(['--from-log', str(FIXTURE), '--json']) == 0
    report = json.loads(capsys.readouterr().out)
    assert report['health']['level'] == 'fault'
    assert report['events'][-1]['event_name'] == 'esp_rx_length_error'


def test_from_log_with_nothing_usable(tmp_path, capsys):
    path = write_log(tmp_path / 'status_x.json', [{'timestamp': T0, 'lift': {}}])
    assert cli.main(['--from-log', str(path)]) == 1
    assert 'No status sample with power_periph data' in capsys.readouterr().err


def test_at_without_from_log_is_an_error():
    with pytest.raises(SystemExit):
        cli.main(['--at', '+1'])


# --- live mode through a fake status reader --------------------------------------------------

class FakeReader:
    up = True
    payload = None
    reads = 0

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        pass

    def read(self, timeout=2.0):
        FakeReader.reads += 1
        if not FakeReader.up:
            return None
        return {'power_periph': FakeReader.payload, 'pimu': FakeReader.payload}


@pytest.fixture
def fake_client(monkeypatch):
    FakeReader.up = True
    FakeReader.payload = None
    FakeReader.reads = 0
    monkeypatch.setattr(cli, 'ServerStatusReader', FakeReader)

    def no_device(*a, **k):
        raise AssertionError('the CLI must not open a device')
    monkeypatch.setattr('serial.Serial', no_device, raising=False)
    return FakeReader


def test_server_down_says_so_and_exits_2(fake_client, capsys):
    fake_client.up = False
    assert cli.main([]) == 2
    err = capsys.readouterr().err
    assert 'stretch_body_server is not running' in err
    assert 'stretch_body_server --launch' in err
    assert '--from-log' in err


def test_live_uses_the_server_health(fake_client, capsys):
    st = status(**REALISTIC)
    st['power_health'] = evaluate(st)
    st['power_health']['summary'] = 'server summary'
    fake_client.payload = st
    assert cli.main(['--no-color']) == 0
    out = capsys.readouterr().out
    assert 'live from stretch_body_server,' in out
    assert 'server summary' in out
    assert fake_client.reads == 1


def test_live_computes_health_for_an_older_server(fake_client, capsys):
    fake_client.payload = status(**REALISTIC, adc_stale=1)
    assert cli.main(['--no-color', '--json']) == 0
    report = json.loads(capsys.readouterr().out)
    assert 'health computed here' in report['source']
    assert report['health']['level'] == 'warn'


def test_live_fault_exits_1(fake_client):
    st = status(**REALISTIC, capability_flags_raw=0x3F | (1 << 9) | (1 << 10) | (1 << 11))
    fake_client.payload = st
    assert cli.main(['--no-color']) == 1


def test_live_without_power_periph(fake_client, capsys):
    fake_client.payload = {}
    assert cli.main([]) == 2
    assert 'no power_periph status' in capsys.readouterr().err


def test_watch_refreshes_until_interrupted(fake_client, monkeypatch, capsys):
    fake_client.payload = status(**REALISTIC)
    sleeps = []

    def fake_sleep(s):
        sleeps.append(s)
        if len(sleeps) == 2:
            raise KeyboardInterrupt
    monkeypatch.setattr(cli.time, 'sleep', fake_sleep)
    assert cli.main(['--watch', '--interval', '0.5', '--no-color']) == 0
    assert sleeps == [0.5, 0.5]
    assert capsys.readouterr().out.count('Overall [ OK  ]') == 2
    assert fake_client.reads == 2


def test_cli_wrapper_and_entry_point():
    from stretch4_body import _cli_wrapper
    assert callable(_cli_wrapper.stretch_power_diagnostics)
    pyproject = (Path(__file__).parent.parent / 'pyproject.toml').read_text()
    assert ('stretch_power_diagnostics = "stretch4_body._cli_wrapper:stretch_power_diagnostics"'
            in pyproject)


def test_report_shows_a_tripped_eoa_output():
    st = status(**REALISTIC, eoa_tripped=True, power_to_eoa=False)
    text = cli.render_report(st, evaluate(st), 'live', events=0, now_wall=T0)
    assert 'Overall [WARN ] warn: eoa_output.' in text
    assert ('EOA output         tripped (overcurrent cut; clear the runstop or '
            'restore_eoa_power()), 0.00 A') in text
    assert cli.report_dict(st, evaluate(st), 'live')['key_values']['EOA output'].startswith('tripped')


def test_charging_text_names_where_the_charge_path_stops():
    on = {'adapter_voltage_present': True, 'esp_adapter_disagree': 0, 'voltage_36v0_mv': 36800,
          'charger_enable_command': True, 'charger_current_ma': 6500, 'battery_soc': 50}
    assert cli.charging_text(on) == 'charging, 6.5 A'
    assert cli.charging_text(dict(on, charger_current_ma=1600)).startswith(
        'voltage present but not charging (1.6 A)')
    assert cli.charging_text(dict(on, charger_current_ma=1600, battery_soc=95)).startswith('pack near full')
    assert cli.charging_text(dict(on, voltage_36v0_mv=33000)) == '36 V bus low at 33.0 V: contacts or supply'
    assert cli.charging_text(dict(on, esp_adapter_disagree=2)).startswith('charger on the 36 V bus but')
    off = dict(on, adapter_voltage_present=False, voltage_36v0_mv=500)
    assert cli.charging_text(off).startswith('no charger voltage at the robot')
    assert cli.charging_text(dict(off, esp_adapter_disagree=1)).startswith(
        'charger at the input, not on the 36 V bus (0.5 V): Q14 or U12')


def test_eoa_output_text_off_by_request_and_older_status():
    assert cli.eoa_output_text({'eoa_output_tripped': False,
                                'periph_power_state': {'power_to_eoa': False},
                                'current_eoa': 0.0}) == 'off by request, 0.00 A'
    assert cli.eoa_output_text({'periph_power_state': {'power_to_eoa': True}}) == 'on, n/a'


def test_live_only_subscribes_to_status_and_sends_nothing(capsys, monkeypatch):
    """A read must not change robot state: one SUB socket on the status port, no sends,
    no RobotClient (whose stop() pushes enable_safety and enable_freewheel_mode)."""
    import zmq
    from stretch4_body.core import client_server, server_status_reader
    from stretch4_body.core.subsystem_client import SubsystemClient
    st = status(**REALISTIC)
    st['power_health'] = evaluate(st)
    sockets = []

    class Socket:
        def __init__(self, kind):
            self.kind, self.sent, self.connected = kind, [], []

        def setsockopt(self, *a):
            pass

        def connect(self, addr):
            self.connected.append(addr)

        def poll(self, *a):
            return 1

        def recv_pyobj(self, flags=0):
            return {'power_periph': st}

        def close(self, linger=None):
            pass

        def __getattr__(self, name):
            if name.startswith('send'):
                return lambda *a, **k: self.sent.append(name)
            raise AttributeError(name)

    class Context:
        def socket(self, kind):
            sockets.append(Socket(kind))
            return sockets[-1]

    def no_command(*a, **k):
        raise AssertionError('a command was sent')
    monkeypatch.setattr(zmq.Context, 'instance', staticmethod(lambda: Context()))
    monkeypatch.setattr(client_server.StretchBodyClient, 'startup', no_command)
    monkeypatch.setattr(client_server.StretchBodyClient, '_do_send_cmd', no_command)
    monkeypatch.setattr(SubsystemClient, 'push_command', no_command)
    monkeypatch.setattr(server_status_reader.os.path, 'exists', lambda p: True)
    assert cli.main(['--no-color']) == 0
    assert 'live from stretch_body_server' in capsys.readouterr().out
    assert [s.kind for s in sockets] == [zmq.SUB]
    assert sockets[0].connected == ['ipc://%s' % client_server.PORT_STATUS]
    assert sockets[0].sent == []
