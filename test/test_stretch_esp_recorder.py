"""stretch_esp_recorder: a dump off a fake ESP console into a log that
stretch_power_diagnostics --from-log replays, the torn record left out, --print text.
Nothing here opens a port."""
import json
import struct

from stretch4_body.tools import stretch_esp_recorder as cli
from stretch4_body.tools import stretch_power_diagnostics

T_DUMP = 1781000600.0


def record(ms, sequence, kind=0, state=0x68, faults=0, cause=0, bus_v=26.4, bus_a=-1.3,
           nuc_a=1.2, event_id=0, data0=0, data1=0):
    """One FlightRecord as the PIMU packs it: state 0x68 is ACTIVE with PWR_EN,
    ROBOT_ACTIVE and NUC_5V high."""
    head = cli.HEAD.pack(ms, sequence, state, kind | (cause << 4), event_id, faults)
    if kind == cli.KIND_EVENT:
        body = cli.EVENT.pack(data0, data1) + b'\x00'
    else:
        body = cli.SAMPLE.pack(int(round(bus_v * 10)), int(round(bus_a * 10)), int(round(nuc_a * 50)))
    raw = head + body
    return raw + bytes([cli.crc8(raw)])


class FakePort:
    """What the ESP console writes after 'frec dump': chatter, blocks, the end mark."""

    def __init__(self, chunks):
        self.data = b''.join(chunks)
        self.written = b''

    def read_until(self, mark):
        i = self.data.find(mark)
        line, self.data = (self.data[:i + 1], self.data[i + 1:]) if i >= 0 else (self.data, b'')
        return line

    def read(self, n):
        out, self.data = self.data[:n], self.data[n:]
        return out

    def write(self, data):
        self.written += data

    def close(self):
        pass


def dump():
    """Twenty samples before a PIMU reboot, a state transition event and a burst after it,
    one record torn mid write."""
    before = [record(100000 + 50 * i, i) for i in range(20)]
    torn = bytearray(record(100000 + 50 * 20, 20))
    torn[9] ^= 0x40
    after = [record(50 * i, 21 + i, bus_v=26.1, bus_a=4.0) for i in range(4)]
    event = record(180, 25, kind=cli.KIND_EVENT, state=0x69, event_id=2, data0=0, data1=3)
    burst = [record(180 + 5 * i, 26 + i, kind=1, state=0xE9, cause=1, faults=0x0002)
             for i in range(8)]
    return [b'frec: 2097152 bytes at 0x290000, write position 1234\n',
            b'FREC %d\n' % 21, b''.join(before) + bytes(torn),
            b'FREC 1\n', after[0],
            b'FREC %d\n' % 12, b''.join(after[1:]) + event + b''.join(burst),
            b'FREC 0\n']


def test_dump_reads_blocks_skips_chatter_and_the_torn_record():
    records = cli.read_dump(FakePort(dump()))
    assert len(records) == 20 + 4 + 1 + 8
    assert [r['sequence'] for r in records[19:22]] == [19, 21, 22]
    assert records[0]['bus_v'] == 26.4 and records[0]['bus_a'] == -1.3
    assert records[0]['nuc_a'] == 1.2
    assert records[0]['pwr_en'] and records[0]['nuc_5v'] and not records[0]['button']
    ev = records[24]
    assert ev['kind'] == cli.KIND_EVENT and ev['event_id'] == 2 and ev['data1'] == 3
    assert records[-1]['runstop'] and records[-1]['runstop_cause'] == 1 and records[-1]['faults'] == 2


def test_wall_times_count_back_from_the_dump_and_nothing_across_a_reboot():
    records = cli.read_dump(FakePort(dump()))
    times = cli.wall_times(records, T_DUMP)
    assert times[-1] == T_DUMP
    assert abs(times[20] - (T_DUMP - 0.215)) < 1e-6
    # The last record before the reboot sits at the first one after it.
    assert times[19] == times[20]
    assert abs(times[0] - (T_DUMP - 0.215 - 0.95)) < 1e-6


def test_log_replays_in_stretch_power_diagnostics(tmp_path, capsys):
    records = cli.read_dump(FakePort(dump()))
    out = tmp_path / 'esp.json'
    out.write_text(json.dumps(cli.samples_of(records, T_DUMP)))
    assert stretch_power_diagnostics.main(['--from-log', str(out), '--no-color']) == 0
    text = capsys.readouterr().out
    assert 'sample 33 of 33' in text
    assert 'STATE_SLEEP (1)' in text
    assert '26.40 V, -1.30 A' in text
    assert 'power_state_transition (2): STATE_ACTIVE -> STATE_SLEEP' in text
    assert stretch_power_diagnostics.main(['--from-log', str(out), '--no-color', '--at', '-0.1']) == 0
    text = capsys.readouterr().out
    assert 'sample 23 of 33' in text
    assert '26.10 V, 4.00 A' in text


def test_main_writes_the_log_and_prints_records(tmp_path, capsys, monkeypatch):
    port = FakePort(dump())
    monkeypatch.setattr(cli, 'open_port', lambda path: port)
    monkeypatch.setattr(cli.time, 'time', lambda: T_DUMP)
    out = tmp_path / 'esp.json'
    assert cli.main(['--seconds', '30', '--out', str(out), '--print']) == 0
    assert port.written == b'frec dump 30\n'
    text = capsys.readouterr().out
    assert '33 records (1 power events)' in text
    assert 'power_state_transition (2): STATE_ACTIVE -> STATE_SLEEP' in text
    assert '26.4 V   -1.3 A  NUC 1.20 A  [PWR_EN ROBOT_ACTIVE NUC_5V; faults 0x0000]' in text
    assert 'runstop button, faults 0x0002' in text
    assert len(json.loads(out.read_text())) == 33


def test_stat_and_clear_pass_the_console_line_through(capsys, monkeypatch):
    port = FakePort([b'ota: idle\n', b'frec: 2097152 bytes at 0x290000, 512 sectors, position 0\n'])
    monkeypatch.setattr(cli, 'open_port', lambda path: port)
    assert cli.main(['--stat']) == 0
    assert port.written == b'frec stat\n'
    assert capsys.readouterr().out.startswith('frec: 2097152 bytes')
    port = FakePort([b'frec: cleared\n'])
    monkeypatch.setattr(cli, 'open_port', lambda path: port)
    assert cli.main(['--clear']) == 0
    assert port.written == b'frec clear\n'
