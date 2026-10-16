"""
Plain-language health of the PIMU power path, derived from the P14 diagnostics.

RobotServer publishes the result as status['power_periph']['power_health']. It is
recomputed when a 2 Hz diagnostics sample arrives and on a 2 Hz tick in between
(so a dead link turns stale without a new sample), never in the 100 Hz loop body.
stretch_power_diagnostics prints it, live or from a status log.

Each check judges one part of the power path and returns a level (ok, warn, fault,
unknown), one sentence a support engineer can act on, and the values behind it.
A field this firmware does not report gives 'unknown', never 'warn'.
Rules, thresholds and what to do when a check fires: docs/primer_power_diagnostics.md.
"""
from collections import deque
import time

from stretch4_body.subsystem.power_periph import PowerPeriphDefn as _Defn

LEVEL_OK = 'ok'
LEVEL_WARN = 'warn'
LEVEL_FAULT = 'fault'
LEVEL_UNKNOWN = 'unknown'
_SEVERITY = {LEVEL_OK: 0, LEVEL_UNKNOWN: 0, LEVEL_WARN: 1, LEVEL_FAULT: 2}

HEALTH_FORMAT = 1


class PowerHealthThresholds:
    """
    Every number the health rules use. Most mirror a firmware constant, so they are
    class constants rather than robot params: changing one on a single robot would
    not change what the PIMU does. The primer gives the reasoning for each.
    """
    # A diagnostics sample older than this is stale. The poll is 2 Hz and the server
    # gives up on a reply after 2 s (POWER_DIAGNOSTICS_RPC_TIMEOUT_S), so 2 s is four
    # missed polls. 10 s without a sample is a fault: the PIMU has stopped answering.
    LINK_STALE_WARN_S = 2.0
    LINK_STALE_FAULT_S = 10.0

    # (warning, fault) on the age of the last good BMS sample and of the last ESP
    # status, in seconds. RobotServer classifies bms_freshness and esp_freshness with
    # these. The ESP pair matches the firmware's 2 s ESP stale mark. The firmware's own
    # BMS decision gate is 5 s (low-battery and charger decisions held past it), which
    # sits between the host's warning and fault.
    BMS_FRESHNESS_S = (3.0, 10.0)
    ESP_FRESHNESS_S = (2.0, 5.0)

    # Counter rates are the increase over the last minute of samples.
    RATE_WINDOW_S = 60.0

    # The BMS is polled at 1 Hz. Three failed polls in a row is when the host
    # freshness turns 'warning' (3 s); 6 errors a minute is 10 percent of polls.
    BMS_CONSECUTIVE_FAILURES_WARN = 3
    BMS_ERRORS_PER_MIN_WARN = 6

    # ESP frames rejected by the length check or as unknown ids (v0.1.42p14). A healthy
    # link has none. The PIMU polls the ESP at 50 Hz, so 60 a minute (one a second)
    # means most frames of some kind are refused: an ESP build with a different
    # Esp_Status layout, which the PIMU then treats as no status at all.
    ESP_FRAME_ERRORS_PER_MIN_FAULT = 60
    # CRC errors, RX overflows and TX queue drops on the same UART, summed.
    ESP_LINK_ERRORS_PER_MIN_WARN = 10

    # An ADC bank is marked stale after 100 ms without a sweep and rebuilt after 1 s
    # (v0.1.39p14). Stale for 5 s means the rebuild is not recovering it.
    ADC_STALE_FAULT_S = 5.0

    # INA228 VBAT witness against the BMS pack voltage (v0.1.43p14). The two sit on
    # either side of the BMS FETs, fuse, R13 (5 mOhm) and the harness, and are sampled
    # up to 1 s apart, so they differ by the path drop under load plus both errors.
    # 500 mV covers about 10 A through 30 mOhm plus the BMS's own error; 2 V is beyond
    # any drop the robot's current can make, so one of the two readings is wrong.
    # A mismatch must hold for 3 samples (1.5 s at 2 Hz) to count, which rejects
    # load steps between the two sample instants. To be confirmed on the bench.
    VBAT_WARN_MV = 500
    VBAT_FAULT_MV = 2000
    VBAT_MISMATCH_SAMPLES = 3

    # A NUC shutdown wait past this fraction of the firmware's 90 s cap warns: the
    # NUC has not reported off and the rails will be cut under it.
    SHUTDOWN_WAIT_WARN_FRACTION = 0.8

    # Reset causes that mean the PIMU restarted itself rather than being powered on.
    SUSPECT_RESET_CAUSES = ('watchdog', 'brownout_core', 'brownout_vdd')


CHECK_TITLES = {
    'diagnostics_link': 'Diagnostics link',
    'firmware_build': 'Firmware build',
    'bms_freshness': 'BMS freshness',
    'bms_link_errors': 'BMS link errors',
    'esp_freshness': 'ESP freshness',
    'esp_link_errors': 'ESP UART errors',
    'esp_frame_errors': 'ESP frame rejects',
    'adc_banks': 'ADC banks',
    'adapter_source': 'Adapter source',
    'adapter_agreement': 'Adapter agreement',
    'vbat_cross_check': 'VBAT cross-check',
    'shutdown_wait': 'NUC shutdown wait',
    'shutdown_command': 'Shutdown command',
    'charge_stop': 'Charge stop',
    'rpc_rejections': 'RPC rejections',
    'pimu_reboots': 'PIMU reboots',
    'pimu_timing': 'PIMU loop timing',
    'event_log': 'Event log',
}
CHECK_IDS = tuple(CHECK_TITLES)

_NOT_REPORTED = 'Not reported by this firmware (schema 1, older than v0.1.47p14).'
_NO_SNAPSHOT = 'No diagnostics snapshot to judge yet.'


def worst_level(levels):
    """fault over warn over ok; unknown never raises the level."""
    worst = LEVEL_OK
    for level in levels:
        if _SEVERITY.get(level, 0) > _SEVERITY[worst]:
            worst = level
    return worst


def _check(check_id, level, message, values=None):
    # None values are left out: this dict rides in every 100 Hz status and every
    # logged status sample, so absent means 'not available' and costs nothing.
    values = {k: v for k, v in (values or {}).items() if v is not None}
    return {'id': check_id, 'level': level, 'message': message, 'values': values}


def _name(table, raw):
    return None if raw is None else table.get(raw, 'unknown_%d' % raw)


def _protocol_number(protocol):
    try:
        return int(protocol[1:]) if protocol.startswith('p') else None
    except (AttributeError, TypeError, ValueError):
        return None


class _CounterWindow:
    """Increase of a monotonic counter over the last window of samples."""

    def __init__(self, window_s):
        self.window_s = window_s
        self.samples = deque()

    def add(self, t, value):
        if value is None:
            self.samples.clear()
            return
        if self.samples and value < self.samples[-1][1]:
            # The counter went backwards: PIMU reboot or server restart. Start over.
            self.samples.clear()
        self.samples.append((t, value))
        start = t - self.window_s
        # Keep one sample at or before the window start as the baseline.
        while len(self.samples) >= 2 and self.samples[1][0] <= start:
            self.samples.popleft()

    def delta(self):
        if len(self.samples) < 2:
            return None
        return self.samples[-1][1] - self.samples[0][1]


class PowerHealthEvaluator:
    """
    Stateful because rates and persistence need history: counter windows, how long
    an ADC bank has been stale, consecutive VBAT mismatches. Feed it every published
    power_periph status you want judged, in time order; history only advances when
    power_diagnostics_meta['sample_seq'] changes.
    """

    # Diagnostics counters whose one-minute increase is reported.
    RATE_COUNTERS = (
        'bms_errors', 'esp_crc_errors', 'esp_rx_overflows', 'esp_tx_queue_drops',
        'esp_rx_length_errors', 'esp_rx_unknown_ids', 'adc_reinit', 'loop_overrun_count',
    )

    def __init__(self, thresholds=PowerHealthThresholds):
        self.t = thresholds
        self._windows = {k: _CounterWindow(thresholds.RATE_WINDOW_S) for k in self.RATE_COUNTERS}
        self._rpc_windows = {k: _CounterWindow(thresholds.RATE_WINDOW_S)
                             for k in ('rpc_rejected', 'rpc_reply_errors')}
        self._last_sample_seq = None
        self._last_rpc_t = None
        self._adc_stale_since = None
        self._adc_stale_for_s = None
        self._vbat_mismatches = 0

    # ------------------------------------------------------------------ public

    def evaluate(self, power_periph_status, now_s, wall_time_s=None):
        """
        power_periph_status: status['power_periph'] as the server publishes it.
        now_s: a clock in seconds for rate windows (monotonic on the server, the
        sample timestamp when replaying a log). Returns the power_health dict.
        """
        st = power_periph_status or {}
        meta = st.get('power_diagnostics_meta') or {}
        diag = st.get('power_diagnostics') or {}
        have = bool(diag.get('_valid'))
        seq = meta.get('sample_seq')
        if have and seq and seq != self._last_sample_seq:
            self._last_sample_seq = seq
            self._ingest_sample(diag, meta, now_s)
        self._ingest_rpc(st, now_s)

        link = self._check_link(st, meta, diag)
        checks = [link]
        if have:
            checks += [
                self._check_firmware_build(diag),
                self._check_bms_freshness(meta, diag),
                self._check_bms_link(diag),
                self._check_esp_freshness(meta, diag),
                self._check_esp_link(diag),
                self._check_esp_frames(diag),
                self._check_adc(diag),
                self._check_adapter_source(diag),
                self._check_adapter_agreement(diag),
                self._check_vbat(meta, diag),
                self._check_shutdown_wait(diag),
                self._check_shutdown_command(diag),
                self._check_charge_stop(diag),
            ]
        else:
            checks += [_check(i, LEVEL_UNKNOWN, _NO_SNAPSHOT) for i in CHECK_IDS[1:14]]
        checks.append(self._check_rpc(st))
        if have:
            checks += [self._check_reboots(meta, diag), self._check_timing(diag),
                       self._check_event_log(st.get('power_event_log') or {})]
        else:
            checks += [_check(i, LEVEL_UNKNOWN, _NO_SNAPSHOT) for i in CHECK_IDS[15:]]

        counts = {LEVEL_OK: 0, LEVEL_WARN: 0, LEVEL_FAULT: 0, LEVEL_UNKNOWN: 0}
        for c in checks:
            counts[c['level']] += 1
        level = worst_level(c['level'] for c in checks)
        if level == LEVEL_OK and link['level'] != LEVEL_OK:
            level = LEVEL_UNKNOWN
        return {
            'format': HEALTH_FORMAT,
            'level': level,
            'summary': self._summary(level, checks, counts, link),
            'evaluated_wall_time_s': time.time() if wall_time_s is None else wall_time_s,
            'sample_seq': seq,
            'counts': counts,
            'checks': checks,
        }

    # ------------------------------------------------------------------ history

    def _ingest_sample(self, diag, meta, t):
        bms_errors = sum(diag.get(k) or 0 for k in (
            'bms_crc_errors', 'bms_timeouts', 'bms_rx_overflows', 'bms_protocol_errors'))
        reinit = diag.get('adc_reinit_cnt')
        values = {
            'bms_errors': bms_errors,
            'esp_crc_errors': diag.get('esp_crc_errors'),
            'esp_rx_overflows': diag.get('esp_rx_overflows'),
            'esp_tx_queue_drops': diag.get('esp_tx_queue_drops'),
            'esp_rx_length_errors': diag.get('esp_rx_length_errors'),
            'esp_rx_unknown_ids': diag.get('esp_rx_unknown_ids'),
            'adc_reinit': None if reinit is None else sum(reinit),
            'loop_overrun_count': diag.get('loop_overrun_count'),
        }
        for key, value in values.items():
            self._windows[key].add(t, value)

        stale = diag.get('adc_stale')
        if stale:
            if self._adc_stale_since is None:
                self._adc_stale_since = t
            self._adc_stale_for_s = t - self._adc_stale_since
        else:
            self._adc_stale_since = None
            self._adc_stale_for_s = None

        diff = self._vbat_difference(meta, diag)
        if diff is not None and abs(diff) >= self.t.VBAT_WARN_MV:
            self._vbat_mismatches += 1
        else:
            self._vbat_mismatches = 0

    def _ingest_rpc(self, st, t):
        if self._last_rpc_t is not None and t <= self._last_rpc_t:
            return
        self._last_rpc_t = t
        rejected = (st.get('rpc_rejected') or {}).get('count')
        self._rpc_windows['rpc_rejected'].add(t, rejected or 0)
        self._rpc_windows['rpc_reply_errors'].add(t, st.get('rpc_reply_errors') or 0)

    def _rate(self, key):
        return self._windows[key].delta()

    # ------------------------------------------------------------------ checks

    def _check_link(self, st, meta, diag):
        cid = 'diagnostics_link'
        schema = diag.get('schema_version') if diag.get('_valid') else None
        age = meta.get('sample_age_s')
        error = meta.get('transport_error') or meta.get('schema_error')
        protocol = meta.get('protocol_version')
        values = {
            'available': bool(meta.get('available')),
            'sample_age_s': age,
            'schema_version': schema,
            'firmware_version': meta.get('firmware_version'),
            'protocol_version': protocol,
            'request_errors': meta.get('request_errors'),
            'request_timeouts': meta.get('request_timeouts'),
            'error': error,
        }
        number = _protocol_number(protocol)
        if number is not None and number < 14:
            return _check(cid, LEVEL_UNKNOWN,
                          'Firmware protocol %s has no diagnostics RPC; only the aux status is '
                          'read. Update the PIMU to v0.1.47p14 or later.' % protocol, values)
        if not meta.get('sample_seq'):
            if meta.get('schema_error'):
                return _check(cid, LEVEL_WARN, 'Diagnostics replies are rejected: %s.'
                              % str(meta['schema_error']).rstrip('.'), values)
            return _check(cid, LEVEL_UNKNOWN,
                          'No diagnostics sample yet: the server is starting or the PIMU is '
                          'not answering RPC 31.', values)
        if not meta.get('available') or age is None or age >= self.t.LINK_STALE_WARN_S:
            level = (LEVEL_FAULT if age is not None and age >= self.t.LINK_STALE_FAULT_S
                     else LEVEL_WARN)
            return _check(cid, level,
                          'No fresh diagnostics for %s (%s); the other checks use the last '
                          'sample.' % ('%.1f s' % age if age is not None else 'an unknown time',
                                       error or 'no reply'), values)
        if schema == 1:
            return _check(cid, LEVEL_OK,
                          'Diagnostics arriving at 2 Hz on schema 1 (firmware older than '
                          'v0.1.47p14); fields added from v0.1.24p14 on are not reported.', values)
        return _check(cid, LEVEL_OK,
                      'Diagnostics arriving at 2 Hz on schema %s, last sample %.1f s old.'
                      % (schema, age), values)

    def _check_firmware_build(self, diag):
        caps = diag.get('capabilities') or {}
        values = {'capability_flags_raw': diag.get('capability_flags_raw')}
        if caps.get('bench_rig_build'):
            return _check('firmware_build', LEVEL_FAULT,
                          'Bench-rig firmware (never shuts down on low battery) is running; a '
                          'fleet robot must never report this, reflash the release build.', values)
        if caps.get('simulated_battery'):
            return _check('firmware_build', LEVEL_WARN,
                          'RDK firmware with a simulated battery is running; battery readings '
                          'are not real.', values)
        return _check('firmware_build', LEVEL_OK, 'Release firmware build.', values)

    @staticmethod
    def _freshness_check(cid, who, freshness, age, warn_s, fault_s, consequence, values):
        if freshness == 'fresh':
            return _check(cid, LEVEL_OK, 'Last %s %.1f s ago.' % (who, age or 0.0), values)
        if freshness == 'warning':
            return _check(cid, LEVEL_WARN, '%s is %.1f s old (warning at %.0f s): %s.'
                          % (who[0].upper() + who[1:], age or 0.0, warn_s, consequence), values)
        if freshness == 'fault':
            return _check(cid, LEVEL_FAULT, '%s is %.1f s old (fault at %.0f s): %s.'
                          % (who[0].upper() + who[1:], age or 0.0, fault_s, consequence), values)
        if freshness == 'unavailable':
            return _check(cid, LEVEL_FAULT, 'No %s since PIMU boot.' % who, values)
        if freshness == 'stale':
            return _check(cid, LEVEL_UNKNOWN,
                          'Diagnostics link is stale, so the %s age is unknown.' % who, values)
        return _check(cid, LEVEL_UNKNOWN, 'Not classified yet.', values)

    def _check_bms_freshness(self, meta, diag):
        warn_s, fault_s = self.t.BMS_FRESHNESS_S
        values = {'bms_freshness': meta.get('bms_freshness'), 'bms_age_s': meta.get('bms_age_s'),
                  'bms_valid_replies': diag.get('bms_valid_replies'),
                  'bms_sample_sequence': diag.get('bms_sample_sequence')}
        check = self._freshness_check(
            'bms_freshness', 'valid BMS sample', meta.get('bms_freshness'), meta.get('bms_age_s'),
            warn_s, fault_s, 'the BMS UART is missing polls, check the pack data lead', values)
        if meta.get('bms_freshness') == 'unavailable':
            check['message'] = ('No valid BMS sample since PIMU boot: check the pack, its data '
                                'lead and the BMS.')
        return check

    def _check_bms_link(self, diag):
        validity = diag.get('validity') or {}
        last = self._rate('bms_errors')
        consecutive = diag.get('bms_consecutive_failures') or 0
        total = sum(diag.get(k) or 0 for k in (
            'bms_crc_errors', 'bms_timeouts', 'bms_rx_overflows', 'bms_protocol_errors'))
        values = {k: diag.get(k) for k in (
            'bms_crc_errors', 'bms_timeouts', 'bms_rx_overflows', 'bms_protocol_errors',
            'bms_consecutive_failures')}
        values['errors_last_min'] = last
        if consecutive:
            values['bms_last_failure_event'] = _name(_Defn.POWER_EVENT_NAMES,
                                                     diag.get('bms_last_failure_event') or None)
        if validity.get('bms_sample') and not validity.get('bms_extended_sample'):
            return _check('bms_link_errors', LEVEL_WARN,
                          'Only short BMS replies so far: cell temperatures, alarms and MOS '
                          'states are not populated.', values)
        if consecutive >= self.t.BMS_CONSECUTIVE_FAILURES_WARN:
            return _check('bms_link_errors', LEVEL_WARN,
                          'The BMS has failed %d polls in a row (last failure: %s).'
                          % (consecutive, values.get('bms_last_failure_event') or 'unknown'),
                          values)
        if last is not None and last >= self.t.BMS_ERRORS_PER_MIN_WARN:
            return _check('bms_link_errors', LEVEL_WARN,
                          '%d BMS poll errors in the last minute (warn at %d): the BMS UART is '
                          'noisy or the pack is slow to answer.'
                          % (last, self.t.BMS_ERRORS_PER_MIN_WARN), values)
        if last:
            return _check('bms_link_errors', LEVEL_OK,
                          '%d BMS poll error%s in the last minute, below the %d limit; %d since '
                          'boot.' % (last, '' if last == 1 else 's',
                                     self.t.BMS_ERRORS_PER_MIN_WARN, total), values)
        return _check('bms_link_errors', LEVEL_OK,
                      'No BMS poll error in the last minute; %d since boot.' % total, values)

    def _check_esp_freshness(self, meta, diag):
        warn_s, fault_s = self.t.ESP_FRESHNESS_S
        values = {'esp_freshness': meta.get('esp_freshness'), 'esp_age_s': meta.get('esp_age_s'),
                  'esp_rx_packets': diag.get('esp_rx_packets')}
        return self._freshness_check(
            'esp_freshness', 'ESP status', meta.get('esp_freshness'), meta.get('esp_age_s'),
            warn_s, fault_s,
            'the 12 V and 20 V readings and cpu_on_status are not current', values)

    def _check_esp_link(self, diag):
        keys = ('esp_crc_errors', 'esp_rx_overflows', 'esp_tx_queue_drops')
        values = {}
        recent = 0
        for key in keys:
            values[key] = diag.get(key)
            values[key + '_last_min'] = self._rate(key)
            recent += self._rate(key) or 0
        if recent >= self.t.ESP_LINK_ERRORS_PER_MIN_WARN:
            return _check('esp_link_errors', LEVEL_WARN,
                          '%d ESP UART errors (CRC, overflow, queue drop) in the last minute '
                          '(warn at %d).' % (recent, self.t.ESP_LINK_ERRORS_PER_MIN_WARN), values)
        total = sum(diag.get(k) or 0 for k in keys)
        return _check('esp_link_errors', LEVEL_OK,
                      '%d ESP UART error%s in the last minute; %d since boot.'
                      % (recent, '' if recent == 1 else 's', total), values)

    def _check_esp_frames(self, diag):
        if diag.get('esp_rx_length_errors') is None:
            return _check('esp_frame_errors', LEVEL_UNKNOWN, _NOT_REPORTED)
        length_last = self._rate('esp_rx_length_errors')
        unknown_last = self._rate('esp_rx_unknown_ids')
        values = {
            'esp_rx_length_errors': diag['esp_rx_length_errors'],
            'esp_rx_length_errors_last_min': length_last,
            'esp_rx_unknown_ids': diag.get('esp_rx_unknown_ids'),
            'esp_rx_unknown_ids_last_min': unknown_last,
            'esp_tx_mark_resets': diag.get('esp_tx_mark_resets'),
        }
        total = (diag['esp_rx_length_errors'] or 0) + (diag.get('esp_rx_unknown_ids') or 0)
        recent = (length_last or 0) + (unknown_last or 0)
        if recent >= self.t.ESP_FRAME_ERRORS_PER_MIN_FAULT:
            return _check('esp_frame_errors', LEVEL_FAULT,
                          '%d ESP frames rejected in the last minute: the ESP firmware sends '
                          'frames this PIMU does not accept, so its status counts as missing. '
                          'Check the hello_esp version.' % recent, values)
        if diag.get('esp_tx_mark_resets'):
            return _check('esp_frame_errors', LEVEL_WARN,
                          'The ESP transmit ring mark was released by its watchdog %d times; no '
                          'known path does this, report it with this output.'
                          % diag['esp_tx_mark_resets'], values)
        if recent:
            return _check('esp_frame_errors', LEVEL_WARN,
                          '%d ESP frame%s rejected in the last minute (length check or unknown '
                          'id); a healthy link has none.' % (recent, '' if recent == 1 else 's'),
                          values)
        if length_last is None:
            return _check('esp_frame_errors', LEVEL_OK,
                          'No rate yet (first sample); %d frames rejected since boot.' % total,
                          values)
        return _check('esp_frame_errors', LEVEL_OK,
                      'No ESP frame rejected in the last minute; %d since boot.' % total, values)

    _ADC_BANKS = ((1, 'ADC0 (5 V, 36VDD, temperature)'),
                  (2, 'ADC1 (charger, 20 V, CPU and EOA current)'))

    def _check_adc(self, diag):
        stale = diag.get('adc_stale')
        if stale is None:
            return _check('adc_banks', LEVEL_UNKNOWN, _NOT_REPORTED)
        reinit = diag.get('adc_reinit_cnt') or [0, 0]
        last = self._rate('adc_reinit')
        values = {'adc_stale': stale, 'adc_reinit_cnt': list(reinit),
                  'adc_reinit_last_min': last, 'adc_stale_for_s': self._adc_stale_for_s,
                  'analog_rails_valid': (diag.get('validity') or {}).get('analog_rails')}
        banks = ' and '.join(name for bit, name in self._ADC_BANKS if stale & bit)
        if stale and (self._adc_stale_for_s or 0) >= self.t.ADC_STALE_FAULT_S:
            return _check('adc_banks', LEVEL_FAULT,
                          '%s has produced no sweep for %.0f s and the 1 s rebuild is not '
                          'recovering it: its readings are held, not measured.'
                          % (banks, self._adc_stale_for_s), values)
        if stale:
            return _check('adc_banks', LEVEL_WARN,
                          '%s is stale: its readings are held at the last value until the '
                          'watchdog rebuilds it.' % banks, values)
        if last:
            return _check('adc_banks', LEVEL_WARN,
                          '%d ADC bank rebuild%s in the last minute: this board\'s ADC sequencer '
                          'is stalling.' % (last, '' if last == 1 else 's'), values)
        if reinit[0] or reinit[1]:
            return _check('adc_banks', LEVEL_OK,
                          'No ADC stall in the last minute; rebuilds since boot: ADC0 %d, ADC1 %d.'
                          % (reinit[0], reinit[1]), values)
        return _check('adc_banks', LEVEL_OK, 'Both ADC banks sweeping; no rebuild since boot.',
                      values)

    def _check_adapter_source(self, diag):
        source = diag.get('adapter_presence_source')
        if source is None:
            return _check('adapter_source', LEVEL_UNKNOWN, _NOT_REPORTED)
        values = {'adapter_presence_source': source,
                  'adapter_presence_source_name': diag.get('adapter_presence_source_name'),
                  'adapter_voltage_present': diag.get('adapter_voltage_present'),
                  'esp_adapter_connected': diag.get('esp_adapter_connected')}
        if source == 0:
            return _check('adapter_source', LEVEL_OK,
                          'Adapter presence comes from the PIMU\'s own 36VDD measurement.', values)
        if source == 1:
            return _check('adapter_source', LEVEL_WARN,
                          'ADC0 is stale, so adapter presence is taken from the ESP '
                          'adapter_connected bit (fallback).', values)
        if source == 2:
            return _check('adapter_source', LEVEL_WARN,
                          'ADC0 is stale and ESP status is not fresh: adapter presence is held '
                          'at its last value.', values)
        return _check('adapter_source', LEVEL_WARN,
                      'Unknown adapter presence source %d.' % source, values)

    def _check_adapter_agreement(self, diag):
        mask = diag.get('esp_adapter_disagree')
        if mask is None:
            return _check('adapter_agreement', LEVEL_UNKNOWN, _NOT_REPORTED)
        values = {'esp_adapter_disagree': mask,
                  'esp_adapter_disagree_name': diag.get('esp_adapter_disagree_name'),
                  'voltage_36v0_mv': diag.get('voltage_36v0_mv'),
                  'esp_adapter_connected': diag.get('esp_adapter_connected')}
        if mask == 0:
            return _check('adapter_agreement', LEVEL_OK,
                          'The ESP adapter_connected bit and the 36 V input agree.', values)
        if mask == 1:
            return _check('adapter_agreement', LEVEL_WARN,
                          'The ESP reports an adapter but the PIMU has measured no 36 V for over '
                          '1 s: check the dock contacts and the 36VDD path.', values)
        if mask == 2:
            return _check('adapter_agreement', LEVEL_WARN,
                          'The PIMU measures 36 V but the ESP adapter_connected bit has been '
                          'clear for over 1 s: check the adapter detect input on the ESP side.',
                          values)
        return _check('adapter_agreement', LEVEL_WARN,
                      'Adapter disagreement mask %d (both directions at once).' % mask, values)

    def _vbat_difference(self, meta, diag):
        """INA228 minus BMS pack voltage in mV, or None when the two cannot be compared."""
        caps = diag.get('capabilities') or {}
        validity = diag.get('validity') or {}
        ina = diag.get('ina_vbat_mv')
        flags = diag.get('ina_flags') or 0
        if (ina is None or not caps.get('ina_vbat') or caps.get('simulated_battery') or
                not (flags & 1) or not (flags & 2) or not validity.get('ina_sample')):
            return None
        if not self._bms_fresh_at_sample(meta, diag):
            return None
        return ina - diag.get('battery_voltage_mv', 0)

    @staticmethod
    def _bms_fresh_at_sample(meta, diag):
        at_sample = meta.get('bms_freshness_at_sample')
        if at_sample is not None:
            return at_sample == 'fresh'
        age = diag.get('bms_sample_age_s')
        return age is not None and age < 3.0

    def _check_vbat(self, meta, diag):
        cid = 'vbat_cross_check'
        if diag.get('ina_vbat_mv') is None:
            return _check(cid, LEVEL_UNKNOWN, _NOT_REPORTED)
        caps = diag.get('capabilities') or {}
        validity = diag.get('validity') or {}
        flags = diag.get('ina_flags') or 0
        values = {'ina_vbat_mv': diag.get('ina_vbat_mv'),
                  'battery_voltage_mv': diag.get('battery_voltage_mv'),
                  'difference_mv': None, 'consecutive_mismatches': self._vbat_mismatches,
                  'ina_current_ma': diag.get('ina_current_ma'),
                  'battery_current_ma': diag.get('battery_current_ma'), 'ina_flags': flags}
        if caps.get('simulated_battery'):
            return _check(cid, LEVEL_UNKNOWN,
                          'RDK build simulates the battery; cross-check skipped.', values)
        if not caps.get('ina_vbat'):
            return _check(cid, LEVEL_UNKNOWN,
                          'This firmware build does not report the INA228 witness.', values)
        if not flags & 1:
            return _check(cid, LEVEL_WARN,
                          'The INA228 did not answer its device id at boot, so there is no '
                          'VBAT witness: check U2 and its I2C bus.', values)
        if not flags & 2 or not validity.get('ina_sample'):
            return _check(cid, LEVEL_WARN,
                          'The last INA228 read failed although the part answered at boot: '
                          'check the I2C pull-ups and clock.', values)
        if not self._bms_fresh_at_sample(meta, diag):
            return _check(cid, LEVEL_UNKNOWN,
                          'The BMS sample was not fresh at this snapshot; cross-check skipped.',
                          values)
        diff = diag['ina_vbat_mv'] - diag.get('battery_voltage_mv', 0)
        values['difference_mv'] = diff
        if abs(diff) < self.t.VBAT_WARN_MV:
            return _check(cid, LEVEL_OK,
                          'INA228 VBAT %.2f V and BMS pack %.2f V agree within %d mV (limit %d).'
                          % (diag['ina_vbat_mv'] / 1000.0, diag['battery_voltage_mv'] / 1000.0,
                             abs(diff), self.t.VBAT_WARN_MV), values)
        if self._vbat_mismatches < self.t.VBAT_MISMATCH_SAMPLES:
            return _check(cid, LEVEL_OK,
                          'INA228 and BMS differ by %d mV on %d sample%s only; treated as a load '
                          'transient until it holds for %d samples.'
                          % (diff, self._vbat_mismatches, '' if self._vbat_mismatches == 1 else 's',
                             self.t.VBAT_MISMATCH_SAMPLES), values)
        if abs(diff) >= self.t.VBAT_FAULT_MV:
            return _check(cid, LEVEL_FAULT,
                          'INA228 VBAT and BMS pack voltage differ by %d mV (fault at %d): one '
                          'of the two readings is wrong, measure VBAT with a DMM.'
                          % (diff, self.t.VBAT_FAULT_MV), values)
        return _check(cid, LEVEL_WARN,
                      'INA228 VBAT and BMS pack voltage differ by %d mV for %d samples (warn at '
                      '%d): compare both with a DMM on VBAT.'
                      % (diff, self._vbat_mismatches, self.t.VBAT_WARN_MV), values)

    _WAIT_END_TEXT = {
        0: 'nuc_safe_shutdown is off, so the rails drop without waiting for the NUC',
        1: 'the last NUC shutdown wait ended with the NUC off after %.1f s',
        3: 'the last NUC shutdown wait was cancelled by a wake after %.1f s',
        255: 'no NUC shutdown wait since PIMU boot',
    }

    def _check_shutdown_wait(self, diag):
        cid = 'shutdown_wait'
        if diag.get('shutdown_wait_last_end') is None:
            return _check(cid, LEVEL_UNKNOWN, _NOT_REPORTED)
        cap_ms = diag.get('shutdown_wait_cap_ms') or 90000
        elapsed_ms = diag.get('shutdown_wait_elapsed_ms') or 0
        last_end = diag['shutdown_wait_last_end']
        last_s = (diag.get('shutdown_wait_last_duration_ms') or 0) / 1000.0
        values = {k: diag.get(k) for k in (
            'shutdown_wait_active', 'shutdown_wait_elapsed_ms', 'shutdown_wait_cap_ms',
            'shutdown_wait_count', 'shutdown_wait_last_end', 'shutdown_wait_last_end_name',
            'shutdown_wait_last_duration_ms')}
        if diag.get('shutdown_wait_active'):
            if elapsed_ms >= self.t.SHUTDOWN_WAIT_WARN_FRACTION * cap_ms:
                return _check(cid, LEVEL_WARN,
                              'Rails held for the NUC for %.1f s of the %.0f s cap: the NUC has '
                              'not reported off and the PIMU will cut the rails at the cap.'
                              % (elapsed_ms / 1000.0, cap_ms / 1000.0), values)
            return _check(cid, LEVEL_OK,
                          'Rails held while the NUC shuts down: %.1f s of the %.0f s cap.'
                          % (elapsed_ms / 1000.0, cap_ms / 1000.0), values)
        if last_end == 2:
            return _check(cid, LEVEL_WARN,
                          'The last NUC shutdown wait ran to the %.0f s cap: the NUC never '
                          'reported off and its rails were cut under it.' % (cap_ms / 1000.0),
                          values)
        if last_end == 4:
            return _check(cid, LEVEL_WARN,
                          'The last NUC shutdown wait ended at the cap with only stale or absent '
                          'ESP status saying the NUC was off, so the NUC state was never '
                          'confirmed.', values)
        text = self._WAIT_END_TEXT.get(last_end)
        if text is None:
            return _check(cid, LEVEL_WARN, 'Unknown shutdown wait end %d.' % last_end, values)
        text = text % last_s if '%' in text else text
        return _check(cid, LEVEL_OK, text[0].upper() + text[1:] + '.', values)

    def _check_shutdown_command(self, diag):
        cid = 'shutdown_command'
        if diag.get('wait_sd_ack') is None:
            return _check(cid, LEVEL_UNKNOWN, _NOT_REPORTED)
        sends = diag.get('esp_shutdown_cmd_sends') or 0
        state = diag.get('power_state')
        values = {'wait_sd_ack': diag['wait_sd_ack'], 'esp_shutdown_cmd_sends': sends,
                  'power_state': state, 'power_state_name': diag.get('power_state_name')}
        if diag['wait_sd_ack'] and state == 0:
            return _check(cid, LEVEL_WARN,
                          'wait_sd_ack is set in ACTIVE although every transition clears it; '
                          'report it with this output.', values)
        if diag['wait_sd_ack']:
            return _check(cid, LEVEL_OK,
                          'Resending the shutdown command to the ESP every 100 ms: %d sends so '
                          'far (from v0.1.48p14 it resends for the whole dwell, up to 90 s).'
                          % sends, values)
        if sends:
            return _check(cid, LEVEL_OK,
                          'Shutdown command resend finished after %d sends.' % sends, values)
        return _check(cid, LEVEL_OK, 'No shutdown command pending.', values)

    _CHARGE_STOP_TEXT = {
        0: 'Charge-stop policy idle: the charger runs under the legacy 28 V / 26.8 V rules.',
        1: 'Charge-stop policy arming: the BMS high-voltage alarm is asserted and the 5 s '
           'sustain is counting.',
        2: 'Charge-stop policy holding: the firmware has disabled the charger on a sustained '
           'BMS high-voltage alarm (10 min dwell).',
    }

    def _check_charge_stop(self, diag):
        cid = 'charge_stop'
        raw = diag.get('charge_stop_state')
        if raw is None:
            return _check(cid, LEVEL_UNKNOWN, _NOT_REPORTED)
        values = {'charge_stop_state': raw,
                  'charge_stop_state_name': diag.get('charge_stop_state_name'),
                  'charge_stop_enabled': diag.get('charge_stop_enabled'),
                  'charger_enable_command': diag.get('charger_enable_command'),
                  'charger_current_ma': diag.get('charger_current_ma')}
        if not diag.get('charge_stop_enabled'):
            return _check(cid, LEVEL_OK,
                          'The charge-stop policy is compiled out of this build; the legacy '
                          '28 V / 26.8 V rules apply.', values)
        text = self._CHARGE_STOP_TEXT.get(raw & 0x0F)
        if text is None:
            return _check(cid, LEVEL_WARN, 'Unknown charge-stop state %d.' % (raw & 0x0F), values)
        return _check(cid, LEVEL_OK, text, values)

    def _check_rpc(self, st):
        rejected = st.get('rpc_rejected') or {}
        count = rejected.get('count') or 0
        errors = st.get('rpc_reply_errors') or 0
        rej_last = self._rpc_windows['rpc_rejected'].delta()
        err_last = self._rpc_windows['rpc_reply_errors'].delta()
        reason = rejected.get('last_reason')
        values = {'rpc_rejected_count': count, 'rpc_rejected_last_min': rej_last,
                  'last_request_id': rejected.get('last_request_id'),
                  'last_reason': _name(_Defn.RPC_REJECT_REASON_NAMES, reason),
                  'rpc_reply_errors': errors, 'rpc_reply_errors_last_min': err_last,
                  'rpc_reply_last_error': st.get('rpc_reply_last_error')}
        if rej_last:
            return _check('rpc_rejections', LEVEL_WARN,
                          'The PIMU rejected RPC %s (%s) in the last minute: the body asked for '
                          'something this firmware does not know, check the version pairing.'
                          % (rejected.get('last_request_id'), values['last_reason']), values)
        if err_last:
            return _check('rpc_rejections', LEVEL_WARN,
                          '%d malformed PIMU replies in the last minute (%s).'
                          % (err_last, st.get('rpc_reply_last_error') or 'no detail'), values)
        if count or errors:
            return _check('rpc_rejections', LEVEL_OK,
                          'None in the last minute; since the server started: %d rejected, %d '
                          'malformed.' % (count, errors), values)
        return _check('rpc_rejections', LEVEL_OK,
                      'No RPC rejected or malformed since the server started.', values)

    def _check_reboots(self, meta, diag):
        count = meta.get('pimu_boot_count_observed') or 0
        causes = diag.get('reset_causes') or []
        uptime_ms = diag.get('uptime_ms')
        values = {'pimu_boot_count_observed': count, 'reset_causes': list(causes),
                  'uptime_s': None if uptime_ms is None else round(uptime_ms / 1000.0, 1)}
        cause_text = ', '.join(causes) or 'none recorded'
        if count:
            return _check('pimu_reboots', LEVEL_WARN,
                          'The PIMU rebooted %d time%s while the server ran (last reset cause: '
                          '%s); see event 1 in the log.' % (count, '' if count == 1 else 's',
                                                           cause_text), values)
        suspect = [c for c in causes if c in self.t.SUSPECT_RESET_CAUSES]
        if suspect:
            return _check('pimu_reboots', LEVEL_WARN,
                          'The PIMU last restarted from %s, not from power-on.'
                          % ', '.join(suspect), values)
        hours = (uptime_ms or 0) / 3600000.0
        return _check('pimu_reboots', LEVEL_OK,
                      'No PIMU reboot since the server started; up %.1f h, reset cause %s.'
                      % (hours, cause_text), values)

    def _check_timing(self, diag):
        last = self._rate('loop_overrun_count')
        values = {k: diag.get(k) for k in ('loop_time_max_us', 'loop_overrun_count')}
        values['loop_overrun_last_min'] = last
        if last:
            return _check('pimu_timing', LEVEL_WARN,
                          'The PIMU 250 Hz handler ran past 4 ms %d time%s in the last minute '
                          '(max %s us).' % (last, '' if last == 1 else 's',
                                            diag.get('loop_time_max_us')), values)
        return _check('pimu_timing', LEVEL_OK,
                      'No PIMU control-loop overrun in the last minute; max handler time %s us.'
                      % diag.get('loop_time_max_us'), values)

    def _check_event_log(self, log):
        values = {k: log.get(k) for k in ('cursor_sequence', 'firmware_dropped_count',
                                          'coverage_gap_count', 'reboot_pages_discarded')}
        if not log.get('available'):
            return _check('event_log', LEVEL_UNKNOWN,
                          'Event log not read yet (%s).' % (log.get('schema_error') or
                                                            'no page'), values)
        gaps = log.get('coverage_gap_count') or 0
        if gaps:
            last = (log.get('coverage_gaps') or [{}])[-1]
            return _check('event_log', LEVEL_WARN,
                          'The host missed events %d time%s; the last gap is after sequence %s, '
                          'resuming at %s.' % (gaps, '' if gaps == 1 else 's',
                                               last.get('after_sequence'),
                                               last.get('resumed_sequence')), values)
        dropped = log.get('firmware_dropped_count') or 0
        if dropped:
            return _check('event_log', LEVEL_OK,
                          'No event missed; the firmware ring has overwritten %d old entries, '
                          'all read before they went.' % dropped, values)
        return _check('event_log', LEVEL_OK, 'No event missed and no firmware drop.', values)

    @staticmethod
    def _summary(level, checks, counts, link):
        if level in (LEVEL_WARN, LEVEL_FAULT):
            parts = []
            for lv in (LEVEL_FAULT, LEVEL_WARN):
                ids = [c['id'] for c in checks if c['level'] == lv]
                if ids:
                    parts.append('%s: %s' % (lv, ', '.join(ids)))
            return '; '.join(parts) + '.'
        if level == LEVEL_UNKNOWN:
            return link['message']
        unknown = counts[LEVEL_UNKNOWN]
        return 'All %d judged checks ok%s.' % (
            counts[LEVEL_OK], '' if not unknown else ', %d not reported by this firmware' % unknown)



def health_line(power_health):
    """
    (level, one line) for tools that show a single row per subsystem, such as
    stretch_system_check, or None when the server publishes no power_health.
    """
    if not isinstance(power_health, dict) or not power_health.get('checks'):
        return None
    level = power_health.get('level', LEVEL_UNKNOWN)
    text = 'P14 power health: %s' % power_health.get('summary', '')
    if level != LEVEL_OK:
        text += ' Details: stretch_power_diagnostics'
    return level, text

# ---------------------------------------------------------------------- events

_RESTART_REASONS = ((0x01, 'alarm clear and SoC below 90'), (0x02, 'alarm clear and pack below 26.8 V'),
                    (0x04, 'policy disabled or host took charger control'),
                    (0x08, 'adapter removed'))
_STOP_REASONS = ((0x01, 'SoC at or above 85'), (0x02, 'cell max at or above 3450 mV'))
_WAIT_CAUSE = {0: 'no wait configured', 1: 'NUC off', 2: 'cap reached',
               4: 'cap reached with only stale "off" status'}


def _bits(value, table):
    names = [text for bit, text in table if value & bit]
    return ' + '.join(names) or 'none'


def _banks(mask):
    return ' + '.join(n for b, n in ((1, 'ADC0'), (2, 'ADC1')) if mask & b) or 'none'


def _state(raw):
    return _Defn.get_power_state_name(raw) if hasattr(_Defn, 'get_power_state_name') else str(raw)


def describe_power_event(event):
    """One line decoding data0/data1 of a published event, for people. Not published."""
    from stretch4_body.subsystem.power_periph import PowerPeriphAux
    eid = event.get('event_id')
    d0 = event.get('data0', 0)
    d1 = event.get('data1', 0)
    reason = _Defn.POWER_TRANSITION_REASON_NAMES
    state = PowerPeriphAux.get_power_state_name
    if eid == 1:
        causes = ', '.join(PowerPeriphAux._decode_reset_causes(d0 & 0xFF)) or 'none'
        return 'reset cause %s; board variant %d' % (causes, d1)
    if eid == 2:
        return '%s -> %s, reason %s' % (state(d0), state(event.get('power_state', 0)),
                                        _name(reason, d1))
    if eid in (3, 4, 6, 7, 8, 16):
        return '%d -> %d' % (d0, d1)
    if eid == 5:
        return 'request %d, reason %s' % (d0, _name(reason, d1))
    if eid in (10, 11, 12, 25):
        return '%d consecutive failures, %s %d' % (
            d0, 'exception or length' if eid == 25 else 'received bytes', d1)
    if eid == 13:
        return 'recovered after %d failures (last failure %s)' % (
            d0, _name(_Defn.POWER_EVENT_NAMES, d1))
    if eid in (14, 15, 20):
        return 'mask 0x%02X -> 0x%02X' % (d0 & 0xFFFF, d1 & 0xFFFF)
    if eid in (17, 18):
        return 'age %d ms, NUC VBUS %d' % (d0, d1)
    if eid in (22, 23, 24):
        return 'total %d, %s %d' % (d0, {22: 'frame length', 23: 'buffer index',
                                        24: 'UART id'}[eid], d1)
    if eid == 26:
        return 'BMS %s after %d ms' % ({1: 'present', 2: 'absent'}.get(d0, 'result %d' % d0), d1)
    if eid == 27:
        return '%s decision held, sample age %s' % (
            {1: 'low-battery shutdown', 2: 'charger'}.get(d0, 'decision %d' % d0),
            'none' if d1 == 32767 else '%d ms' % d1)
    if eid == 28:
        return 'boot watchdog arm %d, reason %s' % (
            d0, {0: 'RDK build', 1: 'no host config yet', 2: 'host nuc_safe_shutdown'}.get(d1, d1))
    if eid == 29:
        return 'reason %s, %s' % (_name(reason, d0),
                                  'waits for the NUC' if d1 else 'no NUC wait')
    if eid == 30:
        return 'ESP told (UART id 0x%02X) after %.1f s' % (d0 & 0xFF, d1 * 0.004)
    if eid == 31:
        return '%s after %.1f s' % (_WAIT_CAUSE.get(d0, 'cause %d' % d0), d1 * 0.004)
    if eid == 32:
        return 'rails down (%s), started by %s' % (
            ' + '.join(n for b, n in ((1, '20V0'), (2, '5V0')) if d0 & b) or 'none',
            _name(reason, d1))
    if eid == 33:
        return 'retargeted by %s, rail mask down 0x%X' % (_name(reason, d0), d1)
    if eid == 34:
        return 'ended by wake %s at step %d' % (_name(reason, d0), d1)
    if eid == 35:
        return 'charger stopped: %s; SoC %d, cell max %d mV' % (
            _bits((d0 >> 8) & 0xFF, _STOP_REASONS), d0 & 0xFF, d1)
    if eid == 36:
        return 'charger restarted: %s; SoC %d, pack %d mV' % (
            _bits((d0 >> 8) & 0xFF, _RESTART_REASONS), d0 & 0xFF, d1)
    if eid == 40:
        return 'ADC%d rebuilt, count %d' % (d0, d1)
    if eid == 41:
        return 'stale %s -> %s' % (_banks(d0), _banks(d1))
    if eid == 42:
        names = _Defn.ADAPTER_PRESENCE_SOURCE_NAMES
        return '%s -> %s' % (_name(names, d0), _name(names, d1))
    if eid == 43:
        return 'cause %s, %d low frames' % (
            _bits(d0, ((1, 'SoC 0'), (2, 'undervoltage'))), d1)
    if eid == 44:
        return 'rejected %s, %d since boot' % (_bits(d0, ((1, 'sleep'), (2, 'wake'))), d1)
    if eid == 45:
        kind = 'unknown id count %d' % -d0 if d0 < 0 else 'length error count %d' % d0
        return '%s, id 0x%02X, length %d' % (kind, (d1 >> 8) & 0xFF, d1 & 0xFF)
    if eid == 46:
        names = _Defn.ESP_ADAPTER_DISAGREE_NAMES
        return '%s -> %s' % (_name(names, d0), _name(names, d1))
    if eid == 47:
        return '%d sends, %d ms after the first' % (d0, d1)
    if eid == 50:
        return 'entered by %s' % {1: 'runstop hold', 2: 'host RPC 39'}.get(d0, d0)
    if eid == 51:
        return 'left by %s' % {2: 'host RPC 39', 3: 'runstop'}.get(d0, d0)
    return 'data0 %d, data1 %d' % (d0, d1)
