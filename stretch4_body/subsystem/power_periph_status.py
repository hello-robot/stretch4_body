#!/usr/bin/env python3
from collections import deque
import time

from stretch4_body.subsystem.power_health import PowerHealthEvaluator, PowerHealthThresholds


class PowerPeriphStatus:
    """
    PowerPeriph auxiliary status, diagnostics, event log and power health for RobotServer,
    published under status['power_periph'].
    """

    def __init__(self):
        # The 100 Hz server remains the sole PIMU serial owner. It samples the
        # dedicated p14 diagnostic RPC at 2 Hz, or the unchanged aux RPC on p13.
        self._power_aux_interval_s = 0.5
        self._power_aux_pending = False
        self._power_aux_expected_reply_sequence = None
        self._power_aux_request_kind = None
        self._power_aux_request_started_mono = None
        self._power_aux_last_request_mono = float("-inf")
        self._power_aux_last_sample_mono = None
        self._power_aux_last_state = None
        self._power_aux_transition_seq = 0
        self._power_aux_transitions = deque(maxlen=64)
        self._power_aux_meta = {
            'available': False,
            'server_started_wall_time_s': time.time(),
            'sample_interval_target_s': self._power_aux_interval_s,
            'sample_seq': 0,
            'sample_wall_time_s': None,
            'sample_monotonic_s': None,
            'sample_age_s': None,
            'request_errors': 0,
            'request_timeouts': 0,
            'request_latency_s': None,
            'request_latency_max_s': 0.0,
            'transport_error': None,
            'schema_error': None,
            'transitions': [],
        }
        self._power_diag_last_sample_mono = None
        self._power_diag_last_uptime_ms = None
        self._power_event_discard_page = False
        self._power_diag_meta = {
            'available': False,
            'sample_interval_target_s': self._power_aux_interval_s,
            'sample_seq': 0,
            'sample_wall_time_s': None,
            'sample_monotonic_s': None,
            'sample_age_s': None,
            'request_errors': 0,
            'request_timeouts': 0,
            'request_latency_s': None,
            'request_latency_max_s': 0.0,
            'transport_error': None,
            'schema_error': None,
            # Live classification: firmware age at the sample plus host time
            # since then. 'stale' once the diagnostics request times out or
            # fails to decode after a sample was seen, 'unknown' before any.
            'bms_freshness': 'unknown',
            'esp_freshness': 'unknown',
            'bms_age_s': None,
            'esp_age_s': None,
            # What the firmware reported when the sample arrived, kept so a
            # consumer can tell host staleness from a BMS or ESP outage.
            'bms_freshness_at_sample': 'unknown',
            'esp_freshness_at_sample': 'unknown',
            'pimu_boot_count_observed': 0,
            # From the PIMU board info at startup, so a status reader (or a status
            # log read months later) knows which firmware produced the numbers.
            'firmware_version': None,
            'protocol_version': None,
        }
        self._power_diag_fw_ages = (None, None)
        self._power_event_after_sequence = 0
        self._power_event_last_dropped_count = 0
        # This history is included in the existing 100 Hz ZMQ status object, so
        # keep it no larger than the firmware's own 64-entry ring. The logger
        # consumes new records every 0.5 s; a larger host cache would repeatedly
        # pickle stale events without improving recoverability from the PIMU.
        self._power_events = deque(maxlen=64)
        self._power_event_meta = {
            'available': False,
            'cursor_sequence': 0,
            'oldest_sequence': 0,
            'newest_sequence': 0,
            'firmware_dropped_count': 0,
            'coverage_gap_count': 0,
            'coverage_gaps': [],
            'request_errors': 0,
            'request_timeouts': 0,
            'reboot_pages_discarded': 0,
            'transport_error': None,
            'schema_error': None,
            'events': [],
        }
        if self.get_subsystem('power_periph') is not None:
            # These are references and update in place before the existing
            # conflated server status publication.
            self.status['power_periph']['status_aux'] = self.power_periph.status_aux
            self.status['power_periph']['status_aux_meta'] = self._power_aux_meta
            self.status['power_periph']['power_diagnostics'] = self.power_periph.power_diagnostics
            self.status['power_periph']['power_diagnostics_meta'] = self._power_diag_meta
            self.status['power_periph']['power_event_log'] = self._power_event_meta
        # Derived plain-language health of the same data (power_health.py). Evaluated
        # when a diagnostics sample lands and on a 2 Hz tick, never per 100 Hz cycle.
        self._power_health_evaluator = PowerHealthEvaluator()
        self._power_health_last_eval_mono = float('-inf')
        self._update_power_health(force=True)

    def _record_power_firmware_version(self):
        pp = self.get_subsystem('power_periph')
        if pp is None:
            return
        board_info = getattr(pp, 'board_info', {}) or {}
        self._power_diag_meta['firmware_version'] = board_info.get('firmware_version')
        self._power_diag_meta['protocol_version'] = board_info.get('protocol_version')
        self._update_power_health(force=True)

    def _refresh_power_periph_status(self):
        if self._power_aux_last_sample_mono is not None:
            self._power_aux_meta['sample_age_s'] = round(
                max(0.0, time.monotonic() - self._power_aux_last_sample_mono), 3
            )
        if self._power_diag_last_sample_mono is not None:
            self._power_diag_meta['sample_age_s'] = round(
                max(0.0, time.monotonic() - self._power_diag_last_sample_mono), 3
            )
        self._refresh_power_freshness()
        self._update_power_health()

    def _queue_power_periph_aux_status(self):
        """Queue p14 diagnostics, with the byte-exact legacy aux fallback."""
        pp = self.get_subsystem('power_periph')
        now = time.monotonic()
        if (
            pp is None
            or self._power_aux_pending
            or now - self._power_aux_last_request_mono < self._power_aux_interval_s
        ):
            return
        self._power_aux_last_request_mono = now
        request_kind = None
        try:
            if getattr(pp, 'supports_power_diagnostics', lambda: False)():
                request_kind = 'diagnostics'
                reply_sequence = getattr(pp, 'power_diagnostics_reply_sequence', None)
                queued = pp.pull_power_diagnostics(
                    blocking=False,
                    after_sequence=self._power_event_after_sequence,
                    max_entries=16,
                )
            else:
                request_kind = 'aux'
                reply_sequence = getattr(pp, 'status_aux_reply_sequence', None)
                queued = pp.pull_status_aux(blocking=False)
            self._power_aux_pending = bool(queued)
            if self._power_aux_pending:
                self._power_aux_request_kind = request_kind
                self._power_aux_expected_reply_sequence = (
                    None if reply_sequence is None else reply_sequence + 1
                )
                self._power_aux_request_started_mono = now
            else:
                self._power_aux_request_kind = None
                self._power_aux_meta['request_errors'] += 1
                self._power_aux_meta['transport_error'] = '%s RPC was not queued' % request_kind
                if request_kind == 'diagnostics':
                    self._power_diag_meta['request_errors'] += 1
                    self._power_event_meta['request_errors'] += 1
        except Exception as exc:
            self._power_aux_request_kind = None
            self._power_aux_meta['request_errors'] += 1
            self._power_aux_meta['transport_error'] = '%s: %s' % (
                type(exc).__name__, exc)
            if request_kind == 'diagnostics':
                self._power_diag_meta['request_errors'] += 1
                self._power_event_meta['request_errors'] += 1
            self.logger.warning(
                'Unable to queue PowerPeriph auxiliary status: %s', exc
            )

    def _complete_power_periph_aux_status(self):
        """Publish only callback-confirmed telemetry; never bless a cached sample."""
        if not self._power_aux_pending:
            return
        request_kind = self._power_aux_request_kind or 'aux'
        expected = self._power_aux_expected_reply_sequence
        sequence_attr = (
            'power_diagnostics_reply_sequence'
            if request_kind == 'diagnostics'
            else 'status_aux_reply_sequence'
        )
        actual = getattr(self.power_periph, sequence_attr, None)
        if expected is not None and (actual is None or actual < expected):
            timeout_s = getattr(self.power_periph, 'POWER_DIAGNOSTICS_RPC_TIMEOUT_S', 2.0)
            if (self._power_aux_request_started_mono is not None and
                    time.monotonic() - self._power_aux_request_started_mono >= timeout_s):
                self._power_aux_pending = False
                self._power_aux_request_kind = None
                self._power_aux_meta['request_errors'] += 1
                self._power_aux_meta['request_timeouts'] += 1
                self._power_aux_meta['transport_error'] = (
                    '%s RPC callback timeout' % request_kind)
                if request_kind == 'diagnostics':
                    self._power_diag_meta['available'] = False
                    self._power_diag_meta['request_errors'] += 1
                    self._power_diag_meta['request_timeouts'] += 1
                    self._power_diag_meta['transport_error'] = (
                        'diagnostics RPC callback timeout')
                    self._refresh_power_freshness()
                    self._power_event_meta['request_errors'] += 1
                    self._power_event_meta['request_timeouts'] += 1
                    self._power_event_meta['transport_error'] = (
                        'diagnostics RPC callback timeout')
                    self._update_power_health(force=True)
            return
        self._power_aux_pending = False
        self._power_aux_request_kind = None
        self._power_aux_meta['transport_error'] = None
        if request_kind == 'diagnostics':
            self._power_diag_meta['transport_error'] = None
            self._power_event_meta['transport_error'] = None
        now_mono = time.monotonic()
        now_wall = time.time()
        request_latency_s = (
            None if self._power_aux_request_started_mono is None
            else max(0.0, now_mono - self._power_aux_request_started_mono)
        )
        if request_latency_s is not None:
            self._power_aux_meta['request_latency_s'] = round(request_latency_s, 6)
            self._power_aux_meta['request_latency_max_s'] = round(max(
                self._power_aux_meta.get('request_latency_max_s', 0.0),
                request_latency_s,
            ), 6)
            if request_kind == 'diagnostics':
                self._power_diag_meta['request_latency_s'] = round(request_latency_s, 6)
                self._power_diag_meta['request_latency_max_s'] = round(max(
                    self._power_diag_meta.get('request_latency_max_s', 0.0),
                    request_latency_s,
                ), 6)
        self._power_aux_last_sample_mono = now_mono
        aux = self.power_periph.status_aux
        if request_kind == 'diagnostics':
            self._ingest_power_periph_diagnostics()
            self._ingest_power_periph_event_log()
            self._update_power_health(force=True)
        required_fields = {
            'current_pwr_state',
            'current_pwr_state_name',
            'runstop_cause_raw',
            'motor_sync_cnt',
        }
        schema_valid = required_fields.issubset(aux)
        if aux.get('_valid') is False:
            schema_valid = False
        if not schema_valid:
            self._power_aux_meta['available'] = False
            self._power_aux_meta['schema_error'] = (
                aux.get('_decode_error')
                or 'PowerPeriph aux decoder does not expose the power-state schema'
            )
            return

        self._power_aux_meta['available'] = True
        self._power_aux_meta['schema_error'] = None
        self._power_aux_meta['sample_seq'] += 1
        self._power_aux_meta['sample_wall_time_s'] = now_wall
        self._power_aux_meta['sample_monotonic_s'] = now_mono
        self._power_aux_meta['sample_age_s'] = 0.0

        state = aux.get('current_pwr_state')
        if state != self._power_aux_last_state:
            self._power_aux_transition_seq += 1
            self._power_aux_transitions.append({
                'seq': self._power_aux_transition_seq,
                'observed_wall_time_s': now_wall,
                'observed_monotonic_s': now_mono,
                'from_state': self._power_aux_last_state,
                'to_state': state,
                'to_state_name': aux.get('current_pwr_state_name'),
                'runstop_cause_raw': aux.get('runstop_cause_raw'),
            })
            self._power_aux_last_state = state
        self._power_aux_meta['transitions'] = list(self._power_aux_transitions)

    @staticmethod
    def _firmware_freshness(age_s, warn_s, fault_s):
        if age_s is None:
            return 'unavailable'
        if age_s >= fault_s:
            return 'fault'
        if age_s >= warn_s:
            return 'warning'
        return 'fresh'

    @staticmethod
    def _u32_sequence_is_newer(candidate, reference):
        delta = (int(candidate) - int(reference)) & 0xFFFFFFFF
        return 0 < delta < 0x80000000

    def _ingest_power_periph_diagnostics(self):
        diagnostics = self.power_periph.power_diagnostics
        if not diagnostics.get('_valid'):
            self._power_diag_meta['available'] = False
            self._power_diag_meta['request_errors'] += 1
            self._power_diag_meta['schema_error'] = diagnostics.get('_decode_error')
            self._refresh_power_freshness()
            return

        now_mono = time.monotonic()
        now_wall = time.time()
        uptime_ms = diagnostics.get('uptime_ms')
        if (self._power_diag_last_uptime_ms is not None and uptime_ms is not None and
                uptime_ms < self._power_diag_last_uptime_ms):
            self._power_diag_meta['pimu_boot_count_observed'] += 1
            # This reply was requested with the previous boot's cursor. Do not
            # ingest its event page: a new boot can already have advanced past
            # that stale cursor, which would silently skip the boot/reset event.
            self._power_event_discard_page = True
            self._power_event_after_sequence = 0
            self._power_events.clear()
        self._power_diag_last_uptime_ms = uptime_ms
        self._power_diag_last_sample_mono = now_mono
        self._power_diag_fw_ages = (
            diagnostics.get('bms_sample_age_s'), diagnostics.get('esp_status_age_s'))
        self._power_diag_meta.update({
            'available': True,
            'sample_seq': self._power_diag_meta['sample_seq'] + 1,
            'sample_wall_time_s': now_wall,
            'sample_monotonic_s': now_mono,
            'sample_age_s': 0.0,
            'schema_error': None,
            'bms_freshness_at_sample': self._firmware_freshness(
                self._power_diag_fw_ages[0], *self.BMS_FRESHNESS_S),
            'esp_freshness_at_sample': self._firmware_freshness(
                self._power_diag_fw_ages[1], *self.ESP_FRESHNESS_S),
        })
        self._refresh_power_freshness()

    # (warning, fault) thresholds on the age of the last good BMS sample and
    # the last ESP status, in seconds; defined with the other health thresholds.
    BMS_FRESHNESS_S = PowerHealthThresholds.BMS_FRESHNESS_S
    ESP_FRESHNESS_S = PowerHealthThresholds.ESP_FRESHNESS_S

    # power_health is re-evaluated at most this often between diagnostics samples,
    # so the link and freshness checks age without a new sample.
    POWER_HEALTH_TICK_S = 0.5

    def _update_power_health(self, force=False):
        """
        Publish status['power_periph']['power_health']. Forced when a diagnostics
        sample or timeout lands; otherwise at most every POWER_HEALTH_TICK_S, so the
        100 Hz loop pays one monotonic read per cycle and the evaluation runs at 2 Hz.
        """
        evaluator = getattr(self, '_power_health_evaluator', None)
        status = getattr(self, 'status', None)
        if evaluator is None or not status or 'power_periph' not in status:
            return
        now = time.monotonic()
        if not force and now - self._power_health_last_eval_mono < self.POWER_HEALTH_TICK_S:
            return
        self._power_health_last_eval_mono = now
        pp_status = status['power_periph']
        try:
            pp_status['power_health'] = evaluator.evaluate(pp_status, now)
        except Exception as exc:  # a health bug must never stop the control loop
            pp_status['power_health'] = {
                'level': 'unknown', 'summary': 'Power health evaluation failed: %s.' % exc,
                'checks': [], 'counts': {}, 'sample_seq': None,
            }
            if not getattr(self, '_power_health_error_logged', False):
                self._power_health_error_logged = True
                self.logger.warning('Power health evaluation failed: %s', exc)

    def _refresh_power_freshness(self):
        """
        Recompute bms_freshness and esp_freshness from the firmware age at the
        last sample plus host monotonic time since then. The firmware measures
        its ages at snapshot time, so a value copied once and left alone reads
        'fresh' forever after the diagnostics stop arriving.
        """
        meta = self._power_diag_meta
        if self._power_diag_last_sample_mono is None:
            return
        elapsed = max(0.0, time.monotonic() - self._power_diag_last_sample_mono)
        for key, fw_age, thresholds in (
            ('bms', self._power_diag_fw_ages[0], self.BMS_FRESHNESS_S),
            ('esp', self._power_diag_fw_ages[1], self.ESP_FRESHNESS_S),
        ):
            age = None if fw_age is None else fw_age + elapsed
            meta[key + '_age_s'] = None if age is None else round(age, 3)
            if not meta['available']:
                meta[key + '_freshness'] = 'stale'
            else:
                meta[key + '_freshness'] = self._firmware_freshness(age, *thresholds)

    def _ingest_power_periph_event_log(self):
        page = self.power_periph.power_event_log_page
        if not page.get('_valid'):
            self._power_event_meta['available'] = False
            self._power_event_meta['request_errors'] += 1
            self._power_event_meta['schema_error'] = page.get('_decode_error')
            return

        if self._power_event_discard_page:
            self._power_event_discard_page = False
            self._power_event_meta['reboot_pages_discarded'] += 1
            self._power_event_meta.update({
                'available': False,
                'cursor_sequence': 0,
                'events': [],
                'schema_error': 'discarded stale-cursor event page after PIMU reboot',
            })
            # Queue one immediate cursor-zero read. Normal 2 Hz throttling
            # resumes after that request; this is not a catch-up loop.
            self._power_aux_last_request_mono = float('-inf')
            return

        newest = page.get('newest_sequence', 0)
        if (self._power_event_after_sequence and newest and
                newest != self._power_event_after_sequence and
                not self._u32_sequence_is_newer(
                    newest, self._power_event_after_sequence)):
            # The PIMU rebooted between diagnostic samples. Re-read its new ring.
            self._power_event_after_sequence = 0
            self._power_events.clear()
            self._power_event_meta.update({
                'available': False,
                'cursor_sequence': 0,
                'events': [],
                'schema_error': 'event sequence regressed; re-reading PIMU ring',
            })
            self._power_aux_last_request_mono = float('-inf')
            return

        events = page.get('events', [])
        if events and self._power_event_after_sequence:
            expected_first = (self._power_event_after_sequence + 1) & 0xFFFFFFFF
            if expected_first == 0:
                expected_first = 1
            if events[0]['sequence'] != expected_first:
                gaps = self._power_event_meta['coverage_gaps']
                gaps.append({
                    'detected_wall_time_s': time.time(),
                    'after_sequence': self._power_event_after_sequence,
                    'resumed_sequence': events[0]['sequence'],
                    'firmware_dropped_count': page.get('dropped_count', 0),
                })
                del gaps[:-32]
                self._power_event_meta['coverage_gap_count'] += 1

        now_wall = time.time()
        diagnostics = self.power_periph.power_diagnostics
        boot_wall_s = None
        current_uptime_ms = None
        if diagnostics.get('_valid') and diagnostics.get('uptime_ms') is not None:
            current_uptime_ms = int(diagnostics['uptime_ms'])
            boot_wall_s = now_wall - current_uptime_ms / 1000.0
        for event in events:
            enriched = event.copy()
            enriched['received_wall_time_s'] = now_wall
            if boot_wall_s is not None and current_uptime_ms is not None:
                event_uptime_ms = (
                    (current_uptime_ms & ~0xFFFFFFFF) |
                    int(event['timestamp_ms'])
                )
                if (event_uptime_ms > current_uptime_ms and
                        event_uptime_ms >= (1 << 32)):
                    event_uptime_ms -= (1 << 32)
                enriched['uptime_ms'] = event_uptime_ms
                enriched['estimated_wall_time_s'] = (
                    boot_wall_s + event_uptime_ms / 1000.0
                )
            self._power_events.append(enriched)
            self._power_event_after_sequence = event['sequence']

        self._power_event_last_dropped_count = page.get('dropped_count', 0)
        self._power_event_meta.update({
            'available': True,
            'cursor_sequence': self._power_event_after_sequence,
            'oldest_sequence': page.get('oldest_sequence', 0),
            'newest_sequence': newest,
            'firmware_dropped_count': self._power_event_last_dropped_count,
            'retained_count': page.get('retained_count', 0),
            'has_more': page.get('has_more', False),
            'reply_status': page.get('reply_status'),
            'reply_status_name': page.get('reply_status_name'),
            'schema_error': None,
            # Re-publish the bounded history so the independent 2 Hz recorder
            # cannot phase-miss a short-lived page. Stable sequence/timestamp
            # deduplication prevents duplicate disk records after Body restarts.
            'events': list(self._power_events),
        })
