"""Derived power health (status['power_periph']['power_health']).

Every rule in stretch4_body/subsystem/power_health.py has a case here: the
healthy baseline, each warn and fault, a schema 1 snapshot, a stale link, the
one-minute rate windows and the enum names. Snapshots are real decoder output:
the firmware fixture's 255 (or 190) bytes with the fields under test packed in
at their Common.h offsets, then decoded by PowerPeriphAux. No hardware.
"""
import json
import struct

import pytest

from stretch4_body.subsystem.power_periph import PowerPeriphAux
from stretch4_body.subsystem.power_health import (
    CHECK_IDS, CHECK_TITLES, PowerHealthEvaluator, PowerHealthThresholds,
    describe_power_event, worst_level)
from test_power_periph_schema2 import SCHEMA1_FIELDS, SCHEMA2_FIELDS, load_fixture

FIELDS = {name: (offset, fmt) for name, offset, fmt in SCHEMA1_FIELDS + SCHEMA2_FIELDS}
SNAPSHOT_SIZE = {1: 190, 2: 255}

CAPS_RELEASE = 0x3F | (1 << 10) | (1 << 11)   # BMS, event log, ESP, eye readback, INA
VALID_ALL = 0x1F

HEALTHY = {
    'reset_cause_raw': 0x01, 'capability_flags_raw': CAPS_RELEASE,
    'validity_flags_raw': VALID_ALL, 'power_state': 0, 'power_transition_reason_raw': 1,
    'runstop_cause_raw': 0, 'bms_sample_age_ms': 400, 'bms_valid_replies': 3600,
    'bms_crc_errors': 0, 'bms_timeouts': 0, 'bms_rx_overflows': 0, 'bms_protocol_errors': 0,
    'bms_consecutive_failures': 0, 'battery_voltage_mv': 27200, 'battery_current_ma': 1500,
    'battery_soc': 80, 'esp_status_age_ms': 60, 'esp_crc_errors': 0, 'esp_rx_overflows': 0,
    'esp_tx_queue_drops': 0, 'loop_overrun_count': 0, 'charge_stop_state': 0x10,
    'shutdown_wait_active': False, 'shutdown_wait_last_end': 255, 'shutdown_wait_count': 0,
    'shutdown_wait_elapsed_ms': 0, 'shutdown_wait_last_duration_ms': 0,
    'shutdown_wait_cap_ms': 90000, 'adc_stale': 0, 'adapter_presence_source': 0,
    'esp_rx_length_errors': 0, 'esp_rx_unknown_ids': 0, 'esp_tx_mark_resets': 0,
    'esp_shutdown_cmd_sends': 0, 'esp_adapter_disagree': 0, 'wait_sd_ack': False,
    'ina_vbat_mv': 27150, 'ina_current_ma': 1480, 'ina_flags': 3,
}


def snapshot(schema=2, adc_reinit=(0, 0), **fields):
    """Decoded power_diagnostics for the fixture snapshot with fields packed in."""
    raw = bytearray(load_fixture(schema)[25:25 + SNAPSHOT_SIZE[schema]])
    values = dict(HEALTHY)
    values.update(fields)
    for name, value in values.items():
        offset, fmt = FIELDS[name]
        if offset >= SNAPSHOT_SIZE[schema]:
            continue
        struct.pack_into('<' + ('B' if fmt == '?' else fmt), raw, offset, int(value))
    if schema >= 2:
        raw[221], raw[222] = adc_reinit
    power = PowerPeriphAux()
    assert power._unpack_power_diagnostics(bytes(raw)) == len(raw)
    return dict(power.power_diagnostics)


def status(schema=2, meta=None, log=None, rpc_rejected=0, rpc_reply_errors=0,
           adc_reinit=(0, 0), eoa_tripped=False, power_to_eoa=True, **fields):
    diag_meta = {
        'available': True, 'sample_seq': 1, 'sample_age_s': 0.1,
        'request_errors': 0, 'request_timeouts': 0, 'schema_error': None,
        'transport_error': None, 'bms_freshness': 'fresh', 'esp_freshness': 'fresh',
        'bms_age_s': 0.5, 'esp_age_s': 0.1, 'bms_freshness_at_sample': 'fresh',
        'esp_freshness_at_sample': 'fresh', 'pimu_boot_count_observed': 0,
        'firmware_version': 'hello-pimu2.v0.1.50p14', 'protocol_version': 'p14',
    }
    diag_meta.update(meta or {})
    event_log = {
        'available': True, 'cursor_sequence': 10, 'firmware_dropped_count': 0,
        'coverage_gap_count': 0, 'coverage_gaps': [], 'reboot_pages_discarded': 0,
        'events': [],
    }
    event_log.update(log or {})
    return {
        'power_diagnostics': snapshot(schema, adc_reinit=adc_reinit, **fields),
        'power_diagnostics_meta': diag_meta,
        'power_event_log': event_log,
        'rpc_rejected': {'count': rpc_rejected, 'last_request_id': 33 if rpc_rejected else None,
                         'last_reason': 1 if rpc_rejected else None, 'last_length': None},
        'rpc_reply_errors': rpc_reply_errors,
        'eoa_output_tripped': eoa_tripped,
        'periph_power_state': {'power_to_eoa': power_to_eoa},
        'current_eoa': 0.0 if eoa_tripped else 0.35,
        'runstop_event': eoa_tripped,
    }


def checks(health):
    return {c['id']: c for c in health['checks']}


def evaluate(st, now=100.0):
    return PowerHealthEvaluator().evaluate(st, now)


def run(samples, evaluator=None):
    """Feed (time, status) pairs in order, bumping sample_seq; return the last health."""
    evaluator = evaluator or PowerHealthEvaluator()
    health = None
    for seq, (t, st) in enumerate(samples, start=1):
        st['power_diagnostics_meta']['sample_seq'] = seq
        health = evaluator.evaluate(st, t)
    return health


# --- healthy baseline and shape -------------------------------------------------------------

def test_healthy_schema2_is_ok_on_every_check():
    health = evaluate(status())
    assert health['level'] == 'ok', [(c['id'], c['message']) for c in health['checks']]
    assert [c['id'] for c in health['checks']] == list(CHECK_IDS)
    assert all(c['level'] == 'ok' for c in health['checks'])
    assert health['counts'] == {'ok': len(CHECK_IDS), 'warn': 0, 'fault': 0, 'unknown': 0}
    assert set(CHECK_TITLES) == set(CHECK_IDS)


def test_every_check_has_a_sentence_and_values():
    for check in evaluate(status())['checks']:
        assert set(check) == {'id', 'level', 'message', 'values'}
        assert check['message'].endswith('.')
        assert isinstance(check['values'], dict)


def test_health_is_compact_json():
    health = evaluate(status())
    text = json.dumps(health)
    assert json.loads(text) == health
    # Published at 100 Hz and logged twice per sample (power_periph and its pimu alias).
    assert len(text) < 4500, len(text)


def test_worst_level_ordering():
    assert worst_level(['ok', 'unknown']) == 'ok'
    assert worst_level(['ok', 'warn', 'unknown']) == 'warn'
    assert worst_level(['warn', 'fault']) == 'fault'
    assert worst_level([]) == 'ok'


# --- diagnostics link --------------------------------------------------------------------

def test_no_sample_yet_is_unknown_everywhere():
    st = status(meta={'available': False, 'sample_seq': 0, 'sample_age_s': None})
    st['power_diagnostics'] = {'_valid': False, '_decode_error': None, 'schema_version': None}
    health = evaluate(st)
    assert health['level'] == 'unknown'
    # rpc_rejections and eoa_output come from the 100 Hz status, not the snapshot.
    assert all(c['level'] == 'unknown' for c in health['checks']
               if c['id'] not in ('rpc_rejections', 'eoa_output'))


def test_p13_firmware_says_so():
    st = status(meta={'available': False, 'sample_seq': 0, 'protocol_version': 'p13',
                      'firmware_version': 'hello-pimu2.v0.1.14p13'})
    st['power_diagnostics'] = {'_valid': False, '_decode_error': None, 'schema_version': None}
    link = checks(evaluate(st))['diagnostics_link']
    assert link['level'] == 'unknown'
    assert 'p13' in link['message']


def test_missing_meta_is_unknown_not_an_exception():
    health = evaluate({})
    assert health['level'] == 'unknown'


def test_stale_link_warns_then_faults():
    st = status(meta={'available': False, 'sample_age_s': 3.0,
                      'transport_error': 'diagnostics RPC callback timeout',
                      'bms_freshness': 'stale', 'esp_freshness': 'stale'})
    c = checks(evaluate(st))
    assert c['diagnostics_link']['level'] == 'warn'
    assert 'timeout' in c['diagnostics_link']['message']
    assert c['bms_freshness']['level'] == 'unknown'
    assert c['esp_freshness']['level'] == 'unknown'
    st['power_diagnostics_meta']['sample_age_s'] = 12.0
    assert checks(evaluate(st))['diagnostics_link']['level'] == 'fault'


def test_old_sample_while_available_warns():
    st = status(meta={'sample_age_s': PowerHealthThresholds.LINK_STALE_WARN_S + 0.5})
    assert checks(evaluate(st))['diagnostics_link']['level'] == 'warn'


def test_unsupported_schema_warns():
    st = status(meta={'available': False, 'sample_seq': 0, 'sample_age_s': None,
                      'schema_error': 'unsupported diagnostics schema 3/size 300'})
    st['power_diagnostics'] = {'_valid': False, '_decode_error': 'unsupported', 'schema_version': None}
    link = checks(evaluate(st))['diagnostics_link']
    assert link['level'] == 'warn'
    assert 'schema' in link['message']


def test_link_values_carry_schema_and_version():
    values = checks(evaluate(status()))['diagnostics_link']['values']
    assert values['schema_version'] == 2
    assert values['firmware_version'] == 'hello-pimu2.v0.1.50p14'
    assert values['sample_age_s'] == 0.1


# --- schema 1 ------------------------------------------------------------------------------

SCHEMA2_CHECKS = ('esp_frame_errors', 'adc_banks', 'adapter_source', 'adapter_agreement',
                  'vbat_cross_check', 'shutdown_wait', 'shutdown_command', 'charge_stop')


def test_schema1_reports_unknown_not_warn():
    health = evaluate(status(schema=1, meta={'firmware_version': 'hello-pimu2.v0.1.23p14'}))
    c = checks(health)
    for check_id in SCHEMA2_CHECKS:
        assert c[check_id]['level'] == 'unknown', check_id
        assert 'not reported by this firmware' in c[check_id]['message'].lower(), check_id
    assert health['level'] == 'ok'
    assert c['diagnostics_link']['level'] == 'ok'
    assert 'schema 1' in c['diagnostics_link']['message']
    # The EOA output bit is also newer than this firmware (v0.1.50p14).
    assert c['eoa_output']['level'] == 'unknown'
    assert health['counts']['unknown'] == len(SCHEMA2_CHECKS) + 1


# --- BMS ---------------------------------------------------------------------------------

@pytest.mark.parametrize('freshness,level', [
    ('fresh', 'ok'), ('warning', 'warn'), ('fault', 'fault'), ('unavailable', 'fault'),
    ('stale', 'unknown'), ('unknown', 'unknown')])
def test_bms_freshness_levels(freshness, level):
    st = status(meta={'bms_freshness': freshness})
    assert checks(evaluate(st))['bms_freshness']['level'] == level


def test_bms_consecutive_failures_warn():
    st = status(bms_consecutive_failures=PowerHealthThresholds.BMS_CONSECUTIVE_FAILURES_WARN)
    c = checks(evaluate(st))['bms_link_errors']
    assert c['level'] == 'warn'
    assert c['values']['bms_consecutive_failures'] == 3


def test_bms_error_rate_over_the_minute():
    t0 = 1000.0
    limit = PowerHealthThresholds.BMS_ERRORS_PER_MIN_WARN
    health = run([(t0, status(bms_crc_errors=5)),
                  (t0 + 30, status(bms_crc_errors=5 + limit))])
    c = checks(health)['bms_link_errors']
    assert c['level'] == 'warn'
    assert c['values']['errors_last_min'] == limit


def test_bms_errors_below_rate_are_ok():
    t0 = 1000.0
    health = run([(t0, status(bms_timeouts=7)), (t0 + 30, status(bms_timeouts=8))])
    c = checks(health)['bms_link_errors']
    assert c['level'] == 'ok'
    assert c['values']['errors_last_min'] == 1


def test_bms_extended_sample_missing_warns():
    st = status(validity_flags_raw=VALID_ALL & ~(1 << 3))
    assert checks(evaluate(st))['bms_link_errors']['level'] == 'warn'


# --- ESP ---------------------------------------------------------------------------------

@pytest.mark.parametrize('freshness,level', [
    ('fresh', 'ok'), ('warning', 'warn'), ('fault', 'fault'), ('unavailable', 'fault'),
    ('stale', 'unknown')])
def test_esp_freshness_levels(freshness, level):
    st = status(meta={'esp_freshness': freshness})
    assert checks(evaluate(st))['esp_freshness']['level'] == level


def test_esp_length_errors_in_last_minute_warn_and_age_out():
    t0 = 500.0
    ev = PowerHealthEvaluator()
    health = run([(t0, status(esp_rx_length_errors=4)),
                  (t0 + 10, status(esp_rx_length_errors=5))], ev)
    c = checks(health)['esp_frame_errors']
    assert c['level'] == 'warn'
    assert c['values']['esp_rx_length_errors'] == 5
    assert c['values']['esp_rx_length_errors_last_min'] == 1
    # Two quiet minutes later the window holds no increase.
    st = status(esp_rx_length_errors=5)
    st['power_diagnostics_meta']['sample_seq'] = 3
    ev.evaluate(st, t0 + 80)
    st = status(esp_rx_length_errors=5)
    st['power_diagnostics_meta']['sample_seq'] = 4
    c = checks(ev.evaluate(st, t0 + 140))['esp_frame_errors']
    assert c['level'] == 'ok'
    assert c['values']['esp_rx_length_errors_last_min'] == 0
    assert 'since boot' in c['message']


def test_esp_unknown_ids_at_poll_rate_fault():
    t0 = 0.0
    n = PowerHealthThresholds.ESP_FRAME_ERRORS_PER_MIN_FAULT
    health = run([(t0, status(esp_rx_unknown_ids=0)), (t0 + 20, status(esp_rx_unknown_ids=n))])
    assert checks(health)['esp_frame_errors']['level'] == 'fault'


def test_esp_tx_mark_reset_warns():
    c = checks(evaluate(status(esp_tx_mark_resets=1)))['esp_frame_errors']
    assert c['level'] == 'warn'
    assert 'mark' in c['message']


def test_esp_crc_rate_warns():
    n = PowerHealthThresholds.ESP_LINK_ERRORS_PER_MIN_WARN
    health = run([(0.0, status(esp_crc_errors=0)), (30.0, status(esp_crc_errors=n))])
    assert checks(health)['esp_link_errors']['level'] == 'warn'


def test_first_sample_has_no_rate_yet():
    c = checks(evaluate(status(esp_rx_length_errors=40)))['esp_frame_errors']
    assert c['level'] == 'ok'
    assert c['values'].get('esp_rx_length_errors_last_min') is None


def test_counter_regression_resets_the_window():
    health = run([(0.0, status(esp_crc_errors=50)), (10.0, status(esp_crc_errors=2))])
    assert checks(health)['esp_link_errors']['values'].get('esp_crc_errors_last_min') is None


# --- ADC (.39) ---------------------------------------------------------------------------

def test_adc_stale_bank_warns_then_faults_when_it_persists():
    ev = PowerHealthEvaluator()
    c = checks(run([(0.0, status(adc_stale=1))], ev))['adc_banks']
    assert c['level'] == 'warn'
    assert 'ADC0' in c['message']
    st = status(adc_stale=1)
    st['power_diagnostics_meta']['sample_seq'] = 2
    c = checks(ev.evaluate(st, PowerHealthThresholds.ADC_STALE_FAULT_S + 0.5))['adc_banks']
    assert c['level'] == 'fault'


def test_adc_rebuild_in_last_minute_warns():
    health = run([(0.0, status(adc_reinit=(2, 0))), (40.0, status(adc_reinit=(3, 0)))])
    c = checks(health)['adc_banks']
    assert c['level'] == 'warn'
    assert c['values']['adc_reinit_last_min'] == 1


def test_adc_old_rebuilds_are_ok_with_a_note():
    c = checks(evaluate(status(adc_reinit=(0, 4))))['adc_banks']
    assert c['level'] == 'ok'
    assert '4' in c['message']


# --- adapter (.39, .42) ------------------------------------------------------------------

@pytest.mark.parametrize('source,level,name', [(0, 'ok', 'adc'), (1, 'warn', 'esp_fallback'),
                                               (2, 'warn', 'held')])
def test_adapter_source(source, level, name):
    c = checks(evaluate(status(adapter_presence_source=source)))['adapter_source']
    assert c['level'] == level
    assert c['values']['adapter_presence_source_name'] == name


@pytest.mark.parametrize('mask,level', [(0, 'ok'), (1, 'warn'), (2, 'warn'), (3, 'warn')])
def test_adapter_agreement(mask, level):
    c = checks(evaluate(status(esp_adapter_disagree=mask)))['adapter_agreement']
    assert c['level'] == level


# --- INA228 witness (.43) ----------------------------------------------------------------

def vbat(diff_mv, n, **extra):
    return run([(i * 0.5, status(battery_voltage_mv=27000, ina_vbat_mv=27000 + diff_mv, **extra))
                for i in range(n)])


def test_vbat_agreement_ok():
    c = checks(vbat(300, 3))['vbat_cross_check']
    assert c['level'] == 'ok'
    assert c['values']['difference_mv'] == 300


def test_vbat_single_mismatch_is_a_transient():
    c = checks(vbat(800, 1))['vbat_cross_check']
    assert c['level'] == 'ok'
    assert 'transient' in c['message']


def test_vbat_sustained_mismatch_warns():
    n = PowerHealthThresholds.VBAT_MISMATCH_SAMPLES
    assert checks(vbat(-800, n))['vbat_cross_check']['level'] == 'warn'


def test_vbat_gross_mismatch_faults():
    n = PowerHealthThresholds.VBAT_MISMATCH_SAMPLES
    assert checks(vbat(2500, n))['vbat_cross_check']['level'] == 'fault'


def test_vbat_skipped_on_stale_bms():
    st = status(meta={'bms_freshness_at_sample': 'warning'}, ina_vbat_mv=20000)
    c = checks(evaluate(st))['vbat_cross_check']
    assert c['level'] == 'unknown'
    assert 'BMS' in c['message']


def test_vbat_ina_read_failed_warns():
    st = status(ina_flags=1, validity_flags_raw=VALID_ALL & ~(1 << 4))
    assert checks(evaluate(st))['vbat_cross_check']['level'] == 'warn'


def test_vbat_ina_absent_at_boot_warns():
    st = status(ina_flags=0, validity_flags_raw=VALID_ALL & ~(1 << 4))
    c = checks(evaluate(st))['vbat_cross_check']
    assert c['level'] == 'warn'
    assert 'boot' in c['message']


def test_vbat_without_capability_is_unknown():
    st = status(capability_flags_raw=CAPS_RELEASE & ~(1 << 11))
    assert checks(evaluate(st))['vbat_cross_check']['level'] == 'unknown'


# --- shutdown (.27, .48) -----------------------------------------------------------------

def test_shutdown_wait_in_progress_is_ok():
    c = checks(evaluate(status(shutdown_wait_active=True, shutdown_wait_elapsed_ms=8000,
                               shutdown_wait_count=1, power_state=4)))['shutdown_wait']
    assert c['level'] == 'ok'
    assert '8.0 s' in c['message']


def test_shutdown_wait_near_cap_warns():
    elapsed = int(90000 * PowerHealthThresholds.SHUTDOWN_WAIT_WARN_FRACTION) + 1000
    c = checks(evaluate(status(shutdown_wait_active=True, shutdown_wait_elapsed_ms=elapsed,
                               power_state=4)))['shutdown_wait']
    assert c['level'] == 'warn'


@pytest.mark.parametrize('end,level,name', [
    (255, 'ok', 'none_since_boot'), (0, 'ok', 'no_wait_configured'), (1, 'ok', 'nuc_off'),
    (2, 'warn', 'cap'), (3, 'ok', 'cancelled_by_wake'), (4, 'warn', 'cap_stale_off')])
def test_shutdown_wait_last_end(end, level, name):
    c = checks(evaluate(status(shutdown_wait_last_end=end, shutdown_wait_count=1,
                               shutdown_wait_last_duration_ms=90004)))['shutdown_wait']
    assert c['level'] == level
    assert c['values']['shutdown_wait_last_end_name'] == name


def test_shutdown_command_resend_in_shutdown_is_ok():
    c = checks(evaluate(status(power_state=4, wait_sd_ack=True,
                               esp_shutdown_cmd_sends=120)))['shutdown_command']
    assert c['level'] == 'ok'
    assert '120' in c['message']


def test_shutdown_command_flag_in_active_warns():
    c = checks(evaluate(status(power_state=0, wait_sd_ack=True,
                               esp_shutdown_cmd_sends=3)))['shutdown_command']
    assert c['level'] == 'warn'


# --- charge stop (.24) -------------------------------------------------------------------

@pytest.mark.parametrize('raw,name,word', [(0x10, 'idle', 'idle'), (0x11, 'arming', 'arming'),
                                           (0x12, 'hold', 'holding')])
def test_charge_stop_states_are_ok_and_named(raw, name, word):
    c = checks(evaluate(status(charge_stop_state=raw)))['charge_stop']
    assert c['level'] == 'ok'
    assert c['values']['charge_stop_state_name'] == name
    assert c['values']['charge_stop_enabled'] is True
    assert word in c['message']


def test_charge_stop_policy_compiled_out_is_noted():
    c = checks(evaluate(status(charge_stop_state=0x00)))['charge_stop']
    assert c['level'] == 'ok'
    assert 'compiled out' in c['message']


# --- RPC, reboots, event log, build, timing ----------------------------------------------

def test_rpc_rejection_in_last_minute_warns():
    health = run([(0.0, status(rpc_rejected=0)), (5.0, status(rpc_rejected=1))])
    c = checks(health)['rpc_rejections']
    assert c['level'] == 'warn'
    assert '33' in c['message']


def test_old_rpc_rejections_are_ok_with_a_note():
    c = checks(evaluate(status(rpc_rejected=2)))['rpc_rejections']
    assert c['level'] == 'ok'
    assert c['values']['rpc_rejected_count'] == 2


def test_rpc_reply_errors_in_last_minute_warn():
    health = run([(0.0, status(rpc_reply_errors=1)), (5.0, status(rpc_reply_errors=3))])
    assert checks(health)['rpc_rejections']['level'] == 'warn'


def test_pimu_reboot_observed_warns():
    c = checks(evaluate(status(meta={'pimu_boot_count_observed': 1})))['pimu_reboots']
    assert c['level'] == 'warn'


def test_watchdog_reset_cause_warns():
    c = checks(evaluate(status(reset_cause_raw=1 << 5)))['pimu_reboots']
    assert c['level'] == 'warn'
    assert 'watchdog' in c['message']


def test_event_log_gap_warns_and_drops_alone_do_not():
    c = checks(evaluate(status(log={'coverage_gap_count': 1, 'firmware_dropped_count': 9})))
    assert c['event_log']['level'] == 'warn'
    c = checks(evaluate(status(log={'firmware_dropped_count': 9})))
    assert c['event_log']['level'] == 'ok'
    assert '9' in c['event_log']['message']


def test_event_log_unavailable_is_unknown():
    c = checks(evaluate(status(log={'available': False})))
    assert c['event_log']['level'] == 'unknown'


def test_bench_rig_build_faults_and_rdk_warns():
    c = checks(evaluate(status(capability_flags_raw=CAPS_RELEASE | (1 << 9))))
    assert c['firmware_build']['level'] == 'fault'
    caps = (CAPS_RELEASE & ~(1 << 11)) | (1 << 12)
    c = checks(evaluate(status(capability_flags_raw=caps)))
    assert c['firmware_build']['level'] == 'warn'
    assert c['vbat_cross_check']['level'] == 'unknown'


def test_loop_overrun_growth_warns():
    health = run([(0.0, status(loop_overrun_count=3)), (20.0, status(loop_overrun_count=4))])
    assert checks(health)['pimu_timing']['level'] == 'warn'


# --- overall -----------------------------------------------------------------------------

def test_overall_takes_the_worst_level():
    health = evaluate(status(adapter_presence_source=1, esp_tx_mark_resets=0,
                             capability_flags_raw=CAPS_RELEASE | (1 << 9)))
    assert health['level'] == 'fault'
    assert health['counts']['fault'] == 1 and health['counts']['warn'] == 1
    assert 'firmware_build' in health['summary']


def test_rates_only_advance_on_a_new_sample():
    ev = PowerHealthEvaluator()
    st = status(esp_crc_errors=0)
    ev.evaluate(st, 0.0)
    st2 = status(esp_crc_errors=100)  # same sample_seq: must not enter the window
    c = checks(ev.evaluate(st2, 1.0))['esp_link_errors']
    assert c['values'].get('esp_crc_errors_last_min') is None


# --- enum names --------------------------------------------------------------------------

def test_decoder_names_every_schema2_enum():
    d = snapshot(adapter_presence_source=1, shutdown_wait_last_end=4, esp_adapter_disagree=2,
                 charge_stop_state=0x12)
    assert d['adapter_presence_source_name'] == 'esp_fallback'
    assert d['shutdown_wait_last_end_name'] == 'cap_stale_off'
    assert d['esp_adapter_disagree_name'] == 'voltage_only'
    assert d['charge_stop_state_name'] == 'hold'
    assert d['charge_stop_enabled'] is True
    assert d['power_state_name'] == 'STATE_ACTIVE'
    assert d['power_transition_reason'] == 'boot_button'
    d = snapshot(adapter_presence_source=9, shutdown_wait_last_end=7)
    assert d['adapter_presence_source_name'] == 'unknown_9'
    assert d['shutdown_wait_last_end_name'] == 'unknown_7'


def test_decoder_names_are_none_on_schema1():
    d = snapshot(schema=1)
    for key in ('adapter_presence_source_name', 'shutdown_wait_last_end_name',
                'esp_adapter_disagree_name', 'charge_stop_state_name', 'charge_stop_enabled'):
        assert d[key] is None, key


def event(event_id, data0=0, data1=0, power_state=0):
    return {'sequence': 1, 'event_id': event_id, 'data0': data0, 'data1': data1,
            'power_state': power_state,
            'event_name': PowerPeriphAux.POWER_EVENT_NAMES.get(event_id, 'unknown_%d' % event_id)}


@pytest.mark.parametrize('ev,text', [
    (event(2, 1, 3), 'STATE_SLEEP -> STATE_ACTIVE, reason user_button'),
    (event(31, 2, 22500), 'cap'),
    (event(31, 1, 2000), 'NUC off after 8.0 s'),
    (event(42, 0, 1), 'adc -> esp_fallback'),
    (event(41, 0, 1), 'ADC0'),
    (event(35, (0x03 << 8) | 87, 3460), 'SoC 87'),
    (event(35, (0x03 << 8) | 87, 3460), 'cell max 3460 mV'),
    (event(46, 0, 1), 'esp_only'),
    (event(45, -3, (0x22 << 8) | 4), 'unknown id'),
    (event(1, 0x21, 0), 'power_on, watchdog'),
    (event(47, 2, 20), '2 sends'),
    (event(48, 1, 2), 'dropped host lidar-on, requested outside ACTIVE'),
    (event(48, 6, 1), 'dropped host AUX-on + wake AUX-on, at the ACTIVE exit'),
    (event(999, 1, 2), 'data0 1, data1 2'),
])
def test_describe_power_event(ev, text):
    assert text in describe_power_event(ev)


# --- one-line form for stretch_system_check ---------------------------------------------------

def test_health_line_for_system_check():
    from stretch4_body.subsystem.power_health import health_line
    assert health_line(None) is None
    assert health_line({'level': 'unknown'}) is None
    level, text = health_line(evaluate(status()))
    assert level == 'ok' and text.startswith('P14 power health: All 19 judged checks ok')
    assert 'stretch_power_diagnostics' not in text
    level, text = health_line(evaluate(status(adapter_presence_source=2)))
    assert level == 'warn'
    assert text == 'P14 power health: warn: adapter_source. Details: stretch_power_diagnostics'


def test_event_48_esp_trigger_dropped_is_named_and_explained():
    assert PowerPeriphAux.POWER_EVENT_NAMES[48] == 'esp_trigger_dropped'
    assert 'host lidar-on' in PowerPeriphAux.POWER_EVENT_EXPLANATIONS[48]


# --- EOA output (firmware v0.1.50p14) -------------------------------------------------------

def test_eoa_output_on_is_ok():
    c = checks(evaluate(status()))['eoa_output']
    assert c['level'] == 'ok'
    assert c['values']['eoa_output_tripped'] is False


def test_eoa_output_off_by_request_is_ok():
    c = checks(evaluate(status(power_to_eoa=False)))['eoa_output']
    assert c['level'] == 'ok'
    assert 'by request' in c['message']


def test_eoa_output_tripped_is_warn_and_says_how_to_recover():
    health = evaluate(status(eoa_tripped=True, power_to_eoa=False))
    c = checks(health)['eoa_output']
    assert c['level'] == 'warn'
    assert 'clear the runstop' in c['message'] and 'restore_eoa_power()' in c['message']
    assert health['summary'] == 'warn: eoa_output.'


def test_eoa_output_refused_clear_is_fault():
    events = [event(49, 1, 845), event(49, 16, 1), event(49, 2, 1167), event(49, 16, 2),
              event(49, 1, 900), event(49, 128, 3)]
    st = status(eoa_tripped=True, power_to_eoa=False, log={'events': events})
    c = checks(evaluate(st))['eoa_output']
    assert c['level'] == 'fault'
    assert c['values']['trips_last_60s'] == 3
    assert 'short' in c['message']


def test_eoa_output_tripped_after_a_later_trip_is_warn_again():
    # A host enable after the refusal, then a new cut: the last event decides.
    events = [event(49, 128, 3), event(49, 32, 3), event(49, 1, 850)]
    st = status(eoa_tripped=True, power_to_eoa=False, log={'events': events})
    assert checks(evaluate(st))['eoa_output']['level'] == 'warn'


@pytest.mark.parametrize('version', ['hello-pimu2.v0.1.49p14', 'hello-pimu2.v0.1.23p14'])
def test_eoa_output_unknown_before_v0_1_50(version):
    c = checks(evaluate(status(meta={'firmware_version': version})))['eoa_output']
    assert c['level'] == 'unknown'
    assert 'v0.1.50p14' in c['message']


def test_eoa_output_unknown_without_the_status_key():
    st = status()
    del st['eoa_output_tripped']
    assert checks(evaluate(st))['eoa_output']['level'] == 'unknown'


@pytest.mark.parametrize('ev,text', [
    (event(49, 1, 845), 'cut by ADC filter at 8.45 A'),
    (event(49, 3, 1167), 'cut by ADC filter + U44 comparator at 11.67 A'),
    (event(49, 16, 1), 'restored by runstop clear, 1 trip in the last 60 s'),
    (event(49, 32, 3), 'restored by host enable, 3 trips in the last 60 s'),
    (event(49, 64, 0), 'restored by ACTIVE entry, 0 trips in the last 60 s'),
    (event(49, 128, 3), 'runstop clear refused (3 trips in 60 s), 3 trips in the last 60 s'),
])
def test_describe_event_49(ev, text):
    assert describe_power_event(ev) == text


def test_event_49_eoa_trip_is_named_and_explained():
    assert PowerPeriphAux.POWER_EVENT_NAMES[49] == 'eoa_trip'
    assert '10 mA' in PowerPeriphAux.POWER_EVENT_EXPLANATIONS[49]
