#!/usr/bin/env python3
from collections import deque
from stretch4_body.core.device import Device
import time
import stretch4_body.core.hello_utils as hello_utils
from stretch4_body.robot.robot_core import RobotCore
from stretch4_body.core.client_server import StretchBodyServer
from stretch4_body.behavior.safe_motions.safe_motion_manager import SafeMotionManager
from stretch4_body.behavior.sentries.sentry_manager import SentryManager
from stretch4_body.behavior.routines.routine_manager import RoutineManager
from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop
from stretch4_body.subsystem.power_periph import PowerPeriphDefn
from stretch4_body.subsystem.power_health import PowerHealthEvaluator, PowerHealthThresholds
from stretch4_body.utils.freeable_file_lock import FreeableFileLock

import logging
import importlib
from colorama import Fore, Back, Style, init
import psutil
import os


# ###########################################################################################
LOOP_STATE_INVALID = "INVALID"
LOOP_STATE_RUNNING = "RUNNING"
LOOP_STATE_PAUSED = "PAUSED"


class RobotServer(RobotCore):
    """
    Extend a standard RobotCore instance with additional RobotController specific subsystems.
    This runs as an independent process, and is managed by the RobotServer.
    """

    def __init__(self):
        RobotCore.__init__(self)
        # Extend RobotCore with additional RobotServer specific subsystems and behaviors

        # Handle end_of_arm in Robot instead of RobotCore, as RobotServer will handle it differently
        if 'end_of_arm' in self.params['subsystems']:
            from stretch4_body.subsystem.end_of_arm.end_of_arm_loop import EndOfArmLoop
            self.eoa_name = self.params['tool']
            self.eoa_loop = self.end_of_arm = self.subsystems['end_of_arm'] = EndOfArmLoop()
            if self.robot_params['robot']['enable_rate_log']:
                self.eoa_loop.enable_rate_logging(self.robot_params['robot']['max_rate_log_samples'])
            self.status['end_of_arm'] = self.subsystems['end_of_arm'].status
            self.status_aux['end_of_arm'] = self.subsystems['end_of_arm'].status_aux
            if 'lift' in self.params['subsystems']:
                self.lift.set_i_feedforward_payload(self.robot_params[self.eoa_name]['i_feedforward_payload'])

        # Instantiate server-onlysubsystems
        for k in self.params['server']['subsystems']: #not available to RobotDirect
            if k == 'line_sensor_loop':
                self.line_sensor_loop = self.subsystems[k] = LineSensorLoop()
                if self.robot_params['robot']['enable_rate_log']:
                    self.line_sensor_loop.enable_rate_logging(self.robot_params['robot']['max_rate_log_samples'])
            self.status[k] = self.subsystems[k].status
            self.status_aux[k] = self.subsystems[k].status_aux

        self.server = StretchBodyServer()

        self.do_exit = False
        self.state = LOOP_STATE_INVALID
        self.loop_mgmt = hello_utils.LoopStats("control_loop", self.params['server']['control_loop_rate_Hz'])

        
        # Instantiate sentry
        self.sentry_manager=SentryManager(self)

        # Instantiate safe_motion checks
        self.safe_motion_manager=SafeMotionManager(self)

        # Instantiate routines
        self.routine_manager=RoutineManager(self)

        self.status['server']={'control_loop':self.loop_mgmt.status, 
                                'state': self.state, 
                                'lease_holder': self.server.lease_holder_id, 
                                'lease_holder_priority': self.server.lease_holder_priority,
                                'lease_expiry': self.server.lease_expiry}
        self.status['routines']=self.routine_manager.status
        self.status['server']['status_id'] =0
        self.status['safety_layer']={
            'safe_motion_manager':self.safe_motion_manager.status,'sentry_manager':self.sentry_manager.status}

        self.cmd_results={}
        
        # CPU Monitoring
        self.status['server']['cpu'] = {}
        self.psutil_processes = {}
        self.last_cpu_update = 0.0
        # We delay process map initialization until startup or first update to ensure processes are running?

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


    def startup(self):
        if (RobotCore.startup(self) and
            self.sentry_manager.startup() and
            self.routine_manager.startup() and
            self.safe_motion_manager.startup() and
            self.server.startup()):
            self.logger.info("RobotServer started successfully.")
            self.state = LOOP_STATE_RUNNING
            self._turn_off_eyes_if_animations_disabled()
            self._record_power_firmware_version()
            return True
        else:
            self.logger.error("Failed to start RobotServer.")
            return False

    def is_homed(self):
        eoa=True
        if self.get_subsystem('end_of_arm') is not None:
            eoa=self.subsystems['end_of_arm'].is_homed()
        return eoa and RobotCore.is_homed(self)

    def _record_power_firmware_version(self):
        pp = self.get_subsystem('power_periph')
        if pp is None:
            return
        board_info = getattr(pp, 'board_info', {}) or {}
        self._power_diag_meta['firmware_version'] = board_info.get('firmware_version')
        self._power_diag_meta['protocol_version'] = board_info.get('protocol_version')
        self._update_power_health(force=True)

    def _turn_off_eyes_if_animations_disabled(self):
        """
        The pimu firmware defaults the eye LEDs to an idle glow at power-on.
        """
        if self.robot_params.get('sentry_eye_animations', {}).get('enabled', 1):
            return
        pp = self.get_subsystem('power_periph')
        if pp is None or not pp.hw_valid:
            return
        protocol = pp.board_info.get('protocol_version')
        if protocol is None or int(protocol[1:]) < 13:
            return
        self.logger.info('sentry_eye_animations disabled. Turning eye LEDs off.')
        pp.set_eye_animation(PowerPeriphDefn.EYE_ANIM_OFF, PowerPeriphDefn.EYE_ANIM_OFF)
        pp.push_command()

    def pause_sentry(self, sentry_name):
        self.sentry_manager.pause([sentry_name])

    def unpause_sentry(self, sentry_name):
        self.sentry_manager.unpause([sentry_name])

    def _cb_admin_dispatch(self,message):

        if message == b"ping":
            print("RobotServer PING!")
            self.logger.debug('RobotServer PING!')
            return message

        if message == b"pause" and self.state == LOOP_STATE_RUNNING:
            self.logger.warn('RobotServer PAUSE!')
            self.safe_motion.enter_safe_stop()
            self.pause_transport()
            hello_utils.free_transport_filelock(self.name)
            self.state = LOOP_STATE_PAUSED
            return message

        if message == b'unpause' and self.state == LOOP_STATE_PAUSED:
            if not hello_utils.acquire_transport_filelock(self.name):
                self.logger.error('Unable for RobotServer to aquire transport_filelock. Not able to UNPAUSE.')
            else:
                self.logger.warn('RobotServer UNPAUSE!')
                self.safe_motion.exit_safe_stop()
                self.state = LOOP_STATE_RUNNING
                self.unpause_transport()
            return message

        if message == b"free_up_control":
            lock = FreeableFileLock('pusher_client')
            lock.free()
            self.server.lease_holder_id = None # free up lease
            self.server.lease_holder_priority = None
            return message

        if message == b"kill":
            self.logger.warn('RobotServer KILL!')
            hello_utils.free_transport_filelock(self.name)
            self.stop()
            self.server.stop()
            self.state = LOOP_STATE_INVALID
            self.do_exit = True
            return message
        return None



    def _cb_command_dispatch(self,cmd_dict):
        """
        #Recieve a dict of commands from client with subsystem -> command key-value mapping
        # A command has the form (subsystem_name,command_name,args,kwargs,cmd_id)
        #Dispatch each command to the appropriate subsystem

        """
        cmd_ids_dispatched=[]
        if self.state == LOOP_STATE_PAUSED or self.state == LOOP_STATE_INVALID:
            return cmd_ids_dispatched #Ignore commands while paused, discard

        is_routine_active = self.routine_manager.status['active_routine'] != 'routine_nop'

        for cmd in cmd_dict:
            subsystem, method, cmd_id, args, kwargs = cmd_dict[cmd]

            if subsystem =='routines':
                if method == 'cancel':
                    self.routine_manager.cancel(*args, **kwargs)
                else:
                    self.routine_manager._routine_set_next(method,cmd_id,*args,**kwargs)
                
                cmd_ids_dispatched.append(cmd_id)
                continue

            if is_routine_active:
                # Reject non-routine commands while a routine is executing
                continue

            # Handle robot commands:
            subsystem_instance = self

            if subsystem !='robot':
                if 'end_of_arm' in subsystem:
                    subsystem = 'end_of_arm' # remove joint from e.g. "end_of_arm.wrist_yaw" to "end_of_arm"
                elif 'omnibase' in subsystem:
                    subsystem = 'omnibase'
                subsystem_instance = self.get_subsystem(subsystem)

            if subsystem_instance is not None:
                if subsystem=='end_of_arm':

                    self.eoa_loop.q_cmd.put(cmd_dict[cmd])
                    cmd_ids_dispatched.append(cmd_id)
                else:
                    try:
                        method_to_call=getattr(subsystem_instance,method)
                        self.cmd_results[cmd_id]={'ts':time.time(),'result':method_to_call(*args, **kwargs)}
                        cmd_ids_dispatched.append(cmd_id)
                    except AttributeError:
                        self.logger.error(f'RobotServer _cb_command_dispatch : invalid  cmd {cmd}')
            else:
                self.logger.error(f'RobotServer not able to run command {method} as subsystem {subsystem} is not present')
        # Cleanup cmd results to keep max history using O(1) operations
        MAX_CMD_HISTORY = 5000
        while len(self.cmd_results) > MAX_CMD_HISTORY:
            # In Python 3.7+, dicts maintain insertion order. iter() gets the oldest key.
            oldest_key = next(iter(self.cmd_results))
            del self.cmd_results[oldest_key]
        return cmd_ids_dispatched

    def publish_status_msg(self):
        self.update_cpu_status()
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
        self.status['server']['state']=self.state
        self.status['server']['status_id'] = self.status['server']['status_id'] + 1
        self.status['server']['lease_holder'] = str(self.server.lease_holder_id)
        self.status['server']['lease_holder_priority'] = str(self.server.lease_holder_priority)
        self.status['server']['lease_expiry'] = self.server.lease_expiry
        
        self.server.publish_status(self.status)

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

    def update_cpu_status(self):
        # Throttle to 1Hz
        if time.time() - self.last_cpu_update < 1.0:
            return
        self.last_cpu_update = time.time()

        # Initialize psutil processes on first run if empty, but only if running

        if not self.psutil_processes and self.state == LOOP_STATE_RUNNING:
             # Main process
            try:
                self.psutil_processes['server'] = psutil.Process(os.getpid())
            except psutil.NoSuchProcess:
                pass

            # End of Arm
            if hasattr(self, 'eoa_loop') and self.eoa_loop.eoa_process:
                try:
                    self.psutil_processes['end_of_arm'] = psutil.Process(self.eoa_loop.eoa_process.pid)
                except psutil.NoSuchProcess:
                    pass
            
            # Line Sensor
            if hasattr(self, 'line_sensor_loop') and self.line_sensor_loop.pjr_process:
                try:
                    self.psutil_processes['line_sensor'] = psutil.Process(self.line_sensor_loop.pjr_process.pid)
                except psutil.NoSuchProcess:
                    pass

            # Self Collision Loop
   
            # Located in sentry_manager -> sentries['self_collision_loop']
            if 'sentry_self_collision' in self.sentry_manager.sentries:
                 scl = self.sentry_manager.sentries['sentry_self_collision'].self_collision_loop
                 if scl.solver_process:
                     try:
                        self.psutil_processes['self_collision'] = psutil.Process(scl.solver_process.pid)
                     except psutil.NoSuchProcess:
                        pass

        # Update status
        for name, proc in self.psutil_processes.items():
            try:
                # cpu_percent(interval=None) is non-blocking after first call
                self.status['server']['cpu'][name] = proc.cpu_percent(interval=None)
            except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
                self.status['server']['cpu'][name] = -1.0


    def _print_server_state(self):
        if time.time() - self.t_print_last > 1.0:
            msg = (f'StretchBodyServer : State: {self.state} | Routine: {self.routine_manager.status['active_routine'].upper()} | Runtime {self.status['server']['control_loop']['execution_time_s']:.8f} (s) | Rate {self.status['server']['control_loop']['avg_rate_hz']:.2f} (Hz)')
            print(msg)
            self.logger.debug(msg)
            self.t_print_last = time.time()

    def run_controller(self):
        self.t_print_last = time.time()
        use_tqdm=0
        if use_tqdm:
            from tqdm import tqdm
            pbar = tqdm(unit="hz")
        self.loop_mgmt.reset()
        self.logger.info("""
==================================

The Stretch Body Server Control Loop is running.

Use `stretch_body_server --help` for available options.
Use `stretch_status --rate 10` to view the status of the robot.

==================================
""")
        try:
            while not self.do_exit:

                if self.state == LOOP_STATE_RUNNING:
                    self.routine_manager._routine_run_next()
                else:
                    if self.state == LOOP_STATE_PAUSED or self.state == LOOP_STATE_INVALID:
                        self.loop_mgmt.mark_loop_start()
                        self.loop_mgmt.busy_wait_until_next_cycle(busy_wait_ms=0.1, warn_delay=5.0, overrun_thresh_s=0.005)
                        self.server.dispatch_admin_messages(self._cb_admin_dispatch)
                        self.server.dispatch_command_messages(self._cb_command_dispatch, self.routine_manager.status['active_routine'] != 'routine_nop')
                        self.publish_status_msg()
                        self.loop_mgmt.mark_loop_end()
                if use_tqdm:
                    pbar.update(1)
                else:
                    self._print_server_state()

        except KeyboardInterrupt:
            self.logger.info("Received request to stop stretch body server via keyboard interrupt.")
        except Exception as e:
            self.logger.error(f"Error in Stretch Body Server's Control Loop. Control Loop has ended. Error message: {e}.")


    def cb_routine_update_controller(self):
        """
        Called by a routine in order to pull status/ push command etc
        Return True if success / Routine should continue
        Return False if no success / Routine should stop gracefully and exit

        This is effectively called in the middle of the control cycle. The control cycle is
        1. (Mark start) Do admin of the control loop
        2. Wait until start of cycle
        3. Trigger a new pull_status (non blocking)
        4. Update sentries
        5. Ingest new control commands from Client
        6. Run routines, possibly overriding control commands
        7. Run safe motion limits, possibly overriding control commands
        8. Push commands to uC
        9. Collect the results of the pull status (for next control cycle) and post to Client
        10. Mark end of loop.

        As this is called from a Routine, we start the cycle at step #7 and wrap around up to 5.
        """

        # 7
        self.safe_motion_manager.step()

        # 8
        self.push_command(blocking=False)
        if self.get_subsystem('power_periph') is not None:
            self.power_periph.trigger_motor_sync(blocking=False)

        # 9
        self.load_rpc_results(wait_on_result=True)
        self._complete_power_periph_aux_status()
        self.publish_status_msg()


        # 10
        self.loop_mgmt.mark_loop_end()

        # 1
        self.loop_mgmt.mark_loop_start()
        self.server.dispatch_admin_messages(self._cb_admin_dispatch)# May pause/exit
        if self.state is not LOOP_STATE_RUNNING:
            return False
        # 2
        self.loop_mgmt.busy_wait_until_next_cycle(busy_wait_ms=1.0, warn_delay=5.0, overrun_thresh_s=0.005)

        # 3
        self.pull_status(blocking=False)
        self._queue_power_periph_aux_status()

        # 4 Step sentries
        self.sentry_manager.step()

        # 5 Get new commands from client
        self.server.dispatch_command_messages(self._cb_command_dispatch, self.routine_manager.status['active_routine'] != 'routine_nop')

        self._print_server_state()
        return True

    def stop(self):
        self.sentry_manager.stop()
        self.safe_motion_manager.stop()
        self.routine_manager.stop()
        RobotCore.stop(self)
        if hasattr(self, 'server') and hasattr(self.server, 'stop'):
            self.server.stop()



# ###########################################################################################
def run_server():
    """
    Launch the RobotController as a seperate process and manage the network interface to it in
    the loop below.
    """
    # num_cores = os.cpu_count()
    rs = RobotServer()
    if not rs.startup():
        rs.logger.error('Failure to start RobotServer')
        rs.stop()
        exit(1)

    profile_enabled = os.environ.get('STRETCH_PROFILE') == '1'
    if profile_enabled:
        try:
            import yappi
            yappi.start()
        except ImportError:
            print("yappi not installed. Profiling disabled.")
            profile_enabled = False

    rs.run_controller()

    if profile_enabled:
        try:
            import yappi
            import sys
            import io
            print("\n" + "="*20 + " PROFILING SUMMARY: STRETCH_BODY_SERVER " + "="*20, file=sys.stderr)
            yappi.stop()
            stats = yappi.get_func_stats()
            stats.sort('ttot', 'desc')
            s = io.StringIO()
            stats.print_all(out=s)
            lines = s.getvalue().splitlines()
            print('\n'.join(lines[:25]), file=sys.stderr)
            print("="*80 + "\n", file=sys.stderr)
        except Exception as e:
            print(f"Error printing profile summary: {e}", file=sys.stderr)
    rs.stop()

# ###########################################################################################

if __name__ == '__main__':
    run_server()
