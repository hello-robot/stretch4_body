#!/usr/bin/env python3
"""
HealthManager: long-lived health reporting inside stretch_body_server.

It evaluates the checks in health_checks.py continuously and publishes the results
in status['health'], which RobotClient pulls like any other status. Status-derived
checks run in the control loop at check_rate_Hz; filesystem checks (USB devices,
calibrations) run in a background process every host_check_period_s.

Health only reports. It never gates commands, routines or safe motions.
"""
import collections
import logging
import signal
import time
from multiprocessing import Process, Event

import stretch4_body.core.hello_utils as hello_utils
from stretch4_body.core.device import Device
from stretch4_body.core.worker_loop import worker_loop
from stretch4_body.behavior.health import health_checks as hc


# ############################## Host check worker ##############################

class _HostCheckWorker:
    def __init__(self, robot_params):
        self.robot_params = robot_params
        self.logger = logging.getLogger('health_manager')


def _cb_host_step(worker, q_cmd, status_out):
    status_out['results'] = hc.run_host_checks(worker.robot_params)
    status_out['ts'] = time.time()
    return True


def _cb_true(worker):
    return True


def _host_check_loop(do_exit, rate_hz, robot_params, q_admin, q_cmd, q_status):
    worker_loop(loop_name='health_host_check_loop', rate_hz=rate_hz,
                worker_instance=_HostCheckWorker(robot_params),
                q_admin=q_admin, q_status=q_status, q_cmd=q_cmd, do_exit=do_exit,
                callback_step=_cb_host_step, callback_pause=_cb_true,
                callback_unpause=_cb_true, callback_exit=_cb_true)


# ################################# Manager #####################################

class HealthManager(Device):
    def __init__(self, robot):
        Device.__init__(self, 'health_manager', req_params=False)
        self.robot = robot
        self.hparams = hc.merged_params(self.robot_params)
        self.enabled = bool(self.hparams.get('enabled', 1))
        # issues: WARN or worse. notices: INFO, states worth knowing that aren't problems (not homed)
        self.status = {'enabled': self.enabled, 'level': hc.OK, 'ts_update': 0.0, 'ts_host_update': 0.0,
                       'num_warn': 0, 'num_fail': 0, 'issues': [], 'notices': [], 'checks': {}}
        self._ts_last_step = 0.0
        self._host_results = []
        self._line_sensor_samples = collections.deque()
        self._counter_samples = collections.deque()  # (ts, line sensor health, server status)
        self.host_process = None
        self.q_admin = self.q_cmd = self.q_status = None
        self.do_exit = Event()

    # ------------------------------------------------------------------ lifecycle
    def startup(self):
        """Never fails server startup: problems are logged and reported in status."""
        Device.startup(self)
        if not self.enabled:
            self.logger.info('HealthManager disabled in params')
            return True
        try:
            self.q_admin = hello_utils.CircularMultiprocessingQueue(10)
            self.q_cmd = hello_utils.CircularMultiprocessingQueue(10)
            self.q_status = hello_utils.CircularMultiprocessingQueue(10)
            rate_hz = 1.0 / max(float(self.hparams['host_check_period_s']), 0.1)
            self.host_process = Process(target=_host_check_loop,
                                        args=(self.do_exit, rate_hz, self.robot_params,
                                              self.q_admin, self.q_cmd, self.q_status))
            self.host_process.start()
        except Exception as e:
            self.logger.error(f'HealthManager could not start its host check process: {e}')
            self.host_process = None
        return True

    def _manage_ctrlC(self, *args):
        self.do_exit.set()

    def stop(self):
        if self.host_process is None:
            return True
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        try:
            self.q_admin.put('exit')
            self.do_exit.set()
            self.host_process.join(timeout=5.0)
            if self.host_process.is_alive():
                self.host_process.terminate()
        finally:
            self.host_process = None
            signal.signal(signal.SIGINT, original_sigint)
        return True

    # ------------------------------------------------------------------ inputs
    def _processes_alive(self):
        """Worker processes the server depends on, by name -> alive."""
        procs = {}
        eoa = getattr(self.robot, 'eoa_loop', None)
        if eoa is not None and getattr(eoa, 'eoa_process', None) is not None:
            procs['end_of_arm'] = eoa.eoa_process.is_alive()
        ls = getattr(self.robot, 'line_sensor_loop', None)
        if ls is not None and getattr(ls, 'pjr_process', None) is not None:
            procs['line_sensor'] = ls.pjr_process.is_alive()
        sentries = getattr(getattr(self.robot, 'sentry_manager', None), 'sentries', {})
        sc = sentries.get('sentry_self_collision')
        scl = getattr(sc, 'self_collision_loop', None)
        if scl is not None and getattr(scl, 'solver_process', None) is not None:
            procs['self_collision'] = scl.solver_process.is_alive()
        ct = sentries.get('sentry_cpu_temp')
        if ct is not None and getattr(ct, 'temp_process', None) is not None:
            procs['cpu_temp'] = ct.temp_process.is_alive()
        if self.host_process is not None:
            procs['health_host_check'] = self.host_process.is_alive()
        return procs

    def _line_sensor_rates(self, now):
        """Per-sensor frame rate from frame_id advance over the last rate window."""
        lss = self.robot.status.get('line_sensor_loop')
        if not lss:
            return None
        names = self.robot_params.get('line_sensor_loop', {}).get('sensor_names', [])
        frame_ids = {sn: (lss.get(sn) or {}).get('frame_id') for sn in names}
        self._line_sensor_samples.append((now, frame_ids))
        window = self.hparams['line_sensor_rate_window_s']
        while len(self._line_sensor_samples) > 2 and now - self._line_sensor_samples[1][0] >= window:
            self._line_sensor_samples.popleft()
        t0, ids0 = self._line_sensor_samples[0]
        if now - t0 < 0.5 * window:
            return None  # Not enough history yet to say anything
        rates = {}
        for sn in names:
            a, b = ids0.get(sn), frame_ids.get(sn)
            if a is not None and b is not None:
                rates[sn] = max(b - a, 0) / (now - t0)
        return rates

    # ------------------------------------------------------------------ step
    def step(self, force=False):
        """Called from the control loop. Throttled to check_rate_Hz."""
        if not self.enabled:
            return
        now = time.time()
        if not force and now - self._ts_last_step < 1.0 / self.hparams['check_rate_Hz']:
            return
        self._ts_last_step = now
        try:
            self._step(now)
        except Exception as e:  # Health must never take down the control loop
            self.logger.warning(f'HealthManager step failed: {type(e).__name__}: {e}')

    def _step(self, now):
        status = self.robot.status
        if self.q_status is not None:
            latest = self.q_status.get_latest()
            if latest is not None and 'results' in latest:
                self._host_results = latest['results']
                self.status['ts_host_update'] = latest.get('ts', now)

        # Counters (loop overruns, line sensor errors) are judged against the oldest
        # sample inside counter_window_s, so one stray overrun doesn't flap the result
        lss = status.get('line_sensor_loop') or {}
        srv = status.get('server') or {}
        self._counter_samples.append((now, dict(lss.get('health') or {}) if lss else None,
                                      {'control_loop': dict(srv.get('control_loop') or {})}))
        while len(self._counter_samples) > 1 and now - self._counter_samples[1][0] >= self.hparams['counter_window_s']:
            self._counter_samples.popleft()
        _, line_sensor_prev, server_prev = self._counter_samples[0]

        results = hc.run_status_checks(
            status, self.robot_params, params=self.hparams,
            line_sensor_rates=self._line_sensor_rates(now),
            line_sensor_prev=line_sensor_prev,
            server_prev=server_prev,
            processes=self._processes_alive(), now=now)

        self._merge(self._host_results + results, now)

    def _merge(self, results, now):
        old = self.status['checks']
        checks = {}
        for r in results:
            prev = old.get(r['id'])
            if prev is not None and prev['level'] == r['level']:
                r['since'] = prev['since']
                r['count'] = prev['count']
            else:
                r['since'] = now
                # count: how many times this check has gone into a non-OK level
                r['count'] = (prev['count'] if prev is not None else 0) + (r['level'] != hc.OK)
                self._log_transition(prev, r)
            checks[r['id']] = r
        for id, prev in old.items():  # A check that stopped being reported (e.g. a counter result)
            if id not in checks and prev['level'] != hc.OK:
                self.logger.info(f'Health {id}: no longer reported')

        # Worst first, so summaries lead with what matters most
        issues = sorted((id for id, r in checks.items() if hc.LEVEL_RANK[r['level']] > 0),
                        key=lambda id: -hc.LEVEL_RANK[checks[id]['level']])
        self.status['checks'] = checks
        self.status['issues'] = issues
        self.status['notices'] = [id for id, r in checks.items() if r['level'] == hc.INFO]
        self.status['num_warn'] = sum(1 for id in issues if checks[id]['level'] == hc.WARN)
        self.status['num_fail'] = len(issues) - self.status['num_warn']
        self.status['level'] = hc.worst_level(checks.values())
        self.status['ts_update'] = now

    def _log_transition(self, prev, r):
        """Log only when a check changes level, so the journal isn't spammed."""
        was = prev['level'] if prev is not None else hc.OK
        if r['level'] == was:
            return
        msg = f'Health {r["id"]}: {was} -> {r["level"]}: {r["message"]}'
        if r['level'] in (hc.OK, hc.INFO):
            self.logger.info(msg)
        elif r['level'] == hc.WARN:
            self.logger.warning(msg)
        else:
            self.logger.error(msg)

    def summary(self, max_items=3):
        """Short text for the server's 1 Hz status line, e.g.
            OK
            OK (Not homed: Arm, Lift, wrist_yaw)
            WARN: Battery SOC = 15% (low — consider charging); also Arm not homed
            FAIL: Lift pos not available; /dev/hello-motor-lift missing
        """
        if not self.enabled:
            return 'DISABLED'
        checks = self.status['checks']

        def describe(ids):
            texts = [hc.short_text(checks[id]) for id in ids if not checks[id].get('joint')]
            unhomed = [checks[id]['joint'] for id in ids if checks[id].get('joint')]
            if len(unhomed) > 1:  # One item for every joint that isn't homed
                texts.insert(0, 'Not homed: ' + ', '.join(unhomed))
            elif unhomed:
                texts.insert(0, f'{unhomed[0]} not homed')
            if len(texts) > max_items:
                texts = texts[:max_items] + [f'+{len(texts) - max_items} more']
            return '; '.join(texts)

        issues, notices = self.status['issues'], self.status['notices']
        if not issues:
            return 'OK' + (f' ({describe(notices)})' if notices else '')
        text = f'{self.status["level"]}: {describe(issues)}'
        return text + (f'; also {describe(notices)}' if notices else '')
