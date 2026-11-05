#!/usr/bin/env python3
"""
HealthManager: long-lived health reporting inside stretch_body_server.

It evaluates the checks in health_checks.py continuously and publishes the results
in status['health'], which RobotClient pulls like any other status. Status-derived
checks run in the control loop at check_rate_Hz. Two background processes do the rest:

    health_host_check_loop    USB devices, ESP32, calibrations; every host_check_period_s
    health_update_check_loop  software updates (software_updates.py); every few hours,
                              never while the server is starting up, only with internet

Health only reports. It never gates commands, routines or safe motions.
"""
import collections
import logging
import os
import signal
import time
from multiprocessing import Process, Event

import stretch4_body.core.hello_utils as hello_utils
from stretch4_body.core.device import Device
from stretch4_body.core.worker_loop import worker_loop
from stretch4_body.behavior.health import health_checks as hc
from stretch4_body.behavior.health import software_updates as su


def _cb_true(worker):
    return True


def _run_worker(loop_name, rate_hz, worker, do_exit, q_admin, q_cmd, q_status, step):
    worker_loop(loop_name=loop_name, rate_hz=rate_hz, worker_instance=worker,
                q_admin=q_admin, q_status=q_status, q_cmd=q_cmd, do_exit=do_exit,
                callback_step=step, callback_pause=_cb_true,
                callback_unpause=_cb_true, callback_exit=_cb_true)


# ############################## Host check worker ##############################

class _HostCheckWorker:
    def __init__(self, robot_params):
        self.robot_params = robot_params
        self.logger = logging.getLogger('health_manager')


def _cb_host_step(worker, q_cmd, status_out):
    status_out['results'] = hc.run_host_checks(worker.robot_params)
    status_out['ts'] = time.time()
    return True


def _host_check_loop(do_exit, rate_hz, robot_params, q_admin, q_cmd, q_status):
    _run_worker('health_host_check_loop', rate_hz, _HostCheckWorker(robot_params),
                do_exit, q_admin, q_cmd, q_status, _cb_host_step)


# ############################# Update check worker #############################

UPDATE_TICK_HZ = 1.0 / 60.0  # Wakes once a minute to see whether anything is due


class _UpdateCheckWorker:
    """
    Network checks run at su.next_check_time(); their result is cached on disk, so a
    restart shows the last result right away without checking again. The cache is judged
    against what's installed (pip metadata, local git) update_local_delay_s after start,
    then every update_local_recheck_s, so an update you install stops being reported.
    """
    def __init__(self, params, boards):
        self.params = params
        self.boards = boards
        self.logger = logging.getLogger('health_manager')
        self.ts_start = time.time()
        self.cache = su.load_cache()
        self.ts_evaluated = 0.0
        self.results = []
        self.firmware = {}
        self.firmware_command = None
        self._judge_firmware()

    def _judge_firmware(self):
        # Pure computation on the cached release list, so it can run right away
        available = self.cache.get('firmware_available') or {}
        self.firmware = su.firmware_recommendations(self.boards, available) if available else {}
        self.firmware_command = su.firmware_command(self.firmware)

    def published(self):
        """Small enough to publish in every status: the remote side, never the release lists."""
        return {'ts_check': self.cache.get('ts_check', 0.0),
                'ts_attempt': self.cache.get('ts_attempt', 0.0),
                'ts_next': su.next_check_time(self.ts_start, self.cache, self.params),
                'ts_evaluated': self.ts_evaluated,
                'online': self.cache.get('online'),
                'pypi_latest': self.cache.get('pypi_latest') or {},
                'repo_remotes': self.cache.get('repo_remotes') or {},
                'firmware_boards': self.boards,
                'firmware': self.firmware,
                'firmware_command': self.firmware_command}


def _cb_update_step(worker, q_cmd, status_out):
    now = time.time()
    p = worker.params
    if now >= su.next_check_time(worker.ts_start, worker.cache, p):
        if su.internet_available():
            worker.cache = su.fetch_remote(worker.boards)
            worker.logger.info('Checked for software updates')
        else:
            worker.cache['online'] = False
        worker.cache['ts_attempt'] = now
        su.save_cache(worker.cache)
        worker._judge_firmware()
        worker.ts_evaluated = 0.0  # Judge the new fetch now
    if (worker.cache.get('ts_check') and now - worker.ts_start >= p['update_local_delay_s']
            and now - worker.ts_evaluated >= p['update_local_recheck_s']):
        worker.results = su.update_results(su.evaluate(worker.cache, worker.boards))
        worker.ts_evaluated = now
    status_out['updates'] = worker.published()
    status_out['results'] = worker.results
    return True


def _update_check_loop(do_exit, params, boards, q_admin, q_cmd, q_status):
    try:
        os.nice(19)  # Never compete with the robot for CPU
    except OSError:
        pass
    _run_worker('health_update_check_loop', UPDATE_TICK_HZ, _UpdateCheckWorker(params, boards),
                do_exit, q_admin, q_cmd, q_status, _cb_update_step)


# ################################# Manager #####################################

class _WorkerHandle:
    def __init__(self, name, target, args):
        self.name = name
        self.q_admin = hello_utils.CircularMultiprocessingQueue(10)
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(10)
        self.q_status = hello_utils.CircularMultiprocessingQueue(10)
        self.process = Process(target=target, args=args + (self.q_admin, self.q_cmd, self.q_status))
        self.process.start()


class HealthManager(Device):
    def __init__(self, robot):
        Device.__init__(self, 'health_manager', req_params=False)
        self.robot = robot
        self.hparams = hc.merged_params(self.robot_params)
        self.enabled = bool(self.hparams.get('enabled', 1))
        # issues: WARN or worse. notices: INFO, states worth knowing that aren't problems (not homed)
        self.status = {'enabled': self.enabled, 'level': hc.OK, 'ts_update': 0.0, 'ts_host_update': 0.0,
                       'num_warn': 0, 'num_fail': 0, 'issues': [], 'notices': [], 'checks': {},
                       'updates': {}}
        self._ts_last_step = 0.0
        self._host_results = []
        self._update_results = []
        self._line_sensor_samples = collections.deque()
        self._counter_samples = collections.deque()  # (ts, line sensor health, server status)
        self.workers = {}  # name -> _WorkerHandle
        self.do_exit = Event()

    # ------------------------------------------------------------------ lifecycle
    def startup(self):
        """Never fails server startup: problems are logged and reported in status."""
        Device.startup(self)
        if not self.enabled:
            self.logger.info('HealthManager disabled in params')
            return True
        rate_hz = 1.0 / max(float(self.hparams['host_check_period_s']), 0.1)
        self._start_worker('health_host_check', _host_check_loop, (self.do_exit, rate_hz, self.robot_params))
        if self.hparams['update_check_enabled']:
            # The boards were read when the robot started, so firmware needs no USB access here
            boards = su.firmware_boards(self.robot)
            self._start_worker('health_update_check', _update_check_loop, (self.do_exit, self.hparams, boards))
        return True

    def _start_worker(self, name, target, args):
        try:
            self.workers[name] = _WorkerHandle(name, target, args)
        except Exception as e:
            self.logger.error(f'HealthManager could not start its {name} process: {e}')

    def _manage_ctrlC(self, *args):
        self.do_exit.set()

    def stop(self):
        if not self.workers:
            return True
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        try:
            for w in self.workers.values():
                w.q_admin.put('exit')
            self.do_exit.set()
            deadline = time.time() + 5.0  # An update check can be mid-request; don't wait on it
            for w in self.workers.values():
                w.process.join(timeout=max(deadline - time.time(), 0.1))
                if w.process.is_alive():
                    w.process.terminate()
        finally:
            self.workers = {}
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
        for name, w in self.workers.items():
            procs[name] = w.process.is_alive()
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
        host = self.workers.get('health_host_check')
        latest = host.q_status.get_latest() if host is not None else None
        if latest is not None and 'results' in latest:
            self._host_results = latest['results']
            self.status['ts_host_update'] = latest.get('ts', now)
        upd = self.workers.get('health_update_check')
        latest = upd.q_status.get_latest() if upd is not None else None
        if latest is not None and 'updates' in latest:
            self._update_results = latest['results']
            self.status['updates'] = latest['updates']

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

        self._merge(self._host_results + results + self._update_results, now)

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
            # Related results (every unhomed joint, every available update) are one item
            texts, groups = [], {}
            for id in ids:
                r = checks[id]
                if r.get('summary_group'):
                    if r['summary_group'] not in groups:
                        groups[r['summary_group']] = []
                        texts.append(r['summary_group'])  # Placeholder, keeps the order
                    groups[r['summary_group']].append(r)
                else:
                    texts.append(hc.short_text(r))
            for name, rs in groups.items():
                if len(rs) == 1:
                    one = hc.short_text(rs[0])
                else:
                    items = [r['summary_item'] for r in rs]
                    more = f' +{len(items) - max_items} more' if len(items) > max_items else ''
                    one = f'{name}: ' + ', '.join(items[:max_items]) + more
                texts[texts.index(name)] = one
            if len(texts) > max_items:
                texts = texts[:max_items] + [f'+{len(texts) - max_items} more']
            return '; '.join(texts)

        issues, notices = self.status['issues'], self.status['notices']
        if not issues:
            return 'OK' + (f' ({describe(notices)})' if notices else '')
        text = f'{self.status["level"]}: {describe(issues)}'
        return text + (f'; also {describe(notices)}' if notices else '')
