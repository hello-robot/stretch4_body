#!/usr/bin/env python3
"""
Lidar idle sentry.

Cuts power to the head lidar rail when nothing on the NUC consumes lidar data, and
restores it when a consumer appears. Detection is passive: a background thread reads
/proc/net/{udp,udp6,tcp,tcp6} once per poll period and counts

  * UDP sockets bound to the lidar data ports (hesai driver, pyhesai, raw listeners)
  * TCP sessions to the lidar PTC port (config tools, REx_lidar --status, firmware upgrade)

The rail is switched only through power_periph.set_lidar_on/off (PIMU trigger bits 17/18).

Manual hold: a set_lidar_on from a client (any caller other than this sentry) holds the
rail on. The hold ends on a client set_lidar_off (rail off, sentry keeps it off until a
consumer appears), on a client release_lidar_hold (sentry resumes, idle countdown restarts),
after hold_timeout_s if that is > 0 (0: no timeout), or at server restart.
"""
import os
import threading
import time

from stretch4_body.behavior.sentries.sentry import Sentry

TCP_ESTABLISHED = '01'
TCP_SYN_SENT = '02'
STATE_ACTIVE = 'STATE_ACTIVE'


def _decode_addr(field):
    """'0100007F:0940' -> ('127.0.0.1', 2368). IPv6 rows map IPv4-mapped addresses to dotted form."""
    hex_ip, hex_port = field.split(':')
    port = int(hex_port, 16)
    if len(hex_ip) == 8:
        b = bytes.fromhex(hex_ip)[::-1]
        return '.'.join(str(x) for x in b), port
    words = [bytes.fromhex(hex_ip[i:i + 8])[::-1] for i in range(0, 32, 8)]
    raw = b''.join(words)
    if raw[:12] == b'\x00' * 10 + b'\xff\xff':
        return '.'.join(str(x) for x in raw[12:]), port
    return ':'.join(raw[i:i + 2].hex() for i in range(0, 16, 2)), port


def parse_proc_net(text, proto, udp_ports, ptc_port=None, lidar_ips=None):
    """
    Return the lidar consumer sockets in one /proc/net/{udp,udp6,tcp,tcp6} text.
    proto: 'udp' or 'tcp'. Each entry: {'proto', 'port', 'inode', 'uid'}.
    UDP rows count when the local port is a lidar data port, in any state.
    TCP rows count when the remote port is ptc_port, the remote address is a lidar
    (when lidar_ips is given), and the state is ESTABLISHED or SYN_SENT.
    """
    out = []
    udp_hex = {'%04X' % p for p in udp_ports}
    ptc_hex = None if ptc_port is None else '%04X' % ptc_port
    for line in text.splitlines()[1:]:
        f = line.split()
        if len(f) < 10:
            continue
        # Cheap port test on the hex text first; most rows are not lidar rows.
        if proto == 'udp' and f[1][-4:] not in udp_hex:
            continue
        if proto == 'tcp' and f[2][-4:] != ptc_hex:
            continue
        try:
            _, lport = _decode_addr(f[1])
            rip, rport = _decode_addr(f[2])
            uid, inode = int(f[7]), int(f[9])
        except ValueError:
            continue
        if proto == 'udp':
            if lport in udp_ports:
                out.append({'proto': 'udp', 'port': lport, 'inode': inode, 'uid': uid})
        elif ptc_port is not None and rport == ptc_port and f[3] in (TCP_ESTABLISHED, TCP_SYN_SENT):
            if not lidar_ips or rip in lidar_ips:
                out.append({'proto': 'tcp', 'port': rport, 'inode': inode, 'uid': uid})
    return out


class ProcNetScanner:
    """Reads /proc once per scan(). Process names are resolved only when the socket set changes."""

    def __init__(self, udp_ports, ptc_port=None, lidar_ips=None, proc_root='/proc'):
        self.udp_ports = set(udp_ports)
        self.ptc_port = ptc_port
        self.lidar_ips = set(lidar_ips or [])
        self.proc_root = proc_root
        self._names = {}
        self._inodes = frozenset()

    def _read(self, name):
        try:
            with open(os.path.join(self.proc_root, 'net', name)) as f:
                return f.read()
        except OSError:
            return ''

    def _resolve(self, inodes):
        """Map socket inodes to (pid, name) through /proc/<pid>/fd. Same-user processes only."""
        found = {}
        want = {'socket:[%d]' % i: i for i in inodes}
        try:
            pids = [p for p in os.listdir(self.proc_root) if p.isdigit()]
        except OSError:
            return found
        for pid in pids:
            fd_dir = os.path.join(self.proc_root, pid, 'fd')
            try:
                fds = os.listdir(fd_dir)
            except OSError:
                continue
            for fd in fds:
                try:
                    link = os.readlink(os.path.join(fd_dir, fd))
                except OSError:
                    continue
                if link in want:
                    try:
                        with open(os.path.join(self.proc_root, pid, 'comm')) as f:
                            name = f.read().strip()
                    except OSError:
                        name = '?'
                    found[want[link]] = (int(pid), name)
            if len(found) == len(want):
                break
        return found

    def scan(self):
        socks = []
        for name in ('udp', 'udp6'):
            socks += parse_proc_net(self._read(name), 'udp', self.udp_ports)
        if self.ptc_port is not None:
            for name in ('tcp', 'tcp6'):
                socks += parse_proc_net(self._read(name), 'tcp', (), self.ptc_port, self.lidar_ips)
        inodes = frozenset(s['inode'] for s in socks)
        if inodes != self._inodes:
            self._inodes = inodes
            self._names = self._resolve(inodes) if inodes else {}
        consumers = []
        for s in socks:
            pid, name = self._names.get(s['inode'], (None, '?'))
            consumers.append({'proto': s['proto'], 'port': s['port'], 'pid': pid, 'name': name})
        return consumers


def _describe(consumers):
    return ', '.join('%s pid %s %s %d' % (c['name'], c['pid'], c['proto'], c['port']) for c in consumers[:4])


class SentryLidarIdle(Sentry):
    """Powers the head lidars down once no lidar data consumer is left (after idle_timeout_s, 0 by default)."""

    def __init__(self, robot):
        Sentry.__init__(self, name='sentry_lidar_idle', robot=robot)
        p = self.params
        self.idle_timeout_s = float(p.get('idle_timeout_s', 0))
        self.min_on_s = float(p.get('min_on_s', 0))
        self.poll_period_s = max(1.0, float(p.get('poll_period_s', 1.0)))
        self.hold_timeout_s = float(p.get('hold_timeout_s', 0))
        self.pimu_state_max_age_s = float(p.get('pimu_state_max_age_s', 2.0))
        self.restore_on_stop = bool(p.get('restore_on_stop', 1))
        self.scanner = ProcNetScanner(
            udp_ports=p.get('udp_ports', [2368, 2378]),
            ptc_port=p.get('ptc_port', 9347) if p.get('ptc_counts_as_consumer', 1) else None,
            lidar_ips=p.get('lidar_ips', []))
        self.clock = time.monotonic
        self._snapshot = None  # (monotonic time, consumers) written by the scan thread
        self._thread = None
        self._stop_evt = threading.Event()
        self._rail_on = True
        self._source = 'firmware_wake'
        self._held = False
        self._hold_until = None
        self._t_idle_ref = None
        self._t_on = None
        self._cmd_seq = None
        self._last_pimu = None
        self.status = {
            'rail_cmd': 'on',
            'rail_cmd_source': self._source,
            'rail_cmd_ts': time.time(),
            'held': False,
            'hold_remaining_s': None,
            'consumers': [],
            'idle_s': 0.0,
            'seconds_to_off': None,
            'pimu_state': 'unknown',
            'scan_age_s': None,
            'on_events': 0,
            'off_events': 0,
            'last_action': 'startup',
            'last_reason': 'assume on: firmware powers the lidars at wake',
        }

    # ---------------------------------------------------------------- lifecycle
    def startup(self):
        if not super().startup():
            return False
        now = self.clock()
        self._t_idle_ref = now
        self._t_on = now
        pp = self.robot.power_periph
        self._cmd_seq = getattr(pp, 'lidar_cmd', {}).get('seq')
        if self._thread is None:
            self._stop_evt.clear()
            self._thread = threading.Thread(target=self._scan_loop, name='lidar_idle_scan', daemon=True)
            self._thread.start()
        self.logger.info('lidar idle sentry started (idle_timeout %.0f s, hold_timeout %.0f s)'
                         % (self.idle_timeout_s, self.hold_timeout_s))
        return True

    def _scan_loop(self):
        while not self._stop_evt.is_set():
            try:
                self._snapshot = (self.clock(), self.scanner.scan())
            except Exception as e:  # never let a /proc oddity kill the thread
                self.logger.warning('lidar idle scan failed: %s' % e)
            self._stop_evt.wait(self.poll_period_s)

    def stop(self):
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        if (self.restore_on_stop and not self._rail_on and self._source == 'sentry'
                and self._pimu_state(self.clock()) == 'active'):
            self._command(True, 'restore on server stop', self.clock())
        return True

    def step(self):
        if not self.is_valid:
            return
        self.update(self.clock(), self._snapshot)

    # ---------------------------------------------------------------- logic
    def _pimu_state(self, now):
        """'active', 'inactive' or 'unknown' (no sample, or older than pimu_state_max_age_s)."""
        meta = getattr(self.robot, '_power_aux_meta', None) or {}
        t = meta.get('sample_monotonic_s')
        if not meta.get('available') or t is None or now - t > self.pimu_state_max_age_s:
            return 'unknown'
        name = self.robot.power_periph.status_aux.get('current_pwr_state_name')
        return 'active' if name == STATE_ACTIVE else 'inactive'

    def _set_state(self, on, source, reason, now):
        self._rail_on = on
        self._source = source
        self._t_idle_ref = now
        if on:
            self._t_on = now
            self.status['on_events'] += 1
        else:
            self.status['off_events'] += 1
        self.status['rail_cmd_ts'] = time.time()
        self.status['last_action'] = 'on' if on else 'off'
        self.status['last_reason'] = reason
        self.logger.info('lidar rail %s by %s (%s)' % ('ON' if on else 'OFF', source, reason))

    def _command(self, on, reason, now):
        pp = self.robot.power_periph
        if on:
            pp.set_lidar_on(source='sentry')
        else:
            pp.set_lidar_off(source='sentry')
        self._set_state(on, 'sentry', reason, now)

    def _client_commands(self, now):
        cmd = getattr(self.robot.power_periph, 'lidar_cmd', None)
        if not cmd or cmd.get('seq') == self._cmd_seq:
            return
        self._cmd_seq = cmd.get('seq')
        if cmd.get('source') == 'sentry':
            return
        action = cmd.get('action')
        if action == 'on':
            self._held = True
            self._hold_until = now + self.hold_timeout_s if self.hold_timeout_s > 0 else None
            self._set_state(True, 'client', 'manual hold', now)
        elif action == 'off':
            self._held = False
            self._hold_until = None
            self._set_state(False, 'client', 'manual off', now)
        elif action == 'release' and self._held:
            self._held = False
            self._hold_until = None
            self._t_idle_ref = now
            self.status['last_action'] = 'release'
            self.status['last_reason'] = 'manual hold released, idle countdown restarted'
            self.logger.info('lidar hold released by client, idle countdown restarted')

    def update(self, now, snapshot):
        self._client_commands(now)

        if self._held and self._hold_until is not None and now >= self._hold_until:
            self._held = False
            self._hold_until = None
            self._t_idle_ref = now
            self.status['last_action'] = 'release'
            self.status['last_reason'] = 'hold_timeout_s elapsed, idle countdown restarted'
            self.logger.info('lidar hold expired after %.0f s, idle countdown restarted' % self.hold_timeout_s)

        pimu = self._pimu_state(now)
        if pimu != 'unknown':
            if pimu == 'active' and self._last_pimu == 'inactive':
                # Firmware powers the rail at every wake.
                self._set_state(True, 'firmware_wake', 'PIMU returned to ACTIVE', now)
            self._last_pimu = pimu

        consumers = snapshot[1] if snapshot is not None else None
        if consumers:
            self._t_idle_ref = now
            if not self._rail_on:
                if pimu == 'active':
                    self._command(True, 'consumer ' + _describe(consumers), now)
                else:
                    self.status['last_reason'] = 'consumer waiting, PIMU state %s' % pimu
        elif consumers is not None and self._rail_on and not self._held:
            idle = now - self._t_idle_ref
            if idle >= self.idle_timeout_s and now - self._t_on >= self.min_on_s and pimu == 'active':
                self._command(False, 'idle %.0f s, no consumer' % idle, now)

        s = self.status
        s['rail_cmd'] = 'on' if self._rail_on else 'off'
        s['rail_cmd_source'] = self._source
        s['held'] = self._held
        s['hold_remaining_s'] = None if self._hold_until is None else round(self._hold_until - now, 1)
        s['consumers'] = consumers or []
        s['pimu_state'] = pimu
        s['scan_age_s'] = None if snapshot is None else round(now - snapshot[0], 2)
        if self._rail_on and not self._held and not consumers:
            s['idle_s'] = round(now - self._t_idle_ref, 1)
            s['seconds_to_off'] = round(max(self.idle_timeout_s - s['idle_s'],
                                            self.min_on_s - (now - self._t_on), 0.0), 1)
        else:
            s['idle_s'] = 0.0
            s['seconds_to_off'] = None
