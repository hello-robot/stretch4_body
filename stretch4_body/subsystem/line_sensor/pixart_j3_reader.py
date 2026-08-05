#!/usr/bin/env python3
import serial
import time
import numpy as np
import json

from stretch4_body.subsystem.line_sensor import protocol


class PixartJ3Reader():
    """
    Reads the firehose of range reports for the 6 PixArt line sensors from the
    hello-pixart-j3 USB serial port and updates the status dictionary.
    step() polls the serial port and should be called as fast as possible
    (~250 Hz or better); each sensor reports at ~30 Hz.

    Per sensor, status['sensor_N'] carries:
        ranges          np.ndarray float64, meters; NaN wherever the bin is
                        not a distance measurement (see protocol.py)
        codes           np.ndarray uint8, per-bin protocol.CODE_* class --
                        the authoritative answer to "was this 5.09 or 5.11",
                        one integer compare, no float tolerances
        n_no_return     count of 5.11 no-return bins in this report
        n_beyond_limit  count of 5.09 beyond-range bins in this report
        missed_frames   consecutive frames this sensor failed to report;
                        0 while healthy (see dead_sensors())
        frame_id, ts_last_read, rate_hz

    Bin ordering (flip_range_ordering) and sentinel classification are both
    applied here at decode, once; downstream code must never re-derive them.
    """

    # A sensor missing this many consecutive frames is reported dead. At ~30 Hz
    # that is ~0.17 s, long enough to ride out a single dropped report.
    DEAD_AFTER_MISSED_FRAMES = 5

    def __init__(self, port_name='/dev/hello-pixart-j3', verbose=False,
                 bus_sensor_map=None, flip_range_ordering=None, report_num=None,
                 _params=None):

        needs_params = (bus_sensor_map is None or flip_range_ordering is None
                        or report_num is None)
        if _params is not None:
            ls_params = _params
        elif needs_params:
            from stretch4_body.core.robot_params import RobotParams
            ls_params = RobotParams.get_params()[1].get('line_sensor_loop', {})
        else:
            ls_params = {}
        if bus_sensor_map is None:
            bus_sensor_map = ls_params.get('bus_sensor_map')
        if bus_sensor_map is None:
            raise ValueError(
                'PixartJ3Reader: bus_sensor_map not passed in and not found in '
                'robot params (line_sensor_loop.bus_sensor_map)')
        if flip_range_ordering is None:
            # No silent default: getting bin order wrong mirrors every reading
            # left-to-right, and nothing downstream can detect it.
            flip_range_ordering = ls_params.get('flip_range_ordering')
        if flip_range_ordering is None:
            raise ValueError(
                'PixartJ3Reader: flip_range_ordering not passed in and not found '
                'in robot params (line_sensor_loop.flip_range_ordering)')
        flip_range_ordering = bool(flip_range_ordering)
        if report_num is None:
            report_num = ls_params.get('line_sensor_geometry', {}).get(
                'pixart_report_num', 320)

        self.bus_sensor_map = bus_sensor_map
        self.key_table = protocol.build_key_table(bus_sensor_map)
        self.flip_range_ordering = flip_range_ordering
        self.PIXART_REPORT_NUM = report_num

        self.port_name = port_name
        self.DEBUG_ENABLED = False
        self.verbose = verbose

        self.sensors_seen = {}
        self.last_frame_id = None
        self.json_line = ""
        self.oob_line = ""
        self.line_count = 0
        self._warned_report_len = False

        self.status = {'frame_advance_err': 0, 'not_six_sensors_err': 0,
                       'frame_not_full_err': 0, 'decode_errors': 0,
                       'rate_hz': 0, 'sensors_last_frame': [],
                       'last_frame_time': 0,
                       # Until a frame arrives nothing has reported, so
                       # everything is dead. Starting empty would make a
                       # totally silent subsystem look healthy.
                       'sensors_dead': ['sensor_%d' % i for i in range(6)]}
        for i in range(6):
            self.status['sensor_%d' % i] = {
                'ts_last_read': 0, 'frame_id': 0, 'rate_hz': 0,
                'ranges': np.zeros(0, dtype=np.float64),
                'codes': np.zeros(0, dtype=np.uint8),
                'n_no_return': 0, 'n_beyond_limit': 0,
                # Never reported yet counts as missing, so a sensor that never
                # comes up at all is reported dead.
                'missed_frames': self.DEAD_AFTER_MISSED_FRAMES}
        self.is_valid = False

    def startup(self):
        try:
            self.debug_print("Attempting to open", self.port_name)
            # Exclusive: two readers on one port each get half the byte stream
            # and both corrupt. Make the second opener fail loudly instead.
            self.ser = serial.Serial(port=self.port_name, exclusive=True)
            self.verbose_print(f"Serial port {self.port_name} opened successfully.")
            self.json_line = ""
            self.oob_line = ""
            self.line_count = 0
            self.is_valid = True
            return True
        except serial.SerialException as e:
            print(f"PixartJ3Reader: Error opening or communicating with serial port: {e}")
            return False
        except Exception as e:
            print(f"PixartJ3Reader: An unexpected error occurred: {e}")
            return False

    def step(self):
        # Return true if status is updated with new sensor data
        updated = False

        if not self.is_valid:
            return updated

        if not self.ser.is_open:
            self.startup()
            return updated
        try:
            while self.ser.in_waiting > 0:
                if not self.ser.is_open:
                    self.is_valid = False
                    return updated
                lines = self.ser.read(self.ser.in_waiting).decode('utf-8').splitlines(True)
                for line in lines:
                    ## Accumulate text line (segment(s)) into JSON line, or out of band line.
                    if self.json_line:
                        # A segment starting with '{' can only be a new report:
                        # the payload has no nested objects, so the sole '{' is
                        # at offset 0. 
                        if line.startswith("{"):
                            self.status['decode_errors'] += 1
                            self.debug_print("Truncated report discarded:",
                                             self.json_line[:80])
                            self.json_line = line.rstrip("\n")
                        else:
                            self.json_line += line.rstrip("\n")
                    else:
                        if self.oob_line:
                            self.oob_line += line.rstrip("\n")
                        else:
                            # This is first after a newline. Assume JSON if leading curly brace.
                            if line.startswith("{"):
                                self.json_line += line.rstrip("\n")
                            else:
                                self.oob_line += line.rstrip("\n")

                    ## Process full line of JSON or OOB
                    if not line.endswith("\n"):
                        continue
                    self.line_count += 1
                    if self.oob_line:
                        if self.line_count > 1 or not self.oob_line.endswith("}"):
                            # Don't print first line if it ends with a curly brace, it might have legitimately started mid-stream.
                            self.debug_print("OOB:", self.oob_line)
                        self.oob_line = ""
                        if self.json_line:
                            print("PixartJ3Reader: Expected blank JSON line")
                            self.json_line = ""
                            self.status['decode_errors'] += 1
                        continue
                    if not self.json_line.endswith("}"):
                        if self.line_count > 1:
                            # Don't print on the first line because it may have legitimately started mid-stream.
                            self.debug_print("JSON line didn't find expected close curly brace before newline. Ignoring. Line:", self.json_line)
                        self.json_line = ""
                    else:
                        if self.process_json_line(self.json_line):
                            updated = True
                        self.json_line = ""
                        self.oob_line = ""
        except serial.SerialException as e:
            print(f"PixartJ3Reader: Error opening or communicating with serial port: {e}")
            self.is_valid = False
            self.ser.close()
        except Exception as e:
            print(f"PixartJ3Reader: An unexpected error occurred: {e}")
            self.is_valid = False
        return updated

    def process_json_line(self, json_line):
        """Decode one complete JSON line (one sensor report). Returns True if
        a sensor's status was updated. Malformed lines are counted and skipped
        rather than killing the reader."""
        try:
            data = json.loads(json_line)
        except ValueError:
            self.status['decode_errors'] += 1
            # If the newline between two reports was lost to dropped bytes,
            # both arrive as one unparsable string. The tail after the last
            # '{' is usually an intact report -- recover it, so a corrupt
            # report does not take its neighbour down with it.
            cut = json_line.rfind("{")
            if cut <= 0:
                self.debug_print("Bad JSON ignored:", json_line[:120])
                return False
            try:
                data = json.loads(json_line[cut:])
            except ValueError:
                self.debug_print("Bad JSON ignored:", json_line[:120])
                return False
            self.debug_print("Recovered report after lost newline:",
                             json_line[:cut][:80])

        frame_id = data.get("frameId")
        if frame_id is None:
            self.status['decode_errors'] += 1
            self.debug_print("JSON line without frameId ignored:", json_line[:120])
            return False

        for key, sensor_index in self.key_table.items():
            if key not in data:
                continue
            payload = data[key]
            if len(payload) != self.PIXART_REPORT_NUM:
                if not self._warned_report_len:
                    print(f"PixartJ3Reader: firmware sent {len(payload)} range "
                          f"elements but pixart_report_num={self.PIXART_REPORT_NUM}; "
                          f"these reports are being dropped -- fix line_sensor_geometry params")
                    self._warned_report_len = True
                self.status['decode_errors'] += 1
                return False
            ranges, codes = protocol.decode_distances_mm(
                payload, flip=self.flip_range_ordering)
            self.process_one_sensor(frame_id, sensor_index, ranges, codes)
            return True

        self.debug_print("JSON line with no known distances key:", json_line[:120])
        self.status['decode_errors'] += 1
        return False

    def process_one_sensor(self, frame_id, sensor_index, ranges, codes):
        now = time.time()

        if frame_id != self.last_frame_id:
            if self.last_frame_id is not None and frame_id != self.last_frame_id + 1:
                # Skipped ahead or went backwards (dropped data / uC restart).
                # Count it and resync on the new frame id rather than dying.
                self.verbose_print(f"** FrameId did not advance by 1: {self.last_frame_id} -> {frame_id}")
                self.status['frame_advance_err'] += 1
            if self.sensors_seen:
                self.status['frame_not_full_err'] += 1
                self.process_frame(now)  # Flush the previous, partial frame
            self.last_frame_id = frame_id

        self.sensors_seen[sensor_index] = self.sensors_seen.get(sensor_index, 0) + 1

        sn = 'sensor_%d' % sensor_index
        s = self.status[sn]
        dt = now - s['ts_last_read']
        if dt > 0:
            s['rate_hz'] = 1.0 / dt  # Jitters around 30hz; uC sensor order varies frame to frame
        s['ts_last_read'] = now
        s['ranges'] = ranges
        s['codes'] = codes
        s['n_no_return'] = int(np.count_nonzero(codes == protocol.CODE_NO_RETURN))
        s['n_beyond_limit'] = int(np.count_nonzero(codes == protocol.CODE_BEYOND_LIMIT))
        s['frame_id'] = frame_id

        if len(self.sensors_seen) == 6:
            self.process_frame(now)

    def process_frame(self, now=None):
        if not self.sensors_seen:
            return
        if now is None:
            now = time.time()
        if len(self.sensors_seen) != 6:
            self.status['not_six_sensors_err'] += 1
        # Per-sensor liveness: a sensor that stops reporting keeps its last
        # ranges in status forever, so without this a dead sensor is
        # indistinguishable from one staring at an unchanging floor.
        dead = []
        for i in range(6):
            s = self.status['sensor_%d' % i]
            if i in self.sensors_seen:
                s['missed_frames'] = 0
            else:
                s['missed_frames'] += 1
            if s['missed_frames'] >= self.DEAD_AFTER_MISSED_FRAMES:
                dead.append('sensor_%d' % i)
        self.status['sensors_dead'] = dead
        last = self.status['last_frame_time']
        if last and now > last:
            self.status['rate_hz'] = 1.0 / (now - last)
        self.status['last_frame_time'] = now
        self.status['sensors_last_frame'] = self.sensors_seen

        self.verbose_print(f"FrameId: {self.last_frame_id} rate: {self.status['rate_hz']:.2f}hz "
                           f"Sensors seen: {list(self.sensors_seen.keys())} {self.error_check_sensor_list(self.sensors_seen)}")
        self.sensors_seen = {}

    def error_check_sensor_list(self, sensor_dict):
        err_str = "  "
        err_count = 0
        for i in range(6):
            if i not in sensor_dict or sensor_dict[i] != 1:
                err_count += 1
                if err_count > 1:
                    err_str += ", "
                if i not in sensor_dict:
                    err_str += f"missing {i}"
                else:
                    err_str += f"extra {i}"
        return err_str

    def debug_print(self, *args, **kwargs):
        if self.DEBUG_ENABLED:
            print("PixartJ3Reader(d):", *args, **kwargs)

    def verbose_print(self, *args, **kwargs):
        if self.verbose:
            print("PixartJ3Reader(v):", *args, **kwargs)

    def bus_sensor_to_index_number(self, bus, sensor):
        return self.bus_sensor_map[(bus - 1)][sensor]

    HEALTH_KEYS = ('rate_hz', 'last_frame_time', 'sensors_dead', 'decode_errors',
                   'frame_advance_err', 'frame_not_full_err', 'not_six_sensors_err')

    def health(self):
        """Subsystem-wide counters, separate from any one sensor's block."""
        h = {k: self.status[k] for k in self.HEALTH_KEYS}
        h['streaming'] = bool(self.is_valid)
        return h

    def dead_sensors(self):
        """Names of sensors that have stopped reporting (or never started)."""
        return list(self.status['sensors_dead'])

    def blind_sensors(self, min_fraction=0.9):
        """Names of sensors that ARE reporting but see almost nothing --
        every bin a no-return. Distinct from dead: the link is fine, the
        sensor just cannot see the floor (very dark surface, or a void)."""
        out = []
        for i in range(6):
            sn = 'sensor_%d' % i
            s = self.status[sn]
            n = len(s['codes'])
            if n and s['missed_frames'] == 0:
                if (s['n_no_return'] + s['n_beyond_limit']) >= min_fraction * n:
                    out.append(sn)
        return out

    def stop(self):
        if hasattr(self, 'ser') and self.ser.is_open:
            self.ser.close()
            self.verbose_print("Serial port closed.")
        self.debug_print("Reader done")


if __name__ == '__main__':
    pjr = PixartJ3Reader(verbose=False)
    try:
        if pjr.startup():
            while True:
                pjr.step()
                time.sleep(.004)
                dead, blind = pjr.dead_sensors(), pjr.blind_sensors()
                for i in range(6):
                    sn = 'sensor_%d' % i
                    s = pjr.status[sn]
                    state = 'DEAD ' if sn in dead else ('BLIND' if sn in blind else 'ok   ')
                    print(f"{sn}: {state} rate {s['rate_hz']:6.2f}hz  "
                          f"no_return(5.11) {s['n_no_return']:3d}  "
                          f"beyond_limit(5.09) {s['n_beyond_limit']:3d}")
                print(f"frame rate {pjr.status['rate_hz']:.2f}hz  "
                      f"decode_errors {pjr.status['decode_errors']}  "
                      f"dead {dead or 'none'}")
                print('---')
    except KeyboardInterrupt:
        pass
    pjr.stop()
