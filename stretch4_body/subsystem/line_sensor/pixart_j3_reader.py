#!/usr/bin/env python3
import serial
import time
import numpy as np
import json


class PixartJ3Reader():
    """
    PixartJ3Reader reads firehose of line sensor ranges for 6 Pixart sensors
    It parses incoming data and updates the status dictionary.
    The step() method polls the serial port, should be called as fast as possible (~1khz)

    """
    def __init__(self,port_name='/dev/hello-pixart-j3',verbose=False,bus_sensor_map=None):

        # Sensor 0: Bus 2, dev 0
        # Sensor 1: Bus 2, dev 1
        # Sensor 2: Bus 1, dev 0
        # Sensor 3: Bus 1, dev 1
        # Sensor 4: Bus 3, dev 0
        # Sensor 5: Bus 3, dev 1
        # Map is indexed by [bus - 1][device number]
        if bus_sensor_map is None:
            self.bus_sensor_map = [ [ 2, 3 ],[ 0, 1 ],[ 4, 5 ] ]
        else:
            self.bus_sensor_map=bus_sensor_map


        self.port_name=port_name
        self.DEBUG_ENABLED = False
        self.verbose=verbose
        self.reader_thread=None

        self.sensors_seen={}
        self.step_count=0
        self.last_frame_id=None
        self.last_second = 0
        self.last_hz = 0
        self.frames_this_sec = 0
        self.PIXART_REPORT_NUM = 320
        self.sensors_this_frame = 0
        self.msg = "\n"
        self.json_line = ""
        self.oob_line = ""
        self.line_count = 0

        self.status = {'frame_advance_err':0,'not_six_sensors_err':0,'frame_not_full_err':0,'rate_hz':0,'sensors_last_frame':[],'last_frame_time':0}
        for i in range(6):
            self.status['sensor_%d'%i]={'ts_last_read':0,'frame_id':0,'rate_hz':0,'ranges':[]}
        self.is_valid=False
    
    def startup(self):
        try:
            self.debug_print("Attempting to open", self.port_name)
            # Open the serial port
            self.ser = serial.Serial(port=self.port_name)
            self.verbose_print(f"Serial port {self.port_name} opened successfully.")
            self.json_line = ""
            self.oob_line = ""
            self.line_count = 0;
            self.is_valid=True
            return True
        except serial.SerialException as e:
            print(f"PixartJ3Reader: Error opening or communicating with serial port: {e}")
            #time.sleep(0.20)    ## TODO: REMOVE ME
            return False
        except Exception as e:
            print(f"PixartJ3Reader: An unexpected error occurred: {e}")
            return False

    def step(self):
        #Return true if status is updated with new sensor data
        updated=False

        if not self.is_valid:
            return updated
        
        if not self.ser.is_open:
            self.startup()
            return updated
        try:
            while self.ser.in_waiting > 0:
                if not self.ser.is_open:
                    self.is_valid=False
                    return updated
                lines = self.ser.read(self.ser.in_waiting).decode('utf-8').splitlines(True)
                for line in lines:
                    ## Accumulate text line (segment(s)) into JSON line, or out of band line.
                    if self.json_line:
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
                        continue;
                    self.line_count += 1
                    if self.oob_line:
                        if self.line_count > 1 or not self.oob_line.endswith("}"):
                            # Don't print first line if it ends with a curly brace, it might have legitimately started mid-stream.
                            self.debug_print("OOB:", self.oob_line)
                        self.oob_line = ""
                        if self.json_line:
                            print("SCRIPT ERROR: Expected blank JSON line")
                            self.is_valid = False
                        continue
                    if not self.json_line.endswith("}"):
                        if self.line_count > 1:
                            # Don't print on the first line because it may have legitimately started mid-stream.
                            self.debug_print("JSON line didn't find expected close curly brace before newline. Ignoring. Line:", self.json_line)
                        self.json_line = ""
                    else:
                        #self.debug_print(self.json_line)
                        data = json.loads(self.json_line)
                        frame_id = "UNKNOWN"
                        if "frameId" in data:
                            frame_id = data.get("frameId")
                        if self.last_frame_id==None:
                            self.last_frame_id=frame_id-1 #Init first time
                        foundMatch = False
                        for bus_number in range(1, 4):
                            for sensor_number in range(2):
                                key = f"distances{bus_number:1d}{sensor_number:1d}"
                                if key in data:
                                    ranges = np.array(data.get(key, self.PIXART_REPORT_NUM), dtype=float)/1000.0
                                    global_sensor_index_number = self.bus_sensor_to_index_number(bus_number, sensor_number)
                                    if len(ranges) != self.PIXART_REPORT_NUM:
                                        self.debug_print("Found", len(ranges), "elements, expected", self.PIXART_REPORT_NUM)
                                        self.debug_print("line:", line)
                                        #self.debug_print("DecodedData:", decodedData)
                                    else:
                                        self.process_one_sensor(frame_id, global_sensor_index_number, ranges)
                                        updated=True
                                    foundMatch = True
                                    if frame_id > self.last_frame_id:
                                        self.last_frame_id = frame_id
                                        if (self.sensors_this_frame != 6):
                                            self.debug_print("** Received", self.sensors_this_frame, "this frame")
                                        self.debug_print("New FrameId:", frame_id, "sensor:", global_sensor_index_number)
                                        self.sensors_this_frame = 1
                                    else:
                                        if frame_id == self.last_frame_id:
                                            self.sensors_this_frame += 1
                                        else:
                                            self.debug_print("*******OUT OF ORDER FrameId:", frame_id, "sensor:", global_sensor_index_number)
                                            self.is_valid = False
                                    break
                            if foundMatch:
                                break
                        self.json_line = ""
                        self.oob_line = ""
        except serial.SerialException as e:
            print(f"PixartJ3Reader: Error opening or communicating with serial port: {e}")
            self.is_valid = False
            #time.sleep(0.50) ### TODO: REMOVE ME
            self.ser.close()
        except Exception as e:
            print(f"PixartJ3Reader: An unexpected error occurred: {e}")
            self.is_valid = False
        return updated

    def debug_print(self,*args, **kwargs):
        if self.DEBUG_ENABLED:
            print("PixartJ3Reader(d):", *args, **kwargs)
    def verbose_print(self,*args, **kwargs):
        if self.verbose:
            print("PixartJ3Reader(v):", *args, **kwargs)
    def bus_sensor_to_index_number(self,bus, sensor):
        return self.bus_sensor_map[(bus - 1)][sensor]

    def error_check_sensor_list(self, sensor_dict):
        err_str = "  "
        err_count = 0
        for i in range(6):
            if not i in sensor_dict or sensor_dict[i] != 1:
                err_count += 1
                if err_count > 1:
                    err_str += ", "
                if not i in sensor_dict:
                    err_str += f"missing {i}"
                else:
                    err_str += f"extra {i}"
        return err_str

    def process_frame(self):
        if len(self.sensors_seen) == 0:
            # Nothing to publish
            return
        if len(self.sensors_seen) != 6:
            self.status['not_six_sensors_err'] += 1
        now = time.time()
        self.status['rate_hz'] = 1 / (now - self.status['last_frame_time'])
        self.status['last_frame_time'] = now
        self.status['sensors_last_frame'] = self.sensors_seen

        self.verbose_print(f"FrameId: {self.last_frame_id} last hz: {self.last_hz/6:.2f}({self.status['rate_hz']:.2f}), now: {time.time():.3f}  Sensors seen: {self.sensors_seen.keys()} {self.error_check_sensor_list(self.sensors_seen)}")


        # Reset everything
        self.sensors_seen = {}


    def process_one_sensor(self, frame_id, sensor_index, ranges):
        self.step_count = self.step_count + 1
        # print("Dq#", self.step_count, "FrameId:", frame_id, "sensor:", sensor_index)
        self.verbose_print("Dq#", self.step_count, "FrameId:", frame_id, "sensor:", sensor_index)

        if frame_id != self.last_frame_id:
            if frame_id != (self.last_frame_id + 1):
                self.verbose_print(f"** FrameId did not advance by 1: {self.last_frame_id} -> {frame_id}")
                self.status['frame_advance_err'] = self.status['frame_advance_err'] + 1
            if len(self.sensors_seen):
                self.status['frame_not_full_err'] += 1
                # Flush out any previous sensor data:
                self.process_frame()

            self.last_frame_id = frame_id

        # Track which sensors are in the current frame
        if sensor_index in self.sensors_seen:
            self.sensors_seen[sensor_index] += 1
        else:
            self.sensors_seen[sensor_index] = 1

        # Assimilate new data into status
        sn = 'sensor_%d' % sensor_index
        dt = time.time() - self.status[sn]['ts_last_read']
        self.status[sn]['ts_last_read'] = time.time()
        self.status[sn]['rate_hz'] = 1 / dt #This may have jitter around 30hz as the order sensor sent from uC may vary frame to frame
        self.status[sn]['ranges'] = ranges
        self.status[sn]['frame_id'] = frame_id


        ## track number of frames each second
        this_second = int(time.time())
        if self.last_second != this_second:
            self.last_second = this_second
            self.last_hz = self.frames_this_sec
            self.frames_this_sec = 0
        self.frames_this_sec += 1

        if len(self.sensors_seen) == 6:
            ## All sensors have been seen, publish
            self.process_frame()

    def stop(self):
        # Close the serial port if it was opened
        if 'ser' in locals() and self.ser.is_open:
            self.ser.close()
            self.verbose_print("Serial port closed.")
        self.debug_print("Exposer/reader process done")


if __name__ == '__main__':
    pjr = PixartJ3Reader(verbose=False)
    try:
        if pjr.startup():
            while True:
                tt=time.time()
                pjr.step()
                dt=time.time()-tt
                time.sleep(.004)
                #print('------%f----------'%(dt*1000))
                for i in range(6):
                    print('Rate sensor %d:'%i,pjr.status['sensor_%d'%i]['rate_hz'])
                # print('---')
                # print('Rate hz',lsr.status['rate_hz'])
                # print('Frame advance error',lsr.status['frame_advance_err'])
                # print('Frame not full error',lsr.status['frame_not_full_err'])
                # print('Not six sensors error',lsr.status['not_six_sensors_err'])
    except KeyboardInterrupt:
        pass
    pjr.stop()
