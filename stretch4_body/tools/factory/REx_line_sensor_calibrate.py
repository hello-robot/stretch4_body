#!/usr/bin/env python3
import argparse
import sys
from stretch4_body.subsystem.line_sensor.line_sensor_loop import LineSensorLoop
from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorCalibration

def main():
    parser = argparse.ArgumentParser(
        prog='Calibrate line sensor background model.'
    )
    parser.add_argument('-s', '--sensor_name', metavar='sensor_name',  type=str, nargs=1,
                        help='Provide a single sensor name e.g.: sensor_0')
    parser.add_argument("--all", help="Calibrate all six sensors",action="store_true")
    args = parser.parse_args()

    target_sensors = []
    if args.all:
        # We'll get the names from params later, or hardcode/assume if needed, 
        # but better to use what's in params.
        # LineSensorLoop params will dictate available sensors.
        target_sensors = None # None means all in LineSensorCalibration
    elif args.sensor_name:
        target_sensors = args.sensor_name
    else:
        parser.print_help()
        sys.exit(0)

    print("Starting LineSensorLoop...")
    lsl = LineSensorLoop()
    if not lsl.startup():
        print("Failed to start Line Sensor Loop")
        sys.exit(1)

    try:
        calib = LineSensorCalibration(lsl)
        
        # Verify target sensors exist
        if target_sensors:
            available = lsl.params['sensor_names']
            for s in target_sensors:
                if s not in available:
                    print(f"Error: Sensor {s} not found in configuration: {available}")
                    sys.exit(1)

        print("\n--- Recording Data ---")
        calib.record_data(itrs=500, sensors=target_sensors)
        
        # Note: adjust load_data or skip it? 
        # record_data populates self.data_samples, so we can compute directly.
        # But to be robust and ensure we are using saved data (like the old script implied by separating steps),
        # we could reload. However, the in-memory data same as saved data.
        # Let's just compute.
        
        print("\n--- Computing Tare ---")
        calib.compute_tare(sensors=target_sensors)
        
        print("\n--- Saving Tare ---")
        calib.save_tare(sensors=target_sensors)

    finally:
        lsl.stop()

if __name__ == '__main__':
    main()
