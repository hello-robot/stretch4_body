#!/usr/bin/env python3

import time
import psutil
import sys
from stretch_body_ii.robot.robot_client import RobotClient
from stretch_body_ii.core.hello_utils import LoopStats

def main():
    robot = RobotClient()
    if not robot.startup():
        print("Failed to start RobotClient")
        sys.exit(1)
        
    print("Robot Client started.")

    hz = 100.0
    period = 1.0 / hz
    
    print_hz = 10.0
    print_period = 1.0 / print_hz

    client_loop_stats = LoopStats(loop_name='client_test', target_loop_rate=hz)

    # Prime the psutil cpu percent
    psutil.cpu_percent()

    start_time = time.time()
    last_print_time = start_time
    
    print("Starting 100Hz command loop. Press Ctrl+C to exit.")
    
    # Pre-calculate to avoid accumulating drift
    next_loop_time = time.time() + period

    try:
        while True:
            # Stream move_by(0) command
            robot.arm.move_by(0, req_calibration=False)
            robot.lift.move_by(0, req_calibration=False)
            for joint_name in robot.end_of_arm.joints:
                robot.end_of_arm.move_by(joint_name, 0)
            robot.push_command()
            
            # Pull status to get the latest server status
            robot.pull_status()
            
            # Get server status
            server_status = robot.status.get('server', {})
            control_loop = server_status.get('control_loop', {})
            curr_rate_hz = control_loop.get('curr_rate_hz', 0.0)
            avg_rate_hz = control_loop.get('avg_rate_hz', 0.0)
            missed_loops = control_loop.get('missed_loops', 0)
            target_rate_hz = control_loop.get('target_rate_hz', 0)
            server_state = server_status.get('state', 'Unknown')
            server_cpus = server_status.get('cpu', {})
            
            current_time = time.time()
            running_time = current_time - start_time
            
            client_loop_stats.mark_loop_start()
            
            # Print at 10Hz
            if current_time - last_print_time >= print_period:
                sys_cpu_load = psutil.cpu_percent()
                
                cpu_str = ", ".join([f"{k}:{v:.1f}%" for k, v in server_cpus.items() if isinstance(v, (int, float))])
                
                msg = (f"Time: {running_time:6.2f}s | "
                       f"Sys CPU: {sys_cpu_load:5.1f}% | "
                       f"Procs: [{cpu_str}] | "
                       f"Server Target: {target_rate_hz}Hz | "
                       f"Server Avg Rate: {avg_rate_hz:6.2f}Hz | "
                       f"Server Curr Rate: {curr_rate_hz:6.2f}Hz | "
                       f"Server Missed: {missed_loops} | "
                       f"Client Avg: {client_loop_stats.status['avg_rate_hz']:6.2f}Hz | "
                       f"Client Curr: {client_loop_stats.status['curr_rate_hz']:6.2f}Hz")
                
                if running_time > 5.0 and avg_rate_hz > 0 and avg_rate_hz < 90.0:
                    print(f"\033[93m[WARNING] {msg}\033[0m")
                else:
                    print(msg)
                
                last_print_time = current_time
            
            client_loop_stats.mark_loop_end()
            time.sleep(0.01)
            # # Precise sleep to maintain loop rate
            # current_time_for_sleep = time.time()
            # sleep_time = next_loop_time - current_time_for_sleep
            # if sleep_time > 0:
            #     time.sleep(sleep_time)
            #     next_loop_time += period
            # else:
            #     # We fell behind and missed the phase boundary.
            #     # We must forcefully sleep here to ensure the downstream push_command throttle
            #     # doesn't take over the loop timing, which would cause a permanent warning state.
            #     time.sleep(period)
            #     next_loop_time = time.time() + period
                
    except KeyboardInterrupt:
        print("\nInterrupted by user.")
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        robot.stop()
        print("Robot Client stopped.")

if __name__ == '__main__':
    main()
