#!/usr/bin/env python3

import re
import argparse
import time
import os
import sys
import subprocess
import logging
import logging.config
from colorama import Fore, Style
import glob
import tarfile
from datetime import datetime
import fcntl
from pathlib import Path


from stretch4_body.core.client_server import StretchBodyServer
import stretch4_body.core.hello_utils as hu
import stretch4_body.robot.robot_server as robot_server
from stretch4_body.core.robot_params import RobotParams
from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.utils.file_access_utils import is_user_in_group




LOG_DIR = hu.get_stretch_directory('log/stretch_body_logger')
LOG_FILE = os.path.join(LOG_DIR,"stretch_body_server.log")
logger = logging.getLogger('stretch_body_server')

def print_status(robot_client):
    print('---- Stretch Body Server Status ----')
    print('State:              '+robot_client.status['server']['state'])
    print('Target rate (Hz):   '+str(robot_client.status['server']['control_loop']['target_rate_hz']))
    print('Current rate (Hz):  %.2f'%robot_client.status['server']['control_loop']['curr_rate_hz'])
    print('Loop count:         ' + str(robot_client.status['server']['control_loop']['num_loops']))
    print('Loop overruns:      ' + str(robot_client.status['server']['control_loop']['missed_loops']))
    print("")
    print("Use `stretch_body_server --print` to view the latest logs.")
    print("")


def init_logs(log_file:str):

    # Create all parent directories
    file_path = Path(log_file)
    file_path.parent.mkdir(parents=True, exist_ok=True)
    file_path.touch(exist_ok=True)


def archive_session_logs():
    archive_dir = os.path.join(LOG_DIR, 'archive')
    os.makedirs(archive_dir, exist_ok=True)
    
    lock_file = os.path.join(archive_dir, '.archiving.lock')
    with open(lock_file, 'w') as lf:
        try:
            # Block until we acquire the lock to prevent concurrent archiving
            fcntl.flock(lf, fcntl.LOCK_EX)
        except OSError:
            pass

        log_files = glob.glob(os.path.join(LOG_DIR, 'stretch_body_server.log*'))
        valid_files = [f for f in log_files if os.path.exists(f) and os.path.getsize(f) > 0]
        
        if not valid_files:
            # Cleanup 0-byte active logs
            for f in log_files:
                try: os.remove(f)
                except OSError: pass
            return

        timestamp = datetime.now().strftime("%Y%m%d%H%M%S")
        archive_path = os.path.join(archive_dir, f'stretch_body_server_logs_{timestamp}.tar.gz')
        print(f"Archiving {len(valid_files)} session log segments to {archive_path}...")
        
        with tarfile.open(archive_path, "w:gz") as tar:
            for f in valid_files:
                tar.add(f, arcname=os.path.basename(f))
                    
        # Cleanup the active logs so next execution rolls fresh
        for f in log_files:
            try:
                os.remove(f)
            except OSError:
                pass
        print("Log aggregation complete!")

def print_last_log_from_archive():    
    archive_dir = os.path.join(LOG_DIR, 'archive')
    archives = glob.glob(os.path.join(archive_dir, 'stretch_body_server_logs_*.tar.gz'))
    if not archives:
        print(f"No log files found. Start stretch_body_server to initialize logging.")
        return
    
    latest_archive = max(archives, key=os.path.getmtime)
    try:
        with tarfile.open(latest_archive, "r:gz") as tar:
            try:
                member = tar.getmember("stretch_body_server.log")
            except KeyError:
                return
            
            f = tar.extractfile(member)
            if f:
                lines = f.readlines()
                if lines:
                    for line in lines[-10:]:
                        color_print(line.decode('utf-8'))
                    print(f"\nSee archive for full session log: {os.path.join(archive_dir, os.path.basename(latest_archive))}")
    except:
        pass 


#Establish a global variable that will prevent recursive logging so std outputs can be captured in the log file and also streamed to console
_IS_LOGGING = False
class LoggerWriter:
    def __init__(self, level, original_stream):
        self.level = level
        self.original_stream = original_stream

    def write(self, message):
        global _IS_LOGGING
        if _IS_LOGGING:
            self.original_stream.write(message)
            return
        else:
            msg = message.strip()
            if msg:
                _IS_LOGGING = True
                try:
                    logger.log(self.level, msg)
                finally:
                    _IS_LOGGING = False
            
    def flush(self):
        self.original_stream.flush()

def color_print(line):

    level_colors = {
        'DEBUG': Fore.CYAN,
        'INFO': Fore.WHITE,
        'WARNING': Fore.YELLOW,
        'ERROR': Fore.RED,
        'CRITICAL': Fore.RED + Style.BRIGHT
    }
    date_color = Fore.BLUE
    name_color = Fore.GREEN
    
    line = line.rstrip('\n')
    match = re.match(r'^(\[[^\]]+\]) (\[[^\]]+\]) (\[(.*?)\]): (.*)$', line)

    if match:
        time_str, name_str, level_wrapper, level_str, msg = match.groups()
        level_color = level_colors.get(level_str, Fore.WHITE)
        colored_line = f"{date_color}{time_str}{Style.RESET_ALL} {name_color}{name_str}{Style.RESET_ALL} {level_color}{level_wrapper}:{Style.RESET_ALL} {level_color}{msg}{Style.RESET_ALL}"
        print(colored_line)
    else:
        print(line)


def tail_log_file(log_file:str, n:int=50):    

    try:
        Path(log_file).touch(exist_ok=True)
        f = open(log_file, "r")
        if n <= 0:
            f.seek(0, 2)
        else:
            positions = [0] * n
            count = 0
            while True:
                pos = f.tell()
                line = f.readline()
                if not line:
                    break
                positions[count % n] = pos
                count += 1
            
            if count <= n:
                f.seek(0)
            else:
                f.seek(positions[count % n])
        file_id = os.stat(log_file).st_ino
        while True:
            line = f.readline()
            if line:
                color_print(line)
            else:
                try:
                    f.seek(f.tell()) # Clear EOF flag
                    current_stat = os.stat(log_file)
                    if current_stat.st_ino != file_id or current_stat.st_size < f.tell():
                        f.close()
                        f = open(log_file, "r")
                        file_id = os.stat(log_file).st_ino
                        continue
                except FileNotFoundError:
                    pass
                time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError:
        print(f"Log file {log_file} not found. Start the server to create the log file.")
    finally:
        try:
            f.close()
        except:
            pass

def _parse_args():

    parser=argparse.ArgumentParser(description='Interact with the Stretch Body Server')

    group = parser.add_mutually_exclusive_group(required=False)
    group.add_argument("--launch", help="Launch the server from CLI", action="store_true")
    
    group.add_argument("--ping", help="Ping a running server",action="store_true")
    group.add_argument("--status", help="Prints status of the running server",action="store_true")

    parser.add_argument("--print", help="Print the server log to console", action="store_true")
    parser.add_argument("--log_level", help="Set server logging level (DEBUG, INFO, WARN, ERROR, CRITICAL)",default="INFO")

    group.add_argument("--kill", help="Kill a running server",action="store_true")
    group.add_argument("--restart", help="Restart a running server",action="store_true")
    group.add_argument("--cleanup", help="Force kill zombie server processes",action="store_true")
    # group.add_argument("--pause", help="Pause control loop",action="store_true")
    # group.add_argument("--unpause", help="Unpause control loop",action="store_true")
    group.add_argument("--free_up_control", help="Kills the process currently controlling the robot",action="store_true")
    parser.add_argument("--profile", help="Enable yappi CPU profiling on all server processes", action="store_true")

    parser.add_argument("--daemon", help="Starts a daemon that runs in the background on login. Use --kill to stop the service.", action="store_true")
    parser.add_argument("--install_daemon", help="Installs a daemon that runs in the background on login.", action="store_true")
    parser.add_argument("--uninstall_daemon", help="Uninstall so that server will not start automatically on login.", action="store_true")
    group.add_argument("--status_daemon", help="Prints status of the daemon",action="store_true")

    args= parser.parse_args()

    return args

def is_server_active(robot_client:RobotClient, verbose:bool=False) -> bool:
    is_active = robot_client.startup(verbose=verbose, allow_different_user_connection=True) and robot_client.is_server_active()
    robot_client.stop()
    return is_active


def main():
    hu.print_stretch_re_use()

    if not is_user_in_group('users'):
        logging.error(f"Error: This user ({os.getlogin()}) is not a member of the 'users' group. The user should be a member of the 'users' group for locks to work properly.")
        return

    args = _parse_args()

    if args.log_level:
        log_level_str = args.log_level.upper()
        if hasattr(logging, log_level_str):
            print(f"Setting log level to {log_level_str}")
            RobotParams.set_logging_level(log_level_str, handler='console_handler')
            RobotParams.set_logging_level(log_level_str, handler='file_handler')
            # Force the master root logger to be at least as permissive
            RobotParams._robot_params['logging']['root']['level'] = log_level_str
        else:
            print(f"Invalid log level: {args.log_level}")

    user_params, robot_params = RobotParams.get_params()
    log_file = robot_params['logging']['handlers']['file_handler']['filename']
    logging.config.dictConfig(robot_params['logging'])
    init_logs(log_file)

    if args.launch or args.daemon:
        sys.stderr = LoggerWriter(logging.ERROR, sys.__stderr__)

    robot_client = RobotClient()

    if args.cleanup:
        hu.force_kill_process('stretch_body_server')
        exit(0)

    if args.daemon:
        if not start_daemon():
            raise RuntimeError("Could not start the Stretch Body Server system service")
        tail_log_file(log_file)
        return


    if args.install_daemon:
        if not install_daemon():
            raise RuntimeError("Could not install the Stretch Body Server system service")


    if args.uninstall_daemon:
        if not uninstall_daemon():
            raise RuntimeError("Could not uninstall the Stretch Body Server system service")

    if args.launch or args.restart:
        if args.restart:
            if restart_daemon(): # if the daemon is running, restart it. Otherwise, launch in the terminal.
                print("\nTailing logs, press Ctrl+C to exit (server will keep running in the background):")
                tail_log_file(log_file)
                return
            if not robot_client.startup():
                print("Stopping existing server...")
                robot_client.kill_server()
                time.sleep(2.0)  # Wait for shutdown
            else: print("No instances of the server was found. Launching a new instance.")
        else: # args.launch is true
            if is_server_active(robot_client):
                if StretchBodyServer.is_server_owned_by_current_user():
                    print(f"""
===============================================
                
StretchBodyClient: A server is already running.
StretchBodyClient: You can run `stretch_body_server --kill` to forcefully end the running session.
StretchBodyClient: You can run `stretch_body_server --restart` to forcefully restart the running session.
                
===============================================
                
""")
                else:
                    print(f"""
===============================================
                
StretchBodyClient: A server is already running, but it was started by a different user ({StretchBodyServer.get_server_owning_user()}).
StretchBodyClient: You can run `stretch_body_server --kill` to forcefully end the other user's session.
                
===============================================
                
""")
                return

        try:
            if args.profile:
                os.environ['STRETCH_PROFILE'] = '1'
            robot_server.run_server()
        except Exception as e: 
            logger.error(f"Unexpected error while running stretch body server: {e}")
        finally:
            archive_session_logs()
    
        return

    if args.kill:
        if not is_server_active(robot_client):
            print("No server is running.")
            return
        
        logger.info("Received user request to kill stretch body server.")
        if daemon_is_running():
            stop_daemon()

        if not robot_client.startup(verbose=False, allow_different_user_connection=True):
            print("Kill Stretch Body Server Daemon: SUCCESS")
            archive_session_logs()
            return

        robot_client.kill_server()

        if robot_client.is_server_active():
            print("Kill Stretch Body Server: FAIL")
        else:
            print("Kill Stretch Body Server: SUCCESS")
            archive_session_logs()

        return

    if args.status_daemon:
        status_daemon()
        return

    if is_server_active(robot_client):
        if args.print:
            tail_log_file(log_file)

        if args.status:
            print_status(robot_client)
        
        try: 
            robot_client.startup(verbose=False, allow_different_user_connection=True)

            if args.ping:
                for i in range(5):
                    if robot_client.ping_server():
                        print("Successful server ping at time", time.time())
                    else:
                        print("Failed to ping server. Retrying..")
                    time.sleep(0.75)
            
            if args.free_up_control:
                robot_client.free_up_control()

        except Exception as e:
            logger.error(Fore.RED + f"Error connecting to stretch body server: {e}" + Style.RESET_ALL)
        finally:
            robot_client.stop()

    else:
        print("No active server found. Printing tail of archived logs:\n")
        print_last_log_from_archive()
    
    return


def get_service_file_content(log_level: str) -> str:
    """Creates the linux service file that gets copied to ~/.config/systemd/user/"""
    user_home = Path.home()
    repo_root = Path(__file__).resolve().parents[2]
    
    python_path = Path(sys.executable)
        
    hello_fleet_path = os.environ["HELLO_FLEET_PATH"]
    hello_fleet_id = os.environ["HELLO_FLEET_ID"]
    
    exec_cmd = "stretch_body_server --launch"
    if log_level:
        exec_cmd += f" --log_level {log_level}"
        
    path_env = f"{user_home}/.local/bin:{user_home}/bin:{python_path.parent}:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

    service_file_template = f"""\
[Unit]
Description=Stretch Body Server Service
After=network.target
Wants=network.target

[Service]
Type=simple
#Environment="PYTHONUNBUFFERED=1"
Environment="HELLO_FLEET_PATH={hello_fleet_path}"
Environment="HELLO_FLEET_ID={hello_fleet_id}"
Environment="RMW_IMPLEMENTATION=rmw_zenoh_cpp"
Environment="PATH={path_env}"
WorkingDirectory={repo_root}
ExecStart=/bin/bash -c "{exec_cmd}"
ExecStopPost={python_path} -c "from stretch4_body.tools.stretch_body_server import archive_session_logs; print('Stopping daemon...'); archive_session_logs()"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=default.target
"""

    return service_file_template

def _manage_daemon(action:str):
    """Expected action words: install, start, stop, restart, status, uninstall"""
    action_with_suffix = f"{action}ing"
    if action == "stop": action_with_suffix = "stopping"
    elif action == "status": action_with_suffix = "getting status of"
    log_level =  RobotParams._robot_params['logging']['root']['level']
    logger.info(f"{action_with_suffix.capitalize()} Stretch Body Server systemd service...")
    
    user_systemd_dir = Path.home() / ".config" / "systemd" / "user"
    service_file_path = user_systemd_dir / "stretch_body_server.service"

    try:
        if action == "install":
            user_systemd_dir.mkdir(parents=True, exist_ok=True)
            with open(service_file_path, "w") as f:
                f.write(get_service_file_content(log_level))
            subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
            subprocess.run(["systemctl", "--user", "enable", "stretch_body_server.service"], check=True)
            
        elif action == "uninstall":
            subprocess.run(["systemctl", "--user", "stop", "stretch_body_server.service"], check=False)
            subprocess.run(["systemctl", "--user", "disable", "stretch_body_server.service"], check=False)
            if service_file_path.exists():
                service_file_path.unlink()
            subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
            
        elif action == "start":
            user_systemd_dir.mkdir(parents=True, exist_ok=True)
            with open(service_file_path, "w") as f:
                f.write(get_service_file_content(log_level))
            subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
            subprocess.run(["systemctl", "--user", "start", "stretch_body_server.service"], check=True)
            
        elif action == "stop":
            subprocess.run(["systemctl", "--user", "stop", "stretch_body_server.service"], check=True)
            
        elif action == "restart":
            user_systemd_dir.mkdir(parents=True, exist_ok=True)
            with open(service_file_path, "w") as f:
                f.write(get_service_file_content(log_level))
            subprocess.run(["systemctl", "--user", "daemon-reload"], check=True)
            subprocess.run(["systemctl", "--user", "restart", "stretch_body_server.service"], check=True)
            
        elif action == "status":
            subprocess.run(["systemctl", "--user", "status", "stretch_body_server.service", "--no-pager"], check=False)
            subprocess.run(["journalctl", "--user", "-u", "stretch_body_server.service", "-n", "20", "--no-pager"], check=False)
            
        logger.info(f"{action_with_suffix.capitalize()} Stretch Body Server systemd service: SUCCESS")
        return True
    except Exception as e:
        logger.error(f"Error while {action_with_suffix} Stretch Body Server system service: {e}")
        return False

def start_daemon() -> bool:
    """Installs the systemctl service, and then calls restart to (re)start it, if there isn't already an active non-daemon server running. Note: It calls restart in case a service is already running."""
    if not _manage_daemon("install"):
        return False

    if not daemon_is_running() and is_server_active(RobotClient()):
        print("A non-daemon server is already active, please --kill it first before launching the daemon.")
        return True

    # Start the daemon if a SBS server isn't already running on CLI.
    return restart_daemon()

def stop_daemon() -> bool:
    return _manage_daemon("stop")

def restart_daemon() -> bool:
    if not daemon_is_running() and is_server_active(RobotClient()):
        logger.error("A non-daemon server is already active, please --kill it first before launching the daemon.")
        return False
    print("Archiving existing stretch_body_server.log files...")
    archive_session_logs()
    return _manage_daemon("restart")

def status_daemon() -> bool:
    return _manage_daemon("status")

def daemon_is_running() -> bool:
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', 'stretch_body_server.service'],
            capture_output=True,
            text=True,
            check=False
        )
        if result.returncode == 0 and result.stdout.strip() == 'active':
            return True
        else:
            return False
    except FileNotFoundError:
        print("Error: 'systemctl' command not found. Are you on a systemd Linux machine?")
        return False

def uninstall_daemon() -> bool:
    return _manage_daemon("uninstall")

def install_daemon() -> bool:
    return _manage_daemon("install")

if __name__ == "__main__":
    main()

