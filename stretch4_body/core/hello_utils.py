import yaml
import math
import os
import pwd
import time
import logging
import numpy as np
import sys
import signal
import pathlib
import numbers
import subprocess
import cv2
import sys, tty, termios
import math
from statistics import mean
import json
import glob
from colorama import Fore, Back, Style, init
import psutil
import click

from stretch4_body.utils.file_access_utils import acquire_lock_if_available, release_lock, setup_shared_directory

def print_stretch_re_use():
    print("For use with S T R E T C H (R) from Hello Robot Inc.")
    print("---------------------------------------------------------------------\n")

# Periodic printing (every second)
qprint = (lambda msg, fg=None, bold=False, state={"last": 0}:
          (click.secho(msg, fg=fg, bold=bold), state.update(last=time.monotonic()))
          if time.monotonic() - state["last"] >= 1 else None)

def get_keystroke():

    fd=sys.stdin.fileno()
    old_settings=termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch=sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd,termios.TCSADRAIN,old_settings)
    return ch


def create_time_string(time_format='%Y%m%d%H%M%S'):
    """Returns current time formatted as `time_format`

    Parameters
    ----------
    time_format : str
        Refer https://docs.python.org/3/library/time.html#time.strftime for options

    Returns
    -------
    str
        time as string in requested format
    """
    return time.strftime(time_format)

def deg_to_rad(x):
    return math.pi*x/180.0

def rad_to_deg(x):
    return 180.0*x/math.pi

def confirm(question):
    reply = None
    while reply not in ("y", "n"):
        reply = input(question + " (y/n)").lower()
    return (reply == "y")


def get_display():
    return os.environ.get('DISPLAY', None)

def get_fleet_id():
    return os.environ['HELLO_FLEET_ID']

def set_fleet_id(id):
    os.environ['HELLO_FLEET_ID']=id

def get_fleet_path():
    return os.environ['HELLO_FLEET_PATH']+'/'

def get_fleet_directory():
    return os.environ['HELLO_FLEET_PATH']+'/'+get_fleet_id()+'/'


def set_fleet_directory(fleet_path,fleet_id):
    os.environ['HELLO_FLEET_ID'] = fleet_id
    os.environ['HELLO_FLEET_PATH'] = fleet_path


def display_most_recent_robot_rates():
    import matplotlib
    matplotlib.use('tkagg')
    import matplotlib.pyplot as plt
    log_files = glob.glob(get_stretch_directory()+'log/robot_rate_log/robot_rate_log_*')
    if len(log_files)>0:
        log_files.sort()
        with open(log_files[-1],'r') as f:
            data=json.load(f) #Data will be nested dicts with list as final element
            n_hist=0
            #Flatten the data
            flat_data={}
            for k in data.keys():
                if type(data[k])==list:
                    flat_data[k]=data[k]
                    n_hist+=1
                if type(data[k])==dict:
                    for k2 in data[k].keys():
                        flat_data[k2]=data[k][k2]
                        n_hist+=1
            if n_hist>0:
                n_row=int(math.sqrt(n_hist))
                n_col=int(math.ceil(n_hist/n_row))
                fig, axs = plt.subplots(n_row, n_col, sharey=True, tight_layout=True)
                print('RATES',n_row,n_col)
                for i,k in enumerate(flat_data.keys()):
                    print('II',i,k,i//n_col,i%n_col,axs)
                    if n_hist>1:
                        axs[i//n_col,i%n_col].hist(flat_data[k], bins=100, color='#0504aa', alpha=0.7, rwidth=0.85)
                        axs[i//n_col,i%n_col].set_title(k.upper())
                    else:
                        axs.hist(flat_data[k], bins=100, color='#0504aa', alpha=0.7, rwidth=0.85)
                        axs.set_title(k.upper())
                    print('Data for ',k, ': Num samples: ',len(flat_data[k]))
                    # if k=='hello-gs2-2':
                    #     print(flat_data[k])
                plt.show()
            else:
                print('No data to display')


def get_stretch_directory(sub_directory=''):
    """Returns path to stretch_user dir if HELLO_FLEET_PATH env var exists

    Parameters
    ----------
    sub_directory : str
        valid sub_directory within stretch_user/

    Returns
    -------
    str
        dirpath to stretch_user/ or dir within it if stretch_user/ exists, else /tmp
    """
    base_path = os.environ.get('HELLO_FLEET_PATH', None)
    full_path = base_path + '/' + sub_directory if base_path is not None else '/tmp/'
    return full_path

def read_fleet_yaml(f,fleet_dir=None):
    """Reads yaml by filename from fleet directory

    Parameters
    ----------
    f : str
        filename of the yaml

    Returns
    -------
    dict
        yaml as dictionary if valid file, else empty dict
    """
    try:
        if fleet_dir is None:
            fleet_dir=get_fleet_directory()
        else:
            if fleet_dir[-1] != '/':
                fleet_dir = fleet_dir + '/'
        with open(fleet_dir+f, 'r') as s:
            p = yaml.load(s,Loader=yaml.FullLoader)
            return {} if p is None else p
    except IOError:
        return {}

def write_fleet_yaml(fn,rp,fleet_dir=None,header=None):
    if fleet_dir is None:
        fleet_dir = get_fleet_directory()
    if fleet_dir[-1]!='/':
        fleet_dir+='/'
    with open(fleet_dir+fn, 'w') as yaml_file:
        if header is not None:
            yaml_file.write(header)
        yaml.dump(rp, yaml_file, default_flow_style=False)


def overwrite_dict(overwritee_dict, overwriter_dict):
    """Merge two dictionaries while overwriting common keys and
    report errors when values of the same key differ in Python
    type. The result gets stored in `overwritee_dict`.

    Parameters
    ----------
    overwritee_dict : dict
        The dictionary which will be overwritten. Use this as the merged result.
    overwriter_dict : dict
        The dictionary which will overwrite.

    Returns
    -------
    bool
        True if no mismatches were found during the overwrite, False otherwise.
    """
    no_mismatches = True
    for k in overwriter_dict.keys():
        if k in overwritee_dict:
            if (isinstance(overwritee_dict[k], dict) and isinstance(overwriter_dict[k], dict)):
                sub_no_mismatches = overwrite_dict(overwritee_dict[k], overwriter_dict[k])
                no_mismatches = no_mismatches and sub_no_mismatches
            else:
                if (type(overwritee_dict[k]) == type(overwriter_dict[k])) or (isinstance(overwritee_dict[k], numbers.Real) and isinstance(overwriter_dict[k], numbers.Real)):
                    overwritee_dict[k] = overwriter_dict[k]
                else:
                    no_mismatches = False
                    print('stretch_body.hello_utils.overwrite_dict ERROR: Type mismatch for key={0}, between overwritee={1} and overwriter={2}'.format(k, overwritee_dict[k], overwriter_dict[k]), file=sys.stderr)
        else: #If key not present, add anyhow (useful for overlaying params)
            overwritee_dict[k] = overwriter_dict[k]
    return no_mismatches

def pretty_print_dict(title, d,level=0):
    """Print human readable representation of dictionary to terminal

    Parameters
    ----------
    title : str
        header title under which the dictionary is printed
    d : dict
        the dictionary to pretty print
    """
    if level==0:
        sep='/'*(max(2,8-level))
    elif level==1:
        sep = '#' * (max(2, 8 - level))
    else:
        sep='-'*(max(2,8-level))
    hdr=sep+' {0} '+sep
    print(hdr.format(title))
    for k in d.keys():
        if type(d[k]) != dict:
            print(k, ' : ', d[k])
    for k in d.keys():
        if type(d[k]) == dict:
            pretty_print_dict(k, d[k],level+1)
    print('')

def force_kill_process(script_name):
    print(f"Searching for all instances of '{script_name}' to force kill...")
    found = False

    # Iterate through all running processes
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Check the full command line for the script name
            cmdline = proc.info.get('cmdline')
            if cmdline is None:
                continue
            is_self = len(cmdline)==3 and cmdline[2]=='--cleanup'
            if cmdline and any(script_name in arg for arg in cmdline) and not is_self:
                pid = proc.info['pid']
                print(f"Forcefully killing {script_name} (PID: {pid})...")

                # proc.kill() sends SIGKILL (kill -9) on Linux
                proc.kill()
                found = True

        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            # Process may have already terminated or requires higher permissions
            continue

    if not found:
        print(f"No processes matching '{script_name}' were found.")

from multiprocessing import   Queue
import queue
class CircularMultiprocessingQueue:
    def __init__(self, maxsize):
        self.queue = Queue(maxsize=maxsize)
        self.maxsize = maxsize

    def put(self, item):
        """
        Manage ring buffer.
        Note: Should never block, but can if there's a race condition on full, so doing multiple tries (hack for now)
        """
        itr=0
        while True:
            if self.queue.full():# Remove the oldest item to make space
                try:
                    self.queue.get_nowait()
                except queue.Empty:# Should not happen if full() is true, but good for robustness
                    pass
            try:
                self.queue.put(item,block=True,timeout=.001)
                return
            except queue.Full:
                itr=itr+1
                print('Full queue race condition. Trying again. Itr %d.'%itr)
                pass

    def get_latest(self):
        #Clear out queue, returning the latest item
        #Return None if N/A
        ret=None
        while True:
            try:
                ret=self.queue.get_nowait()
            except queue.Empty:
                return ret

    def get_nowait(self):
        return self.queue.get_nowait()

    def get(self,block=True,timeout=None):
        return self.queue.get(block,timeout)

    def empty(self):
        return self.queue.empty()

    def full(self):
        return self.queue.full()

    def qsize(self):
        return self.queue.qsize()



class LoopTimer:
    def __init__(self):
        self.reset()

    def reset(self):
        self.loop_total_active_time = 0.0
        self.loop_sum_of_squared_durations = 0.0
        self.loop_iterations = 0
        self.loop_min_time = 1000000.0
        self.loop_max_time = 0.0
        self.loop_current_time = 0.0
        self.loop_iteration_duration = 0.0
        self.loop_total_active_time = 0.0
        self.loop_average_duration = 0.0
        self.loop_average_sum_of_squared_durations = 0.0
        self.loop_duration_standard_deviation = 0.0
        self.loop_recent_timing = []
        self.loop_recent_timing_length = 10

    def start_of_iteration(self):
        self.loop_current_iteration_start_time = time.perf_counter()

    def pretty_print(self, minimum=False):
        if not minimum:
            print()
            print('--- LOOP TIMING ---')
            print('number of iterations =', self.loop_iterations)
            print('average period =', "{:.2f}".format(self.loop_average_duration * 1000.0), 'ms')
            print('period standard deviation =', "{:.2f}".format(self.loop_duration_standard_deviation * 1000.0), 'ms')
            print('min period =', "{:.2f}".format(self.loop_min_time * 1000.0), 'ms')
            print('max period =', "{:.2f}".format(self.loop_max_time * 1000.0), 'ms')
            print('min frequency =', "{:.2f}".format(1.0 / self.loop_max_time), 'Hz')
            print('max frequency =', "{:.2f}".format(1.0 / self.loop_min_time), 'Hz')
            small_std_dev_freq = 1.0 / (self.loop_average_duration + self.loop_duration_standard_deviation)
            high_std_dev_freq = 1.0 / (self.loop_average_duration - self.loop_duration_standard_deviation)
            print('one standard deviation frequencies =',
                  "{:.2f} Hz, {:.2f} Hz".format(small_std_dev_freq, high_std_dev_freq))
            print('average frequency over all time =', "{:.2f}".format(1.0 / self.loop_average_duration), 'Hz')
            recent_length = len(self.loop_recent_timing)
            if recent_length > 0:
                print('average frequency over last ' + str(recent_length) + ' iterations = {:.2f} Hz'.format(
                    1.0 / mean(self.loop_recent_timing)))
            print('-----------------------------------------------')
        else:
            recent_length = len(self.loop_recent_timing)
            if recent_length > 0:
                print('average frequency over last ' + str(recent_length) + ' iterations = {:.2f} Hz'.format(
                    1.0 / mean(self.loop_recent_timing)))

    def end_of_iteration(self):
        self.loop_current_time = time.perf_counter()
        self.loop_iteration_duration = self.loop_current_time - self.loop_current_iteration_start_time
        self.loop_recent_timing.append(self.loop_iteration_duration)
        if len(self.loop_recent_timing) > self.loop_recent_timing_length:
            self.loop_recent_timing.pop(0)
        self.loop_total_active_time = self.loop_total_active_time + self.loop_iteration_duration
        self.loop_iterations = self.loop_iterations + 1
        self.loop_average_duration = self.loop_total_active_time / self.loop_iterations
        self.loop_sum_of_squared_durations = self.loop_sum_of_squared_durations + self.loop_iteration_duration ** 2
        self.loop_average_sum_of_squared_durations = self.loop_sum_of_squared_durations / self.loop_iterations
        self.loop_duration_standard_deviation = math.sqrt(
            self.loop_average_sum_of_squared_durations - self.loop_average_duration ** 2)
        if self.loop_min_time > self.loop_iteration_duration:
            self.loop_min_time = self.loop_iteration_duration
        if self.loop_max_time < self.loop_iteration_duration:
            self.loop_max_time = self.loop_iteration_duration


class LoopStats():
    """Track timing statistics for control loops
    """

    def __init__(self, loop_name, target_loop_rate):
        self.loop_name = loop_name
        self.target_loop_rate = target_loop_rate
        self.ts_0=time.perf_counter()
        self.loop_cycles=0
        self.ts_loop_last=time.perf_counter()
        self.ts_loop_start = None
        self.ts_loop_end = None
        self.last_ts_loop_start = None
        self.status = {'execution_time_s': 0,
                       'curr_rate_hz': 0,
                       'avg_rate_hz': 0,
                       'supportable_rate_hz': 0,
                       'min_rate_hz': float('inf'),
                       'max_rate_hz': 0,
                       'std_rate_hz': 0,
                       'missed_loops': 0,
                       'num_loops': 0,
                       'target_rate_hz':target_loop_rate}
        self.logger = logging.getLogger(self.loop_name)
        self.curr_rate_history = []
        self.supportable_rate_history = []
        self.n_history = 100
        self.debug_freq = 50
        self.sleep_time_s = 0.0
        self.ts_0=time.perf_counter()

    def reset(self):
        """Reset the loop timing statistics and restart the clock.
        """
        self.ts_0 = time.perf_counter()
        self.loop_cycles = 0
        self.ts_loop_last = self.ts_0
        self.ts_loop_start = None
        self.ts_loop_end = None
        self.last_ts_loop_start = None
        self.curr_rate_history = []
        self.supportable_rate_history = []
        self.status['num_loops'] = 0
        self.status['missed_loops'] = 0
        self.status['min_rate_hz'] = float('inf')
        self.status['max_rate_hz'] = 0
        self.status['avg_rate_hz'] = 0
        self.status['curr_rate_hz'] = 0
        self.status['std_rate_hz'] = 0
        self.status['execution_time_s'] = 0

    def pretty_print(self):
        print('--------- TimingStats %s -----------' % self.loop_name)
        print('Target rate (Hz): %.2f' % self.target_loop_rate)
        print('Current rate (Hz): %.2f' % self.status['curr_rate_hz'])
        print('Average rate (Hz): %.2f' % self.status['avg_rate_hz'])
        print('Standard deviation of rate history (Hz): %.2f' % self.status['std_rate_hz'])
        print('Min rate (Hz): %.2f' % self.status['min_rate_hz'])
        print('Max rate (Hz): %.2f' % self.status['max_rate_hz'])
        print('Supportable rate (Hz): %.2f' % self.status['supportable_rate_hz'])
        print('Warnings: %d out of %d' % (self.status['missed_loops'], self.status['num_loops']))

    def mark_loop_start(self):
        self.status['num_loops'] += 1
        self.ts_loop_start=time.perf_counter()

        if self.last_ts_loop_start is None: #Wait until have sufficient data
            self.last_ts_loop_start=self.ts_loop_start
            return

        self.status['curr_rate_hz'] = 1.0 / (self.ts_loop_start - self.last_ts_loop_start)
        self.status['min_rate_hz'] = min(self.status['curr_rate_hz'], self.status['min_rate_hz'])
        self.status['max_rate_hz'] = max(self.status['curr_rate_hz'], self.status['max_rate_hz'])


        # Calculate average and supportable loop rate **must be done before marking loop end**
        if len(self.curr_rate_history) >= self.n_history:
            self.curr_rate_history.pop(0)
        self.curr_rate_history.append(self.status['curr_rate_hz'])
        self.status['avg_rate_hz'] = float(np.mean(self.curr_rate_history))
        self.status['std_rate_hz'] = float(np.std(self.curr_rate_history))
        if len(self.supportable_rate_history) >= self.n_history:
            self.supportable_rate_history.pop(0)
        self.supportable_rate_history.append(1.0 / self.status['execution_time_s'])
        self.status['supportable_rate_hz'] = float(np.mean(self.supportable_rate_history))

        # Log timing stats **must be done before marking loop end**
        if self.status['num_loops'] % self.debug_freq == 0:
            self.logger.debug(f"""--------- TimingStats {self.loop_name} {self.status['num_loops']} -----------
Target rate: {self.target_loop_rate}
Current rate (Hz): {self.status['curr_rate_hz']}
Average rate (Hz): {self.status['avg_rate_hz']}
Standard deviation of rate history (Hz): {self.status['std_rate_hz']}
Min rate (Hz): {self.status['min_rate_hz']}
Max rate (Hz): {self.status['max_rate_hz']}
Supportable rate (Hz): {self.status['supportable_rate_hz']}
Standard deviation of supportable rate history (Hz): {np.std(self.supportable_rate_history)}
Warnings: {self.status['missed_loops']} out of {self.status['num_loops']}
Sleep time (s): {self.sleep_time_s}""")

        self.last_ts_loop_start = self.ts_loop_start

        # Calculate sleep time to achieve desired loop rate
        self.sleep_time_s = (1 / self.target_loop_rate) - self.status['execution_time_s']
        if self.sleep_time_s < 0.0 and time.perf_counter()-self.ts_0>5.0: #Allow 5s for timing to stabilize on startup
            self.status['missed_loops'] += 1
            if self.status['missed_loops'] == 1:
                self.logger.warning(f'Missed target loop rate of {self.target_loop_rate} Hz for {self.loop_name}. Currently {self.status['curr_rate_hz']} Hz')

    def mark_loop_end(self):
        # First two cycles initialize vars / log
        if self.ts_loop_start is None:
            return
        self.ts_loop_end = time.perf_counter()
        self.status['execution_time_s'] = self.ts_loop_end - self.ts_loop_start


    def generate_rate_histogram(self, save=None):
        import matplotlib
        matplotlib.use('tkagg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 1, sharey=True, tight_layout=True)
        fig.suptitle('Distribution of loop rate (Hz). Target of %.2f ' % self.target_loop_rate)
        axs.hist(x=self.curr_rate_history, bins='auto', color='#0504aa', alpha=0.7, rwidth=0.85)
        plt.show() if save is None else plt.savefig(save)

    def get_loop_sleep_time(self):
        """
        Returns
        -------
        float : Time to sleep for to hit target loop rate
        """
        return max(0.0, self.sleep_time_s)

    def wait_until_ready_to_run(self,sleep=.0005):
        if self.ts_loop_start is None:
            time.sleep(.01)
            return True
        while time.perf_counter()-self.ts_loop_start<(1/self.target_loop_rate):
            time.sleep(sleep)

    def wait_until_next_cycle( self,warn_delay=1.0,overrun_thresh_s=0.0,warn_on=True):
        #More effective than wait_until_ready_to_run
        #Aligns self to clock cycles to avoid drift over time
        loop_clock = (time.perf_counter() - self.ts_0)
        target = (1/self.target_loop_rate) * (self.loop_cycles + 1)
        dt = target - loop_clock
        self.loop_cycles = self.loop_cycles + 1
        if dt < -1*abs(overrun_thresh_s):
            if warn_on and (loop_clock>warn_delay): #Ignore at startup
                if not hasattr(self, '_last_warn_ts') or (time.perf_counter() - self._last_warn_ts > 1.0):
                    print(f'{Fore.YELLOW}Warning: {self.loop_name.upper()} loop overrun: {dt} on itr {self.status['num_loops']}{Style.RESET_ALL}')
                    self._last_warn_ts = time.perf_counter()
            # Advance loop cycle to the next valid boundary to drop accumulated sleep debt
            # while maintaining absolute phase lock to ts_0.
            self.loop_cycles = int((time.perf_counter() - self.ts_0) * self.target_loop_rate)
            time.sleep(.0000001)
        else:
            wait_thresh = .0005
            if dt > wait_thresh:
                time.sleep(dt - wait_thresh)
        self.ts_loop_last = loop_clock

    def busy_wait_until_next_cycle(self, busy_wait_ms, warn_delay=1.0, overrun_thresh_s=0.0, warn_on=True):
        # Higher precision version of wait_until_next_cycle.
        # Uses sleep() for most of the duration and busy-waits for the final busy_wait_ms.
        loop_clock = (time.perf_counter() - self.ts_0)
        target_time = (1/self.target_loop_rate) * (self.loop_cycles + 1)
        dt = target_time - loop_clock
        self.loop_cycles = self.loop_cycles + 1

        if dt < -1*abs(overrun_thresh_s):
            if warn_on and (loop_clock > warn_delay):
                if not hasattr(self, '_last_warn_ts') or (time.perf_counter() - self._last_warn_ts > 1.0):
                    print(f'{Fore.YELLOW}Warning: {self.loop_name.upper()} loop overrun: {dt} on itr {self.status['num_loops']}{Style.RESET_ALL}')
                    self._last_warn_ts = time.perf_counter()
            # If we're already late, don't sleep
            # Advance loop cycle to the next valid boundary to drop accumulated sleep debt
            # while maintaining absolute phase lock to ts_0.
            self.loop_cycles = int((time.perf_counter() - self.ts_0) * self.target_loop_rate)
        else:
            busy_wait_s = busy_wait_ms / 1000.0
            sleep_time = dt - busy_wait_s
            if sleep_time > 0.001:  # Only sleep if it's worth it (OS scheduling overhead)
                time.sleep(sleep_time)

            # Busy wait for the remaining time
            while (time.perf_counter() - self.ts_0) < target_time:
                pass

        self.ts_loop_last = (time.perf_counter() - self.ts_0)


class ThreadServiceExit(Exception):
    """
    Custom exception which is used to trigger the clean exit
    of all running threads and the main program.
    """
    pass

#Signal handler, must be set from main thread
def thread_service_shutdown(signum, frame):
    print('Caught signal %d' % signum)
    raise ThreadServiceExit



def pseudo_N_to_effort_pct(joint_name,contact_thresh_N):
    import stretch_body.robot_params
    d = stretch_body.robot_params.RobotParams.get_params()[1] #Get complete param dict
    motor_name = {'arm':'hello-motor-arm', 'lift': 'hello-motor-lift', 'base':'hello-motor-left-wheel'}[joint_name]
    i_feedforward = 0 if joint_name =='base' else d[joint_name]['i_feedforward']
    iMax_name = 'iMax_neg' if contact_thresh_N<0 else 'iMax_pos'
    contact_A = (contact_thresh_N / d[joint_name]['force_N_per_A'])+i_feedforward
    return 100*contact_A / abs(d[motor_name]['gains'][iMax_name])


def check_deprecated_contact_model_base(joint,method_name, contact_thresh_N,contact_thresh ):
    """
    With RE2 we are transitioning entire stretch fleet to use new API (and effort_pct for the contact model)
    Catch older code that is using the older API and require updating of code
    """

    #Check if old parameters still found in YAML
    if ('contact_thresh_max_N' in joint.params) or ('contact_thresh_N' in joint.params):
        msg="Robot is using out-of-date contact parameters"
        msg=msg+'Please run tool RE1_migrate_contacts.py before continuing.\n'
        msg=msg+'For more details, see https://forum.hello-robot.com/t/476 \n'
        msg = msg + 'In method %s.%s' % (joint.name, method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

    #Check if code is passing in old values
    if contact_thresh_N is not None:
        msg='Use of parameter contact_thresh_N is no longer supported\n'
        msg= msg + 'Update your code to use (contact_thresh)\n'
        msg = msg +  'For more details, see https://forum.hello-robot.com/t/476\n'
        msg=msg+'In method %s.%s'%(joint.name,method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

def check_deprecated_contact_model_prismatic_joint(joint,method_name, contact_thresh_pos_N,contact_thresh_neg_N,contact_thresh_pos,contact_thresh_neg ):
    """
    With RE2 we are transitioning entire stretch fleet to use new API (and effort_pct for the contact model)
    Catch older code that is using the older API and require updating of code
    For code that was, for example:
        arm.move_to(x_m=0.1, contact_thresh_pos_N=30.0, contact_thresh_neg_N=-30.0)
    Should now be:
        arm.move_to(x_m=0.1, contact_thresh_pos=pseudo_N_to_effort_pct(30.0),
            contact_thresh_neg=pseudo_N_to_effort_pct(-30.0))
    """

    #Check if old parameters still found in YAML
    if ('contact_thresh_max_N' in joint.params) or ('contact_thresh_N' in joint.params) or ('homing_force_N' in joint.params):
        msg="Robot is using out-of-date contact parameters\n"
        msg=msg+'Please run tool RE1_migrate_contacts.py before continuing.\n'
        msg=msg+'For more details, see https://forum.hello-robot.com/t/476 \n'
        msg = msg + 'In method %s.%s' % (joint.name, method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

    #Check if code is passing in old values
    if contact_thresh_pos_N is not None or contact_thresh_neg_N is not None:
        msg='Use of parameters contact_thresh_pos_N and contact_thresh_neg_N is no longer supported\n'
        msg= msg + 'Update your code to use (contact_thresh_pos, contact_thresh_neg)\n'
        msg = msg +  'For more details, see https://forum.hello-robot.com/t/476\n'
        msg=msg+'In method %s.%s'%(joint.name,method_name)
        print(msg)
        joint.logger.error(msg)
        sys.exit(1)

    #Check if code is passing in new values but not yet migrated
    if contact_thresh_pos is not None or contact_thresh_neg is not None \
            or (contact_thresh_pos is None and contact_thresh_neg is None):
        if ('contact_models' not in joint.params) or ('effort_pct' not in joint.params['contact_models']) or\
                ('contact_thresh_default' not in joint.params['contact_models']['effort_pct']) or\
                ('contact_thresh_homing' not in joint.params['contact_models']['effort_pct']) :
            msg='Effort_Pct contact parameters not available\n'
            msg = msg + 'Please run tool RE1_migrate_contacts.py before continuing.\n'
            msg = msg + 'For more details, see https://forum.hello-robot.com/t/476 \n'
            msg=msg+'In method %s.%s'%(joint.name,method_name)
            print(msg)
            joint.logger.error(msg)
            sys.exit(1)

def check_file_exists(fn):
    if os.path.exists(fn):
        return True
    else:
        print(f"Unable to find file: {fn}")
        return False

def to_parabola_transform(x):
    if x<0:
        return  -1*(abs(x)**2)
    else:
        return x**2

def map_to_range(value, new_min, new_max):
    # Ensure value is between 0 and 1
    value = max(0, min(1, value))
    mapped_value = (value - 0) * (new_max - new_min) / (1 - 0) + new_min
    return mapped_value

def get_video_devices():
    """
    Returns dictionary of all the enumerated video devices found in the robot 
    """
    command = 'v4l2-ctl --list-devices'
    lines = subprocess.getoutput(command).split('\n')
    lines = [l.strip() for l in lines if l != '']
    cameras = [l for l in lines if not ('/dev/' in l)]
    devices = [l for l in lines if '/dev/' in l]

    all_camera_devices = {}
    camera_devices = []
    current_camera = None
    for line in lines:
        if line in cameras:
            if (current_camera is not None) and camera_devices:
                all_camera_devices[current_camera] = camera_devices
                camera_devices = []
            current_camera = line
        elif line in devices:
            camera_devices.append(line)
    if (current_camera is not None) and camera_devices:
        all_camera_devices[current_camera] = camera_devices

    return all_camera_devices

def setup_uvc_camera(device_index, size=None, fps=None, format = None):
    """
    Returns Opencv capture object of the UVC video divice
    """
    cap = cv2.VideoCapture(device_index)
    if format:
        fourcc_value = cv2.VideoWriter_fourcc(*f'{format}')
        cap.set(cv2.CAP_PROP_FOURCC, fourcc_value)
    if size:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, size[0])
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, size[1])
    if fps:
        cap.set(cv2.CAP_PROP_FPS, fps)
    return cap

def get_video_device_port(camera_name):
    """
    Returns the video device port based on the given camera name match
    """
    camera_devices = get_video_devices()
    camera_device = None
    for k,v in camera_devices.items():
        if camera_name in k:
            camera_device = v[0]
            print(f"Found Camera={k} at port={camera_device} ")
            return camera_device
    print('ERROR: Did not find the specified camera_name = ' + str(camera_name))
    return  camera_device


# #################################################################################################
def build_transport_file_lock_path(device_name:str):
    return f'/tmp/stretch_pid_dir/stretch_body_transport_pid_{device_name}.txt'

def acquire_transport_filelock(device_name:str):
    pid_file = pathlib.Path(build_transport_file_lock_path(device_name))
    setup_shared_directory(pid_file.parent)
    return acquire_lock_if_available(str(pid_file), remove_if_exists_and_unused=True)

def free_transport_filelock(device_name:str):
    return release_lock(build_transport_file_lock_path(device_name))

# #################################################################################################
def H0_from_driving_dir(wheel_dia_m,
                        base_radius,
                        forward_dir):
    """
    H0 is the matrix that transforms the base velocity to wheel velocities
    For three wheeled system:
    u[3x1] = H0[3x3] * Vb[3x1]
    u = wheel velocities
    Vb = Twist velocity of the base frame (vx, vy, wz)
    https://control.ros.org/rolling/doc/ros2_controllers/doc/mobile_robot_kinematics.html#omnidirectional-wheeled-mobile-robots
    """
    forwards = {'basquiat': 30.0, 'basquiat+':30,'calder':30}
    gamma = forwards[forward_dir]
    h0 = np.array([[np.sin(np.radians(gamma)), -np.cos(np.radians(gamma)), -base_radius],
                   [np.sin(np.radians(gamma+120.0)), -np.cos(np.radians(gamma+120.0)), -base_radius],
                   [np.sin(np.radians(gamma+240.0)), -np.cos(np.radians(gamma+240.0)), -base_radius]])
    _r = wheel_dia_m/2
    h0 = h0 * (1/_r)
    return h0


def inverse_3x3_matrix(matrix):
    if matrix.shape != (3, 3):
        raise ValueError("Input must be a 3x3 matrix.")
    determinant = np.linalg.det(matrix)
    if determinant == 0:
        raise ValueError("Matrix is singular and cannot be inverted.")
    return np.linalg.inv(matrix)


def rotation_3x3_matrix(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta), np.cos(theta),  0],
                     [0            , 0,              1]])


def get_sounds_dir():
    return str(pathlib.Path(__file__).parent.parent.absolute() / 'media')


def play_sound(filename,player='aplay'):
    if not os.path.exists(filename):
        print(f"Failed to play sound {filename}: File not found")
        return
    try:
        subprocess.Popen([player, filename], stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    except Exception as e:
        print(f"Failed to play sound {filename}: {e}")


class HelloLoggerScreen(logging.Formatter):
    LEVEL_COLORS = {
        logging.DEBUG: Fore.CYAN,
        logging.INFO: Fore.WHITE,
        logging.WARNING: Fore.YELLOW,
        logging.ERROR: Fore.RED,
        logging.CRITICAL: Fore.RED + Style.BRIGHT
    }
    DATE_COLOR = Fore.BLUE
    NAME_COLOR = Fore.GREEN

    def __init__(
        self, 
        fmt="[%(asctime)s] [%(name)s] [%(levelname)s]: %(message)s", 
        datefmt="%m/%d/%Y %H:%M:%S", 
        style='%', 
        validate=False
        ):

        super().__init__(fmt, datefmt, style, validate)
        
        self.base_fmt = fmt
        self.style_char = style
        self.datefmt = datefmt
        self._formatters = {}
        
        for level, lvl_color in self.LEVEL_COLORS.items():
            f = self.base_fmt
            if self.style_char == '%':
                f = f.replace('[%(levelname)s]:', f'{lvl_color}[%(levelname)s]:{Style.RESET_ALL}')
                f = f.replace('[%(asctime)s]', f'{self.DATE_COLOR}[%(asctime)s]{Style.RESET_ALL}')
                f = f.replace('[%(name)s]', f'{self.NAME_COLOR}[%(name)s]{Style.RESET_ALL}')
                f = f.replace('%(message)s', f'{lvl_color}%(message)s{Style.RESET_ALL}')
            elif self.style_char == '{':
                f = f.replace('[%(levelname)s]:', f'{lvl_color}[%(levelname)s]:{Style.RESET_ALL}')
                f = f.replace('[%(asctime)s]', f'{self.DATE_COLOR}[%(asctime)s]{Style.RESET_ALL}')
                f = f.replace('[%(name)s]', f'{self.NAME_COLOR}[%(name)s]{Style.RESET_ALL}')
                f = f.replace('%(message)s', f'{lvl_color}%(message)s{Style.RESET_ALL}')
            elif self.style_char == '$':
                f = f.replace('[%(levelname)s]:', f'{lvl_color}[%(levelname)s]:{Style.RESET_ALL}')
                f = f.replace('[%(asctime)s]', f'{self.DATE_COLOR}[%(asctime)s]{Style.RESET_ALL}')
                f = f.replace('[%(name)s]', f'{self.NAME_COLOR}[%(name)s]{Style.RESET_ALL}')
                f = f.replace('%(message)s', f'{lvl_color}%(message)s{Style.RESET_ALL}')
            
            try:
                self._formatters[level] = logging.Formatter(f, datefmt, style, validate)
            except TypeError:
                self._formatters[level] = logging.Formatter(f, datefmt, style)

    def format(self, record):
        formatter = self._formatters.get(record.levelno)
        if formatter is None:
            try:
                formatter = logging.Formatter(self.base_fmt, self.datefmt, self.style_char, validate=False)
            except TypeError:
                formatter = logging.Formatter(self.base_fmt, self.datefmt, self.style_char)
        return formatter.format(record)

class HelloLoggerFile(logging.Formatter):

    def __init__(
        self, 
        fmt="[%(asctime)s] [%(name)s] [%(levelname)s]: %(message)s", 
        datefmt="%m/%d/%Y %H:%M:%S", 
        style='%', 
        validate=False
        ):

        super().__init__(fmt, datefmt, style, validate)
        
        self.base_fmt = fmt
        self.style_char = style
        self.datefmt = datefmt
        self._formatters = {}

    def format(self, record):
        formatter = self._formatters.get(record.levelno)
        if formatter is None:
            try:
                formatter = logging.Formatter(self.base_fmt, self.datefmt, self.style_char, validate=False)
            except TypeError:
                formatter = logging.Formatter(self.base_fmt, self.datefmt, self.style_char)
        return formatter.format(record)

class LoggerThrottleFilter(logging.Filter):
    def __init__(self, name=""):
        super().__init__(name)
        self.last_log_times = {}

    def filter(self, record):
        throttle_s = getattr(record, 'throttle_s', 0)
        if throttle_s <= 0:
            return True

        now = time.time()
        key = (record.pathname, record.lineno)
        last_time = self.last_log_times.get(key, 0)

        if now - last_time >= throttle_s:
            self.last_log_times[key] = now
            return True

        return False