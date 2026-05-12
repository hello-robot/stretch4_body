#!/usr/bin/env python3
import importlib
import threading
import signal
import traceback
from serial import SerialException
import time
from stretch4_body.robot.robot_core import RobotCore
import stretch4_body.core.hello_utils as hello_utils

class Robot(RobotCore):
    """
    Direct API to the Stretch Robot, similar to the original Stretch Body robot.py
    Designed to be compatible with existing scripts (eg, REx, etc)
    It requires the RobotServer to be paused or not running.
    """

    def __init__(self):
        RobotCore.__init__(self,)

        #Handl end_of_arm in Robot instead of RobotCore, as RobotServer will handle it differently
        if 'end_of_arm' in self.params['subsystems']:
            self.eoa_name = self.params['tool']
            module_name = self.robot_params[self.eoa_name]['py_module_name']
            class_name = self.robot_params[self.eoa_name]['py_class_name']
            self.subsystems['end_of_arm'] = getattr(importlib.import_module(module_name), class_name)()
            self.end_of_arm = self.subsystems['end_of_arm']
            self.status['end_of_arm'] = self.subsystems['end_of_arm'].status
            self.status_aux['end_of_arm'] = self.subsystems['end_of_arm'].status_aux


        self.dirty_push_command = False
        self.lock = threading.RLock()  # Prevent status thread from triggering motor sync prematurely
        self.body_thread = None
        self.end_of_arm_thread = None
        self.sys_thread = None
        self.event_loop_thread = None


        self.GLOBAL_EXCEPTIONS_LIST = []
        threading.excepthook = self.custom_excepthook

    def startup(self):
        if RobotCore.startup(self):
            # Register the signal handlers
            signal.signal(signal.SIGTERM, hello_utils.thread_service_shutdown)
            signal.signal(signal.SIGINT, hello_utils.thread_service_shutdown)

            self.body_thread = BodyStatusThread(self, target_rate_hz=self.params['direct']['BodyStatusThread_Hz'])
            self.end_of_arm_thread = EndOfArmStatusThread(self,
                                                          target_rate_hz=self.params['direct']['EOAStatusThread_Hz'])
            self.sys_thread = SystemMonitorThread(self,
                                                  target_rate_hz=self.params['direct']['SystemMonitorThread_Hz'])

            if self.params['direct']['start_body_thread']:
                self.body_thread.daemon = True
                self.body_thread.start()
                ts = time.time()
                while not self.body_thread.first_status and time.time() - ts < 3.0:
                    time.sleep(0.01)

            if self.params['direct']['start_eoa_thread']:
                self.end_of_arm_thread.daemon = True
                self.end_of_arm_thread.start()

            if self.params['direct']['start_sys_mon_thread']:
                self.sys_thread.daemon = True
                self.sys_thread.start()

            return True
        return False

    def custom_excepthook(self, args):
        thread_name = args.thread.name
        Exec = {}
        Exec[thread_name] = {
            'thread': args.thread,
            'exception': {
                'type': args.exc_type,
                'value': args.exc_value,
                'traceback': args.exc_traceback
            }
        }

        # Filter RuntimeError
        if Exec[thread_name]['exception']['type'] == RuntimeError:
            pass
        else:
            # print(f"Caught Exception in Thread: {thread_name}")
            # traceback.print_exception(args.exc_value)
            self.logger.error(f"Caught Exception in Thread: {thread_name}")
            self.logger.error(traceback.format_exception(args.exc_value))

        self.GLOBAL_EXCEPTIONS_LIST.append(Exec[thread_name])


    def enable_collision_mgmt(self):
        pass #self.collision.enable() #legacy

    def disable_collision_mgmt(self):
        pass #self.collision.disable() #legacy

    def get_stow_pos(self, joint):
        """
        Return the stow position of a joint.
        Allow the end_of_arm to override the defaults in order to accomodate stowing different tools
        """
        if self.get_subsystem('end_of_arm') is not None:
            if 'stow' in self.end_of_arm.params:
                if joint in self.end_of_arm.params['stow']:
                    return self.end_of_arm.params['stow'][joint]
        if self.get_subsystem(joint) is not None:
            return self.params['stow'][joint]

        return 0

    def wait_command(self, timeout=15.0, use_motion_generator=True):
        """Pause program execution until all motion complete.

        Queuing up motion and pushing it to the hardware with
        push_command() is designed to be asynchronous, enabling
        reactive control of the robot. However, you might want
        sychronous control, where each command's motion is completed
        entirely before the program moves on to the next command.
        This is where you would use wait_command()

        Parameters
        ----------
        timeout : float
            How long to wait for motion to complete. Must be > 0.1 sec.

        Returns
        -------
        bool
            True if motion completed, False if timed out before motion completed
        """
        time.sleep(0.1)
        timeout = max(0.0, timeout - 0.1)
        done = []
        def check_wait(wait_method):
            done.append(wait_method(timeout, use_motion_generator))
        start = time.time()
        threads = []
        threads.append(threading.Thread(target=check_wait, args=(self.arm.wait_while_is_moving,)))
        threads.append(threading.Thread(target=check_wait, args=(self.base.wait_while_is_moving,)))
        threads.append(threading.Thread(target=check_wait, args=(self.lift.wait_while_is_moving,)))
        [thread.start() for thread in threads]
        [thread.join() for thread in threads]
        return all(done)


    def stow(self):
        """
        Cause the robot to move to its stow position
        Blocking.
        """


        tool = self.params['tool']
        cfg = self.robot_params[tool]['stow']
        self.logger.info(f'Stowing robot for tool {tool}')
        pos_lift = cfg['lift']
        arm_pos = cfg['arm']

        lift_stowed = False
        if self.get_subsystem('lift') is not None:
            self.logger.info('--------- Pre-Stowing Lift ----')
            self.lift.move_to(0.35)
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 4.0:
                time.sleep(0.1)


        if self.get_subsystem('end_of_arm') is not None:
            # Run pre stow specific to each end of arm
            self.end_of_arm.pre_stow(self)

        if self.get_subsystem('arm') is not None:
            # Bring in arm before bring down
            self.logger.info('--------- Stowing Arm ----')
            self.arm.move_to(self.get_stow_pos('arm'))
            self.push_command()
            time.sleep(0.25)
            ts = time.time()
            while not self.arm.motor.status['near_pos_setpoint'] and time.time() - ts < 6.0:
                time.sleep(0.1)

        if self.get_subsystem('end_of_arm') is not None:
            self.end_of_arm.stow()
            time.sleep(0.25)

        if self.get_subsystem('lift') is not None:
            # Now bring lift down
            if not lift_stowed:
                self.logger.info('--------- Stowing Lift ----')
                self.lift.move_to(pos_lift)
                self.push_command()
                time.sleep(0.5)
                ts = time.time()
                while not self.lift.motor.status['near_pos_setpoint'] and time.time() - ts < 12.0:
                    time.sleep(0.1)



    def home(self):
        """
        Cause the robot to home its joints by moving to hardstops
        Blocking.
        """
    

        if self.get_subsystem('lift') is not None:
            self.logger.info('--------- Homing Lift ----')
            self.lift.home()

        # Home the arm
        if self.get_subsystem('arm') is not None:
            self.logger.info('--------- Homing Arm ----')
            self.arm.home()

        if self.get_subsystem('end_of_arm') is not None:
            self.logger.info('--------- Homing EndOfArm ----')
            self.end_of_arm.home()



        # Let user know it is done
        if self.get_subsystem('power_periph') is not None:
            self.power_periph.trigger_beep()
            self.push_command()


    def stop(self):
        """
        To be called once before exiting a program
        Cleanly stops down motion and communication
        """
        if self.body_thread:
            if self.body_thread.running:
                self.body_thread.shutdown_flag.set()
                self.body_thread.join(1)
        if self.end_of_arm_thread:
            if self.end_of_arm_thread.running:
                self.end_of_arm_thread.shutdown_flag.set()
                self.end_of_arm_thread.join(1)
        if self.sys_thread:
            if self.sys_thread.running:
                self.sys_thread.shutdown_flag.set()
                self.sys_thread.join(1)


        RobotCore.stop(self)

    # ################ Helpers #################################

    def _pull_status_end_of_arm(self):
        try:
            if self.get_subsystem('end_of_arm') is not None:
                self.subsystems['end_of_arm'].pull_status()
        except SerialException:
            self.logger.warning('Serial Exception on Robot._pull_status_end_of_arm')



    def _pull_status_body(self):
        if self.get_subsystem('omnibase') is not None:
            self.subsystems['omnibase'].pull_status()
        if self.get_subsystem('power_periph') is not None:
            self.subsystems['power_periph'].pull_status()
        if self.get_subsystem('arm') is not None:
            self.subsystems['arm'].pull_status()
        if self.get_subsystem('lift') is not None:
            self.subsystems['lift'].pull_status()



    def _step_sentry(self):
        if self.get_subsystem('omnibase') is not None:
            self.subsystems['omnibase'].step_sentry(self.status)
        if self.get_subsystem('arm') is not None:
            self.subsystems['arm'].step_sentry(self.status)
        if self.get_subsystem('lift') is not None:
            self.subsystems['lift'].step_sentry(self.status)
        if self.get_subsystem('end_of_arm') is not None:
            self.subsystems['end_of_arm'].step_sentry(self.status)

class EndOfArmStatusThread(threading.Thread):
    """
    This thread polls the status data of the Feetech devices
    at 15Hz
    """

    def __init__(self, robot, target_rate_hz=15.0):
        threading.Thread.__init__(self, name=self.__class__.__name__)
        self.robot = robot
        self.robot_update_rate_hz = target_rate_hz
        self.stats = hello_utils.LoopStats(loop_name='EndOfArmStatusThread', target_loop_rate=self.robot_update_rate_hz)
        self.shutdown_flag = threading.Event()
        self.running = False

    def step(self):
        self.stats.mark_loop_start()
        self.robot._pull_status_end_of_arm()
        self.stats.mark_loop_end()

    def run(self):
        self.running = True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
        self.robot.logger.debug('Shutting down EndOfArmStatusThread')


class BodyStatusThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry, Monitor functions
    """

    def __init__(self, robot, target_rate_hz=25.0):
        threading.Thread.__init__(self, name=self.__class__.__name__)
        self.robot = robot
        self.robot_update_rate_hz = target_rate_hz
        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='BodyStatusThread', target_loop_rate=self.robot_update_rate_hz)
        self.titr = 0
        self.first_status = False
        self.running = False

    def step(self):
        self.stats.mark_loop_start()
        self.robot._pull_status_body()
        self.stats.mark_loop_end()

    def run(self):
        self.running = True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
            self.first_status = True
        self.stop()
        self.robot.logger.debug('Shutting down BodyStatusThread')

    def stop(self):
        pass


class SystemMonitorThread(threading.Thread):
    """
    This thread runs at 25Hz.
    It updates the status data of the Devices.
    It also steps the Sentry, Monitor functions
    """

    def __init__(self, robot, target_rate_hz=25.0):
        threading.Thread.__init__(self, name=self.__class__.__name__)
        self.robot = robot
        self.robot_update_rate_hz = target_rate_hz
        self.monitor_downrate_int = int(robot.params['direct'][
                                            'SystemMonitorThread_monitor_downrate_int'])  # Step the monitor at every Nth iteration
        self.trace_downrate_int = int(
            robot.params['direct']['SystemMonitorThread_trace_downrate_int'])  # Step the trace at every Nth iteration
        self.sentry_downrate_int = int(
            robot.params['direct']['SystemMonitorThread_sentry_downrate_int'])  # Step the sentry at every Nth iteration

        self.shutdown_flag = threading.Event()
        self.stats = hello_utils.LoopStats(loop_name='SystemMonitorThread', target_loop_rate=self.robot_update_rate_hz)
        self.titr = 0
        self.running = False

    def step(self):
        self.titr = self.titr + 1
        self.stats.mark_loop_start()
        if self.robot.params['direct']['use_monitor']:
            if (self.titr % self.monitor_downrate_int) == 0:
                self.robot.monitor.step()
        if self.robot.params['direct']['use_trace']:
            if (self.titr % self.trace_downrate_int) == 0:
                self.robot.trace.step()
        if self.robot.params['direct']['use_sentries']:
            if (self.titr % self.sentry_downrate_int) == 0:
                self.robot._step_sentry()

        self.stats.mark_loop_end()

    def run(self):
        self.running = True
        while not self.shutdown_flag.is_set():
            self.stats.wait_until_ready_to_run()
            if not self.shutdown_flag.is_set():
                self.step()
        self.robot.logger.debug('Shutting down SystemMonitorThread')


if __name__ == '__main__':
    r = Robot()
    if r.startup():
        r.power_periph.trigger_beep()
        r.push_command()
        for i in range(100):
            r.pull_status()
            print('Voltage CPU',r.status['power_periph']['voltage_cpu'])
            time.sleep(.01)
        r.stop()

