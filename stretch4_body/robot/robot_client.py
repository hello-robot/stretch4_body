#!/usr/bin/env python3
from __future__ import annotations

import importlib
import time
from typing import TYPE_CHECKING

import numpy as np

from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHelloStatus
from stretch4_body.core.hello_utils import rad_to_deg
from stretch4_body.core.prismatic_joint import PrismaticJointStatus
from stretch4_body.core.subsystem_client import SubsystemClient
from stretch4_body.subsystem.line_sensor import calibration
from stretch4_body.subsystem.omnibase import OmnibaseStatus
from stretch4_body.subsystem.power_periph import PowerPeriphStatus
from stretch4_body.utils.tool_metadata import (
    ParallelGripperMetadata,
    StretchGripperMetadata,
)

if TYPE_CHECKING:
    from stretch4_body.utils.tool_metadata import ToolMetadata


class RobotClient(SubsystemClient):
    """
    Client interface for controlling the Stretch robot.
    
    This class provides access to the robot's subsystems (arm, lift, base, etc.) 
    and high-level routines. It communicates with the robot server to execute commands 
    and retrieve status.

    Usage:
    ```
    robot = RobotClient()
    success = robot.startup()
    if not success: raise Exception("Could not start robot client")
    ```
    
    or
    ```
    with RobotClient() as robot:
        if robot is None: raise Exception("Could not start robot client")
    ```
    """
    def __init__(self, client_id=None, ip_address=None):
        """
        Initialize the RobotClient and its subsystems.
        """
        SubsystemClient.__init__(self, name='robot', client_id=client_id, parent=None, ip_address=ip_address)

        # Add on subsystems
        for k in self.params['subsystems']:
            if k == 'power_periph':
                self.power_periph = PowerPeriphClient(parent=self)
                self.subsystems[k] = self.power_periph
            if k == 'arm':
                self.arm = ArmClient(parent=self)
                self.subsystems[k] = self.arm
            if k == 'lift':
                self.lift = LiftClient(parent=self)
                self.subsystems[k] = self.lift
            if k == 'omnibase':
                self.omnibase = OmniBaseClient(parent=self)
                self.subsystems[k] = self.omnibase
                self.base = self.omnibase  # legacy naming
            if k == 'end_of_arm':
                self.eoa_name = self.params['tool']
                eoa_params = self.robot_params.get(self.eoa_name, {})
                if 'client_module_name' in eoa_params and 'client_class_name' in eoa_params:
                    module_name = eoa_params['client_module_name']
                    class_name = eoa_params['client_class_name']
                    
                    from stretch4_body.core.robot_params import RobotParams
                    current_module = RobotParams.import_user_tool_module(self.eoa_name, module_name, is_server=False)
                else:
                    module_name = 'stretch4_body.robot.robot_client'
                    class_name = self.robot_params[self.eoa_name]['py_class_name']+'_Client'
                    
                    # Check if the class is defined in the module
                    current_module = importlib.import_module(module_name)
                    if not hasattr(current_module, class_name):
                        # Dynamically define a subclass of EndOfArmClient with name class_name
                        # and register it into the module
                        from stretch4_body.robot.robot_client import EndOfArmClient
                        
                        def dynamic_init(self_obj, parent=None):
                            EndOfArmClient.__init__(self_obj, name=self.eoa_name, parent=parent)
                            
                        dynamic_class = type(class_name, (EndOfArmClient,), {"__init__": dynamic_init})
                        setattr(current_module, class_name, dynamic_class)

                self.end_of_arm:EndOfArmClient = getattr(current_module, class_name)(parent=self)
                self.subsystems[k] = self.end_of_arm

        for k in self.params['server']['subsystems']:
            if k == 'line_sensor_loop':
                self.line_sensor_loop = LineSensorLoopClient(parent=self)
                self.subsystems[k] = self.line_sensor_loop

        self.subsystems['routines']=self.routines=RoutinesClient(parent=self)

        # Note, self.status isn't a deepcopy, so it will automaticaly
        # update on pull_status of the subsystems
        for k in self.subsystems:
            self.status[k] = self.subsystems[k].status

        # Legacy naming
        if self.get_subsystem('omnibase') is not None:
            self.status['base'] = self.subsystems['omnibase'].status
        if self.get_subsystem('power_periph') is not None:
            self.status['pimu'] = self.subsystems['power_periph'].status

    def __enter__(self):
        if not self.startup():
            self.logger.error("RobotClient startup failed.")
            return None
        return self

    def __exit__(self, exc_type, exc, tb):
        if self.is_server_active():
            self.stop()

    def home(self, do_push=True,wait_on_completion=True,timeout=60, do_pull=True):
        """
        Home the robot.
        
        This routine homes all the joints of the robot. This is required to match the
        internal kinematic model with the physical robot state.
        
        The call blocks until completion.
        """
        self.logger.info('Starting robot homing routine')
        finished, rid = self.routines.routine_robot_home(do_push, wait_on_completion, timeout, do_pull)
        if wait_on_completion:  
            if not finished:
                self.logger.error("Homing routine timed out before completion.")
            else:
                self.logger.info("Homing routine ended.")
        else:
            self.logger.info('Homing routine started with ID: %d'%rid)

    def stow(self, do_push=True,wait_on_completion=True,timeout=30, do_pull=True):
        """
        Stow the robot.
        
        This routine moves the robot to a compact, stowed configuration.
        
        The call blocks until completion.
        """
        self.logger.info('Starting robot stowing routine')
        finished, rid = self.routines.routine_robot_stow(do_push, wait_on_completion, timeout, do_pull)
        if wait_on_completion:  
            if not finished:
                self.logger.error("Stowing routine timed out before completion.")
            else:
                self.logger.info("Stowing routine ended.")
        else:
            self.logger.info('Stowing routine started with ID: %d'%rid)

    def trigger_runstop(self):
        return self.power_periph.trigger_runstop()
    
    def clear_runstop(self):
        return self.power_periph.clear_runstop()
    
    def is_runstopped(self):
        return self.power_periph.is_runstopped()

    def is_homed(self):
        """
        Check if the robot is homed.

        Returns
        -------
        bool
            True if all joints that require calibration are homed/calibrated.
        """
        ready = True
        for s in self.subsystems.values():
            if hasattr(s,'is_homed'):
                ready = ready and s.is_homed()
        return ready
    
    def is_moving(self):
        """
        Check if any joints are moving.

        Returns
        -------
        bool
            True if any joint is moving
        """
        for s in self.subsystems.values():
            if hasattr(s,'is_moving') and s.is_moving():
                return True
        return False

    def wait_on_motion_start(self,subsystem_names,timeout=0.5):
        """
        Wait for the specified subsystems to start moving.
        
        Parameters
        ----------
        subsystem_names : list of str
            List of subsystem names to check (e.g. ['arm', 'lift']).
        timeout : float, optional
            Timeout in seconds, by default 0.5.
        """
        def start_moving():
            for n in subsystem_names:
                if n in self.subsystems and hasattr(self.subsystems[n],'is_moving'):
                    if self.subsystems[n].is_moving():
                        return True
            return False
        self._wait_on_status(start_moving, timeout, do_pull=True)

    def wait_on_motion_finish(self,subsystem_names,timeout=15.0,wait_on_motion_start:bool=True):
        """
        Wait for the specified subsystems to finish moving.
        
        Parameters
        ----------
        subsystem_names : list of str
            List of subsystem names to check.
        timeout : float, optional
            Timeout in seconds, by default 15.0.
        wait_on_motion_start: bool, optional
            Call wait_on_motion_start() to wait for the motion to start before waiting on motion to finish.
        """
        if wait_on_motion_start:
            self.wait_on_motion_start(subsystem_names)
        def done_moving():
            for n in subsystem_names:
                 if n not in self.subsystems:
                     raise ValueError(f"{n} is not a subsystem in {self.subsystems.keys()}")
                 if hasattr(self.subsystems[n],'is_moving'):
                    # print(f"{n=} {self.subsystems[n].is_moving()=}")
                    if self.subsystems[n].is_moving():
                        return False
            return True
        self._wait_on_status(done_moving, timeout, do_pull=True)

    def set_guarded_contact_sensitivity(self, mode_name=None):
        """
        Set the guarded contact sensitivity.
        
        Parameters
        ----------
        mode_name : str, optional
            Name of the sensitivity mode (e.g. 'off', 'default','high_sensitivity_nav', 'high_sensitivity_manipulation')
            None will reset to default
        """
        if mode_name is None:
            mode_name = 'default'
        if mode_name not in self.get_guarded_contact_modes():
            self.logger.error(f"set_guarded_contact_sensitivity: Invalid mode name: {mode_name}")
            return
        for s in self.subsystems:
            if hasattr(self.subsystems[s], 'set_guarded_contact_sensitivity') and s in self.params['guarded_contact'][mode_name]:
                self.subsystems[s].set_guarded_contact_sensitivity(self.params['guarded_contact'][mode_name][s])

    def get_guarded_contact_modes(self):
        """
        Get the guarded contact modes.
        """
        return list(self.robot_params['robot']['guarded_contact'].keys())  # Todo: move to server RPC for this, hack for now

    def pause_sentry(self, sentry_name):
        """
        Pause a system-level sentry by name.
        """
        self._queue_command('robot', 'pause_sentry', sentry_name)

    def unpause_sentry(self, sentry_name):
        """
        Unpause a system-level sentry by name.
        """
        self._queue_command('robot', 'unpause_sentry', sentry_name)
        
    # ################ Legacy API for backward compatability ########################
    def wait_command(self, timeout=15.0, use_motion_generator=True):
        """
        Legacy: Pause program execution until all motion is complete.

        Queuing up motion and pushing it to the hardware with
        push_command() is designed to be asynchronous, enabling
        reactive control of the robot. However, you might want
        synchronous control, where each command's motion is completed
        entirely before the program moves on to the next command.
        This is where you would use wait_command()

        Parameters
        ----------
        timeout : float
            How long to wait for motion to complete. Must be > 0.1 sec.
        use_motion_generator: bool
            Unused, kept for compatibility.

        Returns
        -------
        bool
            True if motion completed, False if timed out before motion completed
        """
        self.wait_on_motion_finish(['arm', 'omnibase', 'lift','end_of_arm'], timeout)


    def trigger_motor_sync(self):
        """ Legacy function. No longer needed."""
        pass



# #####################################################################
class RoutinesClient(SubsystemClient):
    """
    Client interface for executing high-level routines on the robot.
    """
    def __init__(self,parent=None):
        SubsystemClient.__init__(self,name='routine_manager',parent=parent)

    def cancel_routine(self, id: str = None, do_push=True):
        """
        Cancel a running routine.
        
        Parameters
        ----------
        id : str, optional
            String representation of the routine's ID to cancel. If None, cancels the currently running routine.
        do_push : bool, optional
            Whether to call push_command() within this method.
        """
        self._queue_command(subsystem="routines", command="cancel", id=id)
        if do_push:
            self.push_command()

    def run(self, routine_name, do_push=True, wait_on_completion=True, timeout=20, do_pull=True, priority=0):
        """
        Run a specified routine.
        
        do_push & do_pull are useful to disable when you're running Stretch4 Body
        in a multithreaded environment and don't want threads that aren't "main thread"
        to push_command() or pull_status(). E.g. in the ROS2 driver, homing and stowing
        are services by callback threads, so do_push/pull are set to False.

        Parameters
        ----------
        routine_name : str
            Name of the routine to run (e.g. 'routine_robot_home').
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until the routine indicates completion, by default True.
        timeout : float, optional
            Timeout in seconds to wait for completion, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        Returns
        -------
        bool, rid
            True if routine succeeded, False if either:
             - input arg wait_on_completion=False
             - or timed out before routine completed
             - or routine completed but failed
            rid is the id of the routine that was/is running
        """
        rid=self._queue_command(subsystem="routines", command=routine_name)
        if do_push:
            self.push_command(priority=priority)
        if wait_on_completion:
            finished = self._wait_on_routine(rid,timeout=timeout,do_pull=do_pull)
            if finished:
                success = self.status['routines'].get('last_routine_successful', True)
                return success, rid
            return False, rid
        return False, rid

    def routine_robot_stow(self,do_push=True,wait_on_completion=True, timeout=20, do_pull=True):
        """
        Run the robot stow routine.
        
        Parameters
        ----------
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        return self.run('routine_robot_stow', do_push, wait_on_completion, timeout, do_pull)

    def routine_robot_home(self,do_push=True,wait_on_completion=True,timeout=20,do_pull=True):
        """
        Run the robot home routine.
        
        Parameters
        ----------
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        return self.run('routine_robot_home', do_push, wait_on_completion, timeout, do_pull, priority=3)

    def routine_wrist_joint_home(self,joint_name,do_push=True,wait_on_completion=True,timeout=20,do_pull=True):
        """
        Home a specific wrist joint.
        
        Parameters
        ----------
        joint_name : str
            Name of the joint to home (e.g. 'wrist_yaw').
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        rid = self._queue_command(subsystem="routines", command="routine_wrist_joint_home",joint_name=joint_name)
        if do_push:
            self.push_command()
        if wait_on_completion:
            finished = self._wait_on_routine(rid, timeout=timeout, do_pull=do_pull)
            if finished:
                success = self.status['routines'].get('last_routine_successful', True)
                return success, rid
            return False, rid
        return False, rid

    def routine_end_of_arm_home(self,do_push=True,wait_on_completion=True,timeout=30,do_pull=True):
        """
        Home the end of arm (wrist and gripper).
        
        Parameters
        ----------
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        return self.run('routine_end_of_arm_home', do_push, wait_on_completion, timeout, do_pull)

    def routine_lift_home(self,do_push=True,wait_on_completion=True,timeout=20, do_pull=True):
        """
        Home the lift.
        
        Parameters
        ----------
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        return self.run('routine_lift_home', do_push, wait_on_completion, timeout, do_pull)

    def routine_arm_home(self,do_push=True,wait_on_completion=True,timeout=20, do_pull=True):
        """
        Home the arm.
        
        Parameters
        ----------
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        return self.run('routine_arm_home', do_push, wait_on_completion, timeout, do_pull)

    def routine_blind_dock(self,do_push=True,wait_on_completion=True,timeout=60,do_pull=True):
        """
        Run the blind docking routine.
        
        Parameters
        ----------
        do_push : bool, optional
            Whether to call push_command() within this method.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        do_pull : bool, optional
            If wait_on_completion, whether to call pull_status() within this method.
        """
        return self.run('routine_blind_dock', do_push, wait_on_completion, timeout, do_pull)

# #####################################################################
class PowerPeriphClient(SubsystemClient):
    """
    Client interface for the Power and IMU board (Pimu).
    """
    def __init__(self,parent=None):
        SubsystemClient.__init__(self,name='power_periph',parent=parent)
        self.status:PowerPeriphStatus
    
    def trigger_beep(self):
        """
        Trigger the buzzer to beep.
        """
        self._queue_command(subsystem="power_periph", command="trigger_beep")

    def set_charger_on(self):
        """
        Enable the battery charger.
        """
        self._queue_command(subsystem="power_periph", command="set_charger_on")

    def set_charger_off(self):
        """
        Disable the battery charger.
        """
        self._queue_command(subsystem="power_periph", command="set_charger_off")

    def clear_runstop(self):
        self._queue_command(subsystem="power_periph", command="clear_runstop")

    def trigger_runstop(self):
        self._queue_command(subsystem="power_periph", command="trigger_runstop")

    def is_runstopped(self):
        return self.status['runstop_event']

    def set_fan_on(self):
        """
        Turn on the cooling fan.
        """
        self._queue_command(subsystem="power_periph", command="set_fan_on")

    def set_fan_off(self):
        """
        Turn off the cooling fan.
        """
        self._queue_command(subsystem="power_periph", command="set_fan_off")

    def trigger_motor_sync(self):
        """ Legacy function. No longer needed."""
        pass

    def set_eye_animation(self, left_idx=None, right_idx=None):
        """
        Set the eye animations for the left and right eyes.
        
        Parameters
        ----------
        left_idx : int, optional
            Animation index for the left eye.
        right_idx : int, optional
            Animation index for the right eye.
        """
        self._queue_command(subsystem="power_periph", command="set_eye_animation", left_idx=left_idx, right_idx=right_idx)

    def actuator_control(self, motor_type, enable):
        """
        Control power to actuators (not typically used by end users).
        
        Parameters
        ----------
        motor_type : str
            Motor type identifier.
        enable : bool
            True to enable, False to disable.
        """
        self._queue_command("power_periph","actuator_control",motor_type,enable)


# #####################################################################
class OmniBaseClient(SubsystemClient):
    """
    Client interface for the mobile base (OmniBase).
    """
    def __init__(self, parent=None):
        SubsystemClient.__init__(self, name='omnibase', parent=parent)
        self.status:OmnibaseStatus

    def translate_by(self, x_m, y_m, v_m=None, a_m=None):
        """
        Translate the base by a relative amount.
        
        Parameters
        ----------
        x_m : float
        Translation in X direction (meters, forward).
        y_m : float
        Translation in Y direction (meters, left).
        v_m : float, optional
        Velocity limit (m/s).
        a_m : float, optional
        Acceleration limit (m/s^2).
        """
        self._queue_command("omnibase", "translate_by",x_m, y_m, v_m, a_m)

    def wheel_move_to(self, wheel_name, x_rad, v_r=None, a_r=None):
        """
        Move a specific wheel to an absolute position.
        
        Parameters
        ----------
        wheel_name : str
            Name of the wheel (e.g., 'wheel_0', 'wheel_1', 'wheel_2').
        x_rad : float
            Absolute position in radians.
        v_r : float, optional
            Rotational velocity limit (rad/s).
        a_r : float, optional
            Rotational acceleration limit (rad/s^2).
        """
        self._queue_command(f'{wheel_name}.omnibase', "wheel_move_to", wheel_name, x_rad, v_r, a_r)

    def wheel_move_by(self, wheel_name, x_rad, v_r=None, a_r=None):
        """
        Move a specific wheel by a relative amount.
        
        Parameters
        ----------
        wheel_name : str
            Name of the wheel.
        x_rad : float
            Relative motion in radians.
        v_r : float, optional
            Rotational velocity limit (rad/s).
        a_r : float, optional
            Rotational acceleration limit (rad/s^2).
        """
        self._queue_command(f'{wheel_name}.omnibase', "wheel_move_by", wheel_name, x_rad, v_r, a_r)

    def rotate_by(self, w_r, v_r=None, a_r=None):
        """
        Rotate the base by a relative amount.
        
        Parameters
        ----------
        w_r : float
            Rotation angle (radians, counter-clockwise).
        v_r : float, optional
            Rotational velocity limit (rad/s).
        a_r : float, optional
            Rotational acceleration limit (rad/s^2).
        """
        self._queue_command("omnibase", "rotate_by", w_r, v_r, a_r)

    def set_velocity(self, vx_m, vy_m, w_r, a_m=None, a_r=None, stiffness=1.0):
        """
        Set the base velocity.
        
        Parameters
        ----------
        vx_m : float
            Velocity in X direction (m/s).
        vy_m : float
            Velocity in Y direction (m/s).
        w_r : float
            Rotational velocity (rad/s).
        a_m : float, optional
            Linear acceleration limit (m/s^2).
        a_r : float, optional
            Rotational acceleration limit (rad/s^2).
        """
        self._queue_command("omnibase", "set_velocity", vx_m, vy_m, w_r, a_m, a_r, stiffness=stiffness)

    def enable_freewheel_mode(self):
        """
        Enable freewheel mode (motors disabled).
        """
        self._queue_command("omnibase", "enable_freewheel_mode")

    def enable_hold_mode(self):
        """
        Enable hold mode (motors actively holding position).
        """
        self._queue_command("omnibase", "enable_hold_mode")

    def hard_stop(self):
        """
        Stop the base immediately.
        """
        self._queue_command("omnibase", "hard_stop")

    def set_guarded_contact_sensitivity(self, mode_name=None):
        """
        Set the guarded contact sensitivity.
        
        Parameters
        ----------
        mode_name : str, optional
            Name of the sensitivity mode (e.g. 'default', 'high', 'low', 'off')
            None will reset to default
        """
        self._queue_command("omnibase", "set_guarded_contact_sensitivity",mode_name)

    def get_guarded_contact_modes(self):
        """
        Get the guarded contact modes.
        """
        return list(self.robot_params['hello-motor-omni-0']['guarded_contact'].keys()) #Todo: move to server RPC for this, hack for now

    def stop(self):
        """
        Stop the base and put it in freewheel mode.
        """
        self.enable_freewheel_mode()
        SubsystemClient.stop(self)

    def is_moving(self):
        return self.status['wheel_0']['is_mg_moving'] \
            and self.status['wheel_1']['is_mg_moving'] \
                and self.status['wheel_2']['is_mg_moving']


# #####################################################################

class LineSensorLoopClient(SubsystemClient):
    """Client view of the six PixArt line sensors.

    The calibration the body loaded and validated arrives inside the status,
    so a client never opens a tare file or repeats its validation. Ask this class
    for the arrays instead of unpacking status['calibration'] by hand.
    """

    def __init__(self,parent=None):
        SubsystemClient.__init__(self,name='line_sensor_loop',parent=parent)
        self._calib_id = None
        self._calib = {}          # sensor_name -> (offsets, valid_mask, null_rate)
        self._frame_id_last = {}

    # -- calibration -------------------------------------------------------

    def _refresh_calibration(self):
        """Unpack the wire form once per calibration, not once per status."""
        block = self.status.get('calibration') or {}
        cid = block.get('id')
        if cid == self._calib_id:
            return
        calib = {}
        for name in block.get('loaded', []):
            try:
                calib[name] = calibration.unpack_tare(block[name])
            except (KeyError, ValueError) as exc:
                self.logger.error('%s: unusable tare on the wire: %s', name, exc)
        self._calib, self._calib_id = calib, cid

    def calibrated_sensors(self):
        """Names with a tare the body accepted."""
        self._refresh_calibration()
        return sorted(self._calib)

    def uncalibrated_sensors(self):
        """{name: why the body refused its tare}."""
        return dict((self.status.get('calibration') or {}).get('rejected', {}))

    def bin_reliable(self):
        """{name: bool array} -- bins whose tare is trustworthy."""
        self._refresh_calibration()
        return {n: v[1] for n, v in self._calib.items()}

    def bin_null_rate(self):
        """{name: float array} -- per-bin no-return rate seen on clear floor."""
        self._refresh_calibration()
        return {n: v[2] for n, v in self._calib.items()}

    def apply_tare(self, ranges, sensor_name, codes=None):
        """Tare one sensor's ranges. An uncalibrated sensor passes through.

        Uses the same routine the body's own tools use, so there is one
        implementation of what a tare means rather than one per consumer.
        """
        self._refresh_calibration()
        entry = self._calib.get(sensor_name)
        if entry is None:
            return np.asarray(ranges, dtype=np.float64)
        offsets, valid_mask, _ = entry
        return calibration.apply_tare_array(ranges, offsets, valid_mask, codes)

    def tared_ranges(self, sensor_name):
        """Latest ranges for one sensor, tared, with its per-bin codes."""
        s = self.status.get(sensor_name)
        if not s or not len(s['ranges']):
            return np.zeros(0), np.zeros(0, dtype=np.uint8)
        return self.apply_tare(s['ranges'], sensor_name, s['codes']), s['codes']

    # -- liveness ----------------------------------------------------------

    def health(self):
        return dict(self.status.get('health') or {})

    def dead_sensors(self):
        """Names that have stopped reporting, or never started.

        Distinct from disabled_sensors(): dead is a fault, disabled is a
        choice. A disabled sensor never appears here.
        """
        return list((self.status.get('health') or {}).get('sensors_dead', []))

    # -- runtime control ---------------------------------------------------

    def is_streaming(self):
        return bool((self.status.get('health') or {}).get('streaming', False))

    def disabled_sensors(self):
        """Names switched off at runtime. Check this before trusting a clear
        floor: a disabled sensor reports nothing, not 'nothing is there'."""
        return list((self.status.get('health') or {}).get('disabled_sensors', []))

    def reader_restarts(self):
        """How many times the serial port has recovered itself. A number that
        keeps climbing means a flaky cable, not a healthy subsystem."""
        return int((self.status.get('health') or {}).get('reader_restarts', 0))

    def set_streaming(self, on):
        """Pause or resume the line sensors.

        Takes effect after the next push_command(). Turning them off removes
        cliff detection; the body reports it in health['streaming'] rather
        than enforcing anything, so whatever is driving must check.
        """
        self._queue_command('line_sensor_loop', 'set_streaming', bool(on))

    def set_sensor_enabled(self, sensor_name, on):
        """Turn one sensor's decoding on or off. Takes effect after the next
        push_command(). Not persisted -- a restart comes up with all six on."""
        self._queue_command('line_sensor_loop', 'set_sensor_enabled',
                            sensor_name, bool(on))

    def is_sensor_updated(self, sensor_name):
        """True if this sensor produced a new frame since the last check.

        The status socket is CONFLATE=1 and publishes at 100 Hz while sensors
        report at ~30 Hz, so most status messages repeat a sensor's previous
        frame. Without this a consumer processes the same scan three times.
        """
        s = self.status.get(sensor_name) or {}
        fid = s.get('frame_id', 0)
        changed = self._frame_id_last.get(sensor_name) != fid
        self._frame_id_last[sensor_name] = fid
        return changed

# #####################################################################


class PrismaticJointClient(SubsystemClient):
    """
    Client interface for prismatic joints (Arm and Lift).
    """
    def __init__(self,name,parent=None):
        SubsystemClient.__init__(self,name=name,parent=parent)
        self.status:PrismaticJointStatus

    def startup(self,*args,**kwargs): #Ignore old api args
        """
        Start up the client.
        """
        return SubsystemClient.startup(self)

    def home(self):
        """
        Home the joint.
        
        This moves the joint to the hard stop to calibrate its position.
        Blocking call.
        """
        self.logger.info('Homing %s...'%self.name)
        rid=self._queue_command(subsystem="routines", command="routine_%s_home"%self.name)
        self.push_command()
        finished=self._wait_on_routine(rid, timeout=20.0)
        success = finished and self.status['routines'].get('last_routine_successful', True)
        if success and self.is_homed():
            self.logger.info('Successfully homed %s.'%self.name)
        else:
            self.logger.error('Failed to home joint %s.'%self.name)

    def stop(self):
        """
        Stop the joint and enable safety mode.
        """
        self.enable_safety()
        SubsystemClient.stop(self)

    # ####################### Utility ##########################

    def set_guarded_contact_sensitivity(self, mode_name=None):
        """
        Set the guarded contact sensitivity.
        
        Parameters
        ----------
        mode_name : str, optional
            Name of the sensitivity mode (e.g. 'default', 'high', 'low', 'off')
            None will reset to default
        """
        self._queue_command(self.name, "set_guarded_contact_sensitivity",mode_name)

    def is_homed(self):
        """
        Check if homed.
        
        Returns
        -------
        bool
            True if the joint needs to be homed/calibrated.
        """
        return self.status.get('motor', {}).get('pos_calibrated', False)
    
    def is_moving(self):
        return self.status['motor']['is_mg_moving']
    
    def enable_safety(self):
        """
        Enable safety mode.
        """
        self._queue_command(self.name, "enable_safety")

    def disable_sync_mode(self):
        """
        Disable sync mode.
        """
        self._queue_command(self.name, "disable_sync_mode")

    def enable_sync_mode(self):
        """
        Enable sync mode.
        """
        self._queue_command(self.name, "enable_sync_mode")

    def disable_runstop(self):
        """
        Disable runstop (resume operation).
        """
        self._queue_command(self.name, "disable_runstop")

    def enable_runstop(self):
        """
        Enable runstop (halt operation).
        """
        self._queue_command(self.name, "enable_runstop")

    # ####################### Motion ##########################

    def move_by(self, x_m, v_m=None, a_m=None, stiffness=None, req_calibration=True, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Move the joint by a relative amount.
        
        Parameters
        ----------
        x_m : float
            Relative motion in meters.
        v_m : float, optional
            Velocity limit (m/s).
        a_m : float, optional
            Acceleration limit (m/s^2).
        stiffness : float, optional
            Stiffness setting (0.0 to 1.0) or None to leave unchanged.
        req_calibration : bool, optional
            If True, requires the joint to be calibrated, by default True.
        contact_sensitivity_pos : float, optional
            Contact sensitivity in positive direction (0-1).
        contact_sensitivity_neg : float, optional
            Contact threshold in negative direction (0-1).
        """
        if req_calibration and not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.name} has not been homed.")
            return False
        self._queue_command(self.name, "move_by",x_m, v_m=v_m, a_m=a_m, stiffness=stiffness, req_calibration=req_calibration,contact_sensitivity_pos=contact_sensitivity_pos, contact_sensitivity_neg=contact_sensitivity_neg)
        return True

    def set_velocity(self, v_m, a_m=None, stiffness=None, req_calibration=True, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Set the joint velocity.
        
        Parameters
        ----------
        v_m : float
            Velocity (m/s).
        a_m : float, optional
            Acceleration limit (m/s^2).
        stiffness : float, optional
            Stiffness setting.
        req_calibration : bool, optional
            Requirement for calibration.
        contact_sensitivity_pos : float, optional
            Contact sensitivity in positive direction (0-1).
        contact_sensitivity_neg : float, optional
            Contact threshold in negative direction (0-1).
        """
        if req_calibration and not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.name} has not been homed.")
            return False
        self._queue_command(self.name, "set_velocity", v_m, a_m=a_m, stiffness=stiffness,req_calibration=req_calibration, contact_sensitivity_pos=contact_sensitivity_pos, contact_sensitivity_neg=contact_sensitivity_neg)
        return True

    def move_to(self, x_m, v_m=None, a_m=None, stiffness=None, req_calibration=True, contact_sensitivity_pos=None, contact_sensitivity_neg=None):
        """
        Move the joint to an absolute position.
        
        Parameters
        ----------
        x_m : float
            Absolute position in meters.
        v_m : float, optional
            Velocity limit (m/s).
        a_m : float, optional
            Acceleration limit (m/s^2).
        stiffness : float, optional
            Stiffness setting.
        req_calibration : bool, optional
            Requirement for calibration.
        contact_sensitivity_pos : float, optional
            Contact sensitivity in positive direction (0-1).
        contact_sensitivity_neg : float, optional
            Contact threshold in negative direction (0-1).
        """
        if req_calibration and not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.name} has not been homed.")
            return False
        self._queue_command(self.name, "move_to",x_m, v_m=v_m, a_m=a_m, stiffness=stiffness, req_calibration=req_calibration,contact_sensitivity_pos=contact_sensitivity_pos, contact_sensitivity_neg=contact_sensitivity_neg)
        return True


# #####################################################################
class LiftClient(PrismaticJointClient):
    def __init__(self,parent=None):
        PrismaticJointClient.__init__(self,name='lift',parent=parent)

class ArmClient(PrismaticJointClient):
    def __init__(self,parent=None):
        PrismaticJointClient.__init__(self,name='arm',parent=parent)


# #####################################################################
class WristJointClient(SubsystemClient):
    """
    Client interface for wrist joints (Yaw, Pitch, Roll).
    """
    def __init__(self, joint_name:str,parent:EndOfArmClient|None=None, ip_address=None):
        self.joint_name=joint_name
        self.parent = parent # keep this here for typing
        SubsystemClient.__init__(self, name=joint_name, parent=parent, ip_address=ip_address)

    @property
    def status(self) -> FeetechSMHelloStatus:
        if self.parent is not None:
            return self.parent.status.get(self.name, {})
        return self._status
    
    @status.setter
    def status(self, value):
        self._status = value

    def do_ping(self):
        """
        Ping the motor to check connectivity.
        """
        self._queue_command(f'{self.joint_name}.end_of_arm', "do_ping",self.joint_name)
    def is_homed(self):
        """
        Check if homed.
        """
        return self.status.get('pos_calibrated', False)
    

    def is_moving(self):
        return self.status['is_moving']

    def move_by(self, x_r, v_r=None, a_r=None):
        """
        Move the joint by a relative amount.
        
        Parameters
        ----------
        x_r : float
            Relative motion in radians.
        v_r : float, optional
            Velocity limit (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        if not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
            return False
        self._queue_command(f'{self.joint_name}.end_of_arm', "move_by",self.joint_name,x_r, v_r, a_r)
        return True
    def move_to(self, x_r, v_r=None, a_r=None):
        """
        Move the joint to an absolute position.
        
        Parameters
        ----------
        x_r : float
            Absolute position in radians.
        v_r : float, optional
            Velocity limit (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        if not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
            return False
        self._queue_command(f'{self.joint_name}.end_of_arm', "move_to", self.joint_name, x_r, v_r, a_r)
        return True
    def set_velocity(self, v_r, a_r=None):
        """
        Set the joint velocity.
        
        Parameters
        ----------
        v_r : float
            Velocity (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        self.logger.error(f"Cannot send movement command. Joint {self.joint_name} does not support set_velocity.")
        return False
        if not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
            return False
        self._queue_command(f'{self.joint_name}.end_of_arm', "set_velocity", self.joint_name, v_r, a_r)
        return True
    def pose(self, p,v_r=None, a_r=None):
        """
        Move to a named pose.
        
        Parameters
        ----------
        p : str
            Name of the pose.
        v_r : float, optional
            Velocity limit (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        if not self.is_homed():
            self.logger.error(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
            return False
        self._queue_command(f'{self.joint_name}.end_of_arm', "pose", self.joint_name, p, v_r, a_r)
        return True
    def disable_torque(self):
        """
        Disable torque on the joint to make it backdrivable.
        """
        self._queue_command(f'{self.joint_name}.end_of_arm', "disable_torque", self.joint_name)
    def enable_torque(self):
        """
        Enable torque on the joint to actively hold position.
        """
        self._queue_command(f'{self.joint_name}.end_of_arm', "enable_torque", self.joint_name)
    def home(self, end_pos=0,wait_on_completion=True, timeout=20):
        """
        Home the joint.
        
        Parameters
        ----------
        end_pos : float, optional
            Final position after homing (radians), by default 0.
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        """
        rid = self._queue_command(subsystem="routines", command="routine_wrist_joint_home",joint_name=self.joint_name,end_pos=end_pos)
        self.push_command(priority=3)
        if wait_on_completion:
            finished = self._wait_on_routine(rid, timeout=timeout)
            success = finished and self.status['routines'].get('last_routine_successful', True)
            if not success:
                self.logger.error(f'Failed to home wrist joint {self.joint_name}.')
            return success
        return False
    def stop(self):
        """
        Stop the joint.
        """
        SubsystemClient.stop(self)
    def pretty_print(self):
        """
        Print the status of the joint.
        """

        print('----- FeetechSMHello ------ ')
        print('Name', self.name)
        print('Position (rad)', self.status['pos'])
        print('Position (deg)', rad_to_deg(self.status['pos']))
        print('Position (ticks)', self.status['pos_ticks'])
        print('Velocity (rad/s)', self.status['vel'])
        print('Velocity (ticks/s)', self.status['vel_ticks'])
        print('Effort (%)', self.status['effort'])
        print('Current (mA)', self.status['current_mA'])
        print('Temp', self.status['temp'])
        print('Comm Errors', self.status['comm_errors'])
        print('Hardware Error', self.status['hardware_error'])
        print('Hardware Error: Input Voltage Error: ', self.status['input_voltage_error'])
        print('Hardware Error: Overheating Error: ', self.status['overtemp_error'])
        print('Hardware Error: Motor Encoder Error: ', self.status['motor_encoder_error'])
        print('Hardware Error: Over Current Error: ', self.status['over_current_error'])
        print('Hardware Error: Overload Error: ', self.status['overload_error'])
        print('Watchdog Errors: ', self.status['watchdog_errors'])
        print('Timestamp PC', self.status['timestamp_pc'])
        print('Stalled', self.status['stalled'])
        print('Stall Overload', self.status['stall_overload'])
        print('Is Calibrated', self.status['pos_calibrated'])
        print('Is homing: %d' % self.status['is_homing'])

class WristYawClient(WristJointClient):
    """ Client for the wrist yaw joint. """
    def __init__(self, parent:EndOfArmClient|None=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='wrist_yaw', parent=parent, ip_address=ip_address)

class WristRollClient(WristJointClient):
    """ Client for the wrist roll joint. """
    def __init__(self, parent:EndOfArmClient|None=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='wrist_roll', parent=parent, ip_address=ip_address)

class WristPitchClient(WristJointClient):
    """ Client for the wrist pitch joint. """
    def __init__(self, parent:EndOfArmClient|None=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='wrist_pitch', parent=parent, ip_address=ip_address)

<<<<<<< HEAD
<<<<<<< HEAD
<<<<<<< HEAD
class StretchGripperClient(WristJointClient):
    """ Client for the stretch gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='stretch_gripper', parent=parent, ip_address=ip_address)
        self.tool_metadata = StretchGripperMetadata()
        self.poses = self.tool_metadata.poses
        self.pct_max_open = self.poses['open']
        self.status['gripper_conversion'] = {'aperture_m': 0.0,
                                             'finger_rad': 0.0,
                                             'finger_effort': 0.0,
                                             'finger_vel': 0.0}

class ParallelGripperClient(WristJointClient):
    """ Client for the parallel gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='parallel_gripper', parent=parent, ip_address=ip_address)
        self.tool_metadata = ParallelGripperMetadata()
        self.poses = self.tool_metadata.poses

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        """
        Move the parallel gripper by a relative amount in millimeters.
        """
        return self.move_by(x_mm / 1000.0, v_r, a_r)
    
    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        """
        Move the parallel gripper to an absolute position in millimeters.
        """
        return self.move_to(x_mm / 1000.0, v_r, a_r)
=======
=======
>>>>>>> 80c7996 (Resolve circular imports with WristJointClient and tool_metadata)
=======
=======
>>>>>>> 1121587 (Add abstract gripper object to EndOfArmClient)
>>>>>>> 705b723 (Add abstract gripper object to EndOfArmClient)
class ToolJointClient(WristJointClient):
    """Flexible client for the end effector tool joint"""
    def __init__(self, metadata: ToolMetadata, parent: EndOfArmClient | None = None, ip_address=None):
        WristJointClient.__init__(self, joint_name=metadata.joint_name, parent=parent, ip_address=ip_address)
        self.tool_metadata = metadata
        self.poses = metadata.poses
        self.status['gripper_conversion'] = metadata.status
<<<<<<< HEAD
<<<<<<< HEAD
>>>>>>> 0cb0822 (Add abstract gripper object to EndOfArmClient)
=======
=======
>>>>>>> 705b723 (Add abstract gripper object to EndOfArmClient)
=======
class StretchGripperClient(WristJointClient):
    """ Client for the stretch gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='stretch_gripper', parent=parent, ip_address=ip_address)
        self.tool_metadata = StretchGripperMetadata()
        self.poses = self.tool_metadata.poses
        self.pct_max_open = self.poses['open']
        self.status['gripper_conversion'] = {'aperture_m': 0.0,
                                             'finger_rad': 0.0,
                                             'finger_effort': 0.0,
                                             'finger_vel': 0.0}

class ParallelGripperClient(WristJointClient):
    """ Client for the parallel gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='parallel_gripper', parent=parent, ip_address=ip_address)
        self.tool_metadata = ParallelGripperMetadata()
        self.poses = self.tool_metadata.poses

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        """
        Move the parallel gripper by a relative amount in millimeters.
        """
        return self.move_by(x_mm / 1000.0, v_r, a_r)
    
    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        """
        Move the parallel gripper to an absolute position in millimeters.
        """
        return self.move_to(x_mm / 1000.0, v_r, a_r)
>>>>>>> 80d461d (Resolve circular imports with WristJointClient and tool_metadata)
>>>>>>> 80c7996 (Resolve circular imports with WristJointClient and tool_metadata)
# #####################################################################
=======

######################################################################
>>>>>>> 1121587 (Add abstract gripper object to EndOfArmClient)
class EndOfArmClient(SubsystemClient):
    """
    Client interface for the End of Arm (Tool).
    """

    def __init__(self,name='end_of_arm',parent=None):
        SubsystemClient.__init__(self,name=name,parent=parent)
        self.joints = list(self.robot_params[self.name].get('devices', {}).keys())

        # These are populated for python typing.
        self.wrist_pitch: WristPitchClient
        self.wrist_roll: WristRollClient
        self.wrist_yaw: WristYawClient
        self.gripper: ToolJointClient

        from stretch4_body.utils.tool_metadata import get_tool_metadata, is_tool_joint

        for joint in self.joints:
            # The attributes defined above are assigned in this forloop:
            device_params = self.robot_params[self.name].get('devices', {}).get(joint, {})
            if device_params.get('py_class_name') is None:
                continue

            if is_tool_joint(joint):
                # A configured tool (built-in gripper, or a user tool registered under this
                # joint's own name) gets whatever client its metadata declares, and is also
                # aliased as `self.gripper`. A misconfigured tool raises here rather than
                # silently falling back to a plain joint client.
                client = get_tool_metadata(joint).client_class(parent=self)
                self.gripper = client
            else:
                client = WristJointClient(joint_name=joint, parent=self)
            setattr(self, joint, client)

    def do_ping(self, joint):
        """
        Ping a specific joint in the end of arm tool.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "do_ping",joint)
        return True

    def move_by(self, joint,x_r, v_r=None, a_r=None):
        """
        Move a specific joint by a relative amount.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        x_r : float
            Relative motion in radians.
        v_r : float, optional
            Velocity limit (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        if not self.is_homed(joint):
            self.logger.error(f"Cannot send movement command. Joint {joint} has not been homed.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "move_by",joint,x_r, v_r, a_r)
        return True

    def move_to(self, joint,x_r, v_r=None, a_r=None):
        """
        Move a specific joint to an absolute position.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        x_r : float
            Absolute position in radians.
        v_r : float, optional
            Velocity limit (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        if not self.is_homed(joint):
            self.logger.error(f"Cannot send movement command. Joint {joint} has not been homed.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "move_to", joint, x_r, v_r, a_r)
        return True

    def set_velocity(self, joint, v_r, a_r=None):
        """
        Set the velocity of a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        v_r : float
            Velocity (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        self.logger.error(f"Cannot send movement command. Joint {joint} does not support set_velocity.")
        return False
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        if not self.is_homed(joint):
            self.logger.error(f"Cannot send movement command. Joint {joint} has not been homed.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "set_velocity", joint, v_r, a_r)
        return True

    def pose(self,joint, p,v_r=None, a_r=None):
        """
        Move a specific joint to a named pose.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        p : str
            Name of the pose.
        v_r : float, optional
            Velocity limit (rad/s).
        a_r : float, optional
            Acceleration limit (rad/s^2).
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        if not self.is_homed(joint):
            self.logger.error(f"Cannot send movement command. Joint {joint} has not been homed.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "pose", joint, p,v_r, a_r)
        return True

    def quick_stop(self,joint):
        """
        Quickly stop a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "quick_stop",joint)
        return True

    def disable_torque(self, joint):
        """
        Disable torque on a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "disable_torque", joint)
        return True

    def enable_torque(self, joint):
        """
        Enable torque on a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "enable_torque", joint)
        return True

    def pause_sentry(self, joint):
        """
        Pause the safe_motion sentry on a specific joint.
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "pause_sentry", joint)
        return True

    def unpause_sentry(self, joint):
        """
        Unpause the safe_motion sentry on a specific joint.
        """
        if joint not in self.joints:
            self.logger.error(f"Joint {joint} not found in end of arm tool.")
            return False
        self._queue_command(f'{joint}.end_of_arm', "unpause_sentry", joint)
        return True

    def home(self,wait_on_completion=True,timeout=45):
        """
        Home the entire end of arm tool.
        
        Parameters
        ----------
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 45.
        """
        rid=self._queue_command(subsystem="routines", command="routine_end_of_arm_home")
        self.logger.info('Homing %s ...'%self.name)
        self.push_command()
        if wait_on_completion:
            finished = self._wait_on_routine(rid,timeout=timeout)
            success = finished and self.status['routines'].get('last_routine_successful', True)
            if not success:
                self.logger.error(f'Failed to home end of arm tool {self.name}.')
            return success
        return False

    def stow(self,wait_on_completion=True,timeout=20):
        """
        Stow the entire end of arm tool.
        
        Parameters
        ----------
        wait_on_completion : bool, optional
            If True, block until completion, by default True.
        timeout : float, optional
            Timeout in seconds, by default 20.
        """
        rid=self._queue_command(subsystem="routines", command="routine_end_of_arm_stow")
        self.logger.info('Stowing %s ...' % self.name)
        self.push_command()
        if wait_on_completion:
            self._wait_on_routine(rid,timeout=timeout)

    def wait_on_motion_start(self,joint_names,timeout=0.5):
        def start_moving():
            start = self.is_moving(joint_names)
            return start
        self._wait_on_status(start_moving, timeout)

    def wait_on_motion_finish(self,joint_names,timeout=15.0):
        def done_moving():
            done = not self.is_moving(joint_names)
            return  done
        self._wait_on_status(done_moving, timeout)

    def is_homed(self, joint=None):
        if joint is None:
            req=True
            for j in self.joints:
                req_cal = self.robot_params[self.name].get('devices', {}).get(j, {}).get('req_calibration', True)
                req = req and (not req_cal or self.status.get(j, {}).get('pos_calibrated', False))
            return req
        else:
            req_cal = self.robot_params[self.name].get('devices', {}).get(joint, {}).get('req_calibration', True)
            return not req_cal or self.status.get(joint, {}).get('pos_calibrated', False)
        
    def is_moving(self, joint_names:list[str]|None = None):
        req = False
        for j in joint_names or self.joints:
            req = req or self.status[j]['is_moving']
        return req
    
    def is_tool_present(self,class_name):
        """
        Return true if the given tool type is present (eg. StretchGripper)
        Allows for conditional logic when switching end-of-arm tools
        """
        for j in self.joints:
            if class_name == self.params['devices'][j]['py_class_name']:
                return True
        return False
    
    def stop(self):
        """
        Stop the end of arm tool.
        """
        SubsystemClient.stop(self)

# #####################################################################
class EOA_Wrist_DW4_Tool_NIL_Client(EndOfArmClient):
    """
    Wrist Yaw / Pitch / Roll only for version 4 of DexWrist
    """
    def __init__(self, parent=None):
        EndOfArmClient.__init__(self,name='eoa_wrist_dw4_tool_nil',parent=parent)

class EOA_Wrist_DW4_Tool_SG4_Client(EndOfArmClient):
    """
    Wrist Yaw / Pitch / Roll /Gripper only for version 4 of DexWrist
    """
    def __init__(self,parent=None):
        EndOfArmClient.__init__(self,name='eoa_wrist_dw4_tool_sg4',parent=parent)

class EOA_Wrist_DW4_Tool_PG4_Client(EndOfArmClient):
    """
    Wrist Yaw / Pitch / Roll /Gripper only for version 4 of DexWrist
    """
    def __init__(self,parent=None):
        EndOfArmClient.__init__(self,name='eoa_wrist_dw4_tool_pg4',parent=parent)

class EOA_Wrist_DW4_Tool_Calibration_Client(EndOfArmClient):
    """
    Wrist Yaw / Pitch / Roll /Gripper only for version 4 of DexWrist
    """
    def __init__(self,parent=None):
        EndOfArmClient.__init__(self,name='eoa_wrist_dw4_tool_calibration',parent=parent)

class EOA_Wrist_DW4_Tool_Tablet_Client(EndOfArmClient):
    """
    Wrist Yaw / Pitch / Roll /Gripper only for version 4 of DexWrist
    """
    def __init__(self,parent=None):
        EndOfArmClient.__init__(self,name='eoa_wrist_dw4_tool_tablet',parent=parent)



if __name__ == '__main__':
    if 1:
        r = RobotClient()
        if r.startup():
            ts=time.time()
            for i in range(1000):
                r.pull_status()
                print(i)
                # print('-------------%d--------------'%r.status['server']['status_id'])
                # print(r.status['server'])
                #time.sleep(0.1)
            print('RATE',1000/(time.time()-ts))
            r.stop()
    if 0:
        r = RobotClient()
        if r.startup():
            for i in range(1000):
                r.pull_status()
                for j in r.end_of_arm.joints:
                    print(r.end_of_arm.status[j]['pos'])
                    r.end_of_arm.move_to(j,-0.1)
                r.push_command()
                for j in r.end_of_arm.joints:
                    print(r.end_of_arm.status[j]['pos'])
                    r.end_of_arm.move_to(j, 0.1)
                r.push_command()
                time.sleep(.02)
            r.stop()

    if 0:
        e=EndOfArmClient()
        e.startup()
        e.pull_status()
        print(e.status)
        e.stop()

    if 0:
        r = RobotClient()
        if r.startup():
            r.power_periph.trigger_beep()
            for i in range(100):
                print('Voltage CPU',r.status['power_periph']['voltage_cpu'])
                time.sleep(.01)
            r.stop()
    # if 1:
    #     r = RobotClient()
    #     if r.startup():
    #         r.power_periph.trigger_beep()
    #         r.push_command()
    #         r.stop()
    if 0:
        r = RobotClient()
        if r.startup():
            ts=time.time()
            try:
                while(True): #time.time()-ts<3.0):
                    #print('----------',time.time()-ts,'--------')
                    # s=r.pull_status()
                    # sa = r.pull_status_aux()
                    s=r.status
                    print('RobotServer : Runtime %.8f (s) | Rate %.2f (Hz): '%(s['control_loop']['execution_time_s'],s['control_loop']['curr_rate_hz']))
                    #print(r.power_periph.status)
                    #r.worker_thread.stats.pretty_print()
                    time.sleep(0.1)
            except KeyboardInterrupt:
                pass
            r.stop()
        #r.shutdown_server()
        # print('RobotServer state: ', r.get_server_state())
        # time.sleep(0.5)

        #r.stop()
    if 0:
        r = RobotClient()
        if r.startup():
            if 0:
                print('Control loop state: ',r.get_control_loop_state())
                time.sleep(0.5)
                r.pause_control_loop()
                print('Control loop state: ', r.get_control_loop_state())
                time.sleep(0.5)
                r.unpause_control_loop()
                print('Control loop state: ', r.get_control_loop_state())
                time.sleep(0.5)
            ts=time.time()
            try:
                while(True): #time.time()-ts<3.0):
                    #print('----------',time.time()-ts,'--------')
                    s=r.pull_status()
                    sa = r.pull_status_aux()
                    print('S',s)
                    if s is not None:
                        print('RobotServer : Runtime %.2f (s) | Rate %.2f (Hz): ' % (s['control_loop']['loop_clock'], s['control_loop']['loop_rate_actual']))
                    else:
                        time.sleep(0.1)
            except:
                pass
        #r.shutdown_server()
        # print('RobotServer state: ', r.get_server_state())
        # time.sleep(0.5)

        #r.stop()

    if 0:
        r = RobotDirectClient()
        r.startup()
        time.sleep(1.0)
        r.power_periph.trigger_beep()
        r.push_command()
        #time.sleep(1.0)
        s=r.get_status()
        print(s['line_sensors'])
        # r.pause_server()
        # time.sleep(2.0)
        # r.unpause_server()
        # time.sleep(2.0)
        r.stop()

    #
    # cmd={'power_periph':[['STREAM_set_buzzer_on']]}
    #
    # s.push_command(cmd)
    # time.sleep(1.0)
    # cmd = {'power_periph': [['STREAM_set_buzzer_off']]}
    # s.push_command(cmd)
    # for i in range(100):
    #     status=s.pull_status()
    #     print(status)
    #     time.sleep(.01)
    # s.stop()
