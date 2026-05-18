#!/usr/bin/env python3
import time
from stretch4_body.core.prismatic_joint import PrismaticJointStatus
from stretch4_body.core.subsystem_client import SubsystemClient
import importlib
from stretch4_body.subsystem.end_of_arm.stretch_gripper import GripperConversion
from stretch4_body.core.hello_utils import rad_to_deg, deg_to_rad
from stretch4_body.subsystem.omnibase import OmnibaseStatus
from stretch4_body.subsystem.power_periph import PowerPeriphStatus

class RobotClient(SubsystemClient):
    """
    Client interface for controlling the Stretch robot.
    
    This class provides access to the robot's subsystems (arm, lift, base, etc.) 
    and high-level routines. It communicates with the robot server to execute commands 
    and retrieve status.
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
                module_name = 'stretch4_body.robot.robot_client'
                class_name = self.robot_params[self.eoa_name]['py_class_name']+'_Client'
                self.subsystems[k] = getattr(importlib.import_module(module_name), class_name)(parent=self)
                self.end_of_arm = self.subsystems[k]

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
            raise RuntimeError("RobotClient startup failed.")
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
            start = True
            for n in subsystem_names:
                if n in self.subsystems and hasattr(self.subsystems[n],'is_moving'):
                    if not self.subsystems[n].is_moving():
                        start = False
            return  start
        self._wait_on_status(start_moving, timeout, do_pull=True)

    def wait_on_motion_finish(self,subsystem_names,timeout=15.0):
        """
        Wait for the specified subsystems to finish moving.
        
        Parameters
        ----------
        subsystem_names : list of str
            List of subsystem names to check.
        timeout : float, optional
            Timeout in seconds, by default 15.0.
        """
        def done_moving():
            done = True
            for n in subsystem_names:
                if n in self.subsystems and hasattr(self.subsystems[n],'is_moving'):
                    if self.subsystems[n].is_moving():
                        done = False
            return  done
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
        
        do_push & do_pull are useful to disable when you're running Stretch Body II
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
            True if routine completed, False if either:
             - input arg wait_on_completion=False
             - or timed out before routine completed
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

    def set_velocity(self, vx_m, vy_m, w_r, a_m=None, a_r=None):
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
        self._queue_command("omnibase", "set_velocity", vx_m, vy_m, w_r, a_m, a_r)

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


# #####################################################################

class LineSensorLoopClient(SubsystemClient):
    def __init__(self,parent=None):
        SubsystemClient.__init__(self,name='line_sensor_loop',parent=parent)

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
            raise RuntimeError(f"Cannot send movement command. Joint {self.name} has not been homed.")
        self._queue_command(self.name, "move_by",x_m, v_m=v_m, a_m=a_m, stiffness=stiffness, req_calibration=req_calibration,contact_sensitivity_pos=contact_sensitivity_pos, contact_sensitivity_neg=contact_sensitivity_neg)

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
            raise RuntimeError(f"Cannot send movement command. Joint {self.name} has not been homed.")
        self._queue_command(self.name, "set_velocity", v_m, a_m=a_m, stiffness=stiffness,req_calibration=req_calibration, contact_sensitivity_pos=contact_sensitivity_pos, contact_sensitivity_neg=contact_sensitivity_neg)

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
            raise RuntimeError(f"Cannot send movement command. Joint {self.name} has not been homed.")
        self._queue_command(self.name, "move_to",x_m, v_m=v_m, a_m=a_m, stiffness=stiffness, req_calibration=req_calibration,contact_sensitivity_pos=contact_sensitivity_pos, contact_sensitivity_neg=contact_sensitivity_neg)


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
    def __init__(self, joint_name,parent=None, ip_address=None):
        self.joint_name=joint_name
        SubsystemClient.__init__(self, name=joint_name, parent=parent, ip_address=ip_address)
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
            raise RuntimeError(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
        self._queue_command(f'{self.joint_name}.end_of_arm', "move_by",self.joint_name,x_r, v_r, a_r)
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
            raise RuntimeError(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
        self._queue_command(f'{self.joint_name}.end_of_arm', "move_to", self.joint_name, x_r, v_r, a_r)
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
        if not self.is_homed():
            raise RuntimeError(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
        self._queue_command(f'{self.joint_name}.end_of_arm', "set_velocity", self.joint_name, v_r, a_r)
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
            raise RuntimeError(f"Cannot send movement command. Joint {self.joint_name} has not been homed.")
        self._queue_command(f'{self.joint_name}.end_of_arm', "pose", self.joint_name, p, v_r, a_r)
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
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='wrist_yaw', parent=parent, ip_address=ip_address)

class WristRollClient(WristJointClient):
    """ Client for the wrist roll joint. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='wrist_roll', parent=parent, ip_address=ip_address)

class WristPitchClient(WristJointClient):
    """ Client for the wrist pitch joint. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='wrist_pitch', parent=parent, ip_address=ip_address)

class StretchGripperClient(WristJointClient):
    """ Client for the stretch gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='stretch_gripper', parent=parent, ip_address=ip_address)
        self.pct_max_open = 100 * abs(self.params['range_deg'][1] / self.params['range_deg'][0])
        self.poses = {'zero': 0,
                      'open': self.pct_max_open,
                      'close': -100}
        self.status['gripper_conversion'] = {'aperture_m': 0.0,
                                             'finger_rad': 0.0,
                                             'finger_effort': 0.0,
                                             'finger_vel': 0.0}
        self.gripper_conversion = GripperConversion(self.params)

class ParallelGripperClient(WristJointClient):
    """ Client for the parallel gripper. """
    def __init__(self, parent=None, ip_address=None):
        WristJointClient.__init__(self, joint_name='parallel_gripper', parent=parent, ip_address=ip_address)
        self.poses = {'zero': 0,
                      'open': deg_to_rad(self.params['range_deg'][1]),
                      'mid': deg_to_rad(self.params['range_deg'][1]) / 2,
                      'close': 0}

    def move_to_mm(self, x_mm, v_r=None, a_r=None):
        self._queue_command(f'{self.joint_name}.end_of_arm', "move_to_mm", self.joint_name, x_mm, v_r, a_r)

    def move_by_mm(self, x_mm, v_r=None, a_r=None):
        self._queue_command(f'{self.joint_name}.end_of_arm', "move_by_mm", self.joint_name, x_mm, v_r, a_r)
# #####################################################################
class EndOfArmClient(SubsystemClient):
    """
    Client interface for the End of Arm (Tool).
    """

    def __init__(self,name='end_of_arm',parent=None):
        SubsystemClient.__init__(self,name=name,parent=parent)
        self.joints = list(self.robot_params[self.name].get('devices', {}).keys())

    def do_ping(self, joint):
        """
        Ping a specific joint in the end of arm tool.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "do_ping",joint)

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
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        if not self.is_homed(joint):
            raise RuntimeError(f"Cannot send movement command. Joint {joint} has not been homed.")
        self._queue_command(f'{joint}.end_of_arm', "move_by",joint,x_r, v_r, a_r)

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
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        if not self.is_homed(joint):
            raise RuntimeError(f"Cannot send movement command. Joint {joint} has not been homed.")
        self._queue_command(f'{joint}.end_of_arm', "move_to", joint, x_r, v_r, a_r)

    def move_to_mm(self, joint, x_mm, v_r=None, a_r=None):
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "move_to_mm", joint, x_mm, v_r, a_r)

    def move_by_mm(self, joint, x_mm, v_r=None, a_r=None):
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "move_by_mm", joint, x_mm, v_r, a_r)
    
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
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        if not self.is_homed(joint):
            raise RuntimeError(f"Cannot send movement command. Joint {joint} has not been homed.")
        self._queue_command(f'{joint}.end_of_arm', "set_velocity", joint, v_r, a_r)

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
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        if not self.is_homed(joint):
            raise RuntimeError(f"Cannot send movement command. Joint {joint} has not been homed.")
        self._queue_command(f'{joint}.end_of_arm', "pose", joint, p,v_r, a_r)

    def quick_stop(self,joint):
        """
        Quickly stop a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "quick_stop",joint)

    def disable_torque(self, joint):
        """
        Disable torque on a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "disable_torque", joint)

    def enable_torque(self, joint):
        """
        Enable torque on a specific joint.
        
        Parameters
        ----------
        joint : str
            Name of the joint.
        """
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "enable_torque", joint)

    def pause_sentry(self, joint):
        """
        Pause the safe_motion sentry on a specific joint.
        """
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "pause_sentry", joint)

    def unpause_sentry(self, joint):
        """
        Unpause the safe_motion sentry on a specific joint.
        """
        if joint not in self.joints:
            raise ValueError(f"Joint {joint} not found in end of arm tool.")
        self._queue_command(f'{joint}.end_of_arm', "unpause_sentry", joint)

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

    def TODOwait_on_motion_start(self,joint_names,timeout=0.5):
        def start_moving():
            start = True
            for n in joint_names:
                if not self.status[n].is_moving():
                    start = False
            return  start
        self._wait_on_status(start_moving, timeout)

    def TODOwait_on_motion_finish(self,joint_names,timeout=15.0):
        def done_moving():
            done = True
            for n in joint_names:
                if self.subsystems[n].is_moving():
                    done = False
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