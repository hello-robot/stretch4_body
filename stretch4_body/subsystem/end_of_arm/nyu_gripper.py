import time

from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
import stretch4_body.core.hello_utils as hu
from stretch4_body.subsystem.end_of_arm.gripper_conversion import get_angle_from_chord_length_and_radius


class NYUGripper(FeetechSMHello):
    """
    API to the NYU tendon-driven gripper (https://nyu-gripper.pages.dev)
    One Feetech servo winds a Kevlar tendon to close the fingers; a return spring opens them.
    The tendon transmits force only in the closing direction, so homing always drives closed
    and the open limit is enforced in software (range_deg[1] + range_pad_deg).
    Position is a unit-less 'pct': 0.0 is fully closed (the homing datum at the closed
    hardstop) and 100.0 is fully open. World radians are 0 at the closed hardstop,
    increasing toward open.
    """
    def __init__(self, chain=None, usb=None, name='nyu_gripper', is_direct=False):
        FeetechSMHello.__init__(self, name, chain, usb, is_direct=is_direct)
        self.status['pos_pct'] = 0.0
        self.pct_max_open = 100.0

        self.poses = {'zero': 0.0,
                      'open': self.pct_max_open,
                      'close': 0.0}

        self.status['gripper_conversion'] = self.get_conversion_status()

    def startup(self):
        return FeetechSMHello.startup(self)

    def home(self, end_pos=100.0, delay_at_stop=0.5):
        """
        Tendon-aware homing: drive the spool in the closing direction and detect
        the closed hardstop by POSITION settling instead of the base class's
        velocity threshold. At the tendon hardstop the Feetech velocity register
        stays noisy/nonzero, so FeetechSMHello.home() never sees the stall and
        times out. Overridden here (not in the shared base class) because the
        behavior is specific to this tendon mechanism. The calibration
        bookkeeping mirrors FeetechSMHello.home() exactly.

        end_pos is in pct (routed through the overridden move_to).
        delay_at_stop keeps PWM on the closed hardstop briefly to tension the
        tendon so the homing datum is repeatable. Kept short: long dwells at
        the stop overstrain the fingers/tendon on this build.
        """
        self._cancel_homing_clear()
        self.bubble_up_comm_exception = True
        self.status['is_homing'] = False
        try:
            if not self.hw_valid:
                self.logger.warning('Not able to home %s. Hardware not present' % self.name)
                return False
            if not self.params['req_calibration']:
                self.logger.info('Homing not required for: ' + self.name)
                return False

            self.pull_status()
            if not self.check_servo_errors():
                self.logger.warning('Hardware error, unable to home. Exiting')
                return False

            self.status['is_homing'] = True
            # This switches the encoder from multi-turn to single-turn
            self.enable_pwm()

            self.logger.info(f'Moving to closed hardstop ({self.name})...')
            self.set_pwm(self.params['homing_pwm'])
            ts = time.time()
            time.sleep(1.0)
            timeout = False
            stalled = False
            check_s = 0.5  # stall-check interval; shorter = less time pulling at the stop
            pos_window_t = self.params.get('homing_stall_pos_window_t', 60)  # ticks/s
            last_pos = self.motor.get_pos()
            last_pos_ts = time.time()
            while not stalled and not timeout and not self.cancel_homing_event.is_set():
                if time.time() - last_pos_ts >= check_s:
                    p = self.motor.get_pos()
                    stalled = abs(p - last_pos) < pos_window_t * check_s
                    last_pos = p
                    last_pos_ts = time.time()
                timeout = time.time() - ts > 15.0
                time.sleep(0.1)
            time.sleep(delay_at_stop)

            self.set_pwm(0.0)

            if self.cancel_homing_event.is_set():
                self.logger.error('Homing cancelled for: ' + self.name)
                self.status['is_homing'] = False
                return False
            if timeout:
                raise RuntimeError('Timed out moving to closed hardstop. Check that the tendon is anchored to the spool (string slip prevents a stall). Exiting.')
            if not self.check_servo_errors():
                raise RuntimeError('Hardware error, unable to home. Exiting')

            self.home_pos_offset = self.motor.get_pos()

            bias_t = self.params.get('homing_offset_bias_t', 0)
            if bias_t != 0:
                self.logger.info(f"Applying homing offset bias of {bias_t} ticks")
                self.home_pos_offset += bias_t

            self.logger.info('Closed hardstop contact at position (ticks): %d' % self.home_pos_offset)
            self.motor.set_hello_robot_pos_offset(self.home_pos_offset)
            self.motor.set_is_calibrated(1)
            self.status['pos_calibrated'] = True
            self.update_joint_limits()

            # This switches the encoder from single back to multi-turn
            # It locks in the encoder offset at this point
            self.enable_pos()
            if end_pos is not None and not self.cancel_homing_event.is_set():
                self.logger.info(f'Moving to open pose ({end_pos} pct)')
                self.move_to(end_pos)
                time.sleep(2.0)
                self.wait_until_at_setpoint(timeout=6.0)
            self.status['is_homing'] = False
            self.bubble_up_comm_exception = False
            self.logger.info(f"Done homing {self.name}")
            return True
        except RuntimeError as e:
            self.logger.error(f'Runtime error, during homing: {e}')
            return False
        except Exception as e:
            self.logger.error(f'Communication error, unable to home. Exiting. {e=}')
            return False
        finally:
            self.status['is_homing'] = False

    def pretty_print(self):
        print('--- NYUGripper ----')
        print("Position (%)", self.status['pos_pct'])
        FeetechSMHello.pretty_print(self)

    def pose(self, p, v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'close')
        """
        self.move_to(self.poses[p], v_r, a_r)

    def move_to(self, pct, v_r=None, a_r=None):
        """
        pct: commanded absolute position (Pct). 0 fully closed, 100 fully open.
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        x_r = self.pct_to_world_rad(pct)
        FeetechSMHello.move_to(self, x_des=x_r, v_des=v_r, a_des=a_r)

    def move_by(self, delta_pct, v_r=None, a_r=None):
        """
        delta_pct: commanded incremental motion (pct).
        v_r: velocity for trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        if self.is_direct:
            self.pull_status()  # Ensure up to date as server not doing pull_status
        self.move_to(self.status['pos_pct'] + delta_pct, v_r, a_r)

    ############### Utilities ###############

    def pull_status(self, data=None):
        FeetechSMHello.pull_status(self, data)
        self.status['pos_pct'] = self.world_rad_to_pct(self.status['pos'])
        self.status['gripper_conversion'] = self.get_conversion_status()

    def pct_to_world_rad(self, pct):
        return hu.deg_to_rad(self.params['range_deg'][1]) * pct / 100.0

    def world_rad_to_pct(self, r):
        return 100.0 * r / hu.deg_to_rad(self.params['range_deg'][1])

    def get_conversion_status(self):
        """
        Approximate finger state from pct via a linear aperture model
        (aperture is assumed linear in pct between the measured closed/open values).
        Uses the same status keys as the Stretch Gripper so the pose tools and the
        self-collision model work unchanged.
        """
        gc = self.params['gripper_conversion']
        pct = min(max(self.status.get('pos_pct', 0.0), 0.0), 100.0)
        ao = gc['aperture_open_m']
        ac = gc['aperture_closed_m']
        length = gc['finger_length_m']
        aperture_m = ac + (ao - ac) * pct / 100.0
        finger_rad = get_angle_from_chord_length_and_radius(length, aperture_m) / 2.0
        finger_rad_open = get_angle_from_chord_length_and_radius(length, ao) / 2.0
        range_rad = hu.deg_to_rad(self.params['range_deg'][1])
        finger_vel = self.status.get('vel', 0.0) * finger_rad_open / range_rad
        return {'aperture_m': aperture_m,
                'finger_rad': finger_rad,
                'finger_vel': finger_vel,
                'finger_effort': self.status.get('effort', 0.0)}
