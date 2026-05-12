import threading
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
import stretch4_body.core.hello_utils as hu


class WristRoll(FeetechSMHello):
    """
    API to the Stretch RE1 wrist roll joint
    """

    def __init__(self, chain=None, usb=None, name="wrist_roll"):
        FeetechSMHello.__init__(self, name, chain, usb)
        self.poses = {
            "cw_90": hu.deg_to_rad(90.0),
            "forward": hu.deg_to_rad(0.0),
            "ccw_90": hu.deg_to_rad(-90.0),
        }

    def stop(self, close_port=True):
        FeetechSMHello.stop(self, close_port)

    def home(
        self,
        cancel_homing_event: threading.Event,
        end_pos=0,
        delay_at_stop=0.25,
    ):
        """
        Home to hardstops
        """
        return FeetechSMHello.home(
            self,
            cancel_homing_event=cancel_homing_event,
            end_pos=end_pos,
            delay_at_stop=delay_at_stop,
        )
        # time.sleep(1.0) #extra time to get back

    def pose(self, p, v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'forward')
        v_r: velocityfor trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.move_to(self.poses[p], v_r, a_r)
