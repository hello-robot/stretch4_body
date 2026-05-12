import threading
from stretch4_body.core.feetech.feetech_SM_hello import FeetechSMHello
from stretch4_body.core.hello_utils import *


class WristYaw(FeetechSMHello):
    """
    API to the Stretch wrist yaw joint
    """

    def __init__(self, chain=None, usb=None, name="wrist_yaw"):
        FeetechSMHello.__init__(self, name, chain, usb)
        self.poses = {
            "side": deg_to_rad(90.0),
            "forward": deg_to_rad(0.0),
            "stow": deg_to_rad(180.0),
        }

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

    def pose(self, p, v_r=None, a_r=None):
        """
        p: Dictionary key to named pose (eg 'forward')
        v_r: velocityfor trapezoidal motion profile (rad/s).
        a_r: acceleration for trapezoidal motion profile (rad/s^2)
        """
        self.move_to(self.poses[p], v_r, a_r)
