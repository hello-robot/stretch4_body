from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.core.hello_utils import *
from stretch4_body.core.rerun_plot import RRplot
import time
# ######################################################

# class VisualizeTilt():
#     def __init__(self, robot):
#         self.robot = robot
#         self.rrplot = RRplot(name="Overtilt Avoidance", open_browser=False)
#         self.rrplot.register(key="gravity_tilt", color_idx=0)
#         self.rrplot.setup_blueprint(collapse_panels=False)


#     def step(self):
#         v=self.robot.power_periph.status['imu']['gravity_tilt']
#         self.rrplot.log_scalar(key="gravity_tilt", value=rad_to_deg(v))

class SentryLimitVelOnPose(Sentry):
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_limit_vel_on_pose", robot=robot)
        self.status={'limit_omnibase_rotation_by_arm': 0.0, 'limit_omnibase_translation_by_lift': 0.0, 'limit_omnibase_rotation_by_lift': 0.0}
        #self.vt=VisualizeTilt(self.robot)

    def limit_omnibase_rotation_by_arm(self):
        #Limit angular velocity of base depending on arm extension
        vm = self.robot.omnibase.params['motion']['max']['vel_w_r']
        vd = self.robot.omnibase.params['motion']['slow']['vel_w_r']
        am = self.robot.omnibase.params['motion']['max']['accel_w_r']
        ad = self.robot.omnibase.params['motion']['slow']['accel_w_r']
        k = self.robot.arm.status['pos']/self.robot.arm.params['range_m'][1] #Normalized arm extension
        k=max(0,min(1,1-k))
        vm = vd + (vm - vd) * k
        am = ad + (am - ad) * k
        limit_pct = vm / self.robot.omnibase.params['motion']['max']['vel_w_r']
        self.robot.omnibase.set_curr_max_vel_w_r(vm)
        self.robot.omnibase.set_curr_max_accel_w_r(am)
        self.status['limit_omnibase_rotation_by_arm'] = limit_pct
        #print(f"limit_omnibase_rotation_by_arm {limit_pct*100}%")

    def limit_omnibase_translation_by_lift(self):
        #Limit translational velocity of base depending on lift height
        x_pos_neg=self.params['lift_lower_safe_height_m'] #Below this lift height allow for full velocity
        vm = self.robot.omnibase.params['motion']['max']['vel_xy_m']
        vd = self.robot.omnibase.params['motion']['default']['vel_xy_m']
        am = self.robot.omnibase.params['motion']['max']['accel_xy_m']
        ad = self.robot.omnibase.params['motion']['default']['accel_xy_m']
        k = (self.robot.lift.status['pos']-x_pos_neg)/(self.robot.lift.params['range_m'][1]-x_pos_neg) #Normalized lift height
        k=max(0,min(1,1-k))
        vm = vd + (vm - vd) * k
        am = ad + (am - ad) * k
        limit_pct = vm / self.robot.omnibase.params['motion']['max']['vel_xy_m']
        self.robot.omnibase.set_curr_max_vel_xy_m(vm)
        self.robot.omnibase.set_curr_max_accel_xy_m(am)
        self.status['limit_omnibase_translation_by_lift'] = limit_pct
        #print(f"limit_omnibase_translation_by_lift {limit_pct*100}%")

    def limit_omnibase_rotation_by_lift(self):
        #Limit angular velocity of base depending on lift height
        x_pos_neg=self.params['lift_lower_safe_height_m'] #Below this lift height allow for full velocit
        vm = self.robot.omnibase.params['motion']['max']['vel_w_r']
        vd = self.robot.omnibase.params['motion']['slow']['vel_w_r']
        am = self.robot.omnibase.params['motion']['max']['accel_w_r']
        ad = self.robot.omnibase.params['motion']['slow']['accel_w_r']
        k = (self.robot.lift.status['pos']-x_pos_neg)/(self.robot.lift.params['range_m'][1]-x_pos_neg) #Normalized lift height
        k=max(0,min(1,1-k))
        vm = vd + (vm - vd) * k
        am = ad + (am - ad) * k
        limit_pct = vm / self.robot.omnibase.params['motion']['max']['vel_w_r']
        self.robot.omnibase.set_curr_max_vel_w_r(vm)
        self.robot.omnibase.set_curr_max_accel_w_r(am)
        self.status['limit_omnibase_rotation_by_lift'] = limit_pct
        #print(f"limit_omnibase_rotation_by_lift {limit_pct*100}%")

    def step(self):
        if not self.is_valid:
            return
        if self.params['limit_omnibase_rotation_by_arm']:
            self.limit_omnibase_rotation_by_arm()
        if self.params['limit_omnibase_translation_by_lift']:
            self.limit_omnibase_translation_by_lift()
        if self.params['limit_omnibase_rotation_by_lift']:
            self.limit_omnibase_rotation_by_lift()
