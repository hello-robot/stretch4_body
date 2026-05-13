from stretch4_body.behavior.sentries.sentry import Sentry
from stretch4_body.behavior.sentries.self_collision.self_collision_loop import SelfCollisionLoop
import stretch4_body.core.hello_utils as hu
import time

class SentrySelfCollision(Sentry):
    """
    Wrap self collision loop in a device for easy integration into SentryManager
    Step is called at 100hz prior to commands being computed in the control loop
    SelfSentryCollision will set collision flags on each joint so that  commands (arm.move_to, etc) are clipped
    """
    def __init__(self, robot):
        Sentry.__init__(self, name="sentry_self_collision", robot=robot)
        self.self_collision_loop=SelfCollisionLoop(robot)

        self.urdf_joint_map={'lift_joint':'lift',
                             'arm_l1_joint':'arm',
                             'arm_l2_joint': 'arm',
                             'arm_l3_joint': 'arm',
                             'arm_l4_joint': 'arm',
                             'wrist_yaw_joint': 'wrist_yaw',
                             'wrist_pitch_joint': 'wrist_pitch',
                             'wrist_roll_joint': 'wrist_roll'}
        self.status={}
        for j in self.params['urdf_joints_to_sentry']:
            self.status[self.urdf_joint_map[j]]={'in_collision_stop':{'pos': False, 'neg': False},'ts_collision_stop':0}
        self.ts_last_sound=time.time()
        self.in_collision=False
        #print('SS',self.status)
        self.required_subsystems = self.params['required_subsystems']
        self.is_homed_warning=True

    def startup(self):
        super().startup()
        if self.is_valid:
            return self.self_collision_loop.startup()
        return True

    def stop(self):
        self.self_collision_loop.stop()
        return True

    def pause(self):
        # Clear all collision stops on joints to prevent motion lock
        for uj in self.params['urdf_joints_to_sentry']:
            joint_name = self.urdf_joint_map[uj]
            self.status[joint_name]['in_collision_stop'] = {'pos': False, 'neg': False}
            
            if joint_name == 'lift' and self.robot.get_subsystem('lift') is not None:
                self.robot.lift.step_collision_avoidance(self.status[joint_name]['in_collision_stop'])
            elif joint_name == 'arm' and self.robot.get_subsystem('arm') is not None:
                self.robot.arm.step_collision_avoidance(self.status[joint_name]['in_collision_stop'])
            elif (joint_name in ['wrist_yaw', 'wrist_pitch', 'wrist_roll']) and self.robot.get_subsystem('end_of_arm') is not None:
                self.robot.end_of_arm.step_collision_avoidance(joint_name, self.status[joint_name]['in_collision_stop'])

    @staticmethod
    def extract_collision_dirs(ccd, ecd=None):
        #Given nested collison direction dictionary, return dictionary of just the joints and their gradients that will resolve the collision
        #Eg {'lift_joint': -0.7044160659317538} indicates moving the lift in the negative direction will resolve.
        if ecd is None: #avoid mutable
            ecd={}
        for l in ccd:
            if 'joint' in l:
                ecd[l] = float(ccd[l])
            else:
                ecd.update(SentrySelfCollision.extract_collision_dirs(ccd[l],ecd))
        return ecd

    def step(self):
        if not self.is_valid :
            return
        if not self.robot.is_homed():
            if self.is_homed_warning:
                self.logger.warning('Robot is not homed. Disabling SentrySelfCollision until homing complete')
                self.is_homed_warning=False
            return
        self.self_collision_loop.step()
        self.status.update(self.self_collision_loop.status)
        ecd=SentrySelfCollision.extract_collision_dirs(self.status['collision_directions'])

        #Play sound if new collision
        if not self.in_collision and len(self.status['collisions'])>0:# and time.time()-self.ts_last_sound > 2.0:

            self.logger.info(f'New collision {self.status["collisions"]}')
            hu.play_sound(hu.get_sounds_dir()+'/water_drop.wav')
            self.ts_last_sound=time.time()
        self.in_collision=len(self.status['collisions'])>0

        for uj in self.params['urdf_joints_to_sentry']:
            joint_name=self.urdf_joint_map[uj]
            if uj in ecd:
                self.status[joint_name]['in_collision_stop'] = {'pos': ecd[uj] < 0, 'neg': ecd[uj] > 0}
                self.status[joint_name]['ts_collision_stop']=time.time()
            else:
                self.status[joint_name]['in_collision_stop']={'pos': False, 'neg': False}

            if joint_name=='lift' and self.robot.get_subsystem('lift') is not None:
                self.robot.lift.step_collision_avoidance(self.status[joint_name]['in_collision_stop'])
            if joint_name=='arm' and self.robot.get_subsystem('arm') is not None:
                self.robot.arm.step_collision_avoidance(self.status[joint_name]['in_collision_stop'])
            if (joint_name=='wrist_yaw' or joint_name=='wrist_pitch' or joint_name=='wrist_roll') and self.robot.get_subsystem('end_of_arm') is not None:
                self.robot.end_of_arm.step_collision_avoidance(joint_name,self.status[joint_name]['in_collision_stop'])




