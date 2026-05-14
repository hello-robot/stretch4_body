#!/usr/bin/env python3
import time
from stretch4_body.core.device import Device
from multiprocessing import Process, Event
from stretch4_body.core.worker_loop import *
from stretch4_body.behavior.sentries.self_collision.self_collision_mujoco import MujocoJointStates, SelfCollisionMujoco
import math
# ###########################################################################################

def _cb_solver_loop_exit(lsa):
    return True

def _cb_solver_loop_pause(lsa):
    return True

def _cb_solver_loop_unpause(lsa):
    return True

def _cb_solver_loop_step(solver, q_cmd_in, status_out):
    joint_cfg_in=q_cmd_in.get_latest()
    if joint_cfg_in is not None:
        #print('Got',joint_cfg_in)
        state = MujocoJointStates.from_urdf_joint_state(joint_cfg_in)
        status_out['collisions']= solver.get_collisions(state)
        status_out['collision_directions']= solver.get_collision_directions(state)
        status_out['ts_solver'] = time.time()
    return True

# ###########################################################################################

def solver_loop(do_exit, rate_hz, q_admin, q_cmd, q_status):
    """

    """
    solver = SelfCollisionMujoco()
    if solver.startup():
        print("Self Collision Solver Started")
        worker_loop(
            loop_name='self_collision_loop',
            rate_hz=rate_hz,
            worker_instance=solver,
            q_admin=q_admin,
            q_status=q_status,
            q_cmd=q_cmd,
            do_exit=do_exit,
            callback_step=_cb_solver_loop_step,
            callback_pause=_cb_solver_loop_pause,
            callback_unpause=_cb_solver_loop_unpause,
            callback_exit=_cb_solver_loop_exit
        )
        #solver.stop()
        return True
    return False

# ###########################################################################################

class SelfCollisionLoop(Device):
    """
    SelfCollisionLoop runs a background process that runs the Mujuco self collision checker.

    """
    def __init__(self,robot):
        Device.__init__(self, 'self_collision_loop')
        self.solver_process = None
        self.q_cmd = hello_utils.CircularMultiprocessingQueue(10)
        self.q_status = hello_utils.CircularMultiprocessingQueue(10)
        self.q_admin = hello_utils.CircularMultiprocessingQueue(10)
        self.status = {'collisions':{},'collision_directions':{},'ts_solver':0}
        self.do_exit = Event()
        self.n_rate_log = 0
        self.rate_log={}
        self.frame_id_last = {}
        self.robot=robot


    def startup(self):
        """
        Launch the solver loop process.
        """
        timeout = False
        if self.solver_process is None:
            self.solver_process = Process(
                target=solver_loop,
                args=(self.do_exit, self.params['loop_rate_Hz'], self.q_admin, self.q_cmd, self.q_status)
            )
            self.solver_process.start()
            #os.system("taskset -p -c %d %d" % (self.params['cpu_affinity'], self.pjr_process.pid)) #Assign process to core

            # Wait for system to start posting status
            # ts=time.time()
            # while self.status['last_frame_time']==0 and not timeout:
            #     self.status.update(self.q_status.get(block=True, timeout=0.1))
            #     if time.time()-ts>2.0:
            #         timeout=True
        return True #not timeout

    def step(self,joint_cfg=None):
        #Update collision model given joint_cfg (or get joint_cfg from robot if not provided)
        #Called at 100hz from main control loop. 
        # Send joint configuration to solver loop, get back latest collisions (async)
        if joint_cfg is None:
            joint_cfg=self.get_urdf_joint_configuration(self.robot.status)
        self.q_cmd.put(joint_cfg)
        s=self.q_status.get_latest()
        if s is not None:
            self.status.update(s)
            # if len(s['collisions']) > 0:
            #     print('COLL',s['collisions'])


    def _manage_ctrlC(self, *args):
        # If you have multiple event processing processes, set each Event.
        self.do_exit.set()

    def stop(self):
        original_sigint = signal.getsignal(signal.SIGINT)
        signal.signal(signal.SIGINT, self._manage_ctrlC)
        self.q_admin.put('exit')
        if self.solver_process is not None:
            self.solver_process.join()
            self.solver_process = None
            
        self.q_admin.queue.cancel_join_thread()
        self.q_cmd.queue.cancel_join_thread()
        self.q_status.queue.cancel_join_thread()
            
        signal.signal(signal.SIGINT, original_sigint)

    @staticmethod
    def get_urdf_joint_configuration(robot_status):
        """
        Convert robot.status to URDF compatible dictionary of robot's current pose + padding based on current velocity of joint
        """
        s = robot_status
        from stretch4_body.core.robot_params import RobotParams
        _, robot_params = RobotParams.get_params()
        kbd = robot_params['self_collision_mujoco'][robot_params['robot']['model_name']]['k_brake_distance']
        dl = kbd['lift'] * s['lift']['braking_distance']

        configuration = {}

        if 'arm' in kbd:
            da = kbd['arm'] * s['arm']['braking_distance'] / 4.0
            configuration = {
                'arm_l1_joint': da + s['arm']['pos'] / 4.0,
                'arm_l2_joint': da + s['arm']['pos'] / 4.0,
                'arm_l3_joint': da + s['arm']['pos'] / 4.0,
                'arm_l4_joint': da + s['arm']['pos'] / 4.0
            }
        configuration['lift_joint'] = dl + s['lift']['pos']

        if 'end_of_arm' in s:
            dwy = kbd['wrist_yaw'] * s['end_of_arm']['wrist_yaw']['braking_distance']
            dwp = kbd['wrist_pitch'] * s['end_of_arm']['wrist_pitch']['braking_distance']
            dwr = kbd['wrist_roll'] * s['end_of_arm']['wrist_roll']['braking_distance']
            configuration['wrist_yaw_joint']=s['end_of_arm']['wrist_yaw']['pos']+dwy
            configuration['wrist_pitch_joint'] = s['end_of_arm']['wrist_pitch']['pos']+dwp
            configuration['wrist_roll_joint'] = s['end_of_arm']['wrist_roll']['pos']+dwr
            if robot_params['robot']['tool']=='eoa_wrist_dw4_tool_sg4':
                #print('SelfCollisionLoop Gripper Conversion',robot_status['end_of_arm']['stretch_gripper']['gripper_conversion'])
                configuration['gripper_finger_left_joint'] = robot_status['end_of_arm']['stretch_gripper']['gripper_conversion']['finger_rad']
                configuration['gripper_finger_right_joint'] = robot_status['end_of_arm']['stretch_gripper']['gripper_conversion']['finger_rad']
                #configuration['stretch_gripper_joint'] = self.robot.end_of_arm.status['stretch_gripper']['pos']
            elif robot_params['robot']['tool']=='eoa_wrist_dw4_tool_pg4':
                configuration['finger_left_joint'] = robot_status['end_of_arm']['parallel_gripper']['pos']
                configuration['finger_right_joint'] = robot_status['end_of_arm']['parallel_gripper']['pos']

        return configuration



if __name__ == "__main__":

    def get_virtual_joint_cfg():
        jrange={'lift':[0.0, 1.1],'arm':[0, 0.52],'wrist_yaw':[-1.39, 4.42],'wrist_pitch':[-1.57, 0.56],'wrist_roll':[-3.14, 3.14]}
        trange={'lift':15.0,'arm':14.0,'wrist_yaw':13.0,'wrist_pitch':13.0,'wrist_roll':13.0} #seconds to go through range
        toffset={'lift':0.0,'arm':1.0,'wrist_yaw':0.0,'wrist_pitch':1.0,'wrist_roll':2.0}
        q={}
        for jn in ['lift','arm','wrist_yaw','wrist_pitch','wrist_roll']:
            tn=((time.time()+toffset[jn])%trange[jn])/trange[jn] #goes from 0 to 1 triangle
            qq=0.5*math.cos(math.pi*2*tn)+0.5#goes from 0 to 1 oscillating
            q[jn]=jrange[jn][0]+qq*(jrange[jn][1]-jrange[jn][0]) #oscillated across range of motion

        joint_cfg2={'lift_joint': q['lift'], 'arm_l1_joint': q['arm']/4,
         'arm_l2_joint': q['arm']/4, 'arm_l3_joint': q['arm']/4,
         'arm_l4_joint': q['arm']/4, 'wrist_yaw_joint': q['wrist_yaw'],
         'wrist_pitch_joint': q['wrist_roll'], 'wrist_roll_joint': q['wrist_pitch'],
         'gripper': 0.0}

        joint_cfg={'lift_joint': q['lift'], 'arm_l1_joint': q['arm']/4,
         'arm_l2_joint': q['arm']/4, 'arm_l3_joint': q['arm']/4,
         'arm_l4_joint': q['arm']/4, 'wrist_yaw_joint': 0,
         'wrist_pitch_joint': 0, 'wrist_roll_joint': 0,
         'gripper': 0.0}

        return joint_cfg

    import argparse

    parser = argparse.ArgumentParser(description="Check collisions for Stretch robot")
    parser.add_argument("--virtual", type=bool, default=True, help="Use virtual robot")
    parser.add_argument("--visualize", "-v", action="store_true",help="Open MuJoCo viewer to visualize the configuration")
    args = parser.parse_args()


    if args.visualize: #Open MuJoCo viewer to visualize the configuration directly (no loop)
        solver = SelfCollisionMujoco()
        if solver.startup():
            def cb(joint_states):
                 cfg=get_virtual_joint_cfg()
                 return MujocoJointStates.from_urdf_joint_state(cfg)
            
            joint_states=MujocoJointStates.from_urdf_joint_state(get_virtual_joint_cfg())
            solver.visualize(joint_states, highlight_collisions=True,highlight_collision_directions=True,callback=cb)


    if not args.visualize:
        rcl = SelfCollisionLoop(robot=None)
        if rcl.startup():
            try:
                while True:
                    rcl.step(get_virtual_joint_cfg())
                    #print('Rate: %f (Hz)' % pjl.status['rate_hz'])  # ['sensor_0'])
                    # print('Model update rate: ', lsl.status['model_update_stats']['curr_rate_hz'])
                    time.sleep(0.01)
            except:
                rcl.stop()


