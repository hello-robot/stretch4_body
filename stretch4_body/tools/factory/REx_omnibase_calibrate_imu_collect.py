#!/usr/bin/env python3
from __future__ import print_function
from future.builtins import input
import time
import argparse as ap
import stretch4_body.robot.robot as rb
import yaml
import stretch4_body.core.hello_utils as hu

hu.print_stretch_re_use()

def create_sample():
    sample = {'omnibase':{'theta_vel':None, 'theta':None, 'pose_time_s':None},
              'imu':{'ax':None, 'ay':None, 'az':None,
                     'gx':None, 'gy':None, 'gz':None,
                     'mx':None, 'my':None, 'mz':None,
                     'timestamp':None},
              'time':None}
    return sample

def fill_sample_type(sample_type, sample, status):
    s = sample[sample_type]
    for k in s.keys():
        s[k] = status[k]
    
def fill_sample(sample, omnibase_status, power_periph_status):
    sample['time'] = time.time()
    fill_sample_type('omnibase', sample, omnibase_status)
    fill_sample_type('imu', sample, power_periph_status['imu'])


if __name__ == '__main__':
    parser = ap.ArgumentParser(description='Collect mobile omnibase IMU calibration data and work with resulting files.')
    parser.add_argument('--validate', action='store_true', help='Collect measurements using existing calibration parameters to validate its fit.')
    parser.add_argument('--qc', action='store_true', help='QC script mode.')
    args=parser.parse_args()

    try:
        # make a little over one circle
        turn_angle_deg = 380.0 #180.0 #90.0 #10.0
        turn_time = 30.0
        angular_velocity_rad_per_s =hu.deg_to_rad( turn_angle_deg/ turn_time)
        turn_angle_rad = hu.deg_to_rad(turn_angle_deg)
        
        robot = rb.Robot()

        #Temporarily set imu calibration on Pimu to Identity
        if not args.validate:
            robot.power_periph.config['mag_offsets'][0]=0
            robot.power_periph.config['mag_offsets'][1] = 0
            robot.power_periph.config['mag_offsets'][2] = 0

            robot.power_periph.config['mag_softiron_matrix'][0] = 1
            robot.power_periph.config['mag_softiron_matrix'][1] = 0
            robot.power_periph.config['mag_softiron_matrix'][2] = 0
            robot.power_periph.config['mag_softiron_matrix'][3] = 0
            robot.power_periph.config['mag_softiron_matrix'][4] = 1
            robot.power_periph.config['mag_softiron_matrix'][5] = 0
            robot.power_periph.config['mag_softiron_matrix'][6] = 0
            robot.power_periph.config['mag_softiron_matrix'][7] = 0
            robot.power_periph.config['mag_softiron_matrix'][8] = 1

            robot.power_periph.config['gyro_zero_offsets'][0] = 0
            robot.power_periph.config['gyro_zero_offsets'][1] = 0
            robot.power_periph.config['gyro_zero_offsets'][2] = 0

            robot.power_periph.config['rate_gyro_vector_scale']=1.0
            robot.power_periph.config['gravity_vector_scale']=1.0

        # wait so that cables can be unplugged prior to the turn
        if not args.qc:
            print('WARNING: The robot will make a {0} degree turn.'.format(turn_angle_deg))
            seconds = input('Enter the number of seconds to wait prior to turning: ')
            seconds = int(seconds)
            print('Will wait {0} seconds before turning {1} degrees.'.format(seconds, turn_angle_deg))
            count_down = seconds
            print(count_down)
            for n in range(seconds):
                time.sleep(1)
                count_down = count_down - 1
                print(count_down)

        robot.startup()
        robot.power_periph.trigger_beep()
        robot.push_command()

        #robot.omnibase.enable_pos_incr_mode()
        #robot.push_command()

        time_before_command = time.time()
        #robot.omnibase.rotate_by(turn_angle_rad)
        robot.omnibase.rotate_by(turn_angle_rad, angular_velocity_rad_per_s)
        robot.push_command()
        time_after_command = time.time()
        command_duration_s = time_after_command - time_before_command
        print('command duration in seconds =', command_duration_s)
        
        min_time_s = 5.0
        samples = []
        start_time = time.time()
        # collect samples for at least a min_time and while the robot is
        # moving
        #robot.pull_status()
        while (((time.time() - start_time) < min_time_s)):# or abs(robot.omnibase.status['theta_vel'] > 0.01)):
            print('DT {0}'.format(time.time() - start_time))
            print('Vel',robot.omnibase.status['theta_vel'])
            #robot.pull_status()
            sample = create_sample()
            fill_sample(sample, robot.omnibase.status, robot.power_periph.status)
            #print(sample)
            samples.append(sample)
            #time.sleep(0.05)
            time.sleep(0.1)

        robot.power_periph.trigger_beep()
        robot.push_command()
        robot.stop()

        calibration_directory = hu.get_fleet_directory()+'calibration_omnibase_imu/'
        if args.validate:
            filename = calibration_directory + hu.get_fleet_id()+'_omnibase_imu_calibration_data_validate_' + hu.create_time_string() + '.yaml'
        else:
            filename = calibration_directory + hu.get_fleet_id() + '_omnibase_imu_calibration_data_' + hu.create_time_string() + '.yaml'
        print('Saving calibration_data to a YAML file named {0}'.format(filename))
        fid = open(filename, 'w')
        yaml.dump(samples, fid)
        fid.close()
        
    except (KeyboardInterrupt, SystemExit):
        pass


            
