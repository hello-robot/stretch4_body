#!/usr/bin/env python3
import argparse
import stretch4_body.robot.robot_client as rc

parser = argparse.ArgumentParser(
    prog='Run available routines on Stretch Body Server'
)
args = parser.parse_args()

r = rc.RobotClient()
if not r.startup():
    exit()
try:
    while True:
        routines=r.robot_params['routine_manager']['controllers']
        print('')
        print('---------- Available routines --------')
        for i in range(len(routines)):
            print('%d: %s'%(i,routines[i]))
        try:
            idx=int(input('Enter idx of routine to run: '))
            print('Running: %s'%routines[idx])
            finished, rid = r.routines.run(routines[idx], wait_on_completion=True)
            if finished:
                print('Routine %s completed successfully'%routines[idx])
            else:
                print('Routine %s timed out before completion'%routines[idx])
        except (ValueError, IndexError):
            print('Invalid entry')
except (KeyboardInterrupt, SystemExit):
    pass
r.stop()