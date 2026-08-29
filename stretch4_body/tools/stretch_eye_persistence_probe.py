#!/usr/bin/env python3
"""
Diagnostic probe for eye-LED persistence on Stretch 4.

Answers three questions empirically, on hardware:

  1. Can colour reach the eyes from client code today, without
     modifying robot_client.py?
  2. Does SentryEyeAnimations overwrite a commanded eye state?
  3. Does pause_sentry('sentry_eye_animations') make it persist?

This script only reads and writes eye state plus the sentry pause flag.
It touches no motors and no configuration files.
"""
from __future__ import print_function

import argparse
import sys
import time

from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.power_periph import PowerPeriphDefn

SENTRY = 'sentry_eye_animations'

# Distinctive so it is obvious on the robot when it survives and when it does not.
PROBE_ANIM = PowerPeriphDefn.EYE_ANIM_IDLE_GLOW
PROBE_RGB = (0, 255, 0)  # saturated green, nothing the sentry ever picks


def set_eyes(robot, left, right, intensity=255, rgb=(255, 255, 255)):
    """
    Command the eyes including colour.

    Note: on main, PowerPeriphClient.set_eye_animation accepts only
    left_idx/right_idx and silently drops colour. RobotServer dispatch is a
    generic getattr(subsystem, method)(**kwargs) (robot_server.py:235), so
    queueing the command directly forwards colour with no code change.
    """
    r, g, b = rgb
    robot.power_periph._queue_command(
        subsystem='power_periph',
        command='set_eye_animation',
        left_idx=left,
        right_idx=right,
        intensity=intensity,
        r=r,
        g=g,
        b=b,
    )
    # ignore_control_lock: this is an LED-only command, so do not seize the
    # motion-control lock from whoever else may be driving the robot.
    robot.push_command(ignore_control_lock=True)


def banner(text):
    print('\n' + '=' * 68)
    print(text)
    print('=' * 68)


def observe(seconds):
    for remaining in range(seconds, 0, -1):
        sys.stdout.write('\r  watching the eyes... {:2d}s '.format(remaining))
        sys.stdout.flush()
        time.sleep(1.0)
    sys.stdout.write('\r' + ' ' * 40 + '\r')
    sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--watch-sec', type=int, default=20,
                    help='seconds to observe in each phase (default: 20)')
    ap.add_argument('--skip-stomp', action='store_true',
                    help='skip phase 1 and go straight to the paused-sentry test')
    args = ap.parse_args()

    robot = RobotClient()
    if not robot.startup():
        print('ERROR: could not reach the robot server. Is stretch_body_server running?')
        return 1

    try:
        if not args.skip_stomp:
            banner('PHASE 1  sentry ACTIVE  -  expect the green to be overwritten')
            print('Commanding both eyes: IDLE_GLOW, green (0,255,0).')
            print('SentryEyeAnimations re-pushes a random animation and colour')
            print('every 0.5-5s (sentry_eye_animations.py:104), so the green')
            print('should be replaced by white / pink / motion almost immediately.')
            set_eyes(robot, PROBE_ANIM, PROBE_ANIM, rgb=PROBE_RGB)
            observe(args.watch_sec)
            print('  -> Did the eyes stay green the whole time?  Expected: NO')

        banner('PHASE 2  sentry PAUSED  -  expect the green to hold')
        print("Calling robot.pause_sentry('{}')".format(SENTRY))
        robot.pause_sentry(SENTRY)
        robot.push_command(ignore_control_lock=True)
        time.sleep(1.0)
        print('Commanding both eyes: IDLE_GLOW, green (0,255,0)  ...once.')
        set_eyes(robot, PROBE_ANIM, PROBE_ANIM, rgb=PROBE_RGB)
        observe(args.watch_sec)
        print('  -> Did the eyes stay green the whole time?  Expected: YES')

        banner('PHASE 3  per-eye check  -  animation differs, colour cannot')
        print('Commanding LEFT=LEFT_HALF, RIGHT=RIGHT_HALF, colour blue (0,0,255).')
        print('The two eyes should show DIFFERENT animations but the SAME colour:')
        print('the P13 wire format carries one shared (intensity,r,g,b) for both')
        print('eyes (power_periph.py:944-951), so per-eye colour is not expressible.')
        set_eyes(robot,
                 PowerPeriphDefn.EYE_ANIM_LEFT_HALF,
                 PowerPeriphDefn.EYE_ANIM_RIGHT_HALF,
                 rgb=(0, 0, 255))
        observe(args.watch_sec)

        banner('RESTORING')
        print('Unpausing {} and returning the eyes to IDLE_GLOW.'.format(SENTRY))
        set_eyes(robot, PROBE_ANIM, PROBE_ANIM, rgb=(255, 255, 255))
        robot.unpause_sentry(SENTRY)
        robot.push_command(ignore_control_lock=True)
        time.sleep(1.0)
        print('Done. The sentry is running again; the robot is back as found.')
        return 0
    finally:
        robot.stop()


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print('\nInterrupted.')
        sys.exit(130)
