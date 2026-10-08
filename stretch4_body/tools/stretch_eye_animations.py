#!/usr/bin/env python3
from __future__ import print_function
import sys
import time
import argparse
import stretch4_body.core.hello_utils as hu
from stretch4_body.eyes import Eyes, ANIMATIONS, NAMED_COLORS
from stretch4_body.eyes.backends import SENTRY_NAME

hu.print_stretch_re_use()

parser = argparse.ArgumentParser(description='Set the eye LED animations, color and intensity',
                                 epilog='Example: stretch_eye_animations --both look_left --color "#00a0ff" --intensity 0.5 --hold')
parser.add_argument("--list", help="List the animations and named colors", action="store_true")
parser.add_argument("--left", help="Left eye animation, name or id")
parser.add_argument("--right", help="Right eye animation, name or id")
parser.add_argument("--both", help="Animation for both eyes, name or id")
parser.add_argument("--color", help="Color: name, '#rrggbb' or 'r,g,b'")
parser.add_argument("--intensity", help="Brightness. A whole number is the raw byte 0-255, a decimal is a fraction 0.0-1.0: "
                                        "'153' and '0.6' are the same, '1' is raw 1 (nearly off) and '1.0' is full")
parser.add_argument("--off", help="Turn both eyes off", action="store_true")
parser.add_argument("--idle", help="Firmware default idle glow", action="store_true")
parser.add_argument("--hold", help="Pause the eye sentry and hold the eyes until Ctrl-C", action="store_true")
parser.add_argument("--release", help="Resume the eye sentry (after a --hold that could not restore it)", action="store_true")
parser.add_argument("--fake", help="No robot, print what would be sent", action="store_true")
args = parser.parse_args()

def resume_failed(holder=None):
    why = 'another client ({}) holds the server lease'.format(holder) if holder else 'the server did not confirm it'
    return 'Could not resume {}: {}. Run `stretch_eye_animations --release` when the robot is idle.'.format(SENTRY_NAME, why)


def print_list():
    print('Animations:')
    for a in ANIMATIONS:
        print('  {:>2}  {:<13} {}'.format(a.id, a.name, a.description))
    print('Colors:')
    for name, rgb in NAMED_COLORS.items():
        print('  {:<9} {}'.format(name, rgb))


def parse_intensity_text(text):
    """'0.6' is a fraction (153), '153' is raw. Eyes.set applies the same rule to floats and ints."""
    return float(text) if '.' in text else int(text)


def parse_args_to_command():
    left = args.left or args.both
    right = args.right or args.both
    color = args.color
    if color and ',' in color:
        color = tuple(int(c) for c in color.split(','))
    intensity = args.intensity
    if intensity is not None:
        intensity = parse_intensity_text(intensity)
    if args.off:
        left = right = 'off'
    return dict(left=left, right=right, color=color, intensity=intensity)


def print_state(s, fake_sent=None):
    print('Eyes: left={} right={} color={} intensity={} ({})'.format(
        s.left or 'unchanged', s.right or 'unchanged', s.color_hex, s.intensity, s.source))
    if s.source == 'dropped':
        why = 'its lease is held by {}'.format(s.lease_holder) if s.lease_holder else 'a routine is running'
        print('Dropped: stretch_body_server did not accept the eye command ({}). '
              'Try again when the robot is idle.'.format(why))
    if fake_sent is not None:
        print('Payload (left, right, intensity, r, g, b): {}'.format(fake_sent))
    if s.runstop_active:
        print('Runstop is active: the firmware shows the runstop pattern until it is cleared.')
    if s.low_soc_override:
        print('Battery at {}%: the firmware shows the {} low battery pattern.'.format(s.battery_soc, s.low_soc_override))
    if s.sentry_active:
        print('{} is active and will replace these eyes within seconds. Use --hold.'.format(SENTRY_NAME))


def main():
    try:
        cmd = parse_args_to_command()
    except ValueError:
        print("--color takes a name, #rrggbb or r,g,b; --intensity takes a whole number 0-255 or a decimal 0.0-1.0 "
              "(1 is raw 1, 1.0 is full)")
        return 1
    has_command = args.idle or any(v is not None for v in cmd.values())
    if args.list:
        print_list()
        return 0
    if not (has_command or args.hold or args.release):
        parser.print_usage()
        print('Nothing to send. Pick animations with --left/--right/--both, or --off, --idle, --hold, --release; '
              '--list shows the choices.')
        return 2

    try:
        eyes = Eyes(backend='fake' if args.fake else 'server')
    except RuntimeError as e:
        print(e)
        return 1

    rc = 0
    try:
        if args.release:
            if eyes.state().sentry_active is None:
                print('No eye sentry on this robot ({} is not in its params), nothing to resume.'.format(SENTRY_NAME))
            elif eyes.resume_sentry(timeout=3.0):
                print('{} resumed.'.format(SENTRY_NAME))
            else:
                print(resume_failed(eyes.state().lease_holder))
                rc = 1
        if rc == 0 and (has_command or args.hold):
            if args.hold:
                eyes.take_control()
            if args.idle:
                eyes.idle()
            if any(v is not None for v in cmd.values()):
                eyes.set(**cmd)
            s = eyes.state()
            print_state(s, eyes.backend.sent if args.fake else None)
            if s.source == 'dropped':
                rc = 3
            if args.hold:
                if s.sentry_active is None:
                    print('Holding (no eye sentry on this robot). Ctrl-C to exit.')
                else:
                    print('Holding, sentry paused. Ctrl-C to release.')
                while True:
                    time.sleep(0.5)
    except (ValueError, RuntimeError) as e:
        print(e)
        rc = 1
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if not eyes.close():
                print(resume_failed())
                rc = 1
        except RuntimeError as e:  # no server left to resume anything on
            print(e)
            rc = 1
    return rc


if __name__ == '__main__':
    sys.exit(main())
