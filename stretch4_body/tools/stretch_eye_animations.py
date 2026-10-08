#!/usr/bin/env python3
from __future__ import print_function
import os
import sys
import json
import time
import argparse
import stretch4_body.core.hello_utils as hu
from stretch4_body.eyes import Eyes, Library, Sequence, ANIMATIONS, NAMED_COLORS
from stretch4_body.eyes.backends import SENTRY_NAME

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
parser.add_argument("--play", metavar="NAME_OR_FILE", help="Play a look from the library, or a look file, until it ends; Ctrl-C stops it and puts back the eyes from before")
parser.add_argument("--loop", help="With --play, repeat until Ctrl-C", action="store_true")
parser.add_argument("--looks", help="List the looks library and where each look comes from", action="store_true")
parser.add_argument("--export", metavar="NAME", help="Print a look from the library as JSON")
parser.add_argument("--import", dest="import_file", metavar="FILE", help="Check a look file and save it to your looks")
parser.add_argument("--overwrite", help="With --import, replace your look of the same name", action="store_true")
parser.add_argument("--fake", help="No robot, print what would be sent", action="store_true")
args = parser.parse_args()
if not args.export:  # --export prints JSON only, so it can be redirected to a file
    hu.print_stretch_re_use()

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


def print_looks(library):
    print('Looks (first match by name wins):')
    for source, d in library.dirs():
        print('  {:<8} {}'.format(source, d))
    for e in library.list():
        print('  {:<16} {:<8} {:>3} steps {:>6.1f} s{:<6} {}{}'.format(
            e['name'], e['source'], e['steps'], e['duration'], ' loop' if e['loop'] else '', e['title'],
            ' (hides a look of the same name further down)' if e['shadowed'] else ''))
    for e in library.errors():
        print('Skipped {}'.format(e['error']))


def library_command(library):
    """--import, --looks and --export, which need no robot. Returns the exit code. Errors go
    to stderr, so a redirected --export never captures one."""
    if args.import_file:
        try:
            seq = library.import_file(args.import_file, overwrite=args.overwrite)
        except FileExistsError as e:
            print('{} already exists. Use --overwrite to replace it.'.format(e.filename), file=sys.stderr)
            return 1
        except (OSError, ValueError) as e:
            print(e, file=sys.stderr)
            return 1
        print('Imported {} to {}'.format(seq.name, os.path.join(library.user_dir, seq.name + '.json')))
    if args.looks:
        print_looks(library)
    if args.export:
        try:
            print(json.dumps(library.export(args.export), indent=2))
        except KeyError as e:
            print(e.args[0], file=sys.stderr)
            return 1
    return 0


def load_look(library, name_or_file):
    if name_or_file.endswith('.json') or os.path.sep in name_or_file:
        return Sequence.load(name_or_file)
    return library.get(name_or_file)


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


def print_state(s, fake_sent=None, played=False):
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
    if s.sentry_active and not played:
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
    if args.play and (has_command or args.hold):
        print('--play sends its own steps and holds the eyes while it plays; leave out the other eye options.')
        return 2
    if (args.loop and not args.play) or (args.overwrite and not args.import_file):
        print('--loop goes with --play and --overwrite with --import.')
        return 2
    library = Library()
    if args.import_file or args.looks or args.export:
        rc = library_command(library)
        if rc or not args.play:
            return rc
    seq = None
    if args.play:
        try:
            seq = load_look(library, args.play)
        except KeyError as e:
            print(e.args[0])
            return 1
        except (OSError, ValueError) as e:
            print(e)
            return 1
    if not (has_command or args.hold or args.release or seq):
        parser.print_usage()
        print('Nothing to send. Pick animations with --left/--right/--both, or --off, --idle, --hold, --release, '
              '--play; --list and --looks show the choices.')
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
        if rc == 0 and seq:
            loop = True if args.loop else None
            eyes.play(seq, loop=loop)
            # Not eyes.playing: the play thread clears it if the first step already failed
            looping = seq.loop if loop is None else loop
            print('Playing {} ({} steps, {:.1f} s{}). Ctrl-C to stop and put back the eyes from before.'.format(
                seq.name, len(seq.steps), seq.duration, ', looping' if looping else ''), flush=True)
            while not eyes.wait(0.5):
                pass
            s = eyes.state()
            print_state(s, eyes.backend.sent if args.fake else None, played=True)
            if s.source == 'dropped':
                rc = 3
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
        if seq:
            print('Stopped.')
    finally:
        try:
            if not eyes.close():
                print(resume_failed())
                rc = 1
        except RuntimeError as e:  # no server left to resume anything on
            print(e)
            rc = 1
        if args.fake and seq:
            print('Fake robot: {} eye commands sent, {} {}'.format(
                len(eyes.backend.sent), SENTRY_NAME, 'active' if eyes.backend.sentry_active else 'paused'))
    return rc


if __name__ == '__main__':
    sys.exit(main())
