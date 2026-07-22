#!/usr/bin/env python3
"""
stretch_audio_test.py

Diagnose and interactively test the robot's USB speakerphone (speaker + mic).

Checks performed:
  1. A USB audio device is enumerated by the kernel (ALSA sees it over USB).
  2. The speaker (sink) and microphone (source) show up in the Ubuntu audio
     manager (PipeWire / WirePlumber, the backend behind GNOME Sound Settings).
  3. The current user belongs to the 'audio' group (required to access the
     audio devices without sudo).

Unless --check-only is given, the tool then interactively plays a test tone
out the speaker and records/plays back a short clip from the microphone,
asking the user to confirm what they heard.
"""
import stretch4_body.core.hello_utils as hu
hu.print_stretch_re_use()

import os
import re
import sys
import grp
import pwd
import getpass
import argparse
import subprocess
import tempfile

import click

parser = argparse.ArgumentParser(description='Test the robot USB speakerphone (speaker + microphone)')
parser.add_argument('--check-only', help='Only run the automated checks, skip interactive speaker/mic tests', action='store_true')
args = parser.parse_args()


# ==============================================================================
# Output helpers
# ==============================================================================

def print_section(title):
    click.secho(f'\n---- {title} ----', fg='cyan', bold=True)

def print_result(passed, msg, indent=2):
    pad = ' ' * indent
    if passed:
        click.secho(f'{pad}[PASS] {msg}', fg='green')
    else:
        click.secho(f'{pad}[FAIL] {msg}', fg='red')

def print_warn(msg, indent=2):
    click.secho(f'{" " * indent}[WARN] {msg}', fg='yellow')

def print_info(msg, indent=4):
    click.secho(f'{" " * indent}{msg}', fg='white')


# ==============================================================================
# Checks
# ==============================================================================

def find_usb_audio_cards():
    """Parse /proc/asound/cards for ALSA cards driven by the USB-Audio kernel driver."""
    cards = []
    try:
        with open('/proc/asound/cards') as f:
            lines = f.read().splitlines()
    except OSError:
        return cards

    for i in range(0, len(lines) - 1, 2):
        header = lines[i]
        m = re.match(r'\s*(\d+)\s+\[(\S+)\s*\]:\s*(\S+)\s*-\s*(.*)', header)
        if not m:
            continue
        idx, card_id, driver, longname = m.groups()
        if driver != 'USB-Audio':
            continue
        detail = lines[i + 1].strip() if i + 1 < len(lines) else ''
        cards.append({'index': int(idx), 'id': card_id, 'name': longname.strip(), 'detail': detail})
    return cards


def check_usb_connection():
    print_section('USB Speaker Connection')
    cards = find_usb_audio_cards()
    if not cards:
        print_result(False, 'No USB audio device found (checked /proc/asound/cards)')
        print_info('Check that the speakerphone USB cable is plugged in.')
        return False, []

    for c in cards:
        print_result(True, f"card {c['index']} [{c['id']}]: {c['name']}")
        print_info(c['detail'])
    return True, cards


def get_pipewire_audio():
    """Return {'Devices': [...], 'Sinks': [...], 'Sources': [...]} from `wpctl status`, or None if unavailable."""
    try:
        out = subprocess.run(['wpctl', 'status'], capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return None

    sections = {'Devices': [], 'Sinks': [], 'Sources': []}
    current = None
    in_audio = False
    for line in out.splitlines():
        stripped = line.strip(' │')
        if stripped == 'Audio':
            in_audio = True
            current = None
            continue
        if stripped in ('Video', 'Settings'):
            in_audio = False
            continue
        if not in_audio:
            continue
        m = re.match(r'^[├└]?─?\s*(Devices|Sinks|Sources|Sink endpoints|Source endpoints|Streams):\s*$', stripped)
        if m:
            current = m.group(1)
            continue
        if current in sections:
            item_m = re.match(r'^(\*)?\s*(\d+)\.\s+(.+?)\s+\[.*\]\s*$', stripped)
            if item_m:
                sections[current].append({
                    'id': item_m.group(2),
                    'name': item_m.group(3).strip(),
                    'default': bool(item_m.group(1)),
                })
    return sections


def check_audio_manager(usb_cards):
    print_section('Ubuntu Audio Manager (PipeWire)')
    pw = get_pipewire_audio()
    if pw is None:
        print_warn('wpctl not available — cannot confirm devices in the audio manager')
        return None, None, None

    usb_names = [c['name'] for c in usb_cards]

    def find_match(entries):
        for e in entries:
            if any(n.lower() in e['name'].lower() or e['name'].lower() in n.lower() for n in usb_names):
                return e
        for e in entries:
            if 'built-in' not in e['name'].lower():
                return e
        return None

    sink   = find_match(pw['Sinks'])
    source = find_match(pw['Sources'])

    resolved = sink or source
    device = None
    if resolved:
        target_name = resolved['name']
        for e in pw['Devices']:
            if e['name'].lower() in target_name.lower() or target_name.lower() in e['name'].lower():
                device = e
                break
    if device is None:
        device = find_match(pw['Devices'])

    print_result(device is not None, f"Audio device: {device['name']}" if device else 'No matching USB audio device in PipeWire')
    print_result(sink is not None,   f"Speaker (sink): {sink['name']}" if sink else 'No matching speaker (sink) in PipeWire')
    print_result(source is not None, f"Microphone (source): {source['name']}" if source else 'No matching microphone (source) in PipeWire')

    if sink and not sink['default']:
        print_warn(f"{sink['name']} is not the default output — audio may play elsewhere")
    if source and not source['default']:
        print_warn(f"{source['name']} is not the default input — recordings may use another mic")
        if set_default_pipewire_node(source['id']):
            print_result(True, f"Set {source['name']} as the default input")
            source['default'] = True
        else:
            print_warn(f"Could not set {source['name']} as the default input")

    return device, sink, source


def set_default_pipewire_node(node_id):
    """Set a PipeWire node (by wpctl id) as the default sink/source."""
    try:
        r = subprocess.run(['wpctl', 'set-default', str(node_id)],
                            capture_output=True, text=True, timeout=5)
    except (FileNotFoundError, subprocess.TimeoutExpired):
        return False
    return r.returncode == 0


def check_audio_group():
    print_section('Audio Group Membership')
    user = getpass.getuser()

    try:
        audio_gid = grp.getgrnam('audio').gr_gid
        in_group_file = user in grp.getgrnam('audio').gr_mem or os.getgid() == audio_gid
    except KeyError:
        print_result(False, "'audio' group does not exist on this system")
        return False

    in_session = audio_gid in os.getgroups()

    print_result(in_group_file, f"User '{user}' is listed in the 'audio' group (/etc/group)")
    if not in_group_file:
        print_info(f'Run: sudo usermod -aG audio {user}')
        return False

    if not in_session:
        print_warn("Group membership not active in this session — log out and back in (or reboot)")
        return False

    print_result(True, 'Group membership is active in the current session')
    return True


def _pcm_device_paths(card_index, direction):
    """Return existing /dev/snd/pcmC<card_index>D*<direction> paths.

    direction: 'p' for playback (speaker), 'c' for capture (microphone).
    """
    snd_dir = '/dev/snd'
    if not os.path.isdir(snd_dir):
        return []
    pattern = re.compile(rf'^pcmC{card_index}D\d+{direction}$')
    return [os.path.join(snd_dir, name) for name in sorted(os.listdir(snd_dir)) if pattern.match(name)]


def _describe_pid(pid):
    """Best-effort (process name, username) for a PID; '?' if unreadable."""
    try:
        with open(f'/proc/{pid}/comm') as f:
            comm = f.read().strip()
    except OSError:
        comm = '?'
    try:
        user = pwd.getpwuid(os.stat(f'/proc/{pid}').st_uid).pw_name
    except (OSError, KeyError):
        user = '?'
    return comm, user


_EXPECTED_AUDIO_SERVER_PROCESSES = {'pipewire', 'pipewire-pulse', 'wireplumber', 'pulseaudio'}


def find_device_holders(card_index, direction):
    """Return [(pid, comm, user), ...] holding the card's speaker/mic device
    node open (excluding the system's own PipeWire/PulseAudio daemons), or
    None if 'fuser' isn't installed to check with.
    """
    holders = []
    seen_pids = set()
    for path in _pcm_device_paths(card_index, direction):
        try:
            r = subprocess.run(['fuser', path], capture_output=True, text=True, timeout=5)
        except FileNotFoundError:
            return None
        out = r.stdout.strip()
        if ':' in out:
            out = out.split(':', 1)[1]
        for tok in out.split():
            m = re.match(r'(\d+)', tok)
            if not m:
                continue
            pid = int(m.group(1))
            if pid == os.getpid() or pid in seen_pids:
                continue
            seen_pids.add(pid)
            comm, user = _describe_pid(pid)
            if comm in _EXPECTED_AUDIO_SERVER_PROCESSES:
                continue
            holders.append((pid, comm, user))
    return holders


def _format_holders(holders):
    return ', '.join(f'PID {pid} ({comm}, user {user})' for pid, comm, user in holders)


def check_device_availability(usb_cards):
    print_section('Device Availability')
    if not usb_cards:
        print_warn('No USB audio card to check')
        return {}, {}

    speaker_holders = {}
    mic_holders = {}
    for c in usb_cards:
        idx = c['index']
        pb = find_device_holders(idx, 'p')
        cap = find_device_holders(idx, 'c')

        if pb is None:
            print_warn(f"card {idx}: cannot check speaker availability ('fuser' not installed)")
        elif pb:
            print_result(False, f"card {idx} speaker: unavailable — held by {_format_holders(pb)}")
            speaker_holders[idx] = pb
        else:
            print_result(True, f'card {idx} speaker: available')

        if cap is None:
            print_warn(f"card {idx}: cannot check microphone availability ('fuser' not installed)")
        elif cap:
            print_result(False, f"card {idx} microphone: unavailable — held by {_format_holders(cap)}")
            mic_holders[idx] = cap
        else:
            print_result(True, f'card {idx} microphone: available')

    return speaker_holders, mic_holders


# ==============================================================================
# Interactive speaker / mic tests
# ==============================================================================

TEST_WAV = '/usr/share/sounds/alsa/Front_Center.wav'

def interactive_speaker_test(sink):
    print_section('Speaker Test')
    if not os.path.isfile(TEST_WAV):
        print_warn(f'Test file not found: {TEST_WAV} — skipping speaker test')
        return None

    target_args = ['--target', sink['name']] if sink else []
    click.echo('  Playing a test tone...')
    try:
        subprocess.run(['pw-play', *target_args, TEST_WAV], timeout=10)
    except FileNotFoundError:
        print_warn('pw-play not found — skipping speaker test')
        return None
    except subprocess.TimeoutExpired:
        print_warn('Playback timed out')

    heard = click.confirm('  Did you hear the test tone from the speaker?', default=True)
    print_result(heard, 'Speaker test')
    return heard


def interactive_mic_test(source):
    print_section('Microphone Test')
    try:
        subprocess.run(['pw-record', '--help'], capture_output=True, timeout=5)
    except FileNotFoundError:
        print_warn('pw-record not found — skipping microphone test')
        return None

    target_args = ['--target', source['name']] if source else []
    duration = 3

    click.prompt('  Press Enter, then speak into the microphone', default='', show_default=False)
    with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp:
        tmp_path = tmp.name

    try:
        click.echo(f'  Recording for {duration} seconds...')
        subprocess.run(['timeout', str(duration), 'pw-record', *target_args,
                         '--format', 's16', '--rate', '44100', '--channels', '1', tmp_path])

        if not os.path.isfile(tmp_path) or os.path.getsize(tmp_path) <= 44:
            print_warn('No audio was captured')
            return None

        click.echo('  Playing back your recording...')
        subprocess.run(['pw-play', tmp_path], timeout=duration + 5)

        heard = click.confirm('  Did you hear your voice played back?', default=True)
        print_result(heard, 'Microphone test')
        return heard
    except subprocess.TimeoutExpired:
        print_warn('Playback timed out')
        return None
    finally:
        try:
            os.remove(tmp_path)
        except OSError:
            pass


# ==============================================================================
# Main
# ==============================================================================

def main():
    results = {}

    results['USB Connection'], usb_cards = check_usb_connection()
    device, sink, source = check_audio_manager(usb_cards)
    results['Speaker in Audio Manager']     = None if sink is None and device is None else sink is not None
    results['Microphone in Audio Manager']  = None if source is None and device is None else source is not None
    results['Audio Group']                  = check_audio_group()

    speaker_holders, mic_holders = check_device_availability(usb_cards)
    results['Speaker Available']    = None if not usb_cards else not speaker_holders
    results['Microphone Available'] = None if not usb_cards else not mic_holders

    if not args.check_only:
        if any(speaker_holders.values()):
            print_section('Speaker Test')
            who = _format_holders([h for holders in speaker_holders.values() for h in holders])
            print_result(False, f'Speaker unavailable — held by {who}')
            results['Speaker Test'] = False
        else:
            results['Speaker Test'] = interactive_speaker_test(sink)

        if any(mic_holders.values()):
            print_section('Microphone Test')
            who = _format_holders([h for holders in mic_holders.values() for h in holders])
            print_result(False, f'Microphone unavailable — held by {who}')
            results['Microphone Test'] = False
        else:
            results['Microphone Test'] = interactive_mic_test(source)

    print_section('Summary')
    all_pass = True
    for name, passed in results.items():
        if passed is None:
            click.secho(f'  [SKIP] {name}', fg='yellow')
        else:
            print_result(passed, name)
            if not passed:
                all_pass = False

    click.echo()
    click.secho('All audio checks PASSED.' if all_pass else 'One or more audio checks FAILED.',
                fg='green' if all_pass else 'red', bold=True)
    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
