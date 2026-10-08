# Eye LEDs

Stretch 4 has two eye rings of 10 WS2812B pixels each, on the same LED chain as the battery light bar. The PIMU firmware renders the animations at 50 Hz; the host picks one preset animation per eye plus one color and one intensity shared by both eyes. `stretch4_body.eyes` wraps this in a small API that works through the Stretch Body Server, and has a fake backend for development without a robot.

## Python API

```python
from stretch4_body.eyes import Eyes

with Eyes() as eyes, eyes.take_control():          # take_control pauses the eye sentry
    for a in eyes.animations():
        print(a.id, a.name, a.description)
    eyes.set(left='look_left', right='look_left', color='#00a0ff', intensity=0.5)
    eyes.set_color('hot_pink')                       # animations and intensity are kept
    eyes.set(right='blink')                          # only the right eye changes
    print(eyes.state())                              # last commanded values, runstop, low SOC
    eyes.idle()                                      # firmware default idle glow
```

* **Animations** are given by name (`'idle_glow'`, any case) or firmware id. `eyes.animations()` lists the 14 presets with a description and whether color and intensity affect them.
* **Color** is `(r, g, b)`, `'#rrggbb'` or a name from `stretch4_body.eyes.NAMED_COLORS`.
* **Intensity**: an int is the raw byte 0-255, a float is a fraction 0.0-1.0. So `1` is raw 1 (nearly off) and `1.0` is full; `153` and `0.6` are the same brightness.
* **None keeps the last value** commanded through the same `Eyes` object. An eye that has not been commanded through this object is sent as NOP (id 0), which the firmware reads as "keep what this eye shows", so `Eyes().set_color('red')` recolors whatever animation is running instead of replacing it; `state().left` and `right` stay `None` until that eye is commanded. Color and intensity cannot be kept that way, the firmware applies them on every command, so until you set them the firmware defaults (40, 48, 60) at 255 are sent. `state().source` is `'assumed'` until the first command.
* Bad values raise `ValueError` listing the valid choices.
* `Eyes(backend='fake')` records commands in `eyes.backend.sent` instead of sending them. `Eyes(client=robot)` reuses a `RobotClient` you already started.
* `Eyes()` raises `RuntimeError` when `stretch_body_server` cannot be reached. A server that dies later is noticed on the failure branches only (see the lease section), which then raise `RuntimeError('Lost the connection to stretch_body_server')`; after that every call raises.

## Who else drives the eyes

| Source | Effect | What to do |
|---|---|---|
| `sentry_eye_animations` | Picks a new animation every 0.5-5 s while active (default on) | `eyes.take_control()` pauses it, `release()` or leaving the `with` block restores it |
| `stretch_body_server` restart | The sentry comes back active while `Eyes` still reports `in_control` | `release()` and `take_control()` again; for the CLI, rerun `--hold` |
| Runstop | Firmware shows all 20 pixels blinking with the runstop LED | Nothing; clears with the runstop. `state().runstop_active` |
| Battery at or below 25% / 12% | Firmware shows half rings in yellow / red | Nothing; `state().low_soc_override` |
| PIMU reset | Firmware returns to idle glow, (40, 48, 60) | Send the command again |

There is no readback from the firmware, so `state()` reports what was commanded, not what the rings show.

## The server lease

Eye commands are sent with `ignore_control_lock=True`, so they do not take or need the control lock, and at server priority -1, below the default 0. The server gives its command lease (`LEASE_TIMEOUT`, 1.1 s) to whoever pushes while it is free and to any push of higher priority than the holder's, so an eye client never holds the lease against a motion client: `stretch_robot_stow`, the ROS driver or a script pushing at the default priority takes it at once, Eyes Studio dragging a wheel at 20 Hz included.

The other direction still applies: the server only accepts commands from the client holding its lease, so while another client is streaming commands, or a routine is running, eye pushes are dropped without a reply. `Eyes.set()` does not wait to find out. It reads the latest status the server published and returns `source == 'dropped'`, with the holder in `state().lease_holder`, when that status names a running routine or another client with a live lease. Two things make that a strong hint rather than a receipt. The server only clears an expired lease when the next command arrives and keeps publishing the old holder, so on the robot `Eyes` compares the published `lease_expiry` with its own monotonic clock and treats a holder whose lease has run out as free; over tcp (an `Eyes` on another machine) the clocks differ, so a quiet holder is reported as dropped until a command replaces it. And the status predates the push, so a client that starts pushing in the same cycle slips through. The values are cached either way, so the next accepted command carries them.

A server that has died looks the same as a lease problem from the outside: it stops publishing and pushes vanish. So before a command is reported dropped, and when a sentry pause or resume is not confirmed within its wait, `Eyes` pings the server and raises `RuntimeError('Lost the connection to stretch_body_server')` if it is gone, instead of blaming the lease. The happy path never waits on that ping.

`release()` and `close()` go through the same lease when they resume the sentry. If the server rejects the resume for the whole wait (1 s for `release()`, 3 s for `close()`), they return `False`, log a warning, and `sentry_eye_animations` stays paused; leaving a `with eyes.take_control()` block never raises for this. `eyes.resume_sentry()` or `stretch_eye_animations --release` fixes it once the robot is idle. A pause that went out but was interrupted before the server confirmed it is still undone by `close()`.

## PIMU protocol

Eye animations, color and intensity all need PIMU protocol p13 (hello-pimu2 v0.1.9p13 or newer; `circle_cw` and `circle_ccw` need v0.1.10p13, v0.1.9p13 ignores them). A released p12 PIMU (up to v0.1.7p12) has no eye RPC at all: the board ignores every push, `stretch_body_server` logs `Error RPC_REPLY_SET_EYE_ANIMATION` for each one, nothing changes on the rings, and `sentry_eye_animations` disables itself below p13. The server does not publish the PIMU protocol, so `Eyes` cannot tell: `capabilities()['protocol_version']` is `None` through the server and `capabilities()['requires_protocol']` is `'p13'`. `stretch_system_check` reports the board's version.

## Command line

```bash
stretch_eye_animations --list
stretch_eye_animations --both happy --color pink --intensity 0.6
stretch_eye_animations --color red                                   # recolor, animations unchanged
stretch_eye_animations --left look_left --right look_right --hold    # Ctrl-C releases the sentry
stretch_eye_animations --idle
stretch_eye_animations --release                                     # resume the sentry after a failed release
stretch_eye_animations --fake --both alert --color red              # no robot, prints the payload
```

`--intensity` follows the API rule: a whole number is raw 0-255, a decimal is a fraction, so `--intensity 1` is raw 1 (nearly off) and `--intensity 1.0` is full. Exit codes: 2 when there is nothing to send (the usage is printed), 3 when the server dropped the command (a notice names the lease holder), 1 when the sentry could not be resumed on exit or the server could not be reached, 0 otherwise.

`stretch_eyes_studio` opens a browser UI built on the same API.
