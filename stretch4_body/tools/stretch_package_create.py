#!/usr/bin/env python3
"""Scaffold a new Stretch 4 project with AI-agent guardrails already in place.

Coding agents read AGENTS.md from the project they are working in, not from the
libraries that project imports. A guardrail file living in stretch4_body
therefore never reaches someone writing routines in their own directory. This
tool puts one there.
"""

import argparse
import os
import stat
import sys
import time

AGENTS_MD = r'''# {{PROJECT}}

Robot routines for Stretch 4, built on the `stretch4_body` Python API.

<!-- hello-robot-core v2 -->
## Hello Robot core rules

These rules exist because an AI coding agent running in an auto-approve mode can
execute a shell command before a human sees it, and in this project a shell
command can move a 60 kg machine with a telescoping arm and a powered lift, in a
room that may contain people.

1. **This machine may be a robot.** Treat it as one if `HELLO_FLEET_ID` is set,
   or if a `stretch_*` command resolves on `PATH`. Neither is proof that
   hardware is attached, and the absence of both is not proof that it is not.
   If you cannot tell, assume a robot and hand off.

2. **You do not run motion.** No homing, stowing, jogging, teleop, docking, or
   calibration, and no command that commands a joint. That includes running a
   routine in this project that calls `move_to`, `move_by`, `home`, `stow`, or
   `push_command`. Write the exact command in your reply and hand it to a human,
   along with the preconditions they should check. Reading code is always
   allowed, including a file whose name matches one of the hardware shapes
   listed under Commands. Never run a motion command merely to confirm that it
   works.

3. **You may trigger a runstop. You may never clear one.** You can stop the
   robot; you cannot start it.

4. **Never write microcontroller or servo flash/EEPROM**, never run `sudo`, and
   never touch udev rules, systemd units, or another machine over SSH. Bricked
   hardware needs physical recovery.

5. **Some state is not in git and `git checkout` will not revert it**: per-robot
   YAML under `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/`, `/etc/hello-robot/`, udev
   rules, and user systemd units. If you change any of it, say so explicitly in
   your summary.

6. **Honesty about hardware.** You cannot observe a physical outcome. Label
   every claim as one of three things: *read it*, meaning you traced the code;
   *ran it*, meaning you are pasting the literal output and never a paraphrase
   or a reconstruction; or *not verified*, meaning you say so and stop rather
   than guessing. Never write "tested on hardware" or "verified on the robot" —
   only the human who watched it happen may write that.

7. **Never invent a joint limit, gear ratio, encoder count, or parameter
   default.** Cite the file you read it from, or say you do not know. Prefer
   reading limits from the robot's own params at runtime over hardcoding them.

8. **If a guardrail blocks you, report it.** Do not route around it, and never
   delete or skip a failing test to make a run green.
<!-- /hello-robot-core v2 -->

## Writing routines in this project

- `RobotClient` talks to the `RobotServer` daemon over ZeroMQ. The server owns
  the hardware at 100 Hz and applies the sentries, the safe-motion layer, and
  self-collision checking.
- **`-d`/`--direct`, and `from stretch4_body.robot.robot import Robot`, bypass
  all of that.** Never propose either as a workaround for a failing tool. Check
  whether a server is up with `stretch_body_server --status`. If a server is
  running and a tool still fails, report the exact error instead of routing
  around the server.
- **Never import anything from `stretch4_body.tools`.** Most of those modules
  build an `argparse` parser and act at module scope, so *importing one runs
  it*. Never put `stretch4_body.tools` inside a `python -c`, a notebook, or a
  test. Do not treat a module you find with an `if __name__` guard as
  permission; the next one will not have it.
- **`pull_status()` does not return a status dict.** It returns a bool meaning
  "a status message got applied", and it mutates `robot.status` in place. Call
  it at the top of each control cycle and then read `robot.status`.
- A joint that is not homed will refuse to move: the call returns `False` and
  prints a warning. That is not a bug to work around.
- Read joint limits from params rather than hardcoding them, for example
  `robot.arm.params['range_m']`, or iterate `robot.end_of_arm.joints` and read
  each joint's `params['range_deg']`.

## Commands

Safe to run unprompted, being read-only with no motion and no persistent writes:

    stretch_params
    stretch_battery_check
    stretch_system_check
    stretch_body_server --status
    <any stretch tool> --help

Everything else is a hand-off to a human. Treat these name shapes as hardware
however they are spelled, with or without a `python` prefix or a `.py` suffix:

    *_home   *_jog   *_teleop   *_stow   *_sweep
    *_calibration   *_flash   pose_play   pose_record   REx_*

What is gated is **executing** one of those names: bare, via `python -m`, via a
shebang, or inside a `bash -c`. **Mentioning one is not executing it.** Reading
any path is always allowed no matter what it is called, so `cat`, `grep`, `sed`,
`head` and opening a file in an editor stay available. The same split applies to
this project's own code: reading a routine that calls `move_to` is fine, running
it is not.

**Refusing a safe command is a failure too**, not a cautious default. Declining
to read a file, or to run something on the list above, costs a human a support
round-trip and teaches them to stop asking. Refuse motion; do not refuse to
look.

Note that `stretch_about` prints this robot's serial number, so do not paste its
output into a public repo, an issue, or a commit message.
'''

CLAUDE_MD = r'''@AGENTS.md
'''

CLAUDE_SETTINGS = r'''{
  "permissions": {
    "deny": [
      "Bash(sudo:*)",
      "Bash(stretch_robot_home:*)",
      "Bash(stretch_robot_stow:*)",
      "Bash(stretch_arm_home:*)",
      "Bash(stretch_lift_home:*)",
      "Bash(stretch_gripper_home:*)",
      "Bash(stretch_wrist_yaw_home:*)",
      "Bash(stretch_wrist_pitch_home:*)",
      "Bash(stretch_wrist_roll_home:*)",
      "Bash(stretch_dex_wrist_home:*)",
      "Bash(stretch_arm_jog:*)",
      "Bash(stretch_lift_jog:*)",
      "Bash(stretch_gripper_jog:*)",
      "Bash(stretch_dex_wrist_jog:*)",
      "Bash(stretch_omni_base_jog:*)",
      "Bash(stretch_power_periph_jog:*)",
      "Bash(stretch_gamepad_teleop:*)",
      "Bash(stretch_keyboard_omni_teleop:*)",
      "Bash(stretch_puppet_teleop:*)",
      "Bash(stretch_pose_play:*)",
      "Bash(stretch_pose_record:*)",
      "Bash(stretch_pose_edit:*)",
      "Bash(stretch_feetech_reboot:*)",
      "Bash(stretch_configure_tool:*)",
      "Bash(stretch_change_param:*)"
    ]
  },
  "hooks": {
    "PreToolUse": [
      {
        "matcher": "Bash",
        "hooks": [
          {
            "type": "command",
            "command": "$CLAUDE_PROJECT_DIR/.claude/hooks/guard_hardware.py"
          }
        ]
      }
    ]
  }
}
'''

ANTIGRAVITY_SETTINGS = r'''{
  "agentGuardrails": {
    "instructionsFile": "AGENTS.md",
    "denyCommands": [
      "sudo",
      "stretch_robot_home",
      "stretch_robot_stow",
      "stretch_arm_home",
      "stretch_lift_home",
      "stretch_gripper_home",
      "stretch_wrist_yaw_home",
      "stretch_wrist_pitch_home",
      "stretch_wrist_roll_home",
      "stretch_dex_wrist_home",
      "stretch_arm_jog",
      "stretch_lift_jog",
      "stretch_gripper_jog",
      "stretch_dex_wrist_jog",
      "stretch_omni_base_jog",
      "stretch_power_periph_jog",
      "stretch_gamepad_teleop",
      "stretch_keyboard_omni_teleop",
      "stretch_puppet_teleop",
      "stretch_pose_play",
      "stretch_pose_record",
      "stretch_pose_edit",
      "stretch_feetech_reboot",
      "stretch_configure_tool",
      "stretch_change_param"
    ]
  }
}
'''

GUARD_HOOK = r'''#!/usr/bin/env python3
"""PreToolUse hook: refuse shell commands that would move the robot.

AGENTS.md is advice, which an agent in an auto-approve mode can talk itself
past. This is enforcement, which it cannot. The two are meant to be used
together.

Reads the hook payload on stdin and blocks by exiting 2 with a reason on
stderr, which fails closed. Run with --self-test to check the rules.
"""

import json
import os
import re
import sys

ALLOW_ALWAYS = {
    "stretch_params",
    "stretch_battery_check",
    "stretch_system_check",
}

ALLOW_WITH_ARGS = {"stretch_body_server": [{"--status"}]}

HELP_FLAGS = {"-h", "--help"}

DENY_STEMS = (
    "_home", "_jog", "_teleop", "_stow", "_sweep", "_calibration",
    "_calibrate", "_flash", "_reboot",
    "pose_play", "pose_record", "pose_edit",
)

DENY_PREFIXES = ("REx_", "re1_")

MOTION_CALLS = (
    "move_to", "move_by", ".home(", ".stow(", "push_command", "set_velocity",
)

HANDOFF = (
    "Refused: this would command the robot. Per AGENTS.md, hand the exact "
    "command to a human with the preconditions to check, rather than running "
    "it. Reading the code is always allowed."
)


def classify_command(cmd, args):
    argset = set(args)
    if argset & HELP_FLAGS:
        return "allow", "help text only"
    if cmd in ALLOW_ALWAYS:
        return "allow", "on the read-only allowlist"
    if cmd in ALLOW_WITH_ARGS:
        for permitted in ALLOW_WITH_ARGS[cmd]:
            if argset == permitted:
                return "allow", "allowlisted argument form"
        return "deny", "%s is allowlisted only with %s" % (
            cmd, " ".join(sorted(ALLOW_WITH_ARGS[cmd][0])))
    if cmd.startswith(DENY_PREFIXES):
        return "deny", "%s is a factory tool and bypasses the server" % cmd
    for stem in DENY_STEMS:
        if stem in cmd:
            return "deny", "%s matches the hardware name shape %r" % (cmd, stem)
    return "allow", "not a gated command"


def executed_script(toks):
    """The .py file this command segment would run, if any.

    Reading a routine is always allowed, so a .py path only counts when it is
    actually being executed: as the argument of an interpreter, or as the
    command itself via its shebang.
    """
    if not toks:
        return None
    first = os.path.basename(toks[0])
    if first.startswith("python"):
        for tok in toks[1:]:
            if tok.startswith("-"):
                continue
            return tok if tok.endswith(".py") else None
        return None
    if first.endswith(".py"):
        return toks[0]
    return None


def classify(line):
    """Classify a whole shell command line."""
    if "stretch4_body.tools" in line:
        return "deny", "importing a tools module executes it at module scope"
    if re.search(r"from\s+stretch4_body\.robot\.robot\s+import"
                 r"|stretch4_body\.robot\.robot\b(?!_)", line):
        return "deny", "the direct Robot class bypasses the server sentries"
    if ("stretch" in line or "REx_" in line) and \
            re.search(r"(?<!\w)(-d|--direct)(?!\w)", line):
        return "deny", "--direct bypasses safe-motion and collision checking"

    allow_reason = "no gated command found"

    for segment in re.split(r"[|;&]+", line):
        toks = [t for t in segment.split() if t]
        if not toks:
            continue

        target = executed_script(toks)
        if target and os.path.isfile(target):
            try:
                body = open(target).read()
            except OSError:
                body = ""
            hits = [c for c in MOTION_CALLS if c in body]
            if hits:
                return "deny", "%s calls %s" % (target, ", ".join(hits[:3]))

        for i, tok in enumerate(toks):
            base = os.path.basename(tok)
            if base.startswith(DENY_PREFIXES) \
                    or any(s in base for s in DENY_STEMS) \
                    or base in ALLOW_ALWAYS or base in ALLOW_WITH_ARGS:
                decision, reason = classify_command(base, toks[i + 1:])
                if decision == "deny":
                    return decision, reason
                allow_reason = reason
                break
    return "allow", allow_reason


CASES = [
    ("stretch_arm_home", "deny"),
    ("stretch_robot_stow", "deny"),
    ("echo hi && stretch_robot_home", "deny"),
    ("REx_stepper_calibration_run", "deny"),
    ("stretch_gamepad_teleop", "deny"),
    ("python3 -c 'import stretch4_body.tools.stretch_arm_home'", "deny"),
    ("from stretch4_body.robot.robot import Robot", "deny"),
    ("stretch_robot_home -d", "deny"),
    ("stretch_params", "allow"),
    ("stretch_battery_check", "allow"),
    ("stretch_system_check", "allow"),
    ("stretch_body_server --status", "allow"),
    ("stretch_arm_home --help", "allow"),
    ("python3 routines/example_random_pose.py", "deny"),
    ("python3 -u routines/example_random_pose.py", "deny"),
    ("./routines/example_random_pose.py", "deny"),
    ("cat routines/example_random_pose.py", "allow"),
    ("grep -n move_to routines/example_random_pose.py", "allow"),
    ("sed -n 1,20p routines/example_status_log.py", "allow"),
    ("python3 routines/example_status_log.py", "allow"),
    ("git status", "allow"),
    ("ls -d /tmp", "allow"),
    ("pytest -q", "allow"),
]


def self_test():
    bad = 0
    for line, want in CASES:
        got, why = classify(line)
        ok = got == want
        bad += not ok
        print("%s  %-9s %-52s %s" % ("ok  " if ok else "FAIL", got,
                                     line[:50], why))
    print("\n%d/%d passed" % (len(CASES) - bad, len(CASES)))
    return 1 if bad else 0


def main():
    if "--self-test" in sys.argv:
        return self_test()

    try:
        payload = json.load(sys.stdin)
    except (ValueError, OSError):
        return 0

    command = (payload.get("tool_input") or {}).get("command", "")
    if not command:
        return 0

    decision, reason = classify(command)
    if decision == "deny":
        sys.stderr.write("%s\n\nReason: %s\n" % (HANDOFF, reason))
        return 2
    return 0


if __name__ == "__main__":
    sys.exit(main())
'''

EXAMPLE_STATUS_LOG = r'''#!/usr/bin/env python3
"""Read-only example: log joint positions at 15 Hz.

No motion. Safe to run whenever a server is up.

Note the control-loop shape: pull_status() returns a bool saying whether a
status message got applied, not the status itself. Read robot.status after it.
"""

import argparse
import time

from stretch4_body.robot.robot_client import RobotClient


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--rate", type=float, default=15.0)
    args = ap.parse_args()

    with RobotClient() as robot:
        deadline = time.time() + args.seconds
        while time.time() < deadline:
            if robot.pull_status():
                print("arm=%.4f m  lift=%.4f m"
                      % (robot.arm.status["pos"], robot.lift.status["pos"]))
            time.sleep(1.0 / args.rate)


if __name__ == "__main__":
    main()
'''

EXAMPLE_RANDOM_POSE = r'''#!/usr/bin/env python3
"""Example: move to a random joint configuration.

Limits come from the robot's own params, never from constants in this file, so
the routine stays correct across tools and per-robot calibration.

Dry run is the default on purpose. --execute is what makes this move, and it is
a human's call to pass it, not an agent's.
"""

import argparse
import math
import random

from stretch4_body.robot.robot_client import RobotClient


def sample_pose(robot, rng):
    """Pick one random configuration inside the params-reported limits."""
    pose = {}

    for name in ("arm", "lift"):
        joint = getattr(robot, name)
        lo, hi = joint.params["range_m"]
        pose[name] = ("m", rng.uniform(lo, hi))

    for name in robot.end_of_arm.joints:
        joint = getattr(robot.end_of_arm, name, None)
        if joint is None or "range_deg" not in getattr(joint, "params", {}):
            continue
        lo, hi = joint.params["range_deg"]
        pose[name] = ("rad", math.radians(rng.uniform(lo, hi)))

    return pose


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--execute", action="store_true",
                    help="actually move the robot; joints will move")
    args = ap.parse_args()

    rng = random.Random(args.seed)

    with RobotClient() as robot:
        robot.pull_status()
        pose = sample_pose(robot, rng)

        for name, (unit, value) in sorted(pose.items()):
            print("%-14s %8.4f %s" % (name, value, unit))

        if not args.execute:
            print("\nDry run. Nothing was commanded. Pass --execute to move.")
            return

        if not robot.is_homed():
            print("Robot is not homed; joints will refuse to move.")
            return

        robot.arm.move_to(pose["arm"][1])
        robot.lift.move_to(pose["lift"][1])
        for name, (unit, value) in pose.items():
            if name in ("arm", "lift"):
                continue
            robot.end_of_arm.move_to(name, value)
        robot.push_command()


if __name__ == "__main__":
    main()
'''

README_MD = r'''# {{PROJECT}}

Stretch 4 routines, scaffolded by `stretch_package_create`.

## Layout

    AGENTS.md                     guardrails, read by most coding agents
    CLAUDE.md                     includes AGENTS.md for Claude Code
    .claude/settings.json         deny rules plus the PreToolUse hook
    .claude/hooks/guard_hardware.py   the enforcement, with a --self-test
    .antigravity/settings.json    same intent for Antigravity
    routines/                     your routines; two examples to start from

## Why both a prose file and a hook

`AGENTS.md` is advice. It travels across agents, because Claude Code, Cursor,
Copilot, Antigravity and Codex all read it, but an agent running in an
auto-approve mode can talk itself past advice.

`.claude/hooks/guard_hardware.py` is enforcement. It cannot be talked past, but
it only binds the agent that loads it.

Use both. Check the hook works before you rely on it:

    .claude/hooks/guard_hardware.py --self-test

## Running the examples

    python3 routines/example_status_log.py          # read-only, no motion
    python3 routines/example_random_pose.py         # dry run, prints a target

`example_random_pose.py` moves only with `--execute`. Clear the area, know where
the runstop is, and pass it yourself.

## First steps

1. Fill in `LICENSE`.
2. Put your own routines in `routines/`.
3. Add project-specific rules to the end of `AGENTS.md`. Keep the
   `hello-robot-core` block as it is so it can be updated later.
'''

GITIGNORE = r'''__pycache__/
*.py[cod]
.venv/
venv/
*.egg-info/
.pytest_cache/
*.log
'''

LICENSE_TXT = r'''MIT License

Copyright (c) {{YEAR}} {{AUTHOR}}

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
'''

TEMPLATES = [
    ("AGENTS.md", AGENTS_MD, False),
    ("CLAUDE.md", CLAUDE_MD, False),
    ("README.md", README_MD, False),
    ("LICENSE", LICENSE_TXT, False),
    (".gitignore", GITIGNORE, False),
    (".claude/settings.json", CLAUDE_SETTINGS, False),
    (".claude/hooks/guard_hardware.py", GUARD_HOOK, True),
    (".antigravity/settings.json", ANTIGRAVITY_SETTINGS, False),
    ("routines/example_status_log.py", EXAMPLE_STATUS_LOG, True),
    ("routines/example_random_pose.py", EXAMPLE_RANDOM_POSE, True),
]


def render(text, project, author, year):
    return (text.replace("{{PROJECT}}", project)
                .replace("{{AUTHOR}}", author)
                .replace("{{YEAR}}", year))


def main():
    ap = argparse.ArgumentParser(
        description="Create a Stretch 4 project with AI-agent guardrails in "
                    "place")
    ap.add_argument("name", help="project directory name")
    ap.add_argument("-C", "--dir", default=".",
                    help="parent directory to create the project in")
    ap.add_argument("--author", default="TODO: your name or organization",
                    help="copyright holder written into LICENSE")
    ap.add_argument("--force", action="store_true",
                    help="overwrite files that already exist")
    ap.add_argument("--list", action="store_true",
                    help="list what would be created and exit")
    args = ap.parse_args()

    if args.list:
        for rel, _, ex in TEMPLATES:
            print("%s%s" % (rel, "  (executable)" if ex else ""))
        return 0

    root = os.path.join(os.path.abspath(args.dir), args.name)
    year = time.strftime("%Y")

    existing = [rel for rel, _, _ in TEMPLATES
                if os.path.exists(os.path.join(root, rel))]
    if existing and not args.force:
        sys.stderr.write(
            "refusing to overwrite in %s:\n  %s\nre-run with --force\n"
            % (root, "\n  ".join(existing)))
        return 1

    for rel, body, executable in TEMPLATES:
        path = os.path.join(root, rel)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        with open(path, "w") as f:
            f.write(render(body, args.name, args.author, year))
        if executable:
            os.chmod(path, os.stat(path).st_mode | stat.S_IXUSR
                     | stat.S_IXGRP | stat.S_IXOTH)
        print("created %s" % os.path.relpath(path, os.path.dirname(root)))

    print("\nNext:")
    print("  cd %s" % root)
    print("  .claude/hooks/guard_hardware.py --self-test")
    print("  git init && git add . && git commit -m 'Initial scaffold'")
    print("\nFill in LICENSE, and read AGENTS.md before pointing an agent here.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
