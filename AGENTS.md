# stretch4_body

Low-level Python API for Stretch 4 hardware. **Public repo.** A `RobotServer` daemon owns the
hardware at 100 Hz; `RobotClient` talks to it over ZeroMQ. Stretch 3 and earlier live in the
separate `stretch_body` repo.

<!-- hello-robot-core v2 -->
## Hello Robot core rules

1. **This machine may be a robot.** If `HELLO_FLEET_ID` is set, a command you run can physically move a 60 kg machine with a telescoping arm and a powered lift, in a room that may contain people. Also remember that env-var presence isn't proof of hardware.

2. You do not run motion, homing, stowing, jogging, teleop, docking, or calibration, or any command that moves the robot; unless explicitly stated. You write the exact command in your reply and hand it to a human, with the preconditions they must check. Reading code is always allowed. Never run such a command merely to confirm it works.

3. **You may trigger a runstop. You may never clear one.** You can stop the robot; you cannot start it.

4. **Never write microcontroller or servo flash/EEPROM**, and never run `sudo`, touch udev rules, systemd units, or another machine over SSH. Bricked hardware needs physical recovery.

5. **Some state is not in git and `git checkout` will not revert it**: per-robot YAML under `$HELLO_FLEET_PATH/$HELLO_FLEET_ID/`, `/etc/hello-robot/`, udev rules, user systemd units. If you change any of it, say so explicitly in your summary.

6. **Confirm you are in the right clone and on the right branch before your first edit.** Run `git remote get-url origin` to confirm the repo, and `git branch --show-current` to confirm the branch. Several repos have near-identical siblings on disk, and every repo has several branches.

7. **Confidentiality.** Much of the Stretch 4 line is public on GitHub. Never put customer or fleet serial numbers, robot IDs, per-robot calibration values, internal hostnames or IPs, credentials, or production procedures into a public repo, a commit message, a PR body, or an issue, including as pasted command output.

8. **Honesty about hardware.** You cannot observe a physical outcome. Label every claim as one of three things: *read it*, meaning you traced the code; *ran it*, meaning you are pasting the literal output and never a paraphrase or a reconstruction; or *not verified*, meaning you say so and stop rather than guessing.

9. Never write "tested on hardware" or "verified on the robot" in a commit, PR, or comment. Only the human who watched it may write that.

10. Never invent a joint limit, gear ratio, encoder count, or parameter default; cite the file you read it from or say you don't know. Never delete or skip a failing test to make a run green. If a guardrail blocks you, report it rather than working around it.

11. Never commit and push work you have done. All agentic changes remain local and will be reviewed by the human. Generate a commit message, a PR description, and the commands, so the human can do it. Only commit and push when the user says to.

12. Branch names always start with `feature/`, `bugfix/`, `fix/`, `refactor/`, `docs/`, `test/` depending on what is being done.

13. **Code changes.** Make the change only. Do not add comments narrating why you changed something; that belongs in the chat and not in the code file. Leave existing comments, banners, and links alone. Do not run a formatter unless the repo configures one.

<!-- /hello-robot-core v2 -->

## Repo guardrails

1. **Assume no module under `stretch4_body/tools/` or `tools/factory/` has an `if __name__` guard.** Fewer than half do, and the unguarded ones build an `argparse` parser and act at module scope, so *importing one runs it*. That includes `stretch_arm_home`, `stretch_lift_home`, `stretch_robot_stow`, `stretch_gripper_home`, `stretch_wrist_{yaw,pitch,roll}_home`, and every `*_jog`. Never put `stretch4_body.tools` inside a `python -c`, a notebook, or a test. Do not treat a guard you happen to find as permission; the next file will not have one.

2. **The dangerous thing is a name, not a path.** `pyproject.toml` installs 84 console scripts, so `stretch_arm_home` is a valid bare command with no `python` and no `.py`. Treat these stems as hardware however they are spelled: `*_home`, `*_jog`, `*_teleop`, `*_sweep`, `*_calibration`, `*_flash`, `pose_play`, and anything `REx_*`. Core rule 2 applies to every one of them.

3. **`-d`/`--direct`, and `from stretch4_body.robot.robot import Robot`, bypass the server's sentries, safe-motion layer, and self-collision checking.** The `REx_*` factory tools bypass them by construction. Run `stretch_body_server --status` to check whether a server is already running. Only propose the direct path when no server is running. If a server is running but a tool is not working, report the exact information rather than routing around the server.

4. **Safe to run unprompted**, being read-only with no motion and no persistent writes: `stretch_params`, `stretch_battery_check`, `stretch_system_check`, `stretch_body_server --status`, and `--help` on any tool. Note `stretch_about` prints this robot's serial number, so do not paste its output anywhere public. Everything else in `tools/` is a hand-off to a human.

5. **Adding a CLI tool takes three edits**: the module in `tools/` (or `tools/factory/`), a wrapper in `stretch4_body/_cli_wrapper.py`, and the entry in `[project.scripts]`. Forgetting the second is the usual mistake.

6. Never edit `stretch4_body/robot/robot_params_SE4.py` to change one robot's behavior. That file is the model-wide nominal. Use `stretch_change_param` or `stretch_user_params.yaml`.

7. **`pull_status()` does not return a status dict.** It returns a bool meaning "a message got applied", and it mutates `self.status` in place. Reading its return value as data silently gives you `True`. Call it at the top of each control cycle and then read `robot.status`, which is deliberately not a deep copy so subsystem entries stay live.

8. `docs/primer_robot_params.md` has a "Summary for AI Agents" section; read it before touching params. `build_backend.py` and `meson.build` are the non-standard build.
