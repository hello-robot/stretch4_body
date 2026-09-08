# Agent Guardrails

AI coding agents are increasingly used to write robot routines, and several of
them run in modes where a shell command executes before a human sees it. On a
Stretch, a shell command can move a 60 kg machine with a telescoping arm and a
powered lift. This primer covers the guardrails that keep an agent from doing
that, and the `stretch_package_create` tool that installs them in a new project.

## The problem this solves

An agent reads its instruction file from **the project it is working in**, not
from the libraries that project imports. The `AGENTS.md` at the root of this
repo therefore protects work done *inside* `stretch4_body`, and does nothing at
all for someone writing routines in their own directory:

    ~/my_routines/            <- the agent reads AGENTS.md from here
      grasp_demo.py
      sweep_table.py          <- imports stretch4_body, but never reads its AGENTS.md

Nothing warns that agent that `stretch_robot_home` is a bare, valid command, or
that `--direct` bypasses the safe-motion layer. `stretch_package_create` puts
the guardrails where they are actually read.

## Quick start

    stretch_package_create my_routines
    cd my_routines
    .claude/hooks/guard_hardware.py --self-test

That produces:

| Path | Purpose |
| --- | --- |
| `AGENTS.md` | The guardrails. Read by Claude Code, Cursor, Copilot, Antigravity and Codex. |
| `CLAUDE.md` | One line, `@AGENTS.md`, so Claude Code picks the same file up. |
| `.claude/settings.json` | `permissions.deny` entries plus the `PreToolUse` hook. |
| `.claude/hooks/guard_hardware.py` | The enforcement. Refuses motion commands. Has a `--self-test`. |
| `.antigravity/settings.json` | The same intent expressed for Antigravity. |
| `routines/example_status_log.py` | Read-only control loop. Shows the `pull_status()` shape. |
| `routines/example_random_pose.py` | Random joint configuration. Dry run unless `--execute`. |
| `LICENSE`, `README.md`, `.gitignore` | Ordinary project scaffolding. `LICENSE` is yours to fill in. |

Pass `--list` to see the file list without writing anything, `-C` to choose the
parent directory, `--author` to fill in the `LICENSE`, and `--force` to
overwrite an existing scaffold.

## Two layers, and why you need both

**`AGENTS.md` is prose.** Its strength is reach: one file is honoured by most
agents on the market, and it can explain *why*, which generalises to situations
no rule anticipated. Its weakness is that it persuades rather than binds. An
agent in an auto-approve mode, part-way through a task, can talk itself past a
paragraph.

**The hook is enforcement.** `guard_hardware.py` runs before every `Bash` call
and exits non-zero on anything that would command a joint, so it cannot be
talked past. Its weakness is reach: it binds only the agent that loads it.

Neither is sufficient. Prose without enforcement fails exactly when the stakes
are highest, in a long autonomous run. Enforcement without prose produces an
agent that hits a wall it does not understand and starts trying to route around
it.

## What the hook gates

It is an allowlist. These are permitted, being read-only with no motion and no
persistent writes:

    stretch_params
    stretch_battery_check
    stretch_system_check
    stretch_body_server --status
    <any stretch tool> --help

Denied: anything matching `*_home`, `*_jog`, `*_teleop`, `*_stow`, `*_sweep`,
`*_calibration`, `*_flash`, `pose_play`, `pose_record`, or `REx_*`; any use of
`-d`/`--direct`; importing `stretch4_body.tools`; importing the direct `Robot`
class; and executing a `.py` file that calls `move_to`, `move_by`, `home`,
`stow`, `push_command`, or `set_velocity`.

Two distinctions matter, because getting either wrong makes the guardrail
useless or infuriating:

- **Executing a name is gated; mentioning one is not.** `cat
  routines/sweep.py` and `grep -n move_to routines/sweep.py` are always
  allowed. Refusing to read a file is a failure, not a safe default.
- **A hardware name is a name, not a path.** `stretch_arm_home` is a valid bare
  command with no `python` and no `.py`, so the check cannot key on file
  extensions.

Verify all of this yourself rather than trusting it:

    .claude/hooks/guard_hardware.py --self-test

## Customising

Add project-specific rules to the end of `AGENTS.md`. Leave the block between
the `hello-robot-core` comment markers alone, so it can be updated
mechanically later:

    <!-- hello-robot-core v2 -->
    ...
    <!-- /hello-robot-core v2 -->

To gate an additional command, add its stem to `DENY_STEMS` in
`guard_hardware.py`, add a case to `CASES`, and re-run `--self-test`. Extend the
allowlist only for something genuinely read-only, and remember that the cost of
an over-broad denial is a human who stops asking the agent for help.

## Summary for AI Agents

- The guardrail file an agent obeys is the one in the project directory it is
  working in. A file in an imported library does not reach it.
- `AGENTS.md` is advisory and portable; `.claude/hooks/guard_hardware.py` is
  binding and local. Ship both.
- The hook fails closed: it exits 2 with a reason on stderr, so an unrecognised
  payload or a crash blocks the command rather than allowing it.
- Reading code is always allowed, whatever the file is called. Only execution is
  gated.
- If the hook blocks you, report the exact refusal and hand the command to a
  human. Do not route around it, and do not edit the hook to unblock yourself.
