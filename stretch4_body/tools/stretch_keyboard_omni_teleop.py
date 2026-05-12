#!/usr/bin/env python3
"""
Keyboard Teleop for Stretch 4 Omnibase.

You may need to `pip install sshkeyboard` to use this script.
"""

import asyncio
from time import sleep, time
import click
import sshkeyboard
import argparse

def print_keyboard_options():
    click.secho("=========================", fg="yellow")
    click.secho("Keyboard Controls:", fg="yellow")
    click.secho("=========================", fg="yellow")
    print("""
W / A / S / D: Move Lateral
Q / Z / C / E : Move Diagonal
X / V: Rotate in place
Enter: To exit
""")
    click.secho("=========================", fg="yellow")

TRANSLATE_BY = 0.07
def keyboard_control(key:str|None, robot):
    #check_runstop(robot)

    base = robot.base
    if key == "w":
        base.translate_by(TRANSLATE_BY, 0.0)
    elif key == "s":
        base.translate_by(-TRANSLATE_BY, 0.0)
    elif key == "a":
        base.translate_by(0.0,TRANSLATE_BY)
    elif key == "d":
        base.translate_by(0.0,-TRANSLATE_BY)
    elif key == "q":
        base.translate_by(TRANSLATE_BY, TRANSLATE_BY)
    elif key == "z":
        base.translate_by(-TRANSLATE_BY, TRANSLATE_BY)
    elif key == "c":
        base.translate_by(-TRANSLATE_BY, -TRANSLATE_BY)
    elif key == "e":
        base.translate_by(TRANSLATE_BY, -TRANSLATE_BY)
    elif key == "x":
        base.rotate_by(-TRANSLATE_BY)
    elif key == "v":
        base.rotate_by(TRANSLATE_BY)
    elif key == "enter":
        return sshkeyboard.stop_listening()

    robot.push_command()

def check_runstop(robot):
    while robot.power_periph.status['runstop_event']:
        click.secho("The robot is runstopped", fg="red")
        sleep(1)

async def _react_to_input_new(state, options):
    """This method overrides the one in sshkeyboard to allow spamming same key."""
    # Read next character
    state.current = sshkeyboard._read_char(options.debug)

    # Skip and continue if read failed
    if state.current is None:
        return state

    # Handle any character
    elif state.current != "":

        # Make lower case if requested
        if options.lower:
            state.current = state.current.lower()

        # Stop if until character has been read
        if options.until is not None and state.current == options.until:
            sshkeyboard.stop_listening()
            return state

        # Release state.previous if new pressed
        if state.previous != "" and state.current != state.previous:
            await options.on_release_callback(state.previous)
            if sshkeyboard._is_windows and not options.sequential:
                await asyncio.sleep(options.sleep)
        else:
            await options.on_press_callback(state.current)
            state.initial_press_time = time()
            state.previous = state.current

        # Update press time
        if state.current == state.previous:
            state.press_time = time()

    elif state.previous != "" and (
        time() - state.initial_press_time > options.delay_second_char
        and time() - state.press_time > options.delay_other_chars
    ):
        await options.on_release_callback(state.previous)
        state.previous = state.current

    return state


sshkeyboard._react_to_input = _react_to_input_new

def keyboard_teleop_loop(robot, is_stop_robot_on_quit:bool):
    print_keyboard_options()

    sshkeyboard.listen_keyboard(
        on_press=lambda key: keyboard_control(key, robot)
    )

    if is_stop_robot_on_quit:
        robot.stop()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Control Stretch Omnibase from a Keyboard')
    parser.add_argument("-d", "--direct", help="Use direct API (no server)", action="store_true")
    args = parser.parse_args()

    if not args.direct:
        from stretch4_body.robot.robot_client import RobotClient as Robot
    else:
        from stretch4_body.robot.robot import Robot

    robot = Robot()
    robot.startup()

    keyboard_teleop_loop(robot, True)
