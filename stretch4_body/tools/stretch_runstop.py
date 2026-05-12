import stretch4_body.robot.robot_client as rc
import argparse

def clear_runstop():
    robot = rc.RobotClient()
    robot.startup()
    robot.power_periph.clear_runstop()
    return robot.push_command()

def trigger_runstop():
    robot = rc.RobotClient()
    robot.startup()
    robot.power_periph.trigger_runstop()
    return robot.push_command()


if __name__ == "__main__":
    args = argparse.ArgumentParser()
    args.add_argument('--clear', action='store_true', help='Clear the runstop')
    args.add_argument('--trigger', action='store_true', help='Trigger the runstop')
    args = args.parse_args()
    if args.clear:
        clear_runstop()
    elif args.trigger:
        trigger_runstop()
    else:
        raise ValueError("No action specified. Use --clear or --trigger")