# Gamepad Teleop for Stretch

Stretch 4 ships with a gamepad controller for tele-operation.

You can start the gamepad teleop script using `stretch_gamepad_teleop`. 

> Note: `stretch_gamepad_teleop` uses a [RobotClient](./primer_robot_client.md). Make sure the Stretch Body Server is running using `stretch_body_server --status`.

When you boot Stretch 4, the joints need calibration. You may notice that the controller vibrates and does not allow you to move the robot. If this happens, make sure the robot's surroundings are clear, and press the `Start` button on the controller to Home (calibrate) the joints.

By default, `stretch_gamepad_teleop` has two control mapping modes that you can switch between using `Y`. The first is Joint Control, which allows you to control each joint individually. The second is Flying Gripper Control, which allows you to use the `Right Stick` to move the wrist, and move towards where the gripper is pointing using the `Left Stick`. The latter is useful for manipulation and grasping objects.


You can keep the `stretch_gamepad_teleop` application script running in the background while you run other RobotClient applications. The `stretch_gamepad_teleop` script will timeout and stop sending commands to the server after a few milliseconds of not receiving inputs. Note that `stretch_gamepad_teleop` has a higher priority on the RobotClient command queue, so commands from it will override other scripts by default.  

## File Structure

The [`GamePadTeleop`](../stretch4_body/core/gamepad_teleop.py) class holds the main loop for the gamepad teleop application. The loop listens to controller inputs,  to robot motions. 

It supports multiple control mappings, mode switching, and custom user functions.


## Key Features

### Mode Switching

- **Motion Profile**: Hold the **Top Button (Y)** for 2 seconds to cycle between motion profiles (e.g., Default, Slow, Fast).
- **Control Mapping**: Hold the **Select (Back) Button** for 2 seconds to cycle between control mappings (Default, Analog Wrist, Manipulation).
- **Contact Sensitivity**: Short press the **Select (Back) Button** to cycle contact sensitivity profiles.

### Special Functions

- **Homing**: If the robot is not homed, press the **Start Button** to home the robot.
- **Gripper Handedness**: If the robot is homed, hold the **Start Button** for 3 seconds to switch gripper handedness (Left/Right). The robot will perform a motion to reorient the gripper if safe.
- **Shutdown**: Hold the **Select (Back) Button** for 10 seconds to stow the robot and shut down the onboard PC.

## Control Mappings

The system supports three main control mappings:

### 1. Omnibase Mapping
Teleoperation mapping that mixes both manipulation controls with the d-pad and omnibase movement with the analog sticks.
- **Sticks**: Control Arm (Left Stick Y), Lift (Right Stick Y), and Base (Left Stick X/Y, Right Stick X).
- **Bumpers (LB/RB)**: Wrist Yaw.
- **D-Pad**: Wrist Pitch (Up/Down) and Wrist Roll (Left/Right).
- **Buttons**: Gripper Open/Close (A/B).

### 2. Manipulation Mapping
Designed for wrist control using analog inputs. Wrist and arm controls are separated from omnibase movement using the right-trigger.
- **Triggers**:
    - **Left Trigger (>0.9)**: Precision Mode (Slower movements).
    - **Right Trigger (>0.9)**: Manipulation Mode.
- **Manipulation Mode**:
    - Right Stick X/Y controls Wrist Yaw and Pitch.
    - D-Pad Left/Right controls Wrist Roll.


## Customization

You can customize the behavior by:
- Modifying `gamepad_control_mappings.py` to add or edit mappings.
- Setting `params['function_cmd']` in user parameters to execute a shell command via the **Function Button (Xbox / Left Button)**.
