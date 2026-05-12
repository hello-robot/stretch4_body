# Core Components Primer

The `stretch4_body/core` directory contains the foundational base classes, utilities, and communication protocols that power the `stretch4_body` software stack.

These classes are typically **not used directly by end-user developers**. Instead, they provide the internal scaffolding that the higher-level robot subsystems (like `OmniBase`, `Arm`, `Lift`, etc.) build upon to interact with the hardware and network.

## Organizational Structure

The contents of the `core` directory can be grouped into six main functional areas:

1. Device and Hardware Abstractions
2. Teleoperation and Gamepad Control
3. Client/Server and IPC (Inter-Process Communication)
4. Diagnostics, Visualization, and Tracing
5. General Utilities and Configuration
6. Subdirectories (`factory`, `feetech`, `transport`)

---

### 1. Device and Hardware Abstractions

These files define the base Python representations of the physical robot hardware.

*   **`device.py`**: Defines the foundational `Device` base class inherited by all hardware subsystems. It manages basic lifecycle execution (startup, stop), parses configuration dictionaries, and handles logging initialization.
*   **`prismatic_joint.py`**: An abstraction layer for linear joints, inherited specifically by the `Arm` and `Lift` subsystems. It handles the mathematical conversions from motor rotation to linear translation, manages soft-limits, and coordinates homing procedures.
*   **`stepper.py`**: The API interface for the custom Hello Robot stepper motor controllers (used by the mobile base wheels, arm, and lift). It manages the serial command protocols for position/velocity trajectory control, guarded motion (collision detection), and motor telemetry reporting.
*   **`worker_loop.py`**: A utility wrapper for instantiating high-frequency, non-blocking background multiprocessing workers (used extensively to isolate I/O tasks in the `end_of_arm` and `line_sensor` subsystems).

### 2. Teleoperation and Gamepad Control

This module handles interpreting user input from USB/Bluetooth gamepads (like Xbox controllers) into smooth robotic motion.

*   **`gamepad_teleop.py` & `gamepad_controller.py`**: The primary drivers and classes for reading asynchronous events from the controller and smoothing them into velocity or position commands for the robot's joints.
*   **`gamepad_control_mappings.py` & `gamepad_enums.py` & `gamepad_joints.py`**: Define the specific button mappings, state machines, and joint-specific kinematics required to drive the robot intuitively.

### 3. Client/Server and IPC

These files manage the multi-process architecture of Stretch Body, allowing multiple user scripts to communicate with the robot hardware simultaneously safely.

*   **`client_server.py`**: Contains the `StretchBodyServer` and `StretchBodyClient` classes. The server uses ZeroMQ (ZMQ) to publish high-frequency robot status, subscribe to incoming commands, and multiplex access by managing client priorities and leases (preventing command collisions).
*   **`subsystem_client.py`**: Provides the base ZMQ client used by isolated worker processes to push commands and pull status data to/from the main server loop.

### 4. Diagnostics, Visualization, and Tracing

Tools for ensuring the robot operates safely and providing developers with visual debugging aids.

*   **`robot_monitor.py`**: A background thread that continually checks for hardware faults—such as low battery state-of-charge, runstop events, or over-tilting—and can autonomously trigger safety behaviors.
*   **`robot_trace.py` & `scope.py`**: Utilities for capturing, logging, and visualizing high-frequency telemetry. `scope.py` uses Matplotlib to generate oscilloscope-like plots for tuning motor control loops.
*   **`rerun_plot.py` & `rerun_dynamic_plotter.py`**: Integrations with the Rerun SDK for advanced, real-time 3D visual telemetry and state debugging.
*   **`mujoco_urdf.py`**: Uses the MuJoCo physics engine to load the robot's URDF, facilitating dynamic self-collision detection and collision visualization.

### 5. General Utilities and Configuration

*   **`robot_params.py`**: The central configuration manager. It loads, merges, and resolves the robot's layered YAML configuration files (`stretch_factory_params`, `stretch_configuration_params`, and `stretch_user_yaml`), providing the unified dictionary used by every subsystem.
*   **`hello_utils.py`**: A collection of miscellaneous, widely-used helpers for thread management, POSIX file locking, timestamping, and parsing configuration paths.

---

### 6. Subdirectories

The `core` folder also contains three deeply integrated subdirectories handling low-level communication and manufacturing tooling:

*   **`factory/`**: A suite of low-level utility classes and scripts strictly intended for manufacturing, hardware testing, and firmware flashing. Developers should rarely need to interact with these unless debugging deep hardware issues.
*   **`feetech/`**: Communication wrappers and SDK implementations for interfacing with the Feetech smart serial servos. These are used exclusively to actuate the `end_of_arm` tools (like the dexterous wrist and gripper).
*   **`transport/`**: Contains the critical Python and C++ extensions that manage the raw, high-speed serial packet communication to the custom motor controllers and microcontrollers distributed across the robot.
