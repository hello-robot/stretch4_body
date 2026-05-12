# Behaviors Primer

The `stretch4_body/behavior` directory contains the logic for advanced autonomous functionality and safety monitoring within the `RobotServer`. It is structured into three primary types of behaviors: **Sentries**, **Routines**, and **Safe Motions**.

These behaviors operate within a dynamic plug-in architecture, allowing new safety checks or autonomous sequences to be added without modifying the core server loop.

---

## 1. High-Level Roles

### Sentries
**Role:** Continuous background monitoring and limit enforcement.
Sentries constantly watch the robot's state (telemetry, odometry, current draw, CPU temperature) and enforce dynamic limits on the robot's capabilities. For example, the `sentry_limit_vel_on_pose` reduces the maximum allowed base velocity if the arm is extended far out, preventing tipping. The `sentry_self_collision` monitors for self-collision and updates safety limits. Sentries generally do not create motion commands; they enforce the bounds within which motion commands must operate.

### Safe Motions
**Role:** Final-check command modification and hazard avoidance.
Safe motions act as a final layer of defense. They intercept the pending commands immediately before they are sent to the hardware. If a command would cause a hazard (e.g., `safe_motion_overtilt_avoid`), the safe motion plug-in can actively overwrite the setpoint (e.g., zero out the velocity) or trigger a system-wide safe stop. 

### Routines
**Role:** Predefined, autonomous macro sequences.
Routines handle complex, multi-step actions like homing (`routine_homing`), stowing the arm (`routine_stow`), or docking to the charger (`routine_blind_dock`). When a routine is active, it takes over control of the robot, rejecting motion commands from external clients until the routine finishes or is explicitly canceled.

---

## 2. Plug-in Architecture and Management

The `RobotServer` does not hardcode which behaviors run. Instead, it relies on three manager classes (`SentryManager`, `SafeMotionManager`, and `RoutineManager`). 

During startup, each manager reads the `controllers` list from the active parameter configuration (`robot_params`). It then uses `importlib` to dynamically import the corresponding `py_module_name` and instantiate the `py_class_name`. 

```yaml
# Example from YAML configuration

sentry_cpu_temp:
  py_module_name: stretch4_body.behavior.sentries.sentry_cpu_temp
  py_class_name: SentryCPUTemp
  enabled: 1
```

If a behavior is marked with `enabled: 1`, it is instantiated and passed a reference to the `Robot` instance. This allows developers to easily create and inject custom sentries or routines simply by updating the YAML configuration.

---

## 3. Flow of Control within the Server Loop

Understanding the differences between these three behaviors requires looking at exactly *when* they execute inside the `RobotServer`'s 100Hz control loop:

1. **Pull Status:** The server asynchronously pulls the latest state from the hardware.
2. **Step Sentries (`sentry_manager.step()`):** Sentries run immediately after new state data arrives. They analyze the state and update internal limits (like `max_vel` or `max_accel`). They *do not* see incoming commands.
3. **Ingest Client Commands:** The server receives new commands from the ZMQ network (e.g., user scripts, ROS).
4. **Step Routines (`routine_manager.step()`):** If a Routine is active, it overrides step 3. The routine takes over generating the motion commands for this cycle and external client commands are rejected. 
5. **Step Safe Motions (`safe_motion_manager.step()`):** Right before the commands are sent to the motors, Safe Motions analyze the pending trajectory. If the command violates a safety condition, the Safe Motion will rewrite the command (e.g., override the target velocity to 0) or trigger a safe stop.
6. **Push Command:** The server pushes the finalized commands to the hardware via serial/ZMQ.
7. **Publish Status:** The server broadcasts the updated state and results to listening clients.

### Summary of Differences

*   **Sentries** run *early* in the loop. They analyze state and set bounds.
*   **Routines** run in the *middle* of the loop. They *generate* commands and block the client.
*   **Safe Motions** run at the *end* of the loop. They analyze and *overwrite/veto* commands just before execution.
