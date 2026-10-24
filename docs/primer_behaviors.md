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

---

## 4. Lidar idle sentry

`sentry_lidar_idle` powers the two head lidars down when nothing on the NUC uses their data, and powers them back up when something does. Without it the lidar rail stays on for as long as the PIMU is ACTIVE (about 15 W and motor wear for both units).

**Detection.** A background thread reads `/proc/net/udp`, `udp6`, `tcp` and `tcp6` once per `poll_period_s` (at least 1 s; nothing is read in the 100 Hz step). A consumer is:

* a UDP socket bound to a lidar data port (`udp_ports`, 2368 right and 2378 left), owned by any user: the Hesai ROS driver, pyhesai streams, `stretch_system_check`, raw listeners;
* a TCP session to a lidar PTC port (`ptc_port` 9347 at `lidar_ips`) in ESTABLISHED or SYN_SENT, when `ptc_counts_as_consumer` is 1. A connect attempt to an unpowered lidar therefore wakes it; the tool fails once and a retry after the spin-up works.

Process names (status and logs only) are looked up through `/proc/<pid>/fd` when the socket set changes. A raw capture (`tcpdump`) is not a consumer.

**Rail control.** Only through `power_periph.set_lidar_on/off` (PIMU trigger bits 18 and 17); there is no new RPC. The sentry calls them with `source='sentry'`; every other caller is a client.

* ON within one poll of a consumer appearing, only while the PIMU reports `STATE_ACTIVE` in a status sample no older than `pimu_state_max_age_s`.
* OFF after `idle_timeout_s` with no consumer and at least `min_on_s` after the last ON, also only with a fresh ACTIVE sample. Without power-state telemetry the sentry never cuts the rail.
* At server start the rail is assumed on (firmware powers it at every wake) and a full idle countdown starts; with the default `idle_timeout_s` of 0 the rail goes off on the first poll unless a consumer is already there. When the PIMU returns to ACTIVE from another state the same reset happens.
* On server stop, if the sentry had cut the rail and the PIMU is ACTIVE, the rail is turned back on (`restore_on_stop`), so tools run with the server stopped find the lidars powered.

**Manual hold.** A client `set_lidar_on` (`REx_lidar on`, `RobotClient.power_periph.set_lidar_on()`, the FCC exerciser) holds the rail on. The hold ends on:

* a client `set_lidar_off`: rail off; the sentry keeps it off until a consumer appears;
* a client `release_lidar_hold()`: rail unchanged, the idle countdown restarts from the release. Host side only, nothing is sent to the PIMU;
* `hold_timeout_s` after the last client `set_lidar_on`, when that param is above 0 (default 0, no timeout);
* a server restart, which includes every P14 sleep and wake.

`stretch_system_check --sensors` takes a hold, listens up to 40 s for packets from both lidars (spin-up to first packets is about 8 s), runs the PTC checks, then releases. `stretch_monitor_devices` (active mode) stops the server first, which restores the rail, and waits 40 s before reporting a lidar as NO DATA.

**Status.** `status['safety_layer']['sentry_manager']['sentry_lidar_idle']`:

| Key | Meaning |
|---|---|
| `rail_cmd` | `on` or `off`, last commanded state (the eFuse has no sense line, so this is not measured) |
| `rail_cmd_source` | `sentry`, `client` or `firmware_wake` |
| `rail_cmd_ts` | wall time of that command |
| `held`, `hold_remaining_s` | manual hold active, seconds left (None without a timeout) |
| `consumers` | list of `{proto, port, pid, name}` from the last scan |
| `idle_s`, `seconds_to_off` | idle time so far and time until the sentry cuts the rail (None while held, consumed or off) |
| `pimu_state` | `active`, `inactive` or `unknown` |
| `scan_age_s` | age of the last /proc scan |
| `on_events`, `off_events` | rail changes since server start |
| `last_action`, `last_reason` | e.g. `off`, `idle 0 s, no consumer` |

`status['power_periph']['lidar_cmd']` holds the last host request (`action` on, off or release, `source`, `ts`, `seq`) even with the sentry disabled. Every rail change is logged at info by the `sentry_lidar_idle` logger with its reason.

**Parameters** (`robot_params_SE4.py` and `robot_params_SE4UNH.py`; change with `stretch_change_param.py`, check with `stretch_params.py | grep sentry_lidar_idle`):

| Param | Default | Meaning |
|---|---|---|
| `enabled` | 1 | 0: the sentry is not built and the rail behaves as before |
| `idle_timeout_s` | 0 | no consumer for this long, then rail off; 0 cuts on the first poll without one |
| `min_on_s` | 0 | never cut sooner than this after an ON |
| `poll_period_s` | 1.0 | /proc scan period, clamped to at least 1 s |
| `udp_ports` | [2368, 2378] | lidar data ports |
| `lidar_ips` | [192.168.1.201, 192.168.1.202] | PTC peers that count |
| `ptc_port` | 9347 | lidar PTC TCP port |
| `ptc_counts_as_consumer` | 1 | PTC sessions keep the rail on |
| `hold_timeout_s` | 0 | manual hold length; 0 means until released |
| `pimu_state_max_age_s` | 2.0 | power-state sample age that still counts as fresh |
| `restore_on_stop` | 1 | turn the rail back on at server stop if the sentry cut it |

Opt out per robot with `enabled: 0`, per session with a client `set_lidar_on`. A laptop on the head port (LidarUtilities) is invisible to the NUC and needs one of the two.
