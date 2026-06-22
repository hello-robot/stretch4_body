# Robot Status Data and Server Logs

This document explains the organization of robot status data and server logs for the Stretch robot, how to access them, and how to use the visualization and telemetry utilities.

## Overview of Status Data Organization

Stretch Body Server handles data logging in two primary categories:

### 1. Stretch Body Server Logs (Process & Daemon Logging)
These are standard system process and execution logs. They contain debug, info, warning, and error messages from the server, its background systemd daemon, and communication protocols with the robot hardware (e.g., motor serial communication, joint safety limits).
* **Active Log**: A standard text file showing the active session logging.
* **Archived Session Logs**: Old log files are automatically bundled and stored in a `.tar.gz` archive when the server starts up or shuts down.

### 2. Robot Status Logger (JSON Telemetry & States)
These are high-frequency status logs captured by the `SentryStatusLogger` background process. They contain full status telemetry snapshots (JSON objects) representing motor speeds, efforts, battery state, voltages, and safety checks for all joints.
* **Active Telemetry Log**: Periodic status snapshots are batched every 60 seconds into timestamped JSON files under `log/stretch_status/`.
* **Size Management**: The logger automatically deletes the oldest status batches to maintain a maximum target directory size (configurable in MB).

---

## Log Locations and Directories

By default, Stretch Body determines its log directories relative to the `HELLO_FLEET_PATH` environment variable:

| Log Type | Standard Path | Default Fallback (if env not set) | Format |
|---|---|---|---|
| **Active Server Log** | `$HELLO_FLEET_PATH/log/stretch_body_logger/stretch_body_server.log` | `/tmp/log/stretch_body_logger/stretch_body_server.log` | Text (`.log`) |
| **Archived Server Logs** | `$HELLO_FLEET_PATH/log/stretch_body_logger/archive/stretch_body_server_logs_<timestamp>.tar.gz` | `/tmp/log/stretch_body_logger/archive/` | Compressed Tarball (`.tar.gz`) |
| **JSON Telemetry Logs** | `$HELLO_FLEET_PATH/log/stretch_status/status_<timestamp>.json` | `~/log/stretch_status/status_<timestamp>.json` | JSON Array (`.json`) |
| **Control Loop Rate Logs** | `$HELLO_FLEET_PATH/log/robot_rate_log/robot_rate_log_*` | `/tmp/log/robot_rate_log/` | JSON Array (`.json`) |

---

## Telemetry Visualization Tool: `stretch_status`

The `stretch_status` tool is a command-line interface to visualize, replay, pretty-print, and export robot telemetry data. It integrates with **Rerun** to plot active joints, motor diagnostics, power states, and safety sentinels dynamically.

### Core Arguments and Usage Modes

```bash
stretch_status [OPTIONS]
```

* **Live Mode**: Continuously pulls current robot status and streams it. Defaults to printing to console.
* **History Mode (`--history <minutes>`)**: Reads saved telemetry JSON files and replays them.
* **Visualization Mode (`--rerun`)**: Opens Rerun to plot telemetry dynamically while also printing to console.
* **Field Filtering (`--fields <prefixes>`)**: Limits the displayed or plotted telemetry to specific joints or components (e.g., `robot.lift` or `robot.power_periph.voltage`).
* **Interactive Menu**: If `--fields` is omitted, the tool opens an interactive console menu allowing you to choose joints or subsystems to visualize. Pressing Enter with no selection defaults to `robot.server`, `robot.routines`, and all joint positions.
* **Export / Import (`--export <dir>`, `--import <zip_file>`)**: Packages history data into compressed ZIP files or loads exported ZIP runs.

---

## Example Usage

### 1. Tailing & Viewing Server Process Logs
To check the Stretch Body Server's background daemon activity and stream active logging:
```bash
stretch_body_server --print
```

### 2. Live Telemetry Visualization in Rerun
To stream all live joint variables to Rerun at a frequency of 30 Hz:
```bash
stretch_status --rate 30 --rerun
```
This will open the Rerun UI interface, showing active time-series graphs of scalar and boolean data.

### 3. Replaying Offline Telemetry History
To graph the last 15 minutes of status history in Rerun:
```bash
stretch_status --history 15 --rerun
```

### 4. Interactive Console Inspection (No GUI)
To interactively view specific subsystem values directly in the console:
```bash
stretch_status --history 10
```
*Select a subsystem number from the menu (e.g., `robot.lift` or `robot.power_periph`), and the terminal will print out all key-value states.*

### 5. Advanced Field Filtering and Exporting
To export the last 30 minutes of telemetry to a ZIP archive:
```bash
stretch_status --history 30 --export ~/Desktop/
```
To import and view a saved ZIP run, showing only the lift and base joints:
```bash
stretch_status --import ~/Desktop/stretch_status_2026-05-31.zip --fields robot.lift robot.omnibase
```

## Firmware Event Diagnostics

Power-peripheral firmware has a separate RAM-only event log for bench debugging. This is not part of the normal Stretch Body status logger and is off by default except for rare critical events. To enable noisy diagnostic firmware events for the firmware's fixed auto-expiring window:

```python
robot.power_periph.enable_event_diagnostics()
robot.push_command()
```

To disable the diagnostic window immediately:

```python
robot.power_periph.disable_event_diagnostics()
robot.push_command()
```

Use this only while reproducing a firmware issue. It is intended for BMS, charger, rail, ESP pin, and shutdown-charge debugging and should not be left enabled during timing-sensitive control-loop tests.
