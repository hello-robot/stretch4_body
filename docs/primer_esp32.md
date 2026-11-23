# ESP32

The ESP32 on the power board sequences system power with the Pimu (sleep, wake,
shutdown, the NUC shutdown pulse, the 12V, aux 20V and lidar rails). It also
connects the robot to Wi-Fi and publishes battery and power state to Firebase,
from which a remote user can ask the robot to sleep or wake.

It has two links:

* **UART to the Pimu**: the power handshake. It works with the NUC off, so it
  stays on the Pimu.
* **USB to the host** (`/dev/hello-esp32`): an RPC link, the same transport the
  Pimu and steppers use. Firmware `hello-esp.v0.2.0p1` and later answer it.
  Older firmware only printed text and has no RPC.

## Firmware

The firmware (`stretch_firmware_ii/arduino/hello_esp`) runs three FreeRTOS tasks:

| task | priority | does |
|---|---|---|
| power | highest | Pimu UART link and power state machine |
| rpc | middle | host RPC on the USB port |
| `loop()` | lowest | Wi-Fi and Firebase, which can block for seconds |

So a slow TLS handshake can't delay a Pimu handshake or an RPC reply. Nothing
prints to the USB port, because the port carries the RPC. Messages go to a log
ring buffer that the host reads.

Build it with `compile_and_upload.py`, which uses
`USBMode=default,UploadMode=cdc,CDCOnBoot=cdc` (TinyUSB CDC on the USB port).
Without `CDCOnBoot=cdc` the RPC is not on the USB port.

## RPC

| RPC | gives / takes |
|---|---|
| board info | board variant, firmware version |
| status | power state, reset reason, heap, loop times, 12V/aux 20V rails, adapter, the outputs as last driven, the Pimu link (its last `Samd_Status`, freshness, UART counters) |
| Wi-Fi status | state, SSID, RSSI, channel, BSSID, IP/gateway/netmask/DNS, MAC, hostname, attempts, disconnects, last disconnect reason |
| Firebase status | state, publish counts, last error, command stream health (events, age, restarts), whether the clock is set, the last power command (id, action, result) |
| Wi-Fi config (get/set) | SSID and password, saved to flash or not, applied now or not |
| Firebase config (get/set) | database URL, fleet ID and robot key, and optionally a user to sign in as (API key, email, password). Needs `hello-esp.v0.2.6p1` or later |
| config | publish period, Wi-Fi retry period, Wi-Fi/Firebase enable, log level. Pushed at startup, not saved |
| trigger | Wi-Fi reconnect/disconnect/forget, Firebase restart/forget, Wi-Fi scan, reboot, clear log |
| Wi-Fi scan | up to 20 networks, strongest first |
| log | the log entries since the last read |

The layouts are in `EspRpc.h` (firmware) and `subsystem/esp32.py` (Stretch Body).
`test/test_esp32_protocol.py` checks that they agree.

## Stretch Body

The ESP32 is not part of the robot server. Stretch Body talks to it only directly,
over `/dev/hello-esp32`, and one process at a time can have the port.

* `subsystem/esp32.py`, class `Esp32`: the direct API.
* `stretch_esp32_jog`: status and set-up, using `Esp32`. It talks only to the ESP32.
  It prints the status, then offers a menu:
  * `w`, configure Wi-Fi: shows the current connection and the networks the ESP32
    sees, then connects to the one picked from the list, asking for its password.
    It can also use the network the robot's computer is on (from NetworkManager),
    or a hidden network.
  * `f`, configure Firebase: shows the current Firebase connection, then sets up
    remote sleep and wake (see [Firebase set-up](#firebase-set-up)), or takes the
    settings as typed.
  * `s`, print the status: system, power, Pimu link, Wi-Fi and Firebase. Enter, or
    any other key, does the same.
  * `q`, quit.

  Both wizards try new settings before saving them. Settings that work are saved
  to the ESP32's flash, and only there (see [Credentials](#credentials)). If they
  don't work, the wizard puts the ESP32 back as it was, and nothing is saved.
* `stretch_system_check` opens the ESP32 for a moment to check the Pimu link,
  Wi-Fi and Firebase.

```python
from stretch4_body.subsystem.esp32 import Esp32
e = Esp32()
if e.startup():
    s = e.status
    print(s['wifi']['state_name'], s['wifi']['ip'], s['firebase']['state_name'])
    e.set_wifi_config('MyNetwork', 'my password')
    e.push_command()
    for entry in e.read_log() or []:
        print(entry['level_name'], entry['text'])
    e.stop()
```

`python3 -m stretch4_body.subsystem.esp32` prints the status and the ESP32 log.

### What the robot server still sees

The ESP32 also reports to the Pimu over their UART link, as before. The Pimu puts
that into its own status, so `PowerPeriph` (and `RobotClient.power_periph`) still
has, with no ESP32 code in the server:

| `PowerPeriph.status` | from the ESP32 |
|---|---|
| `voltage_12v0`, `voltage_aux_cpu` | 12V and aux 20V rail voltages |
| `adapter_connected`, `adapter_fault` | charger adapter inputs |
| `connected_to_network` | Wi-Fi connected |
| `connected_to_firebase` | Firebase ready |

### Remote sleep and wake

An app sleeps and wakes the robot through the Realtime Database. Each robot's data
is under `robots/<fleet ID>/` (its `HELLO_FLEET_ID`): the app writes a command with a
unique id and the robot key to `power_system_requests/power_command`, and the ESP32
writes the outcome to `power_system_requests/power_command_ack` and the robot's
status, with a `last_seen` heartbeat, to `power_system_status`. The contract is in
`stretch_firmware_ii/arduino/hello_esp/README.md`, "Firebase". `stretch_esp32_jog`
shows the last command and the stream health under Firebase. Firebase `READY` means
the command stream is live; `STREAM_CONNECTING` means started, but commands won't
arrive yet.

#### Trying it from the Firebase console

Without an app, write the command in the console's **Data** tab, under
`robots/<fleet ID>/`. Sleep shuts down the NUC.

* **Sleep:** add `power_system_requests/power_command` with four children,
  `robot_key` = the robot key, `action` = `sleep`, `id` = `s1` and `acknowledged` =
  `false`, in one **Add**. The ESP32 ignores a command without its key.
* **Wake:** delete `power_command`, then add it again with `robot_key`, `action` =
  `wake`, `id` = `w1` and `acknowledged` = `false`. The robot acts only on a new write
  of the whole node with a new `id` (`s2`, `w2`, ...): editing one field of a command
  is ignored. Once it has seen a command, the robot sets its `acknowledged` to `true`,
  and ignores a command that is already acknowledged. Setting it back to `false`
  doesn't run the command again: the robot also keeps the last `id` it acted on. `ts`
  can be left out (the console can't write a server timestamp): while the robot is
  online, a command without one is carried out.

Then `power_system_requests/power_command_ack` goes `pending`, then `done`, and
`power_command/acknowledged` turns `true`. After a sleep,
`power_system_status/power_state` goes `active`, `going_to_sleep` (about 20 s while the
NUC shuts down), `sleeping`, and `last_seen` keeps advancing (Wi-Fi stays up). A wake
sent during `going_to_sleep` stays `pending` until the robot is asleep, then `done`;
the NUC takes about 20 s more to boot. A robot that is shut down (`shutting_down`, then
`last_seen` stops) can't be woken remotely: press its power button.

### Firebase set-up

Remote sleep and wake needs a Firebase project with a Realtime Database. The
customer guide is "Remote Power Management" in stretch4_docs; this is the same for
developers.

Each robot's data is under `robots/<fleet ID>`, and its key is in
`robots/<fleet ID>/robot_key`: 32 random URL-safe characters (192 bits) that
`stretch_esp32_jog` makes. Every write carries `robot_key`, and the rules
(`Esp32Firebase.rules()`) let it in only if it matches; no rule lets anyone read or
write `robot_key` itself (the console and the CLI can). The ESP32 also ignores a
command without its key. Reads are open, so the key keeps out blind writes, not
someone who reads the database first; with a Firebase user to sign in as, reads and
writes also need a signed-in user. The ESP32 checks Google's TLS certificates, so the
key can't be read in transit.

`stretch_esp32_jog`, `f`, then `n` ("set up Firebase"):

1. Takes the fleet ID from `$HELLO_FLEET_ID`, and keeps the robot's key, or makes one.
2. Asks whether to sign in as a Firebase user as well. The default is no. The user
   is asked for once the project is known: with the CLI, the tool reads the project's
   Web API key from its web app (`Esp32Firebase.web_api_key`, via `apps:list WEB` and
   `apps:sdkconfig WEB`; it offers to add a "Stretch power" web app if there is none),
   links to the project's Authentication pages, and asks only for the email and
   password. By hand, it asks for the API key too. Sign-in goes through Google's
   Identity Toolkit (`accounts:signInWithPassword`), which needs the key to find the
   project. With sign-in, `rules_work` also checks that the data can't be read
   without signing in, so rules without sign-in fail it and the tool offers the sign-in
   version (`rules(sign_in=True)`). Rules cover the whole database: every robot on it
   must then sign in.
3. Sets up the database:
   * **With the [Firebase CLI](https://firebase.google.com/docs/cli)** installed
     (recommended: `curl -sL https://firebase.tools | bash`, or with Node.js 20+
     `sudo npm install -g firebase-tools`, then `firebase login`), it offers to do it
     all: pick or create a project, find its Realtime Database, write
     `robots/<fleet ID>/robot_key`, and get the rules in force. It checks the rules
     by what they do, from the robot's computer the way the ESP32 connects
     (`Esp32Firebase.rules_work`): `robot_key` unreadable, a write with another key
     refused, a write with the key let in, the commands readable. If that fails, it
     reads the rules in force (`database:get /.settings/rules`, comments included):
     * a new database's default rules (locked, or a test-mode date) or this tool's
       own: it asks, then publishes the robot rules in their place;
     * other rules: it doesn't replace them. It adds the robot rules to them as a
       comment inside `"rules"` (`Esp32Firebase.rules_with_comments`; the rules in
       force don't change), publishes that, asks you to merge them in the console,
       and checks again when you press Enter.

     Then it waits until the rules work before it gives the ESP32 the settings. The
     CLI can't create a project's first database, so for a new project it asks you
     to click Create Database in the console, then goes on. The CLI must be logged in
     (`firebase login`): if it isn't, or its login has expired, the tool says to run
     `firebase login` (or `firebase login --reauth`), and goes back to the Firebase
     menu. An expired login still shows in `firebase login:list`; the CLI's `--json`
     error hides the cause, so `Esp32Firebase.run_cli` runs the CLI in a temporary
     directory and looks in its `firebase-debug.log` (`FirebaseAuthError`).
   * **Without it**, it says so, shows how to install it, and asks whether to set up
     by hand instead (the default goes back to the menu). By hand, it prints the
     steps for the console: create the project and the database (locked mode),
     publish the rules over the defaults or merge them into other rules (`p` in the
     menu prints them), and add `robots/<fleet ID>/robot_key`. Then it asks for the
     database URL, and checks the rules the same way.

   The suggested project ID is `$HELLO_FLEET_ID-pwr-mgmt`, lowercase (e.g.
   `stretch-se4-4024-pwr-mgmt`). The URL prompts default to that project's default
   database, `https://<project ID>-default-rtdb.firebaseio.com` (a `us-central1`
   database; other locations use `<name>.<region>.firebasedatabase.app`).
4. Tries the settings, and saves them to the ESP32's flash if Firebase comes up
   (`READY`).
5. Prints what the app needs: the database URL, the fleet ID and the robot key. `a`
   in the menu shows them again.

`c` in the Firebase menu prints curl commands and a Python example that sleep and
wake this robot through the database, with its URL, fleet ID and robot key filled in
(and a sign-in step, with `<password>` to fill in, if the robot signs in as a user).
The Python `power('sleep')` / `power('wake')` writes the command with the robot key, a
new id and `acknowledged: false`, and waits for `power_command_ack`. They are made by
`command_snippets()` in `stretch_esp32_jog`.

Several robots can share a database: run `n` on each. Each uses its fleet ID and its
own key, and the rules are put in force once.

If Firebase doesn't come up, the last error says why. `401 unauthorized`: the rules
refused the robot (are the robot rules in force, and does
`robots/<fleet ID>/robot_key` hold its key?).

The rules and the CLI steps are in `subsystem/esp32.py`, class `Esp32Firebase`.
`test/test_esp32_protocol.py` checks the rules in the Realtime Database emulator
where Java and the emulator are installed (`firebase setup:emulators:database`).

### Credentials

The Wi-Fi and Firebase credentials (Wi-Fi password, robot key, and the optional
Firebase password) are secrets, so they are kept only in the ESP32's flash. Nothing
writes them to the robot parameters or `stretch_configuration_params.yaml`.
`Esp32.get_wifi_config()` and `get_firebase_config()` read them back, passwords
included. If the ESP32 loses them (a flash erase, a new board), set it up again:
Wi-Fi with `w`, and Firebase with `n`, which makes a new robot key for the app.

For firmware older than `hello-esp.v0.2.0p1`, the only route is through the Pimu:
`PowerPeriph.send_network_info(ssid, password)` /
`send_firebase_info(url, api_key, user_email, user_password)`, which take the
credentials as arguments. That route carries no robot key: the ESP32 keeps the key
it has. ESP32 reset and
bootloader also go through the Pimu, so they work whatever the ESP32 firmware is
doing: `PowerPeriph.set_esp32_reset()` / `set_esp32_fw_update()`, or `g` / `e` in
`stretch_power_periph_jog -d`.
