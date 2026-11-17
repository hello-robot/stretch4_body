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
| Firebase status | state, publish counts, last error, last power trigger, stream events |
| Wi-Fi config (get/set) | SSID and password, saved to flash or not, applied now or not |
| Firebase config (get/set) | URL, API key, user email and password |
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
    the network saved in the robot configuration, or a hidden network.
  * `f`, configure Firebase: shows the current Firebase connection, then takes the
    settings from the robot configuration or as typed.
  * `s`, print the status: system, power, Pimu link, Wi-Fi and Firebase. Enter, or
    any other key, does the same.
  * `q`, quit.

  Both wizards try new settings before saving them. Settings that work are saved
  to the ESP32's flash and to `esp32.wifi` / `esp32.firebase` in
  `stretch_configuration_params.yaml`. If they don't work, the wizard puts the
  ESP32 back as it was, and nothing is saved.
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

### Credentials

The ESP32 keeps the Wi-Fi and Firebase credentials in its flash. `Esp32.get_wifi_config()`
and `get_firebase_config()` read them back, passwords included.

`send_credentials_from_params()` sends the credentials from the robot parameters:
`esp32.wifi` and `esp32.firebase` in `stretch_configuration_params.yaml`. When those
are `NA`, it falls back to the older `power-periph.firebase` block.

For firmware older than `hello-esp.v0.2.0p1`, the only route is through the Pimu:
`PowerPeriph.send_network_info()` / `send_firebase_info()`. ESP32 reset and
bootloader also go through the Pimu, so they work whatever the ESP32 firmware is
doing: `PowerPeriph.set_esp32_reset()` / `set_esp32_fw_update()`, or `g` / `e` in
`stretch_power_periph_jog -d`.
