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

* `subsystem/esp32.py`, class `Esp32`: the direct API, for use with the server stopped.
* `subsystem/esp32_loop.py`, class `Esp32Loop`: the server subsystem. It runs
  `Esp32` in its own process at `esp32.loop_rate_Hz` (2 Hz), so ESP32 reads never
  touch the control loop. It copies the ESP32 log into the Stretch Body log.
* `robot_client.Esp32Client`: `robot.esp32` on a `RobotClient`.
* `stretch_esp32_jog`: status and set-up. It goes through the server
  (`Esp32Client`) by default, or uses `Esp32` with `-d` (server stopped). It talks
  only to the ESP32. It prints the status, then offers a menu:
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
  `stretch_configuration_params.yaml`. If they don't work, the wizard offers to go
  back to the previous ones, and nothing is saved.

Turn the subsystem on by listing it in `robot.server.subsystems`. It is in the
SE4 defaults, but a `stretch_user_params.yaml` that sets that list (to add
`line_sensor_loop`, for example) replaces it, so add `esp32` there too:

```yaml
robot:
  server:
    subsystems: ['esp32', 'line_sensor_loop']
```

The server starts without an ESP32, and with one on firmware that has no RPC.
`status['esp32']['hw_valid']` is then False, and the loop retries every
`esp32.reconnect_period_s`.

```python
from stretch4_body.robot.robot_client import RobotClient
with RobotClient() as r:
    s = r.esp32.status
    print(s['wifi']['state_name'], s['wifi']['ip'], s['firebase']['state_name'])
    r.esp32.set_wifi_config('MyNetwork', 'my password')
    r.push_command()
```

### Credentials

The ESP32 keeps the Wi-Fi and Firebase credentials in its flash. They can be read back:

* directly: `Esp32.get_wifi_config()` / `get_firebase_config()`;
* through the server: `status['esp32']['wifi_config']` and `['firebase_config']`.
  These carry the SSID, URL and email, plus `*_set` flags. They carry the passwords
  and API key only if `esp32.publish_credentials` is 1, because server status
  reaches every client, including remote ones when network access is on.

`send_credentials_from_params()` sends the credentials
from the robot parameters: `esp32.wifi` and `esp32.firebase` in
`stretch_configuration_params.yaml`. When those are `NA`, it falls back to the
older `power-periph.firebase` block.

For firmware older than `hello-esp.v0.2.0p1`, the only route is through the Pimu:
`PowerPeriph.send_network_info()` / `send_firebase_info()`. ESP32 reset and
bootloader also go through the Pimu, so they work whatever the ESP32 firmware is
doing: `PowerPeriph.set_esp32_reset()` / `set_esp32_fw_update()`, or `g` / `e` in
`stretch_power_periph_jog -d`.
