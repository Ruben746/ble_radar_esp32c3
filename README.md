# ESP32-C3 BLE Radar

**English** · [Français](README.fr.md)

![ESP32-C3 BLE Radar](assets/ble-radar-presentation.png)

The image in `assets/ble-radar-presentation.png` is a GitHub presentation asset only. It is not embedded in the sketch, compiled into the firmware or uploaded to the ESP32.

A standalone BLE presence radar for ESP32-C3 SuperMini, featuring a live French-language Web dashboard, persistent watchlist, RSSI proximity estimates and Telegram alerts. Firmware, HTML, CSS, JavaScript and JSON API are contained in one Arduino sketch.

## Features

- Continuous active BLE scanning: names, MAC addresses, address types, raw/filtered RSSI and available manufacturer/service information.
- Up to **50 visible devices**, **20 watched devices** and **200 remembered MAC addresses**.
- Per-device appearance, proximity and disappearance alerts, confirmations, cooldowns and hysteresis.
- One-meter RSSI calibration and approximate distance estimates.
- Optional new-address alerts with rate limiting, disabled by default. Random addresses are ignored by default for these alerts.
- Persistent settings/watchlist in NVS and a volatile 30-entry event log.
- First-run setup, four-digit Web PIN, sessions and login attempt limiting.
- Wi-Fi reconnection, fallback AP after roughly two minutes offline, optional mDNS.
- Editable fallback AP password in network settings, with confirmation and persistence across reboots.

## Hardware and target versions

| Component | Target configuration |
| --- | --- |
| Board | ESP32-C3 SuperMini; select **ESP32C3 Dev Module** |
| Arduino core | **esp32 by Espressif Systems 3.3.7** |
| Bluetooth | Core-bundled BLE library, NimBLE stack on this target |
| Flash | **4 MB**, if your board actually has 4 MB |
| Partition scheme | **Huge APP (3MB No OTA/1MB SPIFFS)** |
| USB CDC On Boot | **Enabled** for native USB serial |
| Serial monitor | **115200 baud** |
| Onboard blue LED | **GPIO8**, active LOW (`LOW` = on, `HIGH` = off) |

Use a USB data cable, stable power and a 2.4 GHz Wi-Fi network. No additional sensor, display or wiring is required. Check your particular SuperMini board specifications.

The original 3.2.0/Bluedroid header was outdated: the sketch directly uses NimBLE GAP calls. This package targets **3.3.7**, whose bundled BLE library supports this stack. Do not install an old standalone `ESP32 BLE Arduino` library or an external NimBLE library for this configuration. See the [3.3.7 BLE implementation](https://github.com/espressif/arduino-esp32/blob/3.3.7/libraries/BLE/src/BLEDevice.h).

## BLE scan indicator

The standard black SuperMini shown in the banner uses a **blue user LED on GPIO8**, with inverted logic: **LOW turns it on; HIGH turns it off**. The power indicator is separate. See the [board documentation](https://nuttx.incubator.apache.org/docs/latest/platforms/risc-v/esp32c3/boards/esp32c3-supermini/index.html).

In this sketch, `LED_PIN` is **8**. The LED starts off, flashes for approximately **80 ms** when a scan start is accepted, then flashes approximately **once per second while scanning**. The scanner is continuous (`BLE_HS_FOREVER`), so this is an activity heartbeat, not a new scan started every second. It works even with no detected devices and stops pulsing while scanning is stopped or a start fails. After two minutes with no advertisements, the existing scan restart mechanism can briefly interrupt the heartbeat.

LED timing is non-blocking: no `delay()` is added to the scan or Web handling. The main loop turns the LED off after the pulse interval; heavy loop activity can lengthen a pulse. Pulses indicate software scan activity, not proof of a received packet or successful alert.

Set `LED_PIN` to `-1` to disable the indicator. This configuration targets the standard black SuperMini in the supplied photo; other clones may differ. GPIO8 is also a boot strapping pin: do not add external circuitry pulling it low during reset.

## Installation

1. Install Arduino IDE and add this URL to **Additional Boards Manager URLs**:
   `https://espressif.github.io/arduino-esp32/package_esp32_index.json`
2. Install **esp32 by Espressif Systems 3.3.7** in Boards Manager.
3. Extract the archive. Keep the folder name **ble_radar_esp32c3**, matching the sketch name, and open `ble_radar_esp32c3.ino`.
4. Select **ESP32C3 Dev Module**, the serial port and the options above. Leave other options at their defaults.
5. Verify/compile and upload. If download mode does not start, hold BOOT, briefly press RESET, release BOOT and retry.
6. Open the serial monitor at **115200 baud** and reset the board to read connection details.

WiFi, WebServer, Preferences, ESPmDNS, HTTPClient, WiFiClientSecure and BLE are supplied by the core. No additional JSON library is required. See the [official installation guide](https://docs.espressif.com/projects/arduino-esp32/en/latest/installing.html).

### Arduino CLI

Run from the parent of the extracted `ble_radar_esp32c3` folder:

```sh
arduino-cli core update-index --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli core install esp32:esp32@3.3.7 --additional-urls https://espressif.github.io/arduino-esp32/package_esp32_index.json
arduino-cli compile --fqbn esp32:esp32:esp32c3:CDCOnBoot=cdc,PartitionScheme=huge_app ble_radar_esp32c3
```

## First setup

1. Read the AP name **ESP-C3-XXXX** in the serial monitor. With no saved AP password, the default is **`ChangeMe123!`**.
2. Join that network and manually open **http://192.168.4.1**. There is no captive DNS portal.
3. Enter your home Wi-Fi SSID/password, then create and confirm a four-digit Web PIN.
4. Telegram is optional: leave both fields empty to skip it. To enable alerts, create a bot through Telegram’s official BotFather, start a conversation with the bot, and enter the bot token and destination Chat ID. Use **TESTER TELEGRAM** to check them.
5. Save, wait for the reboot and reconnect to your home network. Open the device IP shown in the serial monitor or your router’s DHCP list.
6. Sign in with the PIN, add authorized devices from **RADAR** or manually by MAC, and configure their alerts.
7. Change the public default AP password under **Paramètres → Réseau**.

`USE_MDNS` defaults to **0**: `http://bleradar.local` is unavailable out of the box. Set it to `1` and recompile to enable the configured `.local` hostname on networks supporting mDNS.

The AP shuts down when normal Wi-Fi connects. The existing PIN still protects the interface in fallback mode. Local detection requires no external server; Telegram requires Internet access.

## Change the fallback Wi-Fi password

In **Paramètres → Réseau**, fill in **Nouveau mot de passe du réseau de secours** and its confirmation, then click **ENREGISTRER**. These fields use the existing network settings style and save action.

- Use **8–63 printable ASCII characters**. Browser and server both validate the password and confirmation.
- Leave both fields empty to keep the current password.
- An actual change is saved to NVS and triggers a delayed reboot after the HTTP response. If using the fallback AP, reconnect with the new password.
- The saved password takes precedence over `AP_DEFAULT_PASS`, including after normal reuploads. Changing the constant alone does not overwrite a saved password.
- The home Wi-Fi password has its own separate field, **Mot de passe Wi-Fi (vide = inchangé)**.

Setting `AP_DEFAULT_PASS` to `""` generates a random 12-character password when no valid saved AP password exists. A factory reset clears the stored password; the next boot uses the compiled default or generates a new one in random mode.

## Persistence and recovery

Wi-Fi credentials, Telegram settings and the watchlist are stored on the device, not in the repository. Empty Wi-Fi password/token fields in settings retain their current values. The settings API does not return the Telegram token, but exposes the AP password and Chat ID to authenticated sessions.

Events, visible devices, sessions and notification queues are volatile. Remembered MAC addresses are saved in batches: the newest entries may be lost after sudden power loss.

The interface’s factory reset clears the `bleradar` application namespace, not all flash securely. If the PIN is lost, a full flash erase with Espressif tools followed by a reupload provides recovery, losing all saved data.

## Detection limitations

**A BLE MAC address is not a permanent identity.** Phones and other devices may rotate private/random addresses. A watched device can disappear and return under another address. This firmware does not resolve private identities or bypass address randomization.

Only BLE advertisements can be observed. Classic Bluetooth and silent devices are not detected; missing advertisements do not prove physical absence. Random-address filtering is not perfect device classification.

RSSI varies with antenna orientation, transmit power, walls, people and interference. Distance is approximate, not a location or bearing measurement. One-meter calibration helps but cannot eliminate these effects.

Wi-Fi and BLE share the radio. Alert timing depends on advertisements, confirmations and network/Telegram availability. The four-message Telegram queue drops its oldest entry when full: there is no durable offline queue or guaranteed delivery. This project is not a certified security alarm.

## Security and legitimate use

**`ChangeMe123!` is an intentionally public, generic default. Change it before deployment.** Do not reuse it on other services. Changing a published password does not remove it from Git history.

The interface uses **HTTP**, the PIN has four digits, and the sketch does not configure encrypted NVS/flash. Serial logs may expose sensitive information. Use a trusted local network; do not forward port 80 to the Internet.

**Telegram uses `client.setInsecure()`: its server certificate is not verified.** Traffic is encrypted but server identity is not authenticated, allowing token interception on compromised networks. This original behavior is preserved and explicitly documented. See [SECURITY.md](SECURITY.md), in French, for details and reporting guidance.

Only observe equipment you own or have permission to monitor, in accordance with applicable rules. Do not track people without their knowledge. Telegram alerts may contain MACs, device names, timestamps and distance estimates: restrict recipients and do not publish those records.

## Validation

See [VALIDATION.md](VALIDATION.md) for build and inspection results. No physical-board testing was performed while preparing this package.

After upload, check setup, scanning an authorized device, settings persistence, the three alert types with a test bot, Wi-Fi reconnection and fallback access. Verify AP password changes persist after reboot and the blue LED flashes during scanning, including with an empty watchlist. Only test factory reset when you accept losing the configuration.

## Publish on GitHub

Create an empty repository and upload the **contents** of the extracted folder, including `.gitignore`. GitHub does not automatically unpack an uploaded ZIP; extract it first. The ZIP can also be attached to a release.

Do not add the original unsanitized source, private configurations, NVS exports, unredacted screenshots/logs or binaries containing credentials. `.gitignore` helps prevent accidental additions but cannot remove previously committed secrets.

## License

Distributed under the [MIT License](LICENSE). Separately installed libraries and tools retain their own licenses and are not bundled in this archive.
