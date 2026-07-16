# ESP32 Modbus Gateway — User Manual

Firmware: `ESP32_Modbus_Gateway/ESP32_Modbus_Gateway.ino`
Board: Arduino Nano ESP32 (u-blox NORA-W106)

---

## 1. Overview

This device is a Modbus-to-cloud gateway. It polls Modbus RTU (RS485) and/or
Modbus TCP slaves, exposes the live data on a local web Dashboard, and
forwards it to a master system over one of two uplinks:

- **Raw TCP** — a bidirectional newline-JSON connection to a master/SCADA
  system on your local network.
- **AWS IoT MQTT over TLS** — publishes telemetry to AWS IoT Core, with
  optional two-way commands via AWS IoT Device Shadow.

It also has:
- 4 Digital Inputs, 4 Digital Outputs, and 4 Analog Inputs (local GPIO,
  independent of Modbus).
- An optional SIM7600G-H cellular modem for automatic failover of the MQTT
  uplink when WiFi goes down.
- Store-and-forward buffering so a temporary uplink outage doesn't lose data.

Everything is configured from a web browser — no separate app or CLI tool
is needed.

---

## 2. Hardware / Pinout

| Function | Pin(s) | Notes |
|---|---|---|
| RS485 (Modbus RTU) | RX = D2, TX = D3, DE/RE = D4 | Half-duplex transceiver control on D4 |
| Cellular modem UART | RX = D5, TX = D6 | To SIM7600G-H's UART pins |
| Cellular modem PWRKEY | A5 | Power-on pulse |
| Digital In 1–4 | D7, D8, D9, D10 | See §7 for behavior |
| Digital Out 1–4 | D11, D12, D13, A4 | Driven LOW (off) at boot |
| Analog In 1–4 | A0, A1, A2, A3 | 12-bit ADC (0–4095) |
| WiFi | Onboard | Also runs a permanent Access Point (see §3) |

---

## 3. First-Time Setup

The device always broadcasts its own WiFi Access Point, in addition to (optionally)
joining your existing WiFi network — so you can always reach it even if it's
never been connected to a router.

1. Power on the device.
2. On your phone/laptop, connect to the WiFi network:
   - **SSID**: `ESP32ModbusGateWay` by default (or the device's configured name — see
     Device Identity in §5)
   - **Password**: `12345678`
3. Open a browser to **`http://192.168.4.1`**. This is the Dashboard.
4. Click **Settings** to configure Modbus parameters, WiFi, cloud uplink, etc.
5. To join your site WiFi (so the gateway also gets internet/LAN access),
   set the WiFi SSID/Password under **WiFi & TCP Configuration** in Settings
   and save. The device will connect to that network *in addition to*
   keeping its own AP running.

---

## 4. Dashboard (`/`)

The Dashboard is the live-status home page. It refreshes automatically.

### Top status box
- **WiFi Status** — Connected/Not Connected, plus IP address when connected.
- **MQTT** or **TCP** — connection state and target, depending on Uplink Mode.
- **Poll Interval** — current Modbus poll cadence.
- **Free Heap** — current/minimum-ever free RAM (diagnostic).
- **Buffered (offline)** — only shown when there's an undelivered backlog
  (see §9, Store & Forward).

### Cellular Modem (SIM7600G-H) box
Only meaningful if a modem is wired in. Shows:
- **SIM** — Ready/Not Ready
- **Network** — Registered (with operator name) or Not Registered
- **Signal** — 0–31 (higher is better)
- **Data Session** — whether the cellular data (PDP) context is up
- **MQTT Certs on Modem** / **MQTT Session (cellular)** — only shown in
  MQTT mode; see §8 (Failover)
- **Active MQTT Uplink** — `WiFi (primary)`, `Cellular (failover)`, or a
  live countdown while transitioning — see §8

### Dashboard table
One row per enabled Modbus parameter and TCP-target sensor:
- **Name**, **Source** (which slave/target), **Enabled**, **Value**,
  **Status** (`OK` / `Disconnected` / `Disabled`), **Last Update**.
- **Write** column — for parameters marked writable in Settings, lets you
  type a value and push it directly to the slave (see §7).

### Digital I/O table
Live values for enabled DI/DO/AI channels, with a Control column to toggle
Digital Outputs directly from the browser.

### Status Log
A rolling, human-readable log of key events (connects/disconnects, saves,
writes, errors). See §10 for a full glossary of messages.

---

## 5. Settings (`/settings`)

All settings pages auto-save to flash (NVS) and show a green "Saved
Successfully" banner only once the save is confirmed to have actually
persisted — a red error banner means the write failed (flash may be full)
and the change may not survive a reboot.

### 5.1 Modbus Settings (top of the page)
One row per Modbus parameter, up to 100 rows. Columns:

| Column | Meaning |
|---|---|
| Name | Display name, also the key used for writes/queries/Shadow |
| Transport | **RTU (RS485)** or **TCP (Network)** — see §5.4 |
| Register Area | Holding Register, Input Register, Coil, or Discrete Input |
| Data Type | Scaling/format — see §5.2 (Data Types) |
| Data Length | Auto-filled register count for the selected type (read-only) |
| Device ID / TCP Target | RTU Slave ID, or which configured TCP target (§5.4) |
| Register Address | Modbus register/coil address |
| Writable | Enables the write path for this parameter (Holding Registers and Coils only) |
| Write Min/Max | Allowed value range for writes (rejected outside this range) |
| Enable | Whether this row is polled at all |

Use **Add Record** / **Insert** / **Delete** to manage rows, then
**Save Modbus**.

### 5.2 Data Type Settings (`/types`, linked from Settings)
Defines the scaling types available in the Data Type dropdown above.
Defaults: `INT_16/10`, `INT_16/100`, `INT_16/1000`, `UINT_16/10`,
`UINT_16/100`, `UINT_16/1000`, `FLOAT_16`, `FLOAT_32`. Each type has a
base format (INT16/UINT16/FLOAT16/FLOAT32), a divisor (raw register value
÷ divisor = engineering value), and an implied register length. You can
add up to 20 custom types, or **Reset Data Types** to restore the defaults.

### 5.3 Device Identity
- **Device Name** — an optional short prefix (≤5 chars); the full device
  name becomes `prefix_XXXXX` (last 5 hex chars of the MAC) or just the
  full MAC if left blank. Used as the default MQTT Client ID and shown
  throughout the UI.
- **AP Name (SSID)** — the device's own WiFi hotspot name. Check
  **Match AP Name with Device Name** to keep it automatically in sync with
  Device Name instead of setting it independently.

### 5.4 WiFi & TCP Configuration
- **WiFi SSID / Password** — your site network, for the gateway to join
  as a client (in addition to its own AP).
- **Master IP / Master Port** — target for Raw TCP uplink mode (§6.1).
- **On-Premise NTP Server** — optional local time server, tried before
  public NTP (`pool.ntp.org`, then `time.nist.gov`).

### 5.5 Cloud Uplink
- **Uplink Mode** — **Raw TCP** or **AWS IoT MQTT over TLS** (§6).
- MQTT fields (only used in MQTT mode): **Endpoint**, **Port**,
  **Client ID** (blank = Device Name), **Publish Topic**.
- **Device Shadow Commands** — enables the two-way AWS IoT Shadow command
  channel (§7.2).
- **Root CA / Device Certificate / Private Key** — paste PEM text or
  upload the files from AWS IoT. Fields are never echoed back for
  security; leave blank to keep the currently-stored value.

### 5.6 Modbus TCP Targets
Up to 4 remote Modbus TCP servers (e.g. a CNC controller) polled over
WiFi as a second data source, alongside the RS485 bus. Configure
Name/IP/Port/Unit ID here, then reference the target by name from a
parameter row's "TCP Target" column (§5.1).

### 5.7 Digital I/O
Configure the 4 DI / 4 DO / 4 AI local channels:
- **Digital In / Digital Out** — Name + Enable per channel.
- **Analog In** — Name + Enable + **Scale** + **Offset**. Engineering
  value = raw ADC (0–4095) × Scale + Offset.

### 5.8 Cellular Modem
- **APN** — required by some carriers for the SIM7600G-H's data session;
  leave blank to let the modem auto-provision from the SIM. If the
  Dashboard's Data Session never comes up, set this explicitly.

### 5.9 RS485 Communication Settings
- **Baud Rate**, **Parity** (None/Even/Odd), **Stop Bits** (1/2).
- **Poll Interval (ms)** — delay between full poll cycles (default 2000ms).
- **Slave Recovery Delay (ms)** — pause after each parameter before
  polling the next; raise this if parameters start erroring right after a
  fast one (default 100ms).

---

## 6. Cloud Connectivity Modes

Set under Settings → Cloud Uplink → Uplink Mode.

### 6.1 Raw TCP
The gateway connects **out** to your Master IP/Port as a TCP client and
sends newline-terminated JSON telemetry every poll cycle:
```json
{"device_id":"...","timestamp":1234567890,"time_source":"ntp","sensors":[{"sensor_id":"1","sensor_name":"Temp1","value":42.3,"status":"OK"}, ...]}
```
The same connection is bidirectional — your master can send commands back
down the same socket (see §7.3).

### 6.2 AWS IoT MQTT over TLS
The gateway connects to your AWS IoT endpoint using mutual TLS (device
cert + private key + Root CA, configured in Settings) and publishes the
same telemetry JSON shape to your configured topic on every poll cycle.
If **Device Shadow Commands** is enabled, it also subscribes to
`$aws/things/<Client ID>/shadow/update/delta` for inbound writes (§7.2).

---

## 7. Remote Write / Command Channels

Any Modbus parameter marked **Writable** (Holding Register or Coil), or
any enabled Digital Out channel, can be written through three independent
channels — all validated the same way (range-checked against Write
Min/Max, then queued and executed on the next Modbus cycle):

### 7.1 Dashboard
Type a value into the Write column (Modbus table) or use the toggle
control (Digital I/O table) and it's queued immediately.

### 7.2 AWS IoT Device Shadow (MQTT mode only)
Publish a delta with the parameter name as the key:
```json
{"state":{"desired":{"SetpointTemp":42}}}
```
The device reads the shadow's `delta` (what changed), executes the write,
and reports the new state back to `.../shadow/update`. Non-numeric or
unrecognized keys are logged and skipped, not fatal to the rest of the
delta.

### 7.3 Raw TCP command channel
Over the same bidirectional Raw TCP connection (§6.1), send a
newline-terminated JSON line:
- **Write**: `{"param":"SetpointTemp","value":42}\n`
- **On-demand read**: `{"query":"SetpointTemp"}\n` (answered immediately,
  separate from the periodic telemetry broadcast)

Every write gets an ack line back:
```json
{"ack":"SetpointTemp","value":42,"ok":true,"code":0}
```
(`ok:false` with a `"reason"` field if rejected or if the Modbus write
itself failed — `code` is the Modbus exception/result code.)

---

## 8. WiFi ↔ Cellular Failover (MQTT mode only)

If a SIM7600G-H modem is fitted and MQTT mode + certs are configured, the
gateway automatically fails the **MQTT uplink only** over to cellular when
WiFi is lost — **Raw TCP mode is never affected**, since it targets a
private LAN IP that cellular data can't route to.

- **Failover**: if WiFi is continuously down for **3 minutes**, the
  gateway brings up the modem's onboard MQTT session and starts publishing
  telemetry over cellular instead. (Cellular data is billed/metered, so
  this only activates as a genuine fallback, not routinely.)
- **Failback**: once WiFi has been continuously connected and stable for
  another 3 minutes, the gateway switches back to WiFi and tears down the
  cellular MQTT session (kept warm at the lower level for a fast reconnect
  next time).
- Both directions use the same 3-minute grace period, to avoid flapping
  back and forth on a marginal connection.
- The Dashboard's **Active MQTT Uplink** line shows live status,
  including a countdown while waiting to fail over or fail back, and
  flags an unstable/flapping WiFi connection ("WiFi still down" repeating
  instead of climbing).
- Cellular telemetry publishes on its own 5-second cycle, independent of
  the Modbus poll interval.

---

## 9. Store & Forward

If the active uplink (Raw TCP or WiFi MQTT) is unreachable, undelivered
payloads are buffered in RAM (up to 64 payloads / 32KB) and replayed in
order once the uplink recovers. The Dashboard shows **Buffered (offline)**
whenever there's a backlog. This buffering pauses automatically while
cellular MQTT failover is active, since cellular is independently
delivering fresh data during that window (see §8).

---

## 10. Status Log Message Glossary

The Status Log (Dashboard) shows a curated history of key events. Common
messages, grouped by area:

**Boot / Tasks**
`BOOT`, `AP STARTED: <ssid> <ip>`, `WEB SERVER STARTED`,
`MODBUS POLLING STARTED`, `MODBUS TCP POLLING STARTED`,
`CELLULAR MODEM TASK STARTED`, `NTP TIME SYNCED`

**Polling**
`POLL: X/Y OK[, Z ERR]` — per-cycle summary. `SLAVE <id> NOT RESPONDING —
pausing its polls for 30s` / `SLAVE <id> BACK ONLINE` — per-slave backoff.
Same pattern for `TCP TARGET <name> NOT RESPONDING` / `BACK ONLINE`.

**WiFi / Uplink**
`WIFI CONNECTED: <ip>` / `WIFI DISCONNECTED` / `WIFI NOT CONNECTED -
retrying`, `TCP CONNECTED` / `TCP CONNECT FAILED` / `TCP DISCONNECTED`,
`MQTT CONNECTED` / `MQTT CONNECT FAILED` / `MQTT DISCONNECTED`,
`MQTT CERTS NOT CONFIGURED`, `MQTT WAITING FOR NTP TIME SYNC`.

**Failover (§8)**
`UPLINK FAILOVER: MQTT switching to cellular (WiFi down 3+ min)`,
`UPLINK FAILBACK: MQTT switching back to WiFi (stable 3+ min)`,
`CELLULAR MQTT SESSION UP/DOWN`, `CELLULAR MQTT SESSION FAILED (<step>):
<detail>`, `CELLULAR MQTT PUBLISH FAILED`, `CELLULAR MQTT CERT
PROVISIONING START/OK`, `CELLULAR SIM READY/NOT READY`,
`CELLULAR NETWORK REGISTERED/LOST`, `CELLULAR DATA SESSION UP/DOWN`,
`CELLULAR MODEM RESPONDING/NOT RESPONDING`.

**Writes**
`WRITE QUEUED (<source>): <name> = <value>`, `WRITE OK` / `WRITE FAILED
code <n>`, `WRITE REJECTED (<source>): <reason>` (not writable, out of
range, invalid index), `WRITE QUEUE FULL`. Same pattern with `DO WRITE
...` for Digital Out. `SHADOW DELTA REJECTED/PARSE ERROR`, `TCP CMD
REJECTED/PARSE ERROR`, `TCP QUERY REJECTED`.

**Store & Forward**
`UPLINK DOWN - buffering data (N payloads, M bytes queued)`,
`STORE&FORWARD: replayed N payload(s)[, M still queued]`.

**Settings saves**
`<SECTION> SETTINGS SAVED: ...` on success; `NVS SAVE INCOMPLETE
(<section>): N write(s) failed - flash may be full` if a save partially
failed.

**Diagnostics**
`FREE HEAP: <bytes> (min ever since boot: <bytes>)` — logged periodically.

---

## 11. Troubleshooting

| Symptom | Check |
|---|---|
| Can't reach `192.168.4.1` | Confirm connected to the device's own AP (SSID/password in §3), not your site WiFi |
| Modbus parameter always shows "Disconnected" | Check Slave ID / Register Address / Data Type, wiring, and baud/parity/stop bits match the slave |
| Settings won't save / red error banner | Flash may be full — check Status Log for `NVS SAVE INCOMPLETE`; reduce configured rows/certs if persistent |
| MQTT never connects | Confirm Endpoint/Port, that all 3 cert fields are populated, and that NTP has synced (`MQTT WAITING FOR NTP TIME SYNC` means the clock isn't set yet) |
| Write rejected | Check the parameter is marked Writable and the value is within Write Min/Max in Settings |
| Cellular Data Session never comes up | Set the APN explicitly under Settings → Cellular Modem |
| Cellular failback never happens despite WiFi showing Connected | Watch the Dashboard's "Active MQTT Uplink" countdown — if the elapsed-seconds figure keeps resetting instead of climbing to 180s, WiFi is flapping (weak signal/AP issue), not a firmware fault |
