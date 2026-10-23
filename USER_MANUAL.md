# Modbus Gateway — User Manual

---

## 1. Overview

This device is a Modbus-to-cloud gateway. It polls Modbus RTU (RS485) and/or
Modbus TCP slaves, exposes the live data on a local web Dashboard, and
forwards it to a master system over one of two uplinks:

- **Raw TCP** — a bidirectional newline-JSON connection to a master/SCADA
  system on your local network.
- **MQTT over TLS** — publishes telemetry to any MQTT broker (AWS IoT
  Core, Azure IoT Hub, Mosquitto, …), with an optional two-way command
  topic using plain, cloud-agnostic MQTT (no AWS-specific features).

It also has:
- 4 Digital Inputs, 4 Digital Outputs, and 4 Analog Inputs (local GPIO,
  independent of Modbus).
- An optional SIM7600G-H cellular modem for automatic failover of the MQTT
  uplink and email alerts when WiFi goes down.
- Store-and-forward buffering so a temporary uplink outage doesn't lose data.
- Email alerts when a Modbus parameter leaves its limits or stops updating
  (§10).

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
   - **SSID**: `ModbusGateway` by default (or the device's configured name — see
     Device Identity in §5)
   - **Password**: `12345678` by default (changeable — see §12)
3. Open a browser to **`http://192.168.4.1`** and log in:
   - **Username**: `admin` / **Password**: `admin` (defaults)
   - A red banner reminds you to change this until you do — set your own
     under **Settings → Security** (see §12).
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
- **Email Alerts / Active alarms** — whether email alerts are on, emails
  waiting to send, and how many parameters are currently in alarm (§10).

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
  **Status** (`OK` / `Disconnected` / `Disabled`, plus `ALARM HIGH/LOW/NO
  DATA` in red while a parameter is in alarm — §10), **Last Update**.
- **Write** column — for parameters marked writable in Settings, lets you
  type a value and push it directly to the slave (see §7).

### Digital I/O table
Live values for enabled DI/DO/AI channels, with a Control column to toggle
Digital Outputs directly from the browser.

### Status Log
A rolling, human-readable log of key events (connects/disconnects, saves,
writes, errors). See §11 for a full glossary of messages.

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
| Name | Display name, also the key used for writes/queries over MQTT/TCP |
| Transport | **RTU (RS485)** or **TCP (Network)** — see §5.4 |
| Register Area | Holding Register, Input Register, Coil, or Discrete Input |
| Data Type | Scaling/format — see §5.2 (Data Types) |
| Data Length | Auto-filled register count for the selected type (read-only) |
| Device ID / TCP Target | RTU Slave ID, or which configured TCP target (§5.4) |
| Register Address | Modbus register/coil address |
| Writable | Enables the write path for this parameter (Holding Registers and Coils only) |
| Write Min/Max | Allowed value range for writes (rejected outside this range) |
| Email Alert: Low / High | Tick to email on this parameter; alarm below Low or above High. Leave either blank for no limit on that side — see §10 |
| Deadband / Delay (s) | Hysteresis and persistence for the alert — see §10 |
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
- **AP Password** — WPA2 key for the device's own hotspot (min 8 chars;
  blank keeps the current one). Default `12345678` — change it. Forgot it?
  See §12 (recovery).

### 5.3a Security
- **Web Username / New Web Password** — the login for this web UI
  (defaults `admin`/`admin`; a red banner nags until the password is
  changed). Blank password fields keep the current password.
- Sessions time out after 30 minutes idle; 5 failed logins lock the login
  page for 60 seconds. See §12 for the full security model and recovery.

### 5.4 WiFi & TCP Configuration
- **WiFi SSID / Password** — your site network, for the gateway to join
  as a client (in addition to its own AP). A blank password field keeps
  the saved password.
- **Open network (no password)** — tick this to join an open SSID. It
  clears any saved WiFi password (a blank password field alone can't).
  Open networks are unencrypted, so the web UI and Modbus writes cross
  them in plain text; MQTT stays TLS-encrypted.
- **Device WiFi MAC** — the address your router sees when the gateway
  joins as a client. On MAC-filtered networks, add this address to the
  router's allow list. The device's own hotspot uses a different MAC,
  which the router never sees. In the router's client list the device
  appears as `Gateway-XXXXXX`, where `XXXXXX` is the last six characters
  of this MAC.
- **Master IP / Master Port** — target for Raw TCP uplink mode (§6.1).
- **On-Premise NTP Server** — optional local time server, tried before
  public NTP (`pool.ntp.org`, then `time.nist.gov`).

### 5.5 Cloud Uplink
- **Uplink Mode** — **Raw TCP** or **MQTT over TLS (AWS IoT / any broker)** (§6).
- MQTT fields (only used in MQTT mode): **Endpoint**, **Port**,
  **Client ID** (blank = Device Name), **Publish Topic**.
- **MQTT Command Topic** — enables the two-way MQTT command channel
  (§7.2). The exact subscribe/ack topic names are shown next to the
  checkbox.
- **Root CA / Device Certificate / Private Key** — paste PEM text or
  upload the files from your broker/CA. Fields are never echoed back for
  security; leave blank to keep the currently-stored value.

### 5.5a Email Alerts
Mail server, recipients and alert behaviour for email alerts. Which
parameters alarm, and at what limits, is set per row in the Modbus Settings
table (§5.1). See §10 for the full guide.

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
- **Publish Interval (s)** — how often telemetry is published while the
  MQTT uplink is failed over to cellular (default 5s, range 5–3600s).
  Raise it to reduce SIM data usage. It does not affect WiFi publishing
  (which follows the Modbus poll interval) or how quickly inbound
  commands are handled during failover.

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

### 6.2 MQTT over TLS
The gateway connects to your MQTT endpoint using mutual TLS (device
cert + private key + Root CA, configured in Settings) and publishes the
same telemetry JSON shape to your configured topic on every poll cycle.
Works with AWS IoT Core or any other broker that accepts
certificate-based MQTT/TLS. If **MQTT Command Topic** is enabled, it also
subscribes to `<Client ID>/commands` for inbound writes/reads (§7.2).

---

## 7. Remote Write / Command Channels

Any Modbus parameter marked **Writable** (Holding Register or Coil), or
any enabled Digital Out channel, can be written through three independent
channels — all validated the same way (range-checked against Write
Min/Max, then queued and executed on the next Modbus cycle):

### 7.1 Dashboard
Type a value into the Write column (Modbus table) or use the toggle
control (Digital I/O table) and it's queued immediately.

### 7.2 MQTT command topic (MQTT mode only)
Publish one JSON command per message to `<Client ID>/commands` — the same
command shapes as the raw TCP channel:
- **Write**: `{"param":"SetpointTemp","value":42}`
- **On-demand read**: `{"query":"SetpointTemp"}`

The device replies on `<Client ID>/commands/ack` with the same ack/query
response JSON shapes as the raw TCP channel (§7.3), so a master can share
its parsing code between both transports. These are plain MQTT topics —
no AWS-specific features — so the channel works unchanged against AWS IoT
Core, Azure IoT Hub, Mosquitto, or any other broker.

The command channel follows the active uplink: over WiFi normally, and
over the cellular modem during a failover (§8) — same topics, same JSON,
no changes needed on the master side. During failover, allow a few
seconds' extra latency per command (cellular round trips), and treat a
missing ack as "retry the command".

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
  gateway brings up the modem's onboard MQTT session, re-subscribes to the
  command topic, and continues both telemetry *and* two-way commands over
  cellular. (Cellular data is billed/metered, so this only activates as a
  genuine fallback, not routinely.)
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
- Cellular telemetry publishes on its own configurable interval (default
  5s — Settings → Cellular Modem → Publish Interval), independent of the
  Modbus poll interval.

---

## 9. Store & Forward

If the active uplink (Raw TCP or WiFi MQTT) is unreachable, undelivered
payloads are buffered in RAM (up to 64 payloads / 32KB) and replayed in
order once the uplink recovers. The Dashboard shows **Buffered (offline)**
whenever there's a backlog. This buffering pauses automatically while
cellular MQTT failover is active, since cellular is independently
delivering fresh data during that window (see §8).

---

## 10. Email Alerts

The gateway can email up to four people when a Modbus parameter — RTU or
TCP — goes outside its limits, stops updating, or returns to normal. Email
goes out over WiFi, or over the SIM7600 cellular modem when WiFi can't
deliver it.

### 10.1 Quick start

1. **Settings → Email Alerts:** fill in the SMTP server, login, From address
   and at least one recipient (see §10.2), tick **Email Alerts: Enabled**,
   and press **Save Email Settings**.
2. Press **Send Test Email**, wait about 30 seconds, then refresh the page.
   The result appears under the button. Fix anything it reports (§10.7)
   before going further.
3. **Settings → Modbus Settings table:** on each parameter to watch, tick
   **Email Alert** and set its **Low** and/or **High** limit (§10.3). Press
   **Save Modbus**.
4. Check the Dashboard: an out-of-limits parameter shows **ALARM HIGH/LOW/NO
   DATA** in red in its Status column, and the Status Log shows each alarm
   and each email sent.

Email Alerts is off on a new gateway, and every parameter starts with its
alert unticked — including after a firmware update from a version without
email alerts. Nothing is emailed until you set it up.

### 10.2 Mail server settings (Settings → Email Alerts)

| Field | Notes |
|---|---|
| Email Alerts | Master switch. While off, alarms are still tracked and shown on the Dashboard and Status Log, but nothing is emailed, and switching it off discards any emails still waiting to send. |
| SMTP Server | Outgoing mail server, e.g. `smtp.office365.com`. Up to 64 characters. |
| Security | **STARTTLS** (port 587, the usual choice), **SSL/TLS** (port 465), or **None** (port 25). Changing Security moves the Port to match, unless you have typed a non-standard port. **None** sends the password and every alert unencrypted — only for an internal relay that doesn't support TLS. |
| SMTP Port | Normally set by Security. |
| Username | The mail account to log in with. Leave blank for a relay that needs no login (no password is then sent or kept). |
| Password | Write-only: never shown again after saving. A blank field keeps the saved password; tick **Clear saved password** to remove it. |
| From Address | The sender. Most servers reject mail whose From doesn't match the login account. |
| Recipients 1–4 | Every recipient gets every alert, in one email addressed to all of them. At least one is required. |
| Subject Prefix | Starts every subject in brackets, e.g. `[Boiler House]`, so recipients can tell gateways apart and filter on it. Up to 32 characters. Blank = the Device Name. |
| Reminder Interval (min) | Re-sends an alarm that is still active after this many minutes, then again every interval. 0 = never. Default 60, maximum 1440 (24 h). |
| Cellular | **Send over cellular when WiFi is down.** On by default. Uses SIM data — roughly 10–20 KB per email. See §10.6. |
| Return to Normal | Also email when an alarm clears. On by default. |
| No-Data Alarms | Alarm when an alert-enabled parameter has had no valid reading for this many seconds (0–3600, default 60). See §10.4. |

If a save is rejected, a red banner says why and **nothing** is changed:
`SMTP server is missing or invalid`, `From address is not a valid email
address`, `at least one recipient is required`, `a recipient is not a valid
email address`, or `a field is too long (max 64 characters)`. The address
checks only apply while Email Alerts is ticked, so you can save a
half-finished setup with alerts off.

**Common setups:**

| Mail service | Server | Security / Port | Notes |
|---|---|---|---|
| Microsoft 365 / Office 365 | `smtp.office365.com` | STARTTLS / 587 | The mailbox must have **Authenticated SMTP** (SMTP AUTH) allowed; many company tenants switch it off. If login fails with the correct password (`server said 535`), ask your IT team to allow SMTP AUTH for the sending mailbox, or for an internal relay. From must be the login mailbox. |
| Gmail / Google Workspace | `smtp.gmail.com` | STARTTLS / 587 | Needs an **app password** (Google account → Security → 2-Step Verification → App passwords), not the normal password. |
| Site's internal mail relay | Your relay's name or IP | Usually None / 25, or STARTTLS / 587 | Often needs no login: leave Username blank. **A relay on the site network can't be reached over cellular** — see §10.6. |

Use a dedicated mailbox for the gateway rather than a person's account: its
password is stored on the device (§12).

### 10.3 Choosing which parameters alarm (Modbus Settings table)

Each row in the Modbus Settings table has two alert columns:

| Column | Field | Meaning |
|---|---|---|
| **Email Alert: Low / High** | tick box | Watch this parameter. The row must also be **Enabled** (polled) — a row that isn't polled never alarms. |
| | Low, High | Alarm when the value goes **below Low** or **above High**. Either can be left blank for no limit on that side; with both blank, the row can only raise NO DATA alarms. Limits are in the same engineering units the Dashboard shows (after the Data Type's scaling). |
| **Deadband / Delay (s)** | Deadband | Hysteresis. A HIGH alarm only clears once the value is back down to **High − Deadband** or below; a LOW alarm once it is back up to **Low + Deadband** or above. Stops a value hovering on the limit from sending a stream of alarm/clear emails. 0 = clear as soon as it's back inside the limit. |
| | Delay | How long a breach must last, without a break, before it becomes an alarm — and how long a recovery must last before the alarm clears. Filters out one-poll spikes. 0–3600 s. |

For a Coil or Discrete Input (value 0 or 1): High = `0.5` alarms when it reads
1; Low = `0.5` alarms when it reads 0.

**Worked example** — a tank temperature, High = 80, Deadband = 2, Delay = 10:

| Time | Reading | What happens |
|---|---|---|
| 08:32:51 | 80.4 | Above 80: the 10 s delay starts. Nothing shown yet. |
| 08:33:01 | 81.2 | Still above 80 after 10 s: **ALARM HIGH**. Red on the Dashboard, logged. |
| ~08:33:12 | — | Email sent (alerts wait 10 s to be grouped — §10.5). |
| 08:50:00 | 79.0 | Below 80 but not below 78: **still in alarm** (deadband). |
| 09:10:00 | 77.5 | At or below 78: the 10 s clear delay starts. |
| 09:10:10 | 77.0 | Stayed down for 10 s: **CLEARED**, and a Return-to-Normal email. |

Had the reading gone back above 78 at any point during the clear delay, the
alarm would have stayed active and the delay would start again next time.

### 10.4 How alarms behave

- **HIGH, LOW and NO DATA.** HIGH and LOW come from the limits. NO DATA
  means the parameter hasn't had a valid reading — the slave isn't
  responding, or the read returned an error — for the No-Data time (default
  60 s). That time is gateway-wide rather than per row because every
  parameter reads as invalid briefly at boot, and for about 30 s whenever a
  slave goes into its not-responding pause; a short per-row delay would
  turn both into false alarms. Turn **No-Data Alarms** off to have only
  limit alarms.
- **A slave dropping off the bus** gives one NO DATA alarm per alert-enabled
  parameter on it, all in a single email (§10.5). When it answers again,
  each one clears with a CLEARED email reading `readings are back - device
  responding again` (after the row's Delay, if set).
- **Losing data during a HIGH or LOW alarm** doesn't clear it or turn it
  into NO DATA. The limit breach stays the alarm being reported until a
  valid reading shows the value back inside its limit.
- **Jumping straight from HIGH to LOW** (or data returning to a NO DATA
  alarm while out of limits) is reported as a new alarm.
- **Reminders** repeat a still-active alarm every Reminder Interval, as a
  `STILL IN ALARM` email.
- **Saving the Modbus Settings table keeps active alarms.** Each one stays
  with the row of the same **Name**, even if rows were added, deleted or
  moved, and clears with a CLEARED email as usual. So you can correct a
  Slave ID, address or limit while an alarm is open: once readings are
  back inside the limits, recipients get the CLEARED email. Removing a
  row's High or Low limit while that alarm is active clears it at the next
  reading.
- **Renaming a row, unticking its Email Alert, or disabling the row,**
  drops its alarm silently: no CLEARED email. A renamed row still out of
  limits alarms afresh.
- **Restarting the gateway** (power cycle or Reboot) starts every alarm
  from normal. A parameter still out of limits alarms again after its
  delay, with a new ALARM email.
- **Switching Email Alerts on** while alarms are already active doesn't
  email them straight away. Each one is emailed at its next reminder, or
  when it clears.
- Alarms are always tracked — Dashboard and Status Log — even while Email
  Alerts is off.

### 10.5 What the emails look like

Alarms raised within **10 seconds** of each other are grouped into one
email, so a slave dropping off the bus sends one message listing all of
its parameters rather than one each. An email therefore goes out about
10 seconds after the first alarm in it, plus the time to send. One email
lists up to 12 events; any beyond that are counted at the end.

The subject names the most urgent event — new alarms first, then
reminders, then clears — and how many others the email contains:

```
[Boiler House] ALARM: TankTemp HIGH (+1 more)
[Boiler House] STILL IN ALARM: TankTemp HIGH
[Boiler House] CLEARED: TankTemp
[Boiler House] Test email
```

A typical body:

```
Modbus Gateway alert from GW01_A1B2C
Sent: 2026-10-23 08:33:12 UTC

ALARM     TankTemp: HIGH - value 81.200, high limit 80.000 [RTU slave 1] at 2026-10-23 08:33:01 UTC
ALARM     Flow2: NO DATA - no valid reading (device not responding or read error) [TCP target CNC] at 2026-10-23 08:33:04 UTC

Active alarms on the gateway right now: 2

Sent automatically by the gateway. Alarm limits are set in the Modbus Settings table; email settings under Settings > Email Alerts.
```

Each event line gives the parameter, the condition, the reading and limit,
where it comes from (RTU slave number or TCP target name), and **when it
happened** — which matters for an email that was held back during an
outage (§10.6). Cleared events read `CLEARED   TankTemp: back to normal
(was HIGH) - value 77.000 ...` — or, for a device that has started
answering again, `CLEARED   Flow2: readings are back - device responding
again - value 3.200 ...` — and reminders start `STILL IN`.

**All times are UTC** (India Standard Time is UTC + 5:30). Before the
gateway has the correct time they show as `uptime N s (clock not synced)`.
Names are sent in plain ASCII; any other character appears as `?`.

### 10.6 Delivery: WiFi, cellular, queue and retries

- **WiFi first.** Whenever WiFi is connected, email goes over WiFi.
- **Cellular when WiFi can't deliver.** If WiFi is down — or WiFi is up but
  can't even reach the mail server, e.g. the site's internet is out — the
  email goes over the SIM7600 modem instead, with the same secure
  connection and certificate checks. This doesn't wait for the 3-minute
  MQTT failover (§8); it's used as soon as WiFi can't deliver, as long as
  **Cellular** is ticked and the Dashboard shows the cellular **Data
  Session: Connected**. While a cellular email is sending (usually 10–20 s),
  cellular MQTT pauses, then carries on.
- **Over cellular, only internet mail servers work.** The SIM is on the
  public internet, not your site network, so an internal relay with a
  private address can't be reached over cellular. If you use one, alerts
  still wait for WiFi.
- **Waiting.** If neither path is available, emails wait in a queue (up to
  8 emails) and go out as soon as one is, in order. A waiting email doesn't
  use up its retries. If the queue fills, further alerts are logged as
  `EMAIL QUEUE FULL` and not emailed.
- **Retries.** A send that fails is retried twice more — after 30 s, then
  60 s — and then dropped, with `EMAIL DROPPED after 3 attempts` and the
  last error in the Status Log.
- **The correct time.** Secure email checks the mail server's certificate
  dates, so it needs the right time. The gateway normally gets it from the
  internet over WiFi. If it starts up while WiFi is down (for example after
  a power cut), it gets the time through the cellular network instead and
  logs `CLOCK SET FROM CELLULAR NETWORK`. Until it has the time, secure
  emails wait.

The Dashboard's top box shows whether email alerts are on, how many emails
are waiting to send, and the number of active alarms. The Email Alerts box
in Settings shows the queue and the totals sent and dropped since boot.

### 10.7 Testing and fixing problems

**Send Test Email** sends a real email exactly the way an alert would — same
server, security and recipients — using the **saved** settings, so save
first. It works while Email Alerts is still switched off. **Test via
Cellular** forces the modem, even while WiFi is up and even with the
Cellular box unticked, so you can prove cellular email works during
commissioning without unplugging anything. About 30 seconds later, refresh
the page: the result is shown under the buttons (it reads `sending...` until
then), and also appears in the Status Log as `EMAIL TEST: ...`.

A successful test reads `OK via WiFi - sent to 2 recipients` (or `via
cellular`). A failure names the step that failed:

| Result says | What to check |
|---|---|
| failed at DNS lookup | SMTP server name spelled correctly, and the site network has DNS/internet access |
| failed at TCP connect | Port number, and that the site firewall allows outbound mail on that port. Over cellular, some mobile networks block port 25 — use 587 or 465 |
| failed at TLS handshake | Security matches the port (587 = STARTTLS, 465 = SSL/TLS) |
| failed at STARTTLS | The server doesn't offer STARTTLS on this port: try SSL/TLS on 465 |
| failed at authentication: server said 535 | Username/password. Office 365: SMTP AUTH may be off for the mailbox; Gmail: use an app password (§10.2) |
| failed at sender address | From must be the login account's address |
| failed at recipient address | Every recipient was refused — check the addresses |
| not sent: waiting for a valid clock | The gateway doesn't have the correct time yet (§10.6) |
| not sent: WiFi is not connected and the cellular data session is not up | Neither path is available right now |
| not sent: the cellular data session is not up | (Test via Cellular) Check the Dashboard's Cellular Modem box: SIM, Network, Data Session; set the APN (§5.8) |

If the test says OK but nothing arrives, look in the recipients' spam or
junk folder, and ask IT whether mail from the gateway's mailbox is being
quarantined.

---

## 11. Status Log Message Glossary

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
PROVISIONING START/OK`, `CELLULAR MQTT CMD SUBSCRIBED/SUBSCRIBE FAILED`,
`CELLULAR MQTT CONNECTION LOST (modem URC)`, `CELLULAR MQTT CMD IGNORED/
RX ABORTED`, `CELLULAR SIM READY/NOT READY`,
`CELLULAR NETWORK REGISTERED/LOST`, `CELLULAR DATA SESSION UP/DOWN`,
`CELLULAR MODEM RESPONDING/NOT RESPONDING`.

**Writes**
`WRITE QUEUED (<source>): <name> = <value>`, `WRITE OK` / `WRITE FAILED
code <n>`, `WRITE REJECTED (<source>): <reason>` (not writable, out of
range, invalid index), `WRITE QUEUE FULL`. Same pattern with `DO WRITE
...` for Digital Out. `MQTT CMD SUBSCRIBED/REJECTED/PARSE ERROR`,
`MQTT ACK PUBLISH FAILED`, `TCP CMD REJECTED/PARSE ERROR`,
`TCP QUERY REJECTED` / `MQTT QUERY REJECTED`.

**Store & Forward**
`UPLINK DOWN - buffering data (N payloads, M bytes queued)`,
`STORE&FORWARD: replayed N payload(s)[, M still queued]`.

**Settings saves**
`<SECTION> SETTINGS SAVED: ...` on success; `NVS SAVE INCOMPLETE
(<section>): N write(s) failed - flash may be full` if a save partially
failed.

**Email alerts**
`ALARM HIGH/LOW/NO DATA: <name> = <value> (high/low limit <n>)`,
`CLEARED HIGH/LOW/NO DATA: <name>`, `EMAIL SENT via WiFi/cellular:
<subject> (sent to N recipients)`, `EMAIL FAILED via WiFi/cellular (attempt
N/3), will retry: failed at <step>...`, `EMAIL DROPPED after 3 attempts`,
`EMAIL QUEUE FULL (8 emails waiting) - alert for <name> not emailed`,
`EMAIL TEST: OK/FAILED via WiFi/cellular - ...`,
`EMAIL SETTINGS SAVED`, `EMAIL ALERT TASK STARTED`,
`CLOCK SET FROM CELLULAR NETWORK: <time> (modem NTP / network time)`,
`CELLULAR NETWORK TIME NOT AVAILABLE - retrying in 10 min`.

**Security**
`WEB LOGIN OK: <user>`, `WEB LOGIN FAILED (attempt N/5)`,
`WEB LOGIN LOCKED OUT (60s after N failures)`,
`SECURITY SETTINGS SAVED: web user=<user> (password changed/unchanged)`.

**Diagnostics**
`FREE HEAP: <bytes> (min ever since boot: <bytes>)` — logged periodically.

---

## 12. Security & Recovery

- **Everything requires login.** Every page *and* every save/write/reset
  endpoint checks the session — there is no way to change settings or
  actuate outputs over HTTP without being logged in. Log out with the
  **Log Out** button (Dashboard or Settings).
- **Login protection**: sessions are random per-login tokens
  (HttpOnly cookie), idle out after 30 minutes, and a new login replaces
  any previous session. Five failed login attempts lock the login page
  for 60 seconds (`WEB LOGIN LOCKED OUT` in the Status Log).
- **Credentials storage**: the web password is stored as a salted SHA-256
  hash, never as plain text. Password fields in the UI never echo stored
  values.
- **The email (SMTP) password** can't be hashed — the gateway has to send
  it to the mail server — so it is stored readable in the device's flash.
  It is never shown in the UI, never written to the Status Log, and only
  ever sent to the mail server over the encrypted connection (unless
  Security is set to None). It crosses your local network unencrypted once,
  when you type it into the Settings page. Use a dedicated mailbox for the
  gateway, not a person's account.
- **Forgot the web login or AP password?** Connect the device over USB,
  open a serial monitor at 115200 baud, and send:
  ```
  factory-reset-auth
  ```
  This restores the default web login (`admin`/`admin`) and default AP
  password (`12345678`), then reboots. **Nothing else is touched** — all
  Modbus parameters, data types, certificates, and cloud/cellular
  configuration survive. (Physical USB access is required, which is the
  point: anyone with the cable could re-flash the board anyway.)
- **Honest limitation**: the web UI itself is plain HTTP, so the login
  transits unencrypted on your local network/AP. This is the standard
  posture for LAN-only device UIs; treat the AP password and your site
  LAN as the perimeter. The cloud uplink (MQTT) is fully TLS-encrypted
  and unaffected.

## 13. Troubleshooting

| Symptom | Check |
|---|---|
| Can't reach `192.168.4.1` | Confirm connected to the device's own AP (SSID/password in §3), not your site WiFi |
| Modbus parameter always shows "Disconnected" | Check Slave ID / Register Address / Data Type, wiring, and baud/parity/stop bits match the slave |
| Settings won't save / red error banner | Flash may be full — check Status Log for `NVS SAVE INCOMPLETE`; reduce configured rows/certs if persistent |
| MQTT never connects | Confirm Endpoint/Port, that all 3 cert fields are populated, and that NTP has synced (`MQTT WAITING FOR NTP TIME SYNC` means the clock isn't set yet) |
| Write rejected | Check the parameter is marked Writable and the value is within Write Min/Max in Settings |
| Cellular Data Session never comes up | Set the APN explicitly under Settings → Cellular Modem |
| Test email works but no alert emails arrive | Email Alerts ticked and saved? The row's **Email Alert** ticked and the row **Enabled**? Does the Status Log show `ALARM ...` (alarm raised) and `EMAIL SENT ...`? If sent, check spam/junk folders |
| Alert emails arrive late | WiFi down and **Cellular** unticked, the data session down, or the gateway waiting for the correct time — the Dashboard top box shows emails waiting (§10.6) |
| Too many alarm/clear emails | Raise the row's **Deadband** and/or **Delay**, or lengthen the **Reminder Interval** (§10.3) |
| NO DATA alarms after every restart | The slave takes longer than the No-Data time to answer after power-up: raise **No-Data Alarms** seconds (§10.4) |
| Cellular email fails at TCP connect | The mobile network may block that port — use 587 or 465. An internal relay can't be reached over cellular at all (§10.6) |
| Cellular failback never happens despite WiFi showing Connected | Watch the Dashboard's "Active MQTT Uplink" countdown — if the elapsed-seconds figure keeps resetting instead of climbing to 180s, WiFi is flapping (weak signal/AP issue), not a firmware fault |
