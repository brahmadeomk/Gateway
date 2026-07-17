/* ======================================================================
   ARCHITECTURE NOTES
   ======================================================================
   Modbus RTU polling is the real-time-critical part of this gateway, so
   the whole task/core layout is built around protecting it:

   - modbusTask runs alone on Core 1 at MODBUS_TASK_PRIORITY (near-max).
     webTask (web server, TCP push, WiFi retry) and logTask both run on
     Core 0, alongside the WiFi/lwIP stack, so neither can preempt a poll
     mid-transaction. Sharing Core 0 with WiFi/lwIP is what used to cause
     intermittent missed responses.
   - The RS485_DE_RE (transceiver direction) pin toggle in postTransmission()
     is the single most timing-critical line in the program: it must flip
     the bus back to receive before the slave's ~1ms response window
     starts. It's wrapped in a portENTER/EXIT_CRITICAL section and has no
     added delay.
   - modbusRead() is a small self-contained Modbus RTU master (read
     function codes 0x01-0x04: coils, discrete inputs, input registers,
     holding registers) instead of the ModbusMaster library, because the
     library's response timeout is a hardcoded, unconfigurable 2000ms. It
     enforces an explicit 3s response timeout (MODBUS_RESPONSE_TIMEOUT_MS),
     plus the spec's T3.5 (inter-frame) and T1.5 (inter-byte) silence
     rules computed from the live baud rate.
   - Errors (timeout, bad CRC/length, wrong slave, slave exception) are
     reported immediately with no debounce/grace period.
   - Each poll cycle is strictly poll -> cloud send -> next poll:
     pollModbus() signals pollCompleteSem when done; webTask pushes that
     cycle's data (raw TCP or MQTT/TLS to AWS IoT, selectable in Settings)
     and gives tcpSendDoneSem; modbusTask waits on it with a bounded
     timeout (TCP_SEND_TIMEOUT_MS) so a dead uplink can never stall
     polling.
   - Logging is offloaded from modbusTask via a queue (logQueue) to a
     dedicated low-priority logTask on Core 0, so Serial/String work never
     steals time from the real-time core.
   - Store & forward: payloads that can't be delivered are queued in a
     RAM ring buffer (SF_MAX_ENTRIES/SF_MAX_BYTES, oldest dropped) and
     replayed in order once the uplink recovers - see sendCloudData().
   - Per-slave backoff: after SLAVE_BACKOFF_FAIL_THRESHOLD consecutive
     timeouts a slave's parameters are skipped for SLAVE_BACKOFF_MS and
     then re-probed, so one dead slave can't stretch every poll cycle by
     3s per parameter - see pollModbus()/slaveHealth[].
   - Second data source: a parameter's transport field selects RS485 RTU
     (default) or Modbus TCP client to a remote server such as a CNC
     controller (tcpTargets[]). TCP polling runs on its own task,
     tcpPollTask(), pinned to Core 0 - never Core 1, so a slow/blocking
     TCP connect can never affect RTU timing. Both transports write into
     the same params[]/dataMutex, so the cloud push, dashboard, and
     store-and-forward paths handle either source identically. Per-target
     backoff mirrors the per-slave scheme (tcpTargetHealth[]).
   - Settings storage: the params/types/TCP-target tables persist as one
     compact binary blob per table (StoredParamRow etc., putBytes/
     getBytes) instead of one NVS key per field per row - NVS is a small
     fixed-size partition shared by every namespace including the AWS
     certs, and the old one-key-per-field layout was exhausting it in the
     field (confirmed: NVS_SAVE_INCOMPLETE with only 4 rows configured).
     Every save*() checks each write's result (trackNvsWrite()) and logs
     a key event if any failed, and every load*() transparently migrates
     a config saved by an older per-key firmware version to the compact
     format on next boot, reclaiming the old keys via preferences.clear().
   - Write (command) path: a param opts in via writable/min/maxWriteValue
     (holding registers/coils only - see isWritableArea()). Every source
     (dashboard /writeCommand, MQTT command topic, raw-TCP command line)
     funnels through submitWriteCommand(), which validates and
     enqueues onto rtuWriteQueue or tcpWriteQueue by transport;
     modbusTask/tcpPollTask drain their own queue once per cycle
     (processRtuWriteQueue()/processTcpWriteQueue()), so a write never
     crosses onto the other task's transport. Completed MQTT-sourced
     writes get a JSON ack either way (success or failure) on the command
     ack topic (queueMqttAck()/drainMqttAckQueue()/publishMqttAck());
     raw-TCP writes likewise on the socket (queueTcpAck()/
     drainTcpAckQueue()/sendTcpAckLine()). Both are only ever
     published/sent from webTask, never directly from
     modbusTask/tcpPollTask, which must not touch the network/TLS stack.
   - MQTT command channel (cloud-agnostic): plain <identity>/commands
     subscribe + <identity>/commands/ack publish (see mqttCommandTopic()),
     same JSON command shapes as the raw-TCP channel - deliberately NO AWS
     Device Shadow (which this replaced): shadow's desired/reported model
     is AWS-proprietary, and portability to other brokers (Azure IoT Hub,
     Mosquitto, ...) is a design goal. Only active when cloudConfig.mode
     == UPLINK_MODE_MQTT and cloudConfig.commandsEnabled. Commands are
     received over whichever transport is currently the active uplink:
     WiFi/PubSubClient normally (mqttCallback()), or the modem's onboard
     client during a cellular failover (+CMQTTSUB subscription + the
     +CMQTTRX* URC pump in cellularTask; acks travel back over
     AT+CMQTTPUB via small queues, never inline from a URC handler).
     Caveat: while cellularTask is inside a TinyGSM modem.xxx() status
     call, TinyGSM's own response parsing may consume an inbound-command
     URC (a few-ms window per 5s cycle) - the peer should treat a missing
     ack as "retry", same as any QoS-1 loss.
   - Raw TCP command channel: reuses the existing outbound tcpClient
     connection (the same one sendPayloadTcp() pushes telemetry on) instead
     of a separate listening socket - the master can send back newline-
     framed JSON commands, read by pollTcpCommands() once per webTask
     cycle: a write ({"param":"Name","value":42}, queued/acked) or an
     on-demand read ({"query":"Name"}, answered immediately by
     sendTcpQueryResponse() - separate from the periodic all-params
     telemetry broadcast, which keeps flowing regardless). Only active
     when cloudConfig.mode == UPLINK_MODE_TCP.
   - Local Digital I/O: 4 Digital In (D7-D10), 4 Digital Out (D11-D13,A4),
     4 Analog In (A0-A3) - plain GPIO, entirely independent of the Modbus
     buses (no slave ID/register address). Polled/drained on webTask
     (Core 0) on its own timer (pollDigitalIO()) - never on modbusTask/
     Core 1, since these reads never need to be, but keeping Core 1
     exclusive to RS485 is the one invariant this whole program is built
     around. Digital Out reuses the Modbus write path's shape:
     submitDoWriteCommand() -> doWriteQueue -> processDoWriteQueue()
     (drained on webTask, not a dedicated task, since digitalWrite() can't
     block), with the same WRITE_SOURCE_* tags - so Dashboard/MQTT/
     raw-TCP can all address a DO channel by name, falling back from
     findParamIndexByName() to findDoChannelIndexByName() wherever a
     command name doesn't match a Modbus param.
   - Cellular modem (SIM7600G-H): SIM/network registration/signal quality
     and the PDP data session (modem.gprsConnect(), APN in Settings) are
     live, visible on the Dashboard and in the Status Log. UART AT command
     interface on D5(RX)/D6(TX) via Serial2, PWRKEY power-on pulse on A5,
     driven by TinyGSM. Runs on its own dedicated task (cellularTask,
     Core 0) because modem.init()/testAT()/gprsConnect() etc. can block
     for several seconds - long enough that running it inline on webTask
     would stall the web UI and every other uplink alongside it.
   - Raw-TCP uplink transport plumbing: the raw-TCP uplink client is a
     Client* (tcpClient, pointing at wifiTcpClient or cellularTcpClient - a
     TinyGsmClient) instead of a concrete WiFiClient, so
     connectTcpServer()/sendPayloadTcp()/pollTcpCommands()/the command ack
     helpers all work unchanged regardless of which transport is active -
     see activeUplinkTransport. In practice this always stays WiFi: raw TCP
     mode targets a private LAN IP that cellular data can't route to, so by
     design it never fails over (only the MQTT uplink does - see below).
     WiFiClient-only APIs (setNoDelay(), the 3-arg connect() timeout
     overload) aren't part of the generic Client interface, so those stay
     gated to the concrete wifiTcpClient object.
   - MQTT-over-cellular does NOT reuse PubSubClient/tcpClient at all - per
     the SIM7500/SIM7600/SIM7800 MQTT AT Command Manual, the modem has its
     own onboard MQTT client (the AT+CMQTT* command family) with its own
     certificate store (AT+CCERTDOWN) and SSL context (AT+CSSLCFG,
     authmode 2 for AWS IoT's mutual TLS) - TLS runs on the modem's own
     chip, not the ESP32's mbedTLS, which is also why this doesn't add the
     RAM cost a second WiFiClientSecure-style session would. Driven via a
     small raw AT command helper (sendAtCommand()/sendAtCommandWithData()/
     sendMqttCommand(), right before cellularTask()) that talks to SerialAT
     directly - safe because cellularTask is SerialAT's sole owner and
     never overlaps these with a TinyGSM modem.xxx() call.
   - WiFi<->cellular MQTT failover: webTask tracks how long WiFi has been
     continuously up/down (checkWifiUplinkStatusChange()) and flips
     mqttFailoverActive once it crosses UPLINK_FAILOVER_THRESHOLD_MS (5 min)
     in either direction (updateMqttFailoverState()). cellularTask reads
     that flag to decide whether to bring up/publish over its onboard MQTT
     client or tear the session back down; webTask's sendCloudData() skips
     its own WiFi MQTT attempt (and store & forward buffering) while
     cellular is active, so the two transports never both deliver the same
     data. Raw TCP mode is unaffected (see above).
   ====================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <time.h>
#include <WiFiClient.h>
#include <WiFiClientSecure.h>
// External library: "PubSubClient" by Nick O'Leary, v2.8+ (Arduino Library
// Manager). v2.8 is required for setBufferSize() at runtime.
#include <PubSubClient.h>
// External library: "ArduinoJson" by Benoit Blanchon, v6+ (Arduino Library
// Manager). Used only to parse small inbound command payloads (MQTT
// command topic, raw-TCP command lines) - all outgoing JSON is still
// hand-built via jsonEscape()/String concatenation, unchanged.
#include <ArduinoJson.h>
// External library: "TinyGSM" by Volodymyr Shymanskyy, v0.11+ (Arduino
// Library Manager). Drives the SIM7600G-H cellular modem over its UART AT
// command interface - handles power-on handshake, SIM/network status
// queries, and (later) a Client-compatible TCP/TLS socket, all far more
// robustly than a hand-rolled AT parser would.
#define TINY_GSM_MODEM_SIM7600
#include <TinyGsmClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
// ===================== WiFi AP Settings =====================
// AP_SSID is only the fallback/default - the actual name used is
// deviceConfig.apSsid (or the computed device name if apMatchesDeviceName
// is set), both user-settable on the Settings page. See getEffectiveApSsid().
#define DEFAULT_AP_SSID "ESP32ModbusGateWay"
const char* AP_PASSWORD = "12345678";
// Max length of the user-provided device-name prefix (see getDeviceName()).
#define DEVICE_NAME_PREFIX_MAX_LEN 5

// ===================== NTP (best-effort) =====================
// SNTP sync runs in the background once the uplink WiFi has a route to an
// NTP server. Priority: the user-set on-premise server (Settings page,
// optional) is tried first, then these public servers; if none ever
// responds, timestamps stay seconds-since-boot exactly as before. See
// startNtp() / getTimestamp().
#define NTP_SERVER_1 "pool.ntp.org"
#define NTP_SERVER_2 "time.nist.gov"
#define NTP_SERVER_MAX_LEN 64
// Anything below this can't be a real current date, so time() results
// under it mean "SNTP hasn't synced yet" (epoch for 2025-01-01).
#define MIN_VALID_EPOCH 1735689600UL

// ===================== Cloud Uplink (MQTT / AWS IoT) =====================
// Uplink mode is selectable on the Settings page: raw TCP (the original
// behavior, default) or MQTT over TLS with X.509 client certs - the
// transport AWS IoT Core requires (port 8883). Certificates are pasted
// into the web UI and stored in flash. See connectMqtt()/sendCloudData().
#define UPLINK_MODE_TCP 0
#define UPLINK_MODE_MQTT 1
#define DEFAULT_MQTT_PORT 8883
#define DEFAULT_MQTT_TOPIC "gateway/data"
// Min gap between MQTT connect attempts - a TLS handshake can block
// webTask for several seconds, so failed attempts must not spin.
#define MQTT_RETRY_INTERVAL_MS 10000
#define MQTT_HANDSHAKE_TIMEOUT_S 10
#define MQTT_SOCKET_TIMEOUT_S 5

// ===================== Store & Forward =====================
// When the uplink (TCP or MQTT) is down, each poll cycle's payload is
// queued in a RAM ring buffer and replayed in order once the uplink
// recovers - timestamps are already inside each payload, so history
// arrives intact. Oldest entries are dropped when the caps are hit.
#define SF_MAX_ENTRIES 64
#define SF_MAX_BYTES 32768
// Max backlog replays per poll cycle, so a big drain can't stall the web
// UI for long; the rest goes out on the following cycles.
#define SF_MAX_DRAIN_PER_CYCLE 8

// ===================== Per-Slave Backoff =====================
// After this many consecutive TIMEOUTS (no response at all - CRC errors
// and Modbus exceptions mean the slave is alive), a slave is considered
// offline and its parameters are skipped for SLAVE_BACKOFF_MS, then
// re-probed. Caps a dead slave's cost at one 3s timeout per backoff
// window instead of 3s per parameter on every cycle.
#define SLAVE_BACKOFF_FAIL_THRESHOLD 3
#define SLAVE_BACKOFF_MS 30000

// ===================== Modbus TCP Client (second data source) =====================
// A parameter's data can come from the RS485 RTU bus (TRANSPORT_RTU, the
// original/default behavior) or from a remote Modbus TCP server such as a
// CNC controller (TRANSPORT_TCP), polled over WiFi by a dedicated task on
// Core 0 - never Core 1, so this never competes with the RTU master for
// real-time bus access. See tcpPollTask()/modbusTcpRead().
#define TRANSPORT_RTU 0
#define TRANSPORT_TCP 1
#define MAX_TCP_TARGETS 4
#define TCP_TARGET_SCHEMA_VERSION 1
#define MODBUS_TCP_CONNECT_TIMEOUT_MS 2000
#define MODBUS_TCP_RESPONSE_TIMEOUT_MS 3000
// Same reasoning as SLAVE_BACKOFF_*, applied to a dead/unreachable TCP target.
#define TCP_TARGET_BACKOFF_FAIL_THRESHOLD 3
#define TCP_TARGET_BACKOFF_MS 30000
#define TCP_POLL_TASK_CORE 0
#define TCP_POLL_TASK_PRIORITY 1
#define TCP_POLL_TASK_STACK 8192

// ===================== RS485 Pins =====================
#define RXD2 D2
#define TXD2 D3
#define RS485_DE_RE D4

// ===================== Cellular Modem (SIM7600G-H) Pins =====================
// UART AT command interface (not the module's USB port) - ESP32 ships two
// remappable HardwareSerial instances beyond USB-CDC; Serial1 is RS485
// above, so the modem gets Serial2. A5 drives PWRKEY (power-on pulse - see
// powerOnModem()); A6/A7 stay free for RESET/STATUS if ever needed.
#define SERIAL2_RX_PIN D5
#define SERIAL2_TX_PIN D6
#define MODEM_PWRKEY_PIN A5
#define MODEM_BAUD 115200

// ===================== Digital I/O Pins =====================
// Local GPIO only - no Modbus/RS485/TCP transaction involved, so polling
// these never touches modbusTask/Core 1 (see pollDigitalIO(), run from
// webTask on Core 0). D0/D1 are the USB-CDC serial pins (avoided); D2-D6
// are taken above.
#define DI_CHANNEL_COUNT 4
#define DO_CHANNEL_COUNT 4
#define AI_CHANNEL_COUNT 4

const uint8_t DI_PINS[DI_CHANNEL_COUNT] = { D7, D8, D9, D10 };
const uint8_t DO_PINS[DO_CHANNEL_COUNT] = { D11, D12, D13, A4 };
const uint8_t AI_PINS[AI_CHANNEL_COUNT] = { A0, A1, A2, A3 };

// Board silkscreen labels matching the pin arrays above, purely for
// display on the Settings page (the D../A.. macros resolve to raw GPIO
// numbers at compile time, which wouldn't mean anything useful in the UI).
const char *DI_PIN_LABELS[DI_CHANNEL_COUNT] = { "D7", "D8", "D9", "D10" };
const char *DO_PIN_LABELS[DO_CHANNEL_COUNT] = { "D11", "D12", "D13", "A4" };
const char *AI_PIN_LABELS[AI_CHANNEL_COUNT] = { "A0", "A1", "A2", "A3" };

// ===================== Limits =====================
#define MAX_PARAMS 100
#define MAX_PARAM_TYPES 20
#define DEFAULT_POLL_INTERVAL_MS 2000
#define MIN_POLL_INTERVAL_MS 10
#define MAX_POLL_INTERVAL_MS 3600000
// Pause after each parameter's request/response is done, before starting
// the next one. Modbus's own T3.5 bus-silence timing is already enforced
// per-transaction inside modbusRead() regardless of this
// value - this is purely extra settle time for slower slave devices that
// need a moment to recover before accepting the next request. Adjustable
// on the Settings page; raise it if parameters right after a fast one
// start showing errors, lower it for a faster poll cycle.
#define DEFAULT_SLAVE_RECOVERY_DELAY_MS 100
#define MIN_SLAVE_RECOVERY_DELAY_MS 0
#define MAX_SLAVE_RECOVERY_DELAY_MS 1000
#define TYPE_SCHEMA_VERSION 4

// ===================== Debug Settings =====================
#define DEBUG_ENABLED 1
#define DEBUG_SAVE_LOG 1
#define DEBUG_MODBUS_CHANGE_LOG 1
#define DEBUG_SUMMARY_LOG 1

#define DEBUG_ERROR_REPEAT_MS 30000
#define DEBUG_SUMMARY_INTERVAL_MS 10000
#define WIFI_RETRY_INTERVAL_MS 5000
// How long WiFi must be continuously down before the MQTT uplink fails over
// to cellular, and how long it must be continuously back up before failing
// back - same grace period both directions, per user spec.
#define UPLINK_FAILOVER_THRESHOLD_MS (3UL * 60UL * 1000UL)  // 3 minutes
#define TCP_CONNECT_TIMEOUT_MS 2000
// Max wait for webTask's TCP-send feedback before moving to the next poll.
#define TCP_SEND_TIMEOUT_MS 3000
// Core 1, not 0: Core 0 also hosts WiFi/lwIP's own high-priority tasks,
// which can jitter enough to blow the ~1ms RS485 turnaround window.
#define MODBUS_TASK_CORE 1
// As high as practical, below the levels ESP-IDF reserves for itself.
#define MODBUS_TASK_PRIORITY (configMAX_PRIORITIES - 5)
#define MODBUS_TASK_STACK 8192
// Low priority, Core 0 (WiFi core) - logging must never steal Core 1 time.
#define LOG_TASK_CORE 0
#define LOG_TASK_PRIORITY 1
#define LOG_TASK_STACK 4096
#define LOG_QUEUE_LEN 40
#define LOG_MSG_MAX_LEN 200
// Web server / TCP push / WiFi retry: Core 0, leaving Core 1 to modbusTask.
#define WEB_TASK_CORE 0
#define WEB_TASK_PRIORITY 1
#define WEB_TASK_STACK 8192
#define KEY_LOG_SIZE 15
// ===================== Objects =====================
WebServer server(80);

Preferences preferences;
WiFiClient wifiTcpClient;
WiFiClientSecure tlsClient;
PubSubClient mqttClient(tlsClient);

// Cellular modem (SIM7600G-H). SIM/registration/signal/data-session
// monitoring, cert/SSL provisioning and the modem's own onboard MQTT client
// (AT+CMQTT*) are all live (cellularTask()/CellularStatus) - see
// mqttFailoverActive below for the WiFi<->cellular MQTT failover decision.
//
// Raw TCP uplink mode (cloudConfig.mode == UPLINK_MODE_TCP) is intentionally
// NOT part of this failover: its target is a private LAN IP (e.g. a local
// Node-RED instance), which cellular data simply cannot route to, so per
// design it always stays on WiFi - activeUplinkTransport below is never
// switched to cellular and exists only as unused-for-now plumbing from an
// earlier stage.
HardwareSerial SerialAT(2);
TinyGsm modem(SerialAT);
TinyGsmClient cellularTcpClient(modem);

// Raw-TCP uplink client - a Client* so connectTcpServer()/sendPayloadTcp()/
// pollTcpCommands() work unchanged regardless of transport, but always left
// pointed at WiFi (see comment above - raw TCP mode never fails over).
#define UPLINK_TRANSPORT_WIFI 0
#define UPLINK_TRANSPORT_CELLULAR 1
uint8_t activeUplinkTransport = UPLINK_TRANSPORT_WIFI;
Client *tcpClient = &wifiTcpClient;

// ===================== Timers =====================
unsigned long lastSummaryPrintTime = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;

// ===================== Cross-Task Synchronization =====================
// dataMutex: params[]/typeList[] (webTask writes, modbusTask reads/writes).
// logMutex: debugLogs/keyLog, mainly guarding webTask's keyLog[] reads.
// serialMutex: the RS485 UART, so settings can't change mid-transaction.
// cellularMutex: cellularStatus, kept separate from dataMutex since it's
// written by its own dedicated task (cellularTask), not webTask/modbusTask.
// pollCompleteSem/tcpSendDoneSem: two-way handshake so each cycle's TCP
// push finishes (or is skipped) before the next poll cycle starts.
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t logMutex;
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t cellularMutex;
SemaphoreHandle_t pollCompleteSem;
SemaphoreHandle_t tcpSendDoneSem;
TaskHandle_t modbusTaskHandle;
TaskHandle_t tcpPollTaskHandle;
TaskHandle_t cellularTaskHandle;

// ===================== Cellular Modem Status =====================
// Written only by cellularTask; read by webTask for the Dashboard/data
// endpoint. registrationStatus/operatorName are the raw AT+CREG/AT+COPS
// results turned into something readable - see cellularTask().
#define CELLULAR_TASK_CORE 0
#define CELLULAR_TASK_PRIORITY 1
// 8KB, not the 4KB the other Core-0 tasks get by default: this task's URC
// path (consumeCellularInboundMessage -> handleMqttCommand -> ArduinoJson
// + the write-submission helpers) can nest inside an in-flight AT command
// wait, stacking both call chains at once.
#define CELLULAR_TASK_STACK 8192

struct CellularStatus {
  bool modemResponding;
  bool simReady;
  bool networkRegistered;
  int signalQuality;  // AT+CSQ raw value: 0-31 (higher = better), 99 = unknown
  String operatorName;
  bool dataConnected;  // PDP/data context up (modem.gprsConnect()) - not yet used for any uplink traffic
  bool mqttCertsProvisioned;  // modem's own cert store + SSL context set up
  bool mqttSessionConnected;  // modem's onboard MQTT client is connected + publishing telemetry (only while failover is active)
  bool mqttCommandsSubscribed;  // modem's client is subscribed to the command topic (inbound commands work during failover)
  unsigned long lastUpdateTime;
};

CellularStatus cellularStatus;

// Carries log entries from any task to logTask (Core 0), which does the
// actual Serial/String/keyLog work off the real-time core.
QueueHandle_t logQueue;
TaskHandle_t logTaskHandle;
TaskHandle_t webTaskHandle;

struct LogQueueItem {
  char text[LOG_MSG_MAX_LEN];
  unsigned long timestampSec;
  bool isKeyEvent;
};

// ===================== Write Command Queues =====================
// A write request (from the manual dashboard control, the MQTT command
// topic, or the raw-TCP command channel) is validated
// and enqueued by submitWriteCommand(), then routed to one of two queues by
// the target parameter's transport - RTU writes are drained by modbusTask
// (Core 1, under serialMutex, interleaved with polling); TCP writes are
// drained by tcpPollTask (Core 0). Keeping them as two separate queues
// means each task only ever touches its own transport, with no cross-task
// filtering.
#define WRITE_QUEUE_LEN 16

#define WRITE_SOURCE_DASHBOARD 0
#define WRITE_SOURCE_MQTT 1
#define WRITE_SOURCE_TCP 2

struct WriteCommand {
  int paramIndex;
  float value;  // engineering-unit value to write
  uint8_t source;  // WRITE_SOURCE_* - lets the drain functions know whether
                   // a completed write needs an MQTT command ack or a
                   // raw-TCP command ack
};

QueueHandle_t rtuWriteQueue;
QueueHandle_t tcpWriteQueue;

// A write drain function (modbusTask/tcpPollTask) can't publish MQTT itself
// - that would pull TLS/network work onto the real-time core or block on a
// socket outside webTask's control. Instead it drops a tiny fixed-size item
// here (non-blocking) whenever a WRITE_SOURCE_MQTT command completes;
// webTask drains it and publishes the JSON ack on the command ack topic
// (see mqttCommandAckTopic()). Best-effort: if this small queue is ever
// full, that one ack is just dropped - it only affects the peer's
// visibility, never the actual Modbus write, which has already happened.
#define MQTT_ACK_QUEUE_LEN 8
// Matches STORED_NAME_LEN (param name field length) - kept as its own
// constant here since that's defined later, alongside the blob storage
// structs, and this queue item has nothing to do with blob storage.
#define MQTT_ACK_NAME_LEN 32

struct MqttAckItem {
  char name[MQTT_ACK_NAME_LEN];
  float value;
  uint8_t result;  // MB_SUCCESS or an MB_ERR_*/exception code
};

QueueHandle_t mqttAckQueue;

// Same idea as mqttAckQueue, but for the raw-TCP command channel (see
// handleTcpCommandLine()/pollTcpCommands()): every completed
// WRITE_SOURCE_TCP command gets an ack either way - success or failure -
// hence carrying the result code here.
#define TCP_ACK_QUEUE_LEN 8
#define TCP_ACK_NAME_LEN 32

struct TcpAckItem {
  char name[TCP_ACK_NAME_LEN];
  float value;
  uint8_t result;  // MB_SUCCESS or an MB_ERR_*/exception code
};

QueueHandle_t tcpAckQueue;

// ===================== Digital Out Write Queue =====================
// Mirrors the Modbus write-queue pattern (WriteCommand/rtuWriteQueue/
// tcpWriteQueue) above, but for local Digital Out channels. A plain
// digitalWrite() takes microseconds and can't block, so unlike the Modbus
// writes it doesn't need its own dedicated drain task - processDoWriteQueue()
// just runs inline on webTask (Core 0). Reuses the same WRITE_SOURCE_*
// tags, so mqtt/tcp acks work identically to the Modbus write path.
#define DO_QUEUE_LEN 16

struct DoWriteCommand {
  int channelIndex;
  bool value;
  uint8_t source;
};

QueueHandle_t doWriteQueue;

// Guards only the RS485_DE_RE pin toggle inside preTransmission()/
// postTransmission(). This is the single most timing-critical line in the
// program: postTransmission() must flip the transceiver back to receive
// before the slave's first response bit arrives (~1ms turnaround). The
// critical section stops that one instruction from being preempted
// mid-flight by another task/ISR on the same core.
portMUX_TYPE rs485Mux = portMUX_INITIALIZER_UNLOCKED;

// Renders a captured byte buffer as space-separated hex, e.g. "01 03 04 7E".
// Used only for troubleshooting output, so a plain O(n) String build is
// fine here.
String bytesToHex(const uint8_t *buf, uint8_t len, bool overflowed) {
  if (len == 0) {
    return "(none)";
  }

  String hex = "";

  for (uint8_t i = 0; i < len; i++) {
    if (i > 0) {
      hex += " ";
    }
    if (buf[i] < 0x10) {
      hex += "0";
    }
    hex += String(buf[i], HEX);
  }

  hex.toUpperCase();

  if (overflowed) {
    hex += " ...(truncated)";
  }

  return hex;
}

// ===================== Debug Logs =====================
String debugLogs = "";

// ===================== Key Event Log (field troubleshooting) =====================
// Short, curated ring buffer of milestone/anomaly events (WiFi/TCP state
// changes, poll health, settings saved, etc.) surfaced on the landing page -
// distinct from the verbose per-parameter debugLogs above. Guarded by
// logMutex.
String keyLog[KEY_LOG_SIZE];
int keyLogHead = 0;
int keyLogCount = 0;

bool lastWifiUplinkConnected = false;
bool lastTcpConnectedState = false;
bool lastMqttConnectedState = false;
unsigned long lastTcpFailLogTime = 0;
unsigned long lastWifiFailLogTime = 0;
unsigned long lastMqttFailLogTime = 0;

// ===================== MQTT WiFi<->Cellular Failover =====================
// wifiDownSince/wifiUpSince are set on each WiFi state transition (see
// checkWifiUplinkStatusChange()); 0 means "not currently in that state".
// mqttFailoverActive is written only by webTask (updateMqttFailoverState())
// and read only by cellularTask - a plain bool read/write is atomic on this
// platform, so no mutex, matching how cloudConfig fields are already shared
// across these two tasks.
unsigned long wifiDownSince = 0;
unsigned long wifiUpSince = 0;
bool mqttFailoverActive = false;

// ===================== TCP / WiFi Uplink Config =====================
struct UplinkConfig {
  String ssid;
  String password;
  String serverIP;
  int port;
  String ntpServer;  // optional on-premise NTP; tried before the public servers
};

UplinkConfig uplinkConfig;

// ===================== Cellular Modem Config =====================
// The hardware side (PWRKEY pin, UART pins/baud) stays compile-time
// constant; the runtime settings live here. Blank APN is passed through
// as-is to gprsConnect() - some SIM7600 firmware/carrier combos can
// auto-provision from the SIM without an explicit APN; verify against
// your carrier if the data session never comes up with it left blank.
// publishIntervalMs paces only the telemetry publish during failover -
// the cellularTask loop itself (status polling, URC pump, ack drains)
// stays on its fixed CELLULAR_POLL_INTERVAL_MS cadence regardless, so a
// long publish interval never makes inbound commands sluggish.
#define CELLULAR_PUBLISH_INTERVAL_MIN_MS 5000UL
#define CELLULAR_PUBLISH_INTERVAL_MAX_MS 3600000UL
#define CELLULAR_PUBLISH_INTERVAL_DEFAULT_MS 5000UL

struct CellularConfig {
  String apn;
  uint32_t publishIntervalMs;
};

CellularConfig cellularConfig;

// ===================== Cloud Uplink Config =====================
// clientId blank means "use the Device Name" (also the AWS Thing name
// convention). The three PEM certs live as globals because
// WiFiClientSecure::setCACert()/setCertificate()/setPrivateKey() store the
// POINTER, not a copy - these Strings must stay alive and unmodified while
// a TLS session exists. They are only reassigned from the web handler
// (webTask), the same task that runs the MQTT client, so that's safe.
struct CloudConfig {
  uint8_t mode;  // UPLINK_MODE_TCP / UPLINK_MODE_MQTT
  String endpoint;
  int port;
  String clientId;
  String topic;
  // Cloud-agnostic MQTT command channel (see mqttCommandTopic()/
  // connectMqtt()/mqttCallback()): plain <identity>/commands subscribe +
  // <identity>/commands/ack publish - works on any broker (AWS IoT, Azure
  // IoT Hub via its MQTT bridge, Mosquitto, ...), no AWS Shadow semantics.
  bool commandsEnabled;
};

CloudConfig cloudConfig;

String certRootCA = "";
String certDevice = "";
String certPrivKey = "";

// Per-request accumulators for cert FILE uploads on /saveCloud (multipart
// form). Filled by handleCertUpload() during upload parsing, consumed and
// cleared by handleSaveCloud() right after. webTask-only, so no locking.
#define CERT_UPLOAD_MAX_LEN 8192
String uploadCaCert = "";
String uploadDevCert = "";
String uploadPrivKey = "";

// ===================== Device / AP Identity =====================
// apSsid: user-settable AP name, used as-is unless apMatchesDeviceName is set.
// deviceNamePrefix: user-provided prefix (up to 5 chars); combined with the
// last 5 characters of the MAC to form the device name (see getDeviceName()).
// Empty prefix means the device name defaults to the full MAC address.
struct DeviceConfig {
  String apSsid;
  String deviceNamePrefix;
  bool apMatchesDeviceName;
};

DeviceConfig deviceConfig;

// ===================== Communication Settings =====================
uint32_t commBaudRate = 9600;
String commParity = "N";
uint8_t commStopBits = 1;
uint32_t pollIntervalMs = DEFAULT_POLL_INTERVAL_MS;
uint32_t slaveRecoveryDelayMs = DEFAULT_SLAVE_RECOVERY_DELAY_MS;

// ===================== Parameter Type Structure =====================
struct ParamType {
  String name;
  String baseFormat;
  float divisor;
  uint16_t dataLength;
};

ParamType typeList[MAX_PARAM_TYPES];
int typeCount = 0;

// ===================== Modbus TCP Targets =====================
// A small managed list of remote Modbus TCP servers (e.g. a CNC
// controller), referenced by index from ModbusParam rows with
// transport == TRANSPORT_TCP - same "small list + index reference"
// pattern as typeList[] above. unitId is the MBAP Unit Identifier byte
// (Modbus TCP's equivalent of an RTU slave address).
struct ModbusTcpTarget {
  String name;
  String ip;
  uint16_t port;
  uint8_t unitId;
};

ModbusTcpTarget tcpTargets[MAX_TCP_TARGETS];
int tcpTargetCount = 0;

// One persistent client per target, reused across poll cycles instead of
// reconnecting every time - only touched by tcpPollTask (Core 0).
WiFiClient tcpTargetClients[MAX_TCP_TARGETS];

// ===================== Modbus Parameter Structure =====================
// Which of the four Modbus address spaces a parameter reads from. The
// numeric order matters: 0 = holding register, so configs saved before
// this field existed keep their old behavior. Coils/discrete inputs are
// 1-bit; discrete inputs and input registers are read-only per the spec.
#define AREA_HOLDING_REGISTER 0
#define AREA_INPUT_REGISTER 1
#define AREA_COIL 2
#define AREA_DISCRETE_INPUT 3

bool isBitArea(uint8_t area) {
  return area == AREA_COIL || area == AREA_DISCRETE_INPUT;
}

// Only holding registers and coils are writable per the Modbus spec -
// input registers and discrete inputs are read-only address spaces.
bool isWritableArea(uint8_t area) {
  return area == AREA_HOLDING_REGISTER || area == AREA_COIL;
}

struct ModbusParam {
  String name;
  String type;
  uint8_t area;  // AREA_* address space (determines the read function code)
  uint8_t transport;  // TRANSPORT_RTU (default) or TRANSPORT_TCP
  uint8_t slaveId;  // RTU slave address - only meaningful when transport == TRANSPORT_RTU
  uint8_t tcpTargetIndex;  // index into tcpTargets[] - only meaningful when transport == TRANSPORT_TCP
  uint16_t registerAddress;
  uint16_t registerLength;
  bool enabled;
  float value;
  bool valid;
  uint8_t errorCode;
  unsigned long lastUpdateTime;

  bool lastLoggedValid;
  uint8_t lastLoggedErrorCode;
  unsigned long lastErrorLogTime;

  // Write path: writable is a user opt-in (only meaningful/settable when
  // isWritableArea(area) is true); min/maxWriteValue are an optional
  // engineering-unit safety clamp enforced before any write is dispatched
  // (min==max==0 means "no limit configured" - see submitWriteCommand()).
  bool writable;
  float minWriteValue;
  float maxWriteValue;
  uint8_t lastWriteResult;  // 0xFF = never written; else MB_SUCCESS or an MB_ERR_*/exception code
  unsigned long lastWriteTime;
};

ModbusParam params[MAX_PARAMS];
int paramCount = 0;

// ===================== Local Digital I/O =====================
// Independent of the Modbus params[] table above - plain local GPIO, no
// slave ID/register address/transport, polled directly (pollDigitalIO(),
// webTask/Core 0) instead of via a Modbus transaction. Kept as three small
// fixed-size arrays (DI_CHANNEL_COUNT/DO_CHANNEL_COUNT/AI_CHANNEL_COUNT are
// small constants, unlike the up-to-100-row params[] table) rather than a
// user-resizable table.
struct DigitalInChannel {
  String name;
  bool enabled;
  bool value;
  unsigned long lastUpdateTime;
};

struct DigitalOutChannel {
  String name;
  bool enabled;
  bool value;  // last commanded state (what the pin is currently driven to)
  unsigned long lastWriteTime;
};

struct AnalogInChannel {
  String name;
  bool enabled;
  // Engineering value = raw ADC counts * scale + offset - a simple 2-point
  // linear calibration (e.g. mapping the ESP32's 0-4095 ADC range to a
  // 0-10V or 4-20mA field signal), same spirit as the Modbus type list's
  // divisor but general enough to also handle a non-zero offset.
  float scale;
  float offset;
  int rawValue;
  float value;
  unsigned long lastUpdateTime;
};

DigitalInChannel diChannels[DI_CHANNEL_COUNT];
DigitalOutChannel doChannels[DO_CHANNEL_COUNT];
AnalogInChannel aiChannels[AI_CHANNEL_COUNT];

// ===================== Compact Blob Storage =====================
// params[]/typeList[]/tcpTargets[] used to persist as one NVS key per
// FIELD per ROW - a full params table alone could be up to 900 individual
// entries. NVS is a small, fixed-size partition shared by every namespace
// in this sketch (including the AWS certs, which alone are 1-2KB of PEM
// text), and that many-small-keys layout was exhausting it in practice
// (confirmed in the field: NVS_SAVE_INCOMPLETE errors on the "modbus"
// namespace with only 4 rows configured). Each table is now packed into
// a fixed-size POD struct array and stored as a single blob
// (putBytes/getBytes) - a couple of NVS entries total regardless of row
// count. String fields are truncated into fixed-size char buffers only
// for this on-flash format; params[]/typeList[]/tcpTargets[] themselves
// are untouched. registerLength/baseFormat/dataLength are recomputed
// after load exactly as before, so they aren't stored at all.
#define STORED_NAME_LEN 32
#define STORED_TYPE_LEN 20
#define STORED_IP_LEN 40

// v1: the original compact blob layout, before the write path (writable/
// min/maxWriteValue) existed. Kept only so a device that already migrated
// to v1 (an earlier firmware version) can be read once more and
// transparently re-saved in the current (v2) layout - see loadSettings().
struct StoredParamRowV1 {
  char name[STORED_NAME_LEN];
  char type[STORED_TYPE_LEN];
  uint8_t area;
  uint8_t transport;
  uint8_t slaveId;
  uint8_t tcpTargetIndex;
  uint16_t registerAddress;
  uint8_t enabled;
};

struct StoredParamRow {
  char name[STORED_NAME_LEN];
  char type[STORED_TYPE_LEN];
  uint8_t area;
  uint8_t transport;
  uint8_t slaveId;
  uint8_t tcpTargetIndex;
  uint16_t registerAddress;
  uint8_t enabled;
  uint8_t writable;
  float minWriteValue;
  float maxWriteValue;
};

struct StoredParamTypeRow {
  char name[STORED_TYPE_LEN];
  float divisor;
};

struct StoredTcpTargetRow {
  char name[STORED_NAME_LEN];
  char ip[STORED_IP_LEN];
  uint16_t port;
  uint8_t unitId;
};

// ===================== Forward Declarations =====================
void resetDebugState(int index);
String htmlEscape(String text);
String jsonEscape(String text);
void sendCloudData();

// ===================== Logger =====================
// Builds a small fixed-size item and hands it to logTask (Core 0) - see
// the LOG_TASK_CORE/logQueue comments near the top of this file. Uses a
// non-blocking send: if logTask is ever backed up and the queue is full,
// the message is silently dropped rather than stalling the caller. That
// caller is very often modbusTask, so this must never block.
void enqueueLog(String msg, bool isKeyEvent) {
  LogQueueItem item;

  item.timestampSec = millis() / 1000;
  item.isKeyEvent = isKeyEvent;
  msg.toCharArray(item.text, LOG_MSG_MAX_LEN);

  xQueueSend(logQueue, &item, 0);
}

void logMessage(String msg) {
  enqueueLog(msg, false);
}

// Records a short, curated milestone/anomaly message for field troubleshooting
// on the landing page (see /keylog), in addition to the normal verbose log.
void logKeyEvent(String msg) {
  enqueueLog(msg, true);
}

// ===================== NVS Write Failure Detection =====================
// Every Preferences::put*() call returns 0 on failure (most commonly
// NVS_ERR_NOT_ENOUGH_SPACE, once the flash partition fills up - a real
// risk here given how many settings namespaces and how large the params
// table have grown). A failed write leaves the OLD value on flash while
// the in-RAM value has already changed, which looks exactly like
// "settings revert after a reboot" - because that's exactly what it is.
// Each save*() function resets nvsWriteFailures to 0, wraps every put*()
// call in trackNvsWrite(), then logs once (not per-key, to avoid flooding
// the log if an entire namespace is failing) if anything failed.
int nvsWriteFailures = 0;

void trackNvsWrite(size_t result) {
  if (result == 0) {
    nvsWriteFailures++;
  }
}

// ===================== RS485 Direction Control =====================
void preTransmission() {
  portENTER_CRITICAL(&rs485Mux);
  digitalWrite(RS485_DE_RE, HIGH);
  portEXIT_CRITICAL(&rs485Mux);

  // Transceiver driver-enable settle time, before anything is sent.
  delayMicroseconds(300);
}

void postTransmission() {
  // No added delay: Serial1.flush() (called just before this) already
  // blocks until the last stop bit is on the wire, so it's safe - and
  // per spec, correct - to release the bus immediately.
  portENTER_CRITICAL(&rs485Mux);
  digitalWrite(RS485_DE_RE, LOW);
  portEXIT_CRITICAL(&rs485Mux);
}

// ===================== Modbus RTU Master (custom, exact timing) =====================
// Self-contained RTU master (function 0x03 only) instead of the
// ModbusMaster library, whose response timeout is a hardcoded 2000ms.
#define MODBUS_RESPONSE_TIMEOUT_MS 3000
#define MODBUS_RAW_BUF_SIZE 64

#define MB_SUCCESS 0x00
#define MB_ERR_TIMEOUT 0xE2      // nothing usable arrived within MODBUS_RESPONSE_TIMEOUT_MS
#define MB_ERR_CRC 0xE3          // frame length/byte-count/CRC didn't check out
#define MB_ERR_WRONG_SLAVE 0xE0  // a well-formed frame arrived, but from a different slave ID
#define MB_ERR_BACKOFF 0xE4      // not polled: slave is in its offline backoff window
// Any other nonzero result is a raw Modbus exception code (1-11) reported
// by the slave itself (e.g. 2 = ILLEGAL DATA ADDRESS).

// Per-slave health for the offline backoff, indexed by slave ID. Only
// ever touched by modbusTask, so no locking needed. backoffUntil == 0
// means "not in backoff"; nonzero is the millis() deadline of the window.
struct SlaveHealth {
  uint8_t consecTimeouts;
  unsigned long backoffUntil;
};

SlaveHealth slaveHealth[256] = {};

// Same backoff bookkeeping, keyed by TCP target index instead of RTU slave
// ID. Only ever touched by tcpPollTask (Core 0), so no locking needed.
SlaveHealth tcpTargetHealth[MAX_TCP_TARGETS] = {};

// MB_ERR_* codes reused for TCP targets, plus one new one: a TCP target
// that's simply never been configured (index unset / target list empty).
#define MB_ERR_TCP_UNCONFIGURED 0xE5

// Raw bytes for the most recent attempt, used by pollModbus()'s [POLL] debug
// log line. Only touched while serialMutex is held, so no extra lock needed.
uint8_t lastModbusTx[8];
uint8_t lastModbusTxLen = 0;
uint8_t lastModbusRx[MODBUS_RAW_BUF_SIZE];
uint8_t lastModbusRxLen = 0;
bool lastModbusRxOverflow = false;

// micros() timestamp of the end of the last bus activity (this device's
// own transmission, or the last byte of a received response) - used to
// enforce the Modbus RTU spec's T3.5 minimum inter-frame silence below.
uint32_t lastBusActivityUs = 0;

// Modbus RTU spec character/T1.5/T3.5 timing, computed from the current
// commBaudRate on every call since baud is user-configurable at runtime.
// Per the spec: character time assumes an 11-bit frame (start + 8 data +
// parity + stop) regardless of the actual configured parity/stop bits;
// above 19200 baud, T1.5/T3.5 are fixed at 750us/1750us rather than
// scaling down further.
uint32_t modbusCharTimeUs() {
  return (uint32_t)((11.0 * 1000000.0) / commBaudRate);
}

uint32_t modbusT15Us() {
  return (commBaudRate > 19200) ? 750 : (modbusCharTimeUs() * 3 / 2);
}

uint32_t modbusT35Us() {
  return (commBaudRate > 19200) ? 1750 : (modbusCharTimeUs() * 7 / 2);
}

// Standard Modbus RTU CRC16 (poly 0xA001, init 0xFFFF, low byte first on
// the wire) - identical algorithm used by every Modbus implementation.
uint16_t modbusCRC16(const uint8_t *buf, uint8_t len) {
  uint16_t crc = 0xFFFF;

  for (uint8_t pos = 0; pos < len; pos++) {
    crc ^= (uint16_t)buf[pos];

    for (uint8_t i = 0; i < 8; i++) {
      if (crc & 0x0001) {
        crc >>= 1;
        crc ^= 0xA001;
      } else {
        crc >>= 1;
      }
    }
  }

  return crc;
}

// Read function code for each register area.
uint8_t modbusFunctionForArea(uint8_t area) {
  switch (area) {
    case AREA_INPUT_REGISTER: return 0x04;
    case AREA_COIL: return 0x01;
    case AREA_DISCRETE_INPUT: return 0x02;
    default: return 0x03;  // holding register
  }
}

// Reads `qty` values from `slaveId` at `startAddr` into outRegs[]: 16-bit
// registers for holding/input areas (qty 1-2), single bits (0/1) for
// coil/discrete areas. All four read function codes share the same 8-byte
// request frame; only the response data length differs (2 bytes per
// register vs one packed byte per 8 bits). Must be called with serialMutex
// held. Enforces T3.5/T1.5 spec timing; recognizes a complete response by
// expected frame length. Returns MB_SUCCESS or an MB_ERR_* code.
uint8_t modbusRead(uint8_t slaveId, uint8_t area, uint16_t startAddr, uint16_t qty, uint16_t *outRegs) {
  uint8_t fc = modbusFunctionForArea(area);
  uint8_t expectedDataBytes = isBitArea(area) ? ((qty + 7) / 8) : (qty * 2);

  uint8_t req[8];

  req[0] = slaveId;
  req[1] = fc;
  req[2] = highByte(startAddr);
  req[3] = lowByte(startAddr);
  req[4] = highByte(qty);
  req[5] = lowByte(qty);

  uint16_t reqCrc = modbusCRC16(req, 6);
  req[6] = reqCrc & 0xFF;
  req[7] = (reqCrc >> 8) & 0xFF;

  memcpy(lastModbusTx, req, sizeof(req));
  lastModbusTxLen = sizeof(req);

  // Enforce T3.5 minimum inter-frame silence before transmitting.
  uint32_t t35 = modbusT35Us();
  uint32_t sinceActivity = micros() - lastBusActivityUs;
  if (sinceActivity < t35) {
    delayMicroseconds(t35 - sinceActivity);
  }

  while (Serial1.available()) {
    Serial1.read();
  }

  preTransmission();
  Serial1.write(req, sizeof(req));
  Serial1.flush();
  postTransmission();

  lastBusActivityUs = micros();

  uint8_t rx[MODBUS_RAW_BUF_SIZE];
  uint8_t rxLen = 0;
  uint8_t expectedLen = 0;
  bool rxOverflow = false;
  uint32_t lastByteUs = lastBusActivityUs;
  uint32_t t15 = modbusT15Us();

  unsigned long deadline = millis() + MODBUS_RESPONSE_TIMEOUT_MS;

  while ((long)(millis() - deadline) < 0) {
    if (Serial1.available()) {
      uint8_t b = Serial1.read();
      uint32_t now = micros();

      // A gap exceeding T1.5 marks a new frame boundary - discard and restart.
      if (rxLen > 0 && (now - lastByteUs) > t15) {
        rxLen = 0;
        expectedLen = 0;
      }

      lastByteUs = now;

      if (rxLen < MODBUS_RAW_BUF_SIZE) {
        rx[rxLen++] = b;
      } else {
        rxOverflow = true;
      }

      if (rxLen == 2) {
        // Function code with the high bit set means a slave exception -
        // a fixed, shorter frame than the normal success reply.
        expectedLen = (rx[1] & 0x80) ? 5 : (5 + expectedDataBytes);
      }

      if (expectedLen != 0 && rxLen >= expectedLen) {
        break;
      }
    } else if (rxLen == 0) {
      // Not mid-frame yet, so it's safe to yield during this potentially long wait.
      vTaskDelay(1);
    }
    // else: mid-frame - busy-poll so the T1.5 gap check stays microsecond-accurate.
  }

  lastBusActivityUs = (rxLen > 0) ? lastByteUs : micros();

  memcpy(lastModbusRx, rx, rxLen);
  lastModbusRxLen = rxLen;
  lastModbusRxOverflow = rxOverflow;

  if (rxLen == 0 || expectedLen == 0 || rxLen < expectedLen) {
    return MB_ERR_TIMEOUT;
  }

  uint16_t receivedCrc = rx[expectedLen - 2] | ((uint16_t)rx[expectedLen - 1] << 8);
  uint16_t calcCrc = modbusCRC16(rx, expectedLen - 2);

  if (receivedCrc != calcCrc) {
    return MB_ERR_CRC;
  }

  if (rx[0] != slaveId) {
    return MB_ERR_WRONG_SLAVE;
  }

  if (rx[1] & 0x80) {
    return rx[2];  // raw Modbus exception code from the slave
  }

  if (rx[1] != fc || rx[2] != expectedDataBytes) {
    return MB_ERR_CRC;
  }

  if (isBitArea(area)) {
    // Bits arrive packed LSB-first: bit r of the response's data bytes.
    for (uint16_t r = 0; r < qty; r++) {
      outRegs[r] = (rx[3 + r / 8] >> (r % 8)) & 0x01;
    }
  } else {
    for (uint16_t r = 0; r < qty; r++) {
      outRegs[r] = ((uint16_t)rx[3 + r * 2] << 8) | rx[3 + r * 2 + 1];
    }
  }

  return MB_SUCCESS;
}

// Writes `qty` values to `slaveId` at `startAddr`: FC 0x05 (write single
// coil) for a 1-bit area, FC 0x06 (write single register, qty==1) or
// FC 0x10 (write multiple registers, qty==2 - e.g. FLOAT32) for register
// areas. Must be called with serialMutex held. Same T3.5/T1.5 timing and
// response validation as modbusRead(); write responses are a fixed-length
// echo of the request (no data to unpack). Returns MB_SUCCESS or an
// MB_ERR_* code / raw slave exception code, same conventions as modbusRead().
uint8_t modbusWrite(uint8_t slaveId, uint8_t area, uint16_t startAddr, uint16_t qty, const uint16_t *regs) {
  uint8_t req[13];
  uint8_t reqLen;
  uint8_t fc;

  if (isBitArea(area)) {
    fc = 0x05;
    req[0] = slaveId;
    req[1] = fc;
    req[2] = highByte(startAddr);
    req[3] = lowByte(startAddr);
    req[4] = regs[0] ? 0xFF : 0x00;
    req[5] = 0x00;
    reqLen = 6;
  } else if (qty <= 1) {
    fc = 0x06;
    req[0] = slaveId;
    req[1] = fc;
    req[2] = highByte(startAddr);
    req[3] = lowByte(startAddr);
    req[4] = highByte(regs[0]);
    req[5] = lowByte(regs[0]);
    reqLen = 6;
  } else {
    fc = 0x10;
    req[0] = slaveId;
    req[1] = fc;
    req[2] = highByte(startAddr);
    req[3] = lowByte(startAddr);
    req[4] = highByte(qty);
    req[5] = lowByte(qty);
    req[6] = (uint8_t)(qty * 2);
    for (uint16_t r = 0; r < qty; r++) {
      req[7 + r * 2] = highByte(regs[r]);
      req[8 + r * 2] = lowByte(regs[r]);
    }
    reqLen = 7 + qty * 2;
  }

  // All three write responses are fixed-length: slave ID + FC + 4 data
  // bytes (echoed address+value, or address+quantity) + 2 CRC bytes.
  uint8_t expectedRespLen = 8;

  uint16_t reqCrc = modbusCRC16(req, reqLen);
  req[reqLen] = reqCrc & 0xFF;
  req[reqLen + 1] = (reqCrc >> 8) & 0xFF;
  reqLen += 2;

  memcpy(lastModbusTx, req, reqLen);
  lastModbusTxLen = reqLen;

  uint32_t t35 = modbusT35Us();
  uint32_t sinceActivity = micros() - lastBusActivityUs;
  if (sinceActivity < t35) {
    delayMicroseconds(t35 - sinceActivity);
  }

  while (Serial1.available()) {
    Serial1.read();
  }

  preTransmission();
  Serial1.write(req, reqLen);
  Serial1.flush();
  postTransmission();

  lastBusActivityUs = micros();

  uint8_t rx[MODBUS_RAW_BUF_SIZE];
  uint8_t rxLen = 0;
  uint8_t expectedLen = 0;
  uint32_t lastByteUs = lastBusActivityUs;
  uint32_t t15 = modbusT15Us();

  unsigned long deadline = millis() + MODBUS_RESPONSE_TIMEOUT_MS;

  while ((long)(millis() - deadline) < 0) {
    if (Serial1.available()) {
      uint8_t b = Serial1.read();
      uint32_t now = micros();

      if (rxLen > 0 && (now - lastByteUs) > t15) {
        rxLen = 0;
        expectedLen = 0;
      }

      lastByteUs = now;

      if (rxLen < MODBUS_RAW_BUF_SIZE) {
        rx[rxLen++] = b;
      }

      if (rxLen == 2) {
        expectedLen = (rx[1] & 0x80) ? 5 : expectedRespLen;
      }

      if (expectedLen != 0 && rxLen >= expectedLen) {
        break;
      }
    } else if (rxLen == 0) {
      vTaskDelay(1);
    }
  }

  lastBusActivityUs = (rxLen > 0) ? lastByteUs : micros();

  memcpy(lastModbusRx, rx, rxLen);
  lastModbusRxLen = rxLen;
  lastModbusRxOverflow = false;

  if (rxLen == 0 || expectedLen == 0 || rxLen < expectedLen) {
    return MB_ERR_TIMEOUT;
  }

  uint16_t receivedCrc = rx[expectedLen - 2] | ((uint16_t)rx[expectedLen - 1] << 8);
  uint16_t calcCrc = modbusCRC16(rx, expectedLen - 2);

  if (receivedCrc != calcCrc) {
    return MB_ERR_CRC;
  }

  if (rx[0] != slaveId) {
    return MB_ERR_WRONG_SLAVE;
  }

  if (rx[1] & 0x80) {
    return rx[2];  // raw Modbus exception code from the slave
  }

  if (rx[1] != fc) {
    return MB_ERR_CRC;
  }

  return MB_SUCCESS;
}

// ===================== Modbus TCP Client (second data source) =====================
// Ensures tcpTargetClients[targetIndex] is connected to tcpTargets[targetIndex],
// (re)connecting if needed. Only called from tcpPollTask (Core 0) - a TCP
// connect can block for MODBUS_TCP_CONNECT_TIMEOUT_MS, which is fine here
// since this task has nothing time-critical to protect, unlike Core 1.
bool ensureTcpTargetConnected(int targetIndex) {
  WiFiClient &client = tcpTargetClients[targetIndex];

  if (client.connected()) {
    return true;
  }

  if (WiFi.status() != WL_CONNECTED) {
    return false;
  }

  return client.connect(tcpTargets[targetIndex].ip.c_str(), tcpTargets[targetIndex].port, MODBUS_TCP_CONNECT_TIMEOUT_MS);
}

// Modbus TCP transaction ID, incremented per request so responses can be
// matched to their request (required by the spec, though this gateway only
// ever has one request in flight per target at a time).
uint16_t tcpTransactionId = 0;

// Same read semantics as modbusRead(), but framed as Modbus TCP (MBAP
// header instead of RTU slave-ID+CRC) over tcpTargets[targetIndex]'s
// connection. Returns MB_SUCCESS, an MB_ERR_* code, or a raw slave
// exception code, exactly like modbusRead().
uint8_t modbusTcpRead(int targetIndex, uint8_t area, uint16_t startAddr, uint16_t qty, uint16_t *outRegs) {
  if (targetIndex < 0 || targetIndex >= tcpTargetCount) {
    return MB_ERR_TCP_UNCONFIGURED;
  }

  if (!ensureTcpTargetConnected(targetIndex)) {
    return MB_ERR_TIMEOUT;
  }

  WiFiClient &client = tcpTargetClients[targetIndex];
  uint8_t unitId = tcpTargets[targetIndex].unitId;

  uint8_t fc = modbusFunctionForArea(area);
  uint8_t expectedDataBytes = isBitArea(area) ? ((qty + 7) / 8) : (qty * 2);

  tcpTransactionId++;

  uint8_t req[12];
  req[0] = highByte(tcpTransactionId);
  req[1] = lowByte(tcpTransactionId);
  req[2] = 0x00;  // Protocol ID - always 0 for Modbus
  req[3] = 0x00;
  req[4] = 0x00;  // Length (of Unit ID + PDU that follows)
  req[5] = 0x06;
  req[6] = unitId;
  req[7] = fc;
  req[8] = highByte(startAddr);
  req[9] = lowByte(startAddr);
  req[10] = highByte(qty);
  req[11] = lowByte(qty);

  // Drain any stale bytes left over from a previous transaction before
  // sending a new request, same defensive purpose as the RTU path's
  // Serial1.available() drain.
  while (client.available()) {
    client.read();
  }

  client.write(req, sizeof(req));
  client.flush();

  uint8_t rx[7 + MODBUS_RAW_BUF_SIZE];
  uint8_t rxLen = 0;
  uint16_t expectedTotalLen = 0;  // full MBAP+PDU length, once the header tells us

  unsigned long deadline = millis() + MODBUS_TCP_RESPONSE_TIMEOUT_MS;

  while ((long)(millis() - deadline) < 0) {
    if (client.available()) {
      if (rxLen < sizeof(rx)) {
        rx[rxLen++] = (uint8_t)client.read();
      } else {
        client.read();  // discard - response longer than our buffer, will fail length check below
      }

      if (rxLen == 8) {
        // MBAP header's own Length field covers Unit ID + PDU (from byte 6
        // onward); the header itself is 6 bytes (bytes 0-5), so total frame
        // length is 6 + that value.
        uint16_t mbapLen = ((uint16_t)rx[4] << 8) | rx[5];
        expectedTotalLen = 6 + mbapLen;
      }

      if (expectedTotalLen != 0 && rxLen >= expectedTotalLen) {
        break;
      }
    } else {
      vTaskDelay(1);
    }
  }

  if (rxLen == 0 || expectedTotalLen == 0 || rxLen < expectedTotalLen) {
    client.stop();  // stale/partial TCP session - start clean next attempt
    return MB_ERR_TIMEOUT;
  }

  uint16_t respTxId = ((uint16_t)rx[0] << 8) | rx[1];
  if (respTxId != tcpTransactionId) {
    return MB_ERR_CRC;  // mismatched transaction - treat as a corrupt/unexpected reply
  }

  if (rx[6] != unitId) {
    return MB_ERR_WRONG_SLAVE;
  }

  if (rx[7] & 0x80) {
    return rx[8];  // raw Modbus exception code from the target
  }

  if (rx[7] != fc || rx[8] != expectedDataBytes) {
    return MB_ERR_CRC;
  }

  if (isBitArea(area)) {
    for (uint16_t r = 0; r < qty; r++) {
      outRegs[r] = (rx[9 + r / 8] >> (r % 8)) & 0x01;
    }
  } else {
    for (uint16_t r = 0; r < qty; r++) {
      outRegs[r] = ((uint16_t)rx[9 + r * 2] << 8) | rx[9 + r * 2 + 1];
    }
  }

  return MB_SUCCESS;
}

// Same write semantics as modbusWrite() (FC 0x05/0x06/0x10), framed as
// Modbus TCP (MBAP header) over tcpTargets[targetIndex]'s connection.
// Returns MB_SUCCESS, an MB_ERR_* code, or a raw slave exception code.
uint8_t modbusTcpWrite(int targetIndex, uint8_t area, uint16_t startAddr, uint16_t qty, const uint16_t *regs) {
  if (targetIndex < 0 || targetIndex >= tcpTargetCount) {
    return MB_ERR_TCP_UNCONFIGURED;
  }

  if (!ensureTcpTargetConnected(targetIndex)) {
    return MB_ERR_TIMEOUT;
  }

  WiFiClient &client = tcpTargetClients[targetIndex];
  uint8_t unitId = tcpTargets[targetIndex].unitId;

  uint8_t pdu[11];
  uint8_t pduLen;
  uint8_t fc;

  if (isBitArea(area)) {
    fc = 0x05;
    pdu[0] = fc;
    pdu[1] = highByte(startAddr);
    pdu[2] = lowByte(startAddr);
    pdu[3] = regs[0] ? 0xFF : 0x00;
    pdu[4] = 0x00;
    pduLen = 5;
  } else if (qty <= 1) {
    fc = 0x06;
    pdu[0] = fc;
    pdu[1] = highByte(startAddr);
    pdu[2] = lowByte(startAddr);
    pdu[3] = highByte(regs[0]);
    pdu[4] = lowByte(regs[0]);
    pduLen = 5;
  } else {
    fc = 0x10;
    pdu[0] = fc;
    pdu[1] = highByte(startAddr);
    pdu[2] = lowByte(startAddr);
    pdu[3] = highByte(qty);
    pdu[4] = lowByte(qty);
    pdu[5] = (uint8_t)(qty * 2);
    for (uint16_t r = 0; r < qty; r++) {
      pdu[6 + r * 2] = highByte(regs[r]);
      pdu[7 + r * 2] = lowByte(regs[r]);
    }
    pduLen = 6 + qty * 2;
  }

  tcpTransactionId++;

  uint8_t req[7 + 11];
  req[0] = highByte(tcpTransactionId);
  req[1] = lowByte(tcpTransactionId);
  req[2] = 0x00;
  req[3] = 0x00;
  req[4] = highByte((uint16_t)(1 + pduLen));
  req[5] = lowByte((uint16_t)(1 + pduLen));
  req[6] = unitId;
  memcpy(&req[7], pdu, pduLen);

  uint16_t totalReqLen = 7 + pduLen;

  while (client.available()) {
    client.read();
  }

  client.write(req, totalReqLen);
  client.flush();

  uint8_t rx[7 + MODBUS_RAW_BUF_SIZE];
  uint8_t rxLen = 0;
  uint16_t expectedTotalLen = 0;

  unsigned long deadline = millis() + MODBUS_TCP_RESPONSE_TIMEOUT_MS;

  while ((long)(millis() - deadline) < 0) {
    if (client.available()) {
      if (rxLen < sizeof(rx)) {
        rx[rxLen++] = (uint8_t)client.read();
      } else {
        client.read();
      }

      if (rxLen == 8) {
        uint16_t mbapLen = ((uint16_t)rx[4] << 8) | rx[5];
        expectedTotalLen = 6 + mbapLen;
      }

      if (expectedTotalLen != 0 && rxLen >= expectedTotalLen) {
        break;
      }
    } else {
      vTaskDelay(1);
    }
  }

  if (rxLen == 0 || expectedTotalLen == 0 || rxLen < expectedTotalLen) {
    client.stop();
    return MB_ERR_TIMEOUT;
  }

  uint16_t respTxId = ((uint16_t)rx[0] << 8) | rx[1];
  if (respTxId != tcpTransactionId) {
    return MB_ERR_CRC;
  }

  if (rx[6] != unitId) {
    return MB_ERR_WRONG_SLAVE;
  }

  if (rx[7] & 0x80) {
    return rx[8];
  }

  if (rx[7] != fc) {
    return MB_ERR_CRC;
  }

  return MB_SUCCESS;
}

// ===================== Serial Config =====================
uint32_t getSerialConfig() {
  if (commParity == "E" && commStopBits == 1) return SERIAL_8E1;
  if (commParity == "E" && commStopBits == 2) return SERIAL_8E2;

  if (commParity == "O" && commStopBits == 1) return SERIAL_8O1;
  if (commParity == "O" && commStopBits == 2) return SERIAL_8O2;

  if (commParity == "N" && commStopBits == 2) return SERIAL_8N2;

  return SERIAL_8N1;
}

// ===================== Escape Functions =====================
String htmlEscape(String text) {
  text.replace("&", "&amp;");
  text.replace("<", "&lt;");
  text.replace(">", "&gt;");
  text.replace("\"", "&quot;");
  text.replace("'", "&#39;");
  return text;
}

String jsonEscape(String text) {
  text.replace("\\", "\\\\");
  text.replace("\"", "\\\"");
  text.replace("\n", "\\n");
  text.replace("\r", "\\r");
  return text;
}

// ===================== Type Helpers =====================
String getBaseFormatFromType(String typeName) {
  if (typeName.startsWith("INT_16")) return "INT16";
  if (typeName.startsWith("UINT_16")) return "UINT16";
  if (typeName == "FLOAT_16") return "FLOAT16";
  if (typeName == "FLOAT_32") return "FLOAT32";
  return "UINT16";
}

uint16_t inferDataLengthFromType(String typeName) {
  if (typeName == "FLOAT_32") return 2;
  return 1;
}

int findTypeIndex(String typeName) {
  for (int i = 0; i < typeCount; i++) {
    if (typeList[i].name == typeName) {
      return i;
    }
  }
  return -1;
}

uint16_t getDataLengthForType(String typeName) {
  int index = findTypeIndex(typeName);

  if (index >= 0) {
    return typeList[index].dataLength;
  }

  return inferDataLengthFromType(typeName);
}

// ===================== Communication Settings Save / Load =====================
// Returns true only if every NVS write succeeded - callers use this to
// decide whether the web UI shows "Saved" or a flash-full warning, instead
// of always claiming success once the write is merely attempted.
bool saveCommunicationSettings() {
  nvsWriteFailures = 0;
  preferences.begin("comm", false);

  trackNvsWrite(preferences.putUInt("baud", commBaudRate));
  trackNvsWrite(preferences.putString("parity", commParity));
  trackNvsWrite(preferences.putUChar("stop", commStopBits));
  trackNvsWrite(preferences.putUInt("pollms", pollIntervalMs));
  trackNvsWrite(preferences.putUInt("recoveryms", slaveRecoveryDelayMs));

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (comm): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, settings may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadCommunicationSettings() {
  preferences.begin("comm", true);

  commBaudRate = preferences.getUInt("baud", 9600);
  commParity = preferences.getString("parity", "N");
  commStopBits = preferences.getUChar("stop", 1);
  pollIntervalMs = preferences.getUInt("pollms", DEFAULT_POLL_INTERVAL_MS);
  slaveRecoveryDelayMs = preferences.getUInt("recoveryms", DEFAULT_SLAVE_RECOVERY_DELAY_MS);

  preferences.end();

  if (commBaudRate < 300 || commBaudRate > 1000000) {
    commBaudRate = 9600;
  }

  if (commParity != "N" && commParity != "E" && commParity != "O") {
    commParity = "N";
  }

  if (commStopBits != 1 && commStopBits != 2) {
    commStopBits = 1;
  }

  if (pollIntervalMs < MIN_POLL_INTERVAL_MS || pollIntervalMs > MAX_POLL_INTERVAL_MS) {
    pollIntervalMs = DEFAULT_POLL_INTERVAL_MS;
  }

  if (slaveRecoveryDelayMs > MAX_SLAVE_RECOVERY_DELAY_MS) {
    slaveRecoveryDelayMs = DEFAULT_SLAVE_RECOVERY_DELAY_MS;
  }
}

// ===================== TCP Uplink Config Save / Load =====================
bool saveUplinkConfig() {
  logMessage("========== SAVE TCP CONFIG START ==========");

  logMessage("Saving SSID: " + uplinkConfig.ssid);
  logMessage("Saving Password Length: " + String(uplinkConfig.password.length()));
  logMessage("Saving Server IP: " + uplinkConfig.serverIP);
  logMessage("Saving Port: " + String(uplinkConfig.port));

  nvsWriteFailures = 0;
  preferences.begin("uplink", false);

  trackNvsWrite(preferences.putString("ssid", uplinkConfig.ssid));
  trackNvsWrite(preferences.putString("pass", uplinkConfig.password));
  trackNvsWrite(preferences.putString("ip", uplinkConfig.serverIP));
  trackNvsWrite(preferences.putInt("port", uplinkConfig.port));
  trackNvsWrite(preferences.putString("ntp", uplinkConfig.ntpServer));

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (uplink): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, settings may not persist");
  }

  logMessage("TCP CONFIG SAVED TO FLASH");
  logMessage("========== SAVE TCP CONFIG END ==========");

  return nvsWriteFailures == 0;
}

void loadUplinkConfig() {
  logMessage("========== LOAD TCP CONFIG START ==========");

  preferences.begin("uplink", true);

  uplinkConfig.ssid = preferences.getString("ssid", "");
  uplinkConfig.password = preferences.getString("pass", "");
  uplinkConfig.serverIP = preferences.getString("ip", "");
  uplinkConfig.port = preferences.getInt("port", 0);
  uplinkConfig.ntpServer = preferences.getString("ntp", "");

  preferences.end();

  if (uplinkConfig.port <= 0 || uplinkConfig.port > 65535) {
    uplinkConfig.port = 5000;
  }

  logMessage("Loaded SSID: " + uplinkConfig.ssid);
  logMessage("Loaded Password Length: " + String(uplinkConfig.password.length()));
  logMessage("Loaded Server IP: " + uplinkConfig.serverIP);
  logMessage("Loaded Port: " + String(uplinkConfig.port));

  if (uplinkConfig.serverIP.length() == 0) {
    logMessage("WARNING: TCP SERVER IP IS EMPTY");
  }

  if (uplinkConfig.ssid.length() == 0) {
    logMessage("WARNING: WIFI SSID IS EMPTY");
  }

  logMessage("========== LOAD TCP CONFIG END ==========");
}

// ===================== NTP Start / Restart =====================
// (Re)starts background SNTP with the configured server priority:
// on-premise server first when one is set, then the public pool servers.
// Static buffer because lwIP's SNTP keeps the server-name POINTER, not a
// copy - a temporary String::c_str() here would dangle after return.
void startNtp() {
  static char onPremNtpBuf[NTP_SERVER_MAX_LEN];

  if (uplinkConfig.ntpServer.length() > 0) {
    uplinkConfig.ntpServer.toCharArray(onPremNtpBuf, sizeof(onPremNtpBuf));
    configTime(0, 0, onPremNtpBuf, NTP_SERVER_1, NTP_SERVER_2);
    logMessage("NTP PRIORITY: " + String(onPremNtpBuf) + " > " + NTP_SERVER_1 + " > " + NTP_SERVER_2);
  } else {
    configTime(0, 0, NTP_SERVER_1, NTP_SERVER_2);
    logMessage("NTP PRIORITY: " + String(NTP_SERVER_1) + " > " + NTP_SERVER_2 + " (no on-premise server set)");
  }
}

// ===================== Cloud Uplink Config Save / Load =====================
bool saveCloudConfig() {
  nvsWriteFailures = 0;
  preferences.begin("cloud", false);

  trackNvsWrite(preferences.putUChar("mode", cloudConfig.mode));
  trackNvsWrite(preferences.putString("ep", cloudConfig.endpoint));
  trackNvsWrite(preferences.putInt("port", cloudConfig.port));
  trackNvsWrite(preferences.putString("cid", cloudConfig.clientId));
  trackNvsWrite(preferences.putString("topic", cloudConfig.topic));
  trackNvsWrite(preferences.putBool("cmdEn", cloudConfig.commandsEnabled));

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (cloud): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, settings may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadCloudConfig() {
  preferences.begin("cloud", true);

  cloudConfig.mode = preferences.getUChar("mode", UPLINK_MODE_TCP);
  cloudConfig.endpoint = preferences.getString("ep", "");
  cloudConfig.port = preferences.getInt("port", DEFAULT_MQTT_PORT);
  cloudConfig.clientId = preferences.getString("cid", "");
  cloudConfig.topic = preferences.getString("topic", DEFAULT_MQTT_TOPIC);
  // Falls back to the old Shadow-channel enable flag so a device upgraded
  // from the Shadow firmware keeps its command channel on without a
  // reconfigure - the semantics carried over (inbound MQTT commands).
  cloudConfig.commandsEnabled = preferences.getBool("cmdEn", preferences.getBool("shadowEn", false));

  preferences.end();

  if (cloudConfig.mode != UPLINK_MODE_MQTT) {
    cloudConfig.mode = UPLINK_MODE_TCP;
  }

  if (cloudConfig.port <= 0 || cloudConfig.port > 65535) {
    cloudConfig.port = DEFAULT_MQTT_PORT;
  }

  if (cloudConfig.topic.length() == 0) {
    cloudConfig.topic = DEFAULT_MQTT_TOPIC;
  }
}

// Certs are in their own namespace: PEM blocks are ~1.2-1.7KB each, well
// within the NVS per-string limit but worth keeping apart from small keys.
bool saveCerts() {
  // These three PEM strings are the largest single consumer of NVS space
  // in the whole gateway (often 1-2KB each) - the most likely place an
  // out-of-space failure first shows up.
  nvsWriteFailures = 0;
  preferences.begin("certs", false);

  trackNvsWrite(preferences.putString("ca", certRootCA));
  trackNvsWrite(preferences.putString("cert", certDevice));
  trackNvsWrite(preferences.putString("key", certPrivKey));

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (certs): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, certs may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadCerts() {
  preferences.begin("certs", true);

  certRootCA = preferences.getString("ca", "");
  certDevice = preferences.getString("cert", "");
  certPrivKey = preferences.getString("key", "");

  preferences.end();
}

// ===================== Device / AP Identity Save / Load =====================
bool saveDeviceConfig() {
  nvsWriteFailures = 0;
  preferences.begin("device", false);

  trackNvsWrite(preferences.putString("apssid", deviceConfig.apSsid));
  trackNvsWrite(preferences.putString("prefix", deviceConfig.deviceNamePrefix));
  trackNvsWrite(preferences.putBool("apmatch", deviceConfig.apMatchesDeviceName));

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (device): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, settings may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadDeviceConfig() {
  preferences.begin("device", true);

  deviceConfig.apSsid = preferences.getString("apssid", DEFAULT_AP_SSID);
  deviceConfig.deviceNamePrefix = preferences.getString("prefix", "");
  deviceConfig.apMatchesDeviceName = preferences.getBool("apmatch", false);

  preferences.end();

  if (deviceConfig.apSsid.length() == 0) {
    deviceConfig.apSsid = DEFAULT_AP_SSID;
  }

  if (deviceConfig.deviceNamePrefix.length() > DEVICE_NAME_PREFIX_MAX_LEN) {
    deviceConfig.deviceNamePrefix = deviceConfig.deviceNamePrefix.substring(0, DEVICE_NAME_PREFIX_MAX_LEN);
  }
}

// ===================== Cellular Modem Config Save / Load =====================
bool saveCellularConfig() {
  nvsWriteFailures = 0;
  preferences.begin("cellular", false);

  trackNvsWrite(preferences.putString("apn", cellularConfig.apn));
  trackNvsWrite(preferences.putUInt("pubint", cellularConfig.publishIntervalMs));

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (cellular): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, settings may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadCellularConfig() {
  preferences.begin("cellular", true);
  cellularConfig.apn = preferences.getString("apn", "");
  cellularConfig.publishIntervalMs = preferences.getUInt("pubint", CELLULAR_PUBLISH_INTERVAL_DEFAULT_MS);
  preferences.end();

  if (cellularConfig.publishIntervalMs < CELLULAR_PUBLISH_INTERVAL_MIN_MS
      || cellularConfig.publishIntervalMs > CELLULAR_PUBLISH_INTERVAL_MAX_MS) {
    cellularConfig.publishIntervalMs = CELLULAR_PUBLISH_INTERVAL_DEFAULT_MS;
  }
}

// ===================== WiFi Uplink =====================
void connectUplinkWiFi() {
  if (uplinkConfig.ssid.length() == 0) {
    logMessage("UPLINK WIFI NOT CONFIGURED");
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  logMessage("UPLINK WIFI TRY: " + uplinkConfig.ssid);

  WiFi.begin(
    uplinkConfig.ssid.c_str(),
    uplinkConfig.password.c_str());
}

void handleUplinkWiFiRetry() {
  if (uplinkConfig.ssid.length() == 0) {
    return;
  }

  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  if (millis() - lastWifiAttempt > WIFI_RETRY_INTERVAL_MS) {
    lastWifiAttempt = millis();

    logMessage("UPLINK WIFI RETRY");

    // Throttled so a prolonged outage doesn't flood the key log, but still
    // gives the field tech a periodic "yes, still trying and failing" signal
    // instead of silence.
    if (millis() - lastWifiFailLogTime >= DEBUG_ERROR_REPEAT_MS) {
      lastWifiFailLogTime = millis();
      logKeyEvent("WIFI NOT CONNECTED - retrying: " + uplinkConfig.ssid);
    }

    WiFi.disconnect();
    delay(100);

    WiFi.begin(
      uplinkConfig.ssid.c_str(),
      uplinkConfig.password.c_str());
  }
}

// Logs a key event only on WiFi connect/disconnect transitions, not on every
// retry attempt, so the field-facing log stays short and meaningful.
void checkWifiUplinkStatusChange() {
  if (uplinkConfig.ssid.length() == 0) {
    return;
  }

  bool connected = (WiFi.status() == WL_CONNECTED);

  if (connected != lastWifiUplinkConnected) {
    if (connected) {
      logKeyEvent("WIFI CONNECTED: " + WiFi.localIP().toString());
      wifiUpSince = millis();
      wifiDownSince = 0;
    } else {
      logKeyEvent("WIFI DISCONNECTED");
      wifiDownSince = millis();
      wifiUpSince = 0;
    }
    lastWifiUplinkConnected = connected;
  }
}

// Decides whether the MQTT uplink should be on WiFi (primary) or cellular
// (failover), based on how long WiFi has been continuously down/up - see
// UPLINK_FAILOVER_THRESHOLD_MS. Only meaningful in MQTT mode; raw TCP mode
// always stays on WiFi (see the activeUplinkTransport comment).
void updateMqttFailoverState() {
  bool wifiUp = (WiFi.status() == WL_CONNECTED);

  if (!mqttFailoverActive) {
    if (!wifiUp && wifiDownSince != 0 && millis() - wifiDownSince >= UPLINK_FAILOVER_THRESHOLD_MS) {
      mqttFailoverActive = true;
      logKeyEvent("UPLINK FAILOVER: MQTT switching to cellular (WiFi down " + String(UPLINK_FAILOVER_THRESHOLD_MS / 60000) + "+ min)");
    }
  } else {
    if (wifiUp && wifiUpSince != 0 && millis() - wifiUpSince >= UPLINK_FAILOVER_THRESHOLD_MS) {
      mqttFailoverActive = false;
      logKeyEvent("UPLINK FAILBACK: MQTT switching back to WiFi (stable " + String(UPLINK_FAILOVER_THRESHOLD_MS / 60000) + "+ min)");
    }
  }
}

// ===================== Default Types =====================
void loadDefaultTypes() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  typeCount = 8;

  typeList[0] = { "INT_16/10", "INT16", 10, 1 };
  typeList[1] = { "INT_16/100", "INT16", 100, 1 };
  typeList[2] = { "INT_16/1000", "INT16", 1000, 1 };

  typeList[3] = { "UINT_16/10", "UINT16", 10, 1 };
  typeList[4] = { "UINT_16/100", "UINT16", 100, 1 };
  typeList[5] = { "UINT_16/1000", "UINT16", 1000, 1 };

  typeList[6] = { "FLOAT_16", "FLOAT16", 1, 1 };
  typeList[7] = { "FLOAT_32", "FLOAT32", 1, 2 };

  xSemaphoreGive(dataMutex);
}

// ===================== Save / Load Types =====================
// Normalizes/derives fields after typeList[i].name/divisor have been
// populated from either storage format.
void finalizeLoadedTypeRow(int i) {
  if (typeList[i].name.length() == 0) {
    typeList[i].name = "INT_16/100";
  }

  if (typeList[i].divisor <= 0) {
    typeList[i].divisor = 1;
  }

  typeList[i].baseFormat = getBaseFormatFromType(typeList[i].name);
  typeList[i].dataLength = inferDataLengthFromType(typeList[i].name);
}

bool saveTypes() {
  nvsWriteFailures = 0;
  preferences.begin("types", false);

  // Wipes any leftover keys from the old one-key-per-field format so a
  // migrated config actually reclaims that space.
  preferences.clear();

  trackNvsWrite(preferences.putUInt("schema", TYPE_SCHEMA_VERSION));
  trackNvsWrite(preferences.putInt("count", typeCount));

  if (typeCount > 0) {
    StoredParamTypeRow *rows = new StoredParamTypeRow[typeCount];

    for (int i = 0; i < typeCount; i++) {
      memset(&rows[i], 0, sizeof(StoredParamTypeRow));
      typeList[i].name.toCharArray(rows[i].name, STORED_TYPE_LEN);
      rows[i].divisor = typeList[i].divisor;
    }

    size_t expectedLen = (size_t)typeCount * sizeof(StoredParamTypeRow);
    size_t written = preferences.putBytes("rows", rows, expectedLen);
    if (written != expectedLen) {
      nvsWriteFailures++;
    }

    delete[] rows;
  }

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (types): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, settings may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadTypes() {
  preferences.begin("types", true);

  uint32_t savedSchema = preferences.getUInt("schema", 0);
  typeCount = preferences.getInt("count", 0);

  if (savedSchema != TYPE_SCHEMA_VERSION || typeCount <= 0 || typeCount > MAX_PARAM_TYPES) {
    preferences.end();
    loadDefaultTypes();
    saveTypes();
    return;
  }

  size_t expectedLen = (size_t)typeCount * sizeof(StoredParamTypeRow);
  size_t blobLen = preferences.getBytesLength("rows");

  if (blobLen == expectedLen) {
    StoredParamTypeRow *rows = new StoredParamTypeRow[typeCount];
    preferences.getBytes("rows", rows, expectedLen);
    preferences.end();

    for (int i = 0; i < typeCount; i++) {
      rows[i].name[STORED_TYPE_LEN - 1] = '\0';
      typeList[i].name = String(rows[i].name);
      typeList[i].divisor = rows[i].divisor;
      finalizeLoadedTypeRow(i);
    }

    delete[] rows;
    return;
  }

  // No compact blob yet - a config saved by an earlier firmware version.
  // Read the old per-key layout, then re-save in the compact format.
  for (int i = 0; i < typeCount; i++) {
    String index = String(i);

    typeList[i].name = preferences.getString(("name" + index).c_str(), "INT_16/100");
    typeList[i].divisor = preferences.getFloat(("div" + index).c_str(), 100);
    finalizeLoadedTypeRow(i);
  }

  preferences.end();

  saveTypes();
  logKeyEvent("DATA TYPES MIGRATED to compact storage - freed flash space");
}

// ===================== Save / Load Modbus TCP Targets =====================
bool saveTcpTargets() {
  nvsWriteFailures = 0;
  preferences.begin("tcptgt", false);

  // Wipes any leftover keys from the old one-key-per-field format so a
  // migrated config actually reclaims that space.
  preferences.clear();

  trackNvsWrite(preferences.putUInt("schema", TCP_TARGET_SCHEMA_VERSION));
  trackNvsWrite(preferences.putInt("count", tcpTargetCount));

  if (tcpTargetCount > 0) {
    StoredTcpTargetRow *rows = new StoredTcpTargetRow[tcpTargetCount];

    for (int i = 0; i < tcpTargetCount; i++) {
      memset(&rows[i], 0, sizeof(StoredTcpTargetRow));
      tcpTargets[i].name.toCharArray(rows[i].name, STORED_NAME_LEN);
      tcpTargets[i].ip.toCharArray(rows[i].ip, STORED_IP_LEN);
      rows[i].port = tcpTargets[i].port;
      rows[i].unitId = tcpTargets[i].unitId;
    }

    size_t expectedLen = (size_t)tcpTargetCount * sizeof(StoredTcpTargetRow);
    size_t written = preferences.putBytes("rows", rows, expectedLen);
    if (written != expectedLen) {
      nvsWriteFailures++;
    }

    delete[] rows;
  }

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (tcptgt): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, TCP targets may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadTcpTargets() {
  preferences.begin("tcptgt", true);

  uint32_t savedSchema = preferences.getUInt("schema", 0);
  tcpTargetCount = preferences.getInt("count", 0);

  if (savedSchema != TCP_TARGET_SCHEMA_VERSION || tcpTargetCount < 0 || tcpTargetCount > MAX_TCP_TARGETS) {
    // No prior config at all (e.g. upgrading from a firmware version
    // before this feature existed) - default to zero targets, not sample
    // data, since there's no sensible generic default IP/unit ID to guess at.
    preferences.end();
    tcpTargetCount = 0;
    saveTcpTargets();
    return;
  }

  size_t expectedLen = (size_t)tcpTargetCount * sizeof(StoredTcpTargetRow);
  size_t blobLen = preferences.getBytesLength("rows");

  if (blobLen == expectedLen) {
    StoredTcpTargetRow *rows = new StoredTcpTargetRow[tcpTargetCount];
    preferences.getBytes("rows", rows, expectedLen);
    preferences.end();

    for (int i = 0; i < tcpTargetCount; i++) {
      rows[i].name[STORED_NAME_LEN - 1] = '\0';
      rows[i].ip[STORED_IP_LEN - 1] = '\0';

      tcpTargets[i].name = String(rows[i].name);
      tcpTargets[i].ip = String(rows[i].ip);
      tcpTargets[i].port = (rows[i].port == 0) ? 502 : rows[i].port;
      tcpTargets[i].unitId = rows[i].unitId;
    }

    delete[] rows;
    return;
  }

  // No compact blob yet - a config saved by the earlier per-key format
  // (this table's very first version). Read it, then re-save compactly.
  for (int i = 0; i < tcpTargetCount; i++) {
    String index = String(i);

    tcpTargets[i].name = preferences.getString(("name" + index).c_str(), "TCP Target " + String(i + 1));
    tcpTargets[i].ip = preferences.getString(("ip" + index).c_str(), "");
    tcpTargets[i].port = preferences.getUShort(("port" + index).c_str(), 502);
    tcpTargets[i].unitId = preferences.getUChar(("unit" + index).c_str(), 1);

    if (tcpTargets[i].port == 0) {
      tcpTargets[i].port = 502;
    }
  }

  preferences.end();

  saveTcpTargets();
  logKeyEvent("MODBUS TCP TARGETS MIGRATED to compact storage - freed flash space");
}

// ===================== Digital I/O Save / Load =====================
// Brand new feature (no earlier firmware version ever stored anything
// here), so unlike the tables above there's no legacy per-key format to
// migrate from - just a single "load compact blobs, or default+save if
// they don't exist/match yet" path.
#define DIGITAL_IO_SCHEMA_VERSION 1

struct StoredDiRow {
  char name[STORED_NAME_LEN];
  uint8_t enabled;
};

struct StoredDoRow {
  char name[STORED_NAME_LEN];
  uint8_t enabled;
};

struct StoredAiRow {
  char name[STORED_NAME_LEN];
  uint8_t enabled;
  float scale;
  float offset;
};

bool saveDigitalIOSettings() {
  nvsWriteFailures = 0;
  preferences.begin("digio", false);

  trackNvsWrite(preferences.putUInt("schema", DIGITAL_IO_SCHEMA_VERSION));

  StoredDiRow diRows[DI_CHANNEL_COUNT];
  StoredDoRow doRows[DO_CHANNEL_COUNT];
  StoredAiRow aiRows[AI_CHANNEL_COUNT];

  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    memset(&diRows[i], 0, sizeof(StoredDiRow));
    diChannels[i].name.toCharArray(diRows[i].name, STORED_NAME_LEN);
    diRows[i].enabled = diChannels[i].enabled ? 1 : 0;
  }

  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    memset(&doRows[i], 0, sizeof(StoredDoRow));
    doChannels[i].name.toCharArray(doRows[i].name, STORED_NAME_LEN);
    doRows[i].enabled = doChannels[i].enabled ? 1 : 0;
  }

  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    memset(&aiRows[i], 0, sizeof(StoredAiRow));
    aiChannels[i].name.toCharArray(aiRows[i].name, STORED_NAME_LEN);
    aiRows[i].enabled = aiChannels[i].enabled ? 1 : 0;
    aiRows[i].scale = aiChannels[i].scale;
    aiRows[i].offset = aiChannels[i].offset;
  }

  if (preferences.putBytes("di", diRows, sizeof(diRows)) != sizeof(diRows)) nvsWriteFailures++;
  if (preferences.putBytes("do", doRows, sizeof(doRows)) != sizeof(doRows)) nvsWriteFailures++;
  if (preferences.putBytes("ai", aiRows, sizeof(aiRows)) != sizeof(aiRows)) nvsWriteFailures++;

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (digio): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, I/O settings may not persist");
  }

  return nvsWriteFailures == 0;
}

void loadDigitalIOSettings() {
  preferences.begin("digio", true);

  uint32_t savedSchema = preferences.getUInt("schema", 0);
  bool freshOrMismatched = (savedSchema != DIGITAL_IO_SCHEMA_VERSION)
    || (preferences.getBytesLength("di") != sizeof(StoredDiRow) * DI_CHANNEL_COUNT)
    || (preferences.getBytesLength("do") != sizeof(StoredDoRow) * DO_CHANNEL_COUNT)
    || (preferences.getBytesLength("ai") != sizeof(StoredAiRow) * AI_CHANNEL_COUNT);

  if (freshOrMismatched) {
    preferences.end();
    loadDefaultDigitalIO();
    saveDigitalIOSettings();
    return;
  }

  StoredDiRow diRows[DI_CHANNEL_COUNT];
  StoredDoRow doRows[DO_CHANNEL_COUNT];
  StoredAiRow aiRows[AI_CHANNEL_COUNT];

  preferences.getBytes("di", diRows, sizeof(diRows));
  preferences.getBytes("do", doRows, sizeof(doRows));
  preferences.getBytes("ai", aiRows, sizeof(aiRows));

  preferences.end();

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    diRows[i].name[STORED_NAME_LEN - 1] = '\0';
    diChannels[i].name = String(diRows[i].name);
    diChannels[i].enabled = diRows[i].enabled != 0;
    diChannels[i].value = false;
    diChannels[i].lastUpdateTime = 0;
  }

  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    doRows[i].name[STORED_NAME_LEN - 1] = '\0';
    doChannels[i].name = String(doRows[i].name);
    doChannels[i].enabled = doRows[i].enabled != 0;
    doChannels[i].value = false;
    doChannels[i].lastWriteTime = 0;
  }

  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    aiRows[i].name[STORED_NAME_LEN - 1] = '\0';
    aiChannels[i].name = String(aiRows[i].name);
    aiChannels[i].enabled = aiRows[i].enabled != 0;
    aiChannels[i].scale = (aiRows[i].scale == 0) ? 1 : aiRows[i].scale;
    aiChannels[i].offset = aiRows[i].offset;
    aiChannels[i].rawValue = 0;
    aiChannels[i].value = 0;
    aiChannels[i].lastUpdateTime = 0;
  }

  xSemaphoreGive(dataMutex);
}

// ===================== Debug State =====================
void resetDebugState(int index) {
  params[index].lastLoggedValid = false;
  params[index].lastLoggedErrorCode = 255;
  params[index].lastErrorLogTime = 0;
}

// ===================== Default Modbus Settings =====================
void loadDefaultSettings() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  paramCount = 0;

  for (int i = 0; i < MAX_PARAMS; i++) {
    params[i].name = "";
    params[i].type = "";
    params[i].area = AREA_HOLDING_REGISTER;
    params[i].transport = TRANSPORT_RTU;
    params[i].slaveId = 1;
    params[i].tcpTargetIndex = 0;
    params[i].registerAddress = 0;
    params[i].registerLength = 1;
    params[i].enabled = false;
    params[i].value = 0;
    params[i].valid = false;
    params[i].errorCode = 0;
    params[i].lastUpdateTime = 0;
    params[i].writable = false;
    params[i].minWriteValue = 0;
    params[i].maxWriteValue = 0;
    params[i].lastWriteResult = 0xFF;
    params[i].lastWriteTime = 0;
    resetDebugState(i);
  }

  xSemaphoreGive(dataMutex);
}

void loadDefaultDigitalIO() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    diChannels[i].name = "DI" + String(i + 1);
    diChannels[i].enabled = false;
    diChannels[i].value = false;
    diChannels[i].lastUpdateTime = 0;
  }

  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    doChannels[i].name = "DO" + String(i + 1);
    doChannels[i].enabled = false;
    doChannels[i].value = false;
    doChannels[i].lastWriteTime = 0;
  }

  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    aiChannels[i].name = "AI" + String(i + 1);
    aiChannels[i].enabled = false;
    aiChannels[i].scale = 1;
    aiChannels[i].offset = 0;
    aiChannels[i].rawValue = 0;
    aiChannels[i].value = 0;
    aiChannels[i].lastUpdateTime = 0;
  }

  xSemaphoreGive(dataMutex);
}

// ===================== Save / Load Modbus Settings =====================
// Normalizes/derives fields after params[i].name/type/area/transport/
// slaveId/tcpTargetIndex/registerAddress/enabled have been populated from
// either storage format - shared by both load paths below so the
// validation logic (area/type/transport/target clamping, derived
// registerLength) only exists once.
void finalizeLoadedParamRow(int i) {
  if (params[i].area > AREA_DISCRETE_INPUT) {
    params[i].area = AREA_HOLDING_REGISTER;
  }

  if (isBitArea(params[i].area)) {
    // Fixed pseudo-type for coil/discrete rows - deliberately not run
    // through the type-list fallback below.
    params[i].type = "BIT";
  } else if (findTypeIndex(params[i].type) < 0 && typeCount > 0) {
    params[i].type = typeList[0].name;
  }

  if (params[i].transport > TRANSPORT_TCP) {
    params[i].transport = TRANSPORT_RTU;
  }

  if (params[i].tcpTargetIndex >= MAX_TCP_TARGETS) {
    params[i].tcpTargetIndex = 0;
  }

  // Defensive: a stored writable flag can't apply to a read-only area -
  // if the area itself was invalid/clamped above, don't carry it forward.
  if (!isWritableArea(params[i].area)) {
    params[i].writable = false;
  }

  params[i].registerLength = isBitArea(params[i].area) ? 1 : getDataLengthForType(params[i].type);
  params[i].value = 0;
  params[i].valid = false;
  params[i].errorCode = 0;
  params[i].lastUpdateTime = 0;
  params[i].lastWriteResult = 0xFF;
  params[i].lastWriteTime = 0;
  resetDebugState(i);
}

bool saveSettings() {
  nvsWriteFailures = 0;
  preferences.begin("modbus", false);

  // Wipes any leftover keys from the old one-key-per-field format (see
  // loadSettingsLegacyPerKey()) so a migrated config actually reclaims
  // that space instead of leaving hundreds of orphaned entries behind.
  preferences.clear();

  trackNvsWrite(preferences.putInt("count", paramCount));

  if (paramCount > 0) {
    StoredParamRow *rows = new StoredParamRow[paramCount];

    for (int i = 0; i < paramCount; i++) {
      memset(&rows[i], 0, sizeof(StoredParamRow));
      params[i].name.toCharArray(rows[i].name, STORED_NAME_LEN);
      params[i].type.toCharArray(rows[i].type, STORED_TYPE_LEN);
      rows[i].area = params[i].area;
      rows[i].transport = params[i].transport;
      rows[i].slaveId = params[i].slaveId;
      rows[i].tcpTargetIndex = params[i].tcpTargetIndex;
      rows[i].registerAddress = params[i].registerAddress;
      rows[i].enabled = params[i].enabled ? 1 : 0;
      rows[i].writable = params[i].writable ? 1 : 0;
      rows[i].minWriteValue = params[i].minWriteValue;
      rows[i].maxWriteValue = params[i].maxWriteValue;
    }

    size_t expectedLen = (size_t)paramCount * sizeof(StoredParamRow);
    size_t written = preferences.putBytes("rows", rows, expectedLen);
    if (written != expectedLen) {
      nvsWriteFailures++;
    }

    delete[] rows;
  }

  preferences.end();

  if (nvsWriteFailures > 0) {
    logKeyEvent("NVS SAVE INCOMPLETE (modbus): " + String(nvsWriteFailures) + " write(s) failed - flash may be full, param changes may not persist across reboot");
  }

  return nvsWriteFailures == 0;
}

// Reads the pre-blob one-key-per-field format, for configs saved by a
// firmware version before this compact format existed. Preferences must
// already be open (read-only) when called; does not call end().
void loadSettingsLegacyPerKey() {
  for (int i = 0; i < paramCount; i++) {
    String index = String(i);

    params[i].name = preferences.getString(("name" + index).c_str(), "Parameter");
    params[i].area = preferences.getUChar(("area" + index).c_str(), AREA_HOLDING_REGISTER);
    params[i].type = preferences.getString(("type" + index).c_str(), "INT_16/100");
    params[i].transport = preferences.getUChar(("transport" + index).c_str(), TRANSPORT_RTU);
    params[i].slaveId = preferences.getUChar(("sid" + index).c_str(), 1);
    params[i].tcpTargetIndex = preferences.getUChar(("tcptgt" + index).c_str(), 0);
    params[i].registerAddress = preferences.getUShort(("addr" + index).c_str(), 0);
    params[i].enabled = preferences.getBool(("en" + index).c_str(), true);

    // The write path didn't exist when this per-key format was in use -
    // no keys to read, so every row starts non-writable/no limits set.
    params[i].writable = false;
    params[i].minWriteValue = 0;
    params[i].maxWriteValue = 0;

    finalizeLoadedParamRow(i);
  }
}

void loadSettings() {
  preferences.begin("modbus", true);

  paramCount = preferences.getInt("count", -1);

  if (paramCount < 0 || paramCount > MAX_PARAMS) {
    preferences.end();
    loadDefaultSettings();
    saveSettings();
    return;
  }

  if (paramCount == 0) {
    preferences.end();
    loadDefaultSettings();
    return;
  }

  size_t expectedLenV2 = (size_t)paramCount * sizeof(StoredParamRow);
  size_t expectedLenV1 = (size_t)paramCount * sizeof(StoredParamRowV1);
  size_t blobLen = preferences.getBytesLength("rows");

  if (blobLen == expectedLenV2) {
    StoredParamRow *rows = new StoredParamRow[paramCount];
    preferences.getBytes("rows", rows, expectedLenV2);
    preferences.end();

    for (int i = 0; i < paramCount; i++) {
      // Defensive null-termination in case a name/type was ever truncated
      // to exactly fill its buffer with no room for the terminator.
      rows[i].name[STORED_NAME_LEN - 1] = '\0';
      rows[i].type[STORED_TYPE_LEN - 1] = '\0';

      params[i].name = String(rows[i].name);
      params[i].type = String(rows[i].type);
      params[i].area = rows[i].area;
      params[i].transport = rows[i].transport;
      params[i].slaveId = rows[i].slaveId;
      params[i].tcpTargetIndex = rows[i].tcpTargetIndex;
      params[i].registerAddress = rows[i].registerAddress;
      params[i].enabled = rows[i].enabled != 0;
      params[i].writable = rows[i].writable != 0;
      params[i].minWriteValue = rows[i].minWriteValue;
      params[i].maxWriteValue = rows[i].maxWriteValue;

      finalizeLoadedParamRow(i);
    }

    delete[] rows;
    return;
  }

  if (blobLen == expectedLenV1) {
    // A device that already migrated to the v1 compact blob (before the
    // write path existed) - read it once via the old layout, then
    // re-save in v2, which adds the write fields at their safe defaults.
    StoredParamRowV1 *rows = new StoredParamRowV1[paramCount];
    preferences.getBytes("rows", rows, expectedLenV1);
    preferences.end();

    for (int i = 0; i < paramCount; i++) {
      rows[i].name[STORED_NAME_LEN - 1] = '\0';
      rows[i].type[STORED_TYPE_LEN - 1] = '\0';

      params[i].name = String(rows[i].name);
      params[i].type = String(rows[i].type);
      params[i].area = rows[i].area;
      params[i].transport = rows[i].transport;
      params[i].slaveId = rows[i].slaveId;
      params[i].tcpTargetIndex = rows[i].tcpTargetIndex;
      params[i].registerAddress = rows[i].registerAddress;
      params[i].enabled = rows[i].enabled != 0;
      params[i].writable = false;
      params[i].minWriteValue = 0;
      params[i].maxWriteValue = 0;

      finalizeLoadedParamRow(i);
    }

    delete[] rows;

    saveSettings();
    logKeyEvent("MODBUS SETTINGS MIGRATED to v2 storage (write path fields added)");
    return;
  }

  // No compact blob at all - a config saved by the original per-key
  // format (before either blob version existed). Read it via that old
  // layout, then re-save in the current compact format, which also
  // reclaims the many old per-row keys via preferences.clear().
  loadSettingsLegacyPerKey();
  preferences.end();

  saveSettings();
  logKeyEvent("MODBUS SETTINGS MIGRATED to compact storage - freed flash space");
}

// ===================== Dropdown =====================
String getTypeOptions(String selected) {
  String html = "";

  for (int i = 0; i < typeCount; i++) {
    html += "<option value='" + htmlEscape(typeList[i].name) + "'";

    if (typeList[i].name == selected) {
      html += " selected";
    }

    html += ">" + htmlEscape(typeList[i].name) + "</option>";
  }

  return html;
}

// Display names indexed by AREA_* value - keep in sync with the defines.
const char *AREA_NAMES[4] = { "Holding Register", "Input Register", "Coil", "Discrete Input" };

String getAreaOptions(uint8_t selected) {
  String html = "";

  for (uint8_t a = 0; a < 4; a++) {
    html += "<option value='" + String(a) + "'";

    if (a == selected) {
      html += " selected";
    }

    html += ">" + String(AREA_NAMES[a]) + "</option>";
  }

  return html;
}

// Options for the "TCP Target" dropdown, from the configured tcpTargets[]
// list. Empty when no targets are configured yet - the row's transport
// select should default back to RTU in that case (enforced client-side).
String getTcpTargetOptions(uint8_t selected) {
  String html = "";

  for (int t = 0; t < tcpTargetCount; t++) {
    html += "<option value='" + String(t) + "'";

    if (t == selected) {
      html += " selected";
    }

    html += ">" + htmlEscape(tcpTargets[t].name) + "</option>";
  }

  return html;
}

// ===================== Float Conversion =====================
float halfToFloat(uint16_t half) {
  uint16_t sign = (half >> 15) & 0x0001;
  uint16_t exponent = (half >> 10) & 0x001F;
  uint16_t fraction = half & 0x03FF;

  float value;

  if (exponent == 0) {
    value = fraction * 0.000000059604644775390625f;
  } else if (exponent == 31) {
    value = fraction ? NAN : INFINITY;
  } else {
    value = (1.0f + fraction / 1024.0f) * pow(2, exponent - 15);
  }

  return sign ? -value : value;
}

// ===================== Value Conversion =====================
float convertValue(String selectedType, uint16_t reg1, uint16_t reg2) {
  int index = findTypeIndex(selectedType);

  String base = getBaseFormatFromType(selectedType);
  float divisor = 1;

  if (index >= 0) {
    divisor = typeList[index].divisor;
  }

  if (divisor <= 0) {
    divisor = 1;
  }

  float value = 0;

  if (base == "INT16") {
    value = (int16_t)reg1;
    return value / divisor;
  }

  if (base == "UINT16") {
    value = reg1;
    return value / divisor;
  }

  if (base == "FLOAT16") {
    return halfToFloat(reg1);
  }

  if (base == "FLOAT32") {
    uint32_t raw = ((uint32_t)reg1 << 16) | reg2;
    memcpy(&value, &raw, sizeof(value));
    return value;
  }

  return reg1;
}

// Reverse of halfToFloat(): packs a float into IEEE754 binary16.
uint16_t floatToHalf(float f) {
  uint32_t bits;
  memcpy(&bits, &f, sizeof(bits));

  uint32_t sign = (bits >> 31) & 0x1;
  int32_t exp = (int32_t)((bits >> 23) & 0xFF) - 127 + 15;
  uint32_t mantissa = bits & 0x7FFFFF;

  if (exp <= 0) {
    return (uint16_t)(sign << 15);  // underflow to (signed) zero
  }

  if (exp >= 31) {
    return (uint16_t)((sign << 15) | (0x1Ful << 10));  // overflow to +/-Inf
  }

  return (uint16_t)((sign << 15) | ((uint32_t)exp << 10) | (mantissa >> 13));
}

// Reverse of convertValue(): converts an engineering-unit value into the
// raw register(s) to write, using the same type/divisor rules. outRegs
// must have room for 2 entries (the second is only used for FLOAT32).
void floatToRegs(String selectedType, float value, uint16_t *outRegs) {
  int index = findTypeIndex(selectedType);

  String base = getBaseFormatFromType(selectedType);
  float divisor = 1;

  if (index >= 0) {
    divisor = typeList[index].divisor;
  }

  if (divisor <= 0) {
    divisor = 1;
  }

  if (base == "INT16") {
    float scaled = value * divisor;
    if (scaled < -32768) scaled = -32768;
    if (scaled > 32767) scaled = 32767;
    outRegs[0] = (uint16_t)(int16_t)lroundf(scaled);
    return;
  }

  if (base == "UINT16") {
    float scaled = value * divisor;
    if (scaled < 0) scaled = 0;
    if (scaled > 65535) scaled = 65535;
    outRegs[0] = (uint16_t)lroundf(scaled);
    return;
  }

  if (base == "FLOAT16") {
    outRegs[0] = floatToHalf(value);
    return;
  }

  if (base == "FLOAT32") {
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    outRegs[0] = (uint16_t)(raw >> 16);
    outRegs[1] = (uint16_t)(raw & 0xFFFF);
    return;
  }

  outRegs[0] = (uint16_t)lroundf(value);
}

// ===================== TCP JSON =====================
String getDeviceMacID() {
  String mac = WiFi.macAddress();

  if (mac == "00:00:00:00:00:00") {
    mac = WiFi.softAPmacAddress();
  }

  mac.replace(":", "");
  return mac;
}

// Device name shown on the Settings page and used when apMatchesDeviceName
// is set. Defaults to the full MAC address; if the user sets a prefix (up
// to DEVICE_NAME_PREFIX_MAX_LEN chars), it becomes prefix_<last5MACchars>.
String getDeviceName() {
  String mac = getDeviceMacID();

  if (deviceConfig.deviceNamePrefix.length() == 0) {
    return mac;
  }

  String macSuffix = (mac.length() >= 5) ? mac.substring(mac.length() - 5) : mac;
  return deviceConfig.deviceNamePrefix + "_" + macSuffix;
}

// Actual SSID the AP is started/restarted with - the device name when
// apMatchesDeviceName is checked, otherwise the independently-set apSsid.
String getEffectiveApSsid() {
  return deviceConfig.apMatchesDeviceName ? getDeviceName() : deviceConfig.apSsid;
}

// True once SNTP has produced a plausible current date (see MIN_VALID_EPOCH).
bool isTimeSynced() {
  return time(nullptr) >= (time_t)MIN_VALID_EPOCH;
}

// UTC epoch seconds when NTP has synced; otherwise seconds-since-boot,
// exactly the pre-NTP behavior. The JSON's time_source field tells the
// receiver which one it is getting.
unsigned long getTimestamp() {
  if (isTimeSynced()) {
    return (unsigned long)time(nullptr);
  }
  return millis() / 1000;
}

// Shared by the periodic telemetry payload (buildTcpJson()) and the
// on-demand raw-TCP query response (sendTcpQueryResponse()), so both report
// a param's status the same way. Caller must already hold dataMutex.
String paramStatusText(int i) {
  if (!params[i].enabled) {
    return "Disabled";
  }
  if (params[i].valid) {
    return "OK";
  }
  return "Disconnected";
}

String buildTcpJson() {
  String json = "{";
  // +12 rows covers the worst case of every Digital/Analog I/O channel enabled.
  json.reserve(128 + (paramCount + DI_CHANNEL_COUNT + DO_CHANNEL_COUNT + AI_CHANNEL_COUNT) * 96);

  // Same identity the user sees on the Settings page (Device Name):
  // prefix_XXXXX when a prefix is set, full MAC otherwise.
  json += "\"device_id\":\"" + jsonEscape(getDeviceName()) + "\",";
  json += "\"timestamp\":" + String(getTimestamp()) + ",";
  json += "\"time_source\":\"" + String(isTimeSynced() ? "ntp" : "uptime") + "\",";
  json += "\"sensors\":[";

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  bool firstSensor = true;

  for (int i = 0; i < paramCount; i++) {
    if (!firstSensor) {
      json += ",";
    }
    firstSensor = false;

    String deviceId = String(params[i].slaveId);

    json += "{";
    json += "\"sensor_id\":\"" + jsonEscape(deviceId) + "\",";
    json += "\"sensor_name\":\"" + jsonEscape(params[i].name) + "\",";

    if (params[i].valid) {
      json += "\"value\":" + String(params[i].value, 3) + ",";
    } else {
      json += "\"value\":null,";
    }

    json += "\"status\":\"" + paramStatusText(i) + "\"";
    json += "}";
  }

  // Local Digital/Analog I/O channels ride the same sensors[] array
  // (sensor_id = pin label, since there's no slave ID for local GPIO) -
  // disabled channels are skipped entirely rather than emitting a
  // placeholder, so telemetry is unchanged for anyone not using this
  // feature at all.
  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    if (!diChannels[i].enabled) continue;
    if (!firstSensor) json += ",";
    firstSensor = false;

    json += "{\"sensor_id\":\"" + String(DI_PIN_LABELS[i]) + "\",";
    json += "\"sensor_name\":\"" + jsonEscape(diChannels[i].name) + "\",";
    json += "\"value\":" + String(diChannels[i].value ? 1 : 0) + ",";
    json += "\"status\":\"OK\"}";
  }

  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    if (!doChannels[i].enabled) continue;
    if (!firstSensor) json += ",";
    firstSensor = false;

    json += "{\"sensor_id\":\"" + String(DO_PIN_LABELS[i]) + "\",";
    json += "\"sensor_name\":\"" + jsonEscape(doChannels[i].name) + "\",";
    json += "\"value\":" + String(doChannels[i].value ? 1 : 0) + ",";
    json += "\"status\":\"OK\"}";
  }

  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    if (!aiChannels[i].enabled) continue;
    if (!firstSensor) json += ",";
    firstSensor = false;

    json += "{\"sensor_id\":\"" + String(AI_PIN_LABELS[i]) + "\",";
    json += "\"sensor_name\":\"" + jsonEscape(aiChannels[i].name) + "\",";
    json += "\"value\":" + String(aiChannels[i].value, 3) + ",";
    json += "\"status\":\"OK\"}";
  }

  xSemaphoreGive(dataMutex);

  json += "]";
  json += "}";

  return json;
}

// ===================== TCP Communication =====================
bool connectTcpServer() {
  if (uplinkConfig.serverIP.length() == 0 || uplinkConfig.port == 0) {
    logMessage("TCP TARGET NOT CONFIGURED");
    return false;
  }

  if (activeUplinkTransport == UPLINK_TRANSPORT_WIFI) {
    if (WiFi.status() != WL_CONNECTED) {
      logMessage("TCP SKIP: WIFI NOT CONNECTED");
      return false;
    }
  } else {
    bool cellularDataUp;
    xSemaphoreTake(cellularMutex, portMAX_DELAY);
    cellularDataUp = cellularStatus.dataConnected;
    xSemaphoreGive(cellularMutex);

    if (!cellularDataUp) {
      logMessage("TCP SKIP: CELLULAR DATA SESSION NOT UP");
      return false;
    }
  }

  if (!tcpClient->connected()) {
    logMessage("TCP TRY " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));

    // The 3-arg connect(host, port, timeoutMs) is a WiFiClient-specific
    // extension, not part of the generic Client interface TinyGsmClient
    // also implements - only usable through the concrete object. Over
    // cellular, the base 2-arg connect() is used instead (TinyGsmClient's
    // own internal timeout applies).
    bool connectOk = (activeUplinkTransport == UPLINK_TRANSPORT_WIFI)
      ? wifiTcpClient.connect(uplinkConfig.serverIP.c_str(), uplinkConfig.port, TCP_CONNECT_TIMEOUT_MS)
      : tcpClient->connect(uplinkConfig.serverIP.c_str(), uplinkConfig.port);

    if (!connectOk) {
      // Throttled: a dead master would otherwise fail once per poll cycle
      // and flood the key log.
      if (millis() - lastTcpFailLogTime >= DEBUG_ERROR_REPEAT_MS) {
        lastTcpFailLogTime = millis();
        logKeyEvent("TCP CONNECT FAILED: " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));
      }
      return false;
    }

    if (activeUplinkTransport == UPLINK_TRANSPORT_WIFI) {
      // Disable Nagle's algorithm so small JSON payloads go out immediately
      // instead of being buffered/delayed. Must be set after connect() -
      // some cores reset this flag to its default during connect(). Not
      // part of the generic Client interface (TinyGsmClient doesn't have
      // it - the modem's own TCP stack handles this internally), so this
      // has to go through the concrete WiFiClient object, not the Client*
      // pointer.
      wifiTcpClient.setNoDelay(true);
    }

    logMessage("TCP OK");
  }

  return true;
}

// Logs a key event only on TCP connect/disconnect transitions.
void checkTcpUplinkStatusChange() {
  bool connected = tcpClient->connected();

  if (connected != lastTcpConnectedState) {
    if (connected) {
      logKeyEvent("TCP CONNECTED: " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));
    } else {
      logKeyEvent("TCP DISCONNECTED");
    }
    lastTcpConnectedState = connected;
  }
}

// Sends one payload (newline-framed) over raw TCP. Returns false when the
// connection is down or the write comes up short, so the caller can queue
// the payload for store & forward instead of losing it.
bool sendPayloadTcp(const String &payload) {
  if (!connectTcpServer()) {
    return false;
  }

  String framed = payload;
  framed += "\n";

  // Single write() call so the payload goes out as one TCP send instead of two.
  size_t written = tcpClient->write((const uint8_t *)framed.c_str(), framed.length());

  if (written != framed.length()) {
    logMessage("TCP SEND INCOMPLETE (" + String(written) + "/" + String(framed.length()) + ")");
    tcpClient->stop();
    return false;
  }

  logMessage("TCP DATA SENT");
  logMessage(payload);
  return true;
}

// ArduinoJson quirk (documented in their own FAQ): is<float>() alone does
// NOT reliably detect a JSON number written without a decimal point (e.g.
// a plain `72` vs `72.0`) - it can come back false for integer literals,
// which silently rejected every whole-number command as "malformed".
// Checking is<long>() too catches both forms. Shared by both inbound
// command parsers below (raw-TCP, MQTT command topic).
bool isJsonNumber(JsonVariantConst v) {
  return v.is<float>() || v.is<long>();
}

// ===================== Raw TCP Command Channel =====================
// Bidirectional use of the same outbound tcpClient connection used by
// sendPayloadTcp() above - no separate listening socket. The same peer this
// gateway pushes telemetry to can send newline-framed JSON commands back on
// the connection ({"param":"ParamName","value":42}); each is acked, also
// newline-framed, on the same socket. Only relevant when cloudConfig.mode
// == UPLINK_MODE_TCP - see pollTcpCommands()/webTask().
#define TCP_CMD_MAX_LEN 512

String tcpCmdBuffer;

// Sends one newline-framed JSON ack line back to the connected TCP peer.
// `reason` is only for the immediate-rejection path (invalid/unknown param,
// submitWriteCommand() validation failure) where there's no Modbus result
// code yet (code is sent as 0xFF, the same "never written" sentinel used
// elsewhere); the delayed completion ack (queued via tcpAckQueue) passes
// reason="" and the real Modbus result code instead.
void sendTcpAckLine(const String &name, float value, bool ok, uint8_t code, const String &reason) {
  if (!tcpClient->connected()) {
    return;
  }

  String line = "{\"ack\":\"" + jsonEscape(name) + "\",\"value\":" + String(value, 3) + ",\"ok\":" + (ok ? "true" : "false") + ",\"code\":" + String(code);
  if (reason.length() > 0) {
    line += ",\"reason\":\"" + jsonEscape(reason) + "\"";
  }
  line += "}\n";

  tcpClient->write((const uint8_t *)line.c_str(), line.length());
}

// Searches DI/DO/AI channels by name (in that order) as a fallback for the
// raw-TCP query command, mirroring findDoChannelIndexByName()'s role in the
// write path. Returns true and fills the outputs if found. Caller must NOT
// already hold dataMutex.
bool findDigitalIOValueByName(const String &name, float &outValue, bool &outEnabled, unsigned long &outLastUpdateMs) {
  bool found = false;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < DI_CHANNEL_COUNT && !found; i++) {
    if (diChannels[i].name == name) {
      outValue = diChannels[i].value ? 1 : 0;
      outEnabled = diChannels[i].enabled;
      outLastUpdateMs = diChannels[i].lastUpdateTime;
      found = true;
    }
  }

  for (int i = 0; i < DO_CHANNEL_COUNT && !found; i++) {
    if (doChannels[i].name == name) {
      outValue = doChannels[i].value ? 1 : 0;
      outEnabled = doChannels[i].enabled;
      outLastUpdateMs = doChannels[i].lastWriteTime;
      found = true;
    }
  }

  for (int i = 0; i < AI_CHANNEL_COUNT && !found; i++) {
    if (aiChannels[i].name == name) {
      outValue = aiChannels[i].value;
      outEnabled = aiChannels[i].enabled;
      outLastUpdateMs = aiChannels[i].lastUpdateTime;
      found = true;
    }
  }

  xSemaphoreGive(dataMutex);

  return found;
}

// On-demand read for one parameter, e.g. {"query":"SetpointTemp"} ->
// {"query":"SetpointTemp","value":72.5,"status":"OK","ageMs":1500} - unlike
// the periodic telemetry broadcast (buildTcpJson(), all params, every poll
// cycle, unprompted), this replies about exactly the one parameter asked
// for, on request. "ageMs" is how long ago that value was last updated by a
// poll; omitted (along with value) if it's never been successfully polled.
// Falls back to Digital/Analog I/O channels by name if no Modbus param
// matches, so one query command covers both namespaces.
// Builds the response JSON (no trailing newline) - shared by the raw-TCP
// channel above and the MQTT command channel (handleMqttCommand()), so
// both reply identically. channelLabel only feeds the rejection log line.
String buildQueryResponseJson(const String &name, const char *channelLabel) {
  int idx = findParamIndexByName(name);

  if (idx >= 0) {
    bool valid;
    float value;
    unsigned long lastUpdateTime;
    String statusText;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    valid = params[idx].valid;
    value = params[idx].value;
    lastUpdateTime = params[idx].lastUpdateTime;
    statusText = paramStatusText(idx);
    xSemaphoreGive(dataMutex);

    String line = "{\"query\":\"" + jsonEscape(name) + "\",\"value\":";
    line += valid ? String(value, 3) : "null";
    line += ",\"status\":\"" + statusText + "\"";
    if (lastUpdateTime > 0) {
      line += ",\"ageMs\":" + String(millis() - lastUpdateTime);
    }
    line += "}";
    return line;
  }

  float dioValue;
  bool dioEnabled;
  unsigned long dioLastUpdateMs;

  if (findDigitalIOValueByName(name, dioValue, dioEnabled, dioLastUpdateMs)) {
    String line = "{\"query\":\"" + jsonEscape(name) + "\",\"value\":" + String(dioValue, 3);
    line += ",\"status\":\"" + String(dioEnabled ? "OK" : "Disabled") + "\"";
    if (dioLastUpdateMs > 0) {
      line += ",\"ageMs\":" + String(millis() - dioLastUpdateMs);
    }
    line += "}";
    return line;
  }

  logKeyEvent(String(channelLabel) + " QUERY REJECTED: no parameter named '" + name + "'");
  return "{\"query\":\"" + jsonEscape(name) + "\",\"status\":\"ERR\",\"reason\":\"unknown parameter\"}";
}

void sendTcpQueryResponse(const String &name) {
  if (!tcpClient->connected()) {
    return;
  }

  String line = buildQueryResponseJson(name, "TCP") + "\n";
  tcpClient->write((const uint8_t *)line.c_str(), line.length());
}

// Parses one inbound command line - either a read {"query":"Name"} (answered
// immediately by sendTcpQueryResponse()) or a write
// {"param":"Name","value":N} queued via the same validated
// submitWriteCommand() path as the dashboard/MQTT channels. Immediate
// write rejections (parse error, unknown param, submitWriteCommand()
// validation failure) are acked right away; a successfully queued write's
// real result is acked later, once modbusTask/tcpPollTask actually
// executes it - see queueTcpAck()/drainTcpAckQueue().
void handleTcpCommandLine(const String &line) {
  DynamicJsonDocument doc(line.length() + 128);
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    logKeyEvent("TCP CMD PARSE ERROR: " + String(err.c_str()));
    return;
  }

  if (doc["query"].is<const char *>()) {
    sendTcpQueryResponse(doc["query"].as<String>());
    return;
  }

  if (!doc["param"].is<const char *>() || !isJsonNumber(doc["value"])) {
    logKeyEvent("TCP CMD REJECTED: malformed command (missing param/value, or query)");
    return;
  }

  String paramName = doc["param"].as<String>();
  float value = doc["value"].as<float>();

  int idx = findParamIndexByName(paramName);
  if (idx >= 0) {
    if (!submitWriteCommand(idx, value, WRITE_SOURCE_TCP)) {
      // submitWriteCommand() already logged the specific reason.
      sendTcpAckLine(paramName, value, false, 0xFF, "rejected - see Status Log");
    }
    // else: queued OK - the real ack comes later via tcpAckQueue.
    return;
  }

  int doIdx = findDoChannelIndexByName(paramName);
  if (doIdx >= 0) {
    if (!submitDoWriteCommand(doIdx, value != 0, WRITE_SOURCE_TCP)) {
      sendTcpAckLine(paramName, value, false, 0xFF, "rejected - see Status Log");
    }
    return;
  }

  logKeyEvent("TCP CMD REJECTED: no parameter named '" + paramName + "'");
  sendTcpAckLine(paramName, value, false, 0xFF, "unknown parameter");
}

// Drains any bytes currently buffered on tcpClient into newline-framed
// command lines - never blocks (only reads what's already available), so
// it's safe to call every webTask iteration alongside the telemetry push.
void pollTcpCommands() {
  if (!tcpClient->connected()) {
    tcpCmdBuffer = "";  // discard any partial line from a dropped connection
    return;
  }

  while (tcpClient->available() > 0) {
    char c = (char)tcpClient->read();

    if (c == '\n') {
      tcpCmdBuffer.trim();
      if (tcpCmdBuffer.length() > 0) {
        handleTcpCommandLine(tcpCmdBuffer);
      }
      tcpCmdBuffer = "";
    } else if (c != '\r') {
      if (tcpCmdBuffer.length() < TCP_CMD_MAX_LEN) {
        tcpCmdBuffer += c;
      } else {
        // No newline in sight - drop it so a malformed peer can't grow
        // this buffer unbounded.
        logKeyEvent("TCP CMD IGNORED: line exceeded " + String(TCP_CMD_MAX_LEN) + " bytes");
        tcpCmdBuffer = "";
      }
    }
  }
}

// ===================== MQTT Command Channel (cloud-agnostic) =====================
// Plain MQTT topics with no cloud-specific semantics - works against any
// broker (AWS IoT Core, Azure IoT Hub, Mosquitto, EMQX, ...), unlike the
// AWS Device Shadow channel this replaced. One JSON command per message on
// <identity>/commands, same command shapes as the raw-TCP channel:
//   write: {"param":"Name","value":42}
//   read:  {"query":"Name"}
// Acks and query responses are published on <identity>/commands/ack, in
// the same JSON shapes the raw-TCP channel sends, so a master can share
// its parsing code between both transports. Identity is the MQTT Client
// ID (or Device Name when no client ID is configured) - the same identity
// the connection itself uses. Only active when
// cloudConfig.commandsEnabled is set - see connectMqtt()/mqttCallback().
#define MQTT_CMD_MAX_LEN 512

String mqttCommandIdentity() {
  if (cloudConfig.clientId.length() > 0) return cloudConfig.clientId;
  return getDeviceName();
}

String mqttCommandTopic() {
  return mqttCommandIdentity() + "/commands";
}

String mqttCommandAckTopic() {
  return mqttCommandIdentity() + "/commands/ack";
}

// PubSubClient's buffer holds one full message (topic+payload) in either
// direction and never auto-grows mid-message - it must already be large
// enough for the biggest inbound command/outbound ack before
// mqttClient.loop() can receive one. sendPayloadMqtt() separately grows
// the buffer to fit each outgoing telemetry publish; this is just the
// floor for command traffic, applied right after connect so it covers
// messages that arrive before the first telemetry publish.
uint16_t commandMqttBufferFloor() {
  if (!cloudConfig.commandsEnabled) return 0;
  uint16_t topicLen = (uint16_t)max(mqttCommandTopic().length(), mqttCommandAckTopic().length());
  return topicLen + MQTT_CMD_MAX_LEN + 64;
}

// Ack JSON - same shape as the raw-TCP channel's sendTcpAckLine() (minus
// the newline framing, which MQTT's own message boundaries make
// redundant). Shared by the WiFi publish path below and the cellular ack
// queue (queueCellularAck()/cellularTask).
String buildMqttAckJson(const String &name, float value, bool ok, uint8_t code, const String &reason) {
  String line = "{\"ack\":\"" + jsonEscape(name) + "\",\"value\":" + String(value, 3) + ",\"ok\":" + (ok ? "true" : "false") + ",\"code\":" + String(code);
  if (reason.length() > 0) {
    line += ",\"reason\":\"" + jsonEscape(reason) + "\"";
  }
  line += "}";
  return line;
}

// Publishes one JSON ack on the command ack topic over the WiFi/
// PubSubClient connection. webTask only.
void publishMqttAck(const String &name, float value, bool ok, uint8_t code, const String &reason) {
  if (!mqttClient.connected()) {
    return;
  }

  String line = buildMqttAckJson(name, value, ok, code, reason);

  if (!mqttClient.publish(mqttCommandAckTopic().c_str(), line.c_str())) {
    logKeyEvent("MQTT ACK PUBLISH FAILED: " + name);
  }
}

// Immediate acks/query responses for commands that arrived over the
// CELLULAR path can't be published inline: the inbound message is parsed
// from a URC that may surface in the middle of waiting for another AT
// command's response (see waitForAtResponse()), where issuing a nested
// AT+CMQTTTOPIC/PAYLOAD/PUB sequence would corrupt the command stream.
// They're queued here instead and published by cellularTask at loop level
// (drainCellularAckTxQueue()), where the UART is between commands.
// Best-effort like the other ack queues.
#define CELLULAR_ACK_JSON_MAX_LEN 256
#define CELLULAR_ACK_TX_QUEUE_LEN 4

struct CellularAckItem {
  char json[CELLULAR_ACK_JSON_MAX_LEN];
};

QueueHandle_t cellularAckTxQueue;

void queueCellularAck(const String &json) {
  if (json.length() >= CELLULAR_ACK_JSON_MAX_LEN) {
    logKeyEvent("CELLULAR ACK DROPPED (" + String(json.length()) + " bytes exceeds buffer)");
    return;
  }

  CellularAckItem item;
  json.toCharArray(item.json, CELLULAR_ACK_JSON_MAX_LEN);
  xQueueSend(cellularAckTxQueue, &item, 0);
}

// Parses one inbound command message - mirrors handleTcpCommandLine()
// exactly, just with MQTT publishes instead of socket writes for the
// replies. Immediate rejections (parse error, unknown param, validation
// failure) are acked right away; a successfully queued write's real
// result is acked later via queueMqttAck() (drained by webTask over WiFi,
// or by cellularTask over the modem during failover).
//
// viaCellular: true when the command arrived over the modem's onboard
// MQTT client (cellularTask's URC pump) - replies are then queued for
// cellularTask to publish (queueCellularAck()) instead of being sent
// through the WiFi PubSubClient, which is down during a failover anyway.
//
// On the WiFi path it's safe to publish from inside the PubSubClient
// callback: the inbound payload is fully copied into the JsonDocument
// before any publish call could reuse the client's shared rx/tx buffer.
void handleMqttCommand(const byte *payload, unsigned int length, bool viaCellular) {
  if (length == 0 || length > MQTT_CMD_MAX_LEN) {
    logKeyEvent("MQTT CMD IGNORED: payload size " + String(length) + " out of bounds");
    return;
  }

  DynamicJsonDocument doc(length + 128);
  DeserializationError err = deserializeJson(doc, payload, length);
  if (err) {
    logKeyEvent("MQTT CMD PARSE ERROR: " + String(err.c_str()));
    return;
  }

  if (doc["query"].is<const char *>()) {
    String response = buildQueryResponseJson(doc["query"].as<String>(), viaCellular ? "MQTT-CELL" : "MQTT");
    if (viaCellular) {
      queueCellularAck(response);
    } else {
      mqttClient.publish(mqttCommandAckTopic().c_str(), response.c_str());
    }
    return;
  }

  if (!doc["param"].is<const char *>() || !isJsonNumber(doc["value"])) {
    logKeyEvent("MQTT CMD REJECTED: malformed command (missing param/value, or query)");
    return;
  }

  String paramName = doc["param"].as<String>();
  float value = doc["value"].as<float>();

  int idx = findParamIndexByName(paramName);
  if (idx >= 0) {
    if (!submitWriteCommand(idx, value, WRITE_SOURCE_MQTT)) {
      // submitWriteCommand() already logged the specific reason.
      if (viaCellular) {
        queueCellularAck(buildMqttAckJson(paramName, value, false, 0xFF, "rejected - see Status Log"));
      } else {
        publishMqttAck(paramName, value, false, 0xFF, "rejected - see Status Log");
      }
    }
    // else: queued OK - the real ack comes later via mqttAckQueue.
    return;
  }

  int doIdx = findDoChannelIndexByName(paramName);
  if (doIdx >= 0) {
    if (!submitDoWriteCommand(doIdx, value != 0, WRITE_SOURCE_MQTT)) {
      if (viaCellular) {
        queueCellularAck(buildMqttAckJson(paramName, value, false, 0xFF, "rejected - see Status Log"));
      } else {
        publishMqttAck(paramName, value, false, 0xFF, "rejected - see Status Log");
      }
    }
    return;
  }

  logKeyEvent("MQTT CMD REJECTED: no parameter named '" + paramName + "'");
  if (viaCellular) {
    queueCellularAck(buildMqttAckJson(paramName, value, false, 0xFF, "unknown parameter"));
  } else {
    publishMqttAck(paramName, value, false, 0xFF, "unknown parameter");
  }
}

// PubSubClient callback - runs inside mqttClient.loop() on webTask (Core 0).
// Only the command topic is ever subscribed today, so no topic dispatch
// table is needed yet.
void mqttCallback(char *topic, byte *payload, unsigned int length) {
  handleMqttCommand(payload, length, false);
}

// ===================== MQTT / AWS IoT Uplink =====================
// Runs entirely on webTask (Core 0), like the raw-TCP path, so a slow TLS
// handshake can never disturb Modbus polling. A failed attempt backs off
// for MQTT_RETRY_INTERVAL_MS so the handshake cost isn't paid every cycle.
bool connectMqtt() {
  if (cloudConfig.endpoint.length() == 0) {
    logMessage("MQTT TARGET NOT CONFIGURED");
    return false;
  }

  if (WiFi.status() != WL_CONNECTED) {
    logMessage("MQTT SKIP: WIFI NOT CONNECTED");
    return false;
  }

  if (mqttClient.connected()) {
    return true;
  }

  if (certRootCA.length() == 0 || certDevice.length() == 0 || certPrivKey.length() == 0) {
    if (millis() - lastMqttFailLogTime >= DEBUG_ERROR_REPEAT_MS) {
      lastMqttFailLogTime = millis();
      logKeyEvent("MQTT CERTS NOT CONFIGURED - paste them on the Settings page");
    }
    return false;
  }

  // TLS certificate validation compares the cert's validity window against
  // the system clock - with an unsynced clock (1970) every handshake would
  // fail, so wait for NTP instead of burning a doomed attempt.
  if (!isTimeSynced()) {
    if (millis() - lastMqttFailLogTime >= DEBUG_ERROR_REPEAT_MS) {
      lastMqttFailLogTime = millis();
      logKeyEvent("MQTT WAITING FOR NTP TIME SYNC (TLS needs a valid clock)");
    }
    return false;
  }

  if (millis() - lastMqttAttempt < MQTT_RETRY_INTERVAL_MS) {
    return false;
  }
  lastMqttAttempt = millis();

  // Re-point TLS/MQTT at the current cert/endpoint Strings on every
  // attempt - both libraries store pointers, and the Strings may have
  // been replaced by a settings save since the last attempt.
  tlsClient.stop();
  tlsClient.setCACert(certRootCA.c_str());
  tlsClient.setCertificate(certDevice.c_str());
  tlsClient.setPrivateKey(certPrivKey.c_str());
  tlsClient.setHandshakeTimeout(MQTT_HANDSHAKE_TIMEOUT_S);

  mqttClient.setServer(cloudConfig.endpoint.c_str(), cloudConfig.port);
  mqttClient.setSocketTimeout(MQTT_SOCKET_TIMEOUT_S);
  mqttClient.setKeepAlive(30);

  String clientId = (cloudConfig.clientId.length() > 0) ? cloudConfig.clientId : getDeviceName();

  logMessage("MQTT TRY " + cloudConfig.endpoint + ":" + String(cloudConfig.port) + " as " + clientId);

  if (!mqttClient.connect(clientId.c_str())) {
    if (millis() - lastMqttFailLogTime >= DEBUG_ERROR_REPEAT_MS) {
      lastMqttFailLogTime = millis();
      logKeyEvent("MQTT CONNECT FAILED (state " + String(mqttClient.state()) + "): " + cloudConfig.endpoint);
    }
    return false;
  }

  logMessage("MQTT OK");

  if (cloudConfig.commandsEnabled) {
    // Must be sized before subscribing - a command could arrive on the very
    // next mqttClient.loop() call, and the buffer never auto-grows mid-message.
    mqttClient.setBufferSize(commandMqttBufferFloor());

    String cmdTopic = mqttCommandTopic();
    if (mqttClient.subscribe(cmdTopic.c_str())) {
      logKeyEvent("MQTT CMD SUBSCRIBED: " + cmdTopic);
    } else {
      logKeyEvent("MQTT CMD SUBSCRIBE FAILED: " + cmdTopic);
    }
  }

  return true;
}

// Logs a key event only on MQTT connect/disconnect transitions.
void checkMqttUplinkStatusChange() {
  bool connected = mqttClient.connected();

  if (connected != lastMqttConnectedState) {
    if (connected) {
      logKeyEvent("MQTT CONNECTED: " + cloudConfig.endpoint);
    } else {
      logKeyEvent("MQTT DISCONNECTED");
    }
    lastMqttConnectedState = connected;
  }
}

// Publishes one payload over MQTT. Returns false on connect/alloc/publish
// failure so the caller can queue the payload for store & forward.
bool sendPayloadMqtt(const String &payload) {
  if (!connectMqtt()) {
    return false;
  }

  // PubSubClient's default packet buffer (256 bytes) is far too small for
  // this payload - grow it to fit before every publish (no-op when already
  // large enough). Never shrink below the command floor - the client stays
  // subscribed to the command topic across publishes, and an inbound
  // command must still fit.
  uint16_t needed = payload.length() + cloudConfig.topic.length() + 16;
  needed = max(needed, commandMqttBufferFloor());
  if (!mqttClient.setBufferSize(needed)) {
    logMessage("MQTT BUFFER ALLOC FAILED (" + String(needed) + " bytes)");
    return false;
  }

  if (!mqttClient.publish(cloudConfig.topic.c_str(), payload.c_str())) {
    logMessage("MQTT PUBLISH FAILED");
    return false;
  }

  logMessage("MQTT DATA SENT: " + cloudConfig.topic);
  logMessage(payload);
  return true;
}

bool sendPayload(const String &payload) {
  if (cloudConfig.mode == UPLINK_MODE_MQTT) {
    return sendPayloadMqtt(payload);
  }
  return sendPayloadTcp(payload);
}

// True when the active uplink mode has a target configured at all - an
// unconfigured uplink shouldn't fill the store & forward buffer.
bool uplinkConfigured() {
  if (cloudConfig.mode == UPLINK_MODE_MQTT) {
    return cloudConfig.endpoint.length() > 0;
  }
  return uplinkConfig.serverIP.length() > 0 && uplinkConfig.port != 0;
}

// ===================== Store & Forward =====================
// RAM ring buffer of undelivered payloads. webTask-only, so no locking.
String sfBuffer[SF_MAX_ENTRIES];
int sfHead = 0;
int sfCount = 0;
size_t sfBytes = 0;
unsigned long lastSfLogTime = 0;

void sfPush(const String &payload) {
  if (payload.length() > SF_MAX_BYTES) {
    return;
  }

  // Drop oldest entries until the new payload fits both caps.
  while (sfCount > 0 && (sfBytes + payload.length() > SF_MAX_BYTES || sfCount >= SF_MAX_ENTRIES)) {
    sfBytes -= sfBuffer[sfHead].length();
    sfBuffer[sfHead] = "";
    sfHead = (sfHead + 1) % SF_MAX_ENTRIES;
    sfCount--;
  }

  int tail = (sfHead + sfCount) % SF_MAX_ENTRIES;
  sfBuffer[tail] = payload;
  sfBytes += payload.length();
  sfCount++;

  if (millis() - lastSfLogTime >= DEBUG_ERROR_REPEAT_MS) {
    lastSfLogTime = millis();
    logKeyEvent("UPLINK DOWN - buffering data (" + String(sfCount) + " payloads, " + String(sfBytes) + " bytes queued)");
  }
}

// One entry point for the per-cycle cloud push. Replays any backlog first
// (bounded per cycle) so the receiver always gets data in order; the fresh
// payload is queued behind a remaining backlog rather than jumping it.
void sendCloudData() {
  if (!uplinkConfigured()) {
    if (cloudConfig.mode == UPLINK_MODE_MQTT) {
      logMessage("MQTT TARGET NOT CONFIGURED");
    } else {
      logMessage("TCP TARGET NOT CONFIGURED");
    }
    return;
  }

  // Cellular has taken over the MQTT uplink (see updateMqttFailoverState())
  // and cellularTask independently publishes a fresh snapshot every cycle -
  // nothing for the WiFi path to do. Skipping rather than buffering into
  // store & forward is deliberate: if this still queued into sfBuffer,
  // WiFi reconnecting would replay the whole outage window a second time
  // on top of what cellular already delivered.
  if (cloudConfig.mode == UPLINK_MODE_MQTT && mqttFailoverActive) {
    return;
  }

  String payload = buildTcpJson();

  int drained = 0;
  while (sfCount > 0 && drained < SF_MAX_DRAIN_PER_CYCLE) {
    if (!sendPayload(sfBuffer[sfHead])) {
      break;
    }
    sfBytes -= sfBuffer[sfHead].length();
    sfBuffer[sfHead] = "";
    sfHead = (sfHead + 1) % SF_MAX_ENTRIES;
    sfCount--;
    drained++;
  }

  if (drained > 0) {
    logKeyEvent("STORE&FORWARD: replayed " + String(drained) + " payload(s)" + (sfCount > 0 ? (", " + String(sfCount) + " still queued") : " - backlog clear"));
  }

  if (sfCount > 0) {
    // Backlog remains (uplink still down, or drain cap hit) - keep order.
    sfPush(payload);
    return;
  }

  if (!sendPayload(payload)) {
    sfPush(payload);
  }
}

// ===================== Modbus Debug =====================
void logModbusStatusChange(int index) {
#if DEBUG_ENABLED && DEBUG_MODBUS_CHANGE_LOG
  bool statusChanged =
    (params[index].valid != params[index].lastLoggedValid) || (params[index].errorCode != params[index].lastLoggedErrorCode);

  bool repeatSameError =
    (!params[index].valid) && (millis() - params[index].lastErrorLogTime >= DEBUG_ERROR_REPEAT_MS);

  if (statusChanged || repeatSameError) {
    String msg = "[MODBUS] ";
    msg += params[index].name;
    msg += " | Slave=" + String(params[index].slaveId);
    msg += " | Addr=" + String(params[index].registerAddress);
    msg += " | Len=" + String(params[index].registerLength);
    msg += " | Type=" + params[index].type;

    if (params[index].valid) {
      msg += " | Status=OK";
      msg += " | Value=" + String(params[index].value, 3);
    } else {
      msg += " | Status=ERROR";
      msg += " | Code=" + String(params[index].errorCode);
      params[index].lastErrorLogTime = millis();
    }

    logMessage(msg);

    params[index].lastLoggedValid = params[index].valid;
    params[index].lastLoggedErrorCode = params[index].errorCode;
  }
#endif
}

// Case-sensitive lookup by the user-assigned param name, for command
// channels (MQTT, raw-TCP) that address a parameter by name rather than
// by table index. Returns -1 if no enabled or disabled row matches.
int findParamIndexByName(const String &name) {
  int found = -1;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int i = 0; i < paramCount; i++) {
    if (params[i].name == name) {
      found = i;
      break;
    }
  }
  xSemaphoreGive(dataMutex);

  return found;
}

// Same idea, for Digital Out channels - checked as a fallback wherever a
// command name doesn't match a Modbus param (see handleMqttCommand(),
// handleTcpCommandLine()), so the two namespaces share one command surface.
int findDoChannelIndexByName(const String &name) {
  int found = -1;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    if (doChannels[i].name == name) {
      found = i;
      break;
    }
  }
  xSemaphoreGive(dataMutex);

  return found;
}

// Human-readable tag for key-log lines - keeps the WRITE_SOURCE_* routing
// value as the single source of truth instead of passing a parallel string.
String writeSourceLabel(uint8_t source) {
  if (source == WRITE_SOURCE_MQTT) return "mqtt-cmd";
  if (source == WRITE_SOURCE_TCP) return "tcp-cmd";
  return "dashboard";
}

// ===================== Write Command Submission =====================
// Validates a write request against the target parameter's writable flag
// and optional min/max safety limits, then queues it for whichever task
// owns that parameter's transport. Returns true if queued.
bool submitWriteCommand(int paramIndex, float value, uint8_t source) {
  String sourceLabel = writeSourceLabel(source);

  if (paramIndex < 0 || paramIndex >= paramCount) {
    logKeyEvent("WRITE REJECTED (" + sourceLabel + "): invalid parameter index " + String(paramIndex));
    return false;
  }

  bool writable;
  uint8_t transport;
  float minV, maxV;
  String pname;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  writable = params[paramIndex].writable;
  transport = params[paramIndex].transport;
  minV = params[paramIndex].minWriteValue;
  maxV = params[paramIndex].maxWriteValue;
  pname = params[paramIndex].name;
  xSemaphoreGive(dataMutex);

  if (!writable) {
    logKeyEvent("WRITE REJECTED (" + sourceLabel + "): " + pname + " is not writable");
    return false;
  }

  // maxV > minV is the signal that a limit is actually configured (the
  // default 0/0 means "no limit set" - see the ModbusParam field comment).
  if (maxV > minV && (value < minV || value > maxV)) {
    logKeyEvent("WRITE REJECTED (" + sourceLabel + "): " + pname + " value " + String(value, 3) + " outside allowed range [" + String(minV, 3) + ", " + String(maxV, 3) + "]");
    return false;
  }

  WriteCommand cmd;
  cmd.paramIndex = paramIndex;
  cmd.value = value;
  cmd.source = source;

  QueueHandle_t q = (transport == TRANSPORT_TCP) ? tcpWriteQueue : rtuWriteQueue;

  if (xQueueSend(q, &cmd, 0) != pdTRUE) {
    logKeyEvent("WRITE QUEUE FULL (" + sourceLabel + ") - dropped command for " + pname);
    return false;
  }

  logKeyEvent("WRITE QUEUED (" + sourceLabel + "): " + pname + " = " + String(value, 3));
  return true;
}

// Non-blocking best-effort handoff to webTask - see the MqttAckItem
// comment near mqttAckQueue. Called by the drain functions below after a
// completed WRITE_SOURCE_MQTT write, success or failure - the MQTT peer
// gets an ack either way, same as the raw-TCP channel.
void queueMqttAck(const String &name, float value, uint8_t result) {
  MqttAckItem item;
  name.toCharArray(item.name, MQTT_ACK_NAME_LEN);
  item.value = value;
  item.result = result;
  xQueueSend(mqttAckQueue, &item, 0);
}

// Same non-blocking best-effort handoff, for a completed WRITE_SOURCE_TCP
// write - called by both drain functions below regardless of the result.
void queueTcpAck(const String &name, float value, uint8_t result) {
  TcpAckItem item;
  name.toCharArray(item.name, TCP_ACK_NAME_LEN);
  item.value = value;
  item.result = result;
  xQueueSend(tcpAckQueue, &item, 0);
}

// Converts an engineering-unit write value into the raw register(s)/bit
// for the wire, per the target parameter's area/type - shared by both
// drain functions below so the conversion rule lives in one place.
void buildWriteRegs(uint8_t area, String type, float value, uint16_t *outRegs) {
  if (isBitArea(area)) {
    outRegs[0] = (value != 0) ? 1 : 0;
  } else {
    floatToRegs(type, value, outRegs);
  }
}

// Drains any pending RTU write commands, dispatching each via
// modbusWrite() under serialMutex exactly like a read transaction. Called
// once per modbusTask cycle, interleaved with the poll loop.
void processRtuWriteQueue() {
  WriteCommand cmd;

  while (xQueueReceive(rtuWriteQueue, &cmd, 0) == pdTRUE) {
    if (cmd.paramIndex < 0 || cmd.paramIndex >= paramCount) {
      continue;
    }

    uint8_t area, slaveId;
    uint16_t regAddr, regLen;
    String type, pname;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    area = params[cmd.paramIndex].area;
    slaveId = params[cmd.paramIndex].slaveId;
    regAddr = params[cmd.paramIndex].registerAddress;
    type = params[cmd.paramIndex].type;
    pname = params[cmd.paramIndex].name;
    xSemaphoreGive(dataMutex);

    regLen = isBitArea(area) ? 1 : getDataLengthForType(type);
    if (regLen < 1) regLen = 1;
    if (regLen > 2) regLen = 2;

    uint16_t regs[2] = { 0, 0 };
    buildWriteRegs(area, type, cmd.value, regs);

    xSemaphoreTake(serialMutex, portMAX_DELAY);
    uint8_t result = modbusWrite(slaveId, area, regAddr, regLen, regs);
    xSemaphoreGive(serialMutex);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    params[cmd.paramIndex].lastWriteResult = result;
    params[cmd.paramIndex].lastWriteTime = millis();
    xSemaphoreGive(dataMutex);

    logKeyEvent("WRITE " + String(result == MB_SUCCESS ? "OK" : ("FAILED code " + String(result))) + ": " + pname + " = " + String(cmd.value, 3));

    if (cmd.source == WRITE_SOURCE_MQTT) {
      queueMqttAck(pname, cmd.value, result);
    } else if (cmd.source == WRITE_SOURCE_TCP) {
      queueTcpAck(pname, cmd.value, result);
    }

    delay(slaveRecoveryDelayMs);
  }
}

// Same as processRtuWriteQueue(), but for TCP targets via modbusTcpWrite() -
// called once per tcpPollTask cycle. No serialMutex: never touches Serial1.
void processTcpWriteQueue() {
  WriteCommand cmd;

  while (xQueueReceive(tcpWriteQueue, &cmd, 0) == pdTRUE) {
    if (cmd.paramIndex < 0 || cmd.paramIndex >= paramCount) {
      continue;
    }

    uint8_t area, targetIndex;
    uint16_t regAddr, regLen;
    String type, pname;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    area = params[cmd.paramIndex].area;
    targetIndex = params[cmd.paramIndex].tcpTargetIndex;
    regAddr = params[cmd.paramIndex].registerAddress;
    type = params[cmd.paramIndex].type;
    pname = params[cmd.paramIndex].name;
    xSemaphoreGive(dataMutex);

    regLen = isBitArea(area) ? 1 : getDataLengthForType(type);
    if (regLen < 1) regLen = 1;
    if (regLen > 2) regLen = 2;

    uint16_t regs[2] = { 0, 0 };
    buildWriteRegs(area, type, cmd.value, regs);

    uint8_t result = modbusTcpWrite(targetIndex, area, regAddr, regLen, regs);

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    params[cmd.paramIndex].lastWriteResult = result;
    params[cmd.paramIndex].lastWriteTime = millis();
    xSemaphoreGive(dataMutex);

    logKeyEvent("WRITE " + String(result == MB_SUCCESS ? "OK" : ("FAILED code " + String(result))) + ": " + pname + " = " + String(cmd.value, 3));

    if (cmd.source == WRITE_SOURCE_MQTT) {
      queueMqttAck(pname, cmd.value, result);
    } else if (cmd.source == WRITE_SOURCE_TCP) {
      queueTcpAck(pname, cmd.value, result);
    }
  }
}

// ===================== Digital Out Write Submission/Drain =====================
// Same validate-then-queue shape as submitWriteCommand(), simplified: no
// min/max clamp (a boolean has nothing to clamp), just an enabled check.
bool submitDoWriteCommand(int channelIndex, bool value, uint8_t source) {
  String sourceLabel = writeSourceLabel(source);

  if (channelIndex < 0 || channelIndex >= DO_CHANNEL_COUNT) {
    logKeyEvent("DO WRITE REJECTED (" + sourceLabel + "): invalid channel index " + String(channelIndex));
    return false;
  }

  bool enabled;
  String cname;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  enabled = doChannels[channelIndex].enabled;
  cname = doChannels[channelIndex].name;
  xSemaphoreGive(dataMutex);

  if (!enabled) {
    logKeyEvent("DO WRITE REJECTED (" + sourceLabel + "): " + cname + " is not enabled");
    return false;
  }

  DoWriteCommand cmd;
  cmd.channelIndex = channelIndex;
  cmd.value = value;
  cmd.source = source;

  if (xQueueSend(doWriteQueue, &cmd, 0) != pdTRUE) {
    logKeyEvent("DO WRITE QUEUE FULL (" + sourceLabel + ") - dropped command for " + cname);
    return false;
  }

  logKeyEvent("DO WRITE QUEUED (" + sourceLabel + "): " + cname + " = " + String(value ? "ON" : "OFF"));
  return true;
}

// Drains doWriteQueue - called every webTask cycle (see webTask()). A
// digitalWrite() can't fail the way a Modbus transaction can, so unlike
// processRtuWriteQueue()/processTcpWriteQueue() there's no result code:
// mqtt/tcp acks always report success once a queued command reaches here.
void processDoWriteQueue() {
  DoWriteCommand cmd;

  while (xQueueReceive(doWriteQueue, &cmd, 0) == pdTRUE) {
    if (cmd.channelIndex < 0 || cmd.channelIndex >= DO_CHANNEL_COUNT) {
      continue;
    }

    digitalWrite(DO_PINS[cmd.channelIndex], cmd.value ? HIGH : LOW);

    String cname;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    doChannels[cmd.channelIndex].value = cmd.value;
    doChannels[cmd.channelIndex].lastWriteTime = millis();
    cname = doChannels[cmd.channelIndex].name;
    xSemaphoreGive(dataMutex);

    logKeyEvent("DO WRITE OK: " + cname + " = " + String(cmd.value ? "ON" : "OFF"));

    if (cmd.source == WRITE_SOURCE_MQTT) {
      queueMqttAck(cname, cmd.value ? 1 : 0, MB_SUCCESS);
    } else if (cmd.source == WRITE_SOURCE_TCP) {
      queueTcpAck(cname, cmd.value ? 1 : 0, MB_SUCCESS);
    }
  }
}

// ===================== Poll Modbus =====================
// Runs entirely on modbusTask. The RS485 transaction always happens
// OUTSIDE dataMutex, using values snapshotted under a brief lock, so the
// bus is never held across slow web/data-mutex work. Each parameter gets
// its own up-to-3s attempt; a timeout or corrupt response is reported
// immediately (no debounce), then the next enabled parameter is polled.
void pollModbus() {
  int count;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  count = paramCount;
  xSemaphoreGive(dataMutex);

  for (int i = 0; i < count; i++) {
    bool enabled;
    String type;
    uint8_t area;
    uint8_t slaveId;
    uint16_t regAddr;
    uint16_t regLen;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    bool isRtu = (params[i].transport == TRANSPORT_RTU);
    enabled = params[i].enabled;
    type = params[i].type;
    area = params[i].area;
    slaveId = params[i].slaveId;
    regAddr = params[i].registerAddress;
    regLen = getDataLengthForType(type);
    xSemaphoreGive(dataMutex);

    // TCP-transport rows are owned by tcpPollTask - never touch RS485 for them.
    if (!isRtu) {
      continue;
    }

    if (!enabled) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      params[i].valid = false;
      params[i].errorCode = 0;
      xSemaphoreGive(dataMutex);
      continue;
    }

    if (isBitArea(area)) {
      // Coils/discrete inputs are single bits - the data type's register
      // length doesn't apply.
      regLen = 1;
    }

    if (regLen < 1) {
      regLen = 1;
    }

    if (regLen > 2) {
      regLen = 2;
    }

    // Slave still inside its offline backoff window: skip the transaction
    // entirely (no 3s timeout burned), mark the parameter accordingly.
    SlaveHealth &health = slaveHealth[slaveId];

    if (health.backoffUntil != 0 && (long)(millis() - health.backoffUntil) < 0) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      params[i].valid = false;
      params[i].errorCode = MB_ERR_BACKOFF;
      logModbusStatusChange(i);
      xSemaphoreGive(dataMutex);
      continue;
    }

    // ---- RS485 transaction: serialMutex only guards against a comm-settings
    // save changing baud/parity/stop mid-transaction. ----
    xSemaphoreTake(serialMutex, portMAX_DELAY);

    uint16_t regs[2] = { 0, 0 };
    uint8_t result = modbusRead(slaveId, area, regAddr, regLen, regs);

    // Snapshot the raw bytes before releasing serialMutex - the next
    // transaction would otherwise overwrite lastModbusTx/Rx before the
    // log line below gets a chance to read them.
    String rawTx = bytesToHex(lastModbusTx, lastModbusTxLen, false);
    String rawRx = bytesToHex(lastModbusRx, lastModbusRxLen, lastModbusRxOverflow);

    xSemaphoreGive(serialMutex);
    // ---- End RS485 transaction ----

    // Per-slave backoff accounting. Only a full timeout counts - a CRC
    // error or a Modbus exception means the slave IS responding.
    if (result == MB_ERR_TIMEOUT) {
      if (health.consecTimeouts < 255) {
        health.consecTimeouts++;
      }
      if (health.consecTimeouts >= SLAVE_BACKOFF_FAIL_THRESHOLD) {
        bool enteringBackoff = (health.backoffUntil == 0);
        health.backoffUntil = millis() + SLAVE_BACKOFF_MS;
        if (enteringBackoff) {
          logKeyEvent("SLAVE " + String(slaveId) + " NOT RESPONDING - pausing its polls for " + String(SLAVE_BACKOFF_MS / 1000) + "s");
        }
      }
    } else {
      if (health.backoffUntil != 0) {
        logKeyEvent("SLAVE " + String(slaveId) + " BACK ONLINE");
      }
      health.consecTimeouts = 0;
      health.backoffUntil = 0;
    }

#if DEBUG_ENABLED
    if (result == MB_SUCCESS) {
      logMessage("[POLL] Slave=" + String(slaveId) + " Addr=" + String(regAddr) + " Len=" + String(regLen) + " Result=" + String(result));
    } else {
      // Troubleshooting aid: exact bytes sent and exact bytes captured
      // back off the bus for this failed transaction, whatever they were
      // (empty = nothing came back at all; a partial/garbled frame shows
      // exactly what did).
      logMessage("[POLL] Slave=" + String(slaveId) + " Addr=" + String(regAddr) + " Len=" + String(regLen) + " Result=" + String(result) +
                 " | TX=" + rawTx + " | RX=" + rawRx);
    }
#endif

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    params[i].registerLength = regLen;

    if (result == MB_SUCCESS) {
      if (isBitArea(area)) {
        // Bits are already 0/1 - the data type/divisor doesn't apply.
        params[i].value = regs[0] ? 1 : 0;
      } else {
        params[i].value = convertValue(type, regs[0], regs[1]);
      }
      params[i].valid = true;
      params[i].errorCode = 0;
      params[i].lastUpdateTime = millis();
    } else {
      // Reported immediately - no debounce/grace window.
      params[i].valid = false;
      params[i].errorCode = result;
    }

    logModbusStatusChange(i);

    uint32_t recoveryDelay = slaveRecoveryDelayMs;

    xSemaphoreGive(dataMutex);

    delay(recoveryDelay);
  }

  // Cycle done - signal webTask to push results over TCP.
  xSemaphoreGive(pollCompleteSem);
}

// ===================== Poll Modbus TCP Targets (Core 0) =====================
// Mirrors pollModbus()'s per-parameter logic, but for TRANSPORT_TCP rows:
// reads via modbusTcpRead() over WiFi instead of the RS485 bus, and tracks
// per-target backoff (tcpTargetHealth[]) instead of per-slave. Deliberately
// independent of modbusTask's cycle - it just writes into the same
// params[]/dataMutex, and whichever cloud push fires next (still triggered
// by modbusTask's pollCompleteSem) picks up whatever the latest values are.
// No serialMutex here: this never touches Serial1/RS485.
void pollTcpTargets() {
  int count;

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  count = paramCount;
  xSemaphoreGive(dataMutex);

  for (int i = 0; i < count; i++) {
    bool enabled;
    String type;
    uint8_t area;
    uint8_t targetIndex;
    uint16_t regAddr;
    uint16_t regLen;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    bool isTcp = (params[i].transport == TRANSPORT_TCP);
    enabled = params[i].enabled;
    type = params[i].type;
    area = params[i].area;
    targetIndex = params[i].tcpTargetIndex;
    regAddr = params[i].registerAddress;
    regLen = getDataLengthForType(type);
    xSemaphoreGive(dataMutex);

    if (!isTcp) {
      continue;
    }

    if (!enabled) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      params[i].valid = false;
      params[i].errorCode = 0;
      xSemaphoreGive(dataMutex);
      continue;
    }

    if (isBitArea(area)) {
      regLen = 1;
    }

    if (regLen < 1) {
      regLen = 1;
    }

    if (regLen > 2) {
      regLen = 2;
    }

    if (targetIndex >= tcpTargetCount) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      params[i].valid = false;
      params[i].errorCode = MB_ERR_TCP_UNCONFIGURED;
      logModbusStatusChange(i);
      xSemaphoreGive(dataMutex);
      continue;
    }

    // Target still inside its offline backoff window: skip the transaction
    // entirely, mark the parameter accordingly - same idea as RTU's backoff.
    SlaveHealth &health = tcpTargetHealth[targetIndex];

    if (health.backoffUntil != 0 && (long)(millis() - health.backoffUntil) < 0) {
      xSemaphoreTake(dataMutex, portMAX_DELAY);
      params[i].valid = false;
      params[i].errorCode = MB_ERR_BACKOFF;
      logModbusStatusChange(i);
      xSemaphoreGive(dataMutex);
      continue;
    }

    uint16_t regs[2] = { 0, 0 };
    uint8_t result = modbusTcpRead(targetIndex, area, regAddr, regLen, regs);

    // Per-target backoff accounting - only a full timeout counts.
    if (result == MB_ERR_TIMEOUT) {
      if (health.consecTimeouts < 255) {
        health.consecTimeouts++;
      }
      if (health.consecTimeouts >= TCP_TARGET_BACKOFF_FAIL_THRESHOLD) {
        bool enteringBackoff = (health.backoffUntil == 0);
        health.backoffUntil = millis() + TCP_TARGET_BACKOFF_MS;
        if (enteringBackoff) {
          logKeyEvent("TCP TARGET " + tcpTargets[targetIndex].name + " NOT RESPONDING - pausing its polls for " + String(TCP_TARGET_BACKOFF_MS / 1000) + "s");
        }
      }
    } else {
      if (health.backoffUntil != 0) {
        logKeyEvent("TCP TARGET " + tcpTargets[targetIndex].name + " BACK ONLINE");
      }
      health.consecTimeouts = 0;
      health.backoffUntil = 0;
    }

#if DEBUG_ENABLED
    logMessage("[TCP-POLL] Target=" + tcpTargets[targetIndex].name + " Addr=" + String(regAddr) + " Len=" + String(regLen) + " Result=" + String(result));
#endif

    xSemaphoreTake(dataMutex, portMAX_DELAY);

    params[i].registerLength = regLen;

    if (result == MB_SUCCESS) {
      if (isBitArea(area)) {
        params[i].value = regs[0] ? 1 : 0;
      } else {
        params[i].value = convertValue(type, regs[0], regs[1]);
      }
      params[i].valid = true;
      params[i].errorCode = 0;
      params[i].lastUpdateTime = millis();
    } else {
      params[i].valid = false;
      params[i].errorCode = result;
    }

    logModbusStatusChange(i);

    xSemaphoreGive(dataMutex);
  }
}

// ===================== Root Web Page =====================

// ===================== Landing Page: Dashboard =====================
void handleRoot() {

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Modbus Gateway</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; background:#f5f7fa; margin:15px; }
    h2 { color:#00a5df; }
    .box { background:white; padding:12px; margin-bottom:15px; box-shadow:0 1px 4px rgba(0,0,0,0.2); }
    table { width:100%; border-collapse:collapse; }
    th { background:#087b7b; color:white; padding:8px; }
    td { border:1px solid #2999c5; padding:6px; text-align:center; }
    button { padding:8px 14px; background:#0099cc; color:white; border:none; margin:2px; border-radius:4px; cursor:pointer; }
    .nav { background:#6f42c1; }
    .stale { color:#dc3545; font-weight:bold; }
    .logbox { background:#111; color:#0f0; font-family:monospace; font-size:12px; padding:10px; height:220px; overflow-y:auto; border-radius:4px; }
    .logbox div { white-space:pre-wrap; word-break:break-word; }
  </style>
</head>
<body>

<h2>ESP32 Modbus Gateway</h2>

<button class="nav" type="button" onclick="location.href='/settings'">Settings</button>

<div class='box'>
<b>WiFi Status:</b> )rawliteral";

  if (WiFi.status() == WL_CONNECTED) {
    html += "Connected<br>";
    html += "<b>IP:</b> " + WiFi.localIP().toString() + "<br>";
  } else {
    html += "Not Connected<br>";
  }

  if (cloudConfig.mode == UPLINK_MODE_MQTT) {
    html += "<b>MQTT:</b> ";
    html += mqttClient.connected() ? "Connected" : "Disconnected";
    html += " (Endpoint " + htmlEscape(cloudConfig.endpoint) + ":" + String(cloudConfig.port) + ")<br>";
  } else {
    html += "<b>TCP:</b> ";
    html += tcpClient->connected() ? "Connected" : "Disconnected";
    html += " (Target " + htmlEscape(uplinkConfig.serverIP) + ":" + String(uplinkConfig.port) + ")<br>";
  }
  html += "<b>Poll Interval:</b> " + String(pollIntervalMs) + " ms";
  html += "<br><b>Free Heap:</b> " + String(ESP.getFreeHeap()) + " bytes (min ever: " + String(ESP.getMinFreeHeap()) + ")";

  // Only shown while there is an undelivered backlog (uplink outage).
  if (sfCount > 0) {
    html += "<br><b>Buffered (offline):</b> " + String(sfCount) + " payloads / " + String(sfBytes) + " bytes";
  }
  html += "</div>";

  // Status monitoring only (see cellularTask()) - not yet part of the
  // uplink data path, so this box is informational, independent of the
  // WiFi/MQTT/TCP status above.
  {
    bool modemResponding, simReady, networkRegistered, dataConnected, mqttCertsProvisioned, mqttSessionConnected, mqttCommandsSubscribed;
    int signalQuality;
    String operatorName;
    unsigned long lastUpdateTime;

    xSemaphoreTake(cellularMutex, portMAX_DELAY);
    modemResponding = cellularStatus.modemResponding;
    simReady = cellularStatus.simReady;
    networkRegistered = cellularStatus.networkRegistered;
    signalQuality = cellularStatus.signalQuality;
    operatorName = cellularStatus.operatorName;
    dataConnected = cellularStatus.dataConnected;
    mqttCertsProvisioned = cellularStatus.mqttCertsProvisioned;
    mqttSessionConnected = cellularStatus.mqttSessionConnected;
    mqttCommandsSubscribed = cellularStatus.mqttCommandsSubscribed;
    lastUpdateTime = cellularStatus.lastUpdateTime;
    xSemaphoreGive(cellularMutex);

    html += "<div class='box'>";
    html += "<b>Cellular Modem (SIM7600G-H):</b> ";

    if (lastUpdateTime == 0) {
      html += "Initializing...";
    } else if (!modemResponding) {
      html += "Not Responding (check wiring/power)";
    } else {
      html += "<br><b>SIM:</b> " + String(simReady ? "Ready" : "Not Ready");
      html += "<br><b>Network:</b> " + String(networkRegistered ? ("Registered (" + operatorName + ")") : "Not Registered");
      if (signalQuality >= 0 && signalQuality <= 31) {
        html += "<br><b>Signal:</b> " + String(signalQuality) + "/31";
      } else {
        html += "<br><b>Signal:</b> Unknown";
      }
      html += "<br><b>Data Session:</b> " + String(dataConnected ? "Connected" : "Not Connected");
      if (cloudConfig.mode == UPLINK_MODE_MQTT) {
        html += "<br><b>MQTT Certs on Modem:</b> " + String(mqttCertsProvisioned ? "Provisioned" : "Not yet provisioned");
        html += "<br><b>MQTT Session (cellular):</b> " + String(mqttSessionConnected ? "Connected, publishing" : "Not connected");
        if (cloudConfig.commandsEnabled) {
          html += "<br><b>Command Topic (cellular):</b> " + String(mqttCommandsSubscribed ? "Subscribed" : "Not subscribed");
        }

        html += "<br><b>Active MQTT Uplink:</b> ";
        if (mqttFailoverActive) {
          html += "Cellular (failover)";
          // Previously this branch gave no visibility into whether WiFi had
          // already reconnected and was counting toward failback, or was
          // still down, or was flapping (each reconnect blip resets
          // wifiUpSince via checkWifiUplinkStatusChange(), restarting the
          // stability timer from zero) - all three look identical from the
          // Dashboard without this.
          if (WiFi.status() == WL_CONNECTED && wifiUpSince != 0) {
            unsigned long upSec = (millis() - wifiUpSince) / 1000;
            unsigned long remainSec = (UPLINK_FAILOVER_THRESHOLD_MS / 1000 > upSec) ? (UPLINK_FAILOVER_THRESHOLD_MS / 1000 - upSec) : 0;
            html += " - WiFi back up " + String(upSec) + "s (stable), failing back in " + String(remainSec) + "s if it stays up";
          } else {
            html += " - WiFi still down";
          }
        } else if (wifiDownSince != 0) {
          unsigned long downSec = (millis() - wifiDownSince) / 1000;
          unsigned long remainSec = (UPLINK_FAILOVER_THRESHOLD_MS / 1000 > downSec) ? (UPLINK_FAILOVER_THRESHOLD_MS / 1000 - downSec) : 0;
          html += "WiFi (down " + String(downSec) + "s - failing over to cellular in " + String(remainSec) + "s if not restored)";
        } else {
          html += "WiFi (primary)";
        }
      }
    }

    html += "</div>";
  }

  html += R"rawliteral(
<div class="box">
<h2>Dashboard</h2>
<table>
<thead>
<tr>
<th>Name</th>
<th>Source</th>
<th>Enabled</th>
<th>Value</th>
<th>Status</th>
<th>Last Update</th>
<th>Write</th>
</tr>
</thead>
<tbody id="dataBody"></tbody>
</table>
</div>

<div class="box">
<h2>Digital I/O</h2>
<table>
<thead>
<tr>
<th>Type</th>
<th>Name</th>
<th>Pin</th>
<th>Enabled</th>
<th>Value</th>
<th>Last Update</th>
<th>Control</th>
</tr>
</thead>
<tbody id="digioBody"></tbody>
</table>
</div>

<div class="box">
<h2>Status Log</h2>
<div id="keyLogBox" class="logbox"></div>
</div>

<script>
let lastData = [];
let lastDigitalIn = [];
let lastDigitalOut = [];
let lastAnalogIn = [];
let lastServerNowMs = 0;
let lastFetchClientTime = 0;

function formatAge(lastUpdateMs) {
  if (!lastUpdateMs) return "Never";
  let nowMs = lastServerNowMs + (Date.now() - lastFetchClientTime);
  let ageSec = Math.floor((nowMs - lastUpdateMs) / 1000);
  if (ageSec < 0) ageSec = 0;
  return ageSec + "s ago";
}

function writeResultText(p){
  if (p.lastWriteResult === 255) return "";
  if (p.lastWriteResult === 0) return `<span style="color:green">(OK)</span>`;
  return `<span style="color:red">(ERR 0x${p.lastWriteResult.toString(16)})</span>`;
}

// Row identity (which params exist + which are writable) rarely changes -
// only rebuild the row DOM when it does. The 1s tick just needs to update
// the age counter, so it must NOT touch row markup: a full innerHTML
// rebuild every tick would recreate the write <input> elements out from
// under anyone mid-keystroke, wiping whatever they'd typed.
let renderedRowSignature = "";

function rowSignature(data){
  return data.map(p => p.name + ":" + (p.writable ? 1 : 0)).join("|");
}

function buildRows(data){
  let body = "";
  data.forEach((p,i)=>{
   let writeCell = p.writable
     ? `<input type="text" id="wv${i}" style="width:70px" placeholder="value">
        <button type="button" onclick="sendWriteCommand(${i})">Write</button>
        <span id="wres${i}"></span>`
     : "-";
   body += `<tr>
   <td>${p.name}</td>
   <td id="src${i}"></td>
   <td id="en${i}"></td>
   <td id="val${i}"></td>
   <td id="stat${i}"></td>
   <td id="age${i}"></td>
   <td>${writeCell}</td>
   </tr>`;
  });
  document.getElementById("dataBody").innerHTML = body;
}

// Updates only the cells whose content can change between fetches/ticks.
// Never touches the write <input> - that's why the Writable status is
// baked into rowSignature() instead of being refreshed here.
function updateRowFields(data){
  data.forEach((p,i)=>{
   let ageSec = p.lastUpdateMs ? Math.floor((lastServerNowMs + (Date.now() - lastFetchClientTime) - p.lastUpdateMs) / 1000) : null;
   let staleClass = (ageSec === null || ageSec > 10) ? "stale" : "";

   let srcEl = document.getElementById("src" + i);
   if (srcEl) srcEl.textContent = p.source ?? '-';

   let enEl = document.getElementById("en" + i);
   if (enEl) enEl.textContent = p.enabled ? 'Yes' : 'No';

   let valEl = document.getElementById("val" + i);
   if (valEl) valEl.textContent = p.value ?? '-';

   let statEl = document.getElementById("stat" + i);
   if (statEl) statEl.textContent = p.valid ? 'OK' : 'ERR';

   let ageEl = document.getElementById("age" + i);
   if (ageEl) {
     ageEl.className = staleClass;
     ageEl.textContent = formatAge(p.lastUpdateMs);
   }

   let wresEl = document.getElementById("wres" + i);
   if (wresEl) wresEl.innerHTML = writeResultText(p);
  });
}

function renderTable(){
  let sig = rowSignature(lastData);
  if (sig !== renderedRowSignature) {
    buildRows(lastData);
    renderedRowSignature = sig;
  }
  updateRowFields(lastData);
}

function sendWriteCommand(index){
  let input = document.getElementById("wv" + index);
  let value = parseFloat(input.value);
  if (isNaN(value)) {
    alert("Enter a numeric value to write.");
    return;
  }
  let body = "param=" + encodeURIComponent(index) + "&value=" + encodeURIComponent(value);
  fetch('/writeCommand', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: body })
   .then(r => {
     if (!r.ok) {
       alert("Write rejected - check Status Log for the reason.");
     } else {
       input.value = "";
     }
   })
   .then(loadData);
}

function sendDoCommand(index, value){
  let body = "channel=" + encodeURIComponent(index) + "&value=" + (value ? "1" : "0");
  fetch('/writeDigitalOut', { method: 'POST', headers: {'Content-Type':'application/x-www-form-urlencoded'}, body: body })
   .then(r => { if (!r.ok) alert("Write rejected - check Status Log for the reason."); })
   .then(loadData);
}

function renderDigitalIO(){
  let body = "";

  lastDigitalIn.forEach(c => {
    body += `<tr><td>DI</td><td>${c.name}</td><td>${c.pin}</td><td>${c.enabled ? 'Yes':'No'}</td>`
          + `<td>${c.enabled ? (c.value ? 'Active' : 'Idle') : '-'}</td><td>${c.enabled ? formatAge(c.lastUpdateMs) : '-'}</td><td>-</td></tr>`;
  });

  lastDigitalOut.forEach((c,i) => {
    let control = c.enabled
      ? `<button type="button" onclick="sendDoCommand(${i},true)">ON</button><button type="button" onclick="sendDoCommand(${i},false)">OFF</button>`
      : "-";
    body += `<tr><td>DO</td><td>${c.name}</td><td>${c.pin}</td><td>${c.enabled ? 'Yes':'No'}</td>`
          + `<td>${c.enabled ? (c.value ? 'On' : 'Off') : '-'}</td><td>${c.enabled ? formatAge(c.lastWriteMs) : '-'}</td><td>${control}</td></tr>`;
  });

  lastAnalogIn.forEach(c => {
    body += `<tr><td>AI</td><td>${c.name}</td><td>${c.pin}</td><td>${c.enabled ? 'Yes':'No'}</td>`
          + `<td>${c.enabled ? (c.value.toFixed(3) + ' (raw ' + c.raw + ')') : '-'}</td><td>${c.enabled ? formatAge(c.lastUpdateMs) : '-'}</td><td>-</td></tr>`;
  });

  document.getElementById("digioBody").innerHTML = body;
}

function loadData(){
 fetch('/data')
 .then(r=>r.json())
 .then(d=>{
  lastData = d.data;
  lastDigitalIn = d.digitalIn || [];
  lastDigitalOut = d.digitalOut || [];
  lastAnalogIn = d.analogIn || [];
  lastServerNowMs = d.nowMs;
  lastFetchClientTime = Date.now();
  renderTable();
  renderDigitalIO();
 });
}

setInterval(loadData, 2000);
setInterval(renderTable, 1000);
setInterval(renderDigitalIO, 1000);
loadData();

function escapeHtml(s){
 let d = document.createElement("div");
 d.textContent = s;
 return d.innerHTML;
}

function loadKeyLog(){
 fetch('/keylog')
 .then(r=>r.json())
 .then(entries=>{
  let box = document.getElementById("keyLogBox");
  let wasAtBottom = box.scrollTop + box.clientHeight >= box.scrollHeight - 5;
  box.innerHTML = entries.map(e => `<div>${escapeHtml(e)}</div>`).join("");
  if (wasAtBottom) box.scrollTop = box.scrollHeight;
 });
}
setInterval(loadKeyLog, 3000);
loadKeyLog();
</script>

</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// Sends and clears the accumulated HTML buffer as one chunk of a
// chunked-transfer response (caller must have already started one via
// server.setContentLength(CONTENT_LENGTH_UNKNOWN) + server.send(200, ...,
// "")). Building the whole Settings page as a single String before sending
// - with up to MAX_PARAMS (100) table rows, each expanding a full type
// dropdown (getTypeOptions(), O(typeCount) per row) - could grow into tens
// of KB; a single contiguous allocation/realloc that large can fail on the
// ESP32's fragmented heap. Arduino String silently drops content it
// couldn't append rather than raising an error, which desyncs the HTML tag
// structure and was rendering as literal visible text (a live report: 30
// param rows made the page unusable). Flushing periodically bounds peak
// buffer size to one chunk regardless of row count.
void flushHtmlChunk(String &html) {
  if (html.length() > 0) {
    server.sendContent(html);
    html = "";
  }
}

// ===================== Settings Page =====================
void handleSettings() {
  // Chunked response (see flushHtmlChunk()) instead of one server.send() at
  // the end - required so the page can never need one giant contiguous
  // buffer, no matter how many param/type/TCP-target rows are configured.
  server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  server.send(200, "text/html", "");

  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>ESP32 Modbus Gateway - Settings</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <style>
    body { font-family: Arial; background:#f5f7fa; margin:15px; }
    h2 { color:#00a5df; }
    .box { background:white; padding:12px; margin-bottom:15px; box-shadow:0 1px 4px rgba(0,0,0,0.2); }
    table { width:100%; border-collapse:collapse; }
    th { background:#087b7b; color:white; padding:8px; }
    td { border:1px solid #2999c5; padding:6px; text-align:center; }
    input, select { width:95%; padding:5px; }
    button { padding:8px 14px; background:#0099cc; color:white; border:none; margin:2px; border-radius:4px; cursor:pointer; }
    .nav { background:#6f42c1; }
    .danger { background:#dc3545; }
    .add { background:#28a745; }
    .insert { background:#17a2b8; }
    .remove { background:#f4a340; color:black; }
    .success { background:#d4edda; color:#155724; padding:10px; margin-bottom:15px; }
    .error { background:#f8d7da; color:#721c24; padding:10px; margin-bottom:15px; }
  </style>
</head>
<body>

<h2>Modbus Settings</h2>

<button class="nav" type="button" onclick="location.href='/'">Dashboard</button>
<button class="nav" type="button" onclick="location.href='/types'">Manage Data Types</button>
<button class="danger" type="button" onclick="confirmResetModbus()">Reset Modbus</button>

)rawliteral";

  if (server.hasArg("saveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; param changes may not have persisted across reboot.</div>";
  } else if (server.hasArg("saved")) {
    html += "<div class='success'>Modbus Settings Saved Successfully</div>";
  }

  html += R"rawliteral(
<form action="/save" method="POST">
<input type="hidden" id="rowCount" name="rowCount" value=")rawliteral";

  html += String(paramCount);

  html += R"rawliteral(">

<table id="paramTable">
<thead>
<tr>
<th>Name</th>
<th>Transport</th>
<th>Register Area</th>
<th>Data Type</th>
<th>Data Length</th>
<th>Device ID / TCP Target</th>
<th>Register Address</th>
<th>Writable</th>
<th>Write Min / Max</th>
<th>Enable</th>
<th>Action</th>
</tr>
</thead>
<tbody>
)rawliteral";

  for (int i = 0; i < paramCount; i++) {
    uint16_t dlen = isBitArea(params[i].area) ? 1 : getDataLengthForType(params[i].type);
    bool rowIsTcp = (params[i].transport == TRANSPORT_TCP);

    html += "<tr>";
    html += "<td><input name='name" + String(i) + "' value='" + htmlEscape(params[i].name) + "'></td>";

    html += "<td><select name='transport" + String(i) + "' onchange='updateTransportSelection(this)'>";
    html += "<option value='0'" + String(rowIsTcp ? "" : " selected") + ">RTU (RS485)</option>";
    html += "<option value='1'" + String(rowIsTcp ? " selected" : "") + ">TCP (Network)</option>";
    html += "</select></td>";

    html += "<td><select name='area" + String(i) + "' onchange='updateAreaSelection(this)'>" + getAreaOptions(params[i].area) + "</select></td>";

    // Bit areas auto-select a fixed BIT pseudo-type (the only thing a bit
    // can be); register areas get the normal user-managed type list.
    html += "<td><select name='type" + String(i) + "' onchange='updateDataLength(this)'>";
    if (isBitArea(params[i].area)) {
      html += "<option value='BIT' selected>BIT (0/1)</option>";
    } else {
      html += getTypeOptions(params[i].type);
    }
    html += "</select></td>";
    html += "<td><input class='dlen-display' value='" + String(dlen) + "' readonly></td>";

    // Both fields always render (and submit) - server reads whichever the
    // submitted transport value selects; JS only toggles which is visible.
    html += "<td>";
    html += "<div class='rtu-field' style='display:" + String(rowIsTcp ? "none" : "block") + "'>";
    html += "<input name='sid" + String(i) + "' placeholder='RTU Slave ID' value='" + String(params[i].slaveId) + "'>";
    html += "</div>";
    html += "<div class='tcp-field' style='display:" + String(rowIsTcp ? "block" : "none") + "'>";
    html += "<select name='tcptgt" + String(i) + "'>" + getTcpTargetOptions(params[i].tcpTargetIndex) + "</select>";
    html += "</div>";
    html += "</td>";

    html += "<td><input name='addr" + String(i) + "' value='" + String(params[i].registerAddress) + "'></td>";

    {
      bool rowWritable = isWritableArea(params[i].area);
      html += "<td><input type='checkbox' name='wr" + String(i) + "' onchange='updateWritableFields(this)'";
      if (params[i].writable) html += " checked";
      if (!rowWritable) html += " disabled";
      html += "></td>";
      html += "<td>";
      html += "<input name='wmin" + String(i) + "' placeholder='Min' style='width:70px' value='" + String(params[i].minWriteValue) + "'";
      if (!rowWritable) html += " disabled";
      html += "> / ";
      html += "<input name='wmax" + String(i) + "' placeholder='Max' style='width:70px' value='" + String(params[i].maxWriteValue) + "'";
      if (!rowWritable) html += " disabled";
      html += ">";
      html += "</td>";
    }

    html += "<td><input type='checkbox' name='en" + String(i) + "'";
    if (params[i].enabled) html += " checked";
    html += "></td>";
    html += "<td>";
    html += "<button type='button' class='add' onclick='addParamRowAtEnd()'>Add</button>";
    html += "<button type='button' class='insert' onclick='insertParamRowAfter(this)'>Insert</button>";
    html += "<button type='button' class='remove' onclick='deleteParamRow(this)'>Delete</button>";
    html += "</td>";
    html += "</tr>";

    // Flush every row - this is the loop whose total size scales with
    // paramCount (up to MAX_PARAMS=100), so it's the one that must never
    // be allowed to accumulate into one huge buffer.
    flushHtmlChunk(html);
  }

  html += R"rawliteral(
</tbody>
</table>
<br>
<button class="add" type="button" onclick="addParamRowAtEnd()">Add Record</button>
<button type="submit">Save Modbus</button>
</form>

<script>
const paramTypeInfo = [)rawliteral";

  for (int i = 0; i < typeCount; i++) {
    if (i > 0) html += ",";
    html += "{\"name\":\"" + jsonEscape(typeList[i].name) + "\",\"length\":" + String(typeList[i].dataLength) + "}";
  }

  html += R"rawliteral(];

const tcpTargetInfo = [)rawliteral";

  for (int i = 0; i < tcpTargetCount; i++) {
    if (i > 0) html += ",";
    html += "{\"name\":\"" + jsonEscape(tcpTargets[i].name) + "\"}";
  }

  html += R"rawliteral(];

function buildTcpTargetOptionsHTML(selected) {
  let opts = "";
  tcpTargetInfo.forEach((t, i) => {
    opts += `<option value="${i}"${i === selected ? " selected" : ""}>${t.name}</option>`;
  });
  return opts;
}

function updateTransportSelection(selectEl) {
  let row = selectEl.closest("tr");
  let isTcp = (selectEl.value == "1");
  row.querySelector(".rtu-field").style.display = isTcp ? "none" : "block";
  row.querySelector(".tcp-field").style.display = isTcp ? "block" : "none";
}

function buildTypeOptionsHTML(selected) {
  let opts = "";
  paramTypeInfo.forEach(t => {
    opts += `<option value="${t.name}"${t.name === selected ? " selected" : ""}>${t.name}</option>`;
  });
  return opts;
}

const areaNames = ["Holding Register", "Input Register", "Coil", "Discrete Input"];

function buildAreaOptionsHTML(selected) {
  let opts = "";
  areaNames.forEach((n, i) => {
    opts += `<option value="${i}"${i === selected ? " selected" : ""}>${n}</option>`;
  });
  return opts;
}

function isBitAreaValue(v) {
  return v == 2 || v == 3;  // Coil / Discrete Input
}

function isWritableAreaValue(v) {
  return v == 0 || v == 2;  // Holding Register / Coil (matches isWritableArea() server-side)
}

function updateWritableFields(row) {
  // Accepts either the checkbox itself or its containing <tr>.
  if (row.tagName !== "TR") row = row.closest("tr");
  let wrBox = row.querySelector("input[name^='wr']");
  let minInput = row.querySelector("input[name^='wmin']");
  let maxInput = row.querySelector("input[name^='wmax']");
  let areaSel = row.querySelector("select[name^='area']");
  let writableArea = areaSel && isWritableAreaValue(areaSel.value);

  wrBox.disabled = !writableArea;
  if (!writableArea) wrBox.checked = false;
  minInput.disabled = !writableArea;
  maxInput.disabled = !writableArea;
}

function updateDataLength(selectEl) {
  let row = selectEl.closest("tr");
  let areaSel = row.querySelector("select[name^='area']");
  let lenInput = row.querySelector(".dlen-display");

  if (areaSel && isBitAreaValue(areaSel.value)) {
    lenInput.value = 1;  // bits are always length 1; data type doesn't apply
    return;
  }

  let info = paramTypeInfo.find(t => t.name === selectEl.value);
  lenInput.value = info ? info.length : 1;
}

function updateAreaSelection(selectEl) {
  let row = selectEl.closest("tr");
  let typeSel = row.querySelector("select[name^='type']");

  if (isBitAreaValue(selectEl.value)) {
    // Coil/discrete: auto-select the fixed BIT pseudo-type.
    typeSel.innerHTML = '<option value="BIT" selected>BIT (0/1)</option>';
  } else if (typeSel.querySelector('option[value="BIT"]')) {
    // Switched back to a register area: restore the normal type list.
    let defaultType = paramTypeInfo.length ? paramTypeInfo[0].name : "";
    typeSel.innerHTML = buildTypeOptionsHTML(defaultType);
  }

  updateDataLength(typeSel);
  updateWritableFields(row);
}

function buildParamRow() {
  let row = document.createElement("tr");
  let defaultType = paramTypeInfo.length ? paramTypeInfo[0].name : "";
  let defaultLen = paramTypeInfo.length ? paramTypeInfo[0].length : 1;

  row.innerHTML = `
    <td><input name="nameX" value=""></td>
    <td><select name="transportX" onchange="updateTransportSelection(this)">
      <option value="0" selected>RTU (RS485)</option>
      <option value="1">TCP (Network)</option>
    </select></td>
    <td><select name="areaX" onchange="updateAreaSelection(this)">${buildAreaOptionsHTML(0)}</select></td>
    <td><select name="typeX" onchange="updateDataLength(this)">${buildTypeOptionsHTML(defaultType)}</select></td>
    <td><input class="dlen-display" value="${defaultLen}" readonly></td>
    <td>
      <div class="rtu-field" style="display:block"><input name="sidX" placeholder="RTU Slave ID" value="1"></div>
      <div class="tcp-field" style="display:none"><select name="tcptgtX">${buildTcpTargetOptionsHTML(0)}</select></div>
    </td>
    <td><input name="addrX" value="0"></td>
    <td><input type="checkbox" name="wrX" onchange="updateWritableFields(this)" disabled></td>
    <td><input name="wminX" placeholder="Min" style="width:70px" value="0" disabled> / <input name="wmaxX" placeholder="Max" style="width:70px" value="0" disabled></td>
    <td><input type="checkbox" name="enX" checked></td>
    <td>
      <button type="button" class="add" onclick="addParamRowAtEnd()">Add</button>
      <button type="button" class="insert" onclick="insertParamRowAfter(this)">Insert</button>
      <button type="button" class="remove" onclick="deleteParamRow(this)">Delete</button>
    </td>
  `;

  return row;
}

function addParamRowAtEnd() {
  let body = document.querySelector("#paramTable tbody");
  body.appendChild(buildParamRow());
  renumberParamRows();
}

function insertParamRowAfter(button) {
  let currentRow = button.closest("tr");
  currentRow.after(buildParamRow());
  renumberParamRows();
}

function deleteParamRow(button) {
  button.closest("tr").remove();
  renumberParamRows();
}

function renumberParamRows() {
  let rows = document.querySelectorAll("#paramTable tbody tr");

  rows.forEach((row, index) => {
    row.querySelector("input[name^='name']").name = "name" + index;
    row.querySelector("select[name^='transport']").name = "transport" + index;
    row.querySelector("select[name^='area']").name = "area" + index;
    row.querySelector("select[name^='type']").name = "type" + index;
    row.querySelector("input[name^='sid']").name = "sid" + index;
    row.querySelector("select[name^='tcptgt']").name = "tcptgt" + index;
    row.querySelector("input[name^='addr']").name = "addr" + index;
    row.querySelector("input[name^='wr']").name = "wr" + index;
    row.querySelector("input[name^='wmin']").name = "wmin" + index;
    row.querySelector("input[name^='wmax']").name = "wmax" + index;
    row.querySelector("input[name^='en']").name = "en" + index;
  });

  document.getElementById("rowCount").value = rows.length;
}

function confirmResetModbus() {
  if (confirm("This will permanently delete ALL Modbus parameter records. This cannot be undone. Continue?")) {
    location.href = '/reset';
  }
}
</script>
)rawliteral";

  flushHtmlChunk(html);

  // DEVICE / AP IDENTITY FORM
  html += "<div class='box'>";
  html += "<h2>Device Identity</h2>";

  if (server.hasArg("deviceSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; device settings may not have persisted across reboot.</div>";
  } else if (server.hasArg("deviceSaved")) {
    html += "<div class='success'>Device Settings Saved Successfully</div>";
  }

  {
    String mac = getDeviceMacID();
    String macSuffix = (mac.length() >= 5) ? mac.substring(mac.length() - 5) : mac;

    html += "<script>";
    html += "const macSuffix5 = \"" + jsonEscape(macSuffix) + "\";";
    html += "const fullMac = \"" + jsonEscape(mac) + "\";";
    html += R"rawscript(
function updateDeviceNamePreview() {
  let prefix = document.getElementById('devicePrefixInput').value.trim();
  let preview = prefix.length > 0 ? (prefix + "_" + macSuffix5) : fullMac;
  document.getElementById('deviceNamePreview').textContent = preview;

  if (document.getElementById('apMatchCheckbox').checked) {
    document.getElementById('apSsidInput').value = preview;
  }
}

function toggleApSsidField() {
  let matched = document.getElementById('apMatchCheckbox').checked;
  document.getElementById('apSsidInput').disabled = matched;
  if (matched) {
    updateDeviceNamePreview();
  }
}
)rawscript";
    html += "</script>";

    html += "<form action='/saveDevice' method='POST'>";
    html += "<table>";

    html += "<tr><th>Device Name</th><td>";
    html += "<input type='text' id='devicePrefixInput' name='devicePrefix' maxlength='" + String(DEVICE_NAME_PREFIX_MAX_LEN) + "' placeholder='Up to " + String(DEVICE_NAME_PREFIX_MAX_LEN) + " chars' value='" + htmlEscape(deviceConfig.deviceNamePrefix) + "' oninput='updateDeviceNamePreview()'>";
    html += "<br><small>Final Device Name: <b id='deviceNamePreview'>" + htmlEscape(getDeviceName()) + "</b>";
    html += " (prefix + last 5 characters of the MAC address). Leave blank to use the full MAC address.</small>";
    html += "</td></tr>";

    html += "<tr><th>AP Name (SSID)</th><td>";
    html += "<input type='text' id='apSsidInput' name='apSsid' value='" + htmlEscape(getEffectiveApSsid()) + "'";
    if (deviceConfig.apMatchesDeviceName) {
      html += " disabled";
    }
    html += ">";
    html += "<br><label><input type='checkbox' id='apMatchCheckbox' name='apMatchDevice' onchange='toggleApSsidField()'";
    if (deviceConfig.apMatchesDeviceName) {
      html += " checked";
    }
    html += "> Match AP Name with Device Name</label>";
    html += "</td></tr>";

    html += "<tr><td colspan='2'><button type='submit'>Save Device Settings</button></td></tr>";
    html += "</table>";
    html += "</form>";
  }

  html += "</div>";

  flushHtmlChunk(html);

  // NETWORK CONFIG FORM
  html += "<div class='box'>";
  html += "<h2>WiFi &amp; TCP Configuration</h2>";

  if (server.hasArg("uplinkSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; network settings may not have persisted across reboot.</div>";
  } else if (server.hasArg("uplinkSaved")) {
    html += "<div class='success'>Network Settings Saved Successfully</div>";
  }

  html += "<form action='/saveUplink' method='POST'>";
  html += "<table>";
  html += "<tr><th>WiFi SSID</th><td><input type='text' name='ssid' value='" + htmlEscape(uplinkConfig.ssid) + "'></td></tr>";
  html += "<tr><th>WiFi Password</th><td><input type='password' name='password' placeholder='Leave blank to keep unchanged'></td></tr>";
  html += "<tr><th>Master IP</th><td><input type='text' name='ip' value='" + htmlEscape(uplinkConfig.serverIP) + "'></td></tr>";
  html += "<tr><th>Master Port</th><td><input type='number' name='port' value='" + String(uplinkConfig.port) + "'>"
          "<br><small>When Uplink Mode below is Raw TCP, this connection is bidirectional: the master can send newline-framed JSON commands back - write: <code>{\"param\":\"Name\",\"value\":42}</code> (acked), or on-demand read: <code>{\"query\":\"Name\"}</code> (answered immediately, separate from the periodic telemetry broadcast).</small></td></tr>";

  html += "<tr><th>On-Premise NTP Server</th><td><input type='text' name='ntpServer' maxlength='" + String(NTP_SERVER_MAX_LEN - 1) + "' placeholder='Optional, e.g. 192.168.1.10 or ntp.local' value='" + htmlEscape(uplinkConfig.ntpServer) + "'>"
          "<br><small>Tried first for time sync; falls back to public NTP (" + String(NTP_SERVER_1) + "), then uptime-based timestamps. Leave blank to use public NTP only.</small></td></tr>";
  html += "<tr><td colspan='2'><button type='submit'>Save Network</button></td></tr>";
  html += "</table>";
  html += "</form>";
  html += "</div>";

  flushHtmlChunk(html);

  // CLOUD UPLINK FORM (raw TCP vs AWS IoT MQTT)
  html += "<div class='box'>";
  html += "<h2>Cloud Uplink</h2>";

  if (server.hasArg("cloudSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; cloud settings/certs may not have persisted across reboot.</div>";
  } else if (server.hasArg("cloudSaved")) {
    html += "<div class='success'>Cloud Settings Saved Successfully</div>";
  }

  // multipart so the cert file-upload inputs work; ordinary fields still
  // arrive via server.arg() as usual.
  html += "<form action='/saveCloud' method='POST' enctype='multipart/form-data'>";
  html += "<table>";

  html += "<tr><th>Uplink Mode</th><td><select name='uplinkMode'>";
  html += "<option value='tcp'";
  if (cloudConfig.mode == UPLINK_MODE_TCP) html += " selected";
  html += ">Raw TCP (Master IP/Port above)</option>";
  html += "<option value='mqtt'";
  if (cloudConfig.mode == UPLINK_MODE_MQTT) html += " selected";
  html += ">MQTT over TLS (AWS IoT / any broker)</option>";
  html += "</select></td></tr>";

  html += "<tr><th>MQTT Endpoint</th><td><input type='text' name='mqttEndpoint' placeholder='xxxx-ats.iot.region.amazonaws.com' value='" + htmlEscape(cloudConfig.endpoint) + "'></td></tr>";
  html += "<tr><th>MQTT Port</th><td><input type='number' name='mqttPort' value='" + String(cloudConfig.port) + "'></td></tr>";
  html += "<tr><th>MQTT Client ID</th><td><input type='text' name='mqttClientId' placeholder='Blank = Device Name (" + htmlEscape(getDeviceName()) + ")' value='" + htmlEscape(cloudConfig.clientId) + "'>"
          "<br><small>Should match the AWS IoT Thing name / policy.</small></td></tr>";
  html += "<tr><th>Publish Topic</th><td><input type='text' name='mqttTopic' value='" + htmlEscape(cloudConfig.topic) + "'></td></tr>";

  html += "<tr><th>MQTT Command Topic</th><td><input type='checkbox' name='cmdEnabled'";
  if (cloudConfig.commandsEnabled) html += " checked";
  html += "> Enable (subscribes to <code>" + htmlEscape(mqttCommandTopic()) + "</code> - write: <code>{\"param\":\"Name\",\"value\":42}</code>, read: <code>{\"query\":\"Name\"}</code>; acks/responses on <code>" + htmlEscape(mqttCommandAckTopic()) + "</code>. Plain MQTT topics - works with any broker, no AWS-specific features.)</td></tr>";

  // Certs: never echoed back - a blank/empty field keeps the stored value,
  // same pattern as the WiFi password above. Each can be provided either
  // by uploading the file from AWS as-is or by pasting the PEM text; the
  // uploaded file wins if both are given.
  html += "<tr><th>Root CA (PEM)</th><td>";
  html += "<input type='file' name='caFile' accept='.pem,.crt,.txt'>";
  html += "<br><small>or paste below:</small>";
  html += "<br><textarea name='caCert' rows='3' style='width:95%' placeholder='";
  html += (certRootCA.length() > 0) ? "(stored, " + String(certRootCA.length()) + " bytes - upload or paste to replace)" : "Paste Amazon Root CA 1 PEM here";
  html += "'></textarea></td></tr>";

  html += "<tr><th>Device Certificate (PEM)</th><td>";
  html += "<input type='file' name='devFile' accept='.pem,.crt,.txt'>";
  html += "<br><small>or paste below:</small>";
  html += "<br><textarea name='devCert' rows='3' style='width:95%' placeholder='";
  html += (certDevice.length() > 0) ? "(stored, " + String(certDevice.length()) + " bytes - upload or paste to replace)" : "Paste device certificate PEM here";
  html += "'></textarea></td></tr>";

  html += "<tr><th>Private Key (PEM)</th><td>";
  html += "<input type='file' name='keyFile' accept='.pem,.key,.txt'>";
  html += "<br><small>or paste below:</small>";
  html += "<br><textarea name='privKey' rows='3' style='width:95%' placeholder='";
  html += (certPrivKey.length() > 0) ? "(stored, " + String(certPrivKey.length()) + " bytes - upload or paste to replace)" : "Paste device private key PEM here";
  html += "'></textarea></td></tr>";

  html += "<tr><td colspan='2'><button type='submit'>Save Cloud Settings</button></td></tr>";
  html += "</table>";
  html += "</form>";
  html += "</div>";

  flushHtmlChunk(html);

  // MODBUS TCP TARGETS FORM (second data source, e.g. a CNC controller)
  html += "<div class='box'>";
  html += "<h2>Modbus TCP Targets</h2>";
  html += "<p><small>Remote Modbus TCP servers (e.g. a CNC controller) this gateway polls over WiFi, as a second data source alongside the RS485 RTU bus below. Reference a target from the \"TCP Target\" column in the Modbus Settings table. Leave Name/IP blank to leave a slot unused.</small></p>";

  if (server.hasArg("tcpTargetsSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; TCP targets may not have persisted across reboot.</div>";
  } else if (server.hasArg("tcpTargetsSaved")) {
    html += "<div class='success'>Modbus TCP Targets Saved Successfully</div>";
  }

  html += "<form action='/saveTcpTargets' method='POST'>";
  html += "<table>";
  html += "<tr><th>#</th><th>Name</th><th>IP Address</th><th>Port</th><th>Unit ID</th></tr>";

  for (int i = 0; i < MAX_TCP_TARGETS; i++) {
    bool used = (i < tcpTargetCount);
    String tname = used ? tcpTargets[i].name : "";
    String tip = used ? tcpTargets[i].ip : "";
    uint16_t tport = used ? tcpTargets[i].port : 502;
    uint8_t tunit = used ? tcpTargets[i].unitId : 1;

    html += "<tr>";
    html += "<td>" + String(i) + "</td>";
    html += "<td><input type='text' name='tgtName" + String(i) + "' placeholder='e.g. CNC' value='" + htmlEscape(tname) + "'></td>";
    html += "<td><input type='text' name='tgtIp" + String(i) + "' placeholder='192.168.1.50' value='" + htmlEscape(tip) + "'></td>";
    html += "<td><input type='number' name='tgtPort" + String(i) + "' value='" + String(tport) + "'></td>";
    html += "<td><input type='number' name='tgtUnit" + String(i) + "' min='0' max='255' value='" + String(tunit) + "'></td>";
    html += "</tr>";
  }

  html += "<tr><td colspan='5'><button type='submit'>Save TCP Targets</button></td></tr>";
  html += "</table>";
  html += "</form>";
  html += "</div>";

  flushHtmlChunk(html);

  // DIGITAL I/O FORM (local GPIO - independent of the Modbus buses above)
  html += "<div class='box'>";
  html += "<h2>Digital I/O</h2>";
  html += "<p><small>Local GPIO channels, read/written directly - not Modbus. Digital In reads inverted (see firmware notes: HIGH/idle = false, pulled LOW = true) to match a typical opto-isolator input module; Digital Out writes go through the same write-queue/command-channel path as writable Modbus parameters (Dashboard, MQTT command topic, raw TCP), keyed by name.</small></p>";

  if (server.hasArg("digioSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; Digital I/O settings may not have persisted across reboot.</div>";
  } else if (server.hasArg("digioSaved")) {
    html += "<div class='success'>Digital I/O Settings Saved Successfully</div>";
  }

  html += "<form action='/saveDigitalIO' method='POST'>";

  html += "<h3>Digital In</h3><table>";
  html += "<tr><th>#</th><th>Pin</th><th>Name</th><th>Enable</th></tr>";
  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    html += "<tr><td>" + String(i) + "</td><td>" + String(DI_PIN_LABELS[i]) + "</td>";
    html += "<td><input name='diName" + String(i) + "' value='" + htmlEscape(diChannels[i].name) + "'></td>";
    html += "<td><input type='checkbox' name='diEn" + String(i) + "'" + String(diChannels[i].enabled ? " checked" : "") + "></td></tr>";
  }
  html += "</table>";

  html += "<h3>Digital Out</h3><table>";
  html += "<tr><th>#</th><th>Pin</th><th>Name</th><th>Enable</th></tr>";
  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    html += "<tr><td>" + String(i) + "</td><td>" + String(DO_PIN_LABELS[i]) + "</td>";
    html += "<td><input name='doName" + String(i) + "' value='" + htmlEscape(doChannels[i].name) + "'></td>";
    html += "<td><input type='checkbox' name='doEn" + String(i) + "'" + String(doChannels[i].enabled ? " checked" : "") + "></td></tr>";
  }
  html += "</table>";

  html += "<h3>Analog In</h3><table>";
  html += "<tr><th>#</th><th>Pin</th><th>Name</th><th>Enable</th><th>Scale</th><th>Offset</th></tr>";
  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    html += "<tr><td>" + String(i) + "</td><td>" + String(AI_PIN_LABELS[i]) + "</td>";
    html += "<td><input name='aiName" + String(i) + "' value='" + htmlEscape(aiChannels[i].name) + "'></td>";
    html += "<td><input type='checkbox' name='aiEn" + String(i) + "'" + String(aiChannels[i].enabled ? " checked" : "") + "></td>";
    html += "<td><input name='aiScale" + String(i) + "' value='" + String(aiChannels[i].scale, 5) + "'></td>";
    html += "<td><input name='aiOffset" + String(i) + "' value='" + String(aiChannels[i].offset, 3) + "'></td></tr>";
  }
  html += "</table>";
  html += "<p><small>Engineering value = raw ADC (0-4095) &times; Scale + Offset.</small></p>";

  html += "<button type='submit'>Save Digital I/O</button>";
  html += "</form>";
  html += "</div>";

  flushHtmlChunk(html);

  // CELLULAR MODEM FORM (SIM7600G-H)
  html += "<div class='box'>";
  html += "<h2>Cellular Modem</h2>";
  html += "<p><small>APN for the SIM7600G-H's data session. Leave blank to let the modem attempt auto-provisioning from the SIM - set explicitly if your carrier requires it and the Dashboard's Data Session never comes up.</small></p>";

  if (server.hasArg("cellularSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; APN may not have persisted across reboot.</div>";
  } else if (server.hasArg("cellularSaved")) {
    html += "<div class='success'>Cellular Modem Settings Saved Successfully</div>";
  }

  html += "<form action='/saveCellular' method='POST'>";
  html += "<table>";
  html += "<tr><th>APN</th><td><input type='text' name='apn' placeholder='e.g. www or a carrier-specific M2M APN' value='" + htmlEscape(cellularConfig.apn) + "'></td></tr>";
  html += "<tr><th>Publish Interval (s)</th><td><input type='number' name='pubIntervalS' min='" + String(CELLULAR_PUBLISH_INTERVAL_MIN_MS / 1000) + "' max='" + String(CELLULAR_PUBLISH_INTERVAL_MAX_MS / 1000) + "' value='" + String(cellularConfig.publishIntervalMs / 1000) + "'>"
          "<br><small>How often telemetry is published while the MQTT uplink is failed over to cellular. Higher = less SIM data used. Does not affect WiFi publishing (which follows the Modbus poll interval) or inbound command responsiveness.</small></td></tr>";
  html += "<tr><td colspan='2'><button type='submit'>Save Cellular Modem</button></td></tr>";
  html += "</table>";
  html += "</form>";
  html += "</div>";

  flushHtmlChunk(html);

  // COMMUNICATION (SERIAL) SETTINGS FORM
  html += "<div class='box'>";
  html += "<h2>RS485 Communication Settings</h2>";

  if (server.hasArg("commSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; communication settings may not have persisted across reboot.</div>";
  } else if (server.hasArg("commSaved")) {
    html += "<div class='success'>Communication Settings Saved Successfully</div>";
  }

  html += "<form action='/saveCommunication' method='POST'>";
  html += "<table>";
  html += "<tr><th>Baud Rate</th><td><input type='number' name='baud' value='" + String(commBaudRate) + "'></td></tr>";

  html += "<tr><th>Parity</th><td><select name='parity'>";
  html += "<option value='N'";
  if (commParity == "N") html += " selected";
  html += ">None</option>";
  html += "<option value='E'";
  if (commParity == "E") html += " selected";
  html += ">Even</option>";
  html += "<option value='O'";
  if (commParity == "O") html += " selected";
  html += ">Odd</option>";
  html += "</select></td></tr>";

  html += "<tr><th>Stop Bits</th><td><select name='stop'>";
  html += "<option value='1'";
  if (commStopBits == 1) html += " selected";
  html += ">1</option>";
  html += "<option value='2'";
  if (commStopBits == 2) html += " selected";
  html += ">2</option>";
  html += "</select></td></tr>";

  html += "<tr><th>Poll Interval (ms)</th><td><input type='number' name='pollInterval' min='" + String(MIN_POLL_INTERVAL_MS) + "' max='" + String(MAX_POLL_INTERVAL_MS) + "' value='" + String(pollIntervalMs) + "'></td></tr>";

  html += "<tr><th>Slave Recovery Delay (ms)</th><td><input type='number' name='recoveryDelay' min='" + String(MIN_SLAVE_RECOVERY_DELAY_MS) + "' max='" + String(MAX_SLAVE_RECOVERY_DELAY_MS) + "' value='" + String(slaveRecoveryDelayMs) + "'>"
          "<br><small>Pause after each parameter before polling the next. Lower = faster polling; raise it if parameters start erroring right after a fast one.</small></td></tr>";

  html += "<tr><td colspan='2'><button type='submit'>Save Communication</button></td></tr>";
  html += "</table>";
  html += "</form>";
  html += "</div>";

  html += R"rawliteral(
</body>
</html>
)rawliteral";

  flushHtmlChunk(html);
  server.sendContent("");  // zero-length chunk: terminates the chunked response
}

// ===================== Types Page =====================
void handleTypes() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
<title>Data Type Settings</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<style>
body { font-family: Arial; background:#f5f7fa; margin:15px; }
h2 { color:#00a5df; }
table { width:100%; min-width:700px; border-collapse:collapse; background:white; }
th { background:#087b7b; color:white; padding:10px; }
td { border:1px solid #2999c5; padding:5px; text-align:center; }
input { width:95%; padding:5px; }
button { padding:9px 14px; margin:5px; border:none; border-radius:4px; cursor:pointer; }
.save { background:#0099cc; color:white; }
.add { background:#28a745; color:white; }
.back { background:#6f42c1; color:white; }
.reset { background:#dc3545; color:white; }
.remove { background:#f4a340; color:black; }
.success { background:#d4edda; color:#155724; padding:10px; margin-bottom:15px; }
.error { background:#f8d7da; color:#721c24; padding:10px; margin-bottom:15px; }
</style>
</head>
<body>
<h2>Data Type Settings</h2>
<button class="back" type="button" onclick="location.href='/settings'">Back</button>
<button class="add" type="button" onclick="addTypeRow()">Add Data Type</button>
<button class="reset" type="button" onclick="confirmResetTypes()">Reset Data Types</button>
)rawliteral";

  if (server.hasArg("typeSaveError")) {
    html += "<div class='error'>Save FAILED - flash may be full. Check the Status Log; data type changes may not have persisted across reboot.</div>";
  } else if (server.hasArg("typeSaved")) {
    html += "<div class='success'>Data Types Saved Successfully</div>";
  }

  if (server.hasArg("typeReset")) {
    html += "<div class='success'>Data Types Reset to Defaults</div>";
  }

  html += R"rawliteral(
<form action="/saveTypes" method="POST">
<input type="hidden" id="typeRowCount" name="typeRowCount" value=")rawliteral";

  html += String(typeCount);

  html += R"rawliteral(">
<table id="typeTable">
<thead>
<tr>
<th>Data Type Name</th>
<th>Divisor</th>
<th>Data Length</th>
<th>Remove</th>
</tr>
</thead>
<tbody>
)rawliteral";

  for (int i = 0; i < typeCount; i++) {
    typeList[i].baseFormat = getBaseFormatFromType(typeList[i].name);
    typeList[i].dataLength = inferDataLengthFromType(typeList[i].name);

    html += "<tr>";
    html += "<td><input type='text' name='tname" + String(i) + "' value='" + htmlEscape(typeList[i].name) + "'></td>";
    html += "<td><input type='number' step='0.001' min='0.001' name='div" + String(i) + "' value='" + String(typeList[i].divisor, 3) + "'></td>";
    html += "<td><input type='number' value='" + String(typeList[i].dataLength) + "' readonly></td>";
    html += "<td><button type='button' class='remove' onclick='removeTypeRow(this)'>Remove</button></td>";
    html += "</tr>";
  }

  html += R"rawliteral(
</tbody>
</table>
<br>
<button type="submit" class="save">Save Data Types</button>
</form>

<script>
function addTypeRow() {
  let body = document.querySelector("#typeTable tbody");
  let row = document.createElement("tr");

  row.innerHTML = `
    <td><input type="text" name="tnameX" value="INT_16/100"></td>
    <td><input type="number" step="0.001" min="0.001" name="divX" value="100"></td>
    <td><input type="number" value="1" readonly></td>
    <td><button type="button" class="remove" onclick="removeTypeRow(this)">Remove</button></td>
  `;

  body.appendChild(row);
  renumberTypeRows();
}

function removeTypeRow(button) {
  button.closest("tr").remove();
  renumberTypeRows();
}

function renumberTypeRows() {
  let rows = document.querySelectorAll("#typeTable tbody tr");

  rows.forEach((row, index) => {
    row.querySelector("input[name^='tname']").name = "tname" + index;
    row.querySelector("input[name^='div']").name = "div" + index;
  });

  document.getElementById("typeRowCount").value = rows.length;
}

function confirmResetTypes() {
  if (confirm("This will permanently delete ALL data type records and restore the defaults. This cannot be undone. Continue?")) {
    location.href = '/resetTypes';
  }
}
</script>
</body>
</html>
)rawliteral";

  server.send(200, "text/html", html);
}

// ===================== Save Modbus Settings =====================
void handleSave() {
  int rows = server.arg("rowCount").toInt();

  if (rows < 0) rows = 0;
  if (rows > MAX_PARAMS) rows = MAX_PARAMS;

  logKeyEvent("MODBUS SETTINGS SAVED: " + String(rows) + " rows");

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  paramCount = rows;

  for (int i = 0; i < paramCount; i++) {
    String index = String(i);

    params[i].name = server.arg("name" + index);

    params[i].transport = server.arg("transport" + index).toInt();
    if (params[i].transport > TRANSPORT_TCP) {
      params[i].transport = TRANSPORT_RTU;
    }

    params[i].area = server.arg("area" + index).toInt();
    if (params[i].area > AREA_DISCRETE_INPUT) {
      params[i].area = AREA_HOLDING_REGISTER;
    }

    params[i].type = server.arg("type" + index);

    if (isBitArea(params[i].area)) {
      // Auto-selected pseudo-type for coil/discrete rows - not part of the
      // user-managed type list, so it must skip the fallback below.
      params[i].type = "BIT";
    } else if (params[i].type.length() == 0 || findTypeIndex(params[i].type) < 0) {
      if (typeCount > 0) {
        params[i].type = typeList[0].name;
      } else {
        params[i].type = "INT_16/100";
      }
    }

    params[i].slaveId = server.arg("sid" + index).toInt();

    params[i].tcpTargetIndex = server.arg("tcptgt" + index).toInt();
    if (params[i].tcpTargetIndex >= MAX_TCP_TARGETS) {
      params[i].tcpTargetIndex = 0;
    }

    params[i].registerAddress = server.arg("addr" + index).toInt();
    params[i].registerLength = isBitArea(params[i].area) ? 1 : getDataLengthForType(params[i].type);
    params[i].enabled = server.hasArg("en" + index);

    // Writable/min/max only apply to holding registers and coils - force
    // off/cleared for any other area regardless of what was submitted.
    params[i].writable = isWritableArea(params[i].area) && server.hasArg("wr" + index);
    params[i].minWriteValue = server.arg("wmin" + index).toFloat();
    params[i].maxWriteValue = server.arg("wmax" + index).toFloat();
    if (!params[i].writable || params[i].maxWriteValue <= params[i].minWriteValue) {
      // Invalid/absent range collapses to the "no limit configured" sentinel.
      params[i].minWriteValue = 0;
      params[i].maxWriteValue = 0;
    }

    params[i].value = 0;
    params[i].valid = false;
    params[i].errorCode = 0;
    params[i].lastUpdateTime = 0;

    resetDebugState(i);

    if (params[i].name.length() == 0) {
      params[i].name = "Parameter " + String(i + 1);
    }

    if (params[i].slaveId < 1) params[i].slaveId = 1;
    if (params[i].slaveId > 247) params[i].slaveId = 247;
  }

  xSemaphoreGive(dataMutex);

  // registerLength persisted here is recomputed from type on every load
  // anyway (see loadSettings()), so it's safe to persist without the lock.
  bool ok = saveSettings();

  server.sendHeader("Location", ok ? "/settings?saved=1" : "/settings?saveError=1");
  server.send(303);
}

// ===================== Save Communication =====================
void handleSaveCommunication() {
  if (server.hasArg("baud")) {
    commBaudRate = server.arg("baud").toInt();
  }

  if (server.hasArg("parity")) {
    commParity = server.arg("parity");
  }

  if (server.hasArg("stop")) {
    commStopBits = server.arg("stop").toInt();
  }

  if (server.hasArg("pollInterval")) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    pollIntervalMs = server.arg("pollInterval").toInt();
    xSemaphoreGive(dataMutex);
  }

  if (server.hasArg("recoveryDelay")) {
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    slaveRecoveryDelayMs = server.arg("recoveryDelay").toInt();
    xSemaphoreGive(dataMutex);
  }

  if (commBaudRate < 300 || commBaudRate > 1000000) {
    commBaudRate = 9600;
  }

  if (commParity != "N" && commParity != "E" && commParity != "O") {
    commParity = "N";
  }

  if (commStopBits != 1 && commStopBits != 2) {
    commStopBits = 1;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  if (pollIntervalMs < MIN_POLL_INTERVAL_MS || pollIntervalMs > MAX_POLL_INTERVAL_MS) {
    pollIntervalMs = DEFAULT_POLL_INTERVAL_MS;
  }
  if (slaveRecoveryDelayMs > MAX_SLAVE_RECOVERY_DELAY_MS) {
    slaveRecoveryDelayMs = DEFAULT_SLAVE_RECOVERY_DELAY_MS;
  }
  xSemaphoreGive(dataMutex);

  bool ok = saveCommunicationSettings();

  // serialMutex blocks until modbusTask finishes any in-flight RS485
  // transaction, so we never reconfigure Serial1 out from under it.
  xSemaphoreTake(serialMutex, portMAX_DELAY);

  Serial1.end();
  delay(200);

  Serial1.begin(
    commBaudRate,
    getSerialConfig(),
    RXD2,
    TXD2);

  xSemaphoreGive(serialMutex);

  logKeyEvent("COMM SETTINGS SAVED: " + String(commBaudRate) + " baud, poll " + String(pollIntervalMs) + "ms, recovery " + String(slaveRecoveryDelayMs) + "ms");

  server.sendHeader("Location", ok ? "/settings?commSaved=1" : "/settings?commSaveError=1");
  server.send(303);
}

// ===================== Save TCP Uplink =====================
void handleSaveUplink() {
  logMessage("========== WEB SAVE TCP REQUEST ==========");

  if (server.hasArg("ssid")) {
    uplinkConfig.ssid = server.arg("ssid");
  }

  if (server.hasArg("password") && server.arg("password").length() > 0) {
    uplinkConfig.password = server.arg("password");
  }

  if (server.hasArg("ip")) {
    uplinkConfig.serverIP = server.arg("ip");
  }

  if (server.hasArg("port")) {
    uplinkConfig.port = server.arg("port").toInt();
  }

  if (server.hasArg("ntpServer")) {
    String ntp = server.arg("ntpServer");
    ntp.trim();
    if (ntp.length() >= NTP_SERVER_MAX_LEN) {
      ntp = ntp.substring(0, NTP_SERVER_MAX_LEN - 1);
    }
    uplinkConfig.ntpServer = ntp;
  }

  if (uplinkConfig.port <= 0 || uplinkConfig.port > 65535) {
    uplinkConfig.port = 5000;
  }

  bool ok = saveUplinkConfig();

  WiFi.disconnect();
  delay(200);

  connectUplinkWiFi();

  // Re-arm SNTP so a changed/cleared on-premise server takes effect
  // immediately, not just after the next reboot.
  startNtp();

  logKeyEvent("NETWORK CONFIG SAVED: ssid=" + uplinkConfig.ssid + " tcp=" + uplinkConfig.serverIP + ":" + String(uplinkConfig.port)
              + (uplinkConfig.ntpServer.length() > 0 ? (" ntp=" + uplinkConfig.ntpServer) : ""));

  server.sendHeader("Location", ok ? "/settings?uplinkSaved=1" : "/settings?uplinkSaveError=1");
  server.send(303);
}

// ===================== Save Device / AP Identity =====================
void handleSaveDevice() {
  // Only present in the POST when the AP name field isn't disabled by the
  // "match" checkbox client-side, so a checked box naturally leaves the
  // stored apSsid untouched for whenever the user unchecks it later.
  if (server.hasArg("apSsid")) {
    deviceConfig.apSsid = server.arg("apSsid");
  }

  if (deviceConfig.apSsid.length() == 0) {
    deviceConfig.apSsid = DEFAULT_AP_SSID;
  }

  if (server.hasArg("devicePrefix")) {
    String prefix = server.arg("devicePrefix");
    if (prefix.length() > DEVICE_NAME_PREFIX_MAX_LEN) {
      prefix = prefix.substring(0, DEVICE_NAME_PREFIX_MAX_LEN);
    }
    deviceConfig.deviceNamePrefix = prefix;
  }

  deviceConfig.apMatchesDeviceName = server.hasArg("apMatchDevice");

  bool ok = saveDeviceConfig();

  String apSsidToUse = getEffectiveApSsid();
  WiFi.softAP(apSsidToUse.c_str(), AP_PASSWORD);

  logKeyEvent("DEVICE SETTINGS SAVED: AP=" + apSsidToUse + " deviceName=" + getDeviceName());

  server.sendHeader("Location", ok ? "/settings?deviceSaved=1" : "/settings?deviceSaveError=1");
  server.send(303);
}

// ===================== Save Modbus TCP Targets =====================
void handleSaveTcpTargets() {
  int newCount = 0;

  for (int i = 0; i < MAX_TCP_TARGETS; i++) {
    String index = String(i);
    String name = server.hasArg("tgtName" + index) ? server.arg("tgtName" + index) : "";
    String ip = server.hasArg("tgtIp" + index) ? server.arg("tgtIp" + index) : "";

    name.trim();
    ip.trim();

    // A slot with no name/IP is simply unused - only compacted slots
    // (0..newCount-1) are kept, so gaps in the form don't create holes.
    if (name.length() == 0 || ip.length() == 0) {
      continue;
    }

    int port = server.hasArg("tgtPort" + index) ? server.arg("tgtPort" + index).toInt() : 502;
    int unitId = server.hasArg("tgtUnit" + index) ? server.arg("tgtUnit" + index).toInt() : 1;

    if (port <= 0 || port > 65535) {
      port = 502;
    }

    if (unitId < 0 || unitId > 255) {
      unitId = 1;
    }

    tcpTargets[newCount].name = name;
    tcpTargets[newCount].ip = ip;
    tcpTargets[newCount].port = (uint16_t)port;
    tcpTargets[newCount].unitId = (uint8_t)unitId;
    newCount++;
  }

  xSemaphoreTake(dataMutex, portMAX_DELAY);
  tcpTargetCount = newCount;
  xSemaphoreGive(dataMutex);

  bool ok = saveTcpTargets();

  // Force reconnects so any changed IP/port/unit ID for an existing slot
  // index takes effect immediately rather than on next reboot.
  for (int i = 0; i < MAX_TCP_TARGETS; i++) {
    tcpTargetClients[i].stop();
    tcpTargetHealth[i].consecTimeouts = 0;
    tcpTargetHealth[i].backoffUntil = 0;
  }

  logKeyEvent("MODBUS TCP TARGETS SAVED: " + String(tcpTargetCount) + " configured");

  server.sendHeader("Location", ok ? "/settings?tcpTargetsSaved=1" : "/settings?tcpTargetsSaveError=1");
  server.send(303);
}

// ===================== Save Digital I/O =====================
void handleSaveDigitalIO() {
  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    String index = String(i);
    diChannels[i].name = server.arg("diName" + index);
    if (diChannels[i].name.length() == 0) {
      diChannels[i].name = "DI" + String(i + 1);
    }
    diChannels[i].enabled = server.hasArg("diEn" + index);
  }

  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    String index = String(i);
    doChannels[i].name = server.arg("doName" + index);
    if (doChannels[i].name.length() == 0) {
      doChannels[i].name = "DO" + String(i + 1);
    }
    doChannels[i].enabled = server.hasArg("doEn" + index);
  }

  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    String index = String(i);
    aiChannels[i].name = server.arg("aiName" + index);
    if (aiChannels[i].name.length() == 0) {
      aiChannels[i].name = "AI" + String(i + 1);
    }
    aiChannels[i].enabled = server.hasArg("aiEn" + index);

    float scale = server.arg("aiScale" + index).toFloat();
    aiChannels[i].scale = (scale == 0) ? 1 : scale;
    aiChannels[i].offset = server.arg("aiOffset" + index).toFloat();
  }

  xSemaphoreGive(dataMutex);

  bool ok = saveDigitalIOSettings();

  logKeyEvent("DIGITAL I/O SETTINGS SAVED");

  server.sendHeader("Location", ok ? "/settings?digioSaved=1" : "/settings?digioSaveError=1");
  server.send(303);
}

// ===================== Save Cellular Modem =====================
void handleSaveCellular() {
  cellularConfig.apn = server.arg("apn");
  cellularConfig.apn.trim();

  if (server.hasArg("pubIntervalS")) {
    long secs = server.arg("pubIntervalS").toInt();
    uint32_t ms = (secs > 0) ? (uint32_t)secs * 1000UL : CELLULAR_PUBLISH_INTERVAL_DEFAULT_MS;
    if (ms < CELLULAR_PUBLISH_INTERVAL_MIN_MS) ms = CELLULAR_PUBLISH_INTERVAL_MIN_MS;
    if (ms > CELLULAR_PUBLISH_INTERVAL_MAX_MS) ms = CELLULAR_PUBLISH_INTERVAL_MAX_MS;
    cellularConfig.publishIntervalMs = ms;
  }

  bool ok = saveCellularConfig();

  logKeyEvent("CELLULAR MODEM SETTINGS SAVED: apn=" + (cellularConfig.apn.length() > 0 ? cellularConfig.apn : String("(blank/auto)"))
              + ", publish every " + String(cellularConfig.publishIntervalMs / 1000) + "s");

  server.sendHeader("Location", ok ? "/settings?cellularSaved=1" : "/settings?cellularSaveError=1");
  server.send(303);
}

// ===================== Save Cloud Uplink =====================
// Minimal sanity check that content is PEM text, not an accidentally
// selected binary (DER/.p12) or wrong file - protects the stored certs.
bool looksLikePem(const String &content) {
  return content.indexOf("-----BEGIN") >= 0;
}

// Upload callback for /saveCloud (multipart). Streams each cert file's
// chunks into the matching accumulator; handleSaveCloud() consumes them
// once the whole request is parsed.
void handleCertUpload() {
  HTTPUpload &upload = server.upload();

  String *target = nullptr;
  if (upload.name == "caFile") {
    target = &uploadCaCert;
  } else if (upload.name == "devFile") {
    target = &uploadDevCert;
  } else if (upload.name == "keyFile") {
    target = &uploadPrivKey;
  }

  if (target == nullptr) {
    return;
  }

  if (upload.status == UPLOAD_FILE_START) {
    *target = "";
  } else if (upload.status == UPLOAD_FILE_WRITE) {
    if (target->length() + upload.currentSize <= CERT_UPLOAD_MAX_LEN) {
      target->concat((const char *)upload.buf, upload.currentSize);
    } else {
      // Oversized for any real PEM - discard so it can't half-apply.
      *target = "";
    }
  }
}

// Applies one cert slot: uploaded file wins over pasted text, blank keeps
// the stored value, and non-PEM content is rejected with a key-log entry.
bool applyCertUpdate(String &stored, String &uploaded, const char *pasteArg, const char *label) {
  String incoming;

  if (uploaded.length() > 0) {
    incoming = uploaded;
    uploaded = "";  // free the accumulator either way
  } else if (server.hasArg(pasteArg) && server.arg(pasteArg).length() > 0) {
    incoming = server.arg(pasteArg);
  } else {
    return false;
  }

  if (!looksLikePem(incoming)) {
    logKeyEvent("CERT REJECTED (" + String(label) + "): not PEM text - upload the .pem file, not DER/binary");
    return false;
  }

  stored = incoming;
  return true;
}

void handleSaveCloud() {
  if (server.hasArg("uplinkMode")) {
    cloudConfig.mode = (server.arg("uplinkMode") == "mqtt") ? UPLINK_MODE_MQTT : UPLINK_MODE_TCP;
  }

  if (server.hasArg("mqttEndpoint")) {
    cloudConfig.endpoint = server.arg("mqttEndpoint");
    cloudConfig.endpoint.trim();
  }

  if (server.hasArg("mqttPort")) {
    cloudConfig.port = server.arg("mqttPort").toInt();
  }

  if (cloudConfig.port <= 0 || cloudConfig.port > 65535) {
    cloudConfig.port = DEFAULT_MQTT_PORT;
  }

  if (server.hasArg("mqttClientId")) {
    cloudConfig.clientId = server.arg("mqttClientId");
    cloudConfig.clientId.trim();
  }

  if (server.hasArg("mqttTopic")) {
    cloudConfig.topic = server.arg("mqttTopic");
    cloudConfig.topic.trim();
  }

  if (cloudConfig.topic.length() == 0) {
    cloudConfig.topic = DEFAULT_MQTT_TOPIC;
  }

  cloudConfig.commandsEnabled = server.hasArg("cmdEnabled");

  bool ok = saveCloudConfig();

  // Per slot: uploaded file > pasted text > blank keeps stored value (same
  // pattern as the WiFi password), so re-saving other cloud settings never
  // wipes the certs.
  bool certsChanged = false;
  certsChanged |= applyCertUpdate(certRootCA, uploadCaCert, "caCert", "Root CA");
  certsChanged |= applyCertUpdate(certDevice, uploadDevCert, "devCert", "Device Certificate");
  certsChanged |= applyCertUpdate(certPrivKey, uploadPrivKey, "privKey", "Private Key");

  if (certsChanged) {
    ok = saveCerts() && ok;
  }

  // Drop any live session so the next cycle reconnects with the new
  // endpoint/certs/mode. Safe here: this handler and the MQTT client both
  // run on webTask, so nothing is mid-publish while we do this.
  if (mqttClient.connected()) {
    mqttClient.disconnect();
  }
  tlsClient.stop();
  lastMqttAttempt = 0;  // let the next cycle try immediately, skip backoff

  logKeyEvent("CLOUD CONFIG SAVED: mode=" + String(cloudConfig.mode == UPLINK_MODE_MQTT ? "mqtt" : "tcp")
              + (cloudConfig.endpoint.length() > 0 ? (" endpoint=" + cloudConfig.endpoint) : "")
              + (certsChanged ? " (certs updated)" : ""));

  server.sendHeader("Location", ok ? "/settings?cloudSaved=1" : "/settings?cloudSaveError=1");
  server.send(303);
}

// ===================== Save Types =====================
void handleSaveTypes() {
  int rows = server.arg("typeRowCount").toInt();

  if (rows < 0) rows = 0;
  if (rows > MAX_PARAM_TYPES) rows = MAX_PARAM_TYPES;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  typeCount = rows;

  for (int i = 0; i < typeCount; i++) {
    String index = String(i);

    typeList[i].name = server.arg("tname" + index);
    typeList[i].divisor = server.arg("div" + index).toFloat();

    if (typeList[i].name.length() == 0) {
      typeList[i].name = "INT_16/100";
    }

    if (typeList[i].divisor <= 0) {
      typeList[i].divisor = 1;
    }

    typeList[i].baseFormat = getBaseFormatFromType(typeList[i].name);
    typeList[i].dataLength = inferDataLengthFromType(typeList[i].name);
  }

  xSemaphoreGive(dataMutex);

  bool ok = saveTypes();

  logKeyEvent("DATA TYPES SAVED: " + String(rows) + " types");

  server.sendHeader("Location", ok ? "/types?typeSaved=1" : "/types?typeSaveError=1");
  server.send(303);
}

// ===================== JSON Data Endpoint =====================
void handleData() {
  String json = "{";
  json.reserve(96 + paramCount * 160);  // avoid repeated reallocation while appending below
  json += "\"device\":\"ESP32 Modbus RTU Gateway\",";
  json += "\"nowMs\":" + String(millis()) + ",";

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  json += "\"paramCount\":" + String(paramCount) + ",";
  json += "\"data\":[";

  for (int i = 0; i < paramCount; i++) {
    if (i > 0) {
      json += ",";
    }

    json += "{";
    json += "\"name\":\"" + jsonEscape(params[i].name) + "\",";
    json += "\"type\":\"" + jsonEscape(params[i].type) + "\",";
    json += "\"area\":\"" + String(AREA_NAMES[params[i].area <= AREA_DISCRETE_INPUT ? params[i].area : 0]) + "\",";

    bool rowIsTcp = (params[i].transport == TRANSPORT_TCP);
    json += "\"transport\":\"" + String(rowIsTcp ? "TCP" : "RTU") + "\",";
    json += "\"source\":\"" + (rowIsTcp && params[i].tcpTargetIndex < tcpTargetCount ? jsonEscape(tcpTargets[params[i].tcpTargetIndex].name) : String("RS485")) + "\",";

    json += "\"slaveId\":" + String(params[i].slaveId) + ",";
    json += "\"registerAddress\":" + String(params[i].registerAddress) + ",";
    json += "\"registerLength\":" + String(params[i].registerLength) + ",";
    json += "\"enabled\":" + String(params[i].enabled ? "true" : "false") + ",";
    json += "\"valid\":" + String(params[i].valid ? "true" : "false") + ",";
    json += "\"errorCode\":" + String(params[i].errorCode) + ",";
    json += "\"lastUpdateMs\":" + String(params[i].lastUpdateTime) + ",";

    json += "\"writable\":" + String(params[i].writable ? "true" : "false") + ",";
    json += "\"minWriteValue\":" + String(params[i].minWriteValue, 3) + ",";
    json += "\"maxWriteValue\":" + String(params[i].maxWriteValue, 3) + ",";
    json += "\"lastWriteResult\":" + String(params[i].lastWriteResult) + ",";
    json += "\"lastWriteTime\":" + String(params[i].lastWriteTime) + ",";

    if (params[i].valid) {
      json += "\"value\":" + String(params[i].value, 3);
    } else {
      json += "\"value\":null";
    }

    json += "}";
  }

  json += "],";

  json += "\"digitalIn\":[";
  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + jsonEscape(diChannels[i].name) + "\",";
    json += "\"pin\":\"" + String(DI_PIN_LABELS[i]) + "\",";
    json += "\"enabled\":" + String(diChannels[i].enabled ? "true" : "false") + ",";
    json += "\"value\":" + String(diChannels[i].value ? "true" : "false") + ",";
    json += "\"lastUpdateMs\":" + String(diChannels[i].lastUpdateTime) + "}";
  }
  json += "],";

  json += "\"digitalOut\":[";
  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + jsonEscape(doChannels[i].name) + "\",";
    json += "\"pin\":\"" + String(DO_PIN_LABELS[i]) + "\",";
    json += "\"enabled\":" + String(doChannels[i].enabled ? "true" : "false") + ",";
    json += "\"value\":" + String(doChannels[i].value ? "true" : "false") + ",";
    json += "\"lastWriteMs\":" + String(doChannels[i].lastWriteTime) + "}";
  }
  json += "],";

  json += "\"analogIn\":[";
  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    if (i > 0) json += ",";
    json += "{\"name\":\"" + jsonEscape(aiChannels[i].name) + "\",";
    json += "\"pin\":\"" + String(AI_PIN_LABELS[i]) + "\",";
    json += "\"enabled\":" + String(aiChannels[i].enabled ? "true" : "false") + ",";
    json += "\"value\":" + String(aiChannels[i].value, 3) + ",";
    json += "\"raw\":" + String(aiChannels[i].rawValue) + ",";
    json += "\"lastUpdateMs\":" + String(aiChannels[i].lastUpdateTime) + "}";
  }
  json += "]";

  xSemaphoreGive(dataMutex);

  json += "}";

  server.send(200, "application/json", json);
}

// ===================== Key Event Log Endpoint =====================
void handleKeyLog() {
  String json = "[";

  xSemaphoreTake(logMutex, portMAX_DELAY);

  int count = keyLogCount;

  for (int i = 0; i < count; i++) {
    int idx = (keyLogHead - count + i + KEY_LOG_SIZE) % KEY_LOG_SIZE;

    if (i > 0) {
      json += ",";
    }

    json += "\"" + jsonEscape(keyLog[idx]) + "\"";
  }

  xSemaphoreGive(logMutex);

  json += "]";

  server.send(200, "application/json", json);
}

// ===================== Manual Write Command Endpoint =====================
// Dashboard-driven write - same validated path as the automatic command
// channels (MQTT command topic / raw TCP).
void handleWriteCommand() {
  int paramIndex = server.arg("param").toInt();
  float value = server.arg("value").toFloat();

  bool ok = submitWriteCommand(paramIndex, value, WRITE_SOURCE_DASHBOARD);

  server.send(ok ? 200 : 400, "text/plain", ok ? "queued" : "rejected");
}

// Dashboard-driven Digital Out write - same role as handleWriteCommand()
// above, for the local I/O channels instead of Modbus params.
void handleWriteDigitalOut() {
  int channel = server.arg("channel").toInt();
  bool value = server.arg("value").toInt() != 0;

  bool ok = submitDoWriteCommand(channel, value, WRITE_SOURCE_DASHBOARD);

  server.send(ok ? 200 : 400, "text/plain", ok ? "queued" : "rejected");
}

// ===================== Reset =====================
void handleReset() {
  loadDefaultSettings();
  saveSettings();

  logKeyEvent("MODBUS SETTINGS RESET - all records deleted");

  server.sendHeader("Location", "/settings?saved=1");
  server.send(303);
}

void handleResetTypes() {
  loadDefaultTypes();
  saveTypes();

  logKeyEvent("DATA TYPES RESET to defaults");

  server.sendHeader("Location", "/types?typeReset=1");
  server.send(303);
}

// ===================== Setup =====================
void setup() {
  dataMutex = xSemaphoreCreateMutex();
  logMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();
  cellularMutex = xSemaphoreCreateMutex();
  pollCompleteSem = xSemaphoreCreateBinary();
  tcpSendDoneSem = xSemaphoreCreateBinary();
  logQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogQueueItem));
  rtuWriteQueue = xQueueCreate(WRITE_QUEUE_LEN, sizeof(WriteCommand));
  tcpWriteQueue = xQueueCreate(WRITE_QUEUE_LEN, sizeof(WriteCommand));
  mqttAckQueue = xQueueCreate(MQTT_ACK_QUEUE_LEN, sizeof(MqttAckItem));
  cellularAckTxQueue = xQueueCreate(CELLULAR_ACK_TX_QUEUE_LEN, sizeof(CellularAckItem));
  tcpAckQueue = xQueueCreate(TCP_ACK_QUEUE_LEN, sizeof(TcpAckItem));
  doWriteQueue = xQueueCreate(DO_QUEUE_LEN, sizeof(DoWriteCommand));

  mqttClient.setCallback(mqttCallback);

  Serial.begin(115200);
  delay(1000);

  // Started before the first logKeyEvent() call below so no boot messages
  // are lost - they'd otherwise just sit in logQueue until this runs.
  xTaskCreatePinnedToCore(
    logTask,
    "LogTask",
    LOG_TASK_STACK,
    NULL,
    LOG_TASK_PRIORITY,
    &logTaskHandle,
    LOG_TASK_CORE);

  logKeyEvent("BOOT");

  pinMode(RS485_DE_RE, OUTPUT);
  digitalWrite(RS485_DE_RE, LOW);

  // Digital In: INPUT_PULLUP is the common default for opto-isolator input
  // modules with an open-collector output stage - change to plain INPUT if
  // your specific module already drives a clean logic level (has its own
  // pull-up/pull-down) rather than relying on the ESP32's internal one.
  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    pinMode(DI_PINS[i], INPUT_PULLUP);
  }

  // Digital Out: driven LOW (off) at boot - the safe default for a
  // relay/driver module before any configured state is loaded/commanded.
  for (int i = 0; i < DO_CHANNEL_COUNT; i++) {
    pinMode(DO_PINS[i], OUTPUT);
    digitalWrite(DO_PINS[i], LOW);
  }

  analogReadResolution(12);  // explicit: full 0-4095 range, don't rely on core default

  loadCommunicationSettings();
  loadUplinkConfig();
  loadDeviceConfig();
  loadCloudConfig();
  loadCerts();
  loadCellularConfig();

  Serial1.begin(
    commBaudRate,
    getSerialConfig(),
    RXD2,
    TXD2);

  loadTypes();
  loadSettings();
  loadTcpTargets();
  loadDigitalIOSettings();

  WiFi.mode(WIFI_AP_STA);

  String apSsidToUse = getEffectiveApSsid();
  WiFi.softAP(apSsidToUse.c_str(), AP_PASSWORD);

  connectUplinkWiFi();

  // Best-effort background SNTP (UTC, no TZ offset - cloud data should be
  // UTC). Non-blocking: syncs whenever a configured server is reachable,
  // and getTimestamp() keeps using uptime seconds until then.
  startNtp();

  logKeyEvent("AP STARTED: " + apSsidToUse + " " + WiFi.softAPIP().toString());
  logMessage("RS485 RX=D2 TX=D3 DE/RE=D4");
  logMessage("TCP TARGET: " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));

  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/saveCommunication", HTTP_POST, handleSaveCommunication);
  server.on("/saveUplink", HTTP_POST, handleSaveUplink);
  server.on("/saveDevice", HTTP_POST, handleSaveDevice);
  // Second callback handles the multipart cert file uploads, streamed in
  // before handleSaveCloud runs.
  server.on("/saveCloud", HTTP_POST, handleSaveCloud, handleCertUpload);
  server.on("/data", HTTP_GET, handleData);
  server.on("/writeCommand", HTTP_POST, handleWriteCommand);
  server.on("/writeDigitalOut", HTTP_POST, handleWriteDigitalOut);
  server.on("/keylog", HTTP_GET, handleKeyLog);
  server.on("/types", HTTP_GET, handleTypes);
  server.on("/saveTypes", HTTP_POST, handleSaveTypes);
  server.on("/saveTcpTargets", HTTP_POST, handleSaveTcpTargets);
  server.on("/saveDigitalIO", HTTP_POST, handleSaveDigitalIO);
  server.on("/saveCellular", HTTP_POST, handleSaveCellular);
  server.on("/reset", HTTP_GET, handleReset);
  server.on("/resetTypes", HTTP_GET, handleResetTypes);

  server.begin();

  logKeyEvent("WEB SERVER STARTED");

  xTaskCreatePinnedToCore(
    modbusTask,
    "ModbusTask",
    MODBUS_TASK_STACK,
    NULL,
    MODBUS_TASK_PRIORITY,
    &modbusTaskHandle,
    MODBUS_TASK_CORE);

  logKeyEvent("MODBUS POLLING STARTED (core " + String(MODBUS_TASK_CORE) + ")");

  // Web server / TCP push / WiFi retry, pinned to Core 0 so Core 1 stays dedicated to modbusTask.
  xTaskCreatePinnedToCore(
    webTask,
    "WebTask",
    WEB_TASK_STACK,
    NULL,
    WEB_TASK_PRIORITY,
    &webTaskHandle,
    WEB_TASK_CORE);

  logKeyEvent("WEB/TCP TASK STARTED (core " + String(WEB_TASK_CORE) + ")");

  // Modbus TCP client poller (e.g. a CNC controller) - Core 0, independent
  // of modbusTask/Core 1's real-time RS485 timing.
  xTaskCreatePinnedToCore(
    tcpPollTask,
    "TcpPollTask",
    TCP_POLL_TASK_STACK,
    NULL,
    TCP_POLL_TASK_PRIORITY,
    &tcpPollTaskHandle,
    TCP_POLL_TASK_CORE);

  logKeyEvent("MODBUS TCP POLLING STARTED (core " + String(TCP_POLL_TASK_CORE) + ")");

  // Cellular modem status monitoring - own task since modem AT commands
  // can block for seconds at a time (see cellularTask()'s comment).
  xTaskCreatePinnedToCore(
    cellularTask,
    "CellularTask",
    CELLULAR_TASK_STACK,
    NULL,
    CELLULAR_TASK_PRIORITY,
    &cellularTaskHandle,
    CELLULAR_TASK_CORE);

  logKeyEvent("CELLULAR MODEM TASK STARTED (core " + String(CELLULAR_TASK_CORE) + ")");
}

// ===================== Summary Debug =====================
void printModbusSummary() {
#if DEBUG_ENABLED && DEBUG_SUMMARY_LOG
  if (millis() - lastSummaryPrintTime < DEBUG_SUMMARY_INTERVAL_MS) {
    return;
  }

  lastSummaryPrintTime = millis();

  int enabledCount = 0;
  int okCount = 0;
  int errorCount = 0;
  int disabledCount = 0;
  int total;

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  total = paramCount;

  for (int i = 0; i < paramCount; i++) {
    if (!params[i].enabled) {
      disabledCount++;
    } else {
      enabledCount++;

      if (params[i].valid) {
        okCount++;
      } else {
        errorCount++;
      }
    }
  }

  xSemaphoreGive(dataMutex);

  Serial.print("[SUMMARY] Total=");
  Serial.print(total);
  Serial.print(" | Enabled=");
  Serial.print(enabledCount);
  Serial.print(" | OK=");
  Serial.print(okCount);
  Serial.print(" | Error=");
  Serial.print(errorCount);
  Serial.print(" | Disabled=");
  Serial.println(disabledCount);

  // Rate-limited heartbeat for the field log, not per-parameter spam.
  logKeyEvent("POLL: " + String(okCount) + "/" + String(enabledCount) + " OK" + (errorCount > 0 ? (", " + String(errorCount) + " ERR") : ""));
#endif
}

// ===================== Modbus Task (Core 1 - dedicated) =====================
// Paces itself to pollIntervalMs. Each cycle is poll -> TCP send -> next
// poll: pollModbus() signals pollCompleteSem, then this task waits (up to
// TCP_SEND_TIMEOUT_MS) on tcpSendDoneSem so a hung TCP link can never
// stall real-time polling.
void modbusTask(void *parameter) {
  for (;;) {
    unsigned long cycleStart = millis();

    pollModbus();
    processRtuWriteQueue();

    // Discard any stale signal from a previous cycle's late TCP feedback.
    xSemaphoreTake(tcpSendDoneSem, 0);

    if (xSemaphoreTake(tcpSendDoneSem, pdMS_TO_TICKS(TCP_SEND_TIMEOUT_MS)) != pdTRUE) {
      logKeyEvent("TCP FEEDBACK TIMEOUT (" + String(TCP_SEND_TIMEOUT_MS) + "ms) - continuing to next poll cycle");
    }

    printModbusSummary();

    uint32_t interval;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    interval = pollIntervalMs;
    xSemaphoreGive(dataMutex);

    unsigned long elapsed = millis() - cycleStart;

    if (elapsed < interval) {
      vTaskDelay(pdMS_TO_TICKS(interval - elapsed));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ===================== Modbus TCP Poll Task (Core 0) =====================
// Polls TRANSPORT_TCP parameters (e.g. a CNC controller) over WiFi,
// completely independent of modbusTask/Core 1 - a TCP connect/read here
// can block for seconds without ever affecting RS485 timing. Paced to the
// same pollIntervalMs as the RTU poller for predictable behavior, but runs
// as its own cycle; results land in the same params[]/dataMutex the RTU
// poller and cloud push already share.
void tcpPollTask(void *parameter) {
  for (;;) {
    unsigned long cycleStart = millis();

    pollTcpTargets();
    processTcpWriteQueue();

    uint32_t interval;
    xSemaphoreTake(dataMutex, portMAX_DELAY);
    interval = pollIntervalMs;
    xSemaphoreGive(dataMutex);

    unsigned long elapsed = millis() - cycleStart;

    if (elapsed < interval) {
      vTaskDelay(pdMS_TO_TICKS(interval - elapsed));
    } else {
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

// ===================== Log Task (Core 0 / WiFi core) =====================
// Drains logQueue and does the actual Serial.println()/String work off Core 1.
void logTask(void *parameter) {
  LogQueueItem item;

  for (;;) {
    if (xQueueReceive(logQueue, &item, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    String entry = "[" + String(item.timestampSec) + "] " + String(item.text);

    Serial.println(entry);

    xSemaphoreTake(logMutex, portMAX_DELAY);

    debugLogs += entry + "<br>";

    if (debugLogs.length() > 4000) {
      debugLogs = debugLogs.substring(debugLogs.length() - 3000);
    }

    if (item.isKeyEvent) {
      String keyEntry = "[" + String(item.timestampSec) + "s] " + String(item.text);

      keyLog[keyLogHead] = keyEntry;
      keyLogHead = (keyLogHead + 1) % KEY_LOG_SIZE;

      if (keyLogCount < KEY_LOG_SIZE) {
        keyLogCount++;
      }
    }

    xSemaphoreGive(logMutex);
  }
}

// Publishes a completion ack for each finished WRITE_SOURCE_MQTT command -
// see the MqttAckItem comment near mqttAckQueue. WiFi-side drain (webTask/
// mqttClient); during a cellular failover the items are deliberately left
// in the queue for cellularTask's own drain (drainMqttAckQueueCellular())
// to publish over the modem instead.
void drainMqttAckQueue() {
  if (mqttFailoverActive) {
    return;
  }

  MqttAckItem item;

  while (xQueueReceive(mqttAckQueue, &item, 0) == pdTRUE) {
    if (!cloudConfig.commandsEnabled || !mqttClient.connected()) {
      continue;  // commands turned off / uplink dropped since this was queued
    }

    publishMqttAck(String(item.name), item.value, item.result == MB_SUCCESS, item.result, "");
  }
}

// Sends a completion ack for each finished WRITE_SOURCE_TCP command - see
// the TcpAckItem comment near tcpAckQueue. Only webTask touches tcpClient,
// so this is the only place that drains this queue.
void drainTcpAckQueue() {
  TcpAckItem item;

  while (xQueueReceive(tcpAckQueue, &item, 0) == pdTRUE) {
    sendTcpAckLine(String(item.name), item.value, item.result == MB_SUCCESS, item.result, "");
  }
}

// ===================== Free Heap Monitoring =====================
// Independent of DEBUG_ENABLED (that gates printModbusSummary()'s
// Serial-only poll-cycle stats) - this goes to the persistent Status Log
// so heap headroom is visible in the field, not just during a debug
// session. ESP.getMinFreeHeap() catches transient dips (e.g. a TLS
// handshake's temporary buffers) that a plain getFreeHeap() snapshot
// could land between and miss entirely.
unsigned long lastFreeHeapLogTime = 0;
uint32_t lastLoggedFreeHeap = 0;
#define FREE_HEAP_LOG_INTERVAL_MS 60000

void logFreeHeapPeriodic() {
  if (millis() - lastFreeHeapLogTime < FREE_HEAP_LOG_INTERVAL_MS) {
    return;
  }
  lastFreeHeapLogTime = millis();
  lastLoggedFreeHeap = ESP.getFreeHeap();

  logKeyEvent("FREE HEAP: " + String(lastLoggedFreeHeap) + " bytes (min ever since boot: " + String(ESP.getMinFreeHeap()) + ")");
}

// ===================== Local Digital I/O Polling =====================
// Plain GPIO reads/writes - no bus transaction, never blocks - so this runs
// on webTask (Core 0) on its own timer, fully decoupled from the Modbus
// poll cycle. Never touches modbusTask/Core 1.
unsigned long lastDigitalIoPollTime = 0;
#define DIGITAL_IO_POLL_INTERVAL_MS 200

void pollDigitalIO() {
  if (millis() - lastDigitalIoPollTime < DIGITAL_IO_POLL_INTERVAL_MS) {
    return;
  }
  lastDigitalIoPollTime = millis();

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < DI_CHANNEL_COUNT; i++) {
    if (!diChannels[i].enabled) {
      continue;
    }
    // Pins are INPUT_PULLUP (see setup()): idle/open reads HIGH, an
    // opto-isolator's open-collector output pulls it LOW when active.
    // Inverted here so "value" means "input asserted", not "pin
    // electrically low" - verify this matches your specific I/O module's
    // output polarity.
    diChannels[i].value = (digitalRead(DI_PINS[i]) == LOW);
    diChannels[i].lastUpdateTime = millis();
  }

  for (int i = 0; i < AI_CHANNEL_COUNT; i++) {
    if (!aiChannels[i].enabled) {
      continue;
    }
    int raw = analogRead(AI_PINS[i]);
    aiChannels[i].rawValue = raw;
    aiChannels[i].value = (float)raw * aiChannels[i].scale + aiChannels[i].offset;
    aiChannels[i].lastUpdateTime = millis();
  }

  xSemaphoreGive(dataMutex);
}

// ===================== Cellular Modem (SIM7600G-H) =====================
// Status monitoring only in this pass - proves the UART/AT link and power
// sequencing work before anything is built on top (TCP/MQTT-over-cellular,
// WiFi/cellular failover). Runs on its own dedicated task, NOT webTask,
// because modem.init()/testAT() etc. can block for several seconds each -
// stalling webTask that long would stall the web UI, MQTT/TCP uplink
// servicing, and Digital I/O polling right along with it.
#define MODEM_PWRKEY_PULSE_MS 1000
#define MODEM_BOOT_WAIT_MS 10000
// Cadence of the status-poll/housekeeping loop only - the telemetry
// publish during failover is paced separately by the user-configurable
// cellularConfig.publishIntervalMs (Settings -> Cellular Modem).
#define CELLULAR_POLL_INTERVAL_MS 5000

// PWRKEY pulse per the SIM7600 series' usual convention: briefly pull LOW
// to trigger power-on, then release. Idle level, pulse polarity, and pulse
// duration all vary a bit by board revision - verify against your specific
// SIM7600G-H board's documentation if the modem never responds.
void powerOnModem() {
  pinMode(MODEM_PWRKEY_PIN, OUTPUT);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY_PIN, LOW);
  delay(MODEM_PWRKEY_PULSE_MS);
  digitalWrite(MODEM_PWRKEY_PIN, HIGH);

  logMessage("CELLULAR MODEM POWER-ON PULSE SENT - waiting for boot");
  delay(MODEM_BOOT_WAIT_MS);
}

// ===================== Cellular Modem: Raw AT Command Helper =====================
// TinyGSM's modem.xxx() API is a portable abstraction covering registration/
// signal/GPRS - it doesn't (and isn't meant to) cover the SIM7600's own
// modem-specific AT+CMQTT*/AT+CSSLCFG/AT+CCERTDOWN command family used for
// certificate-backed MQTT run directly on the modem's onboard MQTT client
// (see the architecture note above). These talk to SerialAT directly.
// Safe to do so because cellularTask is the sole owner of that UART and
// never calls these concurrently with a modem.xxx() TinyGSM call - both
// only ever run sequentially within this one task.
#define AT_LINE_MAX_LEN 256
#define AT_DEFAULT_TIMEOUT_MS 5000

// ---- Inbound MQTT message URCs (+CMQTTRXSTART etc.) ----
// While the modem's MQTT client is connected and subscribed, an inbound
// message can surface on SerialAT at ANY time, as an unsolicited block:
//   +CMQTTRXSTART: <client>,<topic_total_len>,<payload_total_len>
//   +CMQTTRXTOPIC: <client>,<chunk_len>   then <chunk_len> raw topic bytes
//   +CMQTTRXPAYLOAD: <client>,<chunk_len> then <chunk_len> raw payload bytes
//   +CMQTTRXEND: <client>
// (TOPIC/PAYLOAD chunks repeat until their totals are reached.) The
// helpers below spot the RXSTART line wherever it appears - between
// commands (pumpCellularMqttUrcs()) or in the middle of waiting for some
// other command's response (waitForAtResponse()) - and synchronously
// consume the rest of the block before carrying on.
#define CELLULAR_RX_TOPIC_MAX_LEN 128
#define CELLULAR_RX_BLOCK_TIMEOUT_MS 10000

// Set when a +CMQTTCONNLOST / +CMQTTNONET URC is spotted; cellularTask
// consumes it at loop level to mark the session down and reconnect.
bool cellularMqttConnLost = false;

// Line assembly for the between-commands pump - persistent so a line that
// arrives split across pump calls isn't corrupted.
String cellularUrcLineBuf;

// Reads one complete non-empty line (blocking up to timeoutMs).
bool readAtLine(String &line, unsigned long timeoutMs) {
  line = "";
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = (char)SerialAT.read();
      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          return true;
        }
      } else if (c != '\r') {
        if (line.length() < AT_LINE_MAX_LEN) {
          line += c;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }

  return false;
}

// Reads exactly len raw bytes (blocking up to timeoutMs) - used for the
// topic/payload chunks, which are length-prefixed and may contain any
// byte, so line-based reading would be wrong.
bool readAtRawBytes(String &out, size_t len, unsigned long timeoutMs) {
  out = "";
  out.reserve(len);
  unsigned long start = millis();

  while (out.length() < len && millis() - start < timeoutMs) {
    while (SerialAT.available() && out.length() < len) {
      out += (char)SerialAT.read();
    }
    if (out.length() < len) {
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }

  return out.length() == len;
}

// Consumes one complete inbound-message block, starting right after its
// +CMQTTRXSTART line, and hands the payload to handleMqttCommand() if the
// topic matches the command topic. Oversized messages are drained (so the
// URC stream stays in sync) but dropped.
void consumeCellularInboundMessage(const String &startLine) {
  int c1 = startLine.indexOf(',');
  int c2 = startLine.lastIndexOf(',');
  if (c1 < 0 || c2 <= c1) {
    return;
  }

  long topicTotal = startLine.substring(c1 + 1, c2).toInt();
  long payloadTotal = startLine.substring(c2 + 1).toInt();
  bool oversized = (topicTotal > CELLULAR_RX_TOPIC_MAX_LEN || payloadTotal > MQTT_CMD_MAX_LEN);

  String topic = "";
  String payload = "";
  unsigned long start = millis();

  for (;;) {
    // Recomputed before every blocking read - the subtraction must never
    // underflow (unsigned), or a stalled modem could wedge this task for
    // ~49 days instead of the intended block timeout.
    unsigned long elapsed = millis() - start;
    if (elapsed >= CELLULAR_RX_BLOCK_TIMEOUT_MS) {
      break;
    }
    unsigned long remaining = CELLULAR_RX_BLOCK_TIMEOUT_MS - elapsed;

    String line;
    if (!readAtLine(line, remaining)) {
      break;
    }

    if (line.startsWith("+CMQTTRXTOPIC:") || line.startsWith("+CMQTTRXPAYLOAD:")) {
      long chunkLen = line.substring(line.lastIndexOf(',') + 1).toInt();
      if (chunkLen < 0 || chunkLen > MQTT_CMD_MAX_LEN) {
        break;  // implausible - abandon the block rather than block on it
      }

      elapsed = millis() - start;
      if (elapsed >= CELLULAR_RX_BLOCK_TIMEOUT_MS) {
        break;
      }

      String chunk;
      if (!readAtRawBytes(chunk, (size_t)chunkLen, CELLULAR_RX_BLOCK_TIMEOUT_MS - elapsed)) {
        break;
      }

      if (!oversized) {
        if (line.startsWith("+CMQTTRXTOPIC:")) {
          topic += chunk;
        } else {
          payload += chunk;
        }
      }
      continue;
    }

    if (line.startsWith("+CMQTTRXEND:")) {
      if (oversized) {
        logKeyEvent("CELLULAR MQTT CMD IGNORED: message too large (topic " + String(topicTotal) + ", payload " + String(payloadTotal) + " bytes)");
      } else if (topic == mqttCommandTopic()) {
        logMessage("CELLULAR MQTT CMD RECEIVED: " + payload);
        handleMqttCommand((const byte *)payload.c_str(), payload.length(), true);
      } else {
        logMessage("CELLULAR MQTT RX IGNORED (topic " + topic + ")");
      }
      return;
    }

    if (line.startsWith("+CMQTTCONNLOST:") || line.startsWith("+CMQTTNONET")) {
      cellularMqttConnLost = true;
      return;  // session is gone - the rest of the block won't arrive
    }

    // Any other line (a late OK, an unrelated URC) - skip and keep
    // scanning for the rest of the block.
  }

  logKeyEvent("CELLULAR MQTT RX ABORTED (incomplete message block)");
}

// Reacts to one complete URC line seen between commands.
void handleCellularUrcLine(const String &line) {
  if (line.startsWith("+CMQTTRXSTART:")) {
    consumeCellularInboundMessage(line);
  } else if (line.startsWith("+CMQTTCONNLOST:") || line.startsWith("+CMQTTNONET")) {
    cellularMqttConnLost = true;
  }
  // Anything else between commands is stale noise (a late OK etc.) -
  // dropped, exactly as the old blind pre-command flush did.
}

// Drains whatever is sitting in the UART RX buffer, processing any URCs
// found - called between AT commands and repeatedly while cellularTask
// sleeps, so an inbound command is handled within ~the pump interval
// instead of waiting for the next full status cycle.
void pumpCellularMqttUrcs() {
  while (SerialAT.available()) {
    char c = (char)SerialAT.read();

    if (c == '\n') {
      cellularUrcLineBuf.trim();
      if (cellularUrcLineBuf.length() > 0) {
        String line = cellularUrcLineBuf;
        cellularUrcLineBuf = "";
        handleCellularUrcLine(line);
      } else {
        cellularUrcLineBuf = "";
      }
    } else if (c != '\r') {
      if (cellularUrcLineBuf.length() < AT_LINE_MAX_LEN) {
        cellularUrcLineBuf += c;
      }
    }
  }
}

// Called before sending any AT command, replacing the old blind flush
// (`while available: read`), which would have silently discarded any
// inbound command sitting in the buffer. Processes pending URCs instead,
// and gives a partially-received line a brief moment to finish (at 115200
// baud a full line takes ~20ms) so the command's response parsing starts
// on a clean line boundary.
void syncCellularUartForCommand() {
  pumpCellularMqttUrcs();

  if (cellularUrcLineBuf.length() > 0) {
    unsigned long start = millis();
    while (cellularUrcLineBuf.length() > 0 && millis() - start < 250) {
      vTaskDelay(pdMS_TO_TICKS(5));
      pumpCellularMqttUrcs();
    }
    cellularUrcLineBuf = "";  // still unfinished - stale, drop it
  }
}

// Reads lines from SerialAT until one starts with expectedPrefix (success,
// optionally copied into resultLine) or is "ERROR"/"+CME ERROR"/"+CMS
// ERROR" (failure), or timeoutMs elapses with neither (failure). An
// inbound-message URC block surfacing mid-wait is consumed inline (see
// above) rather than discarded.
bool waitForAtResponse(const String &expectedPrefix, unsigned long timeoutMs, String *resultLine = nullptr) {
  unsigned long start = millis();
  String line;

  while (millis() - start < timeoutMs) {
    while (SerialAT.available()) {
      char c = (char)SerialAT.read();

      if (c == '\n') {
        line.trim();
        if (line.length() > 0) {
          if (line.startsWith(expectedPrefix)) {
            if (resultLine) *resultLine = line;
            return true;
          }
          if (line.startsWith("+CMQTTRXSTART:")) {
            consumeCellularInboundMessage(line);
            line = "";
            continue;
          }
          if (line.startsWith("+CMQTTCONNLOST:") || line.startsWith("+CMQTTNONET")) {
            cellularMqttConnLost = true;
            line = "";
            continue;
          }
          if (line == "ERROR" || line.startsWith("+CME ERROR") || line.startsWith("+CMS ERROR")) {
            if (resultLine) *resultLine = line;
            return false;
          }
        }
        line = "";
      } else if (c != '\r') {
        if (line.length() < AT_LINE_MAX_LEN) {
          line += c;
        }
      }
    }
    vTaskDelay(pdMS_TO_TICKS(5));
  }

  return false;
}

// Sends a plain "AT...\r\n" command and waits for its response line.
bool sendAtCommand(const String &cmd, const String &expectedPrefix, unsigned long timeoutMs, String *resultLine = nullptr) {
  syncCellularUartForCommand();

  SerialAT.print(cmd);
  SerialAT.print("\r\n");

  return waitForAtResponse(expectedPrefix, timeoutMs, resultLine);
}

// For commands that show a ">" prompt before accepting raw data
// (AT+CCERTDOWN, AT+CMQTTTOPIC, AT+CMQTTPAYLOAD, AT+CMQTTSUB, etc.):
// sends the command, waits for the ">" prompt, writes the raw payload
// bytes, then waits for the final response line (optionally captured in
// resultLine, for callers that need to parse a trailing error code).
bool sendAtCommandWithData(const String &cmd, const uint8_t *data, size_t len, const String &expectedPrefix, unsigned long timeoutMs, String *resultLine = nullptr) {
  syncCellularUartForCommand();

  SerialAT.print(cmd);
  SerialAT.print("\r\n");

  unsigned long start = millis();
  bool gotPrompt = false;

  while (millis() - start < timeoutMs) {
    if (SerialAT.available()) {
      if ((char)SerialAT.read() == '>') {
        gotPrompt = true;
        break;
      }
    } else {
      vTaskDelay(pdMS_TO_TICKS(5));
    }
  }

  if (!gotPrompt) {
    return false;
  }

  SerialAT.write(data, len);

  return waitForAtResponse(expectedPrefix, timeoutMs, resultLine);
}

// Parses the trailing <err> field of a "+CMQTTXXX: ...,<err>" result line
// (or the only field, e.g. "+CMQTTSTART: <err>"). 0 means success.
int atTrailingErrCode(const String &resultLine, const String &resultPrefix) {
  int lastComma = resultLine.lastIndexOf(',');
  return (lastComma >= 0) ? resultLine.substring(lastComma + 1).toInt() : resultLine.substring(resultPrefix.length()).toInt();
}

// Many AT+CMQTT* commands (CONNECT/PUB/SUB/UNSUB/DISC/START) eventually
// report their real result as an asynchronous "+CMQTTXXX: ...,<err>" line,
// which can arrive either before or after (and separate from) the "OK"
// that just means "command accepted" - the manual's own worked examples
// aren't fully consistent on the ordering (e.g. CMQTTSTART's response
// table lists both orders as valid). Waiting directly for the
// result-prefixed line - via waitForAtResponse(), which skips over any
// unrelated lines including "OK" while scanning - is robust to that
// ambiguity regardless of exactly when "OK" shows up. Parses the trailing
// <err> field (or the only field, for CMQTTSTART); true only if it's 0.
// outResultLine, if given, is filled with the raw response line (or a
// placeholder if none arrived) regardless of success/failure - lets callers
// log the modem's actual error code on failure instead of just "it failed".
bool sendMqttCommand(const String &cmd, const String &resultPrefix, unsigned long timeoutMs, String *outResultLine = nullptr) {
  syncCellularUartForCommand();

  SerialAT.print(cmd);
  SerialAT.print("\r\n");

  String resultLine;
  if (!waitForAtResponse(resultPrefix, timeoutMs, &resultLine)) {
    if (outResultLine) {
      *outResultLine = (resultLine.length() > 0) ? resultLine : ("(no " + resultPrefix + " response within " + String(timeoutMs) + "ms)");
    }
    return false;
  }

  if (outResultLine) *outResultLine = resultLine;

  return atTrailingErrCode(resultLine, resultPrefix) == 0;
}

// ===================== Cellular Modem: MQTT Certificate Provisioning =====================
// One-time (per boot) upload of the modem's own certificate store + SSL
// context for mutual-TLS MQTT (AWS IoT) - see the MQTT AT Command
// Manual's "Access to SSL/TLS MQTT server (verify server and client)"
// worked example (authmode 2 = verify both server and client). Reuses the
// same PEM Strings already held in RAM for the WiFi/WiFiClientSecure path
// (certRootCA/certDevice/certPrivKey) - no separate storage, no re-parsing.
#define CELLULAR_CA_CERT_FILENAME "ca_cert.pem"
#define CELLULAR_CLIENT_CERT_FILENAME "client_cert.pem"
#define CELLULAR_CLIENT_KEY_FILENAME "client_key.pem"
#define CELLULAR_SSL_CTX_INDEX 0

// Deletes any stale copy first (result ignored - it may simply not exist
// yet, e.g. on first boot) so a re-provisioning always reflects the
// current cert content, then uploads the fresh one.
bool uploadCertFile(const String &filename, const String &pemContent) {
  sendAtCommand("AT+CCERTDELE=\"" + filename + "\"", "OK", AT_DEFAULT_TIMEOUT_MS);

  String cmd = "AT+CCERTDOWN=\"" + filename + "\"," + String(pemContent.length());
  bool ok = sendAtCommandWithData(cmd, (const uint8_t *)pemContent.c_str(), pemContent.length(), "OK", 10000);

  if (!ok) {
    logKeyEvent("CELLULAR CERT UPLOAD FAILED: " + filename);
  }

  return ok;
}

bool provisionCellularMqttCerts() {
  logKeyEvent("CELLULAR MQTT CERT PROVISIONING START");

  if (!uploadCertFile(CELLULAR_CA_CERT_FILENAME, certRootCA)) return false;
  if (!uploadCertFile(CELLULAR_CLIENT_CERT_FILENAME, certDevice)) return false;
  if (!uploadCertFile(CELLULAR_CLIENT_KEY_FILENAME, certPrivKey)) return false;

  String ctx = String(CELLULAR_SSL_CTX_INDEX);

  if (!sendAtCommand("AT+CSSLCFG=\"sslversion\"," + ctx + ",4", "OK", AT_DEFAULT_TIMEOUT_MS)) return false;
  if (!sendAtCommand("AT+CSSLCFG=\"authmode\"," + ctx + ",2", "OK", AT_DEFAULT_TIMEOUT_MS)) return false;
  if (!sendAtCommand("AT+CSSLCFG=\"cacert\"," + ctx + ",\"" + CELLULAR_CA_CERT_FILENAME + "\"", "OK", AT_DEFAULT_TIMEOUT_MS)) return false;
  if (!sendAtCommand("AT+CSSLCFG=\"clientcert\"," + ctx + ",\"" + CELLULAR_CLIENT_CERT_FILENAME + "\"", "OK", AT_DEFAULT_TIMEOUT_MS)) return false;
  if (!sendAtCommand("AT+CSSLCFG=\"clientkey\"," + ctx + ",\"" + CELLULAR_CLIENT_KEY_FILENAME + "\"", "OK", AT_DEFAULT_TIMEOUT_MS)) return false;

  logKeyEvent("CELLULAR MQTT CERT PROVISIONING OK (SSL context " + ctx + ")");
  return true;
}

// ===================== Cellular Modem: MQTT Session + Commands + Telemetry =====================
// The modem's onboard MQTT client, run only while cellular failover is
// active (see mqttFailoverActive). Reuses the same cloudConfig.endpoint/
// port/topic, buildTcpJson() telemetry payload, and command/ack topics as
// the WiFi/PubSubClient path - same broker identity, same message shapes,
// different transport. Inbound commands arrive as +CMQTTRX* URC blocks
// (see the URC pump above); outbound acks are queued and published at
// loop level in cellularTask.
#define CELLULAR_MQTT_CLIENT_INDEX 0
#define CELLULAR_MQTT_START_TIMEOUT_MS 120000  // matches the manual's stated max response time for CMQTTSTART
#define CELLULAR_MQTT_CONNECT_TIMEOUT_MS 60000
#define CELLULAR_MQTT_PUB_TIMEOUT_MS 60000

// Must match connectMqtt()'s WiFi-side clientId choice exactly - if the two
// transports ever connected with different client IDs, an AWS IoT policy
// scoped to a specific clientId (a common pattern, e.g.
// iot:Connection.Thing.ThingName) would silently reject whichever transport
// used the "wrong" one, and the two would also never contend for the same
// AWS-side session, defeating the point of failover being seamless.
// AT+CMQTTACCQ's <clientID> must additionally be UTF-8, 1-23 bytes - the
// device name (or a configured client ID) could exceed that, so truncate
// defensively.
String cellularMqttClientId() {
  String id = (cloudConfig.clientId.length() > 0) ? cloudConfig.clientId : getDeviceName();
  if (id.length() > 23) {
    id = id.substring(0, 23);
  }
  return id;
}

// Step-by-step session bring-up, matching the manual's worked "verify
// server and client" example order: START -> ACCQ (server_type 1 =
// SSL/TLS) -> SSLCFG (bind the SSL context provisionCellularMqttCerts()
// configured) -> CONNECT. Each of started/clientAcquired/sslBound/
// connected is a persistent flag owned by the caller (cellularTask), so a
// retry after a partial failure only re-attempts whatever step didn't
// complete last time, instead of e.g. re-issuing CMQTTSTART on an
// already-started service.
bool advanceCellularMqttSession(bool &started, bool &clientAcquired, bool &sslBound, bool &connected) {
  String ctx = String(CELLULAR_MQTT_CLIENT_INDEX);

  if (!started) {
    String startResult;
    if (!sendMqttCommand("AT+CMQTTSTART", "+CMQTTSTART:", CELLULAR_MQTT_START_TIMEOUT_MS, &startResult)) {
      logKeyEvent("CELLULAR MQTT SESSION FAILED (CMQTTSTART): " + startResult);
      return false;
    }
    started = true;
  }

  if (!clientAcquired) {
    String accqCmd = "AT+CMQTTACCQ=" + ctx + ",\"" + cellularMqttClientId() + "\",1";
    if (!sendAtCommand(accqCmd, "OK", AT_DEFAULT_TIMEOUT_MS)) {
      logKeyEvent("CELLULAR MQTT SESSION FAILED (CMQTTACCQ)");
      return false;
    }
    clientAcquired = true;
  }

  if (!sslBound) {
    if (!sendAtCommand("AT+CMQTTSSLCFG=" + ctx + "," + String(CELLULAR_SSL_CTX_INDEX), "OK", AT_DEFAULT_TIMEOUT_MS)) {
      logKeyEvent("CELLULAR MQTT SESSION FAILED (CMQTTSSLCFG)");
      return false;
    }
    sslBound = true;
  }

  // "tcp://" is correct even for the SSL/TLS case - the manual's own
  // mutual-TLS example uses this same prefix; SSL-ness comes entirely
  // from the ACCQ server_type + SSLCFG steps above, not the URI scheme.
  String serverAddr = "tcp://" + cloudConfig.endpoint + ":" + String(cloudConfig.port);
  String connectCmd = "AT+CMQTTCONNECT=" + ctx + ",\"" + serverAddr + "\",60,1";
  String connectResult;
  if (!sendMqttCommand(connectCmd, "+CMQTTCONNECT:", CELLULAR_MQTT_CONNECT_TIMEOUT_MS, &connectResult)) {
    logKeyEvent("CELLULAR MQTT SESSION FAILED (CMQTTCONNECT): " + connectResult);
    return false;
  }

  connected = true;
  // Success is logged by the caller (cellularTask's mqttConnected
  // transition check), which also has the endpoint/context to say it once
  // cleanly rather than duplicating that here.
  return true;
}

// Closes the broker connection when WiFi has failed back and cellular MQTT
// is no longer needed as the active uplink. Deliberately leaves
// started/clientAcquired/sslBound alone (only the caller's `connected` flag
// is reset) - AT+CMQTTSTART/ACCQ/SSLCFG don't need repeating for the next
// failover, only AT+CMQTTCONNECT does, same as the existing publish-failure
// retry path. Best-effort: even if the modem doesn't ack cleanly, the
// session is being abandoned either way.
void disconnectCellularMqttSession() {
  String cmd = "AT+CMQTTDISC=" + String(CELLULAR_MQTT_CLIENT_INDEX) + ",60";
  sendMqttCommand(cmd, "+CMQTTDISC:", CELLULAR_MQTT_CONNECT_TIMEOUT_MS);
}

// Publishes one message on any topic over the modem's own MQTT client.
// outResultLine, if given, is filled with a short failure reason on any
// step's failure. cellularTask only, and only at loop level - never from
// inside a URC handler (see queueCellularAck()).
bool publishCellularMqtt(const String &topic, const String &payload, String *outResultLine = nullptr) {
  String ctx = String(CELLULAR_MQTT_CLIENT_INDEX);

  String topicCmd = "AT+CMQTTTOPIC=" + ctx + "," + String(topic.length());
  if (!sendAtCommandWithData(topicCmd, (const uint8_t *)topic.c_str(), topic.length(), "OK", AT_DEFAULT_TIMEOUT_MS)) {
    if (outResultLine) *outResultLine = "(CMQTTTOPIC failed)";
    return false;
  }

  String payloadCmd = "AT+CMQTTPAYLOAD=" + ctx + "," + String(payload.length());
  if (!sendAtCommandWithData(payloadCmd, (const uint8_t *)payload.c_str(), payload.length(), "OK", AT_DEFAULT_TIMEOUT_MS)) {
    if (outResultLine) *outResultLine = "(CMQTTPAYLOAD failed)";
    return false;
  }

  String pubCmd = "AT+CMQTTPUB=" + ctx + ",1,60";
  return sendMqttCommand(pubCmd, "+CMQTTPUB:", CELLULAR_MQTT_PUB_TIMEOUT_MS, outResultLine);
}

// Publishes the same telemetry payload the WiFi/PubSubClient path sends
// (buildTcpJson()) to the same topic, over the modem's client instead.
bool publishCellularMqttTelemetry(String *outResultLine = nullptr) {
  return publishCellularMqtt(cloudConfig.topic, buildTcpJson(), outResultLine);
}

// Subscribes the modem's MQTT client to the command topic (QoS 1), so
// inbound commands keep working during a failover - the counterpart of
// connectMqtt()'s mqttClient.subscribe() on the WiFi path. The topic
// arrives via the same ">"-prompt data mechanism as CMQTTTOPIC, and the
// real result is the asynchronous "+CMQTTSUB: <client>,<err>" line.
bool subscribeCellularMqttCommands() {
  String topic = mqttCommandTopic();
  String cmd = "AT+CMQTTSUB=" + String(CELLULAR_MQTT_CLIENT_INDEX) + "," + String(topic.length()) + ",1";

  String result;
  if (!sendAtCommandWithData(cmd, (const uint8_t *)topic.c_str(), topic.length(), "+CMQTTSUB:", CELLULAR_MQTT_PUB_TIMEOUT_MS, &result)
      || atTrailingErrCode(result, "+CMQTTSUB:") != 0) {
    logKeyEvent("CELLULAR MQTT CMD SUBSCRIBE FAILED: " + (result.length() > 0 ? result : String("(no response)")));
    return false;
  }

  logKeyEvent("CELLULAR MQTT CMD SUBSCRIBED: " + topic);
  return true;
}

// Publishes the immediate acks/query responses queued by
// handleMqttCommand() for commands that arrived over cellular - see the
// CellularAckItem comment. cellularTask loop level only.
void drainCellularAckTxQueue() {
  CellularAckItem item;

  while (xQueueReceive(cellularAckTxQueue, &item, 0) == pdTRUE) {
    publishCellularMqtt(mqttCommandAckTopic(), String(item.json));
  }
}

// Cellular-side counterpart of webTask's drainMqttAckQueue(): during a
// failover the completed-write acks are published over the modem instead.
// Same best-effort semantics. The mqttFailoverActive check mirrors the
// inverse check on the webTask side, so exactly one drain owns the queue
// at any moment.
void drainMqttAckQueueCellular() {
  if (!mqttFailoverActive) {
    return;
  }

  MqttAckItem item;

  while (xQueueReceive(mqttAckQueue, &item, 0) == pdTRUE) {
    String json = buildMqttAckJson(String(item.name), item.value, item.result == MB_SUCCESS, item.result, "");
    publishCellularMqtt(mqttCommandAckTopic(), json);
  }
}

void cellularTask(void *parameter) {
  // Default UART RX buffer is 256 bytes - an inbound command URC block
  // (topic + up to MQTT_CMD_MAX_LEN payload + framing) arriving while this
  // task is asleep between pump calls must fit without overflowing.
  SerialAT.setRxBufferSize(2048);
  SerialAT.begin(MODEM_BAUD, SERIAL_8N1, SERIAL2_RX_PIN, SERIAL2_TX_PIN);

  powerOnModem();

  bool everInitialized = false;
  // Set once provisionCellularMqttCerts() succeeds - only attempted again
  // after a reboot. A cert changed live via Settings while running
  // currently only reaches the cellular path on next reboot (the
  // WiFi/PubSubClient path already reconnects live on a cert change -
  // matching that here is a later polish pass, not needed for this stage).
  bool certsProvisioned = false;
  // MQTT session bring-up state (see advanceCellularMqttSession()) -
  // separate flags so a retry after a partial failure only re-attempts
  // whatever step didn't complete, not the whole sequence from scratch.
  bool mqttStarted = false;
  bool mqttClientAcquired = false;
  bool mqttSslBound = false;
  bool mqttConnected = false;
  // Command-topic subscription state - reset alongside mqttConnected,
  // since CMQTTCONNECT uses clean_session=1 (the broker forgets the
  // subscription on disconnect, so every new CONNECT must re-subscribe).
  bool mqttSubscribed = false;
  // 0 = "never published this session" - the first publish after a
  // session comes up always goes out immediately, regardless of interval.
  unsigned long lastPublishTime = 0;

  for (;;) {
    if (!everInitialized) {
      logKeyEvent("CELLULAR MODEM INIT - sending AT handshake");

      if (modem.testAT(10000)) {
        logKeyEvent("CELLULAR MODEM RESPONDING");
        modem.init();
        everInitialized = true;
      } else {
        logKeyEvent("CELLULAR MODEM NOT RESPONDING - check wiring/power, retrying");
        vTaskDelay(pdMS_TO_TICKS(10000));
        continue;
      }
    }

    // Process any inbound-command URCs buffered during the sleep loop
    // BEFORE handing the UART to TinyGSM's status calls - TinyGSM's own
    // response parsing would discard them as unrecognized lines.
    pumpCellularMqttUrcs();

    bool responding = modem.testAT(3000);
    bool simReady = responding && (modem.getSimStatus() == SIM_READY);
    bool registered = false;
    int signal = 99;
    String opName = "";
    bool dataConnected = false;

    if (simReady) {
      registered = modem.isNetworkConnected();
      signal = modem.getSignalQuality();
      if (registered) {
        opName = modem.getOperator();

        // Bring up (or confirm) the PDP/data context. Not yet used for any
        // actual uplink traffic (see Client* abstraction in a later
        // stage) - this only proves the data session itself comes up.
        dataConnected = modem.isGprsConnected();
        if (!dataConnected) {
          logMessage("CELLULAR DATA SESSION CONNECTING (APN='" + cellularConfig.apn + "')");
          dataConnected = modem.gprsConnect(cellularConfig.apn.c_str());
        }
      }
    }

    bool wasSimReady, wasRegistered, wasDataConnected;

    xSemaphoreTake(cellularMutex, portMAX_DELAY);
    wasSimReady = cellularStatus.simReady;
    wasRegistered = cellularStatus.networkRegistered;
    wasDataConnected = cellularStatus.dataConnected;
    cellularStatus.modemResponding = responding;
    cellularStatus.simReady = simReady;
    cellularStatus.networkRegistered = registered;
    cellularStatus.signalQuality = signal;
    cellularStatus.operatorName = opName;
    cellularStatus.dataConnected = dataConnected;
    cellularStatus.mqttCertsProvisioned = certsProvisioned;
    cellularStatus.lastUpdateTime = millis();
    xSemaphoreGive(cellularMutex);

    if (simReady != wasSimReady) {
      logKeyEvent(simReady ? "CELLULAR SIM READY" : "CELLULAR SIM NOT READY");
    }
    if (registered != wasRegistered) {
      logKeyEvent(registered ? ("CELLULAR NETWORK REGISTERED: " + opName) : "CELLULAR NETWORK LOST");
    }
    if (dataConnected != wasDataConnected) {
      logKeyEvent(dataConnected ? "CELLULAR DATA SESSION UP" : "CELLULAR DATA SESSION DOWN");
    }

    // Runs after the transition logs above (SIM ready/registered/data
    // session up) print, purely so the Status Log reads as a sensible
    // chronological narrative on a cold boot where everything comes up in
    // the same pass - cellularStatus.mqttCertsProvisioned itself may lag
    // this by up to one poll interval, which is fine for a status display.
    // Cert provisioning needs the data session up (AT+CCERTDOWN/
    // AT+CSSLCFG don't themselves transmit over it, but per the AT
    // Command Manual's recommended order, GPRS should be available before
    // any SSL-related operations) and is only worth doing at all when
    // MQTT mode + certs are actually configured.
    if (dataConnected && !certsProvisioned
        && cloudConfig.mode == UPLINK_MODE_MQTT
        && certRootCA.length() > 0 && certDevice.length() > 0 && certPrivKey.length() > 0) {
      certsProvisioned = provisionCellularMqttCerts();
    }

    // Cellular MQTT is only brought up/published while it's actually the
    // active uplink (mqttFailoverActive, set by webTask's
    // updateMqttFailoverState() once WiFi has been down UPLINK_FAILOVER_
    // THRESHOLD_MS) - not run continuously in parallel with WiFi, both to
    // avoid duplicate telemetry reaching AWS IoT over two transports at
    // once and to avoid burning cellular data while WiFi is healthy.
    bool wasMqttConnected = mqttConnected;
    // mqttFailoverActive is only meaningful in MQTT mode (webTask stops
    // updating it otherwise) - re-checking cloudConfig.mode here too covers
    // the edge case of the uplink mode being switched away from MQTT while
    // a cellular failover was active, so the session still tears down.
    bool shouldBeConnected = mqttFailoverActive && cloudConfig.mode == UPLINK_MODE_MQTT;

    // A +CMQTTCONNLOST/+CMQTTNONET URC may have been spotted at any point
    // since the last cycle (by the pump or mid-command) - fold it into the
    // session state before deciding what to do this cycle.
    if (cellularMqttConnLost) {
      cellularMqttConnLost = false;
      if (mqttConnected) {
        logKeyEvent("CELLULAR MQTT CONNECTION LOST (modem URC) - will reconnect");
        mqttConnected = false;
        mqttSubscribed = false;
      }
    }

    if (certsProvisioned && shouldBeConnected) {
      if (!mqttConnected) {
        mqttSubscribed = false;
        if (advanceCellularMqttSession(mqttStarted, mqttClientAcquired, mqttSslBound, mqttConnected)) {
          lastPublishTime = 0;  // fresh session - publish immediately below/next cycle
        }
      } else {
        // Subscribe before the first publish of the session, so a command
        // sent in reaction to the first telemetry can't slip through the gap.
        if (cloudConfig.commandsEnabled && !mqttSubscribed) {
          mqttSubscribed = subscribeCellularMqttCommands();
        }

        // Telemetry is paced by the user-set publish interval (Settings ->
        // Cellular Modem), independent of this loop's own cadence - the
        // loop keeps running every cycle for status/URCs/acks regardless.
        if (lastPublishTime == 0 || millis() - lastPublishTime >= cellularConfig.publishIntervalMs) {
          String pubResult;
          if (!publishCellularMqttTelemetry(&pubResult)) {
            logKeyEvent("CELLULAR MQTT PUBLISH FAILED - will retry connect: " + pubResult);
            mqttConnected = false;  // re-attempt CONNECT next cycle; STARTED/ACCQ/SSLCFG stay done
            mqttSubscribed = false;
          } else {
            lastPublishTime = millis();
            logMessage("CELLULAR MQTT TELEMETRY PUBLISHED");
          }
        }
      }
    } else if (mqttConnected && !shouldBeConnected) {
      // WiFi has failed back - release the broker connection so cellular
      // MQTT doesn't sit connected (and costing data) once it's no longer
      // the active uplink. START/ACCQ/SSLCFG stay done for a fast reconnect
      // on the next failover.
      disconnectCellularMqttSession();
      mqttConnected = false;
      mqttSubscribed = false;
    }

    xSemaphoreTake(cellularMutex, portMAX_DELAY);
    cellularStatus.mqttSessionConnected = mqttConnected;
    cellularStatus.mqttCommandsSubscribed = mqttSubscribed;
    xSemaphoreGive(cellularMutex);

    if (mqttConnected != wasMqttConnected) {
      logKeyEvent(mqttConnected ? ("CELLULAR MQTT SESSION UP: " + cloudConfig.endpoint) : "CELLULAR MQTT SESSION DOWN");
    }

    // Sleep until the next status/telemetry cycle - but while the MQTT
    // session is up, keep pumping the UART for inbound command URCs and
    // flushing any resulting acks, so a command is handled within ~200ms
    // instead of sitting in the RX buffer for the whole poll interval.
    unsigned long sleepStart = millis();
    while (millis() - sleepStart < CELLULAR_POLL_INTERVAL_MS) {
      if (mqttConnected) {
        pumpCellularMqttUrcs();
        drainCellularAckTxQueue();
        drainMqttAckQueueCellular();
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }
}

// ===================== Web / TCP / WiFi Task (Core 0) =====================
// Web server, TCP push, and WiFi retry - pinned to Core 0 so Core 1 stays
// dedicated to modbusTask.
void webTask(void *parameter) {
  bool ntpSyncLogged = false;

  for (;;) {
    server.handleClient();

    handleUplinkWiFiRetry();
    checkWifiUplinkStatusChange();
    if (cloudConfig.mode == UPLINK_MODE_MQTT) {
      updateMqttFailoverState();
    }
    pollDigitalIO();
    logFreeHeapPeriodic();
    processDoWriteQueue();

    // One-time key-log entry the first time SNTP produces real time, so
    // the field log shows when timestamps switched from uptime to UTC.
    if (!ntpSyncLogged && isTimeSynced()) {
      ntpSyncLogged = true;
      logKeyEvent("NTP TIME SYNCED - timestamps now UTC epoch");
    }

    // Fires once per completed poll cycle so the cloud push (TCP or MQTT)
    // always carries fresh data.
    if (xSemaphoreTake(pollCompleteSem, 0) == pdTRUE) {
      sendCloudData();
      xSemaphoreGive(tcpSendDoneSem);
    }

    if (cloudConfig.mode == UPLINK_MODE_MQTT) {
      // Services MQTT keepalive pings and inbound packets between publishes.
      if (mqttClient.connected()) {
        mqttClient.loop();
        drainMqttAckQueue();
      }
      checkMqttUplinkStatusChange();
    } else {
      checkTcpUplinkStatusChange();
      pollTcpCommands();
      drainTcpAckQueue();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ===================== Main Loop (Core 1, unused) =====================
// Intentionally empty: all real work has moved to webTask/modbusTask.
void loop() {
  delay(1000);
}
