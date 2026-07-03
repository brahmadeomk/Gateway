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

// ===================== RS485 Pins =====================
#define RXD2 D2
#define TXD2 D3
#define RS485_DE_RE D4

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
WiFiClient tcpClient;
WiFiClientSecure tlsClient;
PubSubClient mqttClient(tlsClient);

// ===================== Timers =====================
unsigned long lastSummaryPrintTime = 0;
unsigned long lastWifiAttempt = 0;
unsigned long lastMqttAttempt = 0;

// ===================== Cross-Task Synchronization =====================
// dataMutex: params[]/typeList[] (webTask writes, modbusTask reads/writes).
// logMutex: debugLogs/keyLog, mainly guarding webTask's keyLog[] reads.
// serialMutex: the RS485 UART, so settings can't change mid-transaction.
// pollCompleteSem/tcpSendDoneSem: two-way handshake so each cycle's TCP
// push finishes (or is skipped) before the next poll cycle starts.
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t logMutex;
SemaphoreHandle_t serialMutex;
SemaphoreHandle_t pollCompleteSem;
SemaphoreHandle_t tcpSendDoneSem;
TaskHandle_t modbusTaskHandle;

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

// ===================== TCP / WiFi Uplink Config =====================
struct UplinkConfig {
  String ssid;
  String password;
  String serverIP;
  int port;
  String ntpServer;  // optional on-premise NTP; tried before the public servers
};

UplinkConfig uplinkConfig;

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

struct ModbusParam {
  String name;
  String type;
  uint8_t area;  // AREA_* address space (determines the read function code)
  uint8_t slaveId;
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
};

ModbusParam params[MAX_PARAMS];
int paramCount = 0;

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
void saveCommunicationSettings() {
  preferences.begin("comm", false);

  preferences.putUInt("baud", commBaudRate);
  preferences.putString("parity", commParity);
  preferences.putUChar("stop", commStopBits);
  preferences.putUInt("pollms", pollIntervalMs);
  preferences.putUInt("recoveryms", slaveRecoveryDelayMs);

  preferences.end();
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
void saveUplinkConfig() {
  logMessage("========== SAVE TCP CONFIG START ==========");

  logMessage("Saving SSID: " + uplinkConfig.ssid);
  logMessage("Saving Password Length: " + String(uplinkConfig.password.length()));
  logMessage("Saving Server IP: " + uplinkConfig.serverIP);
  logMessage("Saving Port: " + String(uplinkConfig.port));

  preferences.begin("uplink", false);

  preferences.putString("ssid", uplinkConfig.ssid);
  preferences.putString("pass", uplinkConfig.password);
  preferences.putString("ip", uplinkConfig.serverIP);
  preferences.putInt("port", uplinkConfig.port);
  preferences.putString("ntp", uplinkConfig.ntpServer);

  preferences.end();

  logMessage("TCP CONFIG SAVED TO FLASH");
  logMessage("========== SAVE TCP CONFIG END ==========");
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
void saveCloudConfig() {
  preferences.begin("cloud", false);

  preferences.putUChar("mode", cloudConfig.mode);
  preferences.putString("ep", cloudConfig.endpoint);
  preferences.putInt("port", cloudConfig.port);
  preferences.putString("cid", cloudConfig.clientId);
  preferences.putString("topic", cloudConfig.topic);

  preferences.end();
}

void loadCloudConfig() {
  preferences.begin("cloud", true);

  cloudConfig.mode = preferences.getUChar("mode", UPLINK_MODE_TCP);
  cloudConfig.endpoint = preferences.getString("ep", "");
  cloudConfig.port = preferences.getInt("port", DEFAULT_MQTT_PORT);
  cloudConfig.clientId = preferences.getString("cid", "");
  cloudConfig.topic = preferences.getString("topic", DEFAULT_MQTT_TOPIC);

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
void saveCerts() {
  preferences.begin("certs", false);

  preferences.putString("ca", certRootCA);
  preferences.putString("cert", certDevice);
  preferences.putString("key", certPrivKey);

  preferences.end();
}

void loadCerts() {
  preferences.begin("certs", true);

  certRootCA = preferences.getString("ca", "");
  certDevice = preferences.getString("cert", "");
  certPrivKey = preferences.getString("key", "");

  preferences.end();
}

// ===================== Device / AP Identity Save / Load =====================
void saveDeviceConfig() {
  preferences.begin("device", false);

  preferences.putString("apssid", deviceConfig.apSsid);
  preferences.putString("prefix", deviceConfig.deviceNamePrefix);
  preferences.putBool("apmatch", deviceConfig.apMatchesDeviceName);

  preferences.end();
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
    } else {
      logKeyEvent("WIFI DISCONNECTED");
    }
    lastWifiUplinkConnected = connected;
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
void saveTypes() {
  preferences.begin("types", false);

  preferences.putUInt("schema", TYPE_SCHEMA_VERSION);
  preferences.putInt("count", typeCount);

  for (int i = 0; i < typeCount; i++) {
    String index = String(i);

    preferences.putString(("name" + index).c_str(), typeList[i].name);
    preferences.putString(("base" + index).c_str(), typeList[i].baseFormat);
    preferences.putFloat(("div" + index).c_str(), typeList[i].divisor);
    preferences.putUShort(("dlen" + index).c_str(), typeList[i].dataLength);
  }

  preferences.end();
}

void loadTypes() {
  preferences.begin("types", true);

  uint32_t savedSchema = preferences.getUInt("schema", 0);
  typeCount = preferences.getInt("count", 0);

  preferences.end();

  if (savedSchema != TYPE_SCHEMA_VERSION || typeCount <= 0 || typeCount > MAX_PARAM_TYPES) {
    loadDefaultTypes();
    saveTypes();
    return;
  }

  preferences.begin("types", true);

  for (int i = 0; i < typeCount; i++) {
    String index = String(i);

    typeList[i].name = preferences.getString(("name" + index).c_str(), "INT_16/100");
    typeList[i].divisor = preferences.getFloat(("div" + index).c_str(), 100);

    if (typeList[i].name.length() == 0) {
      typeList[i].name = "INT_16/100";
    }

    if (typeList[i].divisor <= 0) {
      typeList[i].divisor = 1;
    }

    typeList[i].baseFormat = getBaseFormatFromType(typeList[i].name);
    typeList[i].dataLength = inferDataLengthFromType(typeList[i].name);
  }

  preferences.end();
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
    params[i].slaveId = 1;
    params[i].registerAddress = 0;
    params[i].registerLength = 1;
    params[i].enabled = false;
    params[i].value = 0;
    params[i].valid = false;
    params[i].errorCode = 0;
    params[i].lastUpdateTime = 0;
    resetDebugState(i);
  }

  xSemaphoreGive(dataMutex);
}

// ===================== Save / Load Modbus Settings =====================
void saveSettings() {
  preferences.begin("modbus", false);

  preferences.putInt("count", paramCount);

  for (int i = 0; i < paramCount; i++) {
    String index = String(i);

    preferences.putString(("name" + index).c_str(), params[i].name);
    preferences.putString(("type" + index).c_str(), params[i].type);
    preferences.putUChar(("area" + index).c_str(), params[i].area);
    preferences.putUChar(("sid" + index).c_str(), params[i].slaveId);
    preferences.putUShort(("addr" + index).c_str(), params[i].registerAddress);
    preferences.putUShort(("len" + index).c_str(), params[i].registerLength);
    preferences.putBool(("en" + index).c_str(), params[i].enabled);
  }

  preferences.end();
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

  for (int i = 0; i < paramCount; i++) {
    String index = String(i);

    params[i].name = preferences.getString(("name" + index).c_str(), "Parameter");
    params[i].type = preferences.getString(("type" + index).c_str(), "INT_16/100");

    if (findTypeIndex(params[i].type) < 0 && typeCount > 0) {
      params[i].type = typeList[0].name;
    }

    // Default AREA_HOLDING_REGISTER (0): configs saved before the area
    // field existed keep their original behavior.
    params[i].area = preferences.getUChar(("area" + index).c_str(), AREA_HOLDING_REGISTER);
    if (params[i].area > AREA_DISCRETE_INPUT) {
      params[i].area = AREA_HOLDING_REGISTER;
    }

    params[i].slaveId = preferences.getUChar(("sid" + index).c_str(), 1);
    params[i].registerAddress = preferences.getUShort(("addr" + index).c_str(), 0);
    params[i].registerLength = getDataLengthForType(params[i].type);
    params[i].enabled = preferences.getBool(("en" + index).c_str(), true);
    params[i].value = 0;
    params[i].valid = false;
    params[i].errorCode = 0;
    params[i].lastUpdateTime = 0;
    resetDebugState(i);
  }

  preferences.end();
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

String buildTcpJson() {
  String json = "{";
  json.reserve(128 + paramCount * 96);  // avoid repeated reallocation while appending below

  // Same identity the user sees on the Settings page (Device Name):
  // prefix_XXXXX when a prefix is set, full MAC otherwise.
  json += "\"device_id\":\"" + jsonEscape(getDeviceName()) + "\",";
  json += "\"timestamp\":" + String(getTimestamp()) + ",";
  json += "\"time_source\":\"" + String(isTimeSynced() ? "ntp" : "uptime") + "\",";
  json += "\"sensors\":[";

  xSemaphoreTake(dataMutex, portMAX_DELAY);

  for (int i = 0; i < paramCount; i++) {
    if (i > 0) {
      json += ",";
    }

    String deviceId = String(params[i].slaveId);

    String statusText;

    if (!params[i].enabled) {
      statusText = "Disabled";
    } else if (params[i].valid) {
      statusText = "OK";
    } else {
      statusText = "Disconnected";
    }

    json += "{";
    json += "\"sensor_id\":\"" + jsonEscape(deviceId) + "\",";
    json += "\"sensor_name\":\"" + jsonEscape(params[i].name) + "\",";

    if (params[i].valid) {
      json += "\"value\":" + String(params[i].value, 3) + ",";
    } else {
      json += "\"value\":null,";
    }

    json += "\"status\":\"" + statusText + "\"";
    json += "}";
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

  if (WiFi.status() != WL_CONNECTED) {
    logMessage("TCP SKIP: WIFI NOT CONNECTED");
    return false;
  }

  if (!tcpClient.connected()) {
    logMessage("TCP TRY " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));

    if (!tcpClient.connect(uplinkConfig.serverIP.c_str(), uplinkConfig.port, TCP_CONNECT_TIMEOUT_MS)) {
      // Throttled: a dead master would otherwise fail once per poll cycle
      // and flood the key log.
      if (millis() - lastTcpFailLogTime >= DEBUG_ERROR_REPEAT_MS) {
        lastTcpFailLogTime = millis();
        logKeyEvent("TCP CONNECT FAILED: " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));
      }
      return false;
    }

    // Disable Nagle's algorithm so small JSON payloads go out immediately
    // instead of being buffered/delayed. Must be set after connect() -
    // some cores reset this flag to its default during connect().
    tcpClient.setNoDelay(true);

    logMessage("TCP OK");
  }

  return true;
}

// Logs a key event only on TCP connect/disconnect transitions.
void checkTcpUplinkStatusChange() {
  bool connected = tcpClient.connected();

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
  size_t written = tcpClient.write((const uint8_t *)framed.c_str(), framed.length());

  if (written != framed.length()) {
    logMessage("TCP SEND INCOMPLETE (" + String(written) + "/" + String(framed.length()) + ")");
    tcpClient.stop();
    return false;
  }

  logMessage("TCP DATA SENT");
  logMessage(payload);
  return true;
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
  // large enough).
  uint16_t needed = payload.length() + cloudConfig.topic.length() + 16;
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
    enabled = params[i].enabled;
    type = params[i].type;
    area = params[i].area;
    slaveId = params[i].slaveId;
    regAddr = params[i].registerAddress;
    regLen = getDataLengthForType(type);
    xSemaphoreGive(dataMutex);

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
    html += tcpClient.connected() ? "Connected" : "Disconnected";
    html += " (Target " + htmlEscape(uplinkConfig.serverIP) + ":" + String(uplinkConfig.port) + ")<br>";
  }
  html += "<b>Poll Interval:</b> " + String(pollIntervalMs) + " ms";

  // Only shown while there is an undelivered backlog (uplink outage).
  if (sfCount > 0) {
    html += "<br><b>Buffered (offline):</b> " + String(sfCount) + " payloads / " + String(sfBytes) + " bytes";
  }
  html += "</div>";

  html += R"rawliteral(
<div class="box">
<h2>Dashboard</h2>
<table>
<thead>
<tr>
<th>Name</th>
<th>Enabled</th>
<th>Value</th>
<th>Status</th>
<th>Last Update</th>
</tr>
</thead>
<tbody id="dataBody"></tbody>
</table>
</div>

<div class="box">
<h2>Status Log</h2>
<div id="keyLogBox" class="logbox"></div>
</div>

<script>
let lastData = [];
let lastServerNowMs = 0;
let lastFetchClientTime = 0;

function formatAge(lastUpdateMs) {
  if (!lastUpdateMs) return "Never";
  let nowMs = lastServerNowMs + (Date.now() - lastFetchClientTime);
  let ageSec = Math.floor((nowMs - lastUpdateMs) / 1000);
  if (ageSec < 0) ageSec = 0;
  return ageSec + "s ago";
}

function renderTable(){
  let body = "";
  lastData.forEach(p=>{
   let ageSec = p.lastUpdateMs ? Math.floor((lastServerNowMs + (Date.now() - lastFetchClientTime) - p.lastUpdateMs) / 1000) : null;
   let staleClass = (ageSec === null || ageSec > 10) ? "stale" : "";
   body += `<tr>
   <td>${p.name}</td>
   <td>${p.enabled ? 'Yes':'No'}</td>
   <td>${p.value ?? '-'}</td>
   <td>${p.valid ? 'OK':'ERR'}</td>
   <td class="${staleClass}">${formatAge(p.lastUpdateMs)}</td>
   </tr>`;
  });
  document.getElementById("dataBody").innerHTML = body;
}

function loadData(){
 fetch('/data')
 .then(r=>r.json())
 .then(d=>{
  lastData = d.data;
  lastServerNowMs = d.nowMs;
  lastFetchClientTime = Date.now();
  renderTable();
 });
}

setInterval(loadData, 2000);
setInterval(renderTable, 1000);
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

// ===================== Settings Page =====================
void handleSettings() {

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
  </style>
</head>
<body>

<h2>Modbus Settings</h2>

<button class="nav" type="button" onclick="location.href='/'">Dashboard</button>
<button class="nav" type="button" onclick="location.href='/types'">Manage Data Types</button>
<button class="danger" type="button" onclick="confirmResetModbus()">Reset Modbus</button>

)rawliteral";

  if (server.hasArg("saved")) {
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
<th>Register Area</th>
<th>Data Type</th>
<th>Data Length</th>
<th>Device ID</th>
<th>Register Address</th>
<th>Enable</th>
<th>Action</th>
</tr>
</thead>
<tbody>
)rawliteral";

  for (int i = 0; i < paramCount; i++) {
    uint16_t dlen = isBitArea(params[i].area) ? 1 : getDataLengthForType(params[i].type);

    html += "<tr>";
    html += "<td><input name='name" + String(i) + "' value='" + htmlEscape(params[i].name) + "'></td>";
    html += "<td><select name='area" + String(i) + "' onchange='updateAreaSelection(this)'>" + getAreaOptions(params[i].area) + "</select></td>";
    html += "<td><select name='type" + String(i) + "' onchange='updateDataLength(this)'>" + getTypeOptions(params[i].type) + "</select></td>";
    html += "<td><input class='dlen-display' value='" + String(dlen) + "' readonly></td>";
    html += "<td><input name='sid" + String(i) + "' value='" + String(params[i].slaveId) + "'></td>";
    html += "<td><input name='addr" + String(i) + "' value='" + String(params[i].registerAddress) + "'></td>";
    html += "<td><input type='checkbox' name='en" + String(i) + "'";
    if (params[i].enabled) html += " checked";
    html += "></td>";
    html += "<td>";
    html += "<button type='button' class='add' onclick='addParamRowAtEnd()'>Add</button>";
    html += "<button type='button' class='insert' onclick='insertParamRowAfter(this)'>Insert</button>";
    html += "<button type='button' class='remove' onclick='deleteParamRow(this)'>Delete</button>";
    html += "</td>";
    html += "</tr>";
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
  updateDataLength(typeSel);
}

function buildParamRow() {
  let row = document.createElement("tr");
  let defaultType = paramTypeInfo.length ? paramTypeInfo[0].name : "";
  let defaultLen = paramTypeInfo.length ? paramTypeInfo[0].length : 1;

  row.innerHTML = `
    <td><input name="nameX" value=""></td>
    <td><select name="areaX" onchange="updateAreaSelection(this)">${buildAreaOptionsHTML(0)}</select></td>
    <td><select name="typeX" onchange="updateDataLength(this)">${buildTypeOptionsHTML(defaultType)}</select></td>
    <td><input class="dlen-display" value="${defaultLen}" readonly></td>
    <td><input name="sidX" value="1"></td>
    <td><input name="addrX" value="0"></td>
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
    row.querySelector("select[name^='area']").name = "area" + index;
    row.querySelector("select[name^='type']").name = "type" + index;
    row.querySelector("input[name^='sid']").name = "sid" + index;
    row.querySelector("input[name^='addr']").name = "addr" + index;
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


  // DEVICE / AP IDENTITY FORM
  html += "<div class='box'>";
  html += "<h2>Device Identity</h2>";

  if (server.hasArg("deviceSaved")) {
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


  // NETWORK CONFIG FORM
  html += "<div class='box'>";
  html += "<h2>WiFi &amp; TCP Configuration</h2>";

  if (server.hasArg("uplinkSaved")) {
    html += "<div class='success'>Network Settings Saved Successfully</div>";
  }

  html += "<form action='/saveUplink' method='POST'>";
  html += "<table>";
  html += "<tr><th>WiFi SSID</th><td><input type='text' name='ssid' value='" + htmlEscape(uplinkConfig.ssid) + "'></td></tr>";
  html += "<tr><th>WiFi Password</th><td><input type='password' name='password' placeholder='Leave blank to keep unchanged'></td></tr>";
  html += "<tr><th>Master IP</th><td><input type='text' name='ip' value='" + htmlEscape(uplinkConfig.serverIP) + "'></td></tr>";
  html += "<tr><th>Master Port</th><td><input type='number' name='port' value='" + String(uplinkConfig.port) + "'></td></tr>";

  html += "<tr><th>On-Premise NTP Server</th><td><input type='text' name='ntpServer' maxlength='" + String(NTP_SERVER_MAX_LEN - 1) + "' placeholder='Optional, e.g. 192.168.1.10 or ntp.local' value='" + htmlEscape(uplinkConfig.ntpServer) + "'>"
          "<br><small>Tried first for time sync; falls back to public NTP (" + String(NTP_SERVER_1) + "), then uptime-based timestamps. Leave blank to use public NTP only.</small></td></tr>";
  html += "<tr><td colspan='2'><button type='submit'>Save Network</button></td></tr>";
  html += "</table>";
  html += "</form>";
  html += "</div>";


  // CLOUD UPLINK FORM (raw TCP vs AWS IoT MQTT)
  html += "<div class='box'>";
  html += "<h2>Cloud Uplink</h2>";

  if (server.hasArg("cloudSaved")) {
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
  html += ">AWS IoT MQTT over TLS</option>";
  html += "</select></td></tr>";

  html += "<tr><th>MQTT Endpoint</th><td><input type='text' name='mqttEndpoint' placeholder='xxxx-ats.iot.region.amazonaws.com' value='" + htmlEscape(cloudConfig.endpoint) + "'></td></tr>";
  html += "<tr><th>MQTT Port</th><td><input type='number' name='mqttPort' value='" + String(cloudConfig.port) + "'></td></tr>";
  html += "<tr><th>MQTT Client ID</th><td><input type='text' name='mqttClientId' placeholder='Blank = Device Name (" + htmlEscape(getDeviceName()) + ")' value='" + htmlEscape(cloudConfig.clientId) + "'>"
          "<br><small>Should match the AWS IoT Thing name / policy.</small></td></tr>";
  html += "<tr><th>Publish Topic</th><td><input type='text' name='mqttTopic' value='" + htmlEscape(cloudConfig.topic) + "'></td></tr>";

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


  // COMMUNICATION (SERIAL) SETTINGS FORM
  html += "<div class='box'>";
  html += "<h2>RS485 Communication Settings</h2>";

  if (server.hasArg("commSaved")) {
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

  server.send(200, "text/html", html);
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
</style>
</head>
<body>
<h2>Data Type Settings</h2>
<button class="back" type="button" onclick="location.href='/settings'">Back</button>
<button class="add" type="button" onclick="addTypeRow()">Add Data Type</button>
<button class="reset" type="button" onclick="confirmResetTypes()">Reset Data Types</button>
)rawliteral";

  if (server.hasArg("typeSaved")) {
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
    params[i].type = server.arg("type" + index);

    if (params[i].type.length() == 0 || findTypeIndex(params[i].type) < 0) {
      if (typeCount > 0) {
        params[i].type = typeList[0].name;
      } else {
        params[i].type = "INT_16/100";
      }
    }

    params[i].area = server.arg("area" + index).toInt();
    if (params[i].area > AREA_DISCRETE_INPUT) {
      params[i].area = AREA_HOLDING_REGISTER;
    }

    params[i].slaveId = server.arg("sid" + index).toInt();
    params[i].registerAddress = server.arg("addr" + index).toInt();
    params[i].registerLength = isBitArea(params[i].area) ? 1 : getDataLengthForType(params[i].type);
    params[i].enabled = server.hasArg("en" + index);

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
  saveSettings();

  server.sendHeader("Location", "/settings?saved=1");
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

  saveCommunicationSettings();

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

  server.sendHeader("Location", "/settings?commSaved=1");
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

  saveUplinkConfig();

  WiFi.disconnect();
  delay(200);

  connectUplinkWiFi();

  // Re-arm SNTP so a changed/cleared on-premise server takes effect
  // immediately, not just after the next reboot.
  startNtp();

  logKeyEvent("NETWORK CONFIG SAVED: ssid=" + uplinkConfig.ssid + " tcp=" + uplinkConfig.serverIP + ":" + String(uplinkConfig.port)
              + (uplinkConfig.ntpServer.length() > 0 ? (" ntp=" + uplinkConfig.ntpServer) : ""));

  server.sendHeader("Location", "/settings?uplinkSaved=1");
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

  saveDeviceConfig();

  String apSsidToUse = getEffectiveApSsid();
  WiFi.softAP(apSsidToUse.c_str(), AP_PASSWORD);

  logKeyEvent("DEVICE SETTINGS SAVED: AP=" + apSsidToUse + " deviceName=" + getDeviceName());

  server.sendHeader("Location", "/settings?deviceSaved=1");
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

  saveCloudConfig();

  // Per slot: uploaded file > pasted text > blank keeps stored value (same
  // pattern as the WiFi password), so re-saving other cloud settings never
  // wipes the certs.
  bool certsChanged = false;
  certsChanged |= applyCertUpdate(certRootCA, uploadCaCert, "caCert", "Root CA");
  certsChanged |= applyCertUpdate(certDevice, uploadDevCert, "devCert", "Device Certificate");
  certsChanged |= applyCertUpdate(certPrivKey, uploadPrivKey, "privKey", "Private Key");

  if (certsChanged) {
    saveCerts();
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

  server.sendHeader("Location", "/settings?cloudSaved=1");
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

  saveTypes();

  logKeyEvent("DATA TYPES SAVED: " + String(rows) + " types");

  server.sendHeader("Location", "/types?typeSaved=1");
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
    json += "\"slaveId\":" + String(params[i].slaveId) + ",";
    json += "\"registerAddress\":" + String(params[i].registerAddress) + ",";
    json += "\"registerLength\":" + String(params[i].registerLength) + ",";
    json += "\"enabled\":" + String(params[i].enabled ? "true" : "false") + ",";
    json += "\"valid\":" + String(params[i].valid ? "true" : "false") + ",";
    json += "\"errorCode\":" + String(params[i].errorCode) + ",";
    json += "\"lastUpdateMs\":" + String(params[i].lastUpdateTime) + ",";

    if (params[i].valid) {
      json += "\"value\":" + String(params[i].value, 3);
    } else {
      json += "\"value\":null";
    }

    json += "}";
  }

  xSemaphoreGive(dataMutex);

  json += "]";
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
  pollCompleteSem = xSemaphoreCreateBinary();
  tcpSendDoneSem = xSemaphoreCreateBinary();
  logQueue = xQueueCreate(LOG_QUEUE_LEN, sizeof(LogQueueItem));

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

  loadCommunicationSettings();
  loadUplinkConfig();
  loadDeviceConfig();
  loadCloudConfig();
  loadCerts();

  Serial1.begin(
    commBaudRate,
    getSerialConfig(),
    RXD2,
    TXD2);

  loadTypes();
  loadSettings();

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
  server.on("/keylog", HTTP_GET, handleKeyLog);
  server.on("/types", HTTP_GET, handleTypes);
  server.on("/saveTypes", HTTP_POST, handleSaveTypes);
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

// ===================== Web / TCP / WiFi Task (Core 0) =====================
// Web server, TCP push, and WiFi retry - pinned to Core 0 so Core 1 stays
// dedicated to modbusTask.
void webTask(void *parameter) {
  bool ntpSyncLogged = false;

  for (;;) {
    server.handleClient();

    handleUplinkWiFiRetry();
    checkWifiUplinkStatusChange();

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
      }
      checkMqttUplinkStatusChange();
    } else {
      checkTcpUplinkStatusChange();
    }

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ===================== Main Loop (Core 1, unused) =====================
// Intentionally empty: all real work has moved to webTask/modbusTask.
void loop() {
  delay(1000);
}
