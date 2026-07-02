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
   - modbusReadHoldingRegisters() is a small self-contained Modbus RTU
     master (function 0x03 only) instead of the ModbusMaster library,
     because the library's response timeout is a hardcoded, unconfigurable
     2000ms. It enforces an explicit 3s response timeout
     (MODBUS_RESPONSE_TIMEOUT_MS), plus the spec's T3.5 (inter-frame) and
     T1.5 (inter-byte) silence rules computed from the live baud rate.
   - Errors (timeout, bad CRC/length, wrong slave, slave exception) are
     reported immediately with no debounce/grace period.
   - Each poll cycle is strictly poll -> TCP send -> next poll:
     pollModbus() signals pollCompleteSem when done; webTask sends that
     cycle's data and gives tcpSendDoneSem; modbusTask waits on it with a
     bounded timeout (TCP_SEND_TIMEOUT_MS) so a dead TCP link can never
     stall polling.
   - Logging is offloaded from modbusTask via a queue (logQueue) to a
     dedicated low-priority logTask on Core 0, so Serial/String work never
     steals time from the real-time core.
   ====================================================================== */

#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>
#include <math.h>
#include <WiFiClient.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
// ===================== WiFi AP Settings =====================
const char* AP_SSID = "ESP32_Modbus_Setup";
const char* AP_PASSWORD = "12345678";

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
// per-transaction inside modbusReadHoldingRegisters() regardless of this
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

// ===================== Timers =====================
unsigned long lastSummaryPrintTime = 0;
unsigned long lastWifiAttempt = 0;

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
unsigned long lastTcpFailLogTime = 0;
unsigned long lastWifiFailLogTime = 0;

// ===================== TCP / WiFi Uplink Config =====================
struct UplinkConfig {
  String ssid;
  String password;
  String serverIP;
  int port;
};

UplinkConfig uplinkConfig;

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
struct ModbusParam {
  String name;
  String type;
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
void sendTcpData();

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
// Any other nonzero result is a raw Modbus exception code (1-11) reported
// by the slave itself (e.g. 2 = ILLEGAL DATA ADDRESS).

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

// Reads `qty` (1-2) holding registers from `slaveId` at `startAddr` into
// outRegs[]. Must be called with serialMutex held. Enforces T3.5/T1.5
// spec timing; recognizes a complete response by expected frame length
// rather than a fixed byte count. Returns MB_SUCCESS or an MB_ERR_* code.
uint8_t modbusReadHoldingRegisters(uint8_t slaveId, uint16_t startAddr, uint16_t qty, uint16_t *outRegs) {
  uint8_t req[8];

  req[0] = slaveId;
  req[1] = 0x03;
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
        expectedLen = (rx[1] & 0x80) ? 5 : (5 + qty * 2);
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

  if (rx[1] != 0x03 || rx[2] != qty * 2) {
    return MB_ERR_CRC;
  }

  for (uint16_t r = 0; r < qty; r++) {
    outRegs[r] = ((uint16_t)rx[3 + r * 2] << 8) | rx[3 + r * 2 + 1];
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

unsigned long getTimestamp() {
  return millis() / 1000;
}

String buildTcpJson() {
  String json = "{";
  json.reserve(96 + paramCount * 96);  // avoid repeated reallocation while appending below

  json += "\"device_id\":\"" + getDeviceMacID() + "\",";
  json += "\"timestamp\":" + String(getTimestamp()) + ",";
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

void sendTcpData() {
  if (!connectTcpServer()) {
    return;
  }

  String payload = buildTcpJson();
  payload += "\n";

  // Single write() call so the payload goes out as one TCP send instead of two.
  tcpClient.write((const uint8_t *)payload.c_str(), payload.length());

  logMessage("TCP DATA SENT");
  logMessage(payload);
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
    uint8_t slaveId;
    uint16_t regAddr;
    uint16_t regLen;

    xSemaphoreTake(dataMutex, portMAX_DELAY);
    enabled = params[i].enabled;
    type = params[i].type;
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

    if (regLen < 1) {
      regLen = 1;
    }

    if (regLen > 2) {
      regLen = 2;
    }

    // ---- RS485 transaction: serialMutex only guards against a comm-settings
    // save changing baud/parity/stop mid-transaction. ----
    xSemaphoreTake(serialMutex, portMAX_DELAY);

    uint16_t regs[2] = { 0, 0 };
    uint8_t result = modbusReadHoldingRegisters(slaveId, regAddr, regLen, regs);

    // Snapshot the raw bytes before releasing serialMutex - the next
    // transaction would otherwise overwrite lastModbusTx/Rx before the
    // log line below gets a chance to read them.
    String rawTx = bytesToHex(lastModbusTx, lastModbusTxLen, false);
    String rawRx = bytesToHex(lastModbusRx, lastModbusRxLen, lastModbusRxOverflow);

    xSemaphoreGive(serialMutex);
    // ---- End RS485 transaction ----

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
      params[i].value = convertValue(type, regs[0], regs[1]);
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

  html += "<b>TCP:</b> ";
  html += tcpClient.connected() ? "Connected" : "Disconnected";
  html += " (Target " + htmlEscape(uplinkConfig.serverIP) + ":" + String(uplinkConfig.port) + ")<br>";
  html += "<b>Poll Interval:</b> " + String(pollIntervalMs) + " ms";
  html += "</div>";

  html += R"rawliteral(
<div class="box">
<h2>Dashboard</h2>
<table>
<thead>
<tr>
<th>Name</th>
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
    uint16_t dlen = getDataLengthForType(params[i].type);

    html += "<tr>";
    html += "<td><input name='name" + String(i) + "' value='" + htmlEscape(params[i].name) + "'></td>";
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

function updateDataLength(selectEl) {
  let row = selectEl.closest("tr");
  let lenInput = row.querySelector(".dlen-display");
  let info = paramTypeInfo.find(t => t.name === selectEl.value);
  lenInput.value = info ? info.length : 1;
}

function buildParamRow() {
  let row = document.createElement("tr");
  let defaultType = paramTypeInfo.length ? paramTypeInfo[0].name : "";
  let defaultLen = paramTypeInfo.length ? paramTypeInfo[0].length : 1;

  row.innerHTML = `
    <td><input name="nameX" value=""></td>
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
  html += "<tr><td colspan='2'><button type='submit'>Save Network</button></td></tr>";
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

    params[i].slaveId = server.arg("sid" + index).toInt();
    params[i].registerAddress = server.arg("addr" + index).toInt();
    params[i].registerLength = getDataLengthForType(params[i].type);
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

  if (uplinkConfig.port <= 0 || uplinkConfig.port > 65535) {
    uplinkConfig.port = 5000;
  }

  saveUplinkConfig();

  WiFi.disconnect();
  delay(200);

  connectUplinkWiFi();

  logKeyEvent("NETWORK CONFIG SAVED: ssid=" + uplinkConfig.ssid + " tcp=" + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));

  server.sendHeader("Location", "/settings?uplinkSaved=1");
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

  Serial1.begin(
    commBaudRate,
    getSerialConfig(),
    RXD2,
    TXD2);

  loadTypes();
  loadSettings();

  WiFi.mode(WIFI_AP_STA);
  WiFi.softAP(AP_SSID, AP_PASSWORD);

  connectUplinkWiFi();

  logKeyEvent("AP STARTED: " + String(AP_SSID) + " " + WiFi.softAPIP().toString());
  logMessage("RS485 RX=D2 TX=D3 DE/RE=D4");
  logMessage("TCP TARGET: " + uplinkConfig.serverIP + ":" + String(uplinkConfig.port));

  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/save", HTTP_POST, handleSave);
  server.on("/saveCommunication", HTTP_POST, handleSaveCommunication);
  server.on("/saveUplink", HTTP_POST, handleSaveUplink);
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
  for (;;) {
    server.handleClient();

    handleUplinkWiFiRetry();
    checkWifiUplinkStatusChange();

    // Fires once per completed poll cycle so TCP always carries fresh data.
    if (xSemaphoreTake(pollCompleteSem, 0) == pdTRUE) {
      sendTcpData();
      xSemaphoreGive(tcpSendDoneSem);
    }

    checkTcpUplinkStatusChange();

    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ===================== Main Loop (Core 1, unused) =====================
// Intentionally empty: all real work has moved to webTask/modbusTask.
void loop() {
  delay(1000);
}
