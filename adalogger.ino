/*
  adalogger - an ESP32 CAN Bus Logger & Debugger with WiFi AP

  VERY ILLEGAL FOR ON FIELD USE WITH WIFI!!!!

  ----------------------------------------------
  Features:
  - WiFi Access Point for phone/tablet access
  - Web interface showing live CAN devices
  - FRC CAN spec device identification
  - Error logging and statistics
  - Downloadable logs from memory
  - Real-time CAN traffic monitoring
  - Relay control based on RoboRIO enable signal
  
  CAN: TX=IO4, RX=IO5 (external transceiver required if not using pcb)
  Relay: IO2 (active HIGH)
  Bitrate: 1 Mbps
  WiFi AP: SSID="adaloger", Password="blackknights2036"
  Web Interface: http://192.168.4.1
*/

#include <WiFi.h>
#include <WebServer.h>
#include "driver/twai.h"
#include <vector>
#include <map>

// ========= WiFi AP Configuration =========
const char* AP_SSID = "adalogger";
const char* AP_PASSWORD = "blackknights2036";
const IPAddress AP_IP(192, 168, 4, 1);
const IPAddress AP_GATEWAY(192, 168, 4, 1);
const IPAddress AP_SUBNET(255, 255, 255, 0);

// ========= CAN Configuration =========
static constexpr int CAN_TX_PIN = 4;
static constexpr int CAN_RX_PIN = 5;
static constexpr uint32_t CAN_BITRATE = 1000000; // 1 Mbps

// ========= Relay Configuration =========
static constexpr int RELAY_PIN = 2;
static constexpr uint32_t ENABLE_HEARTBEAT_TIMEOUT_MS = 500; // 500ms without heartbeat = flash
static constexpr uint32_t FLASH_INTERVAL_MS = 250; // Flash rate when no heartbeat

// ========= FRC CAN Spec Definitions =========
// Device types from FRC CAN specification
enum FRCDeviceType {
  BROADCAST = 0,
  ROBOT_CONTROLLER = 1,
  MOTOR_CONTROLLER = 2,
  RELAY_CONTROLLER = 3,
  GYRO_SENSOR = 4,
  ACCELEROMETER = 5,
  DISTANCE_SENSOR = 6,
  ENCODER = 7,
  POWER_DISTRIBUTION = 8,
  PNEUMATICS_CONTROLLER = 9,
  MISC_CAN_DEVICE = 10,
  IO_BREAKOUT = 11,
  SERVO_CONTROLLER = 12,
  COLOR_SENSOR = 13,
  RESERVED_14 = 14,
  RESERVED_15 = 15,
  RESERVED_16 = 16,
  RESERVED_17 = 17,
  RESERVED_18 = 18,
  RESERVED_19 = 19,
  RESERVED_20 = 20,
  RESERVED_21 = 21,
  RESERVED_22 = 22,
  RESERVED_23 = 23,
  RESERVED_24 = 24,
  RESERVED_25 = 25,
  RESERVED_26 = 26,
  RESERVED_27 = 27,
  RESERVED_28 = 28,
  RESERVED_29 = 29,
  RESERVED_30 = 30,
  FIRMWARE_UPDATE = 31
};

// Manufacturer IDs from FRC CAN specification
enum FRCManufacturer {
  BROADCAST_MFR = 0,
  NI = 1,
  LUMINARY_MICRO = 2,
  DEKA = 3,
  CTR_ELECTRONICS = 4,
  REV_ROBOTICS = 5,
  GRAPPLE = 6,
  MINDSENSORS = 7,
  TEAM_USE = 8,
  KAUAI_LABS = 9,
  COPPERFORGE = 10,
  PLAYING_WITH_FUSION = 11,
  STUDICA = 12,
  THE_THRIFTY_BOT = 13,
  REDUX_ROBOTICS = 14,
  ANDYMARK = 15,
  VIVID_HOSTING = 16,
  VERTOS_ROBOTICS = 17,
  SWYFT_ROBOTICS = 18,
  LUMYN_LABS = 19,
  BRUSHLAND_LABS = 20
};

struct FRCCANIdentifier {
  uint8_t deviceType;
  uint8_t manufacturer;
  uint8_t apiClass;
  uint8_t apiIndex;
  uint8_t deviceNumber;
};

// Decode FRC CAN ID (29-bit extended frame format)
// Bit layout: [28:24]=Device Type, [23:16]=Manufacturer, [15:10]=API Class, [9:6]=API Index, [5:0]=Device Number
FRCCANIdentifier decodeFRCCANID(uint32_t canId) {
  FRCCANIdentifier id;
  id.deviceType = (canId >> 24) & 0x1F;    // bits 28-24 (5 bits)
  id.manufacturer = (canId >> 16) & 0xFF;   // bits 23-16 (8 bits)
  id.apiClass = (canId >> 10) & 0x3F;       // bits 15-10 (6 bits)
  id.apiIndex = (canId >> 6) & 0x0F;        // bits 9-6 (4 bits)
  id.deviceNumber = canId & 0x3F;           // bits 5-0 (6 bits)
  return id;
}

const char* getDeviceTypeName(uint8_t type) {
  switch(type) {
    case BROADCAST: return "Broadcast";
    case ROBOT_CONTROLLER: return "Robot Controller";
    case MOTOR_CONTROLLER: return "Motor Controller";
    case RELAY_CONTROLLER: return "Relay Controller";
    case GYRO_SENSOR: return "Gyro Sensor";
    case ACCELEROMETER: return "Accelerometer";
    case DISTANCE_SENSOR: return "Distance Sensor";
    case ENCODER: return "Encoder";
    case POWER_DISTRIBUTION: return "Power Distribution";
    case PNEUMATICS_CONTROLLER: return "Pneumatics Controller";
    case MISC_CAN_DEVICE: return "Misc CAN Device";
    case IO_BREAKOUT: return "I/O Breakout";
    case SERVO_CONTROLLER: return "Servo Controller";
    case COLOR_SENSOR: return "Color Sensor";
    case FIRMWARE_UPDATE: return "Firmware Update";
    default: return "Reserved/Unknown";
  }
}

const char* getManufacturerName(uint8_t mfr) {
  switch(mfr) {
    case BROADCAST_MFR: return "Broadcast";
    case NI: return "National Instruments";
    case LUMINARY_MICRO: return "Luminary Micro";
    case DEKA: return "DEKA/FIRST";
    case CTR_ELECTRONICS: return "CTR Electronics";
    case REV_ROBOTICS: return "REV Robotics";
    case GRAPPLE: return "Grapple";
    case MINDSENSORS: return "MindSensors";
    case TEAM_USE: return "Team Use";
    case KAUAI_LABS: return "Kauai Labs";
    case COPPERFORGE: return "Copperforge";
    case PLAYING_WITH_FUSION: return "Playing With Fusion";
    case STUDICA: return "Studica";
    case THE_THRIFTY_BOT: return "The Thrifty Bot";
    case REDUX_ROBOTICS: return "Redux Robotics";
    case ANDYMARK: return "AndyMark";
    case VIVID_HOSTING: return "Vivid Hosting";
    case VERTOS_ROBOTICS: return "Vertos Robotics";
    case SWYFT_ROBOTICS: return "SWYFT Robotics";
    case LUMYN_LABS: return "Lumyn Labs";
    case BRUSHLAND_LABS: return "Brushland Labs";
    default: return "Unknown";
  }
}

// ========= Device Tracking =========
struct CANDevice {
  uint32_t canId;
  FRCCANIdentifier frcId;
  uint32_t messageCount;
  uint32_t lastSeenMs;
  uint32_t firstSeenMs;
  bool isExtended;
  uint8_t lastDLC;
  uint64_t lastData;
};

std::map<uint32_t, CANDevice> g_devices;
SemaphoreHandle_t g_devicesMutex;

// ========= Robot Enable State =========
struct RobotEnableState {
  bool isEnabled;
  uint32_t lastEnableHeartbeatMs;
  bool heartbeatActive;
};

RobotEnableState g_robotState = {false, 0, false};
SemaphoreHandle_t g_robotStateMutex;

// ========= Statistics =========
struct CANStats {
  uint32_t totalMessages;
  uint32_t busOffErrors;
  uint32_t arbLostErrors;
  uint32_t rxQueueFull;
  uint32_t txQueueFull;
  uint32_t rxErrors;
  uint32_t txErrors;
  uint32_t startTimeMs;
};

CANStats g_stats = {0};
SemaphoreHandle_t g_statsMutex;

// ========= Log Storage =========
#define MAX_LOG_ENTRIES 500
struct LogEntry {
  uint32_t timestamp;
  uint32_t canId;
  bool isExtended;
  bool isRTR;
  uint8_t dlc;
  uint8_t data[8];
  bool isError;
  char errorMsg[32];
};

std::vector<LogEntry> g_logBuffer;
SemaphoreHandle_t g_logMutex;

// ========= Web Server =========
WebServer server(80);

// ========= CAN Initialization =========
bool canInit() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(
    (gpio_num_t)CAN_TX_PIN, 
    (gpio_num_t)CAN_RX_PIN, 
    TWAI_MODE_NORMAL
  );
  g_config.tx_queue_len = 20;
  g_config.rx_queue_len = 50;
  g_config.alerts_enabled = TWAI_ALERT_ALL;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    return false;
  }
  return true;
}

// ========= Log Entry Management =========
void addLogEntry(const twai_message_t& msg, bool isError = false, const char* errMsg = nullptr) {
  if (xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    LogEntry entry;
    entry.timestamp = millis();
    entry.canId = msg.identifier;
    entry.isExtended = msg.extd;
    entry.isRTR = msg.rtr;
    entry.dlc = msg.data_length_code;
    memcpy(entry.data, msg.data, 8);
    entry.isError = isError;
    if (errMsg) {
      strncpy(entry.errorMsg, errMsg, sizeof(entry.errorMsg) - 1);
      entry.errorMsg[sizeof(entry.errorMsg) - 1] = '\0';
    } else {
      entry.errorMsg[0] = '\0';
    }
    
    if (g_logBuffer.size() >= MAX_LOG_ENTRIES) {
      g_logBuffer.erase(g_logBuffer.begin());
    }
    g_logBuffer.push_back(entry);
    xSemaphoreGive(g_logMutex);
  }
}

// ========= Robot Enable Signal Detection =========
// Check if message is from RoboRIO and contains enable signal
bool isRobotControllerEnableMessage(const twai_message_t& msg) {
  if (!msg.extd) return false; // Must be extended frame
  
  FRCCANIdentifier id = decodeFRCCANID(msg.identifier);
  
  // Check if it's from Robot Controller (device type 1)
  // and manufacturer is NI (National Instruments, value 1)
  if (id.deviceType == ROBOT_CONTROLLER && id.manufacturer == NI) {
    return true;
  }
  
  return false;
}

// Extract enable state from RoboRIO CAN message
bool extractEnableState(const twai_message_t& msg) {
  // The enable state is typically in the first byte of RoboRIO status messages
  // Bit 0 of data[0] indicates enabled state
  if (msg.data_length_code > 0) {
    return (msg.data[0] & 0x01) != 0;
  }
  return false;
}

// ========= Relay Control Task =========
void RelayControlTask(void* param) {
  uint32_t lastFlashToggle = 0;
  bool flashState = false;
  
  while (true) {
    bool shouldBeOn = false;
    bool shouldFlash = false;
    
    if (xSemaphoreTake(g_robotStateMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      uint32_t timeSinceHeartbeat = millis() - g_robotState.lastEnableHeartbeatMs;
      
      if (g_robotState.heartbeatActive && timeSinceHeartbeat < ENABLE_HEARTBEAT_TIMEOUT_MS) {
        // Heartbeat is active and recent
        if (g_robotState.isEnabled) {
          shouldBeOn = true; // Solid ON when enabled
        } else {
          shouldBeOn = false; // Solid OFF when disabled but heartbeat present
        }
      } else {
        // No heartbeat or stale heartbeat - flash the relay
        shouldFlash = true;
      }
      
      xSemaphoreGive(g_robotStateMutex);
    }
    
    if (shouldFlash) {
      // Flash relay at defined interval
      if (millis() - lastFlashToggle >= FLASH_INTERVAL_MS) {
        flashState = !flashState;
        digitalWrite(RELAY_PIN, flashState ? HIGH : LOW);
        lastFlashToggle = millis();
      }
    } else {
      // Solid state
      digitalWrite(RELAY_PIN, shouldBeOn ? HIGH : LOW);
      flashState = false;
      lastFlashToggle = millis();
    }
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ========= CAN Monitoring Task =========
void CANMonitorTask(void* param) {
  twai_message_t msg;
  uint32_t alerts;
  
  while (true) {
    // Check for alerts
    if (twai_read_alerts(&alerts, 0) == ESP_OK) {
      if (xSemaphoreTake(g_statsMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        if (alerts & TWAI_ALERT_BUS_OFF) g_stats.busOffErrors++;
        if (alerts & TWAI_ALERT_ARB_LOST) g_stats.arbLostErrors++;
        if (alerts & TWAI_ALERT_RX_QUEUE_FULL) g_stats.rxQueueFull++;
        if (alerts & TWAI_ALERT_TX_FAILED) g_stats.txQueueFull++;
        if (alerts & TWAI_ALERT_RX_DATA) g_stats.rxErrors++;
        xSemaphoreGive(g_statsMutex);
      }
      
      // Log errors
      if (alerts & TWAI_ALERT_BUS_OFF) {
        twai_message_t dummy = {0};
        addLogEntry(dummy, true, "BUS OFF ERROR");
        twai_initiate_recovery();
      }
    }
    
    // Receive messages
    esp_err_t result = twai_receive(&msg, pdMS_TO_TICKS(10));
    if (result == ESP_OK) {
      // Check for Robot Controller enable messages
      if (isRobotControllerEnableMessage(msg)) {
        bool enableState = extractEnableState(msg);
        if (xSemaphoreTake(g_robotStateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
          g_robotState.isEnabled = enableState;
          g_robotState.lastEnableHeartbeatMs = millis();
          g_robotState.heartbeatActive = true;
          xSemaphoreGive(g_robotStateMutex);
        }
      }
      
      // Update statistics
      if (xSemaphoreTake(g_statsMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        g_stats.totalMessages++;
        xSemaphoreGive(g_statsMutex);
      }
      
      // Track device
      if (xSemaphoreTake(g_devicesMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        uint32_t id = msg.identifier;
        if (g_devices.find(id) == g_devices.end()) {
          // New device
          CANDevice dev;
          dev.canId = id;
          dev.frcId = decodeFRCCANID(id);
          dev.messageCount = 1;
          dev.firstSeenMs = millis();
          dev.lastSeenMs = millis();
          dev.isExtended = msg.extd;
          dev.lastDLC = msg.data_length_code;
          dev.lastData = 0;
          for (int i = 0; i < msg.data_length_code && i < 8; i++) {
            dev.lastData |= ((uint64_t)msg.data[i]) << (i * 8);
          }
          g_devices[id] = dev;
        } else {
          // Update existing device
          g_devices[id].messageCount++;
          g_devices[id].lastSeenMs = millis();
          g_devices[id].lastDLC = msg.data_length_code;
          g_devices[id].lastData = 0;
          for (int i = 0; i < msg.data_length_code && i < 8; i++) {
            g_devices[id].lastData |= ((uint64_t)msg.data[i]) << (i * 8);
          }
        }
        xSemaphoreGive(g_devicesMutex);
      }
      
      // Add to log
      addLogEntry(msg);
    }
    
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}

// ========= Web Interface HTML =========
// inline cause i'm lazy
const char HTML_INDEX[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>adalogger</title>
  <style>
    * { margin: 0; padding: 0; box-sizing: border-box; }
    body { 
      font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
      background: #333;
      color: #fff;
      padding: 20px;
      min-height: 100vh;
    }
    .container { max-width: 1200px; margin: 0 auto; }
    h1 { 
      font-size: 2em; 
      margin-bottom: 10px;
      text-shadow: 2px 2px 4px rgba(0,0,0,0.3);
    }
    .subtitle { 
      opacity: 0.9; 
      margin-bottom: 30px;
      font-size: 1.1em;
    }
    .stats-grid {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 15px;
      margin-bottom: 30px;
    }
    .stat-card {
      background: rgba(255,255,255,0.15);
      backdrop-filter: blur(10px);
      border-radius: 12px;
      padding: 20px;
      border: 1px solid rgba(255,255,255,0.2);
    }
    .stat-value { font-size: 2em; font-weight: bold; margin-top: 5px; }
    .stat-label { opacity: 0.8; font-size: 0.9em; }
    .section {
      background: rgba(255,255,255,0.1);
      backdrop-filter: blur(10px);
      border-radius: 12px;
      padding: 25px;
      margin-bottom: 20px;
      border: 1px solid rgba(255,255,255,0.2);
    }
    .section h2 { margin-bottom: 20px; font-size: 1.5em; }
    table {
      width: 100%;
      border-collapse: collapse;
      background: rgba(0,0,0,0.2);
      border-radius: 8px;
      overflow: hidden;
    }
    th, td {
      padding: 12px;
      text-align: left;
      border-bottom: 1px solid rgba(255,255,255,0.1);
    }
    th {
      background: rgba(0,0,0,0.3);
      font-weight: 600;
      text-transform: uppercase;
      font-size: 0.85em;
      letter-spacing: 0.5px;
    }
    tr:hover { background: rgba(255,255,255,0.05); }
    .device-id { 
      font-family: 'Courier New', monospace; 
      font-weight: bold;
      font-size: 1.1em;
    }
    .btn {
      background: rgba(255,255,255,0.2);
      color: #fff;
      border: 1px solid rgba(255,255,255,0.3);
      padding: 12px 24px;
      border-radius: 8px;
      cursor: pointer;
      font-size: 1em;
      margin: 5px;
      transition: all 0.3s;
    }
    .btn:hover {
      background: rgba(255,255,255,0.3);
      transform: translateY(-2px);
    }
    .error { color: #ff6b6b; font-weight: bold; }
    .active { color: #51cf66; }
    .inactive { color: #ffd43b; opacity: 0.7; }
    .relay-status {
      display: inline-block;
      padding: 8px 16px;
      border-radius: 6px;
      font-weight: bold;
      margin-left: 10px;
    }
    .relay-enabled { background: #51cf66; color: #000; }
    .relay-disabled { background: #868e96; color: #fff; }
    .relay-flashing { background: #ffd43b; color: #000; animation: pulse 1s infinite; }
    @keyframes pulse {
      0%, 100% { opacity: 1; }
      50% { opacity: 0.5; }
    }
    @media (max-width: 768px) {
      body { padding: 10px; }
      h1 { font-size: 1.5em; }
      .stats-grid { grid-template-columns: 1fr 1fr; }
      table { font-size: 0.85em; }
      th, td { padding: 8px; }
    }
  </style>
</head>
<body>
  <div class="container">
    <h1>adalogger</h1>
    <p class="subtitle">2036 is fancy now</p>
    
    <div class="stats-grid">
      <div class="stat-card">
        <div class="stat-label">Total Messages</div>
        <div class="stat-value" id="totalMsg">0</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">Active Devices</div>
        <div class="stat-value" id="deviceCount">0</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">Known Bus Errors</div>
        <div class="stat-value error" id="busErrors">0</div>
      </div>
      <div class="stat-card">
        <div class="stat-label">Uptime</div>
        <div class="stat-value" id="uptime">0s</div>
      </div>
    </div>
    
    <div class="section">
      <h2>Robot Status</h2>
      <table>
        <tr>
          <td><strong>Enable State</strong></td>
          <td><span id="enableState">Unknown</span></td>
        </tr>
        <tr>
          <td><strong>Heartbeat</strong></td>
          <td><span id="heartbeatState">No Signal</span></td>
        </tr>
        <tr>
          <td><strong>Relay Output</strong></td>
          <td><span id="relayState" class="relay-status relay-disabled">Unknown</span></td>
        </tr>
      </table>
    </div>
    
    <div class="section">
      <h2>Connected Devices</h2>
      <div style="overflow-x: auto;">
        <table>
          <thead>
            <tr>
              <th>CAN ID</th>
              <th>Device Type</th>
              <th>Manufacturer</th>
              <th>Device #</th>
              <th>Messages</th>
              <th>Last Seen</th>
              <th>Status</th>
            </tr>
          </thead>
          <tbody id="devicesTable">
            <tr><td colspan="7" style="text-align: center; opacity: 0.5;">No devices detected yet...</td></tr>
          </tbody>
        </table>
      </div>
    </div>
    
    <div class="section">
      <h2>Detected Errors</h2>
      <table>
        <tr><td>Bus Off Errors</td><td id="busOffErr">0</td></tr>
        <tr><td>Arbitration Lost</td><td id="arbLostErr">0</td></tr>
        <tr><td>RX Queue Full</td><td id="rxQueueErr">0</td></tr>
        <tr><td>TX Queue Full</td><td id="txQueueErr">0</td></tr>
      </table>
    </div>
    
    <div style="text-align: center;">
      <button class="btn" onclick="downloadLogs()">Download Logs (CSV)</button>
      <button class="btn" onclick="clearLogs()">Clear Logs</button>
      <button class="btn" onclick="location.reload()">Refresh</button>
    </div>
  </div>
  
  <script>
    function updateData() {
      fetch('/api/status')
        .then(r => r.json())
        .then(data => {
          document.getElementById('totalMsg').textContent = data.totalMessages.toLocaleString();
          document.getElementById('deviceCount').textContent = data.deviceCount;
          document.getElementById('busErrors').textContent = data.totalErrors;
          document.getElementById('uptime').textContent = formatUptime(data.uptimeMs);
          document.getElementById('busOffErr').textContent = data.busOffErrors;
          document.getElementById('arbLostErr').textContent = data.arbLostErrors;
          document.getElementById('rxQueueErr').textContent = data.rxQueueFull;
          document.getElementById('txQueueErr').textContent = data.txQueueFull;
          
          // Update robot enable state
          const enableEl = document.getElementById('enableState');
          if (data.robotEnabled) {
            enableEl.innerHTML = '<span class="active">●</span> ENABLED';
          } else {
            enableEl.innerHTML = '<span style="color:#868e96">●</span> DISABLED';
          }
          
          // Update heartbeat state
          const heartbeatEl = document.getElementById('heartbeatState');
          if (data.heartbeatActive) {
            heartbeatEl.innerHTML = '<span class="active">●</span> Active';
          } else {
            heartbeatEl.innerHTML = '<span class="error">●</span> Lost';
          }
          
          // Update relay state
          const relayEl = document.getElementById('relayState');
          relayEl.className = 'relay-status';
          if (!data.heartbeatActive) {
            relayEl.className += ' relay-flashing';
            relayEl.textContent = 'FLASHING (No Heartbeat)';
          } else if (data.robotEnabled) {
            relayEl.className += ' relay-enabled';
            relayEl.textContent = 'ON (Enabled)';
          } else {
            relayEl.className += ' relay-disabled';
            relayEl.textContent = 'OFF (Disabled)';
          }
        });
      
      fetch('/api/devices')
        .then(r => r.json())
        .then(data => {
          const tbody = document.getElementById('devicesTable');
          if (data.devices.length === 0) {
            tbody.innerHTML = '<tr><td colspan="7" style="text-align: center; opacity: 0.5;">No devices detected yet...</td></tr>';
            return;
          }
          
          tbody.innerHTML = data.devices.map(dev => {
            const age = Date.now() - dev.lastSeenMs;
            const status = age < 2000 ? '<span class="active">●</span> Active' : 
                          age < 10000 ? '<span class="inactive">●</span> Recent' : 
                          '<span style="color:#868e96">●</span> Idle';
            return `
              <tr>
                <td class="device-id">0x${dev.canId.toString(16).toUpperCase().padStart(8, '0')}</td>
                <td>${dev.deviceType}</td>
                <td>${dev.manufacturer}</td>
                <td>${dev.deviceNumber}</td>
                <td>${dev.messageCount.toLocaleString()}</td>
                <td>${formatTime(age)}</td>
                <td>${status}</td>
              </tr>
            `;
          }).join('');
        });
    }
    
    function formatUptime(ms) {
      const s = Math.floor(ms / 1000);
      const m = Math.floor(s / 60);
      const h = Math.floor(m / 60);
      if (h > 0) return h + 'h ' + (m % 60) + 'm';
      if (m > 0) return m + 'm ' + (s % 60) + 's';
      return s + 's';
    }
    
    function formatTime(ms) {
      if (ms < 1000) return 'just now';
      const s = Math.floor(ms / 1000);
      if (s < 60) return s + 's ago';
      const m = Math.floor(s / 60);
      if (m < 60) return m + 'm ago';
      return Math.floor(m / 60) + 'h ago';
    }
    
    function downloadLogs() {
      window.location.href = '/api/logs/download';
    }
    
    function clearLogs() {
      if (confirm('Clear all stored logs?')) {
        fetch('/api/logs/clear', {method: 'POST'})
          .then(() => alert('Logs cleared!'));
      }
    }
    
    updateData();
    setInterval(updateData, 1000);
  </script>
</body>
</html>
)rawliteral";

// ========= API Handlers =========
void handleRoot() {
  server.send(200, "text/html", HTML_INDEX);
}

void handleAPIStatus() {
  String json = "{";
  
  if (xSemaphoreTake(g_statsMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    json += "\"totalMessages\":" + String(g_stats.totalMessages) + ",";
    json += "\"busOffErrors\":" + String(g_stats.busOffErrors) + ",";
    json += "\"arbLostErrors\":" + String(g_stats.arbLostErrors) + ",";
    json += "\"rxQueueFull\":" + String(g_stats.rxQueueFull) + ",";
    json += "\"txQueueFull\":" + String(g_stats.txQueueFull) + ",";
    json += "\"totalErrors\":" + String(g_stats.busOffErrors + g_stats.arbLostErrors + 
                                        g_stats.rxQueueFull + g_stats.txQueueFull) + ",";
    json += "\"uptimeMs\":" + String(millis() - g_stats.startTimeMs);
    xSemaphoreGive(g_statsMutex);
  }
  
  if (xSemaphoreTake(g_devicesMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
    json += ",\"deviceCount\":" + String(g_devices.size());
    xSemaphoreGive(g_devicesMutex);
  }
  
  json += "}";
  server.send(200, "application/json", json);
}

void handleAPIDevices() {
  String json = "{\"devices\":[";
  
  if (xSemaphoreTake(g_devicesMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    bool first = true;
    for (auto& pair : g_devices) {
      if (!first) json += ",";
      first = false;
      
      CANDevice& dev = pair.second;
      json += "{";
      json += "\"canId\":" + String(dev.canId) + ",";
      json += "\"deviceType\":\"" + String(getDeviceTypeName(dev.frcId.deviceType)) + "\",";
      json += "\"manufacturer\":\"" + String(getManufacturerName(dev.frcId.manufacturer)) + "\",";
      json += "\"deviceNumber\":" + String(dev.frcId.deviceNumber) + ",";
      json += "\"messageCount\":" + String(dev.messageCount) + ",";
      json += "\"lastSeenMs\":" + String(dev.lastSeenMs);
      json += "}";
    }
    xSemaphoreGive(g_devicesMutex);
  }
  
  json += "]}";
  server.send(200, "application/json", json);
}

void handleLogsDownload() {
  String csv = "Timestamp,CAN_ID,Extended,RTR,DLC,Data,Error,ErrorMsg\n";
  
  if (xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    for (const auto& entry : g_logBuffer) {
      csv += String(entry.timestamp) + ",";
      csv += "0x" + String(entry.canId, HEX) + ",";
      csv += String(entry.isExtended ? "1" : "0") + ",";
      csv += String(entry.isRTR ? "1" : "0") + ",";
      csv += String(entry.dlc) + ",";
      
      // Data bytes - Fixed: Store result in temporary String variable first
      for (int i = 0; i < 8; i++) {
        String hexByte = String(entry.data[i], HEX);
        hexByte.toUpperCase();
        csv += hexByte;
        if (i < 7) csv += " ";
      }
      csv += ",";
      
      csv += String(entry.isError ? "1" : "0") + ",";
      csv += String(entry.errorMsg) + "\n";
    }
    xSemaphoreGive(g_logMutex);
  }
  
  server.sendHeader("Content-Disposition", "attachment; filename=can_log.csv");
  server.send(200, "text/csv", csv);
}

void handleLogsClear() {
  if (xSemaphoreTake(g_logMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    g_logBuffer.clear();
    xSemaphoreGive(g_logMutex);
  }
  server.send(200, "text/plain", "Logs cleared");
}

// ========= Setup =========
void setup() {
  Serial.begin(115200);
  delay(150);
  Serial.println("\n\n=== ESP32 CAN Bus Logger ===");
  
  // Initialize mutexes
  g_devicesMutex = xSemaphoreCreateMutex();
  g_statsMutex = xSemaphoreCreateMutex();
  g_logMutex = xSemaphoreCreateMutex();
  
  // Initialize stats
  g_stats.startTimeMs = millis();
  
  // Start WiFi AP
  Serial.println("Starting WiFi AP...");
  WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET);
  WiFi.softAP(AP_SSID, AP_PASSWORD);
  Serial.println("WiFi AP Started!");
  Serial.print("SSID: ");
  Serial.println(AP_SSID);
  Serial.print("Password: ");
  Serial.println(AP_PASSWORD);
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());
  
  // Initialize CAN
  Serial.println("Initializing CAN bus...");
  if (!canInit()) {
    Serial.println("ERROR: CAN initialization failed!");
    Serial.println("Check transceiver connection and pins.");
  } else {
    Serial.println("CAN initialized @ 1 Mbps");
  }
  
  // Setup web server routes
  server.on("/", handleRoot);
  server.on("/api/status", handleAPIStatus);
  server.on("/api/devices", handleAPIDevices);
  server.on("/api/logs/download", handleLogsDownload);
  server.on("/api/logs/clear", HTTP_POST, handleLogsClear);
  
  server.begin();
  Serial.println("Web server started!");
  Serial.println("\n>>> Connect to WiFi and open http://192.168.4.1 <<<\n");
  
  // Start CAN monitoring task
  xTaskCreatePinnedToCore(CANMonitorTask, "CANMonitor", 8192, nullptr, 2, nullptr, 1);
}

void loop() {
  server.handleClient();
  delay(2); // Small delay to prevent watchdog issues
}
