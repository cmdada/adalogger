#include <WiFi.h>
#include <WebServer.h>
#include <driver/twai.h>
#include <ArduinoJson.h>

// === WiFi Configuration ===
const char* ap_ssid = "adalog";
const char* ap_password = "Team1157!";  

// === CAN Pin Configuration ===
#define CAN_TX_PIN 4
#define CAN_RX_PIN 5

// === LED Pins ===
#define LED_R 15
#define LED_G 13
#define LED_B 14

// === FRC CAN Constants ===
#define HEARTBEAT_ID 0x01011840

// === Web Server ===
WebServer server(80);

// === CAN Message Buffer ===
#define MAX_MESSAGES 500
struct CANMessage {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  uint32_t timestamp;
  bool extended;
  bool error;
};

CANMessage messageBuffer[MAX_MESSAGES];
int bufferIndex = 0;
int totalMessages = 0;
bool bufferOverflow = false;

// === Device Tracking ===
#define MAX_DEVICES 64
struct DeviceInfo {
  uint32_t canID;
  uint8_t deviceID;
  uint8_t manufacturerID;
  uint16_t apiID;
  uint8_t deviceNumber;
  uint32_t messageCount;
  uint32_t lastSeen;
  bool active;
};

DeviceInfo devices[MAX_DEVICES];
int deviceCount = 0;

// === Statistics ===
uint32_t rxCount = 0;
uint32_t txCount = 0;
uint32_t errCount = 0;
uint32_t lastHeartbeat = 0;

// === FRC CAN Decoding ===
struct DecodedCANID {
  uint8_t deviceID;
  uint8_t manufacturerID;
  uint16_t apiID;
  uint8_t deviceNumber;
};

DecodedCANID decodeCANMsgID(uint32_t can_id) {
  DecodedCANID decoded;
  decoded.deviceID = (can_id >> 24) & 0xFF;
  decoded.manufacturerID = (can_id >> 16) & 0xFF;
  decoded.apiID = (can_id >> 6) & 0x3FF;
  decoded.deviceNumber = can_id & 0x3F;
  return decoded;
}

const char* getDeviceTypeName(uint8_t deviceID) {
  switch(deviceID) {
    case 0x00: return "Broadcast";
    case 0x01: return "RoboRIO";
    case 0x02: return "Motor Controller";
    case 0x03: return "Relay";
    case 0x04: return "Gyro";
    case 0x05: return "Accelerometer";
    case 0x06: return "Ultrasonic";
    case 0x07: return "Gear Tooth";
    case 0x08: return "Power Distribution";
    case 0x09: return "Pneumatics";
    case 0x0A: return "Miscellaneous";
    case 0x0B: return "IO Breakout";
    default: return "Unknown";
  }
}

const char* getManufacturerName(uint8_t mfrID) {
  switch(mfrID) {
    case 0x00: return "Broadcast";
    case 0x01: return "NI";
    case 0x02: return "Luminary Micro";
    case 0x03: return "DEKA";
    case 0x04: return "CTR Electronics";
    case 0x05: return "REV Robotics";
    case 0x06: return "Grapple";
    case 0x07: return "MindSensors";
    case 0x08: return "Team Use";
    default: return "Unknown";
  }
}

void updateDeviceList(uint32_t canID) {
  if (canID == HEARTBEAT_ID) return;  // Skip heartbeat
  
  // Check if device already exists
  for (int i = 0; i < deviceCount; i++) {
    if (devices[i].canID == canID) {
      devices[i].messageCount++;
      devices[i].lastSeen = millis();
      devices[i].active = true;
      return;
    }
  }
  
  // Add new device
  if (deviceCount < MAX_DEVICES) {
    DecodedCANID decoded = decodeCANMsgID(canID);
    devices[deviceCount].canID = canID;
    devices[deviceCount].deviceID = decoded.deviceID;
    devices[deviceCount].manufacturerID = decoded.manufacturerID;
    devices[deviceCount].apiID = decoded.apiID;
    devices[deviceCount].deviceNumber = decoded.deviceNumber;
    devices[deviceCount].messageCount = 1;
    devices[deviceCount].lastSeen = millis();
    devices[deviceCount].active = true;
    deviceCount++;
  }
}

uint32_t makeCANMsgID(uint8_t deviceID, uint8_t manufacturerID, uint16_t apiID, uint8_t deviceNumber) {
  return ((uint32_t)(deviceID & 0xFF) << 24) | 
         ((uint32_t)(manufacturerID & 0xFF) << 16) | 
         ((uint32_t)(apiID & 0x3FF) << 6) | 
         (deviceNumber & 0x3F);
}

// === Setup Functions ===
void setupCAN() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  // FRC uses 1Mbps
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
  
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("CAN driver installed");
  } else {
    Serial.println("Failed to install CAN driver");
    return;
  }
  
  if (twai_start() == ESP_OK) {
    Serial.println("CAN started");
  } else {
    Serial.println("Failed to start CAN");
  }
}

void setupWiFi() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ap_ssid, ap_password);
  
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
  
  setLED(0, 255, 0);  // Green = WiFi ready
}

void setLED(int r, int g, int b) {
  analogWrite(LED_R, 255 - r);
  analogWrite(LED_G, 255 - g);
  analogWrite(LED_B, 255 - b);
}

// === Web Server Handlers ===
void handleRoot() {
  String html = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>adalog</title>
  <style>
    @import url('https://fonts.googleapis.com/css2?family=Fira+Code:wght@300;400;500;600&display=swap');
    
    :root {
      --color-base: #0d181f;
      --color-surface: #1f1d2e;
      --color-overlay: #26233a;
      --color-muted: #6e6a86;
      --color-subtle: #908caa;
      --color-text: #e0def4;
      --color-love: #eb6f92;
      --color-gold: #f6c177;
      --color-rose: #ebbcba;
      --color-pine: #31748f;
      --color-foam: #9ccfd8;
      --color-iris: #c4a7e7;
      --hl-low: #21202e;
      --hl-med: #403d52;
      --hl-high: #524f67;
    }
    
    * { margin: 0; padding: 0; box-sizing: border-box; }
    
    body { 
      font-family: 'Fira Code', monospace;
      background: var(--color-base);
      color: var(--color-text);
      font-size: 14px;
      line-height: 1.6;
      position: relative;
      min-height: 100vh;
      overflow-x: hidden;
    }
    
    body::before {
      content: "";
      position: fixed;
      top: 0;
      left: 0;
      width: 100vw;
      height: 100vh;
      background: radial-gradient(circle at 50% 50%, rgba(156, 207, 216, 0.03) 0%, rgba(235, 111, 146, 0.02) 25%, rgba(196, 167, 231, 0.02) 50%, transparent 70%);
      pointer-events: none;
      z-index: 0;
      opacity: 0.6;
    }
    
    .layout {
      display: grid;
      grid-template-columns: 300px 1fr;
      grid-template-rows: auto 1fr;
      height: 100vh;
      position: relative;
      z-index: 1;
      gap: 0;
    }
    
    .glass {
      backdrop-filter: blur(12px);
      -webkit-backdrop-filter: blur(12px);
      background: rgba(255, 255, 255, 0.08);
      border: 1px solid rgba(255, 255, 255, 0.15);
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.1), inset 0 1px rgba(255, 255, 255, 0.2);
      position: relative;
    }
    
    .glass::before {
      content: "";
      position: absolute;
      top: 0;
      left: 0;
      right: 0;
      height: 1px;
      background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.4), transparent);
      pointer-events: none;
    }
    
    .header { 
      grid-column: 1 / -1;
      padding: 24px;
      margin: 20px;
      border-radius: 12px;
    }
    
    h1 { 
      color: var(--color-rose);
      font-weight: 500;
      margin-bottom: 8px;
      font-size: 28px;
      letter-spacing: 0.5px;
    }
    
    .subtitle {
      color: var(--color-subtle);
      font-size: 13px;
      font-weight: 300;
    }
    
    .sidebar {
      border-radius: 0 12px 12px 0;
      margin: 0 20px 20px 20px;
      padding: 20px;
      overflow-y: auto;
      max-height: calc(100vh - 180px);
    }
    
    .main-content {
      margin: 0 20px 20px 0;
      display: flex;
      flex-direction: column;
      gap: 20px;
      overflow-y: auto;
      max-height: calc(100vh - 180px);
      padding-right: 10px;
    }
    
    .stats {
      display: grid;
      grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
      gap: 16px;
    }
    
    .stat-card {
      padding: 20px;
      border-radius: 12px;
      transition: all 0.3s cubic-bezier(0.4, 0, 0.2, 1);
    }
    
    .stat-card:hover {
      transform: translateY(-2px);
      background: rgba(255, 255, 255, 0.12);
    }
    
    .stat-label { 
      color: var(--color-muted); 
      font-size: 11px;
      text-transform: uppercase;
      letter-spacing: 1px;
      margin-bottom: 8px;
    }
    
    .stat-value { 
      font-size: 32px; 
      font-weight: 600;
      background: linear-gradient(135deg, var(--color-foam), var(--color-iris));
      -webkit-background-clip: text;
      -webkit-text-fill-color: transparent;
      background-clip: text;
    }
    
    .controls {
      padding: 24px;
      border-radius: 12px;
    }
    
    h3 {
      color: var(--color-foam);
      font-weight: 400;
      margin-bottom: 16px;
      font-size: 16px;
    }
    
    .btn {
      background: rgba(255, 255, 255, 0.1);
      color: var(--color-text);
      border: 1px solid rgba(255, 255, 255, 0.2);
      padding: 10px 20px;
      border-radius: 8px;
      cursor: pointer;
      font-weight: 500;
      font-family: 'Fira Code', monospace;
      font-size: 13px;
      margin-right: 10px;
      margin-bottom: 10px;
      transition: all 0.3s ease;
      backdrop-filter: blur(8px);
      position: relative;
      overflow: hidden;
    }
    
    .btn::before {
      content: "";
      position: absolute;
      top: 0;
      left: -100%;
      width: 100%;
      height: 100%;
      background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.2), transparent);
      transition: left 0.5s ease;
    }
    
    .btn:hover {
      background: rgba(255, 255, 255, 0.15);
      transform: translateY(-2px);
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.1);
    }
    
    .btn:hover::before {
      left: 100%;
    }
    
    .btn-primary {
      background: linear-gradient(135deg, rgba(156, 207, 216, 0.2), rgba(156, 207, 216, 0.1));
      border-color: rgba(156, 207, 216, 0.3);
      color: var(--color-foam);
    }
    
    .btn-danger { 
      background: linear-gradient(135deg, rgba(235, 111, 146, 0.2), rgba(235, 111, 146, 0.1));
      border-color: rgba(235, 111, 146, 0.3);
      color: var(--color-love);
    }
    
    input, select {
      background: rgba(255, 255, 255, 0.05);
      color: var(--color-text);
      border: 1px solid rgba(255, 255, 255, 0.15);
      padding: 10px 14px;
      border-radius: 8px;
      margin-right: 10px;
      margin-bottom: 10px;
      font-family: 'Fira Code', monospace;
      font-size: 13px;
      backdrop-filter: blur(8px);
      transition: all 0.3s ease;
    }
    
    input:focus, select:focus {
      outline: none;
      border-color: var(--color-foam);
      background: rgba(255, 255, 255, 0.08);
      box-shadow: 0 0 0 3px rgba(156, 207, 216, 0.1);
    }
    
    .message-list {
      padding: 24px;
      border-radius: 12px;
      flex: 1;
    }
    
    .message-container {
      max-height: 500px;
      overflow-y: auto;
    }
    
    .message {
      background: rgba(255, 255, 255, 0.05);
      padding: 16px;
      margin-bottom: 12px;
      border-radius: 8px;
      border-left: 3px solid var(--color-foam);
      font-size: 12px;
      transition: all 0.2s ease;
    }
    
    .message:hover {
      background: rgba(255, 255, 255, 0.08);
      transform: translateX(4px);
    }
    
    .message.heartbeat { 
      border-left-color: var(--color-love);
      background: rgba(235, 111, 146, 0.05);
    }
    
    .message.error {
      border-left-color: var(--color-gold);
      background: rgba(246, 193, 119, 0.05);
    }
    
    .message-header {
      display: flex;
      justify-content: space-between;
      margin-bottom: 8px;
      color: var(--color-foam);
      font-weight: 500;
    }
    
    .message-data { 
      color: var(--color-text);
      font-weight: 400;
      margin-bottom: 6px;
    }
    
    .decoded { 
      color: var(--color-muted);
      font-size: 11px;
    }
    
    .filter-bar {
      display: flex;
      gap: 12px;
      margin-bottom: 16px;
      flex-wrap: wrap;
    }
    
    .device-item {
      background: rgba(255, 255, 255, 0.05);
      padding: 12px;
      margin-bottom: 8px;
      border-radius: 8px;
      cursor: pointer;
      transition: all 0.2s ease;
      border-left: 3px solid var(--color-foam);
    }
    
    .device-item:hover {
      background: rgba(255, 255, 255, 0.1);
      transform: translateX(4px);
    }
    
    .device-item.selected {
      background: rgba(156, 207, 216, 0.15);
      border-left-color: var(--color-iris);
    }
    
    .device-item.inactive {
      opacity: 0.5;
    }
    
    .device-name {
      color: var(--color-foam);
      font-weight: 500;
      margin-bottom: 4px;
      font-size: 13px;
    }
    
    .device-details {
      color: var(--color-muted);
      font-size: 11px;
    }
    
    .device-stats {
      margin-top: 6px;
      color: var(--color-subtle);
      font-size: 10px;
    }
    
    .divider {
      margin: 24px 0;
      border: none;
      height: 1px;
      background: linear-gradient(90deg, transparent, rgba(255, 255, 255, 0.3), transparent);
    }
    
    .checkbox-label {
      display: flex;
      align-items: center;
      gap: 8px;
      margin-bottom: 12px;
      cursor: pointer;
      color: var(--color-text);
      font-size: 13px;
    }
    
    .checkbox-label input[type="checkbox"] {
      width: 18px;
      height: 18px;
      cursor: pointer;
      margin: 0;
    }
    
    ::-webkit-scrollbar { width: 8px; }
    ::-webkit-scrollbar-track { background: rgba(255, 255, 255, 0.05); border-radius: 4px; }
    ::-webkit-scrollbar-thumb { 
      background: var(--color-foam); 
      border-radius: 4px;
      transition: background 0.3s ease;
    }
    ::-webkit-scrollbar-thumb:hover { background: var(--color-iris); }
    
    @media (max-width: 1024px) {
      .layout {
        grid-template-columns: 1fr;
        grid-template-rows: auto auto 1fr;
      }
      
      .sidebar {
        margin: 0 20px 20px 20px;
        border-radius: 12px;
        max-height: 300px;
      }
      
      .main-content {
        margin: 0 20px 20px 20px;
      }
    }
  </style>
</head>
<body>
  <div class="layout">
    <div class="glass header">
      <h1>adalog</h1>
      <p class="subtitle">https://adabit.org | FRC CAN Monitor</p>
    </div>

    <div class="glass sidebar">
      <h3>Devices</h3>
      <div style="margin-bottom: 16px;">
        <div class="device-item selected" onclick="selectDevice(null)" id="device-all">
          <div class="device-name">All Devices</div>
          <div class="device-details">Show all messages</div>
        </div>
      </div>
      <div id="deviceList"></div>
    </div>

    <div class="main-content">
      <div class="glass">
        <div class="stats">
          <div class="stat-card">
            <div class="stat-label">RX Messages</div>
            <div class="stat-value" id="rxCount">0</div>
          </div>
          <div class="stat-card">
            <div class="stat-label">TX Messages</div>
            <div class="stat-value" id="txCount">0</div>
          </div>
          <div class="stat-card">
            <div class="stat-label">Errors</div>
            <div class="stat-value" id="errCount">0</div>
          </div>
          <div class="stat-card">
            <div class="stat-label">Last Heartbeat</div>
            <div class="stat-value" id="heartbeat">Never</div>
          </div>
        </div>
      </div>

      <div class="glass controls">
        <h3>Controls</h3>
        <button class="btn" onclick="clearMessages()">Clear Messages</button>
        <button class="btn" onclick="toggleMonitoring()">
          <span id="monitorBtn">Pause</span>
        </button>
        <button class="btn btn-danger" onclick="resetStats()">Reset Stats</button>
        
        <div class="divider"></div>
        
        <h3>Send CAN Message</h3>
        <div>
          <input type="text" id="canId" placeholder="CAN ID (hex)" value="0x0A081801">
          <input type="text" id="canData" placeholder="Data (hex, space separated)" value="01 02 03 04">
          <button class="btn btn-primary" onclick="sendMessage()">Send</button>
        </div>
      </div>

      <div class="glass message-list">
        <h3>CAN Messages</h3>
        <div class="filter-bar">
          <input type="text" id="filterID" placeholder="Filter by ID (hex)" onkeyup="applyFilter()">
          <select id="filterType" onchange="applyFilter()">
            <option value="all">All Messages</option>
            <option value="heartbeat">Heartbeat Only</option>
            <option value="no-heartbeat">No Heartbeat</option>
          </select>
          <label class="checkbox-label">
            <input type="checkbox" id="filterErrors" onchange="applyFilter()">
            <span>Hide Errors</span>
          </label>
        </div>
        <div class="message-container" id="messages"></div>
      </div>
    </div>
  </div>

  <script>
    let monitoring = true;
    let allMessages = [];
    let allDevices = [];
    let selectedDevice = null;

    function updateStats() {
      fetch('/stats')
        .then(r => r.json())
        .then(data => {
          document.getElementById('rxCount').textContent = data.rx;
          document.getElementById('txCount').textContent = data.tx;
          document.getElementById('errCount').textContent = data.err;
          if (data.lastHeartbeat > 0) {
            let ago = ((Date.now() - data.lastHeartbeat) / 1000).toFixed(1);
            document.getElementById('heartbeat').textContent = ago + 's ago';
          }
        });
    }

    function updateDevices() {
      fetch('/devices')
        .then(r => r.json())
        .then(data => {
          allDevices = data;
          displayDevices();
        });
    }

    function displayDevices() {
      let html = '';
      allDevices.forEach((device, idx) => {
        let ago = ((Date.now() - device.lastSeen) / 1000).toFixed(1);
        let isActive = ago < 5;
        let isSelected = selectedDevice === device.id;
        
        html += `
          <div class="device-item ${isActive ? '' : 'inactive'} ${isSelected ? 'selected' : ''}" 
               onclick="selectDevice('${device.id}')" 
               id="device-${idx}">
            <div class="device-name">${device.name}</div>
            <div class="device-details">
              ${device.manufacturer} | Device #${device.deviceNum}
            </div>
            <div class="device-stats">
              ${device.msgCount} msgs | Last: ${ago}s ago
            </div>
          </div>
        `;
      });
      document.getElementById('deviceList').innerHTML = html;
    }

    function selectDevice(deviceId) {
      selectedDevice = deviceId;
      
      // Update UI
      document.querySelectorAll('.device-item').forEach(el => {
        el.classList.remove('selected');
      });
      
      if (deviceId === null) {
        document.getElementById('device-all').classList.add('selected');
      } else {
        let idx = allDevices.findIndex(d => d.id === deviceId);
        if (idx >= 0) {
          document.getElementById('device-' + idx).classList.add('selected');
        }
      }
      
      applyFilter();
    }

    function updateMessages() {
      if (!monitoring) return;
      
      fetch('/messages')
        .then(r => r.json())
        .then(data => {
          allMessages = data;
          applyFilter();
        });
    }

    function applyFilter() {
      let filterID = document.getElementById('filterID').value.toLowerCase();
      let filterType = document.getElementById('filterType').value;
      let hideErrors = document.getElementById('filterErrors').checked;
      
      let filtered = allMessages.filter(msg => {
        // Device filter
        if (selectedDevice !== null && msg.id !== selectedDevice) {
          return false;
        }
        
        // ID filter
        let idMatch = !filterID || msg.id.toLowerCase().includes(filterID);
        
        // Type filter
        let typeMatch = filterType === 'all' || 
                       (filterType === 'heartbeat' && msg.id === '0x01011840') ||
                       (filterType === 'no-heartbeat' && msg.id !== '0x01011840');
        
        // Error filter
        let errorMatch = !hideErrors || !msg.error;
        
        return idMatch && typeMatch && errorMatch;
      });
      
      displayMessages(filtered);
    }

    function displayMessages(messages) {
      let html = '';
      messages.slice(-100).reverse().forEach(msg => {
        let isHeartbeat = msg.id === '0x01011840';
        let isError = msg.error || false;
        let classes = isHeartbeat ? 'heartbeat' : (isError ? 'error' : '');
        
        html += `
          <div class="message ${classes}">
            <div class="message-header">
              <span>ID: ${msg.id} ${isHeartbeat ? '(HEARTBEAT)' : ''} ${isError ? '(ERROR)' : ''}</span>
              <span>${msg.timestamp}ms</span>
            </div>
            <div class="message-data">
              Data [${msg.len}]: ${msg.data}
            </div>
            ${msg.decoded ? `<div class="decoded">${msg.decoded}</div>` : ''}
          </div>
        `;
      });
      document.getElementById('messages').innerHTML = html || '<p style="color:var(--color-muted)">No messages</p>';
    }

    function clearMessages() {
      fetch('/clear', {method: 'POST'});
      allMessages = [];
      applyFilter();
    }

    function toggleMonitoring() {
      monitoring = !monitoring;
      document.getElementById('monitorBtn').textContent = monitoring ? 'Pause' : 'Resume';
    }

    function resetStats() {
      fetch('/reset', {method: 'POST'});
    }

    function sendMessage() {
      let id = document.getElementById('canId').value;
      let data = document.getElementById('canData').value;
      
      fetch('/send', {
        method: 'POST',
        headers: {'Content-Type': 'application/x-www-form-urlencoded'},
        body: `id=${encodeURIComponent(id)}&data=${encodeURIComponent(data)}`
      })
      .then(r => r.text())
      .then(msg => alert(msg));
    }

    setInterval(updateStats, 1000);
    setInterval(updateMessages, 500);
    setInterval(updateDevices, 1000);
    updateStats();
    updateMessages();
    updateDevices();
  </script>
</body>
</html>
)rawliteral";
  
  server.send(200, "text/html", html);
}

void handleStats() {
  StaticJsonDocument<200> doc;
  doc["rx"] = rxCount;
  doc["tx"] = txCount;
  doc["err"] = errCount;
  doc["lastHeartbeat"] = lastHeartbeat;
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleDevices() {
  DynamicJsonDocument doc(8192);
  JsonArray devicesArray = doc.to<JsonArray>();
  
  for (int i = 0; i < deviceCount; i++) {
    JsonObject dev = devicesArray.createNestedObject();
    
    char idStr[16];
    sprintf(idStr, "0x%08X", devices[i].canID);
    dev["id"] = idStr;
    
    dev["name"] = getDeviceTypeName(devices[i].deviceID);
    dev["manufacturer"] = getManufacturerName(devices[i].manufacturerID);
    dev["deviceNum"] = devices[i].deviceNumber;
    dev["msgCount"] = devices[i].messageCount;
    dev["lastSeen"] = devices[i].lastSeen;
    dev["active"] = devices[i].active;
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleMessages() {
  DynamicJsonDocument doc(20000);
  JsonArray messages = doc.to<JsonArray>();
  
  for (int i = 0; i < min(bufferIndex, MAX_MESSAGES); i++) {
    JsonObject msg = messages.createNestedObject();
    
    char idStr[16];
    sprintf(idStr, "0x%08X", messageBuffer[i].id);
    msg["id"] = idStr;
    
    String dataStr = "";
    for (int j = 0; j < messageBuffer[i].len; j++) {
      char byte[4];
      sprintf(byte, "%02X ", messageBuffer[i].data[j]);
      dataStr += byte;
    }
    msg["data"] = dataStr;
    msg["len"] = messageBuffer[i].len;
    msg["timestamp"] = messageBuffer[i].timestamp;
    msg["error"] = messageBuffer[i].error;
    
    // Decode FRC CAN if not heartbeat
    if (messageBuffer[i].id != HEARTBEAT_ID && !messageBuffer[i].error) {
      DecodedCANID decoded = decodeCANMsgID(messageBuffer[i].id);
      char decodedStr[128];
      sprintf(decodedStr, "%s | %s | API:0x%03X | #%d", 
              getDeviceTypeName(decoded.deviceID),
              getManufacturerName(decoded.manufacturerID), 
              decoded.apiID, decoded.deviceNumber);
      msg["decoded"] = decodedStr;
    }
  }
  
  String response;
  serializeJson(doc, response);
  server.send(200, "application/json", response);
}

void handleClear() {
  bufferIndex = 0;
  totalMessages = 0;
  bufferOverflow = false;
  server.send(200, "text/plain", "Messages cleared");
}

void handleReset() {
  rxCount = 0;
  txCount = 0;
  errCount = 0;
  lastHeartbeat = 0;
  deviceCount = 0;
  server.send(200, "text/plain", "Stats reset");
}

void handleSend() {
  if (server.hasArg("id") && server.hasArg("data")) {
    String idStr = server.arg("id");
    String dataStr = server.arg("data");
    
    uint32_t canId = strtoul(idStr.c_str(), NULL, 16);
    
    twai_message_t message;
    message.identifier = canId;
    message.extd = 1;
    message.rtr = 0;
    message.ss = 0;
    message.self = 0;
    message.dlc_non_comp = 0;
    
    // Parse data bytes
    int dataLen = 0;
    char* token = strtok((char*)dataStr.c_str(), " ");
    while (token != NULL && dataLen < 8) {
      message.data[dataLen++] = strtoul(token, NULL, 16);
      token = strtok(NULL, " ");
    }
    message.data_length_code = dataLen;
    
    if (twai_transmit(&message, pdMS_TO_TICKS(100)) == ESP_OK) {
      txCount++;
      server.send(200, "text/plain", "Message sent successfully");
    } else {
      server.send(500, "text/plain", "Failed to send message");
    }
  } else {
    server.send(400, "text/plain", "Missing parameters");
  }
}

void setupWebServer() {
  server.on("/", handleRoot);
  server.on("/stats", handleStats);
  server.on("/messages", handleMessages);
  server.on("/devices", handleDevices);
  server.on("/clear", HTTP_POST, handleClear);
  server.on("/reset", HTTP_POST, handleReset);
  server.on("/send", HTTP_POST, handleSend);
  
  server.begin();
  Serial.println("Web server started");
}

// === Main Setup ===
void setup() {
  Serial.begin(115200);
  Serial.println("\n\nESP32 CAN Debug Tool Starting...");
  
  // Setup LEDs
  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);
  setLED(255, 0, 0);  // Red = starting
  
  // Setup WiFi
  setupWiFi();
  
  // Setup CAN
  setupCAN();
  
  // Setup Web Server
  setupWebServer();
  
  setLED(0, 0, 255);  // Blue = ready
  Serial.println("System ready!");
  Serial.printf("Connect to WiFi: %s\n", ap_ssid);
  Serial.printf("Password: %s\n", ap_password);
  Serial.printf("Open browser to: http://%s\n", WiFi.softAPIP().toString().c_str());
}

// === Main Loop ===
void loop() {
  server.handleClient();
  
  // Check for CAN messages
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10)) == ESP_OK) {
    rxCount++;
    
    // Update device list
    updateDeviceList(message.identifier);
    
    // Store in buffer
    if (bufferIndex < MAX_MESSAGES) {
      messageBuffer[bufferIndex].id = message.identifier;
      messageBuffer[bufferIndex].len = message.data_length_code;
      messageBuffer[bufferIndex].extended = message.extd;
      messageBuffer[bufferIndex].timestamp = millis();
      messageBuffer[bufferIndex].error = false;
      memcpy(messageBuffer[bufferIndex].data, message.data, message.data_length_code);
      bufferIndex++;
    } else {
      bufferOverflow = true;
      // Shift buffer (keep last 500 messages)
      memmove(&messageBuffer[0], &messageBuffer[1], sizeof(CANMessage) * (MAX_MESSAGES - 1));
      messageBuffer[MAX_MESSAGES - 1].id = message.identifier;
      messageBuffer[MAX_MESSAGES - 1].len = message.data_length_code;
      messageBuffer[MAX_MESSAGES - 1].extended = message.extd;
      messageBuffer[MAX_MESSAGES - 1].timestamp = millis();
      messageBuffer[MAX_MESSAGES - 1].error = false;
      memcpy(messageBuffer[MAX_MESSAGES - 1].data, message.data, message.data_length_code);
    }
    
    totalMessages++;
    
    // Check for heartbeat
    if (message.identifier == HEARTBEAT_ID) {
      lastHeartbeat = millis();
      setLED(255, 0, 255);  // Flash magenta on heartbeat
      delay(50);
      setLED(0, 0, 255);
    }
  }
  
  // Check for errors
  twai_status_info_t status;
  if (twai_get_status_info(&status) == ESP_OK) {
    if (status.state == TWAI_STATE_BUS_OFF) {
      errCount++;
      
      // Log error message
      if (bufferIndex < MAX_MESSAGES) {
        messageBuffer[bufferIndex].id = 0xFFFFFFFF;
        messageBuffer[bufferIndex].len = 0;
        messageBuffer[bufferIndex].extended = false;
        messageBuffer[bufferIndex].timestamp = millis();
        messageBuffer[bufferIndex].error = true;
        bufferIndex++;
      }
      
      twai_initiate_recovery();
    }
  }
}