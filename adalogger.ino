/*
  adalogger
  Creates WiFi hotspot and serves CAN message logs via minimal HTML interface
  Transmits heartbeat so it appears as a visible device on the FRC CAN bus
  
  Pins:
    CAN TX = GPIO 4
    CAN RX = GPIO 5
  
  Serial Commands:
    &CANID SET xx   (0..63 live)
    &CANID SAVE     (EEPROM + reboot)
    &CANID GET
*/

#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "driver/twai.h"

// WiFi AP credentials
const char* ssid = "FRC-CAN-Logger";
const char* password = "frclogger";

// CAN Bus pins
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

// EEPROM
#define EEPROM_BYTES 64

WebServer server(80);

// FRC CAN constants
#define DEVICE_ID       0x0A
#define MANUFACTURER_ID 0x08
#define DEFAULT_DEVICE_NUMBER 10

// FRC CAN API classes
#define API_CLASS_ROBOT_CONTROL 0x00
#define API_CLASS_MOTOR_CONTROL 0x02
#define API_CLASS_RELAY_CONTROL 0x03
#define API_CLASS_GYRO_SENSOR 0x04
#define API_CLASS_ACCEL_SENSOR 0x05
#define API_CLASS_ULTRASONIC 0x06
#define API_CLASS_GEAR_TOOTH 0x07
#define API_CLASS_MISC_SENSOR 0x08
#define API_CLASS_IO_CONFIG 0x09
#define API_CLASS_POWER 0x0A
#define API_CLASS_FIRMWARE 0x1F

// Custom APIs for logger
#define API_TX_STATUS   0x1A0  // Logger status message
#define API_TX_HEARTBEAT 0x1A1 // Logger heartbeat

// Software version
#define SOFTWARE_VER 1

// Circular buffer for CAN messages
#define MAX_LOGS 500
struct CANLog {
  uint32_t id;
  uint8_t data[8];
  uint8_t len;
  unsigned long timestamp;
  bool is_extended;
  bool is_tx;  // true if we transmitted it
};

CANLog logs[MAX_LOGS];
int logIndex = 0;
int logCount = 0;
unsigned long startTime = 0;
bool can_started = false;

// Device state
volatile uint8_t g_deviceNumber = DEFAULT_DEVICE_NUMBER;
volatile uint16_t g_uptimeSec = 0;

// FRC CAN ID helpers
static inline uint32_t encode_id(uint8_t dt, uint8_t man, uint16_t api, uint8_t dn) {
  return ((uint32_t)dt << 24) | ((uint32_t)man << 16) | ((uint32_t)api << 6) | (dn & 0x3F);
}

static inline void decode_id(uint32_t id, uint8_t &dt, uint8_t &man, uint16_t &api, uint8_t &dn) {
  dt  = (id >> 24) & 0xFF;
  man = (id >> 16) & 0xFF;
  api = (id >> 6)  & 0x3FF;
  dn  = id & 0x3F;
}

static inline uint32_t make_can_id(uint16_t api) {
  return encode_id(DEVICE_ID, MANUFACTURER_ID, api, (uint8_t)g_deviceNumber);
}

// EEPROM functions
void EEPROMReadCANID() {
  uint8_t saved = EEPROM.read(0);
  g_deviceNumber = (saved <= 63) ? saved : DEFAULT_DEVICE_NUMBER;
}

void EEPROMSaveCANID() {
  EEPROM.write(0, (uint8_t)g_deviceNumber);
  EEPROM.commit();
}

bool CAN_Start() {
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
  g_config.tx_queue_len = 10;
  g_config.rx_queue_len = 20;
  g_config.clkout_divider = 0;

  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
    Serial.println("[CAN] Driver install failed");
    return false;
  }
  if (twai_start() != ESP_OK) {
    Serial.println("[CAN] Start failed");
    twai_driver_uninstall();
    return false;
  }
  can_started = true;
  Serial.println("[CAN] Started at 1 Mbps");
  return true;
}

bool CAN_Send(uint32_t id, const uint8_t data[8], int dlc = 8) {
  if (!can_started) return false;
  twai_message_t msg = {};
  msg.identifier = id;
  msg.extd = 1;
  msg.data_length_code = dlc;
  for (int i = 0; i < dlc && i < 8; ++i) msg.data[i] = data[i];
  
  bool success = (twai_transmit(&msg, pdMS_TO_TICKS(20)) == ESP_OK);
  
  // Log our own transmissions
  if (success) {
    logs[logIndex].id = id;
    logs[logIndex].len = dlc;
    logs[logIndex].is_extended = true;
    logs[logIndex].is_tx = true;
    logs[logIndex].timestamp = millis() - startTime;
    for (int i = 0; i < dlc; ++i) {
      logs[logIndex].data[i] = data[i];
    }
    logIndex = (logIndex + 1) % MAX_LOGS;
    if (logCount < MAX_LOGS) logCount++;
  }
  
  return success;
}

void TaskCANTx(void* parameter) {
  Serial.println("[CANTX] Task started");
  uint32_t lastStatus = 0;
  uint32_t lastHeartbeat = 0;
  uint32_t lastSecTick = millis();

  while (true) {
    uint32_t now = millis();

    // Update uptime (saturate at 0xFFFF)
    if (now - lastSecTick >= 1000) {
      lastSecTick += 1000;
      if (g_uptimeSec < 0xFFFF) g_uptimeSec++;
    }

    // Status message at ~2 Hz (API 0x1A0)
    // Format: [software_ver, uptime_lo, uptime_hi, msg_count_lo, msg_count_hi, 0, 0, 0]
    if (now - lastStatus >= 500) {
      lastStatus = now;
      uint8_t buf[8] = {0};
      buf[0] = SOFTWARE_VER;
      buf[1] = (uint8_t)(g_uptimeSec & 0xFF);
      buf[2] = (uint8_t)((g_uptimeSec >> 8) & 0xFF);
      uint16_t msgs = (logCount > 0xFFFF) ? 0xFFFF : logCount;
      buf[3] = (uint8_t)(msgs & 0xFF);
      buf[4] = (uint8_t)((msgs >> 8) & 0xFF);
      CAN_Send(make_can_id(API_TX_STATUS), buf, 8);
    }

    // Heartbeat at ~10 Hz (API 0x1A1)
    // Simple presence beacon
    if (now - lastHeartbeat >= 100) {
      lastHeartbeat = now;
      uint8_t buf[8] = {0};
      buf[0] = 0xAA; // Magic byte
      buf[1] = (uint8_t)g_deviceNumber;
      CAN_Send(make_can_id(API_TX_HEARTBEAT), buf, 8);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void TaskCANIDHelper(void* parameter) {
  Serial.println("[CANID] Helper task started. Use &CANID SET xx / SAVE / GET");
  while (true) {
    if (Serial.available()) {
      String line = Serial.readStringUntil('\n');
      line.trim();

      if (line.startsWith("&CANID SET ")) {
        int val = line.substring(11).toInt();
        if (val >= 0 && val <= 63) {
          g_deviceNumber = (uint8_t)val;
          Serial.printf("[CANID] Running DEVICE_NUMBER set to %d\n", (int)g_deviceNumber);
        } else {
          Serial.println("[CANID] Invalid value. Must be 0-63.");
        }
      } else if (line.equals("&CANID SAVE")) {
        EEPROMSaveCANID();
        Serial.println("[CANID] Saved to EEPROM. Rebooting...");
        delay(1000);
        ESP.restart();
      } else if (line.equals("&CANID GET")) {
        uint8_t eepromVal = EEPROM.read(0);
        Serial.printf("[CANID] Current=%d, EEPROM=%d, Default=%d\n",
                      (int)g_deviceNumber, (int)eepromVal, (int)DEFAULT_DEVICE_NUMBER);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println("\n=== adalogger ===");
  
  startTime = millis();
  
  // EEPROM
  EEPROM.begin(EEPROM_BYTES);
  EEPROMReadCANID();
  Serial.printf("[BOOT] DEVICE_NUMBER=%d\n", (int)g_deviceNumber);
  
  // Initialize CAN bus
  if (!CAN_Start()) {
    Serial.println("[BOOT] CAN init failed. Retrying in 3s...");
    delay(3000);
    CAN_Start();
  }
  
  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[WIFI] AP IP: ");
  Serial.println(IP);
  
  // Web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/clear", handleClear);
  
  server.begin();
  Serial.println("[BOOT] Web server started");
  
  // Start tasks
  xTaskCreatePinnedToCore(TaskCANTx, "TaskCANTx", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCANIDHelper, "TaskCANIDHelper", 4096, nullptr, 1, nullptr, 1);
  
  Serial.println("[BOOT] Setup complete");
}

void loop() {
  server.handleClient();
  
  // Read CAN messages
  if (can_started) {
    twai_message_t message;
    esp_err_t r = twai_receive(&message, pdMS_TO_TICKS(10));
    if (r == ESP_OK) {
      logs[logIndex].id = message.identifier;
      logs[logIndex].len = message.data_length_code;
      logs[logIndex].is_extended = message.extd;
      logs[logIndex].is_tx = false;
      logs[logIndex].timestamp = millis() - startTime;
      for (int i = 0; i < message.data_length_code && i < 8; ++i) {
        logs[logIndex].data[i] = message.data[i];
      }
      
      logIndex = (logIndex + 1) % MAX_LOGS;
      if (logCount < MAX_LOGS) logCount++;
    }
  }
}

void handleRoot() {
  String html = "<html><head><title> adalogger</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<script>";
  html += "function refresh(){fetch('/data').then(r=>r.text()).then(d=>{document.getElementById('log').innerHTML=d;})}";
  html += "setInterval(refresh,500);";
  html += "function clear(){fetch('/clear').then(()=>{document.getElementById('log').innerHTML='';document.getElementById('count').innerText='0';})}";
  html += "</script></head><body>";
  html += "<h1>adalogger</h1>";
  html += "<p>Device Number: " + String(g_deviceNumber) + "</p>";
  html += "<p>Uptime: " + String(g_uptimeSec) + "s</p>";
  html += "<p>Messages: <span id='count'>0</span></p>";
  html += "<button onclick='refresh()'>Refresh</button> ";
  html += "<button onclick='clear()'>Clear</button><hr>";
  html += "<div id='log'></div>";
  html += "</body></html>";
  
  server.send(200, "text/html", html);
}

void handleData() {
  String response = "";
  
  int start = (logCount < MAX_LOGS) ? 0 : logIndex;
  int count = (logCount < MAX_LOGS) ? logCount : MAX_LOGS;
  
  response += "<table border='1'><tr><th>Dir</th><th>Time(ms)</th><th>ID</th><th>Type</th><th>API</th><th>DN</th><th>Data</th></tr>";
  
  for (int i = 0; i < count; i++) {
    int idx = (start + i) % MAX_LOGS;
    CANLog* log = &logs[idx];
    
    response += "<tr>";
    response += "<td>" + String(log->is_tx ? "TX" : "RX") + "</td>";
    response += "<td>" + String(log->timestamp) + "</td>";
    response += "<td>0x" + String(log->id, HEX) + "</td>";
    
    // Decode FRC CAN ID
    if (log->is_extended) {
      uint8_t deviceType, manufacturer, deviceNumber;
      uint16_t apiClass;
      decode_id(log->id, deviceType, manufacturer, apiClass, deviceNumber);
      
      // Check if it's FRC format
      if (deviceType == DEVICE_ID && manufacturer == MANUFACTURER_ID) {
        response += "<td>FRC</td>";
        response += "<td>" + getAPIClassName(apiClass) + "</td>";
        response += "<td>" + String(deviceNumber) + "</td>";
      } else {
        response += "<td>Ext</td>";
        response += "<td>DT:" + String(deviceType, HEX) + " MFG:" + String(manufacturer, HEX) + "</td>";
        response += "<td>" + String(deviceNumber) + "</td>";
      }
    } else {
      response += "<td>Std</td><td>-</td><td>-</td>";
    }
    
    // Data bytes
    response += "<td>";
    for (int j = 0; j < log->len; j++) {
      if (j > 0) response += " ";
      if (log->data[j] < 0x10) response += "0";
      response += String(log->data[j], HEX);
    }
    response += "</td></tr>";
  }
  
  response += "</table>";
  response += "<script>document.getElementById('count').innerText=" + String(logCount) + ";</script>";
  
  server.send(200, "text/html", response);
}

void handleClear() {
  logIndex = 0;
  logCount = 0;
  startTime = millis();
  server.send(200, "text/plain", "OK");
}

String getAPIClassName(uint16_t apiClass) {
  switch(apiClass) {
    case API_CLASS_ROBOT_CONTROL: return "RobotCtrl";
    case API_CLASS_MOTOR_CONTROL: return "Motor";
    case API_CLASS_RELAY_CONTROL: return "Relay";
    case API_CLASS_GYRO_SENSOR: return "Gyro";
    case API_CLASS_ACCEL_SENSOR: return "Accel";
    case API_CLASS_ULTRASONIC: return "Ultrasonic";
    case API_CLASS_GEAR_TOOTH: return "GearTooth";
    case API_CLASS_MISC_SENSOR: return "Sensor";
    case API_CLASS_IO_CONFIG: return "IOConfig";
    case API_CLASS_POWER: return "Power";
    case API_CLASS_FIRMWARE: return "Firmware";
    case 0x185: return "RX_CONTROL";
    case 0x186: return "RX_STATUS";
    case 0x195: return "TX_INPUTS";
    case 0x196: return "TX_RESET";
    case API_TX_STATUS: return "LOGGER_STATUS";
    case API_TX_HEARTBEAT: return "LOGGER_HB";
    default: return "0x" + String(apiClass, HEX);
  }
}