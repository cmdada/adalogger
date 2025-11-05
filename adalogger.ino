/*
  adalogger
  Creates WiFi hotspot and serves CAN message logs via minimal HTML interface
  Transmits heartbeat so it appears as a visible device on the FRC CAN bus
  
  Pins:
    RED_PIN   = 15 (LEDC PWM)
    GREEN_PIN = 13 (LEDC PWM)
    BLUE_PIN  = 14 (LEDC PWM)
    CAN TX    = GPIO 4
    CAN RX    = GPIO 5
  
  Status LED Indicators:
    - Booting: Pulsing blue
    - WiFi AP Ready: Solid green
    - CAN Active: Green with blue flashes on activity
    - CAN Error: Red pulsing
    - High traffic: Faster blue flashes
  
  Serial Commands:
    &CANID SET xx   (0..63 live)
    &CANID SAVE     (EEPROM + reboot)
    &CANID GET
*/

#include <WiFi.h>
#include <WebServer.h>
#include <EEPROM.h>
#include "driver/twai.h"
#include "driver/ledc.h"

// WiFi AP credentials
const char* ssid = "FRC-CAN-Logger";
const char* password = "frclogger";

// Status LED pins
#define RED_PIN    15
#define GREEN_PIN  13
#define BLUE_PIN   14

// CAN Bus pins
#define CAN_TX_PIN GPIO_NUM_4
#define CAN_RX_PIN GPIO_NUM_5

// EEPROM
#define EEPROM_BYTES 64

// LEDC (PWM) configuration
static const uint32_t LEDC_FREQ_HZ = 4000;
static const ledc_timer_bit_t LEDC_RES_BITS = LEDC_TIMER_12_BIT;
static const ledc_mode_t  LEDC_MODE  = LEDC_LOW_SPEED_MODE;
static const ledc_timer_t LEDC_TIMER = LEDC_TIMER_0;
static const ledc_channel_t LEDC_CH_RED   = LEDC_CHANNEL_0;
static const ledc_channel_t LEDC_CH_GREEN = LEDC_CHANNEL_1;
static const ledc_channel_t LEDC_CH_BLUE  = LEDC_CHANNEL_2;

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
#define API_TX_STATUS   0x1A0
#define API_TX_HEARTBEAT 0x1A1

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
  bool is_tx;
};

CANLog logs[MAX_LOGS];
int logIndex = 0;
int logCount = 0;
unsigned long startTime = 0;
bool can_started = false;

// Device state
volatile uint8_t g_deviceNumber = DEFAULT_DEVICE_NUMBER;
volatile uint16_t g_uptimeSec = 0;

// LED state
enum LEDState {
  LED_BOOTING,
  LED_WIFI_READY,
  LED_CAN_ACTIVE,
  LED_CAN_ERROR
};
volatile LEDState g_ledState = LED_BOOTING;
volatile uint16_t g_ledR_12 = 0, g_ledG_12 = 0, g_ledB_12 = 0;
volatile uint16_t g_ledR_target = 0, g_ledG_target = 0, g_ledB_target = 0;
volatile uint32_t g_lastCANActivity = 0;
volatile uint16_t g_recentMessageCount = 0;

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

// LED control functions
void LED_Init() {
  ledc_timer_config_t tcfg = {};
  tcfg.speed_mode       = LEDC_MODE;
  tcfg.duty_resolution  = LEDC_RES_BITS;
  tcfg.timer_num        = LEDC_TIMER;
  tcfg.freq_hz          = LEDC_FREQ_HZ;
  tcfg.clk_cfg          = LEDC_AUTO_CLK;
  ledc_timer_config(&tcfg);

  ledc_channel_config_t chR = {};
  chR.gpio_num   = RED_PIN;
  chR.speed_mode = LEDC_MODE;
  chR.channel    = LEDC_CH_RED;
  chR.intr_type  = LEDC_INTR_DISABLE;
  chR.timer_sel  = LEDC_TIMER;
  chR.duty       = 0;
  chR.hpoint     = 0;
  ledc_channel_config(&chR);

  ledc_channel_config_t chG = chR; 
  chG.gpio_num = GREEN_PIN; 
  chG.channel = LEDC_CH_GREEN;
  ledc_channel_config(&chG);

  ledc_channel_config_t chB = chR; 
  chB.gpio_num = BLUE_PIN;  
  chB.channel = LEDC_CH_BLUE;
  ledc_channel_config(&chB);
}

void LED_SetTarget(uint16_t r, uint16_t g, uint16_t b) {
  g_ledR_target = r;
  g_ledG_target = g;
  g_ledB_target = b;
}

void LED_SetTargetRGB8(uint8_t r8, uint8_t g8, uint8_t b8) {
  LED_SetTarget(((uint16_t)r8) << 4, ((uint16_t)g8) << 4, ((uint16_t)b8) << 4);
}

// CAN functions
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
  
  if (success) {
    g_lastCANActivity = millis();
    g_recentMessageCount++;
    
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

// LED Animation Task
void TaskLEDControl(void* parameter) {
  Serial.println("[LED] Task started");
  const uint16_t step = 64;
  uint32_t lastPulse = 0;
  uint32_t pulsePhase = 0;
  
  while (true) {
    uint32_t now = millis();
    
    // Update LED targets based on state
    switch (g_ledState) {
      case LED_BOOTING:
        // Pulsing blue
        if (now - lastPulse >= 20) {
          lastPulse = now;
          pulsePhase = (pulsePhase + 1) % 100;
          uint16_t brightness = (pulsePhase < 50) ? (pulsePhase * 40) : ((100 - pulsePhase) * 40);
          LED_SetTarget(0, 0, brightness);
        }
        break;
        
      case LED_WIFI_READY:
        // Solid green
        LED_SetTargetRGB8(0, 128, 0);
        break;
        
      case LED_CAN_ACTIVE:
        // Green base with blue flash on activity
        if (now - g_lastCANActivity < 100) {
          // Recent activity - flash blue
          uint16_t intensity = 4095 - ((now - g_lastCANActivity) * 40);
          if (intensity > 4095) intensity = 0;
          LED_SetTarget(0, 2048, intensity);
        } else {
          // Idle - solid green
          LED_SetTargetRGB8(0, 128, 0);
        }
        break;
        
      case LED_CAN_ERROR:
        // Pulsing red
        if (now - lastPulse >= 30) {
          lastPulse = now;
          pulsePhase = (pulsePhase + 1) % 100;
          uint16_t brightness = (pulsePhase < 50) ? (pulsePhase * 40) : ((100 - pulsePhase) * 40);
          LED_SetTarget(brightness, 0, 0);
        }
        break;
    }
    
    // Smooth fade toward targets
    uint16_t curR = g_ledR_12;
    uint16_t curG = g_ledG_12;
    uint16_t curB = g_ledB_12;
    
    uint16_t tgtR = g_ledR_target;
    uint16_t tgtG = g_ledG_target;
    uint16_t tgtB = g_ledB_target;
    
    // Red
    if (curR < tgtR) {
      uint32_t next = (uint32_t)curR + step;
      if (next > tgtR) next = tgtR;
      curR = (uint16_t)next;
    } else if (curR > tgtR) {
      uint16_t next = (curR > step) ? (curR - step) : 0;
      if (next < tgtR) next = tgtR;
      curR = next;
    }
    
    // Green
    if (curG < tgtG) {
      uint32_t next = (uint32_t)curG + step;
      if (next > tgtG) next = tgtG;
      curG = (uint16_t)next;
    } else if (curG > tgtG) {
      uint16_t next = (curG > step) ? (curG - step) : 0;
      if (next < tgtG) next = tgtG;
      curG = next;
    }
    
    // Blue
    if (curB < tgtB) {
      uint32_t next = (uint32_t)curB + step;
      if (next > tgtB) next = tgtB;
      curB = (uint16_t)next;
    } else if (curB > tgtB) {
      uint16_t next = (curB > step) ? (curB - step) : 0;
      if (next < tgtB) next = tgtB;
      curB = next;
    }
    
    g_ledR_12 = curR;
    g_ledG_12 = curG;
    g_ledB_12 = curB;
    
    // Apply PWM
    ledc_set_duty(LEDC_MODE, LEDC_CH_RED, curR);
    ledc_update_duty(LEDC_MODE, LEDC_CH_RED);
    
    ledc_set_duty(LEDC_MODE, LEDC_CH_GREEN, curG);
    ledc_update_duty(LEDC_MODE, LEDC_CH_GREEN);
    
    ledc_set_duty(LEDC_MODE, LEDC_CH_BLUE, curB);
    ledc_update_duty(LEDC_MODE, LEDC_CH_BLUE);
    
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// CAN Health Monitor Task
void TaskCANHealth(void* parameter) {
  Serial.println("[CANHEALTH] Task started");
  uint32_t lastCheck = 0;
  
  while (true) {
    uint32_t now = millis();
    
    if (now - lastCheck >= 1000) {
      lastCheck = now;
      
      if (!can_started) {
        g_ledState = LED_CAN_ERROR;
      } else {
        // Check CAN bus state
        twai_status_info_t status;
        if (twai_get_status_info(&status) == ESP_OK) {
          if (status.state == TWAI_STATE_BUS_OFF || status.state == TWAI_STATE_RECOVERING) {
            g_ledState = LED_CAN_ERROR;
            Serial.println("[CANHEALTH] Bus error detected");
          } else if (g_ledState != LED_CAN_ACTIVE) {
            g_ledState = LED_CAN_ACTIVE;
          }
        }
      }
      
      // Reset recent message counter
      g_recentMessageCount = 0;
    }
    
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void TaskCANTx(void* parameter) {
  Serial.println("[CANTX] Task started");
  uint32_t lastStatus = 0;
  uint32_t lastHeartbeat = 0;
  uint32_t lastSecTick = millis();

  while (true) {
    uint32_t now = millis();

    if (now - lastSecTick >= 1000) {
      lastSecTick += 1000;
      if (g_uptimeSec < 0xFFFF) g_uptimeSec++;
    }

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

    if (now - lastHeartbeat >= 100) {
      lastHeartbeat = now;
      uint8_t buf[8] = {0};
      buf[0] = 0xAA;
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
  Serial.println("\n=== adalogger with Status LEDs ===");
  
  startTime = millis();
  
  // Initialize LEDs first (booting state)
  LED_Init();
  g_ledState = LED_BOOTING;
  
  // EEPROM
  EEPROM.begin(EEPROM_BYTES);
  EEPROMReadCANID();
  Serial.printf("[BOOT] DEVICE_NUMBER=%d\n", (int)g_deviceNumber);
  
  // Initialize CAN bus
  if (!CAN_Start()) {
    Serial.println("[BOOT] CAN init failed. Retrying in 3s...");
    g_ledState = LED_CAN_ERROR;
    delay(3000);
    CAN_Start();
  }
  
  // Setup WiFi AP
  WiFi.softAP(ssid, password);
  IPAddress IP = WiFi.softAPIP();
  Serial.print("[WIFI] AP IP: ");
  Serial.println(IP);
  
  // WiFi ready - change LED state
  g_ledState = LED_WIFI_READY;
  delay(500);
  
  // Once CAN is confirmed working, switch to active state
  if (can_started) {
    g_ledState = LED_CAN_ACTIVE;
  }
  
  // Web server routes
  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/clear", handleClear);
  
  server.begin();
  Serial.println("[BOOT] Web server started");
  
  // Start tasks
  xTaskCreatePinnedToCore(TaskLEDControl, "TaskLEDControl", 4096, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(TaskCANHealth, "TaskCANHealth", 4096, nullptr, 2, nullptr, 1);
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
      g_lastCANActivity = millis();
      g_recentMessageCount++;
      
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
  String html = "<html><head><title>adalogger</title>";
  html += "<meta name='viewport' content='width=device-width,initial-scale=1'>";
  html += "<script>";
  html += "function refresh(){fetch('/data').then(r=>r.text()).then(d=>{document.getElementById('log').innerHTML=d;})}";
  html += "setInterval(refresh,500);";
  html += "function clear(){fetch('/clear').then(()=>{document.getElementById('log').innerHTML='';document.getElementById('count').innerText='0';})}";
  html += "</script></head><body>";
  html += "<h1>adalogger</h1>";
  html += "<p>Device Number: " + String(g_deviceNumber) + "</p>";
  html += "<p>Uptime: " + String(g_uptimeSec) + "s</p>";
  
  // Status indicator
  html += "<p>Status: ";
  switch (g_ledState) {
    case LED_BOOTING: html += "🔵 Booting"; break;
    case LED_WIFI_READY: html += "🟢 WiFi Ready"; break;
    case LED_CAN_ACTIVE: html += "🟢 CAN Active"; break;
    case LED_CAN_ERROR: html += "🔴 CAN Error"; break;
  }
  html += "</p>";
  
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
    
    if (log->is_extended) {
      uint8_t deviceType, manufacturer, deviceNumber;
      uint16_t apiClass;
      decode_id(log->id, deviceType, manufacturer, apiClass, deviceNumber);
      
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
