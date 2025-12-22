/**
 * Heltec WiFi LoRa 32 V2 - River Level Monitoring Node
 * 
 * This node reads distance measurements from a Benewake TF-Luna LiDAR sensor
 * and transmits the data via LoRaWAN to a Wisgate Edge Pro gateway.
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <Wire.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// Use the sensor library - change this to use different sensors
#include "Sensors.h"

// ============================================================================
// LoRaWAN Configuration
// ============================================================================
// IMPORTANT: Replace these with your actual values from the gateway/network server
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 
                                         0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (Heltec WiFi LoRa 32 V2)
// ============================================================================
// LoRa pins are handled by the board library
// I2C pins for TF-Luna: SDA=21, SCL=22 (ESP32 default)

// ============================================================================
// Sensor Configuration
// ============================================================================
// Choose your sensor by uncommenting one of the following:
#define USE_TF_LUNA      // Benewake TF-Luna (I2C)
// #define USE_TF02_PRO   // Benewake TF02-Pro (UART on Serial2)
// #define USE_HCSR04     // HC-SR04 ultrasonic (GPIO 4, 5)
// #define USE_AJSR04M    // AJ-SR04M waterproof ultrasonic (GPIO 4, 5)
// #define USE_JSNSR04T   // JSN-SR04T waterproof ultrasonic (GPIO 4, 5)

// Ultrasonic sensor pins (if using ultrasonic)
#define ULTRASONIC_TRIG_PIN   4
#define ULTRASONIC_ECHO_PIN   5

// TF02-Pro UART pins (if using TF02-Pro)
#define TF02_RX_PIN   17
#define TF02_TX_PIN   16

// Create sensor instance based on selection
#ifdef USE_TF_LUNA
    TFLuna distanceSensor;
#elif defined(USE_TF02_PRO)
    TF02Pro distanceSensor(Serial2, TF02_RX_PIN, TF02_TX_PIN);
#elif defined(USE_HCSR04)
    HCSR04 distanceSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
#elif defined(USE_AJSR04M)
    AJSR04M distanceSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
#elif defined(USE_JSNSR04T)
    JSNSR04T distanceSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
#else
    #error "Please define a sensor type (USE_TF_LUNA, USE_TF02_PRO, etc.)"
#endif

// Store last reading
SensorReading lastReading;

// ============================================================================
// Timing Configuration
// ============================================================================
const unsigned long SENSOR_READ_INTERVAL = 5000;  // Read sensor every 5 seconds
const unsigned long LORA_TX_INTERVAL = 30000;     // Transmit every 30 seconds
unsigned long lastSensorRead = 0;
unsigned long lastLoraTx = 0;

// ============================================================================
// Data Structure
// ============================================================================
struct SensorData {
  char nodeId[16] = "heltec_001";
  uint16_t distance_cm;
  float battery_v;
  int16_t rssi;
  uint32_t timestamp;
} sensorData;

// ============================================================================
// LoRaWAN Job Scheduling
// ============================================================================
static osjob_t sendjob;

// ============================================================================
// LMIC Pin Mapping for Heltec WiFi LoRa 32 V2
// ============================================================================
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},  // DIO0, DIO1, DIO2
};

// ============================================================================
// Forward Declarations
// ============================================================================
void readSensor();
void prepareDataPacket();
void sendData();
void onEvent(ev_t ev);
void do_send(osjob_t* j);

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("Heltec WiFi LoRa 32 V2 - River Level Monitor"));
  Serial.println(F("Initializing..."));

  // Initialize I2C (for I2C sensors)
  Wire.begin();
  delay(100);
  
  // Initialize the sensor
  Serial.print(F("Initializing sensor: "));
  Serial.println(distanceSensor.getName());
  
  if (distanceSensor.begin()) {
    Serial.println(F("Sensor detected and initialized!"));
    Serial.print(F("  Range: "));
    Serial.print(distanceSensor.getMinDistance());
    Serial.print(F(" - "));
    Serial.print(distanceSensor.getMaxDistance());
    Serial.println(F(" cm"));
  } else {
    Serial.println(F("WARNING: Sensor not detected!"));
  }

  // Initialize LMIC
  os_init();
  
  // Reset LMIC
  LMIC_reset();
  
  // Set data rate and transmit power for EU868
  // Adjust these based on your region
  LMIC_setDrTxpow(DR_SF7, 14);  // SF7, 14dBm
  
  // Start job (sending will start when LMIC is joined)
  do_send(&sendjob);
  
  Serial.println(F("Setup complete. Waiting for LoRaWAN join..."));
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  os_runloop_once();
  
  unsigned long currentMillis = millis();
  
  // Read sensor periodically
  if (currentMillis - lastSensorRead >= SENSOR_READ_INTERVAL) {
    readSensor();
    lastSensorRead = currentMillis;
  }
  
  // Send data periodically (only if joined)
  if (currentMillis - lastLoraTx >= LORA_TX_INTERVAL) {
    if (LMIC.devaddr != 0) {  // Check if joined
      prepareDataPacket();
      sendData();
      lastLoraTx = currentMillis;
    }
  }
}

// ============================================================================
// Read Distance Sensor
// ============================================================================
void readSensor() {
  // Use the unified sensor interface
  lastReading = distanceSensor.read();
  
  if (lastReading.valid) {
    sensorData.distance_cm = (uint16_t)lastReading.distance_cm;
    sensorData.timestamp = millis() / 1000;  // Seconds since boot
    
    Serial.print(F("Sensor: "));
    Serial.print(distanceSensor.getName());
    Serial.print(F(" | Distance: "));
    Serial.print(lastReading.distance_cm, 1);
    Serial.print(F(" cm ("));
    Serial.print(lastReading.distance_m, 3);
    Serial.print(F(" m)"));
    
    // Print signal strength if available (LiDAR sensors)
    if (lastReading.signal_strength > 0) {
      Serial.print(F(" | Signal: "));
      Serial.print(lastReading.signal_strength);
    }
    
    // Print temperature if available
    if (lastReading.temperature != 0) {
      Serial.print(F(" | Temp: "));
      Serial.print(lastReading.temperature);
      Serial.print(F(" C"));
    }
    
    Serial.println();
  } else {
    Serial.print(F("ERROR: Failed to read "));
    Serial.println(distanceSensor.getName());
    sensorData.distance_cm = 0;  // Error value
  }
  
  // Read battery voltage (approximate, using ADC)
  // Note: Heltec V2 has battery monitoring circuit
  uint16_t batteryRaw = analogRead(37);  // Battery voltage pin
  sensorData.battery_v = (batteryRaw / 4095.0) * 2.0 * 3.3;  // Voltage divider
  
  // Get RSSI from last transmission
  sensorData.rssi = LMIC.rssi;
}

// ============================================================================
// Prepare Data Packet
// ============================================================================
void prepareDataPacket() {
  // Update sensor reading before sending
  readSensor();
  
  Serial.println(F("Preparing data packet..."));
  Serial.print(F("Node ID: "));
  Serial.println(sensorData.nodeId);
  Serial.print(F("Distance: "));
  Serial.print(sensorData.distance_cm);
  Serial.println(F(" cm"));
  Serial.print(F("Battery: "));
  Serial.print(sensorData.battery_v);
  Serial.println(F(" V"));
}

// ============================================================================
// Send Data via LoRaWAN
// ============================================================================
void sendData() {
  // Check if LMIC is ready
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    return;
  }
  
  // Prepare payload (JSON format)
  String payload = "{";
  payload += "\"node_id\":\"" + String(sensorData.nodeId) + "\",";
  payload += "\"sensor\":\"" + String(distanceSensor.getName()) + "\",";
  payload += "\"distance_cm\":" + String(sensorData.distance_cm) + ",";
  payload += "\"distance_m\":" + String(sensorData.distance_cm / 100.0, 3) + ",";
  payload += "\"signal\":" + String(lastReading.signal_strength) + ",";
  payload += "\"battery_v\":" + String(sensorData.battery_v, 2) + ",";
  payload += "\"rssi\":" + String(sensorData.rssi) + ",";
  payload += "\"valid\":" + String(lastReading.valid ? "true" : "false") + ",";
  payload += "\"timestamp\":" + String(sensorData.timestamp);
  payload += "}";
  
  // Convert to byte array
  uint8_t payloadBytes[payload.length()];
  payload.getBytes(payloadBytes, payload.length());
  
  Serial.print(F("Sending payload: "));
  Serial.println(payload);
  Serial.print(F("Payload length: "));
  Serial.println(payload.length());
  
  // Send packet
  LMIC_setTxData2(1, payloadBytes, payload.length(), 0);
  Serial.println(F("Packet queued"));
}

// ============================================================================
// LMIC Event Handler
// ============================================================================
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(F(": "));
  
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      Serial.print(F("NetID: "));
      Serial.println(LMIC.netid, DEC);
      Serial.print(F("DevAddr: "));
      Serial.print(LMIC.devaddr, HEX);
      Serial.println();
      // Disable link check validation (automatically enabled during join)
      LMIC_setLinkCheckMode(0);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ack"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of payload"));
      }
      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(LORA_TX_INTERVAL/1000), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      Serial.println(F("EV_RXSTART"));
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE"));
      break;
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}

// ============================================================================
// Send Job Handler
// ============================================================================
void do_send(osjob_t* j) {
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    prepareDataPacket();
    sendData();
  }
}



