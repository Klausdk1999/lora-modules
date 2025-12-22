/**
 * LilyGo LoRa32 - River Level Monitoring Node
 * 
 * This node reads distance measurements from a Benewake TF-Luna LiDAR sensor
 * and transmits the data via LoRaWAN to a Wisgate Edge Pro gateway.
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 * 
 * Note: Pin definitions may vary depending on your specific LilyGo LoRa32 model.
 * Adjust the pin mappings in the lmic_pinmap structure if needed.
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ============================================================================
// LoRaWAN Configuration
// ============================================================================
// IMPORTANT: Replace these with your actual values from the gateway/network server
static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8] = { 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 
                                         0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (LilyGo LoRa32)
// ============================================================================
// Note: Pin definitions vary by LilyGo model. Common configurations:
// - LilyGo T-Beam: NSS=18, RST=23, DIO0=26, DIO1=33, DIO2=32
// - LilyGo LoRa32 V1.6: NSS=18, RST=14, DIO0=26, DIO1=33, DIO2=32
// I2C pins for TF-Luna: SDA=21, SCL=22 (ESP32 default)

// ============================================================================
// Sensor Configuration
// ============================================================================
TFLI2C tflI2C;
int16_t tfDist = 0;           // Distance in centimeters
int16_t tfFlux = 0;           // Signal strength
int16_t tfTemp = 0;           // Temperature
uint8_t tfAddr = TFL_DEF_ADR; // Default I2C address: 0x10

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
  char nodeId[16] = "lilygo_001";
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
// LMIC Pin Mapping for LilyGo LoRa32
// ============================================================================
// Common pin configuration - ADJUST BASED ON YOUR SPECIFIC MODEL
const lmic_pinmap lmic_pins = {
    .nss = 18,      // Chip select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,      // Reset pin (may be 14 on some models)
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
  Serial.println(F("LilyGo LoRa32 - River Level Monitor"));
  Serial.println(F("Initializing..."));

  // Initialize I2C for TF-Luna
  Wire.begin();
  delay(100);
  
  // Test TF-Luna connection
  if (tflI2C.getData(tfDist, tfAddr)) {
    Serial.println(F("TF-Luna sensor detected!"));
  } else {
    Serial.println(F("WARNING: TF-Luna sensor not detected!"));
    Serial.println(F("Check I2C wiring and power supply."));
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
// Read TF-Luna Sensor
// ============================================================================
void readSensor() {
  if (tflI2C.getData(tfDist, tfFlux, tfTemp, tfAddr)) {
    sensorData.distance_cm = tfDist;
    sensorData.timestamp = millis() / 1000;  // Seconds since boot
    
    Serial.print(F("Distance: "));
    Serial.print(tfDist);
    Serial.print(F(" cm, Flux: "));
    Serial.print(tfFlux);
    Serial.print(F(", Temp: "));
    Serial.print(tfTemp);
    Serial.println(F(" C"));
  } else {
    Serial.println(F("ERROR: Failed to read TF-Luna sensor"));
    sensorData.distance_cm = 0;  // Error value
  }
  
  // Read battery voltage (approximate, using ADC)
  // Note: Pin number may vary by LilyGo model
  // Common: GPIO 35 or 36 for battery monitoring
  uint16_t batteryRaw = analogRead(35);  // Adjust pin if needed
  // Voltage divider calculation depends on your board's circuit
  // Typical: (batteryRaw / 4095.0) * 2.0 * 3.3
  sensorData.battery_v = (batteryRaw / 4095.0) * 2.0 * 3.3;
  
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
  payload += "\"distance_cm\":" + String(sensorData.distance_cm) + ",";
  payload += "\"battery_v\":" + String(sensorData.battery_v, 2) + ",";
  payload += "\"rssi\":" + String(sensorData.rssi) + ",";
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
      Serial.println(F("Check your DevEUI, AppEUI, and AppKey"));
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

