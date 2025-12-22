/**
 * LoRa Sensor Node Example
 * 
 * This example shows how to integrate the sensor library with
 * the LoRaWAN code for the river monitoring project.
 * 
 * Simply uncomment the sensor you want to use.
 * 
 * Author: Klaus Dieter Kupper
 */

#include <Arduino.h>
#include <Wire.h>
#include "../Sensors.h"

// ============================================================================
// Choose Your Sensor (uncomment one)
// ============================================================================

// Option 1: TF-Luna LiDAR (I2C)
#define USE_TF_LUNA
// #define USE_TF02_PRO
// #define USE_HCSR04
// #define USE_AJSR04M
// #define USE_JSNSR04T

// ============================================================================
// Pin Definitions for ESP32
// ============================================================================

// I2C pins (for TF-Luna)
#define I2C_SDA         21
#define I2C_SCL         22

// UART pins (for TF02-Pro)
#define TF02_RX_PIN     17
#define TF02_TX_PIN     16

// GPIO pins (for ultrasonic sensors)
#define ULTRASONIC_TRIG 4
#define ULTRASONIC_ECHO 5

// ============================================================================
// Create Sensor Instance
// ============================================================================

#ifdef USE_TF_LUNA
    TFLuna sensor;
    #define SENSOR_NEEDS_WIRE
#endif

#ifdef USE_TF02_PRO
    TF02Pro sensor(Serial2, TF02_RX_PIN, TF02_TX_PIN);
#endif

#ifdef USE_HCSR04
    HCSR04 sensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
#endif

#ifdef USE_AJSR04M
    AJSR04M sensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
#endif

#ifdef USE_JSNSR04T
    JSNSR04T sensor(ULTRASONIC_TRIG, ULTRASONIC_ECHO);
#endif

// ============================================================================
// Data Structure for LoRa Payload
// ============================================================================

struct SensorPayload {
    char node_id[16];
    float distance_cm;
    float distance_m;
    int16_t signal_strength;
    int16_t temperature;
    float battery_v;
    bool valid;
};

SensorPayload payload;

// ============================================================================
// Functions
// ============================================================================

/**
 * Initialize the sensor
 */
bool initSensor() {
    #ifdef SENSOR_NEEDS_WIRE
        Wire.begin(I2C_SDA, I2C_SCL);
        delay(100);
    #endif
    
    return sensor.begin();
}

/**
 * Read sensor and populate payload
 */
bool readSensor() {
    SensorReading reading = sensor.read();
    
    payload.distance_cm = reading.distance_cm;
    payload.distance_m = reading.distance_m;
    payload.signal_strength = reading.signal_strength;
    payload.temperature = reading.temperature;
    payload.valid = reading.valid;
    
    return reading.valid;
}

/**
 * Get sensor info string for debugging
 */
void printSensorInfo() {
    Serial.println("=== Sensor Configuration ===");
    Serial.print("Sensor: ");
    Serial.println(sensor.getName());
    Serial.print("Type: ");
    Serial.println(getSensorTypeName(sensor.getType()));
    Serial.print("Min Range: ");
    Serial.print(sensor.getMinDistance());
    Serial.println(" cm");
    Serial.print("Max Range: ");
    Serial.print(sensor.getMaxDistance());
    Serial.println(" cm");
    Serial.print("Connected: ");
    Serial.println(sensor.isConnected() ? "Yes" : "No");
    Serial.println("============================");
}

/**
 * Create JSON payload string for LoRaWAN
 */
String createJsonPayload() {
    String json = "{";
    json += "\"node_id\":\"" + String(payload.node_id) + "\",";
    json += "\"sensor\":\"" + String(sensor.getName()) + "\",";
    json += "\"distance_cm\":" + String(payload.distance_cm, 1) + ",";
    json += "\"distance_m\":" + String(payload.distance_m, 3) + ",";
    json += "\"signal\":" + String(payload.signal_strength) + ",";
    json += "\"temp\":" + String(payload.temperature) + ",";
    json += "\"battery\":" + String(payload.battery_v, 2) + ",";
    json += "\"valid\":" + String(payload.valid ? "true" : "false");
    json += "}";
    return json;
}

/**
 * Create compact binary payload for LoRaWAN (saves bytes)
 * Format: [dist_hi][dist_lo][signal_hi][signal_lo][temp][battery*10][valid]
 */
void createBinaryPayload(uint8_t* buffer, uint8_t* length) {
    uint16_t dist = (uint16_t)payload.distance_cm;
    
    buffer[0] = (dist >> 8) & 0xFF;      // Distance high byte
    buffer[1] = dist & 0xFF;              // Distance low byte
    buffer[2] = (payload.signal_strength >> 8) & 0xFF;  // Signal high
    buffer[3] = payload.signal_strength & 0xFF;          // Signal low
    buffer[4] = payload.temperature & 0xFF;              // Temperature
    buffer[5] = (uint8_t)(payload.battery_v * 10);       // Battery (0.1V resolution)
    buffer[6] = payload.valid ? 1 : 0;                   // Valid flag
    
    *length = 7;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("LoRa Sensor Node - Using Sensor Library");
    Serial.println();
    
    // Set node ID
    strcpy(payload.node_id, "node_001");
    
    // Initialize sensor
    Serial.println("Initializing sensor...");
    if (initSensor()) {
        Serial.println("Sensor initialized successfully!");
    } else {
        Serial.println("ERROR: Sensor not detected!");
    }
    
    printSensorInfo();
    Serial.println();
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    // Read battery voltage (example for ESP32)
    payload.battery_v = analogRead(34) / 4095.0 * 3.3 * 2;  // Adjust for your voltage divider
    
    // Read sensor
    if (readSensor()) {
        Serial.print("Distance: ");
        Serial.print(payload.distance_cm, 1);
        Serial.print(" cm (");
        Serial.print(payload.distance_m, 3);
        Serial.println(" m)");
        
        // Create JSON payload
        String json = createJsonPayload();
        Serial.print("JSON Payload: ");
        Serial.println(json);
        
        // Create binary payload (for actual LoRa transmission)
        uint8_t binaryPayload[10];
        uint8_t payloadLength;
        createBinaryPayload(binaryPayload, &payloadLength);
        
        Serial.print("Binary Payload (");
        Serial.print(payloadLength);
        Serial.print(" bytes): ");
        for (int i = 0; i < payloadLength; i++) {
            if (binaryPayload[i] < 0x10) Serial.print("0");
            Serial.print(binaryPayload[i], HEX);
            Serial.print(" ");
        }
        Serial.println();
        
    } else {
        Serial.println("ERROR: Invalid sensor reading");
    }
    
    Serial.println();
    delay(2000);
}


