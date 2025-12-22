/**
 * Sensor Comparison Example
 * 
 * This example demonstrates how to use multiple sensors simultaneously
 * for comparing their readings. This is useful for the research project
 * evaluating different sensors for river level monitoring.
 * 
 * Wiring for ESP32:
 * 
 * TF-Luna (I2C):
 *   - SDA -> GPIO 21
 *   - SCL -> GPIO 22
 *   - VCC -> 3.3V or 5V
 *   - GND -> GND
 * 
 * TF02-Pro (UART):
 *   - RX -> GPIO 16 (ESP TX)
 *   - TX -> GPIO 17 (ESP RX)
 *   - VCC -> 5V
 *   - GND -> GND
 * 
 * HC-SR04:
 *   - TRIG -> GPIO 4
 *   - ECHO -> GPIO 5
 *   - VCC -> 5V
 *   - GND -> GND
 * 
 * AJ-SR04M:
 *   - TRIG -> GPIO 18
 *   - ECHO -> GPIO 19
 *   - VCC -> 5V
 *   - GND -> GND
 * 
 * Author: Klaus Dieter Kupper
 */

#include <Arduino.h>
#include "../Sensors.h"

// ============================================================================
// Pin Definitions
// ============================================================================

// I2C is on default pins (21, 22)
// TF02-Pro UART pins
#define TF02_RX_PIN     17  // ESP32 receives on this pin (connect to TF02 TX)
#define TF02_TX_PIN     16  // ESP32 transmits on this pin (connect to TF02 RX)

// HC-SR04 pins
#define HCSR04_TRIG     4
#define HCSR04_ECHO     5

// AJ-SR04M pins
#define AJSR04M_TRIG    18
#define AJSR04M_ECHO    19

// ============================================================================
// Sensor Instances
// ============================================================================

TFLuna tfLuna;  // Uses default I2C address
TF02Pro tf02Pro(Serial2, TF02_RX_PIN, TF02_TX_PIN);
HCSR04 hcsr04(HCSR04_TRIG, HCSR04_ECHO);
AJSR04M ajsr04m(AJSR04M_TRIG, AJSR04M_ECHO);

// Array of sensor pointers for easy iteration
SensorBase* sensors[] = {
    &tfLuna,
    &tf02Pro,
    &hcsr04,
    &ajsr04m
};
const int NUM_SENSORS = sizeof(sensors) / sizeof(sensors[0]);

// ============================================================================
// Setup
// ============================================================================

void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println("===========================================");
    Serial.println("  Sensor Comparison for River Monitoring");
    Serial.println("===========================================");
    Serial.println();
    
    // Initialize all sensors
    Serial.println("Initializing sensors...");
    Serial.println();
    
    for (int i = 0; i < NUM_SENSORS; i++) {
        Serial.print("  ");
        Serial.print(sensors[i]->getName());
        Serial.print(": ");
        
        if (sensors[i]->begin()) {
            Serial.println("OK");
        } else {
            Serial.println("NOT DETECTED");
        }
    }
    
    Serial.println();
    Serial.println("Starting measurements...");
    Serial.println();
    
    // Print CSV header
    Serial.println("Timestamp,Sensor,Distance_cm,Valid,Signal,Temp");
}

// ============================================================================
// Main Loop
// ============================================================================

void loop() {
    unsigned long timestamp = millis();
    
    // Read all sensors and print results in CSV format
    for (int i = 0; i < NUM_SENSORS; i++) {
        SensorReading reading = sensors[i]->read();
        
        // Print CSV row
        Serial.print(timestamp);
        Serial.print(",");
        Serial.print(sensors[i]->getName());
        Serial.print(",");
        Serial.print(reading.distance_cm, 1);
        Serial.print(",");
        Serial.print(reading.valid ? "1" : "0");
        Serial.print(",");
        Serial.print(reading.signal_strength);
        Serial.print(",");
        Serial.println(reading.temperature);
    }
    
    Serial.println();  // Blank line between reading sets
    
    // Wait before next reading
    delay(1000);
}

// ============================================================================
// Alternative: Individual Sensor Testing
// ============================================================================

void testSingleSensor(SensorBase* sensor) {
    Serial.print("Testing ");
    Serial.println(sensor->getName());
    Serial.println("----------------------------");
    
    Serial.print("  Type: ");
    Serial.println(getSensorTypeName(sensor->getType()));
    
    Serial.print("  Min Range: ");
    Serial.print(sensor->getMinDistance());
    Serial.println(" cm");
    
    Serial.print("  Max Range: ");
    Serial.print(sensor->getMaxDistance());
    Serial.println(" cm");
    
    Serial.print("  Connected: ");
    Serial.println(sensor->isConnected() ? "Yes" : "No");
    
    if (sensor->isConnected()) {
        Serial.println("  Taking 5 readings...");
        
        for (int i = 0; i < 5; i++) {
            SensorReading reading = sensor->read();
            
            Serial.print("    ");
            Serial.print(i + 1);
            Serial.print(": ");
            
            if (reading.valid) {
                Serial.print(reading.distance_cm, 1);
                Serial.print(" cm (");
                Serial.print(reading.distance_m, 2);
                Serial.println(" m)");
            } else {
                Serial.println("Invalid reading");
            }
            
            delay(500);
        }
    }
    
    Serial.println();
}


