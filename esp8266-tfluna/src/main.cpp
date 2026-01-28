/**
 * ESP8266 NodeMCU - TF-Luna LiDAR Distance Sensor Test
 *
 * This code reads distance measurements from a TF-Luna LiDAR sensor
 * and outputs 5 normalized readings every 20 seconds.
 *
 * Hardware connections (NodeMCU ESP8266):
 *   TF-Luna VCC  -> 5V (or 3.3V if supported)
 *   TF-Luna GND  -> GND
 *   TF-Luna SDA  -> D2 (GPIO4)
 *   TF-Luna SCL  -> D1 (GPIO5)
 *
 * Note: TF-Luna must be configured for I2C mode (default is UART)
 */

#include <Arduino.h>
#include <Wire.h>
#include <TFLI2C.h>

// TF-Luna configuration
TFLI2C tflI2C;
const int16_t TF_LUNA_ADDR = TFL_DEF_ADR;  // Default I2C address: 0x10

// Measurement configuration
const int NUM_READINGS = 5;
const unsigned long READING_INTERVAL_MS = 20000;  // 20 seconds between measurement cycles
const unsigned long SAMPLE_DELAY_MS = 100;        // 100ms between individual samples

// TF-Luna specifications for normalization
const float TF_LUNA_MIN_RANGE_CM = 20.0;    // Minimum reliable range (cm)
const float TF_LUNA_MAX_RANGE_CM = 800.0;   // Maximum range (cm) - 8 meters

// Storage for readings
int16_t readings[NUM_READINGS];
float normalizedReadings[NUM_READINGS];

unsigned long lastMeasurementTime = 0;

/**
 * Normalize a distance reading to 0.0-1.0 range
 * Values below min range are clamped to 0.0
 * Values above max range are clamped to 1.0
 */
float normalizeReading(int16_t distanceCm) {
    if (distanceCm < 0) {
        return -1.0;  // Error indicator
    }

    float distance = (float)distanceCm;

    // Clamp to valid range
    if (distance < TF_LUNA_MIN_RANGE_CM) {
        distance = TF_LUNA_MIN_RANGE_CM;
    }
    if (distance > TF_LUNA_MAX_RANGE_CM) {
        distance = TF_LUNA_MAX_RANGE_CM;
    }

    // Normalize to 0.0 - 1.0
    return (distance - TF_LUNA_MIN_RANGE_CM) / (TF_LUNA_MAX_RANGE_CM - TF_LUNA_MIN_RANGE_CM);
}

/**
 * Take NUM_READINGS measurements and store them
 * Returns true if all readings were successful
 */
bool takeMeasurements() {
    int successCount = 0;

    for (int i = 0; i < NUM_READINGS; i++) {
        int16_t distance;

        if (tflI2C.getData(distance, TF_LUNA_ADDR)) {
            readings[i] = distance;
            normalizedReadings[i] = normalizeReading(distance);
            successCount++;
        } else {
            readings[i] = -1;
            normalizedReadings[i] = -1.0;
            Serial.print("Error reading sample ");
            Serial.println(i + 1);
        }

        delay(SAMPLE_DELAY_MS);
    }

    return (successCount == NUM_READINGS);
}

/**
 * Print measurements to serial in a structured format
 */
void printMeasurements() {
    Serial.println("========================================");
    Serial.println("TF-Luna Measurement Cycle");
    Serial.print("Timestamp: ");
    Serial.print(millis() / 1000);
    Serial.println(" seconds");
    Serial.println("----------------------------------------");

    // Print raw readings
    Serial.print("Raw readings (cm): [");
    for (int i = 0; i < NUM_READINGS; i++) {
        Serial.print(readings[i]);
        if (i < NUM_READINGS - 1) Serial.print(", ");
    }
    Serial.println("]");

    // Print normalized readings
    Serial.print("Normalized (0-1):  [");
    for (int i = 0; i < NUM_READINGS; i++) {
        Serial.print(normalizedReadings[i], 4);
        if (i < NUM_READINGS - 1) Serial.print(", ");
    }
    Serial.println("]");

    // Calculate and print statistics
    float sum = 0;
    float min = 2.0;
    float max = -2.0;
    int validCount = 0;

    for (int i = 0; i < NUM_READINGS; i++) {
        if (normalizedReadings[i] >= 0) {
            sum += normalizedReadings[i];
            if (normalizedReadings[i] < min) min = normalizedReadings[i];
            if (normalizedReadings[i] > max) max = normalizedReadings[i];
            validCount++;
        }
    }

    if (validCount > 0) {
        float avg = sum / validCount;
        Serial.println("----------------------------------------");
        Serial.print("Statistics (normalized): ");
        Serial.print("avg=");
        Serial.print(avg, 4);
        Serial.print(", min=");
        Serial.print(min, 4);
        Serial.print(", max=");
        Serial.print(max, 4);
        Serial.print(", range=");
        Serial.println(max - min, 4);
    }

    Serial.println("========================================");
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    delay(1000);  // Wait for serial to stabilize

    Serial.println();
    Serial.println("========================================");
    Serial.println("ESP8266 TF-Luna Distance Sensor Test");
    Serial.println("========================================");
    Serial.print("Readings per cycle: ");
    Serial.println(NUM_READINGS);
    Serial.print("Cycle interval: ");
    Serial.print(READING_INTERVAL_MS / 1000);
    Serial.println(" seconds");
    Serial.print("Normalization range: ");
    Serial.print(TF_LUNA_MIN_RANGE_CM);
    Serial.print(" - ");
    Serial.print(TF_LUNA_MAX_RANGE_CM);
    Serial.println(" cm");
    Serial.println("========================================");
    Serial.println();

    // Initialize I2C
    Wire.begin();  // D2 (SDA), D1 (SCL) are defaults on NodeMCU

    Serial.println("Initializing TF-Luna sensor...");

    // Test connection with a single reading
    int16_t testDist;
    if (tflI2C.getData(testDist, TF_LUNA_ADDR)) {
        Serial.print("TF-Luna connected! Initial reading: ");
        Serial.print(testDist);
        Serial.println(" cm");
    } else {
        Serial.println("WARNING: Could not read from TF-Luna!");
        Serial.println("Check wiring and ensure sensor is in I2C mode.");
    }

    Serial.println();
    Serial.println("Starting measurement cycles...");
    Serial.println();

    // Take first measurement immediately
    lastMeasurementTime = millis() - READING_INTERVAL_MS;
}

void loop() {
    unsigned long currentTime = millis();

    // Check if it's time for a new measurement cycle
    if (currentTime - lastMeasurementTime >= READING_INTERVAL_MS) {
        lastMeasurementTime = currentTime;

        // Take measurements
        bool success = takeMeasurements();

        // Print results
        printMeasurements();

        if (!success) {
            Serial.println("Note: Some readings failed. Check sensor connection.");
            Serial.println();
        }
    }
}
