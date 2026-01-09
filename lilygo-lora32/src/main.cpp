/**
 * LilyGo LoRa32 - River Level Monitoring with TF02-Pro LiDAR
 * 
 * This node uses a Benewake TF02-Pro LiDAR sensor for distance measurement.
 * Sends real sensor data over LoRaWAN every 15 minutes with deep sleep power management.
 * 
 * Configuration:
 * - Frequency: AU915 (Australia/Brazil)
 * - Sensor: TF02-Pro LiDAR (UART interface)
 * - Power: Battery powered from board's LiPo connector with deep sleep
 * - Transmission interval: 15 minutes (as per thesis)
 * 
 * Wiring (TF02-Pro to LilyGo LoRa32):
 *   VCC -> 5V (or 3.3V if sensor supports it)
 *   GND -> GND
 *   TX  -> GPIO 16 (RX pin for Serial2)
 *   RX  -> GPIO 17 (TX pin for Serial2)
 * 
 * Academic Findings Applied:
 * - Temperature compensation based on paul_2020_a findings (0.1% per °C)
 * - Extended range (22m) for larger rivers (santana_2024_development)
 * - Statistical filtering for noise reduction (Kabi et al. 2023)
 * - Median filtering and outlier removal for improved data quality
 * - Deep sleep for long-term battery operation
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <HardwareSerial.h>

// Include the TF02-Pro sensor library
#include "sensors/TF02Pro/TF02Pro.h"

// ============================================================================
// Configuration Options
// ============================================================================
#define ENABLE_SERIAL_DEBUG     true    // Enable serial output for debugging
#define TX_INTERVAL_SECONDS     900     // Send data every 15 minutes (900 seconds)
#define SENSOR_WARMUP_MS        500     // Sensor warmup time
#define NUM_READINGS_AVG        7       // Number of readings for median filtering (odd recommended)
#define DEEP_SLEEP_ENABLED      true    // Enable deep sleep between transmissions
#define OUTLIER_THRESHOLD_PCT   20.0f   // Outlier threshold: reject readings >20% deviation from median

// ============================================================================
// LoRaWAN Configuration - TTN Device: lilygo-river-lora
// ============================================================================
// JoinEUI/AppEUI - From TTN device: 0000000000000000
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI - From TTN device: 70B3D57ED0074FD2
// In code (LSB first - reversed): D2 4F 07 D0 7E D5 B3 70
static const u1_t PROGMEM DEVEUI[8] = { 
  0xD2, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey - From TTN device: E85C567896B872DFBF87687648FCB5B3
static const u1_t PROGMEM APPKEY[16] = { 
  0xE8, 0x5C, 0x56, 0x78, 0x96, 0xB8, 0x72, 0xDF,
  0xBF, 0x87, 0x68, 0x76, 0x48, 0xFC, 0xB5, 0xB3
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (LilyGo LoRa32)
// ============================================================================
// LoRa pins
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

// UART pins for TF02-Pro (Serial2 on ESP32)
#define TF02_RX_PIN     16  // GPIO 16 for Serial2 RX
#define TF02_TX_PIN     17  // GPIO 17 for Serial2 TX

// Battery monitoring (if available on LilyGo board)
#define BATTERY_ADC_PIN 35  // Common ADC pin for battery voltage (adjust if needed)

// ============================================================================
// Sensor Instance (TF02-Pro via UART)
// ============================================================================
HardwareSerial Serial2(2);  // Use Serial2 for TF02-Pro
TF02Pro lidarSensor(Serial2, TF02_RX_PIN, TF02_TX_PIN);

// ============================================================================
// Timing and Status Variables
// ============================================================================
static osjob_t sendjob;
uint32_t packetCount = 0;
int16_t lastRSSI = 0;
int8_t lastSNR = 0;
bool sensorInitialized = false;
bool joinedNetwork = false;

// ============================================================================
// Sensor Data Structure (for LoRa transmission)
// ============================================================================
struct __attribute__((packed)) SensorPayload {
    uint8_t sensorType;         // 1 = TF02-Pro LiDAR
    uint16_t distanceMm;        // Distance in millimeters
    int16_t signalStrength;     // Signal strength (flux)
    int8_t temperature;         // Temperature in Celsius (for compensation)
    uint8_t batteryPercent;     // Battery level (0-100)
    uint8_t readingCount;       // Number of valid readings in average
};

SensorPayload sensorData;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent(ev_t ev);
void do_send(osjob_t* j);
bool readSensorData();
uint8_t getBatteryPercent();
float applyTemperatureCompensation(float rawDistance, int16_t sensorTemp);
void enterDeepSleep(uint32_t sleepSeconds);
float calculateMedian(float readings[], uint8_t count);
float filterOutliers(float readings[], uint8_t& validCount, float median, float thresholdPct);

// ============================================================================
// Setup
// ============================================================================
void setup() {
    // Check if we're waking from deep sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    if (ENABLE_SERIAL_DEBUG) {
        Serial.begin(115200);
        delay(1000);
        Serial.println(F("\n============================================="));
        Serial.println(F("LilyGo LoRa32 - TF02-Pro LiDAR Sensor Node"));
        Serial.println(F("=============================================\n"));
        
        if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
            Serial.println(F("Woke up from deep sleep"));
        } else {
            Serial.println(F("Cold boot"));
        }
    }
    
    // Initialize Serial2 for TF02-Pro (115200 baud)
    Serial2.begin(115200, SERIAL_8N1, TF02_RX_PIN, TF02_TX_PIN);
    delay(SENSOR_WARMUP_MS);
    
    // Initialize TF02-Pro sensor
    Serial.println(F("Initializing TF02-Pro LiDAR sensor..."));
    if (lidarSensor.begin()) {
        sensorInitialized = true;
        Serial.println(F("✓ TF02-Pro sensor initialized successfully"));
        
        // Set lower frame rate for power saving (10Hz instead of default 100Hz)
        lidarSensor.setFrameRate(10);
        
        // Take a test reading
        SensorReading testReading = lidarSensor.read();
        if (testReading.valid) {
            Serial.print(F("  Test reading: "));
            Serial.print(testReading.distance_cm);
            Serial.println(F(" cm"));
        }
    } else {
        Serial.println(F("✗ TF02-Pro sensor NOT detected!"));
        Serial.println(F("  Check wiring: VCC->5V, GND->GND, TX->GPIO16, RX->GPIO17"));
    }
    
    // Initialize LMIC
    os_init();
    LMIC_reset();
    
    // Set frequency plan to AU915
    LMIC_selectSubBand(1);
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
    
    // Start join process
    do_send(&sendjob);
    
    Serial.println(F("\nSetup complete. Joining LoRaWAN network..."));
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    os_runloop_once();
    
    // If we've joined and transmitted, enter deep sleep
    // This will only execute if deep sleep is enabled and we've completed a transmission cycle
    if (DEEP_SLEEP_ENABLED && joinedNetwork && !(LMIC.opmode & OP_TXRXPEND)) {
        // Small delay to ensure transmission is complete
        delay(1000);
        Serial.println(F("\nEntering deep sleep..."));
        Serial.flush();
        enterDeepSleep(TX_INTERVAL_SECONDS);
    }
}

// ============================================================================
// Read Sensor Data with Temperature Compensation and Statistical Filtering
// Based on paul_2020_a (temperature) and Kabi et al. 2023 (filtering)
// ============================================================================
bool readSensorData() {
    if (!sensorInitialized) {
        Serial.println(F("Sensor not initialized!"));
        return false;
    }
    
    Serial.println(F("\nReading TF02-Pro LiDAR sensor..."));
    
    // Collect readings with temperature compensation
    float distances[NUM_READINGS_AVG];
    int16_t strengths[NUM_READINGS_AVG];
    int16_t temps[NUM_READINGS_AVG];
    uint8_t validReadings = 0;
    
    // Take multiple readings
    for (int i = 0; i < NUM_READINGS_AVG; i++) {
        SensorReading reading = lidarSensor.read();
        
        if (reading.valid && reading.distance_cm > 0) {
            // Apply temperature compensation based on paul_2020_a findings
            float compensatedDistance = applyTemperatureCompensation(
                reading.distance_cm, 
                reading.temperature
            );
            
            distances[validReadings] = compensatedDistance;
            strengths[validReadings] = reading.signal_strength;
            temps[validReadings] = reading.temperature;
            validReadings++;
            
            Serial.print(F("  Reading "));
            Serial.print(i + 1);
            Serial.print(F(": "));
            Serial.print(compensatedDistance);
            Serial.print(F(" cm, temp: "));
            Serial.print(reading.temperature);
            Serial.println(F(" °C"));
        }
        
        delay(100);  // Delay between readings (TF02-Pro at 10Hz needs ~100ms)
    }
    
    if (validReadings == 0) {
        Serial.println(F("No valid sensor readings!"));
        return false;
    }
    
    // Calculate median for robust filtering (Kabi et al. 2023)
    float medianDistance = calculateMedian(distances, validReadings);
    Serial.print(F("\n  Median: "));
    Serial.print(medianDistance);
    Serial.println(F(" cm"));
    
    // Filter outliers based on deviation from median
    uint8_t filteredCount = validReadings;
    float avgDistance = filterOutliers(distances, filteredCount, medianDistance, OUTLIER_THRESHOLD_PCT);
    
    // Use median if filtering removed too many samples (fallback)
    if (filteredCount < validReadings / 2) {
        Serial.println(F("  Warning: Too many outliers, using median instead"));
        avgDistance = medianDistance;
        filteredCount = validReadings;
    } else {
        Serial.print(F("  Outliers removed: "));
        Serial.print(validReadings - filteredCount);
        Serial.print(F("/"));
        Serial.println(validReadings);
    }
    
    // Calculate averages for signal strength and temperature (from all valid readings)
    int16_t sumStrength = 0;
    int16_t sumTemp = 0;
    for (uint8_t i = 0; i < validReadings; i++) {
        sumStrength += strengths[i];
        sumTemp += temps[i];
    }
    int16_t avgStrength = sumStrength / validReadings;
    int8_t avgTemp = sumTemp / validReadings;
    
    // Fill payload structure
    sensorData.sensorType = 1;  // TF02-Pro
    sensorData.distanceMm = (uint16_t)(avgDistance * 10);  // Convert cm to mm
    sensorData.signalStrength = avgStrength;
    sensorData.temperature = avgTemp;
    sensorData.batteryPercent = getBatteryPercent();
    sensorData.readingCount = filteredCount;
    
    Serial.println(F("\n--- Sensor Reading Summary ---"));
    Serial.print(F("Filtered Distance: "));
    Serial.print(avgDistance);
    Serial.println(F(" cm"));
    Serial.print(F("Signal Strength: "));
    Serial.println(avgStrength);
    Serial.print(F("Temperature: "));
    Serial.print(avgTemp);
    Serial.println(F(" °C"));
    Serial.print(F("Battery: "));
    Serial.print(sensorData.batteryPercent);
    Serial.println(F(" %"));
    Serial.print(F("Valid readings: "));
    Serial.print(filteredCount);
    Serial.print(F("/"));
    Serial.println(validReadings);
    Serial.println(F("------------------------------"));
    
    return true;
}

// ============================================================================
// Temperature Compensation (Based on paul_2020_a findings)
// ============================================================================
float applyTemperatureCompensation(float rawDistance, int16_t sensorTemp) {
    // Based on paul_2020_a: "sensor's accuracy was strongly dependent on its internal temperature"
    // Apply correction factor based on sensor temperature
    // Reference temperature: 25°C
    // Correction factor: approximately 0.1% per degree Celsius deviation
    float tempDeviation = sensorTemp - 25.0f;
    float correctionFactor = 1.0f + (tempDeviation * 0.001f);  // 0.1% per °C
    
    return rawDistance * correctionFactor;
}

// ============================================================================
// Get Battery Percentage
// ============================================================================
uint8_t getBatteryPercent() {
    // LilyGo boards may have battery voltage divider on ADC pin
    // Typical range: 3.0V (empty) to 4.2V (full) for LiPo
    // Formula: percentage = ((voltage - 3.0) / (4.2 - 3.0)) * 100
    
    #ifdef BATTERY_ADC_PIN
        // Read battery voltage via ADC
        int adcValue = analogRead(BATTERY_ADC_PIN);
        
        // Convert ADC reading to voltage
        // ESP32 ADC: 12-bit (0-4095) with 3.3V reference
        // If voltage divider is 2:1, multiply by 2
        float voltage = (adcValue / 4095.0f) * 3.3f * 2.0f;  // Adjust multiplier if needed
        
        // Calculate percentage
        if (voltage < 3.0f) return 0;
        if (voltage > 4.2f) return 100;
        
        uint8_t percent = (uint8_t)(((voltage - 3.0f) / 1.2f) * 100.0f);
        return percent;
    #else
        // Placeholder if battery monitoring not available
        return 100;
    #endif
}

// ============================================================================
// Enter Deep Sleep
// ============================================================================
void enterDeepSleep(uint32_t sleepSeconds) {
    Serial.print(F("Entering deep sleep for "));
    Serial.print(sleepSeconds);
    Serial.println(F(" seconds..."));
    Serial.flush();
    
    // Configure RTC timer wakeup
    esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);  // Convert to microseconds
    
    // Enter deep sleep
    // Note: This function never returns - the ESP32 will restart after wakeup
    esp_deep_sleep_start();
}

// ============================================================================
// Calculate Median (for robust filtering)
// Based on Kabi et al. (2023): Statistical filtering for noise reduction
// ============================================================================
float calculateMedian(float readings[], uint8_t count) {
    if (count == 0) return 0;
    if (count == 1) return readings[0];
    
    // Simple bubble sort for small arrays
    float sorted[NUM_READINGS_AVG];
    for (uint8_t i = 0; i < count; i++) {
        sorted[i] = readings[i];
    }
    
    // Bubble sort
    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = 0; j < count - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }
    
    // Return median
    if (count % 2 == 0) {
        // Even number of samples: average middle two
        return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
    } else {
        // Odd number: return middle value
        return sorted[count / 2];
    }
}

// ============================================================================
// Filter Outliers (based on deviation from median)
// Based on Kabi et al. (2023): Remove noise from turbulence and debris
// ============================================================================
float filterOutliers(float readings[], uint8_t& validCount, float median, float thresholdPct) {
    float sum = 0;
    uint8_t filteredIdx = 0;
    
    float threshold = median * (thresholdPct / 100.0f);
    
    // Filter and calculate average in one pass
    for (uint8_t i = 0; i < validCount; i++) {
        float deviation = abs(readings[i] - median);
        
        if (deviation <= threshold) {
            sum += readings[i];
            filteredIdx++;
        }
    }
    
    validCount = filteredIdx;
    
    if (validCount == 0) {
        return median;  // Fallback to median if all filtered
    }
    
    return sum / validCount;
}

// ============================================================================
// LMIC Send Job
// ============================================================================
void do_send(osjob_t* j) {
    // Check if LMIC is busy
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, waiting..."));
        os_setTimedCallback(j, os_getTime() + sec2osticks(1), do_send);
        return;
    }
    
    // Check if we have joined
    if (LMIC.devaddr == 0) {
        Serial.println(F("Not joined yet, starting join..."));
        LMIC_startJoining();
        return;
    }
    
    // Mark as joined
    if (!joinedNetwork) {
        joinedNetwork = true;
    }
    
    // Read sensor data
    if (readSensorData()) {
        packetCount++;
        
        Serial.print(F(">>> Sending packet #"));
        Serial.print(packetCount);
        Serial.print(F(": "));
        Serial.print(sensorData.distanceMm / 10.0f);
        Serial.println(F(" cm"));
        
        // Send the payload
        uint8_t result = LMIC_setTxData2(1, (uint8_t*)&sensorData, sizeof(sensorData), 0);
        
        if (result) {
            Serial.println(F("ERROR: Failed to queue packet!"));
        } else {
            Serial.println(F("Packet queued for transmission"));
        }
    } else {
        // Send error indicator if sensor failed
        Serial.println(F("Sensor read failed, sending error packet"));
        sensorData.sensorType = 0xFF;  // Error indicator
        sensorData.distanceMm = 0;
        sensorData.readingCount = 0;
        LMIC_setTxData2(1, (uint8_t*)&sensorData, sizeof(sensorData), 0);
    }
}

// ============================================================================
// LMIC Event Handler
// ============================================================================
void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(F(": "));
    
    switch(ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING - Joining network..."));
            break;
            
        case EV_JOINED:
            Serial.println(F("EV_JOINED - Successfully joined!"));
            Serial.print(F("DevAddr: "));
            Serial.println(LMIC.devaddr, HEX);
            
            LMIC_setLinkCheckMode(0);
            joinedNetwork = true;
            
            // Schedule first data transmission
            Serial.println(F("Scheduling first sensor transmission in 2 seconds..."));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
            
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED - Join failed!"));
            // Retry join after delay (but don't sleep yet)
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
            break;
            
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE - Join request sent"));
            break;
            
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART - Transmitting..."));
            break;
            
        case EV_RXSTART:
            Serial.println(F("EV_RXSTART - Receiving..."));
            break;
            
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE - Transmission complete"));
            
            if (LMIC.txrxFlags & TXRX_ACK) {
                Serial.println(F(">>> Received ACK"));
            }
            if (LMIC.dataLen) {
                Serial.print(F(">>> Received "));
                Serial.print(LMIC.dataLen);
                Serial.println(F(" bytes downlink"));
            }
            
            lastRSSI = LMIC.rssi;
            lastSNR = LMIC.snr;
            Serial.print(F("RSSI: "));
            Serial.print(lastRSSI);
            Serial.print(F(" dBm, SNR: "));
            Serial.print(lastSNR);
            Serial.println(F(" dB"));
            
            // Schedule next transmission (or enter deep sleep)
            if (DEEP_SLEEP_ENABLED) {
                Serial.print(F("Next transmission in "));
                Serial.print(TX_INTERVAL_SECONDS);
                Serial.println(F(" seconds (deep sleep)"));
                // Deep sleep will be entered in loop() after a short delay
            } else {
                Serial.print(F("Next transmission in "));
                Serial.print(TX_INTERVAL_SECONDS);
                Serial.println(F(" seconds"));
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL_SECONDS), do_send);
            }
            break;
            
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            joinedNetwork = false;
            break;
            
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
            
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned)ev);
            break;
    }
}
