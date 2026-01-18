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
 * - Transmission interval: 15 minutes (900 seconds) - production configuration
 * 
 * Wiring (TF02-Pro to T-Beam V1.2):
 *   VCC -> 5V (or 3.3V if sensor supports it)
 *   GND -> GND
 *   TX  -> GPIO 3 (RX pin for Serial2)
 *   RX  -> GPIO 1 (TX pin for Serial2)
 * 
 * Academic Findings Applied:
 * - Temperature compensation based on Mohammed et al. (2019) and Tawalbeh et al. (2023): critical for sub-centimeter accuracy (0.1% per °C deviation)
 * - Extended range (22m) for larger rivers (santana_2024_development)
 * - Statistical filtering for noise reduction (Kabi et al. 2023)
 * - Median filtering and outlier removal for improved data quality
 * - Deep sleep for long-term battery operation (validated by Casals et al. 2017, Bouguera et al. 2018)
 * - Energy model validation: Casals et al. (2017) shows 2400mAh battery = 6-year lifespan with 5-min intervals at SF7
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <HardwareSerial.h>

// Include the TF02-Pro sensor library (from ../lib/sensors via -I../lib/sensors in platformio.ini)
#include "TF02Pro/TF02Pro.h"

// ============================================================================
// Configuration Options
// ============================================================================
#define ENABLE_SERIAL_DEBUG     true    // Enable serial output for debugging
#define TX_INTERVAL_SECONDS     60      // Send data every 1 minute (60 seconds) - reduced for testing
#define SENSOR_WARMUP_MS        500     // Sensor warmup time
#define NUM_READINGS_AVG        7       // Number of readings for median filtering (odd recommended)
#define DEEP_SLEEP_ENABLED      true    // Enable deep sleep between transmissions
#define OUTLIER_THRESHOLD_PCT   20.0f   // Outlier threshold: reject readings >20% deviation from median

// ============================================================================
// Sensor Test Mode (Optional - for testing without LoRaWAN gateway)
// ============================================================================
// Set to true to disable LoRaWAN and run continuous sensor readings via Serial
// Set to false (default) to use normal LoRaWAN operation
// When enabled, all LoRaWAN code is bypassed using conditional compilation
#define SENSOR_TEST_MODE        false   // Set to false for production LoRaWAN operation (set to true only for sensor testing without gateway)

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
// Pin Definitions (LilyGo T-Beam AXP2101 v1.2 / LoRa32)
// ============================================================================
// LoRa Module Pin Configuration (SX1276/8)
// These pins are specific to the T-Beam AXP2101 v1.2 board and were carefully
// configured to work with the LMIC library. DO NOT change these without verifying
// the board's schematic and pinout documentation.
const lmic_pinmap lmic_pins = {
    .nss = 18,              // Chip Select (CS) - SPI slave select for LoRa module
    .rxtx = LMIC_UNUSED_PIN, // Not used in this configuration
    .rst = 23,              // Reset pin - hardware reset for LoRa module
    .dio = {26, 33, 32},    // Digital I/O pins for LoRa interrupt handling:
                            //   DIO0 (GPIO 26): Used for TX/RX complete interrupts
                            //   DIO1 (GPIO 33): Used for RX timeout interrupts
                            //   DIO2 (GPIO 32): Used for FSK mode (not used in LoRa)
};

// UART pins for TF02-Pro (Serial2 on ESP32)
// T-Beam V1.2 pinout - WORKING CONFIGURATION:
// GPIO 14 (TX) and GPIO 13 (RX) - TESTED AND WORKING
// These pins avoid conflict with USB Serial (GPIO 1/3) and GPS (GPIO 12/34)
// 
// Wiring:
//   TF02-Pro TX -> GPIO 13 (Serial2 RX)
//   TF02-Pro RX -> GPIO 14 (Serial2 TX)
//   TF02-Pro VCC -> 5V
//   TF02-Pro GND -> GND
#define TF02_RX_PIN     13  // GPIO 13 for Serial2 RX (receives from TF02-Pro TX) - WORKING
#define TF02_TX_PIN     14  // GPIO 14 for Serial2 TX (sends to TF02-Pro RX) - WORKING

// Battery monitoring (if available on LilyGo board)
#define BATTERY_ADC_PIN 35  // Common ADC pin for battery voltage (adjust if needed)

// Temperature sentinel value for invalid/unavailable readings
#define INVALID_TEMPERATURE_VALUE -128  // Sentinel value for invalid temperature

// ============================================================================
// Sensor Instance (TF02-Pro via UART)
// ============================================================================
// Note: ESP32 already has Serial2 defined, so we use it directly
// Serial2 is configured in setup() with begin() call
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
bool transmissionCompleted = false;  // Track if we've completed at least one transmission after join

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
    // ========================================================================
    // CRITICAL DEBUG: Initialize Serial IMMEDIATELY
    // ========================================================================
    Serial.begin(115200);
    delay(500);  // Give serial time to initialize
    
    // Force flush to ensure data is sent
    Serial.flush();
    delay(100);
    
    // Print debug messages immediately
    Serial.println(F("\n\n\n"));
    Serial.println(F("============================================="));
    Serial.println(F("*** DEBUG: SETUP STARTED ***"));
    Serial.println(F("============================================="));
    Serial.flush();
    delay(100);
    
    // Check if we're waking from deep sleep
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    
    Serial.print(F("Wakeup reason: "));
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(F("DEEP SLEEP TIMER"));
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT0) {
        Serial.println(F("EXT0"));
    } else if (wakeup_reason == ESP_SLEEP_WAKEUP_EXT1) {
        Serial.println(F("EXT1"));
    } else {
        Serial.println(F("COLD BOOT / POWER ON"));
    }
    Serial.flush();
    delay(100);
    
    Serial.println(F("\n============================================="));
    Serial.println(F("LilyGo LoRa32 - TF02-Pro LiDAR Sensor Node"));
    Serial.println(F("=============================================\n"));
    Serial.flush();
    delay(100);
    
    // Initialize TF02-Pro sensor
    // Note: begin() will initialize Serial2 internally, so we don't initialize it here
    Serial.println(F("*** DEBUG: Starting TF02-Pro initialization ***"));
    Serial.flush();
    delay(100);
    
    Serial.println(F("Initializing TF02-Pro LiDAR sensor..."));
    Serial.print(F("  UART pins: RX=GPIO "));
    Serial.print(TF02_RX_PIN);
    Serial.print(F(" (TF02-Pro TX), TX=GPIO "));
    Serial.print(TF02_TX_PIN);
    Serial.println(F(" (TF02-Pro RX)"));
    Serial.println(F("  Baud rate: 115200"));
    Serial.println(F("  Waiting for sensor data (up to 2 seconds)..."));
    Serial.flush();
    delay(100);
    
    // Add warmup delay to allow sensor to stabilize (critical for first boot)
    // Sensor needs time to power up and initialize UART communication
    Serial.println(F("  Waiting for sensor warmup (1000ms)..."));
    Serial.flush();
    delay(1000);  // 1 second warmup delay for sensor stability
    
    // Try initialization with longer timeout
    Serial.println(F("*** DEBUG: Calling lidarSensor.begin() ***"));
    Serial.flush();
    delay(100);
    
    bool beginResult = lidarSensor.begin();
    
    Serial.print(F("*** DEBUG: lidarSensor.begin() returned: "));
    Serial.println(beginResult ? F("TRUE") : F("FALSE"));
    Serial.flush();
    delay(100);
    
    if (beginResult) {
        sensorInitialized = true;
        Serial.println(F("*** DEBUG: Sensor initialized successfully ***"));
        Serial.println(F("✓ TF02-Pro sensor initialized successfully"));
        
        // Set lower frame rate for power saving (10Hz instead of default 100Hz)
        lidarSensor.setFrameRate(10);
        
        // Take a test reading
        SensorReading testReading = lidarSensor.read();
        if (testReading.valid) {
            Serial.print(F("  Test reading: "));
            Serial.print(testReading.distance_cm);
            Serial.print(F(" cm, Temp: "));
            Serial.print(testReading.temperature);
            Serial.println(F(" °C"));
        } else {
            Serial.println(F("  Warning: Test reading failed, but sensor detected"));
        }
    } else {
        Serial.println(F("*** DEBUG: Sensor initialization FAILED ***"));
        Serial.println(F("✗ TF02-Pro sensor NOT detected!"));
        Serial.println(F("  Troubleshooting:"));
        Serial.println(F("    1. Check power: VCC->5V, GND->GND"));
        Serial.print(F("    2. Check TX wire: TF02-Pro TX -> GPIO "));
        Serial.println(TF02_RX_PIN);
        Serial.print(F("    3. Check RX wire: TF02-Pro RX -> GPIO "));
        Serial.println(TF02_TX_PIN);
        Serial.println(F("    4. Verify sensor is powered (LED should be on)"));
        Serial.println(F("    5. Try swapping TX/RX wires (might be inverted)"));
        Serial.flush();
        delay(100);
    }
    
#if !SENSOR_TEST_MODE
    // ========================================================================
    // LoRaWAN LMIC Initialization (WORKING CONFIGURATION - DO NOT MODIFY)
    // ========================================================================
    // This configuration was carefully tuned and tested. All settings below
    // are critical for proper LoRaWAN operation on the T-Beam AXP2101 v1.2.
    
    // Initialize the LMIC (LoRaWAN MAC In C) library
    // This sets up the OS scheduler and initializes the radio module
    os_init();
    
    // Reset LMIC state machine to ensure clean startup
    // This clears any previous state and prepares for network join
    LMIC_reset();
    
    // ========================================================================
    // Frequency Plan Configuration (AU915 - Australia/Brazil)
    // ========================================================================
    // The AU915 band uses 8 sub-bands (channels 0-7, 8-15, etc.)
    // Sub-band 1 (channels 8-15) is selected to avoid interference with
    // other LoRaWAN devices and comply with regional regulations.
    LMIC_selectSubBand(1);
    
    // ========================================================================
    // Data Rate and Power Configuration
    // ========================================================================
    // Network configuration: LoRaWAN Class A for minimal power consumption (Ballerini et al. 2020)
    // Ballerini et al. (2020) demonstrated that LoRaWAN consumes order of magnitude less energy
    // than NB-IoT for small, sporadic payloads typical of flood monitoring
    
    // Using SF7 (Spreading Factor 7) for optimal balance:
    // - Lower Time on Air (ToA) = lower energy consumption (Casals et al. 2017)
    // - Casals et al. (2017) showed that increasing SF from 7 to 12 increases energy by ~40x
    // - More gateways allow nodes to use lower SFs, directly extending battery life (Casals et al. 2017)
    // Power level 14 (14 dBm) provides good range while maintaining reasonable power consumption
    LMIC_setDrTxpow(DR_SF7, 14);
    
    // ========================================================================
    // Adaptive Data Rate (ADR) and Link Check Configuration
    // ========================================================================
    // ADR disabled (0): Node maintains fixed data rate (SF7)
    // This is important for predictable power consumption and transmission timing
    // Enabling ADR would allow the network to adjust data rate, but we want
    // consistent behavior for battery life estimation
    LMIC_setAdrMode(0);
    
    // Link check mode disabled (0): Node does not request link quality reports
    // This reduces overhead and power consumption. RSSI/SNR are still reported
    // in transmission acknowledgments when available.
    LMIC_setLinkCheckMode(0);
    
    // ========================================================================
    // Start Network Join Process
    // ========================================================================
    // Schedule the initial send job, which will trigger the join process
    // if the device is not yet joined, or send data if already joined
    do_send(&sendjob);
    
    Serial.println(F("\nSetup complete. Joining LoRaWAN network..."));
#else
    // ========================================================================
    // Sensor Test Mode: Skip LoRaWAN initialization
    // ========================================================================
    Serial.println(F("\n============================================="));
    Serial.println(F("SENSOR TEST MODE - LoRaWAN DISABLED"));
    Serial.println(F("============================================="));
    Serial.println(F("Running continuous sensor readings..."));
    Serial.println(F("Set SENSOR_TEST_MODE to false to enable LoRaWAN"));
    Serial.println(F("=============================================\n"));
    Serial.println(F("*** DEBUG: Setup complete, entering loop() ***"));
    Serial.flush();
    delay(100);
#endif
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    // ========================================================================
    // DEBUG: Loop heartbeat
    // ========================================================================
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat > 5000) {  // Every 5 seconds
        Serial.println(F("*** DEBUG: Loop running (heartbeat) ***"));
        Serial.flush();
        lastHeartbeat = millis();
    }
    
#if SENSOR_TEST_MODE
    // ========================================================================
    // Sensor Test Mode: Continuous sensor reading loop
    // ========================================================================
    if (sensorInitialized) {
        // Read sensor data and print to Serial
        if (readSensorData()) {
            Serial.print(F("\n>>> Test Reading Summary: "));
            Serial.print(sensorData.distanceMm / 10.0f);
            Serial.print(F(" cm, Temp: "));
            Serial.print(sensorData.temperature);
            Serial.print(F(" °C, Signal: "));
            Serial.print(sensorData.signalStrength);
            Serial.print(F(", Battery: "));
            Serial.print(sensorData.batteryPercent);
            Serial.println(F("%"));
        } else {
            Serial.println(F("Sensor read failed!"));
        }
        
        // Wait before next reading (5 seconds for test mode)
        Serial.println(F("\n--- Next reading in 5 seconds ---\n"));
        delay(5000);
    } else {
        Serial.println(F("Sensor not initialized! Waiting 5 seconds..."));
        delay(5000);
    }
#else
    // ========================================================================
    // Normal LoRaWAN Operation Mode
    // ========================================================================
    // Run LMIC OS scheduler (handles LoRaWAN state machine, events, timers)
    os_runloop_once();
    
    // If we've joined and completed a transmission, enter deep sleep
    // IMPORTANT: Only sleep after EV_TXCOMPLETE has fired, not immediately after join
    // This ensures the scheduled transmission actually happens before sleeping
    if (DEEP_SLEEP_ENABLED && joinedNetwork && transmissionCompleted && !(LMIC.opmode & OP_TXRXPEND)) {
        // Small delay to ensure transmission is complete and RX windows closed
        delay(2000);  // Increased delay to allow RX windows to complete
        Serial.println(F("\nEntering deep sleep..."));
        Serial.flush();
        enterDeepSleep(TX_INTERVAL_SECONDS);
    }
#endif
}

// ============================================================================
// Read Sensor Data with Temperature Compensation and Statistical Filtering
// Based on Mohammed et al. (2019) and Tawalbeh et al. (2023) for temperature compensation, and Kabi et al. (2023) for filtering
// ============================================================================
bool readSensorData() {
    // Attempt sensor initialization retry if not yet initialized
    // This handles cases where initialization failed on first boot but sensor is now ready
    if (!sensorInitialized) {
        Serial.println(F("Sensor not initialized, attempting retry..."));
        // Quick retry: sensor might be ready now after warmup
        if (lidarSensor.begin()) {
            sensorInitialized = true;
            Serial.println(F("Sensor initialization successful on retry!"));
            lidarSensor.setFrameRate(10);  // Set frame rate for power saving
        } else {
            Serial.println(F("Sensor still not available, will send error payload"));
            // Continue anyway - we'll return false but do_send() will handle it
        }
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
            float distance = reading.distance_cm;
            bool tempValid = false;
            
            // Check if temperature reading is valid (reasonable range: -40 to +85°C)
            // Temperature compensation is only applied if sensor temperature is valid
            if (reading.temperature >= -40 && reading.temperature <= 85) {
                tempValid = true;
                // Apply temperature compensation based on Mohammed et al. (2019) and Tawalbeh et al. (2023) findings
                // Temperature compensation is critical for sub-centimeter accuracy (Tawalbeh et al. 2023)
                distance = applyTemperatureCompensation(reading.distance_cm, reading.temperature);
            }
            // If temperature is invalid, use raw distance without compensation
            
            distances[validReadings] = distance;
            strengths[validReadings] = reading.signal_strength;
            temps[validReadings] = reading.temperature;  // Store temperature even if invalid (will be in payload)
            validReadings++;
            
            Serial.print(F("  Reading "));
            Serial.print(i + 1);
            Serial.print(F(": "));
            Serial.print(distance);
            Serial.print(F(" cm, temp: "));
            Serial.print(reading.temperature);
            if (!tempValid) {
                Serial.print(F(" °C (invalid - no compensation)"));
            } else {
                Serial.print(F(" °C"));
            }
            Serial.println();
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
// Temperature Compensation (Based on Mohammed et al. 2019 and Tawalbeh et al. 2023)
// ============================================================================
// NOTE: This function is only called when temperature reading is valid (-40 to +85°C)
// If temperature is invalid or not available, raw distance is used without compensation
float applyTemperatureCompensation(float rawDistance, int16_t sensorTemp) {
    // Based on Mohammed et al. (2019) and Tawalbeh et al. (2023): 
    // LiDAR sensor accuracy is strongly dependent on internal temperature
    // Temperature compensation is critical for achieving sub-centimeter accuracy (Mohammed et al. 2019)
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
// Power management validated by Casals et al. (2017) and Bouguera et al. (2018) energy models
// ============================================================================
void enterDeepSleep(uint32_t sleepSeconds) {
    Serial.print(F("Entering deep sleep for "));
    Serial.print(sleepSeconds);
    Serial.println(F(" seconds..."));
    Serial.flush();
    
    // Configure RTC timer wakeup
    // Deep sleep energy: ~10-150 µA (Casals et al. 2017, Bouguera et al. 2018)
    // Energy model: E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx (Bouguera et al. 2018)
    esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);  // Convert to microseconds
    
    // Enter deep sleep
    // Note: This function never returns - the ESP32 will restart after wakeup
    // Power consumption drops from 300-450mA (active) to ~10µA (deep sleep)
    // This enables 6-12 month battery life with 2000mAh battery (validated by energy models)
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
// LMIC Send Job (Transmission Handler)
// ============================================================================
// This function is called by the LMIC OS scheduler to handle data transmission.
// It manages the complete transmission cycle: checking join status, reading
// sensor data, formatting payload, and queuing the packet for transmission.
// 
// The function is designed to be called repeatedly by the scheduler until
// transmission is successful or the device joins the network.
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
        // Send error indicator if sensor failed, but always include battery and temperature
        Serial.println(F("Sensor read failed, sending error packet with available data"));
        
        // Always populate battery and temperature in error payload
        sensorData.sensorType = 0xFF;  // Error indicator
        sensorData.distanceMm = 0;
        sensorData.signalStrength = 0;
        sensorData.temperature = INVALID_TEMPERATURE_VALUE;  // Sentinel value for unavailable temp
        sensorData.batteryPercent = getBatteryPercent();     // Always read battery
        sensorData.readingCount = 0;
        
        Serial.print(F("  Error payload: Battery="));
        Serial.print(sensorData.batteryPercent);
        Serial.println(F("%"));
        
        uint8_t result = LMIC_setTxData2(1, (uint8_t*)&sensorData, sizeof(sensorData), 0);
        if (result) {
            Serial.println(F("ERROR: Failed to queue error packet!"));
        } else {
            Serial.println(F("Error packet queued for transmission"));
        }
    }
}

// ============================================================================
// LMIC Event Handler (LoRaWAN State Machine)
// ============================================================================
// This function handles all LoRaWAN events from the LMIC library.
// It is called automatically by the LMIC OS scheduler when events occur
// (join complete, transmission complete, join failed, etc.).
//
// Key events handled:
// - EV_JOINING: Device is attempting to join the network
// - EV_JOINED: Device successfully joined (receives DevAddr)
// - EV_JOIN_FAILED: Join attempt failed (will retry)
// - EV_TXCOMPLETE: Transmission finished (may include ACK or downlink data)
// - EV_TXSTART: Transmission started
// - EV_LINK_DEAD: Connection to gateway lost
//
// This event handler is critical for proper LoRaWAN operation and should
// not be modified without understanding the LMIC state machine.
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
            transmissionCompleted = false;  // Reset flag - we haven't sent anything yet
            
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
            
            // Mark that we've completed a transmission (critical for deep sleep logic)
            transmissionCompleted = true;
            
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
                // Deep sleep will be entered in loop() after RX windows complete
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
