/**
 * LilyGo T-Beam V1.2 (AXP2101) - River Level Monitoring with TF02-Pro LiDAR
 *
 * This node uses a Benewake TF02-Pro LiDAR sensor for distance measurement.
 * Sends real sensor data over LoRaWAN with deep sleep power management.
 *
 * Configuration:
 * - Frequency: AU915 (Australia/Brazil)
 * - Sensor: TF02-Pro LiDAR (UART interface) - Requires 5V-12V power!
 * - Power: 18650 battery (IMR18650) managed by AXP2101 PMIC
 * - Adaptive transmission interval: 1-15 minutes based on water level changes
 *
 * Wiring (TF02-Pro to T-Beam V1.2):
 *   VCC -> External 5V boost converter (TF02-Pro requires 5V-12V, 300mA peak)
 *   GND -> GND
 *   TX  -> GPIO 13 (Serial2 RX)
 *   RX  -> GPIO 14 (Serial2 TX)
 *
 * IMPORTANT: The TF02-Pro requires 5V-12V power supply!
 * The T-Beam V1.2's 5V pin only provides power when USB is connected.
 * For battery operation, you MUST use an external boost converter.
 *
 * Features:
 * - AXP2101 PMIC for accurate battery monitoring via I2C
 * - Adaptive sampling: faster when water level changes rapidly or exceeds threshold
 * - Historical minimum tracking with NVS persistent storage
 * - River level calculation from distance measurements
 * - Temperature compensation based on Mohammed et al. (2019)
 * - Statistical filtering for noise reduction (Kabi et al. 2023)
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <Preferences.h>  // NVS for persistent storage

// AXP2101 Power Management for T-Beam V1.2
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// Include the TF02-Pro sensor library
#include "TF02Pro/TF02Pro.h"

// Include filtering and adaptive sampling libraries
#include "filters/SensorFilters.h"
#include "filters/MovingAverage.h"
#include "filters/AdaptiveSampling.h"

// ============================================================================
// Configuration Options
// ============================================================================
#define ENABLE_SERIAL_DEBUG     true    // Enable serial output for debugging
#define SENSOR_WARMUP_MS        500     // Sensor warmup time
#define NUM_READINGS_AVG        7       // Number of readings for median filtering (odd recommended)
#define DEEP_SLEEP_ENABLED      true    // Enable deep sleep between transmissions
#define OUTLIER_THRESHOLD_PCT   20.0f   // Outlier threshold: reject readings >20% deviation from median

// ============================================================================
// Sensor Reliability Configuration
// ============================================================================
#define SENSOR_READ_RETRIES     3       // Number of retry attempts if all readings fail
#define SENSOR_RETRY_DELAY_MS   500     // Delay between retry attempts (ms)
#define READING_DELAY_MS        150     // Delay between individual readings (TF02-Pro at 10Hz = 100ms min)
#define SENSOR_ERROR_VALUE      -1      // Sentinel value indicating sensor error (vs actual 0)
#define MIN_VALID_READINGS      2       // Minimum valid readings required for a successful measurement

// ============================================================================
// Adaptive Sampling Configuration
// ============================================================================
// Based on Ragnoli et al. (2020) adaptive duty cycling approach
// NOTE: Intervals reduced for testing/debugging - increase for production deployment
#define TX_INTERVAL_MIN_SEC     30      // Minimum interval: 30 seconds (rapid change mode)
#define TX_INTERVAL_NORMAL_SEC  120     // Normal interval: 2 minutes (standard operation)
#define TX_INTERVAL_MAX_SEC     300     // Maximum interval: 5 minutes (stable/low battery mode)

// Thresholds for adaptive behavior
#define CHANGE_THRESHOLD_CM     5.0f    // Level change in cm to trigger faster sampling
#define CRITICAL_LEVEL_CM       50.0f   // River level (cm) above which we sample faster
#define RAPID_CHANGE_CM         10.0f   // Change in cm that indicates rapid rise (alert condition)
#define BATTERY_LOW_PERCENT     20      // Below this, use longer intervals to conserve power

// ============================================================================
// River Level Calculation
// ============================================================================
// The sensor measures distance FROM the sensor TO the water surface.
// River level = Sensor height - measured distance
// These values should be calibrated during installation
#define DEFAULT_SENSOR_HEIGHT_CM  400.0f  // Height of sensor above riverbed (calibrate on install)
#define HISTORICAL_MIN_KEY       "hist_min"  // NVS key for historical minimum distance

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
//   TF02-Pro VCC -> External 5V boost converter (NOT from T-Beam 5V pin on battery!)
//   TF02-Pro GND -> GND
#define TF02_RX_PIN     13  // GPIO 13 for Serial2 RX (receives from TF02-Pro TX) - WORKING
#define TF02_TX_PIN     14  // GPIO 14 for Serial2 TX (sends to TF02-Pro RX) - WORKING

// AXP2101 PMIC I2C pins (T-Beam V1.2)
#define PMU_I2C_SDA     21  // I2C SDA for AXP2101
#define PMU_I2C_SCL     22  // I2C SCL for AXP2101
#define PMU_IRQ_PIN     35  // AXP2101 interrupt pin

// Temperature sentinel value for invalid/unavailable readings
#define INVALID_TEMPERATURE_VALUE -128  // Sentinel value for invalid temperature

// ============================================================================
// Sensor Instance (TF02-Pro via UART)
// ============================================================================
// Note: ESP32 already has Serial2 defined, so we use it directly
// Serial2 is configured in setup() with begin() call
TF02Pro lidarSensor(Serial2, TF02_RX_PIN, TF02_TX_PIN);

// ============================================================================
// AXP2101 Power Management Instance
// ============================================================================
XPowersAXP2101 pmu;
bool pmuInitialized = false;

// ============================================================================
// NVS Persistent Storage
// ============================================================================
Preferences nvs;

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
// Adaptive Sampling State (stored in RTC memory to survive deep sleep)
// ============================================================================
RTC_DATA_ATTR float lastDistanceCm = 0.0f;       // Previous measurement for change detection
RTC_DATA_ATTR float historicalMinDistCm = 9999.0f; // Historical minimum distance (lowest water = max distance)
RTC_DATA_ATTR uint32_t currentIntervalSec = TX_INTERVAL_NORMAL_SEC;  // Current adaptive interval
RTC_DATA_ATTR float sensorHeightCm = DEFAULT_SENSOR_HEIGHT_CM;  // Configurable sensor height

// ============================================================================
// Sensor Data Structure (for LoRa transmission)
// ============================================================================
// Extended payload with filtered values and river level info
// Total size: 16 bytes (fits well within LoRaWAN payload limits)
struct __attribute__((packed)) SensorPayload {
    uint8_t sensorType;         // 1 = TF02-Pro LiDAR, 0xFF = error
    uint16_t rawDistanceMm;     // Raw distance in millimeters
    uint16_t maDistanceMm;      // Moving average distance in mm
    uint16_t kalmanDistanceMm;  // Kalman filtered distance in mm
    uint16_t riverLevelMm;      // Calculated river level in millimeters
    int16_t signalStrength;     // Signal strength (flux)
    int8_t temperature;         // Temperature in Celsius
    uint8_t batteryPercent;     // Battery level (0-100) from AXP2101
    uint8_t flags;              // Status flags
    // Flags: bit0=rapid_change, bit1=critical_level, bit2=battery_low, bit3=error, bit4=kalman_init
};

SensorPayload sensorData;

// ============================================================================
// Filter Chain Instance
// ============================================================================
SensorFilterChain<5> lidarFilterChain(KALMAN_LIDAR_NOISE, KALMAN_PROCESS_NOISE_NORMAL);

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent(ev_t ev);
void do_send(osjob_t* j);
bool readSensorData();
uint8_t getBatteryPercent();
uint16_t getBatteryVoltage();
float applyTemperatureCompensation(float rawDistance, int16_t sensorTemp);
void enterDeepSleep(uint32_t sleepSeconds);
float calculateMedian(float readings[], uint8_t count);
float filterOutliers(float readings[], uint8_t& validCount, float median, float thresholdPct);

// New functions for adaptive sampling and river level
bool initPMU();
uint32_t calculateAdaptiveInterval(float currentDistCm, uint8_t batteryPct);
float calculateRiverLevel(float distanceCm);
void updateHistoricalMinimum(float distanceCm);
void loadCalibrationFromNVS();
void saveCalibrationToNVS();

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
        // On cold boot, load historical minimum from NVS
        loadCalibrationFromNVS();
    }
    Serial.flush();
    delay(100);

    Serial.println(F("\n============================================="));
    Serial.println(F("T-Beam V1.2 (AXP2101) - TF02-Pro LiDAR Node"));
    Serial.println(F("=============================================\n"));
    Serial.flush();
    delay(100);

    // ========================================================================
    // Initialize AXP2101 Power Management Unit
    // ========================================================================
    Serial.println(F("Initializing AXP2101 PMU..."));
    if (initPMU()) {
        Serial.println(F("✓ AXP2101 PMU initialized successfully"));
        Serial.print(F("  Battery: "));
        Serial.print(getBatteryVoltage());
        Serial.print(F(" mV ("));
        Serial.print(getBatteryPercent());
        Serial.println(F("%)"));
    } else {
        Serial.println(F("✗ AXP2101 PMU NOT found! Battery readings unavailable"));
    }
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
            Serial.print(F("\n>>> Test Reading Summary: Raw="));
            Serial.print(sensorData.rawDistanceMm / 10.0f);
            Serial.print(F(" cm, MA="));
            Serial.print(sensorData.maDistanceMm / 10.0f);
            Serial.print(F(" cm, Kalman="));
            Serial.print(sensorData.kalmanDistanceMm / 10.0f);
            Serial.print(F(" cm, Temp: "));
            Serial.print(sensorData.temperature);
            Serial.print(F(" °C, Battery: "));
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
        Serial.print(F("\nEntering deep sleep for "));
        Serial.print(currentIntervalSec);
        Serial.println(F(" seconds (adaptive interval)..."));
        Serial.flush();
        enterDeepSleep(currentIntervalSec);
    }
#endif
}

// ============================================================================
// Sensor Reading Error Codes (for diagnostics)
// ============================================================================
enum SensorErrorCode {
    ERR_NONE = 0,
    ERR_NOT_INITIALIZED = 1,
    ERR_NO_VALID_READINGS = 2,
    ERR_ALL_READINGS_ZERO = 3,
    ERR_CHECKSUM_FAILURES = 4,
    ERR_TIMEOUT = 5,
    ERR_INSUFFICIENT_VALID = 6
};

// Store last error for diagnostics
SensorErrorCode lastSensorError = ERR_NONE;
uint8_t consecutiveFailures = 0;

// ============================================================================
// Read Sensor Data with Moving Average, Kalman Filter, and Adaptive Sampling
// Integrates filtering from SensorFilters.h library
// ============================================================================
bool readSensorData() {
    // Attempt sensor initialization retry if not yet initialized
    if (!sensorInitialized) {
        Serial.println(F("Sensor not initialized, attempting retry..."));
        delay(SENSOR_WARMUP_MS);

        if (lidarSensor.begin()) {
            sensorInitialized = true;
            Serial.println(F("Sensor initialization successful on retry!"));
            lidarSensor.setFrameRate(10);
            delay(200);
        } else {
            Serial.println(F("Sensor still not available, will send error payload"));
            lastSensorError = ERR_NOT_INITIALIZED;
            consecutiveFailures++;
            return false;
        }
    }

    Serial.println(F("\n========================================"));
    Serial.println(F("Reading TF02-Pro LiDAR sensor..."));
    Serial.println(F("========================================"));

    // Retry loop for entire reading process
    for (uint8_t attempt = 0; attempt < SENSOR_READ_RETRIES; attempt++) {
        if (attempt > 0) {
            Serial.print(F("\n>>> RETRY ATTEMPT "));
            Serial.print(attempt + 1);
            Serial.print(F("/"));
            Serial.println(SENSOR_READ_RETRIES);
            delay(SENSOR_RETRY_DELAY_MS);
            lidarSensor.clearBuffer();
            delay(100);
        }

        // Single reading for filter processing
        SensorReading reading = lidarSensor.read();

        if (!reading.valid || reading.distance_cm <= 0) {
            Serial.println(F("Invalid reading, retrying..."));
            lastSensorError = reading.valid ? ERR_ALL_READINGS_ZERO : ERR_CHECKSUM_FAILURES;
            continue;
        }

        // Apply temperature compensation
        float compensatedDist = reading.distance_cm;
        if (reading.temperature >= -40 && reading.temperature <= 85) {
            compensatedDist = applyTemperatureCompensation(reading.distance_cm, reading.temperature);
        }

        // Process through filter chain (moving average + Kalman)
        FilteredReading filtered = lidarFilterChain.process(
            compensatedDist,
            reading.signal_strength,
            (int8_t)reading.temperature
        );

        if (!filtered.valid) {
            Serial.println(F("Filter processing failed"));
            continue;
        }

        // Use Kalman filtered value for river level calculation
        float distanceForLevel = filtered.kalmanFiltered;
        float riverLevelCm = calculateRiverLevel(distanceForLevel);

        // Update historical minimum
        updateHistoricalMinimum(distanceForLevel);

        // Get battery percentage
        uint8_t batteryPct = getBatteryPercent();

        // Calculate adaptive interval for next transmission
        currentIntervalSec = calculateAdaptiveInterval(distanceForLevel, batteryPct);

        // Build status flags
        uint8_t flags = 0;
        float changeFromLast = abs(distanceForLevel - lastDistanceCm);
        if (changeFromLast >= RAPID_CHANGE_CM) flags |= 0x01;
        if (riverLevelCm >= CRITICAL_LEVEL_CM) flags |= 0x02;
        if (batteryPct < BATTERY_LOW_PERCENT) flags |= 0x04;
        if (lidarFilterChain.getKalmanFilter().isInitialized()) flags |= 0x10;

        // Update last distance for next comparison
        lastDistanceCm = distanceForLevel;

        // Fill payload structure with filtered values
        sensorData.sensorType = 1;  // TF02-Pro
        sensorData.rawDistanceMm = (uint16_t)(filtered.rawValue * 10.0f);
        sensorData.maDistanceMm = (uint16_t)(filtered.movingAverage * 10.0f);
        sensorData.kalmanDistanceMm = (uint16_t)(filtered.kalmanFiltered * 10.0f);
        sensorData.riverLevelMm = (uint16_t)(riverLevelCm * 10.0f);
        sensorData.signalStrength = filtered.signalStrength;
        sensorData.temperature = filtered.temperature;
        sensorData.batteryPercent = batteryPct;
        sensorData.flags = flags;

        Serial.println(F("\n========================================"));
        Serial.println(F("       SENSOR READING SUCCESS"));
        Serial.println(F("========================================"));
        Serial.print(F("Raw distance: "));
        Serial.print(filtered.rawValue);
        Serial.println(F(" cm"));
        Serial.print(F("Moving Average: "));
        Serial.print(filtered.movingAverage);
        Serial.print(F(" cm ("));
        Serial.print(filtered.maSampleCount);
        Serial.println(F(" samples)"));
        Serial.print(F("Kalman Filter: "));
        Serial.print(filtered.kalmanFiltered);
        Serial.print(F(" cm (+/- "));
        Serial.print(filtered.kalmanUncertainty);
        Serial.println(F(" cm)"));
        Serial.print(F("River level: "));
        Serial.print(riverLevelCm);
        Serial.println(F(" cm"));
        Serial.print(F("Historical min dist: "));
        Serial.print(historicalMinDistCm);
        Serial.println(F(" cm"));
        Serial.print(F("Signal Strength: "));
        Serial.println(filtered.signalStrength);
        Serial.print(F("Temperature: "));
        Serial.print(filtered.temperature);
        Serial.println(F(" C"));
        Serial.print(F("Battery: "));
        Serial.print(batteryPct);
        Serial.print(F("% ("));
        Serial.print(getBatteryVoltage());
        Serial.println(F(" mV)"));
        Serial.print(F("Flags: 0x"));
        Serial.print(flags, HEX);
        if (flags & 0x01) Serial.print(F(" [RAPID_CHANGE]"));
        if (flags & 0x02) Serial.print(F(" [CRITICAL_LEVEL]"));
        if (flags & 0x04) Serial.print(F(" [BATTERY_LOW]"));
        if (flags & 0x10) Serial.print(F(" [KALMAN_OK]"));
        Serial.println();
        Serial.print(F("Next interval: "));
        Serial.print(currentIntervalSec);
        Serial.println(F(" sec (adaptive)"));
        if (attempt > 0) {
            Serial.print(F("Success after "));
            Serial.print(attempt + 1);
            Serial.println(F(" attempts"));
        }
        Serial.println(F("========================================"));

        lastSensorError = ERR_NONE;
        consecutiveFailures = 0;
        return true;
    }

    // All retry attempts failed
    Serial.println(F("\n========================================"));
    Serial.println(F("    SENSOR READING FAILED"));
    Serial.println(F("========================================"));
    Serial.print(F("All "));
    Serial.print(SENSOR_READ_RETRIES);
    Serial.println(F(" attempts failed!"));
    Serial.print(F("Last error code: "));
    Serial.println(lastSensorError);
    consecutiveFailures++;
    Serial.print(F("Consecutive failures: "));
    Serial.println(consecutiveFailures);
    Serial.println(F("========================================"));

    return false;
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
// AXP2101 PMU Initialization
// ============================================================================
bool initPMU() {
    // Initialize I2C for AXP2101
    if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, PMU_I2C_SDA, PMU_I2C_SCL)) {
        Serial.println(F("  ERROR: AXP2101 not found on I2C bus"));
        pmuInitialized = false;
        return false;
    }

    // Verify chip ID (AXP2101 should return 0x47)
    uint8_t chipId = pmu.getChipID();
    Serial.print(F("  Chip ID: 0x"));
    Serial.println(chipId, HEX);

    if (chipId != 0x47) {
        Serial.println(F("  WARNING: Unexpected chip ID (expected 0x47 for AXP2101)"));
    }

    // Enable battery detection and voltage measurement
    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
    pmu.enableSystemVoltageMeasure();
    pmu.enableVbusVoltageMeasure();

    // Set charging parameters for IMR18650 battery
    // IMR18650 typically: 3.7V nominal, 4.2V max, 2000-3000mAh
    pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);  // Charge at 500mA
    pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);    // 4.2V full charge
    pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);  // 25mA termination
    pmu.enableChargerTerminationLimit();  // Enable charge termination

    // Set system minimum voltage (shutdown threshold)
    pmu.setSysPowerDownVoltage(2600);  // 2.6V minimum

    pmuInitialized = true;
    return true;
}

// ============================================================================
// Get Battery Percentage (from AXP2101)
// ============================================================================
uint8_t getBatteryPercent() {
    if (!pmuInitialized) {
        // Fallback: estimate from voltage if PMU not available
        return 50;  // Return 50% as unknown
    }

    // Check if battery is connected
    if (!pmu.isBatteryConnect()) {
        Serial.println(F("  WARNING: No battery detected by AXP2101"));
        return 0;
    }

    // Get percentage from AXP2101's fuel gauge
    // Note: May be inaccurate at first use, improves after charge/discharge cycles
    int percent = pmu.getBatteryPercent();
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    return (uint8_t)percent;
}

// ============================================================================
// Get Battery Voltage (from AXP2101)
// ============================================================================
uint16_t getBatteryVoltage() {
    if (!pmuInitialized) {
        return 0;
    }

    if (!pmu.isBatteryConnect()) {
        return 0;
    }

    // Returns voltage in millivolts
    return pmu.getBattVoltage();
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
        Serial.print(F(": Raw="));
        Serial.print(sensorData.rawDistanceMm / 10.0f);
        Serial.print(F(" MA="));
        Serial.print(sensorData.maDistanceMm / 10.0f);
        Serial.print(F(" Kalman="));
        Serial.print(sensorData.kalmanDistanceMm / 10.0f);
        Serial.println(F(" cm"));

        // Send the payload
        uint8_t result = LMIC_setTxData2(1, (uint8_t*)&sensorData, sizeof(sensorData), 0);

        if (result) {
            Serial.println(F("ERROR: Failed to queue packet!"));
        } else {
            Serial.println(F("Packet queued for transmission"));
        }
    } else {
        // Send error indicator if sensor failed, but always include battery and diagnostic info
        Serial.println(F("\n>>> SENDING ERROR PACKET <<<"));
        Serial.println(F("Sensor read failed after all retries"));

        uint8_t batteryPct = getBatteryPercent();

        // Use 0xFFFF to indicate error for all distance values
        sensorData.sensorType = 0xFF;  // Error indicator
        sensorData.rawDistanceMm = 0xFFFF;
        sensorData.maDistanceMm = 0xFFFF;
        sensorData.kalmanDistanceMm = 0xFFFF;
        sensorData.riverLevelMm = 0xFFFF;
        sensorData.signalStrength = (int16_t)lastSensorError;
        sensorData.temperature = INVALID_TEMPERATURE_VALUE;
        sensorData.batteryPercent = batteryPct;
        sensorData.flags = (batteryPct < BATTERY_LOW_PERCENT) ? 0x04 : 0x00;
        sensorData.flags |= 0x08;  // Bit 3: sensor error flag

        // Use longer interval on error to conserve power
        currentIntervalSec = TX_INTERVAL_NORMAL_SEC;

        Serial.println(F("Error payload contents:"));
        Serial.println(F("  sensorType: 0xFF (error)"));
        Serial.println(F("  all distances: 0xFFFF (error sentinel)"));
        Serial.print(F("  errorCode: "));
        Serial.println(lastSensorError);
        Serial.print(F("  consecutiveFailures: "));
        Serial.println(consecutiveFailures);
        Serial.print(F("  battery: "));
        Serial.print(batteryPct);
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
            // Use adaptive interval based on water level changes and battery
            if (DEEP_SLEEP_ENABLED) {
                Serial.print(F("Next transmission in "));
                Serial.print(currentIntervalSec);
                Serial.println(F(" seconds (deep sleep, adaptive)"));
                // Deep sleep will be entered in loop() after RX windows complete
            } else {
                Serial.print(F("Next transmission in "));
                Serial.print(currentIntervalSec);
                Serial.println(F(" seconds (adaptive)"));
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(currentIntervalSec), do_send);
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

// ============================================================================
// Calculate Adaptive Sampling Interval
// Based on Ragnoli et al. (2020) adaptive duty cycling approach
// ============================================================================
uint32_t calculateAdaptiveInterval(float currentDistCm, uint8_t batteryPct) {
    // Calculate river level for threshold checks
    float riverLevel = calculateRiverLevel(currentDistCm);

    // Calculate change from last reading
    float changeFromLast = abs(currentDistCm - lastDistanceCm);

    Serial.println(F("\n--- Adaptive Interval Calculation ---"));
    Serial.print(F("  Change from last: "));
    Serial.print(changeFromLast);
    Serial.println(F(" cm"));
    Serial.print(F("  River level: "));
    Serial.print(riverLevel);
    Serial.println(F(" cm"));
    Serial.print(F("  Battery: "));
    Serial.print(batteryPct);
    Serial.println(F("%"));

    // Priority 1: Battery conservation mode
    if (batteryPct < BATTERY_LOW_PERCENT) {
        Serial.println(F("  -> BATTERY_LOW: Using maximum interval"));
        return TX_INTERVAL_MAX_SEC;
    }

    // Priority 2: Rapid change detection (flood warning)
    if (changeFromLast >= RAPID_CHANGE_CM) {
        Serial.println(F("  -> RAPID_CHANGE: Using minimum interval"));
        return TX_INTERVAL_MIN_SEC;
    }

    // Priority 3: Critical level monitoring
    if (riverLevel >= CRITICAL_LEVEL_CM) {
        Serial.println(F("  -> CRITICAL_LEVEL: Using minimum interval"));
        return TX_INTERVAL_MIN_SEC;
    }

    // Priority 4: Moderate change detection
    if (changeFromLast >= CHANGE_THRESHOLD_CM) {
        // Scale interval based on change magnitude
        // More change = shorter interval
        float changeRatio = changeFromLast / RAPID_CHANGE_CM;
        if (changeRatio > 1.0f) changeRatio = 1.0f;

        uint32_t interval = TX_INTERVAL_NORMAL_SEC -
                           (uint32_t)((TX_INTERVAL_NORMAL_SEC - TX_INTERVAL_MIN_SEC) * changeRatio);
        Serial.print(F("  -> MODERATE_CHANGE: Using interval "));
        Serial.print(interval);
        Serial.println(F(" sec"));
        return interval;
    }

    // Default: Normal interval for stable conditions
    Serial.println(F("  -> STABLE: Using normal interval"));
    return TX_INTERVAL_NORMAL_SEC;
}

// ============================================================================
// Calculate River Level from Distance
// River level = Sensor height - measured distance
// ============================================================================
float calculateRiverLevel(float distanceCm) {
    // The sensor measures distance TO the water surface
    // River level = how high the water is from the riverbed
    // River level = Sensor height above riverbed - distance to water

    // If distance is greater than sensor height, water is below our reference point
    if (distanceCm >= sensorHeightCm) {
        return 0.0f;  // River level at or below reference
    }

    float riverLevel = sensorHeightCm - distanceCm;
    return riverLevel;
}

// ============================================================================
// Update Historical Minimum Distance
// The minimum distance = maximum water level recorded
// ============================================================================
void updateHistoricalMinimum(float distanceCm) {
    // Lower distance = higher water level (more important for flood detection)
    // We track the MINIMUM distance (maximum water level) seen
    if (distanceCm < historicalMinDistCm && distanceCm > 0) {
        Serial.print(F("  New historical minimum distance: "));
        Serial.print(distanceCm);
        Serial.print(F(" cm (was "));
        Serial.print(historicalMinDistCm);
        Serial.println(F(" cm)"));

        historicalMinDistCm = distanceCm;

        // Save to NVS periodically (every time we get a new minimum)
        saveCalibrationToNVS();
    }
}

// ============================================================================
// Load Calibration Data from NVS (Non-Volatile Storage)
// ============================================================================
void loadCalibrationFromNVS() {
    Serial.println(F("Loading calibration from NVS..."));

    nvs.begin("rivermon", false);  // Read-write mode

    // Load historical minimum distance
    if (nvs.isKey(HISTORICAL_MIN_KEY)) {
        historicalMinDistCm = nvs.getFloat(HISTORICAL_MIN_KEY, 9999.0f);
        Serial.print(F("  Loaded historical min: "));
        Serial.print(historicalMinDistCm);
        Serial.println(F(" cm"));
    } else {
        Serial.println(F("  No historical min found, using default"));
        historicalMinDistCm = 9999.0f;
    }

    // Load sensor height calibration if available
    if (nvs.isKey("sensor_height")) {
        sensorHeightCm = nvs.getFloat("sensor_height", DEFAULT_SENSOR_HEIGHT_CM);
        Serial.print(F("  Loaded sensor height: "));
        Serial.print(sensorHeightCm);
        Serial.println(F(" cm"));
    } else {
        Serial.print(F("  Using default sensor height: "));
        Serial.print(DEFAULT_SENSOR_HEIGHT_CM);
        Serial.println(F(" cm"));
        sensorHeightCm = DEFAULT_SENSOR_HEIGHT_CM;
    }

    nvs.end();
}

// ============================================================================
// Save Calibration Data to NVS
// ============================================================================
void saveCalibrationToNVS() {
    Serial.println(F("Saving calibration to NVS..."));

    nvs.begin("rivermon", false);

    // Save historical minimum
    nvs.putFloat(HISTORICAL_MIN_KEY, historicalMinDistCm);
    Serial.print(F("  Saved historical min: "));
    Serial.print(historicalMinDistCm);
    Serial.println(F(" cm"));

    // Save sensor height (in case it was modified)
    nvs.putFloat("sensor_height", sensorHeightCm);

    nvs.end();
}
