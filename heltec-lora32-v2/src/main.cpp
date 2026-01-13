/**
 * Heltec WiFi LoRa 32 V2 - River Level Monitoring with Ultrasonic Sensor
 * 
 * This node uses ultrasonic sensors (HC-SR04 or JSN-SR04T) for distance measurement.
 * Sends real sensor data over LoRaWAN every 15 minutes with deep sleep power management.
 * Display shows: Status, Distance, Signal Quality
 * 
 * Configuration:
 * - Frequency: AU915 (Australia/Brazil)
 * - Sensor: JSN-SR04T (Waterproof Ultrasonic, GPIO interface)
 * - Power: External battery board with deep sleep
 * - Transmission interval: 15 minutes (as per thesis)
 * 
 * Wiring (Ultrasonic to Heltec LoRa32 V2):
 *   VCC  -> 5V (or 3.3V for JSN-SR04T)
 *   GND  -> GND
 *   TRIG -> GPIO 13
 *   ECHO -> GPIO 12
 * 
 * Academic Findings Applied:
 * - Ultrasonic sensors validated for water level monitoring (Mohammed et al. 2019, Tawalbeh et al. 2023)
 * - Temperature compensation critical: Mohammed et al. (2019) and Tawalbeh et al. (2023) show 1-2cm drift per 10°C without compensation
 * - Speed-of-sound formula: v(T) = 331.3 + 0.606 * θ (Mohammed et al. 2019, Tawalbeh et al. 2023)
 * - Statistical filtering for noise from turbulence/debris (Kabi et al. 2023)
 * - Median filtering and outlier removal for improved data quality
 * - Deep sleep for long-term battery operation (validated by Casals et al. 2017, Bouguera et al. 2018)
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8g2lib.h>

// Include sensor libraries
#include "sensors/HCSR04/HCSR04.h"
#include "sensors/JSNSR04T/JSNSR04T.h"
#include "DHTesp.h"  // DHT11/DHT22 temperature sensor library (beegee-tokyo/DHTesp)

// ============================================================================
// Configuration Options
// ============================================================================
#define ENABLE_SERIAL_DEBUG     true    // Enable serial output for debugging
#define TX_INTERVAL_SECONDS     900     // Send data every 15 minutes (900 seconds)
#define NUM_READINGS_AVG        7       // Number of readings for median filtering (odd number recommended)
#define READING_DELAY_MS        60      // Delay between readings (ms)
#define DEEP_SLEEP_ENABLED      true    // Enable deep sleep between transmissions
#define OUTLIER_THRESHOLD_PCT   20.0f   // Outlier threshold: reject readings >20% deviation from median
#define DEFAULT_TEMPERATURE     25.0f   // Default temperature (Celsius) if sensor unavailable

// ============================================================================
// Sensor Selection - Uncomment ONE of the following
// ============================================================================
// #define USE_HCSR04              // Standard HC-SR04 (indoor, 2-400cm)
#define USE_JSNSR04T         // Waterproof JSN-SR04T (outdoor, 25-450cm) - DEFAULT

// ============================================================================
// LoRaWAN Configuration - TTN Device: heltec-river-lora
// ============================================================================
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI - From TTN device: 70B3D57ED0074FC9
static const u1_t PROGMEM DEVEUI[8] = { 
  0xC9, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey - From TTN device: 24BFC553D19785B23D3CA9C6BE15B123
static const u1_t PROGMEM APPKEY[16] = { 
  0x24, 0xBF, 0xC5, 0x53, 0xD1, 0x97, 0x85, 0xB2,
  0x3D, 0x3C, 0xA9, 0xC6, 0xBE, 0x15, 0xB1, 0x23
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (Heltec WiFi LoRa 32 V2)
// ============================================================================
// LoRa pins
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 35, 34},
};

// Ultrasonic sensor pins (using pins that don't conflict with LoRa/OLED)
#define ULTRASONIC_TRIG_PIN     13
#define ULTRASONIC_ECHO_PIN     12

// Battery monitoring (Heltec V2 has voltage divider on GPIO 37)
#define BATTERY_ADC_PIN         37      // Battery voltage monitoring pin

// Temperature sensor pin (DHT11) - using GPIO that doesn't conflict with LoRa/OLED/Ultrasonic
#define DHT_PIN                 27      // DHT11 DATA pin (GPIO 27, 25, or other available pin)
#define DHT_TYPE                DHT11   // DHT11 (or DHT22 if available)

// ============================================================================
// OLED Display (Heltec WiFi LoRa 32 V2)
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// ============================================================================
// Sensor Instances
// ============================================================================
#ifdef USE_HCSR04
HCSR04 ultrasonicSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
const char* SENSOR_NAME = "HC-SR04";
const uint8_t SENSOR_TYPE_ID = 2;  // HC-SR04
#else
JSNSR04T ultrasonicSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
const char* SENSOR_NAME = "JSN-SR04T";
const uint8_t SENSOR_TYPE_ID = 3;  // JSN-SR04T
#endif

// ============================================================================
// Temperature Sensor Instance (DHT11 for ultrasonic compensation)
// ============================================================================
DHTesp dhtSensor;  // DHT11 temperature sensor for speed-of-sound compensation
bool temperatureSensorAvailable = false;

// ============================================================================
// Timing and Status Variables
// ============================================================================
static osjob_t sendjob;
uint32_t packetCount = 0;
int16_t lastRSSI = 0;
int8_t lastSNR = 0;
bool sensorInitialized = false;
bool joinedNetwork = false;

// Display status
enum Status {
    STATUS_INIT,
    STATUS_JOINING,
    STATUS_CONNECTED,
    STATUS_SENDING,
    STATUS_SENSOR_ERROR
};
Status currentStatus = STATUS_INIT;
float lastDistance = 0;

// ============================================================================
// Sensor Data Structure (for LoRa transmission)
// ============================================================================
struct __attribute__((packed)) SensorPayload {
    uint8_t sensorType;         // 2 = HC-SR04, 3 = JSN-SR04T
    uint16_t distanceMm;        // Distance in millimeters
    int16_t signalStrength;     // Not available for ultrasonic (0)
    int8_t temperature;         // Ambient temperature (for speed of sound correction)
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
void updateDisplay();
void enterDeepSleep(uint32_t sleepSeconds);
float getAmbientTemperature();
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
        Serial.println(F("\n=============================================="));
        Serial.println(F("Heltec LoRa32 V2 - Ultrasonic Sensor Node"));
        Serial.print(F("Sensor: "));
        Serial.println(SENSOR_NAME);
        Serial.println(F("==============================================\n"));
        
        if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
            Serial.println(F("Woke up from deep sleep"));
        } else {
            Serial.println(F("Cold boot"));
        }
    }
    
    // Initialize OLED Display
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("Initializing...");
    u8g2.setCursor(0, 26);
    u8g2.print(SENSOR_NAME);
    u8g2.sendBuffer();
    
    // Initialize DHT11 temperature sensor (Mohammed et al. 2019, Tawalbeh et al. 2023)
    // Temperature sensor must be physically located near ultrasonic transducer for accurate air column density measurement
    Serial.print(F("Initializing DHT11 temperature sensor on GPIO "));
    Serial.print(DHT_PIN);
    Serial.println(F("..."));
    dhtSensor.setup(DHT_PIN, DHTesp::DHT11);  // DHT11 (or DHT22 if available)
    delay(2000);  // DHT sensors need 2 seconds to stabilize
    TempAndHumidity tempHumid = dhtSensor.getTempAndHumidity();
    if (dhtSensor.getStatus() == 0 && !isnan(tempHumid.temperature)) {
        temperatureSensorAvailable = true;
        Serial.print(F("✓ DHT11 temperature sensor initialized: "));
        Serial.print(tempHumid.temperature);
        Serial.println(F(" °C"));
    } else {
        temperatureSensorAvailable = false;
        Serial.println(F("✗ DHT11 temperature sensor NOT detected or unavailable"));
        Serial.println(F("  Check wiring: VCC->3.3V/5V, GND->GND, DATA->GPIO 27"));
        Serial.println(F("  Using default temperature (25°C) - readings will have reduced accuracy"));
    }
    
    // Initialize ultrasonic sensor
    Serial.print(F("Initializing "));
    Serial.print(SENSOR_NAME);
    Serial.println(F(" sensor..."));
    
    // Get ambient temperature for compensation (Mohammed et al. 2019, Tawalbeh et al. 2023)
    // Temperature sensor must be physically located near ultrasonic transducer for accurate air column density measurement
    float ambientTemp = getAmbientTemperature();
    ultrasonicSensor.setTemperature(ambientTemp);
    Serial.print(F("  Ambient temperature: "));
    Serial.print(ambientTemp);
    Serial.println(F(" °C (for speed-of-sound compensation)"));
    
    if (ultrasonicSensor.begin()) {
        Serial.println(F("✓ Sensor pins initialized"));
        
        // Take a test reading
        delay(500);
        float testDist = ultrasonicSensor.readDistanceCm();
        if (testDist > 0) {
            sensorInitialized = true;
            Serial.print(F("  Test reading: "));
            Serial.print(testDist);
            Serial.println(F(" cm (temperature compensated)"));
        } else {
            Serial.println(F("  Warning: Test reading failed"));
            Serial.println(F("  Sensor may not be connected properly"));
        }
    }
    
    if (!sensorInitialized) {
        Serial.println(F("✗ Sensor initialization failed!"));
        Serial.println(F("  Check wiring:"));
        Serial.println(F("  VCC  -> 5V (or 3.3V for JSN-SR04T)"));
        Serial.println(F("  GND  -> GND"));
        Serial.print(F("  TRIG -> GPIO "));
        Serial.println(ULTRASONIC_TRIG_PIN);
        Serial.print(F("  ECHO -> GPIO "));
        Serial.println(ULTRASONIC_ECHO_PIN);
        currentStatus = STATUS_SENSOR_ERROR;
    }
    
    // Initialize LMIC
    os_init();
    LMIC_reset();
    
    // Set frequency plan to AU915
    // Network configuration: LoRaWAN Class A for minimal power consumption (Ballerini et al. 2020)
    // Ballerini et al. (2020) demonstrated that LoRaWAN consumes order of magnitude less energy
    // than NB-IoT for small, sporadic payloads (4 bytes every 15 minutes) typical of flood monitoring
    LMIC_selectSubBand(1);
    // Using SF7 for optimal balance: lower Time on Air (ToA) = lower energy consumption (Casals et al. 2017)
    // Casals et al. (2017) showed that increasing SF from 7 to 12 increases energy by ~40x
    // More gateways allow nodes to use lower SFs, directly extending battery life (Casals et al. 2017)
    // Mikhaylov et al. (2018) analyzed network capacity: supports up to 1000 devices per gateway at SF7
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
    
    currentStatus = STATUS_JOINING;
    updateDisplay();
    
    // Start join process
    do_send(&sendjob);
    
    Serial.println(F("\nSetup complete. Joining LoRaWAN network..."));
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    os_runloop_once();
    
    // Update display every second (only when not sleeping)
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 1000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }
    
    // If we've joined and transmitted, enter deep sleep
    // This will only execute if deep sleep is enabled and we've completed a transmission cycle
    if (DEEP_SLEEP_ENABLED && joinedNetwork && !(LMIC.opmode & OP_TXRXPEND)) {
        // Small delay to ensure transmission is complete
        delay(2000);  // Give time for display update
        Serial.println(F("\nEntering deep sleep..."));
        Serial.flush();
        enterDeepSleep(TX_INTERVAL_SECONDS);
    }
}

// ============================================================================
// Read Sensor Data with Statistical Filtering (Kabi et al. 2023)
// Temperature Compensation (Mohammed et al. 2019, Tawalbeh et al. 2023)
// ============================================================================
bool readSensorData() {
    Serial.println(F("\nReading ultrasonic sensor..."));
    
    // Update temperature for compensation (Mohammed et al. 2019, Tawalbeh et al. 2023)
    // Tawalbeh et al. (2023) demonstrated that diurnal temperature swings of 20°C can introduce
    // measurement errors of several centimeters without compensation
    float ambientTemp = getAmbientTemperature();
    ultrasonicSensor.setTemperature(ambientTemp);
    
    // Take multiple readings for statistical filtering
    // Based on Kabi et al. (2023): raw sensor data in river environments is prone 
    // to noise from turbulence and debris, necessitating filtering
    float readings[NUM_READINGS_AVG];
    uint8_t validReadings = 0;
    
    // Collect all valid readings
    for (int i = 0; i < NUM_READINGS_AVG; i++) {
        float dist = ultrasonicSensor.readDistanceCm();
        
        if (dist > 0 && dist <= ultrasonicSensor.getMaxDistance()) {
            readings[validReadings] = dist;
            validReadings++;
            Serial.print(F("  Reading "));
            Serial.print(i + 1);
            Serial.print(F(": "));
            Serial.print(dist);
            Serial.println(F(" cm"));
        } else {
            Serial.print(F("  Reading "));
            Serial.print(i + 1);
            Serial.println(F(": INVALID"));
        }
        
        delay(READING_DELAY_MS);
    }
    
    if (validReadings == 0) {
        Serial.println(F("No valid sensor readings!"));
        currentStatus = STATUS_SENSOR_ERROR;
        return false;
    }
    
    // Calculate median (more robust than mean for noisy data)
    float median = calculateMedian(readings, validReadings);
    Serial.print(F("\n  Median: "));
    Serial.print(median);
    Serial.println(F(" cm"));
    
    // Filter outliers based on deviation from median (Kabi et al. filtering approach)
    float filteredReadings[NUM_READINGS_AVG];
    uint8_t filteredCount = validReadings;
    float avgDistance = filterOutliers(readings, filteredCount, median, OUTLIER_THRESHOLD_PCT);
    
    // Use median if filtering removed too many samples (fallback)
    if (filteredCount < validReadings / 2) {
        Serial.println(F("  Warning: Too many outliers, using median instead"));
        avgDistance = median;
        filteredCount = validReadings;
    } else {
        Serial.print(F("  Outliers removed: "));
        Serial.print(validReadings - filteredCount);
        Serial.print(F("/"));
        Serial.println(validReadings);
    }
    
    lastDistance = avgDistance;
    
    // Fill payload structure
    sensorData.sensorType = SENSOR_TYPE_ID;
    sensorData.distanceMm = (uint16_t)(avgDistance * 10);  // Convert cm to mm
    sensorData.signalStrength = 0;  // Not available for ultrasonic
    sensorData.temperature = (int8_t)round(ambientTemp);
    sensorData.batteryPercent = getBatteryPercent();
    sensorData.readingCount = filteredCount;
    
    Serial.println(F("\n--- Sensor Reading Summary ---"));
    Serial.print(F("Filtered Distance: "));
    Serial.print(avgDistance);
    Serial.println(F(" cm"));
    Serial.print(F("Temperature: "));
    Serial.print(ambientTemp);
    Serial.println(F(" °C"));
    Serial.print(F("Valid readings: "));
    Serial.print(filteredCount);
    Serial.print(F("/"));
    Serial.println(validReadings);
    Serial.println(F("------------------------------"));
    
    return true;
}

// ============================================================================
// Get Battery Percentage
// ============================================================================
uint8_t getBatteryPercent() {
    // Heltec V2 has battery voltage divider on GPIO 37
    // Voltage divider ratio: typically 2:1 (battery voltage / 2)
    // ADC reading: 12-bit (0-4095) with 3.3V reference
    // Battery range: 3.0V (empty) to 4.2V (full) for LiPo
    
    #ifdef BATTERY_ADC_PIN
        // Read battery voltage via ADC
        int adcValue = analogRead(BATTERY_ADC_PIN);
        
        // Convert ADC reading to voltage
        // Voltage divider divides by 2, so multiply by 2
        float voltage = (adcValue / 4095.0f) * 3.3f * 2.0f;
        
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
    
    // Turn off OLED display to save power
    u8g2.setPowerSave(1);
    
    // Configure RTC timer wakeup
    // Deep sleep energy: ~10-150 µA (Casals et al. 2017, Bouguera et al. 2018)
    // Energy model: E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx (Bouguera et al. 2018)
    // Casals et al. (2017) validated that ESP32 deep sleep enables multi-year battery life
    esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);  // Convert to microseconds
    
    // Enter deep sleep
    // Note: This function never returns - the ESP32 will restart after wakeup
    // Power consumption drops from 300-450mA (active) to ~10µA (deep sleep)
    // This enables 6-12 month battery life with 2000mAh battery (validated by energy models)
    // Duty cycle <1% (5-10s active per 15min cycle) following validated power management strategies
    esp_deep_sleep_start();
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
    
    currentStatus = STATUS_SENDING;
    updateDisplay();
    
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
            currentStatus = STATUS_JOINING;
            updateDisplay();
            break;
            
        case EV_JOINED:
            Serial.println(F("EV_JOINED - Successfully joined!"));
            Serial.print(F("DevAddr: "));
            Serial.println(LMIC.devaddr, HEX);
            
            LMIC_setLinkCheckMode(0);
            currentStatus = STATUS_CONNECTED;
            joinedNetwork = true;
            updateDisplay();
            
            // Schedule first data transmission
            Serial.println(F("Scheduling first sensor transmission in 2 seconds..."));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
            
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED - Join failed!"));
            currentStatus = STATUS_JOINING;
            updateDisplay();
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
            break;
            
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE - Join request sent"));
            break;
            
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART - Transmitting..."));
            currentStatus = STATUS_SENDING;
            updateDisplay();
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
            
            currentStatus = STATUS_CONNECTED;
            updateDisplay();
            
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
            currentStatus = STATUS_JOINING;
            joinedNetwork = false;
            updateDisplay();
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
// Display Functions
// ============================================================================
void updateDisplay() {
    u8g2.clearBuffer();
    
    // Line 1: Title and sensor type
    u8g2.setCursor(0, 12);
    u8g2.print("Heltec - ");
    u8g2.print(SENSOR_NAME);
    
    // Line 2: Status
    u8g2.setCursor(0, 26);
    switch(currentStatus) {
        case STATUS_INIT:
            u8g2.print("Initializing...");
            break;
        case STATUS_JOINING:
            u8g2.print("Joining LoRa...");
            break;
        case STATUS_CONNECTED:
            u8g2.print("Connected");
            break;
        case STATUS_SENDING:
            u8g2.print("Sending...");
            break;
        case STATUS_SENSOR_ERROR:
            u8g2.print("SENSOR ERROR!");
            break;
    }
    
    // Line 3: Distance or packet count
    u8g2.setCursor(0, 40);
    if (lastDistance > 0) {
        char distStr[24];
        snprintf(distStr, sizeof(distStr), "Dist: %.1f cm", lastDistance);
        u8g2.print(distStr);
    } else if (packetCount > 0) {
        char pktStr[20];
        snprintf(pktStr, sizeof(pktStr), "Pkts: %lu", packetCount);
        u8g2.print(pktStr);
    } else {
        u8g2.print("AU915 - Waiting");
    }
    
    // Line 4: Battery or RSSI
    u8g2.setCursor(0, 54);
    uint8_t battery = getBatteryPercent();
    if (battery < 100) {
        char batStr[20];
        snprintf(batStr, sizeof(batStr), "Bat: %d%%", battery);
        u8g2.print(batStr);
    } else if (lastRSSI != 0) {
        char rssiStr[24];
        snprintf(rssiStr, sizeof(rssiStr), "RSSI:%d SNR:%d", lastRSSI, lastSNR);
        u8g2.print(rssiStr);
    } else {
        u8g2.print("Pkts: ");
        u8g2.print(packetCount);
    }
    
    u8g2.sendBuffer();
}

// ============================================================================
// Get Ambient Temperature (for speed-of-sound compensation)
// Based on Mohammed et al. (2019) and Tawalbeh et al. (2023): Temperature compensation is critical
// Mohammed et al. (2019) showed that integrated temperature sensor (e.g., DHT11/DHT22) physically
// located near the ultrasonic transducer dramatically reduces RMSE, achieving sub-centimeter accuracy
// Tawalbeh et al. (2023) demonstrated that diurnal temperature swings of 20°C can introduce
// measurement errors of several centimeters without compensation
// ============================================================================
float getAmbientTemperature() {
    if (temperatureSensorAvailable) {
        // Read temperature from DHT11 sensor
        // DHTesp library requires getTempAndHumidity() call (already initialized in setup)
        TempAndHumidity tempHumid = dhtSensor.getTempAndHumidity();
        
        // Check if reading is valid
        if (dhtSensor.getStatus() == 0 && !isnan(tempHumid.temperature)) {
            float temp = tempHumid.temperature;
            // Validate temperature range (reasonable for outdoor deployment: -10°C to 50°C)
            if (temp >= -10.0f && temp <= 50.0f) {
                return temp;
            } else {
                Serial.print(F("Warning: DHT11 reading out of range: "));
                Serial.print(temp);
                Serial.println(F(" °C, using default"));
                return DEFAULT_TEMPERATURE;
            }
        } else {
            // Sensor reading failed, use default
            Serial.println(F("Warning: DHT11 reading failed, using default temperature"));
            return DEFAULT_TEMPERATURE;
        }
    } else {
        // DHT11 sensor not available or not initialized, use default
        // This should not happen in normal operation after setup, but provides fallback
        return DEFAULT_TEMPERATURE;
    }
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
