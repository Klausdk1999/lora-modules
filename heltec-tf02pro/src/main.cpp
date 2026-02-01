/**
 * Heltec WiFi LoRa 32 V2 - TF02-Pro LiDAR + DHT11 Monitoring Node
 *
 * This node uses:
 * - TF02-Pro LiDAR (UART) - 0.1-22m range, high accuracy
 * - DHT11 Temperature/Humidity sensor
 *
 * Configuration:
 * - Frequency: AU915 (Australia/Brazil)
 * - Power: 5V external battery
 * - Transmission interval: 10 seconds (rapid testing mode)
 * - Display: OLED shows status, distance, and signal quality
 *
 * Wiring (Heltec WiFi LoRa 32 V2):
 *   TF02-Pro (UART via Serial2):
 *     VCC -> 5V (external battery)
 *     GND -> GND
 *     TX  -> GPIO 13 (Serial2 RX)
 *     RX  -> GPIO 17 (Serial2 TX)
 *
 *   DHT11:
 *     VCC  -> 3.3V or 5V
 *     GND  -> GND
 *     DATA -> GPIO 27
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <HardwareSerial.h>
#include <U8g2lib.h>
#include <DHT.h>

// Include TF02-Pro sensor library
#include "TF02Pro/TF02Pro.h"

// ============================================================================
// Configuration Options
// ============================================================================
#define ENABLE_SERIAL_DEBUG     true
#define TX_INTERVAL_SECONDS     10      // 10 seconds between transmissions (rapid testing mode)
#define NUM_READINGS_AVG        5       // Number of readings to average
#define READING_DELAY_MS        150     // Delay between readings (TF02-Pro at 10Hz = 100ms min)
#define DEEP_SLEEP_ENABLED      true

// ============================================================================
// Sensor Reliability Configuration
// ============================================================================
#define SENSOR_READ_RETRIES     3       // Number of retry attempts
#define SENSOR_RETRY_DELAY_MS   500     // Delay between retry attempts
#define SENSOR_ERROR_VALUE      -1      // Sentinel value for sensor error
#define MIN_VALID_READINGS      2       // Minimum valid readings required

// ============================================================================
// LoRaWAN Configuration - TTN Device: heltec-river-lora
// ============================================================================
static const u1_t PROGMEM APPEUI[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// DevEUI - From TTN device: 70B3D57ED0074FC9
static const u1_t PROGMEM DEVEUI[8] = {
    0xC9, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70
};
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// AppKey - From TTN device: 24BFC553D19785B23D3CA9C6BE15B123
static const u1_t PROGMEM APPKEY[16] = {
    0x24, 0xBF, 0xC5, 0x53, 0xD1, 0x97, 0x85, 0xB2,
    0x3D, 0x3C, 0xA9, 0xC6, 0xBE, 0x15, 0xB1, 0x23
};
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

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

// TF02-Pro UART pins (Serial2)
#define TF02_RX_PIN     13      // GPIO 13 receives from TF02-Pro TX
#define TF02_TX_PIN     17      // GPIO 17 sends to TF02-Pro RX

// DHT11 Temperature/Humidity sensor
// NOTE: GPIO 27 is SPI MOSI on Heltec V2 - DO NOT USE!
// Using GPIO 25 instead (safe, no conflicts)
#define DHT_PIN         25
#define DHT_TYPE        DHT11

// Battery monitoring (Heltec V2 has voltage divider on GPIO 37)
#define BATTERY_ADC_PIN 37

// ============================================================================
// OLED Display (Heltec WiFi LoRa 32 V2)
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// ============================================================================
// Sensor Instances
// ============================================================================
TF02Pro lidarSensor(Serial2, TF02_RX_PIN, TF02_TX_PIN);
DHT dhtSensor(DHT_PIN, DHT_TYPE);

// ============================================================================
// Sensor Status Flags
// ============================================================================
bool lidarInitialized = false;
bool dhtInitialized = false;

// ============================================================================
// Timing and Status Variables
// ============================================================================
static osjob_t sendjob;
uint32_t packetCount = 0;
int16_t lastRSSI = 0;
int8_t lastSNR = 0;
bool joinedNetwork = false;
bool transmissionCompleted = false;

// ============================================================================
// Session Persistence (RTC Memory - survives deep sleep)
// ============================================================================
// Magic number to verify RTC data is valid
#define RTC_DATA_MAGIC 0xCAFE1234

// Structure to save LMIC session state
struct __attribute__((packed)) LMICSessionData {
    uint32_t magic;             // Magic number to verify data
    u4_t netid;                 // Network ID
    devaddr_t devaddr;          // Device address
    u1_t nwkKey[16];            // Network session key
    u1_t artKey[16];            // Application session key
    u4_t seqnoUp;               // Uplink frame counter
    u4_t seqnoDn;               // Downlink frame counter
    u1_t dn2Dr;                 // RX2 data rate
    s1_t adrTxPow;              // ADR TX power
    u4_t rxDelay;               // RX1 delay
};

// Store in RTC memory (survives deep sleep, lost on power cycle)
RTC_DATA_ATTR LMICSessionData rtcSession;
RTC_DATA_ATTR uint32_t rtcPacketCount = 0;
RTC_DATA_ATTR uint8_t rtcTxFailCount = 0;  // Track consecutive TX failures

// Max failures before forcing rejoin (handles gateway reset, session expiry)
#define MAX_TX_FAILURES 3

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
// Total size: 12 bytes
struct __attribute__((packed)) SensorPayload {
    uint8_t sensorType;         // 1 = TF02-Pro, 0xFF = error
    int16_t distanceMm;         // Distance in mm (-1 = error)
    int16_t signalStrength;     // Signal strength (flux)
    int8_t sensorTemp;          // TF02-Pro internal temperature
    int8_t ambientTemp;         // DHT11 temperature (Celsius)
    uint8_t humidity;           // DHT11 humidity (%)
    uint8_t batteryPercent;     // Battery level (0-100)
    uint8_t readingCount;       // Number of valid readings
};

SensorPayload sensorData;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent(ev_t ev);
void do_send(osjob_t* j);
bool readAllSensors();
uint8_t getBatteryPercent();
void updateDisplay();
void enterDeepSleep(uint32_t sleepSeconds);
float calculateMedian(float readings[], uint8_t count);
int16_t readLidarAverage(int16_t& strength, int16_t& sensorTemp);
bool readDHTSensor(float& temperature, float& humidity);
void saveSession();
bool restoreSession();
void clearSession();

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(500);

    Serial.println(F("\n============================================="));
    Serial.println(F("Heltec LoRa32 V2 - TF02-Pro + DHT11 Node"));
    Serial.println(F("=============================================\n"));

    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(F("Woke up from deep sleep"));
    } else {
        Serial.println(F("Cold boot"));
    }

    // Initialize OLED Display
    u8g2.begin();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.clearBuffer();
    u8g2.setCursor(0, 12);
    u8g2.print("Initializing...");
    u8g2.setCursor(0, 26);
    u8g2.print("TF02-Pro + DHT11");
    u8g2.sendBuffer();

    // Initialize DHT11 first (for temperature reference)
    Serial.println(F("\nInitializing DHT11 Temperature/Humidity..."));
    Serial.print(F("  Pin: DATA=GPIO "));
    Serial.println(DHT_PIN);

    dhtSensor.begin();
    delay(2000);  // DHT sensors need 2 seconds to stabilize

    float testTemp = dhtSensor.readTemperature();
    float testHum = dhtSensor.readHumidity();
    if (!isnan(testTemp) && !isnan(testHum)) {
        dhtInitialized = true;
        Serial.print(F("  DHT11 initialized: "));
        Serial.print(testTemp);
        Serial.print(F(" C, "));
        Serial.print(testHum);
        Serial.println(F(" %"));
    } else {
        Serial.println(F("  DHT11 NOT detected!"));
        Serial.println(F("  Check: VCC->3.3V/5V, GND->GND, DATA->GPIO25"));
    }

    // Initialize TF02-Pro LiDAR
    Serial.println(F("\nInitializing TF02-Pro LiDAR (UART)..."));
    Serial.print(F("  UART pins: RX=GPIO "));
    Serial.print(TF02_RX_PIN);
    Serial.print(F(" (TF02-Pro TX), TX=GPIO "));
    Serial.print(TF02_TX_PIN);
    Serial.println(F(" (TF02-Pro RX)"));
    Serial.println(F("  Baud rate: 115200"));

    delay(1000);  // Give sensor time to power up

    if (lidarSensor.begin()) {
        lidarInitialized = true;
        Serial.println(F("  TF02-Pro initialized"));

        // Set lower frame rate for power saving
        lidarSensor.setFrameRate(10);

        // Test reading
        SensorReading testReading = lidarSensor.read();
        if (testReading.valid) {
            Serial.print(F("  Test reading: "));
            Serial.print(testReading.distance_cm);
            Serial.print(F(" cm, Temp: "));
            Serial.print(testReading.temperature);
            Serial.println(F(" C"));
        } else {
            Serial.println(F("  Warning: Test reading failed"));
        }
    } else {
        Serial.println(F("  TF02-Pro NOT detected!"));
        Serial.println(F("  Check: VCC->5V, GND->GND, TX->GPIO13, RX->GPIO17"));
        currentStatus = STATUS_SENSOR_ERROR;
    }

    // Initialize LMIC
    os_init();
    LMIC_reset();
    LMIC_selectSubBand(1);
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);

    // Try to restore session from RTC memory (after deep sleep)
    bool sessionRestored = (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER && restoreSession());

    // Check if we've had too many TX failures - force rejoin
    if (sessionRestored && rtcTxFailCount >= MAX_TX_FAILURES) {
        Serial.println(F("\nToo many TX failures - forcing rejoin"));
        clearSession();
        sessionRestored = false;
    }

    if (sessionRestored) {
        Serial.println(F("\nSession restored from RTC memory!"));
        Serial.print(F("  DevAddr: "));
        Serial.println(LMIC.devaddr, HEX);
        Serial.print(F("  Frame counter: "));
        Serial.println(LMIC.seqnoUp);
        Serial.print(F("  TX fail count: "));
        Serial.println(rtcTxFailCount);

        joinedNetwork = true;
        currentStatus = STATUS_CONNECTED;
        packetCount = rtcPacketCount;
        updateDisplay();

        // Schedule immediate transmission (no join needed)
        Serial.println(F("Scheduling transmission in 1 second..."));
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
    } else {
        // Cold boot or no valid session - need to join
        currentStatus = STATUS_JOINING;
        updateDisplay();

        // Start join process
        do_send(&sendjob);
        Serial.println(F("\nSetup complete. Joining LoRaWAN network..."));
    }
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    os_runloop_once();

    // Yield to prevent watchdog issues
    yield();

    // Update display periodically
    static unsigned long lastDisplayUpdate = 0;
    if (millis() - lastDisplayUpdate >= 1000) {
        updateDisplay();
        lastDisplayUpdate = millis();
    }

    // Enter deep sleep after transmission
    // IMPORTANT: Wait for TX to complete and RX windows to close
    if (DEEP_SLEEP_ENABLED && joinedNetwork && transmissionCompleted && !(LMIC.opmode & OP_TXRXPEND)) {
        // Additional delay to ensure all LoRa operations are complete
        delay(2000);
        Serial.print(F("\nEntering deep sleep for "));
        Serial.print(TX_INTERVAL_SECONDS);
        Serial.println(F(" seconds..."));
        Serial.flush();
        enterDeepSleep(TX_INTERVAL_SECONDS);
    }
}

// ============================================================================
// Read LiDAR with Averaging (returns -1 on error)
// ============================================================================
int16_t readLidarAverage(int16_t& avgStrength, int16_t& avgSensorTemp) {
    if (!lidarInitialized) {
        Serial.println(F("TF02-Pro not initialized, attempting retry..."));
        delay(500);
        lidarSensor.clearBuffer();

        if (lidarSensor.begin()) {
            lidarInitialized = true;
            lidarSensor.setFrameRate(10);
            Serial.println(F("  TF02-Pro reinitialized successfully"));
        } else {
            Serial.println(F("  TF02-Pro still not available"));
            return SENSOR_ERROR_VALUE;
        }
    }

    float distances[NUM_READINGS_AVG];
    int16_t strengths[NUM_READINGS_AVG];
    int16_t temps[NUM_READINGS_AVG];
    uint8_t validCount = 0;

    Serial.println(F("Reading TF02-Pro (5 samples)..."));

    for (int i = 0; i < NUM_READINGS_AVG; i++) {
        SensorReading reading = lidarSensor.read();

        Serial.print(F("  ["));
        Serial.print(i + 1);
        Serial.print(F("] "));

        if (reading.valid && reading.distance_cm > 0) {
            distances[validCount] = reading.distance_cm;
            strengths[validCount] = reading.signal_strength;
            temps[validCount] = reading.temperature;
            validCount++;

            Serial.print(reading.distance_cm);
            Serial.print(F(" cm, sig: "));
            Serial.print(reading.signal_strength);
            Serial.print(F(", temp: "));
            Serial.print(reading.temperature);
            Serial.println(F(" C"));
        } else {
            Serial.println(F("INVALID"));
        }

        delay(READING_DELAY_MS);
        yield();  // Prevent watchdog timeout
    }

    if (validCount < MIN_VALID_READINGS) {
        Serial.println(F("  ERROR: Insufficient valid LiDAR readings"));
        return SENSOR_ERROR_VALUE;
    }

    // Calculate median for distance
    float median = calculateMedian(distances, validCount);

    // Calculate averages for strength and temperature
    int32_t sumStrength = 0;
    int32_t sumTemp = 0;
    for (uint8_t i = 0; i < validCount; i++) {
        sumStrength += strengths[i];
        sumTemp += temps[i];
    }
    avgStrength = sumStrength / validCount;
    avgSensorTemp = sumTemp / validCount;

    Serial.print(F("  TF02-Pro median: "));
    Serial.print(median);
    Serial.print(F(" cm, avg strength: "));
    Serial.print(avgStrength);
    Serial.print(F(", avg temp: "));
    Serial.print(avgSensorTemp);
    Serial.println(F(" C"));

    lastDistance = median;
    return (int16_t)(median * 10);  // Return in mm
}

// ============================================================================
// Read DHT Sensor (with retry)
// ============================================================================
bool readDHTSensor(float& temperature, float& humidity) {
    Serial.println(F("Reading DHT11..."));

    for (int attempt = 0; attempt < SENSOR_READ_RETRIES; attempt++) {
        if (attempt > 0) {
            Serial.print(F("  Retry "));
            Serial.print(attempt + 1);
            Serial.println(F("..."));
            delay(SENSOR_RETRY_DELAY_MS);
        }

        temperature = dhtSensor.readTemperature();
        humidity = dhtSensor.readHumidity();

        if (!isnan(temperature) && !isnan(humidity)) {
            if (temperature >= -40 && temperature <= 80 && humidity >= 0 && humidity <= 100) {
                Serial.print(F("  Temperature: "));
                Serial.print(temperature);
                Serial.print(F(" C, Humidity: "));
                Serial.print(humidity);
                Serial.println(F(" %"));
                dhtInitialized = true;
                return true;
            }
        }
    }

    Serial.println(F("  DHT11 reading failed!"));
    temperature = 0;
    humidity = 0;
    dhtInitialized = false;
    return false;
}

// ============================================================================
// Read All Sensors
// ============================================================================
bool readAllSensors() {
    Serial.println(F("\n========================================"));
    Serial.println(F("Reading all sensors..."));
    Serial.println(F("========================================"));

    // Read DHT11
    float ambientTemp, humidity;
    bool dhtSuccess = readDHTSensor(ambientTemp, humidity);

    // Read TF02-Pro
    int16_t lidarStrength = 0;
    int16_t sensorTemp = 0;
    int16_t distanceMm = readLidarAverage(lidarStrength, sensorTemp);

    // Fill payload
    sensorData.sensorType = (distanceMm > 0) ? 1 : 0xFF;
    sensorData.distanceMm = distanceMm;
    sensorData.signalStrength = lidarStrength;
    sensorData.sensorTemp = (int8_t)sensorTemp;

    if (dhtSuccess) {
        sensorData.ambientTemp = (int8_t)round(ambientTemp);
        sensorData.humidity = (uint8_t)round(humidity);
    } else {
        sensorData.ambientTemp = SENSOR_ERROR_VALUE;
        sensorData.humidity = 0;
    }

    sensorData.batteryPercent = getBatteryPercent();

    // Count valid readings
    uint8_t validCount = 0;
    if (distanceMm > 0) validCount++;
    if (dhtSuccess) validCount++;
    sensorData.readingCount = validCount;

    // Print summary
    Serial.println(F("\n========================================"));
    Serial.println(F("       SENSOR READING SUMMARY"));
    Serial.println(F("========================================"));
    Serial.print(F("TF02-Pro Distance: "));
    if (distanceMm > 0) {
        Serial.print(distanceMm / 10.0f);
        Serial.println(F(" cm"));
    } else {
        Serial.println(F("ERROR"));
    }
    Serial.print(F("TF02-Pro Signal: "));
    Serial.println(lidarStrength);
    Serial.print(F("TF02-Pro Temp: "));
    Serial.print(sensorTemp);
    Serial.println(F(" C"));
    Serial.print(F("Ambient Temp: "));
    if (dhtSuccess) {
        Serial.print(ambientTemp);
        Serial.println(F(" C"));
    } else {
        Serial.println(F("ERROR"));
    }
    Serial.print(F("Humidity: "));
    if (dhtSuccess) {
        Serial.print(humidity);
        Serial.println(F(" %"));
    } else {
        Serial.println(F("ERROR"));
    }
    Serial.print(F("Battery: "));
    Serial.print(sensorData.batteryPercent);
    Serial.println(F("%"));
    Serial.println(F("========================================"));

    currentStatus = (distanceMm > 0) ? STATUS_CONNECTED : STATUS_SENSOR_ERROR;
    return (distanceMm > 0 || dhtSuccess);
}

// ============================================================================
// Calculate Median
// ============================================================================
float calculateMedian(float readings[], uint8_t count) {
    if (count == 0) return 0;
    if (count == 1) return readings[0];

    float sorted[NUM_READINGS_AVG];
    for (uint8_t i = 0; i < count; i++) {
        sorted[i] = readings[i];
    }

    for (uint8_t i = 0; i < count - 1; i++) {
        for (uint8_t j = 0; j < count - i - 1; j++) {
            if (sorted[j] > sorted[j + 1]) {
                float temp = sorted[j];
                sorted[j] = sorted[j + 1];
                sorted[j + 1] = temp;
            }
        }
    }

    if (count % 2 == 0) {
        return (sorted[count / 2 - 1] + sorted[count / 2]) / 2.0f;
    } else {
        return sorted[count / 2];
    }
}

// ============================================================================
// Get Battery Percentage
// ============================================================================
uint8_t getBatteryPercent() {
    // Heltec V2 has battery voltage divider on GPIO 37
    int adcValue = analogRead(BATTERY_ADC_PIN);

    // Convert ADC reading to voltage (voltage divider divides by 2)
    float voltage = (adcValue / 4095.0f) * 3.3f * 2.0f;

    // Calculate percentage (LiPo: 3.0V empty, 4.2V full)
    if (voltage < 3.0f) return 0;
    if (voltage > 4.2f) return 100;

    uint8_t percent = (uint8_t)(((voltage - 3.0f) / 1.2f) * 100.0f);
    return percent;
}

// ============================================================================
// Enter Deep Sleep
// ============================================================================
void enterDeepSleep(uint32_t sleepSeconds) {
    Serial.print(F("Entering deep sleep for "));
    Serial.print(sleepSeconds);
    Serial.println(F(" seconds..."));

    // Save LoRa session to RTC memory before sleep
    saveSession();

    Serial.flush();

    // Turn off OLED display
    u8g2.setPowerSave(1);

    esp_sleep_enable_timer_wakeup(sleepSeconds * 1000000ULL);
    esp_deep_sleep_start();
}

// ============================================================================
// Save LMIC Session to RTC Memory
// ============================================================================
void saveSession() {
    if (LMIC.devaddr == 0) {
        Serial.println(F("  No session to save (not joined)"));
        return;
    }

    rtcSession.magic = RTC_DATA_MAGIC;
    rtcSession.netid = LMIC.netid;
    rtcSession.devaddr = LMIC.devaddr;
    memcpy(rtcSession.nwkKey, LMIC.nwkKey, 16);
    memcpy(rtcSession.artKey, LMIC.artKey, 16);
    rtcSession.seqnoUp = LMIC.seqnoUp;
    rtcSession.seqnoDn = LMIC.seqnoDn;
    rtcSession.dn2Dr = LMIC.dn2Dr;
    rtcSession.adrTxPow = LMIC.adrTxPow;
    rtcSession.rxDelay = LMIC.rxDelay;

    rtcPacketCount = packetCount;

    Serial.println(F("  Session saved to RTC memory"));
    Serial.print(F("    DevAddr: "));
    Serial.println(rtcSession.devaddr, HEX);
    Serial.print(F("    SeqnoUp: "));
    Serial.println(rtcSession.seqnoUp);
}

// ============================================================================
// Restore LMIC Session from RTC Memory
// ============================================================================
bool restoreSession() {
    // Check if we have valid saved session
    if (rtcSession.magic != RTC_DATA_MAGIC) {
        Serial.println(F("No valid session in RTC memory"));
        return false;
    }

    if (rtcSession.devaddr == 0) {
        Serial.println(F("Invalid DevAddr in saved session"));
        return false;
    }

    // Restore session to LMIC
    LMIC.netid = rtcSession.netid;
    LMIC.devaddr = rtcSession.devaddr;
    memcpy(LMIC.nwkKey, rtcSession.nwkKey, 16);
    memcpy(LMIC.artKey, rtcSession.artKey, 16);
    LMIC.seqnoUp = rtcSession.seqnoUp;
    LMIC.seqnoDn = rtcSession.seqnoDn;
    LMIC.dn2Dr = rtcSession.dn2Dr;
    LMIC.adrTxPow = rtcSession.adrTxPow;
    LMIC.rxDelay = rtcSession.rxDelay;

    // Mark as joined
    LMIC.opmode &= ~OP_JOINING;

    return true;
}

// ============================================================================
// Clear Session (force rejoin on next boot)
// ============================================================================
void clearSession() {
    Serial.println(F("Clearing saved session - will rejoin on next boot"));
    rtcSession.magic = 0;  // Invalidate saved session
    rtcSession.devaddr = 0;
    rtcTxFailCount = 0;
    joinedNetwork = false;
}

// ============================================================================
// Update Display
// ============================================================================
void updateDisplay() {
    u8g2.clearBuffer();

    // Line 1: Title
    u8g2.setCursor(0, 12);
    u8g2.print("Heltec TF02-Pro");

    // Line 2: Status
    u8g2.setCursor(0, 26);
    switch (currentStatus) {
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

    // Line 3: Distance
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
// LMIC Send Job
// ============================================================================
void do_send(osjob_t* j) {
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, waiting..."));
        os_setTimedCallback(j, os_getTime() + sec2osticks(1), do_send);
        return;
    }

    if (LMIC.devaddr == 0) {
        Serial.println(F("Not joined yet, starting join..."));
        LMIC_startJoining();
        return;
    }

    if (!joinedNetwork) {
        joinedNetwork = true;
    }

    currentStatus = STATUS_SENDING;
    updateDisplay();

    if (readAllSensors()) {
        packetCount++;

        Serial.print(F("\n>>> Sending packet #"));
        Serial.println(packetCount);

        uint8_t result = LMIC_setTxData2(1, (uint8_t*)&sensorData, sizeof(sensorData), 0);
        if (result) {
            Serial.println(F("ERROR: Failed to queue packet!"));
        } else {
            Serial.println(F("Packet queued for transmission"));
        }
    } else {
        Serial.println(F("\n>>> SENDING ERROR PACKET <<<"));
        sensorData.sensorType = 0xFF;
        sensorData.distanceMm = 0xFFFF;

        uint8_t result = LMIC_setTxData2(1, (uint8_t*)&sensorData, sizeof(sensorData), 0);
        if (result) {
            Serial.println(F("ERROR: Failed to queue error packet!"));
        } else {
            Serial.println(F("Error packet queued"));
        }
    }
}

// ============================================================================
// LMIC Event Handler
// ============================================================================
void onEvent(ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(F(": "));

    switch (ev) {
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            currentStatus = STATUS_JOINING;
            updateDisplay();
            break;

        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            Serial.print(F("DevAddr: "));
            Serial.println(LMIC.devaddr, HEX);

            LMIC_setLinkCheckMode(0);
            currentStatus = STATUS_CONNECTED;
            joinedNetwork = true;
            transmissionCompleted = false;
            updateDisplay();

            Serial.println(F("Scheduling first transmission in 2 seconds..."));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;

        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            currentStatus = STATUS_JOINING;
            updateDisplay();
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
            break;

        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE"));
            break;

        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            currentStatus = STATUS_SENDING;
            updateDisplay();
            break;

        case EV_RXSTART:
            Serial.println(F("EV_RXSTART"));
            break;

        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE"));
            transmissionCompleted = true;

            // TX succeeded - reset failure counter
            rtcTxFailCount = 0;

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

            if (DEEP_SLEEP_ENABLED) {
                Serial.print(F("Next transmission in "));
                Serial.print(TX_INTERVAL_SECONDS);
                Serial.println(F(" seconds (deep sleep)"));
            } else {
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL_SECONDS), do_send);
            }
            break;

        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            rtcTxFailCount++;
            Serial.print(F("TX failures: "));
            Serial.print(rtcTxFailCount);
            Serial.print(F("/"));
            Serial.println(MAX_TX_FAILURES);

            if (rtcTxFailCount >= MAX_TX_FAILURES) {
                Serial.println(F("Too many failures - forcing rejoin"));
                clearSession();
                LMIC_reset();
                LMIC_selectSubBand(1);
                LMIC_setDrTxpow(DR_SF7, 14);
                LMIC_startJoining();
            }
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
