/**
 * LilyGo T-Beam V1.2 (AXP2101) - Multi-Sensor River Level Monitoring
 *
 * This node uses multiple sensors for comprehensive distance measurement:
 * - TF-Nova LiDAR (UART) - 0.1-12m range, high accuracy
 * - AJ-SR04M Ultrasonic (GPIO) - 0.2-8m range, waterproof
 * - DHT11 Temperature/Humidity sensor
 *
 * Configuration:
 * - Frequency: AU915 (Australia/Brazil)
 * - Power: 5V external battery
 * - Transmission interval: 10 seconds (rapid testing mode)
 *
 * Wiring (T-Beam V1.2):
 *   TF-Nova (UART via Serial2):
 *     VCC -> 5V (external boost)
 *     GND -> GND
 *     TX  -> GPIO 13 (Serial2 RX)
 *     RX  -> GPIO 14 (Serial2 TX)
 *
 *   AJ-SR04M Ultrasonic:
 *     VCC  -> 5V (external)
 *     GND  -> GND
 *     TRIG -> GPIO 25
 *     ECHO -> GPIO 35 (input-only pin)
 *
 *   DHT11:
 *     VCC  -> 3.3V or 5V
 *     GND  -> GND
 *     DATA -> GPIO 15
 *
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <DHT.h>

// AXP2101 Power Management for T-Beam V1.2
#define XPOWERS_CHIP_AXP2101
#include <XPowersLib.h>

// Include sensor libraries
#include "TFNova/TFNova.h"
#include "AJSR04M/AJSR04M.h"

// ============================================================================
// Configuration Options
// ============================================================================
#define ENABLE_SERIAL_DEBUG     true
#define TX_INTERVAL_SECONDS     10      // 10 seconds between transmissions (rapid testing mode)
#define NUM_READINGS_AVG        5       // Number of readings to average
#define READING_DELAY_MS        100     // Delay between readings
#define DEEP_SLEEP_ENABLED      true

// ============================================================================
// Sensor Reliability Configuration
// ============================================================================
#define SENSOR_READ_RETRIES     3       // Number of retry attempts
#define SENSOR_RETRY_DELAY_MS   300     // Delay between retry attempts
#define SENSOR_ERROR_VALUE      -1      // Sentinel value for sensor error
#define MIN_VALID_READINGS      2       // Minimum valid readings required

// ============================================================================
// LoRaWAN Configuration - TTN Device
// ============================================================================
// JoinEUI/AppEUI
static const u1_t PROGMEM APPEUI[8] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui(u1_t* buf) { memcpy_P(buf, APPEUI, 8); }

// DevEUI - From TTN device: 70B3D57ED0074FD2
static const u1_t PROGMEM DEVEUI[8] = {
    0xD2, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70
};
void os_getDevEui(u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }

// AppKey - From TTN device: E85C567896B872DFBF87687648FCB5B3
static const u1_t PROGMEM APPKEY[16] = {
    0xE8, 0x5C, 0x56, 0x78, 0x96, 0xB8, 0x72, 0xDF,
    0xBF, 0x87, 0x68, 0x76, 0x48, 0xFC, 0xB5, 0xB3
};
void os_getDevKey(u1_t* buf) { memcpy_P(buf, APPKEY, 16); }

// ============================================================================
// Pin Definitions (LilyGo T-Beam V1.2)
// ============================================================================
// LoRa Module Pin Configuration
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,
    .dio = {26, 33, 32},
};

// I2C pins (for AXP2101 PMU)
#define I2C_SDA_PIN     21
#define I2C_SCL_PIN     22

// TF-Nova UART pins (Serial2)
#define TFNOVA_RX_PIN   13      // GPIO 13 receives from TF-Nova TX
#define TFNOVA_TX_PIN   14      // GPIO 14 sends to TF-Nova RX

// AJ-SR04M Ultrasonic sensor pins
#define ULTRASONIC_TRIG_PIN     25
#define ULTRASONIC_ECHO_PIN     35      // GPIO 35 is input-only (ADC1_CH7)

// DHT11 Temperature/Humidity sensor
#define DHT_PIN         15
#define DHT_TYPE        DHT11

// ============================================================================
// Sensor Instances
// ============================================================================
TFNova tfNova(Serial2, TFNOVA_RX_PIN, TFNOVA_TX_PIN);
AJSR04M ultrasonicSensor(ULTRASONIC_TRIG_PIN, ULTRASONIC_ECHO_PIN);
DHT dhtSensor(DHT_PIN, DHT_TYPE);

// AXP2101 Power Management Instance
XPowersAXP2101 pmu;
bool pmuInitialized = false;

// ============================================================================
// Sensor Status Flags
// ============================================================================
bool tfNovaInitialized = false;
bool ultrasonicInitialized = false;
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
#define RTC_DATA_MAGIC 0xCAFE1234
#define MAX_TX_FAILURES 3

struct __attribute__((packed)) LMICSessionData {
    uint32_t magic;
    u4_t netid;
    devaddr_t devaddr;
    u1_t nwkKey[16];
    u1_t artKey[16];
    u4_t seqnoUp;
    u4_t seqnoDn;
    u1_t dn2Dr;
    s1_t adrTxPow;
    u4_t rxDelay;
};

RTC_DATA_ATTR LMICSessionData rtcSession;
RTC_DATA_ATTR uint32_t rtcPacketCount = 0;
RTC_DATA_ATTR uint8_t rtcTxFailCount = 0;

// ============================================================================
// Sensor Data Structure (for LoRa transmission)
// ============================================================================
// Extended payload with multiple sensors
// Total size: 16 bytes
struct __attribute__((packed)) SensorPayload {
    uint8_t sensorFlags;        // Bit flags: bit0=TFNova, bit1=Ultrasonic, bit2=DHT, bit7=error
    int16_t tfNovaDistMm;       // TF-Nova distance in mm (-1 = error)
    int16_t tfNovaStrength;     // TF-Nova signal strength
    int16_t ultrasonicDistMm;   // Ultrasonic distance in mm (-1 = error)
    int8_t temperature;         // Temperature from DHT11 (Celsius)
    uint8_t humidity;           // Humidity from DHT11 (%)
    uint8_t batteryPercent;     // Battery level (0-100)
    uint16_t batteryMv;         // Battery voltage in mV
    uint8_t readingCount;       // Number of valid readings
};

SensorPayload sensorData;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent(ev_t ev);
void do_send(osjob_t* j);
bool readAllSensors();
bool initPMU();
uint8_t getBatteryPercent();
uint16_t getBatteryVoltage();
void enterDeepSleep(uint32_t sleepSeconds);
float calculateMedian(float readings[], uint8_t count);
int16_t readTFNovaAverage();
int16_t readUltrasonicAverage();
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
    Serial.println(F("T-Beam V1.2 - Multi-Sensor Node"));
    Serial.println(F("TF-Nova + AJ-SR04M + DHT11"));
    Serial.println(F("=============================================\n"));

    // Check wakeup reason
    esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
    if (wakeup_reason == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println(F("Woke up from deep sleep"));
    } else {
        Serial.println(F("Cold boot"));
    }

    // Initialize I2C for AXP2101
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);

    // Initialize AXP2101 PMU
    Serial.println(F("\nInitializing AXP2101 PMU..."));
    if (initPMU()) {
        Serial.println(F("  AXP2101 PMU initialized"));
        Serial.print(F("  Battery: "));
        Serial.print(getBatteryVoltage());
        Serial.print(F(" mV ("));
        Serial.print(getBatteryPercent());
        Serial.println(F("%)"));
    } else {
        Serial.println(F("  AXP2101 PMU NOT found!"));
    }

    // Initialize TF-Nova LiDAR
    Serial.println(F("\nInitializing TF-Nova LiDAR (UART)..."));
    Serial.print(F("  UART pins: RX=GPIO "));
    Serial.print(TFNOVA_RX_PIN);
    Serial.print(F(", TX=GPIO "));
    Serial.println(TFNOVA_TX_PIN);

    delay(500);  // Give sensor time to power up

    if (tfNova.begin()) {
        tfNovaInitialized = true;
        Serial.println(F("  TF-Nova initialized"));

        // Set lower frame rate for power saving
        tfNova.setFrameRate(10);
    } else {
        Serial.println(F("  TF-Nova NOT detected!"));
        Serial.println(F("  Check: VCC->5V, GND->GND, TX->GPIO13, RX->GPIO14"));
    }

    // Initialize AJ-SR04M Ultrasonic
    Serial.println(F("\nInitializing AJ-SR04M Ultrasonic..."));
    Serial.print(F("  Pins: TRIG=GPIO "));
    Serial.print(ULTRASONIC_TRIG_PIN);
    Serial.print(F(", ECHO=GPIO "));
    Serial.println(ULTRASONIC_ECHO_PIN);

    if (ultrasonicSensor.begin()) {
        ultrasonicInitialized = true;
        Serial.println(F("  AJ-SR04M initialized"));

        delay(100);
        float testDist = ultrasonicSensor.readDistanceCm();
        if (testDist > 0) {
            Serial.print(F("  Test reading: "));
            Serial.print(testDist);
            Serial.println(F(" cm"));
        } else {
            Serial.println(F("  Warning: Test reading failed (may need target)"));
        }
    } else {
        Serial.println(F("  AJ-SR04M initialization (pins configured)"));
        ultrasonicInitialized = true;
    }

    // Initialize DHT11
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

        // Set temperature for ultrasonic compensation
        ultrasonicSensor.setTemperature(testTemp);
    } else {
        Serial.println(F("  DHT11 NOT detected!"));
        Serial.println(F("  Using default temperature (25C)"));
        ultrasonicSensor.setTemperature(25.0f);
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
        packetCount = rtcPacketCount;

        // Schedule immediate transmission (no join needed)
        Serial.println(F("Scheduling transmission in 1 second..."));
        os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(1), do_send);
    } else {
        // Cold boot or no valid session - need to join
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

    if (DEEP_SLEEP_ENABLED && joinedNetwork && transmissionCompleted && !(LMIC.opmode & OP_TXRXPEND)) {
        delay(2000);
        Serial.print(F("\nEntering deep sleep for "));
        Serial.print(TX_INTERVAL_SECONDS);
        Serial.println(F(" seconds..."));
        Serial.flush();
        enterDeepSleep(TX_INTERVAL_SECONDS);
    }
}

// ============================================================================
// Read TF-Nova - Single Reading (returns -1 on error)
// ============================================================================
int16_t readTFNovaAverage() {
    if (!tfNovaInitialized) {
        Serial.println(F("TF-Nova not initialized, attempting retry..."));
        delay(500);
        tfNova.clearBuffer();

        if (tfNova.begin()) {
            tfNovaInitialized = true;
            tfNova.setFrameRate(10);
            Serial.println(F("  TF-Nova reinitialized successfully"));
        } else {
            Serial.println(F("  TF-Nova still not available"));
            return SENSOR_ERROR_VALUE;
        }
    }

    // Single reading mode for testing
    Serial.println(F("Reading TF-Nova (single reading)..."));

    SensorReading reading = tfNova.read();

    if (reading.valid && reading.distance_cm > 0) {
        Serial.print(F("  Distance: "));
        Serial.print(reading.distance_cm);
        Serial.print(F(" cm, sig: "));
        Serial.println(reading.signal_strength);

        return (int16_t)(reading.distance_cm * 10);  // Return in mm
    } else {
        Serial.println(F("  ERROR: Invalid reading"));
        return SENSOR_ERROR_VALUE;
    }

    /* AVERAGING CODE - Uncomment to restore averaging
    float readings[NUM_READINGS_AVG];
    uint8_t validCount = 0;

    Serial.println(F("Reading TF-Nova (5 samples)..."));

    for (int i = 0; i < NUM_READINGS_AVG; i++) {
        SensorReading reading = tfNova.read();

        Serial.print(F("  ["));
        Serial.print(i + 1);
        Serial.print(F("] "));

        if (reading.valid && reading.distance_cm > 0) {
            readings[validCount] = reading.distance_cm;
            validCount++;
            Serial.print(reading.distance_cm);
            Serial.print(F(" cm, sig: "));
            Serial.println(reading.signal_strength);
        } else {
            Serial.println(F("INVALID"));
        }

        delay(READING_DELAY_MS);
    }

    if (validCount < MIN_VALID_READINGS) {
        Serial.println(F("  ERROR: Insufficient valid TF-Nova readings"));
        return SENSOR_ERROR_VALUE;
    }

    float median = calculateMedian(readings, validCount);
    Serial.print(F("  TF-Nova median: "));
    Serial.print(median);
    Serial.println(F(" cm"));

    return (int16_t)(median * 10);  // Return in mm
    */
}

// ============================================================================
// Read Ultrasonic - Single Reading (returns -1 on error)
// ============================================================================
int16_t readUltrasonicAverage() {
    // Single reading mode for testing
    Serial.println(F("Reading AJ-SR04M Ultrasonic (single reading)..."));

    float dist = ultrasonicSensor.readDistanceCm();

    if (dist > 0 && dist >= ultrasonicSensor.getMinDistance() && dist <= ultrasonicSensor.getMaxDistance()) {
        Serial.print(F("  Distance: "));
        Serial.print(dist);
        Serial.println(F(" cm"));

        return (int16_t)(dist * 10);  // Return in mm
    } else if (dist < 0) {
        Serial.println(F("  ERROR: TIMEOUT"));
    } else if (dist == 0) {
        Serial.println(F("  ERROR: ZERO (too close?)"));
    } else {
        Serial.print(F("  ERROR: "));
        Serial.print(dist);
        Serial.println(F(" cm (out of range)"));
    }

    return SENSOR_ERROR_VALUE;

    /* AVERAGING CODE - Uncomment to restore averaging
    float readings[NUM_READINGS_AVG];
    uint8_t validCount = 0;

    Serial.println(F("Reading AJ-SR04M Ultrasonic (5 samples)..."));

    for (int i = 0; i < NUM_READINGS_AVG; i++) {
        float dist = ultrasonicSensor.readDistanceCm();

        Serial.print(F("  ["));
        Serial.print(i + 1);
        Serial.print(F("] "));

        if (dist > 0 && dist >= ultrasonicSensor.getMinDistance() && dist <= ultrasonicSensor.getMaxDistance()) {
            readings[validCount] = dist;
            validCount++;
            Serial.print(dist);
            Serial.println(F(" cm"));
        } else if (dist < 0) {
            Serial.println(F("TIMEOUT"));
        } else if (dist == 0) {
            Serial.println(F("ZERO (too close?)"));
        } else {
            Serial.print(dist);
            Serial.println(F(" cm (out of range)"));
        }

        delay(READING_DELAY_MS);
    }

    if (validCount < MIN_VALID_READINGS) {
        Serial.println(F("  ERROR: Insufficient valid ultrasonic readings"));
        return SENSOR_ERROR_VALUE;
    }

    float median = calculateMedian(readings, validCount);
    Serial.print(F("  Ultrasonic median: "));
    Serial.print(median);
    Serial.println(F(" cm"));

    return (int16_t)(median * 10);  // Return in mm
    */
}

// ============================================================================
// Read DHT Sensor (non-blocking approach with retry)
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
    temperature = SENSOR_ERROR_VALUE;
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

    // Read DHT11 first (for temperature compensation)
    float temperature, humidity;
    bool dhtSuccess = readDHTSensor(temperature, humidity);

    if (dhtSuccess) {
        ultrasonicSensor.setTemperature(temperature);
        sensorData.temperature = (int8_t)round(temperature);
        sensorData.humidity = (uint8_t)round(humidity);
    } else {
        sensorData.temperature = SENSOR_ERROR_VALUE;
        sensorData.humidity = 0;
    }

    // Read TF-Nova
    int16_t tfNovaDist = readTFNovaAverage();
    sensorData.tfNovaDistMm = tfNovaDist;
    sensorData.tfNovaStrength = (tfNovaDist > 0) ? tfNova.getSignalStrength() : 0;

    // Read Ultrasonic
    int16_t ultrasonicDist = readUltrasonicAverage();
    sensorData.ultrasonicDistMm = ultrasonicDist;

    // Set sensor flags
    sensorData.sensorFlags = 0;
    if (tfNovaDist > 0) sensorData.sensorFlags |= 0x01;
    if (ultrasonicDist > 0) sensorData.sensorFlags |= 0x02;
    if (dhtSuccess) sensorData.sensorFlags |= 0x04;

    bool anyDistanceSuccess = (tfNovaDist > 0 || ultrasonicDist > 0);
    if (!anyDistanceSuccess) {
        sensorData.sensorFlags |= 0x80;
    }

    // Battery readings
    sensorData.batteryPercent = getBatteryPercent();
    sensorData.batteryMv = getBatteryVoltage();

    // Count valid readings
    uint8_t validSensors = 0;
    if (tfNovaDist > 0) validSensors++;
    if (ultrasonicDist > 0) validSensors++;
    if (dhtSuccess) validSensors++;
    sensorData.readingCount = validSensors;

    // Print summary
    Serial.println(F("\n========================================"));
    Serial.println(F("       SENSOR READING SUMMARY"));
    Serial.println(F("========================================"));
    Serial.print(F("TF-Nova: "));
    if (tfNovaDist > 0) {
        Serial.print(tfNovaDist / 10.0f);
        Serial.println(F(" cm"));
    } else {
        Serial.println(F("ERROR"));
    }
    Serial.print(F("Ultrasonic: "));
    if (ultrasonicDist > 0) {
        Serial.print(ultrasonicDist / 10.0f);
        Serial.println(F(" cm"));
    } else {
        Serial.println(F("ERROR"));
    }
    Serial.print(F("Temperature: "));
    if (dhtSuccess) {
        Serial.print(temperature);
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
    Serial.print(F("% ("));
    Serial.print(sensorData.batteryMv);
    Serial.println(F(" mV)"));
    Serial.print(F("Sensor flags: 0x"));
    Serial.println(sensorData.sensorFlags, HEX);
    Serial.println(F("========================================"));

    return anyDistanceSuccess || dhtSuccess;
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
// AXP2101 PMU Initialization
// ============================================================================
bool initPMU() {
    if (!pmu.begin(Wire, AXP2101_SLAVE_ADDRESS, I2C_SDA_PIN, I2C_SCL_PIN)) {
        pmuInitialized = false;
        return false;
    }

    pmu.enableBattDetection();
    pmu.enableBattVoltageMeasure();
    pmu.enableSystemVoltageMeasure();
    pmu.enableVbusVoltageMeasure();

    pmu.setChargerConstantCurr(XPOWERS_AXP2101_CHG_CUR_500MA);
    pmu.setChargeTargetVoltage(XPOWERS_AXP2101_CHG_VOL_4V2);
    pmu.setChargerTerminationCurr(XPOWERS_AXP2101_CHG_ITERM_25MA);
    pmu.enableChargerTerminationLimit();
    pmu.setSysPowerDownVoltage(2600);

    pmuInitialized = true;
    return true;
}

// ============================================================================
// Get Battery Percentage
// ============================================================================
uint8_t getBatteryPercent() {
    if (!pmuInitialized) return 50;
    if (!pmu.isBatteryConnect()) return 0;

    int percent = pmu.getBatteryPercent();
    if (percent < 0) percent = 0;
    if (percent > 100) percent = 100;

    return (uint8_t)percent;
}

// ============================================================================
// Get Battery Voltage
// ============================================================================
uint16_t getBatteryVoltage() {
    if (!pmuInitialized) return 0;
    if (!pmu.isBatteryConnect()) return 0;

    return pmu.getBattVoltage();
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
    if (rtcSession.magic != RTC_DATA_MAGIC) {
        Serial.println(F("No valid session in RTC memory"));
        return false;
    }

    if (rtcSession.devaddr == 0) {
        Serial.println(F("Invalid DevAddr in saved session"));
        return false;
    }

    LMIC.netid = rtcSession.netid;
    LMIC.devaddr = rtcSession.devaddr;
    memcpy(LMIC.nwkKey, rtcSession.nwkKey, 16);
    memcpy(LMIC.artKey, rtcSession.artKey, 16);
    LMIC.seqnoUp = rtcSession.seqnoUp;
    LMIC.seqnoDn = rtcSession.seqnoDn;
    LMIC.dn2Dr = rtcSession.dn2Dr;
    LMIC.adrTxPow = rtcSession.adrTxPow;
    LMIC.rxDelay = rtcSession.rxDelay;

    LMIC.opmode &= ~OP_JOINING;

    return true;
}

// ============================================================================
// Clear Session (force rejoin on next boot)
// ============================================================================
void clearSession() {
    Serial.println(F("Clearing saved session - will rejoin on next boot"));
    rtcSession.magic = 0;
    rtcSession.devaddr = 0;
    rtcTxFailCount = 0;
    joinedNetwork = false;
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
        sensorData.sensorFlags = 0xFF;

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
            break;

        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            Serial.print(F("DevAddr: "));
            Serial.println(LMIC.devaddr, HEX);

            LMIC_setLinkCheckMode(0);
            joinedNetwork = true;
            transmissionCompleted = false;

            Serial.println(F("Scheduling first transmission in 2 seconds..."));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;

        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(5), do_send);
            break;

        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE"));
            break;

        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
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
