/**
 * LilyGo T-Beam AXP2101 v1.2 - Simple LoRaWAN Test with Random Data
 * 
 * Simple code that connects to LoRaWAN and sends random data packets
 * 
 * Configuration:
 * - Frequency: AU915 (Australia/Brazil)
 * - Device: lilygo-river-lora
 * - Transmission interval: 60 seconds
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ============================================================================
// Configuration Options
// ============================================================================
#define TX_INTERVAL_SECONDS     60      // Send data every 60 seconds
#define DEEP_SLEEP_ENABLED      false   // Disable deep sleep for continuous operation

// ============================================================================
// LoRaWAN Configuration - TTN Device: lilygo-river-lora
// ============================================================================
// JoinEUI: 0000000000000000
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI: 70B3D57ED0074FD2 (LSB first: D2 4F 07 D0 7E D5 B3 70)
static const u1_t PROGMEM DEVEUI[8] = { 
  0xD2, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey: E85C567896B872DFBF87687648FCB5B3 (MSB first)
static const u1_t PROGMEM APPKEY[16] = { 
  0xE8, 0x5C, 0x56, 0x78, 0x96, 0xB8, 0x72, 0xDF,
  0xBF, 0x87, 0x68, 0x76, 0x48, 0xFC, 0xB5, 0xB3
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (LilyGo T-Beam AXP2101 v1.2 / LoRa32)
// ============================================================================
const lmic_pinmap lmic_pins = {
    .nss = 18,              // Chip Select (CS)
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,              // Reset pin
    .dio = {26, 33, 32},    // DIO0, DIO1, DIO2
};

// ============================================================================
// Global Variables
// ============================================================================
static osjob_t sendjob;
uint32_t packetCount = 0;
int16_t lastRSSI = 0;
int8_t lastSNR = 0;
bool joinedNetwork = false;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent(ev_t ev);
void do_send(osjob_t* j);

// ============================================================================
// Setup
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(1000);
    
    Serial.println(F("\n\n============================================"));
    Serial.println(F("LilyGo T-Beam AXP2101 v1.2 - LoRaWAN Test"));
    Serial.println(F("============================================\n"));
    
    // Initialize LMIC
    os_init();
    LMIC_reset();
    
    // Configure for AU915 band
    LMIC_selectSubBand(1);
    
    // Set data rate and power
    LMIC_setDrTxpow(DR_SF7, 14);
    LMIC_setAdrMode(0);
    LMIC_setLinkCheckMode(0);
    
    // Start join process
    Serial.println(F("Starting LoRaWAN join process..."));
    do_send(&sendjob);
    
    Serial.println(F("Setup complete. Waiting for join..."));
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    os_runloop_once();
}

// ============================================================================
// Send Job - Generates and sends random data
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
        Serial.println(F("Network joined successfully!"));
    }
    
    // Generate random data (water level between 300-400 cm)
    packetCount++;
    float randomLevel = 300.0f + (random(0, 10000) / 100.0f);
    
    // Create JSON payload: {"level":347.70,"packet":3}
    char payload[64];
    snprintf(payload, sizeof(payload), "{\"level\":%.2f,\"packet\":%lu}", randomLevel, packetCount);
    
    Serial.print(F(">>> Sending packet #"));
    Serial.print(packetCount);
    Serial.print(F(": level="));
    Serial.print(randomLevel);
    Serial.print(F(" cm"));
    Serial.print(F(" (payload: "));
    Serial.print(payload);
    Serial.println(F(")"));
    
    // Send the payload
    uint8_t result = LMIC_setTxData2(1, (uint8_t*)payload, strlen(payload), 0);
    
    if (result) {
        Serial.println(F("ERROR: Failed to queue packet!"));
    } else {
        Serial.println(F("Packet queued successfully"));
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
            Serial.println(F("Scheduling first transmission in 2 seconds..."));
            os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(2), do_send);
            break;
            
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED - Join failed! Retrying in 5 seconds..."));
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
            
            // Schedule next transmission
            if (DEEP_SLEEP_ENABLED) {
                Serial.print(F("Next transmission in "));
                Serial.print(TX_INTERVAL_SECONDS);
                Serial.println(F(" seconds (deep sleep)"));
            } else {
                Serial.print(F("Next transmission in "));
                Serial.print(TX_INTERVAL_SECONDS);
                Serial.println(F(" seconds"));
                os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL_SECONDS), do_send);
            }
            break;
            
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD - Connection lost"));
            joinedNetwork = false;
            break;
            
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE - Connection alive"));
            break;
            
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned)ev);
            break;
    }
}
