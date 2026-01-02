/**
 * LilyGo LoRa32 - Data Sending Test
 * 
 * Simple LoRaWAN test that continuously sends "hello world" every 10 seconds
 * Configure for AU915 (Australia/Brazil)
 * Based on MCCI LoRaWAN LMIC library ttn-otaa example
 * 
 * Note: Pin definitions may vary depending on your specific LilyGo LoRa32 model.
 * Adjust the pin mappings in the lmic_pinmap structure if needed.
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ============================================================================
// LoRaWAN Configuration - TTN Device: lilygo-river-lora
// ============================================================================
// JoinEUI/AppEUI - From TTN device: 0000000000000000
// In TTN Console: 0000000000000000
// In code (LSB first): Already correct (all zeros)
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // 0000000000000000
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI - From TTN device: 70B3D57ED0074FD2
// In TTN Console: 70B3D57ED0074FD2
// In code (LSB first - reversed): D2 4F 07 D0 7E D5 B3 70
static const u1_t PROGMEM DEVEUI[8] = { 
  0xD2, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70  // 70B3D57ED0074FD2 (reversed)
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey - From TTN device: E85C567896B872DFBF87687648FCB5B3
// Keys are NOT reversed (MSB first)
static const u1_t PROGMEM APPKEY[16] = { 
  0xE8, 0x5C, 0x56, 0x78, 0x96, 0xB8, 0x72, 0xDF,
  0xBF, 0x87, 0x68, 0x76, 0x48, 0xFC, 0xB5, 0xB3  // E85C567896B872DFBF87687648FCB5B3
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (LilyGo LoRa32)
// ============================================================================
// Common pin configuration - ADJUST BASED ON YOUR SPECIFIC MODEL
// - LilyGo T-Beam: NSS=18, RST=23, DIO0=26, DIO1=33, DIO2=32
// - LilyGo LoRa32 V1.6: NSS=18, RST=14, DIO0=26, DIO1=33, DIO2=32
const lmic_pinmap lmic_pins = {
    .nss = 18,      // Chip select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,      // Reset pin (change to 14 for V1.6)
    .dio = {26, 33, 32},  // DIO0, DIO1, DIO2
};

// ============================================================================
// Timing Configuration
// ============================================================================
const unsigned TX_INTERVAL = 10;  // Send every 10 seconds
static osjob_t sendjob;
uint32_t packetCount = 0;

// ============================================================================
// Status Variables
// ============================================================================
int16_t lastRSSI = 0;
int8_t lastSNR = 0;

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
  Serial.println(F("\n====================================="));
  Serial.println(F("LilyGo LoRa32 - Data Sending Test"));
  Serial.println(F("=====================================\n"));
  
  // Initialize LMIC
  os_init();
  
  // Reset LMIC
  LMIC_reset();
  
  // Set frequency plan to AU915
  LMIC_selectSubBand(1);  // AU915 uses subband 1 (channels 8-15)
  
  // Set data rate and TX power (AU915)
  LMIC_setDrTxpow(DR_SF7, 14);
  
  // Disable adaptive data rate
  LMIC_setAdrMode(0);
  
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  
  // Start job (will trigger join, then send when joined)
  do_send(&sendjob);
  
  Serial.println(F("Setup complete. Joining network..."));
  Serial.println(F("Watch for 'EV_JOINED' message to confirm successful join."));
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  os_runloop_once();
}

// ============================================================================
// LMIC Send Job (ttn-otaa pattern)
// ============================================================================
void do_send(osjob_t* j) {
  Serial.print(F("do_send called - OPMODE: 0x"));
  Serial.print(LMIC.opmode, HEX);
  Serial.print(F(", DevAddr: 0x"));
  Serial.println(LMIC.devaddr, HEX);
  
  // Check if LMIC is busy
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending - will retry"));
    os_setTimedCallback(j, os_getTime()+sec2osticks(1), do_send);
    return;
  }
  
  // Check if we have joined
  if (LMIC.devaddr == 0) {
    Serial.println(F("Not joined yet, starting join..."));
    LMIC_startJoining();
    return;
  }
  
  // We are joined, send data
  packetCount++;
  const char* message = "hello world";
  uint8_t message_len = strlen(message);
  
  Serial.print(F(">>> Sending packet #"));
  Serial.print(packetCount);
  Serial.print(F(": "));
  Serial.println(message);
  Serial.print(F("Message length: "));
  Serial.println(message_len);
  
  uint8_t result = LMIC_setTxData2(1, (uint8_t*)message, message_len, 0);
  Serial.print(F("LMIC_setTxData2 result: "));
  Serial.println(result);
  
  if (result) {
    Serial.println(F("ERROR: LMIC_setTxData2 failed!"));
  } else {
    Serial.println(F("Packet queued for transmission"));
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
      Serial.print(F("NetID: "));
      Serial.println(LMIC.netid, DEC);
      Serial.print(F("DevAddr: "));
      Serial.println(LMIC.devaddr, HEX);
      Serial.print(F("OPMODE: 0x"));
      Serial.println(LMIC.opmode, HEX);
      
      // Disable link check validation
      LMIC_setLinkCheckMode(0);
      
      // Schedule first data transmission after a short delay
      Serial.println(F("Scheduling first data transmission in 2 seconds..."));
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);
      break;
      
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED - Join failed!"));
      // Retry join after delay
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(5), do_send);
      break;
      
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F(">>> EV_JOIN_TXCOMPLETE - Join-request transmission complete"));
      Serial.println(F(">>> Waiting for RX window to open..."));
      break;
      
    case EV_TXSTART:
      Serial.println(F(">>> EV_TXSTART - Transmission started!"));
      Serial.print(F("Frequency: "));
      Serial.print(LMIC.txChnl);
      Serial.println(F(" (channel)"));
      break;
      
    case EV_RXSTART:
      Serial.println(F(">>> EV_RXSTART - RX window opened, waiting for data"));
      Serial.print(F("RX frequency: "));
      Serial.print(LMIC.freq);
      Serial.println(F(" Hz"));
      break;
      
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE - Transmission complete"));
      Serial.print(F("txrxFlags: 0x"));
      Serial.println(LMIC.txrxFlags, HEX);
      
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F(">>> Received ACK from network"));
      }
      if (LMIC.txrxFlags & TXRX_NACK) {
        Serial.println(F(">>> Received NACK from network"));
      }
      if (LMIC.dataLen) {
        Serial.print(F(">>> Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes downlink"));
      }
      
      lastRSSI = LMIC.rssi;
      lastSNR = LMIC.snr;
      Serial.print(F("RSSI: "));
      Serial.print(LMIC.rssi);
      Serial.print(F(" dBm, SNR: "));
      Serial.print(LMIC.snr);
      Serial.println(F(" dB"));
      
      // Schedule next transmission
      Serial.print(F("Scheduling next transmission in "));
      Serial.print(TX_INTERVAL);
      Serial.println(F(" seconds"));
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
      
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE - Received data"));
      break;
      
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD - Link dead"));
      break;
      
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE - Link alive"));
      break;
      
    default:
      Serial.print(F("Unknown event: "));
      Serial.println((unsigned) ev);
      break;
  }
}