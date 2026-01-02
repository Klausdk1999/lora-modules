/**
 * Heltec WiFi LoRa 32 V2 - Data Sending Test
 * 
 * Simple LoRaWAN test that continuously sends "hello world" every 10 seconds
 * Display shows: Waiting, Joining, Connected, Sending, etc.
 * 
 * Configure for AU915 (Australia/Brazil)
 * Based on MCCI LoRaWAN LMIC library ttn-otaa example
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8g2lib.h>

// ============================================================================
// LoRaWAN Configuration - Replace with your TTN credentials
// ============================================================================
// JoinEUI/AppEUI - From TTN device: 0000000000000000
// In TTN Console: 0000000000000000
// In code (LSB first): Already correct (all zeros)
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // 0000000000000000
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI - From TTN device: 70B3D57ED0074FC9
// In TTN Console: 70B3D57ED0074FC9
// In code (LSB first - reversed): C9 4F 07 D0 7E D5 B3 70
static const u1_t PROGMEM DEVEUI[8] = { 
  0xC9, 0x4F, 0x07, 0xD0, 0x7E, 0xD5, 0xB3, 0x70  // 70B3D57ED0074FC9 (reversed)
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey - From TTN device: 24BFC553D19785B23D3CA9C6BE15B123
// Keys are NOT reversed (MSB first)
static const u1_t PROGMEM APPKEY[16] = { 
  0x24, 0xBF, 0xC5, 0x53, 0xD1, 0x97, 0x85, 0xB2,
  0x3D, 0x3C, 0xA9, 0xC6, 0xBE, 0x15, 0xB1, 0x23  // 24BFC553D19785B23D3CA9C6BE15B123
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (Heltec WiFi LoRa 32 V2)
// ============================================================================
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 35, 34},  // DIO0, DIO1, DIO2 - Back to pins that were working
};

// ============================================================================
// OLED Display (Heltec WiFi LoRa 32 V2)
// ============================================================================
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// ============================================================================
// Timing Configuration
// ============================================================================
const unsigned TX_INTERVAL = 10;  // Send every 10 seconds
static osjob_t sendjob;
uint32_t packetCount = 0;

// ============================================================================
// Status Variables
// ============================================================================
enum Status {
  STATUS_INIT,
  STATUS_JOINING,
  STATUS_CONNECTED,
  STATUS_SENDING
};

Status currentStatus = STATUS_INIT;
int16_t lastRSSI = 0;
int8_t lastSNR = 0;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent(ev_t ev);
void do_send(osjob_t* j);
void updateDisplay();

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println(F("\n====================================="));
  Serial.println(F("Heltec LoRa32 V2 - Data Sending Test"));
  Serial.println(F("=====================================\n"));
  
  // Initialize OLED Display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  u8g2.clearBuffer();
  u8g2.setCursor(0, 12);
  u8g2.print("Initializing...");
  u8g2.sendBuffer();
  
  // Initialize LMIC
  os_init();
  
  // Reset LMIC
  LMIC_reset();
  
  // Set frequency plan to AU915
  LMIC_selectSubBand(1);  // AU915 uses subband 1 (channels 8-15)
  
  // Set data rate and TX power (AU915)
  LMIC_setDrTxpow(DR_SF7, 14);
  
  // Note: RX2 window for AU915 is automatically configured by LMIC
  // Default: 923.3 MHz, SF10BW500 (used for join-accept)
  
  // Disable adaptive data rate
  LMIC_setAdrMode(0);
  
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  
  currentStatus = STATUS_JOINING;
  updateDisplay();
  
  // Start job (will trigger join, then send when joined)
  do_send(&sendjob);
  
  Serial.println(F("Setup complete. Joining network..."));
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  os_runloop_once();
  
  // Heartbeat - verify loop is running (debug)
  static unsigned long lastHeartbeat = 0;
  static unsigned long heartbeatCount = 0;
  if (millis() - lastHeartbeat >= 5000) {  // Every 5 seconds
    heartbeatCount++;
    Serial.print(F("[HEARTBEAT] Loop running, count: "));
    Serial.print(heartbeatCount);
    Serial.print(F(", OPMODE: 0x"));
    Serial.println(LMIC.opmode, HEX);
    lastHeartbeat = millis();
  }
  
  // Update display every second
  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate >= 1000) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
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
    // Retry after a short delay
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
  
  currentStatus = STATUS_SENDING;
  updateDisplay();
  
  // Prepare upstream data packet at the next possible time
  // Port 1, confirmed=false (unconfirmed uplink)
  uint8_t result = LMIC_setTxData2(1, (uint8_t*)message, message_len, 0);
  Serial.print(F("LMIC_setTxData2 result: "));
  Serial.println(result);
  Serial.print(F("OPMODE after setTxData2: 0x"));
  Serial.println(LMIC.opmode, HEX);
  
  if (result) {
    Serial.println(F("ERROR: LMIC_setTxData2 failed!"));
  } else {
    Serial.println(F("Packet queued for transmission"));
  }
  // Next TX is scheduled after TX_COMPLETE event
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
      Serial.print(F("NetID: "));
      Serial.println(LMIC.netid, DEC);
      Serial.print(F("DevAddr: "));
      Serial.println(LMIC.devaddr, HEX);
      Serial.print(F("OPMODE: 0x"));
      Serial.println(LMIC.opmode, HEX);
      
      // Disable link check validation
      LMIC_setLinkCheckMode(0);
      
      currentStatus = STATUS_CONNECTED;
      updateDisplay();
      
      // Schedule first data transmission after a short delay (non-blocking)
      // This ensures LMIC is fully ready after join
      Serial.println(F("Scheduling first data transmission in 2 seconds..."));
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);
      break;
      
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED - Join failed!"));
      currentStatus = STATUS_JOINING;
      updateDisplay();
      // Retry join after delay
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(5), do_send);
      break;
      
    case EV_TXSTART:
      Serial.println(F(">>> EV_TXSTART - Transmission started!"));
      Serial.print(F("Frequency: "));
      Serial.print(LMIC.txChnl);
      Serial.println(F(" (channel)"));
      currentStatus = STATUS_SENDING;
      updateDisplay();
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
      
      currentStatus = STATUS_CONNECTED;
      updateDisplay();
      
      // Schedule next transmission
      Serial.print(F("Scheduling next transmission in "));
      Serial.print(TX_INTERVAL);
      Serial.println(F(" seconds"));
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
      break;
      
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F(">>> EV_JOIN_TXCOMPLETE - Join-request transmission complete"));
      Serial.println(F(">>> Waiting for RX window to open..."));
      break;
      
    case EV_RXSTART:
      Serial.println(F(">>> EV_RXSTART - RX window opened, waiting for data"));
      Serial.print(F("RX frequency: "));
      Serial.print(LMIC.freq);
      Serial.println(F(" Hz"));
      break;
      
    case EV_RXCOMPLETE:
      Serial.println(F(">>> EV_RXCOMPLETE - Received data"));
      if (LMIC.dataLen) {
        Serial.print(F("Data length: "));
        Serial.println(LMIC.dataLen);
      }
      break;
      
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD - Link dead"));
      currentStatus = STATUS_JOINING;
      updateDisplay();
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

// ============================================================================
// Display Functions
// ============================================================================
void updateDisplay() {
  u8g2.clearBuffer();
  
  // Line 1: Title
  u8g2.setCursor(0, 12);
  u8g2.print("Heltec LoRa32");
  
  // Line 2: Status
  u8g2.setCursor(0, 26);
  switch(currentStatus) {
    case STATUS_INIT:
      u8g2.print("Initializing...");
      break;
    case STATUS_JOINING:
      u8g2.print("Joining...");
      break;
    case STATUS_CONNECTED:
      u8g2.print("Connected");
      break;
    case STATUS_SENDING:
      u8g2.print("Sending...");
      break;
  }
  
  // Line 3: Packet count
  u8g2.setCursor(0, 40);
  if (currentStatus == STATUS_CONNECTED || currentStatus == STATUS_SENDING) {
    char info[20];
    snprintf(info, sizeof(info), "Pkts: %lu", packetCount);
    u8g2.print(info);
  } else {
    u8g2.print("AU915");
  }
  
  // Line 4: RSSI or status
  u8g2.setCursor(0, 54);
  if (lastRSSI != 0) {
    char rssiStr[20];
    snprintf(rssiStr, sizeof(rssiStr), "RSSI: %d dBm", lastRSSI);
    u8g2.print(rssiStr);
  } else {
    u8g2.print("Ready");
  }
  
  u8g2.sendBuffer();
}