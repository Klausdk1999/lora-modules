/**
 * Heltec WiFi LoRa 32 V2 - LoRaWAN Communication Test
 * 
 * Código simplificado para testar comunicação LoRaWAN com gateway
 * Configurado para AU915 (Australia/Brazil)
 * 
 * INSTRUÇÕES:
 * 1. Configure o gateway para AU915
 * 2. Registre este dispositivo no seu servidor LoRaWAN (TTN/ChirpStack)
 * 3. Substitua APPEUI, DEVEUI e APPKEY abaixo com os valores do servidor
 * 4. Faça upload e monitore o Serial Monitor
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8g2lib.h>

// ============================================================================
// LoRaWAN Configuration - Credenciais do TTN
// ============================================================================
// AppEUI (JoinEUI): 0000000000000000 (padrão quando não especificado)
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// DevEUI: 75597D3513647C45 - LSB first (use um valor único para cada dispositivo)
static const u1_t PROGMEM DEVEUI[8] = { 
  0x45, 0x7C, 0x64, 0x13, 0x35, 0x7D, 0x59, 0x75
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// AppKey: BDFC920F78D3AF2053D1FB85D6A3347B - MSB first
static const u1_t PROGMEM APPKEY[16] = { 
  0xBD, 0xFC, 0x92, 0x0F, 0x78, 0xD3, 0xAF, 0x20,
  0x53, 0xD1, 0xFB, 0x85, 0xD6, 0xA3, 0x34, 0x7B
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (Heltec WiFi LoRa 32 V2)
// ============================================================================
const lmic_pinmap lmic_pins = {
    .nss = 18,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 14,
    .dio = {26, 33, 32},  // DIO0, DIO1, DIO2
};

// ============================================================================
// OLED Display (Heltec WiFi LoRa 32 V2)
// ============================================================================
// Heltec V2 uses SSD1306 OLED: SDA=4, SCL=15, RST=16
// Using U8G2 for better control over display
#include <U8g2lib.h>
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, /* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// ============================================================================
// Timing Configuration
// ============================================================================
const unsigned long TX_INTERVAL = 30000;  // Transmitir a cada 30 segundos
static osjob_t sendjob;
uint8_t packetCounter = 0;
unsigned long lastTxTime = 0;
bool txScheduled = false;

// ============================================================================
// Display Status Variables
// ============================================================================
String statusText = "Initializing...";
bool isJoined = false;
int16_t lastRSSI = 0;
int8_t lastSNR = 0;
unsigned long lastUpdate = 0;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent (ev_t ev);
void do_send(osjob_t* j);
void updateDisplay();
void displayStatus(String line1, String line2 = "", String line3 = "", String line4 = "");
String truncateString(String str, int maxLen);

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("====================================="));
  Serial.println(F("Heltec LoRa32 V2 - LoRaWAN Test"));
  Serial.println(F("Frequency: AU915"));
  Serial.println(F("====================================="));
  
  // Initialize OLED Display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);  // Nice readable font
  u8g2.clearBuffer();
  displayStatus("Heltec LoRa32", "LoRaWAN Test", "Init...", "AU915");
  
  // Initialize LMIC
  os_init();
  LMIC_reset();
  
  // Configure for AU915
  // AU915 uses 8 channels (0-7) by default
  // You can enable more channels if needed
  LMIC_selectSubBand(1);  // Use sub-band 1 (channels 8-15) for AU915
  
  // Set data rate and transmit power
  // DR_SF7 = Data Rate 0 (SF7, BW125)
  // 14 = 14 dBm transmit power
  LMIC_setDrTxpow(DR_SF7, 14);
  
  // Disable adaptive data rate
  LMIC_setAdrMode(0);
  
  // Disable link check validation
  LMIC_setLinkCheckMode(0);
  
  // Trigger join (don't send data yet, wait for join)
  LMIC_startJoining();
  
  Serial.println(F("Setup complete. Attempting to join LoRaWAN network..."));
  Serial.println(F("Watch for 'EV_JOINED' message to confirm successful join."));
  displayStatus("Heltec LoRa32", "Joining...", "AU915", "");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  os_runloop_once();
  
  // Update display periodically
  if (millis() - lastUpdate > 1000) {
    updateDisplay();
    lastUpdate = millis();
  }
  
  // Force transmission check using millis() as backup
  if (isJoined && !txScheduled) {
    unsigned long currentTime = millis();
    if (currentTime - lastTxTime > 5000) {  // 5 seconds after join or last TX
      Serial.println(F("=== Loop: Triggering transmission from loop() ==="));
      do_send(&sendjob);
      lastTxTime = currentTime;
      txScheduled = true;
    }
  }
}

// ============================================================================
// Send Data
// ============================================================================
void do_send(osjob_t* j) {
  Serial.print(F("=== do_send called at os_getTime: "));
  Serial.println(os_getTime());
  
  // Check if joined
  if (!isJoined) {
    Serial.println(F("ERROR: Not joined yet, skipping transmission"));
    Serial.println(F("isJoined flag is false!"));
    Serial.print(F("LMIC.devaddr: "));
    Serial.println(LMIC.devaddr, HEX);
    Serial.println(F("Scheduling retry in 5 seconds..."));
    // Try again in 5 seconds
    os_setTimedCallback(j, os_getTime()+sec2osticks(5), do_send);
    return;
  }
  
  Serial.println(F("Device is joined, checking LMIC status..."));
  Serial.print(F("LMIC.opmode: 0x"));
  Serial.println(LMIC.opmode, HEX);
  
  // Check if LMIC is ready
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("WARNING: OP_TXRXPEND, LMIC busy, retrying in 2 seconds"));
    // Try again in 2 seconds
    os_setTimedCallback(j, os_getTime()+sec2osticks(2), do_send);
    return;
  }
  
  if (LMIC.opmode & OP_SHUTDOWN) {
    Serial.println(F("ERROR: LMIC is in SHUTDOWN mode!"));
    return;
  }
  
  // LMIC is ready, prepare and send message
  packetCounter++;
  const char* message = "hello world";
  uint8_t message_len = strlen(message);
  
  Serial.println(F("LMIC is ready, preparing packet..."));
  Serial.print(F("Sending: "));
  Serial.println(message);
  Serial.print(F("Packet counter: "));
  Serial.println(packetCounter);
  Serial.print(F("Message length: "));
  Serial.println(message_len);
  Serial.print(F("fPort: 1, confirmed: 0 (unconfirmed)"));
  Serial.println();
  
  // Send packet (unconfirmed, port 1)
  LMIC_setTxData2(1, (uint8_t*)message, message_len, 0);
  Serial.println(F("LMIC_setTxData2 called - Packet queued for transmission"));
  txScheduled = true;  // Mark as scheduled
  lastTxTime = millis();
  Serial.println(F("Waiting for EV_TXSTART and EV_TXCOMPLETE..."));
  // Note: Next transmission will be scheduled in EV_TXCOMPLETE
}

// ============================================================================
// LMIC Event Handler
// ============================================================================
void onEvent (ev_t ev) {
  Serial.print(os_getTime());
  Serial.print(F(": "));
  
  switch(ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING - Attempting to join network..."));
      statusText = "Joining...";
      displayStatus("Heltec LoRa32", "Joining...", "AU915", "");
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED - Successfully joined network!"));
      Serial.print(F("NetID: "));
      Serial.println(LMIC.netid, DEC);
      Serial.print(F("DevAddr: "));
      Serial.print(LMIC.devaddr, HEX);
      Serial.println();
      isJoined = true;
      statusText = "Joined!";
      displayStatus("Heltec LoRa32", "JOINED!", "AU915", "Ready");
      // Disable link check validation
      LMIC_setLinkCheckMode(0);
      // Reset TX timing
      lastTxTime = millis();
      txScheduled = false;
      Serial.println(F("Join complete - First transmission will trigger in 5 seconds from loop()"));
      // Also try callback approach
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(5), do_send);
      break;
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED - Check your credentials!"));
      Serial.println(F("Verify:"));
      Serial.println(F("  - DevEUI matches network server"));
      Serial.println(F("  - AppEUI matches network server"));
      Serial.println(F("  - AppKey matches network server"));
      Serial.println(F("  - Frequency plan is AU915"));
      isJoined = false;
      statusText = "Join Failed!";
      displayStatus("Heltec LoRa32", "JOIN FAILED", "Check creds", "AU915");
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      break;
    case EV_TXCOMPLETE:
      Serial.println(F("EV_TXCOMPLETE - Transmission completed!"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ACK from gateway"));
      if (LMIC.dataLen) {
        Serial.print(F("Received "));
        Serial.print(LMIC.dataLen);
        Serial.println(F(" bytes of downlink data"));
      }
      lastRSSI = LMIC.rssi;
      lastSNR = LMIC.snr;
      Serial.print(F("RSSI: "));
      Serial.print(LMIC.rssi);
      Serial.println(F(" dBm"));
      Serial.print(F("SNR: "));
      Serial.print(LMIC.snr);
      Serial.println(F(" dB"));
      statusText = "TX Complete";
      // Reset flags for next transmission
      lastTxTime = millis();
      txScheduled = false;
      // Schedule next transmission after TX_INTERVAL
      Serial.print(F("Scheduling next transmission in "));
      Serial.print(TX_INTERVAL/1000);
      Serial.println(F(" seconds..."));
      os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL/1000), do_send);
      Serial.println(F("Next transmission scheduled successfully"));
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART - Transmission started"));
      statusText = "Transmitting...";
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      break;
    case EV_RXCOMPLETE:
      Serial.println(F("EV_RXCOMPLETE"));
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      break;
    case EV_TXCANCELED:
      Serial.println(F("EV_TXCANCELED"));
      break;
    case EV_RXSTART:
      Serial.println(F("EV_RXSTART"));
      break;
    case EV_JOIN_TXCOMPLETE:
      Serial.println(F("EV_JOIN_TXCOMPLETE"));
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

// Helper function to truncate string to fit OLED
String truncateString(String str, int maxLen) {
  if (str.length() <= maxLen) {
    return str;
  }
  return str.substring(0, maxLen);
}

void displayStatus(String line1, String line2, String line3, String line4) {
  u8g2.clearBuffer();
  
  // Line 1 - Y position: 12 (top margin)
  if (line1.length() > 0) {
    u8g2.setCursor(0, 12);
    u8g2.print(truncateString(line1, 20));
  }
  
  // Line 2 - Y position: 26 (12 + 14 spacing)
  if (line2.length() > 0) {
    u8g2.setCursor(0, 26);
    u8g2.print(truncateString(line2, 20));
  }
  
  // Line 3 - Y position: 40 (26 + 14 spacing)
  if (line3.length() > 0) {
    u8g2.setCursor(0, 40);
    u8g2.print(truncateString(line3, 20));
  }
  
  // Line 4 - Y position: 54 (40 + 14 spacing)
  if (line4.length() > 0) {
    u8g2.setCursor(0, 54);
    u8g2.print(truncateString(line4, 20));
  }
  
  u8g2.sendBuffer();
}

void updateDisplay() {
  if (isJoined) {
    char rssiStr[16];
    char snrStr[16];
    char pktStr[16];
    
    // Format strings to fit OLED (max 16 chars per line)
    snprintf(rssiStr, sizeof(rssiStr), "RSSI:%d dBm", lastRSSI);
    snprintf(snrStr, sizeof(snrStr), "SNR:%d dB", lastSNR);
    snprintf(pktStr, sizeof(pktStr), "Pkts:%d", packetCounter);
    
    // Alternate between RSSI and SNR every 2 seconds
    static unsigned long lastSwitch = 0;
    bool showRSSI = (millis() - lastSwitch) % 4000 < 2000;
    
    if (showRSSI) {
      displayStatus("Heltec LoRa32", "JOINED", pktStr, rssiStr);
    } else {
      displayStatus("Heltec LoRa32", "JOINED", pktStr, snrStr);
    }
  } else {
    // Show join status - keep it simple and readable
    String shortStatus = truncateString(statusText, 16);
    displayStatus("Heltec LoRa32", shortStatus, "AU915", "");
  }
}


