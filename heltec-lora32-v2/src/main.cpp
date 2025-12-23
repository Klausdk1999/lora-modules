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
// LoRaWAN Configuration - SUBSTITUA COM SEUS VALORES DO SERVIDOR
// ============================================================================
// IMPORTANTE: Use os valores do seu servidor LoRaWAN (TTN ou ChirpStack)
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Substitua com AppEUI do servidor
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8] = { 
  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77  // Substitua com DevEUI único deste nó
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 
  0x75, 0x59, 0x7d, 0x35, 0x13, 0x64, 0x7c, 0x45,  // AppKey: 75597d3513647c454cbc8fea8ea9e55a
  0x4c, 0xbc, 0x8f, 0xea, 0x8e, 0xa9, 0xe5, 0x5a
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
  
  // Start job (will trigger join)
  do_send(&sendjob);
  
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
}

// ============================================================================
// Send Data
// ============================================================================
void do_send(osjob_t* j) {
  // Check if LMIC is ready
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
  } else {
    // Prepare test message
    packetCounter++;
    char message[50];
    snprintf(message, sizeof(message), "Heltec_Test_%d", packetCounter);
    
    Serial.print(F("Sending: "));
    Serial.println(message);
    Serial.print(F("Packet counter: "));
    Serial.println(packetCounter);
    
    // Send packet
    LMIC_setTxData2(1, (uint8_t*)message, strlen(message), 0);
    Serial.println(F("Packet queued for transmission"));
  }
  
  // Schedule next transmission
  os_setTimedCallback(j, os_getTime()+sec2osticks(TX_INTERVAL/1000), do_send);
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
      Serial.println(F("EV_TXCOMPLETE"));
      if (LMIC.txrxFlags & TXRX_ACK)
        Serial.println(F("Received ACK"));
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


