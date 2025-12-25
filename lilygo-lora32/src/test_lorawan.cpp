/**
 * LilyGo LoRa32 - LoRaWAN Communication Test
 * 
 * Código simplificado para testar comunicação LoRaWAN com gateway
 * Configurado para AU915 (Australia/Brazil)
 * 
 * INSTRUÇÕES:
 * 1. Configure o gateway para AU915
 * 2. Registre este dispositivo no seu servidor LoRaWAN (TTN/ChirpStack)
 * 3. Substitua APPEUI, DEVEUI e APPKEY abaixo com os valores do servidor
 * 4. Ajuste os pinos se necessário (varia conforme modelo LilyGo)
 * 5. Faça upload e monitore o Serial Monitor
 */

#include <Arduino.h>
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

// ============================================================================
// LoRaWAN Configuration - SUBSTITUA COM SEUS VALORES DO SERVIDOR
// ============================================================================
// IMPORTANTE: Use os valores do seu servidor LoRaWAN (TTN ou ChirpStack)
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Substitua com AppEUI do servidor
};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8] = { 
  0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11  // Substitua com DevEUI único deste nó
};
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16] = { 
  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,  // Substitua com AppKey do servidor
  0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};
void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16);}

// ============================================================================
// Pin Definitions (LilyGo LoRa32)
// ============================================================================
// NOTA: Ajuste os pinos conforme seu modelo específico de LilyGo
// - T-Beam: RST=23
// - LoRa32 V1.6: RST=14
// - Outros modelos podem variar
const lmic_pinmap lmic_pins = {
    .nss = 18,      // Chip select
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 23,      // Ajuste se necessário (14 para V1.6)
    .dio = {26, 33, 32},  // DIO0, DIO1, DIO2
};

// ============================================================================
// Timing Configuration
// ============================================================================
const unsigned long TX_INTERVAL = 30000;  // Transmitir a cada 30 segundos
static osjob_t sendjob;
uint8_t packetCounter = 0;

// ============================================================================
// Forward Declarations
// ============================================================================
void onEvent (ev_t ev);
void do_send(osjob_t* j);

// ============================================================================
// Setup
// ============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println(F("====================================="));
  Serial.println(F("LilyGo LoRa32 - LoRaWAN Test"));
  Serial.println(F("Frequency: AU915"));
  Serial.println(F("====================================="));
  
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
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
  os_runloop_once();
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
    snprintf(message, sizeof(message), "LilyGo_Test_%d", packetCounter);
    
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
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED - Successfully joined network!"));
      Serial.print(F("NetID: "));
      Serial.println(LMIC.netid, DEC);
      Serial.print(F("DevAddr: "));
      Serial.print(LMIC.devaddr, HEX);
      Serial.println();
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
      Serial.print(F("RSSI: "));
      Serial.print(LMIC.rssi);
      Serial.println(F(" dBm"));
      Serial.print(F("SNR: "));
      Serial.print(LMIC.snr);
      Serial.println(F(" dB"));
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
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART - Transmission started"));
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





