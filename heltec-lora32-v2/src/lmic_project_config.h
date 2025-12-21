/**
 * LMIC Project Configuration
 * Heltec WiFi LoRa 32 V2
 * 
 * This file configures the MCCI LoRaWAN LMIC library for EU868 region.
 * Adjust the settings below based on your region and requirements.
 */

#ifndef _LMIC_PROJECT_CONFIG_H_
#define _LMIC_PROJECT_CONFIG_H_

// Select your region here:
// - CFG_eu868  for Europe (868 MHz)
// - CFG_us915  for United States (915 MHz)
// - CFG_as923  for Asia (923 MHz)
// - CFG_au915  for Australia (915 MHz)
// - CFG_in866  for India (866 MHz)

#define CFG_eu868 1
// #define CFG_us915 1
// #define CFG_as923 1
// #define CFG_au915 1
// #define CFG_in866 1

// Disable automatic ping slots (Class A devices don't need this)
#define DISABLE_PING
#define DISABLE_BEACONS

// Enable this to use less common spreading factors
// #define LMIC_USE_INTERRUPTS

// Disable this if you don't need downlink
// #define DISABLE_JOIN
// #define DISABLE_LMIC

#endif // _LMIC_PROJECT_CONFIG_H_

