/**
 * EnergyModel.cpp - LoRaWAN Energy Consumption Model Implementation
 * 
 * Implements energy consumption models based on:
 * - Casals et al. (2017): "Modeling the Energy Performance of LoRaWAN"
 * - Bouguera et al. (2018): "Energy Consumption Model for Sensor Nodes Based on LoRa and LoRaWAN"
 * 
 * Key findings from these papers:
 * - Casals et al. (2017): 2400mAh battery = 6-year lifespan with 5-min intervals at SF7
 * - Transmission energy dominates: E_tx = V × I_tx × T_oa
 * - Time on Air increases exponentially with Spreading Factor: T_oa ∝ 2^SF / BW
 * - Increasing SF from 7 to 12 increases energy consumption by ~40x
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#include "EnergyModel.h"
#include <math.h>

// LoRaWAN bandwidth (typically 125 kHz for EU868/US915)
#define LORAWAN_BANDWIDTH_HZ  125000

// Typical current consumption values (validated by Casals et al. 2017, Pires et al. 2025)
#define TYPICAL_SLEEP_CURRENT_UA    10.0f      // Deep sleep: 10-150 µA
#define TYPICAL_ACTIVE_CURRENT_MA   120.0f     // Active mode: 100-150 mA
#define TYPICAL_TX_CURRENT_MA       130.0f     // Transmission: 100-150 mA (SX1276/8)
#define TYPICAL_RX_CURRENT_MA       40.0f      // Receive window: 30-50 mA

// Typical timing values (based on implementation)
#define TYPICAL_WAKEUP_TIME_MS      500.0f     // Wake-up from deep sleep: ~500ms
#define TYPICAL_MEASURE_TIME_MS     2000.0f    // Sensor measurement: ~2 seconds
#define TYPICAL_PROCESS_TIME_MS     100.0f     // Data processing: ~100ms
#define TYPICAL_RX_WINDOW1_MS       1000.0f    // First RX window: ~1 second
#define TYPICAL_RX_WINDOW2_MS       2000.0f    // Second RX window: ~2 seconds

/**
 * Calculate Time on Air (ToA) for LoRaWAN transmission
 * Based on Casals et al. (2017): T_oa ∝ 2^SF / BW
 * 
 * Simplified formula based on LMIC library calculations
 */
float calculateTimeOnAir(LoRaSpreadingFactor spreadingFactor, 
                         uint32_t bandwidth, 
                         uint8_t payloadSize,
                         uint8_t codingRate,
                         uint8_t preambleLength) {
    // Symbol duration = 2^SF / BW (in seconds)
    float symbolDuration = pow(2.0f, (float)spreadingFactor) / (float)bandwidth;
    
    // Preamble time
    float preambleTime = (float)(preambleLength + 4.25f) * symbolDuration;
    
    // Payload calculation (simplified)
    // Number of symbols in payload: depends on SF, CR, and payload size
    float numSymbolsPreamble = (float)preambleLength + 4.25f;
    float payloadBits = (float)(payloadSize * 8);  // Payload in bits
    float numSymbolsPayload = 8.0f + 
                              ceil((16.0f + payloadBits - 4.0f * (float)spreadingFactor + 
                                    28.0f) / (4.0f * (float)spreadingFactor)) * 
                              ((float)codingRate + 4.0f);
    
    float totalTime = (numSymbolsPreamble + numSymbolsPayload) * symbolDuration;
    
    // Convert to milliseconds
    return totalTime * 1000.0f;
}

/**
 * Calculate transmission energy (E_tx)
 * Based on Casals et al. (2017): E_tx = V × I_tx × T_oa
 */
float calculateTransmissionEnergy(float voltage, float txCurrent, float timeOnAir_ms) {
    // Convert to Watt-hours: V (V) × I (A) × T (h)
    // timeOnAir_ms is in milliseconds, convert to hours
    float timeHours = timeOnAir_ms / (1000.0f * 3600.0f);
    float energyWh = voltage * (txCurrent / 1000.0f) * timeHours;
    
    // Convert to micro-Watt-hours (µWh)
    return energyWh * 1000000.0f;
}

/**
 * Calculate total energy per cycle
 * Based on Bouguera et al. (2018): E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx
 */
float calculateEnergyPerCycle(LoRaSpreadingFactor spreadingFactor,
                              float voltage,
                              uint32_t sleepTime_sec,
                              float activeTime_sec,
                              uint8_t payloadSize,
                              EnergyState* outEnergyState) {
    if (outEnergyState == nullptr) {
        return 0.0f;
    }
    
    // Initialize energy state
    *outEnergyState = {0};
    
    // Get typical current consumption values
    float sleepCurrent_µA, activeCurrent_mA, txCurrent_mA, rxCurrent_mA;
    getTypicalCurrentConsumption(&sleepCurrent_µA, &activeCurrent_mA, &txCurrent_mA, &rxCurrent_mA);
    
    // E_sleep: Energy during deep sleep
    // E_sleep = V × I_sleep × T_sleep
    float sleepTimeHours = (float)sleepTime_sec / 3600.0f;
    outEnergyState->sleep = voltage * (sleepCurrent_µA / 1000000.0f) * sleepTimeHours * 1000000.0f;  // µWh
    
    // Calculate Time on Air for transmission
    float timeOnAir_ms = calculateTimeOnAir(spreadingFactor, LORAWAN_BANDWIDTH_HZ, payloadSize);
    
    // E_tx: Transmission energy (dominates total energy)
    outEnergyState->transmit = calculateTransmissionEnergy(voltage, txCurrent_mA, timeOnAir_ms);
    
    // E_wu: Wake-up energy (boot overhead)
    float wakeupTimeHours = TYPICAL_WAKEUP_TIME_MS / (1000.0f * 3600.0f);
    outEnergyState->wakeup = voltage * (activeCurrent_mA / 1000.0f) * wakeupTimeHours * 1000000.0f;  // µWh
    
    // E_meas: Sensor measurement energy
    float measureTimeHours = TYPICAL_MEASURE_TIME_MS / (1000.0f * 3600.0f);
    outEnergyState->measure = voltage * (activeCurrent_mA / 1000.0f) * measureTimeHours * 1000000.0f;  // µWh
    
    // E_proc: Data processing energy
    float processTimeHours = TYPICAL_PROCESS_TIME_MS / (1000.0f * 3600.0f);
    outEnergyState->process = voltage * (activeCurrent_mA / 1000.0f) * processTimeHours * 1000000.0f;  // µWh
    
    // E_rx1: First receive window energy
    float rx1TimeHours = TYPICAL_RX_WINDOW1_MS / (1000.0f * 3600.0f);
    outEnergyState->receive1 = voltage * (rxCurrent_mA / 1000.0f) * rx1TimeHours * 1000000.0f;  // µWh
    
    // E_rx2: Second receive window energy (Class A devices)
    float rx2TimeHours = TYPICAL_RX_WINDOW2_MS / (1000.0f * 3600.0f);
    outEnergyState->receive2 = voltage * (rxCurrent_mA / 1000.0f) * rx2TimeHours * 1000000.0f;  // µWh
    
    // E_total: Sum of all energy components
    outEnergyState->total = outEnergyState->sleep + 
                           outEnergyState->wakeup + 
                           outEnergyState->measure + 
                           outEnergyState->process + 
                           outEnergyState->transmit + 
                           outEnergyState->receive1 + 
                           outEnergyState->receive2;
    
    return outEnergyState->total;
}

/**
 * Estimate battery life based on energy model
 * Based on Casals et al. (2017): Battery life = Battery_Capacity / (E_total_per_cycle × Cycles_per_day)
 */
float estimateBatteryLife(uint16_t batteryCapacity_mAh,
                          float voltage,
                          float energyPerCycle_µWh,
                          uint32_t transmissionInterval_sec,
                          BatteryLifeEstimate* outEstimate) {
    if (outEstimate == nullptr) {
        return 0.0f;
    }
    
    // Calculate battery capacity in Watt-hours
    float batteryCapacity_Wh = (float)batteryCapacity_mAh * voltage / 1000.0f;
    outEstimate->batteryCapacity_Wh = batteryCapacity_Wh;
    
    // Calculate energy per cycle in Watt-hours
    float energyPerCycle_Wh = energyPerCycle_µWh / 1000000.0f;
    outEstimate->energyPerCycle_Wh = energyPerCycle_Wh;
    
    // Calculate cycles per day
    float cyclesPerDay = 86400.0f / (float)transmissionInterval_sec;
    outEstimate->cyclesPerDay = cyclesPerDay;
    
    // Calculate energy per day
    float energyPerDay_Wh = energyPerCycle_Wh * cyclesPerDay;
    outEstimate->energyPerDay_Wh = energyPerDay_Wh;
    
    // Estimate battery life in days
    float estimatedDays = batteryCapacity_Wh / energyPerDay_Wh;
    outEstimate->estimatedDays = estimatedDays;
    
    // Convert to months (30 days per month)
    outEstimate->estimatedMonths = estimatedDays / 30.0f;
    
    // Convert to years (365 days per year) if > 1 year
    if (estimatedDays >= 365.0f) {
        outEstimate->estimatedYears = estimatedDays / 365.0f;
    } else {
        outEstimate->estimatedYears = 0.0f;
    }
    
    return estimatedDays;
}

/**
 * Estimate energy for different spreading factors
 * Based on Casals et al. (2017): Increasing SF from 7 to 12 increases energy by ~40x
 */
void estimateEnergyForSpreadingFactors(float voltage,
                                       uint8_t payloadSize,
                                       LoRaSpreadingFactor spreadingFactors[],
                                       uint8_t numFactors,
                                       float outEnergies[]) {
    float txCurrent_mA;
    getTypicalCurrentConsumption(nullptr, nullptr, &txCurrent_mA, nullptr);
    
    for (uint8_t i = 0; i < numFactors; i++) {
        float timeOnAir_ms = calculateTimeOnAir(spreadingFactors[i], LORAWAN_BANDWIDTH_HZ, payloadSize);
        outEnergies[i] = calculateTransmissionEnergy(voltage, txCurrent_mA, timeOnAir_ms);
    }
}

/**
 * Get typical current consumption values for ESP32-based LoRaWAN nodes
 * Based on Casals et al. (2017), Bouguera et al. (2018), and Pires et al. (2025)
 */
void getTypicalCurrentConsumption(float* sleepCurrent_µA,
                                  float* activeCurrent_mA,
                                  float* txCurrent_mA,
                                  float* rxCurrent_mA) {
    // Deep sleep current: 10-150 µA depending on board design (Casals et al. 2017, Pires et al. 2025)
    if (sleepCurrent_µA != nullptr) {
        *sleepCurrent_µA = TYPICAL_SLEEP_CURRENT_UA;
    }
    
    // Active mode current: 100-150 mA for ESP32 (typical)
    if (activeCurrent_mA != nullptr) {
        *activeCurrent_mA = TYPICAL_ACTIVE_CURRENT_MA;
    }
    
    // Transmission current: 100-150 mA for SX1276/8 LoRa module
    if (txCurrent_mA != nullptr) {
        *txCurrent_mA = TYPICAL_TX_CURRENT_MA;
    }
    
    // Receive current: 30-50 mA for SX1276/8 LoRa module
    if (rxCurrent_mA != nullptr) {
        *rxCurrent_mA = TYPICAL_RX_CURRENT_MA;
    }
}
