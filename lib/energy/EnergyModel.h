/**
 * EnergyModel.h - LoRaWAN Energy Consumption Model
 * 
 * Implements energy consumption models for LoRaWAN sensor nodes based on:
 * - Casals et al. (2017): "Modeling the Energy Performance of LoRaWAN"
 * - Bouguera et al. (2018): "Energy Consumption Model for Sensor Nodes Based on LoRa and LoRaWAN"
 * 
 * The model decomposes node operation into discrete states:
 * E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx1 + E_rx2
 * 
 * Where:
 * - E_sleep: Energy consumed during deep sleep
 * - E_wu: Energy to wake up from deep sleep (boot overhead)
 * - E_meas: Energy to power sensors and acquire data
 * - E_proc: Energy for MCU to process data and format payload
 * - E_tx: Energy to transmit the packet
 * - E_rx1/rx2: Energy to listen for downlink acknowledgments (ACKs) in receive windows
 * 
 * Author: Klaus Dieter Kupper
 * Project: Comparative Evaluation of Low-Cost IoT Sensors for River Level Monitoring
 */

#ifndef ENERGY_MODEL_H
#define ENERGY_MODEL_H

#include <stdint.h>
#include <stdbool.h>

// LoRaWAN Spreading Factors (SF7 to SF12)
typedef enum {
    SF7 = 7,
    SF8 = 8,
    SF9 = 9,
    SF10 = 10,
    SF11 = 11,
    SF12 = 12
} LoRaSpreadingFactor;

// Energy state components (in µWh)
struct EnergyState {
    float sleep;      // E_sleep: Energy during deep sleep
    float wakeup;     // E_wu: Energy to wake up from deep sleep
    float measure;    // E_meas: Energy for sensor measurement
    float process;    // E_proc: Energy for data processing
    float transmit;   // E_tx: Energy for transmission
    float receive1;   // E_rx1: Energy for first receive window
    float receive2;   // E_rx2: Energy for second receive window
    float total;      // E_total: Total energy per cycle
};

// Battery life estimation structure
struct BatteryLifeEstimate {
    float energyPerCycle_Wh;      // Energy per cycle in Watt-hours
    float cyclesPerDay;            // Number of cycles per day
    float energyPerDay_Wh;         // Energy per day in Watt-hours
    float batteryCapacity_Wh;      // Battery capacity in Watt-hours
    float estimatedDays;           // Estimated battery life in days
    float estimatedMonths;         // Estimated battery life in months
    float estimatedYears;          // Estimated battery life in years (if > 1 year)
};

/**
 * Calculate Time on Air (ToA) for LoRaWAN transmission
 * Based on Casals et al. (2017): T_oa ∝ 2^SF / BW
 * 
 * @param spreadingFactor Spreading Factor (SF7-SF12)
 * @param bandwidth Bandwidth in Hz (typically 125000 for LoRaWAN)
 * @param payloadSize Payload size in bytes
 * @param codingRate Coding Rate (typically 4/5 = 1)
 * @param preambleLength Preamble length (typically 8)
 * @return Time on Air in milliseconds
 */
float calculateTimeOnAir(LoRaSpreadingFactor spreadingFactor, 
                         uint32_t bandwidth, 
                         uint8_t payloadSize,
                         uint8_t codingRate = 1,
                         uint8_t preambleLength = 8);

/**
 * Calculate transmission energy (E_tx)
 * Based on Casals et al. (2017): E_tx = V × I_tx × T_oa
 * 
 * @param voltage Supply voltage in Volts (typically 3.3V or 3.7V for LiPo)
 * @param txCurrent Transmission current in mA (typically 100-150mA for SX1276/8)
 * @param timeOnAir_ms Time on Air in milliseconds
 * @return Transmission energy in µWh (micro-Watt-hours)
 */
float calculateTransmissionEnergy(float voltage, float txCurrent, float timeOnAir_ms);

/**
 * Calculate total energy per cycle
 * Based on Bouguera et al. (2018): E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx
 * 
 * @param spreadingFactor Spreading Factor used for transmission
 * @param voltage Supply voltage in Volts
 * @param sleepTime_sec Sleep time between cycles in seconds
 * @param activeTime_sec Active time (wakeup + measure + process + transmit + receive) in seconds
 * @param payloadSize Payload size in bytes
 * @param outEnergyState Pointer to EnergyState structure to fill with component energies
 * @return Total energy per cycle in µWh
 */
float calculateEnergyPerCycle(LoRaSpreadingFactor spreadingFactor,
                              float voltage,
                              uint32_t sleepTime_sec,
                              float activeTime_sec,
                              uint8_t payloadSize,
                              EnergyState* outEnergyState);

/**
 * Estimate battery life based on energy model
 * Based on Casals et al. (2017): Battery life = Battery_Capacity / (E_total_per_cycle × Cycles_per_day)
 * 
 * @param batteryCapacity_mAh Battery capacity in mAh
 * @param voltage Battery voltage in Volts (typically 3.7V for LiPo)
 * @param energyPerCycle_µWh Energy consumed per cycle in µWh
 * @param transmissionInterval_sec Transmission interval in seconds
 * @param outEstimate Pointer to BatteryLifeEstimate structure to fill
 * @return Estimated battery life in days
 */
float estimateBatteryLife(uint16_t batteryCapacity_mAh,
                          float voltage,
                          float energyPerCycle_µWh,
                          uint32_t transmissionInterval_sec,
                          BatteryLifeEstimate* outEstimate);

/**
 * Estimate energy for different spreading factors
 * Useful for analyzing energy trade-off between range and battery life
 * Based on Casals et al. (2017): Increasing SF from 7 to 12 increases energy by ~40x
 * 
 * @param voltage Supply voltage in Volts
 * @param payloadSize Payload size in bytes
 * @param spreadingFactors Array of spreading factors to analyze (typically SF7-SF12)
 * @param numFactors Number of spreading factors in array
 * @param outEnergies Array to store energy values (in µWh) - must have size >= numFactors
 */
void estimateEnergyForSpreadingFactors(float voltage,
                                       uint8_t payloadSize,
                                       LoRaSpreadingFactor spreadingFactors[],
                                       uint8_t numFactors,
                                       float outEnergies[]);

/**
 * Get typical current consumption values for ESP32-based LoRaWAN nodes
 * Based on Casals et al. (2017), Bouguera et al. (2018), and Pires et al. (2025)
 * 
 * @param sleepCurrent_µA Output: Deep sleep current in µA (typically 10-150 µA)
 * @param activeCurrent_mA Output: Active mode current in mA (typically 100-150 mA)
 * @param txCurrent_mA Output: Transmission current in mA (typically 100-150 mA for SX1276/8)
 * @param rxCurrent_mA Output: Receive current in mA (typically 30-50 mA)
 */
void getTypicalCurrentConsumption(float* sleepCurrent_µA,
                                  float* activeCurrent_mA,
                                  float* txCurrent_mA,
                                  float* rxCurrent_mA);

#endif // ENERGY_MODEL_H
