# Energy Model Library

This library implements energy consumption models for LoRaWAN sensor nodes based on validated academic research.

## Academic Foundation

### Casals et al. (2017)
**"Modeling the Energy Performance of LoRaWAN"** - Foundational energy model

- Established theoretical baseline for LoRaWAN battery estimation
- Derived equations linking hardware sleep currents and LoRaWAN protocol overheads
- **Key finding**: A device with a 2400 mAh battery could theoretically achieve a 6-year lifespan with 5-minute reporting intervals, provided the node uses Spreading Factor 7 (SF7)
- Demonstrated that increasing SF from 7 to 12 increases energy consumption by ~40x
- Transmission energy formula: E_tx = V × I_tx × T_oa (where T_oa ∝ 2^SF / BW)

### Bouguera et al. (2018)
**"Energy Consumption Model for Sensor Nodes Based on LoRa and LoRaWAN"** - Discrete state decomposition

- Developed energy consumption model that decomposes node operation into discrete states
- Energy model: E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx1 + E_rx2
- Accounts for operational state machine: sleep states, wake-up overhead, measurement phases, communication windows
- Enables precise lifetime estimation based on duty cycle and transmission parameters

## Model Components

The energy model decomposes node operation into discrete states:

1. **E_sleep**: Energy during deep sleep (~10-150 µA for ESP32)
2. **E_wu**: Energy to wake up from deep sleep (boot overhead, ~500ms)
3. **E_meas**: Energy to power sensors and acquire data (~2 seconds)
4. **E_proc**: Energy for MCU to process data and format payload (~100ms)
5. **E_tx**: Energy to transmit the packet (dominates total energy)
6. **E_rx1/rx2**: Energy to listen for downlink acknowledgments in receive windows

## Usage Example

```cpp
#include "energy/EnergyModel.h"

// Calculate energy per cycle for SF7 transmission
EnergyState energyState;
float voltage = 3.7f;  // LiPo battery voltage
uint32_t sleepTime = 900;  // 15 minutes = 900 seconds
float activeTime = 5.0f;  // 5 seconds active time
uint8_t payloadSize = 8;  // 8-byte payload

float totalEnergy = calculateEnergyPerCycle(
    SF7,                    // Spreading Factor
    voltage,                // Supply voltage
    sleepTime,              // Sleep time (seconds)
    activeTime,             // Active time (seconds)
    payloadSize,            // Payload size (bytes)
    &energyState            // Output: energy components
);

// Estimate battery life
BatteryLifeEstimate estimate;
uint16_t batteryCapacity = 2000;  // 2000 mAh battery
uint32_t txInterval = 900;  // 15 minutes = 900 seconds

float batteryLifeDays = estimateBatteryLife(
    batteryCapacity,
    voltage,
    totalEnergy,  // Energy per cycle in µWh
    txInterval,
    &estimate
);

Serial.print("Battery life: ");
Serial.print(estimate.estimatedDays);
Serial.println(" days");
Serial.print("Energy per cycle: ");
Serial.print(energyState.total / 1000.0f);  // Convert µWh to mWh
Serial.println(" mWh");
```

## Key Functions

### `calculateTimeOnAir()`
Calculates Time on Air (ToA) for LoRaWAN transmission based on Spreading Factor, bandwidth, and payload size.

### `calculateTransmissionEnergy()`
Calculates transmission energy using: E_tx = V × I_tx × T_oa

### `calculateEnergyPerCycle()`
Calculates total energy per cycle by summing all energy components (sleep, wakeup, measure, process, transmit, receive).

### `estimateBatteryLife()`
Estimates battery life in days based on battery capacity, energy per cycle, and transmission interval.

### `estimateEnergyForSpreadingFactors()`
Analyzes energy consumption for different spreading factors, useful for optimizing range vs. battery life trade-off.

## Validation

The model has been validated against:
- **Casals et al. (2017)**: 2400 mAh battery = 6-year lifespan with 5-min intervals at SF7
- **Pires et al. (2025)**: Multi-month battery life with deep sleep implementation
- **Field deployments**: Empirical validation through long-term deployments (Kabi et al. 2023, Ragnoli et al. 2020)

## Notes

- Transmission energy (E_tx) dominates total energy consumption
- Time on Air increases exponentially with Spreading Factor
- More gateways allow nodes to use lower spreading factors, directly extending battery life (Casals et al. 2017)
- Power gating (MOSFET switches) required for sensors during sleep to achieve multi-year lifespans predicted by models
- Current implementation assumes typical ESP32 + SX1276/8 LoRa module current consumption values
