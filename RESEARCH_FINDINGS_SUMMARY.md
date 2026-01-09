# Research Findings Summary - River Level Estimation, Deep Sleep, and Power Consumption

## Overview

This document summarizes the academic research gathered on three critical aspects of the river level monitoring project:

1. **Methods for estimating river level from sensor readings**
2. **Deep sleep validation and power management studies**
3. **Power consumption measurements and validation**

## Key Research Papers Found

### River Level Estimation Methods

#### ✅ Already in Your Bibliography

1. **Pereira et al. (2022)** - "Evaluation of Water Level in Flowing Channels Using Ultrasonic Sensors"
   - **Key Finding:** Temperature compensation critical for ultrasonic sensors
   - **Method:** Speed of sound correction: `v = 331.3 + 0.606 * T`
   - **Location:** `referencias.bib` (line 354)

2. **Mohammadreza MasoudiMoghaddam et al. (2024)** - "A Low-Cost Ultrasonic Sensor for Online Monitoring of Water Levels in Rivers and Channels"
   - **Key Finding:** Average error <3% in field conditions
   - **Method:** Validated in challenging conditions including foamy waves
   - **Location:** `referencias.bib` (line 430)

3. **Paul et al. (2020)** - "A Technical Evaluation of Lidar-Based Measurement of River Water Levels"
   - **Key Finding:** Temperature compensation needed: ~0.1% per °C
   - **Method:** Thermal control or compensation algorithms
   - **Location:** `referencias.bib` (line 442)

4. **Santana et al. (2024)** - "Development and Calibration of a Low-Cost LIDAR Sensor for Water Level Measurements"
   - **Key Finding:** ±0.05m accuracy validated
   - **Method:** Laboratory and field calibration procedures
   - **Location:** `referencias.bib` (line 520)

5. **Tamari & Guerrero-Meza (2016)** - "Flash Flood Monitoring with an Inclined Lidar Installed at a River Bank"
   - **Key Finding:** Turbidity affects LiDAR performance
   - **Method:** Signal strength monitoring for quality assessment
   - **Location:** `referencias.bib` (line 195)

6. **Bresnahan et al. (2023)** - "A Low-Cost, DIY Ultrasonic Water Level Sensor"
   - **Key Finding:** Open-source calibration methods
   - **Method:** Calibration and validation procedures
   - **Location:** `referencias.bib` (line 498)

### Deep Sleep and Power Management

#### ✅ Already in Your Bibliography

1. **ESP32 Datasheet (2023)**
   - **Key Finding:** Deep sleep current ~10µA
   - **Method:** Power consumption specifications
   - **Location:** `referencias.bib` (line 625)

2. **Dao et al. (2025)** - "Low-cost, High Accuracy, and Long Communication Range Energy-Harvesting Beat Sensor with LoRa"
   - **Key Finding:** Energy-harvesting approach validated
   - **Method:** Deep sleep implementation for battery-powered nodes
   - **Location:** `referencias.bib` (line 85)

3. **Ishibashi et al. (2019)** - "Long Battery Life IoT Sensing by Beat Sensors"
   - **Key Finding:** Extended battery life through power management
   - **Method:** Deep sleep strategies validated in field
   - **Location:** `referencias.bib` (line 108)

4. **Ferreira et al. (2023)** - "Conception and Design of WSN Sensor Nodes"
   - **Key Finding:** Power management critical for WSN design
   - **Method:** ESP32-based nodes with optimized power consumption
   - **Location:** `referencias.bib` (line 476)

### Power Consumption Studies

#### ✅ Already in Your Bibliography

1. **Sun et al. (2022)** - "Recent Advances in LoRa: A Comprehensive Survey"
   - **Key Finding:** LoRaWAN power consumption characteristics
   - **Method:** Comprehensive survey of LoRa power consumption
   - **Location:** `referencias.bib` (line 607)

## Implementation Methods from Research

### River Level Estimation - Ultrasonic Sensors

**From Pereira et al. (2022) and Mohammadreza MasoudiMoghaddam et al. (2024):**

1. **Installation:**
   - Position sensor at fixed height (2-4m) above expected water level
   - Ensure line-of-sight to water surface

2. **Measurement:**
   - Measure distance from sensor to water surface
   - Take multiple readings (3-5) and average

3. **Temperature Compensation:**
   ```
   v_sound = 331.3 + 0.606 * T (°C)
   corrected_distance = measured_distance * (v_sound / 331.3)
   ```

4. **Water Level Calculation:**
   ```
   water_level = sensor_height - corrected_distance
   ```

5. **Validation:**
   - Compare with reference measurements
   - Expected accuracy: <3% error

### River Level Estimation - LiDAR Sensors

**From Paul et al. (2020) and Santana et al. (2024):**

1. **Installation:**
   - Position sensor at fixed height above water surface
   - Consider turbidity conditions (higher turbidity = better readings)

2. **Measurement:**
   - Measure distance using Time-of-Flight (ToF)
   - Monitor signal strength/flux for quality assessment
   - Take multiple readings and average

3. **Temperature Compensation:**
   ```
   correction_factor = 1 + 0.001 * (T_sensor - 25)
   corrected_distance = measured_distance * correction_factor
   ```
   Where 0.001 = 0.1% per °C deviation from 25°C

4. **Water Level Calculation:**
   ```
   water_level = sensor_height - corrected_distance
   ```

5. **Validation:**
   - Expected accuracy: ±0.05m (Santana et al.)
   - Monitor signal strength for quality assessment

### Deep Sleep Implementation

**From ESP32 Datasheet and Power Management Studies:**

1. **Power Consumption:**
   - **Deep Sleep:** ~10µA (RTC only)
   - **Active (Transmission):** ~150-200mA
   - **Active (Sensor Reading):** ~30-200mA (depending on sensor)

2. **Implementation:**
   ```cpp
   // Enable timer wakeup
   esp_sleep_enable_timer_wakeup(sleep_seconds * 1000000ULL);
   
   // Enter deep sleep
   esp_deep_sleep_start();
   ```

3. **Battery Life Estimation:**
   ```
   Average Current = (I_active * t_active + I_sleep * t_sleep) / (t_active + t_sleep)
   Battery Life = Battery Capacity / Average Current
   
   Example (15-minute interval, 2000mAh battery):
   - Active: 150mA for 10s
   - Sleep: 10µA for 890s
   - Average: ~1.67mA
   - Battery Life: ~1200 hours ≈ 50 days
   ```

4. **Validation:**
   - Measure current consumption in each mode
   - Validate wakeup timing and reliability
   - Test battery life over extended periods

## Research Gaps and Recommendations

### What We Found

✅ **River Level Estimation Methods:**
- Multiple validated methods for both ultrasonic and LiDAR sensors
- Temperature compensation algorithms documented
- Calibration procedures available

✅ **Deep Sleep Validation:**
- ESP32 power consumption specifications available
- Field validation studies exist (Ishibashi, Dao, Ferreira)
- Battery life estimation methods documented

✅ **Power Consumption:**
- LoRaWAN power characteristics well-documented (Sun et al.)
- Sensor power specifications available
- Battery life estimation methods established

### What Could Be Added (Optional)

1. **Specific ESP32 Deep Sleep Validation Papers:**
   - While we have technical documentation, specific academic validation papers are limited
   - Your implementation can contribute to this research area

2. **Field Deployment Case Studies:**
   - More field validation studies would strengthen the thesis
   - Your deployment will add to this body of knowledge

3. **Comparative Studies:**
   - Direct comparison of LiDAR vs Ultrasonic in same conditions
   - Your deployment provides this opportunity

## How to Use This Research in Your Thesis

### Chapter 2 (Literature Review)

**Section: River Level Estimation Methods**
- Cite Pereira et al. (2022) for ultrasonic temperature compensation
- Cite Mohammadreza MasoudiMoghaddam et al. (2024) for ultrasonic validation
- Cite Paul et al. (2020) for LiDAR temperature compensation
- Cite Santana et al. (2024) for LiDAR calibration
- Cite Tamari & Guerrero-Meza (2016) for turbidity effects

**Section: Power Management and Deep Sleep**
- Cite ESP32 Datasheet for power specifications
- Cite Dao et al. (2025) for energy-harvesting approach
- Cite Ishibashi et al. (2019) for battery life validation
- Cite Ferreira et al. (2023) for WSN power management
- Cite Sun et al. (2022) for LoRaWAN power consumption

### Chapter 4 (Methodology)

**Section: Sensor Calibration**
- Reference calibration methods from Pereira, Santana, and Mohammadreza MasoudiMoghaddam
- Describe temperature compensation implementation
- Explain water level calculation method

**Section: Power Management**
- Reference ESP32 deep sleep specifications
- Describe power management strategy based on research
- Estimate battery life using methods from Sun et al.

### Chapter 5 (Results)

**Section: Sensor Accuracy Validation**
- Compare results with Pereira (ultrasonic) and Santana (LiDAR)
- Validate temperature compensation effectiveness
- Compare with literature findings

**Section: Power Consumption**
- Measure and report actual power consumption
- Compare with ESP32 datasheet specifications
- Validate battery life predictions
- Compare with Ishibashi and Dao studies

## Files Created

1. **`ACADEMIC_RESEARCH_METHODS.md`** - Comprehensive research document with:
   - Detailed methods from each paper
   - Implementation recommendations
   - Calibration procedures
   - Power consumption measurements
   - Validation approaches

2. **`RESEARCH_FINDINGS_SUMMARY.md`** (this file) - Quick reference guide

## Next Steps

1. ✅ **Research Gathered** - All relevant papers identified and documented
2. ⏳ **Implementation** - Methods implemented in code (already done)
3. ⏳ **Validation** - Field testing and measurement
4. ⏳ **Thesis Integration** - Add citations and methods to thesis chapters

## References Location

All references are already in your bibliography file:
- `Sensores-para-Monitoramento-do-Nivel-de-Rios/template_eca_ufsc_abnt/pos_textual/referencias.bib`

No new references need to be added - all research papers mentioned are already cited in your bibliography.
