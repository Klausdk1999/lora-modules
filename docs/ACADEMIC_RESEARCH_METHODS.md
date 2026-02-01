# Academic Research: River Level Estimation Methods, Deep Sleep Validation, and Power Consumption

This document compiles academic research findings on:
1. Methods for estimating river level from distance sensor readings
2. Deep sleep validation and power management studies
3. Power consumption measurements for LoRaWAN sensor nodes

## 1. River Level Estimation from Distance Sensor Readings

### 1.1 Ultrasonic Sensor Calibration and Temperature Compensation

#### Pereira et al. (2022) - Evaluation of Water Level in Flowing Channels Using Ultrasonic Sensors

**Key Findings:**
- Evaluated HC-SR04 ultrasonic sensors for water level monitoring in flowing channels
- Identified temperature compensation as critical for accurate measurements
- Speed of sound in air varies with temperature: `v = 331.3 + 0.606 * T` (m/s), where T is temperature in Celsius
- Recommended calibration against known reference points
- Found sensors suitable for educational and preliminary research applications

**Methodology:**
1. Install sensor at fixed height above expected water level
2. Measure distance from sensor to water surface
3. Calculate water level: `water_level = sensor_height - measured_distance`
4. Apply temperature compensation to correct speed of sound
5. Validate against reference measurements

**Implementation Notes:**
- Temperature sensor required for accurate compensation
- Multiple readings should be averaged to reduce noise
- Sensor should be positioned to avoid interference from waves or debris

**Reference:** Pereira, T. S. R., de Carvalho, T. P., Mendes, T. A., & Formiga, K. T. M. (2022). Evaluation of Water Level in Flowing Channels Using Ultrasonic Sensors. *Sustainability*, 14(9), 5512. DOI: 10.3390/su14095512

---

#### Mohammadreza MasoudiMoghaddam et al. (2024) - A Low-Cost Ultrasonic Sensor for Online Monitoring of Water Levels in Rivers and Channels

**Key Findings:**
- Developed and validated a low-cost ultrasonic sensor system for river monitoring
- Average measurement error below 3% in both laboratory and field settings
- Successfully tested in challenging conditions including foamy waves
- Validated sensor performance in real-world river environments

**Calibration Method:**
1. Laboratory calibration against reference water level
2. Field validation at multiple sites
3. Temperature and humidity compensation
4. Signal filtering to reduce noise from environmental factors

**Field Results:**
- Accuracy maintained in various weather conditions
- Reliable operation in flowing water with surface disturbances
- Cost-effective solution for distributed monitoring networks

**Reference:** Mohammadreza MasoudiMoghaddam, Yazdi, J., & Shahsavandi, M. (2024). A Low-Cost Ultrasonic Sensor for Online Monitoring of Water Levels in Rivers and Channels. *Flow Measurement and Instrumentation*, 102, 102777. DOI: 10.1016/j.flowmeasinst.2024.102777

---

### 1.2 LiDAR Sensor Calibration and Water Level Estimation

#### Paul et al. (2020) - A Technical Evaluation of Lidar-Based Measurement of River Water Levels

**Key Findings:**
- LiDAR sensors show strong temperature dependence requiring thermal compensation
- Temperature correction factor: approximately 0.1% per degree Celsius deviation from 25°C
- Sensor accuracy affected by internal temperature, not just ambient temperature
- Recommended thermal control or compensation algorithms for reliable deployment

**Calibration Methodology:**
1. Install LiDAR sensor at fixed height above water surface
2. Measure distance to water surface using ToF (Time-of-Flight) principle
3. Apply temperature compensation based on sensor internal temperature
4. Calculate water level: `water_level = sensor_height - (compensated_distance)`
5. Validate against reference measurements

**Temperature Compensation:**
- Monitor sensor internal temperature
- Apply correction: `corrected_distance = measured_distance * (1 + α * (T - T_ref))`
- Where α ≈ 0.001 (0.1% per °C), T is sensor temperature, T_ref = 25°C

**Implementation Impact:**
- Temperature sensor or internal temperature reading required
- Real-time compensation necessary for accurate measurements
- Temperature data should be included in data payload for post-processing

**Reference:** Paul, J. D., Buytaert, W., & Sah, N. (2020). A Technical Evaluation of Lidar-Based Measurement of River Water Levels. *Water Resources Research*, 56(4). DOI: 10.1029/2019wr026810

---

#### Santana et al. (2024) - Development and Calibration of a Low-Cost LIDAR Sensor for Water Level Measurements

**Key Findings:**
- Low-cost LiDAR achieved ±0.05m accuracy when compared to physical metric scale
- Measurements not significantly impacted by presence of sediments in water
- Validated for water level monitoring applications
- Extended range (22m) suitable for larger rivers

**Calibration Process:**
1. Laboratory calibration against known distances
2. Field validation at deployment site
3. Comparison with reference measurements
4. Statistical analysis of measurement accuracy

**Accuracy Results:**
- ±5cm accuracy in controlled conditions
- Maintained accuracy in field conditions
- Suitable for river level monitoring applications

**Reference:** Santana, V., Salustiano, R. E., & Tiezzi, R. (2024). Development and Calibration of a Low-Cost LIDAR Sensor for Water Level Measurements. *Flow Measurement and Instrumentation*. DOI: 10.1016/j.flowmeasinst.2024.102729

---

#### Tamari & Guerrero-Meza (2016) - Flash Flood Monitoring with an Inclined Lidar Installed at a River Bank

**Key Findings:**
- LiDAR effectiveness directly correlated with water turbidity
- Higher turbidity leads to better quality readings
- Near-infrared beam detects suspended particles in water
- Inclined installation can improve measurement reliability

**Water Surface Detection:**
- LiDAR beam interacts with suspended particles in turbid water
- Signal strength (flux) indicates measurement quality
- Monitoring signal strength helps assess reading reliability

**Implementation Notes:**
- Monitor signal strength/flux in sensor readings
- Higher turbidity conditions improve measurement quality
- Consider water turbidity when selecting deployment sites

**Reference:** Tamari, S., & Guerrero-Meza, V. (2016). Flash Flood Monitoring with an Inclined Lidar Installed at a River Bank: Proof of Concept. *Remote Sensing*, 8(10), 834. DOI: 10.3390/rs8100834

---

### 1.3 General Water Level Estimation Methods

#### Bresnahan et al. (2023) - A Low-Cost, DIY Ultrasonic Water Level Sensor for Education, Citizen Science, and Research

**Key Findings:**
- Open-source design for low-cost water level monitoring
- Suitable for education, citizen science, and research applications
- Emphasizes calibration and validation procedures

**Calibration Approach:**
1. Install sensor at known height above reference point
2. Take multiple measurements at different known water levels
3. Create calibration curve or lookup table
4. Validate against independent measurements

**Reference:** Bresnahan, P., Briggs, E., Davis, B., Rodriguez, A., Edwards, L., Peach, C., Renner, N., Helling, H., & Merrifield, M. (2023). A Low-Cost, DIY Ultrasonic Water Level Sensor for Education, Citizen Science, and Research. *Oceanography*, 36. DOI: 10.5670/oceanog.2023.101

---

## 2. Deep Sleep Validation and Power Management Studies

### 2.1 ESP32 Deep Sleep Power Consumption

#### Technical Documentation - ESP32 Datasheet

**Power Consumption Measurements:**
- **Active Mode:** 160-240mA (Wi-Fi/BT disabled), up to 790mA (Wi-Fi + BT active)
- **Modem Sleep:** 3-20mA (CPU active, Wi-Fi/BT disabled)
- **Light Sleep:** ~0.8mA (CPU paused, RAM retained)
- **Deep Sleep:** ~10µA (RTC only, most components powered down)
- **Hibernation:** ~5µA (RTC only, all power domains off)

**Deep Sleep Implementation:**
- RTC timer wakeup: configurable intervals
- External wakeup: GPIO interrupts
- ULP coprocessor: can remain active for low-power sensing
- Wakeup time: ~200ms from deep sleep

**Validation Approach:**
1. Measure current consumption in each mode using precision ammeter
2. Validate wakeup timing and reliability
3. Test battery life under different sleep intervals
4. Monitor system stability over extended periods

**Reference:** Espressif Systems. (2023). ESP32 Series Datasheet. Available: https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf

---

### 2.2 LoRaWAN Power Management Studies

#### Dao et al. (2025) - Low-cost, High Accuracy, and Long Communication Range Energy-Harvesting Beat Sensor with LoRa and Ω-Antenna for Water-Level Monitoring

**Key Findings:**
- Energy-harvesting approach for battery-less operation
- LoRaWAN communication for long-range, low-power transmission
- Deep sleep implementation critical for battery-powered nodes
- Demonstrated extended battery life through power management

**Power Management Strategy:**
- Deep sleep between measurements
- Short active periods for sensor reading and transmission
- Duty cycling to minimize active time
- Energy harvesting for extended operation

**Reference:** Dao, M.-H., Ishibashi, K., Nguyen, T.-A., Bui, D.-H., Hirayama, H., Tran, T.-A., & Tran, X.-T. (2025). Low-cost, High Accuracy, and Long Communication Range Energy-Harvesting Beat Sensor with LoRa and Ω-Antenna for Water-Level Monitoring. *IEEE Sensors Journal*. DOI: 10.1109/jsen.2025.3533014

---

#### Ishibashi et al. (2019) - Long Battery Life IoT Sensing by Beat Sensors

**Key Findings:**
- Beat sensor technology for ultra-low power consumption
- Extended battery life through optimized power management
- Deep sleep strategies validated in field deployments

**Power Consumption Results:**
- Significant reduction in power consumption through deep sleep
- Battery life extended to months/years depending on transmission interval
- Validated in real-world IoT deployments

**Reference:** Ishibashi, K., Takitoge, R., Manyvone, D., Ono, N., & Yamaguchi, S. (2019). Long Battery Life IoT Sensing by Beat Sensors. *2019 IEEE International Conference on Industrial Cyber Physical Systems (ICPS)*, 430-435. DOI: 10.1109/icphys.2019.8780159

---

### 2.3 WSN Power Management

#### Ferreira et al. (2023) - Conception and Design of WSN Sensor Nodes Based on Machine Learning, Embedded Systems and IoT Approaches for Pollutant Detection in Aquatic Environments

**Key Findings:**
- Power management critical for WSN node design
- Deep sleep implementation for extended battery life
- ESP32-based nodes with optimized power consumption

**Power Management Approach:**
- Deep sleep between measurement cycles
- Wakeup on timer or external events
- Minimize active time for sensor reading and transmission
- Battery monitoring for system health

**Reference:** Ferreira, Y., Silvério, C., & Viana, J. (2023). Conception and Design of WSN Sensor Nodes Based on Machine Learning, Embedded Systems and IoT Approaches for Pollutant Detection in Aquatic Environments. *IEEE Access*, 11, 117040-117052. DOI: 10.1109/access.2023.3325760

---

## 3. Power Consumption Measurements and Validation

### 3.1 LoRaWAN Power Consumption Characteristics

#### Sun et al. (2022) - Recent Advances in LoRa: A Comprehensive Survey

**Key Findings:**
- LoRaWAN designed for low-power, long-range communication
- Power consumption depends on spreading factor (SF) and transmission power
- Higher SF = longer range but higher power consumption
- Typical transmission power: 14-20dBm (25-100mW)

**Power Consumption Breakdown:**
- **Transmit (SF7, 14dBm):** ~120mA for ~100ms
- **Transmit (SF12, 20dBm):** ~150mA for ~2-3 seconds
- **Receive:** ~15-20mA
- **Idle:** ~1-2mA
- **Sleep:** <1µA (with proper implementation)

**Battery Life Estimation:**
- With 2000mAh battery and 15-minute transmission interval:
  - Active time per cycle: ~5-10 seconds
  - Sleep time: ~900 seconds
  - Average current: ~(10s * 150mA + 900s * 10µA) / 900s ≈ 1.67mA
  - Battery life: 2000mAh / 1.67mA ≈ 1200 hours ≈ 50 days
  - With deep sleep (10µA): Battery life extended to 6-12 months

**Reference:** Sun, Z., Yang, H., Liu, K., Yin, Z., Li, Z., & Xu, W. (2022). Recent Advances in LoRa: A Comprehensive Survey. *ACM Transactions on Sensor Networks*, 18(4). DOI: 10.1145/3543856

---

### 3.2 Sensor Power Consumption

#### TF02-Pro LiDAR Sensor Power Consumption

**Specifications:**
- Average current: ≤200mA
- Power consumption: ≤1W (at 5V)
- Operating voltage: 5V to 12V DC

**Power Management Strategy:**
- Power sensor only during measurement
- Use enable/disable pin if available
- Reduce measurement frequency when possible
- Consider duty cycling for battery-powered applications

**Reference:** Benewake. (2023). TF02-Pro User Manual. Available: https://en.benewake.com/uploadfiles/2023/03/20230331102719932.pdf

---

#### Ultrasonic Sensor Power Consumption

**HC-SR04:**
- Operating current: ~15mA
- Trigger pulse: ~10µs
- Measurement time: ~38ms per reading

**JSN-SR04T:**
- Operating current: ~30-40mA
- Waterproof design with similar power characteristics
- Suitable for outdoor deployment

**Power Management:**
- Power sensor only during measurement
- Multiple readings can be taken quickly
- Low power consumption compared to LiDAR sensors

---

## 4. Implementation Recommendations

### 4.1 River Level Estimation Implementation

**For Ultrasonic Sensors (JSN-SR04T):**
1. Install sensor at fixed height (2-4m above expected water level)
2. Measure distance to water surface
3. Apply temperature compensation: `v_sound = 331.3 + 0.606 * T`
4. Calculate water level: `level = height - distance`
5. Average multiple readings (3-5) to reduce noise
6. Filter outliers before averaging

**For LiDAR Sensors (TF02-Pro):**
1. Install sensor at fixed height above water surface
2. Measure distance using ToF
3. Apply temperature compensation: `corrected = measured * (1 + 0.001 * (T - 25))`
4. Monitor signal strength/flux for quality assessment
5. Calculate water level: `level = height - corrected_distance`
6. Average multiple readings for reliability

**Calibration Procedure:**
1. Install sensor at deployment location
2. Take measurements at known water levels (if possible)
3. Compare with reference measurements
4. Create calibration curve or apply offset correction
5. Validate accuracy over measurement range

---

### 4.2 Deep Sleep Implementation Validation

**Testing Procedure:**
1. Measure current consumption in each mode:
   - Active mode (sensor reading + transmission)
   - Deep sleep mode
   - Wakeup current spike
2. Validate wakeup timing:
   - Measure time from sleep to active
   - Verify timer accuracy
   - Test wakeup reliability over extended periods
3. Battery life estimation:
   - Calculate average current: `I_avg = (I_active * t_active + I_sleep * t_sleep) / (t_active + t_sleep)`
   - Estimate battery life: `t_battery = C_battery / I_avg`
4. Field validation:
   - Deploy nodes with battery monitoring
   - Track power consumption over time
   - Validate battery life predictions

**Expected Results:**
- Deep sleep current: ~10µA
- Active current: ~150-200mA (transmission) + sensor current
- Wakeup time: ~200ms
- Battery life: 6-12 months with 2000mAh battery and 15-minute intervals

---

### 4.3 Power Consumption Measurement

**Measurement Setup:**
1. Use precision ammeter or current shunt
2. Measure current in different modes:
   - Deep sleep
   - Sensor reading
   - LoRa transmission
   - Active processing
3. Calculate duty cycle and average current
4. Validate against datasheet specifications

**Key Metrics:**
- Average current consumption
- Peak current during transmission
- Sleep current
- Battery life estimation
- Power consumption per measurement cycle

---

## 5. References Summary

### River Level Estimation
1. Pereira et al. (2022) - Ultrasonic sensor evaluation and temperature compensation
2. Mohammadreza MasoudiMoghaddam et al. (2024) - Low-cost ultrasonic sensor validation
3. Paul et al. (2020) - LiDAR temperature compensation and calibration
4. Santana et al. (2024) - LiDAR calibration and accuracy validation
5. Tamari & Guerrero-Meza (2016) - LiDAR turbidity effects
6. Bresnahan et al. (2023) - DIY ultrasonic sensor calibration

### Deep Sleep and Power Management
1. ESP32 Datasheet (2023) - Power consumption specifications
2. Dao et al. (2025) - Energy-harvesting LoRaWAN sensor
3. Ishibashi et al. (2019) - Long battery life IoT sensing
4. Ferreira et al. (2023) - WSN power management

### Power Consumption Studies
1. Sun et al. (2022) - LoRaWAN power consumption survey
2. TF02-Pro User Manual - Sensor power specifications

---

## 6. Implementation Status

### Completed
- ✅ Deep sleep implementation for both nodes
- ✅ Temperature compensation algorithm (TF02-Pro)
- ✅ Battery monitoring framework
- ✅ Power management strategy

### Pending Validation
- ⏳ Deep sleep current consumption measurement
- ⏳ Battery life field validation
- ⏳ River level estimation accuracy validation
- ⏳ Temperature compensation effectiveness
- ⏳ Power consumption per measurement cycle

---

## 7. Next Steps for Validation

1. **Laboratory Testing:**
   - Measure power consumption in each mode
   - Validate deep sleep current (~10µA)
   - Test wakeup reliability
   - Calibrate sensors against reference

2. **Field Deployment:**
   - Deploy nodes at river site
   - Monitor battery consumption over time
   - Compare sensor readings with reference measurements
   - Validate temperature compensation

3. **Data Analysis:**
   - Analyze power consumption patterns
   - Calculate actual battery life
   - Evaluate sensor accuracy
   - Compare LiDAR vs Ultrasonic performance
