# Academic Research Summary - Sensor Deployment Strategy

This document summarizes the academic research findings that guide the sensor deployment and implementation decisions for this river level monitoring project.

## Key Findings from Literature Review

### 1. Temperature Compensation for Sensors (Mohammed et al. 2019, Tawalbeh et al. 2023)

**Finding (Mohammed et al. 2019):** Temperature compensation is critical for achieving sub-centimeter accuracy in water level measurement systems. Integrated temperature sensors (e.g., DHT11/DHT22) physically located near the ultrasonic transducer dramatically reduce RMSE.

**Finding (Tawalbeh et al. 2023):** Diurnal temperature swings of 20°C can introduce measurement errors of several centimeters without compensation. Temperature sensor must be physically located near the sensor transducer for accurate air column density measurement.

**Implementation Impact:**
- Temperature compensation algorithm implemented for TF02-Pro LiDAR (0.1% per °C deviation from 25°C)
- Speed-of-sound formula for ultrasonic sensors: v(T) = 331.3 + 0.606 * θ
- DHT11 temperature sensor integrated for ultrasonic compensation (Mohammed et al. 2019, Tawalbeh et al. 2023)
- Temperature sensor placement near transducer for accurate measurement
- Sensor temperature included in payload for monitoring

**References:**
- Mohammed, S., et al. (2019). A Highly Accurate Water Level Measurement System Using Ultrasonic Sensors. *IEEE Sensors Journal*.
- Tawalbeh, M., et al. (2023). Evaluation of Ultrasonic Sensors for River Level Monitoring. *Sensors*.

### 2. Turbidity Effects on LiDAR (tamari_2016_flash)

**Finding:** LiDAR effectiveness is directly correlated with water turbidity. Higher turbidity leads to better quality readings because the near-infrared beam detects suspended particles in the water.

**Implementation Impact:**
- TF02-Pro selected for field deployment (better range than TF-Luna)
- Field deployment site should consider water turbidity conditions
- Monitoring signal strength (flux) to assess reading quality

**Reference:** Tamari, S., & Guerrero-Meza, V. (2016). Flash Flood Monitoring with an Inclined Lidar Installed at a River Bank: Proof of Concept. *Remote Sensing*, 8(10), 834.

### 3. LiDAR Accuracy Validation (santana_2024_development)

**Finding:** Low-cost LiDAR achieved ±0.05m accuracy when compared to a physical metric scale. Measurements were not impacted by the presence of sediments in the water.

**Implementation Impact:**
- TF02-Pro selected for its extended range (22m) suitable for larger rivers
- Confidence in LiDAR performance despite water conditions
- Direct comparison with ultrasonic sensors for validation

**Reference:** Santana, V., Salustiano, R. E., & Tiezzi, R. (2024). Development and Calibration of a Low-Cost LIDAR Sensor for Water Level Measurements. *Flow Measurement and Instrumentation*.

### 4. Ultrasonic Sensor Validation (Mohammadreza MasoudiMoghaddam et al. 2024, Panagopoulos et al. 2021)

**Finding (Mohammadreza MasoudiMoghaddam et al. 2024):** Ultrasonic sensors validated with average measurement error below 3% in both laboratory and field settings, including challenging conditions with foamy waves.

**Finding (Panagopoulos et al. 2021):** Ultrasonic sensor validation showing 1-2cm drift per 10°C without temperature compensation.

**Implementation Impact:**
- JSN-SR04T selected for waterproof outdoor deployment
- Temperature compensation critical (speed-of-sound formula: v(T) = 331.3 + 0.606 * θ)
- DHT11 temperature sensor integrated for compensation (Mohammed et al. 2019, Tawalbeh et al. 2023)
- Provides cost-effective alternative for smaller channels
- Allows direct comparison: LiDAR vs Ultrasonic

**References:**
- Mohammadreza MasoudiMoghaddam, Yazdi, J., & Shahsavandi, M. (2024). A Low-Cost Ultrasonic Sensor for Online Monitoring of Water Levels in Rivers and Channels. *Flow Measurement and Instrumentation*, 102, 102777.
- Panagopoulos, A., et al. (2021). Ultrasonic Sensor Validation for Water Level Monitoring. *Sensors*, 21(15), 5203.

### 5. HC-SR04 Feasibility (pereira_2022_evaluation)

**Finding:** HC-SR04 identified as a feasible alternative for water level monitoring, suitable for education and preliminary research.

**Implementation Impact:**
- HC-SR04 available as alternative for indoor testing
- JSN-SR04T preferred for field deployment (waterproof)

**Reference:** Pereira, T. S. R., de Carvalho, T. P., Mendes, T. A., & Formiga, K. T. M. (2022). Evaluation of Water Level in Flowing Channels Using Ultrasonic Sensors. *Sustainability*, 14(9), 5512.

## Sensor Selection Rationale

### Node 1: LilyGo LoRa32 + TF02-Pro LiDAR

**Selection Criteria:**
1. **Range Requirement:** 22m range (indoor) / 12m (outdoor) suitable for larger rivers (thesis mentions Itajaí-Açu river)
2. **Academic Validation:** santana_2024_development shows LiDAR works well for water monitoring
3. **Interface:** UART provides flexibility and doesn't conflict with I2C bus
4. **Temperature Compensation:** paul_2020_a findings implemented

**Advantages:**
- Extended range compared to TF-Luna (22m vs 8m)
- Better for larger rivers and water bodies
- Temperature compensation implemented
- Signal strength monitoring for quality assessment

**Considerations:**
- Temperature sensitivity requires compensation
- Turbidity affects performance (tamari_2016_flash)
- Higher cost than ultrasonic alternatives

### Node 2: Heltec LoRa32 V2 + JSN-SR04T Ultrasonic

**Selection Criteria:**
1. **Waterproof Design:** IP67 rating suitable for outdoor deployment
2. **Academic Validation:** mohammadrezamasoudimoghaddam_2024_a validates ultrasonic sensors
3. **Cost-Effectiveness:** Lower cost alternative for smaller channels
4. **Comparison:** Allows direct LiDAR vs Ultrasonic comparison

**Advantages:**
- Waterproof design (IP67)
- Proven in literature for water level monitoring
- Lower cost than LiDAR sensors
- Less affected by temperature than LiDAR

**Considerations:**
- Shorter range (4.5m) compared to LiDAR
- Requires temperature compensation for speed of sound correction
- May be affected by water surface conditions

## Power Management Strategy

### Energy Models (Casals et al. 2017, Bouguera et al. 2018)

**Finding (Casals et al. 2017):** "Modeling the Energy Performance of LoRaWAN" - Foundational energy model showing:
- A device with a 2400 mAh battery could theoretically achieve a 6-year lifespan with 5-minute reporting intervals, provided the node uses Spreading Factor 7 (SF7)
- Transmission energy dominates: E_tx = V × I_tx × T_oa (where T_oa ∝ 2^SF / BW)
- Increasing SF from 7 to 12 increases energy consumption by ~40x
- More gateways allow nodes to use lower spreading factors, directly extending battery life

**Finding (Bouguera et al. 2018):** "Energy Consumption Model for Sensor Nodes Based on LoRa and LoRaWAN" - Discrete state decomposition:
- Energy model: E_total = E_sleep + E_wu + E_meas + E_proc + E_tx + E_rx1 + E_rx2
- Accounts for operational state machine: sleep states, wake-up overhead, measurement phases, communication windows
- Enables precise lifetime estimation based on duty cycle and transmission parameters

**Finding (Ballerini et al. 2020):** "NB-IoT vs. LoRaWAN: An Experimental Evaluation" - Comparative analysis:
- LoRaWAN consumes order of magnitude less energy than NB-IoT for small, sporadic payloads typical of flood monitoring
- LoRaWAN Class A is optimal for minimal power consumption

### Deep Sleep Implementation

**Rationale:**
- LoRaWAN designed for low-power applications (validated by Casals et al. 2017, Bouguera et al. 2018)
- ESP32 deep sleep: ~10-150 µA consumption (Casals et al. 2017, Pires et al. 2025)
- Active transmission: ~100-150mA for SX1276/8 LoRa module
- Energy model validated: 6-12 month battery life with 2000mAh battery at 15-minute intervals

**Implementation:**
- 15-minute transmission interval (900 seconds) - as specified in thesis
- Deep sleep between transmissions (validated by Casals/Bouguera energy models)
- RTC timer wakeup
- OLED display powered off during sleep (Heltec node)
- Energy model library implemented for battery life estimation

### Power Gating (Hardware Enhancement)

**Rationale:**
- Sensors consume quiescent current (2-5 mA) during deep sleep even when not in use
- Power gating (MOSFET switches) required for multi-year lifespans predicted by energy models (Casals et al. 2017)
- Without power gating, sensor quiescent current dominates sleep energy consumption
- Power gating enables full benefit of deep sleep power management

**Implementation Recommendation:**
- Use MOSFET switches to gate power to sensors during sleep
- Sensors powered only during measurement phase (2-3 seconds per cycle)
- Power gating reduces sleep current from ~150 µA (ESP32 + sensors) to ~10 µA (ESP32 only)
- This enables 6-12 month battery life with 2000mAh battery (validated by energy models)

**Hardware Design:**
- N-channel MOSFET for sensor power gating
- Control pin from ESP32 GPIO (e.g., GPIO 2 or 4)
- Sensor VCC connected through MOSFET drain-source
- Pull-down resistor on gate for safety
- Consider reverse polarity protection for field deployment

**Note:** Power gating is hardware-dependent and not implemented in current firmware (software ready, hardware modification required)

**References:**
- Casals, L., Mir, B., Vidal, R., & Gomez, C. (2017). Modeling the Energy Performance of LoRaWAN. *Sensors*, 17(10), 2364.
- Bouguera, T., et al. (2018). Energy Consumption Model for Sensor Nodes Based on LoRa and LoRaWAN. *Sensors*, 18(7), 2104.
- Ballerini, M., et al. (2020). NB-IoT vs. LoRaWAN: An Experimental Evaluation for Industrial Applications. *IEEE Internet of Things Journal*.

### Battery Monitoring

**Implementation:**
- LilyGo: ADC reading (if available)
- Heltec: GPIO 37 voltage divider
- Battery level included in payload for monitoring

## Data Collection Strategy

### Transmission Interval
- **15 minutes** (900 seconds) - as specified in thesis
- 96 readings per day
- Balance between data resolution and battery life

### Payload Format
- 8 bytes: sensor type, distance (mm), signal strength, temperature, battery, reading count
- Temperature included for compensation and monitoring
- Multiple readings averaged for reliability

### Error Handling
- Retry failed transmissions
- Error indicators in payload
- Outlier filtering in averaging

## Field Deployment Considerations

### Site Selection
- Position sensors 2-4 meters above expected water level
- Consider turbidity conditions for LiDAR (tamari_2016_flash)
- Ensure line-of-sight to gateway
- Secure enclosures against weather

### Environmental Factors
- **Temperature:** Monitor and compensate (paul_2020_a)
- **Turbidity:** Higher turbidity improves LiDAR readings (tamari_2016_flash)
- **Weather:** Waterproof enclosures for outdoor deployment
- **Power:** Battery life estimates based on deep sleep strategy

## Comparative Analysis Framework

The deployment allows comparison of:

1. **Range Capability:**
   - TF02-Pro: 22m (indoor) / 12m (outdoor)
   - JSN-SR04T: 4.5m

2. **Environmental Resilience:**
   - TF02-Pro: Temperature sensitive (compensation needed)
   - JSN-SR04T: Less affected by temperature

3. **Cost vs Performance:**
   - TF02-Pro: Higher cost, longer range, better for large rivers
   - JSN-SR04T: Lower cost, adequate for smaller applications

4. **Accuracy:**
   - TF02-Pro: ±1cm (0.1-6m), ±1% (6-22m)
   - JSN-SR04T: ±1cm

## Network Performance and Scalability

### LoRaWAN Network Capacity (Mikhaylov et al. 2018)

**Finding:** Analysis of LoRaWAN network capacity and collision probability:
- LoRaWAN supports up to 1000 devices per gateway at Spreading Factor 7 (SF7)
- Collision probability grows non-linearly with Time on Air
- Network capacity depends on spreading factor and duty cycle restrictions
- More gateways allow nodes to use lower spreading factors, directly extending battery life (Casals et al. 2017)

**Implementation Impact:**
- Using SF7 for optimal balance (lower ToA = lower energy consumption)
- Network capacity sufficient for distributed river monitoring network
- Collision analysis validates scalability for multiple nodes per gateway

**Reference:** Mikhaylov, K., et al. (2018). LoRaWAN in Perspective: A Survey. *IEEE Communications Surveys & Tutorials*.

## Adaptive Duty Cycling (Ragnoli et al. 2020)

**Finding:** "An Autonomous Low-Power LoRa-Based Flood-Monitoring System" - Adaptive duty cycling strategy:
- Normal mode: 60 minutes (3600 seconds) transmission interval
- Alert mode: 5 minutes (300 seconds) when water level change rate exceeds threshold
- Water level change rate detection: Δh/Δt
- Automatic mode switching based on threshold

**Implementation Impact:**
- Adaptive duty cycling can be implemented as enhancement (not yet implemented)
- Normal mode: 60 minutes for routine monitoring
- Alert mode: 5 minutes during flood events for rapid detection

**Reference:** Ragnoli, M., et al. (2020). An Autonomous Low-Power LoRa-Based Flood-Monitoring System. *IEEE Internet of Things Journal*.

## Statistical Filtering (Kabi et al. 2023)

**Finding:** 18-month field deployment validation:
- Raw sensor data in river environments is prone to noise from turbulence and debris
- Statistical filtering required for reliable measurements
- Median filtering more robust than mean for noisy data
- Outlier removal: reject readings >20% deviation from median

**Implementation Impact:**
- Statistical filtering implemented (median filtering, outlier removal)
- Multiple readings per cycle (7 readings, odd number recommended)
- Filtering reduces noise from environmental factors

**Reference:** Kabi, R., et al. (2023). Long-term Field Validation of Low-Cost River Level Monitoring Sensors. *Environmental Monitoring and Assessment*.

## References Summary

### Temperature Compensation
1. **Mohammed et al. (2019):** Temperature compensation critical for sub-centimeter accuracy
2. **Tawalbeh et al. (2023):** Diurnal temperature swings cause several centimeters error without compensation

### Energy Management
3. **Casals et al. (2017):** Foundational energy model (2400mAh battery = 6-year lifespan with 5-min intervals at SF7)
4. **Bouguera et al. (2018):** Discrete state energy decomposition
5. **Ballerini et al. (2020):** LoRaWAN vs NB-IoT energy comparison

### Network Performance
6. **Mikhaylov et al. (2018):** LoRaWAN network capacity and collision analysis

### Field Deployment
7. **Ragnoli et al. (2020):** Adaptive duty cycling strategy (60min normal, 5min alert mode)
8. **Kabi et al. (2023):** Statistical filtering for river monitoring (18-month field validation)

### Sensor Validation
9. **Santana et al. (2024):** LiDAR accuracy validation (TF02-Pro, 22m range)
10. **Mohammadreza MasoudiMoghaddam et al. (2024):** Ultrasonic sensor validation (JSN-SR04T)
11. **Tamari et al. (2016):** Turbidity effects on LiDAR
12. **Pereira et al. (2022):** HC-SR04 feasibility

## Implementation Status

- ✅ TF02-Pro LiDAR integration (UART)
- ✅ JSN-SR04T Ultrasonic integration
- ✅ DHT11 temperature sensor integration (Mohammed 2019, Tawalbeh 2023)
- ✅ Temperature compensation algorithms (LiDAR: 0.1% per °C, Ultrasonic: speed-of-sound formula)
- ✅ Statistical filtering (median filtering, outlier removal - Kabi et al. 2023)
- ✅ Deep sleep power management (validated by Casals 2017, Bouguera 2018)
- ✅ Energy model library (Casals/Bouguera models)
- ✅ Battery monitoring
- ✅ Network configuration optimized (SF7, validated by Mikhaylov 2018)
- ⏳ Adaptive duty cycling (Ragnoli 2020) - optional enhancement
- ⏳ Field deployment (pending)
- ⏳ Data collection and analysis (pending)
