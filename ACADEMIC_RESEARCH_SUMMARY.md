# Academic Research Summary - Sensor Deployment Strategy

This document summarizes the academic research findings that guide the sensor deployment and implementation decisions for this river level monitoring project.

## Key Findings from Literature Review

### 1. Temperature Effects on LiDAR Sensors (paul_2020_a)

**Finding:** The sensor's accuracy was strongly dependent on its internal temperature, indicating a need for thermal control or compensation for reliable deployment.

**Implementation Impact:**
- Temperature compensation algorithm implemented for TF02-Pro
- Correction factor: ~0.1% per degree Celsius deviation from 25°C
- Sensor temperature included in payload for monitoring

**Reference:** Paul, J. D., Buytaert, W., & Sah, N. (2020). A Technical Evaluation of Lidar‐Based Measurement of River Water Levels. *Water Resources Research*, 56(4).

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

### 4. Ultrasonic Sensor Validation (mohammadrezamasoudimoghaddam_2024_a)

**Finding:** Ultrasonic sensors validated with average measurement error below 3% in both laboratory and field settings, including challenging conditions with foamy waves.

**Implementation Impact:**
- JSN-SR04T selected for waterproof outdoor deployment
- Provides cost-effective alternative for smaller channels
- Allows direct comparison: LiDAR vs Ultrasonic

**Reference:** Mohammadreza MasoudiMoghaddam, Yazdi, J., & Shahsavandi, M. (2024). A Low-Cost Ultrasonic Sensor for Online Monitoring of Water Levels in Rivers and Channels. *Flow Measurement and Instrumentation*, 102, 102777.

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

### Deep Sleep Implementation

**Rationale:**
- LoRaWAN designed for low-power applications
- ESP32 deep sleep: ~10µA consumption
- Active transmission: ~300-450mA for 5-10 seconds
- Battery life: 6-12 months with 2000mAh battery

**Implementation:**
- 15-minute transmission interval (as per thesis)
- Deep sleep between transmissions
- RTC timer wakeup
- OLED display powered off during sleep (Heltec node)

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

## References Summary

1. **paul_2020_a:** Temperature compensation for LiDAR
2. **tamari_2016_flash:** Turbidity effects on LiDAR
3. **santana_2024_development:** LiDAR accuracy validation
4. **mohammadrezamasoudimoghaddam_2024_a:** Ultrasonic sensor validation
5. **pereira_2022_evaluation:** HC-SR04 feasibility

## Implementation Status

- ✅ TF02-Pro integration (UART)
- ✅ Temperature compensation algorithm
- ✅ Deep sleep power management
- ✅ Battery monitoring
- ✅ JSN-SR04T integration
- ⏳ Field deployment (pending)
- ⏳ Data collection and analysis (pending)
