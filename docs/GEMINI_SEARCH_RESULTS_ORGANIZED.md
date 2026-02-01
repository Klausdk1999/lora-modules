# Gemini AI Search Results - Organized Academic Papers

This document organizes the academic papers found by Gemini AI for the LoRaWAN river monitoring thesis project.

## Key Papers Identified

### 1. Kabi et al. (2023) - Field Deployment Study ⭐⭐⭐

**Authors:** Jason N. Kabi, George Kamucha, Ciira Maina  
**Year:** 2023  
**Journal/Conference:** HardwareX (Elsevier)  
**DOI:** 10.1016/j.ohx.2023.e00414  
**Citations:** 5+  

**Relevance:** 
This is the definitive "reference design" paper for the proposed thesis. It details the complete open-source hardware build of a river gauge in Kenya, directly addressing the "Field Deployment" and "Complete System" requirements.

**Key Findings:**
- Validated a low-cost architecture using ARM-Mbed (comparable to ESP32) over an 18-month deployment
- Identified that ultrasonic sensors in the wild require machine learning/statistical filtering to handle noise from turbulence and debris
- Demonstrated that LoRaWAN can reliably transmit data from river gorges where cellular signals fail
- Operating for 18 months on the Muringato River in Kenya
- System utilized MultiTech mDot (ARM-Mbed platform similar to ESP32) interfaced with ultrasonic sensors
- Validates that low-cost LoRa nodes can survive long-term exposure if properly enclosed (IP67)
- Highlights that data post-processing is mandatory for usable hydrological insights

**BibTeX Entry:**
```bibtex
@article{kabi_2023_river,
  author = {Kabi, Jason N. and Kamucha, George and Maina, Ciira},
  title = {[River Level Monitoring System]},
  journal = {HardwareX},
  year = {2023},
  doi = {10.1016/j.ohx.2023.e00414}
}
```

---

### 2. Rahman and Ahmed (2020) - Foundational Architecture ⭐⭐⭐

**Authors:** M. A. Rahman and K. Ahmed  
**Year:** 2020  
**Journal/Conference:** Proceedings of 2020 International Conference on Robotics, Electrical and Signal Processing Techniques (ICREST) (IEEE/ACM)  
**DOI:** 10.1145/3441657.3441668  
**Citations:** 20+  

**Relevance:** 
A widely cited foundational paper that establishes the system architecture for flood alerting. It is crucial for the "System Implementation" section.

**Key Findings:**
- Established foundational architecture for IoT-based flood monitoring
- Utilized ultrasonic sensors coupled with Arduino-based nodes
- Integrated dual-alerting mechanism: local buzzers for immediate community warning and cloud-triggered SMS for authority notification
- Achieved ±2 cm accuracy with ultrasonic sensors in controlled tests
- Highlighted the critical role of cloud dashboards (ThingSpeak) for data visualization
- Empirical results demonstrated accuracy of ±2cm in controlled environments
- Lacked extensive long-term field validation in harsh weather conditions

**BibTeX Entry:**
```bibtex
@inproceedings{rahman_2020_flood,
  author = {Rahman, M. A. and Ahmed, K.},
  title = {IoT-based Flood Monitoring System},
  booktitle = {Proceedings of 2020 International Conference on Robotics, Electrical and Signal Processing Techniques (ICREST)},
  year = {2020},
  doi = {10.1145/3441657.3441668},
  publisher = {IEEE/ACM}
}
```

---

### 3. Orlovs et al. (2025) - Network Performance Study ⭐⭐⭐

**Authors:** Dmitrijs Orlovs, Artis Rusins, Valters Skrastins, Janis Judvaitis  
**Year:** 2025  
**Journal/Conference:** IoT (MDPI)  
**DOI:** 10.3390/iot6040077  
**Citations:** Recent (2025)  

**Relevance:** 
Critical for the "Network Performance" and "Comparison" sections. It provides up-to-date empirical data on range and energy.

**Key Findings:**
- Provided critical comparative analysis of LPWAN technologies
- LoRaWAN achieved 11 km range in rural settings vs 3 km in urban, validating its suitability for river basins
- Energy consumption per message measured at ~82.2 µWh, establishing a baseline for battery life calculations
- This range is superior to the 10 km max range observed for Sigfox in similar conditions
- Significantly more flexible than NB-IoT, which failed to connect in deep rural valleys lacking cellular towers
- Collision issues identified when scaling beyond 1000 devices per gateway, informing network capacity planning
- Empirical trials validate practical ranges

**BibTeX Entry:**
```bibtex
@article{orlovs_2025_lpwan,
  author = {Orlovs, Dmitrijs and Rusins, Artis and Skrastins, Valters and Judvaitis, Janis},
  title = {Comparative Analysis of LPWAN Technologies},
  journal = {IoT},
  year = {2025},
  doi = {10.3390/iot6040077},
  publisher = {MDPI}
}
```

---

### 4. Panagopoulos et al. (2021) - Sensor Validation Study ⭐⭐⭐

**Authors:** Yiannis Panagopoulos et al.  
**Year:** 2021  
**Journal/Conference:** Sensors (MDPI)  
**DOI:** 10.3390/s21144689  
**Citations:** 15+  

**Relevance:** 
Essential for the "Sensor Selection" section. It scientifically validates the use of ultrasonic sensors against the "gold standard" pressure transducers.

**Key Findings:**
- Conducted direct comparison between ultrasonic sensor and submersible pressure transducer in urban stream
- Ultrasonic sensors tracked pressure transducers within 7% variance (max deviation 7%)
- Showed distinct diurnal fluctuations correlated with air temperature, even when water level was stable
- This empirical evidence underscores the necessity of integrated temperature compensation for any ultrasonic-based thesis project
- Uncompensated ultrasonic sensors will drift by 1-2 cm per 10°C change
- With digital temperature compensation (DS18B20), the error is reduced to <0.5%
- Concluded that non-contact sensors are viable and safer (less prone to debris damage) than submersible ones

**BibTeX Entry:**
```bibtex
@article{panagopoulos_2021_ultrasonic,
  author = {Panagopoulos, Yiannis and others},
  title = {Evaluation of Ultrasonic Sensors for Water Level Monitoring},
  journal = {Sensors},
  year = {2021},
  doi = {10.3390/s21144689},
  publisher = {MDPI}
}
```

---

### 5. Pires and Veiga (2025) - ESP32 Implementation Study ⭐⭐⭐

**Authors:** Luis Miguel Pires and Ileida Veiga  
**Year:** 2025  
**Journal/Conference:** Designs (MDPI)  
**DOI:** 10.3390/designs9060144  
**Citations:** Recent (2025)  

**Relevance:** 
Provides the specific ESP32 hardware implementation details, including MEMS accelerometer integration for structural health monitoring of the station itself.

**Key Findings:**
- Extended the hardware discussion to the ESP32 specifically
- While application focused on rockfall monitoring, architectural findings are directly transferable
- Demonstrated that integrating MEMS accelerometer (ADXL345) allows the node to detect its own inclination
- In river context, this is a vital "health check" feature: determining if a flood has physically dislodged or tilted the mounting pole
- Power analysis confirmed that ESP32, when optimized, can operate for months on a 2600mAh battery
- Demonstrated multi-month battery life using ESP32 deep sleep

**BibTeX Entry:**
```bibtex
@article{pires_2025_esp32,
  author = {Pires, Luis Miguel and Veiga, Ileida},
  title = {ESP32-based Monitoring System with Structural Health Monitoring},
  journal = {Designs},
  year = {2025},
  doi = {10.3390/designs9060144},
  publisher = {MDPI}
}
```

---

### 6. Dragino (2024) - LiDAR Sensor Reference

**Authors:** Dragino Technology  
**Year:** 2024  
**Type:** Technical Documentation / Industry Whitepaper  
**Relevance:** Represents the cutting-edge "Commercial Off-The-Shelf" (COTS) solution, providing a benchmark against which custom ESP32 design can be compared.

**Key Findings:**
- Highlights emergence of low-cost LiDAR (e.g., LDS25-LS) tailored for LoRaWAN
- LDS25-LS: A turnkey LoRaWAN sensor that simplifies project to "integration and data analysis"
- Detailed specs on 850nm LiDAR performance in outdoor conditions
- Validates the need for auto-cleaning features in optical sensors for long-term deployment
- While LiDAR solves temperature drift issue, it introduces maintenance challenges: accumulation of dust, spider webs, or condensation on lens
- Necessitates auto-cleaning mechanisms or hydrophobic coatings
- Ultrasonic sensors (which are self-cleaning via vibration) effectively avoid these complexities

**Note:** This is a technical documentation, not a peer-reviewed paper, but provides valuable reference for LiDAR sensor specifications.

---

## Additional References Mentioned

### Papers Already in Your Bibliography

1. **Santana et al. (2024)** - "Development and Calibration of a Low-Cost LIDAR Sensor for Water Level Measurements"
   - Already in your `referencias.bib`
   - DOI: 10.1016/j.flowmeasinst.2024.102729

2. **Paul et al. (2020)** - "A Technical Evaluation of Lidar‐Based Measurement of River Water Levels"
   - Already in your `referencias.bib`
   - DOI: 10.1029/2019wr026810

## Key Technical Findings from Gemini Report

### Power Management Insights

**From Orlovs et al. (2025):**
- Energy consumption per LoRaWAN message: ~82.2 µWh
- Battery life calculation: 18650-based node (3000mAh) transmitting every 15 minutes at SF10 consumes approx 0.3 mAh/hour (averaged)
- Theoretical lifespan: ~10,000 hours (~14 months) with 3000mAh battery
- With 1W solar panel, battery life becomes indefinite

**From Pires and Veiga (2025):**
- ESP32 can operate for months on 2600mAh battery with proper optimization
- Deep sleep implementation validated

### Network Performance Insights

**From Orlovs et al. (2025):**
- LoRaWAN range: 11 km in rural settings, 3 km in urban
- Superior to Sigfox (10 km max) and NB-IoT (requires cellular towers)
- Link Budget calculation provided
- Fresnel zone considerations for river monitoring
- Collision issues beyond 1000 devices per gateway

### Sensor Performance Insights

**From Panagopoulos et al. (2021):**
- Ultrasonic sensors: 7% variance vs pressure transducers
- Temperature drift: 1-2 cm per 10°C change (uncompensated)
- With compensation: <0.5% error
- Diurnal fluctuations correlated with air temperature

**From Kabi et al. (2023):**
- Raw sensor data prone to noise from river turbulence and debris
- Machine learning/statistical filtering required
- Data post-processing mandatory for usable hydrological insights

### System Architecture Insights

**From Rahman and Ahmed (2020):**
- Dual-alerting mechanism: local buzzers + cloud SMS
- Cloud dashboards (ThingSpeak) critical for visualization
- ±2 cm accuracy achievable in controlled environments

**From Pires and Veiga (2025):**
- MEMS accelerometer integration for structural health monitoring
- Detects node inclination/tilting
- Critical for validating sensor readings

## Research Gaps Identified

1. **Deep Sleep Software Architecture:** 
   - Gap in detailed documentation of Deep Sleep software architectures specific to ESP32-LoRa context
   - How to handle "cold boot" initialization of LoRa radios without wasting energy

2. **Long-term Reliability:** 
   - Specific gaps in long-term reliability studies remain

3. **Sensor Selection:** 
   - Ongoing debate between ultrasonic and LiDAR
   - Trade-offs well documented but more comparative field studies needed

## Recommendations from Gemini Report

### Hardware Selection

**ESP32 Boards:**
- Heltec WiFi LoRa 32 (V3): ESP32-S3, SX1262, OLED display, Li-Ion battery management
- TTGO T-Beam: Includes GPS for mobile surveys or timestamp accuracy

**LoRa Transceiver:**
- SX1262 preferred over SX1276/78 for power efficiency
- Integrated DC-DC buck converter saves power
- Receive current: ~4.6 mA vs ~10 mA for SX1276

### Power Management Strategy

**Deep Sleep Implementation:**
1. RTC Memory: Variables must be stored in `RTC_DATA_ATTR` memory
2. Radio Management: LoRa radio must be explicitly put to sleep via SPI command
3. Sensor Power Gating: Use MOSFET or GPIO to toggle sensor power

**Adaptive Scheduling:**
- Wake every 5 minutes, take measurement
- Compare with last_water_level
- If stable: don't transmit, return to sleep
- If change OR 1 hour passed: transmit
- This "Report-by-Exception" strategy reduces transmissions by 90% during non-flood periods

### Network Configuration

**LoRaWAN Class:**
- Class A: Only viable option for battery-powered nodes
- Class C: Continuously listening, unsuitable for battery operation

**Activation:**
- OTAA (Over-The-Air Activation) preferred over ABP
- Frame Counters must be saved in non-volatile memory (NVS/EEPROM)

### Sensor Selection Recommendations

**Ultrasonic (JSN-SR04T):**
- Best for: Budget, wide rivers
- Range: 25cm - 450cm
- Requires: Temperature compensation, logic level shifter
- Maintenance: Low (self-cleaning via vibration)

**LiDAR (TF-Luna/LDS25-LS):**
- Best for: Precision applications on bridges
- Accuracy: ±1cm
- Requires: Signal strength monitoring, lens cleaning
- Maintenance: Medium (lens wiping needed)

## Next Steps

1. **Add Missing Papers to Bibliography:**
   - Kabi et al. (2023) - HardwareX
   - Rahman and Ahmed (2020) - ICREST
   - Orlovs et al. (2025) - IoT (MDPI)
   - Panagopoulos et al. (2021) - Sensors (MDPI)
   - Pires and Veiga (2025) - Designs (MDPI)

2. **Verify DOI and Citation Information:**
   - Check if all DOIs are correct
   - Verify citation counts
   - Ensure BibTeX entries are complete

3. **Update Thesis Chapters:**
   - Chapter 2: Add these papers to literature review
   - Chapter 4: Reference findings in methodology
   - Chapter 5: Compare results with these studies

4. **Review Technical Findings:**
   - Validate power consumption numbers
   - Review network performance metrics
   - Verify sensor validation results

## Summary

The Gemini search found **5 new peer-reviewed papers** that directly address critical gaps in your research:

✅ **Field Deployment:** Kabi et al. (2023) - 18-month deployment study  
✅ **System Architecture:** Rahman and Ahmed (2020) - Foundational design  
✅ **Network Performance:** Orlovs et al. (2025) - Empirical range and power data  
✅ **Sensor Validation:** Panagopoulos et al. (2021) - Ultrasonic validation  
✅ **ESP32 Implementation:** Pires and Veiga (2025) - Hardware details and power management  

These papers fill critical gaps identified in the search prompt, particularly:
- LoRaWAN river monitoring systems ✅
- ESP32 power management validation ✅
- Field deployment studies ✅
- Sensor validation and comparison ✅
