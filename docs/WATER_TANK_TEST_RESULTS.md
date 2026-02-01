# Water Tank Sensor Comparison Test Results

## Test Setup

**Date:** February 2026
**Location:** Controlled water tank environment
**Tank:** 1-meter diameter circular water tank
**Measurement Method:** Tape measure reference (note: small measurement error of a few centimeters possible)

### Sensors Tested
| Sensor | Type | Interface | Board |
|--------|------|-----------|-------|
| TF02-Pro | LiDAR (ToF) | UART | Heltec LoRa32 V2 |
| TF-Nova | LiDAR (ToF) | UART | LilyGo T-Beam V1.2 |
| JSN-SR04T | Ultrasonic | GPIO | LilyGo T-Beam V1.2 |

### Test Conditions
- **Clear Water:** Tap water with normal visibility
- **Muddy Water:** Water with suspended sediment to simulate river conditions
- **Distances:** 1m, 2m, 3m from sensor to water surface

---

## Raw Data (all values in mm)

### Clear Water

#### 1 Meter Reference
| Sensor | Reading 1 | Reading 2 | Reading 3 | Reading 4 | Reading 5 | Reading 6 |
|--------|-----------|-----------|-----------|-----------|-----------|-----------|
| TF02-Pro | 1070 | 1060 | 1080 | 1060 | 1090 | - |
| TF-Nova | 1020 | 1010 | 980 | 990 | 1000 | 990 |
| JSN-SR04T | 986 | 984 | 985 | 972 | 983 | - |

#### 2 Meters Reference
| Sensor | Reading 1 | Reading 2 | Reading 3 | Reading 4 | Reading 5 |
|--------|-----------|-----------|-----------|-----------|-----------|
| TF02-Pro | 2120 | 2090 | 2020 | 2030 | - |
| TF-Nova | 1890 | 1920 | 1930 | 1870 | - |
| JSN-SR04T | 1924 | 1934 | 1927 | 1954 | - |

#### 3 Meters Reference
| Sensor | Reading 1 | Reading 2 | Reading 3 | Reading 4 | Notes |
|--------|-----------|-----------|-----------|-----------|-------|
| TF02-Pro | 2960 | 2850 | 2940 | 2950 | - |
| TF-Nova | 2880 | 3220 | 2670 | - | Reading 2 & 3: possible interference from tank edge |
| JSN-SR04T | 2874 | 2974 | 2967 | 2812 | - |

---

### Muddy Water

#### 1 Meter Reference
| Sensor | Reading 1 | Reading 2 | Reading 3 | Reading 4 | Reading 5 |
|--------|-----------|-----------|-----------|-----------|-----------|
| TF02-Pro | 1020 | 1030 | 1060 | 970 | - |
| TF-Nova | 1020 | 1070 | 990 | 980 | 1060 |
| JSN-SR04T | 962 | 967 | 989 | 966 | - |

#### 2 Meters Reference
| Sensor | Reading 1 | Reading 2 | Reading 3 | Reading 4 | Reading 5 |
|--------|-----------|-----------|-----------|-----------|-----------|
| TF02-Pro | 2060 | 1950 | 1960 | 2050 | - |
| TF-Nova | 1930 | 1980 | 2020 | 2120 | 2000 |
| JSN-SR04T | 1976 | 1991 | 1993 | 1937 | - |

#### 3 Meters Reference
| Sensor | Reading 1 | Reading 2 | Reading 3 | Reading 4 | Reading 5 |
|--------|-----------|-----------|-----------|-----------|-----------|
| TF02-Pro | 3010 | 2970 | 2970 | 2960 | - |
| TF-Nova | 3170 | 2930 | 2890 | 2880 | 2930 |
| JSN-SR04T | 2935 | 2864 | 2958 | 2953 | - |

---

## Statistical Analysis

### Clear Water Results

| Distance | Sensor | Mean (mm) | Std Dev (mm) | Error from Ref (mm) | Error (%) |
|----------|--------|-----------|--------------|---------------------|-----------|
| 1m | TF02-Pro | 1072.0 | 13.0 | +72.0 | +7.2% |
| 1m | TF-Nova | 998.3 | 14.7 | -1.7 | -0.2% |
| 1m | JSN-SR04T | 982.0 | 5.7 | -18.0 | -1.8% |
| 2m | TF02-Pro | 2065.0 | 47.3 | +65.0 | +3.3% |
| 2m | TF-Nova | 1902.5 | 26.3 | -97.5 | -4.9% |
| 2m | JSN-SR04T | 1934.8 | 13.1 | -65.3 | -3.3% |
| 3m | TF02-Pro | 2925.0 | 50.0 | -75.0 | -2.5% |
| 3m | TF-Nova* | 2923.3 | 275.4 | -76.7 | -2.6% |
| 3m | JSN-SR04T | 2906.8 | 71.8 | -93.3 | -3.1% |

*TF-Nova 3m includes interference readings; excluding outlier (3220): mean=2775, std=148.5

### Muddy Water Results

| Distance | Sensor | Mean (mm) | Std Dev (mm) | Error from Ref (mm) | Error (%) |
|----------|--------|-----------|--------------|---------------------|-----------|
| 1m | TF02-Pro | 1020.0 | 37.4 | +20.0 | +2.0% |
| 1m | TF-Nova | 1024.0 | 40.9 | +24.0 | +2.4% |
| 1m | JSN-SR04T | 971.0 | 12.5 | -29.0 | -2.9% |
| 2m | TF02-Pro | 2005.0 | 55.0 | +5.0 | +0.3% |
| 2m | TF-Nova | 2010.0 | 70.7 | +10.0 | +0.5% |
| 2m | JSN-SR04T | 1974.3 | 25.4 | -25.8 | -1.3% |
| 3m | TF02-Pro | 2977.5 | 22.2 | -22.5 | -0.8% |
| 3m | TF-Nova | 2960.0 | 113.1 | -40.0 | -1.3% |
| 3m | JSN-SR04T | 2927.5 | 43.8 | -72.5 | -2.4% |

---

## Key Findings

### 1. Sensor Consistency (Standard Deviation)

**Most Consistent Sensor: JSN-SR04T (Ultrasonic)**
- Clear water average std dev: 30.2 mm
- Muddy water average std dev: 27.2 mm
- The ultrasonic sensor shows the lowest variation across all tests

**LiDAR Sensors (TF02-Pro & TF-Nova)**
- Higher variation at longer distances
- TF-Nova showed susceptibility to edge interference at 3m in the narrow tank
- TF02-Pro: Clear water std dev 36.8 mm avg, Muddy water 38.2 mm avg

### 2. Clear vs Muddy Water Comparison

| Sensor | Clear Water Avg Std Dev | Muddy Water Avg Std Dev | Change |
|--------|------------------------|------------------------|--------|
| TF02-Pro | 36.8 mm | 38.2 mm | +3.8% (slightly worse) |
| TF-Nova | 105.5 mm* | 74.9 mm | -29.0% (better in muddy) |
| JSN-SR04T | 30.2 mm | 27.2 mm | -9.9% (slightly better) |

*TF-Nova clear water affected by 3m interference readings

**Observation:** Muddy water did not significantly degrade sensor performance. In fact, for ultrasonic sensors, the suspended particles may provide better sound wave reflection, resulting in slightly more consistent readings.

### 3. Accuracy (Error from Reference)

**Best Absolute Accuracy:**
- At 1m: TF-Nova (-0.2% error in clear water)
- At 2m: TF02-Pro (+0.3% error in muddy water)
- At 3m: TF02-Pro (-0.8% error in muddy water)

**Systematic Bias:**
- TF02-Pro tends to read slightly long (+2-7% at 1m)
- JSN-SR04T tends to read slightly short (-1.8% to -3.3%)
- TF-Nova shows variable bias depending on conditions

### 4. Tank Edge Interference (TF-Nova at 3m)

The TF-Nova LiDAR showed significant interference at 3m distance in the 1-meter diameter tank:
- Reading of 3220 mm (7.3% error) - likely detecting tank edge
- Reading of 2670 mm (11% error) - beam partially hitting edge

**Recommendation:** For narrow installations, TF02-Pro may be more suitable due to its narrower beam angle.

---

## Conclusions

1. **For River Monitoring Applications:**
   - **JSN-SR04T** offers the best consistency (lowest std dev) and is waterproof
   - **TF02-Pro** offers good accuracy with moderate consistency
   - **TF-Nova** is accurate but more susceptible to environmental interference

2. **Muddy Water Performance:**
   - All sensors performed comparably in muddy vs clear water
   - No significant accuracy degradation observed
   - Ultrasonic sensors may even benefit from suspended particles

3. **Measurement Uncertainty:**
   - Reference measurements via tape measure introduce ~20-30mm uncertainty
   - Sensor mounting angle and position contribute to systematic errors
   - For comparative studies, relative accuracy is more meaningful than absolute

4. **Recommendations:**
   - Use averaging (5+ readings) for production deployments
   - Consider sensor fusion (multiple sensor types) for critical applications
   - TF02-Pro best for narrow beam applications
   - JSN-SR04T best for consistent readings in variable conditions

---

## Test Notes

- All readings taken in single-reading mode (no averaging)
- 10-second intervals between transmissions
- Sensors pointed straight down at water surface
- Tank diameter (1m) may cause edge reflections at longer distances for LiDAR
- Temperature: ambient (~25°C)
