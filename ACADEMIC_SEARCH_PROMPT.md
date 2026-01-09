# Academic Publication Search Prompt

## Instructions for AI Agent

You are tasked with searching for academic publications related to a Master's thesis project on **LoRaWAN-based wireless sensor networks for river level monitoring**. The project uses ESP32 microcontrollers with LoRa modules, distance sensors (LiDAR and ultrasonic), and implements deep sleep power management.

## Search Requirements

### Minimum Publication Standards

1. **Publication Type:**
   - Peer-reviewed journal articles (preferred)
   - Conference papers from reputable venues (IEEE, ACM, Springer, Elsevier)
   - Technical reports from recognized institutions
   - **EXCLUDE:** Blog posts, forum discussions, non-peer-reviewed content, manufacturer datasheets (unless specifically requested)

2. **Publication Date:**
   - **Preferred:** 2018-2025 (recent research)
   - **Acceptable:** 2015-2017 (if highly relevant)
   - **EXCLUDE:** Papers older than 2015 unless they are foundational/cited works

3. **Citation Requirements:**
   - Papers must have at least 5 citations (use Google Scholar or similar)
   - Exception: Very recent papers (2024-2025) may have fewer citations
   - Prefer papers with 10+ citations when available

4. **Journal/Conference Quality:**
   - IEEE journals and conferences
   - ACM publications
   - Springer journals
   - Elsevier journals
   - MDPI journals (peer-reviewed)
   - Reputable conference proceedings (IEEE, ACM, etc.)

5. **Language:**
   - English (primary)
   - Portuguese (acceptable if highly relevant to Brazilian context)
   - Other languages only if no English equivalent exists

## Research Areas to Search

### Priority 1: Critical Gaps (High Priority)

#### 1.1 LoRaWAN River/Water Level Monitoring Systems
**Search Terms:**
- "LoRaWAN" AND "river level monitoring"
- "LoRaWAN" AND "water level" AND "sensor"
- "LoRa" AND "hydrological monitoring"
- "LoRaWAN" AND "flood monitoring"
- "LPWAN" AND "water level measurement"
- "LoRa" AND "environmental monitoring" AND "water"

**What We're Missing:**
- Complete system implementations using LoRaWAN for river monitoring
- Field deployment case studies
- Network performance evaluation
- Comparison with other communication technologies
- Real-world validation results

**Expected Information:**
- System architecture
- Sensor integration methods
- Data transmission strategies
- Field test results
- Battery life measurements
- Network coverage analysis

---

#### 1.2 ESP32 + LoRa Deep Sleep Power Management Validation
**Search Terms:**
- "ESP32" AND "LoRa" AND "deep sleep"
- "ESP32" AND "LoRaWAN" AND "power consumption"
- "ESP32" AND "LoRa" AND "battery life"
- "ESP32" AND "LoRa" AND "power management"
- "ESP32" AND "LoRa module" AND "energy efficiency"
- "Heltec LoRa32" OR "LilyGo LoRa32" AND "power"

**What We're Missing:**
- Specific validation studies of ESP32 deep sleep with LoRa modules
- Measured power consumption data
- Battery life validation in field conditions
- Wake-up time measurements
- Power consumption breakdown (sleep vs active vs transmission)

**Expected Information:**
- Current consumption measurements (µA in sleep, mA in active)
- Battery life calculations and validations
- Wake-up time measurements
- Power optimization techniques
- Comparison of different sleep modes

---

#### 1.3 LoRaWAN Power Consumption and Battery Life Studies
**Search Terms:**
- "LoRaWAN" AND "power consumption" AND "measurement"
- "LoRaWAN" AND "battery lifetime" AND "validation"
- "LoRaWAN" AND "energy consumption" AND "sensor node"
- "LoRaWAN" AND "power analysis" AND "field test"
- "LoRaWAN" AND "battery life" AND "IoT"
- "LoRa" AND "power consumption" AND "empirical"

**What We're Missing:**
- Empirical power consumption measurements
- Battery life validation studies
- Power consumption models
- Field deployment power measurements
- Comparison of different LoRaWAN configurations

**Expected Information:**
- Measured current consumption in different states
- Battery life calculations
- Power consumption per transmission
- Duty cycle optimization
- Energy harvesting integration

---

### Priority 2: Important Research Areas (Medium Priority)

#### 2.1 LoRaWAN Network Performance for Environmental Monitoring
**Search Terms:**
- "LoRaWAN" AND "environmental monitoring" AND "performance"
- "LoRaWAN" AND "outdoor deployment" AND "range"
- "LoRaWAN" AND "rural" AND "coverage"
- "LoRaWAN" AND "field deployment" AND "evaluation"
- "LoRaWAN" AND "link quality" AND "environmental"

**What We're Missing:**
- Network performance in outdoor/rural environments
- Range and coverage analysis
- Link quality in different conditions
- Gateway placement strategies
- Network reliability studies

---

#### 2.2 Wireless Sensor Networks for Water Monitoring
**Search Terms:**
- "WSN" AND "water level monitoring"
- "wireless sensor network" AND "river monitoring"
- "IoT" AND "water level" AND "sensor network"
- "distributed sensing" AND "water monitoring"
- "sensor network" AND "hydrological monitoring"

**What We're Missing:**
- WSN architectures for water monitoring
- Multi-sensor integration
- Data aggregation strategies
- Network topology for water monitoring
- Comparison of different WSN technologies

---

#### 2.3 LoRaWAN vs Other LPWAN Technologies
**Search Terms:**
- "LoRaWAN" AND "NB-IoT" AND "comparison"
- "LoRaWAN" AND "Sigfox" AND "comparison"
- "LPWAN" AND "comparison" AND "environmental monitoring"
- "LoRaWAN" AND "alternative" AND "water monitoring"
- "LPWAN technologies" AND "evaluation"

**What We're Missing:**
- Comparative studies of LPWAN technologies
- Trade-offs for water monitoring applications
- Cost-benefit analysis
- Performance comparisons

---

### Priority 3: Supporting Research (Lower Priority)

#### 3.1 ESP32 Low-Power Modes Validation
**Search Terms:**
- "ESP32" AND "low power" AND "validation"
- "ESP32" AND "sleep modes" AND "measurement"
- "ESP32" AND "power consumption" AND "empirical"
- "ESP32" AND "battery" AND "life cycle"

**What We're Missing:**
- Independent validation of ESP32 power specifications
- Real-world power consumption measurements
- Battery life validation studies

---

#### 3.2 Sensor Integration with LoRaWAN
**Search Terms:**
- "ultrasonic sensor" AND "LoRaWAN"
- "LiDAR sensor" AND "LoRaWAN"
- "distance sensor" AND "LoRaWAN" AND "integration"
- "sensor" AND "LoRaWAN" AND "power consumption"

**What We're Missing:**
- Specific sensor integration methods with LoRaWAN
- Power consumption of sensor + LoRaWAN systems
- Data formatting for LoRaWAN payloads

---

## Search Strategy

### Database Priority
1. **IEEE Xplore** - Primary source for technical papers
2. **ACM Digital Library** - Computer science and networking
3. **Google Scholar** - Comprehensive search
4. **ScienceDirect (Elsevier)** - Engineering and environmental journals
5. **SpringerLink** - Engineering and computer science
6. **MDPI** - Open access journals (ensure peer-reviewed)
7. **ResearchGate** - For recent preprints and conference papers

### Search Query Format

For each research area, use multiple search queries with variations:

**Example for LoRaWAN River Monitoring:**
```
Query 1: "LoRaWAN" AND "river level monitoring" AND ("sensor" OR "IoT")
Query 2: "LoRa" AND "water level" AND "wireless sensor network"
Query 3: "LPWAN" AND "hydrological monitoring" AND "field deployment"
Query 4: "LoRaWAN" AND "flood" AND "monitoring system"
```

### Filtering Criteria

For each result, check:
1. ✅ Publication date (2015-2025, prefer 2018+)
2. ✅ Publication type (peer-reviewed journal/conference)
3. ✅ Citation count (5+ citations, or recent 2024-2025)
4. ✅ Relevance to search area
5. ✅ Full text availability (abstract minimum, full text preferred)

## Output Format

For each relevant paper found, provide:

```markdown
### [Paper Title]

**Authors:** [Author names]
**Year:** [Publication year]
**Journal/Conference:** [Full name]
**DOI:** [DOI if available]
**Citations:** [Number of citations if available]

**Relevance:** [Why this paper is relevant - 1-2 sentences]

**Key Findings:**
- [Finding 1]
- [Finding 2]
- [Finding 3]

**Methodology/Approach:**
[Brief description of methods used - 2-3 sentences]

**BibTeX Entry:**
```bibtex
@article{key,
  author = {},
  title = {},
  journal = {},
  year = {},
  doi = {}
}
```

**Full Citation:**
[Full citation in academic format]
```

## What We Already Have (Do Not Duplicate)

### LoRaWAN General
- Sun et al. (2022) - "Recent Advances in LoRa: A Comprehensive Survey"

### LoRaWAN Water Monitoring
- Dao et al. (2025) - "Low-cost, High Accuracy, and Long Communication Range Energy-Harvesting Beat Sensor with LoRa and Ω-Antenna for Water-Level Monitoring"
- Ishibashi et al. (2017, 2019) - Beat sensors with LoRa

### Power Management
- ESP32 Datasheet (2023) - Power specifications
- Ferreira et al. (2023) - WSN power management

### Sensor Methods
- Pereira et al. (2022) - Ultrasonic sensor evaluation
- Mohammadreza MasoudiMoghaddam et al. (2024) - Ultrasonic sensor validation
- Paul et al. (2020) - LiDAR temperature compensation
- Santana et al. (2024) - LiDAR calibration

**Focus on finding papers NOT already in our bibliography!**

## Special Considerations

### Brazilian Context (Optional)
If you find papers related to:
- River monitoring in Brazil
- Flood monitoring in South America
- Environmental monitoring in tropical climates

These are highly valuable even if they don't use LoRaWAN.

### Field Deployment Studies
Prioritize papers that include:
- Real field deployments
- Measured results (not just simulations)
- Long-term operation data
- Failure analysis
- Environmental challenges

### Comparative Studies
Highly value papers that:
- Compare multiple technologies
- Provide quantitative comparisons
- Include cost analysis
- Discuss trade-offs

## Expected Deliverables

1. **List of Found Papers** - Organized by research area (Priority 1, 2, 3)
2. **BibTeX Entries** - Ready to add to bibliography
3. **Relevance Summary** - Why each paper is important
4. **Key Findings Extraction** - Main contributions of each paper
5. **Gap Analysis** - What areas still need more research

## Quality Checklist

Before including a paper, verify:
- [ ] Published in peer-reviewed venue
- [ ] Published 2015-2025 (prefer 2018+)
- [ ] Has 5+ citations OR is very recent (2024-2025)
- [ ] Directly relevant to one of the research areas
- [ ] Not already in our bibliography
- [ ] Full citation information available
- [ ] DOI or URL available

## Notes

- **Be thorough but selective** - Quality over quantity
- **Prioritize empirical studies** over theoretical papers
- **Field deployment papers** are more valuable than simulations
- **Recent papers (2023-2025)** are preferred
- **If a paper is highly relevant but doesn't meet all criteria, include it with a note explaining why**

---

## Example Search Queries to Execute

Execute these searches in order of priority:

### Priority 1 Searches:
1. `"LoRaWAN" AND "river level monitoring" AND ("sensor" OR "IoT" OR "wireless")`
2. `"LoRaWAN" AND "water level" AND "sensor" AND ("field" OR "deployment")`
3. `"ESP32" AND "LoRa" AND "deep sleep" AND ("power" OR "battery")`
4. `"ESP32" AND "LoRaWAN" AND "power consumption" AND "measurement"`
5. `"LoRaWAN" AND "battery lifetime" AND ("validation" OR "field test")`
6. `"LoRaWAN" AND "power consumption" AND "sensor node" AND "empirical"`

### Priority 2 Searches:
7. `"LoRaWAN" AND "environmental monitoring" AND ("performance" OR "evaluation")`
8. `"LoRaWAN" AND "outdoor" AND ("range" OR "coverage" OR "link quality")`
9. `"wireless sensor network" AND "water monitoring" AND ("river" OR "hydrological")`
10. `"LPWAN" AND "comparison" AND ("LoRaWAN" OR "NB-IoT" OR "Sigfox")`

### Priority 3 Searches:
11. `"ESP32" AND "low power" AND ("validation" OR "measurement" OR "empirical")`
12. `"ultrasonic sensor" AND "LoRaWAN" AND "integration"`
13. `"LiDAR sensor" AND "LoRaWAN" OR "wireless sensor network"`

---

**Start searching and provide results organized by priority level!**
