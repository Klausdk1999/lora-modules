# River Level Monitoring WSN - Architecture Diagrams

## 1. Heltec Module - Hardware Architecture

```mermaid
graph TB
    subgraph "Heltec WiFi LoRa 32 V2"
        subgraph "ESP32 MCU"
            CPU[ESP32 Dual Core<br/>240 MHz]
            FLASH[4MB Flash]
            RTC[RTC Memory<br/>Session Storage]
        end

        subgraph "LoRa Module"
            SX1276[SX1276<br/>LoRa Transceiver]
            ANT[915 MHz Antenna<br/>AU915 Band]
        end

        subgraph "Display"
            OLED[0.96" OLED<br/>SSD1306<br/>128x64px]
        end

        subgraph "Power Management"
            BATT[Battery ADC<br/>GPIO 37]
            USB[USB-C<br/>5V Input]
        end
    end

    subgraph "External Sensors"
        TF02[TF02-Pro LiDAR<br/>UART 115200<br/>Range: 0.1-22m]
        DHT[DHT11<br/>Temp/Humidity]
    end

    subgraph "Power Supply"
        EXTBAT[External 5V Battery<br/>for TF02-Pro]
        LIPO[3.7V LiPo<br/>for ESP32]
    end

    TF02 -->|TX→GPIO13<br/>RX→GPIO17| CPU
    DHT -->|DATA→GPIO25| CPU
    CPU <-->|SPI| SX1276
    SX1276 --- ANT
    CPU <-->|I2C| OLED
    BATT --> CPU
    EXTBAT -->|5V| TF02
    LIPO --> USB
    USB --> CPU
```

## 2. Heltec Module - Software Architecture

```mermaid
graph TB
    subgraph "Application Layer"
        MAIN[main.cpp<br/>Application Logic]
        SEND[do_send<br/>Transmission Handler]
        EVENT[onEvent<br/>LoRaWAN Events]
    end

    subgraph "Sensor Processing"
        READ[readLidarFiltered<br/>Sensor Reading]
        DHT_READ[readDHTSensor<br/>Temp/Humidity]
    end

    subgraph "Filtering Library"
        subgraph "lib/filters"
            MA[MovingAverage.h<br/>Statistical Smoothing]
            KF[KalmanFilter.h<br/>State Estimation]
            SF[SensorFilters.h<br/>Filter Chain]
            AS[AdaptiveSampling.h<br/>Dynamic Intervals]
        end
    end

    subgraph "Sensor Libraries"
        subgraph "lib/sensors"
            TF02LIB[TF02Pro.h<br/>LiDAR Driver]
            BASE[SensorBase.h<br/>Interface]
        end
    end

    subgraph "LoRaWAN Stack"
        LMIC[MCCI LMIC<br/>LoRaWAN MAC]
        HAL[ESP32 HAL<br/>Hardware Abstraction]
    end

    subgraph "Data Structures"
        PAYLOAD[SensorPayload<br/>16 bytes]
        SESSION[RTC Session<br/>Deep Sleep Persistence]
    end

    MAIN --> SEND
    MAIN --> READ
    MAIN --> DHT_READ
    READ --> SF
    SF --> MA
    SF --> KF
    SF --> AS
    READ --> TF02LIB
    TF02LIB --> BASE
    SEND --> LMIC
    EVENT --> LMIC
    LMIC --> HAL
    READ --> PAYLOAD
    MAIN --> SESSION
```

## 3. LilyGo T-Beam - Hardware Architecture

```mermaid
graph TB
    subgraph "LilyGo T-Beam V1.2"
        subgraph "ESP32 MCU"
            CPU2[ESP32 Dual Core<br/>240 MHz]
            FLASH2[4MB Flash]
            RTC2[RTC Memory<br/>Session Storage]
            NVS[NVS Storage<br/>Calibration Data]
        end

        subgraph "Power Management"
            AXP2101[AXP2101 PMIC<br/>I2C Interface]
            CHG[Battery Charger<br/>500mA]
            FUEL[Fuel Gauge<br/>Battery %]
        end

        subgraph "LoRa Module"
            SX1276_2[SX1276<br/>LoRa Transceiver]
            ANT2[915 MHz Antenna<br/>AU915 Band]
        end

        subgraph "GPS Module"
            GPS[NEO-6M GPS<br/>Optional]
        end
    end

    subgraph "External Sensors"
        TF02_2[TF02-Pro LiDAR<br/>UART 115200<br/>Range: 0.1-22m]
    end

    subgraph "Power Supply"
        BOOST[5V Boost Converter<br/>for TF02-Pro]
        IMR[18650 Battery<br/>IMR 3.7V 2000mAh]
        SOLAR[Solar Panel<br/>Optional 6W]
    end

    TF02_2 -->|TX→GPIO13<br/>RX→GPIO14| CPU2
    CPU2 <-->|I2C SDA:21<br/>SCL:22| AXP2101
    AXP2101 --> CHG
    AXP2101 --> FUEL
    CPU2 <-->|SPI| SX1276_2
    SX1276_2 --- ANT2
    CPU2 <-->|UART| GPS
    IMR --> AXP2101
    SOLAR -.->|Optional| AXP2101
    BOOST -->|5V| TF02_2
    AXP2101 -->|3.3V| CPU2
    AXP2101 -->|Enable| BOOST
```

## 4. LilyGo T-Beam - Software Architecture

```mermaid
graph TB
    subgraph "Application Layer"
        MAIN2[main.cpp<br/>Application Logic]
        SEND2[do_send<br/>Transmission Handler]
        EVENT2[onEvent<br/>LoRaWAN Events]
        SLEEP[enterDeepSleep<br/>Power Management]
    end

    subgraph "Sensor Processing"
        READ2[readSensorData<br/>With Filtering]
        TEMPCOMP[applyTemperatureCompensation<br/>Based on Mohammed 2019]
    end

    subgraph "Filtering Library"
        subgraph "lib/filters"
            MA2[MovingAverage.h]
            KF2[KalmanFilter.h]
            SF2[SensorFilters.h<br/>SensorFilterChain]
            AS2[AdaptiveSampling.h]
        end
    end

    subgraph "Analytics Library"
        subgraph "lib/analytics"
            ANOM[AnomalyDetection.h]
            STATS[StatisticalAnalysis.h]
            TCOMP[TemperatureCompensation.h]
        end
    end

    subgraph "Power Management"
        PMU[XPowersLib<br/>AXP2101 Driver]
        BATT2[getBatteryPercent<br/>getBatteryVoltage]
    end

    subgraph "LoRaWAN Stack"
        LMIC2[MCCI LMIC<br/>LoRaWAN MAC]
        HAL2[ESP32 HAL]
    end

    subgraph "Adaptive Behavior"
        ADAPT[calculateAdaptiveInterval]
        RIVER[calculateRiverLevel]
        HIST[updateHistoricalMinimum]
        CALIB[NVS Calibration<br/>Load/Save]
    end

    MAIN2 --> SEND2
    MAIN2 --> READ2
    MAIN2 --> SLEEP
    READ2 --> SF2
    SF2 --> MA2
    SF2 --> KF2
    READ2 --> TEMPCOMP
    TEMPCOMP --> TCOMP
    SEND2 --> LMIC2
    LMIC2 --> HAL2
    READ2 --> ADAPT
    ADAPT --> AS2
    READ2 --> RIVER
    RIVER --> HIST
    HIST --> CALIB
    PMU --> BATT2
    ADAPT --> BATT2
```

## 5. Full WSN Architecture

```mermaid
graph TB
    subgraph "Sensor Nodes"
        subgraph "Node 1: Heltec"
            H1[Heltec WiFi LoRa 32 V2<br/>TF02-Pro + DHT11]
        end
        subgraph "Node 2: LilyGo"
            L1[LilyGo T-Beam V1.2<br/>TF02-Pro LiDAR]
        end
        subgraph "Node N..."
            N1[Additional Nodes<br/>Ultrasonic/LiDAR]
        end
    end

    subgraph "LoRaWAN Gateway"
        GW[RAK7268<br/>8-Channel Gateway<br/>AU915]
        GWETH[Ethernet/WiFi<br/>Backhaul]
    end

    subgraph "Network Server"
        TTN[The Things Network<br/>V3 Stack]
        MQTT_TTN[MQTT Broker<br/>Integration]
        DECODER[Payload Decoder<br/>JavaScript]
    end

    subgraph "Backend Services"
        subgraph "Go API Server"
            API[go-data-storage<br/>REST API]
            MQTT_CLIENT[MQTT Client<br/>TTN Subscription]
            TTN_HANDLER[TTN Handler<br/>Webhook/MQTT]
        end

        subgraph "Database"
            SQLITE[(SQLite<br/>sensor_data.db)]
        end

        subgraph "ML Backend"
            LSTM[LSTM Forecaster<br/>Python/TensorFlow]
            ALERT[Alert System<br/>SMS/Email/Webhook]
        end
    end

    subgraph "Frontend"
        DASH[Web Dashboard<br/>Real-time Monitoring]
        MOBILE[Mobile App<br/>Optional]
    end

    H1 -->|LoRa<br/>SF7 AU915| GW
    L1 -->|LoRa<br/>SF7 AU915| GW
    N1 -.->|LoRa| GW
    GW -->|GWETH| TTN
    TTN --> MQTT_TTN
    TTN --> DECODER
    MQTT_TTN -->|Subscribe| MQTT_CLIENT
    MQTT_CLIENT --> TTN_HANDLER
    TTN_HANDLER --> API
    API <--> SQLITE
    API --> LSTM
    LSTM --> ALERT
    API --> DASH
    API -.-> MOBILE
```

## 6. Data Flow Timeline

```mermaid
sequenceDiagram
    autonumber
    participant S as Sensor<br/>(TF02-Pro)
    participant M as MCU<br/>(ESP32)
    participant F as Filters<br/>(MA+Kalman)
    participant L as LoRa<br/>(SX1276)
    participant G as Gateway<br/>(RAK7268)
    participant T as TTN<br/>(Network Server)
    participant A as API<br/>(Go Backend)
    participant D as Database<br/>(SQLite)
    participant ML as ML<br/>(LSTM)
    participant AL as Alerts<br/>(SMS/Email)
    participant UI as Dashboard<br/>(Web UI)

    Note over S,M: Sensor Reading Phase
    S->>M: Raw Distance (UART)<br/>~100ms @ 10Hz
    M->>M: Temperature Compensation<br/>v(T) = 331.5 + 0.607*T

    Note over M,F: Filtering Phase
    M->>F: Raw Value
    F->>F: Moving Average<br/>(5-sample window)
    F->>F: Kalman Filter<br/>(State estimation)
    F->>M: Filtered Values<br/>(Raw, MA, Kalman)

    Note over M,M: Adaptive Sampling
    M->>M: Calculate Rate of Change
    M->>M: Determine Next Interval<br/>(30s-5min adaptive)

    Note over M,L: Payload Construction
    M->>M: Build 16-byte Payload<br/>{raw, ma, kalman, river_level, flags}
    M->>L: Queue Transmission

    Note over L,G: LoRaWAN Transmission
    L->>G: Uplink SF7 AU915<br/>~100ms airtime
    G->>G: Receive & Forward

    Note over G,T: Network Processing
    G->>T: MQTT/HTTPS Uplink
    T->>T: Payload Decoding<br/>(JavaScript decoder)
    T->>T: Device Session Management

    Note over T,A: Backend Integration
    T->>A: MQTT Message<br/>or Webhook POST
    A->>A: Parse & Validate
    A->>D: Store Reading<br/>(INSERT INTO readings)

    Note over A,ML: ML Processing
    A->>ML: Trigger Prediction<br/>(if enough data)
    ML->>ML: LSTM Forecast<br/>(6-hour prediction)
    ML->>A: Prediction Result

    Note over ML,AL: Alert Evaluation
    ML->>AL: Check Thresholds
    alt Critical Level Detected
        AL->>AL: SMS via Twilio
        AL->>AL: Email via SMTP
        AL->>AL: Webhook POST
    end

    Note over A,UI: Dashboard Update
    A->>UI: REST API Response<br/>/api/readings
    UI->>UI: Real-time Chart Update
    UI->>UI: Map Visualization

    Note over M,M: Power Management
    M->>M: Save Session to RTC
    M->>M: Enter Deep Sleep<br/>(adaptive interval)
    M->>M: Wake on Timer
```

## 7. Payload Structure

```mermaid
graph LR
    subgraph "LoRaWAN Payload (16 bytes)"
        B0[Byte 0<br/>sensorType<br/>1=LiDAR]
        B1[Bytes 1-2<br/>rawDistanceMm<br/>uint16]
        B2[Bytes 3-4<br/>maDistanceMm<br/>uint16]
        B3[Bytes 5-6<br/>kalmanDistanceMm<br/>uint16]
        B4[Bytes 7-8<br/>riverLevelMm<br/>uint16]
        B5[Bytes 9-10<br/>signalStrength<br/>int16]
        B6[Byte 11<br/>temperature<br/>int8]
        B7[Byte 12<br/>batteryPercent<br/>uint8]
        B8[Byte 13<br/>flags<br/>uint8]
    end

    subgraph "Flags Bitmap"
        F0[Bit 0: RAPID_CHANGE]
        F1[Bit 1: CRITICAL_LEVEL]
        F2[Bit 2: BATTERY_LOW]
        F3[Bit 3: SENSOR_ERROR]
        F4[Bit 4: KALMAN_INIT]
    end

    B8 --> F0
    B8 --> F1
    B8 --> F2
    B8 --> F3
    B8 --> F4
```

## 8. Filter Chain Processing

```mermaid
flowchart LR
    subgraph Input
        RAW[Raw Sensor<br/>Reading]
    end

    subgraph "Moving Average Filter"
        MA_BUF[Circular Buffer<br/>5 samples]
        MA_CALC[Sum / Count]
        MA_OUT[MA Value]
    end

    subgraph "Kalman Filter"
        PRED[Predict Step<br/>P = P + Q]
        UPD[Update Step<br/>K = P/(P+R)]
        EST[State Estimate<br/>x = x + K*innovation]
        KF_OUT[Kalman Value<br/>+ Uncertainty]
    end

    subgraph "Adaptive Sampler"
        RATE[Rate of Change<br/>cm/min]
        THRESH[Threshold Check<br/>5cm = rapid]
        INT[Interval Selection<br/>30s-5min]
    end

    subgraph Output
        RESULT[FilteredReading<br/>raw, ma, kalman<br/>uncertainty, interval]
    end

    RAW --> MA_BUF
    MA_BUF --> MA_CALC
    MA_CALC --> MA_OUT

    RAW --> PRED
    PRED --> UPD
    UPD --> EST
    EST --> KF_OUT

    KF_OUT --> RATE
    RATE --> THRESH
    THRESH --> INT

    MA_OUT --> RESULT
    KF_OUT --> RESULT
    INT --> RESULT
```

## 9. Deep Sleep Power Cycle

```mermaid
stateDiagram-v2
    [*] --> WAKE: Timer/Cold Boot

    WAKE --> INIT: Initialize Hardware

    INIT --> RESTORE: Check RTC Memory

    RESTORE --> JOINED: Valid Session?
    RESTORE --> JOINING: No Session

    JOINING --> JOINED: EV_JOINED
    JOINING --> JOINING: EV_JOIN_FAILED<br/>(retry)

    JOINED --> READING: Read Sensors

    READING --> FILTERING: Apply Filters

    FILTERING --> ADAPTIVE: Calculate Interval

    ADAPTIVE --> SENDING: Queue Packet

    SENDING --> TX_COMPLETE: EV_TXCOMPLETE

    TX_COMPLETE --> SAVE: Save Session<br/>to RTC

    SAVE --> SLEEP: Enter Deep Sleep

    SLEEP --> WAKE: Timer Wakeup

    note right of SLEEP
        Power: ~10µA
        Duration: 30s-5min
        (adaptive)
    end note

    note right of SENDING
        Power: ~120mA
        Duration: ~100ms
    end note
```

## 10. System Deployment

```mermaid
graph TB
    subgraph "River Monitoring Site"
        subgraph "Bridge Installation"
            NODE1[Sensor Node<br/>LilyGo T-Beam]
            MOUNT[Mounting Bracket<br/>IP67 Enclosure]
            PANEL[Solar Panel<br/>6W Optional]
        end

        subgraph "River"
            WATER[Water Surface]
            FLOW[Flow Direction]
        end
    end

    subgraph "Gateway Site (1-15km)"
        GW_SITE[LoRaWAN Gateway<br/>RAK7268]
        INET[Internet<br/>Fiber/4G]
    end

    subgraph "Cloud Infrastructure"
        TTN_CLOUD[TTN Community<br/>or Private Stack]
        VPS[VPS Server<br/>Go API + SQLite]
        WEB[Web Dashboard<br/>HTTPS]
    end

    subgraph "Alert Recipients"
        CIVIL[Civil Defense<br/>SMS Alerts]
        ADMIN[System Admin<br/>Email Alerts]
        PUBLIC[Public Dashboard<br/>Read-only]
    end

    NODE1 -->|LiDAR Beam| WATER
    PANEL -.-> NODE1
    NODE1 --> MOUNT
    NODE1 -->|LoRa 915MHz<br/>up to 15km| GW_SITE
    GW_SITE --> INET
    INET --> TTN_CLOUD
    TTN_CLOUD --> VPS
    VPS --> WEB
    VPS --> CIVIL
    VPS --> ADMIN
    WEB --> PUBLIC
```
