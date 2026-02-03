# River Level Monitoring WSN - Architecture Diagrams

## 1. Heltec Module - Hardware Architecture

```mermaid
graph TB
    subgraph HeltecBoard[Heltec WiFi LoRa 32 V2]
        subgraph MCU[ESP32 MCU]
            CPU[ESP32 Dual Core 240MHz]
            FLASH[4MB Flash]
            RTC[RTC Memory]
        end

        subgraph LoRaMod[LoRa Module]
            SX1276[SX1276 Transceiver]
            ANT[915MHz Antenna AU915]
        end

        subgraph Display[Display]
            OLED[OLED SSD1306 128x64]
        end

        subgraph PowerMgmt[Power Management]
            BATT[Battery ADC GPIO37]
            USB[USB-C 5V Input]
        end
    end

    subgraph ExtSensors[External Sensors]
        TF02[TF02-Pro LiDAR UART]
        DHT[DHT11 Temp Humidity]
    end

    subgraph PowerSupply[Power Supply]
        EXTBAT[External 5V Battery]
        LIPO[3.7V LiPo Battery]
    end

    TF02 -->|UART TX-GPIO13 RX-GPIO17| CPU
    DHT -->|DATA GPIO25| CPU
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
    subgraph AppLayer[Application Layer]
        MAIN[main.cpp Application Logic]
        SEND[do_send Transmission Handler]
        EVENT[onEvent LoRaWAN Events]
    end

    subgraph SensorProc[Sensor Processing]
        READ[readLidarFiltered]
        DHT_READ[readDHTSensor]
    end

    subgraph FilterLib[lib/filters]
        MA[MovingAverage.h]
        KF[KalmanFilter.h]
        SF[SensorFilters.h]
        AS[AdaptiveSampling.h]
    end

    subgraph SensorLib[lib/sensors]
        TF02LIB[TF02Pro.h LiDAR Driver]
        BASE[SensorBase.h Interface]
    end

    subgraph LoRaStack[LoRaWAN Stack]
        LMIC[MCCI LMIC LoRaWAN MAC]
        HAL[ESP32 HAL]
    end

    subgraph DataStruct[Data Structures]
        PAYLOAD[SensorPayload 16 bytes]
        SESSION[RTC Session Storage]
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
    subgraph LilyGoBoard[LilyGo T-Beam V1.2]
        subgraph MCU2[ESP32 MCU]
            CPU2[ESP32 Dual Core 240MHz]
            FLASH2[4MB Flash]
            RTC2[RTC Memory]
            NVS[NVS Calibration Storage]
        end

        subgraph PowerMgmt2[Power Management]
            AXP2101[AXP2101 PMIC I2C]
            CHG[Battery Charger 500mA]
            FUEL[Fuel Gauge]
        end

        subgraph LoRaMod2[LoRa Module]
            SX1276_2[SX1276 Transceiver]
            ANT2[915MHz Antenna AU915]
        end

        subgraph GPSMod[GPS Module]
            GPS[NEO-6M GPS Optional]
        end
    end

    subgraph ExtSensors2[External Sensors]
        TF02_2[TF02-Pro LiDAR UART 115200]
    end

    subgraph PowerSupply2[Power Supply]
        BOOST[5V Boost Converter]
        IMR[18650 Battery 3.7V 2000mAh]
        SOLAR[Solar Panel 6W Optional]
    end

    TF02_2 -->|UART TX-GPIO13 RX-GPIO14| CPU2
    CPU2 <-->|I2C SDA21 SCL22| AXP2101
    AXP2101 --> CHG
    AXP2101 --> FUEL
    CPU2 <-->|SPI| SX1276_2
    SX1276_2 --- ANT2
    CPU2 <-->|UART| GPS
    IMR --> AXP2101
    SOLAR -.-> AXP2101
    BOOST -->|5V| TF02_2
    AXP2101 -->|3.3V| CPU2
    AXP2101 --> BOOST
```

## 4. LilyGo T-Beam - Software Architecture

```mermaid
graph TB
    subgraph AppLayer2[Application Layer]
        MAIN2[main.cpp Application Logic]
        SEND2[do_send Transmission]
        EVENT2[onEvent LoRaWAN Events]
        SLEEP[enterDeepSleep]
    end

    subgraph SensorProc2[Sensor Processing]
        READ2[readSensorData With Filtering]
        TEMPCOMP[applyTemperatureCompensation]
    end

    subgraph FilterLib2[lib/filters]
        MA2[MovingAverage.h]
        KF2[KalmanFilter.h]
        SF2[SensorFilters.h]
        AS2[AdaptiveSampling.h]
    end

    subgraph AnalyticsLib[lib/analytics]
        ANOM[AnomalyDetection.h]
        STATS[StatisticalAnalysis.h]
        TCOMP[TemperatureCompensation.h]
    end

    subgraph PowerMgmt3[Power Management]
        PMU[XPowersLib AXP2101]
        BATT2[getBatteryPercent]
    end

    subgraph LoRaStack2[LoRaWAN Stack]
        LMIC2[MCCI LMIC]
        HAL2[ESP32 HAL]
    end

    subgraph AdaptBehavior[Adaptive Behavior]
        ADAPT[calculateAdaptiveInterval]
        RIVER[calculateRiverLevel]
        HIST[updateHistoricalMinimum]
        CALIB[NVS Calibration]
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
    subgraph SensorNodes[Sensor Nodes]
        subgraph Node1[Node 1 Heltec]
            H1[Heltec WiFi LoRa 32 V2]
        end
        subgraph Node2[Node 2 LilyGo]
            L1[LilyGo T-Beam V1.2]
        end
        subgraph NodeN[Additional Nodes]
            N1[Ultrasonic or LiDAR]
        end
    end

    subgraph GatewayInfra[LoRaWAN Gateway]
        GW[RAK7268 8-Channel AU915]
        BACKHAUL[Internet Backhaul]
    end

    subgraph NetworkServer[Network Server]
        TTN[The Things Network V3]
        MQTT_TTN[MQTT Broker]
        DECODER[Payload Decoder JS]
    end

    subgraph Backend[Backend Services]
        subgraph GoAPI[Go API Server]
            API[go-data-storage REST API]
            MQTT_CLIENT[MQTT Client]
            TTN_HANDLER[TTN Handler]
        end

        subgraph Database[Database]
            SQLITE[(SQLite sensor_data.db)]
        end
    end

    subgraph Frontend[Frontend]
        DASH[Web Dashboard]
    end

    H1 -->|LoRa SF7 AU915| GW
    L1 -->|LoRa SF7 AU915| GW
    N1 -.->|LoRa| GW
    GW --> BACKHAUL
    BACKHAUL --> TTN
    TTN --> MQTT_TTN
    TTN --> DECODER
    MQTT_TTN --> MQTT_CLIENT
    MQTT_CLIENT --> TTN_HANDLER
    TTN_HANDLER --> API
    API <--> SQLITE
    API --> DASH
```

## 6. Data Flow Timeline

```mermaid
sequenceDiagram
    autonumber
    participant S as Sensor TF02-Pro
    participant M as MCU ESP32
    participant F as Filters
    participant L as LoRa SX1276
    participant G as Gateway
    participant T as TTN
    participant A as Go API
    participant D as SQLite
    participant UI as Dashboard

    Note over S,M: Sensor Reading Phase
    S->>M: Raw Distance UART 10Hz
    M->>M: Temperature Compensation

    Note over M,F: Filtering Phase
    M->>F: Raw Value
    F->>F: Moving Average 5 samples
    F->>F: Kalman Filter
    F->>M: Filtered Values

    Note over M,M: Adaptive Sampling
    M->>M: Calculate Rate of Change
    M->>M: Determine Next Interval

    Note over M,L: Payload Construction
    M->>M: Build 16-byte Payload
    M->>L: Queue Transmission

    Note over L,G: LoRaWAN Transmission
    L->>G: Uplink SF7 AU915
    G->>G: Receive and Forward

    Note over G,T: Network Processing
    G->>T: MQTT Uplink
    T->>T: Payload Decoding
    T->>T: Session Management

    Note over T,A: Backend Integration
    T->>A: MQTT or Webhook
    A->>A: Parse and Validate
    A->>D: Store Reading

    Note over A,UI: Dashboard Update
    A->>UI: REST API Response
    UI->>UI: Real-time Chart Update

    Note over M,M: Power Management
    M->>M: Save Session to RTC
    M->>M: Enter Deep Sleep
    M->>M: Wake on Timer
```

## 7. Payload Structure

```mermaid
graph LR
    subgraph Payload[LoRaWAN Payload 16 bytes]
        B0[Byte 0 sensorType]
        B1[Bytes 1-2 rawDistanceMm]
        B2[Bytes 3-4 maDistanceMm]
        B3[Bytes 5-6 kalmanDistanceMm]
        B4[Bytes 7-8 riverLevelMm]
        B5[Bytes 9-10 signalStrength]
        B6[Byte 11 temperature]
        B7[Byte 12 batteryPercent]
        B8[Byte 13 flags]
    end

    subgraph FlagsBits[Flags Bitmap]
        F0[Bit 0 RAPID_CHANGE]
        F1[Bit 1 CRITICAL_LEVEL]
        F2[Bit 2 BATTERY_LOW]
        F3[Bit 3 SENSOR_ERROR]
        F4[Bit 4 KALMAN_INIT]
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
        RAW[Raw Sensor Reading]
    end

    subgraph MovingAvg[Moving Average Filter]
        MA_BUF[Circular Buffer 5 samples]
        MA_CALC[Calculate Average]
        MA_OUT[MA Value]
    end

    subgraph KalmanFilt[Kalman Filter]
        PRED[Predict Step]
        UPD[Update Gain K]
        EST[State Estimate]
        KF_OUT[Kalman Value]
    end

    subgraph AdaptSamp[Adaptive Sampler]
        RATE[Rate of Change cm per min]
        THRESH[Threshold Check 5cm rapid]
        INT[Interval 30s to 5min]
    end

    subgraph Output
        RESULT[FilteredReading]
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
    [*] --> WAKE: Timer or Cold Boot

    WAKE --> INIT: Initialize Hardware

    INIT --> RESTORE: Check RTC Memory

    RESTORE --> JOINED: Valid Session
    RESTORE --> JOINING: No Session

    JOINING --> JOINED: EV_JOINED
    JOINING --> JOINING: EV_JOIN_FAILED retry

    JOINED --> READING: Read Sensors

    READING --> FILTERING: Apply Filters

    FILTERING --> ADAPTIVE: Calculate Interval

    ADAPTIVE --> SENDING: Queue Packet

    SENDING --> TX_COMPLETE: EV_TXCOMPLETE

    TX_COMPLETE --> SAVE: Save Session to RTC

    SAVE --> SLEEP: Enter Deep Sleep

    SLEEP --> WAKE: Timer Wakeup

    note right of SLEEP
        Power 10uA
        Duration 30s-5min adaptive
    end note

    note right of SENDING
        Power 120mA
        Duration 100ms
    end note
```

## 10. System Deployment

```mermaid
graph TB
    subgraph RiverSite[River Monitoring Site]
        subgraph BridgeInstall[Bridge Installation]
            NODE1[Sensor Node LilyGo T-Beam]
            MOUNT[Mounting Bracket IP67]
            PANEL[Solar Panel 6W Optional]
        end

        subgraph River[River]
            WATER[Water Surface]
            FLOW[Flow Direction]
        end
    end

    subgraph GatewaySite[Gateway Site 1-15km range]
        GW_SITE[LoRaWAN Gateway RAK7268]
        INET[Internet Fiber or 4G]
    end

    subgraph CloudInfra[Cloud Infrastructure]
        TTN_CLOUD[TTN Community or Private]
        VPS[VPS Server Go API SQLite]
        WEB[Web Dashboard HTTPS]
    end

    subgraph AlertRecipients[Alert Recipients]
        CIVIL[Civil Defense SMS]
        ADMIN[System Admin Email]
        PUBLIC[Public Dashboard]
    end

    NODE1 -->|LiDAR Beam| WATER
    PANEL -.-> NODE1
    NODE1 --> MOUNT
    NODE1 -->|LoRa 915MHz up to 15km| GW_SITE
    GW_SITE --> INET
    INET --> TTN_CLOUD
    TTN_CLOUD --> VPS
    VPS --> WEB
    VPS --> CIVIL
    VPS --> ADMIN
    WEB --> PUBLIC
```
