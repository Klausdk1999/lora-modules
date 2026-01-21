# System Architecture Diagrams - River Level Monitoring WSN

This document provides comprehensive architecture diagrams for the complete river level monitoring system, including all components from sensor nodes to data visualization.

## Table of Contents

1. [System Overview](#system-overview)
2. [Data Flow Diagram](#data-flow-diagram)
3. [Component Architecture](#component-architecture)
4. [Network Topology](#network-topology)
5. [Deployment Architecture](#deployment-architecture)
6. [Technology Stack](#technology-stack)

---

## System Overview

### High-Level Architecture

```mermaid
graph TB
    subgraph "Field Deployment"
        N1[LilyGo T-Beam<br/>TF02-Pro LiDAR]
        N2[Heltec LoRa32 V2<br/>JSN-SR04T Ultrasonic]
        GW[Wisgate Edge Pro<br/>LoRaWAN Gateway]
    end
    
    subgraph "Cloud Services"
        TTN[The Things Network<br/>LoRaWAN Network Server<br/>MQTT Broker]
    end
    
    subgraph "Data Collection Layer"
        TC[TTN Collector<br/>Node.js/TypeScript<br/>SQLite]
    end
    
    subgraph "Data Storage & API"
        API[Go Data Storage API<br/>REST API<br/>PostgreSQL/SQLite]
    end
    
    subgraph "Visualization Layer"
        VIZ[Data Visualizer<br/>Next.js Frontend<br/>React/TypeScript]
    end
    
    N1 -->|LoRaWAN AU915| GW
    N2 -->|LoRaWAN AU915| GW
    GW -->|Internet<br/>Packet Forwarder| TTN
    TTN -->|MQTT<br/>v3/app/devices/+/up| TC
    TC -->|HTTP REST| API
    API -->|HTTP REST| VIZ
    TTN -.->|MQTT Optional| API
    
    style N1 fill:#e1f5ff
    style N2 fill:#e1f5ff
    style GW fill:#fff4e1
    style TTN fill:#e8f5e9
    style TC fill:#f3e5f5
    style API fill:#fff3e0
    style VIZ fill:#e0f2f1
```

---

## Data Flow Diagram

### Complete Data Flow from Sensor to Visualization

```mermaid
sequenceDiagram
    participant S as Sensor Node<br/>(ESP32)
    participant G as LoRa Gateway<br/>(Wisgate Edge Pro)
    participant T as TTN<br/>(Network Server)
    participant TC as TTN Collector<br/>(Node.js)
    participant API as Go API<br/>(REST Server)
    participant DB as Database<br/>(PostgreSQL/SQLite)
    participant VIZ as Frontend<br/>(Next.js)

    Note over S: Wake from Deep Sleep<br/>(15 min cycle)
    S->>S: Read Sensor Data<br/>(5 readings, avg)
    S->>S: Format Payload<br/>(8 bytes binary)
    S->>G: LoRaWAN Uplink<br/>(AU915, SF7, 14dBm)
    
    G->>T: Forward Packet<br/>(via Internet)
    T->>T: Process & Decode<br/>(LoRaWAN MAC)
    
    par MQTT Distribution
        T->>TC: MQTT Message<br/>(v3/app/devices/+/up)
        TC->>TC: Decode Binary Payload
        TC->>DB: Store Uplink Record<br/>(SQLite)
    and Optional Direct Integration
        T->>API: MQTT Message<br/>(if configured)
        API->>DB: Store Signal Value<br/>(PostgreSQL/SQLite)
    end
    
    VIZ->>API: GET /ttn/uplinks<br/>(with filters)
    API->>DB: Query Uplinks
    DB->>API: Return Data
    API->>VIZ: JSON Response
    VIZ->>VIZ: Render Charts<br/>& Data Tables
```

---

## Component Architecture

### 1. Sensor Node Architecture (LilyGo T-Beam)

```mermaid
graph LR
    subgraph "Hardware Layer"
        ESP[ESP32<br/>Dual-Core<br/>Microcontroller]
        LORA[SX1276/8<br/>LoRa Module<br/>915MHz]
        SENS[TF02-Pro<br/>LiDAR Sensor<br/>UART Serial]
        BAT[LiPo Battery<br/>2000mAh<br/>3.7V]
        ANT[LoRa Antenna<br/>915MHz]
    end
    
    subgraph "Software Layer"
        APP[Main Application<br/>main.cpp]
        LMIC[LMIC Library<br/>LoRaWAN Stack]
        SENS_LIB[TF02Pro Library<br/>Sensor Driver]
        PM[Power Management<br/>Deep Sleep]
    end
    
    subgraph "Data Processing"
        READ[Read Sensor<br/>5 readings]
        FILTER[Filter & Average<br/>Outlier removal]
        TEMP[Temperature<br/>Compensation]
        PAYLOAD[Format Payload<br/>8 bytes binary]
    end
    
    BAT --> ESP
    ESP --> LORA
    ESP --> SENS
    LORA --> ANT
    
    APP --> LMIC
    APP --> SENS_LIB
    APP --> PM
    
    SENS_LIB --> READ
    READ --> FILTER
    FILTER --> TEMP
    TEMP --> PAYLOAD
    PAYLOAD --> LMIC
    
    style ESP fill:#4fc3f7
    style LORA fill:#66bb6a
    style SENS fill:#ffa726
    style APP fill:#ab47bc
```

### 2. TTN Collector Architecture

```mermaid
graph TB
    subgraph "Input Layer"
        MQTT[MQTT Client<br/>mqtt.js]
        TOPIC[TTN Topics<br/>v3/app/devices/+/up<br/>v3/app/devices/+/join]
    end
    
    subgraph "Processing Layer"
        DECODE[Payload Decoder<br/>8-byte binary<br/>to JSON]
        VALIDATE[Data Validation<br/>Range checks]
        TRANSFORM[Data Transformation<br/>Unit conversion]
    end
    
    subgraph "Storage Layer"
        DB[(SQLite Database<br/>ttn-data.db)]
        SCHEMA[Database Schema<br/>uplinks table]
    end
    
    subgraph "API Layer"
        EXPRESS[Express Server<br/>REST API]
        ROUTES[API Routes<br/>/api/uplinks<br/>/api/devices<br/>/api/stats]
    end
    
    MQTT --> TOPIC
    TOPIC --> DECODE
    DECODE --> VALIDATE
    VALIDATE --> TRANSFORM
    TRANSFORM --> DB
    DB --> SCHEMA
    EXPRESS --> ROUTES
    ROUTES --> DB
    
    style MQTT fill:#81c784
    style DECODE fill:#ffb74d
    style DB fill:#64b5f6
    style EXPRESS fill:#ba68c8
```

### 3. Go Data Storage API Architecture

```mermaid
graph TB
    subgraph "API Layer"
        MAIN[main.go<br/>HTTP Server<br/>Port 8080]
        ROUTER[Gorilla Mux<br/>Router]
        CORS[CORS Middleware]
        AUTH[Auth Middleware<br/>JWT Tokens]
    end
    
    subgraph "Handler Layer"
        USER_H[User Handlers<br/>CRUD Operations]
        DEV_H[Device Handlers<br/>CRUD Operations]
        SIG_H[Signal Handlers<br/>Configuration]
        VAL_H[Signal Value Handlers<br/>Data Storage]
        TTN_H[TTN Handlers<br/>Uplink Queries]
    end
    
    subgraph "Business Logic"
        AUTH_LOGIC[Authentication<br/>User & Device Auth]
        VALIDATION[Input Validation]
        TRANSFORM[Data Transformation]
    end
    
    subgraph "Data Layer"
        DB[(Database<br/>PostgreSQL/SQLite)]
        GORM[GORM ORM<br/>Auto-migration]
        MODELS[Data Models<br/>User, Device<br/>Signal, SignalValue]
    end
    
    subgraph "MQTT Integration"
        MQTT_CLIENT[MQTT Client<br/>Optional TTN Integration]
        MQTT_HANDLER[Message Handler<br/>Payload Decoding]
    end
    
    MAIN --> ROUTER
    ROUTER --> CORS
    CORS --> AUTH
    AUTH --> USER_H
    AUTH --> DEV_H
    AUTH --> SIG_H
    AUTH --> VAL_H
    AUTH --> TTN_H
    
    USER_H --> AUTH_LOGIC
    DEV_H --> VALIDATION
    SIG_H --> VALIDATION
    VAL_H --> TRANSFORM
    TTN_H --> TRANSFORM
    
    AUTH_LOGIC --> DB
    VALIDATION --> DB
    TRANSFORM --> DB
    
    DB --> GORM
    GORM --> MODELS
    
    MQTT_CLIENT --> MQTT_HANDLER
    MQTT_HANDLER --> VAL_H
    
    style MAIN fill:#42a5f5
    style AUTH fill:#ef5350
    style DB fill:#66bb6a
    style MQTT_CLIENT fill:#ffa726
```

### 4. Data Visualizer Architecture

```mermaid
graph TB
    subgraph "Frontend Framework"
        NEXT[Next.js<br/>React Framework<br/>SSR/SSG]
        PAGES[Pages Router<br/>/index<br/>/ttn]
    end
    
    subgraph "UI Components"
        LOGIN[Login Component<br/>Authentication]
        DASHBOARD[Dashboard<br/>Main View]
        TTN_DASH[TTN Dashboard<br/>River Monitoring]
        CHARTS[Chart Components<br/>Recharts]
        TABLE[Data Table<br/>Sorting & Filtering]
    end
    
    subgraph "State Management"
        AUTH_STATE[Auth State<br/>JWT Token<br/>localStorage]
        DATA_STATE[Data State<br/>React Hooks]
    end
    
    subgraph "API Integration"
        API_CLIENT[API Client<br/>requestHandlers.ts]
        ENDPOINTS[API Endpoints<br/>/api/ttn/uplinks<br/>/api/auth/login]
    end
    
    subgraph "Styling"
        TAILWIND[Tailwind CSS<br/>Utility Classes]
        SHADCN[shadcn/ui<br/>Component Library]
    end
    
    NEXT --> PAGES
    PAGES --> LOGIN
    PAGES --> DASHBOARD
    PAGES --> TTN_DASH
    
    TTN_DASH --> CHARTS
    TTN_DASH --> TABLE
    
    LOGIN --> AUTH_STATE
    DASHBOARD --> DATA_STATE
    TTN_DASH --> DATA_STATE
    
    DATA_STATE --> API_CLIENT
    API_CLIENT --> ENDPOINTS
    
    NEXT --> TAILWIND
    NEXT --> SHADCN
    
    style NEXT fill:#000000,color:#fff
    style API_CLIENT fill:#4fc3f7
    style CHARTS fill:#ffa726
```

---

## Network Topology

### Physical Network Layout

```mermaid
graph TB
    subgraph "River Site - Deployment Location"
        subgraph "Node 1 - LilyGo T-Beam"
            N1_HW[ESP32 + SX1276<br/>TF02-Pro LiDAR<br/>Battery]
            N1_ANT[915MHz Antenna]
        end
        
        subgraph "Node 2 - Heltec LoRa32 V2"
            N2_HW[ESP32 + SX1276<br/>JSN-SR04T Ultrasonic<br/>Battery]
            N2_ANT[915MHz Antenna]
        end
        
        N1_HW --> N1_ANT
        N2_HW --> N2_ANT
    end
    
    subgraph "Gateway Location - Elevated Position"
        GW[Wisgate Edge Pro<br/>LoRaWAN Gateway<br/>IP: 192.168.0.165]
        GW_ANT[915MHz Antenna<br/>High Gain]
        GW_NET[Ethernet/WiFi<br/>Internet Connection]
        
        GW --> GW_ANT
        GW --> GW_NET
    end
    
    subgraph "Internet"
        INTERNET[Internet<br/>TCP/IP]
    end
    
    subgraph "Cloud Infrastructure"
        TTN_SERVER[TTN Cloud<br/>nam1.cloud.thethings.network<br/>MQTT Broker]
    end
    
    subgraph "Server Infrastructure"
        subgraph "Data Collection"
            TC_SERVER[TTN Collector<br/>Node.js<br/>Port 3000]
            TC_DB[(SQLite<br/>ttn-data.db)]
        end
        
        subgraph "API Server"
            API_SERVER[Go API<br/>Port 8080]
            API_DB[(PostgreSQL/SQLite<br/>iotdb)]
        end
        
        subgraph "Web Server"
            VIZ_SERVER[Next.js<br/>Port 3000<br/>or Nginx Proxy]
        end
    end
    
    N1_ANT -.->|LoRaWAN<br/>915MHz<br/>Up to 15km| GW_ANT
    N2_ANT -.->|LoRaWAN<br/>915MHz<br/>Up to 15km| GW_ANT
    
    GW_NET --> INTERNET
    INTERNET --> TTN_SERVER
    
    TTN_SERVER -->|MQTT<br/>Port 1883| TC_SERVER
    TC_SERVER --> TC_DB
    
    TC_SERVER -->|HTTP REST| API_SERVER
    API_SERVER --> API_DB
    
    VIZ_SERVER -->|HTTP REST| API_SERVER
    
    style N1_HW fill:#e1f5ff
    style N2_HW fill:#e1f5ff
    style GW fill:#fff4e1
    style TTN_SERVER fill:#e8f5e9
    style TC_SERVER fill:#f3e5f5
    style API_SERVER fill:#fff3e0
    style VIZ_SERVER fill:#e0f2f1
```

---

## Deployment Architecture

### Docker Compose Deployment

```mermaid
graph TB
    subgraph "Docker Network"
        subgraph "Frontend Service"
            VIZ_CONTAINER[data-visualizer<br/>Next.js<br/>Port 3000]
        end
        
        subgraph "API Service"
            API_CONTAINER[go-data-storage<br/>Go API<br/>Port 8080]
        end
        
        subgraph "Database Service"
            PG_CONTAINER[(PostgreSQL<br/>Port 5432)]
        end
        
        subgraph "TTN Collector Service"
            TC_CONTAINER[ttn-collector<br/>Node.js<br/>Port 3001]
            TC_VOLUME[(SQLite Volume<br/>./data)]
        end
        
        subgraph "Reverse Proxy"
            NGINX[Nginx<br/>Port 80/443]
        end
    end
    
    subgraph "External Services"
        TTN[TTN MQTT<br/>mqtt://nam1.cloud.thethings.network]
    end
    
    NGINX --> VIZ_CONTAINER
    NGINX --> API_CONTAINER
    NGINX --> TC_CONTAINER
    
    VIZ_CONTAINER -->|HTTP| API_CONTAINER
    API_CONTAINER -->|SQL| PG_CONTAINER
    TC_CONTAINER --> TC_VOLUME
    TC_CONTAINER -->|HTTP| API_CONTAINER
    
    TTN -->|MQTT| TC_CONTAINER
    
    style VIZ_CONTAINER fill:#000000,color:#fff
    style API_CONTAINER fill:#00add8
    style PG_CONTAINER fill:#336791
    style TC_CONTAINER fill:#339933
    style NGINX fill:#009639
```

---

## Technology Stack

### Complete Technology Stack Diagram

```mermaid
graph LR
    subgraph "Hardware"
        H1[ESP32<br/>Microcontroller]
        H2[SX1276/8<br/>LoRa Radio]
        H3[TF02-Pro<br/>LiDAR Sensor]
        H4[JSN-SR04T<br/>Ultrasonic]
        H5[Wisgate Edge Pro<br/>Gateway]
    end
    
    subgraph "Firmware"
        F1[Arduino Framework<br/>C++]
        F2[LMIC Library<br/>LoRaWAN Stack]
        F3[PlatformIO<br/>Build System]
    end
    
    subgraph "Network"
        N1[LoRaWAN<br/>AU915 Band]
        N2[TTN<br/>Network Server]
        N3[MQTT<br/>Message Broker]
    end
    
    subgraph "Backend"
        B1[Node.js<br/>TypeScript]
        B2[Go<br/>Golang]
        B3[Express.js<br/>Web Framework]
        B4[Gorilla Mux<br/>HTTP Router]
    end
    
    subgraph "Database"
        D1[SQLite<br/>Embedded DB]
        D2[PostgreSQL<br/>Relational DB]
        D3[GORM<br/>ORM Library]
    end
    
    subgraph "Frontend"
        FE1[Next.js<br/>React Framework]
        FE2[TypeScript<br/>Type Safety]
        FE3[Recharts<br/>Chart Library]
        FE4[Tailwind CSS<br/>Styling]
        FE5[shadcn/ui<br/>Components]
    end
    
    subgraph "DevOps"
        DO1[Docker<br/>Containerization]
        DO2[Docker Compose<br/>Orchestration]
        DO3[Nginx<br/>Reverse Proxy]
    end
    
    H1 --> F1
    H2 --> F2
    F1 --> F3
    H3 --> F1
    H4 --> F1
    
    F2 --> N1
    N1 --> H5
    H5 --> N2
    N2 --> N3
    
    N3 --> B1
    B1 --> B3
    B3 --> D1
    B1 --> B2
    B2 --> B4
    B4 --> D2
    D2 --> D3
    
    FE1 --> FE2
    FE1 --> FE3
    FE1 --> FE4
    FE1 --> FE5
    FE1 --> B2
    
    DO1 --> DO2
    DO2 --> DO3
    DO3 --> FE1
    DO3 --> B2
    
    style H1 fill:#e1f5ff
    style F1 fill:#ff9800
    style N1 fill:#4caf50
    style B1 fill:#339933
    style B2 fill:#00add8
    style D1 fill:#003b57
    style D2 fill:#336791
    style FE1 fill:#000000,color:#fff
    style DO1 fill:#0db7ed
```

---

## Data Payload Structure

### 8-Byte Binary Payload Format

```mermaid
graph LR
    subgraph "Payload Structure (8 bytes)"
        B0[Byte 0<br/>Sensor Type<br/>uint8]
        B1[Byte 1-2<br/>Distance<br/>uint16 LE]
        B2[Byte 3-4<br/>Signal Strength<br/>int16 LE]
        B3[Byte 5<br/>Temperature<br/>int8]
        B4[Byte 6<br/>Battery %<br/>uint8]
        B5[Byte 7<br/>Reading Count<br/>uint8]
    end
    
    subgraph "Decoded Values"
        D0[1 = TF02-Pro<br/>3 = JSN-SR04T]
        D1[Distance in mm<br/>0-65535]
        D2[Signal flux<br/>or 0]
        D3[Temp in °C<br/>-128 to 127]
        D4[Battery 0-100%]
        D5[Valid readings<br/>typically 5]
    end
    
    B0 --> D0
    B1 --> D1
    B2 --> D2
    B3 --> D3
    B4 --> D4
    B5 --> D5
    
    style B0 fill:#ff9800
    style B1 fill:#4caf50
    style B2 fill:#2196f3
    style B3 fill:#f44336
    style B4 fill:#9c27b0
    style B5 fill:#00bcd4
```

---

## Component Interaction Diagram

### Detailed Component Interactions

```mermaid
graph TB
    subgraph "Sensor Node"
        SENSOR[TF02-Pro/JSN-SR04T]
        ESP32[ESP32 MCU]
        LORA_MOD[LoRa Module]
    end
    
    subgraph "Gateway"
        GW_RADIO[LoRa Radio]
        GW_PROC[Gateway Processor]
        GW_NET[Network Stack]
    end
    
    subgraph "TTN"
        TTN_NS[Network Server]
        TTN_MQTT[MQTT Broker]
        TTN_CONSOLE[TTN Console]
    end
    
    subgraph "Collectors"
        TC[MQTT Client]
        TC_DECODE[Payload Decoder]
        TC_DB[(SQLite)]
    end
    
    subgraph "API"
        API_SERVER[HTTP Server]
        API_HANDLERS[Request Handlers]
        API_DB[(PostgreSQL)]
    end
    
    subgraph "Frontend"
        FE_PAGES[Next.js Pages]
        FE_COMP[React Components]
        FE_CHARTS[Chart Components]
    end
    
    SENSOR --> ESP32
    ESP32 --> LORA_MOD
    LORA_MOD -->|LoRaWAN| GW_RADIO
    GW_RADIO --> GW_PROC
    GW_PROC --> GW_NET
    GW_NET -->|Internet| TTN_NS
    TTN_NS --> TTN_MQTT
    TTN_MQTT -->|MQTT| TC
    TC --> TC_DECODE
    TC_DECODE --> TC_DB
    TC -->|HTTP REST| API_SERVER
    API_SERVER --> API_HANDLERS
    API_HANDLERS --> API_DB
    FE_PAGES --> FE_COMP
    FE_COMP --> FE_CHARTS
    FE_PAGES -->|HTTP REST| API_SERVER
    TTN_CONSOLE -.->|Monitoring| TTN_NS
    
    style SENSOR fill:#ff9800
    style ESP32 fill:#2196f3
    style TTN_MQTT fill:#4caf50
    style API_SERVER fill:#00add8
    style FE_PAGES fill:#000000,color:#fff
```

---

## Summary

This architecture provides:

1. **Scalable Sensor Network**: Multiple LoRaWAN nodes with different sensor types
2. **Reliable Data Collection**: TTN network server with MQTT-based data collection
3. **Flexible Storage**: Multiple storage options (SQLite, PostgreSQL)
4. **RESTful API**: Go-based API with authentication and device management
5. **Modern Frontend**: Next.js dashboard with real-time visualization
6. **Containerized Deployment**: Docker-based deployment for easy scaling

### Key Features:

- **LoRaWAN Communication**: Long-range, low-power wireless communication
- **Multiple Sensor Support**: LiDAR and Ultrasonic sensors
- **Dual Data Collection**: Both TTN Collector and Go API can collect data
- **Real-time Visualization**: Interactive charts and data tables
- **Authentication**: JWT-based user and device authentication
- **RESTful Design**: Standard HTTP API for easy integration

### Data Flow Summary:

1. Sensors collect distance measurements every 15 minutes
2. Data is transmitted via LoRaWAN to gateway
3. Gateway forwards to TTN network server
4. TTN publishes data via MQTT
5. Collectors subscribe and store data
6. API provides REST endpoints for data access
7. Frontend visualizes data in real-time
