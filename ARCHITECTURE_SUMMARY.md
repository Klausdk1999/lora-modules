# Architecture Summary - River Level Monitoring System

## Quick Reference

This document provides a quick overview of the complete system architecture for the river level monitoring Wireless Sensor Network (WSN).

## System Components

### 1. **LoRa Sensor Nodes** (Field Deployment)
- **LilyGo T-Beam AXP2101 v1.2**
  - ESP32 microcontroller
  - SX1276/8 LoRa module (915MHz)
  - TF02-Pro LiDAR sensor (UART, GPIO 14/13)
  - LiPo battery (2000mAh)
  - Range: 0.1m - 22m (indoor), 0.1m - 12m (outdoor)

- **Heltec LoRa32 V2**
  - ESP32-WROOM-32 microcontroller
  - SX1276/8 LoRa module (915MHz)
  - JSN-SR04T Ultrasonic sensor (GPIO 13/12)
  - OLED display (128x64)
  - LiPo battery (2000mAh)
  - Range: 25cm - 450cm

**Key Features:**
- LoRaWAN Class A (OTAA activation)
- Frequency: AU915 (915MHz)
- Data Rate: SF7
- TX Power: 14 dBm
- Deep Sleep: 15-minute cycles
- Payload: 8-byte binary format

### 2. **LoRa Gateway** (Network Infrastructure)
- **Wisgate Edge Pro**
  - IP Address: 192.168.0.165
  - LoRa Radio: SX1276/8
  - Frequency: AU915
  - Network Server: TTN (nam1.cloud.thethings.network)
  - Connection: Ethernet/WiFi
  - Range: Up to 15km (line-of-sight)

**Function:**
- Receives LoRaWAN packets from sensor nodes
- Forwards packets to TTN via Internet
- Packet Forwarder Protocol (UDP port 1700)

### 3. **The Things Network (TTN)** (Cloud Service)
- **Network Server**: nam1.cloud.thethings.network
- **MQTT Broker**: mqtt://nam1.cloud.thethings.network:1883
- **Console**: console.thethings.network

**Function:**
- LoRaWAN Network Server (LNS)
- Device management and authentication
- MQTT message broker
- Payload formatters
- Data storage (temporary)

**MQTT Topics:**
- `v3/{app-id}@ttn/devices/{device-id}/up` - Uplink messages
- `v3/{app-id}@ttn/devices/{device-id}/join` - Join events

### 4. **TTN Collector** (Data Collection Microservice)
- **Technology**: Node.js/TypeScript
- **Port**: 3000
- **Database**: SQLite (./data/ttn-data.db)
- **Location**: `LoRa-River-Monitoring/ttn-collector/`

**Function:**
- Subscribes to TTN MQTT broker
- Decodes 8-byte binary payloads
- Stores uplink data in SQLite
- Provides REST API for data access

**API Endpoints:**
- `GET /api/health` - Health check
- `GET /api/stats` - Database statistics
- `GET /api/devices` - List all devices
- `GET /api/devices/:id/latest` - Latest uplink
- `GET /api/devices/:id/uplinks` - Device uplinks
- `GET /api/uplinks` - All uplinks (with filters)

**Environment Variables:**
- `TTN_MQTT_BROKER` - MQTT broker URL
- `TTN_USERNAME` - Application ID (format: `app-id@ttn`)
- `TTN_PASSWORD` - TTN API key
- `PORT` - HTTP server port (default: 3000)
- `DB_PATH` - SQLite database path

### 5. **Go Data Storage API** (Backend Service)
- **Technology**: Go (Golang)
- **Port**: 8080
- **Database**: PostgreSQL 16+ or SQLite
- **Location**: `go-data-storage/`

**Function:**
- RESTful API for IoT data management
- User and device authentication (JWT)
- Signal configuration management
- Signal value storage and retrieval
- Optional TTN MQTT integration
- Device management

**API Endpoints:**
- **Authentication**: `/auth/login`, `/auth/register-device`
- **Users**: `/users` (CRUD)
- **Devices**: `/devices` (CRUD)
- **Signals**: `/signals` (CRUD)
- **Signal Values**: `/signal-values` (CRUD)
- **TTN**: `/ttn/uplinks`, `/ttn/devices`, `/ttn/stats`

**Environment Variables:**
- `DB_TYPE` - Database type (`sqlite` or `postgres`)
- `DB_HOST`, `DB_PORT`, `DB_USER`, `DB_PASSWORD`, `DB_NAME` - PostgreSQL config
- `DB_PATH` - SQLite path (if `DB_TYPE=sqlite`)
- `PORT` - API server port (default: 8080)
- `TTN_MQTT_BROKER`, `TTN_USERNAME`, `TTN_PASSWORD` - Optional TTN integration

### 6. **Data Visualizer** (Frontend Application)
- **Technology**: Next.js (React/TypeScript)
- **Port**: 3000
- **Location**: `data-visualizer/`

**Function:**
- Web dashboard for data visualization
- User authentication UI
- Device management interface
- Signal configuration UI
- TTN river monitoring dashboard
- Interactive charts (Recharts)
- Data tables with filtering

**Features:**
- Login/authentication
- Device list and management
- Signal configuration
- Signal value visualization
- TTN-specific dashboard:
  - Distance over time charts
  - Battery level monitoring
  - Date range filtering
  - Parameter selection
  - Data table with sorting

**Environment Variables:**
- `NEXT_PUBLIC_API_URL` - API base URL (default: `/api`)

## Data Flow

```
Sensor Node (ESP32)
    ↓ (LoRaWAN AU915, SF7)
LoRa Gateway (Wisgate Edge Pro)
    ↓ (Internet, Packet Forwarder)
TTN Network Server
    ↓ (MQTT)
TTN Collector (Node.js)
    ↓ (HTTP REST)
Go Data Storage API
    ↓ (HTTP REST)
Data Visualizer (Next.js)
    ↓
User Browser
```

## Payload Format

### 8-Byte Binary Payload

| Byte | Type | Field | Description |
|------|------|-------|-------------|
| 0 | uint8 | sensorType | 1 = TF02-Pro, 3 = JSN-SR04T, 0xFF = error |
| 1-2 | uint16 LE | distanceMm | Distance in millimeters (0-65535) |
| 3-4 | int16 LE | signalStrength | Signal flux (LiDAR) or 0 (Ultrasonic) |
| 5 | int8 | temperature | Temperature in Celsius (-128 to 127) |
| 6 | uint8 | batteryPercent | Battery level 0-100% |
| 7 | uint8 | readingCount | Number of valid readings (typically 5) |

## Deployment Options

### Option 1: Docker Compose (Recommended)
```bash
cd go-data-storage/infra
docker-compose -f docker-compose.full.yml up -d
```

**Services:**
- PostgreSQL database
- Go API server
- Next.js frontend
- Nginx reverse proxy (optional)

### Option 2: Standalone Services
1. **TTN Collector**: `cd ttn-collector && npm run dev`
2. **Go API**: `cd go-data-storage && go run ./cmd/api`
3. **Frontend**: `cd data-visualizer && npm run dev`

### Option 3: Hybrid
- TTN Collector: Standalone (Node.js)
- Go API + Frontend: Docker Compose

## Network Configuration

### LoRaWAN Settings
- **Frequency Band**: AU915 (915MHz)
- **Sub-band**: 1 (Channels 8-15)
- **Spreading Factor**: SF7 (fixed)
- **TX Power**: 14 dBm
- **ADR**: Disabled
- **Class**: A (OTAA)

### Gateway Configuration
- **Server**: nam1.cloud.thethings.network
- **Port**: 1700 (UDP)
- **Protocol**: Packet Forwarder

### MQTT Configuration
- **Broker**: mqtt://nam1.cloud.thethings.network:1883
- **Username**: `{app-id}@ttn`
- **Password**: TTN API key
- **Topics**: `v3/{app-id}@ttn/devices/+/up`

## Port Summary

| Service | Port | Protocol | Description |
|---------|------|----------|-------------|
| LoRa Gateway | 1700 | UDP | Packet Forwarder |
| TTN MQTT | 1883 | TCP | MQTT Broker |
| TTN Collector | 3000 | HTTP | REST API |
| Go API | 8080 | HTTP | REST API |
| Frontend | 3000 | HTTP | Next.js (dev) |
| Frontend | 80/443 | HTTP/HTTPS | Nginx (prod) |
| PostgreSQL | 5432 | TCP | Database |

## Database Schemas

### TTN Collector (SQLite)
- **Table**: `uplinks`
  - id, device_id, dev_eui, received_at
  - f_port, f_cnt, raw_payload
  - sensor_type, distance_mm, signal_strength
  - temperature, battery_percent, reading_count
  - rssi, snr, gateway_id, spreading_factor, frequency

### Go API (PostgreSQL/SQLite)
- **Tables**: users, devices, signals, signal_values
- **Relations**: users → devices → signals → signal_values

## Authentication

### User Authentication
- **Method**: JWT tokens
- **Endpoint**: `POST /auth/login`
- **Payload**: `{ "email": "...", "password": "..." }`
- **Response**: `{ "token": "..." }`
- **Usage**: `Authorization: Bearer <token>`

### Device Authentication
- **Method**: Device auth tokens
- **Endpoint**: `POST /auth/register-device`
- **Usage**: `Authorization: Bearer <device_token>`

## Key Files

### Sensor Nodes
- `LoRa-River-Monitoring/lilygo-lora32/src/main.cpp` - LilyGo firmware
- `LoRa-River-Monitoring/heltec-lora32-v2/src/main.cpp` - Heltec firmware
- `LoRa-River-Monitoring/lib/sensors/` - Sensor libraries

### TTN Collector
- `LoRa-River-Monitoring/ttn-collector/src/index.ts` - Main entry
- `LoRa-River-Monitoring/ttn-collector/src/mqtt-client.ts` - MQTT client
- `LoRa-River-Monitoring/ttn-collector/src/payload-decoder.ts` - Payload decoder
- `LoRa-River-Monitoring/ttn-collector/src/database.ts` - Database operations

### Go API
- `go-data-storage/cmd/api/main.go` - API entry point
- `go-data-storage/internal/handlers/` - HTTP handlers
- `go-data-storage/internal/models/` - Data models
- `go-data-storage/internal/mqtt/` - MQTT integration

### Frontend
- `data-visualizer/src/pages/index.tsx` - Main dashboard
- `data-visualizer/src/pages/ttn/index.tsx` - TTN dashboard
- `data-visualizer/src/components/` - React components
- `data-visualizer/src/lib/requestHandlers.ts` - API client

## Development Workflow

1. **Sensor Development**
   ```bash
   cd LoRa-River-Monitoring/lilygo-lora32
   pio run -t upload
   pio device monitor
   ```

2. **TTN Collector Development**
   ```bash
   cd LoRa-River-Monitoring/ttn-collector
   npm install
   npm run dev
   ```

3. **Go API Development**
   ```bash
   cd go-data-storage
   CGO_ENABLED=1 go run ./cmd/api
   ```

4. **Frontend Development**
   ```bash
   cd data-visualizer
   npm install
   npm run dev
   ```

## Monitoring & Debugging

### Sensor Nodes
- Serial monitor (115200 baud)
- Debug messages in code
- LoRaWAN join status
- Battery level monitoring

### Gateway
- Web interface: `https://192.168.0.165`
- Status → LoRa: Packet counters
- Status → System Log: Debug logs

### TTN
- Console: console.thethings.network
- Device status and data
- MQTT message logs

### Services
- TTN Collector: Console logs
- Go API: Console logs (`log.Printf`)
- Frontend: Browser DevTools

## Troubleshooting

### Sensor Not Joining
1. Check DevEUI, AppEUI, AppKey match TTN
2. Verify frequency plan (AU915)
3. Check gateway connectivity
4. Monitor serial output

### No Data Received
1. Verify gateway → TTN connection
2. Check MQTT subscription (TTN Collector)
3. Verify payload decoder
4. Check database connection

### API Errors
1. Check database connection
2. Verify environment variables
3. Check authentication tokens
4. Review API logs

## Next Steps

1. **Field Deployment**: Deploy sensor nodes at river site
2. **Data Collection**: Collect minimum 1 week of data
3. **Analysis**: Compare sensor performance
4. **Optimization**: Adjust duty cycles and power management
5. **Scaling**: Add more sensor nodes as needed

## References

- [Architecture Diagrams](./ARCHITECTURE_DIAGRAMS.md) - Detailed diagrams
- [System Architecture](./SYSTEM_ARCHITECTURE.md) - Original architecture doc
- [Gateway Config](./GATEWAY_CONFIG.md) - Gateway setup guide
- [TTN Collector README](./ttn-collector/README.md) - Collector documentation
- [Go API README](../go-data-storage/README.md) - API documentation
- [Frontend README](../data-visualizer/README.md) - Frontend documentation
