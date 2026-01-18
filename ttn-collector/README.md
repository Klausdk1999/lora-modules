# TTN Data Collector

Microservice that subscribes to The Things Network (TTN) MQTT broker, receives uplink messages from LoRaWAN devices, decodes the binary payload, and stores data in SQLite.

## Setup

1. Copy `.env.example` to `.env` and fill in your TTN API key:
   ```bash
   cp .env.example .env
   ```

2. Install dependencies:
   ```bash
   npm install
   ```

3. Run in development mode:
   ```bash
   npm run dev
   ```

## Environment Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `TTN_MQTT_BROKER` | TTN MQTT broker URL | `mqtt://nam1.cloud.thethings.network:1883` |
| `TTN_USERNAME` | TTN application ID (format: `app-id@ttn`) | - |
| `TTN_PASSWORD` | TTN API key | - |
| `PORT` | HTTP server port | `3000` |
| `DB_PATH` | SQLite database path | `./data/ttn-data.db` |

## API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/health` | Health check with MQTT status |
| GET | `/api/stats` | Database statistics |
| GET | `/api/devices` | List all devices |
| GET | `/api/devices/:id/latest` | Latest uplink for a device |
| GET | `/api/devices/:id/uplinks` | Uplinks for a device |
| GET | `/api/uplinks` | All uplinks (with filters) |

### Query Parameters for `/api/uplinks`

- `device_id` - Filter by device ID
- `limit` - Max results (default: 100)
- `offset` - Pagination offset (default: 0)
- `start_date` - Filter by start date (ISO 8601)
- `end_date` - Filter by end date (ISO 8601)

## Payload Format

The service decodes an 8-byte binary payload from TF02-Pro LiDAR sensors:

| Offset | Type | Field | Description |
|--------|------|-------|-------------|
| 0 | uint8 | sensorType | 1 = TF02-Pro, 0xFF = error |
| 1-2 | uint16 | distanceMm | Distance in millimeters |
| 3-4 | int16 | signalStrength | Signal strength (flux) |
| 5 | int8 | temperature | Temperature in Celsius |
| 6 | uint8 | batteryPercent | Battery level 0-100% |
| 7 | uint8 | readingCount | Valid readings count |

## Example Response

```json
{
  "id": 1,
  "device_id": "lilygo-river-lora",
  "dev_eui": "70B3D57ED0074FD2",
  "received_at": "2024-01-15T10:30:00.000Z",
  "distance_mm": 1234,
  "distance_cm": 123.4,
  "temperature": 25,
  "battery_percent": 95,
  "rssi": -105,
  "snr": 7.5
}
```
