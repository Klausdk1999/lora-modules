# TTN to MQTT Setup Guide

## Overview

You can subscribe to TTN data via MQTT. The TTN provides a native MQTT broker, or you can bridge to your own Mosquitto server.

## Option 1: TTN Native MQTT (Easiest - Recommended for Testing)

### Get MQTT Credentials

1. Go to TTN Console: https://console.cloud.thethings.network
2. Navigate to: **Applications** → `river-monitoring-brusque`
3. Go to **Integrations** → **MQTT**
4. Copy your credentials:
   - **Server**: `nam1.cloud.thethings.network:1883` (or `:8883` for TLS)
   - **Username**: `river-monitoring-brusque@ttn`
   - **Password**: (click "Generate new API key" or use existing)
   - **Topic**: `v3/{application_id}@{tenant_id}/devices/{device_id}/up`

### Subscribe with MQTT Client

**Using mosquitto_sub (command line):**
```bash
mosquitto_sub -h nam1.cloud.thethings.network -p 1883 \
  -u "river-monitoring-brusque@ttn" \
  -P "YOUR_API_KEY" \
  -t "v3/river-monitoring-brusque@ttn/devices/heltec-river-2/up" \
  -v
```

**Using Python (example):**
```python
import paho.mqtt.client as mqtt

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("v3/river-monitoring-brusque@ttn/devices/heltec-river-2/up")

def on_message(client, userdata, msg):
    print(f"Topic: {msg.topic}")
    print(f"Message: {msg.payload.decode()}")

client = mqtt.Client()
client.username_pw_set("river-monitoring-brusque@ttn", "YOUR_API_KEY")
client.on_connect = on_connect
client.on_message = on_message

client.connect("nam1.cloud.thethings.network", 1883, 60)
client.loop_forever()
```

## Option 2: Bridge to Local Mosquitto (Advanced)

To forward TTN data to your local Mosquitto server, you can:

1. **Use TTN Webhooks** → Forward to MQTT bridge service
2. **Use Node-RED** → Subscribe to TTN MQTT and republish to Mosquitto
3. **Use custom script** → Subscribe to TTN MQTT and republish to Mosquitto

### Example: Python Bridge Script

```python
import paho.mqtt.client as mqtt_ttn
import paho.mqtt.client as mqtt_local
import json

# TTN MQTT Client
ttn_client = mqtt_ttn.Client()
ttn_client.username_pw_set("river-monitoring-brusque@ttn", "YOUR_TTN_API_KEY")

# Local Mosquitto Client
local_client = mqtt_local.Client()
local_client.connect("localhost", 1883, 60)

def on_ttn_message(client, userdata, msg):
    data = json.loads(msg.payload.decode())
    
    # Forward to local Mosquitto
    local_client.publish("heltec-river-2/data", json.dumps(data))
    print(f"Forwarded: {data}")

ttn_client.on_message = on_ttn_message
ttn_client.connect("nam1.cloud.thethings.network", 1883, 60)
ttn_client.subscribe("v3/river-monitoring-brusque@ttn/devices/heltec-river-2/up")
ttn_client.loop_forever()
```

## Option 3: Webhook to MQTT Service

1. TTN Console → Applications → `river-monitoring-brusque` → **Integrations** → **Webhooks**
2. Create new webhook pointing to MQTT service
3. Configure payload formatting

## Message Format

TTN MQTT messages are JSON. Example:
```json
{
  "end_device_ids": {
    "device_id": "heltec-river-2",
    "application_ids": {"application_id": "river-monitoring-brusque"}
  },
  "received_at": "2026-01-02T19:48:54.548Z",
  "uplink_message": {
    "f_port": 1,
    "f_cnt": 9,
    "frm_payload": "aGVsbG8gd29ybGQ=",
    "decoded_payload": {
      "text": "hello world"
    },
    "rx_metadata": [{
      "rssi": -19,
      "snr": 11.5
    }]
  }
}
```

## Quick Test with mosquitto_sub

1. Install Mosquitto: `sudo apt-get install mosquitto-clients` (Linux) or download from https://mosquitto.org/download/
2. Get TTN API key from Console
3. Run subscription command (see Option 1)
4. Watch data arrive in real-time!



