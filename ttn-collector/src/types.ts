// TTN Uplink Message (from MQTT)
export interface TTNUplinkMessage {
  end_device_ids: {
    device_id: string;
    application_ids: {
      application_id: string;
    };
    dev_eui: string;
    join_eui?: string;
    dev_addr?: string;
  };
  correlation_ids: string[];
  received_at: string;
  uplink_message: {
    session_key_id?: string;
    f_port: number;
    f_cnt: number;
    frm_payload: string; // Base64 encoded payload
    decoded_payload?: Record<string, unknown>;
    rx_metadata: Array<{
      gateway_ids: {
        gateway_id: string;
        eui?: string;
      };
      time?: string;
      timestamp?: number;
      rssi: number;
      channel_rssi?: number;
      snr: number;
      location?: {
        latitude: number;
        longitude: number;
        altitude?: number;
      };
    }>;
    settings: {
      data_rate: {
        lora?: {
          bandwidth: number;
          spreading_factor: number;
        };
      };
      frequency: string;
      timestamp?: number;
    };
    received_at: string;
    consumed_airtime?: string;
  };
}

// Decoded sensor payload (matches C struct SensorPayload)
export interface DecodedPayload {
  sensorType: number;       // 1 = TF02-Pro LiDAR, 0xFF = error
  distanceMm: number;       // Distance in millimeters
  signalStrength: number;   // Signal strength (flux)
  temperature: number;      // Temperature in Celsius
  batteryPercent: number;   // Battery level (0-100)
  readingCount: number;     // Number of valid readings
}

// Database record for uplink messages
export interface UplinkRecord {
  id?: number;
  device_id: string;
  dev_eui: string;
  received_at: string;
  f_port: number;
  f_cnt: number;
  raw_payload: string;        // Base64 encoded
  sensor_type: number;
  distance_mm: number;
  signal_strength: number;
  temperature: number;
  battery_percent: number;
  reading_count: number;
  rssi: number;
  snr: number;
  gateway_id: string;
  spreading_factor: number | null;
  frequency: string | null;
}

// Config from environment
export interface Config {
  mqtt: {
    broker: string;
    username: string;
    password: string;
  };
  server: {
    port: number;
  };
  database: {
    path: string;
  };
}
