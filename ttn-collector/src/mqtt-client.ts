import mqtt, { MqttClient } from 'mqtt';
import type { Config, TTNUplinkMessage, UplinkRecord } from './types';
import { decodePayload, formatPayload } from './payload-decoder';
import { insertUplink } from './database';

let client: MqttClient | null = null;

export function connectMqtt(config: Config): MqttClient {
  const { broker, username, password } = config.mqtt;

  // TTN topic patterns
  const uplinkTopic = `v3/${username}/devices/+/up`;
  const joinTopic = `v3/${username}/devices/+/join`;

  console.log(`[MQTT] Connecting to ${broker}...`);
  console.log(`[MQTT] Username: ${username}`);

  client = mqtt.connect(broker, {
    username,
    password,
    clientId: `ttn-collector-${Date.now()}`,
    clean: true,
    reconnectPeriod: 5000,
    connectTimeout: 30000,
  });

  client.on('connect', () => {
    console.log('[MQTT] Connected to TTN broker');

    // Subscribe to both uplink and join topics
    client!.subscribe([uplinkTopic, joinTopic], { qos: 1 }, (err, granted) => {
      if (err) {
        console.error('[MQTT] Subscribe error:', err);
      } else if (granted && granted.length > 0) {
        granted.forEach(g => console.log(`[MQTT] Subscribed to: ${g.topic}`));
      }
    });
  });

  client.on('message', (receivedTopic, message) => {
    // Check if this is a join message
    if (receivedTopic.endsWith('/join')) {
      handleJoinMessage(message);
    } else if (receivedTopic.endsWith('/up')) {
      handleUplinkMessage(message);
    }
  });

  client.on('error', (error) => {
    console.error('[MQTT] Error:', error.message);
  });

  client.on('close', () => {
    console.log('[MQTT] Connection closed');
  });

  client.on('reconnect', () => {
    console.log('[MQTT] Reconnecting...');
  });

  client.on('offline', () => {
    console.log('[MQTT] Client offline');
  });

  return client;
}

interface TTNJoinMessage {
  end_device_ids: {
    device_id: string;
    dev_eui: string;
    dev_addr: string;
  };
  received_at: string;
  join_accept: {
    session_key_id: string;
  };
}

function handleJoinMessage(message: Buffer): void {
  try {
    const parsed: TTNJoinMessage = JSON.parse(message.toString());
    const device = parsed.end_device_ids;

    console.log('\n' + '='.repeat(60));
    console.log(`[JOIN] Device: ${device.device_id}`);
    console.log(`[JOIN] DevEUI: ${device.dev_eui}`);
    console.log(`[JOIN] DevAddr: ${device.dev_addr}`);
    console.log(`[JOIN] Time: ${parsed.received_at}`);
    console.log('='.repeat(60));

  } catch (error) {
    console.error('[MQTT] Error processing join message:', error);
  }
}

function handleUplinkMessage(message: Buffer): void {
  try {
    const parsed: TTNUplinkMessage = JSON.parse(message.toString());
    const uplink = parsed.uplink_message;
    const device = parsed.end_device_ids;

    console.log('\n' + '='.repeat(60));
    console.log(`[UPLINK] Device: ${device.device_id}`);
    console.log(`[UPLINK] DevEUI: ${device.dev_eui}`);
    console.log(`[UPLINK] Time: ${parsed.received_at}`);
    console.log(`[UPLINK] FPort: ${uplink.f_port}, FCnt: ${uplink.f_cnt ?? 'N/A'}`);

    // Decode payload
    const decoded = decodePayload(uplink.frm_payload);
    if (!decoded) {
      console.error('[UPLINK] Failed to decode payload');
      return;
    }

    console.log(`[UPLINK] Payload: ${formatPayload(decoded)}`);

    // Get best gateway metadata
    const bestGateway = uplink.rx_metadata.reduce((best, curr) =>
      curr.rssi > best.rssi ? curr : best
    , uplink.rx_metadata[0]);

    console.log(`[UPLINK] Gateway: ${bestGateway.gateway_ids.gateway_id}`);
    console.log(`[UPLINK] RSSI: ${bestGateway.rssi} dBm, SNR: ${bestGateway.snr} dB`);

    // Extract spreading factor
    const sf = uplink.settings.data_rate.lora?.spreading_factor ?? null;
    if (sf) {
      console.log(`[UPLINK] SF: ${sf}`);
    }

    // Prepare database record (convert undefined to null for sql.js)
    const record: UplinkRecord = {
      device_id: device.device_id,
      dev_eui: device.dev_eui,
      received_at: parsed.received_at,
      f_port: uplink.f_port ?? 0,
      f_cnt: uplink.f_cnt ?? 0,
      raw_payload: uplink.frm_payload,
      sensor_type: decoded.sensorType,
      distance_mm: decoded.distanceMm,
      signal_strength: decoded.signalStrength,
      temperature: decoded.temperature,
      battery_percent: decoded.batteryPercent,
      reading_count: decoded.readingCount,
      rssi: bestGateway.rssi,
      snr: bestGateway.snr,
      gateway_id: bestGateway.gateway_ids.gateway_id,
      spreading_factor: sf,
      frequency: uplink.settings.frequency ?? null,
    };

    // Insert into database
    const insertedId = insertUplink(record);
    console.log(`[DB] Stored uplink with ID: ${insertedId}`);
    console.log('='.repeat(60));

  } catch (error) {
    console.error('[MQTT] Error processing message:', error);
  }
}

export function disconnectMqtt(): void {
  if (client) {
    client.end();
    client = null;
    console.log('[MQTT] Disconnected');
  }
}

export function getMqttClient(): MqttClient | null {
  return client;
}
