import type { DecodedPayload } from './types';

/**
 * Decodes the binary payload from TF02-Pro LiDAR sensor nodes.
 *
 * Payload structure (8 bytes, packed little-endian):
 * - Offset 0: sensorType (uint8_t) - 1 = TF02-Pro, 0xFF = error
 * - Offset 1-2: distanceMm (uint16_t) - Distance in millimeters
 * - Offset 3-4: signalStrength (int16_t) - Signal strength (flux)
 * - Offset 5: temperature (int8_t) - Temperature in Celsius
 * - Offset 6: batteryPercent (uint8_t) - Battery level 0-100
 * - Offset 7: readingCount (uint8_t) - Number of valid readings
 */
export function decodePayload(base64Payload: string): DecodedPayload | null {
  try {
    const buffer = Buffer.from(base64Payload, 'base64');

    if (buffer.length < 8) {
      console.error(`[Decoder] Payload too short: ${buffer.length} bytes (expected 8)`);
      return null;
    }

    const sensorType = buffer.readUInt8(0);
    const distanceMm = buffer.readUInt16LE(1);
    const signalStrength = buffer.readInt16LE(3);
    const temperature = buffer.readInt8(5);
    const batteryPercent = buffer.readUInt8(6);
    const readingCount = buffer.readUInt8(7);

    return {
      sensorType,
      distanceMm,
      signalStrength,
      temperature,
      batteryPercent,
      readingCount,
    };
  } catch (error) {
    console.error('[Decoder] Failed to decode payload:', error);
    return null;
  }
}

/**
 * Formats decoded payload as human-readable string
 */
export function formatPayload(payload: DecodedPayload): string {
  const sensorName = payload.sensorType === 1 ? 'TF02-Pro' :
                     payload.sensorType === 0xFF ? 'Error' :
                     `Unknown (${payload.sensorType})`;

  const distanceCm = (payload.distanceMm / 10).toFixed(1);

  return [
    `Sensor: ${sensorName}`,
    `Distance: ${distanceCm} cm (${payload.distanceMm} mm)`,
    `Signal: ${payload.signalStrength}`,
    `Temp: ${payload.temperature}°C`,
    `Battery: ${payload.batteryPercent}%`,
    `Readings: ${payload.readingCount}`,
  ].join(' | ');
}
