/**
 * TTN Payload Formatter for LilyGo T-Beam V1.2 - TF02 Pro Sensor Node
 *
 * Decodes binary SensorPayload struct from TF02 Pro LiDAR sensor
 * with adaptive sampling and river level calculation
 *
 * Payload Structure (packed, little-endian):
 * - uint8_t  sensorType         (1 byte)  - Sensor type (1 = TF02-Pro, 0xFF = error)
 * - uint16_t distanceMm         (2 bytes) - Distance to water in millimeters
 * - uint16_t riverLevelMm       (2 bytes) - Calculated river level in millimeters
 * - int16_t  signalStrength     (2 bytes) - Signal strength (flux)
 * - int8_t   temperature        (1 byte)  - Temperature in Celsius
 * - uint8_t  batteryPercent     (1 byte)  - Battery level (0-100)
 * - uint8_t  readingCount       (1 byte)  - Number of valid readings in average
 * - uint8_t  flags              (1 byte)  - Status flags
 *
 * Flags:
 *   bit 0: rapid_change     - Water level changed rapidly (>10cm)
 *   bit 1: critical_level   - Water level exceeds critical threshold (>50cm)
 *   bit 2: battery_low      - Battery below 20%
 *
 * Total size: 12 bytes
 *
 * To use in TTN Console:
 * 1. Go to: Applications > river-monitoring-brusque > End Devices > lilygo-river-lora > Payload Formatters
 * 2. Select "Uplink" formatter
 * 3. Choose "JavaScript" as formatter type
 * 4. Paste this code
 * 5. Save
 */

// Helper function to read little-endian uint16_t from byte array
function readUInt16LE(bytes, offset) {
  return bytes[offset] | (bytes[offset + 1] << 8);
}

// Helper function to read little-endian int16_t from byte array
function readInt16LE(bytes, offset) {
  const value = readUInt16LE(bytes, offset);
  return (value > 32767) ? value - 65536 : value;
}

// Helper function to read int8_t (signed byte)
function readInt8(bytes, offset) {
  const value = bytes[offset];
  return (value > 127) ? value - 256 : value;
}

// Parse flags byte into readable object
function parseFlags(flags) {
  return {
    rapid_change: (flags & 0x01) !== 0,
    critical_level: (flags & 0x02) !== 0,
    battery_low: (flags & 0x04) !== 0
  };
}

// Uplink decoder function
function decodeUplink(input) {
  try {
    const bytes = input.bytes;

    // Check minimum payload size (12 bytes for extended SensorPayload struct)
    // Also support legacy 8-byte payload for backwards compatibility
    if (!bytes || bytes.length < 8) {
      return {
        data: {
          raw_bytes: bytes,
          raw_hex: bytes ? bytes.map(b => ('0' + b.toString(16)).slice(-2)).join(' ') : 'empty'
        },
        warnings: ["Payload too short (expected 12 bytes, got " + (bytes ? bytes.length : 0) + ")"],
        errors: []
      };
    }

    // Check for error indicator (sensorType = 0xFF means sensor read failed)
    if (bytes[0] === 0xFF) {
      // Still decode battery and flags from error payload
      let batteryPercent = 0;
      let flags = 0;
      let flagsObj = { rapid_change: false, critical_level: false, battery_low: false };

      if (bytes.length >= 12) {
        batteryPercent = bytes[9];
        flags = bytes[11];
        flagsObj = parseFlags(flags);
      }

      return {
        data: {
          sensor_type: "ERROR",
          error: true,
          message: "Sensor read failed",
          battery_percent: batteryPercent,
          flags: flagsObj,
          flags_raw: flags
        },
        warnings: [],
        errors: []
      };
    }

    // Determine if this is legacy (8 bytes) or extended (12 bytes) payload
    const isExtendedPayload = bytes.length >= 12;

    // Parse SensorPayload struct (little-endian, packed)
    let offset = 0;
    const sensorType = bytes[offset++];                    // uint8_t (1 byte)
    const distanceMm = readUInt16LE(bytes, offset);        // uint16_t (2 bytes)
    offset += 2;

    let riverLevelMm = 0;
    if (isExtendedPayload) {
      riverLevelMm = readUInt16LE(bytes, offset);          // uint16_t (2 bytes) - new field
      offset += 2;
    }

    const signalStrength = readInt16LE(bytes, offset);     // int16_t (2 bytes)
    offset += 2;
    const temperature = readInt8(bytes, offset);           // int8_t (1 byte)
    offset += 1;
    const batteryPercent = bytes[offset++];                // uint8_t (1 byte)
    const readingCount = bytes[offset++];                  // uint8_t (1 byte)

    let flags = 0;
    let flagsObj = { rapid_change: false, critical_level: false, battery_low: false };
    if (isExtendedPayload && bytes.length > offset) {
      flags = bytes[offset++];                              // uint8_t (1 byte) - new field
      flagsObj = parseFlags(flags);
    }

    // Convert distance from millimeters to centimeters for readability
    const distanceCm = distanceMm / 10.0;
    const riverLevelCm = riverLevelMm / 10.0;

    // Determine sensor type name
    let sensorTypeName = "Unknown";
    if (sensorType === 1) {
      sensorTypeName = "TF02-Pro LiDAR";
    } else if (sensorType === 0) {
      sensorTypeName = "Invalid/Uninitialized";
    }

    // Build alerts array based on flags
    const alerts = [];
    if (flagsObj.rapid_change) alerts.push("RAPID_CHANGE");
    if (flagsObj.critical_level) alerts.push("CRITICAL_LEVEL");
    if (flagsObj.battery_low) alerts.push("BATTERY_LOW");

    // Return structured data
    return {
      data: {
        // Sensor identification
        sensor_type: sensorTypeName,
        sensor_type_id: sensorType,

        // Distance measurements
        distance_cm: Math.round(distanceCm * 100) / 100,
        distance_mm: distanceMm,
        distance_m: Math.round((distanceCm / 100) * 1000) / 1000,

        // River level (calculated from sensor height - distance)
        river_level_cm: Math.round(riverLevelCm * 100) / 100,
        river_level_mm: riverLevelMm,
        river_level_m: Math.round((riverLevelCm / 100) * 1000) / 1000,

        // Signal quality
        signal_strength: signalStrength,

        // Environmental
        temperature_celsius: temperature,

        // Power status
        battery_percent: batteryPercent,

        // Data quality
        reading_count: readingCount,

        // Status flags
        flags: flagsObj,
        flags_raw: flags,
        alerts: alerts,
        alert_count: alerts.length,

        // Metadata
        payload_version: isExtendedPayload ? 2 : 1,
        timestamp: new Date().toISOString()
      },
      warnings: [],
      errors: []
    };
  } catch (error) {
    // If parsing fails, return raw data with error
    return {
      data: {
        raw_bytes: input.bytes || [],
        raw_hex: input.bytes ? input.bytes.map(b => ('0' + b.toString(16)).slice(-2)).join(' ') : 'empty'
      },
      warnings: [],
      errors: ["Failed to decode payload: " + error.message]
    };
  }
}

// Downlink encoder function
function encodeDownlink(input) {
  // For downlinks, you can encode commands
  // Example commands:
  //   set_interval: Set transmission interval in seconds
  //   set_sensor_height: Set sensor height in cm for river level calculation
  //   reset_historical_min: Reset historical minimum distance
  const command = input.data || {};
  const bytes = [];

  if (command.set_interval) {
    // Command format: [0x01, INTERVAL_HI, INTERVAL_LO]
    bytes.push(0x01);  // Command type: set interval
    bytes.push((command.set_interval >> 8) & 0xFF);
    bytes.push(command.set_interval & 0xFF);
  } else if (command.set_sensor_height) {
    // Command format: [0x02, HEIGHT_HI, HEIGHT_LO] (in cm)
    bytes.push(0x02);  // Command type: set sensor height
    bytes.push((command.set_sensor_height >> 8) & 0xFF);
    bytes.push(command.set_sensor_height & 0xFF);
  } else if (command.reset_historical_min) {
    // Command format: [0x03]
    bytes.push(0x03);  // Command type: reset historical minimum
  } else {
    // Default: no-op command
    bytes.push(0x00);
  }

  return {
    bytes: bytes,
    fPort: input.fPort || 1,
    warnings: [],
    errors: []
  };
}
