/**
 * TTN Payload Formatter for LilyGo LoRa32 - TF02 Pro Sensor Node
 * 
 * Decodes binary SensorPayload struct from TF02 Pro LiDAR sensor
 * 
 * Payload Structure (packed, little-endian):
 * - uint8_t  sensorType         (1 byte)  - Sensor type (1 = TF02-Pro)
 * - uint16_t distanceMm         (2 bytes) - Distance in millimeters
 * - int16_t  signalStrength     (2 bytes) - Signal strength (flux)
 * - int8_t   temperature        (1 byte)  - Temperature in Celsius
 * - uint8_t  batteryPercent     (1 byte)  - Battery level (0-100)
 * - uint8_t  readingCount       (1 byte)  - Number of valid readings in average
 * 
 * Total size: 8 bytes
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

// Uplink decoder function
function decodeUplink(input) {
  try {
    const bytes = input.bytes;
    
    // Check minimum payload size (8 bytes for SensorPayload struct)
    if (!bytes || bytes.length < 8) {
      return {
        data: {
          raw_bytes: bytes,
          raw_hex: bytes ? bytes.map(b => ('0' + b.toString(16)).slice(-2)).join(' ') : 'empty'
        },
        warnings: ["Payload too short (expected 8 bytes, got " + (bytes ? bytes.length : 0) + ")"],
        errors: []
      };
    }
    
    // Check for error indicator (sensorType = 0xFF means sensor read failed)
    if (bytes[0] === 0xFF) {
      return {
        data: {
          sensor_type: "ERROR",
          error: true,
          message: "Sensor read failed"
        },
        warnings: [],
        errors: []
      };
    }
    
    // Parse SensorPayload struct (little-endian, packed)
    let offset = 0;
    const sensorType = bytes[offset++];                    // uint8_t (1 byte)
    const distanceMm = readUInt16LE(bytes, offset);        // uint16_t (2 bytes, little-endian)
    offset += 2;
    const signalStrength = readInt16LE(bytes, offset);     // int16_t (2 bytes, little-endian)
    offset += 2;
    const temperature = readInt8(bytes, offset);           // int8_t (1 byte, signed)
    offset += 1;
    const batteryPercent = bytes[offset++];                // uint8_t (1 byte)
    const readingCount = bytes[offset++];                  // uint8_t (1 byte)
    
    // Convert distance from millimeters to centimeters for readability
    const distanceCm = distanceMm / 10.0;
    
    // Determine sensor type name
    let sensorTypeName = "Unknown";
    if (sensorType === 1) {
      sensorTypeName = "TF02-Pro LiDAR";
    } else if (sensorType === 0) {
      sensorTypeName = "Invalid/Uninitialized";
    }
    
    // Return structured data
    return {
      data: {
        sensor_type: sensorTypeName,
        sensor_type_id: sensorType,
        distance_cm: Math.round(distanceCm * 100) / 100,  // Round to 2 decimal places
        distance_mm: distanceMm,
        distance_m: Math.round((distanceCm / 100) * 1000) / 1000,  // Convert to meters, 3 decimals
        signal_strength: signalStrength,
        temperature_celsius: temperature,
        battery_percent: batteryPercent,
        reading_count: readingCount,
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

// Downlink encoder function (if needed)
function encodeDownlink(input) {
  // For downlinks, you can encode commands
  // Example: Set transmission interval, sensor settings, etc.
  const command = input.data || {};
  const bytes = [];
  
  // Simple example: encode a command byte
  if (command.set_interval) {
    // Command format: [CMD_TYPE, INTERVAL_HI, INTERVAL_LO]
    bytes.push(0x01);  // Command type: set interval
    bytes.push((command.set_interval >> 8) & 0xFF);  // High byte
    bytes.push(command.set_interval & 0xFF);         // Low byte
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
