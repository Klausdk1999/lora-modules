/**
 * TTN Payload Formatter for Heltec LoRa32 V2
 * 
 * Decodes base64 payload to UTF-8 text
 * 
 * To use in TTN Console:
 * 1. Go to: Applications > river-monitoring-brusque > Payload Formatters
 * 2. Select "Uplink" formatter
 * 3. Choose "JavaScript" as formatter type
 * 4. Paste this code
 * 5. Save
 */

// Decode base64 to UTF-8 string
function decodeBase64(base64) {
  const binaryString = atob(base64);
  const bytes = new Uint8Array(binaryString.length);
  for (let i = 0; i < binaryString.length; i++) {
    bytes[i] = binaryString.charCodeAt(i);
  }
  return String.fromCharCode.apply(null, bytes);
}

// Uplink decoder function
function decodeUplink(input) {
  try {
    // Get the base64 payload from frm_payload
    const base64Payload = input.bytes;
    
    // Decode from base64 to UTF-8 string
    const decodedText = decodeBase64(base64Payload);
    
    return {
      data: {
        text: decodedText,
        raw_bytes: input.bytes,
        length: decodedText.length
      },
      warnings: [],
      errors: []
    };
  } catch (error) {
    return {
      data: {},
      warnings: [],
      errors: ["Failed to decode payload: " + error.message]
    };
  }
}

// Downlink encoder function (if needed)
function encodeDownlink(input) {
  // Encode text to base64
  const text = input.text || "";
  const bytes = new Uint8Array(text.length);
  for (let i = 0; i < text.length; i++) {
    bytes[i] = text.charCodeAt(i);
  }
  const binaryString = String.fromCharCode.apply(null, bytes);
  const base64 = btoa(binaryString);
  
  return {
    bytes: base64.split('').map(c => c.charCodeAt(0)),
    fPort: input.fPort || 1,
    warnings: [],
    errors: []
  };
}



