# TTN Payload Formatter Setup Guide

## What is a Payload Formatter?

A payload formatter decodes the raw base64 payload (`aGVsbG8gd29ybGQ=`) into human-readable format (`hello world`) directly in the TTN Console.

## Current Payload

- **Raw (base64)**: `aGVsbG8gd29ybGQ=`
- **Decoded**: `hello world`
- **Port**: 1 (f_port: 1)

## How to Install

1. **Go to TTN Console:**
   - Navigate to: https://console.cloud.thethings.network
   - Applications → `river-monitoring-brusque`
   - Go to **Payload Formatters** (in left menu)

2. **Select Uplink Formatter:**
   - Click on **Uplink** tab
   - Select **JavaScript** as formatter type
   - Copy and paste the code from `ttn-payload-formatter.js`
   - Click **Save**

3. **Verify:**
   - Go to **End Devices** → `heltec-river-2` → **Live Data**
   - You should now see the decoded text instead of base64!

## What You'll See

**Before (without formatter):**
```json
{
  "frm_payload": "aGVsbG8gd29ybGQ="
}
```

**After (with formatter):**
```json
{
  "text": "hello world",
  "raw_bytes": "aGVsbG8gd29ybGQ=",
  "length": 11
}
```

## Future Enhancements

When you start sending sensor data (JSON, binary, etc.), you can modify the formatter to:
- Parse JSON payloads
- Extract sensor values (temperature, humidity, etc.)
- Format data for visualization
- Handle binary sensor data

The formatter code is in `ttn-payload-formatter.js` and can be easily customized.

