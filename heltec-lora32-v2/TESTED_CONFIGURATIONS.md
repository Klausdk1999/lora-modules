# Heltec LoRa32 V2 - Tested Configurations

This document lists all the configurations that were tested to find the best match for TTN.

## Test Summary

| Test # | Radio Chip | Pins (NSS/RST/DIO) | Data Rate | TX Power | Subband | Status |
|--------|------------|-------------------|-----------|----------|---------|--------|
| 1 | SX1276 | 18/14/{26,33,32} | SF7 | 14 dBm | 1 | ✅ Uploaded |
| 2 | SX1278 | 18/14/{26,33,32} | SF7 | 14 dBm | 1 | ⚠️ Not recognized (warnings) |
| 3 | SX1276 | 18/14/{32,33,26} | SF7 | 14 dBm | 1 | ✅ Uploaded (swapped DIO) |
| 4 | SX1276 | 18/14/{26,33,32} | SF8 | 14 dBm | 1 | ✅ Uploaded |
| 5 | SX1276 | 18/14/{26,33,32} | SF7 | 14 dBm | 2 | ✅ Uploaded |
| 6 | SX1276 | 18/14/{26,33,32} | SF7 | 20 dBm | 1 | ✅ Uploaded |
| FINAL | SX1276 | 18/14/{26,33,32} | SF7 | 14 dBm | 1 | ✅ Uploaded (Original) |

## Configuration Details

### Test 1: SX1276 (Original/Recommended)
- **Radio**: SX1276
- **Pins**: NSS=18, RST=14, DIO0=26, DIO1=33, DIO2=32
- **Data Rate**: SF7 (Spreading Factor 7)
- **TX Power**: 14 dBm
- **Subband**: 1 (channels 8-15)
- **Notes**: This is the standard configuration for Heltec WiFi LoRa 32 V2

### Test 2: SX1278
- **Radio**: SX1278 (attempted)
- **Result**: Not recognized by LMIC library - warnings shown, defaults to SX1276

### Test 3: Alternative Pin Order
- **Radio**: SX1276
- **Pins**: DIO pins swapped: {32,33,26} instead of {26,33,32}
- **Notes**: Testing if pin order matters

### Test 4: Data Rate SF8
- **Radio**: SX1276
- **Data Rate**: SF8 (slower, better range)
- **Notes**: SF8 provides better range but slower transmission

### Test 5: Subband 2
- **Radio**: SX1276
- **Subband**: 2 (channels 16-23)
- **Notes**: Testing different frequency channels

### Test 6: Higher TX Power
- **Radio**: SX1276
- **TX Power**: 20 dBm (maximum)
- **Notes**: Higher power for better signal strength

## Current Configuration (FINAL)

The code is now set back to the **original recommended configuration**:
- **Radio**: SX1276
- **Pins**: NSS=18, RST=14, DIO={26,33,32}
- **Data Rate**: SF7
- **TX Power**: 14 dBm
- **Subband**: 1 (channels 8-15)
- **Frequency Plan**: AU915

## Next Steps

1. **Monitor Serial Output**: Check which configuration successfully joins and sends data
2. **Check TTN Console**: Verify which configuration receives data in Live Data feed
3. **Gateway Logs**: Check gateway logs to see which configuration is received

## Notes

- The Heltec WiFi LoRa 32 V2 board typically uses **SX1276** radio chip
- Standard pin configuration is: NSS=18, RST=14, DIO0=26, DIO1=33, DIO2=32
- AU915 subband 1 (channels 8-15) is the standard for TTN in Australia/Brazil
- SF7 is a good balance between speed and range
- TX power of 14 dBm is standard; 20 dBm is maximum but may not be legal in all regions

