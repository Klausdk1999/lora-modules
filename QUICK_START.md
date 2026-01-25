# Quick Start Guide
## LoRa River Level Monitoring System

This guide will help you get started quickly with the LoRa river level monitoring system.

## 🚀 Prerequisites

1. **Hardware:**
   - Heltec WiFi LoRa 32 V2 board
   - LilyGo T-Beam AXP2101 v1.2 board
   - Benewake TF02-Pro sensor
   - Wisgate Edge Pro gateway
   - USB cables for programming
   - Antennas (matching your frequency band)

2. **Software:**
   - PlatformIO IDE (VS Code extension) or PlatformIO CLI
   - USB drivers for ESP32 (usually CH340 or CP2102)

3. **Network Server Account:**
   - The Things Network account, OR
   - ChirpStack instance

## ⚡ Quick Setup (5 Steps)

### Step 1: Install PlatformIO

1. Install VS Code
2. Install PlatformIO extension
3. Open PlatformIO Home
4. Verify installation

### Step 2: Configure Gateway

1. Power on Wisgate Edge Pro
2. Connect to network (Ethernet/WiFi)
3. Access web interface: `http://<gateway-ip>`
4. Configure frequency plan for your region
5. Register gateway in your network server (TTN/ChirpStack)
6. See [GATEWAY_CONFIG.md](GATEWAY_CONFIG.md) for details

### Step 3: Register Devices

1. **In your network server (TTN/ChirpStack):**
   - Create an application
   - Register two devices (one for Heltec, one for LilyGo)
   - Generate/copy DevEUI, AppEUI, and AppKey for each device
   - **Save these credentials securely!**

2. **Device Credentials Format:**
   - DevEUI: `00:11:22:33:44:55:66:77` (8 bytes, hex)
   - AppEUI: `00:00:00:00:00:00:00:00` (8 bytes, hex)
   - AppKey: `00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF` (16 bytes, hex)

### Step 4: Configure Node Firmware

#### For Heltec Board:

1. Open `LoRa-River-Monitoring/heltec-lora32-v2/` in PlatformIO
2. Edit `src/main.cpp`:
   ```cpp
   // Replace these with your actual credentials:
   static const u1_t PROGMEM APPEUI[8] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
   static const u1_t PROGMEM DEVEUI[8] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77 };
   static const u1_t PROGMEM APPKEY[16] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 
                                            0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
   ```
3. Update node ID if desired:
   ```cpp
   char nodeId[16] = "heltec_001";
   ```
4. Adjust region in `src/lmic_project_config.h` if needed:
   ```cpp
   #define CFG_eu868 1  // Change to your region
   ```

#### For LilyGo Board (T-Beam AXP2101 v1.2):

1. Open `LoRa-River-Monitoring/lilygo-lora32/` in PlatformIO
2. **IMPORTANT:** This project uses T-Beam AXP2101 v1.2 with TF02-Pro on GPIO 13/14
3. Edit `src/main.cpp` with your credentials (same format as Heltec)
4. Use external 5V boost for TF02-Pro if running on battery
5. Update region in `src/lmic_project_config.h`

### Step 5: Build, Upload, and Test

#### Heltec Board:

```bash
cd LoRa-River-Monitoring/heltec-lora32-v2

# Connect board via USB
# Build and upload
pio run -t upload

# Monitor serial output
pio device monitor
```

#### LilyGo Board:

```bash
cd LoRa-River-Monitoring/lilygo-lora32

# Connect board via USB
# Build and upload
pio run -t upload

# Monitor serial output
pio device monitor
```

## ✅ Verification Checklist

### Hardware Connections
- [ ] TF02-Pro sensor connected (VCC, GND, TX, RX)
- [ ] 5V boost connected for TF02-Pro (battery-only)
- [ ] LoRa antenna connected
- [ ] Board powered (USB or battery)
- [ ] Serial monitor shows initialization messages

### Software Configuration
- [ ] Credentials match network server exactly
- [ ] Frequency plan matches region
- [ ] Node ID is unique
- [ ] Region setting correct in `lmic_project_config.h`

### Network Connection
- [ ] Gateway powered and connected
- [ ] Gateway shows "Connected" status
- [ ] Devices registered in network server
- [ ] Serial monitor shows "EV_JOINED" message

### Data Transmission
- [ ] Sensor readings appear in serial monitor
- [ ] "EV_TXCOMPLETE" messages appear
- [ ] Data visible in network server dashboard
- [ ] Payload format correct (JSON)

## 🔍 Troubleshooting

### "EV_JOIN_FAILED"
- **Cause:** Credentials don't match or gateway not reachable
- **Solution:** 
  - Double-check DevEUI, AppEUI, AppKey
  - Verify gateway is online
  - Check frequency plan matches

### "TF02-Pro sensor not detected"
- **Cause:** Wiring issue or power problem
- **Solution:**
  - Check UART connections (TX/RX)
  - Verify 5V boost output
  - Confirm baud rate and Serial2 init

### No data in network server
- **Cause:** Join not completed or transmission issue
- **Solution:**
  - Wait for "EV_JOINED" message
  - Check transmission interval elapsed
  - Verify gateway is forwarding packets

### Wrong pin configuration (LilyGo)
- **Cause:** Board model differs from code
- **Solution:**
  - Identify your exact board model
  - Check board documentation/schematic
  - Update `lmic_pins` structure

## 📊 Expected Serial Output

### Successful Join:
```
Heltec WiFi LoRa 32 V2 - River Level Monitor
Initializing...
TF-Luna sensor detected!
Setup complete. Waiting for LoRaWAN join...
123456: EV_JOINING
123789: EV_JOINED
NetID: 13
DevAddr: 26012345
```

### Sensor Reading:
```
Distance: 150 cm, Flux: 1234, Temp: 25 C
```

### Transmission:
```
Preparing data packet...
Node ID: heltec_001
Distance: 150 cm
Battery: 3.70 V
Sending payload: {"node_id":"heltec_001","distance_cm":150,"battery_v":3.70,"rssi":-95,"timestamp":12345}
Payload length: 78
Packet queued
123890: EV_TXSTART
123950: EV_TXCOMPLETE (includes waiting for RX windows)
```

## 📝 Next Steps

1. **Calibration:** Calibrate sensors at known distances
2. **Range Testing:** Test LoRa communication range
3. **Power Optimization:** Implement deep sleep if needed
4. **Field Deployment:** Deploy to monitoring location
5. **Data Analysis:** Set up data visualization/analysis

## 📚 Additional Resources

- [Full Documentation](README.md)
- [Gateway Configuration](GATEWAY_CONFIG.md)
- [Implementation Plan](IMPLEMENTATION_PLAN.md)
- [Wiring Diagrams](docs/wiring.md)

## 🆘 Getting Help

If you encounter issues:

1. Check serial monitor output for error messages
2. Review troubleshooting sections in documentation
3. Verify all configuration matches exactly
4. Test components individually
5. Check network server logs

---

**Ready to deploy?** Follow the [Implementation Plan](IMPLEMENTATION_PLAN.md) for detailed deployment procedures.








