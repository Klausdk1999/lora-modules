# Data Transmission Information

## Current Implementation

### What Data Are We Sending?
- **Payload**: The string `"hello world"` (11 bytes)
- **Port**: 1
- **Message Type**: Unconfirmed uplink (ack = 0)

### When Are We Sending?
1. **First Transmission**: 3 seconds after successful join (EV_JOINED event)
   - Scheduled via `os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(3), do_first_send)`

2. **Subsequent Transmissions**: Every 10 seconds after each successful transmission
   - Scheduled via `os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(10), do_send)` in EV_TXCOMPLETE event

### Code Location
```cpp
// Line 189: sendData() function
LMIC_setTxData2(1, (uint8_t*)"hello world", strlen("hello world"), 0);
// Parameters: port 1, data pointer, data length (11 bytes), unconfirmed (0)
```

### Scheduling Flow
1. `setup()` → `connectToLoRaNetwork()` → `LMIC_startJoining()`
2. `EV_JOINED` → Schedule `do_first_send()` in 3 seconds
3. `do_first_send()` → `sendData("hello world")` → `LMIC_setTxData2()`
4. `EV_TXCOMPLETE` → Schedule `do_send()` in 10 seconds
5. `do_send()` → `sendData("hello world")` → Repeat from step 4

---

## Library Information for Arduino IDE

### Primary Library: MCCI LoRaWAN LMIC Library
- **Library Name**: `MCCI LoRaWAN LMIC library`
- **Arduino Library Manager Name**: `MCCI LoRaWAN LMIC library`
- **Author**: MCCI Corporation
- **Version**: 4.1.1 (we're using `^4.1.1`)

### Where to Find Examples in Arduino IDE:
1. **File → Examples → MCCI LoRaWAN LMIC library →**
   - `ttn-otaa` - OTAA join example (most relevant for TTN)
   - `ttn-abp` - ABP activation example
   - `ttn-otaa-feather-us915` - US915 specific example
   - `ttn-otaa-feather-eu868` - EU868 specific example

2. **Key Example File**: `ttn-otaa`
   - Location: `Arduino/libraries/MCCI_LoRaWAN_LMIC_library/examples/ttn-otaa/`
   - This is the reference implementation for TTN OTAA

### Other Libraries Used:
1. **U8g2** (for OLED display)
   - Library Name: `U8g2`
   - Author: olikraus
   - Version: 2.36.15

2. **TinyGPSPlus** (not used in current code, but in dependencies)
   - Library Name: `TinyGPSPlus`
   - Author: mikalhart

---

## Important Configuration Files

### LMIC Configuration (config.h)
The LMIC library requires configuration in `config.h`. In PlatformIO, we use build flags instead:

**Current Build Flags** (platformio.ini):
```ini
-DCFG_au915=1              # Frequency plan: AU915
-DCFG_sx1276_radio=1       # Radio type: SX1276/77/78/79
-DDISABLE_PING=1
-DDISABLE_BEACONS=1
```

**For Arduino IDE**, you need to edit:
- `Arduino/libraries/MCCI_LoRaWAN_LMIC_library/src/lmic/config.h`
- Uncomment: `#define CFG_sx1276_radio 1`
- Set correct frequency plan: `#define CFG_au915 1` (or your region)

---

## Potential Issues to Check

### 1. Gateway Configuration
- **Frequency Plan**: Must match node (AU915)
- **Sub-band**: Must match `LMIC_selectSubBand(1)` (channels 8-15)
- **Server Address**: Must point to TTN
- **Gateway ID**: Must match TTN registration

### 2. TTN Device Configuration
- **Frequency Plan**: AU915 - Australia 915-928 MHz, FSB 2
- **LoRaWAN Version**: 1.0.3
- **Activation**: OTAA
- **DevEUI, AppEUI, AppKey**: Must match code (already configured)

### 3. Data Format
- Currently sending raw string `"hello world"` (11 bytes)
- Port 1 is standard for data
- No payload formatter needed, but TTN will show raw hex

### 4. Timing Issues
- Check if `EV_TXCOMPLETE` is being received
- Verify `os_setTimedCallback` is scheduling correctly
- Check serial logs for "Sending: hello world" and "EV_TXCOMPLETE"

### 5. Radio Configuration
- Verify SX1276 radio is correctly configured
- Check pin mappings match hardware
- Verify antenna is connected

---

## Debugging Steps

1. **Check Serial Monitor for**:
   - "EV_JOINED - Successfully joined!"
   - "First send scheduled after join"
   - "Sending: hello world"
   - "EV_TXCOMPLETE - Transmission complete"
   - RSSI and SNR values

2. **Check TTN Console**:
   - Gateway shows activity (rxin, rxok, rxfw > 0)
   - Device Live Data shows uplink messages
   - Gateway status shows connection

3. **Check Gateway Logs**:
   - Verify gateway is receiving packets
   - Check frequency plan matches
   - Verify gateway-server connection

---

## Comparison with Arduino IDE Example

The Arduino IDE `ttn-otaa` example typically:
1. Uses `do_send(&sendjob)` directly in `setup()` (but only sends after join)
2. Schedules next send in `EV_TXCOMPLETE` using `os_setTimedCallback`
3. Uses port 1 for data transmission
4. Has similar structure to our code

**Key Difference**: Our code schedules first send explicitly in `EV_JOINED`, while some examples schedule it in `setup()` and let LMIC handle the join timing.

---

## Gateway Configuration Checklist

Since no data is being received on TTN, check these gateway settings:

### 1. Frequency Plan Match
- **Gateway Frequency Plan**: Must be `AU915 - Australia 915-928 MHz, FSB 2`
- **Device Sub-band**: Using `LMIC_selectSubBand(1)` = channels 8-15
- **Gateway Channels**: Must include channels 8-15

### 2. Gateway Server Settings
- **Server Address**: `eu1.cloud.thethings.network` (for EU region) or correct region
- **Server Port**: `1700` (UDP)
- **Protocol**: `Semtech UDP GWMP Protocol` (Packet Forwarder)

### 3. Gateway Status in TTN
- Gateway should show "Connected" status
- Check gateway metrics: `rxin`, `rxok`, `rxfw` should increase when receiving

### 4. Device Range
- Device must be within range of gateway
- Check RSSI values in serial monitor (should be > -120 dBm)

### 5. Antenna
- Both gateway and device antennas must be connected
- Verify antenna is for correct frequency band (915 MHz for AU915)

