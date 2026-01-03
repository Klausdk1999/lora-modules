# TTN Device Setup Guide - OTAA Manual Configuration

## Current Code Values

Based on your current code configuration, use these values when adding the device manually in TTN Console:

### Values to Use in TTN Console:

#### 1. JoinEUI/AppEUI
**Option A (Recommended for testing):**
```
0000000000000000
```
(All zeros - this is acceptable for programmable devices per TTN docs)

**Option B (If you want to use the application's AppEUI):**
- Go to your application → Overview
- Copy the "Application EUI" (starts with 70B3D57ED...)
- Use that value here

#### 2. DevEUI
```
75597D3513647C45
```
(This is already in your code - use this EXACT value)

#### 3. AppKey
```
BDFC920F78D3AF2053D1FB85D6A3347B
```
(This is already in your code - use this EXACT value)

#### 4. End Device ID
```
heltec-00
```
(Or any unique ID you prefer)

## Step-by-Step Instructions

### 1. Go to TTN Console
- Navigate to: https://console.cloud.thethings.network/
- Log in

### 2. Select Your Application
- Click **Applications** → **river-monitoring-brusque**

### 3. Add End Device
- Click **End Devices** → **Add End Device**
- Select **Manually**

### 4. Fill in the Form

**Frequency Plan:**
- Select: **Australia 915-928 MHz, FSB 2 (used by TTN)**

**LoRaWAN version:**
- Select: **MAC V1.0.2** (or V1.0.3 if available)

**Regional Parameters revision:**
- Select: **RP001 Regional Parameters 1.0.2 revision B**

**JoinEUI (AppEUI):**
- Enter: `0000000000000000` (all zeros) OR your application's AppEUI

**DevEUI:**
- Enter: `75597D3513647C45`
- You can paste it with or without colons - TTN will format it

**AppKey:**
- Enter: `BDFC920F78D3AF2053D1FB85D6A3347B`
- This is your secret key - keep it secure!

**End device ID:**
- Enter: `heltec-00` (or any unique name)

### 5. Confirm and Save
- Click **Confirm**
- Review the settings
- Click **Add end device**

## Important Notes

### Byte Order (Endianness)
- **In TTN Console**: Enter values as shown above (MSB first)
- **In Code**: The LMIC library expects LSB first (little-endian)
- **Your current code**: Already has values in the correct format for LMIC

### Current Code Status
Your code currently has:
- ✅ DevEUI: Correct (75597D3513647C45)
- ✅ AppKey: Correct (BDFC920F78D3AF2053D1FB85D6A3347B)
- ⚠️ AppEUI: Currently all zeros (which is OK if you use all zeros in TTN)

### After Adding Device
1. Make sure the values in TTN match your code
2. Upload the code to your Heltec board
3. Open Serial Monitor (115200 baud)
4. Watch for "EV_JOINED" message
5. Check TTN Console → Live Data for received messages

## Troubleshooting

### Join Fails
- Verify all three values (JoinEUI, DevEUI, AppKey) match exactly
- Check frequency plan matches (AU915 FSB 2)
- Ensure gateway is online and connected

### Join Works but No Data
- Check Serial Monitor for "EV_TXSTART" and "EV_TXCOMPLETE"
- Verify gateway is receiving packets (check gateway logs)
- Check TTN Console → Live Data

### Still Having Issues?
- Double-check byte order (use values exactly as shown above in TTN)
- Delete the device and re-add it
- Reset your Heltec board and try again



