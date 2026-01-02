# How to Get Your AppEUI from TTN Console

## Problem
The AppEUI in the code is currently set to all zeros (`00:00:00:00:00:00:00:00`), which is incorrect for TTN v3.

## Solution
You need to get the **correct AppEUI** from your TTN application.

## Steps

### 1. Go to TTN Console
- Navigate to: https://console.cloud.thethings.network/
- Log in to your account

### 2. Find Your Application
- Click on **Applications** in the left menu
- Select your application: **river-monitoring-brusque**

### 3. Get the AppEUI
- In the application overview page, look for **Application EUI** or **JoinEUI**
- It should look like: `70B3D57ED0001234` (example)
- **Copy this value**

### 4. Convert to Code Format

The AppEUI needs to be **reversed (LSB first)** for the LMIC library.

**Example:**
- TTN shows: `70B3D57ED0001234`
- In code (reversed): `{0x34, 0x12, 0x00, 0xED, 0x57, 0xD5, 0xB3, 0x70}`

**How to convert:**
1. Take the AppEUI from TTN (e.g., `70B3D57ED0001234`)
2. Split into byte pairs: `70 B3 D5 7E D0 00 12 34`
3. Reverse the order: `34 12 00 D0 7E D5 B3 70`
4. Add `0x` prefix: `0x34, 0x12, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70`

### 5. Update the Code

In `src/main.cpp`, replace the APPEUI array:

```cpp
static const u1_t PROGMEM APPEUI[8] = { 
  0x34, 0x12, 0x00, 0xD0, 0x7E, 0xD5, 0xB3, 0x70  // YOUR AppEUI (reversed)
};
```

### 6. Verify Device Registration

Make sure your device in TTN Console has:
- **DevEUI**: `75597D3513647C45` ✅ (already correct)
- **JoinEUI** (AppEUI): Must match the AppEUI you just got from the application
- **AppKey**: `BDFC920F78D3AF2053D1FB85D6A3347B` ✅ (already correct)

## Quick Check

After updating, the AppEUI should:
- Start with `70B3D57ED...` (TTN v3 format)
- Match the JoinEUI shown in your device registration
- Be reversed (LSB first) in the code

## Still Not Working?

If you still can't find the AppEUI:
1. Check if your application was created correctly
2. Look in **Applications → river-monitoring-brusque → Overview**
3. The AppEUI might be labeled as **JoinEUI** in some places

