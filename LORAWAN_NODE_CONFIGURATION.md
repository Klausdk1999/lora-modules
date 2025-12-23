# LoRaWAN Node Configuration Guide

## 📡 What Information is Needed to Send Data to the Gateway

When sending LoRa data from your nodes (Heltec/LilyGo) to the Wisgate Edge Pro gateway, you need to configure **LoRaWAN credentials** that match what's registered in your **LoRaWAN Network Server** (TTN, ChirpStack, etc.).

### ⚠️ Important: LoRaWAN Architecture

**You don't directly "target" the gateway IP address.** Instead:
1. **Nodes broadcast** LoRaWAN packets on the configured frequency
2. **Gateway receives** any LoRaWAN packets in range (on the same frequency plan)
3. **Gateway forwards** packets to the **LoRaWAN Network Server** (LNS)
4. **Network Server** validates credentials and routes data to your application

---

## 🔑 Required Information for Each Node

### 1. **DevEUI** (Device Extended Unique Identifier)
- **8 bytes** (16 hex characters)
- **Unique identifier** for each device/node
- Example: `00:11:22:33:44:55:66:77`
- **Where to get it:**
  - Generate a unique one for each node
  - Or use the MAC address of your ESP32 board

### 2. **AppEUI** (Application Extended Unique Identifier)
- **8 bytes** (16 hex characters)
- **Same for all devices** in your application
- Example: `00:00:00:00:00:00:00:00` (or generate a unique one)
- **Where to get it:**
  - From your LoRaWAN Network Server (TTN/ChirpStack)
  - Or generate one when creating your application

### 3. **AppKey** (Application Key)
- **16 bytes** (32 hex characters)
- **Secret key** used for encryption
- **Must match exactly** between node and network server
- Example: `00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF`
- **Where to get it:**
  - Generated when you register the device in the network server
  - **Keep this secret!**

### 4. **Frequency Plan**
- **Must match** between:
  - Gateway configuration (AS923-1 for Brazil)
  - Network Server configuration
  - Node code configuration
- **For Brazil:** Use **AS923-1**

---

## 📝 Step-by-Step Configuration

### Step 1: Configure Gateway
1. Access gateway at: **https://192.168.0.165**
2. Go to **Network → LoRa**
3. Set **Frequency Plan:** **AS923-1** (Brazil)
4. Configure **LoRaWAN Network Server:**
   - **Server Type:** TTN or ChirpStack
   - **Server Address:** Your LNS address
   - **Port:** 1700 (default)
   - **Gateway EUI:** From network server registration

### Step 2: Register Devices in Network Server

#### If using The Things Network (TTN):
1. Create an **Application** in TTN Console
2. Note the **Application EUI** (AppEUI)
3. For each node:
   - Go to **End Devices → Add End Device**
   - Choose **Manually**
   - Enter:
     - **DevEUI:** Unique for each node (e.g., `00:11:22:33:44:55:66:77`)
     - **AppEUI:** Your application's AppEUI
     - **AppKey:** TTN will generate one (copy it!)
   - Set **Frequency Plan:** AS923-1
   - Set **LoRaWAN Version:** 1.0.3

#### If using ChirpStack:
1. Create an **Application** in ChirpStack
2. Note the **Application EUI** (AppEUI)
3. For each node:
   - Go to **Devices → Add**
   - Enter:
     - **Device EUI:** Unique for each node
     - **Application EUI:** Your application's AppEUI
     - **Application Key:** Generate or enter one
   - Set **Device Profile:** AS923-1

### Step 3: Update Node Code

Edit your node code (`heltec-lora32-v2/src/main.cpp` or `lilygo-lora32/src/main.cpp`):

```cpp
// Replace these with values from your network server
static const u1_t PROGMEM APPEUI[8] = { 
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00  // Your AppEUI from network server
};

static const u1_t PROGMEM DEVEUI[8] = { 
  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77  // Unique DevEUI for this node
};

static const u1_t PROGMEM APPKEY[16] = { 
  0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,  // Your AppKey from network server
  0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF
};
```

**Important:** 
- Use **hex format** (0x00, 0x11, etc.)
- **AppEUI** should be the same for all nodes in your application
- **DevEUI** must be unique for each node
- **AppKey** must match exactly what's in the network server

### Step 4: Configure Frequency Plan in Node Code

For **AS923-1 (Brazil)**, you may need to adjust the LMIC configuration:

```cpp
// In setup() function, after LMIC_reset():
// For AS923-1, you might need:
LMIC_selectSubBand(1);  // Select sub-band 1 for AS923-1
LMIC_setDrTxpow(DR_SF7, 14);  // SF7, 14dBm
```

Also check your `platformio.ini` - make sure it's configured for the right region:
```ini
build_flags = 
    -DCFG_as923=1  # For AS923 (not CFG_us915 or CFG_eu868)
```

---

## 🔍 How It Works

1. **Node broadcasts** LoRaWAN join request with:
   - DevEUI
   - AppEUI
   - Encrypted with AppKey

2. **Gateway receives** the packet (if on same frequency plan)

3. **Gateway forwards** to Network Server via internet

4. **Network Server validates:**
   - Checks if DevEUI is registered
   - Verifies AppEUI matches
   - Validates AppKey
   - Sends join accept back

5. **Node receives** join accept and becomes active

6. **Node sends data** packets periodically

7. **Gateway forwards** data packets to Network Server

8. **Network Server** routes data to your application

---

## ✅ Verification Checklist

- [ ] Gateway frequency plan set to **AS923-1**
- [ ] Gateway connected to Network Server (check gateway status)
- [ ] Devices registered in Network Server with:
  - [ ] Unique DevEUI for each node
  - [ ] Same AppEUI for all nodes
  - [ ] AppKey matches node code exactly
  - [ ] Frequency plan set to AS923-1
- [ ] Node code updated with:
  - [ ] Correct AppEUI
  - [ ] Unique DevEUI for each node
  - [ ] Correct AppKey
  - [ ] Frequency plan configuration (AS923-1)
- [ ] Node within range of gateway (check RSSI > -120 dBm)

---

## 🐛 Troubleshooting

### Node Not Joining
- **Check credentials match exactly** (no spaces, correct hex format)
- **Verify frequency plan** matches (AS923-1 everywhere)
- **Check gateway is online** and connected to network server
- **Verify node is in range** (check gateway logs for RSSI)

### Gateway Not Receiving Packets
- **Check gateway frequency plan** matches node
- **Verify gateway is connected** to network server
- **Check antenna connections** on both gateway and node
- **Verify distance** (should be < 10km in open area)

### Network Server Not Receiving Data
- **Check gateway internet connection**
- **Verify gateway is registered** in network server
- **Check gateway logs** for connection status
- **Verify server address and port** in gateway config

---

## 📊 Current Node Configuration

### Heltec Node (heltec-lora32-v2)
- **Current DevEUI:** `00:11:22:33:44:55:66:77` (needs to be unique)
- **Current AppEUI:** `00:00:00:00:00:00:00:00` (needs to match network server)
- **Current AppKey:** `00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF` (needs to match network server)

### LilyGo Node (lilygo-lora32)
- **Current DevEUI:** `AA:BB:CC:DD:EE:FF:00:11` (needs to be unique)
- **Current AppEUI:** `00:00:00:00:00:00:00:00` (needs to match network server)
- **Current AppKey:** `00:11:22:33:44:55:66:77:88:99:AA:BB:CC:DD:EE:FF` (needs to match network server)

**⚠️ These are placeholder values - you MUST replace them with values from your network server!**

---

## 🔗 Next Steps

1. **Set up your LoRaWAN Network Server** (TTN or ChirpStack)
2. **Register your gateway** in the network server
3. **Create an application** and register your devices
4. **Update node code** with the correct credentials
5. **Upload firmware** to your nodes
6. **Monitor** the network server dashboard for incoming data




