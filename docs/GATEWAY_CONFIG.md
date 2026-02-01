# Wisgate Edge Pro Configuration Guide

This guide explains how to configure the Wisgate Edge Pro gateway to receive data from the Heltec and LilyGo LoRa nodes.

## 🔧 Initial Setup

### 1. Access Gateway Web Interface

**Current Gateway IP: `192.168.0.165`**

1. Connect the Wisgate Edge Pro to your network via Ethernet or WiFi
2. Find the gateway IP address (check router DHCP table or use RAK's discovery tool)
3. Open a web browser and navigate to: `https://192.168.0.165`
4. Default credentials:
   - Username: `admin`
   - Password: `admin` (change on first login)

### 2. Basic Network Configuration

1. Navigate to **Network** → **WAN**
2. Configure:
   - **Connection Type:** Static IP, DHCP, or PPPoE (as per your network)
   - **IP Address:** Set static IP if needed
   - **Gateway:** Your router's IP
   - **DNS:** Your DNS server (e.g., 8.8.8.8)

3. Navigate to **Network** → **LoRa**
4. Configure:
   - **Frequency Plan:** Select your region
     - EU868 (Europe)
     - US915 (United States)
     - AS923 (Asia)
     - AU915 (Australia)
   - **Server Address:** Your LoRaWAN Network Server (LNS) address
     - For TTN: `eu1.cloud.thethings.network` (EU) or `nam1.cloud.thethings.network` (US)
     - For ChirpStack: Your server IP
   - **Port:** 1700 (default)
   - **Gateway ID:** Note this value (will be used in network server)

### 3. LoRaWAN Network Server Setup

#### Option A: The Things Network (TTN)

1. Create account at [console.thethings.network](https://console.thethings.network)
2. Register your gateway:
   - Go to **Gateways** → **Add Gateway**
   - Enter Gateway ID (from gateway web interface)
   - Set frequency plan
   - Enter gateway coordinates
   - Copy **Gateway EUI**

3. In Wisgate Edge Pro:
   - Navigate to **LoRa** → **LoRaWAN**
   - Set **Server Type:** TTN
   - Enter **Gateway EUI**
   - Set **Server Address** and **Port**

#### Option B: ChirpStack

1. Install ChirpStack on your server (or use cloud instance)
2. In ChirpStack, create gateway:
   - Gateway ID: Match Wisgate Gateway ID
   - Frequency plan: Match gateway setting
3. In Wisgate Edge Pro:
   - Set **Server Type:** ChirpStack
   - Enter **Server Address** (ChirpStack server IP)
   - Set **Port:** 1700

### 4. Device Registration

#### For TTN:

1. Create Application:
   - Go to **Applications** → **Add Application**
   - Note the **Application ID**

2. Register Devices:
   - Go to your Application → **End Devices** → **Add End Device**
   - Choose **Manually**
   - Enter:
     - **DevEUI:** From node code (e.g., `00:11:22:33:44:55:66:77`)
     - **AppEUI:** From node code (e.g., `00:00:00:00:00:00:00:00`)
     - **AppKey:** From node code (keep secure!)
   - Set **Frequency Plan** to match gateway
   - Set **LoRaWAN Version:** 1.0.3
   - Set **Regional Parameters Revision:** A or B

#### For ChirpStack:

1. Create Application:
   - Go to **Applications** → **Add**
   - Enter Application name

2. Register Devices:
   - Go to Application → **Devices** → **Add**
   - Enter:
     - **Device EUI:** From node code
     - **Application EUI:** From node code
     - **Application Key:** From node code
   - Set **Device Profile:** Select appropriate profile

### 5. Gateway Verification

1. Check Gateway Status:
   - In Wisgate web interface: **Status** → **LoRa**
   - Verify **Status:** Connected
   - Check **Uplink/Downlink** counters

2. Test Connection:
   - Power on your LoRa node
   - Watch for join requests in network server
   - Verify data packets are received

## 📊 Monitoring

### Gateway Dashboard
- **Status** → **Overview:** System status, uptime, memory
- **Status** → **LoRa:** LoRa statistics, packet counters
- **Status** → **Network:** Network interface status

### Network Server Dashboard
- View device data in real-time
- Check packet delivery rates
- Monitor signal quality (RSSI, SNR)

## 🔍 Troubleshooting

### Gateway Not Connecting to Network Server

1. **Check Network:**
   - Verify gateway has internet access
   - Ping server address from gateway
   - Check firewall rules

2. **Verify Configuration:**
   - Server address and port correct
   - Gateway ID matches network server registration
   - Frequency plan matches

3. **Check Logs:**
   - Gateway: **Status** → **System Log**
   - Network Server: Check application logs

### Devices Not Joining

1. **Verify Device Credentials:**
   - DevEUI, AppEUI, AppKey match exactly
   - No extra spaces or formatting errors
   - Use hex format (00:11:22:...)

2. **Check Frequency Plan:**
   - Must match between gateway, network server, and device

3. **Signal Quality:**
   - Check RSSI values (should be > -120 dBm)
   - Verify antenna connections
   - Check distance to gateway

### No Data Received

1. **Verify Join Success:**
   - Check network server for join accept messages
   - Device should show "joined" status

2. **Check Transmission:**
   - Verify device is sending data
   - Check serial monitor on device
   - Verify transmission interval

3. **Gateway Status:**
   - Check uplink packet counter
   - Verify gateway is forwarding to network server

## 📝 Important Notes

- **Security:** Always change default passwords
- **Frequency Regulations:** Ensure you comply with local regulations
- **Gateway Location:** Place gateway in elevated position for better coverage
- **Backup Configuration:** Export gateway configuration for backup
- **Firmware Updates:** Keep gateway firmware updated

## 🔗 Useful Links

- [Wisgate Edge Pro User Manual](https://docs.rakwireless.com/Product-Categories/WisGate/RAK7249/Overview/)
- [The Things Network Documentation](https://www.thethingsnetwork.org/docs/)
- [ChirpStack Documentation](https://www.chirpstack.io/docs/)
- [LoRaWAN Regional Parameters](https://lora-alliance.org/lorawan-for-developers/)



