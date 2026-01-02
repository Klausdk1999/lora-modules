# Gateway Log Level Configuration Guide

## Recommended Log Levels for Debugging

### For Troubleshooting (When Not Receiving Data)
**Recommended: `INFO` or `DEBUG`**

- **INFO**: Shows general operational messages, packet reception, transmission status
- **DEBUG**: Most verbose, shows detailed packet information, radio operations

### For Normal Operation
**Recommended: `WARNING` or `ERROR`**

- **WARNING**: Only shows potential problems
- **ERROR**: Only shows critical errors

---

## How to Configure Log Levels on Wisgate Edge Pro

### Option 1: Web Interface (Recommended)

1. **Access Gateway Web Interface**
   - Navigate to: `http://<gateway-ip>`
   - Login with admin credentials

2. **Find Log Settings**
   - Navigate to: **System** → **Logging** (or **Status** → **System Log**)
   - Look for **Log Level** setting

3. **Set Log Level**
   - For debugging: Select **INFO** or **DEBUG**
   - For normal operation: Select **WARNING**

4. **Apply Changes**
   - Click **Save** or **Apply**
   - Gateway may restart logging service

### Option 2: SSH/Terminal Access (Advanced)

If you have SSH access to the gateway:

1. **Connect via SSH**
   ```bash
   ssh root@<gateway-ip>
   # Default password: often "admin" or check documentation
   ```

2. **Find Packet Forwarder Config**
   ```bash
   # Usually located at:
   /etc/chirpstack-gateway-bridge/chirpstack-gateway-bridge.toml
   # OR
   /etc/lorawan-gateway-bridge/lorawan-gateway-bridge.toml
   # OR for Semtech Packet Forwarder:
   /opt/ttn-gateway/packet_forwarder/lora_pkt_fwd/global_conf.json
   ```

3. **Edit Configuration**
   ```bash
   # For TOML config:
   nano /etc/chirpstack-gateway-bridge/chirpstack-gateway-bridge.toml
   
   # Look for log_level setting:
   [logging]
   level = "info"  # Options: "debug", "info", "warn", "error"
   ```

4. **Restart Service**
   ```bash
   # Restart packet forwarder service
   systemctl restart lorawan-gateway-bridge
   # OR
   systemctl restart chirpstack-gateway-bridge
   # OR
   systemctl restart ttn-gateway
   ```

---

## Log Levels Explained

### DEBUG
- **Use When**: Deep troubleshooting, packet inspection
- **Shows**: Every packet detail, radio operations, internal states
- **Example Messages**:
  - Raw packet data
  - Radio configuration changes
  - Detailed timing information
  - Internal state transitions

### INFO (Recommended for Debugging)
- **Use When**: Normal troubleshooting, monitoring operations
- **Shows**: Packet reception/transmission, gateway status, connection status
- **Example Messages**:
  - "Received uplink packet from device X"
  - "Forwarding packet to server"
  - "Gateway connected to server"
  - Packet counters

### WARNING
- **Use When**: Normal operation, want to see issues
- **Shows**: Potential problems, but not detailed operations
- **Example Messages**:
  - Connection timeouts
  - Configuration mismatches
  - Resource warnings

### ERROR
- **Use When**: Production, minimal logging
- **Shows**: Only critical failures
- **Example Messages**:
  - Service crashes
  - Fatal configuration errors
  - Hardware failures

---

## What to Look For in Logs When Debugging

### 1. Packet Reception Logs (INFO/DEBUG)
Look for messages like:
```
[INFO] Received uplink packet
[INFO] RXPK: frequency=915.2, datr=SF7BW125, rssi=-85, lsnr=9.5
[INFO] Forwarding packet to server
```

### 2. Server Connection Logs
Look for:
```
[INFO] Connected to server: eu1.cloud.thethings.network:1700
[INFO] Gateway status: online
```

### 3. Error Messages
Watch for:
```
[ERROR] Failed to connect to server
[WARN] Packet dropped: invalid format
[ERROR] Frequency plan mismatch
```

### 4. Packet Forwarding Status
Look for:
```
[INFO] Packet forwarded successfully
[INFO] Server acknowledged packet
[WARN] Packet forwarding timeout
```

---

## Accessing Gateway Logs

### Via Web Interface

1. **Navigate to Logs**
   - **Status** → **System Log**
   - OR **System** → **Logging** → **View Logs**

2. **Filter Logs**
   - Filter by log level (INFO, DEBUG, WARNING, ERROR)
   - Filter by service (LoRa, Network, System)
   - Search for keywords

3. **Export Logs**
   - Download log file for analysis
   - Copy relevant log entries

### Via SSH (If Available)

```bash
# View real-time logs
journalctl -u lorawan-gateway-bridge -f

# View last 100 lines
journalctl -u lorawan-gateway-bridge -n 100

# View logs with timestamps
journalctl -u lorawan-gateway-bridge --since "1 hour ago"

# Filter by log level
journalctl -u lorawan-gateway-bridge -p info

# Search for specific patterns
journalctl -u lorawan-gateway-bridge | grep "uplink"
journalctl -u lorawan-gateway-bridge | grep "error"
```

---

## Specific Settings for Your Current Issue

### Since You're Not Receiving Data on TTN:

1. **Set Log Level to INFO or DEBUG**
   - This will show if packets are being received at all

2. **Check These Log Sections**:
   - **LoRa Packet Reception**: Look for "RXPK" or "uplink" messages
   - **Packet Forwarding**: Look for "forward" or "PUSH_DATA" messages
   - **Server Connection**: Verify connection to TTN server
   - **Frequency Plan**: Check if frequency matches your device

3. **Key Indicators to Look For**:
   ```
   ✅ Good: "Received uplink packet" → "Forwarding to server" → "Server acknowledged"
   ❌ Problem: "Received uplink packet" → but no "Forwarding to server"
   ❌ Problem: "Frequency plan mismatch"
   ❌ Problem: "Server connection timeout"
   ❌ Problem: No "Received uplink packet" messages at all
   ```

---

## Log Location on Different Gateway Types

### Wisgate Edge Pro (RAK)
- **Web Interface**: Status → System Log
- **Config File**: Usually in `/etc/` directory
- **Service**: `lorawan-gateway-bridge` or similar

### ChirpStack Gateway Bridge
- **Config**: `/etc/chirpstack-gateway-bridge/chirpstack-gateway-bridge.toml`
- **Logs**: `journalctl -u chirpstack-gateway-bridge`

### Semtech Packet Forwarder (TTN)
- **Config**: `/opt/ttn-gateway/packet_forwarder/lora_pkt_fwd/global_conf.json`
- **Logs**: Usually in `/var/log/` or via systemd journal

---

## Quick Debugging Checklist

1. ✅ Set log level to **INFO** or **DEBUG**
2. ✅ Restart gateway service
3. ✅ Send test packet from device
4. ✅ Check logs for "Received uplink packet" messages
5. ✅ Check logs for "Forwarding to server" messages
6. ✅ Check logs for any ERROR or WARNING messages
7. ✅ Verify server connection status in logs
8. ✅ Check frequency plan in logs matches device

---

## Notes

- **Performance Impact**: DEBUG level can generate lots of logs, use sparingly
- **Log Rotation**: Logs may rotate automatically, check log retention settings
- **Storage**: DEBUG logs can fill up storage quickly, monitor disk space
- **Security**: Logs may contain sensitive information (device IDs, etc.)

---

## Example: What Good Logs Look Like

```
[INFO] Gateway started
[INFO] Connecting to server: eu1.cloud.thethings.network:1700
[INFO] Connected to server successfully
[INFO] Gateway ID: AC1F09FFFE18A833
[INFO] Frequency plan: AU915
[INFO] Received uplink packet
[INFO] RXPK: freq=915.2, datr=SF7BW125, rssi=-85, lsnr=9.5, size=23
[INFO] Forwarding packet to server
[INFO] Server acknowledged packet
[INFO] Packet forwarded successfully
```

---

## If Logs Don't Show Packet Reception

If you don't see any "Received uplink packet" messages:

1. **Check Device**: Verify device is actually transmitting (Serial Monitor)
2. **Check Range**: Device may be too far from gateway
3. **Check Antennas**: Both gateway and device antennas connected
4. **Check Frequency**: Verify frequency plan matches between device and gateway
5. **Check Radio**: Gateway radio may not be working (check hardware)


