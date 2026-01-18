# PlatformIO Terminal Commands

This guide provides common PlatformIO commands for the LilyGo LoRa32 project.

## Prerequisites

Ensure PlatformIO is installed and accessible via terminal. If `pio` command is not recognized, you may need to:
- Use `platformio` instead of `pio`
- Add PlatformIO to your PATH
- Or use PlatformIO through VS Code's integrated terminal

## Basic Commands

### Navigate to Project Directory
```powershell
cd "C:\Users\Klaus\Documents\Mestrado CA\lora-modules\lilygo-lora32"
```

### List Connected Devices
```powershell
pio device list
```
Shows all connected serial devices (COM ports on Windows).

### Build Project
```powershell
pio run
```
Compiles the project without uploading.

### Upload to Device
```powershell
pio run --target upload
```
Builds and uploads firmware to the connected board.

### Specify Upload Port (if auto-detection fails)
```powershell
pio run --target upload --upload-port COM9
```
Replace `COM9` with your actual COM port (check with `pio device list`).

### Monitor Serial Output
```powershell
pio device monitor --baud 115200
```
Opens serial monitor at 115200 baud (matches platformio.ini configuration).

### Monitor and Auto-Connect (with filters)
```powershell
pio device monitor --baud 115200 --filter default
```

### Upload and Monitor (Combined)
```powershell
pio run --target upload && pio device monitor --baud 115200
```
Note: PowerShell may require `;` instead of `&&`:
```powershell
pio run --target upload; pio device monitor --baud 115200
```

### Clean Build Files
```powershell
pio run --target clean
```
Removes compiled files for a fresh build.

### Show Project Information
```powershell
pio project config
```
Displays project configuration.

## Quick Workflow

### Typical Development Cycle

1. **Build and Upload:**
   ```powershell
   cd lilygo-lora32
   pio run --target upload
   ```

2. **Monitor Serial Output:**
   ```powershell
   pio device monitor --baud 115200
   ```

3. **Or Combined (if supported):**
   ```powershell
   pio run --target upload; pio device monitor --baud 115200
   ```

## Troubleshooting

### Port Already in Use
If you get "port is busy" error:
- Close any other serial monitors (VS Code, Arduino IDE, etc.)
- Close and reopen the terminal
- Try specifying the port explicitly: `--upload-port COM9`

### Upload Fails
- Check that the board is connected
- Verify the correct COM port with `pio device list`
- Ensure board is in upload mode (some boards require button press)
- Try lowering upload speed in `platformio.ini`

### Serial Monitor Shows Garbage
- Verify baud rate matches code (should be 115200)
- Check `monitor_speed` in `platformio.ini`
- Ensure correct COM port is selected

## Current Payload Format

**Expected Binary Payload (Current Implementation):**
- 8-byte binary `SensorPayload` struct
- Structure: `sensorType` (1), `distanceMm` (2), `signalStrength` (2), `temperature` (1), `batteryPercent` (1), `readingCount` (1)

**Legacy JSON Payload (Old Code):**
- JSON string: `{"level":324.67,"packet":2}`
- If you see this format, the old code is still running - re-upload the firmware

## Notes

- Serial monitor baud rate: **115200** (configured in `platformio.ini`)
- Upload baud rate: **115200** (configured in `platformio.ini`)
- Board: **ttgo-t-beam** (LilyGo T-Beam AXP2101 v1.2)
