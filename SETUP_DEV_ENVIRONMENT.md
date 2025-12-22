# Development Environment Setup Guide

This guide will help you set up a complete development environment for working with the LoRa river monitoring project, including PlatformIO, ESP32 toolchains, and gateway configuration tools.

## 📋 Prerequisites Check

Before starting, verify you have:
- Windows 10/11
- WSL 2 with Ubuntu (already installed)
- Docker Desktop (already installed)
- Administrator privileges for some installations

## 🚀 Setup Options

You have two main options:
1. **Option A: VS Code + PlatformIO Extension** (Recommended - Easiest)
2. **Option B: PlatformIO CLI + Docker** (More control, Docker-based)

---

## Option A: VS Code + PlatformIO Extension (Recommended)

### Step 1: Install VS Code

1. Download VS Code from: https://code.visualstudio.com/
2. Install with default settings
3. Add to PATH during installation (check the option)

### Step 2: Install PlatformIO Extension

1. Open VS Code
2. Go to Extensions (Ctrl+Shift+X)
3. Search for "PlatformIO IDE"
4. Click "Install" on the official PlatformIO IDE extension
5. Wait for installation (this will also install PlatformIO Core)

### Step 3: Install Python (Required for PlatformIO)

PlatformIO requires Python. Install it:

**Option 3a: Install Python directly on Windows**
1. Download Python 3.11+ from: https://www.python.org/downloads/
2. **IMPORTANT**: Check "Add Python to PATH" during installation
3. Verify installation:
   ```powershell
   python --version
   pip --version
   ```

**Option 3b: Use Python from WSL (Alternative)**
If you prefer using WSL Python, PlatformIO can use it, but Windows installation is simpler.

### Step 4: Install USB Drivers

For ESP32 boards, you'll need USB-to-Serial drivers:

1. **CH340 Driver** (common on many ESP32 boards):
   - Download: https://github.com/WCHSoftGroup/ch34xser_driver
   - Or: https://www.wch.cn/downloads/CH341SER_EXE.html
   - Install the driver

2. **CP2102 Driver** (alternative):
   - Download: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
   - Install if your board uses CP2102

### Step 5: Verify PlatformIO Installation

1. Open VS Code
2. Open PlatformIO Home (click the PlatformIO icon in the sidebar)
3. Or open a terminal in VS Code and run:
   ```bash
   pio --version
   ```

### Step 6: Open Your Project

1. In VS Code: File → Open Folder
2. Navigate to: `LoRa-River-Monitoring/heltec-lora32-v2` or `lilygo-lora32`
3. PlatformIO will automatically detect the project and install dependencies

---

## Option B: PlatformIO CLI + Docker (Advanced)

### Step 1: Install Python on Windows

1. Download Python 3.11+ from: https://www.python.org/downloads/
2. **Check "Add Python to PATH"**
3. Verify:
   ```powershell
   python --version
   pip --version
   ```

### Step 2: Install PlatformIO Core

```powershell
# Install PlatformIO Core
pip install platformio

# Verify installation
pio --version
```

### Step 3: Install USB Drivers

Same as Option A, Step 4.

### Step 4: Docker Setup (Optional - for isolated builds)

Create a Docker-based development environment:

```powershell
# Navigate to project directory
cd LoRa-River-Monitoring

# Use the provided docker-compose.yml (see below)
docker-compose up -d
```

---

## 🔧 Additional Tools

### 1. Serial Port Monitor

PlatformIO includes a serial monitor, but you can also use:
- **PuTTY** (Windows): https://www.putty.org/
- **Tera Term**: https://ttssh2.osdn.jp/
- **PlatformIO Serial Monitor** (built-in, recommended)

### 2. LoRaWAN Network Server (for Gateway)

For testing and development, you can run a local LoRaWAN Network Server:

**Option A: ChirpStack (Docker)**
```powershell
# Clone ChirpStack Docker Compose
git clone https://github.com/chirpstack/chirpstack-docker.git
cd chirpstack-docker
docker-compose up -d
```

**Option B: The Things Network (Cloud - Free)**
- Sign up at: https://console.thethingsnetwork.org/
- No local installation needed

### 3. Gateway Configuration Tools

The Wisgate Edge Pro uses a web interface, but you may need:
- **Network tools**: To find gateway IP address
- **Web browser**: For gateway configuration
- **SSH client** (optional): For advanced gateway configuration
  - **PuTTY**: https://www.putty.org/
  - **Windows Terminal**: Built into Windows 11

---

## 📦 Project Setup

### First Time Setup

1. **Open Project in VS Code**:
   ```powershell
   cd LoRa-River-Monitoring\heltec-lora32-v2
   code .
   ```

2. **PlatformIO will automatically**:
   - Download ESP32 platform
   - Install required libraries
   - Set up build environment

3. **Install Libraries Manually** (if needed):
   ```powershell
   pio lib install
   ```

### Verify Board Connection

1. Connect your board via USB
2. Check COM port in Device Manager (Windows)
3. In PlatformIO, go to: Platform → Devices
4. You should see your board listed

### Build and Upload

1. **Build project**:
   - Click the checkmark icon (✓) in PlatformIO toolbar
   - Or: `pio run`

2. **Upload to board**:
   - Click the arrow icon (→) in PlatformIO toolbar
   - Or: `pio run -t upload`

3. **Monitor serial output**:
   - Click the plug icon (🔌) in PlatformIO toolbar
   - Or: `pio device monitor`

---

## 🐳 Docker Development Environment (Optional)

If you prefer a containerized environment, use the provided `docker-compose.yml`:

```powershell
# Start development environment
docker-compose up -d

# Access container
docker exec -it platformio-dev bash

# Inside container, you can use PlatformIO
pio --version
```

---

## 🔍 Troubleshooting

### PlatformIO Not Found

```powershell
# Add PlatformIO to PATH manually
# PlatformIO is usually installed at:
# C:\Users\<YourUser>\.platformio\penv\Scripts

# Add to PATH in Windows Environment Variables
```

### Board Not Detected

1. Check USB cable (must support data, not just power)
2. Install correct USB driver (CH340 or CP2102)
3. Check Device Manager for COM port
4. Try different USB port

### Python Not Found

1. Reinstall Python with "Add to PATH" checked
2. Restart terminal/VS Code after installation
3. Verify: `python --version`

### Library Installation Fails

```powershell
# Clear PlatformIO cache
pio cache clean

# Reinstall libraries
pio lib install
```

### Build Errors

1. Check `platformio.ini` configuration
2. Verify board type matches your hardware
3. Check library versions
4. Review error messages for missing dependencies

---

## 📚 Next Steps

After setup is complete:

1. **Configure LoRaWAN credentials** in `src/main.cpp`
2. **Set up gateway** (see `GATEWAY_CONFIG.md`)
3. **Test sensor connection** (see `QUICK_START.md`)
4. **Build and upload** firmware to boards

---

## 🆘 Getting Help

- **PlatformIO Documentation**: https://docs.platformio.org/
- **ESP32 Documentation**: https://docs.espressif.com/
- **LoRaWAN Documentation**: https://lora-alliance.org/
- **Project Issues**: Check project README files

---

## ✅ Verification Checklist

- [ ] VS Code installed (or PlatformIO CLI)
- [ ] PlatformIO extension/CLI installed
- [ ] Python installed and in PATH
- [ ] USB drivers installed (CH340/CP2102)
- [ ] Board detected in Device Manager
- [ ] Project opens in PlatformIO
- [ ] Libraries install successfully
- [ ] Build completes without errors
- [ ] Serial monitor works

---

**Ready to code!** 🚀




