# Quick Setup Steps

Based on the environment check, here's what you need to install:

## ✅ What You Already Have
- Docker Desktop (installed)
- WSL with Ubuntu (installed)

## 📦 What You Need to Install

### 1. Python 3.11+ (REQUIRED)

**Download and Install:**
1. Go to: https://www.python.org/downloads/
2. Download Python 3.11 or newer
3. **IMPORTANT**: During installation, check ✅ **"Add Python to PATH"**
4. Choose "Install Now" or "Customize installation"
5. Verify installation:
   ```powershell
   python --version
   pip --version
   ```

### 2. VS Code (RECOMMENDED)

**Download and Install:**
1. Go to: https://code.visualstudio.com/
2. Download and install VS Code
3. **IMPORTANT**: Check ✅ "Add to PATH" during installation
4. After installation, open VS Code

**Install PlatformIO Extension:**
1. In VS Code, press `Ctrl+Shift+X` (Extensions)
2. Search for "PlatformIO IDE"
3. Click "Install" on the official PlatformIO IDE extension
4. Wait for installation (this installs PlatformIO Core automatically)

### 3. USB Drivers (REQUIRED for boards)

Your ESP32 boards need USB-to-Serial drivers:

**Option A: CH340 Driver (Most Common)**
- Download: https://github.com/WCHSoftGroup/ch34xser_driver/releases
- Or: https://www.wch.cn/downloads/CH341SER_EXE.html
- Run the installer

**Option B: CP2102 Driver (Alternative)**
- Download: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers
- Run the installer

**To determine which driver you need:**
1. Connect your ESP32 board via USB
2. Open Device Manager (Win+X → Device Manager)
3. Look under "Ports (COM & LPT)" or "Other devices"
4. If you see "CH340" → Install CH340 driver
5. If you see "CP2102" or "Silicon Labs" → Install CP2102 driver

### 4. Verify Installation

Run the setup script again:
```powershell
cd LoRa-River-Monitoring
powershell -ExecutionPolicy Bypass -File setup-dev-env.ps1
```

You should see:
- ✅ Python: Ready
- ✅ VS Code: Ready
- ✅ PlatformIO: Ready (after VS Code extension installs)

## 🚀 Next Steps After Installation

### 1. Open Your Project in VS Code

```powershell
cd LoRa-River-Monitoring\heltec-lora32-v2
code .
```

Or for LilyGo:
```powershell
cd LoRa-River-Monitoring\lilygo-lora32
code .
```

### 2. PlatformIO Will Auto-Setup

When you open the project:
- PlatformIO will detect `platformio.ini`
- It will download ESP32 platform automatically
- It will install required libraries automatically
- This may take a few minutes the first time

### 3. Connect Your Board

1. Connect ESP32 board via USB
2. Check Device Manager for COM port (e.g., COM3, COM4)
3. In PlatformIO, go to: Platform → Devices
4. Your board should appear in the list

### 4. Build and Upload

1. **Build**: Click ✓ (checkmark) in PlatformIO toolbar
2. **Upload**: Click → (arrow) in PlatformIO toolbar
3. **Monitor**: Click 🔌 (plug) in PlatformIO toolbar

## 🐳 Optional: Docker Setup for LoRaWAN Server

If you want to run a local LoRaWAN Network Server for testing:

```powershell
cd LoRa-River-Monitoring
docker-compose up -d
```

This starts:
- ChirpStack LoRaWAN Network Server (http://localhost:8080)
- PostgreSQL database
- Redis cache

## 📚 Additional Resources

- **Full Setup Guide**: See `SETUP_DEV_ENVIRONMENT.md`
- **Quick Start**: See `QUICK_START.md`
- **Gateway Config**: See `GATEWAY_CONFIG.md`

## 🆘 Troubleshooting

### Python Not Found After Installation
- Restart your terminal/VS Code
- Verify PATH: `python --version`
- If still not found, manually add Python to PATH:
  - Search "Environment Variables" in Windows
  - Edit "Path" variable
  - Add: `C:\Users\<YourUser>\AppData\Local\Programs\Python\Python3XX\`
  - Add: `C:\Users\<YourUser>\AppData\Local\Programs\Python\Python3XX\Scripts\`

### Board Not Detected
- Check USB cable (must support data, not just power)
- Install correct USB driver (CH340 or CP2102)
- Try different USB port
- Check Device Manager for COM port

### PlatformIO Extension Not Installing
- Check internet connection
- Restart VS Code
- Try installing PlatformIO Core manually:
  ```powershell
  pip install platformio
  ```

---

**Ready to start coding!** 🎉








