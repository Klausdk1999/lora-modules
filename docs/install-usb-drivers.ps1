# USB Driver Installation Helper Script
# This script helps you download and install USB drivers for ESP32 boards

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "ESP32 USB Driver Installation Helper" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

Write-Host "ESP32 boards typically use one of these USB-to-Serial chips:" -ForegroundColor Yellow
Write-Host "1. CH340/CH341 (Most common)" -ForegroundColor White
Write-Host "2. CP2102/CP2104 (Alternative)" -ForegroundColor White
Write-Host ""

# Check which driver is needed
Write-Host "To determine which driver you need:" -ForegroundColor Cyan
Write-Host "1. Connect your ESP32 board via USB" -ForegroundColor White
Write-Host "2. Open Device Manager (Win+X → Device Manager)" -ForegroundColor White
Write-Host "3. Look under 'Ports (COM & LPT)' or 'Other devices'" -ForegroundColor White
Write-Host "4. If you see 'USB-SERIAL CH340' or similar → Need CH340 driver" -ForegroundColor White
Write-Host "5. If you see 'CP2102' or 'Silicon Labs' → Need CP2102 driver" -ForegroundColor White
Write-Host ""

$choice = Read-Host "Which driver do you want to download? (1=CH340, 2=CP2102, 3=Both, 0=Skip)"

if ($choice -eq "1" -or $choice -eq "3") {
    Write-Host ""
    Write-Host "CH340 Driver Download:" -ForegroundColor Green
    Write-Host "Official: https://www.wch.cn/downloads/CH341SER_EXE.html" -ForegroundColor Cyan
    Write-Host "GitHub: https://github.com/WCHSoftGroup/ch34xser_driver" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Opening download page..." -ForegroundColor Yellow
    Start-Process "https://github.com/WCHSoftGroup/ch34xser_driver/releases"
}

if ($choice -eq "2" -or $choice -eq "3") {
    Write-Host ""
    Write-Host "CP2102 Driver Download:" -ForegroundColor Green
    Write-Host "Official: https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers" -ForegroundColor Cyan
    Write-Host ""
    Write-Host "Opening download page..." -ForegroundColor Yellow
    Start-Process "https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers"
}

Write-Host ""
Write-Host "After downloading:" -ForegroundColor Cyan
Write-Host "1. Extract the driver files" -ForegroundColor White
Write-Host "2. Run the installer (usually SETUP.EXE)" -ForegroundColor White
Write-Host "3. Restart your computer if prompted" -ForegroundColor White
Write-Host "4. Reconnect your ESP32 board" -ForegroundColor White
Write-Host "5. Check Device Manager - you should see a COM port" -ForegroundColor White
Write-Host ""

Write-Host "To verify installation:" -ForegroundColor Cyan
Write-Host "1. Open Device Manager" -ForegroundColor White
Write-Host "2. Connect your ESP32 board" -ForegroundColor White
Write-Host "3. Look for 'USB-SERIAL CH340' or 'Silicon Labs CP210x' under 'Ports (COM & LPT)'" -ForegroundColor White
Write-Host "4. Note the COM port number (e.g., COM3, COM4)" -ForegroundColor White
Write-Host ""



