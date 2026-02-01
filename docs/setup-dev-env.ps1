# PlatformIO Development Environment Setup Script for Windows
# Run this script as Administrator for best results

Write-Host "========================================" -ForegroundColor Cyan
Write-Host "LoRa Development Environment Setup" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

# Check if running as Administrator
$isAdmin = ([Security.Principal.WindowsPrincipal] [Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "Warning: Not running as Administrator. Some steps may require elevation." -ForegroundColor Yellow
    Write-Host ""
}

# Step 1: Check Python
Write-Host "[1/6] Checking Python installation..." -ForegroundColor Green
$pythonInstalled = $false
$pythonCmd = Get-Command python -ErrorAction SilentlyContinue
if ($pythonCmd) {
    $pythonVersion = python --version 2>&1
    Write-Host "  [OK] Python found: $pythonVersion" -ForegroundColor Green
    $pythonInstalled = $true
} else {
    Write-Host "  [X] Python not found" -ForegroundColor Red
    Write-Host "  Please install Python 3.11+ from https://www.python.org/downloads/" -ForegroundColor Yellow
    Write-Host "  Make sure to check 'Add Python to PATH' during installation" -ForegroundColor Yellow
}
Write-Host ""

# Step 2: Check pip
if ($pythonInstalled) {
    Write-Host "[2/6] Checking pip..." -ForegroundColor Green
    $pipCmd = Get-Command pip -ErrorAction SilentlyContinue
    if ($pipCmd) {
        $pipVersion = pip --version 2>&1
        Write-Host "  [OK] pip found: $pipVersion" -ForegroundColor Green
    } else {
        Write-Host "  [X] pip not found" -ForegroundColor Red
        Write-Host "  Installing pip..." -ForegroundColor Yellow
        python -m ensurepip --upgrade
    }
    Write-Host ""
}

# Step 3: Install/Check PlatformIO
Write-Host "[3/6] Checking PlatformIO..." -ForegroundColor Green
$pioCmd = Get-Command pio -ErrorAction SilentlyContinue
if ($pioCmd) {
    $pioVersion = pio --version 2>&1
    Write-Host "  [OK] PlatformIO found: $pioVersion" -ForegroundColor Green
} else {
    Write-Host "  [X] PlatformIO not found" -ForegroundColor Red
    if ($pythonInstalled) {
        $install = Read-Host "  Install PlatformIO now? (Y/N)"
        if ($install -eq "Y" -or $install -eq "y") {
            Write-Host "  Installing PlatformIO..." -ForegroundColor Yellow
            pip install platformio
            Write-Host "  [OK] PlatformIO installed" -ForegroundColor Green
        }
    } else {
        Write-Host "  Cannot install PlatformIO without Python" -ForegroundColor Red
    }
}
Write-Host ""

# Step 4: Check VS Code
Write-Host "[4/6] Checking VS Code..." -ForegroundColor Green
$vscodePaths = @(
    "${env:ProgramFiles}\Microsoft VS Code\Code.exe",
    "${env:ProgramFiles(x86)}\Microsoft VS Code\Code.exe",
    "${env:LOCALAPPDATA}\Programs\Microsoft VS Code\Code.exe"
)
$vscodeFound = $false
foreach ($path in $vscodePaths) {
    if (Test-Path $path) {
        Write-Host "  [OK] VS Code found at: $path" -ForegroundColor Green
        $vscodeFound = $true
        break
    }
}
if (-not $vscodeFound) {
    Write-Host "  [X] VS Code not found" -ForegroundColor Yellow
    Write-Host "  Download from: https://code.visualstudio.com/" -ForegroundColor Yellow
    Write-Host "  After installation, install the PlatformIO IDE extension" -ForegroundColor Yellow
}
Write-Host ""

# Step 5: Check Docker
Write-Host "[5/6] Checking Docker..." -ForegroundColor Green
$dockerCmd = Get-Command docker -ErrorAction SilentlyContinue
if ($dockerCmd) {
    $dockerVersion = docker --version 2>&1
    Write-Host "  [OK] Docker found: $dockerVersion" -ForegroundColor Green
} else {
    Write-Host "  [~] Docker not found" -ForegroundColor Yellow
    Write-Host "  Docker is optional but useful for LoRaWAN server testing" -ForegroundColor Yellow
}
Write-Host ""

# Step 6: Check WSL
Write-Host "[6/6] Checking WSL..." -ForegroundColor Green
$wslCmd = Get-Command wsl -ErrorAction SilentlyContinue
if ($wslCmd) {
    $wslList = wsl --list --verbose 2>&1
    Write-Host "  [OK] WSL found:" -ForegroundColor Green
    Write-Host $wslList -ForegroundColor Gray
} else {
    Write-Host "  [~] WSL not found" -ForegroundColor Yellow
    Write-Host "  WSL is optional but can be useful for some tools" -ForegroundColor Yellow
}
Write-Host ""

# Summary
Write-Host "========================================" -ForegroundColor Cyan
Write-Host "Setup Summary" -ForegroundColor Cyan
Write-Host "========================================" -ForegroundColor Cyan
Write-Host ""

if ($pythonInstalled) {
    Write-Host "[OK] Python: Ready" -ForegroundColor Green
} else {
    Write-Host "[X] Python: Needs installation" -ForegroundColor Red
}

if ($vscodeFound) {
    Write-Host "[OK] VS Code: Ready" -ForegroundColor Green
    Write-Host "  -> Install PlatformIO IDE extension from Extensions marketplace" -ForegroundColor Yellow
} else {
    Write-Host "[X] VS Code: Needs installation" -ForegroundColor Yellow
}

Write-Host ""
Write-Host "Next Steps:" -ForegroundColor Cyan
Write-Host "1. Install USB drivers (CH340 or CP2102) for your ESP32 boards" -ForegroundColor White
Write-Host "2. Open project folder in VS Code: LoRa-River-Monitoring\heltec-lora32-v2" -ForegroundColor White
Write-Host "3. PlatformIO will auto-install ESP32 platform and libraries" -ForegroundColor White
Write-Host "4. Connect your board and upload firmware!" -ForegroundColor White
Write-Host ""
Write-Host "For detailed instructions, see: SETUP_DEV_ENVIRONMENT.md" -ForegroundColor Cyan
Write-Host ""

