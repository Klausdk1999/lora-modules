# Script to test Ethernet connection for Wisgate Edge Pro Gateway
# Run this script to check if Ethernet cable is working

Write-Host "=== Ethernet Connection Test ===" -ForegroundColor Cyan
Write-Host ""

# Check Ethernet adapter status
Write-Host "1. Checking Ethernet adapter status..." -ForegroundColor Yellow
$ethernet = Get-NetAdapter | Where-Object {$_.Name -like "*Ethernet*" -and $_.InterfaceDescription -notlike "*Virtual*" -and $_.InterfaceDescription -notlike "*Hyper*"}
if ($ethernet) {
    Write-Host "   Adapter Name: $($ethernet.Name)" -ForegroundColor White
    Write-Host "   Status: $($ethernet.Status)" -ForegroundColor $(if ($ethernet.Status -eq "Up") { "Green" } else { "Red" })
    Write-Host "   Link Speed: $($ethernet.LinkSpeed)" -ForegroundColor White
    
    if ($ethernet.Status -eq "Up") {
        Write-Host "   ✓ Ethernet adapter is UP" -ForegroundColor Green
    } else {
        Write-Host "   ✗ Ethernet adapter is DOWN" -ForegroundColor Red
        Write-Host "   → Check if cable is properly connected" -ForegroundColor Yellow
    }
} else {
    Write-Host "   ✗ No Ethernet adapter found" -ForegroundColor Red
}

Write-Host ""

# Check for IP address on Ethernet
Write-Host "2. Checking for IP address on Ethernet interface..." -ForegroundColor Yellow
$ethernetIP = Get-NetIPAddress | Where-Object {$_.InterfaceAlias -like "*Ethernet*" -and $_.InterfaceAlias -notlike "*Virtual*" -and $_.IPAddress -notlike "169.254.*"}
if ($ethernetIP) {
    Write-Host "   ✓ Ethernet has IP address: $($ethernetIP.IPAddress)" -ForegroundColor Green
    Write-Host "   Subnet: $($ethernetIP.IPAddress)/$($ethernetIP.PrefixLength)" -ForegroundColor White
} else {
    Write-Host "   ✗ No IP address assigned to Ethernet" -ForegroundColor Red
    Write-Host "   → This could mean:" -ForegroundColor Yellow
    Write-Host "     - Cable is not connected" -ForegroundColor Yellow
    Write-Host "     - DHCP is not working" -ForegroundColor Yellow
    Write-Host "     - Static IP needs to be configured" -ForegroundColor Yellow
}

Write-Host ""

# Test connectivity to common gateway IPs
Write-Host "3. Testing connectivity to potential gateway IPs..." -ForegroundColor Yellow
$testIPs = @("192.168.0.1", "192.168.0.100", "192.168.0.161", "192.168.4.1", "10.0.0.1")
foreach ($ip in $testIPs) {
    $result = Test-Connection -ComputerName $ip -Count 1 -Quiet -ErrorAction SilentlyContinue
    if ($result) {
        Write-Host "   ✓ $ip is reachable" -ForegroundColor Green
        try {
            $web = Invoke-WebRequest -Uri "http://$ip" -TimeoutSec 2 -UseBasicParsing -ErrorAction SilentlyContinue
            Write-Host "     → Web interface available at http://$ip" -ForegroundColor Cyan
        } catch {
            Write-Host "     → Device responding but no web interface" -ForegroundColor Gray
        }
    }
}

Write-Host ""
Write-Host "=== Recommendations ===" -ForegroundColor Cyan
Write-Host ""
Write-Host "To enable Ethernet connection:" -ForegroundColor Yellow
Write-Host "1. Ensure Ethernet cable is properly connected to:" -ForegroundColor White
Write-Host "   - Gateway Ethernet port" -ForegroundColor White
Write-Host "   - Router/switch Ethernet port" -ForegroundColor White
Write-Host ""
Write-Host "2. Check LED indicators on Ethernet ports:" -ForegroundColor White
Write-Host "   - Link LED should be solid (usually green/amber)" -ForegroundColor White
Write-Host "   - Activity LED should blink when data is transmitted" -ForegroundColor White
Write-Host ""
Write-Host "3. In Gateway web interface:" -ForegroundColor White
Write-Host "   - Go to Network → WAN" -ForegroundColor White
Write-Host "   - Ensure Connection Type is set to 'DHCP' or 'Static IP'" -ForegroundColor White
Write-Host "   - Check that Ethernet interface is enabled" -ForegroundColor White
Write-Host ""
Write-Host "4. For Brazil, change frequency plan from AU915 to AS923-1" -ForegroundColor Yellow
Write-Host "   - Go to Network → LoRa → Frequency Plan" -ForegroundColor White
Write-Host "   - Select: AS923-1 (Brazil uses this plan)" -ForegroundColor White





