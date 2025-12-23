# Script to find Wisgate Edge Pro Gateway IP
$gatewayIPs = @("192.168.0.100", "192.168.0.150", "192.168.0.153", "192.168.0.154", "192.168.0.157", "192.168.0.161", "192.168.0.174")

Write-Host "Scanning for Wisgate Gateway..." -ForegroundColor Cyan
Write-Host ""

foreach ($ip in $gatewayIPs) {
    Write-Host "Testing $ip..." -ForegroundColor Yellow
    
    $ping = Test-Connection -ComputerName $ip -Count 1 -Quiet -ErrorAction SilentlyContinue
    if ($ping) {
        Write-Host "  Device is online" -ForegroundColor Green
        
        try {
            $response = Invoke-WebRequest -Uri "http://$ip" -TimeoutSec 3 -UseBasicParsing -ErrorAction Stop
            Write-Host "  Web interface found! Status: $($response.StatusCode)" -ForegroundColor Green
            Write-Host "  Access at: http://$ip" -ForegroundColor Cyan
            
            $content = $response.Content.ToLower()
            if ($content -match "wisgate|rak|gateway" -or $response.Headers.Server -match "wisgate|rak") {
                Write-Host ""
                Write-Host "*** THIS IS THE WISGATE GATEWAY! ***" -ForegroundColor Green -BackgroundColor DarkGreen
                Write-Host "Gateway IP: $ip" -ForegroundColor Yellow
                Write-Host ""
            }
        } catch {
            $statusCode = $_.Exception.Response.StatusCode.value__
            if ($statusCode) {
                Write-Host "  Web server responding (HTTP $statusCode) - Try: http://$ip" -ForegroundColor Yellow
            } else {
                Write-Host "  No web interface detected" -ForegroundColor Gray
            }
        }
    } else {
        Write-Host "  Device not responding" -ForegroundColor Red
    }
    Write-Host ""
}




