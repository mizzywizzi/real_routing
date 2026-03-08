Add-Type -AssemblyName System.Device
$watcher = New-Object System.Device.Location.GeoCoordinateWatcher
$watcher.Start()

$timeout = 10
$timer = [System.Diagnostics.Stopwatch]::StartNew()

while (($watcher.Status -ne 'Ready') -and ($timer.Elapsed.TotalSeconds -lt $timeout)) {
    Start-Sleep -Milliseconds 100
}

if ($watcher.Status -eq 'Ready') {
    $coord = $watcher.Position.Location
    if (-not $coord.IsUnknown) {
        Write-Output "$($coord.Latitude), $($coord.Longitude)"
    } else {
        Write-Error "Location unknown"
        exit 1
    }
} else {
    Write-Error "Location timeout or access denied"
    exit 1
}
