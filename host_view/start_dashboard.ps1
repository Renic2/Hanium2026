[CmdletBinding()]
param(
    [string]$RobotHost = "192.168.55.151",
    [string]$RobotUser = "hanium",
    [ValidateRange(1, 65535)]
    [int]$DashboardPort = 8080,
    [ValidateRange(1, 65535)]
    [int]$RosbridgePort = 19092,
    [ValidateRange(1, 65535)]
    [int]$ImageBridgePort = 19093,
    [string]$IdentityFile = ""
)

$sshArgs = @(
    "-N",
    "-L", "${DashboardPort}:127.0.0.1:8080",
    "-L", "${RosbridgePort}:127.0.0.1:9092",
    "-L", "${ImageBridgePort}:127.0.0.1:9093",
    "-o", "ExitOnForwardFailure=yes",
    "-o", "ServerAliveInterval=30",
    "-o", "ServerAliveCountMax=3"
)

$temporaryIdentity = $null
try {
    if ($IdentityFile) {
        $resolvedIdentity = (Resolve-Path -LiteralPath $IdentityFile -ErrorAction Stop).Path
        $temporaryIdentity = Join-Path ([IO.Path]::GetTempPath()) (
            "sensor_dashboard_{0}.key" -f [guid]::NewGuid().ToString("N")
        )
        Copy-Item -LiteralPath $resolvedIdentity -Destination $temporaryIdentity -ErrorAction Stop

        $currentIdentity = [System.Security.Principal.WindowsIdentity]::GetCurrent().Name
        $keyAcl = New-Object System.Security.AccessControl.FileSecurity
        $keyAcl.SetOwner((New-Object System.Security.Principal.NTAccount($currentIdentity)))
        $keyAcl.SetAccessRuleProtection($true, $false)
        $keyRule = New-Object System.Security.AccessControl.FileSystemAccessRule(
            $currentIdentity, "FullControl", "Allow"
        )
        $keyAcl.AddAccessRule($keyRule)
        Set-Acl -LiteralPath $temporaryIdentity -AclObject $keyAcl -ErrorAction Stop
        $sshArgs += @("-i", $temporaryIdentity, "-o", "IdentitiesOnly=yes")
    }

    $sshArgs += "${RobotUser}@${RobotHost}"
    $dashboardUrl = "http://localhost:${DashboardPort}/?rosbridgePort=${RosbridgePort}&imageBridgePort=${ImageBridgePort}"

    Write-Host "Opening the secure sensor-dashboard tunnel to ${RobotUser}@${RobotHost}."
    Write-Host "Keep this window open and browse to ${dashboardUrl}"
    Write-Host "Press Ctrl+C here to close the dashboard tunnel."

    & ssh @sshArgs
    if ($LASTEXITCODE -ne 0) {
        throw "SSH tunnel exited with code $LASTEXITCODE."
    }
} finally {
    if ($temporaryIdentity -and (Test-Path -LiteralPath $temporaryIdentity)) {
        Remove-Item -LiteralPath $temporaryIdentity -Force
    }
}
