[CmdletBinding()]
param(
    [string]$RobotHost = "192.168.55.151",
    [string]$RobotUser = "hanium",
    [ValidateRange(1, 65535)]
    [int]$LocalPort = 8765,
    [string]$IdentityFile = ""
)

$sshArgs = @(
    "-N",
    "-L", "${LocalPort}:127.0.0.1:8765",
    "-o", "ExitOnForwardFailure=yes",
    "-o", "ServerAliveInterval=30",
    "-o", "ServerAliveCountMax=3"
)

$temporaryIdentity = $null
try {
    if ($IdentityFile) {
        $resolvedIdentity = (Resolve-Path -LiteralPath $IdentityFile -ErrorAction Stop).Path
        $temporaryIdentity = Join-Path ([IO.Path]::GetTempPath()) (
            "foxglove_tunnel_{0}.key" -f [guid]::NewGuid().ToString("N")
        )
        Copy-Item -LiteralPath $resolvedIdentity -Destination $temporaryIdentity -ErrorAction Stop

        # OpenSSH rejects keys inherited from an SSHFS/network share because
        # their Windows ACL is too broad. Stage an owner-only copy for this
        # process and remove it as soon as the tunnel exits.
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

    Write-Host "Opening a secure Foxglove tunnel to ${RobotUser}@${RobotHost}."
    Write-Host "Keep this window open, then connect Foxglove to ws://localhost:${LocalPort}."
    Write-Host "Press Ctrl+C here to close the tunnel."

    & ssh @sshArgs
    if ($LASTEXITCODE -ne 0) {
        throw "SSH tunnel exited with code $LASTEXITCODE."
    }
} finally {
    if ($temporaryIdentity -and (Test-Path -LiteralPath $temporaryIdentity)) {
        Remove-Item -LiteralPath $temporaryIdentity -Force
    }
}
