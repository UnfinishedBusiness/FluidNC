# JetCad3 firmware updater — refreshes the FluidNC firmware images inside an installed
# JetCad3 release, so machines can be flashed with the latest firmware without waiting
# for a full JetCad3 release.
#
# Run from any PowerShell prompt (no admin needed for a per-user install):
#   irm https://raw.githubusercontent.com/UnfinishedBusiness/FluidNC/main/update-jc3-firmware.ps1 | iex
#
# What it does:
#   1. Finds the JetCad3 install's resources\firmwares\FluidNC directory.
#   2. Downloads the latest firmware bundle from the FluidNC GitHub release "jc3-firmware".
#   3. Backs up the current firmwares, swaps in the new ones, and verifies every image's
#      SHA-256 against the bundled manifest.
# After it finishes: restart JetCad3, then flash the controller from the GcodePilot
# workspace's Flash Firmware dialog (ESP32 — No radio) as usual.

$ErrorActionPreference = 'Stop'
[Net.ServicePointManager]::SecurityProtocol = [Net.ServicePointManager]::SecurityProtocol -bor [Net.SecurityProtocolType]::Tls12

$bundleUrl = 'https://github.com/UnfinishedBusiness/FluidNC/releases/download/jc3-firmware/fluidnc-firmwares.zip'

Write-Host ''
Write-Host 'JetCad3 FluidNC firmware updater' -ForegroundColor Cyan
Write-Host '--------------------------------'

# 1. Locate the installed JetCad3 firmwares directory.
$candidates = @(
    (Join-Path $env:LOCALAPPDATA 'Programs\JetCad3\resources\firmwares\FluidNC'),
    (Join-Path $env:ProgramFiles 'JetCad3\resources\firmwares\FluidNC')
)
if (${env:ProgramFiles(x86)}) {
    $candidates += (Join-Path ${env:ProgramFiles(x86)} 'JetCad3\resources\firmwares\FluidNC')
}
$fwDir = $candidates | Where-Object { Test-Path $_ } | Select-Object -First 1
if (-not $fwDir) {
    Write-Host 'Could not find a JetCad3 install (looked in:)' -ForegroundColor Red
    $candidates | ForEach-Object { Write-Host "  $_" }
    throw 'JetCad3 install not found — is JetCad3 installed on this machine?'
}
$oldCommit = 'unknown'
$oldManifest = Join-Path $fwDir 'manifest.json'
if (Test-Path $oldManifest) {
    try { $oldCommit = (Get-Content $oldManifest -Raw | ConvertFrom-Json).build.gitCommit } catch {}
}
Write-Host "Install found : $fwDir"
Write-Host "Current build : $oldCommit"

# 2. Download the bundle.
$tmp = Join-Path $env:TEMP ("jc3-fw-" + [Guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $tmp | Out-Null
$zip = Join-Path $tmp 'fluidnc-firmwares.zip'
Write-Host "Downloading   : $bundleUrl"
Invoke-WebRequest -UseBasicParsing -Uri $bundleUrl -OutFile $zip
$new = Join-Path $tmp 'new'
Expand-Archive -Path $zip -DestinationPath $new -Force

# 3. Verify every image against the bundled manifest BEFORE touching the install.
$manifest = Get-Content (Join-Path $new 'manifest.json') -Raw | ConvertFrom-Json
foreach ($t in $manifest.targets) {
    $img = Join-Path $new ($t.image.path -replace '/', '\')
    $hash = (Get-FileHash -Algorithm SHA256 -Path $img).Hash.ToLower()
    if ($hash -ne $t.image.sha256.ToLower()) {
        throw "Checksum mismatch for target '$($t.id)' — download corrupted, nothing was changed. Please re-run."
    }
    Write-Host ("Verified      : {0}  ({1})" -f $t.id, $hash.Substring(0, 12))
}

# 4. Back up the current firmwares, then swap in the new set.
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss'
$backup = "$fwDir.backup-$stamp"
Copy-Item -Path $fwDir -Destination $backup -Recurse
Copy-Item -Path (Join-Path $new '*') -Destination $fwDir -Recurse -Force
Remove-Item -Path $tmp -Recurse -Force

Write-Host ''
Write-Host ("Updated {0} -> {1}" -f $oldCommit, $manifest.build.gitCommit) -ForegroundColor Green
Write-Host "Backup kept at: $backup"
Write-Host ''
Write-Host 'Next steps:' -ForegroundColor Yellow
Write-Host '  1. Restart JetCad3 if it is open.'
Write-Host '  2. GcodePilot workspace -> Flash Firmware -> "ESP32 - No radio" -> flash the controller.'
Write-Host '  3. IMPORTANT: after flashing, push the machine config back to the board (the flash'
Write-Host '     dialog reminds you) - flashing resets the controller to defaults.'
