param(
    [string]$BuildDirectory = "build_stable_a1"
)

$ErrorActionPreference = "Stop"
$repo = Split-Path -Parent $PSScriptRoot
$build = Join-Path $repo $BuildDirectory
$stage = Join-Path $repo "_package_stage"

New-Item -ItemType Directory -Force -Path $stage | Out-Null
Copy-Item -LiteralPath (Join-Path $build "bootloader\bootloader.bin") `
    -Destination (Join-Path $stage "bootloader.bin") -Force
Copy-Item -LiteralPath (Join-Path $build "partition_table\partition-table.bin") `
    -Destination (Join-Path $stage "partition-table.bin") -Force
Copy-Item -LiteralPath (Join-Path $build "C5VRX.bin") `
    -Destination (Join-Path $stage "C5VRX.bin") -Force
Copy-Item -LiteralPath (Join-Path $build "flasher_args.json") `
    -Destination (Join-Path $stage "flasher_args.json") -Force
Copy-Item -LiteralPath (Join-Path $repo "firmware_profiles\xiao-esp32c5.json") `
    -Destination (Join-Path $stage "profile.json") -Force

Push-Location $repo
try {
    python -m PyInstaller --noconfirm --clean `
        "C5VRX-XIAO-A1-Fixed-Receiver-Console.spec"
    if ($LASTEXITCODE -ne 0) {
        throw "PyInstaller failed with exit code $LASTEXITCODE"
    }
    Get-FileHash `
        (Join-Path $repo "dist\C5VRX-XIAO-A1-Fixed-Receiver-Console.exe") `
        -Algorithm SHA256
}
finally {
    Pop-Location
}
