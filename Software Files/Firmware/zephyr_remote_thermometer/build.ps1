# Build the remote thermometer with the installed nRF Connect SDK. Every target is built for one board,
# nrf52833dk/nrf52833 (the thermometer PCB's nRF52833); the emulator runs the nRF52833 firmware on its
# nRF52840 model.
#   .\build.ps1                     # emulator: prj.conf + emulation\emulator.conf / emulator.overlay
#   .\build.ps1 -Target hardware    # thermometer PCB: prj.conf only
#   .\build.ps1 -Target client      # test client for a second emulator node (emulation\thermometer_client)
#   .\build.ps1 -Target dongle      # virtual BLE dongle (HCI over uart0) for Bumble apps on the PC (emulation\hci_dongle)
#   .\build.ps1 -Config Debug       # debug configuration: logging (UART in the emulator, RTT on hardware),
#                                   # debug info. Default: -Config Release
#   .\build.ps1 -Pristine           # clean rebuild
#
# Release is the plain configuration: no logging, no UART, optimized, what goes on the battery-powered
# thermometer and what the emulator's Power tab should measure. Debug adds config\debug.conf and
# config\debug_<target>.conf on top. -Config applies to the thermometer (emulator, hardware); the client
# and dongle helpers have a single configuration.
#
# Output goes to <repo>\build\<target>, outside "Software Files": the Zephyr build and Renode's monitor
# both break on spaces in paths, so the ELF must live on a path without them.
param(
    [ValidateSet("emulator", "hardware", "client", "dongle")][string]$Target = "emulator",
    [string]$Ncs = "C:\ncs\v3.4.0",
    [string]$Toolchain = "",
    [ValidateSet("Release", "Debug")][string]$Config = "Release",
    [switch]$Pristine
)
$ErrorActionPreference = "Stop"
$here = $PSScriptRoot
$repo = (Resolve-Path (Join-Path $here "..\..\..")).Path

$board = "nrf52833dk/nrf52833"
$appRel = @{ emulator = ""; hardware = ""; client = "emulation\thermometer_client"; dongle = "emulation\hci_dongle" }[$Target]
$buildDir = Join-Path $repo "build\$Target"
if ($buildDir -match " ") { throw "Build directory '$buildDir' contains spaces" }

# Map the folder above this project to a temporary drive letter so the sources have no spaces either
# (sysbuild names the app after its folder, so the source can't be a drive root).
$drive = $null
$root = $here
if ($here -match " ") {
    $used = (Get-PSDrive -PSProvider FileSystem).Name
    $letter = [char[]]"ZYXWVUTSRQPONMLKJIHG" | Where-Object { $used -notcontains [string]$_ } | Select-Object -First 1
    if (-not $letter) { throw "No free drive letter to map '$here' without spaces" }
    $drive = "${letter}:"
    subst $drive (Split-Path $here)
    if ($LASTEXITCODE) { throw "subst $drive failed" }
    $root = Join-Path "$drive\" (Split-Path $here -Leaf)
}
$src = if ($appRel) { Join-Path $root $appRel } else { $root }

$westArgs = @("build", "-b", $board, "-d", $buildDir, $src)
if ($Pristine) { $westArgs += @("-p", "always") }
# Configuration fragments on top of prj.conf: the emulator differences (emulation\emulator.*) and the
# debug configuration. Always passed (empty if none) so a cached setting from an earlier build doesn't stick.
$isThermometer = $Target -in @("emulator", "hardware")
$extraConf = @()
$extraDts = @()
if ($Target -eq "emulator") {
    $extraConf += "emulation\emulator.conf"
    $extraDts += "emulation\emulator.overlay"
}
if ($Config -eq "Debug" -and $isThermometer) {
    $extraConf += @("config\debug.conf", "config\debug_$Target.conf")
}
$toCmake = { param($files) ($files | ForEach-Object { (Join-Path $root $_).Replace("\", "/") }) -join ";" }
$westArgs += @("--", "-DEXTRA_CONF_FILE=$(& $toCmake $extraConf)", "-DEXTRA_DTC_OVERLAY_FILE=$(& $toCmake $extraDts)")

if (-not $Toolchain) {
    $Toolchain = (Get-ChildItem "C:\ncs\toolchains" -Directory | Where-Object { Test-Path (Join-Path $_.FullName "environment.json") } |
        Sort-Object LastWriteTime -Descending | Select-Object -First 1).FullName
}
$envSpec = Get-Content (Join-Path $Toolchain "environment.json") -Raw | ConvertFrom-Json

# The toolchain environment is only applied for the build and the caller's values are restored afterwards:
# the toolchain's PYTHONPATH (Python 3.12) breaks any other Python started from this shell.
$savedEnv = @{}
foreach ($key in @($envSpec.env_vars.key) + "ZEPHYR_BASE") { $savedEnv[$key] = [Environment]::GetEnvironmentVariable($key) }
$pushed = $false
try {
    # Same environment as nRF Connect for VS Code / nrfutil toolchain-manager
    foreach ($v in $envSpec.env_vars) {
        if ($v.type -eq "string") { $value = $v.value }
        else { $value = ($v.values | ForEach-Object { Join-Path $Toolchain $_ }) -join ";" }
        if ($v.existing_value_treatment -eq "prepend_to") { $value = "$value;" + [Environment]::GetEnvironmentVariable($v.key) }
        [Environment]::SetEnvironmentVariable($v.key, $value)
    }
    # west needs the source and the current directory on one drive; it finds the workspace through ZEPHYR_BASE.
    $env:ZEPHYR_BASE = Join-Path $Ncs "zephyr"
    Push-Location $src
    $pushed = $true
    & west @westArgs
    if ($LASTEXITCODE) { throw "west build failed" }
}
finally {
    if ($pushed) { Pop-Location }
    if ($drive) { subst $drive /d }
    foreach ($key in $savedEnv.Keys) { [Environment]::SetEnvironmentVariable($key, $savedEnv[$key]) }
}

$app = Split-Path $src -Leaf
if ($isThermometer) { Write-Host "Configuration: $Target, $Config" }
Write-Host "ELF: $(Join-Path $buildDir "$app\zephyr\zephyr.elf")"
if ($Target -eq "hardware") { Write-Host "HEX: $(Join-Path $buildDir "$app\zephyr\zephyr.hex")" }
