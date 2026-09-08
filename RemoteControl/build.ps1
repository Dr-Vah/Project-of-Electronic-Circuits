$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
if (!(Test-Path 'main/tls/server-cert.pem')) {
    python setup_tls.py
    if ($LASTEXITCODE -ne 0) { throw 'TLS certificate setup failed' }
}
. 'C:/Espressif/tools/Microsoft.v5.4.4.PowerShell_profile.ps1'
$env:PATH = 'C:/Espressif/tools/ninja/1.12.1;C:/Espressif/tools/xtensa-esp-elf/esp-14.2.0_20260121/xtensa-esp-elf/bin;' + $env:PATH
idf.py -DCCACHE_ENABLE=0 build
if ($LASTEXITCODE -ne 0) { throw 'ESP-IDF build failed' }
