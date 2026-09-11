$ErrorActionPreference = 'Stop'
Push-Location (Split-Path $PSScriptRoot -Parent)
try {
    & python tests/test_uvc_cleanup.py
    if ($LASTEXITCODE -ne 0) { throw 'UVC cleanup fault-injection test failed' }
    foreach ($name in @('test_state', 'test_ble_protocol', 'test_ble_integration', 'test_voice', 'test_mood')) {
        & gcc -std=c11 -Wall -Wextra -Werror "tests/$name.c" -o "tests/$name.exe"
        if ($LASTEXITCODE -ne 0) { throw "Compile failed: $name" }
        & "./tests/$name.exe"
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $name" }
    }
    foreach ($name in @('test_ble_ui', 'test_integrated_ui', 'test_ui', 'test_fpv_ui')) {
        & node "tests/$name.cjs"
        if ($LASTEXITCODE -ne 0) { throw "Test failed: $name" }
    }
    Write-Output 'All integrated controller host tests passed.'
} finally {
    Pop-Location
}
