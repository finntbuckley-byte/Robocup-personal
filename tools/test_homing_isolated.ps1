param([Parameter(Mandatory=$true)][ValidateSet(1,2)][int]$Test)
$ErrorActionPreference='Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    if($Test -eq 1) {
        & g++ -std=c++17 -Wall -Wextra -Werror -Itools/native_stubs -Iinclude src/navigation.cpp src/round.cpp src/collection.cpp src/smooth_servo.cpp src/homing.cpp src/home_core.cpp src/imu.cpp src/pose.cpp src/drive.cpp src/weight_detect.cpp tools/homing_integration_test.cpp -o .pio/homing_isolated_test.exe
        if($LASTEXITCODE -ne 0){throw 'Isolated test compilation failed'}
        & .pio/homing_isolated_test.exe handoff_trace
        if($LASTEXITCODE -ne 0){throw 'Third-pickup handoff test failed'}
    } else {
        if(!(Test-Path .pio/homing_isolated_test.exe)){throw 'Run test 1 first'}
        foreach($fault in @('front_stale','front_close','rear_stale','rear_close','left_unknown','right_close','rear_unknown_front_186','rear_unknown_clear','gate_top_missing','imu_loss_stationary','imu_loss_moving','imu_reset')){
            & .pio/homing_isolated_test.exe "block_$fault"
            if($LASTEXITCODE -ne 0){throw "Isolated blocker test failed: $fault"}
        }
        Write-Output 'PASS isolated test 2: 12 independent blocker/recovery cases, each in a fresh process'
    }
} finally {Pop-Location}
