$ErrorActionPreference='Stop'
Push-Location (Split-Path -Parent $PSScriptRoot)
try {
    New-Item -ItemType Directory -Force .pio | Out-Null
    & g++ -std=c++17 -Wall -Wextra -Werror -Itools/native_stubs -Iinclude src/imu.cpp tools/imu_test.cpp -o .pio/imu_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU test compilation failed' }
    & .pio/imu_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'IMU test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Itools/gate_stubs -Iinclude src/gate.cpp tools/gate_test.cpp -o .pio/gate_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Gate test compilation failed' }
    & .pio/gate_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Gate test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Iinclude src/home_core.cpp tools/home_core_test.cpp -o .pio/home_core_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Homing test compilation failed' }
    & .pio/home_core_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Homing test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Iinclude tools/home_detour_test.cpp -o .pio/home_detour_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Detour test compilation failed' }
    & .pio/home_detour_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Detour test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Itools/native_stubs -Iinclude src/round.cpp tools/round_test.cpp -o .pio/round_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Round test compilation failed' }
    & .pio/round_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Round test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Itools/native_stubs -Iinclude src/collection.cpp src/smooth_servo.cpp tools/collection_test.cpp -o .pio/collection_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Collection test compilation failed' }
    & .pio/collection_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Collection test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Itools/native_stubs -Iinclude src/weight_detect.cpp tools/weight_detect_test.cpp -o .pio/weight_detect_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Weight detector test compilation failed' }
    & .pio/weight_detect_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Weight detector test failed' }
    & g++ -std=c++17 -Wall -Wextra -Werror -Itools/native_stubs -Iinclude src/navigation.cpp src/round.cpp src/collection.cpp src/smooth_servo.cpp src/homing.cpp src/home_core.cpp src/imu.cpp src/pose.cpp src/drive.cpp src/weight_detect.cpp tools/homing_integration_test.cpp -o .pio/homing_integration_test.exe
    if ($LASTEXITCODE -ne 0) { throw 'Homing integration compilation failed' }
    foreach ($scenario in @('normal','rear_unknown','gate_missing','no_home_colour','late_pickup','top_weight','physical_left','physical_right','delivery_start_30','delivery_start_210','delivery_start_355','home_detour','nav_parity','pickup_first_success','pickup_second_success','pickup_third_success','pickup_all_miss','pickup_flicker','pickup_late_miss','pickup_third_retry','pickup_sample_gap')) {
        & .pio/homing_integration_test.exe $scenario
        if ($LASTEXITCODE -ne 0) { throw "Homing integration failed: $scenario" }
    }
} finally { Pop-Location }
