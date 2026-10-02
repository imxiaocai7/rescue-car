$ErrorActionPreference = 'Stop'
$stm32ProjectRoot = Split-Path -Parent $PSScriptRoot
$softwareRoot = Split-Path -Parent $stm32ProjectRoot
$testOutput = Join-Path ([IO.Path]::GetTempPath()) ('rescue-stm32-tests-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $testOutput | Out-Null
$nativeCompiler = (Get-Command gcc -ErrorAction Stop).Source
$cases = @(
    @{ Name = 'motion_protocol'; Sources = @('motion_protocol', 'motion_command_guard') },
    @{ Name = 'wheel_speed_controller'; Sources = @('wheel_speed_controller') },
    @{ Name = 'motion_control'; Sources = @('motion_protocol', 'motion_command_guard', 'wheel_speed_controller', 'motion_control') },
    @{ Name = 'encoder_speed_estimator'; Sources = @('encoder_speed_estimator') },
    @{ Name = 'rover_hardware_profile'; Sources = @('motion_protocol', 'rover_telemetry') },
    @{ Name = 'commissioning_application'; Sources = @('motion_protocol', 'motion_command_guard', 'rover_telemetry', 'commissioning_application') },
    @{ Name = 'uart_rx_queue'; Sources = @('uart_rx_queue') }
)
foreach ($case in $cases) {
    $nativeArgs = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-pedantic', '-I', (Join-Path $stm32ProjectRoot 'include'))
    foreach ($sourceName in $case.Sources) {
        $nativeArgs += Join-Path $stm32ProjectRoot ('src/' + $sourceName + '.c')
    }
    $nativeArgs += Join-Path $softwareRoot ('tests/' + $case.Name + '_test.c')
    $executable = Join-Path $testOutput ($case.Name + '.exe')
    $nativeArgs += @('-lm', '-o', $executable)
    & $nativeCompiler @nativeArgs
    if ($LASTEXITCODE -ne 0) { throw ('C compilation failed: ' + $case.Name) }
    & $executable
    if ($LASTEXITCODE -ne 0) { throw ('C test failed: ' + $case.Name) }
}
Write-Output ('All 7 native C test programs passed. Temporary executables: ' + $testOutput)
