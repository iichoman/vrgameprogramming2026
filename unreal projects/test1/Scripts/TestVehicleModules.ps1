param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.8')
$ErrorActionPreference = 'Stop'
$projectDir = Split-Path $PSScriptRoot -Parent
$projectFile = (Join-Path $projectDir 'test1.uproject').Replace('\','/')
$editorCmd = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$scriptFile = (Join-Path $PSScriptRoot 'import_process_vehicle.py').Replace('\','/')
$common = @('-unattended','-nosplash','-nosound','-nullrhi','-NoSourceControl','-stdout',
    '-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False',
    '-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False')
$started = [DateTime]::UtcNow
& $editorCmd $projectFile '-run=pythonscript' "-script=$scriptFile" '-ProcessModulesVerifyOnly' '-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities' '-log=ProcessModulesColdLoad.log' @common | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Saved map validation failed. Read Saved/Logs/ProcessModulesColdLoad.log.' }
& $editorCmd $projectFile '/Game/DigitalTwin/Maps/CAR_MODULE_TEST' '-game' '-VehicleModulesAutoTest' '-log=ProcessModulesRuntime.log' @common | Out-Null
if ($LASTEXITCODE -ne 0) { throw 'Runtime checks failed. Read Saved/Logs/ProcessModulesRuntime.log.' }
foreach ($name in @('CAR_MODULE_TEST_ColdLoadReport.json','VehicleModulesRuntimeReport.json')) {
    $reportFile = Get-Item -LiteralPath (Join-Path $projectDir ('Saved\'+$name))
    if ($reportFile.LastWriteTimeUtc -lt $started) { throw "Stale report: $name" }
    $report = Get-Content -LiteralPath $reportFile.FullName -Raw | ConvertFrom-Json
    if (-not $report.success) { throw "Failed report: $name" }
}
Write-Output 'PASS: saved module map and 15 native runtime checks.'
