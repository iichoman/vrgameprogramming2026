param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.8')
$ErrorActionPreference = 'Stop'
$camryProject = Split-Path -Parent $PSScriptRoot
$camryProjectFile = Join-Path $camryProject 'test1.uproject'
if (-not (Test-Path -LiteralPath $camryProjectFile)) { throw 'test1.uproject is missing.' }
if ($camryProjectFile -match '[^\x00-\x7F]') { throw 'Use an ASCII project path.' }
if (Get-Process UnrealEditor -ErrorAction SilentlyContinue) { throw 'Use Live Coding while the editor is open. A full module build requires it to be closed.' }
& (Join-Path $EngineRoot 'Engine\Build\BatchFiles\Build.bat') test1Editor Win64 Development "-Project=$camryProjectFile" -WaitMutex -NoHotReloadFromIDE -NoUBA -MaxParallelActions=2
exit $LASTEXITCODE
