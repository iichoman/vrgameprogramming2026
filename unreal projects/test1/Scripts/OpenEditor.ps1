param([string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.8', [switch]$ValidateOnly)
$ErrorActionPreference = 'Stop'
$camryProject = Split-Path -Parent $PSScriptRoot
$camryProjectFile = Join-Path $camryProject 'test1.uproject'
$camryEditor = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor.exe'
if (-not (Test-Path -LiteralPath $camryProjectFile)) { throw 'test1.uproject is missing.' }
if (-not (Test-Path -LiteralPath $camryEditor)) { throw "Editor not found: $camryEditor" }
if ($camryProjectFile -match '[^\x00-\x7F]') { throw 'Use an ASCII project path.' }
if ($ValidateOnly) { Write-Output "Validated project: $camryProjectFile"; exit 0 }
if (Get-Process UnrealEditor -ErrorAction SilentlyContinue) { throw 'An Unreal Editor is already open. This launcher will not open a second instance.' }
Start-Process -FilePath $camryEditor -ArgumentList ('"' + $camryProjectFile + '"')
