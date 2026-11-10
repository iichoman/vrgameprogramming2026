param([int]$TimeoutSeconds=300)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$exe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe'
# Picking needs a real viewport projection; NullRHI has no usable screen extent.
$argsList=@((Join-Path $project 'test1.uproject'),'/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryDirectAutoTest','-RenderOffscreen','-windowed','-ForceRes','-ResX=1600','-ResY=900','-unattended','-nosplash','-nosound','-NoSourceControl','-log=FactoryDirectTest.log','-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False','-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False')
$started=[DateTime]::UtcNow
$process=Start-Process -FilePath $exe -ArgumentList @($argsList | ForEach-Object {'"'+$_+'"'}) -WorkingDirectory $project -WindowStyle Hidden -PassThru
if(-not $process.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $process.Id;throw 'Direct editing test timed out.'}
$process.Refresh()
$path=Join-Path $project 'Saved/FactoryDirectReport.json'
if(-not(Test-Path -LiteralPath $path) -or (Get-Item -LiteralPath $path).LastWriteTimeUtc -lt $started){throw 'Missing or stale direct editing report.'}
$report=Get-Content -LiteralPath $path -Raw -Encoding UTF8 | ConvertFrom-Json
$report | ConvertTo-Json -Depth 8
if($process.ExitCode -ne 0 -or -not $report.success){throw 'Direct editing test failed.'}
