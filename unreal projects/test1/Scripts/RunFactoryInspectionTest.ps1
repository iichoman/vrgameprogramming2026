param([switch]$Capture,[int]$TimeoutSeconds=240)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$exe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe'
$flag=if($Capture){'-FactoryInspectionCapture'}else{'-FactoryInspectionAutoTest'}
$arguments=@((Join-Path $project 'test1.uproject'),'/Game/DigitalTwin/Maps/factory_p1','-game',$flag,'-RenderOffscreen','-windowed','-ForceRes','-ResX=1600','-ResY=900','-unattended','-nosplash','-nosound','-NoSourceControl','-log=FactoryInspectionTest.log','-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False','-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False')
$started=[DateTime]::UtcNow
$process=Start-Process -FilePath $exe -ArgumentList @($arguments|ForEach-Object {'"'+$_+'"'}) -WorkingDirectory $project -WindowStyle Hidden -PassThru
if(-not $process.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $process.Id;throw 'Inspector validation timed out.'}
$process.Refresh()
$path=Join-Path $project 'Saved/FactoryInspectionReport.json'
if(-not(Test-Path -LiteralPath $path) -or (Get-Item -LiteralPath $path).LastWriteTimeUtc -lt $started){throw 'Missing or stale inspector report. See Saved/Logs/FactoryInspectionTest.log.'}
$report=Get-Content -LiteralPath $path -Raw -Encoding UTF8|ConvertFrom-Json
$report|ConvertTo-Json -Depth 8
if($process.ExitCode -ne 0 -or -not $report.success){throw 'Inspector validation failed.'}
if($Capture){foreach($file in @('01_Selection.png','02_Problem.png','03_Sensors.png')){$image=Join-Path $project ('Saved/FactoryInspectionPreview/'+$file);if(-not(Test-Path -LiteralPath $image) -or (Get-Item -LiteralPath $image).LastWriteTimeUtc -lt $started){throw "Missing or stale image: $file"}}}
