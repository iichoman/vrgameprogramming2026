param([int]$TimeoutSeconds=240)
$ErrorActionPreference='Stop'
$factoryProject=Split-Path $PSScriptRoot -Parent
$factoryExe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe'
$factoryArgs=@((Join-Path $factoryProject 'test1.uproject'),'/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryRuntimeCapture','-RenderOffscreen','-windowed','-ForceRes','-ResX=1600','-ResY=900','-unattended','-nosplash','-nosound','-NoSourceControl','-log=FactoryEditorRender.log','-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False','-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False','-ExecCmds=r.ScreenPercentage 100,r.AntiAliasingMethod 1')
$factoryQuoted=@($factoryArgs | ForEach-Object { '"'+$_+'"' })
$factoryStarted=[DateTime]::UtcNow
$factoryProcess=Start-Process -FilePath $factoryExe -ArgumentList $factoryQuoted -WorkingDirectory $factoryProject -WindowStyle Hidden -PassThru
if(-not $factoryProcess.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $factoryProcess.Id;throw 'Factory capture timed out.'}
$factoryProcess.Refresh()
foreach($factoryName in @('FactoryEditorUIReport.json','FactoryRuntimePreview/report.json')) {
 $factoryReport=Join-Path $factoryProject ('Saved/'+$factoryName)
 if(-not (Test-Path -LiteralPath $factoryReport)){throw "Missing report: $factoryName"}
 if((Get-Item -LiteralPath $factoryReport).LastWriteTimeUtc -lt $factoryStarted){throw "Stale report: $factoryName"}
 $factoryData=Get-Content -LiteralPath $factoryReport -Raw -Encoding UTF8 | ConvertFrom-Json
 if(-not $factoryData.success){throw "Failed report: $factoryName"}
}
Write-Output 'Factory rendered UI checks and wheel cycle passed.'
