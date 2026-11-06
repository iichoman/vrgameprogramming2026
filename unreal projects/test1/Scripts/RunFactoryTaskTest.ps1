param([int]$TimeoutSeconds=360)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$exe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$argsList=@((Join-Path $project 'test1.uproject'),'/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryTaskAutoTest','-nullrhi','-unattended','-nosplash','-nosound','-NoSourceControl','-log=FactoryTaskTest.log')
$started=[DateTime]::UtcNow
$quoted=@($argsList | ForEach-Object {'"'+$_+'"'})
$process=Start-Process -FilePath $exe -ArgumentList $quoted -WorkingDirectory $project -WindowStyle Hidden -PassThru
if(-not $process.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $process.Id;throw 'Factory task test timed out.'}
$process.Refresh()
$report=Join-Path $project 'Saved/FactoryTaskReport.json'
if(-not(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTimeUtc -lt $started){throw 'Missing or stale report; inspect Saved/Logs/FactoryTaskTest.log.'}
$result=Get-Content -LiteralPath $report -Raw -Encoding UTF8 | ConvertFrom-Json
$result | ConvertTo-Json -Depth 8
if($process.ExitCode -ne 0 -or -not $result.success){throw 'Factory task checks failed.'}
