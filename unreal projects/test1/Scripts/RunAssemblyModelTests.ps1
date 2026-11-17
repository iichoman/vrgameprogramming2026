param([int]$TimeoutSeconds=120)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$exe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$report=Join-Path $project 'Saved/AssemblyModelTests'
$argsList=@((Join-Path $project 'test1.uproject'),'-nullrhi','-unattended','-nosplash','-nosound','-NoSourceControl',
 '-ExecCmds=Automation RunTests DigitalTwin.Assembly','-TestExit=Automation Test Queue Empty',
 ('-ReportExportPath='+$report),('-abslog='+(Join-Path $project 'Saved/Logs/AssemblyModelTests.log')))
$started=[DateTime]::UtcNow
$quoted=@($argsList | ForEach-Object {'"'+$_+'"'})
$process=Start-Process -FilePath $exe -ArgumentList $quoted -WorkingDirectory $project -WindowStyle Hidden -PassThru
if(-not $process.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $process.Id;throw 'Assembly model tests timed out.'}
$process.Refresh()
$resultFile=Join-Path $report 'index.json'
if(-not(Test-Path $resultFile) -or (Get-Item $resultFile).LastWriteTimeUtc -lt $started){throw 'Missing or stale automation report.'}
$result=Get-Content $resultFile -Raw -Encoding UTF8 | ConvertFrom-Json
$result | ConvertTo-Json -Depth 8
if($process.ExitCode -ne 0 -or $result.failed -ne 0 -or $result.succeeded -lt 2){throw 'Assembly model/motion tests failed.'}
