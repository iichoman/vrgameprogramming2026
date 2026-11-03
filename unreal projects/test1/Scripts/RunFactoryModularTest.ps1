param([ValidateSet('Install','Test')][string]$Mode='Test',[int]$TimeoutSeconds=360)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$exe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$argsList=@((Join-Path $project 'test1.uproject'),'-nullrhi','-unattended','-nosplash','-nosound','-NoSourceControl')
if($Mode -eq 'Install'){
 if(Get-Process UnrealEditor -ErrorAction SilentlyContinue){throw 'Save and close Unreal Editor before map migration.'}
 $argsList+=@('-run=pythonscript',('-script='+((Join-Path $PSScriptRoot 'upgrade_factory_modular.py').Replace('\','/'))),'-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities')
 $reportName='FactoryModularInstallReport.json'
}else{
 $argsList+=@('/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryModularAutoTest')
 $reportName='FactoryModularReport.json'
}
$argsList+=@('-log=FactoryModular'+$Mode+'.log')
$started=[DateTime]::UtcNow
$quoted=@($argsList | ForEach-Object {'"'+$_+'"'})
$process=Start-Process -FilePath $exe -ArgumentList $quoted -WorkingDirectory $project -WindowStyle Hidden -PassThru
if(-not $process.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $process.Id;throw 'Factory modular test timed out.'}
$process.Refresh()
$report=Join-Path $project ('Saved/'+$reportName)
if(-not(Test-Path -LiteralPath $report) -or (Get-Item -LiteralPath $report).LastWriteTimeUtc -lt $started){throw 'Missing or stale report; inspect Saved/Logs/FactoryModular*.log.'}
$result=Get-Content -LiteralPath $report -Raw -Encoding UTF8 | ConvertFrom-Json
$result | ConvertTo-Json -Depth 8
if($process.ExitCode -ne 0 -or -not $result.success){throw 'Factory modular checks failed.'}
