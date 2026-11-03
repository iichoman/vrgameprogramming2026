param([ValidateSet('Install','Environment','Test','Regression','Capture')][string]$Mode='Test',[int]$TimeoutSeconds=480)
$ErrorActionPreference='Stop'
$assemblyProject=Split-Path $PSScriptRoot -Parent
$assemblyExe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$assemblyArgs=@((Join-Path $assemblyProject 'test1.uproject'),'-unattended','-nosplash','-nosound','-NoSourceControl','-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False','-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False')
if($Mode -eq 'Install') {
 if(Get-Process UnrealEditor -ErrorAction SilentlyContinue){throw 'Save and close the editor before modifying the map.'}
 $assemblyArgs+=@('-run=pythonscript',('-script='+((Join-Path $PSScriptRoot 'integrate_factory_assembly.py').Replace('\','/'))),'-nullrhi','-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities')
 $assemblyReportName='FactoryAssemblyInstallReport.json'
} elseif($Mode -eq 'Environment') {
 if(Get-Process UnrealEditor -ErrorAction SilentlyContinue){throw 'Close the editor before updating map lighting.'}
 $assemblyArgs+=@('-run=pythonscript',('-script='+((Join-Path $PSScriptRoot 'configure_factory_environment.py').Replace('\','/'))),'-nullrhi','-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities')
 $assemblyReportName='FactoryAssemblyEnvironmentReport.json'
} elseif($Mode -eq 'Regression') {
 $assemblyArgs+=@('/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryRuntimeAutoTest','-nullrhi')
 $assemblyReportName='FactoryRuntimeTestReport.json'
} elseif($Mode -eq 'Test') {
 $assemblyArgs+=@('/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryAssemblyAutoTest','-nullrhi')
 $assemblyReportName='FactoryAssemblyIntegrationReport.json'
} else {
 $assemblyExe='C:\Program Files\Epic Games\UE_5.8\Engine\Binaries\Win64\UnrealEditor.exe'
 $assemblyArgs+=@('/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryRuntimeCapture','-RenderOffscreen','-windowed','-ForceRes','-ResX=1600','-ResY=900','-ExecCmds=r.ScreenPercentage 80,r.AntiAliasingMethod 1')
 $assemblyReportName='FactoryRuntimePreview/report.json'
}
$assemblyArgs+=@("-log=FactoryAssembly$Mode.log")
$assemblyQuoted=@($assemblyArgs | ForEach-Object {'"'+$_+'"'})
$assemblyStarted=[DateTime]::UtcNow
$assemblyProc=Start-Process -FilePath $assemblyExe -ArgumentList $assemblyQuoted -WorkingDirectory $assemblyProject -WindowStyle Hidden -PassThru
if(-not $assemblyProc.WaitForExit($TimeoutSeconds*1000)){Stop-Process -Id $assemblyProc.Id;throw 'Assembly test timed out.'}
$assemblyProc.Refresh()
$assemblyReport=Join-Path $assemblyProject ('Saved/'+$assemblyReportName)
if(-not (Test-Path -LiteralPath $assemblyReport)){throw "Missing report. See Saved/Logs/FactoryAssembly$Mode.log"}
if((Get-Item -LiteralPath $assemblyReport).LastWriteTimeUtc -lt $assemblyStarted){throw 'Stale report rejected.'}
$assemblyData=Get-Content -LiteralPath $assemblyReport -Raw -Encoding UTF8 | ConvertFrom-Json
if(-not $assemblyData.success -or $assemblyProc.ExitCode -ne 0){throw "Assembly $Mode failed. See report and log."}
$assemblyData | ConvertTo-Json -Depth 8
