param([int]$TimeoutSeconds=600,[switch]$ExtendedChecks,[switch]$UIOnly,[switch]$ModularOnly,[switch]$IncidentFocus)
$ErrorActionPreference='Stop'
$project=Split-Path $PSScriptRoot -Parent
$uproject=Join-Path $project 'test1.uproject'
$engine='C:\Program Files\Epic Games\UE_5.8\Engine'
$build=Join-Path $engine 'Build\BatchFiles\Build.bat'
$editor=Join-Path $engine 'Binaries\Win64\UnrealEditor-Cmd.exe'
$logs=Join-Path $project 'Saved\Logs'
$reportFolder=Join-Path $project 'Saved\MotionVerification'
New-Item -ItemType Directory -Path $reportFolder -Force | Out-Null
$result=[ordered]@{}
if($IncidentFocus){$ModularOnly=$true}

# Attempt the active original project's Live Coding compiler path once. This
# does not claim that LiveCodingConsole linked/applied the editor patch.
$binaries=Join-Path $project 'Binaries\Win64'
$loaded=@(Get-Process UnrealEditor -ErrorAction SilentlyContinue | ForEach-Object {
    try { $_.Modules | Where-Object { $_.ModuleName -like 'UnrealEditor-test1*.dll' -and $_.FileName.StartsWith($binaries,[StringComparison]::OrdinalIgnoreCase) } | ForEach-Object FileName } catch {}
})
if($loaded.Count -gt 0) {
    $modules=Join-Path $reportFolder 'LiveCodingModules.json'
    @{EnabledModules=$loaded;LazyLoadModules=@()} | ConvertTo-Json -Depth 4 | Set-Content -LiteralPath $modules -Encoding UTF8
    & $build test1Editor Win64 Development "-Project=$uproject" -LiveCoding "-LiveCodingModules=$modules" "-LiveCodingManifest=$reportFolder\LiveCodingManifest.json" -LiveCodingLimit=100 -WaitMutex -NoUBA -MaxParallelActions=2 *> (Join-Path $logs 'MotionLiveCodingBuild.log')
    $result.liveCodingCompileExit=$LASTEXITCODE
    $result.editorPatchApplied=$false
} else { $result.liveCoding='No original editor module available';$result.editorPatchApplied=$false }

# A numbered DLL in the SAME project avoids overwriting the editor's loaded DLL.
# No project copy, worktree, editor termination, or user asset save is performed.
$suffix=[int](Get-Date -Format 'HHmmss')
& $build test1Editor Win64 Development "-Project=$uproject" "-ModuleWithSuffix=test1,$suffix" -WaitMutex -NoHotReloadFromIDE -NoUBA -MaxParallelActions=2 *> (Join-Path $logs 'MotionVerificationBuild.log')
$result.buildExit=$LASTEXITCODE
if($LASTEXITCODE -ne 0) { $result | ConvertTo-Json | Set-Content "$reportFolder\summary.json" -Encoding UTF8;Get-Content (Join-Path $logs 'MotionVerificationBuild.log') -Tail 30;exit 1 }

function Run-EngineCheck([string]$Name,[string[]]$Extra,[bool]$Render=$false) {
    $display=if($Render){@('-RenderOffscreen','-windowed','-ForceRes','-ResX=1600','-ResY=900','-RobotMotionSynchronousTest','-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False','-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False')}else{@('-nullrhi')}
    $exe=if($Render){Join-Path $engine 'Binaries\Win64\UnrealEditor.exe'}else{$editor}
    $argsList=@($uproject)+$Extra+$display+@('-unattended','-nosplash','-nosound','-NoSourceControl','-NoLiveCoding',"-abslog=$logs\$Name.log")
    $quoted=@($argsList | ForEach-Object {'"'+$_+'"'})
    $process=Start-Process -FilePath $exe -ArgumentList $quoted -WorkingDirectory $project -WindowStyle Hidden -PassThru
    if(-not $process.WaitForExit($TimeoutSeconds*1000)) { Stop-Process -Id $process.Id;return 'timeout' }
    $process.Refresh();return $process.ExitCode
}
$started=[DateTime]::UtcNow
if(-not $UIOnly -and -not $ModularOnly) {
$result.unitExit=Run-EngineCheck 'MotionUnitTests' @('-ExecCmds=Automation RunTests DigitalTwin.Assembly+test1.RobotControl','-TestExit=Automation Test Queue Empty',"-ReportExportPath=$reportFolder\UnitTests")
$unitReport=Join-Path $reportFolder 'UnitTests\index.json'
if((Test-Path -LiteralPath $unitReport) -and (Get-Item -LiteralPath $unitReport).LastWriteTimeUtc -gt $started) {
    $unit=Get-Content -LiteralPath $unitReport -Raw -Encoding UTF8 | ConvertFrom-Json
    $result.unitPassed=@($unit.tests | Where-Object state -eq 'Success').Count
    $result.unitFailed=@($unit.tests | Where-Object state -ne 'Success').Count
} else { $result.unitFailed='Missing or stale report' }
$result.factoryExit=Run-EngineCheck 'MotionFactoryTest' @('/Game/DigitalTwin/Maps/factory_p1','-game','-FactoryTaskAutoTest','-LogCmds=LogTemp Verbose')
$factory=Join-Path $project 'Saved\FactoryTaskReport.json'
if((Test-Path -LiteralPath $factory) -and (Get-Item -LiteralPath $factory).LastWriteTimeUtc -gt $started) {
    Copy-Item -LiteralPath $factory -Destination "$reportFolder\FactoryTaskReport.json" -Force
    $result.factory=Get-Content -LiteralPath $factory -Raw -Encoding UTF8 | ConvertFrom-Json
} else { $result.factory='Missing or stale report' }
}
$extendedFailed=$false
if($UIOnly -or $ModularOnly -or ($ExtendedChecks -and $result.factory.success -eq $true)) {
    $checks=@(
        @{Key='direct';Flag='FactoryDirectAutoTest';Report='FactoryDirectReport.json'},
        @{Key='inspection';Flag='FactoryInspectionAutoTest';Report='FactoryInspectionReport.json'},
        @{Key='integration';Flag='FactoryAssemblyAutoTest';Report='FactoryAssemblyIntegrationReport.json'},
        @{Key='modular';Flag='FactoryModularAutoTest';Report='FactoryModularReport.json'}
    )
    if($UIOnly){$checks=@($checks | Where-Object {$_.Key -in @('direct','inspection')})}
    if($ModularOnly){$checks=@($checks | Where-Object Key -eq 'modular')}
    if($IncidentFocus){$checks=@(@{Key='merge';Flag='FactoryModularAutoTest';Report='FactoryMergeReport.json'})}
    foreach($check in $checks) {
        $checkStart=[DateTime]::UtcNow
        $extra=@('/Game/DigitalTwin/Maps/factory_p1','-game',('-'+$check.Flag))
        if($IncidentFocus){$extra+='-FactoryIncidentFocus'}
        $exitCode=Run-EngineCheck ('Motion_'+$check.Key) $extra ($check.Key -in @('direct','inspection'))
        $path=Join-Path $project ('Saved\'+$check.Report)
        if((Test-Path -LiteralPath $path) -and (Get-Item -LiteralPath $path).LastWriteTimeUtc -gt $checkStart) {
            $report=Get-Content -LiteralPath $path -Raw -Encoding UTF8 | ConvertFrom-Json
            Copy-Item -LiteralPath $path -Destination $reportFolder -Force
            $result[$check.Key]=@{exitCode=$exitCode;success=$report.success;passed=@($report.checks | Where-Object passed).Count;failed=@($report.checks | Where-Object {-not $_.passed})}
            $extendedFailed=$extendedFailed -or $exitCode -ne 0 -or $report.success -ne $true
        } else {$result[$check.Key]='Missing or stale report';$extendedFailed=$true}
    }
}
$summaryName=if($UIOnly){'ui-summary.json'}elseif($IncidentFocus){'merge-summary.json'}elseif($ModularOnly){'modular-summary.json'}else{'summary.json'}
$result | ConvertTo-Json -Depth 12 | Set-Content (Join-Path $reportFolder $summaryName) -Encoding UTF8
$result | ConvertTo-Json -Depth 12
if($extendedFailed -or (-not $UIOnly -and -not $ModularOnly -and ($result.unitExit -ne 0 -or $result.unitFailed -ne 0 -or $result.factoryExit -ne 0 -or $result.factory.success -ne $true))) {exit 1}
