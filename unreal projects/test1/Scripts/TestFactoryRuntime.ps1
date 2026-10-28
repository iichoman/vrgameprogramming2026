param(
    [string]$EngineRoot = 'C:\Program Files\Epic Games\UE_5.8',
    [ValidateRange(30, 900)][int]$TimeoutSeconds = 180
)

$ErrorActionPreference = 'Stop'
$factoryProjectDir = Split-Path $PSScriptRoot -Parent
$factoryProjectFile = (Join-Path $factoryProjectDir 'test1.uproject').Replace('\', '/')
$factoryEditorCmd = Join-Path $EngineRoot 'Engine\Binaries\Win64\UnrealEditor-Cmd.exe'
$factoryInstallScript = (Join-Path $PSScriptRoot 'install_factory_runtime.py').Replace('\', '/')
$factoryReportDir = Join-Path $factoryProjectDir 'Saved'

foreach ($factoryRequiredFile in @($factoryProjectFile, $factoryEditorCmd, $factoryInstallScript)) {
    if (-not (Test-Path -LiteralPath $factoryRequiredFile -PathType Leaf)) {
        throw "Required file is missing: $factoryRequiredFile"
    }
}
if ($factoryProjectFile -match '[^\x00-\x7F]') {
    throw 'Run this Unreal project from an ASCII path.'
}

# This runs fresh headless processes against the compiled module. It does not
# build, modify saved maps, or overwrite the user's normal layout_A/B files.
# Native tests use Saved/FactoryLayouts/AutomatedTests instead.
$factoryCommonArguments = @(
    '-unattended', '-nosplash', '-nosound', '-nullrhi', '-NoSourceControl',
    '-stdout', '-FullStdOutLogOutput',
    '-ini:Engine:[/Script/Engine.RendererSettings]:r.GenerateMeshDistanceFields=False',
    '-ini:Engine:[/Script/Engine.RendererSettings]:r.RayTracing=False'
)

function Invoke-FactoryCheck {
    param(
        [string[]]$CheckArguments,
        [string]$ReportName,
        [string]$LogName
    )
    $factoryStartedUtc = [DateTime]::UtcNow
    # Start-Process joins ArgumentList strings. Quote each complete argument so
    # the space in 'unreal projects' survives without a second command shell.
    $factoryAllArguments = @($factoryProjectFile) + $CheckArguments +
        @('-log=' + $LogName) + $factoryCommonArguments
    $factoryQuotedArguments = @($factoryAllArguments | ForEach-Object {
        if ($_.Contains('"')) { throw 'An argument unexpectedly contains a quote.' }
        '"' + $_ + '"'
    })
    $factoryProcess = Start-Process -FilePath $factoryEditorCmd -ArgumentList $factoryQuotedArguments `
        -WorkingDirectory $factoryProjectDir -WindowStyle Hidden -PassThru
    if (-not $factoryProcess.WaitForExit($TimeoutSeconds * 1000)) {
        Stop-Process -Id $factoryProcess.Id -ErrorAction SilentlyContinue
        throw "Factory check timed out. See Saved/Logs/$LogName."
    }
    $factoryProcess.Refresh()
    if ($factoryProcess.ExitCode -ne 0) {
        throw "Factory check failed (exit $($factoryProcess.ExitCode)). See Saved/Logs/$LogName."
    }
    $factoryReportPath = Join-Path $factoryReportDir $ReportName
    if (-not (Test-Path -LiteralPath $factoryReportPath -PathType Leaf)) {
        throw "Factory check did not create its report: $ReportName"
    }
    $factoryReportInfo = Get-Item -LiteralPath $factoryReportPath
    if ($factoryReportInfo.LastWriteTimeUtc -lt $factoryStartedUtc) {
        throw "Stale report rejected: $ReportName"
    }
    $factoryReport = Get-Content -LiteralPath $factoryReportPath -Raw -Encoding UTF8 | ConvertFrom-Json
    if ($factoryReport.success -ne $true) {
        throw "Report contains a failure: $ReportName. See Saved/Logs/$LogName."
    }
    return $factoryReport
}

$factoryColdReport = Invoke-FactoryCheck -CheckArguments @(
    '-run=pythonscript', "-script=$factoryInstallScript", '-FactoryRuntimeVerifyOnly',
    '-EnablePlugins=PythonScriptPlugin,EditorScriptingUtilities'
) -ReportName 'FactoryRuntimeColdLoadReport.json' -LogName 'FactoryRuntimeColdLoad.log'

$factoryRuntimeReport = Invoke-FactoryCheck -CheckArguments @(
    '/Game/DigitalTwin/Maps/factory_p1', '-game', '-FactoryRuntimeAutoTest'
) -ReportName 'FactoryRuntimeTestReport.json' -LogName 'FactoryRuntimeTests.log'

if (@($factoryRuntimeReport.checks).Count -eq 0) {
    throw 'The runtime report contains no executed checks.'
}
$factoryFailedChecks = @($factoryRuntimeReport.checks | Where-Object { $_.passed -ne $true })
if ($factoryFailedChecks.Count -gt 0) {
    throw ('Failed native checks: ' + (($factoryFailedChecks | ForEach-Object { $_.check }) -join ', '))
}

Write-Output ("PASS: factory_p1 cold-load audit and {0} native runtime checks." -f @($factoryRuntimeReport.checks).Count)
Write-Output 'UI pointer interaction and rendered Korean labels require separate visual/manual checks.'
