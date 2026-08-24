$ErrorActionPreference = 'Stop'

$keilExe = 'G:\Keil_v5\UV4\UV4.exe'
$projectFile = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\MDK-ARM\chassis_motor.uvprojx')
)
$buildLog = [System.IO.Path]::GetFullPath(
    (Join-Path $PSScriptRoot '..\MDK-ARM\chassis_motor\chassis_motor.build_log.htm')
)

if (-not (Test-Path -LiteralPath $keilExe)) {
    Write-Error "Keil executable not found: $keilExe"
    exit 1
}

Write-Host "Building $projectFile"
$buildStart = Get-Date
& $keilExe -b $projectFile -j0

$deadline = (Get-Date).AddSeconds(60)
$summary = $null
$logText = ''
do {
    if (Test-Path -LiteralPath $buildLog) {
        $logFile = Get-Item -LiteralPath $buildLog
        if ($logFile.LastWriteTime -ge $buildStart.AddSeconds(-2)) {
            $logText = Get-Content -LiteralPath $buildLog -Raw
            $summary = [regex]::Match(
                $logText,
                '"[^\r\n]+\.axf"\s+-\s+(?<errors>\d+) Error\(s\),\s+(?<warnings>\d+) Warning\(s\)\.'
            )
            if ($summary.Success) { break }
        }
    }
    Start-Sleep -Milliseconds 250
} while ((Get-Date) -lt $deadline)

if (($null -eq $summary) -or (-not $summary.Success)) {
    Get-Content -LiteralPath $buildLog | Select-String -Pattern 'error:|warning:|Target not created'
    Write-Error 'Could not find the Keil build summary in the build log.'
    exit 1
}

$errorCount = [int]$summary.Groups['errors'].Value
$warningCount = [int]$summary.Groups['warnings'].Value
Write-Host "Keil build finished: $errorCount error(s), $warningCount warning(s)."

if ($errorCount -ne 0) {
    Get-Content -LiteralPath $buildLog | Select-String -Pattern 'error:|warning:|Target not created'
    exit 1
}

Write-Host (Join-Path $PSScriptRoot '..\MDK-ARM\chassis_motor\chassis_motor.hex')
