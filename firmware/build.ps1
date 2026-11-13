[CmdletBinding()]
param(
    [string]$KeilRoot='D:\Keil_32',
    [ValidateSet('All','Firmware','Parking','Review','Stop')][string]$Target='All'
)
$ErrorActionPreference='Stop'
$uv4=Join-Path $KeilRoot 'UV4\UV4.exe'
if(!(Test-Path -LiteralPath $uv4)){throw "Keil not found: $uv4"}
$jobs=@(
    @{Key='Firmware';Project='Competition';Target='Car_Competition_STLink';Output='car_competition'},
    @{Key='Parking';Project='Competition_Regression';Target='Competition_Regression_Simulator';Output='competition_regression'},
    @{Key='Review';Project='Competition_Review';Target='Competition_Review_Simulator';Output='competition_review'},
    @{Key='Stop';Project='Competition_Stop';Target='Competition_Stop_Simulator';Output='competition_stop'}
)
foreach($job in $jobs){
    if($Target -ne 'All' -and $Target -ne $job.Key){continue}
    $log=Join-Path $PSScriptRoot ('evidence\build-'+$job.Target+'.log')
    # Compile only. Physical download/debug switches are deliberately absent.
    $process=Start-Process -FilePath $uv4 -ArgumentList @('-j0','-r',
        ('"{0}"' -f (Join-Path $PSScriptRoot ($job.Project+'.uvprojx'))),'-t',$job.Target,
        '-o',('"{0}"' -f $log)) -WorkingDirectory $PSScriptRoot -WindowStyle Hidden -Wait -PassThru
    $text=Get-Content -LiteralPath $log -Raw
    Write-Output $text
    if($process.ExitCode -ne 0 -or !$text.Contains('0 Error(s), 0 Warning(s).')){throw "Build not clean: $($job.Target)"}
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot ('Listings\'+$job.Target+'\'+$job.Output+'.map')) -Destination (Join-Path $PSScriptRoot ('evidence\'+$job.Output+'.map'))
    if($job.Key -eq 'Firmware'){
        foreach($ext in @('axf','hex')){
            Copy-Item -LiteralPath (Join-Path $PSScriptRoot ('Objects\'+$job.Target+'\'+$job.Output+'.'+$ext)) -Destination (Join-Path $PSScriptRoot 'release')
        }
        & (Join-Path $KeilRoot 'ARM\ARMCC\bin\fromelf.exe') --bin --output (Join-Path $PSScriptRoot 'release\car_competition.bin') (Join-Path $PSScriptRoot 'release\car_competition.axf')
        if($LASTEXITCODE -ne 0){throw 'fromelf failed'}
    }
}
