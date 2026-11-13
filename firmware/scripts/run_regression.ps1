[CmdletBinding()]
param(
    [string]$KeilRoot='D:\Keil_32',
    [ValidateSet('Parking','Review','Stop')][string]$Suite='Parking',
    [int]$ExpectedChecks=0
)
$ErrorActionPreference='Stop'
$root=Split-Path -Parent $PSScriptRoot
$review=$Suite -eq 'Review'
$stop=$Suite -eq 'Stop'
$project=Join-Path $root $(if($stop){'Competition_Stop.uvprojx'}elseif($review){'Competition_Review.uvprojx'}else{'Competition_Regression.uvprojx'})
$target=if($stop){'Competition_Stop_Simulator'}elseif($review){'Competition_Review_Simulator'}else{'Competition_Regression_Simulator'}
$ini=Join-Path $root $(if($stop){'tests\competition_stop.ini'}elseif($review){'tests\competition_review.ini'}else{'tests\competition_regression.ini'})
$log=Join-Path $root $(if($stop){'evidence\competition-stop.log'}elseif($review){'evidence\competition-review.log'}else{'evidence\competition-regression.log'})
$expected=if($ExpectedChecks){$ExpectedChecks}elseif($stop){37}elseif($review){166}else{164}
[xml]$options=Get-Content -LiteralPath ([IO.Path]::ChangeExtension($project,'.uvoptx')) -Raw
[xml]$settings=Get-Content -LiteralPath $project -Raw
$ot=$options.ProjectOpt.Target | Where-Object TargetName -eq $target
$pt=$settings.Project.Targets.Target | Where-Object TargetName -eq $target
if($ot.TargetOption.DebugOpt.uSim -ne '1' -or $ot.TargetOption.DebugOpt.uTrg -ne '0' -or
   $pt.TargetOption.DebugOption.Simulator.UseSimulator -ne '1' -or $pt.TargetOption.DebugOption.Target.UseTarget -ne '0' -or
   $pt.TargetOption.Utilities.Flash1.UseTargetDll -ne '0'){throw 'Refusing non-simulator debug target'}
if(Test-Path -LiteralPath $log){
    $archive=Join-Path $root ('evidence\prior-'+$Suite+'-'+[DateTime]::UtcNow.ToString('yyyyMMddTHHmmssfff')+'.log')
    Move-Item -LiteralPath $log -Destination $archive
}
$original=Get-Content -LiteralPath $ini -Raw
[IO.File]::WriteAllText($ini,($original -replace '(?m)^LOG >[^\r\n]*',('LOG >'+$log)),[Text.Encoding]::ASCII)
$proc=Start-Process -FilePath (Join-Path $KeilRoot 'UV4\UV4.exe') -ArgumentList @('-j0','-d',('"{0}"' -f $project),'-t',$target) -WorkingDirectory $root -WindowStyle Hidden -PassThru
try{
    $finished=$false
    for($i=0;$i -lt 60;$i++){
        Start-Sleep -Milliseconds 750
        if(!(Test-Path -LiteralPath $log)){continue}
        $stream=[IO.File]::Open($log,[IO.FileMode]::Open,[IO.FileAccess]::Read,[IO.FileShare]::ReadWrite)
        $reader=[IO.StreamReader]::new($stream)
        try{$text=$reader.ReadToEnd()}finally{$reader.Dispose()}
        if($text.Contains('LOG OFF') -and $text -match 'SELFTEST finished=1 checks=(\d+) failures=(\d+)'){
            Write-Output $text
            if([int]$Matches[1] -ne $expected -or [int]$Matches[2] -ne 0){throw "$Suite regression failed or count changed (expected $expected)"}
            if(!$text.Contains('DEFAULT command=0 status=0 stby=0 pwm_a=0 pwm_b=0')){throw 'Default state is not stopped'}
            $finished=$true;break
        }
    }
    if(!$finished){throw 'Simulator did not finish within 45 seconds'}
}finally{
    if(!$proc.HasExited){Stop-Process -Id $proc.Id -Force}
    [IO.File]::WriteAllText($ini,$original,[Text.Encoding]::ASCII)
}
