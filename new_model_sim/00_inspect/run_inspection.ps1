$ErrorActionPreference = 'Stop'
$awp = 'D:\Program Files\ANSYS Inc\v242'
$env:AWP_ROOT242 = $awp
$exe = Join-Path $awp 'scdm\SpaceClaim.exe'
$script = 'D:\work4\new_model_sim\00_inspect\inspect_new_model.py'
$arguments = @('/Headless=True', '/Splash=False', '/Welcome=False', '/ExitAfterScript=True', "/RunScript=`"$script`"")
$proc = Start-Process -FilePath $exe -ArgumentList $arguments -PassThru -WindowStyle Hidden
Write-Output "started pid=$($proc.Id)"
if (-not $proc.WaitForExit(900000)) {
    Stop-Process -Id $proc.Id -Force
    Write-Output 'TIMEOUT: SpaceClaim stopped after 15 minutes'
    exit 2
}
Write-Output "exit=$($proc.ExitCode)"
