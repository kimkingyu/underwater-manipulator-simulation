param([string]$Ids)
$list = $Ids -split '[,\s]+' | Where-Object { $_ } | ForEach-Object { [int]$_ }
foreach ($i in $list) { Stop-Process -Id $i -Force -ErrorAction SilentlyContinue }
Start-Sleep 1
foreach ($i in $list) { if (Get-Process -Id $i -ErrorAction SilentlyContinue) { "alive $i" } else { "gone $i" } }
