$s = Get-Counter '\Process(*)\% Processor Time' -SampleInterval 3 -MaxSamples 1
$s.CounterSamples | Where-Object { $_.CookedValue -gt 30 -and $_.InstanceName -notmatch 'idle|_total|fl_mpi' } |
    Sort-Object CookedValue -Descending | Select-Object -First 8 InstanceName, @{n = 'cpu'; e = { [int]$_.CookedValue } } |
    Format-Table -AutoSize
