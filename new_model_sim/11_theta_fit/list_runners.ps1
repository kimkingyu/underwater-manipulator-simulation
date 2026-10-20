Get-CimInstance Win32_Process |
    Where-Object { $_.CommandLine -match 'fluent_run\.py|run_steady_batch|fl_mpi2420|fl2420|cx2420' } |
    Select-Object ProcessId, Name, CreationDate |
    Format-Table -AutoSize
