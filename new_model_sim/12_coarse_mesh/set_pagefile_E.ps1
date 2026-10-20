# Move the Windows page file from C: to E: (system managed size). Requires admin; takes effect after reboot.
# Same as sysdm.cpl -> Advanced -> Performance -> Advanced -> Virtual memory: C: "No paging file",
# E: "System managed size". Result log: set_pagefile_E.log next to this script.
$log = Join-Path $PSScriptRoot 'set_pagefile_E.log'
"start $(Get-Date -Format s)" | Out-File $log -Encoding utf8
try {
    $cs = Get-CimInstance Win32_ComputerSystem
    "before: AutomaticManagedPagefile=$($cs.AutomaticManagedPagefile)" | Out-File $log -Append
    Get-CimInstance Win32_PageFileSetting | ForEach-Object { "before setting: $($_.Name) $($_.InitialSize) $($_.MaximumSize)" } | Out-File $log -Append
    Set-CimInstance -InputObject $cs -Property @{AutomaticManagedPagefile = $false}
    Get-CimInstance Win32_PageFileSetting | Remove-CimInstance
    New-CimInstance -ClassName Win32_PageFileSetting -Property @{Name = 'E:\pagefile.sys'; InitialSize = [uint32]0; MaximumSize = [uint32]0} | Out-Null
    $cs = Get-CimInstance Win32_ComputerSystem
    "after: AutomaticManagedPagefile=$($cs.AutomaticManagedPagefile)" | Out-File $log -Append
    Get-CimInstance Win32_PageFileSetting | ForEach-Object { "after setting: $($_.Name) $($_.InitialSize) $($_.MaximumSize)" } | Out-File $log -Append
    "OK - reboot required" | Out-File $log -Append
} catch {
    "ERROR $_" | Out-File $log -Append
}
