# 还原"以太网"口的 mesh 静态配置（原值记录于 2026-08-21 开 ICS 之前）。
# 必须以管理员身份运行：
#     powershell -ExecutionPolicy Bypass -File tools/restore_mesh_nic.ps1
#
# 写文件的两条硬约束见 enable_ics.ps1 顶部（UTF-8 带 BOM；注释里不写反斜杠路径）。

$ifAlias = '以太网'

$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "需要管理员权限。请在'以管理员身份运行'的 PowerShell 里执行。" -ForegroundColor Red
    exit 1
}

$i = (Get-NetAdapter -Name $ifAlias).ifIndex
Write-Host ("还原 " + $ifAlias + " (ifIndex=" + $i + ") 到 mesh 静态配置...")

Get-NetIPAddress -InterfaceIndex $i -AddressFamily IPv4 -ErrorAction SilentlyContinue |
    Remove-NetIPAddress -Confirm:$false -ErrorAction SilentlyContinue
Remove-NetRoute -InterfaceIndex $i -DestinationPrefix "0.0.0.0/0" -Confirm:$false -ErrorAction SilentlyContinue

New-NetIPAddress -InterfaceIndex $i -IPAddress 192.168.1.123 -PrefixLength 24 -DefaultGateway 192.168.1.1 | Out-Null
Set-DnsClientServerAddress -InterfaceIndex $i -ServerAddresses 8.8.8.8

Write-Host "完成。核对:" -ForegroundColor Green
Get-NetIPAddress -InterfaceIndex $i -AddressFamily IPv4 | Format-Table IPAddress, PrefixLength -AutoSize
