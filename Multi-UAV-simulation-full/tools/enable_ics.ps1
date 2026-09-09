# 开启/关闭 ICS：把 WLAN 的外网共享给"以太网"口，给直连的另一台电脑用。
# 必须以管理员身份运行：
#     powershell -ExecutionPolicy Bypass -File tools/enable_ics.ps1
#     powershell -ExecutionPolicy Bypass -File tools/enable_ics.ps1 -Off
#
# 开启后"以太网"口会被强制改成 192.168.137.1，覆盖 mesh 的静态配置；
# 要飞的时候先 -Off，再跑 tools/restore_mesh_nic.ps1 还原。
#
# 生成本文件的两条硬约束（2026-08-21 各踩一次）：
#   1) 必须存成 UTF-8 带 BOM。PowerShell 5.1 默认按 ANSI(GBK) 读 .ps1，
#      无 BOM 的 UTF-8 会让中文变乱码并破坏引号配对。
#   2) 文件内容里一律不要出现反斜杠。生成脚本时它会被当成转义字符，
#      变成真的回车或换行，把一行劈成两行，后半截被当命令执行，
#      而且语法检查查不出来。路径统一用正斜杠。
param([switch]$Off)

$isAdmin = ([Security.Principal.WindowsPrincipal][Security.Principal.WindowsIdentity]::GetCurrent()).IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)
if (-not $isAdmin) {
    Write-Host "需要管理员权限。请在以管理员身份运行的 PowerShell 里执行。" -ForegroundColor Red
    exit 1
}

$PUBLIC  = 'WLAN'
$PRIVATE = '以太网'

$m = New-Object -ComObject HNetCfg.HNetShare
$conns = @{}
foreach ($c in $m.EnumEveryConnection) { $conns[$m.NetConnectionProps($c).Name] = $c }
Write-Host ("检测到的连接: " + ($conns.Keys -join ", "))

# 先全部关掉共享，避免已共享给别的口导致设置失败
foreach ($c in $m.EnumEveryConnection) {
    $cfg = $m.INetSharingConfigurationForINetConnection($c)
    if ($cfg.SharingEnabled) {
        $cfg.DisableSharing()
        Write-Host ("已关共享: " + $m.NetConnectionProps($c).Name)
    }
}

if ($Off) {
    Write-Host "已关闭所有 ICS 共享。" -ForegroundColor Green
    Start-Sleep -Seconds 2
    Get-NetIPAddress -InterfaceAlias $PRIVATE -AddressFamily IPv4 -ErrorAction SilentlyContinue | Format-Table IPAddress, PrefixLength -AutoSize
    Write-Host "若地址被清空或 137.x 仍在，跑 restore_mesh_nic.ps1 还原 mesh 配置。"
    exit 0
}

foreach ($n in @($PUBLIC, $PRIVATE)) {
    if (-not $conns.ContainsKey($n)) {
        Write-Host ("找不到连接 " + $n + "，请改脚本顶部的变量。") -ForegroundColor Red
        exit 1
    }
}
$m.INetSharingConfigurationForINetConnection($conns[$PUBLIC]).EnableSharing(0)
$m.INetSharingConfigurationForINetConnection($conns[$PRIVATE]).EnableSharing(1)
Write-Host ("已开启: " + $PUBLIC + " 外网 -> " + $PRIVATE + " 共享给对端") -ForegroundColor Green
Start-Sleep -Seconds 3
Get-NetIPAddress -InterfaceAlias $PRIVATE -AddressFamily IPv4 | Format-Table IPAddress, PrefixLength -AutoSize
Write-Host "对端把网口设成自动获取 IP，应拿到 192.168.137.x"
