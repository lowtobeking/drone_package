# 飞控 MAVLink 工具（2026-08-18 新增）

在机载电脑上跑，通过以太网 MAVLink 直连飞控。**都不需要 QGC**。

## 前置

```bash
python3 -m pip install -i https://pypi.tuna.tsinghua.edu.cn/simple pymavlink
```

连接串一律用 **`udpin:0.0.0.0:14550`**（绑本地端口直接收飞控广播）。
不要用 `udpout` —— 临时本地端口收不到广播，要靠飞控先学到我们，时序不稳
（2026-07-27 台架实测：dry-run 偶通、`--apply` 连续两次失败）。

🔴 **所有工具都在连上后先发一拍 GCS 心跳**，否则飞控不会把 `PARAM_VALUE` 单播回来，
表现为"写入静默失败/回读还是旧值"。自己写新脚本务必照抄这一步。

## 工具

| 脚本 | 用途 | 是否写飞控 |
|---|---|---|
| `sensor_check.py` | 查 `SYS_STATUS` 传感器位图（陀螺/加速度/**磁罗盘**/气压/GPS 的 存在-启用-健康）+ 解码 `CAL_MAG0_ID` | 只读 |
| `dump_params2.py` | 全量参数导出成 QGC `.params` | 只读 |
| `set_param.py` | 通用单参数写，带写前读 + 写后回读校验，默认演练 | `--apply` 才写 |
| `fc_reboot.py` | 重启飞控（先查是否上锁，解锁状态拒绝执行） | 发指令 |
| `uplink_test.py` | 上行通路验证：发 `body_rate` 心跳看 `offboard_control_signal_lost` 翻转 | 不解锁/不发 setpoint |

## 用法

```bash
# 传感器体检（A10 飞控在 10.41.10.2 / A08 在 192.168.77.2）
python3 tools/sensor_check.py udpin:0.0.0.0:14550

# 全量存档（换机型/换配置前务必先存）
python3 tools/dump_params2.py udpin:0.0.0.0:14550 ~/backup.params "备注"

# 改参数：先演练，确认无误再 --apply
python3 tools/set_param.py udpin:0.0.0.0:14550 SYS_HAS_MAG 1
python3 tools/set_param.py udpin:0.0.0.0:14550 SYS_HAS_MAG 1 --apply

python3 tools/fc_reboot.py udpin:0.0.0.0:14550
```

## 已知坑

- **`PARAM_REQUEST_LIST` 在本机飞控上无响应** ⇒ `dump_params2.py` 改成先单读一个参数拿
  `param_count`、再按 index 批量请求。实测 1112/1112 一轮抓全。
- **`fc_reboot.py` 末尾的"60s 内没等到心跳"是假报警** —— 第二个连接抢不到已被占用的
  14550 端口。以 `ping 飞控IP` + `sudo tcpdump -i <口> -n port 14550` 为准。
- **`SYS_STATUS` 里 GPS 的 present 位反映定位状态、不是硬件在位**。查硬件要看
  `/fmu/out/vehicle_gps_position` 的 `device_id`（非零 = 驱动认到硬件）。
- **传感器相关参数改完必须重启飞控**（PX4 启动时读标定），否则 `sensor_check` 看不到变化。
- `uplink_test.py` 必须用 `body_rate`：velocity/position/acceleration 三种模式
  PX4 都会查 `local_position` 有效性，室内无定位必被拒，与链路好坏无关。
