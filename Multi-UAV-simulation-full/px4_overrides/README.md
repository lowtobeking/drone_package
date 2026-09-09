# PX4 SITL 本地改动备份（px4_overrides）

本目录固化的是对 **PX4-Autopilot v1.14**（`~/PX4-Autopilot-1.14/`）的本地 SITL 定制改动。
这些改动是多机 grid9 仿真能干净起飞 / 真高一致的必要环境，但它们：

- **不在本项目（`mpc_control`）的正常构建路径里**，`colcon build` 不会带上；
- PX4 仓库虽是 git，但**没有我们自己的远程**，改动只活在本地工作树 / PX4 本地 git，一旦
  误 `git checkout` / 重装 PX4 / 机器故障就会**永久丢失**。

因此在此单独备份（patch + 整文件副本），随本项目 git 一起推到远程，作为唯一可靠备份。

> 生成日期：2026-06-25。base = PX4 v1.14 stock airframe `4001_gz_x500` / world `default.sdf`。

---

## 文件清单

| 文件 | 说明 |
|------|------|
| `4001_gz_x500` | 改好的 airframe **整文件副本**，应急可直接 cp 覆盖 |
| `4001_gz_x500.patch` | airframe 相对 PX4 stock 的 `git diff`（精确改动） |
对应的 PX4 路径：

```
airframe:  ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500
world:     Tools/simulation/gz/worlds/default.sdf
```

---

## 改动一：airframe `4001_gz_x500`（3 个 param override）

均为 `param set-default`，治多机 SITL 起飞 / 长跑可靠性，**与控制器无关**：

| param | 值 | 为什么 |
|-------|----|--------|
| `SIM_BAT_MIN_PCT` | `100` | 仿真电池不掉电。stock `SIM_BAT_DRAIN=60` 会 60s 掉到 50% → 长跑触 "Battery unhealthy" 预检失败 → 挡住 re-arm。钉死 100%。（注：实测此项偏红鲱鱼，留着无害） |
| `COM_OBL_RC_ACT` | `5` | offboard 信号丢失时进 **Hold 悬停**（而非降落），各机自主悬停等恢复 |
| `COM_DISARM_PRFLT` | `0` | **关键修复**。关掉 10s 自动预检解锁砍刀。起飞瞬态 offboard setpoint 断续会把 nav 在地面 OFFBOARD↔Hold 来回弹，默认 10s 砍刀会在飞机爬出去之前 disarm 趴地 → retry 死循环。`0` = 永不自动 disarm，给无限时间爬出去 |

### 应用方法

```bash
PX4=~/PX4-Autopilot-1.14
AIR=ROMFS/px4fmu_common/init.d-posix/airframes/4001_gz_x500

# 方式 A：整文件覆盖（最省事）
cp px4_overrides/4001_gz_x500 "$PX4/$AIR"

# 方式 B：打 patch（保留 PX4 端其它改动时用）
git -C "$PX4" apply /path/to/px4_overrides/4001_gz_x500.patch

# 两种方式都要再同步到 build 副本（改 airframe 无需重编，cp 即可）：
cp "$PX4/$AIR" "$PX4/build/px4_sitl_default/etc/init.d-posix/airframes/4001_gz_x500"
```

---

## `default.sdf` 物理步长（已验证回退 stock，无需保留）

固化时曾发现本地 `default.sdf` 的 `max_step_size` 被改为 `0.001`（stock = `0.004`），
来历不明（推测为 2026-04-30 项目初期误留）。

**结论（2026-06-25 S8 验证）**：

- 已回退至 stock `0.004`；`git -C ~/PX4-Autopilot-1.14 diff` 显示无差异。
- 在 stock 步长下完整跑了 S8_grid9_circle（891s，9 机）：
  - pos_err 稳态 0.10–0.11 m，min_spacing 2.30 m，0 违规，0 RELINQUISH
  - gz 真高 4.01–4.23 m（scatter 0.22 m，优于 run6 的 0.25 m）
- **仿真速度提升约 4 倍**（RTF 上限从 0.25 恢复至 1.0），控制质量无退化。
- `default_sdf_max_step.patch` 已删除，`default.sdf` 与 PX4 stock 完全一致，**不再需要本备份**。

若将来需要确认步长：
```bash
grep max_step_size ~/PX4-Autopilot-1.14/Tools/simulation/gz/worlds/default.sdf
# 期望输出：<max_step_size>0.004</max_step_size>
```
