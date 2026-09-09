# mpc_port — ROS MPC 完整移植 Simulink 闭环验证

把 `mpc_control/mpc_node.py` 的 MPC **算法本体**完整移植到 MATLAB/Simulink，
在 6DOF+串级 PID 被控对象（与 PID 基线同一植物）上闭环，剥离 ROS/EKF/通信
工程噪声，单独验证算法稳定性。

## 真机部署映射（每文件 = 一个真机检验单元）

| 入口文件 | 机数 | 运动 | 真机对应 |
|---|---|---|---|
| `run_mpc_solo1_hover/line/circle.m` | 1 | 悬停/直线/圆周 | 单机首飞三连 |
| `run_mpc_pair2_hover/line/circle.m` | 2 | 同上 | 双机编队 |
| `run_mpc_trio3_hover/line/circle.m` | 3 | 同上 | 三机编队 |
| `run_mpc_cross5_hover/line/circle.m` | 5 | 同上 | 五机十字编队 |

真机测哪个场景，先跑对应文件确认 `VERDICT: PASS/REVIEW`。
输出：`report/figures/mpc_<formation>_<mode>.png` + 本目录 `results.csv`。

## 移植对照（→ mpc_node.py）

| 本目录 | 移植自 | 说明 |
|---|---|---|
| `formation_cfg.m` | scenarios.yaml defaults+formations | 参数单一真值（N=30, q_vel=2, max_accel=4 等） |
| `leader_state.m` | leader_node | hover/line(端点减速)/circle(向心加速度广播) |
| `mpc_precompute.m` | `DoubleIntegratorMPC._setup_ocp` | 双积分精确离散 + 凝聚矩阵 + 常量约束 |
| `mpc_solve_rti.m` | `DoubleIntegratorMPC.solve` (SQP_RTI) | 单次 GN 迭代；碰撞/编队残差在上拍预测处线性化 |
| `qp_admm.m` | (acados HPIPM 的替身) | OSQP 风格 ADMM；本机无优化工具箱 |
| `mpc_swarm_step.m` | `control_loop` | 参考二阶预测/邻居预测交换(一拍延迟)/速度合成(xy=MPC, z=纯P)/限幅 |
| `build_plant_io.m` `build_swarm_model.m` | — | 母版 slx → IO 化 → n 机闭环模型(50Hz Interpreted MATLAB Fcn) |
| `scenario_run.m` | verify_formation.py 判定口径 | 指标+三级 verdict(PASS/REVIEW/FAIL)+图 |

## 与 ROS 端的已知一致性

line/circle 的稳态 track_err≈0.5–1.1m 为**共模滞后**（form_err≈0，队形完好）——
与 Ubuntu 实跑 trio3 circle 1.129m 同源（速度内环无前馈滞后），判 REVIEW 非缺陷。
编队反馈、碰撞避免、邻居降级（队形推断）行为与 ROS 端同构。

## 注意

- 跑前无需手动 init：`scenario_run` 自动把 `../init.m` 注入 base 工作区。
- 改 `formation_cfg.m` 参数后直接重跑；改模型结构需删 `uav_plant_io.slx` /
  `mpc_swarm_*.slx` 强制重建（或 `build_*( true)`）。
- 与 scenarios.yaml 的参数同步是手动的：改 yaml 记得改 `formation_cfg.m`。

## 分布式架构 — swarm_distributed.slx

与上面的集中式模型（1 个 MATLAB Fcn 内部 `for i=1:n` 算全部机）不同，这套
按"每架无人机一个机载计算机"的真实部署结构建模：N 机的动力学
（`uav_plant_io` 拷贝）和 N 份**完全相同**的机载计算机（`onboard_computer`
拷贝，仅 `my_id` 字面量不同）各自独立、结构对称；机载计算机的输入线在
图上**只连得到 `cfg.nbrs{i}` 对应的邻居广播段**（由 Selector 精确摘取），
不像集中式版本那样把全部 N 机状态一次性送进同一个函数内部再自行取舍——
"仅交互通信范围内邻居飞行状态数据"是接线决定的结构性事实，不是代码内部
的君子协定。MPC 与二阶一致性两套算法都在同一个文件里，由 workspace 变量
`CTRL_MODE`（1=MPC，2=一致性）整体二选一，全部 N 机同步切换；`ctrl_mode`
信号同时拼进两分支输入 Mux 末位做**门控**——未选中的分支直接回零不进
求解器（Multiport Switch 只选输出不阻止分支执行，没有这道门时跑一致性
模式 N 份 MPC ADMM QP 每步白算）。

**文件**：
| 文件 | 作用 |
|---|---|
| `mpc_agent_step.m` / `consensus_agent_step.m` | 单机控制律（从集中式 for 循环体拆出，与集中式版本逐拍数值 bit-exact，见 `test_agent_equiv.m` 风格验证） |
| `mpc_agent_step_packed.m` / `consensus_agent_step_packed.m` | Interpreted MATLAB Fcn 包装（该块只支持单输入单输出向量） |
| `build_onboard_computer.m` | 生成 `onboard_computer.slx`（4 入 2 出：t/self_xv/nbr_raw/nbr_pred → cmd/pred_out；双分支 + Multiport Switch + ctrl_mode 门控） |
| `build_distributed_swarm_model.m` | 生成/覆盖重建 `swarm_distributed.slx`（**固定文件名，磁盘上任何时候只有这一个**）：N 份动力学拷贝 + N 份机载计算机拷贝 + 全局广播总线(瞬时态 + 一拍延迟预测态) + 每机 Selector 只摘邻居段 + UAV Animation |
| `run_distributed.m` | 交互式一行入口（自动 init/cfg/开关/出生点/开图/仿真+动画） |

**用法（交互式看动画）**：
```matlab
run_distributed('cross5', 'circle')               % MPC
run_distributed('cross5', 'circle', 'consensus')  % 一致性
```

**用法（批量指标/成绩单，自动注掉动画）**：
```matlab
scenario_run('pair2', 'hover', 'mpc_dist')        % → results_distributed.csv + mpcd_*.png
scenario_run('pair2', 'hover', 'consensus_dist')  % → results_distributed_consensus.csv + consd_*.png
```

**手动底层用法**：
```matlab
global MPC_CFG CONS_CFG
MPC_CFG  = formation_cfg('cross5', 'circle');   % 建模拓扑 + MPC 分支运行时都读它
CONS_CFG = consensus_cfg('cross5', 'circle');   % 只跑 MPC 可不设(门控后未选中分支不读 cfg 本体)
mdl = build_distributed_swarm_model(true);      % 第2参 with_anim(默认true)
for i = 1:MPC_CFG.n
    set_param(sprintf('%s/drone_%d/6DOF (Euler Angles)', mdl, i-1), 'xme_0', mat2str(MPC_CFG.births(i,:)));
end
set_param(mdl, 'StopTime', num2str(MPC_CFG.T));
assignin('base', 'CTRL_MODE', 1);   % 1=MPC, 2=一致性；全部 N 机同步切换，不是逐机各选
sim(mdl);
```

**换队形/改机数**：磁盘上只有 `swarm_distributed.slx` 这一个文件——换一个
`formation_cfg(...)` 场景（solo1=1 机 … grid9=9 机，或自定义 births/nbrs）
重跑 `build_distributed_swarm_model(...)` 即可**覆盖重建这同一个文件**，
几秒内完成，不是另存一份、也无需手动增删任何拷贝。函数会用一个同名
`.cfg.mat` 旁路文件记录当前文件对应的 formation/n：只要与调用时的
`MPC_CFG` 不一致，即使 `force=false` 也会自动重建，避免"文件存在就直接
复用"导致悄悄跑错机数。
⚠️ 改过 `onboard_computer` 模板或 packed 函数的 u 布局后，务必 `force=true`
重建一次，否则旧文件的内部布局会与新代码错位。

**数值验证**：pair2(MPC+一致性)、cross5(MPC，臂机度 1/中心度 4 混合邻居数)、
grid9(MPC，9 机最大规模)均与已验证的集中式模型逐点误差 `0.000e+00`——
纯架构重排(集中1个函数拆成N个结构对称的实例+边界化接线)，数学完全不变。

**UAV Animation 使用注意**（两个坑，均已在生成脚本里修好）：
- `NumUAV>1` 时该模块要求 Translation/Rotation 是真正的 `[N x 3]` 2-D 矩阵
  （见 `UAVAnimation.m` 的 `validateInputsImpl`），Mux 拼出来的只是 1-D 向量、
  不满足；已用 Reshape(→`[1,3]`) + Matrix Concatenate(按行堆叠) 处理。
- **该模块 SampleTime 默认继承基速率**（本项目 FixedStep=0.004s≈250Hz），
  不显式设的话渲染逐积分步触发，50s 场景能拖到近 50 分钟（曾一度误判为
  "卡死"，实为未限速导致的极慢）。生成脚本已显式设 `SampleTime=0.1`
  （10Hz，肉眼观感足够流畅），同样 50s 场景实测 ~74s——`-batch`/交互式
  会话都能正常出动画，不再需要绕开。
- 若仍想跳过动画加速纯数值验证/批量回归，可用
  `build_distributed_swarm_model(force, false)`（第2参 `with_anim=false`）。
