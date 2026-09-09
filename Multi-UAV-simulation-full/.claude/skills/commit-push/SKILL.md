---
name: commit-push
description: 将当前所有改动提交并推送到 GitHub。自动生成简洁的 commit 信息，用户可修改后确认。适合在每次代码迭代后快速保存进度。
disable-model-invocation: false
---

用户运行 /commit-push 时，执行以下步骤：

1. 运行 `git -C "C:/Users/Aaron/Desktop/5机仿真" status` 查看改动文件列表，展示给用户。

2. 根据改动内容生成一条简洁的 commit 信息（中文或英文均可），格式：
   - `feat: ` 新功能
   - `fix: ` 修复问题
   - `tune: ` 参数调整
   - `refactor: ` 代码重构
   示例：`tune: lower max_speed to 3m/s, increase neighbour_timeout to 2s`

3. 询问用户是否使用该 commit 信息，或提供自定义信息。

4. 执行（用 `git add -A` 暂存全部改动——含 `mpc_control/` 模块、`launch/`、`config/`、根脚本、文档，以及删除/新增；`.gitignore` 已自动排除 `__pycache__`/`build`/`acados_*`，无需手动列文件）：
   ```
   git -C "C:/Users/Aaron/Desktop/5机仿真" add -A
   git -C "C:/Users/Aaron/Desktop/5机仿真" commit -m "<确认的 commit 信息>"
   git -C "C:/Users/Aaron/Desktop/5机仿真" push origin main
   ```

5. 显示推送结果，确认 GitHub 上已更新。

注意：
- **绝不再手动列根目录文件名**。旧版 skill 曾 `add mpc_node.py swarm_launch.py leader_node.py ...`，但这些根文件是重构前的过时副本（已删除）；ROS2 实际编译的源码在 `mpc_control/` 模块与 `launch/`、`config/`。手动列名会提交错文件、漏掉真实改动，导致 Ubuntu 端 `colcon build` 拉到旧代码。`git add -A` 才能保证正确文件被提交。
- 如果 push 失败（rejected），先运行 `git pull origin main` 合并后再推送。
