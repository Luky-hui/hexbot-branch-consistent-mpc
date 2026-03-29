# Hexbot Codex 交接手册（2026-03-27）

> 提示
>
> 这份文档对应 2026-03-27 的项目状态。
> 当前最新、已和 `phase33b` 主线对齐的交接手册是：
> `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_claude_handoff_20260328_phase33b.md`

## 1. 项目背景

本项目当前的总目标只有一个：

- 在现有 `ROS 2 + Isaac Sim + Nav2` 架构上，推进 Hexbot 六足机器人的 `MPC + WBC` 主线

当前主线不是继续调快，而是先把这条方案做成：

- 可重复 bring-up
- 可闭环验收
- 可形成旧 baseline vs 新主线的 A/B 证据
- 可直接支撑中国发明专利技术交底书

当前建议主张的系统方案是：

- 宿主机 `ROS 2 Humble` 跑上层运行时、bridge、Isaac 侧话题
- 容器 `ROS 2 Jazzy` 跑 OCS2 Hexbot 主体
- 保持现有 `Nav2 + RTAB-Map + Isaac` 上层链路不推翻
- 主命令链为  
  `/cmd_vel -> /hexbot_mpc/input/cmd_vel -> OCS2/MPC -> /hexbot_mpc/output/joint_command -> /joint_command`

## 2. 关键工作区与包

工作区根目录：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws`

源码根目录：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src`

关键包：

- `hexbot_description_ros2`
- `hexbot_isaac_runtime`
- `hexbot_locomotion_v2`
- `hexbot_locomotion_mpc`
- `ocs2_hexbot_legged_robot`
- `ocs2_hexbot_legged_robot_ros`

上游 OCS2 临时源码：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/.tmp_upstream/ocs2_ros2/ocs2-ros2`

## 3. 双环境说明

宿主机：

- `ROS 2 Humble`
- 负责 Isaac 话题、bridge、baseline 运行时、闭环探针

容器：

- `ROS 2 Jazzy`
- 负责 OCS2 编译与运行

推荐的宿主机环境加载顺序：

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

推荐的容器环境加载顺序：

```bash
source /opt/ros/jazzy/setup.bash
source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

当前常用容器名：

- `ocs2_hexbot_jazzy`

## 4. 已有构建与运行脚本

宿主机构建：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/host_build_runtime_stack.sh`

容器环境与构建：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_setup_env.sh`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_build_hexbot_stack.sh`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_run_hexbot_jazzy.sh`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_exec_hexbot_jazzy.sh`

运行时清场：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/cleanup_hexbot_runtime.sh`

说明：

- `host_build_runtime_stack.sh` 现已使用 `build_humble/install_humble/log_humble`
- `container_build_hexbot_stack.sh` 现已使用 `build_jazzy/install_jazzy/log_jazzy`
- 目的是隔离 Humble/Jazzy 产物，避免互相污染

## 5. 当前确认可用的核心工具

### 5.1 宿主机运行态/闭环探针

运行态探针：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py`

闭环探针：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py`

闭环一键入口：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/scripts/run_patent_closed_loop_case.sh`

Isaac clean start 预检：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/ensure_isaac_clean_start.py`

### 5.2 Isaac 远程执行与控制

终端侧执行器：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_vscode_exec.py`

常见运行控制：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py`

已确认可用的 Isaac 侧端口：

- `127.0.0.1:8226`
  用于把 Python 代码直接发进当前 Isaac Sim 实例
- `127.0.0.1:3000`
  用于 VS Code attach 调试

常用命令：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py status
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py stop
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py play
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py reload-stage
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py reload-and-play
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py articulation-roots
```

当前确认的 articulation root：

- `/hexbot/hexbot`

### 5.3 Isaac 资产修复工具

主要脚本位于：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac`

关键文件：

- `reimport_hexbot_urdf_asset.py`
- `synthesize_hexbot_foot_colliders.py`
- `apply_hexbot_isaac_drives.py`
- `audit_hexbot_isaac_setup.py`
- `run_hexbot_isaac_repair_pipeline.py`
- `hexbot_drive_profile.json`

## 6. 当前已经确认过的系统事实

### 6.1 OCS2 / bridge / runtime 侧

已经确认的事实：

- Humble 宿主机和 Jazzy 容器双环境链路已经打通过
- `hexbot_locomotion_mpc_bridge.py` 已支持按 Isaac `joint_states.name` 对后端关节顺序重排
- `HexbotRuntimeMrtNode.cpp` 已加入：
  - 非有限 observation 丢弃
  - 非有限 policy 丢弃
  - policy 过期熔断
  - 大幅等价角跳变保护
- `LeggedRobotVisualizer` 已从全局 `/joint_states` 改为  
  `/hexbot/visualization/joint_states`
  避免污染真实反馈
- `GaitReceiver.cpp` 的时间窗 bug 已修为  
  `finalTime + timeHorizon`

### 6.2 已确认的 OCS2 当前局限

已经确认但尚未解决的事实：

- `hexbot_mpc_target` 可以是正确的中性站姿
- `solver_diagnostics` 可以保持 `ok`
- 但 solver 仍可能在第一帧就把某些关节推向错误极端平衡点
- mixed 工况下仍存在明显欠跟踪，尤其 yaw
- 当前不要把“继续提速”当成第一主线

## 7. 当前最重要的新根因：Isaac reimport 资产层

到 2026-03-27 当前轮次为止，最前面的真实阻塞已经前移到 Isaac 资产层：

- 用户多次观察到：
  `躯干持续向中心下沉，随后偏向 L 侧倒塌；tarsus 先贴地，后续支撑失效`
- 在无 ROS 控制发布者的情况下，机器人也能保持趴地状态
- 直接读取 live stage 和磁盘上的  
  `hexbot_isaac_rooted_reimport.usd`
  后确认：
  - `/hexbot/hexbot` 带坏的 `xformOp:translate/orient`
  - 多个关节带坏的 `state:angular:physics:position`
- 这说明 `reload-stage` 重新打开的是同一个坏资产，不是 clean reset

对照资产：

- 干净旧资产：  
  `hexbot_description_ros2/urdf/hexbot_isaac_rooted/hexbot_isaac_rooted.usd`
- 当前问题资产：  
  `hexbot_description_ros2/urdf/hexbot_isaac_rooted_reimport/hexbot_isaac_rooted_reimport.usd`

## 8. 当前 Isaac 现场状态与风险

本轮最后一次关键操作：

- 通过 `8226` 远程口，在当前 live Isaac 实例里执行了  
  `run_hexbot_isaac_repair_pipeline.py --write --skip-inspect`

随后出现：

- Isaac Sim 卡住
- `hexbot_isaac_rooted_reimport.usd` 从 `163142` 字节变成 `687` 字节
- `file` 仍显示它是 `USD crate, version 0.8.0`

当前必须把这份 reimport USD 视为：

- 本轮 live write 之后的可疑资产
- 在重新生成并重新审计前，不应继续作为可信仿真基线

## 9. 当前 drive profile 状态

这一轮已修改：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/hexbot_drive_profile.json`

最新状态：

- `robot_usd` 已指向  
  `../urdf/hexbot_isaac_rooted_reimport/hexbot_isaac_rooted_reimport.usd`
- `joint_targets_deg` 已对齐  
  `ocs2_hexbot_legged_robot/config/command/reference.info`
  的 18 关节中性站姿

含义：

- 以后重新生成并应用 drive 时，默认站立目标不再是全 0 角
- 但这一步不能替代“先恢复 clean reimport 资产”

## 10. 下一任 Codex 的建议接手顺序

### 第一步：先处理 Isaac 冻结与资产可信性

1. 恢复或重启 Isaac Sim
2. 确认 `8226` 重新可访问
3. 检查  
   `hexbot_isaac_rooted_reimport.usd`
   是否仍是当前这份 `687` 字节的小 crate
4. 如果仍异常：
   - 不要在当前 live stage 上继续写盘
   - 在独立、干净的 Isaac 进程里重新执行 repair pipeline
   - 重新审计生成后的 reimport USD

### 第二步：先恢复 clean reset，再恢复 gait / OCS2

必须先验证：

- 无 OCS2、无 bridge，仅 Isaac 资产本身能否 clean reset
- 必要时仅靠 drive 默认目标是否能维持站立，不再直接趴地

### 第三步：再回到 OCS2 / gait

资产 clean 后，恢复以下顺序：

1. `cleanup_hexbot_runtime.sh`
2. Isaac clean reset
3. 宿主机 bridge bring-up
4. Jazzy 容器 OCS2 bring-up
5. 先跑 stance / 低速直行 / 纯转向
6. 再做 A/B 证据沉淀

## 11. 当前工作要求

当前阶段的正确主线是：

- 先把 Isaac reimport 资产和 clean reset 修干净
- 再恢复 Hexbot 基础站立与步态验证
- 再恢复 OCS2 闭环与专利证据链

当前不应优先做：

- 继续提速
- 继续做 endurance
- 继续做 Nav2 / RTAB-Map 耦合

## 12. 后续核心目标

短期核心目标：

- 恢复可信的 `hexbot_isaac_rooted_reimport.usd`
- 让机器人在 Isaac 中先稳定站住
- 然后再验证基础 walking / turning

中期核心目标：

- 继续推进  
  `/cmd_vel -> OCS2/MPC -> /hexbot_mpc/output/joint_command -> /joint_command`
  的稳定闭环
- 形成旧 baseline vs 新 OCS2 主线的 A/B 数据

最终目标：

- 把这条“六足 OCS2 + 跨 Humble/Jazzy + bridge 闭环 + 现有导航链路兼容”的系统方案做成可专利化的成品
