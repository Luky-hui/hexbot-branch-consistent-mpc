# OCS2 Hexbot 适配说明

## 目标

在不推翻现有 `hexbot_description_ros2`、`hexbot_isaac_runtime`、`hexbot_locomotion_v2` 资产的前提下，把上游四足 `ocs2_legged_robot` 恢复并迁移成 Hexbot 六足版本。

## 当前拆分

- `hexbot_locomotion_mpc`
  - 负责 ROS 2 入口边界、就绪性检查、种子配置管理。
  - 不负责求解器本体。
- `ocs2_hexbot_legged_robot`
  - 负责上游 OCS2 足式机器人核心代码的六足化恢复。
- `ocs2_hexbot_legged_robot_ros`
  - 负责 launch、可视化、ROS 节点壳层。

## 已经确认的硬约束

- 上游 `ocs2` 的 `ros2` 分支主要面向 Ubuntu 24.04 + ROS 2 Jazzy。
- 上游 `ocs2_legged_robot` 例子不是通用 N 足模板，原始代码里存在多处四足假设。
- 当前工作区恢复出来的是“源码骨架 + Hexbot 配置”，不是已经打通的最终闭环。

## 已完成的关键恢复

- 六足 `ModeNumber` 位掩码：`RR/RM/RF/LR/LM/LF/TRIPOD_A/TRIPOD_B/STANCE`
- 六足 `feet_array_t` / `contact_flag_t`
- Hexbot `jointNames`
- Hexbot `contactNames3DoF`
- Hexbot 默认站姿和 gait seed
- ROS 侧默认 URDF / launch 入口

## 下一阶段

1. 在独立 overlay 或 Jazzy/容器环境中尝试编译 `ocs2_hexbot_legged_robot` 和 `ocs2_hexbot_legged_robot_ros`
2. 继续搜索并替换上游残留的四足专用逻辑
3. 把 OCS2 输出稳定接回 `/hexbot_mpc/output/joint_command`
4. 再由 `hexbot_locomotion_mpc_bridge.py` 转回现有运行时 `/joint_command`
