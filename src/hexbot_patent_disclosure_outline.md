# Hexbot 发明专利技术交底书目录

## 1. 发明名称候选

可选方向：

- 一种面向六足机器人的跨异构 ROS2 运行环境预测控制集成方法
- 一种兼容现有导航链路的六足机器人预测控制桥接系统
- 一种六足机器人控制观测时间归一化与步态调度联合处理方法

## 2. 技术领域

本发明涉及机器人控制与机器人软件系统集成领域，特别涉及一种面向六足机器人的预测控制集成方法、桥接系统及其运行机制。

## 3. 背景技术

### 现有工程背景

- 现有机器人运行环境为 `ROS 2 + Isaac Sim + RTAB-Map + Nav2`
- 历史已验收控制链为 `hexbot_locomotion_v2` tripod 路线
- 上层导航接口已经围绕 `/cmd_vel` 和 `/joint_command` 固化

### 现有技术不足

- 直接引入 OCS2 时，上游示例面向四足，不适配六足 18 关节结构
- 宿主机与容器分属不同 ROS 2 发行版，运行环境异构
- 若推翻现有 `/cmd_vel`、`/joint_command`、Nav2、RTAB-Map、Isaac 集成边界，会导致系统迁移成本高
- 运行时 observation 使用绝对 Unix 时间戳会破坏 OCS2 gait 调度稳定性

## 4. 要解决的技术问题

建议固定为以下问题组合：

- 如何在不破坏现有导航和仿真链路的前提下，将预测控制主线集成到六足机器人系统中
- 如何在宿主机 Humble 与容器 Jazzy 的异构 ROS2 环境之间稳定传递控制观测与关节命令
- 如何使六足机器人的 OCS2 runtime observation 在长时运行中保持时间尺度稳定

## 5. 技术方案

### 5.1 系统总体结构

- 宿主机运行 Isaac Sim、RTAB-Map、Nav2 及 bridge
- Jazzy 容器运行 OCS2 六足求解与 runtime MRT
- 宿主机与容器通过统一 ROS2 DDS 域通信

### 5.2 主数据链

- 上层导航输出 `/cmd_vel`
- bridge 将宿主机运行时话题转发到 `/hexbot_mpc/input/*`
- `hexbot_cmd_vel_target` 将速度命令转为 OCS2 目标
- `hexbot_runtime_mrt` 结合 `/joint_states + /odom` 形成 runtime observation
- OCS2 输出 `/hexbot_mpc/output/joint_command`
- bridge 将后端命令回写到 `/joint_command`

### 5.3 六足适配特征

- 六足 18 关节状态映射
- 六足接触 / gait mode schedule 适配
- AnyMal 四足示例到 Hexbot 六足 URDF / joints / topics 的替换

### 5.4 跨环境集成特征

- 宿主机 Humble 与容器 Jazzy 的分层运行
- `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 的跨环境稳定通信约束
- 容器构建链自动补齐上游 OCS2 可见性

### 5.5 稳定运行特征

- `hexbot_runtime_mrt` 的观测时间归一化
- 不直接向 OCS2 传递 Unix 绝对时间
- 保持现有 `/cmd_vel` 与 `/joint_command` 边界不变

## 6. 有益效果

建议从系统效果来写，而不是只写“更快”：

- 不改上层导航接口即可接入预测控制主线
- 在跨 ROS2 发行版环境下形成稳定的控制闭环
- 六足机器人可在 Isaac 场景中获得连续关节命令输出与稳定动作响应
- 现有 tripod 路线可作为旧方案保留，便于 A/B 对比与回退

## 7. 附图建议

- 系统总体架构图
- 宿主机与 Jazzy 容器分层图
- `/cmd_vel -> OCS2 -> /joint_command` 闭环图
- bridge 输入输出 topic 对照图
- 六足 18 关节映射与 mode schedule 示意图

## 8. 具体实施方式

### 实施例 1：历史 baseline

- `hexbot_locomotion_v2`
- 作为现有技术或对比实施例

### 实施例 2：本发明主方案

- `hexbot_locomotion_mpc`
- `ocs2_hexbot_legged_robot`
- `ocs2_hexbot_legged_robot_ros`
- `hexbot_runtime_mrt`
- `hexbot_cmd_vel_target`

### 实施例 3：闭环验收方法

- Isaac 在线
- 宿主机运行态话题在线
- bridge 在线
- OCS2 在线
- `/hexbot_mpc/output/joint_command` 连续输出
- `/joint_command` 驱动机器人产生实际位姿变化

## 9. 对比实验建议

至少保留三组：

- 同一 `cmd_vel` 下旧方案 vs 新方案的跟踪结果
- 扰动 / 重启后的恢复时间
- 长时运行下的停顿与稳定性

## 10. 当前仓库中的对应证据

- 闭环验收文档：`hexbot_navigation_runtime_baseline.md`
- 闭环探针：`hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py`
- 宿主机运行态探针：`hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py`
- 旧方案 walking 基线：`hexbot_locomotion_v2/scripts/launch_basic_walk_loaded_tuned.sh`
- OCS2 ROS 主入口：`ocs2_hexbot_legged_robot_ros/launch/hexbot_sqp.launch.py`
