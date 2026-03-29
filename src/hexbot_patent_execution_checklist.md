# Hexbot 专利成品推进清单

## 当前主线目标

当前第一优先级不再是“把 tripod / OCS2 调得更快”，而是把 Hexbot 的跨环境预测控制集成方案做成：

- 可重复启动
- 可闭环验收
- 可与历史 baseline 做 A/B 对比
- 可直接沉淀为技术交底书与发明点说明

当前拟主张的成品对象不是单一 gait，而是这套系统方案：

- 宿主机 `ROS 2 Humble`
- 容器 `ROS 2 Jazzy`
- `hexbot_locomotion_mpc` 作为桥接层
- `ocs2_hexbot_legged_robot` / `ocs2_hexbot_legged_robot_ros` 作为 OCS2 六足主体
- 不推翻现有 `Isaac Sim + RTAB-Map + Nav2` 上层链路
- 主闭环为  
  `/cmd_vel -> /hexbot_mpc/input/cmd_vel -> OCS2/MPC -> /hexbot_mpc/output/joint_command -> /joint_command`

## 第一阶段：闭环成品化

### 1. 统一启动面

- 宿主机运行侧构建固定为：  
  `bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/host_build_runtime_stack.sh`
- Jazzy 容器入口固定为：
  `bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_run_hexbot_jazzy.sh`
- 容器进入方式固定为：
  `bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_exec_hexbot_jazzy.sh`
- 容器内依赖安装与构建固定为：
  `bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_setup_env.sh`
  `bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_build_hexbot_stack.sh`

### 2. 统一验收口径

- Isaac 在线性：`/clock + /joint_states + /odom`
- 宿主机 bridge 在线性：`/hexbot_mpc/input/*`
- OCS2 后端在线性：`/hexbot_mpc/output/joint_command`
- 宿主机最终控制在线性：`/joint_command`
- 机器人响应在线性：`/odom` 对 `cmd_vel` 有实际位姿变化

### 3. 固定验收工具

- 宿主机话题探针：  
  `src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py`
- 闭环探针：  
  `src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py`
- 一键闭环验收入口：  
  `src/hexbot_locomotion_mpc/scripts/run_patent_closed_loop_case.sh`

## 第二阶段：A/B 证据采集

### A/B 对比对象

- 旧方案：`hexbot_locomotion_v2`
- 新方案：`hexbot_locomotion_mpc + ocs2_hexbot_legged_robot_ros`

### 最少保留三类证据

1. 同一 `cmd_vel` 下的跟踪结果对比

- `delta_forward_m`
- `delta_lateral_m`
- `delta_yaw_rad`
- `max_still_sec`

2. 扰动 / 停机 / 重启恢复结果

- 关停后是否能干净释放 `/joint_command`
- 重启后站姿恢复是否稳定
- 重新进入 gait 前的准备时间

3. 长时运行稳定性

- 10 分钟以上持续控制是否出现停顿
- 是否出现重复 OOM / reset / message buffer warning
- `/joint_command` 是否保持单发布者

### 当前建议的最小 A/B 数据集

- `5s @ linear.x=0.03`
- `5s @ angular.z=0.12`
- `10s @ linear.x=0.03, angular.z=0.12`

所有 case 都保留：

- 原始 JSON 输出
- 对应启动命令
- 运行日期
- 当前使用的配置文件

## 第三阶段：发明点固化

### 当前更适合主张的发明方向

- 一种面向六足机器人的跨异构 ROS2 运行环境预测控制集成方法
- 一种兼容既有导航链路的六足机器人预测控制桥接系统
- 一种六足机器人控制观测时间归一化与运行时桥接联合处理方法

### 当前不建议作为唯一主张点的方向

- “走得更快”
- “某个 tripod 参数更优”
- “纯粹 gait 调参”

这些更适合作为实施例中的性能效果，而不是主发明点本身。

## 第四阶段：第二优先级优化

以下内容继续做，但都排在闭环成品和证据沉淀之后：

- 纯转向精度优化
- mixed endurance
- 更高速度边界
- running / jogging 模式

## 本周建议产出

1. 固定一条 OCS2 主线闭环 bring-up
2. 补齐旧方案 vs 新方案的统一 probe 输出
3. 形成一版技术交底书骨架
4. 形成一版“技术问题 - 技术手段 - 技术效果”中文说明
