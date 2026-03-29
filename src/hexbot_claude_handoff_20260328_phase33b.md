# Hexbot Claude Handoff 2026-03-28

## 1. 项目核心背景与当前精准态势

### 1.1 核心目标

目标仍然是让 Hexbot 六足机器人在 Isaac Sim 中基于 OCS2 MPC 实现可验收的闭环稳定行走。

当前验收标准：

- `delta_forward_m > 0.05`，统计窗口 `8 s`
- `max_still_sec < 1.0`
- `trunk_sink_detected = false`
- `weird_pose_detected = false`
- `21/42` 三足 gait 可持续循环

### 1.2 历史稳定基线

历史稳定基线仍以 `Phase33b` 记账：

| 指标 | 数值 |
| --- | ---: |
| `delta_forward_m` | `0.004815` |
| `max_still_sec` | `3.73` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| gait | `21/42` |

它不是 walking 成功样本，但它定义了“任何新逻辑不能把站立稳定性搞坏”。

### 1.3 当前保留主线的真实状态（2026-03-29 更新）

当前代码主线在 Phase35 基础上保留了以下修复：

1. `HexbotCmdVelTargetNode.cpp`
   - `cmd_vel = 0` 时锁存 live observation 站立关节态，不再追静态 `reference.info`
   - moving target 使用 projected mode schedule
   - 最近一次非零 `cmd_vel` 持有 `3.0 s`
2. `HexbotRuntimeMrtNode.cpp`
   - observation normalization 使用 Pinocchio/URDF 宽限位
   - backend command safety 继续使用 actuator guarded limits
   - joint branch-consistency guard 已启用
   - **[2026-03-29 新增]** `updateTouchdownAnchors` 使用 `currentFootPositions`（实测值）
3. `GaitReceiver.cpp/.h`
   - identical gait template dedupe 已启用
4. **[2026-03-29 新增]** `LeggedRobotPreComputation.cpp`
   - `eeZeroVelConConfig` 增加 XY 平面锚定约束（stance foot pinned in world XY）
5. **[2026-03-29 新增]** `hexbot_sqp.launch.py`
   - 新增运行时阈值参数：`commandTrackingResetThreshold=2.0`, `equivalentAngleWrapResetThreshold=4.5`
   - 新增 guard 参数：`enableJointBranchConsistencyGuard=true`, `jointBranchGuardCommandDeviationThreshold=1.5`
6. **[2026-03-29 新增]** `task.info`
   - R(18-35) = 500.0（joint velocities，从 5000 降低）
   - 其他 Q/R 保持原始基线

当前保留主线的最佳 moving probe（2026-03-29 更新）：

- 最佳安全配置：`codex_r500thresh_fwd_20260329_042000`

| 指标 | 数值 |
| --- | ---: |
| `initial_policy_received` | `1` |
| `reset_request_sent` | `1` |
| `reject_backend_joint_command` | `0` |
| `delta_forward_m` | `+0.0067` |
| `max_still_sec` | `5.27` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| `mode_sequence` | `[21, 42]` |

- Guard-off 概念验证：`codex_r500noguard_fwd_20260329_044000`

| 指标 | 数值 |
| --- | ---: |
| `delta_forward_m` | `+0.034` |
| `reject_backend_joint_command` | `81` |
| `reset_request_sent` | `82` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |

当前保留主线的最佳 idle probe：

- `codex_r500thresh_idle_20260329_041500`
- `ok = true`, `trunk_sink_detected = false`, `weird_pose_detected = false`, `mode_sequence = [63]`

### 1.4 当前卡点已精确收敛（2026-03-29 最终判定）

已排除的非卡点：

- `cmd_vel` 没进 OCS2
- gait 没切 moving
- observation 被全链路漂白
- gait template 重复注入
- R/Q/threshold 参数调优不够精确（**已穷举 10+ 组实验**）
- 2π 角度归一化（已测试 `branchNormalizeJointAngles`，无效）
- 推力系数不足（4x 推力使情况更差）

**最终根因判定：SQP 求解器 tarsus 关节 π-branch 局部极小值**

- 求解器在 tarsus 关节上收敛到错误的 π 偏移局部最优
- 左 tarsus 应为负值，求解器输出正值；右 tarsus 反之
- branch guard 每 8s 触发 ~470 次将错误命令替换为 target ≈ measured = "不动"
- 关闭 guard 后前进 +0.034m（5 倍提升），**证明 guard 是直接运动阻断器**
- 但 guard 存在是正确的——它阻止了危险命令

一句话：

`SQP 单次迭代线性化无法逃离 tarsus 错支路局部极小值；参数调优已穷尽无效；必须从约束层结构性封死错误 kinematic branch。`

## 2. 已有工具与环境基建

### 2.1 工作区与容器

- 工作区根目录：
  `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws`
- 核心容器：
  `ocs2_hexbot_jazzy`
- 宿主负责：
  Isaac Sim、runtime 清场、probe 启停、日志留档
- Jazzy 容器负责：
  OCS2 Hexbot 核心编译

### 2.2 核心路径

OCS2 核心包：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot`

ROS 接口与 runtime：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros`

重点源码：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros/src/HexbotRuntimeMrtNode.cpp`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros/src/HexbotCmdVelTargetNode.cpp`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros/src/gait/GaitReceiver.cpp`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros/include/ocs2_hexbot_legged_robot_ros/gait/GaitReceiver.h`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/include/ocs2_hexbot_legged_robot/common/HexbotActuatorJointLimits.h`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/reference_manager/SwitchedModelReferenceManager.cpp`

Probe 与分析脚本：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/scripts/run_idle_reference_safety_probe.sh`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/probe_hexbot_posture.py`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/summarize_hexbot_solver_trace.py`

配置与参考：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/config/mpc/task.info`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/config/command/reference.info`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/config/command/reference_loaded_equilibrium.info`

Isaac / URDF：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/cleanup_hexbot_runtime.sh`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf`

本轮核心审计文档：

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/glimmering-tumbling-prism.md`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_forward_progress_root_cause_20260328.md`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_algorithm_bottom_up_audit_20260328.md`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_contact_execution_audit_20260328.md`
- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_claude_handoff_20260328_phase33b.md`

### 2.3 修改红线

允许修改：

- `ocs2_hexbot_legged_robot`
- `ocs2_hexbot_legged_robot_ros`
- 配置文件
- probe / 分析脚本
- 审计文档

禁止修改：

- 外部 OCS2 上游库
- Pinocchio
- CppAD
- Docker 基础镜像

当前最该优先修改的是接口层、target 生成层、reference/anchor 链，而不是再去改 OCS2 上游核心。

## 3. IsaacSim 仿真对接

### 3.1 控制脚本

Isaac 控制脚本：

- `python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py status`
- `python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py stop`
- `python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py play`

### 3.2 每轮 probe 前的清场规则

每次 probe 前都必须先清场再停仿真：

```bash
HEXBOT_SUDO_PASSWORD=root bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/cleanup_hexbot_runtime.sh
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/isaac/isaac_runtime_control.py stop
```

不要在前一轮进程未完全退出时直接复跑 probe。

### 3.3 状态校验

最基本检查：

- `isaac_runtime_control.py status` 看 `playing`
- 运行后检查 `/joint_states`、`/odom`、`/hexbot_mpc_*`

### 3.4 必须用 Isaac 真位移排除 Odom 假象

不要只看 `summary.probe.motion.delta_forward_m`。

必须同时检查：

- `*_posture.json -> isaac.start.articulation.world_position`
- `*_posture.json -> isaac.end.articulation.world_position`

真实物理位移：

- `delta_x = end[0] - start[0]`

单行范例：

```bash
python3 - <<'PY'
import json
p='/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe/<label>_posture.json'
d=json.load(open(p))
s=d['isaac']['start']['articulation']['world_position'][0]
e=d['isaac']['end']['articulation']['world_position'][0]
print({'isaac_delta_x_m': e - s, 'start_x': s, 'end_x': e})
PY
```

## 4. 调试管道与数据验证

### 4.1 最常用 probe 命令

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/scripts/run_idle_reference_safety_probe.sh \
  --label <label> \
  --duration 12 \
  --probe-duration 8 \
  --linear-x 0.03 \
  --angular-z 0.0
```

idle 回归时把 `--linear-x 0.03` 改为 `0.0`。

### 4.2 编译命令

代码改动后必须进 Jazzy 容器编译：

```bash
printf 'root\n' | sudo -S -k docker exec -i ocs2_hexbot_jazzy bash -lc \
  "cd /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws && bash container_build_hexbot_stack.sh"
```

### 4.3 日志目录

- `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe`

### 4.4 四类日志的排查逻辑

1. `*_summary.json`
   - 首看总结果
   - 重点字段：
     `counts.*`、`probe.motion.*`、`posture.trunk_sink_detected`、`posture.weird_pose_detected`
2. `*_posture.json`
   - 看 Isaac 真位移和姿态异常
3. `*_solver_trace.json`
   - 看 moving phase 的 joint gap、foot gap、touchdown-anchor gap
4. `*_container.log`
   - 看 branch guard、reject、reset、CppAD/solver 报警

### 4.5 常用 Python 单行提取

快速看 summary 关键字段：

```bash
python3 - <<'PY'
import json
p='/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe/<label>_summary.json'
d=json.load(open(p))
print({
  'counts': d['counts'],
  'motion': d['probe']['motion'],
  'posture_ok': {
    'trunk_sink_detected': d['posture']['trunk_sink_detected'],
    'weird_pose_detected': d['posture']['weird_pose_detected'],
  },
  'mode_sequence': d['ocs2_topics']['mode_schedule_snapshot']['mode_sequence'],
  'moving_cycle_detected': d['ocs2_topics']['mode_schedule_snapshot'].get('moving_cycle_detected'),
})
PY
```

快速看 solver trace 核心指标：

```bash
python3 - <<'PY'
import json
p='/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe/<label>_summary.json'
m=json.load(open(p))['solver_trace']['metrics']
print({
  'mode_counts': m['mode_counts'],
  'optimized_vs_target_joint_max_abs_delta_mean_rad': m['optimized_vs_target_joint_max_abs_delta_mean_rad'],
  'optimized_vs_current_foot_max_position_error_mean_m': m['optimized_vs_current_foot_max_position_error_mean_m'],
  'current_vs_touchdown_anchor_stance_max_position_error_mean_m': m['current_vs_touchdown_anchor_stance_max_position_error_mean_m'],
  'optimized_vs_touchdown_anchor_stance_max_position_error_mean_m': m['optimized_vs_touchdown_anchor_stance_max_position_error_mean_m'],
})
PY
```

快速统计 container log 里的关键报警：

```bash
python3 - <<'PY'
from pathlib import Path
p=Path('/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe/<label>_container.log')
t=p.read_text(errors='ignore')
for key in [
  'Applied joint branch-consistency guard',
  'Rejecting backend joint command',
  'MPC is reset',
  'Setting new gait after time',
]:
  print(key, t.count(key))
PY
```

## 5. 权限与密码

- 当前宿主用户：`u-zhuang`
- `sudo` 密码：`root`
- Docker/容器操作统一用无交互传参：

```bash
printf 'root\n' | sudo -S -k docker exec -i ocs2_hexbot_jazzy bash -lc "<cmd>"
```

- cleanup 脚本常用：

```bash
HEXBOT_SUDO_PASSWORD=root bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/cleanup_hexbot_runtime.sh
```

## 6. 最新核心代码节点状态（2026-03-29 更新）

### 6.1 `HexbotRuntimeMrtNode.cpp`

当前保留逻辑：

- 观测归一化使用 Pinocchio model limits
- backend command safety 保持 actuator guarded limits
- branch-consistency guard 默认开启（launch 可配）
- **[新增]** `updateTouchdownAnchors` 使用 `currentFootPositions`（实测值，非 target）
- **[新增]** launch 参数化：`commandTrackingResetThreshold=2.0`, `equivalentAngleWrapResetThreshold=4.5`, `jointBranchGuardCommandDeviationThreshold=1.5`

关键现象：

- branch guard 每 8s 仍触发 ~470 次，主要在 `tarsus_LM` (225) 和 `tarsus_LF` (217)
- guard 将错误命令替换为 target ≈ measured，等效于”tarsus 不动”
- 关闭 guard 后前进 +0.034m（5 倍），证明 guard 是运动阻断的直接原因

代码默认值 vs launch 覆盖值：

| 参数 | 代码默认 | Launch 默认 |
| --- | --- | --- |
| `commandTrackingResetThreshold` | 1.0 | **2.0** |
| `equivalentAngleWrapResetThreshold` | π (3.14) | **4.5** |
| `jointBranchGuardCommandDeviationThreshold` | 0.60 | **1.5** |
| `enableJointBranchConsistencyGuard` | true | **true** |

### 6.2 `HexbotCmdVelTargetNode.cpp`

当前保留逻辑：

- `modeScheduleTopic` 已订阅
- `movingCmdHoldTime` 默认 `3.0`
- stationary target 锁存 live observation joints
- moving target 通过 `projectModeAtTime(...)` 预测 knot mode
- whole-body IK 仍使用 guarded joint limits
- `translationLookaheadTime` launch 默认 `0.35` (代码默认 1.20)

关键现象：

- moving target 确实能持续请求前进
- 0.03 m/s × 0.35s = 10.5mm target displacement（已验证正确）
- 前进目标编码无误，问题在求解器执行端

### 6.3 `GaitReceiver.cpp/.h`

当前保留逻辑：

- identical gait template dedupe 已启用
- `Setting new gait after time` 只出现 2 次（正常）

### 6.4 `LeggedRobotPreComputation.cpp`（新增修改）

- `eeZeroVelConConfig` lambda 增加了 XY 平面锚定约束
- 使用 `swingTrajectoryPlannerPtr_->getPlanarPositionConstraint(footIndex, t)` 获取锚点
- 增益 = `positionErrorGain` (4.0)
- 效果：stance foot drift 从 0.054m 降至 0.023m

### 6.5 `LeggedRobotInitializer.cpp`

- `branchNormalizeJointAngles` 已测试并**回退**（使情况更糟，+30 resets）
- 当前状态：`nextState = state`，无额外处理

### 6.6 `task.info` 当前配置

- R(0-17) = 1.0（contact forces）
- R(18-35) = **500.0**（joint velocities，从 5000 降低）
- Q(6-8) = 500.0（base position），Q(9) = 100.0（yaw），Q(10-11) = 200.0
- Q(12-29) = 20.0（all joints）
- `sqpIteration = 1`, `propulsionForceScale = 2.0`

相关 grep 线索：

- `sameModeSequenceTemplate(...)` 约在 `39`
- dedupe 逻辑约在 `88-93`
- `hasReceivedGaitMessage_` 在头文件约 `70`

### 6.4 当前明确不要重复踩的坑

已经证明失败并回退：

1. `Tarsus/Femur` 支路惩罚
   - 样本：`codex_swingbranch_forward_20260328_200322`
   - 结果：branch guard 次数 `445 -> 450`，前进更差
2. 全局放宽 femur 软件限位
   - 样本：`codex_femurlimit_idle_20260328_201456`
   - 样本：`codex_femurlimit_forward_20260328_201549`
   - 结果：forward 变成 `19/19/18` 的 reset/reject 风暴

当前代码已经回退到这些试验之前的主线，并重新编译完成。

## 7. 接手后的第一刀（Next Action）

不要再从全局 MPC 权重开始。

Claude 接手后的第一刀应该直接做这件事：

1. 以 `codex_gaitdedupe_forward_20260328_194801` 为主样本，逐项追 moving phase 的支撑足执行误差。
2. 重点看 `21/42` 相位里：
   - `solver_trace.metrics.current_vs_touchdown_anchor_stance_max_position_error_mean_m`
   - `solver_trace.metrics.optimized_vs_touchdown_anchor_stance_max_position_error_mean_m`
   - `solver_trace.metrics.optimized_vs_current_foot_max_position_error_mean_m`
   - `solver_trace.metrics.optimized_vs_target_joint_max_abs_delta_mean_rad`
3. 结合 `container.log` 中
   - `Applied joint branch-consistency guard`
   去定位到底是哪条腿、哪次相位切换在翻错支路。
4. 源码优先看：
   - `HexbotRuntimeMrtNode.cpp`
   - `HexbotCmdVelTargetNode.cpp`
   - `SwitchedModelReferenceManager.cpp`

真正该回答的问题是：

- touchdown-anchor 是不是在 `63 -> 21 -> 42` 切换时更新得不一致
- 支撑足世界锚定是不是在 moving phase 里漂了
- solver 输出是不是仍在围绕错误 `tarsus/femur` 关节支路构解

在没有把这三件事查清之前，不要再回去做：

- 全局 `Q/R` 调权
- 再加一轮 joint tracking cost
- 再放宽一轮全局 femur 限位
