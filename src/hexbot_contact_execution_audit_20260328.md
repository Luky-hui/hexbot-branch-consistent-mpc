# Hexbot Contact Execution Audit 2026-03-28

## 1. 目的

这份文档冻结当前保留主线在 contact execution 层的最新证据。

截至 2026-03-28，当前主线已经不再主要卡在：

- `cmd_vel` 没送到 OCS2
- gait 没切 moving
- observation 被 runtime 全链路漂白
- identical gait template 反复注入

当前主线已经能：

- 保持 idle 稳定站立
- 在 forward probe 中进入真实 `21/42` moving cycle
- 把 reset/reject 压到 `1/0`

但它仍然几乎不前进，因此这份审计只聚焦：

- moving phase 下的支撑足执行误差
- touchdown-anchor 一致性
- swing 腿 `tarsus/femur` 错支路与真实执行之间的关系

## 2. 当前保留主线样本

当前最重要的 retained forward sample：

- `codex_gaitdedupe_forward_20260328_194801`

关键 summary 指标：

| 指标 | 数值 |
| --- | ---: |
| `initial_policy_received` | `1` |
| `reset_request_sent` | `1` |
| `reject_backend_joint_command` | `0` |
| `runtime_observations_stale` | `0` |
| `delta_forward_m` | `-0.0009468660` |
| `max_still_sec` | `6.666667` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| `mode_sequence` | `[21, 42]` |
| `moving_cycle_detected` | `true` |

对应 solver trace 指标：

| 指标 | 数值 |
| --- | ---: |
| `mode_counts` | `{'21': 6, '42': 6, '63': 6}` |
| `optimized_vs_target_joint_max_abs_delta_mean_rad` | `0.7146` |
| `optimized_vs_current_foot_max_position_error_mean_m` | `0.02936` |
| `current_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.05072` |
| `optimized_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.03259` |
| `optimized_vs_target_foot_max_position_error_mean_m` | `0.03099` |
| `optimized_vs_target_foot_max_abs_z_delta_mean_m` | `0.03000` |

对应 container log 计数：

- `Applied joint branch-consistency guard = 445`
- `Rejecting backend joint command = 0`
- `Setting new gait after time = 2`

## 3. 当前最可信的 contact execution 结论

### 3.1 gait 已经活着，但接触执行还没对上

`mode_sequence = [21, 42]` 且 `moving_cycle_detected = true`，
说明 moving gait 这次不是假象。

但：

- `delta_forward_m` 仍接近 `0`
- `max_still_sec` 仍高达 `6.67 s`

这说明问题已经不是“没有 moving mode”，而是：

- moving mode 的 contact execution 仍然没有转化成净推进

### 3.2 支撑足与 touchdown-anchor 之间仍存在明显偏差

当前 retained sample 里：

- `current_vs_touchdown_anchor_stance_max_position_error_mean_m = 0.05072`
- `optimized_vs_touchdown_anchor_stance_max_position_error_mean_m = 0.03259`

这代表：

- solver 名义上比当前实际状态更接近 touchdown anchor
- 但真实执行到支撑足世界锚定这一层，仍有约 `5 cm` 级误差

这已经足以解释为什么 gait 在“看起来循环”时仍然不产生有效 body push-off。

### 3.3 支撑足执行 gap 与错误支路问题是同时存在的

当前 retained sample 里：

- `optimized_vs_current_foot_max_position_error_mean_m = 0.02936`
- `optimized_vs_target_joint_max_abs_delta_mean_rad = 0.7146`

同时 container log 还在大量触发：

- `Applied joint branch-consistency guard`

这说明：

1. 足端执行 gap 没消失
2. swing-side `tarsus/femur` 错支路也没消失
3. runtime guard 只是避免把最坏解直接发出去，并没有让 contact execution 真正变得健康

## 4. 已验证失败并回退的 contact-execution 相关分支

### 4.1 Tarsus/Femur 支路惩罚试验失败

试验：

- `codex_swingbranch_forward_20260328_200322`

目标：

- 通过在 quadratic tracking cost 里对 swing 腿 `tarsus/femur` 追加 continuity 惩罚，压低错误支路翻转

结果：

| 指标 | 数值 |
| --- | ---: |
| `delta_forward_m` | `-0.0025227602` |
| `max_still_sec` | `8.033334` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| `Applied joint branch-consistency guard` | `450` |

对比 retained mainline：

- branch guard 次数几乎没变：`445 -> 450`
- gait reinsertion 次数没变：`2 -> 2`
- 真实前进更差

结论：

- 这类局部 tracking penalty 不是有效的 contact-execution 修复
- 该分支已完整回退

### 4.2 全局 Femur 软件限位放宽失败

试验：

- `codex_femurlimit_idle_20260328_201456`
- `codex_femurlimit_forward_20260328_201549`

结果：

Forward 样本直接进入：

- `initial_policy_received = 19`
- `reset_request_sent = 19`
- `reject_backend_joint_command = 18`

同时 `command_vs_actual_joint_state` 已出现：

- `femur_joint_RF`: command `-1.23`, state `-1.4508`
- `femur_joint_RM`: command `-1.23`, state `-1.4356`
- `femur_joint_LF`: command `1.23`, state `1.4067`

结论：

- 虽然当前 femur guard 是软件自定义窗口，不是 URDF 真极限
- 但不能把它全局粗暴放宽
- 这个方向会把 contact execution 直接推回 reset/reject 风暴
- 该分支已完整回退

## 5. 当前 contact execution 的最小闭环解释

到 2026-03-28 为止，最可信的解释已经收敛为：

1. target 层已经能给出 moving intent。
2. gait 层已经能保留 `21/42` moving cycle。
3. runtime 也不再被 observation whitening 和 gait spam 主导。
4. 真正卡住的是 moving phase 的接触执行一致性：
   - stance 足对 touchdown-anchor 的跟随不够好
   - swing 腿 `tarsus/femur` 仍会翻错支路
   - solver 名义足端位置与真实执行足端位置之间仍存在 `3 cm` 量级 gap

因此当前 Hexbot 不再是“没有走路策略”，而是：

`有 moving 策略，但 moving 接触执行还没有闭环到能产生净推进。`

## 6. 2026-03-29 更新：参数穷举已完成，确认必须结构性修复

### 6.1 已保留的代码修复

1. **XY 平面锚定约束** (`LeggedRobotPreComputation.cpp` 117-141 行)：
   - `eeZeroVelConConfig` 增加了 `getPlanarPositionConstraint()` 锚定
   - 将 stance foot drift 从 0.054m 降低到 0.023m

2. **实测触地锚点** (`HexbotRuntimeMrtNode.cpp` 1247 行)：
   - `updateTouchdownAnchors` 使用 `currentFootPositions`（实测值），不再用 target

3. **Launch 安全阈值参数化** (`hexbot_sqp.launch.py`)：
   - `commandTrackingResetThreshold=2.0`, `equivalentAngleWrapResetThreshold=4.5`
   - `jointBranchGuardCommandDeviationThreshold=1.5`, `enableJointBranchConsistencyGuard=true`

### 6.2 参数穷举总结（10+ 组对照实验全部失败）

| 配置 | delta_forward_m | rejects | 结论 |
| --- | ---: | ---: | --- |
| R=500, guard=on, 阈值放宽 | **+0.0067** | 0 | 最佳安全配置，仍仅 13% 目标 |
| R=500, guard=off | **+0.034** | 81 | 证明 guard 是 motion blocker |
| R=200 | +0.004 | 11 | 更多自由度 = 更多 resets |
| SQP=3 | +0.0006 | 3 | 更慢求解，同样错支路 |
| Q(tarsus)=100 | -0.007 | 2 | 过度约束 |
| 推力 4x | -0.002 | 4 | 不稳定 |
| 2π 初始化归一 | -0.002 | 30 | 问题是 π 偏移，不是 2π |

### 6.3 根因精确化：tarsus π-branch 局部极小值

求解器反复收敛到 tarsus 关节的 **π 偏移局部最优**：

```
实例: tarsus_joint_LM — optimized=+0.05, target=-0.77, measured=-0.81
  左侧 tarsus 应为负值，求解器输出正值（差约 0.82 rad ≈ π/4）
```

- branch guard 每 8s 触发 ~470 次，主要在 tarsus_LM (225) 和 tarsus_LF (217)
- guard 将错误命令替换为 target ≈ measured，等效于"不动"
- 关闭 guard 后前进距离提升 5 倍，**证明 guard 是运动阻断的直接原因**
- 但 guard 存在是正确的——它阻止了危险的错支路命令

### 6.4 结论

**参数调优方向已穷尽**。R/Q 矩阵、安全阈值、SQP 迭代次数、推力系数的任何组合都不能解决这个问题。

下一步必须从 **OCS2 底层约束层** 结构性修复：

1. **硬性状态不等式约束**：左 tarsus ≤ 0，右 tarsus ≥ 0，使错支路在数学上不可行
2. **零空间姿态惩罚**：对 tarsus 符号偏离施加定向惩罚
3. **Warm-start 投影**：求解器输出后将 tarsus 投影到正确符号再作为下一轮初始值

需要审计的文件：
- `src/ocs2_hexbot_legged_robot/src/LeggedRobotInterface.cpp` — 约束注册
- OCS2 上游 `StateConstraint` / `StateInputConstraint` 接口
- `src/ocs2_hexbot_legged_robot/src/constraint/` — 现有约束实现

这是截至 2026-03-29 最准确的 contact execution 审计结论。
