# Hexbot 2026-03-28 调试冻结纪要（glimmering-tumbling-prism）

## 1. 文档定位

这份文档冻结 2026-03-28 当前工作区的真实状态。

它不再只记录 `Phase35` 本身，而是把 `Phase35` 之后已经保留到主线的修复、已经证明失败并完整回退的试验、以及当前剩余主阻塞一起固化下来，供后续模型直接接力。

## 2. 当前保留主线

当前代码主线已经不再是原始 `Phase35`，而是“在不破坏稳定站立的前提下，连续修掉三条失配链”后的版本。

### 2.1 当前保留的有效修改

1. `HexbotCmdVelTargetNode.cpp`
   - `cmd_vel = 0` 时，不再回拉静态 `reference.info` 关节角。
   - 改为锁存 live observation 的站立平衡关节态，避免控制器持续追一个 Isaac 并不存在的“幻觉平衡位姿”。
   - moving target 生成时，按最新 mode schedule template 预测 knot mode。
   - 最近一次非零 `cmd_vel` 会保持 `3.0 s`，避免桥接短暂掉线把 moving phase 误打回静止逻辑。
2. `HexbotRuntimeMrtNode.cpp`
   - 观测归一化与命令安全钳制已拆分。
   - observation normalization 现在使用 Pinocchio/URDF 宽限位，不再把真实 `-1.2 rad` 观测直接漂白到 `-0.95 rad`。
   - backend command safety 仍保留 actuator guarded limits。
   - joint branch-consistency guard 保留，专门拦截优化器在 `femur/tarsus` 上跳到错误关节支路。
3. `GaitReceiver.cpp/.h`
   - 已加入 identical gait template dedupe。
   - 相同 mode sequence template 不再反复触发 `gaitUpdated_`，避免 gait spam 反复重插入。

### 2.2 当前保留主线的最佳前进样本

保留主线的最佳 moving probe 是：

- `codex_gaitdedupe_forward_20260328_194801`

对应摘要：

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
| target `delta_body.forward_m` | `0.0035` |

关键 solver trace 指标：

| 指标 | 数值 |
| --- | ---: |
| `mode_counts` | `{'21': 6, '42': 6, '63': 6}` |
| `optimized_vs_target_joint_max_abs_delta_mean_rad` | `0.7146` |
| `optimized_vs_current_foot_max_position_error_mean_m` | `0.02936` |
| `current_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.05072` |
| `optimized_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.03259` |

结论：

- 站立稳定性保住了。
- gait 已经真实进入 `21/42` moving cycle。
- command starvation、gait republish spam、观测漂白已经不是当前主阻塞。
- 但机器人仍然没有产生可验收的物理前进。

### 2.3 当前保留主线的最佳站立样本

- `codex_gaitdedupe_idle_20260328_194904`

对应结论：

- `initial_policy_received = 1`
- `reset_request_sent = 1`
- `reject_backend_joint_command = 0`
- `runtime_observations_stale = 0`
- `trunk_sink_detected = false`
- `weird_pose_detected = false`
- `mode_sequence = [63]`

这说明当前主线仍满足“`linear_x = 0` 下不劣化站立基线”的硬约束。

## 3. 2026-03-28 已经真正修掉的链路

### 3.1 错误静态平衡位姿回拉

早先 runtime 在静止阶段会持续把系统拉回静态 `reference.info`。
这会强迫 MPC 追一个与 Isaac 加载稳态不一致的参考姿态。

当前已修复为：

- stationary target 直接锁存 live observation joints

效果：

- idle 样本恢复到 `1/0` 或 `1/1/0` 级别稳定计数
- 站立时不再出现明显的 reference-induced reset 风暴

### 3.2 观测漂白 / 伪误差链

此前 runtime 会把真实超过 actuator guarded window 的观测直接夹到软件安全边界，制造出人为的状态漂白。

当前已修复为：

- observation normalization 用 Pinocchio model limits
- backend command safety 才用 actuator guarded limits

效果：

- observation normalization 相关警告降到 `0`
- “测量态先被压扁，再被 MPC 当成误差纠正”的伪链路被切断

### 3.3 gait template 重复注入

此前 gait publisher 会重复发送相同 template，`GaitReceiver` 每次都当成新 gait 处理，导致 moving phase 易被扰动。

当前已修复为：

- identical template dedupe

效果：

- `codex_gaitdedupe_forward_20260328_194801_container.log` 中
  `Setting new gait after time` 只有 `2` 次
- moving cycle 能稳定保留到 probe 后段

## 4. 已验证失败并完整回退的试验

### 4.1 Tarsus/Femur 支路惩罚试验失败并回退

试验：

- `codex_swingbranch_forward_20260328_200322`

修改方向：

- 在 `LeggedRobotQuadraticTrackingCost` 里对 swing 腿 `femur/tarsus` branch continuity 追加惩罚

结果：

| 指标 | 数值 |
| --- | ---: |
| `delta_forward_m` | `-0.0025227602` |
| `max_still_sec` | `8.033334` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |

对比主线 `codex_gaitdedupe_forward_20260328_194801`：

- `Applied joint branch-consistency guard` 次数从 `445` 变成 `450`
- `Rejecting backend joint command` 仍为 `0`
- `Setting new gait after time` 仍为 `2`

结论：

- 这类局部 tracking 惩罚并没有实质减少 branch flip
- forward 样本反而更差
- 该试验已完整回退，不在当前代码中

### 4.2 全局放宽 Femur 软件限位失败并回退

试验：

- `codex_femurlimit_idle_20260328_201456`
- `codex_femurlimit_forward_20260328_201549`

修改方向：

- 全局放宽 `HexbotActuatorJointLimits` / runtime 的 femur 软件限位

Idle 结果：

| 指标 | 数值 |
| --- | ---: |
| `initial_policy_received` | `1` |
| `reset_request_sent` | `1` |
| `reject_backend_joint_command` | `0` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| `optimized_vs_target_joint_max_abs_delta_mean_rad` | `0.2974` |
| `current_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.09917` |
| `optimized_vs_touchdown_anchor_stance_max_position_error_mean_m` | `0.09419` |

Forward 结果：

| 指标 | 数值 |
| --- | ---: |
| `initial_policy_received` | `19` |
| `reset_request_sent` | `19` |
| `reject_backend_joint_command` | `18` |
| `delta_forward_m` | `-0.0015540209` |
| `max_still_sec` | `3.933334` |
| `trunk_sink_detected` | `false` |
| `weird_pose_detected` | `false` |
| `optimized_vs_target_joint_max_abs_delta_mean_rad` | `1.0153` |

并且 `command_vs_actual_joint_state` 已出现：

- command 侧 `femur_joint_RF/RM/RR` 被打到 `-1.23`
- actual state 侧对应腿跑到 `-1.37 ~ -1.45`

结论：

- `±0.95` 软件 guard 不是可以粗暴全局放开的按钮
- 全局放宽会直接引入 reset / reject 风暴
- 该试验已完整回退，不在当前代码中

## 5. 当前根因判定

### 5.1 已降级为背景根因的层

以下问题依然成立，但在当前保留主线上已经不再是最先要修的“活跃断点”：

1. `task.info` 中接触力成本过高，水平推进力代价远高于追踪收益。
2. `reference.info`、`reference_loaded_equilibrium.info` 与 Isaac loaded equilibrium 仍存在偏差。
3. `HexbotActuatorJointLimits` 的 femur guard 目前仍是软件级自定义窗口，而不是 URDF 真极限。

这些问题决定了系统为什么“天然不容易走起来”，但它们不是当前最直接的调试入口。

### 5.2 当前活跃主阻塞

当前最直接的 live blocker 已经收敛为：

1. 机器人在 `21/42` moving phase 中，stance 足与 touchdown-anchor 的一致性仍然不好。
2. solver 会在真实 moving phase 里把 swing 腿 `tarsus/femur` 推到错误关节支路，runtime branch guard 只能止损，不能把它变成有效推进。
3. 当前主线样本中，`current_vs_touchdown_anchor_stance_max_position_error_mean_m` 仍约 `5 cm`，说明支撑足世界锚定误差并未解决。
4. `optimized_vs_target_joint_max_abs_delta_mean_rad` 仍约 `0.71 rad`，说明 moving target 到 solver joint branch 之间仍有明显失配。

一句话：

- 已经不再是“站不稳”
- 已经不再是“没收到命令”
- 已经不再是“gait 一直被刷掉”
- 当前卡在“真正 moving phase 的 touchdown-anchor 漂移 + joint branch mismatch”

## 6. 下一步硬规则

### 6.1 不要再做的事

不要继续把时间花在：

- 全局 MPC 权重盲调
- 再做一轮 swing-joint tracking penalty
- 再做一轮全局 femur software limit widening
- 回头重复 `comHeight` / `positionErrorGain` 局部调参

这些路径 2026-03-28 已经明确验证过，收益很低，且容易再次破坏站立基线。

### 6.2 下一刀该看的地方

后续调试应直接对准：

1. `HexbotRuntimeMrtNode.cpp`
   - moving phase 下的支撑足执行误差、touchdown-anchor 相关日志、branch guard 触发位置
2. `HexbotCmdVelTargetNode.cpp`
   - mode projection 与 touchdown anchor 更新逻辑
3. `SwitchedModelReferenceManager.cpp`
   - gait 切换期间 foot target / touchdown anchor 的参考生成链
4. 最新日志
   - `*_summary.json`
   - `*_solver_trace.json`
   - `*_container.log`

优先检查字段：

- `solver_trace.metrics.current_vs_touchdown_anchor_stance_max_position_error_mean_m`
- `solver_trace.metrics.optimized_vs_touchdown_anchor_stance_max_position_error_mean_m`
- `solver_trace.metrics.optimized_vs_current_foot_max_position_error_mean_m`
- `solver_trace.metrics.optimized_vs_target_joint_max_abs_delta_mean_rad`
- container log 中的 `Applied joint branch-consistency guard`

## 7. 当前工作区说明

本文档对应的工作区状态是：

- swing-branch penalty 已回退
- global femur software widening 已回退
- 当前代码已经重新编译回保留主线

因此后续模型必须以“当前主线 = stationarity latch + observation/command split + gait dedupe + mode projection/cmd hold + branch guard”为真实起点。
