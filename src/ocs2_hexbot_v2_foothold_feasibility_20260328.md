# OCS2 Hexbot `hexbot_locomotion_v2` 落脚点逻辑接入可行性验证

日期：2026-03-28

## 结论

结论：方案可行，但不是“直接搬运 Python 逻辑即可运行”的零改动移植；它应被视为一个可安全移植到 OCS2 底层的 `foothold generator` 模板。

更准确地说：

- 可行的是：把 `hexbot_locomotion_v2` 的 `tripod_step_delta + tripod_step(front/rear)` 思路移植进 OCS2 的 `ReferenceManager -> SwingTrajectoryPlanner` 链路，作为 swing 腿的 `x/y` touchdown / trajectory 生成器。
- 不可行的是：直接把 `v2` 当前的脚端位置输出或 phase 机理原样塞进 OCS2 solver。两者使用的状态载体、参考输入和约束形式不同。

## 验证目标

本次验证聚焦 4 个问题：

1. `hexbot_locomotion_v2` 的 tripod step / foothold 逻辑接口，是否与当前 OCS2 的 `SwingTrajectoryPlanner`、`GaitSchedule`、`SwitchedModelReferenceManager` 兼容。
2. 该方案是否会破坏当前 `phase14b` 已恢复出来的稳定支撑姿态。
3. 最小修改范围能否限制在 OCS2 六足底层核心，不回流到外围参数和无关模块。
4. 给出明确结论：可行/不可行、风险点、最小接入路径。

## 核心事实

### 1. `v2` 的 tripod 逻辑是清晰、可抽象的

`hexbot_locomotion_v2` 的落脚逻辑可归纳为三层：

- tripod 相位调度：
  - [tripod_scheduler.py](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/src/tripod_scheduler.py#L13)
- 每条腿的步幅增量 `step_delta`：
  - [hexbot_locomotion_v2_node.py](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/src/hexbot_locomotion_v2_node.py#L1143)
- 以前后端点 `rear -> front` 生成 swing / stance 脚端轨迹：
  - [foot_trajectory.py](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/src/foot_trajectory.py#L35)

其中 `step_delta` 的核心是：

- 平移步幅：`translation_dx / translation_dy`
- 旋转步幅：基于脚端 home 点切向方向的 `rotation_dx / rotation_dy`
- 输出：每条腿相对于 `home` 的三维增量，其中 `z = 0`

这说明 `v2` 的可移植“核心资产”不是整个控制器，而是：

- 一套 `tripod group -> swing leg -> front/rear foothold` 的几何生成规则
- 一套随 `cmd_vel` 归一化变化的步幅尺度映射

### 2. OCS2 当前链路的确缺少这层 `x/y foothold`

当前 OCS2 的 `SwingTrajectoryPlanner` 只维护摆动腿法向高度轨迹：

- 接口只有 `getZvelocityConstraint()` / `getZpositionConstraint()`
  - [SwingTrajectoryPlanner.h](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/include/ocs2_hexbot_legged_robot/foot_planner/SwingTrajectoryPlanner.h#L51)
- 内部也只构造 `feetHeightTrajectories_`
  - [SwingTrajectoryPlanner.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/foot_planner/SwingTrajectoryPlanner.cpp#L78)

`SwitchedModelReferenceManager` 当前虽然已经为每个 phase 计算了参考状态对应的脚端世界坐标：

- [SwitchedModelReferenceManager.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/reference_manager/SwitchedModelReferenceManager.cpp#L150)

但它只把这些数据压缩成：

- liftoff normal distance
- touchdown normal distance

最后只喂给 swing planner 的 `z` 序列：

- [SwitchedModelReferenceManager.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/reference_manager/SwitchedModelReferenceManager.cpp#L157)
- [SwitchedModelReferenceManager.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/reference_manager/SwitchedModelReferenceManager.cpp#L165)

PreComputation 和约束层也只消费“法向速度/位置”：

- [LeggedRobotPreComputation.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/LeggedRobotPreComputation.cpp#L85)

这与前期问题定位是一致的：当前 OCS2 仍然没有显式的 swing-foot `x/y` 落脚规划。

### 3. tripod 分组是兼容的

`v2` 的 tripod 分组：

- `A = [RR, RF, LM]`
- `B = [RM, LR, LF]`

见：

- [hexbot_v2_basic_walk_loaded_safe.yaml](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/config/hexbot_v2_basic_walk_loaded_safe.yaml#L95)

OCS2 的 `ModeNumber` 和 `gait.info` 使用的是同一组分腿：

- `TRIPOD_A = RR | RF | LM`
- `TRIPOD_B = RM | LR | LF`

见：

- [MotionPhaseDefinition.h](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/include/ocs2_hexbot_legged_robot/gait/MotionPhaseDefinition.h#L26)
- [gait.info](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/config/command/gait.info#L27)

因此在 gait phase 语义上，`v2` 与 OCS2 是直接兼容的，不需要重写分组定义。

### 4. `home` 足位与 OCS2 loaded-equilibrium 参考是相容的

`v2` 的 loaded-safe stance 足位定义在：

- [hexbot_v2_basic_walk_loaded_safe.yaml](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/config/hexbot_v2_basic_walk_loaded_safe.yaml#L99)

它们来自 live Isaac 站立姿态，属于机身坐标系下的脚端 home 点。

`hexbot_locomotion_v2` 也明确是通过几何链 FK 在 body frame 中计算 neutral/home 足位：

- [hexapod_geometry.py](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/src/hexapod_geometry.py#L126)
- [hexapod_geometry.py](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/src/hexapod_geometry.py#L187)

而 OCS2 当前 loaded-equilibrium 的模型审计显示，参考站姿脚端世界坐标为：

- [hexbot_model_audit_reference_loaded_equilibrium_20260328_031702.txt](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/model_audit/hexbot_model_audit_reference_loaded_equilibrium_20260328_031702.txt#L11)

把两者对比后可见：

- `x/y` 只差约 `1.6 mm ~ 9.5 mm`
- `z` 差约 `95 mm`

其中 `z` 差异是预期现象，不是冲突：`v2` stance 足位是机身坐标系，而 OCS2 审计输出的是世界坐标系。

因此静态几何上可以判定：

- `v2` 的 `home` 足位能作为 OCS2 swing-foot `x/y` 规划的来源模板
- 接入时必须做 body/world 或 terrain-frame 的坐标变换，不能直接拿数值硬塞

### 5. 当前 `HexbotCmdVelTargetNode` 仍只给出 body target，而不是足端 foothold target

当前 OCS2 目标节点只发布两帧 target state：

- 当前 nominal
- lookahead 后的 body target

见：

- [HexbotCmdVelTargetNode.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros/src/HexbotCmdVelTargetNode.cpp#L170)

它做的是 body 位移前推：

- [HexbotCmdVelTargetNode.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot_ros/src/HexbotCmdVelTargetNode.cpp#L191)

并没有输出每条 swing 腿的 touchdown `x/y`。这正是 `v2 foothold generator` 可以接入的位置。

## 与当前 OCS2 的兼容性判断

### 兼容点

1. gait 分腿完全一致，可直接复用 tripod A/B 语义。
2. `SwitchedModelReferenceManager` 已具备每 phase 的参考脚端世界坐标，可作为 foothold 生成的宿主。
3. `SwingTrajectoryPlanner` 已经按“每腿、每相位”组织轨迹数据结构，扩展到 `x/y` 有自然挂点。
4. `EndEffectorLinearConstraint` 本身支持位置和速度线性约束，不需要改整个 solver 架构。

### 不兼容点

1. `v2` 使用 body-frame `home` 足位，OCS2 约束链现在主要工作在 world / terrain frame。
2. `v2` 的步幅是基于归一化 `cmd_vel` 包络和经验 stride 参数；OCS2 当前只有 body lookahead target，没有独立 foothold 参数表。
3. `v2` 的轨迹生成器是显式脚端轨迹控制器；OCS2 是优化求解器，不能直接替换为“命令式脚端控制”。

## 是否会破坏 `phase14b` 稳定姿态

### 当前 `phase14b` 稳态的关键来源

`phase14b` 当前最重要的稳定性证据是：

- `base_drop_m = 0.00754`
- `problematic_block_count = 0`

对应文件：

- [phase14b_deferred_terminal_stance_20260328_084325_summary.json](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/idle_reference_probe/phase14b_deferred_terminal_stance_20260328_084325_summary.json)

这轮稳态主要来自此前已经修复的：

- reference-anchored support plane
- terrain-relative stance / swing normal handling
- hard friction cone
- deferred terminal stance timing

这些都不在 `v2 foothold logic` 的最小接入范围内。

### 结构性判断

如果按最小路径接入，`phase14b` 的稳定链不会被直接破坏，因为：

1. 不需要改 `GaitSchedule` 的 phase 顺序和时序。
2. 不需要改 support-plane 估计逻辑。
3. 不需要改 stance 足端 zero-velocity 约束。
4. 不需要改 friction cone、contact force、runtime guard。

最小接入只会影响：

- swing 腿的目标 `x/y` 端点
- swing 腿在摆动相位中的切向轨迹参考

也就是说，稳定支撑仍由现有 `phase14b` 链条负责，新增逻辑只负责给 swing 腿“往前迈”的几何目标。

### 真实风险

虽然结构上不会直接破坏 `phase14b`，但有 3 个实际风险必须承认：

1. `v2` 的固定 stride 量级可能大于当前 OCS2 body target lookahead 位移。
   - 例如 loaded-safe 配置中 `tripod_forward_stride_m = 0.03`，而当前 OCS2 `0.03 m/s * 0.35 s` 的 body 前推仅约 `0.0105 m`。
   - 如果直接搬固定 stride，swing 腿 touchdown 可能比 body target 激进得多，重新制造 base drop 或 contact residual。
2. frame 变换若处理错误，会把 body-frame foothold 错投到 world / terrain frame，重新引入 `RF/RM` 偏置。
3. 如果同时去改 stance 轨迹，而不是只改 swing 腿，会冲击当前已经恢复好的 support-plane 约束。

因此，“不会破坏 `phase14b`”只能在以下前提下成立：

- 只动 swing 腿 `x/y`
- 保持 stance 逻辑不动
- 步幅先按 OCS2 当前 target body delta 做守门或缩放

## 最小修改范围

最小可行接入范围应严格限制为 3 个文件族：

1. [SwitchedModelReferenceManager.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/reference_manager/SwitchedModelReferenceManager.cpp)
   - 在 `modifyReferences()` 里生成每腿、每 phase 的 swing-foot `x/y` liftoff / touchdown 序列
   - 这里最适合放 `v2` 的 `tripod_step_delta` 思路

2. [SwingTrajectoryPlanner.h](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/include/ocs2_hexbot_legged_robot/foot_planner/SwingTrajectoryPlanner.h)
   - [SwingTrajectoryPlanner.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/foot_planner/SwingTrajectoryPlanner.cpp)
   - 从当前仅 `z` spline，扩展到 `x/y/z` 或至少 `terrain tangential + normal`

3. [LeggedRobotPreComputation.cpp](/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/src/LeggedRobotPreComputation.cpp)
   - 新增 swing 腿的切向目标配置缓存
   - 保留当前 stance zero-velocity 与 support-plane 链不动

不建议在第一步就改：

- `GaitReceiver.cpp`
- `GaitSchedule.cpp`
- `HexbotRuntimeMrtNode.cpp`
- Isaac 侧 drive profile
- 外围 `Q/R/lookahead` 配置

## 最小接入路径

推荐的最小接入路径如下。

### Path A：可行且最小

1. 在 `SwitchedModelReferenceManager::modifyReferences()` 中，为每个 phase 计算：
   - `home` 足位
   - `step_delta`
   - swing 腿 `rear/front` foothold
2. `home` 取当前参考站姿的 body-frame 足位，优先用 loaded-equilibrium stance。
3. `step_delta` 复用 `v2` 公式，但步幅尺度不要直接用固定 `0.03 / 0.015 / 0.035`，而应先受当前 OCS2 target body delta 限幅。
4. 把 `rear -> front` 端点喂给 `SwingTrajectoryPlanner`，扩展成 swing 腿 `x/y/z` 轨迹。
5. 在 `LeggedRobotPreComputation` 中为 swing 腿生成切向跟踪参考，只对 swing 腿生效。

这条路径的特点：

- 不动 gait schedule
- 不动支撑平面
- 不动 stance 约束
- 只补 swing-foot tangential progression

### Path B：不建议作为第一步

直接把 `v2` 的 explicit foot-target controller 或 cadence/filter/IK 机制整体搬到 OCS2。

原因：

- 改动面过大
- 会把两套控制哲学混在一起
- 很难判断是 solver 改善了，还是外层控制绕过了 solver

## 风险点清单

1. 步幅尺度失配：`v2` stride 大于 OCS2 body lookahead 位移时，可能重新触发 trunk sink。
2. 坐标系失配：body-frame foothold 与 world / terrain-frame 约束若没对齐，会再次放大 `RF/RM` 偏置。
3. 参考自洽性：如果 swing touchdown 已前推，而 target body state 仍过短或过小，solver 会在“脚想往前，身子没跟上”之间折衷。
4. 旋转步幅项风险：`tangent_x / tangent_y` 逻辑要在 terrain frame 使用，否则斜支撑平面下会出现非期望 yaw 耦合。

## 最终判断

最终判断：可行。

但这一定义应严格理解为：

- 可行的是“以 `v2` 落脚几何为模板，最小化接入 OCS2 swing-foot tangential planning”。
- 不可行的是“直接照搬 `v2` 全套 tripod 控制实现到 OCS2 solver 主线”。

## 建议的下一步

如果进入实现阶段，建议严格按以下顺序：

1. 先在 `ReferenceManager` 中生成并打印每腿、每 phase 的 `home / rear / front / touchdown`。
2. 只给 swing 腿新增 `x/y` 轨迹，不改 stance 足端逻辑。
3. 先把 stride 限幅到“不超过当前 target body delta 的 1.0~1.2 倍”。
4. 先跑 `phase14b` A/B：
   - 看 `base_drop_m`
   - 看 `problematic_block_count`
   - 看 `optimized_vs_target_foot_max_position_error_mean_m`
   - 看 `delta_forward_m`
5. 只有在 `phase14b` 稳态不退化时，再逐步放宽 stride 上限。
