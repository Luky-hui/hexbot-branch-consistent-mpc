# 六足机器人运行基线

## 当前主线定位

当前第一优先级不是继续推高速度边界，而是把 Hexbot 的 OCS2 集成路线固化成一个：

- 可重复 bring-up
- 可闭环验收
- 可与历史 tripod baseline 做 A/B
- 可直接沉淀为专利技术交底书的系统方案

当前建议主张的系统方案是：

- Isaac Sim + ROS bridge 保持现有上层边界
- 宿主机 `ROS 2 Humble` 运行 bridge 与上层导航
- 容器 `ROS 2 Jazzy` 运行 OCS2 Hexbot 主体
- 主数据闭环为  
  `/cmd_vel -> /hexbot_mpc/input/cmd_vel -> OCS2/MPC -> /hexbot_mpc/output/joint_command -> /joint_command`

## 当前结论

- 历史上已经完成导航耦合验收的是 `hexbot_locomotion_v2 + RTAB-Map + Nav2` tripod 路线。
- 当前正在推进的主线是 `hexbot_locomotion_mpc + ocs2_hexbot_legged_robot_ros`。
- 到 2026-03-27 为止，Humble 宿主机与 Jazzy 容器的 MPC 主发布链已经验证到：
  `/cmd_vel -> OCS2/MPC -> /hexbot_mpc/output/joint_command -> /joint_command`
- 到 2026-03-27 晚些时候为止，`hexbot_runtime_mrt` 已新增运行保护：
  当 `/odom` 或 `/joint_states` 出现非有限值，或 MPC 返回非有限 state/input，或 active policy 已过期时，运行侧会停止继续下发坏命令并主动请求 reset。
- 到 2026-03-27 更晚这轮为止，又确认了 3 个更靠前的系统根因：
  1. OCS2 后端 joint order 与 Isaac 运行时 DOF 顺序不一致，bridge 必须按 Isaac 的 `joint_states.name` 重排后再发 `/joint_command`
  2. `hexbot_isaac_rooted.urdf` 的关节限位过宽（统一 `[-3.14, 3.14]`），会让运行时等价角归一化误落到错误分支；`hexbot_runtime_mrt` 现已收紧到 `coxa ±1.05 / femur ±1.0 / tarsus ±1.35`
  3. `GaitReceiver::preSolverRun()` 原先把 gait 插入接口的结束时间误传成了“时域长度”，现已改成 `finalTime + timeHorizon`
- 到 2026-03-27 当晚最新一轮 bring-up 为止，还确认了 4 个更细的事实：
  1. 关闭 bridge 对 Isaac 的 `velocity/effort` 透传后，Hexbot 仍会继续下沉，因此“执行接口速度字段不兼容”不是主根因
  2. `/hexbot_mpc_target` 始终是正确的中性站姿参考，`defaultJointState` 与已验证 tripod baseline 的 `neutral_joint_positions` 一致
  3. `/hexbot_mpc_solver_diagnostics` 保持 `level=ok`，说明当前不是 solver 直接报失败，而是“求出了一个错误平衡点”
  4. 容器里曾反复残留旧 `hexbot_target` / `hexbot_gait_command` / `robot_state_publisher`，会污染实验现场，因此后续每轮实验前必须先做 runtime cleanup
- 这条保护已经在 `10s @ linear.x=0.03, angular.z=0.12` 的 mixed 用例里真实触发过一次：
  日志出现 `MPC returned non-finite state/input. Dropping backend command and requesting a reset.`，
  之后 `/joint_command` 与 `/odom` 仍保持有限值，没有再次污染成 `NaN`。
- 现在“闭环是否成立”和“相对历史 baseline 是否构成新技术方案”要一起考虑，不能只盯速度优化。

## 2026-03-27 最新前置阻塞：Isaac 资产层已成为第一优先级

到 2026-03-27 深夜这一轮为止，又确认了一个比 OCS2 本体更靠前的阻塞：

- 用户观测到的真实现象已经稳定复现为：
  `躯干 / base_link 持续向中心下沉，随后倾向 L 侧倒塌；tarsus 一开始贴地较稳，后续支撑失效`
- 在没有任何 ROS 控制发布者的情况下，宿主机直接采样到：
  `/joint_command` 发布者 `0`
  `/hexbot_mpc/output/joint_command` 不存在
  但 `/odom` 仍只有约 `z=0.00767`
- 这说明“仿真一开始就已经脏了”本身就是一个真问题，而不是只有 OCS2 才会把机器人带坏

进一步做 Isaac 资产层内窥后，拿到了更硬的证据：

- live stage 中的 `hexbot_isaac_rooted_reimport.usd` 直接包含坏状态
- `/hexbot/hexbot` 的本地 `xformOp:translate` 被写成了  
  `(0.00659299, 0.00083041, -0.04999816)`
- 多个关节的 `state:angular:physics:position` 已经出现大幅异常值，例如：
  `femur_joint_RF=-22.4600`
  `tarsus_joint_RF=22.7974`
  `femur_joint_LF=19.3744`
  `tarsus_joint_LF=-21.4564`
- 这些坏值不是只存在于 live stage，直接用 Isaac 进程打开磁盘上的  
  `hexbot_isaac_rooted_reimport.usd`
  也能读到相同的坏 `xform/state`
- 对照之下，旧的  
  `hexbot_description_ros2/urdf/hexbot_isaac_rooted/hexbot_isaac_rooted.usd`
  没有这些坏状态：
  `/hexbot/hexbot` 仍是零位姿，关节 `state:angular:physics:position` 也仍是 `0`

当前对这条证据链的结论是：

- `reload-stage` / `reload-and-play` 只能重新打开同一个坏 USD，不等于“干净复位”
- 只靠 live articulation 临时把关节摆回参考站姿，机器人仍会慢慢收敛回压缩/趴地姿态
- 这说明下一轮调试前必须先修 Isaac 资产本体，而不是继续盲调 gait / OCS2 代价

### 当前 Isaac 资产层的最新处置状态

本轮已完成的动作：

- `hexbot_drive_profile.json` 已改为默认指向  
  `hexbot_isaac_rooted_reimport/hexbot_isaac_rooted_reimport.usd`
- 同一 profile 的 `joint_targets_deg` 已正式对齐到  
  `ocs2_hexbot_legged_robot/config/command/reference.info`
  的 18 关节中性站姿

本轮未完成且需要警惕的动作：

- 通过 `8226` 远程口在“当前正在运行且加载了目标 reimport USD 的 Isaac 实例”里直接执行  
  `run_hexbot_isaac_repair_pipeline.py --write --skip-inspect`
  后，Isaac 出现卡住现象
- 同时观察到磁盘上的  
  `hexbot_isaac_rooted_reimport.usd`
  从原来的 `163142` 字节变成了 `687` 字节
- `file` 仍然显示它是 `USD crate, version 0.8.0`
  但当前必须把这份文件视为“本轮 live reimport 中途写入后的可疑资产”，在重新生成前不应继续作为可信基线

因此，下一任接手者在重新进入运行实验前，必须先做：

1. 重启或恢复 Isaac Sim，使 `8226` 接口重新可用
2. 不要在“已经加载了目标 reimport USD 的同一 live stage”里直接跑写盘 repair pipeline
3. 改为在独立、干净的 Isaac 进程里重新执行：
   `run_hexbot_isaac_repair_pipeline.py --write`
4. 重新审计生成后的 `hexbot_isaac_rooted_reimport.usd`
   是否恢复为完整资产，再继续 OCS2 / gait 调试

## 专利主线闭环验收

### 闭环验收目标

一次有效验收至少同时满足：

- Isaac 在线
- `/clock + /joint_states + /odom` 在线
- 宿主机 bridge 在线
- Jazzy 容器 OCS2 在线
- `/hexbot_mpc/output/joint_command` 连续输出
- `/joint_command` 真正驱动机器人产生可观测动作

### 统一验收工具

宿主机运行态探针：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py
```

闭环验收探针：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/probe_hexbot_closed_loop.py \
  --use-sim-time \
  --duration 5.0 \
  --linear-x 0.03 \
  --angular-z 0.0
```

一键闭环验收入口：

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/scripts/run_patent_closed_loop_case.sh
```

这条一键脚本现在会同时产出：

- 宿主机闭环探针 JSON：
  `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/patent_closed_loop_probe_latest.json`
- 容器侧 `hexbot_mpc_target`、`mode_schedule`、`solver_diagnostics` 采样：
  `patent_closed_loop_target_latest.yaml`
  `patent_closed_loop_mode_latest.yaml`
  `patent_closed_loop_solver_latest.yaml`
- 合成后的统一证据 JSON：
  `/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/log_humble/patent_closed_loop_summary_latest.json`

### 当前 A/B 采证建议

至少保留以下三类对比：

1. 同一 `cmd_vel` 下旧 `hexbot_locomotion_v2` 与新 OCS2 主线的跟踪结果
2. 扰动 / 重启后的恢复时间
3. 长时运行下的稳定性与停顿情况

所有 A/B 结果都建议保留 JSON 原始输出，而不是只留截图。

### 当前已拿到的首轮 A/B 证据

同样使用 `5s @ linear.x=0.03, angular.z=0.0`：

| 对比项 | 旧 `hexbot_locomotion_v2` | 新 `hexbot_locomotion_mpc + OCS2` | 当前解读 |
| --- | --- | --- | --- |
| `/joint_command` 是否连续输出 | 是 | 是 | 两条链都已形成最终关节命令 |
| `/hexbot_mpc/output/joint_command` 是否存在 | 否 | 是 | 新方案后端命令链成立 |
| 后端命令与运行时命令是否一致 | 不适用 | `max_abs_diff_rad=0.0` | bridge 回写正确 |
| `delta_forward_m` | `0.1190` | `-0.0318` | OCS2 闭环已通，但当前前向跟踪效果明显不足 |
| `delta_lateral_m` | `0.00006` | `0.0606` | OCS2 当前存在明显侧向偏移 |
| `max_still_sec` | `0.0667` | `0.3000` | OCS2 当前停顿更明显 |

这组结果的意义是：

- 新方案已经具备“可作为系统发明主张对象”的闭环结构证据
- 但它还没有在运动效果上超过旧 baseline
- 因此当前工作重点应是继续围绕这条闭环收集 A/B 与问题归因，而不是先冲更高速度

### 当前最新稳定性结论

- 纯转向 `5s @ angular.z=0.12` 已可得到约 `delta_yaw_rad=0.4424`
- mixed `10s @ linear.x=0.03, angular.z=0.12` 仍可能触发后端非有限 state/input
- 但最新运行保护已经把问题从“全链路 NaN 崩坏”收敛成“自动 reset 后动作退化、跟踪效果仍差”
- 因此当前第一优先级不再是追更高速度，而是继续解决线速度与 mixed case 的可跟踪性

### 2026-03-27 最新 mixed 闭环证据

基于
`bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/scripts/run_patent_closed_loop_case.sh --duration 10.0 --linear-x 0.03 --angular-z 0.12`
这一轮最新合成结果显示：

- `ready=true`
- `solver_diagnostics.max_level=0`
- `mode_sequence={21, 42}`
- `backend_vs_runtime_joint_command.max_abs_diff_rad=0.0`
- `target snapshot` 的局部目标量不是零：
  `forward=0.0360 m`，`yaw=0.1440 rad`
- 但真实运动只有：
  `actual_forward_m=0.0127`，`actual_yaw_rad=0.0219`
- 对应缺口为：
  `forward_gap_m=-0.0233`，`yaw_gap_rad=-0.1221`

这组证据的当前解读是：

- mixed 工况下，“目标生成错误”已经不是第一怀疑对象
- 步态调度也已经切到了运动 gait，不是一直停在 stance
- 当前主矛盾已进一步收敛为：
  求解与执行链在 mixed 条件下明显欠跟踪，尤其是 yaw 跟踪不足

### 2026-03-27 站立坍塌最新结论

这一轮在“只给 stance、不给 `cmd_vel`”的干净测试里，新增了几条非常关键的证据：

- 用户肉眼观测到的真实现象是：
  `base_link / 躯干持续下沉，六足基本踩地不走，随后开始抽搐`
- 关闭 bridge 对 Isaac 的 joint velocity / effort 透传后，上述现象仍然存在
- `/hexbot_mpc_target` 抓到的参考仍是中性站姿，不是某个蹲伏目标
- `/hexbot_mpc_solver_diagnostics` 抓到的是 `hexbot/mpc_solver: ok`
- 但 `hexbot_runtime_mrt` 第一帧就记录到：
  `femur_joint_RF raw≈-1.271 / measured≈-0.205`
  `tarsus_joint_LF command≈-1.272 / measured≈-0.208`
  也就是 solver 一启动就想把正确的中性站姿拉向错误的极端关节解

因此到目前为止可以明确排除：

- 不是 bridge 重排错了
- 不是 `reference.info` 本身错了
- 不是 runtime target 节点把站姿参考发错了
- 也不是单纯因为透传 velocity / effort 才把 Isaac 驱坏

当前最合理的主根因假设已经收敛为：

- 六足 OCS2 模型 / 约束 / 代价在 stance 下存在错误平衡点
- 或者仍有“脏现场多实例发布”在扰动 `hexbot_mpc_target / mode_schedule`

为避免后者继续污染判断，后续实验统一改为先执行：

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/cleanup_hexbot_runtime.sh
```

### 2026-03-27 输入代价缩放试验结论

为了验证“是不是足端/关节速度代价把 locomotion 压死了”，这轮做过一次最小单参数试验：

- 现有六足 `task.info` 的 `R` 使用 `scaling 1e+0`
- 上游四足 example 的 `R` 使用 `scaling 1e-3`
- `LeggedRobotInterface::initializeInputCostWeight()` 会把 `R` 的后半块当作足端速度代价，再通过 Jacobian 映射到关节速度代价

结论如下：

- 原始 `1e+0`：
  solver 稳定，但 pure forward / pure yaw / mixed 都明显欠跟踪
- 试验 `1e-3`：
  机体运动量略有提升，但 `/joint_command -> /joint_states` 偏差暴涨到约 `2.74 rad`，并出现显著 `dynamics_violation` 与 `eq_constraints_sse`
- 试验 `1e-2`：
  比 `1e-3` 更保守，但仍在运行中触发  
  `MPC returned non-finite state/input. Dropping backend command and requesting a reset.`

因此当前可以确认：

- `R scaling` 是一个高敏感参数
- 现有六足系统确实不能直接照搬上游四足的 `1e-3`
- 下一步应从“关节角连续性 / actuator-feasible 归一化 / 六足专属代价再整定”联动收敛，而不是只改一个缩放值

### 2026-03-27 反馈污染与运行保护修复

这轮又确认了两个更靠前的真实根因：

- `LeggedRobotVisualizer` 原先会向全局 `/joint_states` 发布可视化关节状态。
  这会让 Isaac 的真实 `/joint_states` 和 OCS2 可视化状态混到同一总线里，
  宿主机 bridge 再把混合后的消息转回 `/hexbot_mpc/input/joint_states`，形成假反馈污染。
- `hexbot_runtime_mrt` 在检测到大幅关节归一化跳变后已经会主动 `reset`，
  但旧逻辑仍会继续访问 `mrt_->getCommand()`，导致节点直接崩溃。

已完成的修复：

- `LeggedRobotVisualizer` 的关节可视化输出已改到
  `/hexbot/visualization/joint_states`，不再污染真实 `/joint_states`
- `hexbot_runtime_mrt` 新增了“等价角大跳 / 归一化后跟踪误差过大”保护，
  并修复了保护触发后仍访问旧 command 的崩溃路径

修复后的最新证据：

- `/joint_states` 已恢复为单发布源，`/hexbot/visualization/joint_states` 单独存在
- pure yaw `5s @ angular.z=0.12`：
  `command_vs_actual_joint_state.max_abs_diff_rad=0.0588`
  但 `actual_yaw_rad=-0.0264`，仍明显欠跟踪且方向符号异常
- pure forward `5s @ linear.x=0.03`：
  `command_vs_actual_joint_state.max_abs_diff_rad=0.0593`
  但 `actual_forward_m=-0.00065`，基本没有形成期望前进

这说明当前主矛盾已经进一步收敛为：

- 执行层与反馈链的主要污染已经切掉
- 关节命令和实测关节已经基本对齐
- 剩余问题更像是六足模型 / 参考方向 / 代价与接触切换的系统性欠跟踪，而不是单纯执行层不跟命令

## 标准编译

```bash
cd /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/host_build_runtime_stack.sh
source /opt/ros/humble/setup.bash
source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_humble/setup.bash
```

## 宿主机前置检查

先确认 URDF 就绪：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/check_hexbot_mpc_readiness.py \
  --urdf /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf
```

再确认 Isaac Sim 运行态话题真的活着：

```bash
python3 /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/src/check_hexbot_runtime_topics.py
```

探针默认要求：

- `/clock`
- `/joint_states`
- `/odom`

如果这里返回 `ready=false`，优先排查：

- Isaac 场景是否已经真正加载
- 仿真是否已经开始播放
- ROS bridge 是否真的启用

注意：

- 只看到 Isaac Sim 进程并不代表运行链 ready
- 在这一步失败时，不要先怀疑 OCS2 求解器

## 当前 MPC/WBC bring-up 顺序

### 2026-03-28 OCS2 底层审计入口

当前主线已经停止继续做外围 `Q/R/lookahead/smoke` 调参。
本轮最新结论和后续修复顺序已整理到：

`/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_algorithm_bottom_up_audit_20260328.md`

结论摘要：

- 外围调参只能让下压和倒地变慢，不能消除根因
- 当前最关键的问题已经收敛到 OCS2 底层
- 重点不是再调速度，而是修正：
  - `terrainHeight = 0.0` 的平地世界假设
  - terrain-relative 足端约束
  - nominal contact force / 初始化支撑分配
  - 六足运动学与接触映射的数值审计
- Phase 0 仪器化之后又确认了一条新的底层事实：
  - upstream 继承下来的摩擦锥 `regularization` 对 Hexbot 六足支撑力级别过大，会把“几乎纯法向支撑”也算成约束违例；这条已在 2026-03-28 代码中按名义支撑力做了自适应缩放
- 2026-03-28 又补充确认了另一条底层事实：
  - OCS2 的 guarded joint limit 和 runtime 的 equivalent-angle branch 必须使用同一套可行域；否则求解器会从“先天违约”的关节分支起跑，最后只剩 runtime 末端裁剪在兜底
- 当天已落地的新增证据链：
  - `hexbot_model_audit` 数值审计表明 nominal reference 下并不存在只属于 `RF/RM` 的 Jacobian/镜像几何异常
  - `bottomup_phase2_observationwrap_20260328_023708_summary.json`
    - 观测分支归一化后，`backend_vs_runtime_max_abs_diff_rad = 0.0`
    - 但 `command_vs_actual_max_abs_diff_rad` 仍约 `0.2676`
  - `bottomup_phase2_hardguard_relinked_20260328_024525_summary.json`
    - OCS2 guarded joint limit 已经生效
    - 但 runtime 仍按硬限位分支工作，导致 `command_vs_actual_max_abs_diff_rad` 仍约 `0.2886`
  - `bottomup_phase2_guarded_runtime_20260328_024839_summary.json`
    - runtime 观测与命令分支都已对齐到 guarded feasible branch
    - `command_vs_actual_max_abs_diff_rad` 收敛到约 `0.0308`
    - `solver_diagnostics.perf_ineq_constraints_sse = 0.0`
    - 但 `delta_forward_m` 仍只有约 `0.0031 / 5s`，说明闭环一致性明显改善，稳定行走仍未恢复
- 本轮所有 probe 都按同一纪律执行：
  - 脚本前先 `cleanup_hexbot_runtime.sh` 和 Isaac `stop`
  - 脚本后立刻再次 cleanup，并确认 Isaac 回到 `stopped: True`

### 1. 宿主机启动 bridge

```bash
cd /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws
source /opt/ros/humble/setup.bash
source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_humble/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch hexbot_locomotion_mpc hexbot_locomotion_mpc.launch.py \
  use_sim_time:=true \
  geometry_config_path:=/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_mpc/config/hexbot_mpc_default.yaml
```

### 2. 容器内启动默认 SQP

建议先用 detached 容器固定运行边界：

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_run_hexbot_jazzy.sh
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/container_exec_hexbot_jazzy.sh
```

再在容器内执行：

```bash
cd /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws
source /opt/ros/jazzy/setup.bash
source /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/install_jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_DOMAIN_ID=0
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
ros2 launch ocs2_hexbot_legged_robot_ros hexbot_sqp.launch.py
```

### 3. 如需保守 smoke 配置

```bash
ros2 launch ocs2_hexbot_legged_robot_ros hexbot_sqp.launch.py \
  taskFile:=/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/config/mpc/task_smoke.info \
  gaitCommandFile:=/home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/ocs2_hexbot_legged_robot/config/command/gait_smoke.info
```

### 4. 当前已确认的关键根因

- Humble 宿主机和 Jazzy 容器混跑时，必须显式设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`
- `hexbot_runtime_mrt` 不能把 Unix 绝对时间戳直接传给 OCS2
- 如果 observation time 用绝对秒，`GaitSchedule::tileModeSequenceTemplate()` 会把 gait 从 `0` 铺到超大时间上界，最终触发 OOM killer

## 历史导航验收基线

下面这条仍然是历史上已验收的导航链路，但它属于旧 tripod 控制，只能作为 A/B 对比对象，不是当前专利主线：

### 1. 启动运动控制

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py use_sim_time:=true mode:=tripod
```

### 2. 启动 RTAB-Map

```bash
ros2 launch hexbot_isaac_runtime rtab-map-scan.launch.py use_sim_time:=true launch_viz:=false
```

### 3. 启动 Nav2

```bash
ros2 launch hexbot_isaac_runtime navigation2.launch.py use_sim_time:=true
```

### 4. 启动 RViz

```bash
ros2 launch hexbot_isaac_runtime rviz.launch.py use_sim_time:=true
```

### 5. 旧导航验收入口

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/scripts/run_single_nav_case.sh --preset nav_fwd_0p5
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/scripts/run_single_nav_case.sh --preset nav_fwd_1p0
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/scripts/run_single_nav_case.sh --preset nav_turn15_fwd_0p5
```

历史验收结果摘要：

| 预设 | 是否成功 | 是否绊腿/拖腿 | 是否停顿超过 2 秒 | 终点误差 m | 横向误差 m | 偏航误差 rad |
| --- | --- | --- | --- | --- | --- | --- |
| `nav_fwd_0p5` | 是 | 否 | 否 | 0.0474 | -0.0086 | 0.0947 |
| `nav_fwd_1p0` | 是 | 否 | 否 | 0.0218 | -0.0124 | 0.0988 |
| `nav_turn15_fwd_0p5` | 是 | 否 | 否 | 0.0367 | -0.0046 | 0.0942 |

## 2026-03-28 自主闭环追踪新增结论

本轮已经把“看画面猜问题”升级成自动证据链，新增了：

- `src/hexbot_locomotion_mpc/src/probe_hexbot_posture.py`
  - 自动判定 `/hexbot/hexbot` 或 `/hexbot/hexbot/base_link` 是否出现下压或怪异姿态
- `src/hexbot_locomotion_mpc/src/capture_hexbot_ocs2_topics.py`
  - 在 Jazzy 容器内直接抓取 `/hexbot_mpc/input/cmd_vel`、`/hexbot_mpc_target`、`/hexbot_mpc_mode_schedule`、`/hexbot_mpc_solver_diagnostics`
- `src/hexbot_locomotion_mpc/scripts/run_idle_reference_safety_probe.sh`
  - 现在会在 run 前后自动做 clean reset / stop，并把 posture、runtime_topics、ocs2_topics 一起汇总

最新稳定证据见：

- `log_humble/idle_reference_probe/phase3_autonomous_cmd_trace_fixed_20260328_031249_summary.json`
- `src/hexbot_contact_execution_audit_20260328.md`

这轮关键结论：

- `/hexbot_mpc/input/cmd_vel` 已确认收到非零命令
  - `bridge_cmd_vel_snapshot.max_abs_linear_x = 0.03`
- `hexbot_auto_gait_command` 已确认切到 moving gait
  - `mode_sequence = [21, 42]`
- `hexbot_cmd_vel_target` 已确认生成前推目标
  - `target_snapshot.delta_body.forward_m = 0.0105`
  - `horizon_sec = 0.35`
- solver 当前不是直接报错退出
  - `solver max_level = 0`
- 但机器人真实位移仍很小
  - `delta_forward_m ≈ 0.00177 / 8s`
  - `max_still_sec ≈ 6.27s`
- 自动姿态判定未把当前 run 归类为“躯干下压/怪异姿态”
  - `base_drop_m ≈ 0.0283`
  - `max_abs_roll_rad ≈ 0.0547`
  - `max_abs_pitch_rad ≈ 0.0569`
  - `trunk_sink_detected = false`
  - `weird_pose_detected = false`

因此，主阻塞已经进一步收缩为：

- 不是 `/cmd_vel` 没进容器
- 不是 auto gait 没切 moving
- 不是 target 没前推
- 而是 “moving gait + forward target 已经成立，但执行层/接触层仍没有把它转成有效前进”

换句话说，当前更像是：

- OCS2/bridge/target 链条已经能生成看起来正确的意图
- 但 solver 输出到物理执行之间，Hexbot 仍停在低效支撑/错误平衡点附近
- 2026-03-28 新增的动态接触摘要还进一步表明：
  - `RM / LR / LF` 在多个 moving gait 时间窗里长期达不到参考离地高度
  - `LM / RM` 中腿会反复承担明显高于同组三角支撑其余两腿的法向载荷
  - 当前更像“动态接触执行失真”，而不是“意图生成链路中断”

## 运行规则

- RViz 的 `Fixed Frame` 必须为 `map`
- `/joint_command` 只能有一个有效发布者
- `rtabmap` 只能保留一个有效进程
- 任何时候都先确认宿主机真实话题在线，再继续查容器或控制器
- 运行实验脚本前必须保证 Isaac 已回到干净初始态；当前已新增 `src/hexbot_locomotion_mpc/src/ensure_isaac_clean_start.py`
- `run_idle_reference_safety_probe.sh` 现在会先做 Isaac preflight，但 clean-start 已不再硬编码 `world z > 0.05`
- clean-start 目前主要看：
  - `/odom` 的 roll / pitch 是否过大
  - `/joint_states` 相对 `reference.info` 的站姿误差是否过大
  - 关节速度是否异常
- 运行实验后必须及时停掉 bridge / backend / Isaac playback，避免残留控制指令继续污染现场
- 目前闭环 run 控制台仍可能出现 `sequence size exceeds remaining buffer` 的 DDS 噪声；它没有阻止 summary 生成，但来源还未完全定位，后续要继续审计

## 相关文档

- `启动命令`
- `Docker容器说明`
- `hexbot_patent_execution_checklist.md`
- `hexbot_patent_disclosure_outline.md`
- `hexapod_rebuild_preparation.md`
- `hexapod_reference_parameter_comparison.md`
- `hexbot_locomotion_v2/ARCHITECT_HANDOFF_2026-03-25.md`

## 2026-03-28 SQP 硬摩擦锥修复新增结论

本轮继续沿着 `src/hexbot_contact_execution_audit_20260328.md` 往下修，确认了一个很关键的底层缺口：

- 当前 Hexbot 主线虽然一直在跑 SQP
- 但 `legged_robot_sqp_mpc` 过去实际上仍默认使用 soft friction cone
- 这会允许 solver 在 tripod stance 中保留不够物理的接触力解

本轮已做的修复：

- `src/ocs2_hexbot_legged_robot_ros/src/LeggedRobotSqpMpcNode.cpp`
  默认启用 `useHardFrictionConeConstraint = true`
- `src/ocs2_hexbot_legged_robot_ros/launch/hexbot_sqp.launch.py`
  同步增加并默认传入该参数
- `src/hexbot_locomotion_mpc/scripts/run_idle_reference_safety_probe.sh`
  现在会在 run 前先 `stop` Isaac，并在退出清理时再次 `stop`
  以避免残留 playback/控制指令继续污染现场

最新验证证据：

- `log_humble/idle_reference_probe/phase4_sqp_hard_friction_20260328_032918_summary.json`
- `log_humble/idle_reference_probe/phase4_sqp_hard_friction_20260328_032918_precomp.json`

这轮结果的意义很明确：

- `perf_ineq_constraints_sse = 0.0`
- `trunk_sink_detected = false`
- `weird_pose_detected = false`
- swing 腿对参考 clearance 的跟随明显改善
  - 之前 `RM / LR / LF` 最差落后约 `2.5 cm ~ 3.5 cm`
  - 现在 clean block 里只剩约 `7 mm` 量级的早期缺口
  - 且没有 swing 腿再掉到 support plane 以下

但这并不等于“已经恢复正常行走”：

- `delta_forward_m` 仍只有约 `0.00378 / 8s`
- `max_still_sec` 仍约 `8.03s`
- `command_vs_actual_joint_state.max_abs_diff_rad ≈ 0.2203`
- 最大 tracking gap 仍集中在 `RF / RM / RR` 的 femur

因此当前主阻塞已再次收窄为：

- OCS2 moving-contact 解的可行性明显改善了
- 机器人也没有重新回到明显下压/怪姿态
- 但“可行支撑”还没有转成“有效推进”
- 下一阶段应把主焦点放到：
  - solver 输出的 joint/posture 为什么仍偏向前侧 femur 压低
  - backend 命令到 Isaac 实际关节状态之间为什么还保留约 `0.1 ~ 0.22 rad` 级别的 tracking lag

## 2026-03-28 Phase5b 目标高度链进一步收紧

本轮继续沿着 `phase5b_joint_bias_trace_warm_20260328_033900_*` 做了更细的 target / posture / reference 对照，新增结论如下：

- `expected_vs_backend_joint_command` 与 `expected_vs_runtime_joint_command` 最大差值都到了约 `0.897 rad`
- `backend_vs_runtime_joint_command.max_abs_diff_rad = 0.0`
- 说明当前主要不是 runtime 把正常命令拉歪，而是 solver 本身就在主动下发深蹲姿态
- 同一轮 `posture.json` 还显示：
  - `trunk_sink_detected = true`
  - `base_drop_m ≈ 0.0521`
  - 但 `weird_pose_detected = false`
- 同时 `mode_schedule_snapshot` 又停在单一 `63`，`target_snapshot` 末端也是零位移
  - 这说明问题不只发生在 moving gait
  - 就连 stance / hold target 链本身也允许基准高度继续漂移

因此本轮源码继续做了一个更底层的修复准备：

- `src/ocs2_hexbot_legged_robot/include/ocs2_hexbot_legged_robot/common/utils.h`
  新增了：
  - `estimateSupportPlaneFromLowestFeet()`
  - `solveSupportPlaneHeightAtXY()`
- `src/ocs2_hexbot_legged_robot_ros/src/HexbotCmdVelTargetNode.cpp`
  不再直接把 live 观测 `z` 当 nominal target，而是：
  - 先从 reference/defaultJointState 求出“名义 base 相对支撑平面的法向高度”
  - 再把这个 nominal height 映射到 live support plane 上
- `src/ocs2_hexbot_legged_robot_ros/src/HexbotRuntimeMrtNode.cpp`
  reset/hold target 也同步切到同一套 nominal base-to-support-plane 逻辑

当前验证状态需要单独说明：

- `HexbotCmdVelTargetNode.cpp` 已通过基于保存下来的 Jazzy compile command 的宿主机语法校验
- 但这台机器当前拿不到 Docker / Jazzy 容器权限，无法完成真正的 Jazzy 重编与运行态回归
- 所以这轮修复已经落在源码，但尚未形成新的 runtime 证据文件

后续验证更新：

- 2026-03-28 随后已重新打通 Jazzy 容器执行链，并完成真实重编与回归
- `phase6_support_plane_nominal_height_20260328_035656_*`
  证明“support-plane-relative nominal height”代码路径已经生效：
  - container log 中新日志已出现
  - `target_snapshot.goal_pose.z` 被明确拉到新的 nominal height
- 但第一版实现仍把 support plane 跟着 live 下沉一起更新
  - 导致 target `z` 被错误带到约 `-0.115`
  - 结果下压并未改善

因此又补了第二轮修复：

- target/reset 参考面改为 latched support-plane baseline
  - 启动后只允许 support plane 高度“向上修正”
  - 不再允许随着沉陷足端继续向下漂移

最新对照证据：

- `phase6b_latched_support_plane_baseline_20260328_035943_*`
  - target `z` 已从约 `-0.115` 拉回约 `-0.034`
  - 说明“高度参考链本身”已经被修正到预期方向
  - 但 `expected_vs_backend_joint_command.max_abs_diff_rad` 仍约 `0.95`
  - `base_drop_m` 仍约 `0.052`
  - 即 solver 对修正后的高度目标几乎没有响应

这轮最关键的新结论是：

- 现在不能再把主阻塞归到 target/base-height reference 链
- 这条链已经被修正，并且能在 topic 证据里看到 target `z` 变化
- 但 solver 仍主动把 femur/tarsus 推向深蹲分支

于是我又做了一个 reference A/B：

- `phase6c_loaded_equilibrium_ref_20260328_040145_*`
  使用 `reference_loaded_equilibrium.info`

这个 A/B 的意义非常大：

- `perf_ineq_constraints_sse` 从约 `0.146967` 收敛到约 `0.000230`
- `base_drop_m` 从约 `0.0521` 降到约 `0.0449`
- `expected_vs_backend_joint_command.max_abs_diff_rad` 从约 `0.95` 降到约 `0.80`
- `problematic_block_count` 下降，clean blocks 增加

但它还没有把机器人带回正常行走：

- `delta_forward_m` 反而只有约 `0.000096`
- 仍然存在真实下压
- femur_joint_RF / RM / RR 仍然明显偏向深蹲分支

因此当前最可信的收敛判断变成：

- “错误 nominal reference / loaded equilibrium 失配”是一个真实次级根因
- 它会放大 solver 走向错误平衡点和约束违例
- 但即便切到 loaded equilibrium reference，仍不足以恢复稳定前进
- 所以下一阶段应继续深挖 solver 为什么在更干净 reference 下仍保留前侧 femur 深蹲偏置

随后我补了 Phase 7 的 runtime solver trace 仪器化：

- `HexbotRuntimeMrtNode.cpp` 现在会周期性输出 `HEXBOT_SOLVER_TRACE`
- `run_idle_reference_safety_probe.sh` 会自动聚合成 `*_solver_trace.json`

验证证据：

- `phase7_solver_trace_reference_20260328_041349_*`
- `phase7b_solver_trace_loaded_equilibrium_20260328_041438_*`

这两轮把结论又往前推进了一层：

- 不论是 `reference.info` 还是 `reference_loaded_equilibrium.info`
- `optimized_minus_target_base_height_mean_m` 都是正值
  - 分别约 `+0.032 m` 和 `+0.025 m`
- `crouch_sample_count = 0`

这意味着：

- solver 内部的 optimized state 并没有把 base height 解到低于 target
- 所以“solver 在状态层直接下压 base”已经不是最符合证据的解释

但这里要保留一个关键限定：

- 这个判断说的是“相对 support plane 的 base height”
- 原始 trace 里 `optimized_base_z` 仍经常低于 `target_interpolated_base_z`
- 只是 support plane / 足端世界高度掉得更快，所以 `optimized_base_height_above_support` 反而更高

因此更准确的表述不是“solver 完全没压低 base”，而是：

- solver 更像是在选一条 `base` 和 `support plane` 一起下沉的 joint-space branch
- 其中足端世界位置塌得比 base 更快
- 这也更符合我们肉眼看到的机身下压与支撑失效

但与此同时：

- `optimized_vs_target_joint_max_abs_delta_mean_rad`
  - reference 约 `0.879`
  - loaded equilibrium 约 `0.722`
- 主导偏差关节仍集中在
  - `femur_joint_RF / RM / RR`
  - `tarsus_joint_RF / RM / RR`
- 实际姿态探针仍报告真实 trunk sink

因此当前最新判断应改写为：

- target/base-height reference 链已基本修正
- solver state 自身的 base height 也不是直接塌下去的
- 真正剩下的主阻塞更像是
  - solver 仍选择了一个与 target 严重偏离的 joint-space crouched branch
  - 而这条 branch 在 Isaac 物理层无法实现为“高 base + 有效推进”，最终表现成机身下压和几乎不前进

下一步不该再回头调外围参数，而应继续审计：

- centroidal / RBD / joint target 的一致性
- 为什么优化态 base 仍高，但 femur/tarsus 已经显著偏离 target
- 这条高-base / 深蹲腿的解是在哪里被允许或偏好的

随后我把 solver trace 又往下打到足端层，新增了：

- optimized foot world pose vs target foot world pose
- target support plane vs optimized support plane 的高度差

验证证据：

- `phase8_foot_trace_reference_20260328_042300_*`

这轮给出了目前最硬的一组定位证据：

- `optimized_minus_target_support_height_mean_m ≈ -0.0657`
- `optimized_vs_target_foot_max_position_error_mean_m ≈ 0.0867`
- `optimized_vs_target_foot_max_position_error_max_m ≈ 0.1236`
- `optimized_vs_target_foot_max_abs_z_delta_mean_m ≈ 0.0738`
- `optimized_vs_target_foot_max_abs_z_delta_max_m ≈ 0.1008`
- `optimized_vs_target_foot_most_negative_normal_delta_min_m ≈ -0.1000`

最关键的是支配腿已经浮出来了：

- `foot_RF` 是最主要的坏分支腿
- `foot_LF` 次之
- `foot_RM` 也反复出现

也就是说，现在不能再泛泛地说“腿部整体选错分支”，而应该更具体地说：

- solver 正在把目标足端世界位置拉塌
- 其中 `RF` 最严重，局部样本里能掉到比 target 低约 `10 cm`
- 同时 `LF` 与 `RM` 也会参与把 support plane 往下拖
- base 下压是这个足端世界坐标塌陷的结果，而不是单独的 base 目标错误

因此最新主阻塞已经进一步收缩成：

- `target foot world pose -> optimized foot world pose` 的一致性被破坏
- 问题核心更接近前右腿主导的 foot/joint branch collapse
- 下一步应该直接审 `reference/swing planner/contact mode` 到 `end-effector kinematics` 这一段，查为什么 `RF` 会系统性落到错误世界高度

随后我继续把 support-plane 链从“候选状态自估”改成“reference/target 锚定”，核心改动落在：

- `SwitchedModelReferenceManager`
- `LeggedRobotPreComputation`
- `LeggedRobotQuadraticTrackingCost`
- `LeggedRobotInitializer`

验证证据：

- `phase9_reference_anchored_support_plane_20260328_043250_*`

这轮是有效收敛，而不是噪声波动：

- `optimized_minus_target_support_height_mean_m`
  - 从 phase8 的约 `-0.0657 m`
  - 收敛到约 `-0.0246 m`
- `optimized_vs_target_foot_max_position_error_mean_m`
  - 从约 `0.0867 m`
  - 收敛到约 `0.0341 m`
- `optimized_vs_target_foot_max_abs_z_delta_mean_m`
  - 从约 `0.0738 m`
  - 收敛到约 `0.0302 m`
- `optimized_vs_target_joint_max_abs_delta_mean_rad`
  - 从约 `0.879`
  - 收敛到约 `0.491`
- posture probe
  - `base_drop_m ≈ 0.0276`
  - `trunk_sink_detected = false`
  - `weird_pose_detected = false`

这说明：

- “support plane 跟着候选状态一起掉”确实是一个真实底层 defect
- 修掉后，solver 的足端世界高度塌陷明显缩小
- 但机器人仍没有恢复有效推进

phase9 后仍残留的主阻塞是：

- `delta_forward_m ≈ 0.00394 / 8s`
- `max_still_sec ≈ 6.47s`
- `foot_RF` 仍主导最坏 foot-drop 样本
- `foot_RM` 仍在 moving 样本中反复参与 secondary collapse

我随后又试了一轮更直接的实验补丁：

- 给 stance 脚额外加 `normal distance = 0` 的硬约束

验证证据：

- `phase10_stance_normal_distance_20260328_044013_*`

结论是这条路当前不适合留在主线：

- 它没有恢复净推进
  - `delta_forward_m ≈ 0.00377 / 8s`
- 虽然 `max_still_sec` 降到约 `3.37s`
  - 但 solver 代价与约束残差变差
- `perf_eq_constraints_sse ≈ 0.002622`
- `perf_ineq_constraints_sse ≈ 0.515147`
- container log 里还出现了新的
  - `StanceNormalDistanceConstraintCppAd` 大残差
  - friction cone residual

因此当前主线结论更新为：

- `phase9` 的 reference-anchored support-plane 修复应保留
- `phase10` 的直接 stance-normal-distance 硬约束实验应视为失败实验，不留在代码主线
- 下一步不该再继续堆新硬约束，而应回到
  - `RF/RM` 主导的 target foot world pose 与 optimized foot world pose 为什么仍在 moving gait 下分叉
  - 以及 zero-velocity / normal-velocity 在离散化和执行层之间为什么还留有厘米级残差

我随后把 runtime solver trace 再补成三方足端对比：

- `current/measured`
- `optimized`
- `target`

并新增了 `measured_vs_target_joint` 统计，验证证据：

- `phase11_current_optimized_target_foot_trace_20260328_044951_*`

这轮把“solver 问题”和“执行层问题”切得很清楚：

- `optimized_vs_current_foot_max_position_error_mean_m ≈ 0.00186`
- `optimized_vs_current_foot_max_position_error_max_m ≈ 0.00449`
- `optimized_vs_current_foot_max_abs_z_delta_mean_m ≈ 0.00170`
- `optimized_vs_current_foot_max_abs_z_delta_max_m ≈ 0.00436`

但与此同时：

- `optimized_vs_target_foot_max_position_error_mean_m ≈ 0.0365`
- `current_vs_target_foot_max_position_error_mean_m ≈ 0.0369`
- `optimized_vs_target_foot_max_abs_z_delta_mean_m ≈ 0.0328`
- `current_vs_target_foot_max_abs_z_delta_mean_m ≈ 0.0339`
- `optimized_minus_target_support_height_mean_m ≈ -0.0267`
- `current_minus_target_support_height_mean_m ≈ -0.0268`

而 joint 层也同样吻合：

- `measured_vs_target_joint_max_abs_delta_mean_rad ≈ 0.522`
- `optimized_vs_target_joint_max_abs_delta_mean_rad ≈ 0.506`
- `optimized_vs_measured_joint_max_abs_delta_mean_rad ≈ 0.054`

这组证据说明：

- Isaac 执行层当前不是主阻塞
- runtime / physics 基本是在忠实执行 solver 给出的 branch
- 当前 real robot posture 与 solver optimized state 之间只有毫米级足端差
- 但 solver 和 measured 一起相对 target 偏离了厘米级

所以当前最准确的表述应更新为：

- 剩余主阻塞已经不再是“执行层把好解跑坏”
- 而是“solver/reference 本身仍在 moving gait 下选出错误的 foot-world / joint branch”
- 其中 `RF` 仍是主责任腿，`RM` 仍是次级坏腿

因此后续下一步应该正式转向：

- `reference target` 与 `swing planner / mode schedule`
- `ZeroVelocityConstraintCppAd / NormalVelocityConstraintCppAd`
- `RF/RM` 在 moving gait 下为何会被允许一起收缩到错误 world height

而不是继续怀疑 Isaac drive tracking 或 bridge/runtime 执行层

我随后又做了两轮更聚焦的 reference 修复：

- `phase12_observed_joint_target_20260328_083340_*`
- `phase13_observed_joint_target_full_height_sequence_20260328_083545_*`

Phase 12 修复内容：

- `HexbotCmdVelTargetNode` 在 moving command 下不再把 future target joints
  强行拉回 `defaultJointState_`
- 改为复用 live observation 的 joint posture 作为 moving target 的 joint seed

Phase 12 结果非常明确：

- `measured_vs_target_joint_max_abs_delta_mean_rad`
  - `0.522 -> 0.0514`
- `optimized_vs_target_joint_max_abs_delta_mean_rad`
  - `0.506 -> 0.1348`
- `optimized_vs_target_foot_max_position_error_mean_m`
  - `0.0365 -> 0.0202`
- `optimized_minus_target_support_height_mean_m`
  - `-0.0267 -> -0.00684`

这说明先前 moving target 确实存在一个核心内部冲突：

- base 在前推
- joints 却被固定回静态 `defaultJointState_`
- stance / transition 腿因此在 target 里被迫跟着机身一起走

Phase 13 继续修复：

- `SwitchedModelReferenceManager::modifyReferences()` 不再只给当前 phase
  一点点 `liftOffHeightSequence`
- 改为按整段 `ModeSchedule` 的 phase start / phase end 参考足端高度
  填满 `liftOffHeightSequence` 与 `touchDownHeightSequence`

Phase 13 结果继续收敛：

- `optimized_vs_target_foot_max_position_error_mean_m`
  - `0.0202 -> 0.0162`
- `current_vs_target_foot_max_position_error_mean_m`
  - `0.0212 -> 0.0177`
- `optimized_minus_target_support_height_mean_m`
  - `-0.00684 -> -0.00480`
- `current_minus_target_support_height_mean_m`
  - `-0.00618 -> -0.00425`
- `posture.base_drop_m`
  - `0.02236 -> 0.01966`

同时 precomputation 侧的重要变化是：

- `phase12` 还残留 `RF/RM/LF` swing clearance gap
- `phase13` 里这些 swing clearance 问题基本被压掉
- 剩余最显著的问题已经收缩成 all-contact / transition 期间的受力偏载
  - 尤其 `RM` 法向载荷显著高于其它腿

因此当前主线结论应更新为：

- `phase12` 的 moving target joint-seed 修复应保留
- `phase13` 的 full height sequence 修复也应保留
- 剩余主阻塞已经不再是大范围 target / swing reference 撕裂
- 当前更像是 transition / all-contact 阶段的 load distribution 仍偏载
- 机器人姿态更稳了，但净推进仍没有实质恢复

随后我又沿着 all-contact / transition 偏载继续做了三轮：

- `phase14_periodic_terminal_gait_20260328_084059_*`
- `phase14b_deferred_terminal_stance_20260328_084325_*`
- `phase15_immediate_gait_insertion_20260328_084525_*`

`phase14` 是一次失败实验：

- 我直接把 `GaitSchedule` 的终端 phase 从 `STANCE` 改成了周期模板的下一相
- 结果 swing planner 在最后一段 swing 上失去 touchdown 定义
- container log 直接报：
  - `The time of touch-down for the last swing of the EE with ID 1 is not defined.`

因此这一条结论已经明确：

- 当前 swing planner 仍依赖一个 far-future touchdown 终点
- 不能直接删掉终端 `STANCE`

`phase14b` 则是这轮最有效的主线修复：

- 保留终端 `STANCE`
- 但把 `SwitchedModelReferenceManager` 请求的 future padding 从 `+1 horizon`
  扩到 `+2 horizons`
- 这样 terminal `STANCE` 仍存在，但被推到 solver 近端 horizon 之外

`phase14b` 的结果是这轮最干净的一次：

- `posture.base_drop_m`
  - `0.0197 -> 0.00754`
- `perf_eq_constraints_sse`
  - `0.000376 -> 0.000000`
- `perf_ineq_constraints_sse`
  - `1.709644 -> 0.000000`
- `precomputation.problematic_block_count`
  - `1 -> 0`
- `precomputation.max_stance_force_ratio`
  - `2.809 -> clean / no problematic block`

也就是说：

- all-contact / transition 偏载已经被实质压掉
- 躯干下沉进一步减轻
- contact feasibility 进入当前最干净的状态

但 `phase14b` 还没有恢复净推进：

- `delta_forward_m ≈ 0.00244 / 8 s`

我随后又试了 `phase15`：

- 把 `GaitReceiver` 从“after `finalTime`”改成“当前 horizon 立即插入 gait”

这条路没有变成正向修复，现已撤回，不留在主线：

- `delta_forward_m` 进一步降到约 `0.00167 / 8 s`
- `reset_request_sent` 从 `1` 升到 `2`
- 虽然接触依旧干净
  - 但 moving gait 没有更好地转成净推进

所以当前最佳主线应更新为：

- 保留 `phase12`
- 保留 `phase13`
- 保留 `phase14b`
- 不保留 `phase14`
- 不保留 `phase15`

当前最准确的剩余主阻塞表述应改成：

- 大块 reference mismatch 已修
- all-contact / transition force bias 已基本压掉
- 剩余更像是“moving intent 已成立，但推进仍偏弱”
- 下一步应继续查 gait insertion 与 target trajectory 的精确时序对齐，而不是回头做外层调参

随后我又做了两轮更细的 gait insertion 时序实验：

- `phase16_target_aligned_gait_insertion_20260328_085646_*`
- `phase17_target_window_start_gait_insertion_20260328_085844_*`

这两轮的共同点是：

- 不再盲目用 `finalTime` 或 `initTime`
- 改成尝试按 active `TargetTrajectories` 的时间窗去插 gait

结果说明这条线已经被进一步诊断清楚了：

- 它确实能把 moving mode 更早带回 solver
  - `phase16`：`mode_counts = {'21': 1, '63': 3}`
  - `phase17`：`mode_counts = {'21': 1, '42': 1, '63': 3}`
- 也确实让表面前进量略有增加
  - `delta_forward_m`
    - `phase14b: 0.00244`
    - `phase16: 0.00353`
    - `phase17: 0.00387`

但它没有把系统带回“更稳定的闭环”，反而把姿态和接触重新拉差：

- `posture.base_drop_m`
  - `0.00754 -> 0.02301 -> 0.02760`
- `precomputation.problematic_block_count`
  - `0 -> 2 -> 3`
- `precomputation.max_stance_force_ratio`
  - `clean -> 1.612 -> 1.891`
- `optimized_vs_target_foot_max_position_error_mean_m`
  - `0.00754 -> 0.02185 -> 0.02366`

因此这两轮都不能留在主线：

- `phase16`
  - 虽然比 `phase14b` 更容易进入 moving mode
  - 但 `RF` contact / foot-world 偏差和 trunk drop 明显回升
- `phase17`
  - 进一步把 `42` 也带回 solver
  - 但 posture / contact regression 更明显，`max_still_sec` 甚至升到约 `8.07s`

所以当前主线再次明确为：

- 保留 `phase12`
- 保留 `phase13`
- 保留 `phase14b`
- 不保留 `phase16`
- 不保留 `phase17`

当前最准确的更新结论是：

- gait timing 本身确实影响“能否进入 moving mode”
- 但单独把 gait 向 target 时间窗对齐，仍不能恢复稳定推进
- 若后续继续碰这条线，必须和 contact/load 保护联动验证，而不是单独留下

我随后又做了 `phase18_guarded_immediate_gait_insertion_20260328_090414_*`，
尝试把这条思路再做得更保守：

- 默认仍保持 `phase14b`
- 只有当当前状态已经足够贴近 active target 时
  - 才允许把 gait 从 `finalTime` 提前到 `initTime`

运行日志确认这轮确实发生了“先 defer，再 guarded immediate”的混合行为：

- 第一段：
  - `Deferring gait update to finalTime because state is not yet close to target...`
- 后一段：
  - `Setting new gait at initTime 0.7000 with guarded target tracking gate`

但结果并没有变成主线修复，反而更差：

- `delta_forward_m`
  - `0.00244 -> 0.000115`
- `delta_lateral_m`
  - 直接恶化到约 `-0.0214 m`
- `reset_request_sent`
  - `1 -> 2`
- `reject_backend_joint_command`
  - `0 -> 1`
- `precomputation.problematic_block_count`
  - `0 -> 4`
- `optimized_vs_target_joint_max_abs_delta_mean_rad`
  - `0.194 -> 0.264`

所以这轮也应明确撤回，不留在主线：

- `phase18` 证明了
  - “带简单状态门槛的即时 gait insertion”
  - 仍不足以恢复稳定 walking

因此当前主线最终再次收紧为：

- 保留 `phase12`
- 保留 `phase13`
- 保留 `phase14b`
- 不保留 `phase16`
- 不保留 `phase17`
- 不保留 `phase18`

这也把下一步方向进一步压实了：

- 先不要再继续碰 gait insertion timing
- 下一步应回到 moving mode 内部的 branch / load allocation / contact
  演化路径本体

补充入口：

- 更完整的底层原理链、根因定位和分阶段修复计划
  - `ocs2_hexbot_forward_progress_root_cause_20260328.md`

我随后开始把 `hexbot_locomotion_v2` 的 tripod foothold 逻辑以最小形式移植进
OCS2 主线，先后做了两轮：

- `phase22_v2_foothold_swing_tracking_20260328_094707_*`
  - 新增 swing 腿 `x/y` foothold 端点与速度跟踪
- `phase23_v2_foothold_tracking_gain_20260328_094905_*`
  - 在同一条 swing-foot 轨迹链上补充温和的位置跟踪增益

这两轮都没有破坏当前主线稳定性门槛：

- `trunk_sink_detected = false`
- `weird_pose_detected = false`
- `backend_vs_runtime_joint_command.max_abs_diff_rad = 0.0`

但它们带来了真实、可量化的推进改善：

- `delta_forward_m`
  - `phase14b: 0.00244`
  - `phase21b: 0.00299`
  - `phase22: 0.00388`
  - `phase23: 0.00401`
- `max_still_sec`
  - `phase14b: 5.73`
  - `phase21b: 8.00`
  - `phase22: 7.57`
  - `phase23: 3.83`

因此当前结论更新为：

- `v2` foothold 逻辑移植方向是正确的
- OCS2 现在已经不再只是“稳稳站住但几乎完全不迈腿”
- 但 `0.00401 m / 8s` 仍远低于 walking 恢复标准，当前还不能称为“稳定前进”

同时，`phase23` 相比 `phase22` 更适合作为新的实验主线：

- `delta_forward_m`
  - `0.00388 -> 0.00401`
- `max_still_sec`
  - `7.57 -> 3.83`
- `base_drop_m`
  - `0.02636 -> 0.02437`
- `perf_ineq_constraints_sse`
  - `0.026017 -> 0.000000`

所以这轮之后的主线应更新为：

- 保留 `phase12`
- 保留 `phase13`
- 保留 `phase14b`
- 保留 `phase23`
- 不保留 `phase16`
- 不保留 `phase17`
- 不保留 `phase18`

新的最准确说法是：

- gait timing 本身不是主阻塞
- `v2` 风格 foothold/swing tracking 已经证明能把系统从“近乎静止”
  往“有迈腿意图且 still time 显著缩短”推进
- 下一步应该继续沿着这条 OCS2 swing-foot foothold 主线细化 touchdown
  位置、gain 和 phase-consistency，而不是回头做外围调参

随后我又验证了一轮 `phase24b_v2_touchdown_consistency_warm_20260328_095540_*`
和一轮回退确认 `phase23b_restore_planar_velocity_20260328_095850_*`：

- `phase24b`
  - 尝试让 planar swing spline 在 lift-off / touchdown 端点都回到零速度
  - 同时把 touchdown 的 terrain frame 计算全面切到 `endSupportPlane`
- `phase23b`
  - 撤回上面两点，恢复到 `phase23` 风格的非零 planar 端点速度和旧 frame

结果非常直接：

- `phase24b` 明显退化回“几乎静止”
  - `delta_forward_m: 0.00401 -> 0.00226`
  - `max_still_sec: 3.83 -> 8.03`
  - `base_drop_m: 0.02437 -> 0.00531`
- 回退后的 `phase23b` 恢复了“有一点迈腿”的状态
  - `delta_forward_m = 0.00377`
  - `max_still_sec = 6.40`
  - `base_drop_m = 0.02238`
  - `mode_counts = {63: 2, 21: 1, 42: 1}`

因此当前主线决策再补充一条：

- 不保留 `phase24b`
- 保留 `phase23` 风格 planar swing 端点速度

这也修正了对现场现象的表述：

- 用户肉眼看到“机器人基本不迈腿”是对的
- `phase23/phase23b` 只是把系统从“完全近似静止”推进到“有轻微迈腿/轻微位移”
- 距离“真正稳定前进”仍然有明显差距

随后我又连续验证了两条更激进的整步补偿：

- `phase25_v2_rear_front_swing_anchor_20260328_100359_*`
  - 直接把 swing 锚点改成 `rear -> front`
- `phase26_touchdown_full_stride_compensation_20260328_100551_*`
  - 保留当前 lift-off
  - 只把 touchdown 前推到“整步跨度”补偿

结果都不值得留在主线：

- `phase25` 明显失败
  - `delta_forward_m = 0.00135`
  - `max_still_sec = 8.07`
  - `runtime_observations_stale = 1`
- `phase26` 虽然比 `phase25` 好，但仍没有超过 `phase23`
  - `delta_forward_m = 0.00385 < 0.00401`
  - `max_still_sec = 6.27 > 3.83`
  - `perf_ineq_constraints_sse = 0.936955 > 0`
  - `base_drop_m = 0.02754 > 0.02437`

因此当前最稳妥的主线决策维持为：

- 保留 `phase23`
- 不保留 `phase25`
- 不保留 `phase26`

新的更准确结论是：

- 直接把 `v2` 的整步幅度硬塞进当前 OCS2 swing reference，会重新伤到可行性
- 当前系统仍然缺的是“既能放大步幅，又不破坏 contact feasibility”的
  body-target / foothold co-design

随后我把实验方向转到 `body target` 本体，验证“持续前推的多段 target”
是否能和现有 foothold 形成协同：

- `phase27b_multi_knot_body_target_warm_20260328_101254_*`
- `phase27c_multi_knot_body_target_repeat_20260328_101359_*`
- `phase28_half_cycle_body_target_knots_20260328_101600_*`

结论是：

- `phase27` 系列方向有一定启发，但还不能留在主线
  - `phase27b`
    - `delta_forward_m = 0.00471`
    - `base_drop_m = 0.01193`
    - `perf_ineq_constraints_sse = 0.0`
  - `phase27c`
    - `delta_forward_m = 0.00544`
    - `perf_ineq_constraints_sse = 0.0`
  - 但两轮 `max_still_sec` 都仍在 `8s` 级别
  - 说明它更像“增大净位移”，而不是“恢复连续、肉眼可见的迈腿”
- `phase28` 把 body target knot 加密到 `0.35s` 半周期后更差
  - `delta_forward_m = 0.00376`
  - `max_still_sec = 7.33`
  - `perf_ineq_constraints_sse = 0.169268`
  - `base_drop_m = 0.02943`

因此这轮之后的主线决策保持谨慎：

- 不保留 `phase27`
- 不保留 `phase28`
- 当前代码已回退到 `phase23` 风格主线

这也让问题更聚焦了：

- 单独增强 `body target` 连续前推，确实能抬高净位移
- 但如果没有同步修好 touchdown / stance 过渡，机器人仍会表现成
  “整体略有前挪，但视觉上基本不走”

随后我又沿着 `touchdown -> stance` 的世界锚点一致性做了两步定点实验：

- `phase30_stance_anchor_constraint_20260328_103655_*`
  - 在 `hexbot_runtime_mrt` solver trace 里新增了
    `target/current/optimized vs touchdown anchor`
    三组 stance 漂移指标
  - 并尝试在 `LeggedRobotPreComputation::eeZeroVelConConfig()`
    里把 stance 腿的 `x/y/z` 锚定到
    `SwingTrajectoryPlanner` 已有的恒定 touchdown 世界坐标
- `phase30c_stance_anchor_constraint_warm2_20260328_103848_*`
  - 在 CppAD 运行时动态库完成重编后做的真正 warm rerun

这轮拿到的关键结论非常重要：

- 新增 trace 已经直接证明：
  当前问题不只是 `optimized/current` 偏离 target，
  而是 `target` 自己在 stance 阶段也会偏离刚刚落下来的 touchdown 世界锚点
  - `target_vs_touchdown_anchor_stance_max_position_error_max_m = 0.03811`
  - `optimized_vs_touchdown_anchor_stance_max_position_error_max_m = 0.038181`
  - `current_vs_touchdown_anchor_stance_max_position_error_max_m = 0.042582`
- 也就是说，
  `touchdown placement -> subsequent stance-world anchoring`
  这条链目前在 reference/constraint 协同层仍然是破的
- 但“只在 zero-velocity constraint 里硬补 stance x/y 锚定”不能作为主线修复
  - warm rerun 只有
    `delta_forward_m = 0.00370`
  - `max_still_sec` 回到 `8.00`
  - `perf_ineq_constraints_sse = 5.298747`
  - 比 `phase23` 更差

因此这轮之后的主线决策应再补充两条：

- 保留 `phase30` 的 touchdown-anchor 结构化观测接口
- 不保留 `phase30` 的实验性 stance x/y 硬锚定约束

新的最准确表述是：

- 主阻塞已经进一步收紧成
  `reference touchdown anchor drift + stance anchoring inconsistency`
- 但修复位置不该只放在 `ZeroVelocityConstraintCppAd`
  这一层硬补
- 当前代码已回退到更安全的 `phase23` 主线，并只保留新的诊断能力

随后我验证了一个更保守的方向：
只在 `SwitchedModelReferenceManager` 的 reference 读取侧尝试传递
stance-world anchor，而不再动 `SwingTrajectoryPlanner` / `support plane`
主生成链。

- 首轮结构补丁：
  - `phase31_reference_stance_anchor_evolution_20260328_105123_*`
- warm rerun：
  - `phase31b_reference_stance_anchor_evolution_warm_20260328_105320_*`
- 缩小侵入范围后的 read-side-only 版本：
  - `phase31c_reference_stance_anchor_readside_only_20260328_105622_*`

这轮结论是：

- 第一版和 warm 版都不适合当有效证据
  - moving template 已到达，但很快出现
    `runtime_observations_stale = 1`
  - solver trace 只剩 `1` 个有效样本
  - 表明“把 stance anchor 混进 `modifyReferences()` 主生成链”
    的侵入式修法会破坏当前稳定主线
- `phase31c` 把系统拉回了“可完整运行”的状态
  - `runtime_observations_stale = 0`
  - `moving_cycle_detected = true`
  - `perf_ineq_constraints_sse = 0.0`
  - `delta_forward_m = 0.003925`
- 但它并没有超过 `phase23`
  - `phase23 delta_forward_m = 0.004015`
  - `phase23 max_still_sec = 3.83`
  - `phase31c max_still_sec = 8.03`
- 更关键的是，touchdown-anchor 漂移并没有被真正压下去
  - `target_vs_touchdown_anchor_stance_max_position_error_max_m = 0.03608`
  - `current_vs_touchdown_anchor_stance_max_position_error_max_m = 0.045203`
  - `optimized_vs_touchdown_anchor_stance_max_position_error_max_m = 0.043521`

因此当前主线结论再补一条：

- 单纯在 `ReferenceManager` 的 read-side 挂 stance anchor 仍然不够
- 真正缺的是“让 target state / joint target 本身在 stance 阶段保持
  touchdown-consistent”的更深层参考生成修复
- 这轮试验代码已经撤回，代码主线重新回到 `phase23` 基线

随后我直接验证了这条更深一层的修复猜想：
在 `HexbotCmdVelTargetNode` 里，对 moving 期间的 partial-stance 模式
增加 Jacobian 级 `stance-aware goal joint target` 补偿，
尝试在 body 前推时，让支撑腿 joint target 主动抵消足端世界坐标漂移。

- 证据：
  - `phase32_stance_aware_goal_joint_target_20260328_110616_*`

结果是否定的：

- `delta_forward_m = 0.003725`
  - 低于 `phase23 = 0.004015`
- `max_still_sec = 8.03`
  - 明显差于 `phase23 = 3.83`
- `base_drop_m = 0.02883`
  - 高于 `phase23 = 0.02437`
- solver 也不再是完全干净
  - `perf_eq_constraints_sse = 0.000200`
  - `perf_ineq_constraints_sse = 0.000061`

更重要的是，touchdown-anchor 漂移并没有被真正压下去：

- `target_vs_touchdown_anchor_stance_max_position_error_max_m = 0.038136`
- `current_vs_touchdown_anchor_stance_max_position_error_max_m = 0.042624`
- `optimized_vs_touchdown_anchor_stance_max_position_error_max_m = 0.040893`

因此这轮也不能留主线。最新主线决策更新为：

- 否定“只靠 `HexbotCmdVelTargetNode` 的 Jacobian 级支撑腿补偿，
  就能恢复真正 stepping”的假设
- 当前代码已撤回这段试验性实现，并重新编译回 `phase23` 主线

随后我实现了更深一层的版本：在 `HexbotCmdVelTargetNode.cpp`
里，把 moving target 扩成 `4` 个 knot，并在每个 knot 上做
接触一致的完整目标状态重建 + 全身 IK。

- 冷启动 `phase33_contact_consistent_full_target_state_ik_*`
  - 不作为算法证据
  - 首次回归被容器里的 CppAD 动态库冷编译吃掉了主要 probe 窗口
- 热缓存 `phase33b_contact_consistent_full_target_state_ik_warm_*`
  - `delta_forward_m = 0.004815`
  - `max_still_sec = 3.73`
  - `base_drop_m = 0.02635`
  - `target state_count = 4`
  - `moving_cycle_detected = true`
  - 相比 `phase23`
    - 前进量更高：`0.004815 > 0.004015`
    - 静止时长略降：`3.73 < 3.83`
  - 但它仍不等于“真正稳定行走”
    - `target_vs_touchdown_anchor_stance_max_position_error_max_m = 0.040372`
    - `perf_ineq_constraints_sse = 0.016051`

因此主线先从 `phase23` 推进到 `phase33b`，
但只把它定义为“更好的完整目标状态主线”，
还不能定义为 walking 已恢复。

我又追了一轮 `phase34_latched_stance_anchor_full_target_ik_*`，
把 stance anchor 做成跨 publish 周期记忆：

- 好处：
  - `target_vs_touchdown_anchor_stance_max_position_error_mean_m`
    从 `0.02255` 压到 `0.01639`
  - `target_vs_touchdown_anchor_stance_max_position_error_max_m`
    从 `0.040372` 压到 `0.035945`
  - `base_drop_m` 回到 `0.02438`
  - `perf_ineq_constraints_sse = 0.0`
- 坏处：
  - `delta_forward_m` 回落到 `0.003918`
  - `max_still_sec` 恶化回 `8.03`

所以 `phase34` 不保留，源码和安装产物都已经回退并重编回
`phase33b` 版本。
