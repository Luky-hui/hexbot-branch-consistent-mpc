# 六足机器人运动控制包说明

`hexbot_locomotion_v2` 是当前工程中正在使用的六足运动控制包，也是当前唯一保留的主动运动控制实现。

## 包的职责

- 订阅 `/cmd_vel`、`/joint_states`、`/imu`
- 发布 `/joint_command`
- 生成当前导航系统使用的 tripod（三脚架）步态
- 维护运行时的 `odom -> base_link -> imu_link` 关系

## 当前关键文件

- 当前 walking 推荐启动：`scripts/launch_basic_walk_loaded_tuned.sh`
- 当前 walking 推荐站姿配置：`config/hexbot_v2_stage2_loaded_equilibrium_candidate.yaml`
- 更保守的恢复配置：`config/hexbot_v2_basic_walk_loaded_safe.yaml`
- 历史隐式基线参考：`config/hexbot_v2.yaml`
- 启动文件：`launch/hexbot_locomotion_v2.launch.py`
- 主节点：`src/hexbot_locomotion_v2_node.py`

## 默认启动方式

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py use_sim_time:=true mode:=tripod
```

当前默认启动已经切换到：

- `config/hexbot_v2_stage2_explicit_current_candidate.yaml`

## 2026-03-27 基础步态验收基线

在当前 Isaac Sim 运行态里，机器人初始朝向并不保证对齐 `odom` 世界系 `+x`。因此基础步态验收必须使用“机体系前向/横向位移”，不能只看世界系 `delta_x/delta_y`。

推荐启动入口：

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/scripts/launch_basic_walk_loaded_tuned.sh
```

推荐 smoke 入口：

```bash
bash /home/u-zhuang/ros2_ws/src/ocs2_hexbot_ws/src/hexbot_locomotion_v2/scripts/run_basic_gait_smoke.sh
```

2026-03-27 这轮已实际验证到：

- clean reload 后，站姿恢复可在 `0.10s` 左右进入 `best_max_joint_diff_rad=0.0712`
- `5s @ linear.x=0.03` 时，`delta_forward_m=0.1118`，`delta_lateral_m=-0.0022`
- `5s @ linear.x=0.05` 时，`delta_forward_m=0.2010`，`delta_lateral_m=-0.0051`
- `5s @ angular.z=0.12` 时，`delta_yaw_rad=0.4544`
- `10s @ linear.x=0.03, angular.z=0.12` 时，`delta_forward_m=0.1884`，`delta_lateral_m=0.1418`，`delta_yaw_rad=1.0356`
- 同一 mixed case 的 `arc_lateral_excess_m=0.0345`，也就是超出理想圆弧的额外侧滑约 `3.5 cm`
- mixed case 结束后仍可在 `0.12 rad` 阈值内回站，`best_max_joint_diff_rad=0.1064`

当前结论：

- 基础 walking 已可在 `launch_basic_walk_loaded_tuned.sh` 下稳定复现
- `run_basic_gait_smoke.sh` 已改为输出 `delta_forward_m` / `delta_lateral_m`
- `odom_ab_runner.py` 现在还会输出 `arc_expected_lateral_m` 和 `arc_lateral_excess_m`，弧线测试不要再把全部横向位移都误判为“侧滑”
- 在没有重新放宽速度上限前，不要把这轮结果直接等价为“running 已验收”

## 速度缩放

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py use_sim_time:=true mode:=tripod speed_scale:=2.0
```

## 显式参数覆盖

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
  use_sim_time:=true \
  mode:=tripod \
  tripod_max_linear_x:=0.30 \
  tripod_max_angular_z:=1.20 \
  tripod_forward_stride_m:=0.14 \
  tripod_turn_stride_m:=0.14
```

## 试验配置切换

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
  use_sim_time:=true \
  mode:=tripod \
  geometry_config_path:=/绝对路径/hexbot_v2_experiment.yaml
```

## Fresh-Start 验收

主入口：

```bash
bash scripts/run_single_ab_case.sh --preset baseline_fwd_0p10
```

这个入口的设计目标是：

- 每个 case 都从全新的 locomotion 实例开始
- 不把 gait 结果和“停下后回站姿”的恢复问题混在一起
- 专门用于架构与步态 A/B 对比

当前标准验证流程如下：

```bash
bash scripts/run_single_ab_case.sh --preset baseline_fwd_0p10
bash scripts/run_single_ab_case.sh --preset baseline_fwd_0p15
bash scripts/run_single_ab_case.sh --preset baseline_turn_0p35

bash scripts/run_single_ab_case.sh \
  --preset stage1_fwd_0p10 \
  --stage1-cfg /绝对路径/hexbot_v2_stage2_explicit_current_candidate.yaml

bash scripts/run_single_ab_case.sh \
  --preset stage1_fwd_0p15 \
  --stage1-cfg /绝对路径/hexbot_v2_stage2_explicit_current_candidate.yaml

bash scripts/run_single_ab_case.sh \
  --preset stage1_turn_0p35 \
  --stage1-cfg /绝对路径/hexbot_v2_stage2_explicit_current_candidate.yaml
```

## 当前结论

- `config/hexbot_v2_stage2_explicit_current_candidate.yaml` 已通过导航耦合验收，现已成为默认运行基线
- 已淘汰的显式基线、中间 stance、`xy20` probe 等配置已从活跃目录移出
- 原始 gait 与 navigation 测试结果已归档，不再放在当前运行目录内

详细结论见：

- `ARCHITECT_HANDOFF_2026-03-25.md`

## 导航耦合验收

当前被批准的候选配置启动方式：

```bash
ros2 launch hexbot_locomotion_v2 hexbot_locomotion_v2.launch.py \
  use_sim_time:=true \
  mode:=tripod \
  locomotion_startup_mode:=fixed_stand \
  geometry_config_path:=/绝对路径/hexbot_v2_stage2_explicit_current_candidate.yaml
```

单次导航验收入口：

```bash
bash scripts/run_single_nav_case.sh --preset nav_fwd_0p5
```

三组标准导航验收：

```bash
bash scripts/run_single_nav_case.sh --preset nav_fwd_0p5
bash scripts/run_single_nav_case.sh --preset nav_fwd_1p0
bash scripts/run_single_nav_case.sh --preset nav_turn15_fwd_0p5
```

导航耦合验收结果：

| 预设 | 是否成功 | 是否绊腿/拖腿 | 是否停顿超过 2 秒 | 终点距离误差 m | 横向误差 m | 末端偏航误差 rad |
| --- | --- | --- | --- | --- | --- | --- |
| `nav_fwd_0p5` | 是 | 否 | 否 | 0.0474 | -0.0086 | 0.0947 |
| `nav_fwd_1p0` | 是 | 否 | 否 | 0.0218 | -0.0124 | 0.0988 |
| `nav_turn15_fwd_0p5` | 是 | 否 | 否 | 0.0367 | -0.0046 | 0.0942 |

## 使用注意

- `/joint_command` 始终只能有一个有效发布者
- 当前已支持显式 `stance` 配置，启动、恢复、步态 home 都可以从显式落脚点求解
- Nav2 的速度参数应与 locomotion 的速度参数联动调整
- 更完整的运行基线说明见根目录 `hexbot_navigation_runtime_baseline.md`
- 后续对 `hexapod_ros` 与 `OpenSHC` 的复现准备见根目录 `hexapod_reference_parameter_comparison.md`
