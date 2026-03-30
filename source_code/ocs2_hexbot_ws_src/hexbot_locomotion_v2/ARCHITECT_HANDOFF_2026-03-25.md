# 六足机器人运动控制架构交接摘要

日期：2026-03-25

## 本文目的

本文用于说明：

- 本轮显式 stance 迁移工作的目标
- fresh-start A/B 测试如何执行
- 当前被保留的显式运行基线是什么
- 导航耦合验收是否通过

本轮已经不再讨论“隐式还是显式架构”这个问题。该问题已经关闭。剩余问题只有一个：

- 在显式 stance 方案中，哪一个脚端落点模型适合作为下一阶段默认基线

## 验收协议

主入口：

```bash
bash scripts/run_single_ab_case.sh --preset baseline_fwd_0p10
```

本轮刻意不再把多 case 连续运行作为主验收，因为旧的多 case 串行流程会把 gait 表现和 stop-to-stand 恢复行为混在一起。

fresh-start 验收规则：

- 每个 case 前先清理 ROS2 与 Python 控制器进程
- 每个 case 前重置 Isaac Sim
- 每个 case 只启动一个全新的 locomotion 实例
- 启动方式固定使用 `locomotion_startup_mode=fixed_stand`
- 在真正开始跑 10 秒 odom 统计之前，必须先通过 `/joint_states` 的预备收敛门控

主脚本：

- `scripts/run_single_ab_case.sh`
- `scripts/wait_joint_state_convergence.py`

## 已关闭结论

1. 显式 stance 语义已经验证完成。  
   启动、恢复、站姿和 tripod gait 的 home 都可以从同一套显式落脚点求解，不再依赖隐式中立关节位姿加手工偏移。

2. 显式零回归控制组已经通过。  
   它证明显式架构本身成立，架构问题已经关闭。

3. 更宽的 mid stance 不能作为正式运行基线。  
   在三组 fresh-start case 中，它都无法通过 pre-run settle。

4. 当前推荐候选是 `xy15` 提升后的正式版本。  
   即：`config/hexbot_v2_stage2_explicit_current_candidate.yaml`

5. `xy20` 只是边界参考，不是正式基线。  
   它虽然还能通过，但收敛余量更小，而且没有形成明确的动态优势。

## 结果汇总

### 基线控制组

| 配置 | 指令 | 是否绊腿/拖腿 | 是否停顿超过 2 秒 | 前进/转向结果 |
| --- | --- | --- | --- | --- |
| baseline | `vx=0.10` 10s | 否 | 否 | `delta_x=0.4230` |
| baseline | `vx=0.15` 10s | 否 | 否 | `delta_x=0.5789` |
| baseline | `wz=0.35` 10s | 否 | 否 | `delta_yaw=1.6677` |

### 当前显式候选

| 配置 | 指令 | 是否绊腿/拖腿 | 是否停顿超过 2 秒 | 前进/转向结果 |
| --- | --- | --- | --- | --- |
| current candidate | `vx=0.10` 10s | 否 | 否 | `delta_x=0.4991` |
| current candidate | `vx=0.15` 10s | 否 | 否 | `delta_x=0.5715` |
| current candidate | `wz=0.35` 10s | 否 | 否 | `delta_yaw=2.1157` |

### 边界参考

| 配置 | 指令 | 是否绊腿/拖腿 | 是否停顿超过 2 秒 | 前进/转向结果 |
| --- | --- | --- | --- | --- |
| xy20 probe | `vx=0.10` 10s | 否 | 否 | `delta_x=0.4988` |
| xy20 probe | `vx=0.15` 10s | 否 | 否 | `delta_x=0.5257` |
| xy20 probe | `wz=0.35` 10s | 否 | 否 | `delta_yaw=2.1003` |

## 推荐结论

推荐作为当前默认运行基线的配置：

- `config/hexbot_v2_stage2_explicit_current_candidate.yaml`

理由：

- 能稳定通过 fresh-start settle
- 没有引入绊腿、拖腿、长时间停顿等回归
- 在关键测试中相对旧基线表现更优或至少持平
- 相比 `xy20` 更保守、更安全
- 避开了更宽 mid stance 的启动失败问题

## 导航耦合验收

完成 fresh-start gait 验收后，又使用导航耦合流程进行了复验。该流程包含：

- 每个 case 都重新 fresh-start 启动 locomotion
- 固定使用 `locomotion_startup_mode=fixed_stand`
- 在发出导航目标前核对 Nav2 真实容差参数
- 将旧的 20 cm 轮式提前停车逻辑收紧为真正适合六足机器人的容差

验收结果：

| 预设 | 是否成功 | 是否绊腿/拖腿 | 是否停顿超过 2 秒 | 终点距离误差 m | 横向误差 m | 末端偏航误差 rad |
| --- | --- | --- | --- | --- | --- | --- |
| `nav_fwd_0p5` | 是 | 否 | 否 | 0.0474 | -0.0086 | 0.0947 |
| `nav_fwd_1p0` | 是 | 否 | 否 | 0.0218 | -0.0124 | 0.0988 |
| `nav_turn15_fwd_0p5` | 是 | 否 | 否 | 0.0367 | -0.0046 | 0.0942 |

## 当前状态

可以明确说明：

- 显式 stance 已经成为正式方向
- 当前推荐基线是 `current_candidate`
- 该配置已经通过导航耦合验收
- `xy20` 只保留为边界参考，不再继续扩张

## 关键文件

- `README.md`
- `config/hexbot_v2_stage2_explicit_current_candidate.yaml`
- `scripts/run_single_nav_case.sh`
- `scripts/nav_goal_acceptance_runner.py`
- `scripts/run_single_ab_case.sh`
- `scripts/wait_joint_state_convergence.py`
