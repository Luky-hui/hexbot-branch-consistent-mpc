# 六足参考参数对比

## 当前优先级说明

本文仍有价值，但它现在只服务于后续 gait / 参数组织优化，不是当前第一主线。

当前第一主线已经改为：

- 固化 OCS2 六足主闭环
- 形成旧 baseline vs 新主线的 A/B 证据
- 围绕跨 Humble/Jazzy、时间归一化、桥接闭环形成专利材料

因此，本文中的参数对比不再作为“下一步立即实施清单”，而是作为闭环成品化完成后的第二阶段优化输入。

同时需要补充一个最新现实约束：

- 到 2026-03-27 当前轮次为止，Hexbot 的第一前置阻塞已经不只是 gait 参数，而是 Isaac reimport 资产本身
- `hexbot_isaac_rooted_reimport.usd` 已被确认可能带坏的 `xform/state`，会让机器人在没有控制时也直接趴地
- 因此，本文所有参数对比在近期都应被视为“后续输入”，而不是解决当前趴地问题的第一抓手

## 文档目的

本文用于比较当前 V2 运行基线与两个主要参考对象：

- `hexapod_ros`
- `OpenSHC`

目的不是立即替换现有工程，而是明确下一步最值得借鉴的内容。

## 主要对比对象

- 历史 V2 正式运行基线：  
  `hexbot_locomotion_v2/config/hexbot_v2_stage2_explicit_current_candidate.yaml`
- 当前 gait 稳定 walking 入口：  
  `hexbot_locomotion_v2/scripts/launch_basic_walk_loaded_tuned.sh`
- 当前 OCS2 专利主线：  
  `hexbot_locomotion_mpc + ocs2_hexbot_legged_robot_ros`
- 历史隐式参考：  
  `hexbot_locomotion_v2/config/hexbot_v2.yaml`
- 当前机器人几何：  
  `hexbot_description_ros2/urdf/hexbot_isaac_rooted.urdf`

## 高价值结论

| 对比项 | 当前 V2 | `hexapod_ros` | `OpenSHC` | 工程意义 |
| --- | --- | --- | --- | --- |
| 机体几何尺度 | 与当前实机一致 | 与 PhantomX 类六足较接近 | 示例平台更偏架构参考 | 几何上优先参考 `hexapod_ros` |
| 站立高度语义 | 仍偏隐式 | 有明确站立高度参数 | 有明确 `body_clearance` | 当前 V2 应继续显式化站立高度 |
| 初始 stance | 较保守 | 更宽 | 有逐腿 `stance_position` | 当前机器人需要更清晰的显式 stance 模型 |
| 抬腿高度 | 当前为 `0.03` 左右 | 常见为 `0.04` | 默认更灵活 | 下一轮可向 `0.04` 试探 |
| 步态时序 | 当前较紧凑 | 时序更显式 | `step_frequency` 为一级参数 | 未来应加强参数显式化 |
| 前进速度 | 当前已经不低 | 更保守 | 多为缩放式 | 当前问题重点不是速度上限，而是稳定性 |
| 转向能力 | 当前已经较激进 | 更保守 | 控制更细 | 当前问题重点是转向稳定性与一致性 |
| IMU 稳定化 | 走行中使用偏弱 | 自调平更积极 | 有清晰 IMU/body posing 结构 | 当前 V2 应更好利用 IMU |
| 参数组织 | 仍偏混合 | 单体式 | 分层更清晰 | `OpenSHC` 更适合作为结构参考 |

## 核心判断

1. `hexapod_ros` 更适合拿来做第一轮 stance 与稳定性复现，因为它和当前机器人几何最接近。
2. 当前 V2 的最大短板不是速度不够，而是这些参数优化本身还不构成当前最值得主张的发明点。
3. `OpenSHC` 更适合作为“参数组织方式”和“控制器架构分层”的参考，而不是直接作为几何模板。
4. 在专利主线阶段，真正更有价值的是  
   `跨 ROS2 发行版环境 + 六足 OCS2 适配 + bridge 闭环 + 时间归一化`
5. 在进入第二阶段参数优化前，必须先恢复并验证  
   `hexbot_isaac_rooted_reimport.usd`
   的 clean reset 能力；否则 gait 参数实验会持续被坏资产污染

## 推荐复现顺序

### 第一阶段：先做 `hexapod_ros` 风格试验

重点试验项：

- 更明确的站立高度 / body clearance 语义
- 更宽但可控的 stance
- 略高的抬腿高度
- 非零的行走中 IMU 稳定化
- 更保守的姿态修正速率

### 第二阶段：再做 `OpenSHC` 风格参数重组

重点重组项：

- `body_clearance`
- `swing_height`
- `step_frequency`
- 逐腿 `stance_position`
- `imu_posing`
- `inclination_posing`

## 下一步建议

下一步不应直接从这些参考参数继续硬推，而应：

- 先完成 OCS2 闭环成品化与 A/B 证据沉淀
- 先修复 Isaac reimport 资产与 clean reset
- 再从当前稳定 baseline 分叉 gait 实验配置
- 保持当前已验收的旧 baseline 与 OCS2 主线都可回归
